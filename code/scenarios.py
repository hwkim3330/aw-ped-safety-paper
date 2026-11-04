#!/usr/bin/env python3
"""Hazards in front of the car Autoware drives in CARLA, and what it did about them.

Runs on the bench next to CARLA while patrol.py drives. Every --every s, once the ego is driving (> 18 km/h) on a
straight stretch with no junction ahead, one scenario is staged in front of it:
  ped      a pedestrian waits at the kerb 6 s ahead and steps off when the ego is 3.2 s away, crossing at 1.4 m/s
  cutin    a car in the next lane overtakes and cuts in 12 m in front, then slows
  brake    a car merges into the ego's lane ahead at its speed (or, with one lane each way, drives 40 m ahead in it),
           then brakes at 5 m/s2 to a stop
Braking is timed from the hazard's start (negative: Autoware slowed before, seeing it coming); a run where the ego
stopped short of it for something else (a red light) is not counted.
The actors keep their physics (so the lidar sees them, as it sees any car), and are steered by this script on every
world tick (pure pursuit + a speed loop); it never ticks the world itself (the Autoware CARLA interface does).
Logged per run (stdout and ~/.cache/aw-demo/scenarios.jsonl): whether the ego came to a stop, the smallest gap
between the two bodies, a contact (gap < 0.3 m), the time from the hazard's start to the ego's first braking, and what
Autoware said held it (the gateway's velocity factors). The actors are removed after each run.
    python3 scenarios.py [--every 300] [--only ped|cutin|brake] [--once] [--runs N] [--tag round8] [--walker any|adult|child]"""
import argparse
import json
import math
import os
import random
import signal
import threading
import time
import urllib.request

import carla

ap = argparse.ArgumentParser()
ap.add_argument('--carla', default='127.0.0.1')
ap.add_argument('--gateway', default='http://10.42.0.90:8780')
ap.add_argument('--every', type=float, default=300)
ap.add_argument('--only', help='one kind, or several: ped,brake')
ap.add_argument('--once', action='store_true')
ap.add_argument('--runs', type=int, default=0, help='stop after this many counted runs')
ap.add_argument('--tag', default='', help='written into each record (the round of a test series)')
ap.add_argument('--walker', default='any', choices=['any', 'adult', 'child'], help='ped: the walker models (child: the children and teenagers)')
a = ap.parse_args()

# SIGTERM as an exception: run()'s finally removes the actors (a pedestrian left standing in a lane stops the patrol)
signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(0)))
LOG = os.path.expanduser('~/.cache/aw-demo/scenarios.jsonl')
os.makedirs(os.path.dirname(LOG), exist_ok=True)


def log(*m):
    print(time.strftime('%T'), *m, flush=True)


def ped_seen():
    """pedestrians in Autoware's object list now (the gateway's per-class count), None if it did not answer"""
    try:
        c = json.load(urllib.request.urlopen(f'{a.gateway}/vss/Vehicle.Private.Perception.Classes', timeout=1)).get('Vehicle.Private.Perception.Classes') or {}
        return int(c.get('pedestrian', 0))
    except Exception:
        return None


def factors():
    try:
        vf = json.load(urllib.request.urlopen(f'{a.gateway}/vss/Vehicle.Private.Autoware.VelocityFactors', timeout=3))
        return vf.get('Vehicle.Private.Autoware.VelocityFactors')
    except Exception as e:
        return repr(e)[:80]


c = carla.Client(a.carla, 2000)
c.set_timeout(30)
world = c.get_world()
world.wait_for_tick()             # a fresh client sees no actors before its first tick (sync mode)
cmap = world.get_map()
bp = world.get_blueprint_library()
rng = random.Random()


def walker_bps():
    """the walker models --walker picks: children and teenagers (age child / teenager) are the ones CenterPoint and
    YOLOX see least (2026-11-04: 28 of 29 crossings by a child hit it, adults 38 of 242)"""
    allw = list(bp.filter('walker.pedestrian.*'))
    if a.walker == 'any':
        return allw
    young = [b for b in allw if b.has_attribute('age') and b.get_attribute('age').as_str() in ('child', 'teenager')]
    return young if a.walker == 'child' else [b for b in allw if b not in young]


def ego():
    for v in world.get_actors().filter('vehicle.*'):
        if v.attributes.get('role_name') == 'ego_vehicle':
            return v
    return None


def speed(actor):
    v = actor.get_velocity()
    return math.hypot(v.x, v.y)


def straight_ahead(wp, dist):
    """the lane `dist` m ahead in 5 m steps, None if a junction or a bend (> 12 deg) is on the way"""
    out, y0, cur = [wp], wp.transform.rotation.yaw, wp
    for _ in range(int(dist / 5)):
        nx = cur.next(5.0)
        if not nx:
            return None
        cur = nx[0]
        if cur.is_junction or abs((cur.transform.rotation.yaw - y0 + 180) % 360 - 180) > 15:
            return None
        out.append(cur)
    return out


def side_lane(wp):
    for s in (wp.get_left_lane(), wp.get_right_lane()):
        if s and s.lane_type == carla.LaneType.Driving and (s.lane_id > 0) == (wp.lane_id > 0):
            return s
    return None


def gap(e, o):
    """the distance between the two bodies (their bounding boxes as circles along the line between them, minus)"""
    el, ol = e.get_transform(), o.get_transform()
    d = el.location.distance(ol.location)
    eb, ob = e.bounding_box.extent, o.bounding_box.extent
    # half-length along the line of centres: the long axis when the other is ahead, the short one beside
    dx, dy = ol.location.x - el.location.x, ol.location.y - el.location.y
    def half(t, b):
        yaw = math.radians(t.rotation.yaw)
        cth = abs(math.cos(yaw) * dx + math.sin(yaw) * dy) / (d or 1)
        return b.x * cth + b.y * math.sqrt(max(0.0, 1 - cth * cth))
    return d - half(el, eb) - half(ol, ob)


class Driver:
    """a car steered along waypoints by pure pursuit, its speed by a P loop on throttle / brake"""
    def __init__(self, car, path, v_of_t):
        self.car, self.path, self.v_of_t, self.t0 = car, path, v_of_t, None

    def step(self, t):
        if self.t0 is None:
            self.t0 = t
        tr = self.car.get_transform()
        p, yaw = tr.location, math.radians(tr.rotation.yaw)
        # the first path point 7 m+ ahead
        tgt = self.path[-1]
        for w in self.path:
            q = w.transform.location
            if (q.x - p.x) * math.cos(yaw) + (q.y - p.y) * math.sin(yaw) > 7.0:
                tgt = w
                break
        q = tgt.transform.location
        alpha = math.atan2(q.y - p.y, q.x - p.x) - yaw
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi
        ld = max(4.0, p.distance(q))
        steer = math.atan2(2 * 2.9 * math.sin(alpha), ld) / math.radians(70)
        vt, v = self.v_of_t(t - self.t0), speed(self.car)
        e = vt - v
        ctl = carla.VehicleControl(steer=max(-1, min(1, steer)))
        if vt < 0.05 and v < 0.3:
            ctl.brake, ctl.hand_brake = 1.0, True
        elif e >= 0:
            ctl.throttle = max(0.0, min(1.0, 0.25 + 0.25 * e))
        else:
            ctl.brake = max(0.0, min(1.0, -0.18 * e))
        self.car.apply_control(ctl)


SKIP = {}


def skip(why):
    SKIP[why] = SKIP.get(why, 0) + 1


def spawn_car(spawn, path, v_of_t, v0, actors, drivers):
    tr = spawn.transform
    tr.location.z += 0.4
    car = world.try_spawn_actor(rng.choice([b for b in bp.filter('vehicle.*') if int(b.get_attribute('number_of_wheels')) == 4 and 'cybertruck' not in b.id]), tr)
    if car is None:
        return None
    actors.append(car)
    fwd = tr.get_forward_vector()
    car.set_target_velocity(carla.Vector3D(fwd.x * v0, fwd.y * v0, 0))
    drv = Driver(car, path, v_of_t)
    drivers.append(lambda snap: drv.step(snap.timestamp.elapsed_seconds))
    return car


def run(kind):
    e = ego()
    if e is None:
        return skip('no ego')
    v0 = speed(e)
    wp = cmap.get_waypoint(e.get_location())
    if v0 < 5:
        return skip('slow')
    lane = straight_ahead(wp, 70)
    if lane is None:
        return skip('bend/junction ahead')
    actors, drivers = [], []
    hazard = {'t': None, 'v': None, 'at': None}     # when the hazard begins (walker steps off / car brakes or cuts in)
    trace = []                                      # (sim time, ego speed) every tick
    try:
        if kind == 'ped':
            # standing at the kerb on the right 6 s ahead (50 m at least), as a pedestrian waits before crossing -- not
            # appearing out of nothing 30 m ahead, which measured the tracker's start-up more than Autoware
            d = max(50.0, 6.0 * v0)
            w = lane[min(len(lane) - 1, int(d / 5))]
            r = w.transform.get_right_vector()
            off = w.lane_width / 2 + 1.2
            loc = w.transform.location + carla.Location(x=r.x * off, y=r.y * off, z=1.0)
            walker = world.try_spawn_actor(rng.choice(walker_bps()), carla.Transform(loc))
            if walker is None:
                return skip('spawn failed')
            actors.append(walker)
            cross = carla.Vector3D(-r.x, -r.y, 0.0)
            start = {'t': None}

            def walk(snap):
                ts = snap.timestamp.elapsed_seconds
                if start['t'] is None:
                    # steps off when the ego is 3.2 s away (10 m at least): a stop is possible, from the moment it moves
                    el, wl, f = e.get_location(), walker.get_location(), e.get_transform().get_forward_vector()
                    ahead = (wl.x - el.x) * f.x + (wl.y - el.y) * f.y
                    if ahead > max(3.2 * speed(e), 10.0):
                        walker.apply_control(carla.WalkerControl(direction=cross, speed=0.0))
                        return
                    start['t'] = ts
                    hazard['t'] = ts
                    hazard['v'] = speed(e)
                    hazard['wall'] = time.time()
                # in the ego's lane yet (a walker can stick on the kerb: then nothing was tested)
                wl = walker.get_location()
                lat = (wl.x - w.transform.location.x) * r.x + (wl.y - w.transform.location.y) * r.y
                if abs(lat) < w.lane_width / 2 + 0.3:
                    hazard['in_lane'] = True
                # crosses the lane and the next one (about 7 m) then stands
                go = ts - start['t'] < (w.lane_width + off + 3.5) / 1.4
                walker.apply_control(carla.WalkerControl(direction=cross, speed=1.4 if go else 0.0))
            drivers.append(walk)
            other = walker
        elif kind == 'brake' and side_lane(wp) is None:
            # one lane each way (Town01): the car 40 m ahead in the ego's own lane at its speed, then hard on the brakes
            spawn = lane[8]
            path = lane[9:]
            v_of_t = lambda t: v0 if t < 5 else max(0.0, v0 - 5.0 * (t - 5))
            hazard['at'] = 5.0
            other = spawn_car(spawn, path, v_of_t, v0, actors, drivers)
            if other is None:
                return skip('spawn failed')
        else:
            side = side_lane(wp)
            if side is None:
                return skip('no lane beside')
            sl = straight_ahead(side, 70)
            if sl is None:
                return skip('lane beside bends')
            if kind == 'cutin':
                # beside and 4 m behind the ego in the next lane, 1.35x its speed, into the ego's lane 12 m ahead of it
                spawn = side.previous(4.0)[0] if side.previous(4.0) else side
                path = [spawn] + sl[2:5] + lane[int((v0 * 3 + 12) / 5):]
                v_of_t = lambda t: v0 * 1.35 if t < 4.5 else max(0.0, v0 * 1.35 - 2.5 * (t - 4.5)) if t < 7 else v0 * 0.6
                hazard['at'] = 2.0
            else:   # brake: in the next lane 25 m ahead, into the ego's lane, then hard on the brakes
                spawn = sl[5]
                path = [spawn] + lane[9:]
                v_of_t = lambda t: v0 if t < 5 else max(0.0, v0 - 5.0 * (t - 5))
                hazard['at'] = 5.0
            other = spawn_car(spawn, path, v_of_t, v0, actors, drivers)
            if other is None:
                return skip('spawn failed')

        res = {'kind': kind, 'actor': other.type_id, 'at': time.strftime('%F %T'), 'v0_kmh': round(v0 * 3.6, 1), 'min_gap_m': 99.0,
               'stopped': False, 'contact': False, 'v_at_min_kmh': None, 'brake_after_s': None, 'factors': None, 'first_brake_t': None, 'v_hazard_kmh': None,
               # when Autoware's object list first held a pedestrian: the gap then and the time from the hazard's start
               # (negative: seen at the kerb before it stepped out) -- a late detection and a late brake look the same in min_gap
               'seen_gap_m': None, 'seen_after_s': None}
        now = {'g': None}
        lock, t0 = threading.Lock(), {'t': None, 'v_prev': v0}

        def on_tick(snap):
            with lock:
                for f in drivers:
                    try:
                        f(snap)
                    except RuntimeError:
                        pass
                ts = snap.timestamp.elapsed_seconds
                if t0['t'] is None:
                    t0['t'] = ts
                try:
                    g, ve = gap(e, other), speed(e)
                except RuntimeError:
                    return
                now['g'] = g
                if g < res['min_gap_m']:
                    res['min_gap_m'], res['v_at_min_kmh'] = round(g, 2), round(ve * 3.6, 1)
                # a contact only while the ego moves: a pedestrian walking into the side of a car that has stopped for
                # it is not the car's doing
                if g < 0.3 and ve > 0.5:
                    res['contact'] = True
                if hazard['t'] is None and hazard['at'] is not None and ts - t0['t'] >= hazard['at']:
                    hazard['t'], hazard['v'], hazard['wall'] = ts, ve, time.time()
                trace.append((ts, ve))
                t0['v_prev'] = ve
                if ve < 0.3 and g < 25:
                    res['stopped'] = True
        log(f'{kind}: staged ({other.type_id}, ego {v0 * 3.6:.0f} km/h)')
        t_stage = time.time()
        cb = world.on_tick(on_tick)
        t_end = time.time() + 40
        stood = None
        while time.time() < t_end:
            time.sleep(0.2)
            # what Autoware said it was slowing for, through the approach (first time each behaviour appeared, s from staging):
            # the slow-down for a walker at the kerb (obstacle_slow_down) engaged in some runs and not in others
            if kind == 'ped':
                fv = factors()
                if isinstance(fv, list):
                    for f in fv:
                        b = f.get('behavior') or f.get('text') if isinstance(f, dict) else str(f)
                        res.setdefault('vf_first', {}).setdefault(str(b), round(time.time() - t_stage, 1))
            if kind == 'ped' and res['seen_gap_m'] is None and (ped_seen() or 0) > 0 and now['g'] is not None:
                res['seen_gap_m'] = round(now['g'], 1)
                res['seen_at_wall'] = time.time()
            if res['stopped'] and res['factors'] is None:
                res['factors'] = factors()
                stood = stood or time.time()
            if stood and time.time() - stood > 4:
                break
        world.remove_on_tick(cb)
        if hazard['t'] is None:
            return skip('hazard never began (the ego stopped short)')
        if kind == 'ped' and not hazard.get('in_lane'):
            log('ped: the walker never reached the lane (stuck at the kerb): not counted')
            return skip('walker stuck')
        res['v_hazard_kmh'] = round((hazard['v'] or 0) * 3.6, 1)
        if res.get('seen_at_wall') and hazard.get('wall'):
            res['seen_after_s'] = round(res.pop('seen_at_wall') - hazard['wall'], 2)
        res.pop('seen_at_wall', None)
        if hazard.get('wall'):
            res['hazard_at_s'] = round(hazard['wall'] - t_stage, 1)      # the walker set off this long after staging (vf_first's clock)
        # braking onset: from 2 s before the hazard, the first time the speed is 1 m/s under its peak since then
        # (negative: Autoware slowed before the hazard began, seeing it coming)
        pk = None
        for ts, ve in trace:
            if ts < hazard['t'] - 2.0:
                continue
            pk = ve if pk is None else max(pk, ve)
            if ve < pk - 1.0:
                res['first_brake_t'] = ts
                break
        if res['first_brake_t'] is not None:
            res['brake_after_s'] = round(res['first_brake_t'] - hazard['t'], 2)
        del res['first_brake_t']
        # the deceleration the car actually reached from the hazard on (0.5 s windows): a stop decision in time with a weak
        # brake and a late decision with a strong one end in the same contact (11-26 km/h hits after braking began early)
        W = [(ts, ve) for ts, ve in trace if ts >= hazard['t'] - 2.0]
        best = 0.0
        j = 0
        for i in range(len(W)):
            while j < len(W) and W[j][0] - W[i][0] < 0.5:
                j += 1
            if j < len(W):
                best = max(best, (W[i][1] - W[j][1]) / (W[j][0] - W[i][0]))
        res['peak_decel'] = round(best, 2)
        return res
    finally:
        for x in actors:
            try:
                x.destroy()
            except RuntimeError:
                pass


kinds = a.only.split(',') if a.only else ['ped', 'cutin', 'brake']
i, since = 0, time.time()
log(f'scenarios {kinds} every {a.every:.0f} s')
while True:
    kind = kinds[i % len(kinds)]
    try:
        r = run(kind)
    except Exception as ex:
        log('error:', repr(ex)[:160])
        r = None
    if r is None:
        time.sleep(3)          # not driving / not straight here: try again soon
        if time.time() - getattr(run, 'said', 0) > 120:
            run.said = time.time()
            log(f'{kind}: waiting for a place ({SKIP})')
        if len(kinds) > 1 and time.time() - since > 90:      # no place for this one on the patrol's roads now: the next
            log(f'{kind}: no place in 90 s, skipped')
            i, since = i + 1, time.time()
        continue
    i, since = i + 1, time.time()
    verdict = 'CONTACT' if r['contact'] else 'stopped' if r['stopped'] else 'passed'
    log(f"{kind}: {verdict} · v0 {r['v0_kmh']} km/h · at hazard {r['v_hazard_kmh']} km/h · min gap {r['min_gap_m']} m at {r['v_at_min_kmh']} km/h · brake after {r['brake_after_s']} s · peak decel {r.get('peak_decel')} · seen at {r.get('seen_gap_m')} m ({r.get('seen_after_s')} s) · vf {r.get('vf_first')} · {json.dumps(r['factors'], ensure_ascii=False)[:160]}")
    if a.tag:
        r['tag'] = a.tag
    with open(LOG, 'a') as f:
        f.write(json.dumps(r, ensure_ascii=False) + '\n')
    if a.once and i >= len(kinds) or a.runs and i >= a.runs:
        break
    time.sleep(a.every)
