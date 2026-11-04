#!/usr/bin/env python3
"""Keep the Autoware ego driving around the CARLA town on its own: whenever it has arrived (or has no route), pick a
destination 1-2 km ahead along the road network and send it the way the PLEOS HMI does (the gateway's
navigation_request + autopilot, with the control token), then watch. On an MRM / failed engage: stop, re-initialise
localisation (AD API, inside the Autoware container), retry. Logs one line per event.
    python3 patrol.py [--gateway http://10.42.0.90:8780] [--laps 0 (forever)]"""
import argparse
import json
import os
import heapq
import random
import re
import subprocess
import time
import urllib.request

import carla

ap = argparse.ArgumentParser()
ap.add_argument('--gateway', default='http://10.42.0.90:8780')
ap.add_argument('--carla', default='127.0.0.1')
ap.add_argument('--laps', type=int, default=0)
ap.add_argument('--min', type=float, default=1000)
ap.add_argument('--max', type=float, default=2000)
ap.add_argument('--city', action='store_true', help='destinations only in the town centre (Town04: the junction / traffic-light '
                'cluster at CARLA x 150..450, y -450..-150), at least --city-min m away')
ap.add_argument('--city-min', type=float, default=250)
ap.add_argument('--idle-local', type=float, default=60, help='s standing in LOCAL (the driver took over) before engaging again')
ap.add_argument('--stuck', type=float, default=90, help='s standing still while driving with a route before recovering')
ap.add_argument('--poll', type=float, default=3, help='s between arrival checks (was 10: up to 10 s standing at each goal)')
a = ap.parse_args()
TOK = open(os.path.expanduser('~/.config/pleos_gateway_token')).read().strip()
D = 111319.49            # the gateway's pseudo lat/lon for a Local projector: lat = y / D, lon = x / D
SSH = ['ssh', '-o', 'ConnectTimeout=5', 'aw-pc']


def log(*m):
    print(time.strftime('%T'), *m, flush=True)


# one line per leg in patrol_legs.jsonl: when, how far the goal was, how it ended (arrived / stuck / mrm / local /
# lost), metres driven and seconds -- days of patrol left only goals and recoveries, never whether it got there
LEGS = os.path.expanduser('~/.cache/aw-demo/patrol_legs.jsonl')
cur = {}


def leg_end(outcome):
    if not cur:
        return
    r = dict(cur, outcome=outcome, end=time.strftime('%F %T'), secs=round(time.time() - cur['t0']), m=round(cur['m']))
    r.pop('t0')
    with open(LEGS, 'a') as f:
        f.write(json.dumps(r) + '\n')
    cur.clear()


def cmd(name, body):
    r = urllib.request.Request(f'{a.gateway}/api/1/vehicles/kdrive/command/{name}', data=json.dumps(body).encode(),
                               headers={'Content-Type': 'application/json', 'X-Control-Token': TOK}, method='POST')
    try:
        return json.load(urllib.request.urlopen(r, timeout=20)).get('response', {})
    except Exception as e:
        return {'result': False, 'reason': str(e)}


def aw_state():
    """the gateway's one-line status: 'AW <state> · op <mode> · <v> km/h · route <state>'"""
    try:
        out = subprocess.run(SSH + ['sg docker -c "docker exec aw-carla tail -1 /tmp/gateway.log"'], capture_output=True, text=True, timeout=20).stdout
        return out.strip()
    except Exception:
        return ''


def adapi(script):
    return subprocess.run(SSH + [f'sg docker -c "docker exec aw-carla bash -lc \\"source /opt/autoware/setup.bash; {script}\\""'],
                          capture_output=True, text=True, timeout=60).stdout


c = carla.Client(a.carla, 2000); c.set_timeout(30)
laps, rng = 0, random.Random()


CITY = {'Carla/Maps/Town04': (150, 450, -450, -150)}   # x0, x1, y0, y1 in CARLA coordinates


def city_goal(m, wp0):
    """a destination in the town centre's inner streets, reached by driving on from the ego's lane (CARLA's road graph
    with lane changes, so the route exists). Following lanes only, it never got off the perimeter roads: turning in
    needs the other lane (223 inner points reachable with lane changes, 7 on the edge without)."""
    x0, x1, y0, y1 = CITY[m.name]
    inner = lambda l: x0 + 30 <= l.x <= x1 - 60 and y0 + 90 <= l.y <= y1 - 30
    pq, seen, hits, i = [(0.0, 0, wp0)], set(), [], 0
    while pq and len(seen) < 30000:
        d, _, wp = heapq.heappop(pq)
        key = (wp.road_id, wp.section_id, wp.lane_id, int(wp.s // 10))
        if key in seen:
            continue
        seen.add(key)
        if inner(wp.transform.location) and not wp.is_junction and d >= a.city_min:
            hits.append((wp, d))
        if d < 4000:
            nb = [(n, 10.0) for n in wp.next(10.0)]
            for side in (wp.get_left_lane(), wp.get_right_lane()):
                if side and side.lane_type == carla.LaneType.Driving and side.lane_id * wp.lane_id > 0:
                    nb.append((side, 15.0))
            for n, cost in nb:
                i += 1
                heapq.heappush(pq, (d + cost, i, n))
    hits.sort(key=lambda h: h[1])
    wp, d = rng.choice(hits[:max(1, len(hits) // 3)])
    return wp, d


def next_goal():
    w = c.get_world(); w.wait_for_tick(10); m = w.get_map()
    ego = [x for x in w.get_actors().filter('vehicle.*') if x.attributes.get('role_name') == 'ego_vehicle'][0]
    if a.city and m.name in CITY:
        wp, d = city_goal(m, m.get_waypoint(ego.get_transform().location, project_to_road=True,
                                             lane_type=carla.LaneType.Driving))
        L = wp.transform.location
        return -L.y / D, L.x / D, d
    wp = m.get_waypoint(ego.get_transform().location, project_to_road=True, lane_type=carla.LaneType.Driving)
    want, d = rng.uniform(a.min, a.max), 0
    while d < want:
        nx = wp.next(10.0)
        if not nx:
            break
        wp = rng.choice(nx); d += 10
    L = wp.transform.location
    return -L.y / D, L.x / D, d


def align():
    """a standing ego put square on its lane centre (after a driver left it askew: 1.2 m and 13 deg off kept the
    planning validator rejecting every trajectory)"""
    w = c.get_world(); w.wait_for_tick(10); m = w.get_map()
    ego = [x for x in w.get_actors().filter('vehicle.*') if x.attributes.get('role_name') == 'ego_vehicle'][0]
    if ego.get_velocity().length() > 0.3:
        return
    t = ego.get_transform()
    wp = m.get_waypoint(t.location, project_to_road=True, lane_type=carla.LaneType.Driving)
    off = t.location.distance(wp.transform.location)
    dyaw = abs((t.rotation.yaw - wp.transform.rotation.yaw + 180) % 360 - 180)
    if off > 0.5 or dyaw > 5:
        nt = wp.transform
        nt.location.z = t.location.z + 0.2
        ego.set_transform(nt)
        log(f'aligned the ego on its lane ({off:.1f} m, {dyaw:.0f} deg off)')


def recover(clear=False):
    log('recover: stop, re-initialise localisation' + (', clear the route' if clear else ''))
    adapi('timeout 15 ros2 service call /api/operation_mode/change_to_stop autoware_adapi_v1_msgs/srv/ChangeOperationMode {}')
    time.sleep(3)
    try:
        align()
    except Exception as e:
        log('align:', repr(e)[:120])
    time.sleep(2)
    if clear:
        adapi('timeout 15 ros2 service call /api/routing/clear_route autoware_adapi_v1_msgs/srv/ClearRoute {}')
    adapi('timeout 20 ros2 service call /api/localization/initialize autoware_adapi_v1_msgs/srv/InitializeLocalization \\\\\\"{pose: []}\\\\\\"')
    time.sleep(5)


def leg():
    """one check / one leg; False = try again shortly"""
    global laps
    s = aw_state()
    if 'op LOCAL' in s and 'route SET' in s:
        if cur and not cur.get('local'):
            cur['local'] = True                          # the driver took it: counted, the leg goes on
        # the driver took over on the wheel: theirs while they drive; standing 60 s in LOCAL = let go -> engage again
        m = re.search(r'([-\d.]+) km/h', s)
        v = abs(float(m.group(1))) if m else 1.0
        leg.local = (getattr(leg, 'local', None) or time.time()) if v < 0.5 else None
        if leg.local and time.time() - leg.local > a.idle_local:
            leg.local = None
            log(f'LOCAL and standing for {a.idle_local:.0f} s: engaging again on the same route')
            log('engage:', cmd('autopilot', {'on': True}))
        time.sleep(a.poll)
        return True
    leg.local = None
    if 'route SET' in s and 'AUTONOMOUS' in s and 'MRM' not in s:
        m = re.search(r'([-\d.]+) km/h', s)
        v = abs(float(m.group(1))) if m else 1.0
        leg.still = (getattr(leg, 'still', None) or time.time()) if v < 0.5 else None
        if cur:
            cur['m'] += v / 3.6 * a.poll
        if leg.still and time.time() - leg.still > a.stuck:
            # driving, route set, standing still: the final trajectory rejected by the planning validator (seen after
            # CARLA stalled on a 100 Mb/s link) -- clear the route, re-localise; three times in a row: restart it all
            leg.still, leg.stucks = None, getattr(leg, 'stucks', 0) + 1
            why = ''
            try:                       # what Autoware says is holding it, and where (for the morning's log)
                vf = json.load(urllib.request.urlopen(f'{a.gateway}/vss/Vehicle.Private.Autoware.VelocityFactors', timeout=5))
                why = json.dumps(vf.get('Vehicle.Private.Autoware.VelocityFactors'), ensure_ascii=False)[:300]
                ds = json.load(urllib.request.urlopen(f'{a.gateway}/api/1/vehicles/kdrive/vehicle_data', timeout=5))['response']['drive_state']
                why += f" at ({ds['map_x']:.0f}, {ds['map_y']:.0f})"
            except Exception as e:
                why = repr(e)[:80]
            log(f'standing still for {a.stuck:.0f} s while driving ({leg.stucks}): {why} -- clear the route, re-localise')
            leg_end('stuck')
            if leg.stucks >= 3:
                log('still stuck: asking the supervisor for a full restart')
                open(os.path.expanduser('~/.config/aw-demo/restart'), 'w').close()
                leg.stucks = 0
            recover(clear=True)
            return False
        if v >= 0.5:
            leg.stucks = 0
        time.sleep(a.poll)
        return True
    leg_end('arrived' if 'ARRIVED' in s else 'mrm' if ('MRM' in s or 'EMERGENCY' in s) else 'lost')
    if 'MRM' in s or 'EMERGENCY' in s:
        recover()
    lat, lon, d = next_goal()
    r = cmd('navigation_request', {'lat': lat, 'lon': lon})
    log(f'goal x {lon * D:.0f} y {lat * D:.0f} ({d:.0f} m ahead):', r)
    if r.get('result') is False:
        leg.fails = getattr(leg, 'fails', 0) + 1
        if leg.fails % 5 == 0:   # five goals refused in a row: localisation is off (a respawned ego), start over
            recover()
        time.sleep(5)
        return False
    leg.fails = 0
    time.sleep(3)            # the route is set by then (it answered); 6 s added to every stop at a goal
    # engage, retried for up to 30 s: on a big map (Town04) the planner is not ready 3 s after a new route, and the
    # refusal sent patrol into recover -> new route -> refused, round and round (2026-10-29)
    for _ in range(15):
        r = cmd('autopilot', {'on': True})
        if r.get('result') is not False:
            break
        time.sleep(2)
    log('engage:', r)
    if r.get('result') is False:
        recover()
        return False
    laps += 1
    cur.update(t0=time.time(), start=time.strftime('%F %T'), goal_m=round(d), m=0.0)
    time.sleep(20)
    return True


while a.laps == 0 or laps < a.laps:
    try:
        leg()
    except Exception as e:     # CARLA paused (the link to the Autoware PC cut) or restarting: wait, do not die
        log('error:', repr(e)[:160])
        time.sleep(5)
log('done', laps, 'legs')
