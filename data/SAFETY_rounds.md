# Pedestrian safety in CARLA (Autoware on the Autoware PC, CARLA Town01 on the bench)

Goal: **no contact** with a pedestrian stepping into the lane, without making normal driving worse.

## The test

`python3 scenarios.py --only ped --every 30` while patrol.py drives: a CARLA walker waits at the kerb on the right 6 s ahead
(2.95 m from the lane centre, about 2 m from the car's side), steps off when the ego is 3.2 s away and crosses at 1.4 m/s.
Each run records (scenarios.jsonl):

- contact (gap < 0.3 m while the ego moves) and the speed then, the minimum gap
- `seen_gap_m` / `seen_after_s`: when Autoware's object list first held a pedestrian (gap, time from the walker setting off)
- `brake_after_s`: when the ego first slowed by 1 m/s (negative: before the walker set off)
- `peak_decel`: the highest deceleration reached over 0.5 s (the brake reaching the car, or not)

A per-stage probe in the Autoware container (`pedstage.py`, scratch) logs pedestrians per stage every 0.5 s: CenterPoint
(LiDAR), the YOLOX boxes and their scores, the camera's 3D objects, tracking, the final objects.

## Rounds (2026-11-03, Town01, 30-40 min each)

| round | settings | runs | contacts |
| --- | --- | ---: | ---: |
| 1 | default (BRAKE=normal) | 29 | 18 (62 %) |
| 2 | `BRAKE=strong` (planner emergency limits -4 m/s2, jerk -3) | 22 | 8 (36 %) |
| 3 | + `RUNOUT=strict` (run_out: no "ego arrives first" / "first and cannot stop" ignores) | 23 | 8 (35 %) |
| 4 | + `PEDSLOW=1` v1 (obstacle_slow_down: pedestrians out to 3 m, 20 km/h) | 29 | 8 (28 %) |
| 5 | + PEDSLOW v2 (slow-down decel -2.0 was -1.0, entry after 2 frames was 5, 3.5 m band, 18 km/h) | 29 | 5 (17 %) |
| 6 | + run_out stop after 0.1 s (was 0.3), collision time margin 1.0 s (was 0.5) | 33 | 9 (27 %) |
| 7 | + CenterPoint PEDESTRIAN floor 0.25 (was 0.35), camera VRU roi confidence 0.35 (was 0.50) | 50 | 6 (12 %) |
| 8 | run_out back to the round-5 values (stop after 0.3 s, margin 0.5 s), detection as in 7 (2026-11-04) | 30 | 5 (17 %) |
| 9 | + `PEDTRACK=1`: tracker patch, a pedestrian track decays once per elapsed time (half-life 1 s), detections count as score >= 0.75 | 30 | 4 (13 %): **adults 0/26, children 4/4** |
| 10c | + front camera FOV 70 deg (was 120); **children only** (`--walker child`) | 15 | 8 (53 %) |
| 11c | + YOLOX-sPlus INT8 (was tiny); children only | 15 | 12 (80 %): sPlus labelled a child at the kerb UNKNOWN |
| 12c | + YOLOX UNKNOWN read as PEDESTRIAN; children only | 15 | 8 (53 %): camera-placed tracks were never published (range variance 8-19 m2) |
| 13c | + tracker publishes a pedestrian up to 25 m2 position variance (camera-placed); children only | 15 | 1 (7 %) |
| 14c | + slow-down held 20 frames after the pedestrian is gone (was 5); children only | 15 | 5 (33 %) |
| 15c | + PEDSLOW v3: kerbside pedestrian passed at ~10 km/h (was ~14); children only | 15 | 6 (40 %): most hit at 23-30 km/h, not seen standing |
| 16c | + camera VRU range 60 m (was 40), tracks up to 45 m2; children only | 15 | 5 (33 %) |
| 17c | + front camera FOV 45 deg (was 70); children only | 15 | **0** |

Patrol over the same day (all rounds): 79 of 83 legs arrived (95 %), 40.5 km, no stuck legs.

## What the numbers showed

- **Detection is not the main cause.** CenterPoint had the walker standing at the kerb 40-56 m out in nearly every run;
  Autoware's final objects 25-46 m. YOLOX scores far walkers 0.31-0.38 (under the locator's 0.5), near ones ~0.9.
- **The brake reaches the car.** Peak decelerations were ~4.8 m/s2 (the planner limit) or ~12 (emergency stop).
- **Speed when the walker sets off decides it.** Every contact in round 4 came at 26-30 km/h; a car already at ~20 km/h
  never hit. The slow-down was capped at -1 m/s2 (30 -> 16 km/h took ~25 m) -- round 5.
- Left after round 5: 2 walkers seen only 2-4 m out (detection), 3 seen 33-45 m out but acted on 1.8-2 s after setting
  off (reaction) -- round 6.

## Switches (demo_all.sh / run_autoware_carla.sh, environment)

- `BRAKE=strong`: aw_overrides/planning_common.strong.param.yaml + core_common.strong.param.yaml
- `RUNOUT=strict`: aw_overrides/run_out.strict.param.yaml
- `PEDSLOW=1`: aw_overrides/obstacle_slow_down.ped.param.yaml

`BRAKE=strong RUNOUT=strict PEDSLOW=1 ./demo_all.sh` runs all three. The supervisor's own restarts still use the defaults.
