# Pedestrian safety for Autoware in CARLA: what made the car stop for a child

Working notes + code for a short paper (to be written from this repo).

## Setup
- CARLA 0.9.16, Town01, bench PC; Autoware (ROS 2 jazzy, Docker) on a second PC; LiDAR CenterPoint + front camera YOLOX.
- Test (`code/scenarios.py --only ped`): a walker (adult or child) waits at the kerb 6 s ahead, steps off when the ego is 3.2 s away, crosses at 1.4 m/s. Contact = gap < 0.3 m while moving. Also logged: first-seen distance, brake reaction, peak deceleration.
- Normal-driving check: `code/patrol.py` random legs, arrival rate (`patrol_stats.py`).

## Result (data/SAFETY_rounds.md has every round)
- Default Autoware: 18/29 contacts (62 %). Final: adults 0/26 (since round 9), children 0/15 (round 17c). Patrol arrival stayed ~95 %.
- What mattered, in order of effect:
  1. Planner braking limits (`config/*strong*`), run_out without "ego passes first" ignores (`run_out.strict`).
  2. Slow-down near a kerbside pedestrian (`obstacle_slow_down.ped`): pass at ~10 km/h; held 20 frames after loss. Speed at walker onset decided outcomes (no hit ever below ~20 km/h).
  3. Tracker persistence for pedestrians (`config/mot_pedtrack`, patch on multi_object_tracker) + publish camera-placed tracks up to 25-45 m² position variance.
  4. Camera: YOLOX UNKNOWN → PEDESTRIAN (`roi_label_remap.csv`), VRU range 60 m, FOV 120→70→45° (a child at 50 m was ~15 px).
  5. Lowering the detection threshold (0.25) created kerbside ghosts; reverted to 0.35 once tracking held sparse detections.
- Negative results: shorter run_out stop delay/larger margin (round 6, worse); YOLOX-sPlus alone (labelled children UNKNOWN).

## Open / to state as limitations
- **Child confirmation run (val50c, 2026-11-04, final config): 2 contacts in 47 runs (4.3 %)** — so round 17c's 0/15 was partly luck. One contact: walker seen 44 m out, brake 2.1 s after onset, hit at 9.4 km/h (late run_out decision, not detection). Cut-in / lead-brake / patrol regression with the 45° camera not yet rerun.
- Narrow FOV may miss lateral run-outs close by. Single town, synthetic walkers, one weather.

## Next experiments (reviewer advice, not done yet)
1. Controlled comparison: baseline vs final, same walkers/speeds/positions/seeds, ≥100 runs each, adults and children separately.
2. FOV 45° vs 70° vs 120° (lateral run-outs), night, rain, occlusion.
3. Verify contacts with CARLA's collision sensor; publish plots per config.

## Files
- `config/` Autoware parameter overrides and the tracker patch. `code/` scenario, patrol and stats scripts (paths/hosts are local to our bench).
- `data/SAFETY_rounds.md` rounds table + method notes; `data/change_log.txt` the commits with their measured effect; `data/scenarios.jsonl` raw per-run records (field `tag` = round), `data/patrol_legs.jsonl` patrol legs.
