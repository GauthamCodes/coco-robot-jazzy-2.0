# C2-NAV.40 — shifted `enclosure_entry` goal: implementation and validation, REJECTED

**Agent:** implementation and simulation validation only. No investigation
tree was opened. No nav2 parameter, safety gate or goal tolerance moved.

## 1. Commits (`worktree-c2nav0-diagnosis`)

| commit | what |
|---|---|
| `321df0e` | `bench.goals` in the experiment schema. Forwarded to nav_bench.py's existing `--goal`. Scenario names checked against TOUR (read with `ast`, no ROS) before launch. `verify-goals` compares each leg's recorded `goal_world` with the request after the run; a mismatch fails the run. `experiments/entry_corridor_centre.yaml`. Report `legs` mode and `--arm`. **The experiment commit every tour ran at.** |
| `8a34433` | Report fix: carry a PolygonStop hold across a leg boundary (found in r03, below) |
| `63aa9b8` | REJECT: `experiments/entry_corridor_centre.yaml` removed; tests keep the same experiment in `tmp_path` |

## 2. The change and the override path (measured)

- **Exact goal change:** `enclosure_entry` (−3.45, 2.95) → **(−3.575, 2.95)**.
  Nothing else moved.
- **The experiment file, verbatim:** `name`, `description`,
  `bench.goals.enclosure_entry: [-3.575, 2.95]`. Repeats 1, timeout 75 s and
  the diagnostic flag stay at their baseline defaults.
- **Resolve:** 0 parameter changes and 1 goal change. `params_file` is the
  shipped `nav2_params.yaml`, sha256 `6f61e499…`, the same as `baseline.yaml`.
  No merged file. `goal_args` = `enclosure_entry:-3.575,2.95`.
- **Tests:** `gazebo_models` 68 → **80/0**. The new tests cover:
  - the file contains only the goal;
  - only `enclosure_entry` differs from baseline;
  - the argument parses as nav_bench parses it;
  - unknown scenario, wrong length, string, bool, NaN and non-mapping are
    refused;
  - `verify-goals` accepts the driven goal and catches an override that was
    not driven or never exercised;
  - `tour_goals()` works with `rclpy` blocked.
- **Live, every run:**
  - `GOAL OVERRIDE enclosure_entry: (-3.45, 2.95) -> (-3.575, 2.95)` printed
    once;
  - `goals_check.txt` 7/7 OK, `enclosure_entry` driven = requested
    (−3.575, 2.95);
  - `params_live.txt` 12/12 OK: local CSF 65, global CSF 5, multi-pose BT,
    BaseObstacle 8.0, FollowPath xy 0.05, goal checker 0.25/0.25, PolygonStop
    0.25/4, PolygonSlow 0.3, `resample_interval` 1.

## 3. Runs (measured 2026-09-15 UTC, real simulation, topology A)

`nav_tour_run.sh experiments/entry_corridor_centre.yaml` × 3, each a fresh
headless Gazebo, commit `321df0e`, 0 dirty paths.

| run | start UTC | wall s | /scan | Nav2 active | nav_bench exit | teardown |
|---|---|---|---|---|---|---|
| `entry_corridor_centre_r01` | 21:02:35 | 437 | 9 s | 36 s | 0 | `0 matched` |
| `entry_corridor_centre_r02` | 21:11:34 | 429 | 9 s | 37 s | 0 | `0 matched` |
| `entry_corridor_centre_r03` | 21:19:07 | 434 | 8 s | 26 s | 0 | `0 matched` |

Runs are in `~/coco_nav_runs/entry_corridor_centre/`.

**Two starts were refused, rc=3, before any run directory was allocated.**
The cause was a log watch whose command line contained `nav_bench`, which
matched the runner's own busy check. That is the check working as designed.
The two tours were re-run with the watch pattern moved into a file. The busy
check and `ros_clean.sh --list` were proven empty before launch.

## 4. Per leg (`c2nav39_tour_report.py legs`)

Each cell is status, sim s, goal err m, yaw err rad, true clearance m. True
clearance is to world-file geometry.

| leg | r01 | r02 | r03 |
|---|---|---|---|
| `open_space` | S 19.60 · 0.118 · 0.378 · 0.5133 | S 20.34 · 0.077 · 0.381 · 0.4920 | S 17.83 · 0.163 · 0.404 · 0.5132 |
| `wall_adjacent` | S 44.28 · 0.028 · −0.397 · 0.4027 | S 43.51 · 0.043 · −0.508 · 0.3775 | S 64.37 · 0.063 · 0.292 · 0.3963 |
| `wall_parallel` | S 21.80 · 0.064 · −0.210 · 0.3995 | S 21.61 · 0.060 · −0.205 · 0.3661 | S 21.25 · 0.074 · −0.208 · 0.4628 |
| `obstacle_corner` | S 17.93 · 0.080 · 0.487 · 0.4912 | S 18.58 · 0.071 · 0.436 · 0.4968 | S 18.62 · 0.132 · 0.433 · 0.5213 |
| `corridor_gate` | S 26.52 · 0.113 · −0.424 · 0.6100 | S 30.54 · 0.081 · 0.386 · 0.3846 | S 21.52 · 0.011 · −0.467 · 0.4120 |
| `enclosure_entry` | **T** 70.32 · 0.186 · 2.929 · 0.2980 | **T** 55.65 · 1.638 · −2.642 · 0.4718 | **T** 70.88 · 1.103 · 2.112 · 0.2470 |
| `enclosure_exit` | S 34.16 · 0.136 · −0.390 · 0.2879 | S 13.46 · 0.079 · −0.460 · 0.4674 | **T** 65.95 · 2.730 · 2.112 · 0.2470 |
| **legs** | **6/7** | **6/7** | **5/7** |
| total sim s | 234.61 | 203.69 | 280.42 |
| min true clearance | 0.2879 | 0.3661 | **0.2470** |
| PolygonStop activations / s | 0 / 0 | 0 / 0 | 1 / 9.25 on entry, plus the exit hold below |
| deadlock / timeout | entry timeout | entry timeout | **entry timeout → exit deadlock** |

- **Ordinary legs:** 15/15, 0 PolygonStop.
- **`enclosure_entry`:** **0/3**.
- **`enclosure_exit`:** scored 2/3. **Only r01's exit started inside the
  pocket.**

## 5. What each enclosure leg did (measured from the traces)

**r01, terminal-yaw timeout; the one valid pocket exit.**
- The entry went east of `box_obstacle_1`, then north of it.
- It was inside the 0.25 m goal tolerance at 27.2 s and spent 61.3 % of the
  leg afterwards, the last 30.7 s turning at (−3.463, 3.089) under
  PolygonSlow.
- It ended 0.186 m and 2.929 rad from the goal.
- The exit then SUCCEEDED in 34.16 s from inside the pocket, with 0
  PolygonStop and true clearance 0.2879 m.

**r02, an entry stall that never reached the pocket.**
- It went up the east side and stopped: still for 21.0 s at (−2.311, 1.920),
  0.496 m from `box_obstacle_1`.
- Commanded linear speed was essentially zero from about 10 s. DWB's best
  vx was 0 on 71 % of frames, and commands were below 0.05 m/s on 80.6 %.
- The collision monitor recorded no STOP (DO_NOTHING 0.508, SLOWDOWN 0.377,
  LIMIT 0.115).
- It ended at (−2.3111, 1.9083), 1.638 m out, never level with the box
  (max y 1.978). The last plan was 2.284 m.
- Its exit started from there, outside the pocket, so the exit's 13.46 s
  SUCCEEDED is **not a test of the exit**.

**r03, a PolygonStop deadlock at a new corner.**
- It went up the east side and was still for 43.7 s at (−2.095, 2.124),
  0.656 m from the box.
- It then rounded `box_obstacle_1`'s **NE corner (−2.75, 2.65)**. PolygonStop
  fired at (−2.4901, 2.6582); the robot moved 29.1 mm to (−2.5052, 2.6831),
  **0.2470 m** from the box. That is 3 mm inside the 0.25 m circle and
  41.9 mm outside the 0.2051 m circumscribed radius. The hold lasted to the
  end of the entry (9.25 s).
- **The exit began in that pose and did not move for 65.95 s.** Nav2
  commanded 1.0 rad/s and minimum scan range was 0.205 m. The exit trace has
  no collision-monitor rows at all: the monitor publishes on change, and the
  last state it published was STOP.
- Total immobile under the hold was about 75 s, until the tour ended.
- **Instrument defect found here and fixed (`8a34433`).** The per-leg
  detector could not see a hold that crosses the leg boundary and reported
  "no deadlock". It now inherits the hold when no release is recorded.
  Selftest 21 → **27/0**. Over all eight C2-NAV.39/.40 tours it flags r03's
  exit and nothing new in the baseline.

**East-side stalls are not new** (longest ≥ 0.05 m-still span per entry).
- At the committed goal, `baseline_r01` stalled 28.1 s at (−2.145, 1.925) and
  `baseline_amcl_diag_r03` 16.6 s at (−2.160, 1.974). Both recovered and
  reached the pocket.
- **What is new at the shifted goal is the outcome after the stall:** r02
  never recovered, and r03 came round the NE corner into PolygonStop. None
  of the five committed-goal entries recorded any PolygonStop.

## 6. The known failure mode (Task 4)

- **C2-NAV.8's SW-corner deadlock at (−3.3009, 1.9100):** did **not** occur.
  No run went west of the box on entry.
- **A PolygonStop deadlock did occur:** 1 of 3, at the **NE** corner,
  (−2.5052, 2.6831).
- **The shifted goal moves the failure elsewhere.** `box_obstacle_1` now has
  a recorded PolygonStop trap at three corners:
  - NW (C2-NAV.6), with the goal at (−3.45, 2.95);
  - SW (C2-NAV.8 and C2-NAV.12), both with the goal at (−3.575, 2.95);
  - NE (C2-NAV.40), with the goal at (−3.575, 2.95).

## 7. Against the baselines (`c2nav39_tour_report.py report`)

| | C2-NAV.5 | C2-NAV.39 | **C2-NAV.40** |
|---|---|---|---|
| total | 18/21 | 30/35 (baseline ×3: 17/21) | **17/21** |
| ordinary legs | 15/15 | 25/25 | **15/15** |
| `enclosure_entry` | 2/3 | 3/5 | **0/3** |
| entry median goal err m | 0.081 | 0.086 | **1.103** |
| `enclosure_exit` | 1/3 | 2/5 | 2/3 scored, **1/1 from inside the pocket** |
| exit PolygonStop | 142.86 s (2 legs) | 176.38 s (3 legs) | 0 recorded, but one inherited ~66 s immobile hold |
| PolygonStop deadlocks | exit | 3 exits (NW pocket) | **1 (NE corner, entry → exit)** |
| min true clearance m | not recorded | 0.2423 | 0.2470 |

- **C2-NAV.8 (same goal, 200 s entry cap):** 18/21, entry 1/3, exit 2/3, one
  SW-corner deadlock of 269.5 s.
- **The historical C2-NAV.7 3/3 exit was a two-leg run from the spawn.** It
  is not reproduced in the tour. Only 1 of 3 entries reached the pocket, so
  exit reliability is measured on one attempt.
- N=3 is reproducibility, not a rate.

## 8. `mission.launch.py` regression (Task 7)

**Protocol.** One fresh headless sim (`full_world_robo.launch.py
traverse:=true gui:=false`), then `mission.launch.py rviz:=false` with web,
executive and monitor at their defaults. It is a bring-up check only: no
mission is started and no goal is sent. Checks:
- every Nav2 lifecycle node active;
- the 12 live parameter readbacks against the shipped file;
- the mission nodes present, with exactly one arbiter;
- exactly one publisher on `/diff_drive_controller/cmd_vel`;
- `/mission/state` publishing;
- the `navigate_to_pose` action server present;
- no process died.

Runs are in `~/coco_nav_runs/mission_launch_smoke/`.

**Two runs were VOID for environment reasons.** Each directory holds a
`VOID.txt`.

- **r01:** `Package 'coco_mission' not found`.
  - This worktree's overlay held only C2-NAV.39's four packages, so
    mission.launch.py never started.
  - Fixed by building `coco_config custom_teleop gazebo_models coco_sim
    coco_rl coco_perception coco_moveit_config coco_web coco_mission` into
    the same overlay: `--symlink-install`, exit 0, 9 packages.
- **r02:** `No module named 'moveit_configs_utils'`.
  - `setup_env.sh` takes the workspace as two directories above itself.
    From a worktree that is `.claude/`, so it silently skips
    `<ws>/moveit_prefix`.
  - The check now adds that prefix exactly as `setup_env.sh` does, and it
    refuses to start unless `moveit_configs_utils` imports and every mission
    package resolves into the worktree overlay.
  - The run also exposed a script bug: a run stopped early printed
    "overall PASS". It now prints INCOMPLETE.

**Tests on the rebuilt overlay:** `coco_mission` **281/0**, clean graph, cwd
= package dir.

**r03: PASS, every check** (21:39:34 → 21:42:41 UTC,
`mission_launch_smoke/c2nav40_r03`, script exit 0).

| check | result |
|---|---|
| pre-flight | `moveit_configs_utils` imports; all 7 mission packages resolve into the worktree overlay |
| sim `/scan` | 9 s |
| Nav2 lifecycle, all 10 nodes | **active after 44 s** |
| live parameter readback | **12/12 OK**, 0 mismatches, against sha256 `6f61e499…` — local CSF 65, global CSF 5, multi-pose BT, BaseObstacle 8.0, FollowPath xy 0.05, goal checker 0.25/0.25, PolygonStop 0.25/4, PolygonSlow 0.3, `resample_interval` 1 |
| mission nodes | `mission_executive`, `localization_monitor`, `mission_hud`, `ramp_driver`, `approach_server`, `grasp_server`, `move_group`, `bt_navigator`, `collision_monitor`, `target_finder` — all present (48 nodes) |
| arbiter | exactly one, `cmd_vel_arbiter` |
| `/diff_drive_controller/cmd_vel` | `geometry_msgs/msg/TwistStamped`, **publisher count 1** |
| `/mission/state` | publishing, `state=IDLE … mode=idle` — bring-up does not move the robot |
| `navigate_to_pose` | present |
| processes | none died |
| teardown | `ros_clean: 0 matched` |

**The shipped default does not break `mission.launch.py`.** Topology B brings
up with exactly the parameters the tours ran, and the arbiter is the sole
publisher to the wheels.

**Not tested here:** no mission was started and no goal sent, so this is
bring-up, not navigation, in topology B. That is §11's action.

`planner_server` logs `Inflation layer either not found or inflation is not
set sufficiently for optimized non-circular collision checking` once at
start-up. It appears once in every topology-A tour too (all three C2-NAV.40
tours and `baseline_r02`), so it comes with the shipped file and is not
specific to `mission.launch.py`.

## 9. `nav_bench.py` shutdown segfault (Task 8)

- **Results are saved before the crash (measured).** Every one of the 12 logs
  on disk that recorded a core dump (11 in `.navbench/`, plus `baseline_r01`)
  printed `[nav_bench] wrote …json` on the line immediately before it. Every
  one of those JSONs parses, with every started leg present.
- **The write order explains it (source).** `main()` closes the results JSON
  inside a `with` block before `ex.shutdown()`, `destroy_node()` and
  `rclpy.shutdown()`. Traces are written per leg, earlier still.
- **The runner turns 139 into success only on proof.** It needs the
  `wrote <run>.json` line, and it records `nav_bench_shutdown_crash` in the
  manifest.
- **C2-NAV.40:** 0 of 3 runs crashed.
- **Verdict:** it does not affect experiment validity. Safe to defer.

## 10. Decision: **REJECT**

- **The historical improvement did not reproduce.** The shifted goal did not
  make the exit reliable, because 2 of 3 entries never delivered the robot to
  the pocket.
- **It created a failure the baseline did not have.** A PolygonStop deadlock
  at `box_obstacle_1`'s NE corner, carried into the exit. Entry fell from 3/5
  to 0/3.
- **The totals are unchanged:** 17/21 against the baseline's 17/21.
  "Unchanged total" hides a worse failure mix.
- **This reproduces C2-NAV.8's verdict on the same goal.** The failure moves
  between corners; it does not go away.

The default goal is unchanged. The experiment file is removed. The goal
override tooling stays, off by default and tested.

## 11. One next implementation action

**Validate the shipped default in topology B.**
- **What:** give `nav_tour_run.sh` a topology-B mode (`nav.launch.py
  arbiter:=true` plus `arbiter.launch.py` in nav mode), then run
  `experiments/baseline.yaml` × 3 fresh.
- **Why this one:** changing the default changed `mission.launch.py`'s
  navigation, and every C2-NAV.39/.40 tour is topology A. §8 checks
  mission.launch.py bring-up only.
- **Judge it against** C2-NAV.39's topology-A 17/21 with the same report.

Every route change to the pocket so far has been rejected: the goal shift
(C2-NAV.8 and .40), a waypoint leg (C2-NAV.10), NavigateThroughPoses in the
tour (C2-NAV.12), and a heading via-pose (C2-NAV.14). No further route
change is proposed.
