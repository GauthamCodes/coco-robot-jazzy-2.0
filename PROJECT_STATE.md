# PROJECT_STATE.md

**Authoritative snapshot. A fresh agent reads this first.**
Last updated: 2026-08-16.

---

## READ ORDER FOR A FRESH SESSION

1. This file.
2. `docs/ROADMAP.md` — the active milestone.
3. `docs/SESSION_LOG.md` — **the session log lives here, not at the repo
   root.** It holds 1000+ lines of history and `CLAUDE.md` points to it.
   A second root-level log would fragment that history, so there is only
   this one.
4. `CLAUDE.md` — non-negotiable rules. Read before touching anything.
5. `git status`, then the files under **CURRENT FILES** below.

---

## MILESTONE NUMBERING — READ THIS OR YOU WILL WORK ON THE WRONG THING

Two numbering schemes exist and **they collide**:

| Scheme | Meaning | Status |
|---|---|---|
| **M0–M6** | v1, the wedge world. The fetch mission. | **CLOSED**, 19/20 measured |
| **M7** | v2, "The Yard" — randomised terrain + MuJoCo + RL baselines | Phases 1–3 done, **Phase 4 gated** |
| **C2-M1 … C2-M9** | The **COCO 2.0** plan (2026-08-16). The active track. | C2-M1 done |

"M2" is ambiguous. **Always write `C2-M2` for the COCO 2.0 plan** and
plain `M2` only for the historical v1 milestone. This file and
`docs/ROADMAP.md` use the `C2-` prefix throughout.

---

## CURRENT MILESTONE

**C2-M2 — Terrain control experiment.** Not started.

## CURRENT OBJECTIVE

Finish the terrain-control research before adding any RL. Specifically:
tip-termination correction, classical baseline re-evaluation, a grade
estimator, a friction estimator, and an observer-driven controller.

**The decision rule is fixed in advance:** train or expand RL *only* if
the observer-driven controller stays **more than 10 percentage points
below** the privileged controller on a measured task. If the observer
closes the gap, that is a successful result and RL is not added.

## MILESTONE STATUS

- **C2-M1 (observability): COMPLETE and verified.** See below.
- **C2-M2 (terrain control): NOT STARTED.** This is the current milestone.
- C2-M3…C2-M9: not started. See `docs/ROADMAP.md`.

---

## COMPLETED WORK (C2-M1)

Branch `coco2-m1-observability`, 3 commits, pushed to remote `jazzy2`.

| Artefact | What it does |
|---|---|
| `coco_mission/scripts/mission_hud.py` | Subscribes 10 status topics, renders one block on `/mission/hud` at 2 Hz. Also publishes `/mission/goal`. **Subscribe-only otherwise — it cannot affect a run.** |
| `coco_mission/test/test_mission_hud.py` | 30 tests, all passing |
| `gazebo_models/rviz/mission.rviz` | 14 displays, 3 groups, fixed frame `map`. **New file.** |
| `gazebo_models/scripts/traverse_demo.py` | Now publishes `/mission/state` (step labels) + terminal COMPLETE/FAILED/ABORT |
| `gazebo_models/scripts/ros_clean.sh` | Gained a `mission_hu[d]` pattern |
| `docs/RESULTS.md` | New section "M1 observability" with the full topic table |

`coco_robot.rviz` was **deliberately not modified** — it is loaded by
`rsp.launch.py` where `base_footprint` is the only frame that exists.

## WORK CURRENTLY IN PROGRESS

**None.** C2-M1 is closed. The tree is clean and everything is pushed.

---

## MOST RECENT VERIFIED MEASUREMENTS

From two live fetch missions, 2026-08-16, fresh simulator each, never `--fast`:

| Measurement | Value |
|---|---|
| Fetch run 2 | **COMPLETE** — blue delivered, home to **0.06 m** |
| Approach stop accuracy | base-x **0.1541** vs window centre **0.1537** = **0.4 mm**, inside the 5.5 mm window |
| Fetch run 1 | **FAILED** at step 6 (nav home); vision `found=0`, cross-track **+0.52 m** at climb end |
| AMCL covariance, stationary | ~0 (yaw term **1.09e-13**) |
| AMCL covariance, driving | sigma x **0.229 m**, y **0.167 m**, yaw **13.1 deg** |
| AMCL covariance, at platform | sigma x **0.452 m** |
| RViz config load | **0** plugin / type / QoS errors; 3 occupancy grids created (243x175 x2, 60x60) |

**1 of 2 is NOT a success rate** and is not offered as one. The standing
M6 figure is **19/20** from a dedicated matrix.

Older standing results (unchanged, from `docs/RESULTS.md`): MuJoCo
throughput **3,712 steps/s at 8 workers = 427x**; cross-engine parity
**0.138 mm** geometric; contact calibration **1.2696x**; M7 Phase 3
baselines B2 = A 98% / B 3% / C 15%.

---

## LAST VERIFIED COMMIT

```
dfcc49c  docs: M1 verification recorded, and the 361-test baseline corrected
```
Branch `worktree-coco2-m1-observability`, pushed as
`jazzy2/coco2-m1-observability`. 0 commits unpushed. Working tree clean.

Remotes: `origin` = `coco-robot-ros2`, **`jazzy2` = `coco-robot-jazzy-2.0`**
(the COCO 2.0 repo). `jazzy-harmonic-port` is in sync on both.

---

## TESTS LAST RUN (2026-08-16)

Per package, **cwd set to the package directory**:

| package | passing | failing |
|---|---|---|
| `coco_config` | 70 | 0 |
| `custom_teleop` | 67 | 0 |
| `coco_rl` | 106 | 0 |
| `coco_perception` | 44 | 0 |
| `gazebo_models` | 20 | 0 |
| `coco_moveit_config` | 12 | 0 |
| `coco_sim` | 55 | 0 |
| `coco_mission` | 30 | 0 |
| **total** | **404** | **0** |

**This holds only where `coco_sim` has been rebuilt.** Measured both
ways in the same session, which proves the cause:

| `coco_sim` build | `coco_rl` result |
|---|---|
| stale (the user's `~/ros2_ws`) | 77 passed, **29 failing** |
| fresh (this worktree's overlay) | **106 passed, 0 failing** |

**Not run:** the `launch_testing` integration test
(`gazebo_models/test_integration/`, off by default, needs
`-DBUILD_SIM_INTEGRATION_TESTS=ON`).

---

## EXACT REPRODUCTION COMMAND

```bash
# Tests — from INSIDE each package dir (see KNOWN PROBLEMS #2)
cd ~/ros2_ws/src/coco-robot-ros2/coco_mission && python3 -m pytest test -q

# The live C2-M1 verification, 4 terminals.
# Source setup_env.sh in every terminal first. NEVER --fast.
bash gazebo_models/scripts/ros_clean.sh
# T1 — fresh simulator, every run (the DetachableJoint binds once)
ros2 launch gazebo_models full_world_robo.launch.py traverse:=true gui:=false
# T2
ros2 launch coco_mission mission.launch.py \
    policy:=/home/gautham/coco_rl_runs/curriculum_20260726_211008/phase5_24deg_s0p0.zip \
    rviz:=true
# T3
ros2 run gazebo_models traverse_demo.py --colour blue
# T4
ros2 topic echo /mission/hud --field data
```

---

## KNOWN PROBLEMS

1. **The user's `~/ros2_ws` has a stale `coco_sim` build, which fails 29
   `coco_rl` tests.** Every failure is `FileNotFoundError` on
   `~/ros2_ws/build/coco_sim/worlds/yard_params.yaml` — that directory
   does not exist while the file IS in source. **Not a code regression**,
   and the fix is now **measured, not assumed**: rebuilding `coco_sim`
   takes `coco_rl` from 77/29 to **106/0**. Deliberately **not applied**
   to the user's workspace:
   ```bash
   cd ~/ros2_ws && colcon build --packages-select coco_sim
   cd ~/ros2_ws/src/coco-robot-ros2/coco_rl && python3 -m pytest test -q
   ```
   Expect `106 passed`. **Do this first in the next session** — an agent
   that skips it will see 29 red tests and may chase a phantom.

2. **Run pytest from inside each package directory.** From the repo root
   the `coco_rl/` *directory* shadows the installed `coco_rl` module.
   This is also why the six "pre-existing" flake8/pep257/copyright
   failures and 7 uncollected `coco_moveit_config` tests looked broken —
   with the correct cwd they pass. `colcon test` was never affected
   (`ament_add_pytest_test` sets `WORKING_DIRECTORY`).

3. **Fetch run 1 failed** with `+0.52 m` cross-track at climb end and
   vision `found=0`. **Not diagnosed** — variance, regression, or
   `lateral_hold` not engaging. Two runs cannot separate them.

4. **`ROBOT PITCH` read `-0.314 rad` during the platform approach**,
   where the robot should be flat. Either genuine, or `/ramp/status`'s
   `pitch` field is held from the climb while the driver is idle.
   **Not diagnosed, and it gates C2-M2** — a grade estimator would be
   built on that field.

5. `/approach/target` publishes **exactly once**, VOLATILE, at arrival.
   A subscriber connecting later never sees it.

6. `rviz_2d_overlay_plugins` is not installed, so
   `mission_hud._publish_overlay` has **never executed**. It degrades
   cleanly to the String topic. `sudo apt install ros-jazzy-rviz-2d-overlay-plugins`.

7. The rendered RViz window has **never been visually inspected**. Only
   "loads without error and receives data" is measured.

8. `docs/RSE_ASSIGNMENT_PLAN_V2.md` is **untracked and belongs to a
   different project** (an AMR fleet assignment). Not part of COCO.

---

## UNRESOLVED QUESTIONS

**Three open decisions gate M7 Phase 4** (carried forward, unchanged):

1. **Deck convergence geometry.** The deck demands up to 1.95 m of
   lateral shift in 1.80 m of travel before a 0.65 m bridge, against a
   0.40 m minimum turn radius. B1 reaches the deck 99% of the time then
   falls off the bridge 105 times in 120. **Nothing changed.**
2. **Route B viability.** Best success 8%. **39.3% of its episodes have
   mu < tan(grade) and are physically unclimbable** — no controller can
   help. Four options costed in `RESULTS.md`. **None chosen.**
3. **Route C's tip terminator.** 101/120 tips are pitch events, 0 of 101
   roll-dominated. `TIP_LIMIT` is 0.6 rad **absolute**; the 16.3 deg
   grade consumes 16.3 of it while true static rear-over is **54.5 deg**.
   **This is instrumentation, not control.** The fix (measure tip
   relative to the local surface normal) is **not applied** because
   `TIP_LIMIT` is shared with `ramp_env`, the v1 curriculum and the
   shipped policy's training conditions.

**C2-M2 begins with decision 3** — "terrain tip-termination correction"
is the same problem.

---

## CURRENT FILES

Nothing is mid-edit. The files C2-M2 will touch first:

- `coco_rl/coco_rl/baselines.py` — B0/B1/B2 and the reference path
- `coco_rl/coco_rl/baseline_eval.py` — the runner and failure taxonomy
- `coco_rl/coco_rl/ramp_env.py` — where `TIP_LIMIT` is consumed
- `coco_config/` — wherever `TIP_LIMIT` is defined (shared; see above)
- `coco_rl/coco_rl/ramp_driver.py` — publishes `pitch` on `/ramp/status`
  (problem #4 lives here)
- `docs/M7_DESIGN.md`, `docs/M7_PHASES.md` — the spec and phase blocks

---

## IMMEDIATE NEXT ACTIONS

1. **Resolve KNOWN PROBLEM #1** — `colcon build --packages-select coco_sim`,
   then re-run tests and confirm 404 passing / 0 failing.
2. **Diagnose KNOWN PROBLEM #4** (`ROBOT PITCH` during the approach)
   before building any grade estimator on that field.
3. **Take the three open decisions above**, starting with Route C's tip
   terminator, since C2-M2's first item is that same correction.
4. Then C2-M2 proper: grade estimator, friction estimator,
   observer-driven controller, and the A/B/C comparison
   (fixed / privileged / deployable).

---

## FROZEN — DO NOT CHANGE WITHOUT AN EXPLICIT INSTRUCTION

From `CLAUDE.md` §4, plus this session's additions:

- The action space `(linear, angular)`, normalised `[-1, 1]`
- `cmd_vel_arbiter`, and its position as the **sole** publisher to the controller
- Camera RPY `(0,0,0)` — two tests assert this; a −0.6 rad pitch was
  proposed and is wrong in **both sign and magnitude**
- `GRASP_SELF_COLLISION_X = 0.150` — measured by probing
  `/check_state_validity` at 1 mm steps
- The target bay geometry, and anything in `coco_perception`
- The v1 wedge world, frozen as `world_v1`
- `GOAL_SUMMIT` / `GOAL_MARGIN`
- **`gazebo_models/rviz/coco_robot.rviz`** — the TF-only view. Its
  `base_footprint` fixed frame is correct *there*. The mission view is a
  separate file.
- The training environment (`coco_rl/coco_rl/mujoco_env.py` and
  everything it touches) **must never import `rclpy`**.

**Baseline:** the immutable v1 result is M6's **19/20** fetch matrix on
the frozen `world_v1`. Any experiment changing the world, reward, robot
model, action space, controller, map or perception assumptions **must
state explicitly whether it remains comparable to that baseline.**

---

## FUTURE IDEAS (recorded, NOT to be started)

- Install `rviz_2d_overlay_plugins` and record the demo video (C2-M9).
- Make `/approach/target` TRANSIENT_LOCAL so late subscribers see it.
  Touches a load-bearing node; needs justification.
- The web panel (`coco_web`) could render `/mission/hud` directly.
