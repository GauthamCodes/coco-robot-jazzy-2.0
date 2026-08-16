# PROJECT_STATE.md

**Authoritative snapshot. A fresh agent reads this first.**
Lives on the trunk (`jazzy-harmonic-port`) and is edited **only** there —
see `docs/STATE_PROTOCOL.md`.

Last updated: 2026-08-17.

---

## BRANCH MAP — READ BEFORE `git status` CONFUSES YOU

The trunk does **not** contain all completed work. This table is what
makes a trunk-only state file honest.

| Branch | Contains | Merged? |
|---|---|---|
| `jazzy-harmonic-port` | **the trunk.** Everything through M7 Phase 3, plus this state layer | — |
| `coco2-m1-observability` | **C2-M1 complete**: `mission_hud`, `mission.rviz`, `/mission/state`, 30 tests, the M1 results | **NO — unmerged** |

Remotes: `origin` = `coco-robot-ros2`, **`jazzy2` = `coco-robot-jazzy-2.0`**
(the COCO 2.0 repo). Both carry the trunk. `coco2-m1-observability` is on
`jazzy2` only.

**If you are looking for `mission_hud.py` and it is not there, you are on
the trunk and it has not been merged yet.** That is expected, not a bug:

```bash
git checkout coco2-m1-observability   # to work on / review C2-M1
```

---

## READ ORDER FOR A FRESH SESSION

1. This file.
2. `docs/STATE_PROTOCOL.md` — which branch owns which file. Short.
3. `docs/ROADMAP.md` — the active milestone.
4. `docs/SESSION_LOG.md` — **the session log lives here, not at the repo
   root.** 1000+ lines of history; `CLAUDE.md` points here. A second
   root-level log would fragment it.
5. `CLAUDE.md` — non-negotiable rules. Read before touching anything.
6. `git status`, then the files under **CURRENT FILES** below.

---

## MILESTONE NUMBERING — READ THIS OR YOU WILL WORK ON THE WRONG THING

Two schemes exist and **they collide**:

| Scheme | Meaning | Status |
|---|---|---|
| **M0–M6** | v1, the wedge world. The fetch mission. | **CLOSED**, 19/20 measured |
| **M7** | v2, "The Yard" — randomised terrain, MuJoCo, RL baselines | Phases 1–3 done, **Phase 4 gated** |
| **C2-M1 … C2-M9** | The **COCO 2.0** plan. The active track. | C2-M1 done (unmerged) |

"M2" is ambiguous. **Always write `C2-M2` for the COCO 2.0 plan** and
plain `M2` only for the historical v1 milestone.

---

## CURRENT MILESTONE

**C2-M2 — Terrain control experiment.** Not started.

## CURRENT OBJECTIVE

Finish the terrain-control research before adding any RL:
tip-termination correction, classical baseline re-evaluation, a grade
estimator, a friction estimator, an observer-driven controller.

**Decision rule, fixed in advance:** expand RL *only* if the
observer-driven controller stays **more than 10 percentage points below**
the privileged controller on a measured task. If the observer closes the
gap, **that is the successful result** and RL is not added.

## MILESTONE STATUS

- **C2-M1 (observability): COMPLETE and verified — on branch
  `coco2-m1-observability`, not merged.**
- **C2-M2 (terrain control): NOT STARTED.** Current milestone.
- C2-M3…C2-M9: not started. See `docs/ROADMAP.md`.

---

## COMPLETED WORK (C2-M1) — on `coco2-m1-observability`

| Artefact | What it does |
|---|---|
| `coco_mission/scripts/mission_hud.py` | Subscribes 10 status topics → one block on `/mission/hud` at 2 Hz; also publishes `/mission/goal`. **Subscribe-only otherwise — it cannot affect a run.** |
| `coco_mission/test/test_mission_hud.py` | 30 tests, all passing |
| `gazebo_models/rviz/mission.rviz` | 14 displays, 3 groups, fixed frame `map`. **New file.** |
| `gazebo_models/scripts/traverse_demo.py` | Publishes `/mission/state` + terminal COMPLETE/FAILED/ABORT |
| `gazebo_models/scripts/ros_clean.sh` | Gained a `mission_hu[d]` pattern |
| `docs/RESULTS.md` | Section "M1 observability" — the full topic table |

`coco_robot.rviz` was **deliberately not modified** — `rsp.launch.py`
loads it where `base_footprint` is the only frame that exists.

## WORK CURRENTLY IN PROGRESS

**None.** C2-M1 is closed. Both branches are clean and pushed.

---

## MOST RECENT VERIFIED MEASUREMENTS

Two live fetch missions, 2026-08-16, fresh simulator each, never `--fast`:

| Measurement | Value |
|---|---|
| Fetch run 2 | **COMPLETE** — blue delivered, home to **0.06 m** |
| Approach stop accuracy | base-x **0.1541** vs window centre **0.1537** = **0.4 mm**, inside the 5.5 mm window |
| Fetch run 1 | **FAILED** at step 6 (nav home); vision `found=0`, cross-track **+0.52 m** at climb end |
| AMCL covariance, stationary | ~0 (yaw term **1.09e-13**) |
| AMCL covariance, driving | sigma x **0.229 m**, y **0.167 m**, yaw **13.1 deg** |
| AMCL covariance, at platform | sigma x **0.452 m** |
| RViz config load | **0** plugin / type / QoS errors; 3 occupancy grids created |

**1 of 2 is NOT a success rate** and is not offered as one. The standing
M6 figure is **19/20** from a dedicated matrix.

Standing from earlier phases: MuJoCo **3,712 steps/s at 8 workers =
427x**; cross-engine parity **0.138 mm** geometric; contact calibration
**1.2696x**; M7 Phase 3 B2 = A 98% / B 3% / C 15%.

---

## LAST VERIFIED COMMIT

- **Trunk** `jazzy-harmonic-port`: `33110a6` + this state layer.
- **C2-M1** `coco2-m1-observability`: `625a659`, pushed to `jazzy2`.

---

## TESTS LAST RUN (2026-08-16)

Per package, **cwd set to the package directory**. On
`coco2-m1-observability`, with `coco_sim` rebuilt:

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

On the **trunk**, `coco_mission` does not exist yet, so the trunk total
is **374**. The 30 arrive with the C2-M1 merge.

**404/374 holds only where `coco_sim` has been rebuilt.** Measured both
ways, which proves the cause:

| `coco_sim` build | `coco_rl` |
|---|---|
| stale (the user's `~/ros2_ws`) | 77 passed, **29 failing** |
| fresh | **106 passed, 0 failing** |

**Not run:** the `launch_testing` integration test
(`gazebo_models/test_integration/`, off by default, needs
`-DBUILD_SIM_INTEGRATION_TESTS=ON`).

---

## EXACT REPRODUCTION COMMAND

```bash
# Tests — from INSIDE each package dir (see KNOWN PROBLEMS #2)
cd ~/ros2_ws/src/coco-robot-ros2/coco_rl && python3 -m pytest test -q

# The live C2-M1 verification (needs `git checkout coco2-m1-observability`).
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

1. **The user's `~/ros2_ws` has a stale `coco_sim` build → 29 red
   `coco_rl` tests.** Every failure is `FileNotFoundError` on
   `~/ros2_ws/build/coco_sim/worlds/yard_params.yaml` — a directory that
   does not exist while the file IS in source. **Not a code regression**,
   and the fix is **measured, not assumed**: rebuilding takes `coco_rl`
   from 77/29 to 106/0. Deliberately **not applied**:
   ```bash
   cd ~/ros2_ws && colcon build --packages-select coco_sim
   cd ~/ros2_ws/src/coco-robot-ros2/coco_rl && python3 -m pytest test -q
   ```
   Expect `106 passed`. **Do this first** — an agent that skips it sees
   29 red tests and may chase a phantom.

2. **Run pytest from inside each package directory.** From the repo root
   the `coco_rl/` *directory* shadows the installed module. This is also
   why six "pre-existing" lint failures and 7 uncollected
   `coco_moveit_config` tests looked broken — with the correct cwd they
   pass. `colcon test` was never affected.

3. **Fetch run 1 failed** with `+0.52 m` cross-track at climb end and
   vision `found=0`. **Not diagnosed** — variance, regression, or
   `lateral_hold` not engaging. Two runs cannot separate them.

4. **`ROBOT PITCH` read `-0.314 rad` during the platform approach**,
   where the robot should be flat. Either genuine, or `/ramp/status`'s
   `pitch` is held from the climb while the driver is idle.
   **Not diagnosed, and it gates C2-M2** — a grade estimator would be
   built on that field.

5. `/approach/target` publishes **exactly once**, VOLATILE, at arrival.

6. `rviz_2d_overlay_plugins` is not installed, so
   `mission_hud._publish_overlay` has **never executed**. It degrades
   cleanly to the String topic.

7. The rendered RViz window has **never been visually inspected**.

8. `docs/RSE_ASSIGNMENT_PLAN_V2.md` is **untracked and belongs to a
   different project** (an AMR fleet assignment). Not part of COCO.

---

## UNRESOLVED QUESTIONS

**Three open decisions gate M7 Phase 4** (carried forward, unchanged):

1. **Deck convergence geometry** — 1.95 m lateral shift in 1.80 m of
   travel before a 0.65 m bridge, against a 0.40 m turn radius. B1
   reaches the deck 99% of the time then falls off the bridge 105 times
   in 120. **Nothing changed.**
2. **Route B viability** — best success 8%; **39.3% of episodes have
   mu < tan(grade) and are physically unclimbable**. Four options costed
   in `RESULTS.md`. **None chosen.**
3. **Route C's tip terminator** — 101/120 tips are pitch events, 0 of 101
   roll-dominated. `TIP_LIMIT` is 0.6 rad **absolute**; the 16.3 deg
   grade consumes 16.3 of it while true static rear-over is **54.5 deg**.
   **Instrumentation, not control.** Fix not applied because `TIP_LIMIT`
   is shared with `ramp_env`, the v1 curriculum and the shipped policy.

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

1. **Merge `coco2-m1-observability` into the trunk**, or decide not to.
   Until then the trunk has no `coco_mission` package. Nothing else
   depends on it, so this is not blocking — but the BRANCH MAP above
   must be updated when it happens.
2. **Resolve KNOWN PROBLEM #1** — `colcon build --packages-select coco_sim`.
3. **Diagnose KNOWN PROBLEM #4** (`ROBOT PITCH` during the approach)
   before building any grade estimator on that field.
4. **Take the three open decisions**, starting with Route C's tip
   terminator, since C2-M2's first item is that same correction.
5. Then C2-M2 proper: grade estimator, friction estimator,
   observer-driven controller, and the A/B/C comparison
   (fixed / privileged / deployable).

---

## FROZEN — DO NOT CHANGE WITHOUT AN EXPLICIT INSTRUCTION

From `CLAUDE.md` §4, plus additions:

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
  `base_footprint` fixed frame is correct *there*.
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
