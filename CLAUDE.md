# CLAUDE.md

Repo-level engineering constraints and reproducibility rules. Read this
before touching anything. This file is for people working *on* the repo;
the project itself is introduced in `README.md`.

## State first — START HERE

**`PROJECT_STATE.md`** (repo root) is the authoritative snapshot: what is
done, what is broken, what was measured, and the known limitations.

**COCO 2.0 is frozen.** Everything is on `main` — one branch, complete.
The development-era split, where implementation lived on a feature branch
and the state files on the trunk, is over; `docs/STATE_PROTOCOL.md`
records how that worked and is kept as history, not as a live rule. A
fresh clone of `main` is sufficient. **A missing package now means a
build problem, not an unmerged branch.**

## Read first

`README.md`, `docs/ARCHITECTURE.md`, `docs/DESIGN_DECISIONS.md`,
`docs/RESULTS.md`, `docs/FUTURE_WORK.md`, `docs/M7_DESIGN.md`,
`docs/SESSION_LOG.md`.

They are long. Read them anyway. Most of what you need to avoid is already
written down, usually with the cost of learning it attached.

## What this project is

A ROS 2 Jazzy + Gazebo Harmonic mobile manipulator. A 4-wheel
differential-drive base with a 2-DOF planar arm fetches a colour-selected
cylinder from a raised platform and brings it home.

Four control paradigms hand the same wheels back and forth through
`cmd_vel_arbiter`: Nav2 on the flat, a PPO policy on the ramp, a visual servo
across the platform, MoveIt for the arm. Nine packages (eight with test
suites), `coco_config` at the bottom holding shared constants,
`coco_mission` at the top composing everything.

## Where the work is

**v1 (M0–M6) — the wedge world. CLOSED and measured.** M6, the full fetch,
completes end to end: **19 of 20** in the fetch matrix, five runs per colour,
fresh simulator each. The approach holds a **5.5 mm** window 20/20 (sd
0.6 mm) and the magnet grasp held 20/20. The single failure was run 15, which
lost the mission *after* a successful pick when AMCL drifted 3.4 m in the
deliberately unmapped corridor; DWB then scored 0 of 819 trajectories and
`bt_navigator` aborted in 1.7 s. That is a localisation failure, not a grasp
one, and it is what M7_DESIGN §2.7 item 1 (EKF) exists to fix.

(This section previously said M6 "has never completed end to end" and that
the `[0.1510, 0.1565]` window fix was unverified. Both are now false.)

**v2 (M7+) — The Yard.** The v1 policy climbs a fixed parametric wedge, which
a tuned PD could also do; `FUTURE_WORK.md` item 9(a) already says as much. M7
builds randomised multi-route terrain where learning is genuinely required,
moves RL training to headless MuJoCo for throughput, and builds classical
baselines capable of proving the policy unnecessary. Full spec in
`docs/M7_DESIGN.md`. Phase blocks in `docs/M7_PHASES.md`.

**Phases 1, 1.5 and 2 are complete**, along with the Phase 2 aftermath.
Measured and standing:

- MuJoCo training throughput **3,712 steps/s at 8 workers = 427×** real time
  on the flat model; **2,287 / 2,222 / 751** on Yard routes A / B / C, with
  Route C 3× more expensive because of its heightfield.
- Cross-engine parity **0.242 mm** worst case over 264 settle probes, of
  which 0.197 mm is a constant compliance offset — **geometric parity
  0.138 mm**.
- Contact calibration worst yaw deviation **1.2696× over seven commands**,
  inside the 1.3× target.
- Per-route open-loop feasibility: A completable, B marginal
  (friction-limited), C completable but throttle-sensitive.

**Phase 3 (classical baselines) is DONE.** Read `docs/SESSION_LOG.md`
from the most recent entry backwards before touching anything — it
carries the open decisions and the traps.

**COCO 2.0 (C2-M1 … C2-M5) — COMPLETE and FROZEN.** Observability, the
terrain observer, the mission executive, perception-driven manipulation,
and localization health + recovery. C2-M6 … C2-M9 were scoped and not
undertaken. **Two limitations are live and must not be claimed away:**
severe confident AMCL divergence is *detected* but not reliably
*recovered* to a Nav2-plannable pose, and the collision monitor's gating
still shows a short-streak residual at the wheels (0.088–2.92 % of trace
samples per tour, not attributed). The `/cmd_vel_nav` loop itself was
removed (C2-NAV.42) and re-validated in C2-NAV.43: raw-controller bypass 0.
M6's 19/20 predates the fix; **C2-NAV.44 re-measured the mission on the
fixed path — 3 of 6 fresh missions complete, bypass 0 in all six, and the
three aborts are all green-lane `PRE_RAMP_POSE_OUT_OF_REGION`, a
ground-truth arrival gate set to the same 0.25 m as Nav2's own goal
checker, not a command-path failure.** 19/20 is not a control for it: that
matrix ran `traverse_demo.py`, which has no such gate — and run on this
branch that harness **delivers the green fetch end to end**. All of this is
in `PROJECT_STATE.md` with the measurements.

**That gate is FIXED (C2-NAV.45) and the three green missions now
complete, 3/3.** `_check_nav_leg` no longer has one threshold: an arrival
Nav2 has already succeeded at is **clean** inside `xy_tolerance` (0.25 m),
**accepted, recorded and logged at WARN** inside `xy_consistency`, and a
hard failure only beyond it. The band is derived, not invented —
`GOAL_XY_CONSISTENCY = 2 x GOAL_XY_TOLERANCE = 0.50 m`, past which the
pose Nav2 steered by is wrong by more than the whole arrival window,
which is C2-M5's business. Setting `xy_consistency == xy_tolerance`
restores the old gate exactly. The measured discrepancy still happens
(0.348 / 0.315 / 0.316 m in the three runs) — it is now reported instead
of retried, because the retry was measured to command **0.000 m/s**. Do
not "tighten" this back to a single threshold, and do not fix it instead
by changing Nav2's `xy_goal_tolerance`: two tolerances at the same value
measured from two different poses is the defect. Same reasoning as
`GOAL_YAW_TOLERANCE`, one axis over. `docs/agents/C2-NAV.45_RESULTS.md`.

**The colour matrix is done (C2-NAV.46): 11 of 12 fetches, 0 void** — red
3/3, green 3/3, blue 2/3, yellow 3/3, three fresh runs per new colour on
the fixed gate, no runtime code changed. **The gate generalises** — the
0.50 m outer band was never reached and futile retries were 0 in all 12 —
but **the discrepancy it absorbs is green's alone**: green 0.315–0.348 m
against red 0.108–0.131, blue 0.066–0.085, yellow 0.030–0.047, all clean
inside the original 0.25 m. Do not generalise green's number to the other
lanes. The one failure, `r2_blue` `RETURN_FAILED`, is a **navigation**
deadlock on the *return* leg — clean pre-ramp gate at 0.066 m, successful
pick, then PolygonStop for 595.5 s with AMCL CONSISTENT and bypass 0.
3 runs per colour is **not a rate**. `docs/agents/C2-NAV.46_RESULTS.md`.

(This paragraph used to call that deadlock "classified, not diagnosed". It is
now diagnosed and fixed — see below — and **11/12 is no longer the current
baseline**, because it was measured on the value that caused it.)

**That deadlock is DIAGNOSED and FIXED (C2-NAV.48), and the fix is one
parameter: `local_costmap.robot_radius` 0.20 → 0.25.** The local costmap and
the collision monitor disagreed about which poses are navigable.
`cylinder_obstacle` sat **0.2486 m** from `base_footprint` — 1.4 mm *inside*
PolygonStop's 0.25 m circle and 43 mm *outside* the costmap's real inscribed
radius of 0.2060 m — so a 44 mm band was free to the planner and fatal to the
monitor. PolygonStop is a **circle** with `action_type "stop"`, i.e.
direction-agnostic, so nothing escaped: Nav2 commanded motion in 5,423 of the
5,955 held rows, including 905 spin and 603 backup, and the wheels moved in
**0**. **The costmap's threshold is not `robot_radius`** — `Costmap2DROS`
builds a 16-gon of circumradius `robot_radius`, pads it by
`footprint_padding`, and `LayeredCostmap` takes the apothem (C2-NAV.0
measured 0.205879 m for 0.20). 0.25 gives **0.255004 m**, 5.0 mm past the
stop circle, closing the band. **Do not "fix" this by moving PolygonStop**
(C2-NAV.6 ruled neither of its knobs should move) and **do not raise the
GLOBAL costmap** — it stays 0.20 because at `cost_scaling_factor` 5.0 it
already prices that pose at 203.6 of 254, while the local costmap's 65.0
prices it at 15.8. The defect is local, so the fix is local. A test pins all
three values. `docs/agents/C2-NAV.48_RESULTS.md`.

**The colour matrix was RE-MEASURED on 0.25 (C2-NAV.49): 12 of 12 fetches,
12 valid, 0 void — every colour 3/3.** All four lanes re-run, none carried
over, because a changed costmap parameter invalidates every lane. Bypass 0,
stale drops 0, PolygonStop 0, recoveries 0, relocalizations 0, 22/22 checks
in all twelve. **The deadlock did not recur — but the mechanism was never
exercised**: closest approach to `cylinder_obstacle` in any run was
**0.2894 m**, outside even the *old* 0.2059 m inscribed radius, so no run
entered the band and none of the twelve would have deadlocked on 0.20 either.
**This is not an A/B of the fix, and 12 runs is not a rate.** Blue is the
exposed lane by geometry (0.2894 / 0.4024 / 0.3335 m, all on the return leg,
against red's 0.6214 m minimum). Green's pre-ramp discrepancy **reproduces**
at 0.298–0.330 m and is still green's alone. Wheels above the monitor
**0.1028 %** here against C2-NAV.46's 0.0411 % — still unattributed, **not**
claimed fixed. `min_scan_m` is useless for clearance: it saturates at the
0.15 m LiDAR floor in every run. `docs/agents/C2-NAV.49_RESULTS.md`.

**Optional depth perception exists and is OFF by default** (C2-NAV.43):
`nav.launch.py depth_cloud:=true` plus an experiment `perception` block. Do
not feed a costmap the bridged `/camera/points`: its points are in the
x-forward link convention under an optical frame_id. A full-resolution depth
cloud is also not delivered over best effort. Read
`docs/agents/C2-NAV.43_RESULTS.md` first.

**Two of the four `TIP_LIMIT` homes now mean different things.** C2-M2.0
made `coco_rl/yard_env.py`'s terminator **surface-relative** — it was
measuring 34.4° against *world vertical*, so Route C's grade consumed
half the budget before the robot moved, and 101 of 120 episodes were
scored as falls 34° short of the model's measured 54.5° rear-over. The
other three (`reward.py`, `mujoco_env.py`, `ramp_driver.py`) are
**unchanged at 0.6 rad absolute** and carry the v1 curriculum, the
shipped policy and the mission's runtime check. A test asserts that
split. Do not "unify" them.

**And one physical result that bounds what any terrain estimator here
can do.** Coulomb friction is **not identifiable on this robot** from an
IMU and wheel encoders: a steady climb is in equilibrium, so the traction
ratio is pinned at `tan(grade)` whatever μ is, and the drivetrain cannot
saturate the contact on the flat (`MAX_LINEAR_ACCEL` 2.0 m/s² against
`μg` ≥ 3.43). Measured: τ spans **0.0003** across a μ span of 0.35.
Grade, by contrast, is observable to **0.1–1.4° MAE**. Before building
anything that claims to estimate friction, read the "What a robot can
know about the ground it is on" entry in `docs/DESIGN_DECISIONS.md` —
including the two formulations that were wrong in ways that *looked like
the result being sought*.

## Non-negotiable rules

### 1. Evidence discipline

This repo marks every claim `(measured)` or `(derived)`, and states plainly
when something is unverified. That property is the most valuable thing in it.
Preserve it exactly.

- Never write a number into a doc that you did not produce from a run in this
  session.
- If you did not run it, write "not yet measured".
- Do not round, extrapolate, or infer performance numbers.
- If a result is bad, report it. `--target` re-targeting is recorded as 0/5
  and later 5/14. That is the standard.

### 2. The training environment must never import `rclpy`

`coco_rl/coco_rl/mujoco_env.py` and everything it touches is pure Python +
Gymnasium + MuJoCo. No ROS, no `/clock`, no DDS, no watchdog.

This is structural, not stylistic. It is what makes the `--fast` class of
timing bug *impossible* rather than merely avoided by discipline. There is a
test asserting the module imports without ROS on the path; keep it passing.

### 3. One source of truth for robot parameters

Wheelbase, wheel radius, masses, joint limits, sensor poses: `coco_config`.
The MJCF is generated from those values. Never hand-author a second robot
model — two hand-maintained models diverge within a week and the divergence
presents as a mysterious sim-to-sim transfer gap.

### 4. Do not touch without being asked explicitly

- The action space `(linear, angular)`, normalised `[-1, 1]`
- `cmd_vel_arbiter`, and its position as **sole** publisher to the controller
- Camera RPY `(0,0,0)` — two tests assert this, and a −0.6 rad pitch was
  proposed and is wrong in both sign and magnitude
- `GRASP_SELF_COLLISION_X = 0.150` — a measured constant from probing
  `/check_state_validity` at 1 mm steps
- The target bay geometry, or anything in `coco_perception`
- The v1 wedge world, frozen as `world_v1`
- `GOAL_SUMMIT` / `GOAL_MARGIN`

### 5. Simulator hygiene

- **Never `--fast`.** Training or evaluation. Unlocking RTF makes sim time
  outrun ROS delivery, the 0.5 s `cmd_vel` watchdog pumps the wheels, and the
  chassis rears over backwards. Measured: 531/533 episodes tipped, eval 0/10.
  Without it, 0/533 tipped, 10/10 — and it ran *faster*.
- **Fresh simulator per mission run.** The gz `DetachableJoint` binds its
  child once on first spawn. A second run welds nothing and **reports
  success**.
- **Kill by process name, never launch-file name.** `full_world_robo.launch.py`
  spawns `parameter_bridge`, `robot_state_publisher` and `cmd_vel_relay` as
  separate processes whose command lines do not contain "full_world_robo".
  Orphans leave a stale `/clock`, time jumps backwards, TF buffers clear, AMCL
  never updates, and `bt_navigator` rejects every goal as "Action server is
  inactive" — four layers from the fault. Use `ros_clean.sh`. The tell is that
  each run is worse than the last.
- **Anything added to a launch file must be added to `ros_clean.sh`.**
  Its patterns are process names, and a new node's command line does not
  contain the launch file's name. `mission_hud` was added without a
  pattern and survived every sweep; two of them then published
  `/mission/hud` at once and the stale one won often enough that a field
  already fixed in the source still read wrong on the topic.
- **One Gazebo at a time**, on this machine, always. That is a rule about
  what may *run*, not a licence to kill what does: since C2-NAV.49
  `ros_clean.sh` sweeps `g[z] sim.*gazebo_models/worlds`, not a bare
  `g[z] sim`, because the bare pattern killed an unrelated
  `eyantra_kepler_colony` simulator mid-run (C2-NAV.44, measured). Every coco
  simulator still matches — `full_world_robo.launch.py` always passes a world
  out of `gazebo_models/worlds/`, in both `gui` modes — so orphan-killing is
  intact. **Do not "simplify" it back, and do not scope the sweep to the
  current session instead**: this file exists to kill orphans of *previous*
  runs, which are never in the current process group. Three tests guard it,
  in both directions.
- **`mission.launch.py` starts the control panel, so it passes
  `arbiter:=false` to `web.launch.py`.** The panel's own launch file
  starts `cmd_vel_arbiter` by default — correct when the panel is run
  alone, because otherwise its joystick moves nothing — but
  `mission.launch.py` already starts one. Letting it start a second puts
  **two** publishers on `/diff_drive_controller/cmd_vel`, and the robot
  tracks their average instead of obeying one. `web:=false` opts the
  panel out for an evaluation sweep.

### 6. Keep the package graph acyclic

`coco_config` must never depend on `gazebo_models`. Anything composing
`move_group` belongs in `coco_mission`, not `gazebo_models`. colcon refuses to
order the workspace at all if this breaks, and it has broken twice.

### 7. Ask before assuming

If a design choice is underdetermined by the docs, stop and ask. Do not pick
one and build on it. A wrong assumption compounds across a session and the
symptom usually surfaces several layers from the cause.

### 8. Tests are green or the phase is not done

**Release baseline: 829 passing, 0 failing, 0 skipped.** On this branch
it is **1139** (C2-NAV.49's 1004, plus 135 from P0.1).
`gazebo_models` carried most of the earlier growth — 41 on the release
tree, **178** here — and `coco_web` carries all of the latest. Measured on
the
release tree, per package, **with cwd set to the package directory**, on
a clean ROS graph:

| package | tests |
|---|---|
| `coco_config` | 70 |
| `custom_teleop` | 67 |
| `coco_rl` | 164 |
| `coco_perception` | 139 |
| `gazebo_models` | 41 |
| `coco_moveit_config` | 12 |
| `coco_sim` | 55 |
| `coco_mission` | 281 |
| **total** | **829** |

`coco_web` used to have no `test/` directory — pytest exited 4 there,
which was recorded here as "not a failure". **That is no longer true.**
P0.1 gave it 116 tests and, for the first time, the flake8/pep257/
copyright linters: expect **116** from `coco_web`, and note that adding
the linters is what surfaced the pre-existing docstring failures in
`web.launch.py`.

**1004 -> 1139 breakdown (P0.1).** `coco_web` 0 -> 116, `coco_mission`
311 -> 315 (the `platform:=` web-layer selection), `coco_rl` 164 -> 179
(the Docker build-context guards). Nothing else moved.

**Three invocation facts that change the total and are NOT regressions.**
All three were measured both ways.

1. **cwd must be the package directory.** From the repo root the
   `coco_rl/` *directory* shadows the installed module. This is also what
   makes the six `flake8`/`pep257`/`copyright` "pre-existing failures"
   that older revisions of this file recorded disappear — they were an
   artefact of the wrong cwd, not real breakage. Run tests as
   `ament_add_pytest_test`'s `WORKING_DIRECTORY` does.
2. **`gazebo_models` needs `--ignore=test_integration`**, or pytest dies
   importing `test_sim_bringup.launch.py` during collection and silently
   reports **0** tests for the package rather than failing loudly.
3. **The user-space MoveIt prefix must be on the path.**
   `coco_moveit_config`'s 7 `test_pick_poses` tests *skip* without it.
   `setup_env.sh` puts it there; a hand-rolled environment easily omits it.

**And one build fact.** Against a stale `coco_sim` build, 29 `coco_rl`
tests fail with `FileNotFoundError` on
`build/coco_sim/worlds/yard_params.yaml` — a directory that does not
exist while the file is present in source. Measured both ways: stale
gives 77/29, fresh 106/0. If you see the 29:

```bash
cd ~/ros2_ws && colcon build --packages-select coco_sim
```

**Run them on a clean ROS graph.** A live stack makes `coco_mission`
fail: its fixtures construct real nodes, and a second `/mission/mode`
publisher changes what they see.

Run them per package. Several packages contain identically-named test
modules (`test_copyright.py`), and a single pytest invocation across all of
them dies with `ImportPathMismatchError` before running anything.

## The web platform (P0.1)

Productization opened a new track. `docs/ROADMAP.md` Track 4 is the plan,
`docs/PRODUCT_ARCHITECTURE.md` the design, `docs/WEB_API.md` the wire
contract, `docs/DOCKER.md` the runtime. What must not be relearned:

- **The browser speaks `coco.v1`, not rosbridge.** `platform_server`
  (`coco_web`) is one ROS node serving the UI on **:8080** and the
  protocol on **/ws**. A client names an INTENT and can never name a
  topic — no frame in the schema has a field that could carry one.
  The old rosbridge panel survives one release at **:8000/legacy.html**;
  it let any tab publish any topic, which is why it is going.
- **The web layer publishes `/cmd_vel_teleop` and nothing that reaches
  the wheels.** `coco_web/coco_web/safety.py` holds the allowlist and
  `assert_publish_safe()` checks it at node CONSTRUCTION, including
  values that arrived as ROS parameters — so
  `-p teleop_topic:=/diff_drive_controller/cmd_vel` refuses to start
  rather than quietly defeating the arbiter. Verified live: the node's
  only velocity publisher is `/cmd_vel_teleop`, and the wheel topic does
  not exist on the graph.
- **`mission.launch.py platform:=`** picks the web layer, default true
  (the platform). Exactly ONE is ever selected — they collide on 8081 —
  and BOTH are included `arbiter:=false`, which was already load-bearing.
- **`/healthz` answers 503 until the stack has converged**, and Docker's
  HEALTHCHECK uses it, so "healthy" and "drivable" are one statement.
- **`/clock` alone does NOT prove the simulator is ours. Measured.** With
  no coco simulator running, a probe on this machine found
  `count_publishers('/clock') == 1` — an unrelated project's Gazebo was
  up on the same graph — and the health check reported "simulator: up".
  It now also requires `/model/coco/odometry`. The same foreign graph is
  why `ros_clean.sh` scopes its sweep, and why
  `run_platform.sh --native` REFUSES to start rather than sweeping.
- **The Docker image has never been built.** Docker is not installed on
  this machine. The Dockerfile and compose file are authored and
  statically tested (`coco_rl/test/test_docker_context.py`) but
  `docker build` has never run. Do not report it as working.
- **`.dockerignore` is NOT shell globbing.** Docker uses Go's
  `filepath.Match`, where `*` does not cross `/`; Python's `fnmatch`
  does. A bare `*.zip` therefore never touched
  `coco_rl/policies/phase5_24deg_s0p0.zip`. This cost a wrong "fix" and
  a wrong bug report before the test caught it — the test now compiles
  the pattern with Docker's semantics and pins both directions.
- **Arm and gripper limits come from `coco_config.joint_limits`**, never
  re-typed in the browser. The old panel hard-coded them in HTML and both
  had drifted: its gripper slider said `0.05..0.5` where the URDF says
  `-0.35..1.1`.
- **`gazebo_models/test/test_cmd_vel_wiring.py::TestTheOldLoopIsDetected`
  is flaky at roughly 1 run in 14**, with
  `RCLError: error creating node` in the `looped` fixture — an rclpy
  node-creation race on its dedicated domain, not a logic failure. Seen
  on the P0.1 branch, which does not touch that file or `custom_teleop`.
  Re-run before believing it; it has not been root-caused.

## Language traps already paid for

| Trap | Symptom if ignored |
|---|---|
| Bracket every `pkill` pattern (`'full_world_rob[o]'`) and run from a FILE | a `bash -c` process's own command line contains the script text, so it kills itself |
| Never edit a running bash script | bash reads lazily by byte offset; the script executes garbage mid-run |
| Camera topics are BEST_EFFORT | a RELIABLE subscriber never matches and the node goes **silently blind**. Take the flag from `robot.is_best_effort()` |
| `/diff_drive_controller/cmd_vel` carries **two** types; the arbiter publishes `TwistStamped` | a `Twist` subscriber matches nothing, receives nothing, raises nothing, and `ros2 topic info` still reads healthy. It cost C2-M3.1 a run: the recorder captured 0 commands, which reads exactly like "no stale command was issued". **Any check whose success condition is "we saw nothing" must first prove it can see something** |
| `cv_bridge`: name `'bgr8'` and `'32FC1'` explicitly | `'passthrough'` turns red into blue with **no error** |
| `rclpy.spin()` and `spin_once()` both fall back to the GLOBAL executor | "Executor is already spinning" — killed the first end-to-end fetch at step 2c |
| A welded magnet | robot drives but **cannot turn** |
| `target_finder` owns **two** topics the mission needs: `/perception/target` AND `/perception/status` | swap only the point topic and `SEARCH_TARGET` never leaves RUNNING — the gate reads `found=1` on the status line — then times out as `TARGET_NOT_FOUND`, which reads as a camera fault. Use `target_source:=` and let the launch file set both |
| Running a script from a shared scratch dir | Python puts the script's own directory at `sys.path[0]`, so a stray `numbers.py` shadows the stdlib and breaks **numpy** inside `rclpy`'s parameter service, and a stray `trace.py` **silently prints another run's output into yours**. Run instruments from a directory you control |
| `/approach/run`, `/grasp/stow`, `/grasp/pick`, `/grasp/place` are **asynchronous** | every one starts a worker thread and returns `success=True` with "watch /<name>/status" **immediately**. The Trigger reply is the ACCEPTANCE, not the outcome. Read it as the result and a 71 s grasp reports "ok" at 17 s with no approach fix, which looks exactly like a perception failure. Wait for `phase=idle` **and** a non-empty `outcome=` on the status topic. It cost C2-M4.1 a run |

## Environment

```bash
source <ws>/src/<clone>/setup_env.sh   # every terminal, first
cd <ws> && colcon build --symlink-install   # always from the ws root
```

The workspace also contains `red_ball_nav` / turtlebot3 packages;
`turtlebot3_node` fails on a missing `dynamixel_sdk` — pre-existing and
unrelated. Use `--packages-select` with the `coco*` / `custom_teleop` /
`gazebo_models` packages to avoid the noise.

| | |
|---|---|
| Repo | `coco-robot-jazzy-2.0`, cloned under `<ws>/src/`. `setup_env.sh` finds the workspace from its own path, so the clone name and location do not matter |
| Workspace root | `<ws>`, e.g. `~/ros2_ws` |
| Shipped v1 policy | `coco_rl/policies/phase5_24deg_s0p0.zip`, **in the repository** and installed to `share/coco_rl/policies/`. It is `mission.launch.py`'s `policy` default, so no path and no `COCO_POLICY` is needed to run. Original training artefact: `/home/gautham/coco_rl_runs/curriculum_20260726_211008/phase5_24deg_s0p0.zip` (identical, md5 `1421ce4a…`) |
| RL run archive | `/home/gautham/coco_rl_runs/` |

## Working style

Small commits with real messages. After each substantive change, run the
affected package's tests.

At the end of every session, append a checkpoint to `docs/SESSION_LOG.md`:
what was built, what was **measured**, what remains **unverified**, and the
exact next command to run. Follow the format already in that file.
