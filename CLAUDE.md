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
*recovered* to a Nav2-plannable pose, and the `/cmd_vel_nav` topic loop
means the collision monitor's gating does not reach the wheels. Both are
in `PROJECT_STATE.md` with the measurements.

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
- **One Gazebo at a time**, on this machine, always.
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

**Release baseline: 829 passing, 0 failing, 0 skipped.** Measured on the
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

`coco_web` has no `test/` directory; pytest exits 4 there, and that is
not a failure.

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
