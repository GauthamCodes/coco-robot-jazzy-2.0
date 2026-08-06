# CLAUDE.md

Repo-level instructions. Read this before touching anything.

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
across the platform, MoveIt for the arm. Eight packages, `coco_config` at the
bottom holding shared constants, `coco_mission` at the top composing
everything.

## Where the work is

**v1 (M0–M6) — the wedge world.** M0–M5 are closed and measured. M6, the full
fetch, is written, committed and unit-tested but **has never completed end to
end**. Its last known failure was the approach stopping at base-x 0.1443,
inside the arm's measured self-collision bound of 0.150. The window was
tightened to `[0.1510, 0.1565]` in code and **that fix is unverified**.

**v2 (M7+) — The Yard.** In progress. The v1 policy climbs a fixed parametric
wedge, which a tuned PD could also do; `FUTURE_WORK.md` item 9(a) already says
as much. M7 builds randomised multi-route terrain where learning is genuinely
required, moves RL training to headless MuJoCo for throughput, and builds
classical baselines capable of proving the policy unnecessary. Full spec in
`docs/M7_DESIGN.md`. Phase blocks in `docs/M7_PHASES.md`.

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
- **One Gazebo at a time**, on this machine, always.

### 6. Keep the package graph acyclic

`coco_config` must never depend on `gazebo_models`. Anything composing
`move_group` belongs in `coco_mission`, not `gazebo_models`. colcon refuses to
order the workspace at all if this breaks, and it has broken twice.

### 7. Ask before assuming

If a design choice is underdetermined by the docs, stop and ask. Do not pick
one and build on it. A wrong assumption compounds across a session and the
symptom usually surfaces several layers from the cause.

### 8. Tests are green or the phase is not done

Baseline: 250 tests, 0 failures across `coco_config` 57, `custom_teleop` 67,
`coco_rl` 50, `coco_perception` 44, `gazebo_models` 20, `coco_moveit_config`
12. That number only goes up.

## Language traps already paid for

| Trap | Symptom if ignored |
|---|---|
| Bracket every `pkill` pattern (`'full_world_rob[o]'`) and run from a FILE | a `bash -c` process's own command line contains the script text, so it kills itself |
| Never edit a running bash script | bash reads lazily by byte offset; the script executes garbage mid-run |
| Camera topics are BEST_EFFORT | a RELIABLE subscriber never matches and the node goes **silently blind**. Take the flag from `robot.is_best_effort()` |
| `cv_bridge`: name `'bgr8'` and `'32FC1'` explicitly | `'passthrough'` turns red into blue with **no error** |
| `rclpy.spin()` and `spin_once()` both fall back to the GLOBAL executor | "Executor is already spinning" — killed the first end-to-end fetch at step 2c |
| A welded magnet | robot drives but **cannot turn** |

## Environment

```bash
source ~/ros2_ws/src/coco-robot-ros2/setup_env.sh   # every terminal, first
cd ~/ros2_ws && colcon build --symlink-install       # always from ws root
```

The workspace also contains `red_ball_nav` / turtlebot3 packages;
`turtlebot3_node` fails on a missing `dynamixel_sdk` — pre-existing and
unrelated. Use `--packages-select` with the `coco*` / `custom_teleop` /
`gazebo_models` packages to avoid the noise.

| | |
|---|---|
| Repo | `/home/gautham/ros2_ws/src/coco-robot-ros2` |
| Workspace root | `/home/gautham/ros2_ws` |
| Shipped v1 policy | `/home/gautham/coco_rl_runs/curriculum_20260726_211008/phase5_24deg_s0p0.zip` |
| RL run archive | `/home/gautham/coco_rl_runs/` |

## Working style

Small commits with real messages. After each substantive change, run the
affected package's tests.

At the end of every session, append a checkpoint to `docs/SESSION_LOG.md`:
what was built, what was **measured**, what remains **unverified**, and the
exact next command to run. Follow the format already in that file.
