# HOW_TO_RUN.md

Every command here was run against this repository. If a command is not
in this file, it was not verified.

Read `CLAUDE.md` first — it carries the rules that make the difference
between a run that measures something and a run that wastes an hour.

---

## Requirements

### Required

| | |
|---|---|
| OS | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| Gazebo | Harmonic (`gz sim`) |
| Python | 3.12 |

ROS packages, all from the Jazzy binaries:

```
ros-jazzy-desktop
ros-jazzy-ros-gz
ros-jazzy-navigation2  ros-jazzy-nav2-bringup
ros-jazzy-moveit
ros-jazzy-ros2-control  ros-jazzy-ros2-controllers
ros-jazzy-cv-bridge
ros-jazzy-rmw-cyclonedds-cpp
```

Python, beyond what ROS pulls in:

```
numpy scipy pillow pyyaml
```

`numpy` and `scipy` are **required** — `localization_health.LikelihoodField`
builds the scan-vs-map distance field with `scipy.ndimage`, and the
localization monitor will not start without them. Verified present:
numpy 1.26.4, scipy 1.11.4.

### Optional

| | for |
|---|---|
| `stable-baselines3`, `gymnasium` | the RL policy in `coco_rl`. Only needed to *train*; running a shipped `.zip` policy needs them too |
| `mujoco` | M7 headless training and the cross-engine parity probes |
| `matplotlib` | the analysis scripts' `--plot` flags |

The workspace also contains `red_ball_nav` / turtlebot3 packages;
`turtlebot3_node` fails on a missing `dynamixel_sdk`. **Pre-existing and
unrelated** — use `--packages-select` to avoid it.

---

## Clone

Everything is on `main`. No other branch is needed.

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/GauthamCodes/coco-robot-jazzy-2.0.git
```

The clone directory name does not matter to the build, but `setup_env.sh`
locates the workspace from its own path (`<ws>/src/<clone>/setup_env.sh`),
so keep the clone two levels under the workspace root as above.

---

## Build

Always from the workspace root, never from a package directory.

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select \
    coco_config coco_sim coco_rl coco_perception \
    coco_moveit_config custom_teleop gazebo_models coco_mission coco_web
```

`--packages-select` is not optional if your workspace holds anything else:
this one also carries `red_ball_nav` / turtlebot3, and `turtlebot3_node`
fails on a missing `dynamixel_sdk`.

---

## Source

Every terminal, first, before anything else:

```bash
source ~/ros2_ws/src/coco-robot-jazzy-2.0/setup_env.sh
```

It sources ROS 2 Jazzy, this workspace's overlay, CycloneDDS on loopback,
Gazebo Harmonic, the user-space MoveIt prefix if one is present, and a
render-engine fallback for when the NVIDIA driver is not loaded. It finds
the workspace from its own path, so a clone anywhere works.

The RL climb policy is passed to the mission by path. Export it once per
terminal so the commands below can be copied verbatim:

```bash
export COCO_POLICY=~/coco_rl_runs/curriculum_20260726_211008/phase5_24deg_s0p0.zip
```

---

## Launch the simulation

**One Gazebo at a time, on this machine, always.** A fresh simulator for
every mission run — the `DetachableJoint` binds its child on first spawn,
and a second run welds nothing and *reports success*.

```bash
ros2 launch gazebo_models full_world_robo.launch.py traverse:=true gui:=false
```

| argument | values | meaning |
|---|---|---|
| `traverse` | `true` / `false` | adds the platform and the far slope, turning the climb into an up-over-down route. The fetch mission needs `true` |
| `gui` | `true` / `false` | the Gazebo GUI. `false` for measurement runs |
| `ramp_angle` | degrees | wedge grade. Default 18 |
| `world` | path | the world file |
| `use_sim_time` | `true` / `false` | |

**Never pass `--fast`, and there is deliberately no argument for it.**
Unlocking RTF makes sim time outrun ROS delivery, the 0.5 s `cmd_vel`
watchdog pumps the wheels, and the chassis rears over backwards.
Measured: 531/533 episodes tipped, eval 0/10. Without it, 0/533 tipped,
10/10 — and it ran *faster*.

---

## RViz

Two configurations, both in `gazebo_models/rviz/`.

Clean mission view, brought up with the stack:

```bash
ros2 launch coco_mission mission.launch.py rviz:=true rviz_config:=mission
```

Debug view — costmaps, particle cloud, TF:

```bash
ros2 launch coco_mission mission.launch.py rviz:=true rviz_config:=mission_debug
```

RViz standalone, against a stack that is already up:

```bash
rviz2 -d $(ros2 pkg prefix gazebo_models)/share/gazebo_models/rviz/mission.rviz
```

**Measurement runs use `rviz:=false`.** KNOWN PROBLEMS 1 and 3b both
carry a Gazebo + RViz + `move_group` confound, and every C2-M5.0 and
C2-M5.1 number was taken with RViz off.

---

## A normal autonomous mission

Three terminals. T1 is the simulator above; T2 and T3 follow.

**T2 — the stack:**

```bash
ros2 launch coco_mission mission.launch.py rviz:=false \
    target_source:=target_pose target_colour:=blue \
    policy:="$COCO_POLICY"
```

| argument | default | meaning |
|---|---|---|
| `target_colour` | `blue` | which cylinder to fetch. Picks the lane |
| `target_source` | `target_finder` | `target_pose` uses the C2-M4 measured-pose pipeline. Sets **both** `point_topic` and `status_compat_topic` — setting one without the other is the C2-M4.2 defect |
| `policy` | — | the RL climb policy `.zip` |
| `executive` | `true` | `false` to run `traverse_demo.py` instead |
| `mission_autostart` | `false` | start without waiting for `/mission/start`. Bringing the stack up should not move the robot |
| `localization_monitor` | `true` | publish `/localization/health` |
| `localization_recovery` | `true` | let the executive *act* on it |
| `lateral_hold` | `true` | hold the lane centreline during the climb |
| `rviz` / `rviz_config` | `true` / `mission` | see above |

**T3 — check the invariants, then start.** Each of these has cost a run
before:

```bash
ros2 lifecycle get /amcl                          # must read: active [3]
ros2 topic info /perception/target                # Publisher count: 1
ros2 topic info /perception/status                # Publisher count: 1
ros2 topic info /localization/health              # Publisher count: 1
ros2 topic info /diff_drive_controller/cmd_vel    # Publisher count: 1

ros2 service call /mission/start std_srvs/srv/Trigger
```

Watch it:

```bash
ros2 topic echo /mission/state
ros2 topic echo /localization/health
ros2 service call /mission/abort std_srvs/srv/Trigger    # operator stop
```

The mission ends in `COMPLETE` or `ABORT`, and `ABORT` always carries a
reason. A verified nominal run takes about **184 s** wall.

---

## Terrain-control demonstration

The terrain observer publishes grade and traction from the IMU and the
wheels. It publishes only; it drives nothing.

```bash
ros2 run coco_rl terrain_observer
ros2 topic echo /terrain/state
```

The C2-M2 live gate, which is the measured version:

```bash
cd docs/data && python3 c2m2_live_gate.py --help
```

---

## Perception demonstration

The target-pose pipeline on its own, against a running simulator:

```bash
ros2 launch coco_perception perception.launch.py target_source:=target_pose
ros2 topic echo /perception/target
ros2 topic echo /perception/status
```

**Exactly one node may own `/perception/target`.** Run `target_pose_node`
or `target_finder`, never both — check the publisher count above.

The 60-placement benchmark:

```bash
cd docs/data && python3 c2m4_localisation.py --benchmark \
    --frames 12 --out c2m4_benchmark.csv
python3 c2m4_analysis.py c2m4_benchmark.csv
```

One live grasp, which needs a **fresh simulator** each time:

```bash
cd docs/data && python3 c2m4_grasp.py --colour blue --standoff 0.45 \
    --lateral 0.0 --out c2m4_grasp.csv
```

---

## Localization recovery demonstration

The health monitor detects a scan-vs-map divergence, the executive
safe-stops through the arbiter, re-seeds AMCL, spins to re-observe, and
resumes only when the monitor independently reports health again.

**Watch the signal on a healthy mission** (it should stay quiet):

```bash
ros2 topic echo /localization/health
```

**Record a run**, in a third terminal, before calling `/mission/start`:

```bash
cd docs/data
python3 c2m51_hrec.py --out run.csv --tag myrun --hz 10 --stop-on-terminal
```

**Inject the C2-M5.0 class-A divergence** — a 3 m pose error with a tight
covariance, fired when the mission reaches `RETURN_HOME`:

```bash
cd docs/data
python3 c2m51_inject.py --state RETURN_HOME --dy -3.0 --dyaw 0.0
```

`--dyaw 0.4` reproduces the `diverged1` variant, which also carries a
heading error. `--repeat` keeps injecting, which is how the failed-
recovery path was exercised; `--state RETURN_HOME,RELOCALIZE` keeps it
firing during the recovery itself.

**Score it:**

```bash
python3 c2m51_hrec.py --summarise run.csv
```

That prints, per mission state, how many samples were on mapped ground,
the worst and median scan-vs-map distance, and — the number that matters
— `DISTINCT RECOVERY TRIGGERS`.

**Read the result honestly.** On a healthy mission this must be 0. The
recovery restores the health signal; on the injected 3 m divergence it
does **not** reliably restore a pose Nav2 can plan from, and the mission
then aborts safely with an explicit reason. See `docs/RESULTS.md`,
"C2-M5.1", and `docs/data/c2m51_planner_after_recovery.txt`.

To run a mission with the signal published but **not acted on** — how
the false-positive check was done:

```bash
ros2 launch coco_mission mission.launch.py rviz:=false \
    localization_monitor:=true localization_recovery:=false \
    target_source:=target_pose policy:="$COCO_POLICY"
```

---

## Test suite

**Per package, from inside that package's directory, on a clean ROS
graph.** Several packages contain identically-named test modules
(`test_copyright.py`), and one pytest invocation across the workspace
dies with `ImportPathMismatchError` before running anything.

```bash
source ~/ros2_ws/src/coco-robot-jazzy-2.0/setup_env.sh
bash gazebo_models/scripts/ros_clean.sh          # clean graph first
cd ~/ros2_ws/src/coco-robot-jazzy-2.0/coco_mission && python3 -m pytest -q
```

Repeat for each package. `gazebo_models` needs
`--ignore=test_integration`, or pytest dies importing
`test_sim_bringup.launch.py` during collection and silently reports **0**
tests for the whole package rather than failing loudly:

```bash
cd ~/ros2_ws/src/coco-robot-jazzy-2.0/gazebo_models
python3 -m pytest -q --ignore=test_integration
```

Verified counts, clean graph, release tree:

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
| **total** | **829 passing, 0 failing** |

`coco_web` has no `test/` directory; pytest exits 4 there and that is not
a failure.

**Run them on a clean graph.** A live stack makes `coco_mission` fail —
the node fixtures construct real nodes and a second `/mission/mode`
publisher changes what they see.

---

## Cleanup

```bash
bash $(ros2 pkg prefix gazebo_models)/lib/gazebo_models/ros_clean.sh
```

or from the checkout:

```bash
bash gazebo_models/scripts/ros_clean.sh
bash gazebo_models/scripts/ros_clean.sh --list   # show, kill nothing
```

Then confirm:

```bash
pgrep -af 'gz sim|gazebo|rviz2|coco_mission|nav2|amcl|ros2'
```

Only `ros2-daemon` should remain.

---

## Troubleshooting

Only problems actually observed in this repository.

### Every Nav2 goal is rejected: "Action server is inactive"

Orphaned processes from a previous run. `full_world_robo.launch.py`
spawns `parameter_bridge`, `robot_state_publisher` and `cmd_vel_relay` as
separate processes whose command lines do **not** contain
"full_world_robo", so killing the launch pattern leaves them running. A
stale `/clock` makes time jump backwards, TF buffers clear, AMCL never
updates, and `bt_navigator` rejects everything — four layers from the
fault.

**The tell is that each run is worse than the last.** Run `ros_clean.sh`.

### A stale build

`--symlink-install` does not help a package whose `CMakeLists.txt`
changed, and a new script is not installed until you rebuild. If a node
you just added is "not found", rebuild before debugging anything else.

### Duplicate publisher

Check the counts in the invariants block above. Two `mission_hud`s, two
`mission_executive`s, two `target_pose_node`s or two
`localization_monitor`s all produce symptoms far from the cause — an
orphan asserting a stale value that the live one keeps overwriting.
`ros_clean.sh` kills all of them by process name.

### `ros2 launch coco_mission ...` says the package does not exist

The overlay was not sourced, or was sourced before the build. Order
matters: `colcon build` from the workspace root, **then**
`source setup_env.sh` in each terminal. `setup_env.sh` prints
`note: no overlay at <ws>/install — run colcon build` when the workspace
has not been built yet, and that note is the whole diagnosis.

During development this package lived on an unmerged branch and was built
into a side overlay. That is no longer true — `main` carries every
package, and a plain workspace build is all that is required.

### Target-pose source selection

`/perception/target` must have exactly one publisher. `target_source`
sets **both** `point_topic` and `status_compat_topic`; setting one
without the other leaves the executive's `SEARCH_TARGET` gate reading a
topic nobody is publishing, and the mission times out there.

### The localization monitor reports UNKNOWN for a whole mission

Either `/map` never arrived — it is latched `TRANSIENT_LOCAL` and
published once, so check `map_server` came up — or the scan subscription
never matched. `/scan` is **BEST_EFFORT**; a RELIABLE subscriber never
matches and the node goes silently blind.

### The localization monitor triggers while the robot is stationary

Fixed in C2-M5.1, and recorded because the diagnosis is reusable: AMCL
publishes `/amcl_pose` only after the robot has moved `update_min_d`
(0.25 m) or turned `update_min_a` (0.2 rad). A 50 s stationary grasp ages
that topic without bound. The gap on an **event-driven** topic is not a
staleness test; `map->odom` freshness is, because AMCL republishes that
on its own schedule.

### `/cmd_vel_nav` has seven publishers

Known and documented, **not fixed**. `nav2_bringup` remaps
`controller_server` to the topic the arbiter reads, and `cmd_vel_relay`
feeds the collision monitor's output back into the velocity smoother's
input. The consequence is that the collision monitor's gating does not
reach the wheels. See `docs/RESULTS.md` and `PROJECT_STATE.md`
UNRESOLVED QUESTIONS. Do not assume the collision monitor can stop this
robot.
