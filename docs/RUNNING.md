# Running the Coco robot — quickstart

Everything below was verified end-to-end on this machine (Ubuntu 24.04,
ROS 2 Jazzy, gz-sim 8.11) on 2026-07-04. **Every terminal needs the env
script first:**

```bash
source ~/ros2_ws/src/coco-robot-ros2/setup_env.sh
```

It sources ROS + the workspace, sets CycloneDDS-on-loopback, picks a
working render engine (NVIDIA if the driver is up, otherwise Intel iGPU),
and wires in the user-space MoveIt/rosbridge prefix.

Build (once, or after edits):

```bash
cd ~/ros2_ws
colcon build --symlink-install \
  --packages-select gazebo_models custom_teleop coco_config coco_moveit_config coco_web coco_rl
```

---

## Demo 1 — Simulation + keyboard teleop

```bash
# T1
ros2 launch gazebo_models full_world_robo.launch.py          # gui:=false headless
# T2
ros2 run custom_teleop teleop_wheels_node                    # w/s/a/d, x stop
# T3 (optional)
ros2 run custom_teleop teleop_arm_node                       # w/s e/d r/f
```

Expect: robot upright at (-2,0), RTF ≈ 1.0, four controllers active
(`ros2 control list_controllers`). One-shot health check of the whole
graph (sensor rates measured in sim time, works at any RTF):

```bash
python3 ~/ros2_ws/src/coco-robot-ros2/gazebo_models/scripts/verify_sim.py
```

## Demo 2 — SLAM mapping

```bash
# T1: sim (above)   T2:
ros2 launch gazebo_models slam.launch.py
# T3: closed-loop waypoint mapping drive around the whole arena
#     (south lane -> east half behind the ramp -> north lane -> home)
python3 ~/ros2_ws/src/coco-robot-ros2/gazebo_models/scripts/map_drive.py
# save when done:
ros2 run nav2_map_server map_saver_cli \
  -f ~/ros2_ws/src/coco-robot-ros2/gazebo_models/maps/coco_world \
  --ros-args -p use_sim_time:=true
```

A saved map ships with the repo, so this is optional. **Map from a fresh
sim session**: slam_toolbox anchors the map frame at the *odom* pose it
sees on startup, and `nav.launch.py` auto-initialises AMCL assuming map
origin = spawn pose. If the robot has already driven around (skid-steer
odometry drifts, especially through in-place turns), restart the sim
before mapping or AMCL will initialise in the wrong place.

## Demo 3 — Autonomous navigation (Nav2)

Robot must start at the spawn pose (map origin = SLAM start pose).

```bash
# T1: sim   T2:
ros2 launch gazebo_models nav.launch.py
# T3: send a goal (or click in RViz / the web panel)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

Goals in gray (unobserved) map areas are rejected by the planner — pick
points inside the white region.

## Demo 4 — MoveIt2 pick-and-place

```bash
# T1: sim   T2:
ros2 launch coco_moveit_config move_group.launch.py     # wait for "You can start planning now!"
# T3:
ros2 run coco_moveit_config pick_place.py
```

Spawns a pedestal + red cylinder behind the robot (the arm works at the
rear), mirrors them into the planning scene, and runs 9 collision-checked
motions: up → open → stage scene → hover → grasp → close → raise → lift →
place → open → hover → home. The cylinder is genuinely carried — grasped,
lifted through the arc, and set back down on the pedestal (fingertip
end-stop lips keep it caged). Ground-truth pose checks before and after
the run catch any physics blow-up. Re-runs are safe: the script clears
stale scene objects and re-spawns the props itself. Optional:
`pick_place.py --target X Z` re-targets the grasp anywhere the analytic
IK (`arm_ik.py`) finds reachable.

## Demo 5 — Web control panel

```bash
# T1: sim   (T2: nav.launch.py if you want the map + click-to-goal)   T3:
ros2 launch coco_web web.launch.py
```

Open `http://<laptop-ip>:8000` from any device on your WiFi:
joystick, arm/gripper sliders, live camera, click-to-navigate map,
Teleop/Autonomous toggle. Camera stream is `:8081`, rosbridge is `:9090`.

## Demo 6 — RL training (ramp traversal)

```bash
# T1:
ros2 launch gazebo_models full_world_robo.launch.py gui:=false
# T2:
python3 -m coco_rl.train_ppo --steps 1024          # smoke test, ~3 min
python3 -m coco_rl.train_ppo --steps 200000 --fast # real run
```

`--fast` unlocks the physics real-time-factor cap for the duration of the
run (restored on exit); the env steps on **sim time** via the ground-truth
odometry stamps, so training speed scales with whatever RTF the machine
manages. Rewards use ground-truth pose (`/model/coco/odometry`), falling
back to wheel odometry if the plugin topic is absent. Progress lands in a
Monitor CSV (`--out` prefix) with periodic checkpoints every 25k steps.

More knobs:

```bash
# continue a previous run (step counter + optimizer state preserved)
python3 -m coco_rl.train_ppo --steps 75000 --resume ppo_model.zip --fast
# domain randomization: spawn lateral offset and yaw vary per episode
python3 -m coco_rl.train_ppo --steps 200000 --randomize --fast
# deterministic evaluation -> per-episode outcomes + success rate
python3 -m coco_rl.evaluate ppo_model.zip --episodes 10 --fast
# learning-curve PNG from the Monitor CSV(s); -o is required and must be .png
python3 -m coco_rl.plot_curve run.monitor.csv -o curve.png
```

**Long runs: detach and write outside `/tmp`.** A multi-hour run dies with
its terminal, and two were lost that way. Use `nohup`, put `--out`
somewhere persistent (`~/ros2_ws/rl_runs/`, *not* a scratch dir), and rely
on the periodic checkpoints — `ppo_<prefix>_25000_steps.zip` is what
survived both interruptions and is what `--resume` and `evaluate.py` ate.

```bash
nohup python3 -m coco_rl.train_ppo --steps 200000 \
      --out ~/ros2_ws/rl_runs/ppo200k --fast > ~/ros2_ws/rl_runs/ppo200k.log 2>&1 &
```

Concatenating a pre-resume CSV with its post-resume continuation
double-counts the steps between the checkpoint and the interruption; trim
the first CSV at the checkpoint step count before plotting both.

---

## Machine-specific notes (July 2026)

| Issue | Status / fix |
|---|---|
| **NVIDIA driver not loaded** (SecureBoot, post-Windows reboot) | `sudo modprobe nvidia` or reboot. Until then `setup_env.sh` auto-falls back to the Intel iGPU (RTF still ≈ 1.0). Forcing the NVIDIA EGL vendor with the driver down segfaults gz-sim. |
| **MoveIt / rosbridge / web_video_server not apt-installed** | They run from `~/ros2_ws/moveit_prefix/` (user-space deb extraction, no root). With sudo: `sudo apt install ros-jazzy-moveit ros-jazzy-rosbridge-suite ros-jazzy-web-video-server`, then delete the prefix dir. |
| pip user packages | `tornado pymongo cbor2` (rosbridge), `torch` (CPU build), `stable-baselines3 gymnasium` (RL) — installed with `pip install --user --break-system-packages`. |
| `~/assignment_ws` in `.bashrc` | Disabled (it was Humble-built and broke Jazzy shells). Backup: `~/.bashrc.bak-2026-06-12`. |

## Troubleshooting

- **Robot won't move on `/cmd_vel`** — Jazzy's `diff_drive_controller` is
  TwistStamped-only on `/diff_drive_controller/cmd_vel`. Nav2's `/cmd_vel`
  reaches it through the `cmd_vel_relay` node (started by `nav.launch.py`).
- **slam_toolbox silent** — it's a lifecycle node; use `slam.launch.py`
  (auto configure→activate).
- **gz-sim segfault in `driCreateNewScreen3`** — wrong EGL vendor for the
  current driver state; re-source `setup_env.sh`.
- **`ros2` CLI crashes with a `get_type_description` symbol error** — a
  Humble workspace is being sourced into a Jazzy shell; check `.bashrc`.
