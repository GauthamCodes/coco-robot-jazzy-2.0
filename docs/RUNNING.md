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
(`ros2 control list_controllers`).

## Demo 2 — SLAM mapping

```bash
# T1: sim (above)   T2:
ros2 launch gazebo_models slam.launch.py
# T3: scripted mapping drive (or drive manually)
python3 ~/ros2_ws/src/coco-robot-ros2/gazebo_models/scripts/map_drive.py
# save when done:
ros2 run nav2_map_server map_saver_cli \
  -f ~/ros2_ws/src/coco-robot-ros2/gazebo_models/maps/coco_world \
  --ros-args -p use_sim_time:=true
```

A saved map ships with the repo, so this is optional.

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

Spawns a pedestal + red cylinder in front of the robot, mirrors them into
the planning scene, runs 7 collision-checked motions. The gripper pinches
and drags the cylinder; the rigid CAD fingers drop it partway through the
lift (see FUTURE_WORK). **Restart the sim (or `gz service`-remove
`pick_pedestal`/`pick_target`) before driving again** — the pedestal sits
right in front of the robot.

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
python3 -m coco_rl.train_ppo --steps 200000        # real run, overnight
```

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
