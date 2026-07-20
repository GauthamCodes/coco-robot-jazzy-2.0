# Launch Files

## Main Launch Files

### `full_world_robo.launch.py` (Primary)
Complete simulation with robot, ramp, and world.
- Launches Gazebo with `coco_world.world`
- Spawns robot and ramp
- Starts and activates all four controllers

The arm holds its home position because the joint trajectory controller
latches the state it is activated in — there is no separate home publisher.

**Usage:**
```bash
ros2 launch gazebo_models full_world_robo.launch.py
ros2 launch gazebo_models full_world_robo.launch.py gui:=false
```

## Component Launch Files

### `rsp.launch.py`
Robot state publisher only (for visualization without Gazebo).

### `slam.launch.py`
slam_toolbox in async online mode, driven through its lifecycle
(auto configure → activate). Publishes `/map` and the `map -> odom` TF.

### `nav.launch.py`
Nav2 bringup against the saved `maps/coco_world.yaml`, plus the
`cmd_vel_relay` node from `custom_teleop` (Nav2 publishes `Twist` on
`/cmd_vel`; Jazzy's `diff_drive_controller` accepts `TwistStamped` only).
Auto-initialises AMCL at the spawn pose.

## Quick Reference

| Task | Command |
|------|---------|
| Full simulation | `ros2 launch gazebo_models full_world_robo.launch.py` |
| Headless mode | `ros2 launch gazebo_models full_world_robo.launch.py gui:=false` |
| RViz only | `ros2 launch gazebo_models rsp.launch.py` then `rviz2` |
| SLAM mapping | `ros2 launch gazebo_models slam.launch.py` |
| Autonomous nav | `ros2 launch gazebo_models nav.launch.py` |
| Health check | `ros2 run gazebo_models verify_sim.py` |

