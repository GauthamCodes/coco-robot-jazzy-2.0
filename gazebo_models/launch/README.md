# Launch Files

## Main Launch Files

### `full_world_robo.launch.py` (Primary)
Complete simulation with robot, ramp, and world.
- Launches Gazebo with `coco_world.world`
- Spawns robot and ramp
- Starts all controllers
- Initializes arm to home position

**Usage:**
```bash
ros2 launch gazebo_models full_world_robo.launch.py
ros2 launch gazebo_models full_world_robo.launch.py gui:=false
```

## Component Launch Files

### `rsp.launch.py`
Robot state publisher only (for visualization without Gazebo).

### `spawn_robot.launch.py`
Spawn robot entity in existing Gazebo instance.

### `spawn_ramp.launch.py`
Spawn ramp platform in existing Gazebo instance.

### `full_world.launch.py`
Legacy launch file (use `full_world_robo.launch.py` instead).

## Quick Reference

| Task | Command |
|------|---------|
| Full simulation | `ros2 launch gazebo_models full_world_robo.launch.py` |
| Headless mode | `ros2 launch gazebo_models full_world_robo.launch.py gui:=false` |
| RViz only | `ros2 launch gazebo_models rsp.launch.py` then `rviz2` |

