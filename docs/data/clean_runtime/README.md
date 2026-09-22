# Clean COCO runtime — evidence, 2026-09-22

Branch `coco-clean-runtime`. Overlay `~/coco_ws_build`, built by
`scripts/build_overlay.sh` (9 packages, `--symlink-install`, underlay
chain `/opt/ros/jazzy` only, 0 dangling symlinks). Every run: a fresh
simulator, green target, never `--fast`, depth fusion off, no Nav2 /
costmap / PolygonStop / AMCL / goal change.

Every shell here was a `bash --noprofile --norc` started from a shell
carrying the same `~/.bashrc` exports as the developer's terminal (four
`$HOME/ros2_ws/install` prefixes on `AMENT_PREFIX_PATH`).

| dir | what |
|---|---|
| `env_before/` | the user's current setup: main checkout's `setup_env.sh`, `<ws>/install`. `probe.txt` = the environment; `launch.log` = `full_world_robo.launch.py traverse:=true gui:=true` dying with `package 'turtlebot3_teleop' not found`, exit 1 |
| `env_after/` | the fixed `setup_env.sh` with `COCO_WS=~/coco_ws_build`: 0 `$HOME/ros2_ws/` and 0 turtlebot entries across nine path variables |
| `run1_green/` | `gui:=true`, the user's two commands. **ABORT `RETURN_FAILED`** |
| `run2_green/` | identical, fresh simulator. **ABORT `RETURN_FAILED`** |
| `run3_headless_green/` | `scripts/browser_check/live_run.sh` unmodified (`gui:=false`). **COMPLETE, `result=fetch`** |

## What was measured

| | run 1 (GUI) | run 2 (GUI) | run 3 (headless) |
|---|---|---|---|
| controllers active after sim launch | 7 s | 10 s | 11 s |
| `/healthz` 200 after mission launch | 10 s | 10 s | 8 s |
| climb, search, approach, grasp | yes | yes | yes |
| target lifted (grasp_server, ground truth) | 35.8 mm | 34.8 mm | 35.2 mm |
| return leg | `Start occupied` ×3 from (8.00, 1.17) | `Start occupied` ×3 from (7.87, 1.25) | clean |
| final | `ABORT RETURN_FAILED` | `ABORT RETURN_FAILED` | `COMPLETE fetch`, `attempts={}` |
| every executive state rendered on the page | yes | yes | yes |
| STOP with W held: first zero command | 16.2 ms | 3.8 ms | 3.0 ms |
| browser SIGKILLed mid-drive: first zero | 75.5 ms | 89.2 ms | 92.4 ms |
| moving commands > 600 ms after either | 0 | 0 | 0 |
| publishers on `/diff_drive_controller/cmd_vel` | 1 (`cmd_vel_arbiter`) | 1 | 1 |
| hostile socket frames refused | 8 / 8 | 8 / 8 | 8 / 8 |
| dropped frames, any stream | 0 | 0 | 0 |
| orphans after teardown (55 `ros_clean.sh` patterns) | 0 | 0 | 0 |

Run 1 sensors, 10 s wall window after bring-up: `/scan` 9.70/s,
`/camera/image_raw` 14.80/s, `/camera/depth/image_raw` 14.70/s, `/imu`
48.2/s, `/model/coco/odometry` 48.1/s. `sim.log`: 0 turtlebot mentions,
0 `[ERROR]` lines. All four `gz` processes were one tree rooted at our
`gz sim …/gazebo_models/worlds/coco_world.world` (server + GUI children).

Run 1's in-run orphan sweep reported 1 match: the sweeping `bash -c`
itself, whose command line contains `full_world_robo.launch.py` (the
CLAUDE.md self-match trap). Re-swept after it exited: 0.

## What is NOT established

- **Why the GUI runs fail the return leg.** GUI 0 of 2, headless 1 of 1 —
  three runs, not a rate. The global planner judged the robot's own cell
  occupied at the foot of the ramp; that is Nav2 territory and was not
  investigated or tuned here. P0.2's first pass recorded the same
  `Start occupied` abort headless (1 of 2), so the GUI is implicated,
  not proven.
- Nothing here was run from the user's own terminal.
