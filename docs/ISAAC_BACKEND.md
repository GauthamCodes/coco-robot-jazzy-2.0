# Isaac Sim 4.5 backend foundation

**Hardening update:** See [Isaac Nav2 hardening](ISAAC_NAV2_HARDENING.md) for
the current readiness/zero-motion shutdown contract, 200 ms Isaac-only BT
acknowledgement budget, managed Mesa RViz mode, and fresh repeatability evidence.
The foundation outcomes and teardown failures below are retained as history.


Branch: `p03-isaac-backend`, created from
`b3c65985de6aa68f4701687f9b35a51e04239595`. That was both the local
and remote `p03-episode-spec` tip when checked. P0.2 RC remains
`c40098f`; neither it nor the original working tree was edited.

This is an **experimentally validated, unsupported Ubuntu 24.04 / ROS 2
Jazzy peer configuration**, not NVIDIA platform certification. The existing
installation is **Isaac Sim 4.5.0.0 at `~/isaac-sim/venv`**.
No simulator, driver, global Humble installation, or shell profile is installed
or changed by this backend.

## Architecture and scope

`coco_config/coco_config/robot.py` supplies dimensions, masses, wheel
layout, acceleration limits and camera/sensor poses. Its existing tests
compare these constants to `gazebo_models/urdf/coco_robo2.xacro` and
`coco_controllers.yaml`. The Isaac USD and minimal RViz URDF are generated,
not separately hand-maintained models.

`coco_sim/coco_sim/isaac/` is an optional, explicitly launched backend.
Normal imports of coco_sim, Gazebo launches, mission code, perception,
MoveIt, browser/session code, and the episode generator are unchanged.
The robotics stack remains in Jazzy. Isaac supplies physics and interfaces.

The minimal base has four driven wheels (COCO is skid steer), a chassis
collision box, and the non-wheel payload mass lumped at the existing
mounting locations. Wheel centres, radius, width, track and wheelbase come
from coco_config. The chassis uses a primitive diagonal inertia plus
parallel-axis lump contributions; it is **not** CAD inertia parity.
Wheel collisions use generated 64-sided convex cylinders. The initial native
USD-cylinder setup let the chassis rest on the ground while wheels spun;
the explicit convex geometry resolved that measured failure. The underlying
native-cylinder collision issue was not separately diagnosed.

The scene is a deterministic, simple rectangular sensor/navigation fixture with one off-axis landmark,
defined once in `isaac/config.py`. Its raster map is generated from the
same rectangles. The fixed episode manifest is archived and supplies the
robot start. **The fixture does not instantiate the episode's fetch arena or
targets**; `scene.json` states that explicitly. No Isaac-specific target
selection, placement or randomization is implemented.

The real control path is:

```
plain Twist /cmd_vel → isaac_ros.py → /cmd_vel_teleop ─┐
                                                    ↓
Nav2 → smoother → collision monitor → existing relay → arbiter
                                                    ↓
                         /diff_drive_controller/cmd_vel (TwistStamped)
                                                    ↓
                         Isaac wheel velocity drives → ground contact
```

`isaac_ros.py` instantiates the **existing CmdVelArbiter**, without changing
its selection/safety logic. Plain Twist and Nav2's TwistStamped are distinct
DDS types: the plain-Twist adapter does not match Nav2's stamped output.
The existing `nav.launch.py arbiter:=true` feeds `/cmd_vel_gated`, as usual.

## Launch

Build a separate overlay (no changes to the existing workspace install):

```bash
cd ~/coco-isaac-backend
bash scripts/build_overlay.sh ~/coco_isaac_ws
export COCO_WS=~/coco_isaac_ws
source setup_env.sh
export ROS_DOMAIN_ID=78
```

Terminal 1, from this checkout:

```bash
bash scripts/isaac_sim_coco.sh --output /tmp/coco-isaac-run-001 --gui
# omit --gui for headless; optional --seconds 60 means simulation seconds
```

The output directory must be new. Terminal 2, with the Jazzy environment above:

```bash
python3 scripts/isaac_ros.py --ros-args -p use_sim_time:=true
```

Terminal 3, with the same environment:

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'
rviz2 -d coco_sim/rviz/isaac.rviz --ros-args -p use_sim_time:=true
```

Ctrl-C stops each process. Stopping the command publisher exercises the
arbiter timeout. The simulator also ages the **original stamped command**
against simulation time, with a 0.5 s timeout; stale/future/nonfinite inputs
cannot sustain motion. Its keep-last-1 queue prevents a faster arbiter from
building a backlog behind the rendering loop. Forward is +base x; positive
angular z turns counterclockwise. Linear/angular limits are 1 m/s and 2 rad/s,
with 2 m/s² and 4 rad/s² ramp limits, matching coco_controllers.yaml.
Only joint velocity targets are applied; chassis pose/velocity is never set
from commands. Wheel angular targets use the existing 1.10 track multiplier.

## Process environment and ordering

The launcher executes an `env -i` allowlist. It uses only the existing
Isaac Python, the bundled bridge Humble library directory, and the two pure
project modules on PYTHONPATH. No Jazzy/foreign AMENT or COLCON prefix is
inherited by Isaac. Both processes use **rmw_cyclonedds_cpp**, the selected
private domain (default 78), and loopback CycloneDDS.

`ROS_DISTRO=humble` is **Isaac-process-local**; Jazzy stays the host ROS
distribution. Do not source the launcher. It is deliberately a single wrapper,
not another ROS installation or a permanent shell configuration.

Startup is: SimulationApp(create_new_stage=False) → new USD stage → physical
COCO/fixture/camera creation → enable PhysX sensor and ROS bridge extensions →
configure sensor graph → play. There is no new_stage after bridge loading.
Allow first-context shader compilation to finish; the harness allows 1200
wall seconds for initialization. The launcher locks out a second COCO Isaac
instance. Do not run an unrelated Isaac instance alongside it.

Compatibility provenance remains on `isaac-compat-4x@20a9619` in
`docs/data/isaac_compat/README.md` and its scripts. The old hardware-blocked
verdict in the episode baseline is superseded by those findings and the
new backend measurements. Local installed 4.5 API/schema files were used
for implementation; [NVIDIA's 4.5 physics reference](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/physics/simulation_fundamentals.html)
describes its rigid-body/collider model.

## ROS contract and time

Topic names and frame names are centralized in `isaac/config.py`.

| Topic | Message | Nominal rate / QoS |
|---|---|---|
| /diff_drive_controller/cmd_vel | TwistStamped | newest sample consumed at 30 Hz; reliable, depth 1 |
| /diff_drive_controller/odom | Odometry | 30 Hz, reliable |
| /clock | Clock | 30 Hz, reliable |
| /tf | TFMessage | 30 Hz, reliable |
| /tf_static | TFMessage | transient-local, reliable |
| /joint_states | JointState | 30 Hz, reliable, four simulated wheels |
| /robot_description | String | transient-local, reliable, generated minimal model |
| /scan | LaserScan | 10 Hz, 480 beams, full 360°, 0.15–12 m |
| /camera/image_raw | Image | 15 Hz, 320×240, rgb8, reliable |
| /camera/depth/image_raw | Image | 15 Hz, 320×240, 32FC1 metres, reliable |
| /camera/camera_info | CameraInfo | 15 Hz, calibrated to generated camera, reliable |

Physics runs at 120 Hz; rendering/control at 30 Hz, paced to never intentionally
outrun wall-clock delivery. Slow rendering lowers wall rates. All messages use
Isaac **simulation time**, not wall time. Evidence duration fields end
`_sim_s` or `_wall_s`.

Odometry is **measured simulator ground truth**, not integrated commanded
velocity or wheel-encoder odometry. Pose is expressed in an odom frame with
x/y origin at the episode spawn; twist is transformed into the child frame,
including the base_link-to-footprint velocity offset. It contains full
orientation and velocity. This privileged ideal odometry is an explicit
foundation limitation for later localization benchmarking.

TF ownership:

* Isaac alone: odom → base_footprint; base_footprint → base_link (clearance);
  base_link → four wheels; base_link → lidar_link / camera_link;
  camera_link → camera_optical_frame.
* AMCL (when Nav2 is launched): map → odom.
* Do **not** additionally launch Gazebo, a second clock, robot_state_publisher,
  diff_drive_controller, or another publisher of those transforms on this domain.

The camera mounting RPY remains (0,0,0), looking +base x. Its USD camera
orientation implements that convention, and ROS images use the REP-103
optical frame. Depth is axial distance, not a slant range image. CameraInfo
is generated by the bridge from the same render product.

The 360° LiDAR is a deliberate difference from Gazebo's 240° front arc.
Acceptance checks angle_min, angle_max, increment, 480 ranges, and the known
wall distance. Isaac 4.2's angle behavior is not used.

The host receive buffer can be smaller than one depth image. Use **reliable
subscriptions** for the initial RGB/depth validation. The existing perception
node has best-effort subscriptions: they can match reliable publishers, but
that does not solve fragmentation losses. Perception execution and its QoS
adaptation are **not integrated or claimed** in this phase; the validated
depth encoding, optical frame, calibration and distances provide its input
contract. No costmap point cloud is enabled.

## Automated tests and Nav2

```bash
# Source Jazzy + the isolated overlay as above.
python3 scripts/isaac_smoke.py /tmp/coco-smoke-001
python3 scripts/isaac_smoke.py /tmp/coco-smoke-gui-001 --gui --rviz

# Only after a raw smoke run passed (Nav2 is PARTIAL; see limitations):
python3 scripts/isaac_smoke.py /tmp/coco-nav-001 --nav --rviz \
  --raw-evidence /tmp/coco-smoke-001/result.json
```

Each run owns its child process groups, records logs, graph, samples, checks,
resource usage and exits, and never performs a global process sweep. Capture
of the owned RViz window uses optional Pillow/xdotool/xwd; capture errors are
archived. The harness shuts down Nav2 and the ROS peer, then Isaac, then
RViz, so RViz destroys its image widgets after sensor publication stops.
Closing RViz while the image producers were active crashed in two recorded
runs; producer-first shutdown exited normally in the final raw run. An interrupted
Nav2 startup still crashed RViz, so universal teardown reliability is not claimed. Existing unrelated applications are not stopped.

The raw test checks forward 0.5 m/s for 8 simulation seconds (2.8–4.6 m
acceptance, lateral <0.5 m), positive 0.5 rad/s yaw for 3 simulation seconds,
dropout stopping, settled contact height, sensor geometry, timing, TF and
publisher ownership. It also checks camera intrinsics, measured simulation-time
sensor rates, and all scan bearings against independent ray/fixture
intersections both before and after motion. These are foundation tolerances,
not simulator parity.
The red east wall should be ~6.825 m axial depth and ~7.04 m LiDAR range
from the settled start (derived from fixture and mounts).

The Nav2 test uses the existing `gazebo_models/nav.launch.py`,
`nav2_params.yaml`, relay and arbiter. The harness uses `use_composition:=False` and a run-local copy of the
existing parameters with only `map_server.ros__parameters.yaml_filename`
changed to the fixture map. The composed mode completed goals but its
container exited -6 during shutdown. The initial noncomposed run retained
`map.yaml` despite the map argument and failed localization; the explicit
run-local parameter addresses that concrete launch incompatibility. It waits for lifecycle activation (action-server discovery alone
is insufficient), requests map (1.5,0.5), records the action result and
command-chain activity, and checks the arbiter is the sole wheel publisher.
No Nav2 algorithm, footprint, tolerance, controller or mission code is changed.

For manual Nav2, generate a map with
`python3 scripts/isaac_fixture_map.py /tmp/coco-fixture-map`, start the ROS peer
with `-p initial_mode:=nav`, then run
`ros2 launch gazebo_models nav.launch.py arbiter:=true map:=/tmp/coco-fixture-map/fixture.yaml`.
This manual command uses the default composed mode, whose shutdown is a
known failure. Add `--nav-composed` to the harness to reproduce that mode. Prefer the automated harness for the tested noncomposed
parameter override and child-process exit audit.

## Evidence and limitations

See [the measured run index](data/isaac_backend/README.md).
All failed development runs are retained, including logger failure, no-contact
wheels, command backlog, and full-extension-cleanup failure. Normal Isaac
fast-close is used: clean acceptance requires the normal-close marker,
external zero exit, and absence of a failure marker; a forced timeout does
not qualify. The final harness also requires zero exits from all auxiliary
processes. Earlier run result files predate that stronger auxiliary-exit gate;
the measured index explicitly identifies their teardown failures.

No mission, manipulator actuation, MoveIt port, target-world generation,
perception execution, IMU, noisy encoder odometry, model liveness odometry,
terrain policy, random layouts, browser sessions or Isaac Lab is provided.
The arm is a mass approximation, not a working manipulator. Skid-steer yaw
is materially lower than requested and requires later calibration/benchmarking.
The first bare-room navigation attempt aborted; the landmark fixture later
completed a goal with unchanged Nav2 parameters. That comparison does not
isolate the cause of the earlier failure. A simple successful navigation goal
is not full mission compatibility.

Current Nav2 status is **PARTIAL**: two composed-mode goals succeeded, but
its container crashed on teardown. The noncomposed map override activated
the full stack and moved the robot, then a `compute_path_to_pose` goal-request
acknowledgement timeout aborted the goal. RViz also intermittently throws
an Ogre vertex-buffer allocation error at shutdown, even with producer-first
ordering. Isaac itself closed normally in these final runs. These are concrete
remaining integration/reliability blockers; no navigation tuning or broader
mission port is hidden in this foundation change.
