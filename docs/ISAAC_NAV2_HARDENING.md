# Isaac Nav2 hardening

This pass starts at `ffb3fc6` on `p03-isaac-backend` in
`~/coco-isaac-backend`. The frozen P0.2 reference and shared Gazebo, Nav2,
mission, episode, physics and arbiter implementations are unchanged.
The unsupported Ubuntu 24.04/Jazzy + existing Isaac 4.5 setup remains an
experimentally validated configuration. No installation or global environment
changes are part of this pass.

## What failed and why

The unchanged backend reproduced an acknowledgement timeout on `follow_path`
after moving 0.476 m. The earlier `nav07_map` failure was on
`compute_path_to_pose`. These are the **internal behavior-tree action clients**,
not failure to discover the outer NavigateToPose server. Lifecycle activation,
planning, controller execution and motion had already occurred.

The installed `nav2_behavior_tree/bt_action_server_impl.hpp` defaults
`default_server_timeout` to 20 ms. The installed `bt_action_node.hpp` measures
elapsed time with `node_->now()` (simulation time), then its
`is_future_goal_handle_complete()` discards the future when the remaining
budget is nonpositive, before spinning to collect a response. Isaac's clock
advances approximately 33.333 ms per rendered step. A response can therefore
be available on DDS, yet be discarded on the next clock step. This is a
clock-resolution/acknowledgement-budget incompatibility, sensitive to executor
scheduling, not evidence that Isaac lost the planner.

Evidence is stronger than a timeout adjustment alone:

* Thirty direct planner probes all generated paths. Maximum measured wall
  acknowledgement time was 15.824 ms, while some acknowledgements crossed
  33.333 ms of received simulation time.
* `scripts/isaac_timing_probe/` compiles against the **installed Nav2 client**.
  A controlled server accepts after 15 ms. After allowing delivery, the test
  advances ROS time by 33.333 ms: the 20 ms client returns FAILURE, while the
  200 ms client returns SUCCESS. This is a synthetic timing reproducer, not
  another simulator or a navigation algorithm implementation.
* The live default-20-ms diagnostic also completed a goal. The defect is
  intermittent; that successful counterexample is retained, not omitted.

The Isaac run-local Nav2 parameter copy now sets `default_server_timeout: 200`
(about six clock updates, leaving scheduling margin below the 1 Hz replanning
interval). It also retains the fixture map filename override. No planner,
controller, costmap, footprint, goal tolerance, or arbiter setting changes.
Nav2 is noncomposed; the previously measured composed-container destruction
failure is not claimed fixed. `--ack-ms 0` and `--nav-composed` remain explicit
diagnostic options.

## Observable startup

1. Create a new evidence directory and owned POSIX process sessions. Isaac's
   advisory lock rejects a second COCO Isaac instance. An unrelated or stale
   lock owner is never killed by name.
2. Start the existing environment-isolated Isaac wrapper. Its scene-before-
   bridge order is unchanged. Wait on its ready marker with a bounded deadline.
3. Independently receive progressing `/clock`, odometry, joints, required base
   and sensor TF, LiDAR, RGB and depth. A marker alone is insufficient.
4. Start the existing arbiter in **idle**, and optional managed RViz.
5. Start existing Nav2 with the run-local map/acknowledgement overrides.
6. Query active lifecycle states for map server, AMCL, planner, controller,
   BT navigator, velocity smoother and collision monitor; resolve map-to-base
   TF; discover NavigateToPose, ComputePathToPose and FollowPath actions.
7. Select nav mode through the existing interface, send the known map goal
   (1.5, 0.5), and record acceptance, plans, action UUID/status transitions,
   pose/error and wall/simulation elapsed times. The existing 0.35 m final
   position threshold remains unchanged.

Polling intervals only service observable conditions; there is no fixed
startup sleep used as proof of readiness. A condition timeout records its
specific stage and the condition that was missing. The outer harness also
archives a failure result when a child fails before it can write a result.

## Shutdown and process ownership

The harness owns only the processes it starts: Isaac (including its bridge),
ROS adapter/arbiter, readiness/navigation/stop probes, optional Nav2 children
and optional RViz. Commands, PIDs, kernel start ticks, sessions, exits and
signals are recorded. PID reuse is checked before signalling. Descendants
are scoped to owned sessions, not executable-name matches.

Shutdown requests idle mode, cancels navigation, and sends fresh **zero**
TwistStamped commands to the arbiter's existing teleop input. It never
publishes wheel commands. This is necessary for observation as well as safety:
the arbiter emits a finite idle-stop burst, which a late observer can miss.
The probe verifies final wheel-topic commands are exactly zero and measured
odometry remains below 0.02 m/s and 0.03 rad/s for at least 0.5 simulation
seconds with fresh clock/odom/command samples.

Then stop the action client and Nav2, request normal window-manager closure
of owned RViz, stop the ROS integration, and finally stop Isaac. Nav2's launch
leader receives one SIGINT and forwards it; children are not double-signalled.
SIGTERM/SIGKILL are bounded recovery only, logged and disqualifying a clean
run. Owned descendants are audited even if their launch leader exits first.
Any leftover, auxiliary crash, nonzero child exit, forced escalation, missing
zero-motion proof, or abnormal Isaac close prevents a whole-run PASS.

No global stale-process sweep is performed. Do not manually start a second
backend on the same domain. If an externally killed harness left a process,
inspect its `session.json` identities and command before recovery; never use
`pkill ros2`, `pkill python` or `killall`.

## RViz: bounded graphics workaround

Both producer-first SIGINT and window-manager close were tried with native
NVIDIA OpenGL. Native RViz intermittently exited -6 with an Ogre vertex-buffer
allocation exception or -11 during destruction; Isaac exited normally.
One gdb run exited normally. An older RViz core did not yield a reliable
symbolized culprit (including library build-ID mismatch), so a precise
upstream driver/library defect is **not** claimed identified. The allocation
exception alone is not proof of physical VRAM exhaustion.

Managed RViz uses process-local `__GLX_VENDOR_LIBRARY_NAME=mesa`,
`LIBGL_ALWAYS_SOFTWARE=1`, `LP_NUM_THREADS=2`. `renderer.txt` verifies llvmpipe;
LIBGL_ALWAYS_SOFTWARE alone was experimentally insufficient on this host.
All robot/sensor displays remain enabled. Isaac still uses the NVIDIA GPU.
This avoids the failing native rendering path without modifying system drivers
or installing anything. CPU rendering is a resource tradeoff, not a general
RViz performance recommendation. `--rviz-native` reproduces the native path.
Managed RViz uses a run-local configuration copy so saved views cannot modify
the source configuration.

## Commands and evidence

From the worktree, in a clean shell:

```bash
export COCO_WS=~/coco_isaac_ws
source setup_env.sh
python3 scripts/isaac_smoke.py /tmp/isaac-raw-new --rviz
python3 scripts/isaac_smoke.py /tmp/isaac-nav-new --nav --rviz \
  --raw-evidence /tmp/isaac-raw-new/result.json
python3 scripts/isaac_repeatability.py /tmp/isaac-ten-new \
  --raw-evidence /tmp/isaac-raw-new/result.json
```

Every directory must be new. The ten-trial runner never retries a trial: two
headless, six headless Isaac + RViz, two Isaac GUI + RViz; first five separate
domains, last five reuse domain 78. It stops the batch if owned cleanup fails.
`--goals 2` alternates the known goal and map origin within one session.
`--startup-only` verifies readiness without navigation/motion testing.
`--inject-failure after-ready` or `--inject-failure during-goal` exercises the
same bounded safety/cleanup path and deliberately returns a failed run.

To reproduce the timing mechanism without Isaac:

```bash
cmake -S scripts/isaac_timing_probe -B /tmp/coco-timing-build
cmake --build /tmp/coco-timing-build -j2
ROS_DOMAIN_ID=89 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  /tmp/coco-timing-build/timing_probe
```

New evidence: [data/isaac_hardening/README.md](data/isaac_hardening/README.md).
`session.json` contains commands and ownership; `.jsonl` probe traces contain
monotonic wall timestamps and simulation timestamps; `navigation.json` contains
action states and sampled TF/odom; `safe_stop.json` contains stop proof;
`process_exit.json` and `result.json` include child/teardown verdicts. Earlier
foundation evidence remains unmodified.

FollowPath status 6 on superseded internal goal IDs can be normal replanning:
the installed SimpleActionServer aborts the old handle when accepting its
replacement. Both those transitions and the final controller success are
recorded; only the outer navigation result determines overall goal success.

## Scope limits

This is a short fixture-world reliability result, not a long-duration or
cross-machine guarantee. Ideal odometry, approximate payload inertia and
under-tracking skid-steer yaw remain. Navigation duration and final error vary.
The native RViz and composed Nav2 destruction paths remain diagnostic-only.
No mission, perception execution, manipulation, episode arena generation,
randomization, dynamic obstacle, browser or Isaac Lab integration is added.
