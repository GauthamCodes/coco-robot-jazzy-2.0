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
ros2 run gazebo_models verify_sim.py
```

## RViz

```bash
# alongside a running sim (it already publishes robot_state_publisher)
ros2 run gazebo_models verify_sim.py          # confirm the graph first
ros2 launch gazebo_models rsp.launch.py rsp:=false use_sim_time:=true

# or standalone, to inspect the model with no simulator at all
ros2 launch gazebo_models rsp.launch.py
```

Expect the robot rendered with the laser scan, `Global Status: Ok`, and
**2D Pose Estimate** / **2D Goal Pose** in the toolbar for sending Nav2
goals by clicking. Pass `rviz:=false` for TF only.

## Demo 2 — SLAM mapping

```bash
# T1: sim (above)   T2:
ros2 launch gazebo_models slam.launch.py
# T3: closed-loop waypoint mapping drive around the whole arena
#     (south lane -> east half behind the ramp -> north lane -> home)
ros2 run gazebo_models map_drive.py
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
rear), mirrors them into the planning scene, and runs a collision-checked
13-step sequence: move up → open gripper → stage scene objects → hover
above target → allow gripper-target contact → grasp approach → close
gripper → raise → lift → place → release → retreat above target → home.
The cylinder is genuinely carried — grasped,
lifted through the arc, and set back down on the pedestal (fingertip
end-stop lips keep it caged). Ground-truth pose checks before and after
the run catch any physics blow-up. Re-runs are safe: the script clears
stale scene objects and re-spawns the props itself. Optional:
`pick_place.py --target X Z` re-targets the grasp. Note the analytic IK
(`arm_ik.py`) solving a point does **not** mean the grasp will succeed
there — measured 0/5 on nearby targets versus 4/4 at the shipped one, and
the approach often knocks the cylinder off the pedestal first. Failures
abort naming the step. See docs/RESULTS.md.

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
# runs are seeded (--seed, default 0); the seed is echoed at start-up so a
# run can be reproduced from its log. Note the seed pins the policy and
# sampling, not Gazebo — the physics is not bit-reproducible.
python3 -m coco_rl.train_ppo --steps 200000 --seed 42 --fast
```

**Long runs: detach and write outside `/tmp`.** A multi-hour run dies with
its terminal, and two were lost that way. Use `nohup`, put `--out`
somewhere persistent (`~/ros2_ws/rl_runs/`, *not* a scratch dir), and rely
on the periodic checkpoints — `<prefix>_25000_steps.zip` (e.g.
`ppo200k_25000_steps.zip`) is what survived both interruptions and is what
`--resume` and `evaluate.py` ate.

Ctrl-C is now safe: the run saves to `<prefix>_interrupted.zip` and prints
the resume command. (It didn't used to be — rclpy invalidates the context
from its own SIGINT handler, so the interrupt surfaced as
`ExternalShutdownException` or a raw `RCLError` rather than
`KeyboardInterrupt`, and the teardown then raised again while stopping the
robot, masking the original error and skipping the save.)

```bash
nohup python3 -m coco_rl.train_ppo --steps 200000 \
      --out ~/ros2_ws/rl_runs/ppo200k --fast > ~/ros2_ws/rl_runs/ppo200k.log 2>&1 &
```

Concatenating a pre-resume CSV with its post-resume continuation
double-counts the steps between the checkpoint and the interruption; trim
the first CSV at the checkpoint step count before plotting both.

### The full curriculum, unattended

`train_curriculum.sh` runs the whole 12° → 18° → 24° progression by itself.
It is the only way to run a curriculum correctly, because **the grade lives
in the running simulator, not in a training flag** — `--ramp-angle` is
recorded in the run header, but the geometry comes from
`ros2 launch ... ramp_angle:=<deg>`. So each phase has to relaunch the sim,
wait for its topics, train, and hand its weights to the next phase.

```bash
./train_curriculum.sh                    # 3 phases x 60k steps, ~7 h
./train_curriculum.sh --steps 30000      # shorter, ~3.5 h
./train_curriculum.sh --grades 18        # one grade, no curriculum
./train_curriculum.sh --steps 600 --grades "12 18" --eval-episodes 2  # smoke
./train_curriculum.sh --retries 0        # fail a phase instead of retrying
```

Do not source anything first — it sources `setup_env.sh` itself, then
refuses to start unless `ros2`, `gz`, `stable_baselines3`, `torch` and
`coco_rl` all resolve. Per phase it: tears down any stale nodes, launches
the sim at that grade, gates on **both** odometry topics, trains
(`--resume`-ing the previous phase), then runs `evaluate.py` on that same
grade for a real success rate.

Watch it with:

```bash
./watch_training.py            # live bar, newest run, refreshes every 5s
./watch_training.py --once     # one frame (for a log or a screenshot)
```

```
┌───────────── Coco RL curriculum — curriculum_20260725_235536 ──────────────┐
  overall  ███░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░   7.7%  13,788/180,000 steps

  12° running ██████░░░░░░░░░░░░░░░░░░  23.0%  13,788/60,000
         219 episodes   mean  -11.81   best   12.82
  18° pending ░░░░░░░░░░░░░░░░░░░░░░░░   0.0%       0/60,000
  24° pending ░░░░░░░░░░░░░░░░░░░░░░░░   0.0%       0/60,000

  throughput 8.2 steps/s   remaining 166,212   ETA 5h36m

  runner✓  trainer✓  sim✓  no-sleep✓  AC✓  7.8GB free
  grade✓ sim 12° = trainer 12°
└────────────────────────────────────────────────────────────────────────────┘
```

It is strictly read-only — it reads the run directory and `/proc`, so starting
and stopping it can never disturb training. The `grade✓` line is the one worth
understanding: the wedge angle lives in the **running simulator**, not in a
training flag, so a sim launched at one grade while the trainer records another
is the one way a curriculum can be silently wrong. That line compares them.

The raw artifacts are still there if you prefer them:

```bash
cat  ~/coco_rl_runs/<run>/STATUS          # one line: which phase, since when
tail -f ~/coco_rl_runs/<run>/curriculum.log
ls   ~/coco_rl_runs/<run>/DONE            # exists once the run is over
```

When it finishes it writes `SUMMARY.md` (per-phase verdict, success rate and
wall clock, plus an episode table from the Monitor CSVs) and
`curriculum_curve.png` spanning all three phases.

Two things it handles that a hand-rolled loop does not:

- **Sleep.** The run is re-exec'd under
  `systemd-inhibit --what=idle:sleep:handle-lid-switch`, so idle and lid-close
  suspend are blocked outright. Note that on AC this is belt-and-braces —
  GNOME's `sleep-inactive-ac-timeout` is `0` (never) on this machine, and the
  300 s `idle-delay` only blanks the screen. What the inhibitor cannot stop is
  a flat battery, so preflight reports AC vs battery and pauses if you are
  unplugged. Suspend is *also* survivable now regardless: `ramp_env`'s
  deadlines use `time.monotonic()`, which does not tick while suspended, so
  the run pauses and continues rather than expiring every deadline at once
  and truncating the rest of the run as `sim_stalled`.
- **Teardown between phases.** Killing `gz sim` is not enough: the bridge,
  `robot_state_publisher` and controller spawners survive and keep
  publishing TF stamped with the *old* sim clock, so the next phase's sim
  (clock restarting at 0) makes tf2 see a jump back in time. Launches run
  under `setsid` and the process group is killed, then stragglers are swept
  by name — the same approach `verify_all.sh` uses, and for the same reason.

**Surviving a reboot.** A suspend is now transparent, but a shutdown, power
cut or panic is not, so the run opts into a GNOME login hook that resumes it:

```bash
./resume_curriculum.sh          # or just log in — the hook runs it
./train_curriculum.sh --resume-run ~/coco_rl_runs/curriculum_<stamp>
./train_curriculum.sh --resume-latest
./train_curriculum.sh --no-autoresume    # don't install the hook
rm ~/.config/autostart/coco-curriculum-resume.desktop   # remove it later
```

Resuming asks for the **remaining** steps, not a fresh phase: `steps_done()`
sums episode lengths across every Monitor CSV a phase has, and completed
phases (those with a `.zip`) are skipped with their model carried forward.
Because SB3's `Monitor` opens its CSV with mode `wt` and would truncate that
history, the live file is parked as `.partN` first; the watcher, the summary
table and the plotted curve all read the parts back in chronological order.
The hook is inert unless the run opted in (an `AUTORESUME` marker), has no
`DONE` marker, and no curriculum is already running — and it deletes itself
once the run finishes, so it cannot become a job that starts training on
every login forever.

A phase that dies without saving still leaves its 25k checkpoints, so the
script carries the newest artifact forward and marks that phase `PARTIAL`
in the summary rather than throwing the night away or overstating what ran.
Each phase also gets `--retries` extra attempts (default 2) with a fresh sim,
resuming from its own newest checkpoint if it has one. That backstop exists
because the first real curriculum run lost all three phases to a transient
gz-transport timeout on the per-episode `set_pose`, before any phase had
reached its first 25k checkpoint — the root cause is fixed in
`gz_service()` (see
[DESIGN_DECISIONS.md](DESIGN_DECISIONS.md#a-3-second-timeout-that-destroyed-a-180000-step-run)),
but the next transient will be a different one.

## Demo 7 — Fetch: drive out, climb, pick an object by colour, bring it home

```bash
# T1 — the simulator. RESTART IT BETWEEN RUNS (see below).
ros2 launch gazebo_models full_world_robo.launch.py traverse:=true gui:=false

# T2 — everything else: Nav2 (arbiter:=true), cmd_vel_arbiter, perception,
#      move_group, ramp_driver, approach_server, grasp_server.
ros2 launch coco_mission mission.launch.py \
    policy:=~/coco_rl_runs/curriculum_20260726_211008/phase5_24deg_s0p0.zip

# T3 (optional — the phone picks the target and shows what the camera sees):
ros2 launch coco_web web.launch.py

# T4 — the sequencer.
ros2 run gazebo_models traverse_demo.py --colour blue
```

**A fresh simulator per run is not optional.** The gz `DetachableJoint`
binds to each target once, on first spawn. Run the mission twice in one
simulator and the second grasp reports "attached" while welding nothing —
a silent success. `gazebo_models/scripts/ros_clean.sh` tears the previous
run down properly (it kills by **process** name, which is the whole
point — see the trap note at the end of this file).

`mission.launch.py` deliberately does not start the simulator: only one
Gazebo runs at a time on this machine, and bringing it up is the one step
worth being deliberate about.

The nine steps, and which controller owns the wheels for each:

| step | mode | what runs |
|---|---|---|
| 1 | `nav` | Nav2 to the pre-ramp pose `(0.5, lane_y)` |
| 2 | `rl` | `/ramp/climb` — the PPO policy, inside the lane hold |
| 2b | — | vision must confirm the colour is in front (**gates the grasp**) |
| 2c | `idle` | `/grasp/stow` — arm up, before driving at the target |
| 3 | `approach` | `/approach/run` — 1.198 m across the platform, on vision |
| 4 | `idle` | `/grasp/pick` — hover, weld, close, lift, carry at `up` |
| 5 | `rl` | `/ramp/descend` — carrying |
| 6 | `nav` | Nav2 home |
| 7 | `idle` | `/grasp/place` — set it down, release, arm home |

A failed 2b skips 3, 4 and 7 but still runs 5 and 6: a robot that comes
home empty is recoverable, a robot parked on a 0.65 m platform is not. It
prints `FETCH FAILED (wrong lane: saw <colour>)` and exits non-zero.

`--no-grasp` runs 1, 2, 2b (reporting only), 5, 6 — the M4/M5 traverse,
kept runnable so those measurements stay reproducible.

`target_finder` reports on `/perception/status`, one line of
space-separated `key=value`:

```
sel=blue found=1 u=161 v=118 area=372 w=9 h=47 range=0.724 \
  x=0.859 y=+0.013 z=0.079 lane=+0.25 seen=green,blue age=0.030
```

`x y z` are the target in `base_footprint`, which is what the grasp will
aim at. When `found=0`, **`seen` is the interesting field**: the
neighbouring lane's object stays in frame at the working distance, so
`sel=yellow found=0 seen=blue` means the robot climbed into the wrong
lane rather than that the camera failed.

The annotated frame is on `/perception/annotated` —
`http://<host>:8081/stream?topic=/perception/annotated&type=mjpeg`.
`web_video_server` discovers it on its own, so nothing in `coco_web`
needs starting differently.

Two things worth knowing before debugging a silent node:

- **The camera topics are BEST_EFFORT.** The gz→ROS bridge republishes
  with sensor QoS, so a RELIABLE subscriber never matches and sees
  nothing, with no error anywhere. `ros2 topic info /camera/image_raw
  --verbose` shows what a subscriber actually asked for.
- **Nothing on the platform is visible from the flat ground.** The crest
  edge occludes it: from the pre-ramp pose the lowest visible point at
  the target row is z=0.907, and the targets top out at 0.808. The
  colour→lane decision is a table lookup in `coco_config.robot`, made
  before the climb, not something the camera can be asked.

Omit `--colour` and the sequencer waits for the phone to pick one on
`/mission/target_colour`. With `--colour` it publishes that topic itself,
because `approach_server` and `grasp_server` both refuse to start without
a colour — stopping in the wrong approach window, or welding the wrong
object, is worse than not starting.

Watch the three status topics; they all use the same `key=value` shape:

```bash
ros2 topic echo /cmd_vel_arbiter/status   # which controller owns the wheels
ros2 topic echo /approach/status          # phase, target bearing, creep
ros2 topic echo /grasp/status             # phase, solved pose, lifted
```

If a run goes wrong, tear it down by **process** name before trying again:

```bash
ros2 run gazebo_models ros_clean.sh          # or --list to see what is up
```

---

## Machine-specific notes (July 2026)

| Issue | Status / fix |
|---|---|
| **NVIDIA driver won't load** — `modprobe nvidia` says `Operation not permitted` | SecureBoot is rejecting an unenrolled module signature, *not* a missing module. Check `mokutil --list-enrolled`: if it shows only Canonical's CA, run `sudo mokutil --import /var/lib/shim-signed/mok/MOK.der`, reboot, and choose **Enroll MOK → Continue** at the blue screen. Until then `setup_env.sh` auto-falls back to the Intel iGPU (RTF still ≈ 1.0); forcing the NVIDIA EGL vendor while the driver is down segfaults gz-sim in `driCreateNewScreen3`. |
| **MoveIt / rosbridge / web_video_server not apt-installed** | They run from `~/ros2_ws/moveit_prefix/` (user-space deb extraction, no root). With sudo: `sudo apt install ros-jazzy-moveit ros-jazzy-rosbridge-suite ros-jazzy-web-video-server`, then delete the prefix dir. |
| Python (non-ROS) deps | `pip install --user --break-system-packages -r requirements.txt` — pinned to the versions the published results were produced on. numpy is held at 1.26.x because the Jazzy debs are built against the numpy 1.x ABI. |
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
