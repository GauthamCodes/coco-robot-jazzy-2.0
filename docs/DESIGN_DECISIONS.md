# Design decisions

The problems in this project that took real diagnosis, and why they were
solved the way they were. Each one is stated as **Problem → Diagnosis →
Fix → Evidence**, with pointers into the code so the claims are checkable.

If you only read one, read [the CAD frame
re-root](#1-the-robot-was-resting-on-its-own-elbow) — it is the bug that
looked like a performance problem.

1. [The robot was resting on its own elbow](#1-the-robot-was-resting-on-its-own-elbow)
2. [Analytic IK instead of a numerical solver](#2-analytic-ik-instead-of-a-numerical-solver)
3. [Ctrl-C corrupted every long-running script](#3-ctrl-c-corrupted-every-long-running-script)
4. [The gripper could not hold anything](#4-the-gripper-could-not-hold-anything)
5. [One corridor would not map](#5-one-corridor-would-not-map)
6. [Smaller decisions, and things deliberately not done](#6-smaller-decisions-and-things-deliberately-not-done)

---

## 1. The robot was resting on its own elbow

**Problem.** The simulation ran at a real-time factor of about 0.23 — four
times slower than wall clock — and the arm oscillated visibly for several
seconds after every spawn. Both read like performance problems: too much
collision geometry, a physics step size that needed tuning, a controller
gain that needed lowering.

**Diagnosis.** Neither. The CAD export used a **Y-up** coordinate frame,
while ROS and Gazebo use **Z-up** (REP-103). Re-rooting a Y-up model
without fixing the joint origins left the arm bracket mounted on the
*bottom* face of the chassis, so the arm hung down through the floor plane
and the robot came to rest on its own elbow. Every physics step was
resolving a permanent penetrating contact between two links of the same
robot, which is expensive, and the contact solver's oscillation was what
looked like controller instability.

The tell was that the RTF was bad *before the robot was commanded to do
anything*. A control problem needs a control input; this happened at
spawn.

**Fix.** The xacro re-roots the model Z-up and mounts the arm on the top
face, so the kinematic chain matches the physical layout.
See `gazebo_models/urdf/coco_robo2.xacro`.

**Evidence.** RTF went from ~0.23 to ~1.0 and the spawn oscillation
disappeared. Current sensor rates measured in simulation time are in
[RESULTS.md](RESULTS.md); `ros2 run gazebo_models verify_sim.py`
reproduces them.

**What this is worth knowing for.** A wrong coordinate convention does not
announce itself as a wrong coordinate convention. It announces itself as
bad performance, and it will absorb an arbitrary amount of gain tuning
before you find it.

---

## 2. Analytic IK instead of a numerical solver

**Problem.** The pick-and-place demo needs Cartesian targets — "grasp the
cylinder at (x, z)" — but MoveIt goals here are joint-space. A numerical
IK plugin (KDL, TRAC-IK) is the default answer.

**Diagnosis.** The arm does not need one. Both joints rotate about the
base y-axis, so the arm works entirely in the robot's x-z plane. That is
the classic planar **2R** problem, which has a closed-form solution.

A numerical solver would have brought real costs for no benefit: it is
seeded, so it is not deterministic run to run; it returns whichever of the
two elbow branches it converges to; and it cannot be tested without
standing up a MoveIt instance.

**Fix.** `coco_moveit_config/scripts/arm_ik.py` — a closed-form 2R solver
derived from the URDF chain:

```
elbow = S + L1 * u(alpha),      alpha = PHI1 - q_shoulder
pinch = elbow + L2 * u(beta),   beta  = alpha + GAMMA0 + q_elbow
```

`PHI1` and `GAMMA0` are phase offsets that exist because the CAD zero-pose
is not the kinematic zero-pose — the links are already rotated when all
joint values are zero. The reference point is the **pinch point**, the
midpoint between the closed fingertips, because that is what actually has
to arrive at the object.

**Evidence.** `coco_moveit_config/test/test_arm_ik.py`: 2,000 randomised
round-trips agree to better than 1e-9, forward kinematics matches the
verified grasp pose to within 2 mm, and every solution is checked against
the URDF joint limits. All of it runs in milliseconds with no simulator.

**Trade-off accepted.** This solver is specific to this arm. Adding a
third joint means re-deriving it. That is the right trade for a 2-DOF arm
and the wrong one for a 6-DOF one.

---

## 3. Ctrl-C corrupted every long-running script

**Problem.** A 50,000-step reinforcement-learning run was interrupted at
47,000 steps and saved nothing. The code had a `try/except
KeyboardInterrupt` around the training loop that was supposed to save the
model. It never ran. The same pattern silently failed in three separate
scripts.

**Diagnosis.** `rclpy` installs its **own** SIGINT handler and invalidates
the ROS context *before* Python's exception machinery sees the signal. So
Ctrl-C does not surface as `KeyboardInterrupt`. It surfaces as:

- `ExternalShutdownException` from `spin()` / `spin_once()`, or
- a raw `RCLError` about an invalid context from `publish()`,

depending on where in the call stack the teardown lands. `RCLError` is
worse than it looks: it has no public import path, so it cannot even be
caught by name.

Then a second-order failure compounds it. The obvious cleanup —

```python
finally:
    self.publish(stop_command)   # be safe, stop the robot
```

— raises `RCLError` *again*, because the context is already dead. That
exception replaces the one being unwound, so the original error is
destroyed and the save is skipped.

**Fix.** `coco_rl/coco_rl/ramp_env.py` defines `_rclpy_call()`, which
normalises both failure modes to `ExternalShutdownException` and re-raises
anything else unchanged (a failure while the context is *healthy* is a
real bug and must not be swallowed). Every entry point catches
`(KeyboardInterrupt, ExternalShutdownException)`, and every teardown path
is guarded with `if rclpy.ok():`.

**Evidence.** `grep -rn ExternalShutdownException` finds the pattern at
every entry point. Reproduced directly on the pre-fix code: sending a real
SIGINT to `cmd_vel_relay` raised
`RCLError: failed to shutdown: rcl_shutdown already called`; the fixed
version exits silently with status 0. `train_ppo` now saves to
`<prefix>_interrupted.zip` and prints the resume command.

**The uncomfortable part.** Once the context is dead you *cannot* publish
a stop command — there is no way to make that work. What actually stops
the robot is the `diff_drive_controller` watchdog
(`cmd_vel_timeout: 0.5`), measured bringing it from 0.25 m/s to about
1e-4 m/s in under a second. The docstring in `teleop_wheels_node.py` used
to promise a "guaranteed stop command on exit"; it now says what really
happens. A safety claim that only holds on the clean path is worse than no
claim.

---

## 4. The gripper could not hold anything

**Problem.** The arm would reach the cylinder, close the fingers, and lift
— and the cylinder would stay on the pedestal. Tightening the grip and
slowing the lift did not help.

**Diagnosis.** The fingertips were flat pads. A flat pad holds an object
by friction alone, and the friction force available was less than the
inertial load during the lift arc, so the cylinder slid along the pads and
out. More grip force just increased normal force on a contact that was
sliding in the wrong direction anyway.

**Fix.** 6×7×5 mm end-stop lips added to both fingertip *collision* boxes,
protruding 5 mm inward at the pad tips. These **cage** the cylinder — it is
held by geometry, not by friction, so the grasp no longer depends on the
friction coefficient or the lift acceleration.

**Evidence.** Full pick → carry → place verified repeatedly, with the
cylinder confirmed by Gazebo ground truth back on the pedestal at
z = 0.1280 every time. See [RESULTS.md](RESULTS.md).

Better evidence came later. `pick_place.py` used to confirm the grasp by
sleeping 1.5 seconds and assuming success. It now watches `/joint_states`,
and the two outcomes turn out to be cleanly distinguishable:

| Closing the gripper | Finger position |
|---|---|
| onto the cylinder | **stalls at ~0.23–0.25 rad** |
| on empty air | runs to the 0.02 rad setpoint |

So the close step asserts `expect_object=True`, where *reaching* the
setpoint is the failure — it means the fingers met nothing. Every run now
logs the stall position, which is physical evidence that something is
actually held.

**Honest limitation.** This is a simulation-side fix: the lips are
collision geometry, and the visual mesh is unchanged. The hardware
equivalent is a printed fingertip with a raised tip edge or a compliant
pad. Grasp depth is also chassis-limited — poses that extend further curl
the pinch point back toward the chassis, so extra reach needs a longer
wrist link, not better collision boxes. Both are recorded in
[FUTURE_WORK.md](FUTURE_WORK.md).

---

## 5. One corridor would not map

**Problem.** Three mapping attempts produced maps that were locally sharp
but globally wrong — walls at the wrong angle, the arena not closing on
itself. One corridor behind the ramp was consistently the worst.

**Diagnosis.** Two causes, and they needed separating.

*Scan-matcher degeneracy.* The lanes beside the walled ramp are ~2 m
corridors: two long parallel walls and nothing else in view. A laser scan
in that geometry constrains position **across** the corridor and rotation,
but almost nothing **along** it — every candidate pose that slides forward
or back matches the walls equally well. This is the classic lidar-SLAM
degenerate case, and the matcher will happily report high confidence in a
pose that is metres off along the corridor axis.

*Map-frame anchoring.* `slam_toolbox` anchors the map frame at the **odom
pose it sees when it starts**. Skid-steer odometry drifts badly during
in-place rotation — four wheels scrubbing sideways is exactly what wheel
odometry cannot measure. So driving the robot around *before* starting
SLAM meant the map frame was anchored to an already-wrong odom estimate,
and `nav.launch.py` then auto-initialised AMCL assuming map origin = spawn
pose. The result was a map that could not be localised in.

**Fix.**

- Raised `distance_variance_penalty` to 0.8 and `angle_variance_penalty`
  to 1.8 (`gazebo_models/config/slam_params.yaml`), which makes the
  matcher distrust the unconstrained direction instead of overcommitting
  to it.
- Route design over route length: out-and-back lanes so the same walls are
  re-observed from both directions, giving the pose graph real loop
  closures, and **no in-place spins in corridors** — the worst input for
  scan matching in the worst geometry for it.
- Map only from a freshly started simulation, so the map frame is anchored
  at the true spawn pose. Documented in `docs/RUNNING.md`.
- The corridor behind the ramp (x > 5.5) is **deliberately left unmapped**.
  Nav2 rejects goals there, which is the correct behaviour: a planner
  refusing to path into unobserved space is right, and a map that claimed
  to know that corridor would be lying.

**Evidence.** The shipped map covers the whole arena including the ramp
walls, and Nav2 goals at the ramp base and the south wall both succeed
end-to-end — see [RESULTS.md](RESULTS.md).

**Why this is filed here and not under "limitations".** The unmapped
corridor is a *decision*, not an omission. Mapping it properly needs
either visual features on the east wall or tighter odometry fusion; faking
it with a longer drive would produce a confident, wrong map, which is
strictly worse than an honest gap.

---

## 6. Smaller decisions, and things deliberately not done

**Plain nodes for teleop; lifecycle for `slam_toolbox`.** The teleop and
diagnostics nodes are plain `rclpy` nodes. They have no meaningful
configure/activate distinction — they are useful the moment they start.
`slam_toolbox` genuinely is a managed `LifecycleNode` and does nothing
until transitioned, which is why `slam.launch.py:70-81` drives it through
configure → activate explicitly with lifecycle transition events. Using lifecycle
nodes where the lifecycle is meaningless is ceremony, not architecture.

**Velocity caps differ per interface, on purpose.** The controller
saturates at 1.0 m/s / 2.0 rad/s. Teleop matches that; the web joystick
caps at 0.5 / 1.2, and the RL environment at 0.6 / 1.2. Those are
deliberately *below* saturation so that a command is always achievable —
an agent or a phone joystick asking for more than the robot can deliver
gets a silently clipped response, which corrupts both RL credit assignment
and joystick feel. Nav2's own limits are lower again because it plans for
smooth motion, not maximum speed.

**Joint limits are constants, not ROS parameters.** Step sizes and
velocity caps are parameters (`custom_teleop/config/teleop.yaml`) because
they are preferences. Joint limits are not — they describe the hardware,
and a parameter file that lets you *raise* a joint limit is a footgun.
They live in `coco_config/joint_limits.py`, and
`coco_config/test/test_limits_match_urdf.py` parses the xacro and asserts
they still agree. That test exists because they had already drifted:
teleop clamped the gripper to (-0.3, 1.047) under a comment claiming it
matched the URDF, which says (-0.35, 1.1).

**No `black` / `isort`.** The tree is hand-wrapped at ~79 columns with
alignment chosen per-block. Adopting an autoformatter now would produce a
repo-wide reformat commit that buries the actual history, for a
consistency the tree already has. The reasoning is recorded in
`.pre-commit-config.yaml` so the absence reads as a decision. `flake8` and
`pep257` *are* enforced in CI, because those catch defects rather than
opinions.

## Diagnosing and replacing the unclimbable ramp

The RL policy's first result was **0/10 — the robot never climbed the ramp**
(the "before" table in [RESULTS.md](RESULTS.md#reinforcement-learning)). The
easy story is "compute-bound, needs more steps." That was reported honestly at
the time, but it was incomplete: profiling the actual assets showed the policy
was never going to climb *no matter how long it trained*, for two reasons that
are engineering, not training.

**The shipped mesh was not a ramp.** `rampcoco.stl` is a CAD shell,
4.40 m × 2.63 m × 1.10 m (drawn down from millimetres by `scale 0.001` in the
old `ramp.sdf`), 3664 triangles. Profiling its surface along the drive
direction: flat for ~0.4 m, then a **~66° near-vertical face with an
overhang**, then a sustained ~39° grade. Nothing on wheels mounts a 66° face or
a step taller than its own wheel radius. The robot itself was fine —
wheel-contact friction is `mu = 0.7` (no-slip to ~35°) — so the blocker was
purely the geometry the robot was asked to drive up.

**The goal was the foot, not the summit.** `GOAL_X_PROGRESS` was 3.0 m of
forward progress, which from the spawn at `(-2,0)` reaches only world x≈1.0 —
the ramp *foot*. Even on a perfect ramp the episode would have ended before the
climb began. "Climbing" was never the trained objective.

**The fix: a parametric wedge + a real summit goal + a curriculum.** Rather
than hand-fix a CAD mesh, `gazebo_models/scripts/gen_ramp.py` (stdlib only, so
it runs at build time and in a unit test without numpy) emits a clean
right-triangular-prism wedge in metres: one flat driving surface at exactly the
requested grade, foot flush with the ground (zero step). Three curriculum
grades are committed (`ramp_wedge_{12,18,24}.stl`) and selected by the launch
arg `ramp_angle:=`. `coco_config.robot` now owns the ramp geometry
(`RAMP_FOOT_X`, `RAMP_RUN`, `RAMP_SUMMIT_X`, …) as the single source of truth
the launch file and `coco_rl` both read, so the goal (`RAMP_SUMMIT_X - spawn`)
can never drift from where the ramp actually is. The episode terminates at the
crest, so the robot never drives off the wedge's vertical back face.

**Measured, not assumed.** `climb_check.py` drives the robot at the wedge and
reports `peak pitch 18.1 deg` against a requested 18.0° grade while reaching
the summit. That agreement is the point: it proves the physics engine sees the
geometry the generator was asked to emit, so "18°" is a measured property of
the running sim rather than a number in a CAD file.

**Why the grades stop at 24°.** `coco_rl`'s tip-over terminator fires at
`|pitch| > 0.6 rad` (~34°), and on the ramp the robot's nose-up pitch is
roughly the grade — as the 18.1° reading confirms. A 24° wedge is 0.42 rad,
comfortably under the limit, while a 35° wedge would auto-terminate every
episode on contact. The curriculum lives in that window on purpose.

This is why the old 0/10 is kept as a labelled *before* rather than deleted: a
portfolio that only contains successes is not evidence of judgement. The value
is in the diagnosis (mesh profiling → 66°/39° faces → the goal was the foot)
and the engineered replacement, not in hiding the negative result. The compute
reality is unchanged — ~1–8 env steps/s, so a full curriculum is
hours-to-days of wall clock; the scaling paths are in `FUTURE_WORK.md` item 9.

---

## A 3-second timeout that destroyed a 180,000-step run

**Problem.** The first full curriculum run was launched unattended and came
back 35 minutes later with all three phases dead and no model at all:

```
phase 1 (12°): NO MODEL (exit 1)
phase 2 (18°): NO MODEL (exit 1)
phase 3 (24°): NO MODEL (exit 1)
```

Each phase had been training normally — 132, 67 and 130 episodes logged with
returns in the expected band — and then raised
`RuntimeError: set_pose failed for world 'coco_world' — is the sim running?`

**Diagnosis.** The error message pointed at a dead simulator, and it was
wrong. Two pieces of evidence said so. First, `train_ppo.py`'s `finally`
block calls `set_physics` to restore the real-time factor, and its
`set_physics(rtf=1.0): ok` printed *after* the traceback — a successful
service call to the same world moments later. Second, all three simulator
logs ended on the same gz-transport line:

```
[gazebo-1] NodeShared::RecvSrvRequest() error sending response: Host unreachable
```

That inverts the story. Gazebo *received* the request and failed to deliver
its **reply**. `gz_service()` shells out to the `gz service` CLI, which binds
a short-lived ephemeral gz-transport node, waits `--timeout 3000` ms, and
exits. Under `--fast` the real-time-factor cap is removed, so physics
saturates the CPU and the round trip occasionally overruns 3 s. The CLI gives
up and exits; Gazebo then tries to answer a process that no longer exists and
logs "Host unreachable". The call returns false, and `reset()` — which raises
deliberately, because a silently failed teleport would rebase the episode
origin onto a robot lying at the ramp top and poison the whole run — turned a
transient miss into a fatal one.

`reset()` runs **once per episode**, so this was a lottery with thousands of
tickets: at roughly a 1% per-call miss rate, surviving a 60,000-step phase
(~1,700 resets) was never going to happen. The 600-step smoke tests that
validated the runner only performed ~30 resets, which is why they passed.

**Fix.** `gz_service()` takes an `attempts` argument and retries a call that
timed out or answered false, and `reset()` uses `timeout_ms=5000, attempts=5`.
Retrying is only correct because the request is **idempotent** — `set_pose`
carries an absolute pose, so re-sending it is harmless even in the case where
the original request *did* land and only its reply was lost. The hard raise is
kept for the case where all five attempts fail, so a genuinely dead simulator
still stops the run instead of training on fabricated transitions. Default
`attempts=1` leaves every other caller's behaviour unchanged.

**Evidence.** Five tests in `coco_rl/test/test_env_helpers.py` pin the
behaviour with a scripted fake `subprocess.run`: retry until true (and stop
on first success), retry a `TimeoutExpired`, give up after exactly N attempts
rather than silently succeeding, default to a single attempt, and — the one
that matters for diagnosis speed — never burn retries on a missing `gz`
binary, which will not fix itself.

**The transferable lesson.** The failure surfaced as a message accusing the
simulator of being dead, and the fastest route to the truth was the one log
line the error did not mention. A `RuntimeError` raised at the point of
detection describes a *symptom*; when a retry-free call sits inside a loop
that runs thousands of times, the interesting question is never "is the peer
alive" but "what is the per-call failure probability, and how many calls am I
making". `train_curriculum.sh` also retries a whole phase up to twice as a
backstop, because the next transient will be a different one.

## The camera change that would have made things worse

**The claim.** The approved fetch-mission plan called one line "the
highest-value single change in the project": pitch the camera down by
changing `camera_joint`'s rpy from `0 0 0` to `0 -0.6 0`, so it could see
the gripper's workspace. It came from a real measurement — reachable ∩
visible was empty, 2625 reachable cells against 2380 visible, 0 shared.

**It was wrong twice.**

*Wrong sign.* In URDF a positive pitch rotates the forward axis to
`(cos θ, 0, −sin θ)` — nose **down**. `-0.6` aims the camera 34° **up**,
into the back wall.

*Wrong magnitude, either sign.* With fx = fy = 221.765 px and a
half-vertical-FOV of 0.49599 rad, the visible band's far limit is
infinite **only while the pitch stays under the half-vFOV**. Past that a
hard cutoff appears at `z_cam / tan(p − 0.496)`:

| pitch | ground visible | 60 mm target's centre |
|---|---|---|
| 0.000 | d ∈ [0.127, ∞) | [0.071, ∞) |
| 0.400 | [0.055, ∞) | [0.031, ∞) |
| **0.600** | [0.035, **0.656**] | [0.020, **0.369**] |
| 0.800 | [0.019, 0.218] | [0.011, 0.123] |

0.6 rad cuts classification off at 0.37 m — inside the range the mission
needs.

**And it would not have bought the thing it was for.** At 0.6 rad the
nearest visible ground is base-x 0.160 while the arm reaches to 0.1617.
The overlap is a ~1 mm sliver at one height, and at the grasp height it
is empty for *every* pitch. Seeing the gripper's workspace is not
available from this mount at any angle; it is a consequence of a camera
68.5 mm off the ground, not of a rotation. The design that works is the
one the plan also specified: classify at range, approach open-loop. At
the closest station there are 73 mm left to the grasp pose, which at ~1 %
wheel-odometry error is under 1 mm against a 27 mm window.

**What changed instead.** Nothing — `CAMERA_RPY` stays `(0, 0, 0)`, and
carries the reasoning next to the value. `coco_config`'s
`test_the_camera_is_deliberately_unpitched` asserts the pitch stays
inside the half-vFOV, so reversing this means reading why first, and
`test_a_positive_pitch_is_nose_down` pins the sign convention the whole
argument rests on.

**The transferable lesson.** The original measurement was sound and the
conclusion drawn from it was not. "Reachable ∩ visible is empty" says
*something* must change; it does not say a rotation can fix it, and the
one-line fix was attractive enough that nobody checked whether the
rotation had the range to reach. The check that settles it — is the
required angle inside the FOV at all — is one line of trigonometry and
was never run.

## Two objects the arm could not have picked up

**Found while planning M5, would have surfaced in M6.** The fetch
mission's four targets were 60 mm cylinders standing on the crest
platform, which puts their grasp band at base-z 0.030. Scanned from
`arm_ik.ik()`, the arm reaches forward to base-x **0.1299** at that
height, and the chassis collision box ends at base-x **0.120** (its
0.24 × 0.06 × 0.274 box maps through `chassis_joint`'s π/2 rotation to
x ∈ [−0.120, +0.120]). The window the target's axis had to land in:

| target | Ø | `[0.120 + r, 0.1299]` |
|---|---|---|
| red | 12 mm | +3.9 mm |
| green | 18 mm | +0.9 mm |
| blue | 24 mm | **−2.1 mm — impossible** |
| yellow | 30 mm | **−5.1 mm — impossible** |

**Why nothing caught it.** The reach lives in `coco_moveit_config`, the
target geometry in `coco_config`, and the chassis bound in the URDF. Each
file is correct on its own; the defect only exists in their product. And
the failure would have appeared at the *last* step of the mission, after
a successful drive, climb and identification, as an unreachable MoveIt
goal — the most expensive place to find it.

The pick-and-place demo never hit this because its 98 mm pedestal lifts
the target to z = 0.128, where reach is 0.1608. The pedestal was doing
load-bearing work nobody had written down.

**The fix, and why not a plinth.** The targets are now 158 mm-tall
cylinders standing directly on the platform, so the grasp band lands at
z = 0.128 — `arm_ik.fk(0.30, 0.58) = (0.15231, 0.12809)`, the exact pinch
point with measured 32–40 mm lifts. The only change to that validated
geometry is that an obstacle was *removed*.

Cloning the demo's plinth was the obvious alternative and is worse for
three reasons. It parks a static 98 mm block in the lane the robot has to
drive **through** on the up-over-down descent at x = 4.05, where a
cylinder simply leaves with the robot. It reinstates the palm-vs-pedestal
planning collision that `FUTURE_WORK` 5b is about. And its approach
window would have been ~10 mm, extrapolated rather than measured, against
~27 mm measured for the cylinders.

The cost was a static tip angle of 7.2–11.4°, which is the one thing that
had to be measured rather than argued: RPY stays within 2 µrad through
spawn settling and through the robot arriving 0.15 m away in all four
lanes.

**The transferable lesson.** `coco_config/test/test_reach.py` exists
because this class of defect is invisible to every single-file test. It
imports the IK, the geometry and the chassis bound and multiplies them
together — the assertion is not "these numbers are right" but "these
numbers are compatible". `docs/FUTURE_WORK.md` 5b had even asked M5 to
confirm the pedestal obstruction "should not apply" on the flat platform.
It did not apply. The grasp never happened.

## The drift that was blamed on the wrong thing for two milestones

M4 measured +0.61 m of lateral drift over the RL climb, M5 measured
+0.59 m, and both wrote it up the same way: *the policy has no closed-loop
lateral control*. `FUTURE_WORK` item 8b said so, the M5 handoff note said
so, and the recommended fix was a `--randomize` retrain — hours of
unattended compute on the one machine that can run a simulator.

It was the wrong diagnosis, and the experiment that showed it took four
minutes. Teleport the robot to the pre-ramp pose at **exactly yaw 0** and
the same policy climbs 2.5 m with **+0.03 m** of drift, in all four lanes.
Run it again with the lane hold disabled and you get +0.04 m. The policy
was never the problem — it holds a line to three centimetres.

What it cannot do is *correct* a line. And `nav2_params.yaml` sets
`yaw_goal_tolerance: 0.25`, so the Nav2 leg that puts the robot at the
pre-ramp pose is allowed to finish a quarter of a radian off heading.
2.5 m of climb at 0.25 rad **is** 0.64 m of lateral. The entire measured
drift was the previous step's goal tolerance, arriving one stage later.

Two things follow, and the second is the more important one.

**The A/B has to hold the start pose constant, or it measures nothing.**
The first version of this experiment teleported to yaw 0 and compared the
lane hold on and off: +0.03 versus +0.04. On that evidence the lane hold
does nothing and the whole idea is dead. It was only running it against a
*Nav2-legal* heading error — the condition the mission actually operates
in — that produced +0.05 versus +0.58.

**It was a safety defect, not an accuracy one.** Open loop from a 0.25 rad
start heading, the two outer-lane adverse cases finished at −1.174 and
+1.182 on a platform that ends at ±1.25, and neither reached the summit at
all. "Arrives in the next lane" understated it: the robot was driving off
the edge.

The fix is 20 lines and no training. `ramp_driver.lateral_hold` adds a
clamped cross-track + heading correction to the policy's yaw action —
exactly the shape `descend_cmd` already uses on the down-slope, which is
the evidence that heading-hold works on a grade. Worst case 0.053 m, 8/8
summits, `LATERAL_GAIN`/`HEADING_GAIN`/`LATERAL_CLAMP` swept rather than
guessed, and `lateral_hold:=false` still reproduces the bare policy so the
comparison stays runnable.

The gain sweep is worth reading for its own sake. The first instinct was
that the correction was authority-limited — it sat pinned at the clamp for
every step of every climb — so the clamp was swept 0.4 → 2.0. It moved the
residual by 6 mm. The limit was bandwidth: at K_Y = 1.2 the loop's ω_n is
0.49 rad/s against a climb lasting 6 s, so it gets under half a correction
cycle. Raising K_Y to 3.0 fixed it, and raising it further made things
worse *with a sign flip* — the loop crossing the centreline before the
climb ends — which is what makes 3.0 a real minimum rather than the best of
four noisy numbers.

## Stopping where the arm can work, not where the demo stopped

The pick-and-place demo drives to a fixed pinch point at base-x 0.152 and
has measured 32–40 mm lifts there. The obvious thing for the mission is to
stop at the same place. It is the wrong choice, for two reasons that only
show up when the numbers are written down.

**0.152 is not central in the window.** A target's axis has to sit between
the chassis nose plus its own radius and the arm's forward reach. For the
32 mm target that is [0.1410, 0.1565], and 0.152 sits 11.0 mm above the
near bound but only 4.5 mm below the far one. Half the error budget is
spent before the robot moves. The window centre, 0.1488, makes it ±7.75 mm.

**Nothing is lost by moving.** The grasp pose is not a constant to be
matched — `arm_ik.ik_or_none` solves it from wherever the target actually
ends up, which `pick_place.retarget()` has always done. What 0.152 buys is
a *verified* geometry, and that verification is about the pinch height and
the gripper clearance, not about the base's stopping distance.

The far bound was also wrong, by 4.3 mm, and in the dangerous direction.
`test_reach.py` computed it as the arm's reach at the grasp height
(0.16085). But the approach is a vertical descent from
`GRASP_HOVER_CLEARANCE` above, where reach is only 0.15651 — and both ends
have to be in the envelope or the plan cannot be *started*. move_group
reports an unreachable start state and an unreachable goal with the same
error code, so the failure would have read as "the target is too far away"
while the target was fine and the hover above it was not.

## Aligning before a leg you cannot watch

The approach across the crest platform ends blind. `target_finder`'s
minimum range is 0.15 m of surface depth and the camera sits at base-x
0.125, so the last fix lands with the target axis about 0.29 m out while
the base has to stop at about 0.15. Roughly 0.17 m of the approach happens
with nothing looking at it.

The tempting design is to servo until vision drops out and then drive the
remaining distance. That closes the range but does nothing about lateral
error: any residual heading at handoff turns straight into offset over
exactly the stretch that cannot be corrected, and the arm is planar — it
cannot reach sideways at all, so lateral error is not a tolerance, it is a
refusal.

So the approach stops, turns in place until the *bearing* to the target is
nulled, and only then drives straight. The blind leg then runs along the
line to the target: it closes range without reintroducing offset, and
0.17 m at the ~1 % wheel-odometry error measured for this base is under
2 mm against a 7.75 mm half-window.

Turning in place is the one thing `descend_cmd` refuses to do, and its
docstring says why: a stationary skid-steer pivot on a grade is how this
base loses its footing. That is a rule about grades. The align happens on
the level platform, where pivoting is the only way to point at something
without translating away from it — which is exactly the distinction the
handoff between the two controllers exists to make.

## The mesh that both simulators agreed on and only one of them could see

**Problem.** M7 needs the same rough terrain — a washboard, a rubble
heightfield, a bridge with a gap either side, a curb — in both Gazebo and
MuJoCo, agreeing closely enough that a policy trained in one can be
evaluated in the other. The repo already generates terrain as STL
(`gen_ramp.py` writes the wedge with nothing but `struct`), Gazebo already
consumes STL, and MuJoCo loads STL too. Sharing one mesh is the obvious
answer, costs nothing, and is wrong.

**Diagnosis.** MuJoCo replaces every mesh with its **convex hull** for
collision purposes. The manual says so plainly — *"Meshes specified by the
user can be non-convex, and are rendered as such. For collision purposes
however they are replaced with their convex hulls"* — but it is easy to
read past, because the mesh still *renders* correctly. The simulator looks
right.

Measured rather than argued: a V-trough STL with its floor at **z = −0.400**
was loaded into MuJoCo and a 0.05 m sphere dropped into it. It settled at
**z = +0.0496** — on the lid of the hull, **450 mm above the floor it was
supposed to fall into**.

Every concave feature in the Yard is that shape. The washboard troughs, the
rubble depressions, the bridge gap and the curb undercut would all have
existed in Gazebo and been paved flat in MuJoCo.

**What makes this worth writing down is how it would have been missed.**
The two simulators would have agreed on the mesh file, on its checksum, and
on every height sampled from the analytic function that produced it. The
parity test as originally specified — sample terrain height at ~200 points
and compare — would have **passed cleanly** while the robot drove on two
completely different surfaces. The bug is invisible to every check that
does not involve contact.

**Fix.** One analytic `height(x, y)` in `yard_params.yaml` is the single
source of truth, and each simulator gets a representation it can actually
collide with: primitives (boxes) wherever the shape is exactly expressible,
a MuJoCo `hfield` and a matching Gazebo mesh only for the genuinely rough
patches, emitted on the same triangulation. No shared concave mesh
anywhere.

**Evidence.** The parity test is physics-based: it drops probe spheres of
wheel radius at sample points in both engines and compares where they come
to rest, including points deliberately placed inside troughs, depressions,
the bridge gap and the cavity beneath the deck slabs. A height-sampling
test would have reported success on a world that could not be driven.

(That list said "the curb undercut" until the probes were actually run.
The undercut is gone: it had no floor, and flooring it would have made the
curb unclimbable at every approach speed. Concave coverage now comes from
the under-deck cavity, which the robot never drives into. See RESULTS.md.)

**The transferable lesson.** When two systems are supposed to agree, test
the thing that matters rather than the thing that is easy to compare. File
identity, checksums and sampled geometry are all cheap proxies for "the
robot experiences the same surface", and all three would have been green
here. The expensive check — put a body in the world and see where it
ends up — is the only one that was measuring the actual claim.

---

## Two derivations that were right about the wrong regime

Both of these were mine, both were written confidently, and both were
wrong in the same way: a correct argument applied to a regime that was not
the one being asked about. Recording them because the failure mode is
reusable, not because the conclusions were dramatic.

### "A 60 mm curb is impossible for a 58.5 mm wheel"

**What was argued.** A vertical step is mountable only while the wheel's
contact with the step edge is *below* the axle. Above it, the reaction
from the face has a moment about the axle that drives the wheel down and
back, and drive torque makes it worse rather than better. At 60 mm the lip
sits 1.5 mm above a 58.5 mm axle, so the sign is wrong — not marginal,
wrong. The chassis then settles it: the underside is 13.5 mm up and the
box front face is at base-x 0.120 against a wheel front at 0.1485, so a
60 mm face is struck by the belly 28.5 mm behind the wheel.

**Why it is right.** Quasi-statically, it is. Push the robot slowly into a
60 mm step and it will not climb it, for exactly those reasons.

**Why it is wrong.** The question was never quasi-static. §2.2's whole
premise for Route C is a *momentum* strategy — back off, accelerate, strike
the step with stored kinetic energy. Driven, the 60 mm step **is**
mountable, at an approach speed of **1.00 m/s** (measured). The robot
pitches on impact and the contact geometry that the static argument fixes
at the lip is not the geometry that obtains a few milliseconds later.

**What should have been written.** Not "impossible" but "not mountable
below 1.00 m/s" — which is still decisive here, because `MAX_LIN` is
0.4 m/s and 1.00 is 2.5x outside the action space. The *conclusion*
survived; the *reason* did not, and the reason is what a reader would have
carried forward to the next obstacle.

**The transferable lesson.** A static force balance answers "can it hold
this position", never "can it reach that position". When the capability
under test is explicitly dynamic, a quasi-static derivation is evidence
about a different question, and the word "impossible" should be reserved
for claims that hold in the regime actually being used.

### "NavFn's A\* terminates the fill early"

**What was argued.** NavFn's A\* mode produces worse paths than its own
Dijkstra because A\* stops the potential-field fill early under heuristic
guidance, trading optimality for speed.

**Why it is wrong, verified against the source.** Both modes stop in the
same place. `propNavFnAstar` breaks on `potarr[startCell] < POT_HIGH`, and
`propNavFnDijkstra` does the same whenever `atStart` is true — which
**`nav2_navfn_planner/src/navfn_planner.cpp:272` passes**
(`planner_->calcNavFnDijkstra(cancel_checker, true)`). Neither fills the
map; both stop when the start is reached. Nor is the heuristic stored in
the field: `updateCellAstar` writes `potarr[n] = pot` and only then adds
the Euclidean distance-to-start, and only for choosing a priority bucket.
`potarr` holds pure cost-to-goal in both modes.

**The actual mechanism.** NavFn does not read its path off the search
tree. `calcPath` descends the *gradient* of the potential field, and at
each step it inspects the cell and all eight neighbours; if **any of the
nine** is unvisited (`potarr >= POT_HIGH`) it abandons the interpolated
gradient and takes a grid-locked 8-connected step to the lowest neighbour
— the log line is literally "Pot fn boundary, following grid". `gradCell`
also skips any neighbour still at `POT_HIGH` when forming the gradient, so
the estimate goes one-sided near the frontier.

A\*'s entire purpose is to visit fewer cells, so in A\* mode the unvisited
frontier runs **close to the path**, and every path step whose 3x3
neighbourhood touches it degrades to a grid step. Dijkstra expands
isotropically, so by the time it reaches the start the path is padded with
visited cells and smooth descent is available almost everywhere.

**So the honest statement is not "A\* is worse than Dijkstra".** It is
that NavFn recovers its path from a *neighbourhood* of a potential field,
and A\* deliberately does not fill the neighbourhood. That is a property
of this implementation, and it is a genuine point of contrast with
`SmacPlanner2D`, which back-traces the node chain and has no coupling
between how much was explored and how good the path is.

**The transferable lesson.** "Where does the search stop" and "what does
the consumer of the search need" are different questions. The bug in the
reasoning was assuming the path came from the search, when it came from a
field the search happened to populate. Read the consumer, not just the
producer.

---

## A field that went stale inside a topic that never did

C2-M1's HUD marked every source stale by the age of its last message.
That rule is right, and it still missed the worst number on the display.

`ROBOT PITCH` was copied from `/ramp/status`, which `ramp_driver`
publishes on a 5 Hz timer that runs whatever the node is doing. The
`pitch` field inside it is only ever *written* in two places — the climb
loop and the descend loop. Between segments nothing assigns it, so the
attribute keeps its last value and the timer keeps broadcasting it. The
message was never late. The number in it was 30 seconds old.

Measured over a full fetch: the field held **−0.314 rad** through the
platform approach and the whole pick while `/imu` had already returned to
0.000 — a **0.314 rad** error, on a topic arriving punctually at 5 Hz.
Later in the same run it held `−0.000` for **79.2 s** of the drive home
while the robot genuinely pitched to **−0.217**. So it does not merely
report an old slope; it reports whatever the last segment left behind,
which can be too flat as easily as too steep.

**The part that makes this worth writing down** is what it would have
done next. The climb terminates `GOAL_MARGIN = 0.3` m short of the
crest, so the last sample is always taken on the uniform 18° face, quasi
-statically — where body pitch and surface grade are the same number.
`RAMP_ANGLE_DEG = 18` is `0.31416` rad. The stale value is therefore
*always* almost exactly the terrain grade. C2-M2's first deliverable is a
grade estimator. Built on this field it would have matched ground truth
on every metre of ramp in every test, and then reported an 18° slope on
flat ground forever after. It would have passed.

Two changes, because the defect has two halves and each is wrong on its
own terms:

- **The source stops claiming a measurement it is not taking.**
  `pitch=--` while `segment == 'idle'`, which is the convention
  `lateral` in the same status line already used for "no lane chosen, so
  no cross-track to report". The segment's final sample moves to the
  `climb finished` log line, where a value that has stopped being true
  belongs. Nothing is lost; it is filed under a timestamp instead of
  broadcast as current.
- **The consumer reads the right topic.** `ROBOT PITCH` comes from
  `/imu` at 50 Hz, aged like every other HUD field. Body attitude was
  never the ramp driver's to publish, and routing it through the node
  that happens to run the policy meant the field could only be alive
  during two of the mission's seven steps.

**The transferable lesson.** Message age and value age are different
quantities, and a periodic publisher silently converts the second into
the first. Staleness detection on the subscriber can only ever see when a
*publisher* stopped; it cannot see when a *measurement* stopped. Where
those differ, the publisher has to say so — which is what `--` is for.

A second, smaller one: this was invisible from reading the code and
obvious from one instrumented run. `pitch_probe.py` exists because
putting the candidate signal and two independent ground truths in the
same CSV with timestamps turned "either genuine, or held from the climb"
into a table in about four minutes.

---

## `/approach/target` publishes once, and that is correct

Recorded because the obvious-looking fix was already written down as a
future idea, and it was wrong.

`/approach/target` is a `geometry_msgs/PointStamped` published **exactly
once per approach**, VOLATILE, at the moment `approach_server` arrives. A
subscriber that connects afterwards never sees it. That reads like a
reliability hole, and `PROJECT_STATE.md` carried "make it
TRANSIENT_LOCAL so late subscribers see it" as a candidate improvement.

Making it TRANSIENT_LOCAL would introduce a defect rather than remove
one. The payload's `frame_id` is **`base_footprint`**. Durability
delivers the last message to whoever joins later; here that means handing
a new subscriber a coordinate expressed in a frame whose pose has since
changed — a point that was correct when it was measured and is wrong by
however far the robot has driven since. Latching a robot-relative
measurement is a category error: durability persists a value, and this
value's meaning is not persistent.

The reliability worry does not survive measurement either. Both
endpoints are started by `mission.launch.py`, and `grasp_server` creates
its subscription in `__init__`, minutes before any approach runs — there
is no startup race in the deployed configuration. `ros2 topic info -v`
against a live mission: publisher count 1, subscription count 2
(`grasp_server` and `rviz2`), QoS compatible. And the consumer already
defends itself: `_target_pose()` uses the fix only within
`APPROACH_FIX_MAX_AGE = 120 s` and otherwise warns and grasps at the
nominal stop pose, which the approach reaches by construction. The cost
of a missed message is a warned nominal grasp, not a stalled mission.

**No change made.** What is genuinely imperfect is the shape, not the
QoS: this is the *result of* `/approach/run`, and a `std_srvs/Trigger`
response has nowhere to put a point, so the result travels beside the
call instead of in it. That is an architectural mismatch with no measured
consequence. C2-M3 replaces these Trigger services with actions; the
estimate should become an action result there, for that reason and not
this one.

**The transferable lesson.** "Late subscribers miss it" is a symptom
description, not a diagnosis. Ask what the message *is* — state, event,
request, or result — and what frame its contents are valid in. A
transient event in a robot-relative frame wants VOLATILE, and reaching
for TRANSIENT_LOCAL because it sounds safer would have made a
correctly-designed topic subtly wrong.

## The tip terminator was measuring the wrong angle

**C2-M2.0. Decision: option B — a Yard-specific rule, with v1 frozen.**

M7 Phase 3 left this undecided and said so. 101 of 120 Route C episodes
under a tuned B2 ended as `tipped`, at the **lowest cross-track in the
whole matrix (0.035 m)**, with **0 of 101 roll-dominated**. Steering was
not the failure. `TIP_LIMIT = 0.6` rad is 34.4° measured against **world
vertical**, so Route C's 16.3° grade consumed 16.3° of the budget before
the robot had done anything, leaving 18.1° against a measured 20.6°
excursion. Computed from the model's own mass distribution — total
2.9715 kg, CoM at (−6.5, +3.2, 59.6) mm — genuine static rear-over
relative to the surface is **54.5°**. The terminator fired **34° short of
falling over**, on the manoeuvre that mounts the curb, which is the
momentum strategy M7_DESIGN §2.2 exists to test.

**Reproduced live this session**, one Route C episode, seed 7, open loop
at throttle 0.6: terminated at step 184 with body pitch **−45.30°** on a
**+20.16°** surface — **25.14° surface-relative**, against 54.5°.

### What was changed

The **reference frame, not the threshold**. `yard_env` now measures
`|roll|` and `|pitch|` from the local surface normal, using the analytic
surface `coco_sim.yard.height` the Yard is generated from. 0.6 rad is
kept exactly, so **no new tuning constant enters the repo** and the
terminator keeps its protective role: a robot 34.4° off the slope it is
standing on is still in trouble, while a robot standing still on a 26°
chute is not.

Two guards, both from numbers that already existed:

- **An absolute backstop at 54.5°**, the measured static rear-over. Past
  it the robot has fallen over on any reference frame. This is what
  catches the case where the surface is not defined — over the bridge
  void the analytic surface drops 0.650 m and a central difference across
  that edge reports tens of degrees of "grade" that is an artefact of the
  discontinuity.
- **The surface correction is itself bounded by `TIP_LIMIT`**, so the
  worst a bad surface reading can do is double the effective absolute
  limit, which the backstop then catches.

On the seed-7 episode the terminator now fires at step **185** instead of
184, with body pitch −54.51° — a genuine rear-over. The mechanism is
fixed; whether the *population* of 101 changes is a C2-M2.1 measurement
and is **not yet measured**.

### Why option B was available at all, and it is structural

`TIP_LIMIT` **is not in `coco_config`.** It is written out independently
in four modules:

| module | owns | changed? |
|---|---|---|
| `coco_rl/yard_env.py` | the Yard | **yes** |
| `coco_rl/reward.py` | the v1 curriculum, and so the shipped PPO policy | no |
| `coco_rl/mujoco_env.py` | the flat parity model | no |
| `coco_rl/ramp_driver.py` | the mission's runtime check | no |

So the Yard's terminator could be corrected **without touching a single
number the v1 results were measured against**. M7 Phase 3 declined to
apply the fix precisely because it believed the constant was shared; it
is not, and that is what makes this decision cheap. `test_yard_env.py`
asserts the other three are still 0.6 rad absolute, so the separation
cannot rot.

**v1 impact: none.** The 19/20 fetch matrix, the 10/10 traverse and the
shipped policy's training conditions are all untouched. Yard numbers
measured before this change — M7 Phase 3's 1,080-episode baseline table —
are **not comparable** on the tipped/completed split for Route C, and
C2-M2.1 must say so when it reports.

---

## What a robot can know about the ground it is on

**C2-M2.0. The observer, and one negative result worth more than the
estimator it replaced.**

C2-M2 asks whether the robot can estimate terrain state from onboard
signals and recover a privileged controller's performance. The answer
splits cleanly, and the split is the finding.

### Grade: observable, and accurate

Measured on Route A's uniform 12.000° face: body pitch reads **−12.00°**
against a true surface grade of **+12.00°**. So **nose-up is NEGATIVE
pitch**, and

    grade = -(filtered body pitch - flat-ground reference)

A rename of `body_pitch` to `grade` would have been wrong in **sign** as
well as in reference. That is worth recording next to the C2-M1.5 entry
above: that milestone caught a pitch field that was stale, and this one
found that even the correct field needs a sign flip and a reference
before it is a grade.

The reference is not assumed to be zero. It absorbs IMU mounting
misalignment and the standing pitch the compliant contact takes under the
robot's own weight, and it is measured once, on the flat apron, inside
`CocoYardEnv._measure_rest_z` — which was already settling the bare robot
there for four seconds. Measuring it per episode instead would have given
the observer-driven controller stationary steps the baselines do not get,
and an unequal step budget is not a controller comparison.

**Body pitch is not terrain grade**, and the filter is where that is
handled. On a smooth face they agree to **0.03°**; on Route C's rubble
they disagree by **1.3–2.7°**, because the chassis sits on two contact
patches and not on the surface under its centre. The low-pass time
constant is **derived from the rubble's own correlation length** — 0.12 m
at ~0.25 m/s is a 0.48 s feature, so 0.5 s suppresses it while still
tracking a ramp entry within 0.12 m of travel. The residual scatter is
published as `grade_roughness`, which is a live measurement of how badly
body pitch is representing the surface.

Measured accuracy against the analytic surface, both axles on one plane,
seed 3, open loop: **route A 0.106° MAE, route B 0.366°, route C 1.433°**.

### Friction: NOT identifiable, and that is a property of the robot

Three candidate signals were built and two were killed by measurement.

**Wheel encoders cannot see friction.** Sweeping only μ on Route B at
fixed geometry and seed:

| μ | wheel speed | servo lag | body speed | true slip |
|---|---|---|---|---|
| 0.55 | 0.3189 | 0.0185 | 0.2146 | 0.327 |
| 0.70 | 0.3189 | 0.0185 | 0.2758 | 0.135 |

The wheel speed is identical to four decimals and so is the velocity
servo's lag behind its command — the actuators have authority to spare.
Wheel-odometry slip is **identically zero by construction**, and the only
observable separating the two surfaces is body velocity.

**Inertial body velocity was built, and rejected.** Integrating specific
force with gravity removed by the measured attitude, after a
zero-velocity update, at 10 Hz and again at the IMU's real 50 Hz: it lost
**0.10–0.15 m/s inside two seconds** against a true 0.28 m/s, and the
estimated slip came out in the **wrong order** between the two surfaces.
A world-frame mechanisation would have been exact — and exactly circular,
because the Yard's IMU is **noiseless**: `yard_params.yaml` records
`imu_noise_sigma: not_yet_measured` because `coco_robo2.xacro` declares no
`<noise>` element. An integrator would have scored its own arithmetic
rather than the robot's observability, and transferred nothing. **Nothing
in the observer integrates.**

**What is reported is a traction-demand ratio**, `tau = f_t / f_n`, the
tangential over the normal specific force at the contact patch. No
integration, no drift, no fitted constant.

And it does not move with μ:

| route | tan(grade) | μ 0.35 | 0.45 | 0.55 | 0.70 |
|---|---|---|---|---|---|
| A, 12° | 0.2126 | 0.2131 | 0.2128 | 0.2128 | 0.2127 |
| B, 26° | 0.4877 | — | — | 0.4950 | 0.4874 |

**τ equals tan(grade) to four decimal places on every surface the Yard
builds.** Route A spans **0.0003** across a μ span of 0.35.

The physics closes it:

- A robot climbing steadily is in equilibrium, so the tangential force is
  `m g sin(grade)` whatever μ is. Equilibrium pins τ at `tan(grade)` — a
  property of the **geometry**.
- τ reveals μ only when the contact saturates, and saturation needs a
  demand above `μ g cos(grade)`. On level ground the drivetrain cannot
  produce one: `MAX_LINEAR_ACCEL` is **2.0 m/s²** against `μg = 3.43
  m/s²` at the slick end. **This robot cannot spin its wheels on the
  flat.**
- On a grade the margin shrinks and saturation becomes reachable — but
  that is exactly where equilibrium has already pinned τ.

The two conditions never overlap. **Coulomb friction is not identifiable
on this robot, with an IMU and wheel encoders, anywhere in the Yard's
operating envelope.** That is a statement about the robot and its
instrumentation, not about this estimator.

**Two false starts, both kept in the source**, because each looked right
until it was checked. Modelling the normal load as `g·cos(grade)` instead
of measuring it left the bound `τ ≤ μ` holding on **27%** of Route B's
samples. Taking the ratio in the body frame rather than the contact frame
— they differ when the chassis rears, measured at body pitch −30° on a
26.66° chute — broke it on **47%**, *and produced a spurious monotone
reading in μ that looked exactly like the result being sought*. The
apparent signal was the error.

### The information boundary is a type, not a convention

`DeployableSignals` is the only thing `TerrainObserver` accepts: IMU
attitude, IMU specific force, wheel speeds, commanded twist. `GroundTruth`
lives in `sensor_model`, shares **no field name with it**, and is what
scores the observer and schedules B2. A leak is therefore a `TypeError`
rather than a review miss — and a test asserts the two share no field
name, so a copy-paste across the boundary cannot typecheck.

That is deliberate, and C2-M1.5 is why. A signal that is correct
everywhere it happens to be tested and wrong everywhere else passes
review, passes tests, and costs a milestone. A convention would not have
caught it.

**Two things B3 is *not* deployable with respect to, stated rather than
buried.** It receives `x_world`/`y_world`, exactly as B0, B1 and B2 do,
because the experiment isolates the *terrain* channel: all four
controllers get identical pose, so the only difference between B2 and B3
is grade and friction. Degrading the pose channel too would confound the
measurement and break comparability with M7 Phase 3's 1,080 episodes —
and localisation is C2-M5's milestone. It also knows which route it is
on, like B1 and B2, and reads `y_centre` from it; that is fixed design
geometry in `yard_params.yaml`, not among the randomised quantities, and
it is the reference path the baselines module already hands every
baseline on purpose.

### What this means for the experiment

**B3 is a grade-aware controller with a traction bound, not a
friction-aware one.** B2's privileged advantage, as tuned, is exactly one
number — throttle interpolated on true μ, since `TUNED_SCHEDULE` sets
`grade_k = 0.0` and `lateral_lo == lateral_hi` on all three routes — and
the observer cannot recover that number. How much of B2's performance
survives anyway is what C2-M2.1 measures. The decision rule is unchanged
and was fixed before any of this ran.

---

## An estimator runs on the sensor's clock, never on the consumer's

C2-M2.0 fixed the terrain observer's rate at **50 Hz** — the rate
`coco_robo2.xacro` declares for `/imu` — and `MAX_AGE` at 0.1 s, "five
missed samples" at that rate. `terrain_observer_node` then advanced the
estimator from its **10 Hz publish timer**, sampling whichever IMU message
happened to be latest.

Those two facts are incompatible and the arithmetic says so immediately:
a 10 Hz timer picking up a 49 Hz signal advances the stamp by about five
quanta, ≈ 0.102 s, which is past `MAX_AGE`. Measured live: the observer
withdrew its own estimate on **431 of 431 samples** of a complete 18°
climb, reporting `stale input: 0.100 s > 0.100 s` every time. It never
produced a single valid estimate, on a perfectly healthy robot.

**The decision: estimation happens in the sensor callback, publication in
the timer, and they are separate methods so the two rates cannot be
confused again.** `_estimate` folds in each IMU sample at 50 Hz;
`_publish` is a pure read of the most recent estimate at 10 Hz. A test
pins both halves, including the failure: feeding at 0.102 s spacing must
still withdraw.

This generalises past this node. **Decimating a sensor to the control rate
produces a different sensor**, and `B3.observe` already said so for a
concrete reason — the traction channel's acceleration deficit is a
transient a 10 Hz sample misses. Any future consumer of `/terrain/state`
subscribes at 10 Hz and that is fine; what must not move is where the
*estimator* is stepped.

**Why no off-line test caught it.** Every existing test drove
`TerrainObserver` directly, at 50 Hz, which is the rate the estimator is
designed for — so the tests exercised the estimator and never the wiring
around it. The node had never been constructed by anything, anywhere,
before C2-M2.1 launched it. That is the general lesson: a pure core with
thorough unit tests and an untested adapter is not a tested system, and
the adapter is where the rate, the QoS and the frame conventions live.

---

## The decision rule was not moved after the result arrived

C2-M2.1 measured that the observer-driven controller clears the
10-percentage-point bar on every route — gaps of **+0.0, +1.7 and
+7.5 pp** on `ascent` — and, in the same run, that the privileged
controller **completes Route A 97.5 % of the time against B3's 0.0 %**.

The gap is 0.0 pp on Route A because ascent does not discriminate there
(every controller including open-loop reaches the deck 92–99 % of the
time), and **not** because the observer recovered anything: it fell back
on 120 of 120 Route A episodes and was byte-identical to B1. The task the
rule scores is insensitive to the largest privileged advantage the
benchmark found.

That is a good reason to think `ascent` was the wrong task. It is **not**
a good reason to change the task, and the rule was left exactly as frozen.

**The decision: apply the rule as written, record the verdict, and record
the evidence against the rule's own premise beside it.** Re-scoring on
completion after seeing that completion tells a different story would be
choosing the metric that gives the answer — the specific failure the
pre-registration in C2-M2.0 existed to prevent, and it would have
converted an honest "no RL" into a dishonest "RL justified" with no new
measurement behind it.

The premise that has actually weakened is worth stating plainly, because
it is what a future rule-setter needs. C2-M2.0 chose ascent because M7
Phase 3 measured B1 reaching the deck 99 % of the time and then falling
off the bridge in 105 of 120, which made completion look like a score on
the deck-convergence geometry — an open M7 Phase 4 decision, not a
terrain-control result. **B2 now crosses that bridge 117 times in 120
using nothing but terrain-aware throttle.** A pure geometry problem does
not yield to terrain information. Whoever sets the next rule should
weigh that; whoever ran this one had no business acting on it.
