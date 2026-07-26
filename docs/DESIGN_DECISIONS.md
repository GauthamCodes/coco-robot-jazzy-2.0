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
