# M7–M10 — The Yard: making the learned policy load-bearing

**Status: proposed. Nothing in this document has been built or measured.**
Every number here is a *target* or a *derivation*, never a measurement. When a
milestone lands, its numbers move to `docs/RESULTS.md` with the `(measured)`
marker the rest of this repo uses.

**Precondition: do not start M7 until M6 closes.** One verified end-to-end
fetch, one pushed branch, one recorded video. M6 currently has an unverified
fix in it (the `[0.1510, 0.1565]` window). Stacking a new world, a new reward
and a new simulator on top of an open failure means five candidate causes for
the next thing that breaks.

---

## 1. The problem with the current RL

Stated plainly, because an interviewer will state it for you if you don't.

The v1 policy climbs a **fixed parametric wedge** — one grade, one friction,
one width, no obstacles, straight line, full state observable. A tuned PD
controller does that. Worse, the thing that actually threatened the mission —
the +0.6 m lateral drift — was fixed by `lateral_hold()`, which is **analytic
PD, not learning**. The RL was not what saved the traverse.

So the honest reading of v1 is: *PPO was applied to a problem that did not
require it, and the real control insight was classical.* That is a fine
learning exercise and a bad portfolio claim.

M7 does not add RL. It **builds the terrain that makes RL the right answer**,
and — critically — it builds the classical baselines that could prove it
isn't. A learned policy with no baseline is a demo. A learned policy that
beats a gain-scheduled PD across a randomised terrain distribution, with the
table to show it, is a result.

### 1.1 Why a maze is the wrong instinct

A maze is a **planning** problem. `SmacPlanner2D` solves it optimally and
already went 10/10 on a ten-goal tour. Putting a policy in a maze is worse
planning at higher cost, and it is the single most common way a robotics
portfolio project signals that the author does not know what RL is for.

RL earns its place where the **contact dynamics cannot be written down**:
slip, tipping, momentum, unknown friction, terrain the robot cannot see
underneath itself.

The maze idea still has a home — just one layer up. See §2.1.

### 1.2 The layer split, stated once

| Layer | Owner | Why |
|---|---|---|
| *Which route to the platform?* | **Nav2 / A\*** | Discrete, fully observable, graph search is optimal |
| *How do I survive this route?* | **RL policy** | Continuous, partially observable, contact-dominated |
| *Where exactly is the target?* | **Perception + approach_server** | Needs mm precision; a policy cannot hold 5.5 mm |
| *How do I pick it up?* | **MoveIt** | Kinematics, not dynamics |

Every layer is doing the thing it is best at, and removing any one of them
breaks the mission. That sentence is the whole pitch for this project.

---

## 2. The Yard — v2 world

The wedge is replaced by a **circuit with three competing ascent routes** onto
a raised platform, an obstacle-carrying platform deck, and a loaded descent.

```
                      ┌──────── platform deck, z = 0.650 ────────┐
                      │  washboard →  narrow bridge → target bay  │
                      └──▲──────────▲──────────▲──────────────┬───┘
                         │          │          │              │
                    Route A     Route B     Route C        descent
                   long haul     chute      rubble         20°, loaded
                    12°, cambered  26°, slick  16°, stepped
                         │          │          │              │
    ┌────────────────────┴──────────┴──────────┴──────────────▼───┐
    │           flat apron — Nav2 territory, route selection      │
    │                    (optional: walled maze)                  │
    └──────────────────────────────────────────────────────────────┘
```

### 2.1 The flat apron, and where the maze goes

The apron keeps the existing SLAM/Nav2 arena. If you want the maze, **put it
here** — a walled layout on the flat that Nav2 must plan through to reach the
foot of the chosen route. This makes the maze a legitimate showcase of
`SmacPlanner2D` and the costmap stack, and keeps it out of the policy's way.

It also gives you a clean A\*-vs-Dijkstra comparison on a topology where the
difference actually shows, which strengthens the M2 result you already have.

### 2.2 The three ascent routes

Each route is deliberately *good at something and bad at something else*, so
route choice is a real decision rather than a fixed answer.

| | Route A — long haul | Route B — the chute | Route C — the rubble |
|---|---|---|---|
| Grade | 12° | 26° | 16° mean |
| Run | 3.1 m | 1.4 m | 2.4 m |
| Width | 2.5 m | **1.2 m** | 2.0 m |
| Surface | smooth | smooth | heightfield, RMS 25 mm |
| Cross-slope | **0–8° camber** over the middle 1.5 m | 0° | 0–4° |
| Friction μ | 0.7–1.1 | **0.35–1.10** | 0.6–1.0 |
| Special | — | narrow, no recovery room | **60 mm curb at the top lip** |
| Fast when | never | dry and confident | never |
| Fails by | drifting off the low side | spinning out, sliding back | high-centring on the curb |

**Route A's camber is the sharpest test in the world.** Lateral drift on a
cross-slope is a function of grade, camber, friction, speed and mass
distribution simultaneously. There is no clean analytic law. `lateral_hold`'s
fixed `LATERAL_GAIN = 3.0` was tuned at zero camber; at 8° it will either
undershoot or oscillate, and the existing gain table in §6.2 already shows
that past 3.0/2.5 the error **changes sign** rather than shrinking. That table
is the evidence that a single gain set cannot cover a distribution.

**Route B's friction range is the second test.** One gain set tuned at μ = 0.9
is sluggish at μ = 1.1 and spins out at μ = 0.35.

**Route C's curb is the third, and the most interesting.** Mounting a step
taller than the belly requires a *momentum strategy* — back off, accelerate,
strike it with stored kinetic energy. It is discontinuous and non-obvious, and
a PD controller stalls against it indefinitely. This is the single clearest
"a policy found something I would not have written" result available.

> **Amended in Phase 2 (measured).** Two numbers in the paragraph above
> were wrong. The wheels are **58.5 mm in radius**, not 65 mm. And the
> 60 mm curb is **not mountable inside this robot's action space**: it
> needs an approach speed of **1.00 m/s**, against `MAX_LIN = 0.4 m/s`.
> The curb is built at **28 mm**, which needs **0.35 m/s** — 88 % of
> maximum — so the momentum strategy is still required and still cannot be
> reached by a PD controller creeping up to the step.
>
> Note this also corrects the Phase 2 survey's own *derivation*, which
> argued 60 mm was impossible because it exceeds the wheel radius and puts
> the contact above the axle. That is true quasi-statically and false
> dynamically; "impossible" should have read "not below 1.0 m/s".
> Numbers, and the friction sensitivity, in `docs/RESULTS.md`.

> **Route runs are derived, not specified.** The grades and runs in the
> table above are mutually inconsistent with the 0.650 m deck: 3.1 m at
> 12° rises 0.659, 1.4 m at 26° rises 0.683, 2.4 m at 16° rises 0.688.
> Grade is held — it is what the physics responds to — and run is computed
> as `rise / tan(grade)`, which also makes §2.5's grade jitter well
> defined: the foot slides and the deck never moves. Route B's jitter is
> capped at ±2° rather than ±3°, because 29° leaves only 5.4° to the
> 0.6 rad tip terminator and a successful climb could be scored as a fall.

### 2.3 The platform deck

No longer a plain slab. Three sections in series:

| Section | Geometry | What it tests |
|---|---|---|
| Washboard | sinusoid, 40 mm amplitude, 0.35 m wavelength, 1.2 m long | Pitch oscillation couples to wheel contact; needs anticipatory throttle. Drive it at the wrong speed and the chassis resonates |
| Narrow bridge | 0.5 m wide, 1.0 m long, gap either side | Catastrophic failure zone. A 2D lidar **cannot see a hole** — this is a negative obstacle, and it is why the policy must hold a line rather than merely arrive |
| Target bay | flat, 1.0 m, four lanes as in v1 | Unchanged. `approach_server` and `grasp_server` take over here exactly as they do today |

The bay staying flat and unchanged is deliberate: it protects the 16/16
perception result and the M6 grasp window from being invalidated by this work.

> **Amended in Phase 2 (measured).** Two of the three sections above are
> undrivable as specified, against the robot's **13.5 mm ground
> clearance** — a number that had never been written down when this table
> was drafted.
>
> - **Washboard 40 mm → 8 mm.** The belly-strike threshold is **12.92 mm**
>   amplitude; at 40 mm the chassis is **28.29 mm inside** the terrain.
>   The **wavelength is unchanged at 0.35 m** on purpose: the difficulty
>   is the wheelbase-to-wavelength ratio (0.18 m = 0.514 λ, near
>   antiphase), so keeping λ keeps the resonance and only the amplitude is
>   scaled to fit under the belly. Length is **1.225 m**, not 1.2, so the
>   sinusoid is exactly 3.5 wavelengths and meets the neighbouring slabs
>   flush instead of ending on a 3.47 mm step.
> - **Bridge 0.5 m → 0.65 m.** The robot is 0.314 m wide. At 0.5 m the
>   lateral margin is ±0.093 m against a measured worst-case cross-track
>   of 0.301 m, so **every classical baseline is 0 % by construction** and
>   the M7.2 matrix has no gradient in it — an experiment whose result is
>   fixed before it runs. At 0.65 m the better baselines clear it and the
>   worse ones do not, which is what a baseline table is for.
> - **Rubble RMS 25 mm → 8 mm**, by the same belly-strike bound: over 400
>   realisations × 4 m, 25 mm intrudes **+12.73 mm**.
>
> The capability under test survives every rescale. The obstacle was
> rescaled, never the robot: the 10/10 traverse, the 19/20 fetch matrix,
> the 5.5 mm grasp window and the contact calibration are all measurements
> against a frozen machine.

### 2.4 The descent

Single 20° ramp, but taken **carrying the object**. Payload mass and CoG
offset are randomised, so the tipping margin changes run to run. The `up`
carry pose clears +0.210 at 24° per the existing table — that stays valid, but
the *dynamic* margin under braking on a rough descent does not, and has never
been measured.

### 2.5 Randomisation — the part that makes it RL

Sampled fresh per episode. Without this the whole exercise collapses back into
a memorisable fixed course.

| Parameter | Range | Justification |
|---|---|---|
| Grade jitter | ±3° per route | Prevents geometry memorisation |
| Friction μ | **0.35 – 0.70** | The core adaptation demand — narrowed, see below |
| Heightfield seed | new each episode | Route C never repeats |
| Camber | 0 – 8° | Breaks fixed-gain lateral hold |
| Payload | 0 – 0.5 kg, CoG offset ±30 mm | Descent tipping margin varies |
| Wheel torque scale | 0.85 – 1.15 | Actuator uncertainty |
| IMU noise σ | from measured Gazebo values | Keeps observations honest |
| **Initial yaw** | **±0.25 rad** | **Exactly `yaw_goal_tolerance`** |
| **Yaw gain** | **0.70 – 1.45** | **Covers the measured sim-to-sim steering-authority gap with margin** |

**Why the friction range is 0.35–0.70 and not 0.35–1.10** (Phase 2
aftermath, measured). **Gazebo cannot express terrain friction above
0.7.** Sweeping the ground plane's μ and measuring achieved yaw gives
69.63 / 69.30 / 69.59 % at |cmd| 1.00 for μ = 0.70 / 0.90 / 1.10 — flat
within run-to-run noise. That is saturation: the wheels are pinned at
`mu1 = mu2 = 0.7` in `coco_robo2.xacro`, which is frozen v1, so no ground
value above 0.7 reaches the robot.

The consequence for the old range was not cosmetic. The top third of
0.35–1.10 existed **only in the training simulator**. A policy would have
been rewarded for adapting to surfaces the evaluation simulator can never
present, and the sim-to-sim gap measured there was a *definition
mismatch* rather than an engine disagreement — the yaw ratio fell to
0.526, outside `YAW_GAIN_RANGE`, purely because MuJoCo was at an effective
1.1 while Gazebo was still at 0.7.

**A range both engines can express beats a wider one that is fictional in
the deployment simulator.** And 0.35–0.70 is still a **2× span**, which is
a real adaptation demand: one gain set tuned at 0.7 spins out at 0.35,
which is the failure Route B is built around. Across the narrowed range
the Gazebo/MuJoCo yaw ratio stays inside 0.70–1.45 at every measured
combination.

This is the exact mirror of the bug that made MuJoCo's `<pair>` elements
necessary. There, MuJoCo's element-wise **max** made terrain *below* the
wheels' 0.4 unreachable. Here, Gazebo makes terrain *above* the wheels'
0.7 unreachable. Both were invisible from inside one engine, and both made
a randomisation range partly fictional. **When a range is swept across two
simulators, measure that both can enter it before trusting either.**

The per-route ranges were re-derived rather than clipped: Route A's
0.7–1.1 lay entirely at or above the cap, so clipping would have collapsed
it to a single value. See `coco_sim/worlds/yard_params.yaml`.

**Why a yaw-gain term exists at all** (Phase 1.5, measured). After
calibration the MuJoCo base still delivers a different amount of yaw per
unit command than Gazebo does: the ratio runs **1.27× at small commands
down to 0.86× at full authority**, against 1.71×–1.22× before calibrating.
That residual is not noise and it is not going away — contact parameters
were a weak lever on it (sliding friction 0.2 → 1.5 moved yaw efficiency
only 59.5 % → 65.2 %), and Gazebo is not even self-consistent at the top
of the range, disagreeing with its own mirrored command by **1.36× at
2.5 rad**.

The range **0.70 – 1.45** brackets the measured 0.86–1.27 with roughly
20 % margin on each side, so a policy trained across it has seen worse
steering authority than either simulator actually delivers, in both
directions. Sample it per episode alongside the other terms.

That last row matters more than it looks. §6.1 found the +0.6 m drift was
*Nav2's legal heading error*, not the policy, and patched it downstream with
`lateral_hold`. Sampling initial yaw across the full Nav2 tolerance makes the
policy **learn to correct from a legal handoff error** — which is the honest
fix, one layer up from the patch.

---

### 2.6 Dynamic obstacles — apron only

**Status: spec only. Nothing here has been built or measured.**
Implementation lands in **Phase 6**, alongside route selection. Every
number below is derived from a named file; none is a measurement.

One or two moving actors, **confined to the flat apron**, off by default
behind `actors:=false`.

#### Why apron only, and nowhere else

Two independent reasons, either sufficient.

**There is nowhere to evade to on a route.** Route B is a **1.2 m** chute
and the deck bridge is **0.5 m** wide, against a robot **0.314 m** wide
whose measured worst-case cross-track is **0.301 m**. A moving obstacle in
either place does not test avoidance — it tests whether the robot falls
off. The interesting behaviour would be unreachable, and the failure would
be attributed to terrain.

**It would contaminate the Phase 3 ablation.** M7.2 exists to separate
route difficulty across B0/B1/B2 and the policy. An actor injects variance
that has nothing to do with terrain, and Phase 3 is precisely the phase
that must not have a second uncontrolled variable in it.

A third, specific to this repo: `ramp_env`'s observation carries **no scan
term**, so the RL policy is blind to obstacles. An actor anywhere the
policy drives would simply be driven through.

#### Form

| | | derivation |
|---|---|---|
| shape | upright cylinder, **Ø0.30 m × 0.60 m** | |
| Ø0.30 m | ≥4 lidar returns to **8.6 m** | the collision monitor's polygons require `min_points: 4`; the lidar's angular step is 4.1888/479 = **0.008745 rad**, so 4 returns needs 0.0349 rad ⇒ width ≥ 0.0349 × range. The apron's diagonal is 8.38 m, so the actor is detectable from anywhere on it. Below Ø0.25 m that range falls to 7.2 m |
| 0.60 m tall | straddles the scan plane | the lidar sits at 0.20 + 0.0135 = **0.2135 m**; 0.60 m also matches `cylinder_obstacle` exactly, so its scan signature and inflation behaviour are already characterised |
| colour | neutral grey, **not** saturated | `coco_perception` classifies by hue; the four targets are deliberately the arena's only saturated colours |
| dynamics | **kinematic** (driven pose, infinite effective mass) | the robot is 2.9715 kg with 13.5 mm clearance — a dynamic collision is a tipped robot, not a graceful degradation. Any contact scores the run as **failed** |

Actors must never appear in a `<xacro:magnet model="...">` macro: a
`DetachableJoint` binding one would silently stop the base turning, which
RESULTS.md already records costing an afternoon.

#### Speed — 0.25 m/s nominal, 0.20–0.30 m/s range, 0.35 m/s hard cap

Upper bounds, all derived:

- **Costmap fidelity.** `local_costmap` updates at 5 Hz, resolution 0.05 m.
  At 0.25 m/s the actor's mark is stale by exactly one cell; 0.35 m/s is
  where it becomes persistently more than one cell wrong.
- **DWB horizon validity.** `sim_time` 1.5 s against a *frozen* costmap
  snapshot. At 0.25 m/s the actor moves 0.375 m during a horizon only
  0.45 m long at `max_vel_x` 0.3 — faster and the rollout scores a world
  that no longer exists anywhere along it.
- **Monitor reaction band.** Head-on closing speed is 0.30 + 0.25 =
  0.55 m/s; the 0.15 m from `PolygonLimit` (0.55) to `PolygonSlow` (0.40)
  is crossed in 0.27 s ≈ 2.7 scans. At 0.6 m/s it is 0.17 s — under two
  scans and under the scan `source_timeout` of 0.2 s.

**And a lower bound, which is the counter-intuitive one.**
`SimpleProgressChecker` needs 0.1 m of motion within 10 s, and a
collision-monitor stop is exactly zero motion. A crossing actor blocks the
lane while |Δy| ≤ 0.15 + 0.20 + 0.50 = **0.85 m**, i.e. 1.70 m of actor
travel: 6.8 s at 0.25 m/s, 8.5 s at 0.20 m/s, but **11.3 s at 0.15 m/s —
past the allowance, so the leg fails into recovery behaviours.** Do not
spec an actor slower than 0.20 m/s on a crossing path unless testing
recovery is the point.

#### Motion pattern

**Default: linear shuttle** along a fixed segment with a **declared start
phase** — fixed start position and direction, not randomised. The reason
is measurement, not taste: RESULTS.md's ten-goal tour had legs ranging
11.1–123.5 s, so with a free-running actor whether the robot meets it at
all is decided by arrival phase, and run-to-run comparison becomes
meaningless.

**Second named scenario: robot-triggered crossing** — the actor holds until
the robot's x crosses a trigger, then crosses at a fixed offset. This
guarantees the encounter geometry instead of leaving it to chance.

Rejected: random walk / Poisson arrival. Most realistic, worst for this
repo — CLAUDE.md rule 1 requires every reported number to come from a run,
and a stochastic actor makes any single run uncomparable.

#### Expected response, to be checked against reality in Phase 6

Chain: `controller_server → /cmd_vel_nav → velocity_smoother →
/cmd_vel_smoothed → collision_monitor → /cmd_vel`, so every zone genuinely
gates the wheels. Expected: the actor marks the local costmap, DWB's
`BaseObstacle` critic steers around it, and `PolygonSlow` then
`PolygonStop` fire on approach. **Predicted, not observed** — and worth
noting that RESULTS.md already records `BaseObstacle.scale` measuring
*nothing* in this arena because the geometry saturated it, so the critic's
contribution here is genuinely unknown.

---

### 2.7 Phase 6 additions — spec only, nothing built

**Status: spec. Nothing here is implemented or measured.** These are
additive to an already large M7 and **may be cut or deferred**. If they
are cut, **items 1 and 2 are the ones to keep** — item 1 fixes a measured
mission failure and item 2 removes a dependency on ground truth that does
not exist off a simulator.

Priority order.

#### 1. EKF sensor fusion — `robot_localization`, wheel odometry + IMU

Motivated by a measured failure, not by a shopping list. **Run 15 of the
Phase 0.5 fetch matrix** lost the mission after a *successful* pick: AMCL
drifted **3.4 m** in the deliberately unmapped corridor at x > 5.5, DWB
then scored **0 of 819** trajectories, and `bt_navigator` aborted in
**1.7 s**. One run in twenty, and the pick had already worked.

The robot publishes wheel odometry and an IMU and fuses neither; AMCL is
the only thing correcting pose, and in an unmapped corridor it has nothing
to correct against. An EKF over wheel odometry + IMU gives a locally
consistent pose exactly where the map stops helping.

**Judge it on this metric, before and after:** the **descent-end AMCL
gap**, currently **0.119–1.183 m** across the 20-run matrix, with **every
run under 0.470 m getting home**. The threshold is a bracket, not a
boundary: nothing was sampled between 0.470 and 1.183 m. The EKF earns its
place if it moves the *distribution* down, not if it merely improves the
best case.

#### 2. A VLM task interface above the sequencer

Two concrete uses, both replacing something that is currently a cheat:

- **Open-vocabulary target selection.** Today `--colour` is a table lookup
  to a `lane_y`. A VLM turns "the fattest one" or "the one next to green"
  into a lane, which is the same interface with the table removed.
- **Grasp verification from the camera frame.** Today `check_lifted`
  reads the object's height out of gz. That is **ground truth, and it does
  not exist on hardware** — and it is precisely the input a retry
  behaviour would need. Verifying the grasp from the image makes the check
  honest and makes retry possible.

**This is a VLM, not a VLA, and the distinction is the embodiment, not
fashion.** The arm is 2-DOF planar with **no wrist**; the camera is
chassis-mounted and unpitched; the grasp is a magnet weld, not a closure;
and the approach holds a **5.5 mm** window. VLAs emit 6–7 DOF end-effector
deltas at centimetre precision. There is nothing here for an action head
to drive: four of its degrees of freedom do not exist, and its precision
is an order of magnitude coarser than the window that already works.
Language belongs **above** the sequencer, choosing goals. The servo below
it is already better than a learned policy would be at this.

#### 3. MPPI evaluated against DWB as the local planner

Already flagged in §11 as a Phase 6 decision; this makes it a
**deliverable with a comparison table** rather than a preference. Route B
is 1.2 m wide and the bridge 0.65 m, and DWB scores rollouts against a
frozen costmap snapshot — weakest exactly where clearance is tightest.
Same routes, same seeds, same success criteria.

#### 4. Residual RL — classical baseline plus a learned correction

A B2 baseline with a learned residual on top, giving the ablation
**B2 alone / B2 + residual / policy alone**. Nearly free once the Phase 3
baselines exist, and it is the cleanest available answer to "was the
policy necessary, or just sufficient?"

#### Two exclusions, recorded so they are not revisited by default

- **A VLA on Coco — no.** Wrong embodiment, per item 2's reasoning. This
  is not "later"; it is a mismatch that more compute does not fix.
- **SmolVLA fine-tuned on the ST3215 6-DOF arm — worth doing, but not
  here.** That arm has the degrees of freedom a VLA needs. It belongs in
  its own repo with its own results, not bolted onto a project whose
  claims are about a 2-DOF planar arm and a 5.5 mm grasp window.


## 3. Why RL — five claims, each with a falsifier

This section exists so that the project can be *wrong*. Each claim names the
classical baseline that would refute it.

| # | Claim | Refuted if | **Verdict (Phase 3, measured)** |
|---|---|---|---|
| 1 | Camber rejection needs adaptation | A single retuned PD holds ≤5 cm across camber 0–8° | **REFUTED** |
| 2 | Friction adaptation needs learning | One gain set succeeds ≥90% across the friction range | stands |
| 3 | Curb mounting needs a momentum strategy | A fixed rule mounts the step ≥90% | **REFUTED at 24 mm**; stands only at 60 mm |
| 4 | Washboard needs anticipatory throttle | Constant velocity crosses without resonance at all speeds | stands |
| 5 | Loaded descent needs payload-aware braking | A fixed descent profile never tips across the payload range | **not tested** |

Measured over 1,080 episodes, 120 per baseline per route, in Phase 3.
B2's gains were tuned on seeds disjoint from the evaluation set. Full
tables in `docs/RESULTS.md`.

#### Claim 1 — REFUTED. This is the important one.

Measured **on the ramp**, because camber exists only there; the
whole-traverse cross-track is dominated by the deck's commanded 1.95 m
lane change, which is a manoeuvre and not a camber-rejection error.

| camber | B1 (shipped gains, no retune) | **B2 (retuned)** |
|---|---|---|
| 0–2° | 3.32 cm | **1.39 cm** |
| 2–4° | 2.57 cm | **1.05 cm** |
| 4–6° | 5.49 cm | **1.23 cm** |
| 6–8° | 3.25 cm | **1.31 cm** |
| **all** | **3.79 cm** | **1.26 cm** (worst 6.66 cm) |

A retuned PD holds **1.26 cm mean, four times inside the 5 cm falsifier,
and shows no trend in camber at all**. It completes Route A **98 %** of
the time. Even the shipped gains, never retuned for this world, average
3.79 cm.

§2.2 called Route A's camber "the sharpest test in the world", on the
reasoning that `lateral_hold`'s gains were tuned at zero camber and would
undershoot or oscillate at 8°. **They do neither.** The gain table that
motivated the claim was measured on the v1 wedge, where the error changes
sign past 3.0/2.5; on the Yard's ramp a gain of 6.0 tracks flat across the
entire camber range.

**Consequence: camber alone is not evidence for learning.** A policy that
merely matches B2 on Route A has demonstrated nothing. Route A's
contribution to the M8 case is now the **deck convergence and the
bridge**, and B2's 98 % is the bar.

#### Claim 3 — restated against the height the world contains

The falsifier as originally written names the **60 mm** spec step, and at
that height it stands trivially: 60 mm needs an approach of 1.00 m/s,
**2.5× `MAX_LIN`**, so no fixed rule mounts it at any commandable speed.
That is the obstacle being outside the action space, not a momentum
strategy being subtle.

The world contains **24 mm**, and **at that height the claim is refuted**:
B2's fixed schedule mounts the curb at 0.50 throttle across Route C's
whole friction range. Route C's 15 % success is limited by tipping on the
rubble (101 of 120), not by the curb.

#### Claim 5 — not tested, and why

The Phase 3 task ends at the target bay. **The loaded descent is never
exercised**, so nothing measured bears on payload-aware braking. Recorded
as unmeasured rather than assumed either way. Testing it requires
extending the task past the bay and through the descent.

### 3.1 The baselines — build these *before* the policy

This is the discipline that separates a result from a claim, and it is cheap.

| ID | Baseline | Notes |
|---|---|---|
| **B0** | Open-loop constant throttle | Floor. Establishes that the terrain is non-trivial |
| **B1** | Current `lateral_hold` PD, single global gain set | The v1 controller, unchanged |
| **B2** | **Gain-scheduled PD**, retuned per route, given true grade + friction | **The strongest honest classical baseline.** Give it privileged information the policy does not get. If RL cannot beat a PD that *knows* the friction, that is worth knowing and worth reporting |
| B3 | MPC over a unicycle model with slip term | Optional. Only if time permits |
| **RL** | The policy | |

Report the full matrix: success rate and mean traverse time, per route, per
baseline, over ≥100 randomised episodes each. If RL loses to B2 on Route A,
**publish that**. This repo already reports `--target` re-targeting as 0/5
rather than omitting it; the standard is set.

---

## 4. Observation and action spaces

### 4.1 Observations

The v1 observation is proprioception only, which cannot support terrain
reasoning. Three groups:

| Group | Contents | Dim |
|---|---|---|
| Proprioception | base linear vel (3), angular vel (3), roll/pitch (2), wheel velocities (4) | 12 |
| History | last 10 frames of (pitch, roll, pitch rate, last action) | 40 |
| Exteroception | height scan, 11 × 7 grid at 0.10 m spacing, sampled ahead and around the base, expressed relative to base height | 77 |
| Goal | next waypoint (x, y) in base frame, heading error | 3 |
| | **total** | **132** |

The **history block is what makes the terrain inferable**. The robot cannot
see under its own wheels; 10 frames of pitch and roll response to a known
commanded action is a friction and roughness estimator. This is the same
mechanism legged-locomotion policies use, and it is the direct answer to
"where is the partial observability that justifies a learned policy?"

### 4.2 Actions

Keep v1's shape — `(linear, angular)` normalised to [-1, 1] — so `ramp_driver`
and `cmd_vel_arbiter` need no changes at all. Do not widen the action space in
M7. One variable at a time.

### 4.3 Reward

`reward.py` currently has **no lateral or heading term at all**, which is why
§6.7 correctly refused a retrain in v1. M7 replaces it wholesale:

| Term | Sign | Purpose |
|---|---|---|
| Progress along route centreline | + | Primary drive |
| Cross-track error | − | Replaces `lateral_hold`'s job with a learned equivalent |
| Heading error at waypoints | − | Clean handoff to `approach_server` |
| Roll/pitch magnitude beyond threshold | − | Tip avoidance, before the terminator fires |
| Wheel slip (commanded vs achieved velocity) | − | Discourages spinning out on Route B |
| Action rate | − | Smoothness; keeps commands physically realisable |
| Reached platform | + terminal | |
| Tipped / fell off / timed out | − terminal | |

**The cross-track term has a measured baseline to beat: 0.301 m.** That is
the worst cross-track from the target lane centreline over the 20-run M6
fetch matrix, under mission conditions — arriving on a real Nav2 leg, with
`lateral_hold` on and the shipped v1 policy driving (mean 0.120 m, 3 of 20
runs beyond a half-lane). It is the number this reward term exists to
improve on, and it is *not* the 0.053 m the v1 gain sweep reports: that
figure was measured from a teleported start at exactly ±0.25 rad and
measures displacement from where the climb began rather than distance from
the lane. Quoting 0.053 m here would set the bar four times too low. Both
figures, with their conditions, are in
[RESULTS.md](RESULTS.md#the-lane-holds-envelope-is-wider-than-0053-m-and-here-is-why).

Log every term separately. When a curriculum phase stalls, the per-term
breakdown is what tells you which one is dominating — and v1 already
demonstrated that three bugs can mask each other when you only watch the
scalar.

### 4.4 Teacher–student (stretch, M9)

Standard recipe: train a **teacher** with privileged observations (true
friction, true heightfield, true payload), then distil into a **student** that
sees only deployable inputs (depth camera → elevation map, IMU history).

This is genuinely strong on a CV and it is also the only version of the
exteroception block that transfers to hardware. Mark it a stretch — get the
teacher working and measured first.

---

## 5. MuJoCo — training architecture

### 5.1 Why, and how much

Gazebo trains at **8.7 steps/s**, and `--fast` is permanently unavailable
because unlocking RTF makes sim time outrun ROS delivery and the 0.5 s
`cmd_vel` watchdog pumps the wheels (531/533 episodes tipped). A randomised
heightfield world is *more* expensive than a wedge, so v2 in Gazebo would push
past 15 h per run on a laptop. That kills iteration, and iteration is the
entire point of adding randomisation.

MuJoCo headless with no ROS in the loop, 12 subprocess workers:
**target 2,000–6,000 steps/s** *(to be measured — do not quote until it is)*.
That turns an overnight run into a coffee break.

MJX (JAX/GPU) is the stretch option. On 6 GB VRAM it will be tight alongside
the heightfield, and CPU MuJoCo is very likely enough for a 4-wheel base.
**Do not start with MJX.** Get CPU MuJoCo measured first; only reach for MJX
if the measured throughput is the bottleneck.

### 5.1a MuJoCo convex-hulls every mesh, and it will not tell you

**Load-bearing for both world generators, so it is stated before either.**

> *"Meshes specified by the user can be non-convex, and are rendered as
> such. For collision purposes however they are replaced with their convex
> hulls."* — MuJoCo `doc/computation/index.rst`

Measured on this machine: a V-trough STL whose floor sits at **z = −0.400**
was loaded into MuJoCo and a 0.05 m probe sphere dropped on it. The probe
settled at **z = +0.0496** — resting on the hull lid, **450 mm above the
real surface**.

Every concave feature in the Yard is exactly this shape of geometry:
washboard troughs, rubble depressions, the bridge gap, the curb undercut.
Sharing one STL between the two simulators — the obvious approach, and the
one this repo's existing `gen_ramp.py` STL writer invites — would leave all
of them **present in Gazebo and absent in MuJoCo**.

**And the check most people would write would have passed.** Both
simulators would agree on the file, on its checksum, and on any height
sampled from the analytic function that generated it. A file-level or
analytic parity test is blind to this by construction. Only *physics*
sees it — which is why M7.1's parity test drops probes and compares where
they **settle**, not where a height function says the surface is.

**The rule this implies, for the Yard and anything after it:**

| | |
|---|---|
| MuJoCo collision geometry | `hfield` or primitives (`box`, `plane`, `cylinder`) **only** |
| shared STL between engines | **never**, for anything concave |
| a future mesh that must be concave | requires explicit convex decomposition into multiple geoms |
| parity verification | physics-based (probes settling), never file or height comparison alone |

### 5.2 The architectural rule

> **The training environment must never import `rclpy`.**

This is the whole design. Training is a pure Python/Gymnasium/MuJoCo loop with
no ROS, no `/clock`, no DDS, no watchdog. Every timing pathology that produced
the `--fast` disaster is *structurally absent* rather than avoided by
discipline.

ROS re-enters only at deployment, where it already works.

```
coco_config  ──────────────┐  single source of truth
 (robot.py, joint_limits)  │  wheelbase, radius, mass, limits
                           │
        ┌──────────────────┴──────────────────┐
        ▼                                     ▼
  coco_sim/mjcf                      gazebo_models/urdf
  (MuJoCo model, generated)          (existing xacro)
        │                                     │
        ▼                                     ▼
  coco_rl/mujoco_env.py               full_world_robo.launch.py
  Gymnasium, NO rclpy                 Gazebo + ROS 2 + sensors
        │                                     ▲
        ▼                                     │
   PPO training  ──────► policy.zip ──────────┘
   12 workers, headless        (SB3 format, unchanged —
                                ramp_driver loads it as today)
```

The MJCF must be **generated from `coco_config`**, not hand-authored. Two
hand-maintained robot models diverge within a week, and the divergence
presents as a mysterious transfer gap.

### 5.3 The sim-to-sim gap — name it, measure it, publish it

MuJoCo's soft-contact solver and Gazebo's DART contact are **not the same
physics**, and wheel-ground contact is exactly where they differ most. A
policy that scores 100% in MuJoCo will not score 100% in Gazebo.

That gap is not a failure. **It is a headline result** — a measured sim-to-sim
transfer number is a more sophisticated artefact than a single-simulator
success rate, and it is the honest precursor to sim-to-real.

Mitigations, in order:

1. Domain randomisation over friction, torque scale, mass, CoG (§2.5)
2. Match integrator timestep and control rate exactly across both simulators
3. Calibrate MuJoCo contact parameters against a measured Gazebo rollout —
   drive a fixed open-loop command sequence in both, minimise trajectory
   divergence over `solref` / `solimp` / friction
4. Report the transfer table below regardless of what it says

**Transfer is bought by making the policy insensitive to steering
authority, not by making the two engines agree.** Phase 1.5 calibrated the
contact parameters and got the worst yaw deviation from 1.71× to 1.27×,
and that is where it plateaus: friction is a weak lever on skid-steer
scrub, and Gazebo's own sign asymmetry reaches 1.36× at full authority, so
there is no parameter set that agrees with a reference that disagrees with
itself. The residual is therefore handled in §2.5 by randomising yaw gain
over 0.70–1.45 — a policy that has trained across worse steering authority
than either simulator delivers does not need them to match. Chasing the
last 27 % with unphysical contact values would buy a fit at one yaw rate
and a worse model everywhere else.

**The reference has a noise floor, and this table will inherit it.**
Measured in Phase 1.5: Gazebo's response to a commanded yaw is symmetric
to within 1.014× up to 1.0 rad, but **its own +/− disagreement reaches
1.174× at 1.5 rad and 1.361× at 2.5 rad** — wider than the 1.3×
calibration tolerance itself. Gazebo is not repeatable against itself for
aggressive turns, so a transfer gap measured there cannot be attributed to
the policy or to MuJoCo; below about 1 rad/s commanded it is clean.

The practical consequence for the Yard: **no route or reward should
require a sustained commanded yaw above ~1 rad/s.** Anything that does is
unmeasurable in the reference, and a transfer number taken there is noise
being reported as a result. Route geometry should be drivable with
corrections inside that band — which the lane-hold band (≤0.25 rad
commands) comfortably is.

| Route | MuJoCo success | Gazebo success | Gap |
|---|---|---|---|
| A | — | — | — |
| B | — | — | — |
| C | — | — | — |

---

## 6. Milestones

| ID | Deliverable | Done when |
|---|---|---|
| **M7.0** | MuJoCo model + throughput baseline | MJCF generated from `coco_config`; headless steps/s measured with 1, 4, 12 workers; open-loop rollout matches Gazebo within a stated tolerance |
| **M7.1** | The Yard, both simulators | Same world in MJCF and SDF, generated from one parameter file; visual and geometric parity checked |
| **M7.2** | **Baselines B0–B2** | Success matrix over ≥100 randomised episodes per route. **Before any policy training.** |
| **M8.0** | Single-route policies | One policy per route, each beating its own B2, measured in MuJoCo |
| **M8.1** | Unified policy + curriculum | One policy across all three routes and the full randomisation range |
| **M8.2** | **Sim-to-sim transfer** | Policy deployed unchanged into Gazebo; transfer table filled in |
| **M9.0** | Route selection | Nav2 chooses among A/B/C by cost; full mission end to end on the new world |
| **M9.1** | Teacher–student *(stretch)* | Student on deployable observations within a stated margin of teacher |
| **M10** | Write-up | Ablation table, transfer table, video, README rewrite |

M7.2 landing before M8.0 is not negotiable. Building the baselines after the
policy means unconsciously tuning them to lose.

---

## 7. What must not change

Carried forward from `DESIGN_DECISIONS.md` §6.7, plus new ones.

- **The v1 world stays, frozen, as `world_v1`.** The 10/10 at 18° and 24° is a
  published reproducible result. v2 is an *addition*, not a replacement, and
  the progression v1 → v2 reads better than a substitution anyway.
- **The target bay stays flat and unchanged.** Protects 16/16 perception and
  the M6 grasp window.
- **The action space stays `(linear, angular)`.** No changes to
  `cmd_vel_arbiter`, `ramp_driver`, or the mode machine.
- **The camera stays unpitched.** Settled in M5, asserted by two tests.
- **The arbiter stays the sole publisher to the controller.** A new velocity
  source in M7 would be a safety regression.
- **Never `--fast`.** Applies to Gazebo evaluation runs too, not just training.
- **A fresh simulator per Gazebo mission run.** The `DetachableJoint` binds
  once on first spawn and a second run welds nothing while reporting success.
- **Kill by process name.** `ros_clean.sh`, every time.

---

## 8. Honest scope

This is two to three months of part-time work alongside a job search. If the
schedule compresses, the ranking by interview value per hour:

1. **M7.2 baselines on the existing v1 wedge** — a day's work, and it
   retroactively strengthens the RL claim you already have
2. **M7.0 MuJoCo throughput** — the unlock everything else depends on
3. Route C alone (rubble + curb) with a policy and its baseline — the single
   most compelling "the policy found something I wouldn't have written" result
4. Everything else

A trimmed version that lands is worth more than a complete version that
doesn't. The v1 README's credibility comes from every number having a
reproduction command — preserve that property above feature count.

---

## 11. Planner naming, and an A*-vs-Dijkstra demonstration that is honest

Recorded because `SmacPlanner2D` does not read as "A\*" to anyone skimming
the config, and because the obvious way to demonstrate the difference turns
out not to exist.

### It is A\*, and the heuristic is NOT cost-aware

`nav2_smac_planner::SmacPlanner2D` is a genuine grid A\* — 8-connected
Moore neighbourhood, path recovered by back-tracing the node chain rather
than by NavFn's gradient descent over a potential field.

But the heuristic is **plain unweighted Euclidean distance**, and it knows
nothing about the costmap:

```cpp
// nav2_smac_planner/src/node_2d.cpp
float Node2D::getHeuristicCost(const Coordinates & node_coords,
                               const Coordinates & goal_coordinates)
{
  auto dx = goal_coordinates.x - node_coords.x;
  auto dy = goal_coordinates.y - node_coords.y;
  return std::sqrt(dx * dx + dy * dy);
}
```

Cost-awareness lives entirely in the **g**-cost, via
`cost_travel_multiplier` in `getTraversalCost`. The heuristic stays
admissible *because* it is not cost-aware. So "A\* with a cost-aware
heuristic" would be wrong — it is A\* with a cost-aware **traversal cost**
and a pure geometric heuristic. (The *Hybrid*-A\* planner's obstacle
heuristic **is** cost-aware and does expose `cost_penalty`; this repo does
not use it, and the two must not be conflated.)

### There is no heuristic-weight parameter. None.

The complete set `SmacPlanner2D` declares in Jazzy: `tolerance`,
`downsample_costmap`, `downsampling_factor`, `cost_travel_multiplier`,
`allow_unknown`, `max_iterations`, `max_on_approach_iterations`,
`terminal_checking_interval`, `use_final_approach_orientation`,
`max_planning_time`. That is all of them, and it matches the private
members in the installed header — there is no heuristic-weight member for
a parameter to bind to.

**So "set the heuristic weight to 0 and get Dijkstra" is not available**,
and writing it up that way would be checkably false to anyone who greps the
source. `cost_travel_multiplier: 0.0` is the parameter that looks like it
should do this and does not: it zeroes the *obstacle penalty in g*, giving
uniform-cost A\* with the Euclidean heuristic still fully active. Nav2's
own docs call that "a naive binary search A\*" — note **A\***, not
Dijkstra.

### The demonstration that is actually available

Two experiments, and neither is the textbook one.

#### (a) Cost-awareness on/off inside SmacPlanner2D

`cost_travel_multiplier` scales the cost term in the **g-cost**, not the
heuristic. Setting it to `0.0` gives uniform-cost A\* — Nav2's own docs
call this "a naive binary search A\*" — while the default keeps costmap
cost in the traversal expense. Same map, same goals, one parameter.

**Planning DURATION is the expansion proxy, and it is a proxy.** Neither
planner publishes node-expansion counts: there is no expansions topic and
no such field on the `ComputePathToPose` result, so the honest options are
patching Nav2 (out of scope) or measuring the thing expansions cause.
Duration is contaminated by costmap access patterns, cache behaviour and
whatever else shares the CPU, so it must be reported as *planning time*,
never relabelled as *nodes expanded*. `plan_compare.py` already measures
duration and path length.

#### (b) NavFn's A\* against NavFn's Dijkstra — the better story

`NavfnPlanner` exposes `use_astar` (default `false`), branching between
`calcNavFnAstar` and `calcNavFnDijkstra`. One implementation, heuristic as
the only difference, and NavFn is already registered here. Registering it
twice under two ids (`NavFnDijkstra` / `NavFnAstar`) lets `plan_compare.py`
hit both in one run.

This repo's config already records that **NavFn's A\* produces *worse*
paths than its own Dijkstra**. That is a property of *this implementation*,
not of A\*, and the mechanism is worth stating precisely because the
obvious explanation is wrong.

**Verified against the source** (`nav2_navfn_planner/src/navfn.cpp`,
jazzy):

1. The heuristic is admissible and is **not** stored in the field.
   `updateCellAstar` computes the potential, writes `potarr[n] = pot`, and
   only *then* adds the Euclidean distance-to-start for the purpose of
   choosing a priority bucket. `potarr` holds pure cost-to-goal in both
   modes.
2. **Both modes stop at the same place.** `propNavFnAstar` breaks on
   `potarr[startCell] < POT_HIGH`, and `propNavFnDijkstra` does the same
   whenever `atStart` is true — which `navfn_planner.cpp:272` passes. So
   the difference is *not* that A\* terminates early and Dijkstra fills
   the map. It is that A\* expands a **narrower corridor** to get there.
3. **The path is not read off the search tree.** `calcPath` walks the
   field by gradient descent (`gradCell`). At each step it inspects the
   cell and all eight neighbours, and if **any** of the nine is unvisited
   (`potarr >= POT_HIGH`) it abandons the interpolated gradient and takes
   a grid-locked 8-connected step to the lowest neighbour — the log line
   is literally "Pot fn boundary, following grid". `gradCell` also skips
   any neighbour still at `POT_HIGH` when forming the gradient, so the
   estimate goes one-sided near the frontier.

Put together: A\*'s entire purpose is to visit fewer cells, so in A\*
mode the unvisited frontier runs **close to the path**, and path steps
whose 3×3 neighbourhood touches it degrade to grid steps. Dijkstra
expands isotropically, so by the time it reaches the start the path is
padded with visited cells and smooth descent is available almost
everywhere.

**So: A\* is not worse than Dijkstra. NavFn recovers its path from a
neighbourhood of a potential field, and A\* deliberately does not fill the
neighbourhood.** Being able to measure that and explain it is worth more
than a clean textbook comparison — and it is a genuine distinction from
`SmacPlanner2D`, which back-traces the node chain and has no such coupling
between how much was explored and how good the path is.

### Local planner

Currently **DWB**. `nav2_mppi_controller` is available in Jazzy and is a
candidate swap for the Yard, where Route B is 1.2 m and the bridge 0.5 m —
DWB scores a rollout against a frozen costmap snapshot, which is weakest
exactly where clearance is tightest. **A Phase 6 decision, not now**, and
it would want its own measurement rather than a preference.
