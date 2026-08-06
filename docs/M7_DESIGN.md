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

**Route C's curb is the third, and the most interesting.** Mounting a 60 mm
step with 65 mm wheels requires a *momentum strategy* — back off, accelerate,
strike it with stored kinetic energy. It is discontinuous and non-obvious, and
a PD controller stalls against it indefinitely. This is the single clearest
"a policy found something I would not have written" result available.

### 2.3 The platform deck

No longer a plain slab. Three sections in series:

| Section | Geometry | What it tests |
|---|---|---|
| Washboard | sinusoid, 40 mm amplitude, 0.35 m wavelength, 1.2 m long | Pitch oscillation couples to wheel contact; needs anticipatory throttle. Drive it at the wrong speed and the chassis resonates |
| Narrow bridge | 0.5 m wide, 1.0 m long, gap either side | Catastrophic failure zone. A 2D lidar **cannot see a hole** — this is a negative obstacle, and it is why the policy must hold a line rather than merely arrive |
| Target bay | flat, 1.0 m, four lanes as in v1 | Unchanged. `approach_server` and `grasp_server` take over here exactly as they do today |

The bay staying flat and unchanged is deliberate: it protects the 16/16
perception result and the M6 grasp window from being invalidated by this work.

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
| Friction μ | 0.35 – 1.10 | The core adaptation demand |
| Heightfield seed | new each episode | Route C never repeats |
| Camber | 0 – 8° | Breaks fixed-gain lateral hold |
| Payload | 0 – 0.5 kg, CoG offset ±30 mm | Descent tipping margin varies |
| Wheel torque scale | 0.85 – 1.15 | Actuator uncertainty |
| IMU noise σ | from measured Gazebo values | Keeps observations honest |
| **Initial yaw** | **±0.25 rad** | **Exactly `yaw_goal_tolerance`** |

That last row matters more than it looks. §6.1 found the +0.6 m drift was
*Nav2's legal heading error*, not the policy, and patched it downstream with
`lateral_hold`. Sampling initial yaw across the full Nav2 tolerance makes the
policy **learn to correct from a legal handoff error** — which is the honest
fix, one layer up from the patch.

---

## 3. Why RL — five claims, each with a falsifier

This section exists so that the project can be *wrong*. Each claim names the
classical baseline that would refute it.

| # | Claim | Refuted if |
|---|---|---|
| 1 | Camber rejection needs adaptation | A single retuned PD holds ≤5 cm across camber 0–8° |
| 2 | Friction adaptation needs learning | One gain set succeeds ≥90% across μ 0.35–1.10 |
| 3 | Curb mounting needs a momentum strategy | A fixed-throttle or bang-bang rule mounts the 60 mm step ≥90% |
| 4 | Washboard needs anticipatory throttle | Constant velocity crosses without resonance at all speeds |
| 5 | Loaded descent needs payload-aware braking | A fixed descent profile never tips across the payload range |

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
