# ROADMAP

Long-term milestone tracking. **No session history here** — that is
`docs/SESSION_LOG.md`. Current snapshot is `PROJECT_STATE.md`.

---

## Numbering

Two schemes exist and they collide. **`C2-` prefixes the COCO 2.0 plan.**
Bare `M0`–`M7` are the historical milestones.

---

## Track 1 — v1, the wedge world (M0–M6). CLOSED

| ID | Objective | Status | Measured result |
|---|---|---|---|
| M0 | Magnet grasp via `DetachableJoint` | DONE | 5/14 then superseded; **zero empty grasps** |
| M1 | `cmd_vel_arbiter`, sole publisher to the wheels | DONE | Publisher count on controller topic = 1; teleop preempts mid-nav |
| M2 | World + SLAM map rebuild | DONE | Map 254x199 @0.05 m, 74.3 m² free |
| M3 | A* global planner (`SmacPlanner2D`) | DONE | 3.165 m / 5.5 ms / 62 poses vs NavFn 3.373 / 5.6 / 134 |
| M4 | Traverse: nav → RL climb → descent → nav home | DONE | Home within 0.04 m |
| M5 | RGB-D perception, colour target selection | DONE | **16/16 detected, all within ±2 mm** |
| M6 | Full fetch (approach, grasp, carry, place) | **DONE** | **19/20** fetch matrix; approach holds a **5.5 mm** window 20/20 (sd 0.6 mm); magnet held 20/20 |

**Baseline: M6's 19/20 on the frozen `world_v1`.** The single failure was
run 15 — AMCL drifted 3.4 m in the deliberately unmapped corridor *after*
a successful pick. That is a localisation failure, not a grasp one.

---

## Track 2 — v2, "The Yard" (M7). Phases 1–3 done, Phase 4 GATED

Randomised multi-route terrain where learning is genuinely required,
headless MuJoCo for throughput, and classical baselines capable of
proving a policy unnecessary. Spec: `docs/M7_DESIGN.md`.

| Phase | Objective | Status | Measured result |
|---|---|---|---|
| 1 | MuJoCo throughput + sim-to-sim fidelity | DONE | **3,712 steps/s at 8 workers = 427x** real time |
| 1.5 | Contact calibration | DONE | Worst yaw deviation **1.2696x** over 7 commands, inside the 1.3x target |
| 2 | The Yard in both simulators | DONE | Cross-engine parity **0.242 mm** worst, **0.138 mm** geometric |
| 3 | Classical baselines B0/B1/B2 | DONE | B2: **A 98% / B 3% / C 15%**, 1,080 episodes |
| 4 | Policy training | **GATED** | Blocked on 3 decisions — see below |

### What Phase 3 settled

- **Claim 1 (camber needs adaptation): REFUTED.** A retuned PD holds
  **1.26 cm mean / 6.66 cm worst** across camber 0–8°, four times inside
  the 5 cm falsifier, **with no trend in camber**. Route A's contribution
  to any RL argument is now the deck convergence and the bridge, and
  **98% is the bar**.
- **Claim 3 (curb): REFUTED at the built 24 mm.** Stands only at the
  60 mm spec step, and there only because 60 mm needs 2.5x `MAX_LIN`.
- **Claims 2 (friction) and 4 (washboard): STAND.**
- **Claim 5 (loaded descent): NOT TESTED** — the Phase 3 task ends at the bay.

### The three decisions gating Phase 4

1. **Deck convergence geometry** — 1.95 m lateral shift in 1.80 m of
   travel against a 0.40 m turn radius. Nothing changed.
2. **Route B viability** — **39.3% of episodes physically unclimbable**
   (mu < tan(grade)). Four options costed, none chosen.
3. **Route C tip terminator** — `TIP_LIMIT` is absolute and fires **34°
   short** of the true 54.5° static rear-over. **Instrumentation, not
   control.** Fix not applied because `TIP_LIMIT` is shared with
   `ramp_env`, the v1 curriculum and the shipped policy.

---

## Track 3 — COCO 2.0 (C2-M1 … C2-M9). ACTIVE

Goal: a technically rigorous, recruiter-facing autonomous mobile
manipulation system. Positioning is **"ROS 2 autonomous mobile
manipulation"**, never "RL robot" or "AI robot".

### C2-M1 — Visualization and observability — **COMPLETE**

- **Objective:** make the navigation state visually obvious; a mission
  HUD a viewer can read without the source.
- **Dependencies:** none.
- **Completion criteria:** RViz shows map, both costmaps, plans,
  localization, goal, targets; a HUD renders real state; **every display
  verified against a topic that actually publishes in a live run**.
- **Measured result:** all criteria met. 14 displays; RViz loads with
  **0** plugin/type/QoS errors and creates 3 occupancy grids. Full fetch
  completed end to end with the changes in place (home to **0.06 m**).
  30 new tests. 3 defects found and fixed that were invisible from
  reading code. Full table in `RESULTS.md`, "M1 observability".
- **Remaining:** the rendered window has not been visually inspected;
  the optional overlay plugin is not installed, so that code path has
  never executed.

### C2-M2 — Terrain control experiment — **CURRENT, NOT STARTED**

- **Objective:** finish the terrain-control research **before** adding
  any RL. Tip-termination correction, classical baseline re-evaluation,
  grade estimator, friction estimator, observer-driven controller.
- **Dependencies:** M7 Phase 3 (done). **Its first item is the same
  Route C tip-terminator decision Phase 4 is gated on.**
- **Completion criteria:** measured comparison of
  **A** fixed controller / **B** privileged controller with true
  grade+friction / **C** deployable controller using *estimated*
  grade+friction, reporting grade error, friction error, convergence
  time, steady-state error, cross-track error, climb success, failure
  mode, mission impact.
- **Decision rule, fixed in advance:** expand RL **only if** the
  observer-driven controller stays **>10 percentage points below** the
  privileged controller on a measured task. If the observer closes the
  gap, **that is the successful result** and RL is not added.
- **Measured result:** none yet.
- **Blocker to clear first:** `ROBOT PITCH` read `-0.314 rad` during the
  platform approach where the robot should be flat. Diagnose before
  building a grade estimator on that field.

### C2-M3 — Real mission executive — not started

- **Objective:** turn `traverse_demo.py` (a blocking script) into an
  explicit state machine with entry condition, action, success
  condition, timeout, failure condition, diagnostics and recovery
  **per state**.
- **Dependencies:** C2-M2 (so terrain states are real).
- **Completion criteria:** states are ROS actions/services/events, not
  one monolithic blocking script; the executive knows which subsystem
  owns the robot at each stage; the existing arbiter architecture is
  preserved.
- **Note:** `/mission/state` already exists (C2-M1) but only reports a
  blocking script's step label. **That is a stepping stone, not a
  substitute** — no state has an entry condition, timeout or recovery.

### C2-M4 — Perception-driven manipulation — not started

- **Objective:** replace the single hard-coded grasp coordinate with
  detection → depth → 3D position → TF → candidate grasps → IK →
  collision check → ranking → approach → grasp → verification.
- **Dependencies:** C2-M3.
- **Completion criteria:** the system distinguishes perception failure,
  target unreachable, IK failure, collision-planning failure, approach
  failure, grasp failure and placement failure. Measures target
  localization error, reachable-target %, planning success, grasp
  success, placement success.
- **Constraint:** the grasp window is **5.5 mm and colour-independent**;
  `GRASP_SELF_COLLISION_X = 0.150` is the binding bound, not the
  target's radius. Do not replace deterministic components with neural
  ones without a measured reason.

### C2-M5 — Localization health and recovery — not started

- **Objective:** detect unsafe localization and recover.
- **Dependencies:** C2-M3.
- **Completion criteria:** stop safely, block the mission, execute a
  recovery, relocalize, validate, resume or abort. Measures failure
  rate, detection latency, recovery success rate, recovery time,
  mission completion after recovery.
- **Note:** C2-M1 deliberately **withheld** a GOOD/DEGRADED verdict in
  the HUD because that threshold has never been calibrated against a
  known-bad run. **C2-M5 is where it gets measured.** M6's run-15 AMCL
  drift is the natural benchmark.

### C2-M6 — Dynamic obstacle — not started

- **Objective:** a controlled moving obstacle handled by Nav2 replanning.
- **Dependencies:** C2-M3.
- **Completion criteria:** measures collision rate, minimum clearance,
  replanning latency, number of replans, path-length increase, time
  increase, mission success. Deterministic enough to reproduce.

### C2-M7 — Robot health / diagnostics — not started

- **Objective:** a system-health layer over controller heartbeat, command
  and sensor freshness, map, TF, localization, nav state, manipulation
  state, mission state.
- **Dependencies:** C2-M5.
- **Completion criteria:** a readable status the robot can reason about.
- **Note:** `mission_hud`'s staleness tracking is the seed of this.

### C2-M8 — Standardized benchmark — not started

- **Objective:** a reproducible evaluation suite: nominal, initial
  localization error, target variation, reduced friction, sensor
  degradation, dynamic obstacle, failed grasp, combined disturbances.
- **Dependencies:** C2-M4, C2-M5, C2-M6.
- **Completion criteria:** fixed seeds, repeated trials, a final
  benchmark table. **No cherry-picking.**

### C2-M9 — Visually polished demonstration + 60–90 s video — not started

- **Dependencies:** all of the above.
- **Completion criteria:** clean dark technical UI, Gazebo + RViz split
  view, state overlays, real metrics only. Every displayed metric
  corresponds to real data.
- **Needs:** `sudo apt install ros-jazzy-rviz-2d-overlay-plugins`.

---

## Cross-cutting rules

- Never fabricate a measurement. Anything not run is **"not yet measured"**.
- No success claim without an explicit success condition; no grasp
  success without physical/ground-truth verification; no controller
  improvement without a controlled comparison.
- Failures are preserved and explained, never rewritten.
- Never `--fast`. Fresh simulator per mission run. Kill by process name.
- Anything added to a launch file must be added to `ros_clean.sh`.
