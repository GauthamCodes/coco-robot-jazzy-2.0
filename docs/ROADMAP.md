# ROADMAP

Long-term milestone tracking. **No session history here** — that is
`docs/SESSION_LOG.md`. Current snapshot is `PROJECT_STATE.md`.

> **This roadmap is closed.** COCO 2.0 is frozen at the release described
> in `PROJECT_STATE.md`. Everything below marked DONE was built and
> measured. **C2-M6 through C2-M9 were scoped and deliberately not
> undertaken** — they are kept as a record of what was designed and
> costed, not as pending work. Nothing here is a commitment.

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
| 4 | Policy training | **GATED** | Blocked on 2 decisions (was 3; the tip terminator is closed) — see below. **And C2-M2 measured that RL is not justified by the observer gap**, which changes what Phase 4 would be *for* |

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

### The three decisions gating Phase 4 — one is now closed

1. **Deck convergence geometry** — 1.95 m lateral shift in 1.80 m of
   travel against a 0.40 m turn radius. **Partly contradicted by
   C2-M2.1:** B1 and B3 still fall off 93 times in 120, but **B2 falls
   off zero times** using only terrain-aware throttle. A pure geometry
   problem does not yield to terrain information, so the premise that
   this is *only* geometry no longer holds. Still not chosen.
2. **Route B viability** — **39.3% of episodes physically unclimbable**
   (mu < tan(grade)). Four options costed, none chosen. C2-M2.1 flagged
   rather than dropped them and reports `ascent|climbable` beside the
   raw rate (51–54 % against 32–34 %).
3. ~~**Route C tip terminator**~~ — **CLOSED by C2-M2.0.** Made
   **surface-relative** in `yard_env` only, with 0.6 rad kept exactly and
   a 54.5° absolute backstop; the other three `TIP_LIMIT` homes stay
   absolute and a test asserts the split. **Do not unify them.**
   C2-M2.1 measured the consequence: the tip population did **not**
   shrink (B1 106, B3 116 vs Phase 3's 101). What changed is that it now
   fires at a genuine rear-over rather than 34° short of one.

---

## Track 3 — COCO 2.0 (C2-M1 … C2-M9). CLOSED AT C2-M5

Goal: a technically rigorous, recruiter-facing autonomous mobile
manipulation system. Positioning is **"ROS 2 autonomous mobile
manipulation"**, never "RL robot" or "AI robot".

**C2-M1 through C2-M5 are complete and measured. C2-M6 through C2-M9 were
scoped and not undertaken**; the release is frozen at C2-M5.

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
- **Remaining:** the optional overlay plugin is not installed, so that
  code path has never executed. (The rendered window *has* now been
  inspected — see C2-M1.5.)

### C2-M1.5 — Runtime integrity gate — **COMPLETE**

Inserted, not planned. A gate rather than a milestone: C2-M2's first
deliverable is a grade estimator, and C2-M1 had left the field it would
be built on undiagnosed. The rule was diagnose first, and change only
what a diagnosis proves.

- **Objective:** establish that the signals C2-M2 needs are trustworthy.
- **Dependencies:** C2-M1.
- **Completion criteria:** pitch source, semantics, frame, sign
  convention and staleness contract all known; the failed fetch's first
  divergence identified or the hypotheses explicitly bounded;
  `/approach/target`'s communication semantics settled; RViz actually
  looked at; no speculative control tuning.
- **Measured result:** all met.
  - **`ROBOT PITCH` was a stale field inside a punctual topic.**
    `ramp_driver` writes `self.pitch` only inside its climb and descend
    loops; the 5 Hz status timer republished the last value forever.
    Peak error **0.314 rad**, held across the whole pick; the field
    changed 21 times in 1,899 samples against `/imu`'s 144. Because the
    climb ends `GOAL_MARGIN` short of the crest the stale value is always
    ≈ the terrain grade, so **a grade estimator built on it would have
    passed every ramp test and then reported 18° on flat ground.** Fixed
    at both ends.
  - **The failed fetch was two independent failures.** First divergence
    inside the RL climb (cross-track − disp = 14 mm: Nav2 delivered
    on-lane); `found=0` logged 3.0 s later is a **consequence**; and the
    step that actually ended the run, nav home, reproduced on a run with
    a clean climb and a successful pick.
  - **`/approach/target` is correct as it stands.** No change.
  - **RViz inspected**, one objective defect (robot leaves the viewport)
    found and fixed by measurement.
- **Tests:** 404 → **414**, 0 failing.
- **Verdict: C2-M2 is READY.**

### C2-M1.6 — RViz navigation visualization — **COMPLETE**

Inserted, not planned, and narrow on purpose. C2-M1.5 looked at the
rendered window for the first time and reported it functional but
cluttered. That left an ambiguity worth resolving before anyone acted on
it: a bad map and a busy overlay look the same on screen.

- **Objective:** decide whether the occupancy map is poor or the
  presentation is merely cluttered, then fix only the second.
- **Dependencies:** C2-M1.5.
- **Completion criteria:** raw `/map` inspected separately from the
  costmaps; map quality classified explicitly; no speculative SLAM
  change; a clean `mission.rviz` and a still-useful `mission_debug.rviz`;
  robot visible, plans readable, goal obvious, costmaps not overwhelming;
  the rendered windows actually inspected; no navigation or control
  behaviour changed; tests green.
- **Measured result:** all met.
  - **The map is GOOD, and that is a measurement.** Five free-standing
    objects in `coco_world.world` located independently in the map agree
    on a **single rigid offset (+2.0560, +0.0150) m**, worst residual
    **25 mm — half a cell**. Drift and a bad loop closure cannot produce
    that; they make landmarks disagree and duplicate structure. 156 of
    186 occupied components are ≤ 2 cells, and the eight largest are
    every structure that exists. The ramp reads short by 0.575 m and
    0.625 m at its two feet, implying a scan plane at 186.8 and
    203.1 mm — symmetric, and matching `LIDAR_MOUNT_XYZ` z = 0.200.
    **No SLAM change was made.** Reproduce with
    `python3 docs/data/map_audit.py`.
  - **Recorded, not fixed:** the north and south walls have 0.55 m and
    0.85 m gaps in the far east corners the mapping drive never entered.
    They beat the robot's 0.297 m footprint but open onto *unknown*
    cells, and `track_unknown_space: true` with `allow_unknown: false`
    means no plan can route through them. Unobserved, not distorted.
  - **The clutter was the global costmap**, which spans the whole arena
    by construction and covered the map it is computed from with its
    inscribed-cyan and lethal-magenta bands. Split into two configs
    rather than compromising one; **neither drops a topic**.
  - **Framing measured, not guessed.** Distance 13 / pitch 1.45 /
    yaw 3π/2 draws the map at **949 × 652 px** with margins
    135/136/90/64 — **36% larger linearly** than the preserved C2-M1.5
    camera's 700 px, both still fitting the whole map.
  - **Two defects only looking could find:** the robot lost the frame to
    its own local costmap, and the laser was invisible against white
    free space. Plus one config comment disproved by measurement — the
    camera pane costs **zero** render width and 304 px of display tree.
- **Tests:** 414 → **435**, 0 failing.
- **Explicitly not changed:** SLAM, Nav2, planner, controller, AMCL,
  costmap runtime behaviour, robot model, terrain, PPO, perception,
  mission sequencing, action spaces.
- **Verdict: C2-M2 unaffected and still READY.**

### C2-M2 — Terrain control experiment — **COMPLETE**

Two sessions: C2-M2.0 built and froze, C2-M2.1 measured and decided.

- **Objective:** finish the terrain-control research **before** adding
  any RL.
- **Dependencies:** M7 Phase 3 (done).
- **Completion criteria:** measured comparison of a fixed controller, a
  privileged controller with true grade+friction, and a deployable
  controller using *estimated* terrain state, reporting grade error,
  convergence, cross-track, climb success and failure mode. **All met.**
- **Decision rule, fixed in advance:** expand RL **only if** the
  observer-driven controller stays **>10 percentage points below** the
  privileged controller on a measured task.
- **Measured result — 1,440 episodes, all accounted for:**
  - **Grade is observable.** MAE **0.057° / 0.253° / 2.681°** on routes
    A / B / C; convergence **0.94 / 2.73 / 10.10 s**. Route C's rubble is
    where body pitch stops representing the surface, with a tail to 20°.
  - **Friction is NOT identifiable**, and this is the phase's substantive
    physics result. A steady climb is in equilibrium, so the traction
    ratio is pinned at `tan(grade)` whatever μ is, and the drivetrain
    cannot saturate the contact on the flat. Measured:
    **τ − tan(grade) = −0.0012 / −0.0034 / +0.0043** over 1,440 episodes,
    and **0.3248 vs tan(18°) = 0.3249** live in Gazebo. What is reported
    is a **traction-demand ratio**, never a friction coefficient.
  - **The rule, applied unchanged** (task `ascent`): gaps **+0.0 pp**
    (A), **+1.7 pp** (B), **+7.5 pp** (C). **RL justified on 0 of 3
    routes — additional learned control is NOT justified.**
- **The verdict must be read with its caveat.** B3 ≈ B2 on ascent is a
  statement about the **task**, not the estimator. On Route A the
  observer recovered **nothing** — B3 fell back on 120 of 120 episodes
  and is byte-identical to B1, because tan(12°) = 0.213 sits below the
  0.35 a-priori friction floor — while **B2 completed 97.5 % against
  B3's 0.0 %**. Ascent does not discriminate there. See UNRESOLVED
  QUESTIONS 0 in `PROJECT_STATE.md`.
- **Where the observer costs something:** Route C, where B3 ascends
  **58.3 %** against B1's **84.2 %** — worse than the baseline it falls
  back to.
- **The live gate found three defects** in `terrain_observer_node` that
  no pure-core test could see, including one that made the observer
  withdraw its own estimate on **431 of 431** samples. Run
  `docs/data/c2m2_live_gate.py` whenever that node is touched.
- **The standing warning, now discharged and worth keeping:** body pitch
  is not terrain grade. They coincide on the v1 wedge only because the
  robot is quasi-static on a uniform rigid face — and Route C is exactly
  where that stops being true, which is where both the estimator and B3
  degrade.
- **Evidence:** `RESULTS.md` "C2-M2.1 the terrain benchmark",
  `docs/data/c2m2_benchmark.json`, four figures under `docs/images/`.

### C2-M3 — Real mission executive

#### C2-M3.0 — the executive itself — **COMPLETE**

- **Objective:** turn `traverse_demo.py` (a blocking script) into an
  explicit state machine with entry condition, action, success
  condition, timeout, failure condition, diagnostics and recovery
  **per state**.
- **Dependencies:** C2-M2 — satisfied.
- **Completion criteria:** states are ROS actions/services/events, not
  one monolithic blocking script; the executive knows which subsystem
  owns the robot at each stage; the existing arbiter architecture is
  preserved. **All met.**
- **Measured result.** `coco_mission/scripts/mission_states.py` (pure,
  no `rclpy`) plus `mission_executive.py` (the ROS adapter). 18 states,
  a contract table, ~40 structured failure reasons, bounded retries,
  `RECOVERY` and `ABORT`. **One full fetch completed live**: all 15
  nominal transitions in order, `result=fetch`, **0 recoveries, 0
  retries**, 175.8 s, **home to 7 mm**, and
  `/diff_drive_controller/cmd_vel` publisher count **1 before and after**.
  Tests **490 → 589**. Full table in `RESULTS.md`, "C2-M3.0".
- **Two defects the live runs found**, both recorded in `RESULTS.md` and
  `DESIGN_DECISIONS.md`: a launch argument named `autostart` leaked into
  `nav2_bringup` and left every Nav2 lifecycle node `unconfigured` with
  `/amcl_pose` at 0 publishers; and the `ALIGN_FOR_CLIMB` heading gate
  was calibrated against Nav2's own tolerance, which is judged against
  the AMCL pose rather than ground truth, so it aborted a mission that
  completes. **The heading is now reported and not gated** — the same
  treatment C2-M1 gave the HUD's localization verdict.
- **The invariant survived:** `cmd_vel_arbiter` is still the **sole**
  publisher to the controller topic, measured live before and after the
  mission, and three tests assert the executive adds none — one of them
  by asserting the string `Twist` appears nowhere in the package.
- **`traverse_demo.py` is unchanged and kept.** It is the harness the
  M4/M5/M6 numbers were measured with. `executive:=false` selects it.

#### C2-M3.1 — live failure injection and recovery validation — **COMPLETE**

- **Objective:** exercise the failure paths on the robot, not only in
  the harness, and decide whether `RECOVERY` needs behaviours beyond
  stopping.
- **Dependencies:** C2-M3.0 — satisfied.
- **Completion criteria:** `OPERATOR_ABORT`, `skip_grasp`, at least one
  worker-outcome failure and at least one timeout observed live; a
  decision on whether `ALIGN_FOR_CLIMB`'s heading gate can be
  calibrated, and if so against what. **All four observation criteria
  met. The heading-gate decision was NOT taken** — see below.
- **Measured result: no defect found, and no source changed.** Five
  live missions, four deliberately broken, fresh simulator each, never
  `--fast`. Every run followed its contract exactly.
  `mission_states.py` and `mission_executive.py` are **byte-identical
  to C2-M3.0**.

  | Scenario | Trigger | Retries | Final |
  |---|---|---|---|
  | Operator abort during `CLIMB` | `/mission/abort` on a moving robot | **0** | `ABORT` `OPERATOR_ABORT` (x3) |
  | Navigation failure | `--lane 5.0`, goal off the map | **2** | `ABORT` `NAVIGATION_FAILED` |
  | Perception failure | `target_blue` removed from the sim | **2** | `ABORT` `TARGET_NOT_FOUND` |
  | Manipulation failure | cylinder removed at `GRASP` entry | **2** | `ABORT` `GRASP_FAILED` |

  All four routes into `RECOVERY` — operator request, navigation action
  status, state timeout, worker terminal outcome — now have a live run,
  and both escalations (`ESCALATE_ABORT`, `ESCALATE_SKIP_GRASP`) were
  reached. Retry counts are exact, read from the executive's own
  `attempts={...}` line. Full table in `RESULTS.md`, "C2-M3.1".
- **The abort, three times.** Last nonzero controller command
  **+20 / +30 ms** after the service call, then **10 explicit zero
  commands over 0.88 s** (`ZERO_HOLD_SECONDS = 1.0`) — a commanded stop,
  not a watchdog coast. Travel after the abort **13.1 / 15.3 / 23.6 mm**.
  `max |vx| = 0.0` afterwards across 50, 264 and 482 samples.
- **No accidental COMPLETE**, measured: runs 3 and 4 descended and drove
  home (**120 mm**, **63 mm**) and still ended `ABORT` with the original
  reason.
- **Arbiter invariant: 1,134 publisher-count samples across five runs,
  every one of them 1.** **0** states entered after `ABORT` in 5 of 5.
- **What this does NOT claim.** Four *representative* branches ran live.
  `CLOCK_STALLED`, `--no-grasp` through the executive,
  `NAVIGATION_REJECTED`, `NAVIGATION_UNAVAILABLE`,
  `SERVICE_UNAVAILABLE`, `SERVICE_REFUSED`, `RECOVERY_TIMEOUT`, every
  `ALIGN_*`, `CLIMB_TIPPED`, every `DESCENT_*`, `RETURN_*`, `STOW_*`,
  `APPROACH_*`, `PLACE_*` and `VERIFY_PLACEMENT` did **not** run and
  remain unit-tested only. The **no-stale-completion** invariant was
  never provoked and is still argued from the code rather than measured.
  The accurate sentence is *"live validation completed for operator
  abort, navigation failure, perception failure and grasp retry."*
- **`RECOVERY` gained no new behaviours, deliberately.** Stopping was
  measured to be sufficient in every branch that ran: the arbiter
  reached `active=none` within 158 ms worst case and the robot was
  below 2 mm/s within 436 ms worst case, in every run. Adding a
  behaviour nothing had asked for would have been a change without
  evidence.
- **Carried forward, not closed: `ALIGN_FOR_CLIMB`'s heading gate.**
  C2-M3.1 produced no climb that failed for a heading reason, so there
  was nothing to calibrate a threshold against. The gate stays off and
  the number stays reported. This moves to **C2-M5**, where localization
  quality is the subject and the AMCL-versus-ground-truth gap that
  causes it is measured directly.
- **One instrumentation trap, recorded in `CLAUDE.md`.**
  `/diff_drive_controller/cmd_vel` carries both `Twist` and
  `TwistStamped`; the arbiter publishes the second, and a `Twist`
  subscriber captures nothing while `ros2 topic info` still reads
  healthy. The empty capture looked exactly like the result being
  sought. Cost one run.
- **Tests: 589 passing / 0 failing, unchanged.** No test was added or
  modified — the paths these runs exercised were already asserted in the
  pure harness by C2-M3.0, and the live runs agree with them.

### C2-M4 — Perception-driven manipulation — **COMPLETE**

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

#### C2-M4.0 — perception → 3D pose → TF → reachability — **COMPLETE**

Commit `16e952f` on `coco2-m1-observability`, pushed.

- **Built:** `coco_perception/target_pose.py` (pure, no `rclpy`) and
  `target_pose_node.py` (thin, `tf2`), beside an unchanged
  `target_finder.py`. New topics `/perception/target_pose`
  (`vision_msgs/Detection3DArray`), `/perception/grasp_point`
  (`PoseStamped`), `/perception/target_pose/status`.
- **Measured, live, fresh simulator, never `--fast`:** four colours ×
  five stand-offs, 20 placements, **240 of 240 frames detected**.
  Horizontal error **1.1 / 1.6 / 2.1 mm** (min/median/max) over the
  0.35–0.90 m stand-offs, **colour-independent to within 0.8 mm**.
  Frame-to-frame spread **0.0000 m**, so the residual is bias, not
  noise. The estimate tracks a moving target: 70.1 mm measured against
  70 mm commanded.
- **One defect, diagnosed and NOT fixed:** `min_range` gates an extended
  object by its *near face*, which is a radius closer than its axis. At
  a 0.28 m stand-off `dx` ran **+4.1 to +8.3 mm, proportional to
  radius**; the identical placements at `min_range:=0.11` gave −1.0 to
  −1.4 mm. Left at 0.15 to match `target_finder` and because the
  operating envelope starts near 0.30 m. **C2-M4.1's call.**
- **Not done:** no grasp, no driven approach, on-lane only.

#### C2-M4.1 — four-colour benchmark + grasp integration — **COMPLETE**

Commit `33028ed` on `coco2-m1-observability`, pushed. **C2-M4 is
closed.**

- **Built:** `target_pose_node` gained `point_topic`, empty by default.
  Set to `/perception/target` it stands where `target_finder` stood and
  the whole downstream chain — servo, align, creep, `/approach/target`,
  `check_target_pose`, `arm_ik`, MoveIt, the magnet — runs unmodified.
  That is the entire integration; `approach_server`, `grasp_server`,
  `arm_ik` and `arm_control` are **byte-identical**. Plus two
  instruments, `docs/data/c2m4_grasp.py` and `docs/data/c2m4_analysis.py`.
- **Measured, perception:** the frozen 60-placement grid ran unmodified.
  **60 of 60 placements, 720 of 720 frames detected, 0 wrong-colour
  selections.** Horizontal error **0.7 / 1.4 / 2.4 mm** (min/median/max),
  colour-independent to within 0.47 mm of median, frame-to-frame spread
  **0.0000 m** throughout.
- **Measured, manipulation:** 8 live runs, **one fresh simulator each**,
  never `--fast`. **Grasp physically verified 8 of 8** — the object's own
  height read from gz, not an action result — placement 7 of 8, and
  every fix inside the 5.5 mm window (0.15341–0.15471).
- **The result, and it inverts the premise this block was written with:**
  the static reachability verdict is a **lower bound, not a forecast**.
  This block used to say "the approach drives straight forward, so it
  fixes x and leaves y alone". That is
  `reachability_after_approach`'s model and **not** what
  `approach_server` does — its `align` phase pivots until the bearing is
  nulled and only then takes the fix. Measured: a **+0.030** placement
  reached the grasp as **−3.0 mm** and a **−0.010** placement as
  **+1.68 mm**, and **both grasped successfully** despite both being
  judged `OFF_ARM_PLANE`. The verdict credits the approach with
  translation and not rotation, so it under-predicts feasibility —
  the safe direction, and **not changed**.
- **`min_range` decided: no change**, with the envelope documented
  instead. At the 0.30 m operating floor the C2-M4.0 defect is already
  gone (`qual` 0.9989+ against 0.0423–0.0706 at 0.28 m). **`qual`
  announces the failure without ground truth**, which covers stand-offs
  nobody characterised.
- **Two unstated preconditions found in the verification, neither
  fixed:** `check_lifted` verifies the object moved up, **not that it is
  upright** (a toppled cylinder was lifted, carried and delivered lying
  down with every step reporting success — the one placement failure);
  and `check_released` asserts the floor height **at home**, so all
  eight platform placements failed it including the seven that released
  perfectly.
- **`GRASP_MAX_LATERAL` was not retuned.**
- **Closed by C2-M4.2** (below): the full mission through the executive
  on the new path. 8 runs is not a rate; the mission figure is still
  M6's 19/20.

#### C2-M4.2 — integration gate: the mission runs on the new path — **COMPLETE**

Commit `8c3660c` on `coco2-m1-observability`, pushed. **C2-M4 is closed
including its integration.**

- **The defect, found statically before a run was spent on it.**
  `point_topic` feeds `approach_server` and is genuinely all the
  *manipulation* chain needs. The *executive* needs a second topic:
  `mission_states._check_search_target` gates `SEARCH_TARGET` on
  **`/perception/status`** reading `found=1`, and that was
  `target_finder`'s alone. Swapping the point topic only gives zero
  publishers on the status topic, `SEARCH_TARGET` stuck in RUNNING, and
  death on its 15 s timeout as `TARGET_NOT_FOUND` — a topic-name
  problem wearing a perception diagnosis. **First broken boundary: the
  subscriber assumption.** Message type, QoS and frame were already
  compatible.
- **Built:** `target_pose.finder_status_fields()` (pure) and
  `target_pose_node`'s `status_compat_topic` (empty by default,
  `found=1` iff `validity == VALID`, on the existing 5 Hz timer); plus
  `target_source` in `perception.launch.py`, dispatched in an
  **`OpaqueFunction`** so exactly one node exists by construction, an
  unknown value **raises**, and **both** handover parameters are set
  together. `mission.launch.py` declares and forwards it.
  The format itself keeps one definition, in
  `target_finder.format_status`.
- **Default is still `target_finder`**, so the path M6's 19/20 was
  measured on is untouched. `approach_server`, `grasp_server`,
  `arm_ik`, `arm_control`, `mission_states` and `mission_executive` are
  **byte-identical**.
- **Measured, one full fetch:** fresh simulator, clean graph, sim time,
  `rviz:=false`, never `--fast`. **COMPLETE — all 16 states,
  `retries=0`, `reason=--` at every sample, 178 s.** Exactly **one**
  publisher on `/perception/target` and on `/perception/status`, both
  `target_pose_node`, verified **before and after**; `target_finder`
  never ran. **62 `found=1` samples and 62 `validity=VALID` samples —
  the same number.** 190 points published. Approach `arrived`, travel
  1.139 m, bearing nulled to `-0.000`. Grasp **`x=0.1540`** held then
  placed — inside the 5.5 mm window, and from the camera. Record:
  `docs/data/c2m42_mission.log`.
- **`RETURN_HOME` succeeded in 59.9 s** — KNOWN PROBLEMS 1's leg, second
  consecutive success under light load with RViz off. Three of six
  recorded legs have failed; **six is not a rate** and it stays open.
- **This is one run.** The standing mission figure is still M6's
  **19/20**. An existence proof that the swap works through the
  executive — not a rate, and no claim the new path is better.
- **Verification limitations untouched.** `VERIFY_PLACEMENT` passed
  because this mission places **at home**, which is `check_released`'s
  unstated precondition — not a fix. Platform placement stays **7 of
  8**. `check_lifted` still checks *up*, not *upright*.

### C2-M5 — Localization health and recovery — **C2-M5.0 and C2-M5.1 BOTH DONE**

- **Objective:** detect unsafe localization and recover.
- **Dependencies:** C2-M3.
- **C2-M5.0 (characterization) is COMPLETE, 2026-08-31.** Five
  instrumented missions. Findings, in `RESULTS.md`, "C2-M5.0
  localization health":
  - **AMCL covariance is the wrong signal and points the wrong way.**
    `sigma_xy` fell to 0.070 m — below anything in either leg that
    finished — at the instant an injected pose became 3 m wrong, and
    took 24.5 s (13.9 s on the second run) to pass the healthy maximum.
    The GOOD/DEGRADED verdict `mission_hud` has withheld since C2-M1
    **stays withheld**; the calibration says the signal does not work.
  - **The scan-vs-map likelihood detects it in 0.4 s**, replicated on
    both divergence runs. Computed from the map, the laser and TF.
  - **No threshold was picked.** Class A separates at almost any value;
    class B does not separate at all (gap 0.054 m on common ground).
    `localization_health.Thresholds` has no defaults, deliberately.
  - **Collision-monitor activity is not the discriminator.** A leg that
    finished and a leg that aborted logged the same 36 PolygonLimit
    entries; a leg 3.2 m wrong logged none. `/collision_monitor_state`
    is **edge-triggered**, so silence is not safety.
  - **A safety defect was found and not fixed:** the collision monitor's
    gating does not reach the wheels, because `/cmd_vel_nav` carries
    both `controller_server`'s raw output and `cmd_vel_relay`'s gated
    echo. **C2-M5.1 must not assume the monitor can stop the robot.**
  - **No recovery was implemented**, by design.
- **C2-M5.1 (recovery + resume) is COMPLETE, 2026-08-31.** Findings in
  `RESULTS.md`, "C2-M5.1 localization recovery":
  - **A threshold was picked, and not by searching.** `lik_mean_d >
    0.40 m`, justified as strictly above every gated sample on a leg
    that finished (largest 0.3851). One candidate, replayed once over
    the five committed C2-M5.0 CSVs. `lik_frac_near` ships **disabled**,
    and that too is a measurement.
  - **Zero false positives.** The scan signal fired 0 times over two
    whole healthy missions (1714 and 1753 samples) and three healthy
    C2-M5.0 legs.
  - **The `/amcl_pose` gap is not a staleness test.** AMCL publishes
    only after `update_min_d` of MOTION, so a 50 s stationary grasp ages
    it without bound. All three of Experiment 1's triggers were this and
    none were `SCAN_DISAGREES`. The check was removed; `map->odom`
    freshness covers a dead filter.
  - **Persistence accumulates rather than requiring continuity.** Strict
    contiguity missed a live 3 m divergence — longest unbroken stretch
    1.80 s against a 2.0 s hold — while the same stretch was ≥80% bad
    for 4.60 s.
  - **The mapped-ground gate needed a y.** The robot drives *around* the
    wedge to get home, and an x-only gate blanked 65% of the return leg.
  - **The safe stop works and is proved at the arbiter**, 0.30–0.40 s,
    with the wheel-topic publisher count unchanged at 1.
  - **The recovery does NOT reliably restore a planning-capable pose.**
    `recovery_alpha_fast/slow: 0.0` means AMCL cannot escape a confident
    wrong mode, and global relocalization on this near-rectangular map
    converged to world (2.60, −0.64) — inside the wedge — after which
    the planner reported "Start occupied". **No live run produced
    degradation → recovery → resume → COMPLETE**, and that is recorded
    as UNIT-TESTED ONLY.
  - **The negative path is clean and measured:** bounded attempts, an
    explicit reason, no infinite loop, no accidental COMPLETE.
- **Completion criteria, against what was measured:** stop safely
  **yes**; block the mission **yes**; execute a recovery **yes**;
  relocalize **yes, but not to a usable pose for class A**; validate
  **yes, by the monitor and not by ground truth**; resume or abort
  **yes**. Detection latency **3.33 / 4.52 / 82.9 s** — highly variable.
  Recovery time **9.1–33.9 s**. **Mission completion after recovery: not
  achieved live.**
- **Note:** C2-M1 deliberately **withheld** a GOOD/DEGRADED verdict in
  the HUD because that threshold has never been calibrated against a
  known-bad run. **C2-M5 is where it gets measured.** M6's run-15 AMCL
  drift is the natural benchmark.
- **C2-M1.5 handed this milestone a second, different benchmark.** Nav
  home has failed in 2 of 4 recorded legs by **two distinct mechanisms**:
  AMCL divergence of ≈3.2 m in y (the run-15 family), and a run with AMCL
  within 0.45 m that stalled 2.59 m short of home behind repeated
  `collision_monitor: PolygonStop` and `Failed to make progress`, ending
  on the sequencer's 240 s timeout. A degraded control loop (4.8 Hz
  against a 10 Hz target, under Gazebo + RViz + move_group) is an
  un-isolated confound in the second. Four runs are not a success rate.
  Detail in `RESULTS.md`, "C2-M1.5 runtime integrity".

### C2-M6 — Dynamic obstacle — scoped, not undertaken

- **Objective:** a controlled moving obstacle handled by Nav2 replanning.
- **Dependencies:** C2-M3.
- **Completion criteria:** measures collision rate, minimum clearance,
  replanning latency, number of replans, path-length increase, time
  increase, mission success. Deterministic enough to reproduce.

### C2-M7 — Robot health / diagnostics — scoped, not undertaken

- **Objective:** a system-health layer over controller heartbeat, command
  and sensor freshness, map, TF, localization, nav state, manipulation
  state, mission state.
- **Dependencies:** C2-M5.
- **Completion criteria:** a readable status the robot can reason about.
- **Note:** `mission_hud`'s staleness tracking is the seed of this.

### C2-M8 — Standardized benchmark — scoped, not undertaken

- **Objective:** a reproducible evaluation suite: nominal, initial
  localization error, target variation, reduced friction, sensor
  degradation, dynamic obstacle, failed grasp, combined disturbances.
- **Dependencies:** C2-M4, C2-M5, C2-M6.
- **Completion criteria:** fixed seeds, repeated trials, a final
  benchmark table. **No cherry-picking.**

### C2-M9 — Visually polished demonstration + 60–90 s video — scoped, not undertaken

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

---

## C2-NAV.1 — navigation tuning and validation (scoped 2026-09-01, NOT started)

C2-NAV.0 measured the baseline and ranked the causes; see
`docs/RESULTS.md`, "C2-NAV.0 navigation movement quality". This is the
follow-up it earned. **At most four changes, tested in controlled
combinations against the same seven-leg tour**, `nav_bench.py --repeats 3`
on both topologies, so every claim is comparable to the numbers already
recorded.

The baseline to beat, topology A: **16/21 legs**, median transit speed
**0.208 m/s**, terminal phase **35 %** of a leg, `enclosure_entry` **0/3**.

### Ranked proposals

**1. Stop requiring a goal yaw the mission does not need.**
*Change:* have callers send the goal with no orientation constraint, or
raise `goal_checker.yaw_goal_tolerance` toward π for the flat-ground
legs. `use_final_approach_orientation: false` is already set on the
planner with the comment "the goal has no meaningful heading here", so
the yaw requirement is unintended.
*Expected:* removes the terminal phase — a median 35 % of every leg — and
with it the `wall_adjacent` failure mode (transit 3.8 s, terminal 73.6 s).
*Risk:* **low for navigation, but not zero for the mission.** The ramp
approach and `ALIGN_FOR_CLIMB` may depend on arriving roughly head-on;
`PROJECT_STATE.md` records that the leg arrives at +0.26 to +0.28 rad
every time and that the heading gate is deliberately OFF. Verify the
climb still starts before adopting.
*Validates by:* terminal-phase seconds per leg, and a full fetch mission.

**2. Reduce the local costmap's `inflation_radius` to fit the arena.**
*Change:* `local_costmap.inflation_layer.inflation_radius` from 0.50
toward ~0.35, leaving the global costmap alone.
*Expected:* restores a zero-cost band in the 0.63 m and 0.75 m passages,
which currently have **none**, so BaseObstacle stops reading a large
absolute cost everywhere in them. This is the direct cause of the
`enclosure_entry` stall.
*Risk:* medium — less margin against the wall. Bounded by the fact that
`robot_radius` (0.20) still marks everything within 0.20 m as inscribed,
so it cannot plan into contact.
*Validates by:* `enclosure_entry` success, and `min_clearance_m` must not
fall below the 0.286 m already recorded for `obstacle_corner`.

**3. Lower `BaseObstacle.scale`, or move to `ObstacleFootprint`.**
*Change:* 8.0 → ~2.0, **or** swap the critic for `ObstacleFootprint`,
whose `getScale()` is `resolution * scale` and which checks the real
footprint rather than the centre cell.
*Expected:* BaseObstacle is currently 55 % of the chosen trajectory's
score against 2.4 % for PathDist + PathAlign; this rebalances it without
touching the cost field.
*Risk:* medium — this is the value C2-M1 raised from 0.02 deliberately.
**Do not return to 0.02.** The honest reading of that change is that it
corrected a real defect and overshot, because the two critics are scaled
in different units.
*Validates by:* the score-share table, driven `min_clearance_m`, and the
1.30 m Zone A gate still being taken.

**4. Make the progress checker survive a rotating robot.**
*Change:* raise `movement_time_allowance` above the time a 180° turn
takes at the throttled rate, or adopt `PoseProgressChecker`, which counts
rotation as progress.
*Expected:* removes the 27 (topology A) and 45 (topology B)
`Failed to make progress` aborts, each of which currently kills
`follow_path` and forces a BT replan.
*Risk:* low, but it masks genuine stalls — pair it with 1 so the rotation
it is forgiving is short.
*Validates by:* `n_progress_failures` per leg.

### Deliberately not in the first round

* **`vx_samples` / `vtheta_samples` (819 trajectories per cycle) and
  `publish_evaluation` / `publish_trajectories`.** The 8.76 Hz measured
  against a configured 10.0 Hz is confirmed; the attribution is not. Test
  these **one at a time against the rate alone** before touching
  behaviour, or the tuning above will be measured on a moving control
  rate.
* **`FollowPath.xy_goal_tolerance` 0.05 vs `goal_checker` 0.25.** A real
  5× disagreement, but proposal 1 may make it moot.
* **The collision monitor's square polygons.** Circles of the intended
  radius would stop `PolygonSlow` engaging at 0.566 m. Deferred because
  the monitor aggravates rather than causes, and because changing safety
  zones deserves its own session.

### Separately, and not part of the tuning

**C2-NAV.2 — the `/cmd_vel_nav` ownership loop.** Now measured against a
control: the wheels exceed the collision monitor's output on **0.06 % of
samples without the loop and 14.0 % with it**, worst case 0.300 m/s
against a commanded 0.0. The fix is the topic rename already described in
`PROJECT_STATE.md` KNOWN LIMITATIONS 0 — give the relay its own output
and point the arbiter at it — plus re-stamping in `cmd_vel_relay`, which
is why topology A drops 233 wheel commands as stale and topology B drops
none. This is a **safety** fix; it is not expected to fix the stalling,
because `enclosure_entry` fails 0/3 in both topologies.

---

## C2-NAV.1 — proposal 1 only, DONE and measured (2026-09-01)

**The ranked list above is the scope as it was written. Only proposal 1
was run, deliberately: one variable, one experiment.** Proposals 2, 3 and
4 are untouched and remain candidates. The full A/B is in
`docs/RESULTS.md`, "C2-NAV.1 navigation terminal yaw".

**What was changed:** exactly one line of behaviour —
`controller_server.goal_checker.plugin`, `SimpleGoalChecker` →
`nav2_controller::PositionGoalChecker`. Not the "raise
`yaw_goal_tolerance` toward π" variant the scope offered: Nav2 ships a
plugin for a position-only goal, and using it means there is no yaw
tolerance left in the file to be tempted to tune later.

**Outcome: PARTIALLY CONFIRMED.** 16/21 → 18/21, median leg −37 %,
`wall_adjacent` 1/3 → 3/3, RotateToGoal rejections 465 063 → 0, progress
aborts 27 → 13, and min clearance *improved* (worst 0.273 → 0.331 m).
**`enclosure_entry` stayed 0/3** and its stall got longer. The predicted
"removes the `wall_adjacent` failure mode" was right; the hoped-for
enclosure fix was never on offer, because that leg never reaches the
goal tolerance and so has no terminal phase to remove.

**The risk the scope flagged is now measured and is real.** It said
"verify the climb still starts before adopting". Two costs:

* Ground-truth arrival error 0.118 → 0.263 m median; 7 of 21 legs
  reached within 0.25 m by ground truth against 18 of 21.
* Median |final heading| 0.449 → 1.583 rad.

**Neither the ramp nor a full fetch was run.** Until they are, this
change is measured-but-unadopted.

### C2-NAV.2 candidates, re-ranked by what C2-NAV.1 eliminated

Three of four candidate causes of the `enclosure_entry` stall are now
ruled out by measurement — the `/cmd_vel_nav` loop (C2-NAV.0, 0/3 either
way), terminal yaw (C2-NAV.1, 0/3 either way), and the collision monitor
(C2-NAV.1: the stall now happens with `DO_NOTHING` 84–88 % of the time
and 0.55 m of free space). That promotes what is left.

1. **`BaseObstacle` — now the only surviving hypothesis.** Either
   `BaseObstacle.scale` 8.0 → ~2.0, or swap to `ObstacleFootprint`.
   **Do not return to 0.02.** Validates by: `enclosure_entry` success,
   the score-share table, and `min_clearance_m` not falling below the
   0.331 m C2-NAV.1 now holds.
2. **`local_costmap.inflation_radius` 0.50 → ~0.35.** Unchanged in
   rationale from proposal 2 above; the 0.63 m and 0.75 m passages still
   have a 0.00 m zero-cost band.
3. **Restore arrival accuracy without restoring the spin.** The
   position-only goal gave away the late GoalDist correction along with
   the rotation. Options: a tighter `goal_checker.xy_goal_tolerance` now
   that there is no yaw settle to pay for, or the `precise_goal_checker`
   pattern `PROJECT_STATE.md` already describes. **This is what makes
   C2-NAV.1 mergeable**, so it ranks above cosmetic tuning.
4. **Verify the ramp leg under a position-only goal**, or keep
   `SimpleGoalChecker` for the pre-ramp leg specifically via a second
   goal-checker plugin and `FollowPath`'s `goal_checker_id`. Required
   before merge either way.
5. **A cross-track metric that survives a change in leg structure.**
   `xtrack_med_m` compares the whole driven track to the *last* global
   plan, so it silently rewards parking at the goal; it moved
   0.571 → 1.227 m for that reason alone. Compute it over the transit
   phase against a contemporaneous plan.
6. **The control rate** (8.31 Hz against a configured 10.0). Still
   unattributed, still to be tested one sampler at a time against the
   rate alone, as C2-NAV.0 said.

---

## C2-NAV.2 — candidate 1 only, DONE and measured (2026-09-02)

One variable: `FollowPath.BaseObstacle.scale`, **8.0 → 2.0**, against the
**C2-NAV.0** baseline (`SimpleGoalChecker`, `yaw_goal_tolerance` 0.25).
The C2-NAV.1 goal-checker change was reverted first so that exactly one
value moves. Full record and every number: `docs/RESULTS.md`, "C2-NAV.2
navigation BaseObstacle scale".

**Verdict: REJECTED.** `enclosure_entry` is **0/3 before and 0/3 after**,
and every movement metric is worse — the stall is longer (median
47.84 → 64.21 s), the robot stops further out (1.150 → 1.322 m), and DWB
selects zero velocity more often (0.680 → 0.921).

This is a rejection rather than a null result, because the intervention
demonstrably worked on the quantity it targeted: `BaseObstacle`'s share of
the chosen trajectory's score fell from **71.8 % to 0.0 %**. The named
mechanism was removed and the symptom did not move.

**`BaseObstacle.scale: 2.0` is NOT approved and must not be merged.** It
is retained in the worktree only as the record of the experiment.

### What C2-NAV.2 settled

1. **`BaseObstacle` scale is the wrong control for this stall, with the
   required value bounded.** Because `sum_scores` is false and the MapGrid
   critics' effective weight is `resolution * 0.5 * scale` (0.60 per cell
   for `GoalDist`), the winning zero-velocity total is only ≈ 33. A
   forward trajectory is disqualified once `cost × scale` exceeds that —
   cost ≈ 17 at scale 2.0, ≈ 4 at scale 8.0. The pinch presents measured
   cell costs of **60–131**, so admitting them needs `scale < 0.26–0.57`,
   **below the 0.02-class ratio C2-NAV.0 forbade returning to**. The knob
   cannot reach the behaviour without recreating the defect it fixed.
2. **`BaseObstacle` is not a necessary condition for the stall.** At one
   captured stall pose, 8 of 10 sampled forward speeds are scored to
   completion with `BaseObstacle` = **0.00** and still lose to standing
   still, by a median of 7.90 points carried entirely by `PathAlign`
   (+34.40), `GoalAlign` (+29.40), `GoalDist` (+18.00) and `PathDist`
   (+14.40), summed over 12 cycles.
3. **The falsifier was already in the committed baseline.** C2-NAV.0
   repeat 2 stalled 48.21 s with `BaseObstacle` at 0.0 % of the chosen
   score. The 93.4 % figure was one instant in one repeat, not the
   population.
4. **The robot is rotating, not frozen** — 5.550 rad over a 64.21 s
   stall, commanded `w` reaching `max_vel_theta` 1.0 rad/s against an
   actual median of 0.027 rad/s, while `/cmd_vel_nav` linear is zero on
   96.7 % of samples.

### C2-NAV.3 candidates, re-ranked by what C2-NAV.2 eliminated

**Four of the original candidates are now dead by measurement**: the
`/cmd_vel_nav` ownership loop (C2-NAV.0, 0/3 either way), terminal yaw
(C2-NAV.1, 0/3 either way), the collision monitor's square zones
(C2-NAV.1, and C2-NAV.2 stalls with `DO_NOTHING` 16.3 % and 0.50 m of free
space), and `BaseObstacle` scaling (C2-NAV.2). The ranking that follows is
what survives.

1. **Why the goal/path MapGrid says forward is farther. THE OPEN
   QUESTION, and the next experiment.** Measured at the stall: the robot
   is 39.7° off the goal bearing but only **11.8° off its own global
   plan's heading** over that plan's first 0.30 m, the plan is present and
   25 poses long, and moving forward nonetheless *increases* `GoalDist`
   (15.60 → 17.40, i.e. 26 → 29 cells) and `PathAlign` (0.00 → 4.00).
   **Diagnosis before intervention**: instrument the `GoalDist` and
   `PathDist` grids themselves — dump the MapGrid cell values along the
   plan and across the trajectory endpoints — and establish whether the
   propagation is blocked at the pinch, truncated by the 3 × 3 m local
   costmap window, or seeded from a plan whose in-window portion ends
   short. No parameter change until that is known.
2. **`local_costmap.inflation_radius` 0.50 → ~0.35, or
   `cost_scaling_factor`.** Unchanged in rationale, and *promoted* by
   C2-NAV.2: the measured disqualifying costs of 60–131 are what the
   inflation produces in a 0.63 m pinch whose zero-cost band is 0.00 m.
   Attacking the cost field is the alternative to attacking the weight
   that multiplies it, and C2-NAV.2 showed the weight cannot do it.
   Validates by `enclosure_entry` success and `min_clearance_m` not
   falling below the 0.331 m C2-NAV.1 holds.
3. **`ObstacleFootprint` instead of `BaseObstacle`.** C2-NAV.0 listed it
   beside the scale change; C2-NAV.2 tested only the scale. It scores
   footprint collision rather than centre-cell cost, so it is not the same
   experiment and is not refuted by this one. Still ranks below (1)
   because it is an intervention and (1) is a diagnosis.
4. **The 3 × 3 m local costmap** against a goal 1.31 m away and a
   `sim_time` of 1.5 s. Named here for the first time: it is the window
   the MapGrid critics are computed in, and (1) may well end by
   implicating it.
5. **Restore arrival accuracy without restoring the spin.** Unchanged
   from C2-NAV.1's list. **This is what makes C2-NAV.1 mergeable**, and it
   is now the highest-ranked item that is *not* about the enclosure stall.
6. **Verify the ramp leg under a position-only goal.** Required before
   C2-NAV.1 can go to `main`, either way.
7. **A cross-track metric that survives a change in leg structure.**
   Unchanged; `xtrack_med_m` is retired until then.
8. **The control rate**, 7.86–8.45 Hz against a configured 10.0. Still
   unattributed across all three experiments.

### One infrastructure item, recorded and deliberately not fixed here
### (FIXED in C2-NAV.3, commit `323471f` — see the C2-NAV.3 section below)

`gazebo_models/scripts/ros_clean.sh` brackets every `pkill` pattern so
that a pattern cannot match the process doing the matching — **except
`'nav2_'`**. Any helper whose name contains that substring is killed by
the sweep it invokes; it cost this session a run (`c2nav2_up.sh`, exit
144, before the simulator started). Every C2-NAV.2 artefact is therefore
named `c2n2_*` rather than `c2nav2_*`. Bracketing it to `'nav[2]_'` is a
one-character fix that preserves the pattern exactly, and it was **not**
made in the C2-NAV.2 commit because that commit must carry one variable
and its documentation, nothing else.

## C2-NAV.3 — candidate 1 only, DONE and measured (2026-09-02)

**Candidate 1 was the diagnosis, and it is closed.** No parameter moved.
Full record: `docs/RESULTS.md`, "C2-NAV.3 navigation MapGrid diagnosis".

### What C2-NAV.3 settled

**The four MapGrid critics are not the cause of the enclosure stall.**
In a controlled sweep at the captured stall pose with `wz` held at exactly
0.0, `GoalDist` falls **29 → 24 cells** and `GoalAlign` **30 → 24** as
`vx` rises 0 → 0.30, while `PathAlign` and `PathDist` stay in 0–1. All
four reward forward motion or ignore it.

**`BaseObstacle` is the gate, and it is the cost field rather than the
weight.** Every pose of the transformed plan sits in an inflated cell,
cost **60–164** (run A) and **60–157** (run B), **none at cost 0**, both
runs. `BaseObstacle` charges the final pose's cost × 8.0, so the cheapest
step onto the plan costs **480** against a standing-still total of
**36.20**. Between **532 and 648 of 819** trajectories abort at critic
**3 of 7**, before `GoalDist` is computed. Across every fully-scored
trajectory in both runs, `GoalDist` never falls below the robot's own
value.

**The arithmetic that closes item 1 and re-opens item 2.**
`aggregation_type` is `last` and `sim_time × max_vel_x` = 0.45 m = 9
cells, so the four MapGrid critics are worth at most
9 × (0.6 + 0.8 + 0.8 + 0.6) = **25.20** in total. At scale 8.0 that is
spent by a cell cost of **3.15**. Nothing on the plan is below 60.

Three of the sub-hypotheses item 1 listed are answered and none of them
was right: the propagation is **not** blocked at the pinch (it ignores
obstacles entirely — `MapGridQueue::validCellToQueue` returns `true`
unconditionally), it is **not** truncated by the window in a way that
matters (the whole remaining plan fits: 28–29 poses, and the `GoalDist`
seed is the plan's own last pose), and the seed is **not** short (it sits
at the goal end of the plan, 1.450 m in L1 from the robot). What is true
is that the seed cell itself has cost **164** — `GoalDist` is measuring
the distance to a cell `BaseObstacle` would charge **1312** to stand on.

**Item 4 is answered too.** The 3 × 3 m window is not the constraint on
`GoalDist`; the goal is inside it. The window matters only through the
9-cell horizon above, which is set by `sim_time × max_vel_x`, not by the
costmap size.

### C2-NAV.4 candidates, re-ranked by what C2-NAV.3 established

1. **`local_costmap.inflation_layer.cost_scaling_factor`, 5.0 → higher.
   THE NEXT EXPERIMENT.** Promoted from item 2. A steeper decay lowers
   the corridor's cost without moving the inscribed radius, so it cannot
   make a cell the robot physically cannot occupy look safe. One variable.
2. **`local_costmap.inflation_layer.inflation_radius`, 0.50 m.** It is
   more than twice the 0.315 m half-width of the 0.63 m NW pinch, so **no
   cell in the pinch can be cheap at any scaling factor**. Ranked second
   only because it is the blunter of the two and (1) may suffice.
3. **`ObstacleFootprint` instead of `BaseObstacle`.** Unchanged in
   rationale and now better motivated: it scores footprint collision
   rather than centre-cell inflation cost, which is exactly the quantity
   C2-NAV.3 shows is doing the blocking. Ranked below (1) and (2) because
   it swaps a plugin rather than moving a number.
4. **`min_vel_x` is 0.0, so reverse is never sampled.** Recorded here for
   the first time. Once stalled, DWB cannot consider backing out — not
   because reverse scores badly, but because it is not in the sample set.
   This is a candidate for the *recovery* problem, not the entry problem.
5. **Restore arrival accuracy without restoring the spin.** Unchanged.
   Still the highest-ranked item that is not about the enclosure stall,
   and still what makes C2-NAV.1 mergeable.
6. **Verify the ramp leg under a position-only goal.** Unchanged.
7. **A cross-track metric that survives a change in leg structure.**
   Unchanged; `xtrack_med_m` stays retired.
8. **The control rate**, 7.86–8.45 Hz against a configured 10.0. Still
   unattributed across all four experiments.

### The acceptance test for C2-NAV.4, to be run BEFORE any drive

C2-NAV.3 leaves a cheap falsifier behind. Whatever the change, bring the
stack up on it, capture one stall, and rebuild the grids:

```bash
bash .navbench/c2n3_capture.sh .navbench/results/c2n4
cd docs/data && python3 c2nav3_probe.py ../../.navbench/results/c2n4_stall.json 0
```

The last line reports `cost along the transformed plan: min N`. **If N is
not below about 3, the robot will not move**, and a benchmark sweep is not
needed to establish it. Report N whether it passes or fails.

### The infrastructure item is now fixed, separately

`ros_clean.sh`'s three unbracketed patterns — `nav2_`,
`ros2_control_node`, `rosbridge` — are bracketed as of commit `323471f`,
which carries nothing else. Verified both ways: every real `nav2_*` node
still matches, and no pattern matches its own text.

**And be precise about what that does not fix**, because C2-NAV.3's own
commit message first claimed more than it delivers. Bracketing stops a
pattern matching its **own text**. It does **not** stop `nav2_` matching
another process whose command line merely *contains* that substring —
`'nav[2]_'` and `'nav2_'` match exactly the same strings, measured both
ways. **A helper named `c2nav2_up.sh` is still killed by the sweep it
invokes**, and so is any `ros2 launch ... params_file:=<…>/nav2_params.yaml`.
The mitigation remains **naming**: C2-NAV.2's helpers are `c2n2_*`,
C2-NAV.3's are `c2n3_*`, and C2-NAV.3's parameter copy is
`docs/data/c2nav3_baseline_params.yaml` rather than `*nav2_params.yaml`.
The committed `docs/data/c2nav3_*` artefacts are safe because none of
those names contains `nav2_`.

## C2-NAV.4 — candidate 1 only, DONE and measured (2026-09-02)

**One variable: `local_costmap.inflation_layer.cost_scaling_factor`.**
Baseline is **C2-NAV.0** exactly (`SimpleGoalChecker`,
`BaseObstacle.scale` 8.0, `yaw_goal_tolerance` 0.25) — neither C2-NAV.1's
goal checker nor C2-NAV.2's rejected `BaseObstacle.scale` 2.0 was
inherited, and the global costmap's scaling factor was deliberately left
at 5.0 so the plan the critics receive does not move. Full record and
every number: `docs/RESULTS.md`, "C2-NAV.4 navigation inflation cost
field".

### What C2-NAV.4 settled

1. **The brief's direction was inverted, and the source settles it.**
   `InflationLayer::computeCost` is
   `252·exp(−CSF·(d − inscribed))`, so a **higher** scaling factor makes
   the field cheaper. The lower direction was tested statically: CSF 2.5
   raises the cheapest plan cell 60 → 123 and does not move the decision.

2. **The inflation layer's inscribed radius is 0.205879 m, not
   `robot_radius` 0.20.** `Costmap2DROS` pads the 16-gon footprint by
   `footprint_padding` (default **0.01**) before `LayeredCostmap` takes
   its apothem. Only that value reproduces all 34 distinct inflated costs
   in the captured grid; `robot_radius` misses 29 of them. Confirmed
   against the live node.

3. **"Minimum plan cost below 3" is the wrong screen.** It passes CSF 15
   and CSF 20, which still stall, and it passes the **unmodified
   baseline**, whose own transformed plan already contains cost-0 cells.
   The real criterion is that the trajectory's *final* pose lands in a
   cell of cost **0**: the realised MapGrid margin at the stall is 2.0 to
   6.0 points, and `BaseObstacle.scale` 8.0 spends that on a single unit
   of raw cost.

4. **The decision does flip, and the flip point is measured.** Replaying
   all 819 evaluated samples to completion, DWB's argmin moves off
   vx 0.0000 at CSF ≈ 21 (run A 20.5→21, run B 15→20, this session's
   capture 20→22). The replay reproduces DWB's real command exactly at
   CSF 5.0 in all three captures.

5. **`cost_scaling_factor` cannot touch the 253/254 bands.** The
   inscribed and lethal costs are assigned before the exponential. That
   is this knob's ceiling, and it is what the live runs ran into.
### The live result

Eleven approaches, one fresh simulator each, RTF 0.91–0.99, on two leg
budgets. Traversed = within the 0.25 m `xy_goal_tolerance`; SUCCEEDED =
`nav_bench`'s status, which additionally needs the goal yaw.

| | 75 s status / goal err | 150 s status / goal err | traversed | SUCCEEDED |
|---|---|---|---|---|
| baseline CSF 5.0 | TIMEOUT / 1.307 m | TIMEOUT / 1.414 m | **0/3** | 0/3 |
| CSF 22.0 | TIMEOUT / 1.193 m | TIMEOUT / 1.075 m | **0/3** | 0/3 |
| CSF 30.0 | TIMEOUT / 0.961 m | TIMEOUT / **0.010 m** | **2/3** | 0/3 |
| **CSF 65.0** | **SUCCEEDED / 0.056 m** | **SUCCEEDED / 0.053 m** | **3/3** | **2/2** |

**Verdict: CONFIRMED at CSF 65.0** — mechanism and behaviour both
measured. **PARTIALLY CONFIRMED at CSF 30.0**, which traverses 2 of 3 and
never passes the goal checker: at 150 s it reaches the goal *position* to
0.010 m and still reports TIMEOUT, on the goal **yaw**, which is
C2-NAV.1's mechanism and not this one. **REJECTED at CSF 22.0**, 0 of 3.
**REJECTED for lowering the factor at all.**

**Nothing is approved for merge.** `gazebo_models/config/nav2_params.yaml`
was not touched: it still carries C2-NAV.2's rejected `BaseObstacle.scale`
2.0, and the C2-NAV.4 candidates live as separate one-line derivative
files under `docs/data/`.

### C2-NAV.5 candidates, re-ranked by what C2-NAV.4 established

1. **A rate for CSF 65, and a rate for the baseline.** n = 3 is a
   contrast, not a rate. `--repeats 1` on n fresh simulators each, and
   report **traversed** and **SUCCEEDED** as two columns so C2-NAV.4's
   result and C2-NAV.1's are not scored against each other by accident.
2. **The other six tour legs at CSF 65.** A near-binary cost field is a
   real change to open-space behaviour and only `enclosure_entry` was
   run. `wall_adjacent` (goal 0.35 m from the south wall) and
   `wall_parallel` (2.5 m held ~0.36 m off it) are the two a steeper
   decay could plausibly make worse, because CSF 65 now prices both
   clearances at zero. C2-NAV.0's committed baselines for all seven legs
   exist to compare against.
3. **`footprint_padding`, which this repository has never considered.**
   It is 0.01 by default and it is why the inscribed radius is 0.2059 m
   rather than 0.1962 m. In a 0.63 m pinch that difference is 2 % of the
   robot centre's entire lateral freedom, and the 253/254 bands it sets
   are the one thing `cost_scaling_factor` provably cannot move. If a
   later leg fails where this one now succeeds, look here.
4. **NOT `inflation_radius`, and C2-NAV.4 changes why.** C2-NAV.3 ranked
   it second on the grounds that 0.5 m is more than twice the pinch's
   half-width, so no cell in the pinch could be cheap at any scaling
   factor. That is now falsified by measurement: at CSF 65 every cell
   with clearance ≥ 0.2909 m is cost 0, the 0.315 m pinch centre
   included, with `inflation_radius` still 0.5. The radius sets where the
   field ends; the factor sets how fast it falls, and the second was
   sufficient. There is no measurement demanding the first.

## C2-NAV.5 — validation of CSF 65, DONE and measured (2026-09-02)

**PARTIALLY VALIDATED.** A validation pass, not a tuning session: exactly
two configurations, differing in one line, on genuinely fresh simulators.
Full record with every number: `docs/RESULTS.md`, "C2-NAV.5 navigation
CSF 65 validation — fresh simulators".

- BASELINE `docs/data/c2nav3_baseline_params.yaml` (`dbcee9ca…`), local
  `cost_scaling_factor` **5.0**
- CANDIDATE `docs/data/c2nav4_csf65_params.yaml` (`3d9623d6…`), local
  `cost_scaling_factor` **65.0**, global held at 5.0

### What C2-NAV.5 settled

1. **CSF 65 is reliable on `enclosure_entry`, and the baseline reliably
   is not.** Ten fresh simulators, five per condition, interleaved,
   150 s each: baseline **0/5 traversed, 0/5 SUCCEEDED**; CSF 65 **5/5
   and 5/5**, median 93.77 s, median final error 0.064 m against 1.298 m.
2. **The baseline failure is deterministic.** Five stalls inside a
   4.6 × 12.8 cm box, 1.240–1.324 m from the goal, median commanded `vx`
   exactly 0.0, crawl 90.5–90.8 s in four of five. Two of the five occur
   with the collision monitor at `DO_NOTHING`, so gating is not the
   cause.
3. **The two wall-constrained legs the brief singled out did not
   regress — both improved.** `wall_adjacent` 2/3 → **3/3** SUCCEEDED;
   `wall_parallel` 3/3 in both but median duration **56.10 → 18.97 s**.
   The cost is 3–5 cm of clearance on each.
4. **The cost-field mechanism is confirmed on fresh runs.** At the stall
   distance the baseline's transformed plan has **0 of 24 poses at cost
   0** (min 59, median 164, max 230) and `BaseObstacle` charges **456.00**;
   at CSF 65 every pose is cost **0**, `BaseObstacle` charges **0.00**,
   and forward beats zero by 1.8–6.8 points. Its 1.3 m rung shows the
   knife-edge directly: forward total 36.60 **equals** zero total 36.60,
   and DWB picks zero.
5. **One real regression, and it belongs to a different subsystem.**
   `enclosure_exit` is **1/3** at CSF 65 against 3/3 at the baseline —
   but the baseline never attempted the same leg, because its
   `enclosure_entry` always failed and left the robot outside the pocket.
   On the two failures DWB commands a median **0.2684 m/s** and never
   selects zero, while the **collision monitor holds STOP for 91.4 % and
   94.1 %** of the leg and the wheels see **0.0142 m/s**. The robot parks
   inside its own `PolygonStop` circle and is gated from leaving.

### C2-NAV.6 candidates, ranked by what C2-NAV.5 established

1. **`PolygonStop.min_points`, 4 → higher.** The measured trigger. The
   escaping run had a *closer* scan return (0.153 m) than a trapped one
   (0.218 m) and never entered STOP, so the discriminator is how many
   returns fall inside the 0.25 m circle, not the nearest one. Cheapest
   to test and changes no geometry.
2. **`PolygonStop.radius`, 0.25 → between 0.2051 and 0.25.** The zone
   extends 4.5 cm past the circumscribed radius. C2-NAV.0 raised it
   *from* 0.1 because 0.1 sat inside the chassis, so lowering it needs a
   stated floor.
3. **The `enclosure_entry` goal itself.** At 0.35 m from geometry it may
   not be a pose the robot can be left in and still command its way out.
   A benchmark-design question, to be answered before either knob moves.
4. **Topology B, which is the gap that matters for shipping.** Everything
   in C2-NAV.0 … C2-NAV.5 is topology A. `mission.launch.py` runs
   topology B, where C2-NAV.0 measured 14/21 against 16/21 and a 25 %
   transit-speed cost, and where the collision monitor's path to the
   wheels differs. **CSF 65 is unvalidated in the configuration the robot
   ships in**, and that must be closed before it goes near `main`.
5. **NOT another `cost_scaling_factor` sweep.** 22, 30 and 65 are
   measured (C2-NAV.4); 65 against 5.0 is validated on fresh simulators
   (C2-NAV.5). Every open question is downstream of DWB.
6. **NOT `inflation_radius`**, for the reason C2-NAV.4 gave and this
   session did not disturb.

**`footprint_padding` drops down the list, and C2-NAV.5 says why.**
C2-NAV.4 predicted that if a later leg failed where `enclosure_entry` now
succeeds, `footprint_padding` and `robot_radius` were where to look. A
later leg did fail. The cause is neither — it is `PolygonStop`, downstream
of the costmap entirely. The prediction that success would expose a new
failure was right; its localisation was not.

---

## C2-NAV.6 — the PolygonStop threshold, DONE and measured (2026-09-02)

**One variable, `PolygonStop.min_points` 4 → 7, on top of the C2-NAV.4/.5
candidate configuration.** `cost_scaling_factor` stayed at 65.0 and no
CSF experiment was re-run. Two fresh simulators, one per condition, both
driving `enclosure_entry,enclosure_exit` back to back at 150 s per leg —
because `enclosure_exit` is only a real test when the entry succeeded
first.

Full record with every number: `docs/RESULTS.md`, "C2-NAV.6 navigation
PolygonStop threshold". Artifacts: `docs/data/c2nav6_*`.

### What C2-NAV.6 settled

**The trigger is now measured, and it is sparse.** `PolygonStop` fires at
the exit stall on exactly **6** laser returns inside the 0.25 m circle,
on **1470 of 1470** STOP frames — zero variance — against
`min_points: 4`. The six are contiguous beams spanning **10.2 mm** of a
**convex corner** that penetrates the circle by **5.5 mm**. C2-NAV.5's
hypothesis about the mechanism was right.

**The remedy is wrong, and that is the finding.** `min_points: 7` removed
that stop, the robot moved — and 4.4 cm later STOP re-armed at exactly
**8** points, on 1418 of 1418 frames, with the obstacle 9.3 mm inside the
circle over a 16.3 mm sliver. Both exit legs still TIMEOUT 3.14 m from
the goal; driven distance went 0.263 m → 0.307 m.

**The count is a function of penetration depth, not a false positive.**
5.5 mm → 6 beams, 9.3 mm → 8 beams, both matching sliver ÷ beam-spacing
to under one beam. So a `min_points` high enough to clear the escape path
is **a radius reduction in disguise**, applied non-linearly and
pose-dependently. `min_points` is **CLOSED**; it is the wrong knob.

**Safety was not traded away, and the change still is not free.** Neither
run drove below the **0.2051 m** circumscribed radius — nearest returns
were 0.2445 m and 0.2407 m, leaving 39.4 mm and 35.6 mm of margin. But
`min_points: 7` means an obstacle showing six or fewer returns — about
**1 cm** of visible surface — no longer stops the robot, for 4.4 cm of
benefit. **Not recommended for adoption.**

**And the gate is on all three axes.** `STOP` sets `req_vel.x`, `.y` and
`.tw` to zero, so the reverse command of −0.15 m/s recorded on both exit
legs was zeroed too: the manoeuvre that would resolve the trap is gated
by the rule that created it.

**The monitor is authoritative in topology A, to the frame.** Baseline
exit: **1470** frames holding a wheel command of exactly 0.0 against
**1470** frames in STOP. The same integer.

### C2-NAV.7 candidates, re-ranked by what C2-NAV.6 established

1. **The `enclosure_entry` goal itself — now FIRST.** C2-NAV.5 ranked
   this third; C2-NAV.6 promotes it, because the trap is not a sensing
   artefact a threshold can filter out. It is the robot parked with real
   geometry 3.5–3.9 cm from its hull inside a stop zone that extends
   4.5 cm. Move the goal from 0.35 m off geometry to a stand-off that
   leaves the nearest geometry past 0.25 m from the base origin —
   roughly **5–10 cm** more clearance — and re-run the two legs unchanged
   otherwise. If the exit then succeeds, neither knob should move at all.
2. **`PolygonStop.radius`, 0.25 → between 0.2051 and 0.25 — only if (1)
   fails.** The evidence now points specifically at the polygon geometry,
   which is the condition C2-NAV.5 set for promoting it. The floor is
   **0.2051 m**, the measured circumscribed radius — *not* `robot_radius`
   0.20, which C2-NAV.0 showed is 5.1 mm smaller than the robot. A value
   must be chosen and justified before the run.
3. **Topology B, still the gap that matters for shipping.** Everything in
   C2-NAV.0 … C2-NAV.6 is topology A. **CSF 65 remains unvalidated in the
   configuration the robot ships in**, and that must be closed before it
   goes near `main`.
4. **NOT `PolygonStop.min_points`.** Measured at 4 and 7, mechanism
   understood, closed.
5. **NOT another `cost_scaling_factor` sweep**, and **NOT
   `inflation_radius`**, for the reasons C2-NAV.4 and C2-NAV.5 gave and
   this session did not disturb.

---

## C2-NAV.7 — the enclosure goal stand-off, DONE and measured (2026-09-03)

**One variable, and it is not a Nav2 parameter: the `enclosure_entry`
goal, (−3.45, 2.95) → (−3.575, 2.95).** Everything else is the C2-NAV.5
validated configuration, `PolygonStop.radius` 0.25 and `min_points` **4**
— C2-NAV.6's rejected 7 was NOT carried forward. The live parameter
read-back is **byte-identical** to C2-NAV.6's baseline, so "no navigation
parameter changed" is a diff, not a claim.

Full record: `docs/RESULTS.md`, "C2-NAV.7 navigation enclosure goal
stand-off". Artifacts: `docs/data/c2nav7_*`.

### What C2-NAV.7 settled

**The brief's hypothesis was wrong in form and right in substance.** The
goal is **0.3606 m** from the nearest geometry — **111 mm OUTSIDE** the
0.25 m stop circle — so it was never "too close" in absolute terms. What
is wrong is its relationship to the **exit path**.

**The obstacle is named, from measurement.** C2-NAV.6's six inside-circle
returns transform into the world within **0.1 mm** of
`box_obstacle_1`'s **north-west corner (−3.25, 2.65)**, and the stall
pose is 0.2437 m from it against a measured 0.2445 m.

**The constraint is a 0.150 m corridor.** The exit must cross the NW
pinch between `wall_west` (east face x = −3.900) and `box_obstacle_1`
(west face x = −3.250). Staying further than `PolygonStop.radius` from
both requires **x ∈ [−3.650, −3.500]**. The original goal at −3.450 is
**50 mm east of that band**, and so is the C2-NAV.6 stall pose. The
candidate −3.5750 is its centre, ±75 mm.

**The exit works, 3 of 3, and the stop never fires.** Three fresh
simulators, **5325 frames, 0 STOP frames, 0 returns inside the circle on
every leg**. `enclosure_exit` SUCCEEDED 3/3, driving 4.228 / 3.461 /
3.495 m in 41.42 / 33.19 / 33.27 s — against the baseline's TIMEOUT after
**0.263 m**, ending 3.139 m short. Median `v_nav` 0.2842 → wheel
**0.0853** (the 0.3 `slowdown_ratio`) where the baseline's arrived as
**0.0**: throttled, not gated. **The safety gate was not touched.**

**Two costs, recorded not tuned away.** `enclosure_entry` SUCCEEDED **1
of 3** (traversed 3/3, goal error 0.153 / 0.116 / 0.069 m) and runs
**2–2.7× slower** — 116.56 / 150.68 / 150.01 s against 55.85 s — because
at 0.325 m from `wall_west` the robot sits permanently inside
`PolygonSlow`, whose `slowdown_ratio: 0.3` scales **angular** velocity
too (C2-NAV.0 mechanism 3). And r3's entry passed within **0.2 mm** of
re-triggering the stop, so the ±75 mm design margin is not the achieved
margin.

**A measurement trap worth keeping.** `nav_bench`'s `min_clearance_m` is
**not reliable in this pocket**: it is quantised to the 5 cm map grid and
disagreed with exact world-file geometry by up to 106 mm in both
directions — reporting 0.201 m where the truth was 0.3066 m, and 0.339 m
for the one leg that genuinely entered the stop circle at 0.2437 m. The
laser and the exact geometry agree to 0.1–0.2 mm. Use
`c2nav7_geom.py track`.

**Harness note.** The goal is a position-only Python constant in
`nav_bench.py`'s `TOUR` (yaw is a shared `orientation.w = 1.0`). It is
moved by a new default-off `--goal NAME:X,Y` override, so `TOUR` stays
byte-identical to `8f05c45` and every earlier experiment reproduces
without it; the goal that ran is recorded per leg as `goal_world`.

### C2-NAV.8 candidates, ranked by what C2-NAV.7 established

1. **The seven-leg tour at the shifted goal — FIRST.** Several fresh
   simulators, 75 s per leg, `--goal enclosure_entry:-3.575,2.95`,
   against C2-NAV.5's committed 18/21. Watch `enclosure_entry`'s
   SUCCEEDED rate (1 of 3 here), whether `enclosure_exit` holds 0 STOP
   frames from a tour-length approach, and how often the entry path comes
   as close to `box_obstacle_1` as r3's 0.2502 m.
2. **`PolygonSlow`'s angular throttling — only if entry reliability is
   the blocker.** `slowdown_ratio: 0.3` scales angular velocity at a goal
   permanently inside a 0.40 m square that reaches 0.566 m on the
   diagonal. C2-NAV.0 named it as mechanism 3 and it has never been
   tested. A collision-monitor experiment, and it belongs after the tour.
3. **Topology B, still the gap that matters for shipping.** Everything in
   C2-NAV.0 … C2-NAV.7 is topology A. **CSF 65 remains unvalidated in the
   configuration the robot ships in**, and that must close before
   anything goes near `main`.
4. **NOT `PolygonStop.radius`.** C2-NAV.6 ranked it second because the
   trigger tracked penetration depth; C2-NAV.7 drove that depth to zero
   without touching the polygon. Lowering it toward the 0.2051 m floor
   would buy margin the robot no longer needs and re-open the defect
   C2-NAV.0 raised it from 0.1 to fix. **Leave it at 0.25.**
5. **NOT `PolygonStop.min_points`** (C2-NAV.6, measured at 4 and 7, wrong
   knob), **NOT another `cost_scaling_factor` sweep** (C2-NAV.4/.5), and
   **NOT `inflation_radius`**.

---

## C2-NAV.8 — the seven-leg tour at the shifted goal, DONE and measured (2026-09-03)

**A validation pass. NO variable at all: not one navigation parameter
moved.** Three complete seven-leg tours, one fresh simulator each, at the
C2-NAV.5 validated configuration with C2-NAV.7's `enclosure_entry` goal
override `(−3.45, 2.95) → (−3.575, 2.95)`. The live read-back is
**byte-identical on all three tours and byte-identical to C2-NAV.7's**,
so "no navigation parameter changed" is a diff across four experiments.

Full record: `docs/RESULTS.md`, "C2-NAV.8 navigation seven-leg tour at
the shifted enclosure goal". Artifacts: `docs/data/c2nav8_*`.

### What C2-NAV.8 settled

**The tour total does not improve. What changes is which leg fails.**
18/21 SUCCEEDED against C2-NAV.5's committed 18/21, with
`enclosure_entry` 2/3 → **1/3** and `enclosure_exit` 1/3 → **2/3**. The
five ordinary legs are 15/15 in both, and in C2-NAV.8 they record **0
STOP frames on 3016 frames** with true clearance 0.3792–0.5160 m.

**The exit mechanism is confirmed a third time.** On the two tours that
reached the pocket: **0 STOP frames on 827 exit frames**, 3.515 and
4.280 m driven in 34.28 and 47.71 s, command chain `v_nav` 0.2684 →
wheel **0.0853** — the 0.3 `slowdown_ratio`, reproducing C2-NAV.7 to the
digit. **Throttled, not gated.**

**And a failure two-leg runs could not have found.** One tour in three
ends in a **269.5 s continuous `PolygonStop` deadlock** at
(−3.3009, +1.9100), two poses 0.8 mm apart, `v_wheel` exactly **0.0 on
all 2673 frames** while `v_nav` spans −0.15 to +0.2526. The gating
geometry is `box_obstacle_1`'s **SOUTH-west corner (−3.250, +2.150)** at
0.2453 m — **4.7 mm inside** the circle, 5–6 returns. C2-NAV.6's trap was
the same box's **NORTH**-west corner at 5.5 mm and 6 returns. Both
enclosure legs are lost; the exit leg drove **0.000 m**.

**Why C2-NAV.7 could not have seen it, and it is structural.** C2-NAV.7
ran `--only enclosure_entry,enclosure_exit`, so its entry started at the
**spawn (−2.000, 0.000)**. In the tour the entry is leg 6 and starts
where `corridor_gate` ended, ≈**(−2.58, −0.02)** — 0.6 m further west,
a different approach into the NW pinch, clipping the SW corner **before**
reaching the x ∈ [−3.650, −3.500] corridor C2-NAV.7 derived. **The
corridor argument is about where the robot ENDS and says nothing about
how it gets there.**

**The entry cost is terminal rotation, not approach speed.** The two
tours that arrived reached the 0.25 m tolerance in **25.61 / 26.45 s** —
faster than C2-NAV.5's 74.91 s whole-leg median — then spent
**174.61 / 97.23 s** (87.2 % / 78.6 % of the leg) turning on the spot.

**Safety held everywhere.** Minimum true clearance over all 21 legs and
10 626 frames is **0.2453 m**, **40.2 mm above** the 0.2051 m
circumscribed radius. But a persistent deadlock the robot cannot escape
is a failure in its own right, whatever the clearance.

**Verdict: PARTIALLY VALIDATED.** Exit clean, safety intact, ordinary
legs untouched — but the tour is not reliably successful, entry is not
operationally acceptable, and 1 in 3 fresh simulators immobilises the
robot.

### C2-NAV.9 candidates, ranked by what C2-NAV.8 established

1. **The APPROACH corridor, computed offline before any simulator.**
   This is C2-NAV.7's method applied to the half of the problem it did
   not cover. The band of x that clears `PolygonStop.radius` from
   `box_obstacle_1`'s west face **and** its south-west corner on a
   northbound approach is not the band that clears the NW pinch, and the
   tour's approach from (−2.58, −0.02) must satisfy both. **If no single
   goal satisfies both, that is the result** — the shifted goal is not
   repairable by moving it again, and the answer is a planner or costmap
   change rather than a pose.
2. **The terminal yaw, independently.** It costs 78–87 % of every
   successful entry. The measured cause is C2-NAV.0's mechanism 3, never
   tested: `PolygonSlow` scales **angular** velocity by
   `slowdown_ratio: 0.3` at a goal permanently inside a 0.40 m square
   reaching 0.566 m on the diagonal. A collision-monitor experiment,
   orthogonal to the deadlock.
3. **Topology B, which is still the gap that matters most.** Every run in
   C2-NAV.0 … C2-NAV.8 is topology A. **CSF 65 AND the shifted goal are
   both unvalidated in the configuration the robot ships in.**
4. **NOT another goal offset** until (1) is computed. C2-NAV.7 moved the
   goal on an analysis correct about the exit and silent about the
   approach; this is what that silence cost.
5. **NOT `PolygonStop.radius`** (C2-NAV.7 removed the motive), **NOT
   `min_points`** (C2-NAV.6, wrong knob), **NOT another
   `cost_scaling_factor` sweep** (C2-NAV.4/.5), **NOT
   `BaseObstacle.scale`** (C2-NAV.2), **NOT `inflation_radius`**.

## C2-NAV.9 — the approach corridor, offline, DONE and measured (2026-09-03)

**Answers candidate 1 and candidate 2 from the C2-NAV.8 list above, both
offline, no simulator.** Full record: `docs/RESULTS.md`, "C2-NAV.9
navigation approach-corridor reconstruction".

### What C2-NAV.9 settled

**Candidate 1 (the approach corridor) has an answer, and it is not "no
path exists".** A widest-path search (binary search over
`scipy.ndimage.label` connectivity, 3 mm grid) from every one of
C2-NAV.8's three real `corridor_gate` exit poses to the current goal
finds a **326.0 mm** bottleneck — **76 mm above** the 250 mm
`PolygonStop.radius` needs. The tightest point on that path, 323 mm, is
closest to `box_obstacle_1`'s **SW corner** (not the NW pinch C2-NAV.7
already characterised) — the corner is real and load-bearing on the
tightest feasible route, but a route through it with margin does exist.
**The C2-NAV.8 deadlock is a controller/path-selection problem, not a
closed corridor.**

**Candidate 2 (terminal yaw) is now closed-form, not just measured.**
`PolygonSlow` (a 0.8×0.8 m square fixed to the robot body, minimum
possible reach 0.400 m for ANY heading) is **mathematically unavoidable**
at the current goal, because the goal's own clearance to the nearest
geometry (`wall_west`, 0.325 m) is less than that minimum. A 720-heading
sweep and a ±0.30 m feasible-pose map around the goal both confirm it:
**0.0%** of the local pocket is `PolygonSlow`-clear for any heading, for
any nearby goal position. This matches C2-NAV.8's own 93–94% `SLOWDOWN`
fractions exactly. But it does **not** explain the observed 97–175 s —
`terminal_yaw_travel_rad` of 8.5–10.6 rad is 2.7–3.4× a worst-case single
turn, a hunting signature on top of the geometrically-proven `SLOWDOWN`
tax, not explained by it alone.

**The likely mechanism for the deadlock's 1-in-3 rate (INFERRED, not
instrumented)**: `local_costmap.cost_scaling_factor = 65.0` (C2-NAV.4)
was chosen so `BaseObstacle` reaches cost 0 at 0.291 m — meaning DWB
cannot distinguish 257 mm from 326 mm at all. Nothing in the local cost
function rewards the wider, safer route; which one a given fresh
simulator's sampling converges to is exactly the kind of variance three
tours would show one instance of.

### C2-NAV.10 candidates, ranked by what C2-NAV.9 established

1. **A single corridor-aligned intermediate waypoint on the approach to
   `enclosure_entry`**, sitting in the wide part of the corridor
   (`x≈-3.6`, `y≈1.2-1.5`, ≥0.45 m clearance per C2-NAV.9's grid) so that
   `PathAlign`/`PathDist` — not `BaseObstacle`, blind above 0.291 m — pull
   DWB toward the route that already exists with 76 mm of margin. This is
   the only candidate that targets the diagnosed mechanism directly.
2. **The terminal yaw hunting, independently.** C2-NAV.9 proved the
   `SLOWDOWN` floor is unavoidable and closed-form; it did NOT
   instrument why 8.5–10.6 rad of travel happens instead of ≤π. A DWB
   per-cycle capture of the terminal phase (chosen `wz`, which critic
   dominates, whether recoveries fire) would separate "the 0.3 rad/s cap
   alone" from "hunting on top of it" — currently INFERRED, not measured.
3. **Topology B, still the gap that matters most and still untouched.**
   Every run in C2-NAV.0 … C2-NAV.9 is topology A. CSF 65, the shifted
   goal, and now the corridor/waypoint mechanism are all unvalidated in
   the configuration the robot ships in.
4. **NOT another goal offset.** C2-NAV.9's feasible-pose map shows no
   position within 0.30 m of the current goal escapes `PolygonSlow`, and
   the corridor bottleneck (326 mm) already clears `PolygonStop` with
   margin from every real approach pose measured. A third goal move
   would not be informed by any new geometry.
5. **NOT `PolygonStop.radius`/`min_points`** (C2-NAV.6/.7 removed the
   motive), **NOT another `cost_scaling_factor` sweep** (C2-NAV.4/.5,
   and C2-NAV.9 explains mechanistically why raising it further would not
   help — the flat-cost-0 region is what CSF 65 was chosen to produce),
   **NOT `BaseObstacle.scale`** (C2-NAV.2), **NOT `inflation_radius`**.
