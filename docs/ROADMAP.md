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

### C2-M5 — Localization health and recovery — **C2-M5.0 DONE, C2-M5.1 NEXT**

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
- **C2-M5.1 (recovery + resume) is NEXT.** Its inherited requirements
  are listed in `RESULTS.md`, "Recovery requirements for C2-M5.1". The
  evidence gap it must close first is **healthy spread**, not more
  failure examples: only one of the three recorded failures was
  spontaneous.
- **Completion criteria:** stop safely, block the mission, execute a
  recovery, relocalize, validate, resume or abort. Measures failure
  rate, detection latency, recovery success rate, recovery time,
  mission completion after recovery.
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
