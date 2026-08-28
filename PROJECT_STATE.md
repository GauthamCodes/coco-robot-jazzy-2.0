# PROJECT_STATE.md

**Authoritative snapshot. A fresh agent reads this first.**
Lives on the trunk (`jazzy-harmonic-port`) and is edited **only** there —
see `docs/STATE_PROTOCOL.md`.

Last updated: 2026-08-22, after **C2-M3.1 — the recovery paths ran on
the robot and the state machine did not need changing.**

---

## BRANCH MAP — READ BEFORE `git status` CONFUSES YOU

The trunk does **not** contain all completed work. This table is what
makes a trunk-only state file honest.

| Branch | Contains | Merged? |
|---|---|---|
| `jazzy-harmonic-port` | **the trunk.** Everything through M7 Phase 3, plus this state layer | — |
| `coco2-m1-observability` | **C2-M1 through C2-M3.1 complete.** `mission_hud`, `/mission/state`, `pitch_probe.py`, the pitch fix, both RViz views + the map audit, the terrain observer, B3 and the 1,440-episode benchmark, **the mission executive (`mission_states.py` + `mission_executive.py`)**, and **C2-M3.1's live failure validation** (documentation only — the executive is byte-identical to C2-M3.0) | **NO — unmerged** |
| `coco2-state` | this state layer only; fast-forwards onto the trunk | **NO — awaiting the owner** |

Remotes: `origin` = `coco-robot-ros2`, **`jazzy2` = `coco-robot-jazzy-2.0`**
(the COCO 2.0 repo). Both carry the trunk. `coco2-m1-observability` is on
`jazzy2` only.

**If you are looking for `mission_hud.py` or `pitch_probe.py` and they are
not there, you are on the trunk and it has not been merged yet.** That is
expected, not a bug:

```bash
git checkout coco2-m1-observability   # to work on / review C2-M1(.5)
```

Merging is the owner's call and has not been done.

---

## READ ORDER FOR A FRESH SESSION

1. This file.
2. `docs/STATE_PROTOCOL.md` — which branch owns which file. Short.
3. `docs/ROADMAP.md` — the active milestone.
4. `docs/SESSION_LOG.md` — **the session log lives here, not at the repo
   root.** 1400+ lines of history; `CLAUDE.md` points here. A second
   root-level log would fragment it.
5. `CLAUDE.md` — non-negotiable rules. Read before touching anything.
6. `git status`, then the files under **CURRENT FILES** below.

---

## MILESTONE NUMBERING — READ THIS OR YOU WILL WORK ON THE WRONG THING

Two schemes exist and **they collide**:

| Scheme | Meaning | Status |
|---|---|---|
| **M0–M6** | v1, the wedge world. The fetch mission. | **CLOSED**, 19/20 measured |
| **M7** | v2, "The Yard" — randomised terrain, MuJoCo, RL baselines | Phases 1–3 done, **Phase 4 gated** |
| **C2-M1 … C2-M9** | The **COCO 2.0** plan. The active track. | C2-M1 through **C2-M3 done** (unmerged) |

"M2" is ambiguous. **Always write `C2-M2` for the COCO 2.0 plan** and
plain `M2` only for the historical v1 milestone.

---

## CURRENT MILESTONE

**C2-M4 — perception-driven manipulation.** Not started.
**Gate cleared: C2-M3 is CLOSED.** C2-M3.0 built the executive and
completed a fetch through it; C2-M3.1 broke that mission four different
ways on the robot and the machine held.

## CURRENT STATUS

**C2-M3.1 is COMPLETE, and its result is that nothing needed fixing.**
Five live missions, four deliberately broken, fresh simulator each,
never `--fast`. Every run followed its contract exactly — retry counts,
escalation targets and terminal states all matched `mission_states.py`
and the pure-harness tests. **`mission_states.py` and
`mission_executive.py` are byte-identical to C2-M3.0**, verified with
`git diff`; the C2-M3.1 commit is documentation and the measurements
behind it.

| Scenario | Trigger | Retries | Final |
|---|---|---|---|
| Operator abort during `CLIMB` | `/mission/abort` on a moving robot | **0** | `ABORT` `OPERATOR_ABORT` (x3) |
| Navigation failure | `--lane 5.0`, goal off the map | **2** | `ABORT` `NAVIGATION_FAILED` |
| Perception failure | `target_blue` removed from the sim | **2** | `ABORT` `TARGET_NOT_FOUND` |
| Manipulation failure | cylinder removed at `GRASP` entry | **2** | `ABORT` `GRASP_FAILED` |

All four routes into `RECOVERY` — operator request, navigation action
status, state timeout, worker terminal outcome — now have a live run.
Both escalations, `ESCALATE_ABORT` and `ESCALATE_SKIP_GRASP`, were
reached. Retry counts are exact, read from the executive's own
`attempts={...}` line, each equal to that state's `max_retries`.

**The abort, measured three times.** Fired on an observation, never a
timer: only once `/mission/state` read `CLIMB` **and** three consecutive
odometry samples exceeded 0.05 m/s. Last nonzero controller command at
**+20 / +30 ms**, then **10 explicit zero commands over 0.88 s**
(`ZERO_HOLD_SECONDS = 1.0`) — the wheels are commanded to stop, not left
to age out. Travel after the abort **13.1 / 15.3 / 23.6 mm**. No stale
command resumed motion: `max |vx| = 0.0` across 50, 264 and 482 later
samples. **0 states entered after `ABORT` in 5 of 5 runs.**

**`cmd_vel` invariant held throughout: 1,134 publisher-count samples
across five runs, every one of them 1.**

**READ THIS BEFORE SAYING "RECOVERY IS VALIDATED".** Four
*representative* branches ran live. **These did not** and remain
unit-tested only: `CLOCK_STALLED`, `--no-grasp` through the executive,
`NAVIGATION_REJECTED`, `NAVIGATION_UNAVAILABLE`, `SERVICE_UNAVAILABLE`,
`SERVICE_REFUSED`, `RECOVERY_TIMEOUT`, every `ALIGN_*`, `CLIMB_TIPPED`,
every `DESCENT_*`, `RETURN_*`, `STOW_*`, `APPROACH_*`, `PLACE_*`,
`VERIFY_PLACEMENT`. The **no-stale-completion** invariant was never
provoked either — no late worker reply arrived after a cancel — so it is
still argued from the code, not measured. The correct sentence is *"live
validation completed for operator abort, navigation failure, perception
failure and grasp retry."*

**And one clean run is still not a rate.** These five runs say the
failure paths behave; they say nothing about how often the mission
fails. The standing figure remains M6's **19/20**.

**One instrumentation trap, paid for and recorded.**
`/diff_drive_controller/cmd_vel` carries **both** `Twist` and
`TwistStamped`; the arbiter publishes `TwistStamped`. A `Twist`
subscriber matches nothing, captures nothing, and raises nothing, while
`ros2 topic info` still reads healthy — and an empty capture looks
exactly like "no stale command was issued", which is the conclusion the
test existed to reach. In `CLAUDE.md`'s trap table now: **any check
whose success condition is "we saw nothing" must first prove it can see
something.**

**One C2-M3.0 open item confirmed live, not fixed.** `grasp_server`
writes `outcome=failed at magnet attach` into a space-separated
`key=value` line, so `parse_kv` keeps `failed` and drops the diagnosis.
Classification is still correct; `grasp_server` is out of scope.

## CURRENT OBJECTIVE

**C2-M4 — perception-driven manipulation.** Replace the single
hard-coded grasp coordinate with detection to depth to 3D position to TF
to candidate grasps to IK to collision check to ranking to approach to
grasp to verification. See `docs/ROADMAP.md`. The grasp window is
**5.5 mm and colour-independent**, and
`GRASP_SELF_COLLISION_X = 0.150` is the binding bound.

## MILESTONE STATUS

- **C2-M1 (observability): COMPLETE and verified** — branch
  `coco2-m1-observability`, not merged.
- **C2-M1.5 (runtime integrity gate): COMPLETE** — same branch, commit
  `1c99415`, pushed.
- **C2-M1.6 (RViz presentation): COMPLETE** — same branch, commit
  `a7bfc23`, pushed. Presentation only; **map quality classified GOOD**.
- **C2-M2.0 (terrain observer): COMPLETE** — same branch, commit
  `1aa6670`, pushed.
- **C2-M2.1 (benchmark + verdict): COMPLETE** — same branch, commit
  `7796d04`, pushed. **C2-M2 is closed.**
- **C2-M3.0 (mission executive): COMPLETE** — same branch, commits
  `1c36499` and `7796d04`..`fb2ed09`, pushed. **A full fetch ran
  through it.**
- **C2-M3.1 (live failure injection + recovery validation): COMPLETE** —
  same branch, commit `9a7368c`, pushed. Five live missions, four
  deliberately broken; **no defect found and no source changed**.
  **C2-M3 is closed.**
- **C2-M4 (perception-driven manipulation): NOT STARTED.** Current
  milestone, unblocked.
- C2-M5…C2-M9: not started. See `docs/ROADMAP.md`.

---

## COMPLETED (C2-M3.1) — the failure paths on the robot

| Item | Outcome |
|---|---|
| **The headline** | **No defect found.** Five live missions, four deliberately broken; every one followed its contract exactly. `mission_states.py` and `mission_executive.py` are **byte-identical to C2-M3.0** |
| Operator abort mid-climb, **x3** | Fired on an observation (state `CLIMB` **and** three odometry samples > 0.05 m/s), never a timer. `RECOVERY` at +36/+44/+104 ms, arbiter `active=none` at +44/+152/+158 ms, `ABORT` at +180/+204/+304 ms. **Travel after the abort 13.1 / 15.3 / 23.6 mm** |
| The stop is **commanded** | Last nonzero controller command +20/+30 ms after the call, then **10 explicit zeros over 0.88 s** — `ZERO_HOLD_SECONDS = 1.0`. Not a watchdog coast |
| No stale command | `max |vx| = 0.0` and `max |wz| = 0.0` across 50, 264 and 482 samples after the last motion, in the three abort runs |
| Navigation failure | `--lane 5.0` puts the pre-ramp goal off the map — **measured** from `coco_world.pgm` (free cells map-y `[-4.585, 3.565]`, array ends `3.840`), not guessed. Nav2 aborted all three goals: `"Goal Coordinates of(2.500000, 5.000000) was outside bounds"`. `IDLE → ABORT` in **1.2 s** |
| Perception failure | `target_blue` removed with `gz service .../remove`; **`coco_perception` untouched**. `SEARCH_TARGET` timed out **15.09 / 15.00 / 15.09 s** against a 15.0 s contract |
| Manipulation failure | Cylinder removed at the instant `GRASP` was entered, after a **successful** approach (12.54 s). `grasp_server` ran its whole unmodified sequence and reported `outcome=failed at magnet attach`. Three attempts at **13.99 / 15.60 / 15.39 s** against a 180 s timeout — a genuine **worker outcome**, not a timeout |
| Retry counts | **Exact**, from the executive's own `attempts={...}`: `NAVIGATE_TO_RAMP` 2, `SEARCH_TARGET` 2, `GRASP` 2 |
| Both escalations reached | `ESCALATE_ABORT` (run 2) and `ESCALATE_SKIP_GRASP` (runs 3 and 4) |
| **No accidental COMPLETE** | Runs 3 and 4 descended and drove home (**120 mm**, **63 mm** from home) and still ended `ABORT` carrying the original reason |
| Arbiter invariant | **1,134 publisher-count samples across five runs, every one of them 1** |
| No state after `ABORT` | **0**, in 5 of 5 runs |
| Instrumentation trap found | `/diff_drive_controller/cmd_vel` carries **both** `Twist` and `TwistStamped`; the arbiter publishes the second. A `Twist` witness captured **zero** commands, which reads exactly like the result being sought. Cost one run; now in `CLAUDE.md`'s trap table |
| Tests after the work | **589 passing / 0 failing, unchanged.** Run on a **clean** ROS graph: with a live stack up, `coco_mission` gives 134/2 |
| Checkpoint committed and pushed | `9a7368c` on `jazzy2/coco2-m1-observability` |

**Not changed, deliberately:** everything. No source file was edited —
the failure injections are the executive's own documented `--lane`
parameter and simulator-side entity removal. `cmd_vel_arbiter`,
`ramp_driver`, `approach_server`, `grasp_server`, `target_finder`,
MoveIt, Nav2, AMCL, SLAM, the map, the robot model, the world, the
action space, the reward, the shipped policy and every C2-M2 artefact
are untouched.

**Still unverified live** (unit-tested only): `CLOCK_STALLED`,
`--no-grasp` through the executive, `NAVIGATION_REJECTED`,
`NAVIGATION_UNAVAILABLE`, `SERVICE_UNAVAILABLE`, `SERVICE_REFUSED`,
`RECOVERY_TIMEOUT`, every `ALIGN_*`, `CLIMB_TIPPED`, every `DESCENT_*`,
`RETURN_*`, `STOW_*`, `APPROACH_*`, `PLACE_*`, `VERIFY_PLACEMENT`, and
the **no-stale-completion** invariant.

---

## COMPLETED (C2-M3.0) — the executive, and what the live runs cost

| Item | Outcome |
|---|---|
| **The machine** | `mission_states.py`, **pure Python, no `rclpy`**: an `Observation` in, a `Directive` out. 18 states, a `StateContract` per state (mode, owner, timeout, max retries, retry target, escalation), ~40 structured failure reasons, one uniform failure path through `RECOVERY` |
| **The adapter** | `mission_executive.py`. Subscriptions → `Observation`; one idempotent request out. Offers `/mission/start` and `/mission/abort`. **Publishes no velocity** |
| **Live fetch** | **`COMPLETE`, `result=fetch`, 0 recoveries, 0 retries, 175.8 s, home to 7 mm.** All 15 nominal transitions in order |
| **Arbiter invariant** | `/diff_drive_controller/cmd_vel` publisher count **1**, measured before the mission and again after. Three tests assert it, one by asserting `Twist` appears nowhere in the package |
| **Stronger success conditions, all exercised** | Nav legs verified against the **ground-truth** world pose, not only the action's SUCCEEDED; the climb verified against summit x **and** cross-track; the grasp against `lifted` re-read after the action returned; the place against `lifted=0` with `outcome=placed` |
| **Defect 1, `autostart`** | A launch configuration is inherited by every include and **shadows** the included file's own default. `mission.launch.py`'s `autostart` became `nav2_bringup`'s. Every lifecycle node `unconfigured`; `/amcl_pose` **0 publishers**; `/clock` healthy at 378 Hz; `ros2 param get /lifecycle_manager_localization autostart` → `False` against a params file that never mentions it. **Nothing in any log said the word.** Fixed twice: `mission_autostart`, and `nav.launch.py` pins Nav2's `autostart` |
| **Defect 2, the heading gate** | Gated on nav2_params' 0.25 rad `yaw_goal_tolerance`. Measured **+0.28** and, re-driven, **+0.26** rad; run 4 measured **+0.281**. All inside Nav2's checker, all outside a ground-truth gate, because Nav2 judges yaw against AMCL. Re-driving is structurally futile — the same goal checker cannot beat its own tolerance. **Gate off by default; the number is measured, logged and exposed** |
| **Bug caught by the unit tests first** | A `RECOVERY` that timed out was routed through the ordinary failure path, which re-entered `RECOVERY` and reset its clock — the mission would have sat there for ever. It escalates to `ABORT` now |
| **KNOWN PROBLEMS 3b did not reproduce** | Descent `outcome=goal` in **16.5 s** against 90.1 s twice in C2-M1.6 — under light load with RViz off, **which is exactly the confound 3b named**. Not closed |
| **KNOWN PROBLEMS 1 did not reproduce** | Nav home succeeded first time, against 2 failures in 4 recorded legs. Not closed |
| Tests after the work | **490 → 589**, 0 failing (+99, of which **35 construct the real node**) |
| Checkpoint committed and pushed | `fb2ed09` on `jazzy2/coco2-m1-observability` |

**Not changed, deliberately:** `traverse_demo.py` is **byte-identical** —
it is the harness the M4/M5/M6 numbers were measured with, and
`executive:=false` selects it. Nor were `cmd_vel_arbiter`, `ramp_driver`,
`approach_server`, `grasp_server`, `target_finder`, Nav2's planner /
controller / costmaps / behaviour tree, AMCL, SLAM, the map, the robot
model, the world, the action space, the reward, the shipped policy, or
any C2-M2 artefact. The only change outside `coco_mission` is the pinned
`autostart` in `nav.launch.py` and one pattern in `ros_clean.sh`.

---

## COMPLETED (C2-M2.1) — the benchmark, and what it actually shows

| Item | Outcome |
|---|---|
| **Live Gazebo observer validation** (C2-M2.0 never ran the node) | **PASSED, after three defects it exposed.** Every one invisible to the pure-core tests because nothing had ever *constructed* the node |
| Defect 1 | `is_best_effort()` called with **no argument** — it takes the topic. `TypeError` in `__init__`: **the node could not start at all** |
| Defect 2, the substantive one | The estimator was advanced from the **10 Hz publish timer**, so samples arrived exactly `MAX_AGE` apart and the observer withdrew itself on **431 of 431** samples of a full climb. Estimation now runs in the IMU callback at **50 Hz** — the rate C2-M2.0 fixed — and publication stays at 10 Hz |
| Defect 3 | `on_declared_flat` never passed, so the flat reference could never be learned. Now an explicit operator parameter |
| Live rates / integrity | `/imu` **49.1 Hz** (declared 50), `/terrain/state` **10.02 Hz**, **422/422** estimates finite, stamps monotonic sim-time |
| Live grade | **0.0000°** on the flat at confidence **1.000**; **0.0035°** off the built 18.000 at the settled tail |
| Arbiter invariant | `/diff_drive_controller/cmd_vel` publisher count **1**, measured before and after the observer started. The observer publishes `/terrain/state` and nothing else |
| B3 engage + fallback, live | Route B: bound established at t=3.10 s, **B3 engaged 167/200**, and on deliberate withdrawal fell to throttle **0.5** / lateral **3.0** — B1's shipped gains exactly |
| **Cross-engine bonus result** | τ settles at **0.3248** vs tan(18°)=0.3249 and **0.4865** vs tan(26°)=0.4877. C2-M2.0's pinning result was MuJoCo-only; **it now holds in Gazebo** |
| **The benchmark** | **1,440 intended, 1,440 completed, 0 runner errors.** Nothing dropped, retried or re-seeded |
| Estimator, by route | grade MAE **0.057 / 0.253 / 2.681°**, convergence **0.94 / 2.73 / 10.10 s**. Traction bound held on **100.0 %** of single-plane samples on all three routes |
| **The decision rule** | Applied **unchanged**. Gaps **+0.0 / +1.7 / +7.5 pp**. **RL justified on 0 of 3 routes** |
| **The caveat that matters** | On Route A B3 is **B1** (fallback 1.000, 120/120 identical), and B2 completes **97.5 %** against B3's **0.0 %**. The ascent gap is 0.0 pp because the task does not discriminate, **not** because estimation succeeded |
| Where the observer **hurts** | **Route C**: B3 ascends **58.3 %** against B1's **84.2 %** — 25.9 points worse than the baseline it falls back to — losing ascent on 32 seeds, engaging on only 13 % of steps, against a grade MAE of 2.681° |
| Route C tips, the C2-M2.0 open question | **Not smaller.** B1 106, B3 116 under the surface-relative terminator vs Phase 3's 101 under the absolute one. What changed is that it now fires at a **genuine rear-over** instead of 34° short of one. No improvement is claimed |
| Terminology corrected **before** the run | `mu_mae`→`sched_mu_gap_mae`, `mu_hat` on the wire→`mu_sched_input`, new `tau_minus_tangrade_*`. **No friction MAE is reported and none exists to report** |
| Tests after the work | **478 → 490**, 0 failing (+12, all constructing the real node) |
| Checkpoint committed and pushed | `7796d04` on `jazzy2/coco2-m1-observability` |

**Not changed, deliberately:** `baselines.py`, `yard_env.py`,
`terrain_observer.py` and `sensor_model.py` are **byte-identical to
C2-M2.0**, verified with `git diff` before the benchmark ran. The tuned
schedule, the routes, the seeds, the decision task and the
10-percentage-point margin did not move. Nor did Nav2, SLAM, AMCL, the
map, perception, the robot model, the terrain, the action space,
`cmd_vel_arbiter`, the reward, the shipped policy, or the v1 tip
terminator in its three non-Yard homes.

---

## COMPLETED (C2-M1.6)

| Item | Outcome |
|---|---|
| **Raw `/map` inspected separately from the costmaps** | **GOOD.** Five world landmarks → one rigid offset, **worst residual 25 mm = 0.50 cell**. No ghost walls; 156 of 186 occupied components are ≤ 2 cells |
| Map quality classified explicitly | **GOOD, no SLAM defect.** Nothing in SLAM changed |
| The one caveat, recorded not fixed | North/south walls have **0.55 m and 0.85 m** gaps in the far east corners the mapping drive never entered. They beat the robot's 0.297 m footprint but open onto **unknown** cells, and `track_unknown_space: true` + `allow_unknown: false` mean no plan can route through them |
| `mission.rviz` | Rewritten clean. Map α 1.0, global costmap **off**, local costmap α 0.22, TF and particles off, camera pane removed, near-top-down framing |
| `mission_debug.rviz` | **New**, and it is the C2-M1.5 view **byte-identical below its header** — verified by diff |
| Framing, measured not guessed | D 13 / pitch 1.45 / yaw 3π/2 → map **949 × 652 px**, margins 135/136/90/64. **36% larger linearly** than the preserved camera's 700 px, both fitting whole |
| Live visual verification | **Performed.** One fresh sim, 7 screenshots, both configs on the same run. Two defects found only by looking (robot lost to costmap blooms; laser invisible on white) and one config comment disproved (the camera pane costs **0** render width, 304 px of display tree) |
| Tests after the work | **435 passing / 0 failing** (+21) |
| Checkpoint committed and pushed | `a7bfc23` on `jazzy2/coco2-m1-observability` |

**Not changed, deliberately:** SLAM, slam_toolbox, map generation, Nav2
planner, controller, AMCL, costmap runtime behaviour, robot model,
terrain, PPO, mission sequencing, perception, controller gains, action
spaces. The only runtime addition is `rviz_config:=mission|mission_debug`
on `mission.launch.py`.

---

## COMPLETED (C2-M1.5)

| Item | Outcome |
|---|---|
| Fresh-session bootstrap, branch/state reconstruction | done; state files recovered from the trunk |
| Test baseline re-verified before any change | **404 passing / 0 failing** — matches the record exactly, workspace not stale |
| **Investigation B — `ROBOT PITCH = -0.314 rad`** | **DIAGNOSED and FIXED.** Stale ramp-driver state |
| **Investigation A — the failed fetch** | **First divergence identified**; `found=0` proved a consequence; nav home proved independent |
| **Investigation C — `/approach/target`** | **Semantics established. VOLATILE is correct. No change made** |
| RViz visual inspection | **Performed** — 5 screenshots of the rendered window. One objective defect found and fixed |
| Tests after the work | **414 passing / 0 failing** (+10) |
| Checkpoint committed and pushed | `1c99415` on `jazzy2/coco2-m1-observability` |

**The pitch answer, in one paragraph.** `ramp_driver` writes `self.pitch`
only inside its climb and descend loops; between segments nothing assigns
it while the 5 Hz status timer keeps publishing it. The message is never
late — the number in it is minutes old, which no subscriber-side
staleness check can see. `-0.314 rad` is a *genuine* 18° nose-up sample
taken on the ramp (`RAMP_ANGLE_DEG = 18`, 18° = 0.31416 rad), held
through the platform approach and the whole pick while `/imu` had already
returned to 0.000. Because the climb ends `GOAL_MARGIN = 0.3` m short of
the crest, that sample is *always* taken on the uniform 18° face, so the
stale value is always almost exactly the terrain grade — **a grade
estimator built on it would have matched ground truth on every metre of
ramp and then reported 18° on flat ground forever. It would have
passed.** Fixed at both ends: `/ramp/status` now publishes `pitch=--`
while idle, and `mission_hud` reads `ROBOT PITCH` from `/imu`.

## NOT COMPLETED (deliberately out of scope)

- No C2-M2 implementation. No grade estimator, no friction estimator, no
  observer-driven controller.
- No PPO retraining, no reward change, no `lateral_hold` gain change, no
  Nav2 tuning, no geometry change. None was justified by a diagnosis.
- `coco2-m1-observability` **not merged** into the trunk.
- The three M7 Phase 4 decisions: untouched.

---

## KNOWN PROBLEMS

1. **Nav home fails, by at least two distinct mechanisms, and it is not
   downstream of the climb or of vision.** Four recorded legs: FAILED,
   SUCCEEDED, FAILED, SUCCEEDED. Run 1 was AMCL divergence (**≈3.2 m** in
   y at the leg start — the M6 run-15 family). The 2026-08-17 run had
   AMCL within **0.45 m**, a clean climb, confirmed vision and a
   successful pick, and *still* stalled **2.59 m short of home** behind
   repeated `collision_monitor: PolygonStop`, 11× `Failed to make
   progress` and two timed-out Spin recoveries, ending on the client's
   240 s timeout. **Confound not isolated:** that run logged `Control
   loop missed its desired rate of 10.0000 Hz. Current loop rate is
   4.8077 Hz` with Gazebo, RViz, move_group and the probe all running.
   **Four runs are not a success rate**; the standing figure is M6's
   19/20. **This belongs to C2-M5**, which already names M6 run 15 as its
   benchmark. **C2-M3.0 (2026-08-20) drove home successfully on the
   first attempt**, under light load with RViz off. Five runs are still
   not a success rate and the problem stays open.

2. **Why *that* climb drifted 0.51 m is still unknown.** `lateral_hold`
   was on and reached its clamp (peak 0.800 = `LATERAL_CLAMP`), so "not
   engaging" is **refuted** — but the clamp is **not** established as the
   binding constraint, because a good run also peaked at 0.800 and
   finished at cross-track +0.036 m.

3. **Which gate in `target_finder._locate` rejected blue is not
   determined** — the 0.15–2.00 m range gate or the `plausible_blob`
   width check. The status line records `found=0` and not the reason.

3b. **The scripted descent timed out in both C2-M1.6 traverse runs.**
   `--no-grasp`, fresh simulator each. Both navigated out, climbed
   cleanly (`outcome=goal`, cross-track −0.01 m, disp +0.03 m) and
   confirmed blue at 1.159 m; both then **timed out at 90.1 s** in step 5
   with the robot at world **(4.50, 0.24)**, the far edge of the
   platform. **No diagnosis was attempted** — C2-M1.6 was
   presentation-only and nothing it changed can reach the controller.
   **Confound not isolated:** run 1 had two RViz instances alive (a
   harness fault, since fixed), and problem 1 above already records a
   control loop degraded to 4.8 Hz against a 10 Hz target under
   Gazebo + RViz + move_group. **Two runs are not a rate**; the standing
   figure is M6's 19/20. Whoever next runs the mission for its own sake
   should check whether this reproduces under light load before treating
   it as a defect. **C2-M3.0 did exactly that (2026-08-20): descent
   `outcome=goal` in 16.5 s, RViz off, load average under 4.0.** That is
   consistent with 3b being load-induced and does not establish it —
   three runs, and the two that failed shared a confound this one
   removed.

4. **The user's `~/ros2_ws` may still have a stale `coco_sim` build.**
   Signature: 29 `coco_rl` tests failing with `FileNotFoundError` on
   `build/coco_sim/worlds/yard_params.yaml`. Fix, **measured**:
   ```bash
   cd ~/ros2_ws && colcon build --packages-select coco_sim
   ```
   **Confirmed still true 2026-08-19**: `~/ros2_ws/install/coco_sim`
   exists but has **no `worlds/yard_params.yaml`** at all. The feature
   work is unaffected because the branch builds its own overlay (below),
   but a fresh session running from `~/ros2_ws` will see the 29 failures.

4b. **The feature branch uses a worktree-local overlay, not
   `~/ros2_ws/install`.** This is how the branch's tests and nodes
   actually resolve, and it is not obvious from anywhere else:
   ```bash
   cd <worktree> && colcon build --symlink-install \
       --build-base .build_wt --install-base .install_wt \
       --packages-select coco_rl coco_sim gazebo_models
   . <worktree>/.install_wt/setup.bash
   ```
   Both `.build_wt/` and `.install_wt/` are gitignored. **It goes stale
   like any other build**: C2-M2.1 found the overlay predated C2-M2.0's
   `terrain_observer` entry point, so `ros2 run coco_rl terrain_observer`
   did not exist until it was rebuilt. Rebuild before trusting a live run.

5. **Run pytest from inside each package directory** — from the repo root
   the `coco_rl/` *directory* shadows the installed module — **and pass
   `--ignore=test_integration`**, or `gazebo_models` dies during
   collection on `test_sim_bringup.launch.py` and silently reports zero
   tests for the package. Also source the user-space MoveIt prefix
   (`setup_env.sh` does), or `coco_moveit_config` reports 5 passed and 7
   **skipped** rather than 12. Neither is a regression; both move the
   headline total.

6. `rviz_2d_overlay_plugins` is not installed, so
   `mission_hud._publish_overlay` has **still never executed**. It
   degrades cleanly to the String topic.

7. `docs/RSE_ASSIGNMENT_PLAN_V2.md` is **untracked and belongs to a
   different project** (an AMR fleet assignment). Not part of COCO. Not
   touched.

**Resolved this session, removed from this list:** the `-0.314 rad` pitch
(#4 previously), and `/approach/target`'s one-shot VOLATILE publication
(#5 previously — investigated, and it is correct as it stands).

---

## CURRENT CHECKOUT / BRANCHES / LAST VERIFIED COMMIT

| | |
|---|---|
| **Authoritative state branch** | `jazzy-harmonic-port` (the trunk). `coco2-state` fast-forwards onto it and carries this file |
| **Active COCO feature branch** | `coco2-m1-observability` |
| **Last verified commit** | `9a7368c` — *C2-M3.1: the failure paths ran on the robot, and nothing needed changing* — on `coco2-m1-observability`, **pushed to `jazzy2`** |
| Previous checkpoint | `fb2ed09` — *C2-M3.0: the fetch completed through the executive, and two live defects* |
| Before that | `1c36499` — *C2-M3.0: the mission is a state machine, and it can say why it stopped* |
| Trunk head | `6c06c45`, pushed to `origin/jazzy-harmonic-port` |

---

## TESTS RUN (2026-08-22)

Per package, **cwd set to the package directory**, against the branch's
overlay build. Run before *and* after the changes, every milestone.

| package | C2-M2.0 | C2-M2.1 | C2-M3.0 | **C2-M3.1 after** |
|---|---|---|---|---|
| `coco_config` | 70 | 70 | 70 | 70 |
| `custom_teleop` | 67 | 67 | 67 | 67 |
| `coco_rl` | **152** | **164** | 164 | 164 |
| `coco_perception` | 44 | 44 | 44 | 44 |
| `gazebo_models` | 41 | 41 | 41 | 41 |
| `coco_moveit_config` | 5 (+7 skipped) | 12 | 12 | 12 |
| `coco_sim` | 55 | 55 | 55 | 55 |
| `coco_mission` | 37 | 37 | **136** | 136 |
| **total** | **471** | **490** | **589** | **589** |

Zero failing, every time. On the **trunk**, `coco_mission` does not exist
yet; the rest arrive with the merge.

**C2-M3.1 added no tests and changed none** — the failure paths it ran
live were already asserted in the pure harness by C2-M3.0, and the live
runs agree with them. **Run the suite on a clean ROS graph:** with a
stack still up from a mission run, `coco_mission` gives **134 / 2** —
35 of its tests construct the real node and a populated graph is not the
graph they assume. `ros_clean.sh` first, then 136 / 0.

**Two invocation facts that change the total and are NOT regressions.**
Both were re-measured in C2-M2.1 and the 471 was reproduced exactly on
the unmodified tree before anything changed.

1. **The user-space MoveIt prefix.** `coco_moveit_config`'s 7
   `test_pick_poses` tests **skip** when `<ws>/moveit_prefix` is not on
   the path. `setup_env.sh` puts it there; a hand-rolled environment
   easily omits it, and C2-M2.0's 471 was measured without it. Sourced,
   they pass — same tree, **478**. C2-M2.1's +12 then gives **490**.
2. **`gazebo_models` needs `--ignore=test_integration`.** That directory
   holds the `launch_testing` suite (off by default, needs
   `-DBUILD_SIM_INTEGRATION_TESTS=ON`). A bare `pytest` tries to import
   `test_sim_bringup.launch.py`, dies during collection, and reports
   **0 tests** for the whole package rather than failing loudly.

The 12 new `coco_rl` tests are `test/test_terrain_observer_node.py`, and
they exist because C2-M2.0 tested the observer's pure core thoroughly and
**never constructed the ROS node**. Three defects lived in that gap. A
pure core with good unit tests plus an untested adapter is not a tested
system.

The 21 new `gazebo_models` tests are `test/test_rviz_configs.py`, and
every one is a **silent** failure mode — a QoS mismatch, a wrong fixed
frame, a topic nobody publishes, a plugin pointed at a message type it
cannot subscribe to. RViz errors on none of those; it draws nothing and
looks like a broken robot.

**Not run:** the `launch_testing` integration test
(`gazebo_models/test_integration/`, off by default, needs
`-DBUILD_SIM_INTEGRATION_TESTS=ON`).

---

## EXPERIMENTS RUN

### C2-M3.0 (2026-08-20)

Fresh simulator each, `ros_clean.sh` between, `gui:=false`, RViz off,
never `--fast`. One Gazebo at a time.

| # | What | Result |
|---|---|---|
| 1 | Full fetch through the executive | **ABORT / `NO_LOCALIZATION`.** Every Nav2 lifecycle node `unconfigured`, `/amcl_pose` **0 publishers** — the `autostart` leak |
| 2 | Repeat after the fix | **Contaminated and discarded.** An orphaned stack from a killed foreground launch: wheel-topic publisher count **2**, two `mission_executive` processes. It is why runs 3 and 4 gate on publisher count = 1 before starting |
| 3 | Repeat, clean stack | **ABORT / `ALIGN_HEADING`** at +0.28 rad, re-driven +0.26. `NAVIGATE_TO_RAMP`'s ground-truth region check **passed** |
| 4 | Repeat, heading gate off | **COMPLETE, `result=fetch`.** 15/15 transitions, 0 recoveries, 0 retries, 175.8 s, home to **7 mm**, arbiter publisher count 1 before and after |

### C2-M1.5 / C2-M1.6 (2026-08-17)

Fresh simulator each, `ros_clean.sh` between, `gui:=false`, never
`--fast`. One Gazebo at a time.

| # | What | Runs | Result |
|---|---|---|---|
| 1 | Full fetch, blue, with `pitch_probe` at 10 Hz | 1 | 1,900 samples. Pitch defect quantified: **0.314 rad** peak error off-segment, field changed 21× vs `/imu`'s 144. Mission **failed at nav home** with a clean climb and a successful pick |
| 2 | Traverse only (`--no-grasp`) after the fix | 1 | 2,200 samples. `pitch=--` off-segment throughout; on-segment tracks `/imu` to one sample. **TRAVERSE COMPLETE, home to 0.10 m** |
| 3 | RViz framing check, robot at spawn | 1 | First re-framing attempt was **worse**; caught by screenshot |
| 4 | RViz `Distance` sweep, one stack, viewer restarted per value | 3 viewers | **14 overflows, 18 fits with margin, 22 too far.** Shipped 18 |
| — | Archive re-read of the two 2026-08-16 runs in `~/.ros/log` | 0 new | Supplied the whole failed-fetch answer without re-running anything |

Also measured live: `ros2 topic info -v /approach/target` — publisher 1,
subscriptions 2, QoS compatible, VOLATILE both ends.

### C2-M1.6 (same day, later)

| # | What | Runs | Result |
|---|---|---|---|
| 5 | Offline map audit against `coco_world.world` | 0 sim | **Map GOOD.** 5 landmarks → one rigid offset, worst residual **25 mm = 0.50 cell**. Reproduce: `python3 docs/data/map_audit.py` |
| 6 | RViz framing sweep, map_server + rviz2 only, viewer restarted per value | 5 viewers | D12 bottom margin 24 px; **D13/pitch 1.45 → 949 × 652 px, margins 135/136/90/64, shipped**; D14 and D16 wasteful |
| 7 | Live traverse, `--no-grasp`, both configs on one sim | 1 | 7 screenshots. Nav OK, climb `outcome=goal`, blue CONFIRMED, **descent timed out at 90.1 s** (see KNOWN PROBLEMS 3b) |
| 8 | Second live traverse (first attempt, 2 viewers alive) | 1 | Same outcome; the two-viewer harness fault was found here and fixed |

**Three harness traps worth not re-paying.** `x11grab` captures a screen
*region*, so another session's window landed in one shot — use
`xwd -id <win>`, which asks the X server for the window's own pixels.
Parking the pointer in a screen corner hits a desktop hot corner and
silently orbits the camera away from the config under test; the tell is
the status bar reading "Left-Click: Rotate" instead of "RViz is ready".
And kill the previous viewer, or `xdotool search --name RViz` returns the
wrong window. RViz does **not** write the `-d` config back on exit —
checked.

---

## CURRENT FILES

Nothing is mid-edit. Changed by C2-M1.5:

Changed by C2-M1.5:

- `coco_rl/coco_rl/ramp_driver.py` — `pitch=--` while idle, `live_pitch()`
- `coco_mission/scripts/mission_hud.py` — `ROBOT PITCH` from `/imu`
- `gazebo_models/scripts/pitch_probe.py` — **new**, the instrument
- `gazebo_models/CMakeLists.txt`, `gazebo_models/scripts/ros_clean.sh`
- `coco_mission/package.xml`, and the two test files

Changed by C2-M1.6 (`a7bfc23`):

- `gazebo_models/rviz/mission.rviz` — rewritten as the clean view
- `gazebo_models/rviz/mission_debug.rviz` — **new**; the C2-M1.5 view,
  byte-identical below its comment header
- `coco_mission/launch/mission.launch.py` — `rviz_config:` argument
- `gazebo_models/test/test_rviz_configs.py` — **new**, 21 tests
- `docs/data/map_audit.py` — **new**, the map instrument. Read-only, no
  ROS, deliberately not installed by any `CMakeLists.txt`
- `docs/RESULTS.md`, `docs/RUNNING.md`, `docs/SESSION_LOG.md`,
  `docs/data/README.md`, and three figures under `docs/images/`

Changed by C2-M2.0 (`1aa6670`):

- `coco_rl/coco_rl/terrain_observer.py` — **new**, the estimator. Pure
  Python, no `rclpy`
- `coco_rl/coco_rl/sensor_model.py` — **new**, the information boundary as
  a type: `DeployableSignals` vs `GroundTruth`, sharing no field name
- `coco_rl/coco_rl/terrain_observer_node.py` — **new**, the ROS face
- `coco_rl/coco_rl/terrain_benchmark.py` — **new**, the frozen benchmark
- `coco_rl/coco_rl/baselines.py` — `B3` and `schedule_gains()`
- `coco_rl/coco_rl/yard_env.py` — the surface-relative tip terminator
- `docs/data/c2m2_sanity.py` — **new**, the five implementation checks

Changed by C2-M2.1 (`7796d04`):

- `coco_rl/coco_rl/terrain_observer_node.py` — **the three live defects**:
  per-topic QoS from `is_best_effort(topic)`, estimation moved into the
  IMU callback at 50 Hz, `declare_flat` wired through
- `coco_rl/coco_rl/baseline_eval.py` — metric terminology, and `tau` /
  `tan_grade_true` recorded
- `coco_rl/coco_rl/terrain_benchmark.py` — reporting terminology only
- `coco_rl/test/test_terrain_observer_node.py` — **new**, 12 tests that
  construct the real node
- `docs/data/c2m2_benchmark.json` — **the 1,440 episodes, raw**
- `docs/data/c2m2_analysis.py`, `c2m2_plots.py`, `c2m2_live_gate.py` —
  **new** instruments; the two live-gate CSVs beside them
- `docs/RESULTS.md`, `docs/SESSION_LOG.md`, `docs/DESIGN_DECISIONS.md`,
  `docs/data/README.md`, four figures under `docs/images/`

Changed by C2-M3.0 (`1c36499`, `fb2ed09`):

- `coco_mission/scripts/mission_states.py` — **new**, the state machine.
  Pure Python, no `rclpy`. Read this one first
- `coco_mission/scripts/mission_executive.py` — **new**, the ROS adapter
- `coco_mission/scripts/mission_hud.py` — renders the new
  `/mission/state` line; the `RECOVERY` row has a source now
- `coco_mission/launch/mission.launch.py` — `executive:` and
  `mission_autostart:` arguments
- `coco_mission/CMakeLists.txt`, `coco_mission/package.xml`
- `coco_mission/test/test_mission_states.py` — **new**, 62 tests
- `coco_mission/test/test_mission_executive.py` — **new**, 35 tests that
  construct the real node
- `gazebo_models/launch/nav.launch.py` — pins `autostart: 'true'` on the
  nav2_bringup include. **The one change outside `coco_mission`, and it
  is an interface bug fix** — see RESULTS "the `autostart` leak"
- `gazebo_models/scripts/ros_clean.sh` — `mission_executiv[e]`
- `docs/ARCHITECTURE.md`, `docs/DESIGN_DECISIONS.md`, `docs/RESULTS.md`,
  `docs/RUNNING.md`, `docs/SESSION_LOG.md`

**`gazebo_models/scripts/traverse_demo.py` is byte-identical.** It is the
harness the M4/M5/M6 numbers were measured with. `executive:=false`
selects it, and running both at once is an operator error nothing
enforces against — two publishers on `/mission/mode`, which the arbiter
latches.

**C2-M3.1 touched none of these.** It ran the failure paths on the robot
and the machine held, so `mission_states.py` and `mission_executive.py`
are still byte-identical to C2-M3.0 and `RECOVERY` still stops and
nothing more — which was measured to be sufficient in every branch that
ran.

The files C2-M4 will touch first:

- `coco_perception/` — the target's 3D pose is what C2-M4 has to
  produce, and localization error is the number it lives on
- `coco_moveit_config/scripts/grasp_server.py` — the hard-coded grasp
  coordinate (`x=0.1535` in every C2-M3.1 grasp status line) is what
  gets replaced
- `custom_teleop/custom_teleop/cmd_vel_arbiter.py` — **read, do not
  change.** Sole publisher to the controller topic, and 1,134 of 1,134
  C2-M3.1 samples say so

---

## UNRESOLVED QUESTIONS

**Opened by C2-M2.1, and the most important thing on this list:**

0. **Was `ascent` the right decision task?** The benchmark says it does
   not discriminate where the privileged advantage is largest: on Route A
   every controller including open-loop reaches the deck 92–99 %, while
   B2 **completes** 97.5 % against B3's 0.0 %. C2-M2.0 chose ascent
   because Phase 3 made completion look like a score on deck geometry —
   but **B2 crosses that bridge 117 times in 120 on terrain-aware
   throttle alone**, and a pure geometry problem does not yield to
   terrain information. **Deliberately NOT resolved in C2-M2.1**:
   changing the task after seeing the result is the failure the freeze
   existed to prevent. It belongs to whoever sets the next rule. Evidence
   in `RESULTS.md` "C2-M2.1" and `DESIGN_DECISIONS.md` "The decision rule
   was not moved after the result arrived".

**Two of M7 Phase 4's three gates remain** (the third is now closed):

1. **Deck convergence geometry** — 1.95 m lateral shift in 1.80 m of
   travel before a 0.65 m bridge, against a 0.40 m turn radius. **Now
   partly contradicted:** B1 and B3 still fall off 93 times in 120, but
   **B2 falls off zero times**. The geometry is not the whole story.
2. **Route B viability** — **39.3% of episodes have mu < tan(grade) and
   are physically unclimbable**. Four options costed in `RESULTS.md`.
   **None chosen.** C2-M2.1 flagged rather than dropped them: the
   `ascent|climbable` column reads 51–54 % against a raw 32–34 %.
3. ~~Route C's tip terminator~~ — **CLOSED by C2-M2.0.** Made
   surface-relative, 0.6 rad kept exactly, with a 54.5° absolute
   backstop; the other three `TIP_LIMIT` homes are untouched at 0.6 rad
   absolute and a test asserts the split. **Do not "unify" them.**
   C2-M2.1 measured the consequence: the tip population did **not**
   shrink (B1 106, B3 116 vs Phase 3's 101). What changed is that it now
   fires at a genuine rear-over instead of 34° short of one.

**Opened by C2-M3.0:**

- **Can `ALIGN_FOR_CLIMB`'s heading gate be calibrated at all?** It needs
  either a tighter goal checker for that one leg (nav2_params already
  defines a `precise_goal_checker` at 0.05 m) or an aligner **behind the
  arbiter**, plus a threshold measured against climbs that actually
  failed rather than against Nav2's tolerance. Nothing measures the
  relationship between start yaw and climb drift today.
- **Every failure path is unverified live.** One clean run entered no
  recovery at all. Until one does, the recovery architecture is a
  well-tested description of behaviour nobody has watched.
- Whether two publishers on `/mission/mode` (executive plus
  `traverse_demo.py`) should be prevented at runtime rather than by
  documentation. The arbiter latches the last value it saw.

**Also open, from C2-M2.1:**

- Whether Route C's tips are avoidable by control at all.
- Whether B3's poor Route C behaviour (58.3 % ascent vs B1's 84.2 %)
  improves with a better grade channel on rubble. The correlation with
  grade MAE 2.681° / 10.10 s convergence / 13 % engagement is
  **suggestive and not a demonstrated cause.**
- The simulated IMU is still **noiseless**
  (`imu_noise_sigma: not_yet_measured`). Nothing in the observer
  integrates, and this still bounds what any of it claims about a real
  robot.

---

## NEXT EXACT ACTION

**Begin C2-M4 by localizing the target in 3D from perception, not by
touching the grasp.** The grasp itself is measured and works: the
approach holds a **5.5 mm** window 20/20 and the magnet held 20/20. What
C2-M4 replaces is the single hard-coded grasp coordinate — `x=0.1535`,
visible in every C2-M3.1 grasp status line — with detection → depth →
3D position → TF → candidate grasps → IK → collision check → ranking.

The first concrete step is to publish a **measured** target pose in a
robot frame and compare it against the world truth Gazebo already
knows, over the four colours and several stand-off distances. That
number — target localization error — is the one C2-M4 lives or dies on,
and nothing downstream should be built until it exists.

**Before that, one gate worth clearing cheaply:** the environment. The
workspace checkout at `~/ros2_ws/src/coco-robot-ros2` is on the
**trunk**, which does not contain `coco_mission` at all, so
`~/ros2_ws/install` cannot run any of this. C2-M3.1 built the feature
worktree into a separate overlay and left the user's workspace alone:

```bash
source ~/ros2_ws/c2m31_overlay/env.sh    # trunk underlay + worktree overlay
bash   ~/ros2_ws/c2m31_overlay/build.sh  # rebuild it
```

Reproducing a C2-M3.1 run, in full:

```bash
# T1 — fresh simulator, ALWAYS. Never --fast.
ros2 launch gazebo_models full_world_robo.launch.py traverse:=true gui:=false

# T2 — the stack. executive:=false so the executive can be run directly
#      with its documented --lane / --no-grasp parameters, which
#      mission.launch.py does not forward. rviz:=false unless you are
#      looking at it: KNOWN PROBLEMS 1 and 3b both carry a
#      Gazebo+RViz+move_group confound.
ros2 launch coco_mission mission.launch.py rviz:=false executive:=false \
    policy:=/home/gautham/coco_rl_runs/curriculum_20260726_211008/phase5_24deg_s0p0.zip

# T3 — WAIT for Nav2 before starting. The executive will abort in
#      LOCALIZE after 40 s otherwise, correctly and unhelpfully.
ros2 lifecycle get /amcl            # must read `active [3]`
ros2 topic info -v /diff_drive_controller/cmd_vel | grep -i 'publisher count'
#      ^ must read 1. If it reads 2 you have an orphan; run ros_clean.sh.

ros2 run coco_mission mission_executive.py --colour blue \
    --ros-args -p use_sim_time:=true
ros2 service call /mission/start std_srvs/srv/Trigger

# the four failure injections C2-M3.1 used, none of which edit source:
ros2 service call /mission/abort std_srvs/srv/Trigger      # mid-climb
#   --lane 5.0 on the executive                            # goal off the map
gz service -s /world/coco_world/remove --reqtype gz.msgs.Entity \
    --reptype gz.msgs.Boolean --timeout 5000 \
    --req 'name: "target_blue" type: MODEL'                # target unavailable
#   the same removal, fired on entry to GRASP              # pick against nothing
```

**Fire an abort on an observation, never on a sleep.** C2-M3.1's trigger
waits for `/mission/state` to read `CLIMB` **and** three consecutive
odometry samples above 0.05 m/s. A wall-clock guess lands in
`ALIGN_FOR_CLIMB` or after the climb has finished and proves nothing.

**And subscribe with the right type.** `/diff_drive_controller/cmd_vel`
carries both `Twist` and `TwistStamped`; the arbiter publishes
`TwistStamped`. A `Twist` subscriber captures nothing and raises nothing
— see `CLAUDE.md`'s trap table. It cost C2-M3.1 an entire run.

**Do not turn the `ALIGN_FOR_CLIMB` heading gate back on without
measuring a threshold.** It is off for a measured reason: the leg arrives
at +0.26 to +0.28 rad against ground truth, every time, because Nav2's
0.25 rad `yaw_goal_tolerance` is judged against the AMCL pose it is
steering by. Re-driving the leg cannot beat the goal checker that decided
it had arrived. See `DESIGN_DECISIONS.md` and `RESULTS.md`, "the heading
gate, and why it is off".

**Do not re-open the friction question.** It is settled and measured
twice over: τ − tan(grade) is −0.0012 / −0.0034 / +0.0043 across 1,440
episodes in MuJoCo, and 0.3248 vs tan(18°)=0.3249 live in Gazebo. Read
`DESIGN_DECISIONS.md`, "What a robot can know about the ground it is on",
before building anything that claims to estimate friction.

---

## FILES TO READ FIRST IN THE NEXT SESSION

1. `PROJECT_STATE.md` (this file)
2. `docs/ROADMAP.md` — the **C2-M4** block, which is the current
   milestone, and the C2-M3.1 block above it
3. `docs/STATE_PROTOCOL.md` — which branch owns which file
4. `docs/SESSION_LOG.md`, the **2026-08-22 C2-M3.1** entry at the tail
5. `docs/RESULTS.md`, section **"C2-M3.1 live failure injection"** —
   the five runs, every measured number, and the explicit list of
   recovery branches that did **not** run live
6. `coco_mission/scripts/mission_states.py` — the machine itself, and
   the only file where a transition is decided. Read its module
   docstring before its code. It is byte-identical to C2-M3.0 and the
   live runs are the reason
7. `coco_mission/scripts/mission_executive.py` — the ROS adapter, and
   where `--lane` / `--no-grasp` / `xy_tolerance` are read
8. `docs/ARCHITECTURE.md`, section **"The mission executive"** — the
   states, who owns the wheels in each, and what "success" means
9. `docs/DESIGN_DECISIONS.md`, the **seven** entries at the tail — why
   the executive never drives; why an action returning success is not
   success; why a platform failure comes home first; why Nav2's
   autostart is decided by whoever includes it; why a stop is proved by
   the arbiter and not by the driver asked to stop; how the failure
   injections avoided touching the code under test; and why a second
   type on the wheel topic makes a subscriber silently blind
10. `custom_teleop/custom_teleop/cmd_vel_arbiter.py` — the invariant that
    must keep surviving, and `ZERO_HOLD_SECONDS`, which is what makes an
    abort a commanded stop rather than a coast

## FROZEN — DO NOT CHANGE WITHOUT AN EXPLICIT INSTRUCTION

From `CLAUDE.md` §4, plus additions:

- The action space `(linear, angular)`, normalised `[-1, 1]`
- `cmd_vel_arbiter`, and its position as the **sole** publisher to the controller
- Camera RPY `(0,0,0)` — two tests assert this; a −0.6 rad pitch was
  proposed and is wrong in **both sign and magnitude**
- `GRASP_SELF_COLLISION_X = 0.150` — measured by probing
  `/check_state_validity` at 1 mm steps
- The target bay geometry, and anything in `coco_perception`
- The v1 wedge world, frozen as `world_v1`
- `GOAL_SUMMIT` / `GOAL_MARGIN`
- **`gazebo_models/rviz/coco_robot.rviz`** — the TF-only view. Its
  `base_footprint` fixed frame is correct *there*. A test asserts it.
- **`gazebo_models/rviz/mission_debug.rviz` is the C2-M1.5 view,
  preserved byte-identically below its comment header.** Its oblique
  Distance-18 camera is deliberate: TF triads, the arm's pose and the
  costmaps' z-stacking are readable at 35° and degenerate near straight
  down. If it needs re-framing, re-measure it — do not copy the clean
  view's numbers, which were measured for a different camera.
- **The occupancy map `maps/coco_world.pgm`.** C2-M1.6 classified it
  **GOOD** by registration against the world (worst residual 25 mm, half
  a cell). There is no SLAM problem to fix, and a rebuild would have to
  beat that.
- The training environment (`coco_rl/coco_rl/mujoco_env.py` and
  everything it touches) **must never import `rclpy`**.
- **`/approach/target` stays VOLATILE.** Investigated in C2-M1.5 and the
  current semantics are correct; see `docs/DESIGN_DECISIONS.md`.
- **The C2-M2 benchmark configuration.** Controllers B0/B1/B2/B3, routes
  A/B/C, seeds 0–119, `TUNED_SCHEDULE`, decision task `ascent`, margin
  **10 percentage points**. It was frozen before any result existed and
  the result is now recorded against it. **Re-scoring the same run on a
  different task would be choosing the metric that gives the answer.** If
  a future milestone wants a different task, it states so in advance and
  re-runs — see UNRESOLVED QUESTIONS 0.
- **The four `TIP_LIMIT` homes and their deliberate split.**
  `yard_env.py` is **surface-relative**; `reward.py`, `mujoco_env.py` and
  `ramp_driver.py` are **0.6 rad absolute** and carry the v1 curriculum,
  the shipped policy and the mission's runtime check. A test asserts it.
  **Do not "unify" them.**
- **Friction is not identifiable on this robot** from an IMU and wheel
  encoders. Measured in MuJoCo (τ spans 0.0003 over a μ span of 0.35;
  τ − tan(grade) ≈ 0 over 1,440 episodes) and confirmed live in Gazebo
  (0.3248 vs tan 18° = 0.3249). **Nothing may report a friction
  estimate**; τ is a traction-demand ratio and `mu_lower` a proven bound.

- **`gazebo_models/scripts/traverse_demo.py`.** Byte-identical through
  C2-M3.0 and it stays that way: it is the harness the M4/M5/M6 numbers
  were measured with. The executive supersedes it; it does not replace
  it in the record.
- **`ALIGN_FOR_CLIMB`'s heading gate is OFF**, and the threshold is not
  to be re-asserted without a measurement. Nav2 judges yaw against the
  AMCL pose it is steering by; this check reads ground truth; the two
  differ by the localisation error, and the leg arrives at +0.26 to
  +0.28 rad every time. Turning it on aborts missions that complete.
- **`nav.launch.py` pins `autostart: 'true'`** on its nav2_bringup
  include, and no launch file above it may declare a bare `autostart`.
  A launch configuration is inherited by every include and shadows the
  included file's own default; the symptom is every Nav2 node
  `unconfigured` with nothing in any log naming the cause.

**Baseline:** the immutable v1 result is M6's **19/20** fetch matrix on
the frozen `world_v1`. Any experiment changing the world, reward, robot
model, action space, controller, map or perception assumptions **must
state explicitly whether it remains comparable to that baseline.**

---

## FUTURE IDEAS (recorded, NOT to be started)

- Install `rviz_2d_overlay_plugins` and record the demo video (C2-M9).
- The web panel (`coco_web`) could render `/mission/hud` directly.
- Give `/approach/target` a home in an **action result** when C2-M3
  replaces the `Trigger` services — for the interface mismatch, not for
  durability. **Making it TRANSIENT_LOCAL was removed from this list**:
  C2-M1.5 established it would be a defect, because the payload is in
  `base_footprint` and latching a robot-relative point hands a late
  subscriber a coordinate in a frame that has since moved.
