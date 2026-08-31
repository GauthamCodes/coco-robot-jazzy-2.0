# PROJECT_STATE.md

**Authoritative snapshot. A fresh agent reads this first.**
Lives on the trunk (`jazzy-harmonic-port`) and is edited **only** there —
see `docs/STATE_PROTOCOL.md`.

Last updated: 2026-08-31, after **C2-M5.1 — localization health,
recovery and resume. The signal now acts: a threshold was picked from
C2-M5.0's own evidence, the monitor fires ZERO times across two whole
healthy missions, the safe stop is proved at the arbiter, and the
recovery is bounded and verifies health before resuming. It does NOT
reliably restore a pose Nav2 can plan from after a 3 m confidently-wrong
divergence, and that is recorded rather than rounded up.**

---

## BRANCH MAP — READ BEFORE `git status` CONFUSES YOU

The trunk does **not** contain all completed work. This table is what
makes a trunk-only state file honest.

| Branch | Contains | Merged? |
|---|---|---|
| `jazzy-harmonic-port` | **the trunk.** Everything through M7 Phase 3, plus this state layer | — |
| `coco2-m1-observability` | **C2-M1 through C2-M4.1 complete.** `mission_hud`, `/mission/state`, `pitch_probe.py`, the pitch fix, both RViz views + the map audit, the terrain observer, B3 and the 1,440-episode benchmark, **the mission executive (`mission_states.py` + `mission_executive.py`)**, C2-M3.1's live failure validation, **C2-M4.0's target-pose pipeline (`target_pose.py` + `target_pose_node.py`)**, and **C2-M4.1's `point_topic` grasp integration + the 60-placement benchmark + 8 live grasps**, and **C2-M4.2's `target_source` launch swap + the full mission that ran on it**, and **C2-M5.0's localization recorder + analysis + the five instrumented runs + the pure `localization_health.py`**, and **C2-M5.1's `localization_monitor.py`, the `RELOCALIZE` state, the recovery, `HOW_TO_RUN.md` and the four recorded C2-M5.1 runs** | **NO — unmerged** |
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

**C2-M5 — localization health and recovery. BOTH C2-M5.0 AND C2-M5.1
ARE COMPLETE.** C2-M5.0 characterized the failure and refused to pick a
threshold; C2-M5.1 picked one from that evidence, wired the signal to a
node, gave the executive a bounded recovery, and measured it.

**The threshold was not searched for.** One candidate was proposed from
the healthy maximum and replayed once over all five committed C2-M5.0
CSVs: `lik_mean_d > 0.40 m`, justified as **strictly above every gated
sample recorded on a leg that finished** (largest 0.3851, `obstacle1`).
`lik_frac_near` ships **disabled**, and that is a measurement too — on
mapped ground `diverged2`'s floor (0.2500) is *higher* than
`obstacle1`'s (0.1795), so no threshold below the healthy floor can fire
on it.

**Zero false positives.** The scan signal fired **0 times** across two
whole healthy missions (1714 and 1753 samples) and three healthy C2-M5.0
legs. The final nominal mission completed in **184 s with
`attempts={}`** and not one trigger.

**Five defects were found live and fixed, each with a test.** The node
built its own `Thresholds` and silently kept a default; `amcl_age` is
not a staleness test on an event-driven topic; strict-contiguity
persistence discarded real evidence; the two latches could both be set;
and the mapped-ground gate was x-only, blanking the corridor the robot
drives home through.

**What the recovery cannot do, measured.** It restores the health signal
and does **not** reliably restore a pose Nav2 can plan from.
`nav2_params` sets `recovery_alpha_fast/slow: 0.0`, so AMCL cannot
escape a mode it is confident in; and global relocalization on this
near-rectangular map converged to world **(2.60, −0.64)** — inside the
wedge footprint — after which the planner reported "Start occupied" and
"no valid path found". **No live run produced degradation → recovery →
resume → COMPLETE.** RESULTS.md records that as UNIT-TESTED ONLY.

**The `/cmd_vel_nav` loop is untouched**, deliberately. It remains
UNRESOLVED QUESTION 00 and the collision monitor still cannot stop this
robot.

## PREVIOUS MILESTONE

**C2-M4 — perception-driven manipulation. CLOSED, integration
included.** C2-M4.0 measured the pose; C2-M4.1 made it drive a grasp and
measured that; **C2-M4.2 ran a whole mission on it through the real
executive**. The robot picks the target up using a position it measured
itself, through the unmodified M6 manipulation stack, and the grasp is
verified by reading the object's own height out of Gazebo rather than by
believing an action result. **Next milestone is C2-M5 — localization
health and recovery.**

**A correction to what this file used to say.** It claimed
`grasp_server.py` "still carries `x=0.1535`". It never did. `grasp_server`
takes a fresh `/approach/target` fix when it has one and falls back to
`approach_stop_x(colour)` — a value *derived* from the window geometry,
0.1537 — only when it does not. There was no hard-coded grasp
coordinate to remove, so C2-M4.1's job was to change **where the fix
comes from**, not to delete a literal.

## CURRENT STATUS

**C2-M5.0 is COMPLETE, and the headline is that the obvious design was
wrong.** Five missions, one fresh simulator each, clean graph, sim time,
`rviz:=false`, **never `--fast`**, `target_source:=target_pose`, colour
blue. Recorder `docs/data/c2m5_locrec.py` at 10 Hz; raw CSVs committed.

| run | injection | RETURN_HOME | outcome | true error, median |
|---|---|---|---|---|
| `healthy1` | none | 80.3 s | **COMPLETE**, home to 0.078 m | 0.257 m |
| `healthy2` | **none** | 12.0 s, 3 attempts | **ABORT** | 0.300 m |
| `obstacle1` | a cylinder into the corridor | 50.0 s | **COMPLETE**, home to 0.079 m | 0.190 m |
| `diverged1` | `/initialpose` −3 m in y, tight covariance, + heading error | 131.5 s | **ABORT** `RETURN_FAILED` | 2.824 m |
| `diverged2` | the same, heading preserved | 24.7 s | **ABORT** `RETURN_FAILED` | 3.248 m |

**`healthy2` failed with no injection at all** — the spontaneous
return-home failure KNOWN PROBLEMS 1 describes, caught with
instrumentation running for the first time. Its position error stayed
inside the healthy band (0.300 m median) while its **yaw** error reached
1.31 rad; the plan lengthened 9.73 → 13.93 m, motion stopped for the
final 4 s, and `navigate_to_pose` returned ABORTED.

**Two successes and three failures is not a rate**, and two of the three
failures were induced. The standing mission figure is still M6's
**19/20**.

**Detection latency, measured against `healthy1`'s own envelope:**

| signal | `diverged1` | `diverged2` |
|---|---|---|
| scan-vs-map mean endpoint distance | **+0.4 s** | **+0.4 s** |
| AMCL `sigma_xy` | +24.5 s | +13.9 s |

**Covariance points the wrong way.** On common ground `diverged2`, 3.14 m
wrong, reported `sigma_xy` **0.281** against 0.370 / 0.389 / 0.372 on the
legs that were right. `healthy2`, the uninjected failure, had the lowest
whole-leg median of all five. Part of the dip is imposed by the
injection; the time AMCL took to notice is not.

**Collision-monitor activity is not the discriminator, in either
direction.** `obstacle1` (finished) and `diverged1` (aborted) logged the
**same 36 PolygonLimit entries**; `diverged2` was 3.2 m wrong with the
monitor at `DO_NOTHING` for the whole leg. And
`/collision_monitor_state` is **edge-triggered** — `healthy1` received
**zero messages in 219.7 s**, so silence and "not running" are identical
to a subscriber. **`PolygonStop` never fired in any of the five runs.**

**Not reproduced:** the 2026-08-17 `PolygonStop` stall and the 4.8 Hz
control loop. RTF never fell below 0.818 and `/scan` held 10 Hz, with
RViz off throughout. Consistent with load-induced; not established.

**`localization_health.py` is imported by nothing**, by design, and its
`Thresholds` has **no defaults** — it cannot be constructed without
someone naming every number, and `classify()` returns `UNKNOWN` rather
than guessing. `UNKNOWN` is falsy so `if health:` cannot read it as good
news. 30 unit tests, including one that reads the observation
dataclass's own field names and fails if anything ground-truth-shaped
appears.

---

**C2-M4.2 is COMPLETE, and the headline is that a whole fetch ran on the
measured pose.** One mission, fresh simulator, clean graph, never
`--fast`, `rviz:=false`: **COMPLETE — all 16 states, `retries=0`,
`reason=--` at every sample, 178 s** from LOCALIZE to COMPLETE. Exactly
**one** publisher on `/perception/target` **and** on
`/perception/status`, both `target_pose_node`, verified **before and
after**; `target_finder` never ran.

**The swap needed a second topic, and that was found by reading rather
than by spending a run.** C2-M4.1's `point_topic` feeds
`approach_server` and is all the *manipulation* chain needs. The
*executive* gates `SEARCH_TARGET` on **`/perception/status`** reading
`found=1` — `mission_states._check_search_target` — and that topic was
`target_finder`'s alone. Swap only the point topic and there are **zero
publishers** on the status topic, `SEARCH_TARGET` never leaves RUNNING,
and the mission dies on its 15 s timeout as `TARGET_NOT_FOUND`: a
topic-name problem wearing a perception diagnosis. **First broken
boundary: the subscriber assumption.** Type, QoS and frame were already
compatible.

The fix is `status_compat_topic`, the other half of the same handover,
and **`target_source:=target_pose`** in `perception.launch.py` sets both
together in an `OpaqueFunction` — one node by construction, an unknown
value raises. **The default is still `target_finder`**, so the path
M6's 19/20 was measured on is untouched.

Chain, measured: 62 `found=1` samples and **62 `validity=VALID` samples
— the same number**, which is the check that `found` is exactly
`validity == VALID`. 190 points on `/perception/target`. Approach
`arrived`, travel 1.139 m, bearing nulled to `-0.000`. Grasp
**`x=0.1540`** held then placed — inside the 5.5 mm window, from the
camera. Record: `docs/data/c2m42_mission.log`.

**One run is not a rate.** The standing mission figure is still M6's
**19/20**.

---

**C2-M4.1 is COMPLETE, and its headline is that the robot grasps a
target using a position it measured itself.** Eight live runs, one
fresh simulator each, never `--fast`, `target_finder` deliberately not
running and the publisher count on `/perception/target` verified 1
before every run.

| | |
|---|---|
| perception VALID at the start | **8 of 8** |
| approach `arrived` | **8 of 8** |
| `check_target_pose` accepted the perception-derived fix | **8 of 8** |
| IK + MoveIt planned and executed | **8 of 8** |
| **grasp physically verified** — the object rose, read from gz | **8 of 8** |
| **placement physically verified** — back on its own deck | **7 of 8** |
| every fix inside the 5.5 mm window [0.1510, 0.1565] | **8 of 8**, 0.15341-0.15471 |

**The integration is one parameter, not a rewrite.** `target_pose_node`
gained `point_topic`, **empty by default**; set to `/perception/target`
it stands exactly where `target_finder` stood and the whole downstream
chain runs unmodified. `approach_server`, `grasp_server`, `arm_ik`,
`arm_control` and MoveIt are **byte-identical**.

**Perception: 60 of 60 placements, 720 of 720 frames detected, 0
wrong-colour selections.** Horizontal error **0.7 / 1.4 / 2.4 mm**
(min/median/max), colour-independent to within 0.47 mm of median.
Frame-to-frame spread **0.0000 m in all 60** — bias, not noise, and a
statement about gz's noiseless depth camera rather than about a real
sensor.

**The lateral residual is sub-pixel and geometric.** On-lane, `dy` is
identical across all four colours to within 0.01 mm at every stand-off —
four diameters, four lanes, one number — and equals **0.29 to 0.43
pixels** on the 320x240 sensor. `CameraInfo` reads `cx=160.00` on a
320-wide image, half a pixel off the geometric centre, which is the
right sign and order; the equivalent offset *rises* across the sweep
rather than holding flat, so the mechanism is **not claimed**.

**THE RESULT: the static reachability verdict is a lower bound, not a
forecast.** Both lateral placements were judged `OFF_ARM_PLANE` and
**both grasped successfully**:

| lateral | perception `y` | static verdict | `y` delivered to the grasp | live |
|---|---|---|---|---|
| −0.010 | +10.2 mm | OFF_ARM_PLANE | **+1.68 mm** | **grasped, verified** |
| +0.030 | −29.2 mm | OFF_ARM_PLANE | **−3.0 mm** | **grasped, verified** |

`approach_server`'s `align` phase pivots until the bearing is nulled and
only then takes the fix the creep and the grasp use, so lateral offset
is **absorbed, not carried**. `reachability_after_approach`'s docstring
models the approach as a straight forward creep that "leaves y alone";
the real approach also turns, so the verdict **under-predicts**
feasibility. That is the safe direction for a gate to be wrong in, and
it was **not changed**. Both lateral runs are `n = 1`; 30 mm is the
largest offset tried, not a characterised limit.

**`min_range`: decision B — no change, envelope documented instead.** At
the 0.30 m operating floor C2-M4.0's defect is **already gone**: `qual`
reads **0.9989 or better** against **0.0423-0.0706** at 0.28 m, and `dx`
is the ordinary negative far-field residual (−0.68 to −1.58 mm), not the
positive radius-proportional one. It stays at 0.15 because that matches
`target_finder`, because the defect does not occur inside the envelope,
and — the reason that generalises — because **`qual` announces the
failure without ground truth**. The characterised envelope starts at
**0.30 m of stand-off**.

**TWO UNSTATED PRECONDITIONS FOUND, BOTH RECORDED AND NEITHER FIXED.**

1. **`check_lifted` verifies the object moved up, not that it is
   upright.** The one placement failure (`blue`, 0.30 m) was a cylinder
   **toppled during the pick sequence** — the instrument read it
   standing at 0.72884 right after the approach, `grasp_server`'s own
   pre-grasp read was 0.6638, and `0.64984 + r = 0.66384` is exactly a
   blue cylinder lying on its side. The magnet welded to it, lifted it
   43.7 mm, and `check_lifted` **passed**, correctly by its contract.
   It was then carried and delivered lying down with every step
   reporting success. Which motion toppled it was **not isolated**
   (1 of 1 at 0.30 m, 0 of 4 at 0.45 m, 0 of 1 at 0.70 m).
2. **`check_released` asserts the object stands at `TARGET_HEIGHT/2` —
   the floor AT HOME.** All eight platform placements failed it,
   **including the seven that released perfectly**. Correct in the M6
   mission, where the robot *is* at home; the precondition was simply
   never written down.

**`GRASP_MAX_LATERAL` was NOT retuned**, and neither was `min_range`.

**READ THIS BEFORE SAYING "MANIPULATION IS VALIDATED".** Eight runs is
not a rate — the standing mission figure is still M6's **19/20**. The
robot was **placed** on the platform with `gz set_pose`, so the climb,
the lane hold and the crest transition were not exercised; the approach
was, from `crest` onwards. Placement was verified **on the platform**,
not at home. The executive, Nav2 and AMCL were **not in the loop** —
this is the perception -> approach -> grasp chain in isolation,
deliberately, to keep the Gazebo + RViz + `move_group` confound of
KNOWN PROBLEMS 1 and 3b out of the measurement. And **nothing launches
with `point_topic` set**: `perception.launch.py` still starts
`target_finder`, which is deliberate until a full mission has run on the
new path.

---

### C2-M4.0 (previous milestone, still standing)

Target localisation measured at **1.1-2.1 mm horizontal** over 16
placements at 0.35-0.90 m, four colours, **240 of 240 frames**. The
estimate tracks a moving target (70.1 mm measured against 70 mm
commanded). The `min_range` near-face defect was found there at 0.28 m
(`dx` +4.1 to +8.3 mm, proportional to radius; the `min_range:=0.11`
control gave −1.0 to −1.4 mm) and left for C2-M4.1, which decided it
above. `target_finder.py` remains **byte-identical**.

### C2-M3.1 (previous milestone, still standing)

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

**C2-M5.1 — localization recovery and mission resume.** C2-M5.0 is
complete and its requirements are in `RESULTS.md`, "Recovery
requirements for C2-M5.1". The first one is the awkward one: **the
collision monitor cannot be relied on to stop the robot**, so the stop
must be the arbiter's and must be proved at the arbiter, as C2-M3.1
established.

M6's run-15 failure — AMCL drifting 3.4 m in the deliberately unmapped
corridor, DWB then scoring 0 of 819 trajectories — is still the case
this milestone exists to recover from, and it is the single failure in
the standing 19/20. C2-M5.0 reproduced its *shape* by injection
(`diverged1`, `diverged2`) and caught one **spontaneous** failure
(`healthy2`) that is a different mechanism.

**That C2-M4 loose end is closed.** C2-M4.2 ran the fetch:
`mission.launch.py target_source:=target_pose` completed all 16 states
with `retries=0`. `perception.launch.py` still **defaults** to
`target_finder` — deliberately, because one completed mission is not a
rate and the standing 19/20 was measured on that path. Moving the
default is a decision for whoever has a second measured comparison, not
for this gate.

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
- **C2-M4.0 (perception → 3D pose → TF → reachability): COMPLETE** —
  same branch, commit `16e952f`, pushed. **Localisation error measured
  at 1.1–2.1 mm horizontal**, four colours, 20 placements. One defect
  found (`min_range` vs the target's radius), diagnosed, **not fixed**.
- **C2-M4.1 (four-colour benchmark + grasp integration): COMPLETE** —
  same branch, commit `33028ed`, pushed. **60 of 60 placements, 720 of
  720 frames, horizontal error 0.7 / 1.4 / 2.4 mm.** **8 of 8 live
  grasps physically verified**, 7 of 8 placements. `min_range` decided
  (no change, envelope documented). Two unstated preconditions found in
  the verification and **not fixed**. **C2-M4 is closed.**
- **C2-M4.2 (integration gate: the mission runs on the new path):
  COMPLETE** — same branch, commit `8c3660c`, pushed. **One full fetch
  through the executive on `target_source:=target_pose`: COMPLETE, all
  16 states, `retries=0`, 178 s**, one publisher on each of
  `/perception/target` and `/perception/status`. A defect was found
  statically first: `point_topic` alone leaves the executive's vision
  gate unpublished. **C2-M4 is closed, integration included.**
- **C2-M5 (localization health + recovery): NOT STARTED.** Current
  milestone, unblocked.
- C2-M6…C2-M9: not started. See `docs/ROADMAP.md`.

---

## COMPLETED (C2-M4.2) — the mission runs on the measured pose

| Item | Outcome |
|---|---|
| **The headline** | **A full fetch completed through the real mission executive with `target_pose_node` in `target_finder`'s place.** All 16 states, **`retries=0`**, `reason=--` at every sample, **178 s** LOCALIZE→COMPLETE |
| **The defect, found statically** | `point_topic` alone is not enough. `mission_states._check_search_target` gates `SEARCH_TARGET` on **`/perception/status`** reading `found=1`; `target_pose_node` publishes `/perception/target_pose/status`, a different topic with no `found` key. Swap the point topic only and the executive sees **zero publishers**, `SEARCH_TARGET` stays RUNNING, and the run dies on its 15 s timeout as `TARGET_NOT_FOUND` |
| First broken boundary | **the subscriber assumption.** Message type (`PointStamped`), QoS (depth 10, RELIABLE) and frame (`base_footprint`) were all already compatible |
| The fix | `status_compat_topic`, empty by default, `found=1` **iff** `validity == VALID`, on the existing 5 Hz timer. The line is rendered by **`target_finder.format_status`**, so the format keeps exactly one definition |
| The selector | **`target_source`** in `perception.launch.py`, dispatched in an **`OpaqueFunction`** — one node by construction, an unknown value **raises**, and **both** handover parameters are set together. `mission.launch.py` declares and forwards it |
| Publisher invariant | **1** on `/perception/target` and **1** on `/perception/status`, both `target_pose_node`, verified **before and after** the run. `target_finder` never ran; one executive; `/amcl` `active [3]` |
| Both legacy consumers | `mission_executive` **and** `mission_hud` took the compat line unchanged |
| The chain, measured | **62 `found=1` = 62 `validity=VALID`** (the same number, which is the check). **190** points published. Approach `arrived`, travel **1.139 m**, bearing nulled to `-0.000`. Grasp **`x=0.1540`** held, then placed — inside [0.1510, 0.1565] |
| `RETURN_HOME` | **succeeded in 59.9 s** — KNOWN PROBLEMS 1's leg, second consecutive success under light load with RViz off. **Three of six recorded legs have failed; six is not a rate** and it stays open |
| Default unchanged | **`target_finder` is still the default.** `approach_server`, `grasp_server`, `arm_ik`, `arm_control`, `mission_states`, `mission_executive` **byte-identical** |
| Tests after the work | **684 passing / 0 failing**, up from 662, on a clean graph. All +22 in `coco_perception` (117 → 139) |
| Record | `docs/data/c2m42_mission.log` |
| Checkpoint committed and pushed | `8c3660c` on `jazzy2/coco2-m1-observability` |

**This is ONE run.** The standing mission figure is still M6's
**19/20**. It is an existence proof that the swap survives the
executive — not a rate, not a comparison against `target_finder` on the
same course, and **no claim that the new path is better**. It is
measured to work, not to win.

**`VERIFY_PLACEMENT` passed, and that is a precondition holding, not a
fix.** `check_released` asserts the floor height **at home**, and this
mission places at home. C2-M4.1's finding that it fails every correct
*platform* placement stands; the platform figure stays **7 of 8**.
`check_lifted` still verifies the object moved **up**, not that it is
**upright**. Neither was changed — the gate did not require it.

---

## COMPLETED (C2-M4.1) — the grasp runs on the measured pose

| Item | Outcome |
|---|---|
| **The headline** | **The robot picks the target up using a position it measured itself.** 8 live runs, fresh simulator each, **grasp physically verified 8 of 8** (the object's own height read from gz, not an action result), placement verified **7 of 8** |
| The benchmark | **60 of 60 placements, 720 of 720 frames detected, 0 wrong-colour selections.** Horizontal error **0.7 / 1.4 / 2.4 mm** (min/median/max), colour-independent to within 0.47 mm of median |
| The integration | **one parameter.** `target_pose_node` gained `point_topic`, empty by default; set to `/perception/target` it stands where `target_finder` stood. `approach_server`, `grasp_server`, `arm_ik`, `arm_control` and MoveIt are **byte-identical** |
| Every fix in the window | **8 of 8**, spanning 0.15341–0.15471 inside [0.1510, 0.1565] — 1.3 mm of spread in a 5.5 mm window, across four colours and three stand-offs |
| **The result** | **the static reachability verdict is a LOWER BOUND, not a forecast.** Both lateral placements read `OFF_ARM_PLANE` and **both grasped**: −0.010 arrived at the grasp as **+1.68 mm**, +0.030 as **−3.0 mm**. `approach_server`'s `align` nulls the bearing before the fix is taken, so offset is **absorbed, not carried** |
| The model gap, recorded | `reachability_after_approach` credits the approach with translation and not rotation, so it under-predicts feasibility. Safe direction; **not changed** |
| Lateral bias, diagnosed | **sub-pixel and geometric.** On-lane `dy` is identical across all four colours to within **0.01 mm** at every stand-off, and equals **0.29–0.43 px** on 320×240. `cx=160.00` on a 320-wide image is half a pixel off centre — right sign and order, but the offset *rises* across the sweep, so the mechanism is **not claimed** |
| `min_range` | **decision B: no change.** At the 0.30 m floor `qual` reads **0.9989+** against **0.0423–0.0706** at 0.28 m and `dx` is the ordinary far-field residual. Envelope documented instead; **`qual` is the runtime tell and needs no ground truth** |
| **Defect 1, not fixed** | **`check_lifted` verifies the object moved up, not that it is upright.** The one placement failure was a cylinder **toppled during the pick** (standing centre 0.72884, observed 0.66384 = deck + radius). The magnet lifted it 43.7 mm, `check_lifted` **passed**, and it was delivered lying down with every step reporting success |
| **Defect 2, not fixed** | **`check_released` asserts the floor height AT HOME** (`TARGET_HEIGHT/2`). All 8 platform placements failed it, **including the 7 that released perfectly**. Correct in the M6 mission; the precondition was never written down |
| Instruments | `docs/data/c2m4_analysis.py` (post-processing, no simulator), `docs/data/c2m4_grasp.py` (one grasp, one fresh sim). Data: `c2m4_benchmark.csv`, `c2m4_grasp.csv`, `c2m4_scatter.png` |
| Tests after the work | **662 passing / 0 failing**, up from 656, on a clean graph |
| Checkpoint committed and pushed | `33028ed` on `jazzy2/coco2-m1-observability` |

**Not changed, deliberately:** `GRASP_MAX_LATERAL` and `min_range` were
**not retuned**; `target_finder.py`, `approach_server.py`,
`grasp_server.py`, `arm_ik.py`, `arm_control.py`, MoveIt, the arbiter,
Nav2, AMCL, the map, the robot model, the world, the action space and
the shipped policy are untouched. `point_topic` is **opt-in and nothing
launches with it** — `perception.launch.py` still starts
`target_finder`.

**Was "not done", now closed by C2-M4.2:** the full mission through the
executive on the new path, including the climb and the delivery at home.
8 grasp runs is still not a rate, and the standing mission figure is
still M6's **19/20**.

---

## COMPLETED (C2-M4.0) — the target's position, measured

| Item | Outcome |
|---|---|
| **The headline** | **Target localisation error 1.1 / 1.6 / 2.1 mm horizontal** (min/median/max) over 16 placements at 0.35–0.90 m stand-off, four colours. **240 of 240 frames detected**, 20 of 20 placements measured |
| Colour independence | **within 0.8 mm** across red/green/blue/yellow — the evidence that one configured pipeline serves all four, with no per-colour branch anywhere |
| Corroboration | independently confirms the `~2.0 mm` perception residual `GRASP_MAX_LATERAL`'s comment has carried since M5 as a *budget line*. It is now a measurement |
| Bias, not noise | frame-to-frame spread **0.0000 m in all 20 placements**. Also a statement about gz: the simulated depth camera is noiseless |
| Tracks a moving target | robot parked, **target** displaced: **70.1 mm measured against 70 mm commanded** in x, **100.9 vs 100 mm** in y, and "home" repeated to the last digit after an excursion |
| **The defect** | **`min_range` gates an extended object by its near face**, a radius closer than its axis. At 0.28 m stand-off `dx` = **+4.1 / +5.5 / +6.9 / +8.3 mm** for d = 20/24/28/32 mm — proportional to radius, which is the signature |
| The control | the identical placements at `min_range:=0.11` gave **−1.0 / −1.0 / −1.3 / −1.4 mm**. One parameter changed, nothing else. **Not fixed** — C2-M4.1's call |
| Self-announcing | `hypothesis.score` (usable-depth fraction) read **1.0000 from 0.35 m out** and **0.0423–0.0706 at 0.28 m**. The failure is detectable **without ground truth** |
| A second, separate effect | `dz` at 0.28 m was −4.3 to −5.4 mm and **did not move** when the gate was lowered. That is the framing effect `target_finder`'s docstring predicted — the cylinder's top has left the frame. Costs the grasp nothing: `grasp_point.z` is `TARGET_GRASP_Z` |
| The far-field bias, explained | `SURFACE_TO_AXIS = 0.8` under-shoots a cylinder's true median offset of `0.866r` by `0.066r` = −0.7 to −1.1 mm. **Recorded, not tuned** |
| Frame semantics | `frame_id = base_footprint` 20 of 20; **`tf_age = 0.0000 s` 20 of 20** — the transform resolves at the image's own stamp, asked for there rather than at "latest" |
| Identity | `id` and `class_id` matched the requested target 20 of 20 |
| Reachability | `reach = OUT_OF_WORKSPACE` 20 of 20 — **correct**, the arm reaches base-x 0.157 and perception sees the target from 0.28 m — and `reach_appr = REACHABLE` 20 of 20, evaluated at `approach_stop_x` with the *measured* lateral offset |
| Multiple targets | exercised, not assumed: at 0.9 m the neighbouring lanes enter frame and `seen` reported two or three colours while `cand` stayed 1 and `id` stayed correct |
| Validity, live | target present → `VALID` with a full pose; `gz remove` of the model → `NOT_DETECTED`, every field `--`, and a **zero-length `detections` array still publishing** |
| Arbiter invariant | publisher count **1** on `/perception/target_pose` and **1** on `/perception/grasp_point` |
| Tests after the work | **656 passing / 0 failing**, up from 589. `coco_perception` 44 → 111 |
| Checkpoint committed and pushed | `16e952f` on `jazzy2/coco2-m1-observability` |

**Not changed, deliberately:** `target_finder.py` is **byte-identical**,
and so is everything in `coco_moveit_config`. `/perception/target` still
carries `PointStamped` in `base_footprint` and `approach_server`'s servo
mode still consumes it — the path M6's 20/20 approach was measured
through. C2-M4.0 added a path *beside* the measured one. Also untouched:
`cmd_vel_arbiter`, `grasp_server`, MoveIt, Nav2, AMCL, SLAM, the map,
the robot model, the world, the action space, the shipped policy.

**Not done:** no grasp, no driven approach (the robot was **placed**
with `gz set_pose`), on-lane only. Lateral error is exactly what decides
post-approach feasibility and C2-M4.0 did not sweep it.

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

0. **NEW, C2-M5.0: the collision monitor's gating does not reach the
   wheels.** `/cmd_vel_nav` has **7 publishers and 2 subscribers** on the
   live graph — `nav2_bringup` remaps `controller_server` and
   `velocity_smoother` to it, and `nav.launch.py arbiter:=true` points
   `cmd_vel_relay`'s **output** at the same topic, so the collision
   monitor's output is fed back into the velocity smoother's input and
   the arbiter sees the raw and gated commands together. Measured over
   three runs: the wheels receive **10.15–10.77 Hz more than the
   collision monitor publishes**, exactly `controller_frequency: 10.0`.
   During an active SLOWDOWN, gated cap `max_vel_x × slowdown_ratio` =
   0.090 m/s, wheel commands reached **0.300 m/s** — 84.2% of
   `obstacle1`'s slowdown samples, 40.0% of `diverged1`'s. **A safety
   defect. NOT a localization problem. NOT fixed** — the wheel path is
   frozen (`CLAUDE.md` §4) and C2-M5.0's mandate was to characterize.
   The fix is a topic rename; it needs a decision and a measured run.
   **C2-M5.1 must not assume the collision monitor can stop the robot.**

1. **Nav home fails, by at least two distinct mechanisms, and it is not
   downstream of the climb or of vision.** **C2-M5.0 instrumented it and
   separated two classes** — see `RESULTS.md`, "C2-M5.0 localization
   health". Class A (the pose is wrong and the filter is sure) is
   detectable from onboard signals in **0.4 s**, but **not by
   covariance**. Class B (`healthy2`, uninjected: position error 0.300 m,
   inside the healthy band, yaw error to 1.31 rad, plan lengthening,
   `navigate_to_pose` ABORTED) is **not separable by any signal
   recorded**, and no threshold is proposed. The recorded tally over
   RETURN_HOME legs is now **six failed, five succeeded across all
   sessions** — still not a rate; the standing figure is M6's 19/20.
   The 2026-08-17 `PolygonStop` stall was **not reproduced** in five
   runs and stays open. The original entry follows. Four recorded legs: FAILED,
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
| **Last verified commit** | `1cc96c7` — *C2-M5.0: covariance is the wrong signal, and the wheel path has a loop* — on `coco2-m1-observability`, **pushed to `jazzy2`** |
| Previous checkpoint | `8c3660c` — *C2-M4.2: the swap needed a second topic, and the mission completed on it* |
| Before that | `33028ed` — *C2-M4.1: the grasp runs on the measured pose, and the lateral verdict is a lower bound* |
| Trunk head | `6c06c45`, pushed to `origin/jazzy-harmonic-port` |

---

## TESTS RUN (2026-08-31)

Per package, **cwd set to the package directory**, against the branch's
overlay build. Run before *and* after the changes, every milestone.

| package | C2-M3.1 | C2-M4.0 | C2-M4.1 | C2-M4.2 | C2-M5.0 | **C2-M5.1 after** |
|---|---|---|---|---|---|---|
| `coco_config` | 70 | 70 | 70 | 70 | 70 | 70 |
| `custom_teleop` | 67 | 67 | 67 | 67 | 67 | 67 |
| `coco_rl` | 164 | 164 | 164 | 164 | 164 | 164 |
| `coco_perception` | 44 | **111** | **117** | **139** | 139 | 139 |
| `gazebo_models` | 41 | 41 | 41 | 41 | 41 | 41 |
| `coco_moveit_config` | 12 | 12 | 12 | 12 | 12 | 12 |
| `coco_sim` | 55 | 55 | 55 | 55 | 55 | 55 |
| `coco_mission` | 136 | 136 | 136 | 136 | **166** | **281** |
| **total** | **589** | **656** | **662** | **684** | **714** | **829** |

**C2-M5.1 added 115 tests, all in `coco_mission`, 0 failing**, and none
were edited to restore an old number. Two were *changed* because the
design changed underneath them and the commit says why: `Persistence`
stopped requiring strict contiguity, and `RELOCALIZE` gained its own
retry budget. One invariant test was widened — the executive now
publishes `/initialpose` as well as the three mission topics — and the
two `cmd_vel` invariant tests it sits beside were untouched and still
pass, which is the point.

`coco_web` has no `test/` directory; pytest exits 4 there, and that is
pre-existing, not breakage.

**Run them on a CLEAN ROS GRAPH.** A live stack makes `coco_mission`
fail: its node fixtures construct real nodes, and a second
`/mission/mode` publisher changes what they see.

**C2-M5.0 added 30 tests, all in `coco_mission`**, and zero were edited
to preserve a number. They pin the promises the milestone made, because
those are the parts a later session would otherwise quietly undo: that
`Thresholds` has **no defaults** and raises `TypeError` if constructed
without them; that `classify()` with no thresholds returns `UNKNOWN`
rather than guessing; that `UNKNOWN`, `STALE` and `INCONSISTENT` are all
**falsy** so `if health:` cannot read them as good news; that freshness
is checked **before** consistency and the mapped-ground gate **before**
both; that a healthy `map_odom_age` is **negative**; that covariance
alone never changes a verdict; that both divergence runs reported a
**smaller** `sigma_xy` than either leg that finished; that the 0.4 s
detection latency is recorded for **both** divergence runs; that
`healthy2` is listed as a FAILED leg despite its name; and — as a real
test rather than a comment — that no ground-truth-shaped field name can
appear in the observation dataclass.

**C2-M4.2 added 22 tests, all in `coco_perception`**, and zero were
edited to preserve a number. Twelve pin the compat status line
(`found=1` only when VALID; `found=0` for each of the five non-VALID
states; the key set is `target_finder`'s **exactly**; geometry withheld
unless valid; `range` is the **axis**, not the surface; `lane`/`age`
absent rather than invented). Four pin the parameter (off by default,
conditional publisher, separate from the node's own status topic,
published on the status timer). Six pin the launch invariant: each
source builds **exactly one** node, the two are different executables,
an unknown value **raises**, `target_pose` sets **both** handover
parameters, and the default is still `target_finder`. The launch tests
run the real dispatch through a `LaunchContext` rather than reading
source, because the invariant is behavioural.

**C2-M4.1 added 6 tests, all in `coco_perception`**, pinning the
`point_topic` seam: the default is off, the publisher is conditional,
the **axis** point is published and not the grasp point, the stamp is
the **image's**, and the publish sits inside the `is_valid` branch.
They read the node's source rather than importing it —
`target_pose_node` needs `rclpy`, `cv_bridge` and `tf2` — which is the
technique `test_camera_rpy_is_still_zero` already uses on the xacro, for
the same reason: the property is a contract and is worth pinning even
when the object holding it will not load in the test environment.

Zero failing, every time. On the **trunk**, `coco_mission` does not exist
yet; the rest arrive with the merge.

**C2-M4.0 added 67 tests, all in `coco_perception`.** The 589 baseline
was re-measured on this tree, not remembered: `coco_perception` reads
44 with the new test file ignored, and 111 with it. **Its `flake8` and
`pep257` baselines were clean**, so the 14 style errors the new files
introduced were fixed rather than counted against the pre-existing
allowance CLAUDE.md describes — that allowance covers `custom_teleop`,
not this package.

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

Changed by C2-M4.0 (`16e952f`):

- `coco_perception/coco_perception/target_pose.py` — **new**, the
  geometry and the policy. Pure Python, no `rclpy`, no `tf2`, no message
  types. **Read this one first**, and its module docstring before its
  code: it defines what the published point *means*
- `coco_perception/coco_perception/target_pose_node.py` — **new**, the
  ROS face. Holds no geometry
- `coco_perception/test/test_target_pose.py` — **new**, 67 tests
- `docs/data/c2m4_localisation.py` — **new**, the instrument **and the
  C2-M4.1 benchmark runner**. Deliberately not installed by any
  `CMakeLists.txt`
- `docs/data/c2m4_sanity_sweep.csv` — the 20 placements, raw
- `docs/data/c2m4_minrange_probe.csv` — the 8-placement control
- `coco_perception/setup.py`, `package.xml` — the entry point, and
  `vision_msgs` / `tf2_ros` / `coco_moveit_config`
- `gazebo_models/scripts/ros_clean.sh` — `target_pose_nod[e]`
- `CLAUDE.md` — the `sys.path[0]` shadowing trap
- `docs/ARCHITECTURE.md`, `docs/DESIGN_DECISIONS.md`, `docs/RESULTS.md`,
  `docs/SESSION_LOG.md`, `docs/data/README.md`

**`coco_perception/coco_perception/target_finder.py` is byte-identical**,
and so is everything in `coco_moveit_config`. C2-M4.0 added a path
beside the measured one rather than changing it.

**What C2-M4.1 actually did to them:**

- `docs/data/c2m4_localisation.py` — **run, not written.** `--benchmark`
  produced all 60 placements unmodified. The experiment was not
  redesigned
- `coco_perception/coco_perception/target_pose_node.py` — **the only
  source file changed.** Gained `point_topic`, empty by default;
  `min_range` was decided and **left alone**
- `coco_moveit_config/scripts/grasp_server.py` — **not touched.** It
  never held a hard-coded grasp coordinate: it takes a fresh
  `/approach/target` fix and falls back to `approach_stop_x(colour)`
  = 0.1537, a value derived from the window geometry. C2-M4.1 changed
  **where the fix comes from**, upstream, and left the grasp path that
  M6 measured exactly as it was
- `custom_teleop/custom_teleop/approach_server.py` — **not touched**,
  and it turned out to be the component that makes the lateral result
  come out well: its `align` phase nulls the bearing before the fix is
  taken
- `custom_teleop/custom_teleop/cmd_vel_arbiter.py` — **read, not
  changed.** Sole publisher to the controller topic; verified 1 before
  every C2-M4.1 grasp run

---

## UNRESOLVED QUESTIONS

**Opened by C2-M5.1, and it is the honest end of the milestone:**

0a. **How should a confidently-wrong AMCL be recovered on this map?**
    C2-M5.1 detects the divergence, stops safely, and re-seeds the
    filter, and the mission still cannot get home. Two measured causes,
    neither in the executive: `amcl.recovery_alpha_fast` and
    `recovery_alpha_slow` are **0.0**, so AMCL has no random-particle
    injection and cannot leave a mode it is confident in; and
    `/reinitialize_global_localization` converged to world
    **(2.60, −0.64)** — inside the wedge footprint — because the map's
    2D slice is highly self-similar and a 360° scan from a standing
    robot does not disambiguate it. The planner then reported "Start
    occupied". **Turning the `recovery_alpha_*` pair on is the obvious
    next experiment and is NOT obviously right**: the standing 19/20 was
    measured with them at 0, and changing an AMCL parameter to make one
    mission pass is what NEXT EXACT ACTION has forbidden since C2-M5.0.
    It needs measuring, on both a healthy matrix and the injection.

0b. **Why does detection latency vary by 25×?** The same class-A
    injection measured **3.33 s, 4.52 s and 82.9 s** across three runs.
    Three is not a sample, and until it is larger the latency figure in
    RESULTS.md is a range and not a number.

**Opened by C2-M5.0, and it is a decision somebody has to take:**

00. **Should `cmd_vel_relay`'s arbiter output move off `/cmd_vel_nav`?**
    It is the same topic `nav2_bringup` remaps `controller_server` and
    `velocity_smoother` onto, so the relay feeds the collision monitor's
    output back into the smoother's input and the arbiter receives the
    raw and gated commands together. Measured: the wheels see
    **10.15–10.77 Hz more than the collision monitor publishes**
    (= `controller_frequency: 10.0`), and during an active SLOWDOWN with
    a 0.090 m/s gated cap the wheels were commanded **0.300 m/s** on
    84.2% of `obstacle1`'s slowdown samples.

    **The fix is a topic rename** — give the relay its own output, e.g.
    `/cmd_vel_gated`, and point the arbiter's `nav_topic` at it. It was
    **deliberately not done in C2-M5.0**: the wheel path is frozen
    (`CLAUDE.md` §4), the change alters what the robot is commanded on
    every leg, and the standing **19/20** was measured with the loop in
    place. Whoever changes it owes a measured run and a statement about
    comparability to that baseline. Until then, **C2-M5.1 must not
    assume the collision monitor can stop the robot.**

01. **What is the healthy spread of the scan-vs-map signal?** C2-M5.0
    has two legs that finished and cannot place a threshold in the
    0.054 m gap between the worst of them and the best failure. The
    missing evidence is **more healthy legs**, not more failures. Until
    it exists, `localization_health.Thresholds` stays without defaults.

02. **What caused `healthy2`?** An uninjected RETURN_HOME failure with
    position error inside the healthy band (0.300 m median) and yaw
    error reaching 1.31 rad — measured **not** to be AMCL staleness
    (an upper-bound lag estimate explains 0 of the 55 samples above
    0.5 rad). The plan lengthened 9.73 → 13.93 m and
    `navigate_to_pose` returned ABORTED, with the two retries aborting
    in 0.1 s each. **One run. Not diagnosed.**

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

**Opened by C2-M4.0, and it is one parameter:**

- **Should `min_range` drop from 0.15 to 0.11?** The gate rejects an
  extended target's near face, which sits a full radius in front of its
  axis, so below a ~0.30 m stand-off the reported range is long by an
  amount proportional to the target's radius: **+4.1 to +8.3 mm**
  measured, collapsing to **−1.0 to −1.4 mm** at 0.11. The argument for
  leaving it: it matches `target_finder`, and the approach's last leg is
  blind below `min_range` by construction, so perception's operating
  envelope starts near 0.30 m regardless. The argument for changing it:
  it is a bias with a known mechanism sitting inside a pipeline whose
  whole job is a millimetre-scale number. **Deliberately not decided in
  C2-M4.0** — the control run is in `RESULTS.md` and the decision
  belongs with whoever runs the benchmark. A radius-aware gate is a
  third option and nobody has costed it.
- **Is the vertical estimate worth reporting at all?** `point.z` is
  framing-dependent by construction and no consumer uses it —
  `grasp_point.z` comes from `TARGET_GRASP_Z`. It is currently published
  so the framing effect stays visible. If C2-M4.1 finds nothing reads
  it, that is a case for dropping it rather than for defending it.

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

**C2-M5.0 and C2-M5.1 are both closed. COCO 2.0 is FROZEN — no further
feature milestone is planned.** The one action outstanding belongs to the
owner, not to an agent:

```bash
# Review, then merge. Merging has never been done unasked.
git log --oneline jazzy-harmonic-port..coco2-m1-observability
git checkout jazzy-harmonic-port
git merge --no-ff coco2-m1-observability
git merge --ff-only coco2-state
```

**Read `HOW_TO_RUN.md` first if you only want to run it.** Every command
in it was verified against this tree.

**If you are picking the work up again**, the honest starting point is
the one thing C2-M5.1 could not do: **the recovery restores the health
signal but not a pose Nav2 can plan from**, for a 3 m confidently-wrong
divergence. Two measured causes, neither of them in the executive:

1. `nav2_params.yaml` sets `amcl.recovery_alpha_fast: 0.0` and
   `recovery_alpha_slow: 0.0`, so AMCL's random-particle injection is
   off and the filter cannot leave a mode it is confident in. **Do not
   simply switch them on to make a mission pass** — that is tuning AMCL
   for one run, and the standing 19/20 was measured with them at 0.
   Whether they help is a measurement somebody has to make.
2. `/reinitialize_global_localization` converged to world (2.60, −0.64),
   inside the wedge footprint, on a map whose 2D slice is highly
   self-similar. Evidence in
   `docs/data/c2m51_planner_after_recovery.txt`.

**Do not treat detection latency as settled.** It measured **3.33 s,
4.52 s and 82.9 s** on three runs of the *same* injection. The 0.4 s
C2-M5.0 reported is an offline figure against `healthy1`'s envelope; the
live latency is variable and the sample is three.

**Class B is still not separated**, exactly as C2-M5.0 said. `healthy2`
failed with no injection and its worst gated sample, 0.3091, is under
the threshold.

**The first requirement is the awkward one.** C2-M5.0 measured that
**the collision monitor cannot stop this robot** — the wheels receive
`controller_frequency` worth of ungated commands because `/cmd_vel_nav`
carries both the raw controller output and the relayed, gated one. So a
recovery that begins "stop the robot" must use the arbiter's zero-hold
path and must **prove** the stop at the arbiter, exactly as C2-M3.1
established. Do not design around a `PolygonStop` that is not reaching
the wheels.

**The second is that there is no threshold yet, and inventing one is the
failure mode to avoid.** `localization_health.Thresholds` deliberately
has no defaults. Class A separates at almost any value; class B does not
separate at all, the measured gap on common ground being **0.054 m**.
The experiment that would settle it is more *healthy* legs — the
evidence is short of healthy spread, not of failure examples. Two of the
three failures on record were induced; only `healthy2` was not.

**The concrete first task:** wire `localization_health.py` into a
subscriber that publishes its verdict, with the `on_mapped_ground` gate
and a persistence requirement, and **measure how often it fires on legs
that finish** before anything acts on it. The healthy run's own worst
scan-vs-map samples reach 0.31 m, so a single-sample trigger would have
fired on a mission that went home to 0.078 m.

**Reproducing C2-M5.0:**

```bash
source ~/ros2_ws/c2m31_overlay/env.sh
# T1 — fresh simulator, ALWAYS. Never --fast.
ros2 launch gazebo_models full_world_robo.launch.py traverse:=true gui:=false
# T2
ros2 launch coco_mission mission.launch.py rviz:=false \
    target_source:=target_pose policy:="$COCO_POLICY"
ros2 lifecycle get /amcl                              # active [3]
# T3 — what is actually wired to what, off the LIVE graph
python3 docs/data/c2m5_locrec.py --topology           # 7 pubs on /cmd_vel_nav
# T4 — record, then start
cd docs/data && python3 c2m5_locrec.py --out run.csv --events run_events.txt \
    --tag mytag --hz 10 --map ../../gazebo_models/maps/coco_world.yaml \
    --stop-on-terminal &
ros2 service call /mission/start std_srvs/srv/Trigger
# score it
python3 docs/data/c2m5_analysis.py docs/data/c2m5_*.csv --states --compare
```

**Two traps in the data, both of which cost time in C2-M5.0.**
`/amcl_pose` is in the **map** frame and `gt_*` is Gazebo's **world**
frame; map (0,0) is world (−2, 0), and subtracting them raw makes the
healthy run read as 2.2 m of error on a mission that finished 0.078 m
from home. And **`mo_age` is normally negative** — AMCL post-dates
`map->odom` by `transform_tolerance`, so −0.44 s is the healthy value.

**Do not tune AMCL to make one mission succeed**, and do not turn the
covariance verdict on in `mission_hud`: C2-M5.0 measured that the
threshold `mission_hud` has been withholding since C2-M1 **should stay
withheld**, because the signal points the wrong way.

**The older C2-M4 reproduction notes follow, unchanged.**

**C2-M4.2 gave C2-M5 a second data point on its own benchmark.**
`RETURN_HOME` succeeded in **59.9 s**, the second consecutive success
under light load with RViz off. The tally over recorded legs is now
**three failed, three succeeded — six is not a rate**, and the two
mechanisms KNOWN PROBLEMS 1 names are still un-separated.

**The C2-M4 loose end is closed.** The run that closes it, and the way
to reproduce it — note it is now ONE argument, not a hand-swap:

```bash
# T1 — fresh simulator, ALWAYS. Never --fast.
ros2 launch gazebo_models full_world_robo.launch.py traverse:=true gui:=false

# T2 — the stack, on the C2-M4 perception path. target_source sets BOTH
#      point_topic and status_compat_topic; setting one without the
#      other is the C2-M4.2 defect. rviz:=false — KNOWN PROBLEMS 1 and
#      3b both carry a Gazebo+RViz+move_group confound.
ros2 launch coco_mission mission.launch.py rviz:=false \
    target_source:=target_pose target_colour:=blue \
    policy:=/home/gautham/coco_rl_runs/curriculum_20260726_211008/phase5_24deg_s0p0.zip

# T3 — check the invariants, THEN start. Each has cost a run before.
ros2 topic info -v /perception/target | grep -i 'publisher count'   # must read 1
ros2 topic info -v /perception/status | grep -i 'publisher count'   # must read 1
ros2 lifecycle get /amcl                                           # active [3]
ros2 service call /mission/start std_srvs/srv/Trigger
```

**Do not move `perception.launch.py`'s default to `target_pose` on this
result.** One completed mission is an existence proof. The standing
19/20 was measured on `target_finder`, and moving the default trades a
measured figure for an unmeasured one on a sample of one.

**Reproducing C2-M4.1 exactly:**

```bash
# perception, 60 placements — needs only the sim + the node
cd docs/data && python3 c2m4_localisation.py --benchmark \
    --frames 12 --out c2m4_benchmark.csv
python3 c2m4_analysis.py c2m4_benchmark.csv --plot c2m4_scatter.png

# one live grasp — needs sim + arbiter + move_group + the three servers,
# and a FRESH SIMULATOR each time
python3 c2m4_grasp.py --colour blue --standoff 0.45 --lateral 0.0 \
    --out c2m4_grasp.csv
```

**The two things C2-M4.1 found and did not fix, in priority order.**
Both are in the *verification*, which is the part that decides whether a
reported success is real:

1. **`check_lifted` does not check upright.** A cylinder toppled before
   the grasp is welded, lifted, carried and delivered lying down, and
   every step reports success. Fixing it means deciding what "upright"
   means for a grasp allowed to be imperfect — a design decision, not a
   patch. `arm_control.check_lifted`.
2. **`check_released` asserts the floor height AT HOME.** It is right
   for the M6 mission and wrong anywhere else, and it failed all eight
   C2-M4.1 placements including the seven that released perfectly.
   `grasp_server.check_released`.

**Do not retune `GRASP_MAX_LATERAL` on the 60-placement table.** The
"0 of 20 feasible at −0.010" is the verdict for a robot grasping from
where it stands. The robot drives an approach that **turns to face the
target first**, and measured live that absorbed 10.2 mm to 1.68 mm and
29.2 mm to 3.0 mm — both grasped. `reachability_after_approach` is a
**lower bound on feasibility, not a forecast**, because it credits the
approach with translation and not rotation.

**Do not retune `min_range` either.** C2-M4.1 decided it: no change. At
the 0.30 m operating floor the C2-M4.0 defect is already gone (`qual`
0.9989+ against 0.0423–0.0706 at 0.28 m). The envelope is documented and
**`qual` is the runtime tell that needs no ground truth**.

**Do not average the residual away.** Frame-to-frame spread was
**0.0000 m** on all 60 C2-M4.1 placements. The residual is bias, and
the simulated depth camera is noiseless, so a filter would be tuned
against a sensor that does not exist.

**And remember the services are asynchronous.** `/approach/run`,
`/grasp/stow`, `/grasp/pick` and `/grasp/place` all return
`success=True` immediately with "watch /<name>/status". That reply is
the **acceptance, not the outcome**. It cost C2-M4.1 a run.

**Before any of it, the environment gate.** The workspace checkout at
`~/ros2_ws/src/coco-robot-ros2` is on the **trunk**, which does not
contain `coco_mission` at all, so `~/ros2_ws/install` cannot run any of
this. C2-M3.1 built the feature worktree into a separate overlay and
left the user's workspace alone; C2-M4.0 and C2-M4.1 both used it
unchanged:

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

0. **`HOW_TO_RUN.md`** (repo root, on `coco2-m1-observability`) — every
   verified command: requirements, build, the overlay, launching the
   simulator and the stack, RViz, a normal mission, the terrain,
   perception and localization-recovery demonstrations, the test suite,
   cleanup, and a troubleshooting list containing only faults that were
   actually observed. **If you only want to run the robot, stop here.**
0b. `coco_mission/scripts/localization_health.py` — **the C2-M5.1 block
   at the foot of the file before anything else.** It is where every
   threshold is named together with the replay that justifies it, why
   `lik_frac_near` ships disabled, why `max_amcl_age` is `None`, and why
   `Persistence` accumulates instead of requiring continuity. Each is a
   correction to a first guess, with the run that forced it.
0c. `coco_mission/scripts/localization_monitor.py` — the node. Its
   docstring says what it refuses to read as much as what it reads.
0d. `docs/RESULTS.md`, section **"C2-M5.1 localization recovery"** — the
   four experiments, and the explicit split between DEMONSTRATED,
   UNIT-TESTED ONLY and KNOWN LIMITATION. **Read "What C2-M5.1
   establishes, and what it does not" before concluding anything.**
0e. `docs/data/c2m51_planner_after_recovery.txt` — the Nav2 logs showing
   why the mission does not resume after a class-A recovery. Short.

1. `PROJECT_STATE.md` (this file)
1b. `docs/RESULTS.md`, section **"C2-M5.0 localization health"** — the
   five runs, the failure taxonomy, the healthy and bad profiles, the
   collision-monitor profile, the control-loop numbers, the proposed
   health signal, **the threshold that was deliberately not picked**,
   and the recovery requirements C2-M5.1 inherits. Read "The verdict,
   including what it does not support" before concluding anything
1c. `coco_mission/scripts/localization_health.py` — **its module
   docstring before its code.** It is where the health signal is
   defined, why it is not covariance, and why `Thresholds` has no
   defaults. Imported by nothing, by design
1d. `coco_mission/test/test_localization_health.py` — what the milestone
   promised, written as assertions. If a future change makes one of
   these fail, the verdict needs revisiting, not the test
1e. `docs/data/c2m5_locrec.py` — the recorder, and specifically its
   **ground-truth boundary** and the `map` vs `world` frame note. The
   five CSVs beside it are the raw evidence; `c2m5_topology.txt` is the
   live-graph proof of the `/cmd_vel_nav` loop
1f. `docs/DESIGN_DECISIONS.md`, the **four C2-M5.0 entries** at the tail
2. `docs/ROADMAP.md` — the **C2-M5** block, which is the current
   milestone, and the **C2-M4** block above it, now closed
2b. `docs/RESULTS.md`, section **"C2-M4.1 the four-colour benchmark"** —
   the 60 placements, the 8 live grasps, the `min_range` decision, and
   the explicit list of what C2-M4.1 does *not* establish. **Read "the
   static verdict is a lower bound" before you conclude anything from
   the 0-of-20 row**
2c. `coco_perception/coco_perception/target_pose.py` — **its module
   docstring before its code.** It is where "the target's position"
   is given a meaning; comparing any number without it is a mistake.
   Note that `reachability_after_approach`'s docstring models the
   approach as translation-only, and C2-M4.1 measured that the real
   approach also **turns**
2d. `coco_perception/coco_perception/target_pose_node.py` — the
   `point_topic` **and `status_compat_topic`** docstrings. Together they
   are the whole integration, and the C2-M4.2 paragraph says why
   `point_topic` alone is not enough: the executive's `SEARCH_TARGET`
   gate reads `/perception/status`, not the point topic
2d-ii. `coco_perception/launch/perception.launch.py` — the
   `target_source` docstring. Why the selection is an `OpaqueFunction`
   and not two `IfCondition`s, and why exactly one node may own
   `/perception/target`
2d-iii. `docs/RESULTS.md`, section **"C2-M4.2 the integration gate"**,
   and `docs/data/c2m42_mission.log` — the one mission that ran on the
   new path, and the explicit list of what one run does **not**
   establish
2e. `docs/data/c2m4_localisation.py` (the benchmark runner and its
   ground-truth boundary), `docs/data/c2m4_grasp.py` (one grasp, one
   fresh simulator), `docs/data/c2m4_analysis.py` (post-processing)
2f. `docs/DESIGN_DECISIONS.md`, the **C2-M4.1** section at the tail
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
- The target bay geometry. **`coco_perception/target_finder.py` is
  frozen** — `/perception/target` feeds `approach_server`'s servo
  mode and is the path M6's 20/20 approach was measured through, so
  C2-M4.0 added `target_pose.py` / `target_pose_node.py` beside it
  rather than editing it. A test
  (`test_optical_to_base_matches_the_urdf_chain`) pins its hard-coded
  extrinsics against the xacro so the two frame paths cannot drift
  while both exist. New files in the package are fine; changing that
  one needs an explicit instruction
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
