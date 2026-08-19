# PROJECT_STATE.md

**Authoritative snapshot. A fresh agent reads this first.**
Lives on the trunk (`jazzy-harmonic-port`) and is edited **only** there —
see `docs/STATE_PROTOCOL.md`.

Last updated: 2026-08-19, after **C2-M2.1 — the whole C2-M2 phase is
CLOSED.**

---

## BRANCH MAP — READ BEFORE `git status` CONFUSES YOU

The trunk does **not** contain all completed work. This table is what
makes a trunk-only state file honest.

| Branch | Contains | Merged? |
|---|---|---|
| `jazzy-harmonic-port` | **the trunk.** Everything through M7 Phase 3, plus this state layer | — |
| `coco2-m1-observability` | **C2-M1, C2-M1.5, C2-M1.6, C2-M2.0 and C2-M2.1 complete.** `mission_hud`, `/mission/state`, `pitch_probe.py`, the pitch fix, both RViz views + the map audit, **the terrain observer, B3, the ROS node, and the 1,440-episode benchmark with its verdict** | **NO — unmerged** |
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
| **C2-M1 … C2-M9** | The **COCO 2.0** plan. The active track. | C2-M1 through **C2-M2 done** (unmerged) |

"M2" is ambiguous. **Always write `C2-M2` for the COCO 2.0 plan** and
plain `M2` only for the historical v1 milestone.

---

## CURRENT MILESTONE

**C2-M3 — Real mission executive.** Not started. **Gate cleared: C2-M2
is CLOSED, in both its sessions.**

## CURRENT STATUS

**C2-M2 is COMPLETE and the terrain-control question has its measured
answer.** C2-M2.0 built the observer and froze the experiment; C2-M2.1
validated it live, ran the benchmark and applied the rule.

**The benchmark: 1,440 intended, 1,440 completed, 0 runner errors.**
B0/B1/B2/B3 × routes A/B/C × seeds 0–119.

**The rule, applied unchanged** (task `ascent`, margin 10 pp, both fixed
before any result existed):

| route | B2 privileged | B3 observer | gap |
|---|---|---|---|
| A | 99.2 % | 99.2 % | **+0.0 pp** |
| B | 34.2 % | 32.5 % | **+1.7 pp** |
| C | 65.8 % | 58.3 % | **+7.5 pp** |

**RL is justified on 0 of 3 routes. Additional learned control is NOT
justified by this benchmark.**

**READ THE NEXT PARAGRAPH BEFORE QUOTING THAT VERDICT.** B3 ≈ B2 on
ascent is a statement about the **task**, not about the estimator. On
Route A, B3 fell back on **120 of 120** episodes and is byte-identical to
B1 — tan(12°) = 0.213 is below the 0.35 a-priori friction floor, so the
bound can never become informative and the observer correctly refuses to
schedule on an assumption. **It recovered nothing.** Meanwhile B2
**completed 97.5 % of Route A against B3's 0.0 %**. The ascent gap is
0.0 pp because ascent does not discriminate there, not because
estimation succeeded. The rule was applied as frozen and the evidence
against its own premise is recorded beside the verdict, in
`RESULTS.md` and `DESIGN_DECISIONS.md`.

**Friction remains not identifiable, now confirmed at scale.**
τ − tan(grade) is **−0.0012 / −0.0034 / +0.0043** over 1,440 episodes.
Nothing in this repo reports a friction estimate, and **nothing should
be added that does** without new instrumentation to justify it.

## CURRENT OBJECTIVE

**C2-M3 — turn `traverse_demo.py` (a blocking script) into a real state
machine** with entry condition, action, success condition, timeout,
failure condition, diagnostics and recovery **per state**. See
`docs/ROADMAP.md`.

`/mission/state` already exists from C2-M1 but only reports a blocking
script's step label. **That is a stepping stone, not a substitute** — no
state has an entry condition, a timeout or a recovery.

## MILESTONE STATUS

- **C2-M1 (observability): COMPLETE and verified** — branch
  `coco2-m1-observability`, not merged.
- **C2-M1.5 (runtime integrity gate): COMPLETE** — same branch, commit
  `1c99415`, pushed.
- **C2-M1.6 (RViz presentation): COMPLETE** — same branch, commit
  `a7bfc23`, pushed. Presentation only; **map quality classified GOOD**.
- **C2-M2.0 (terrain observer): COMPLETE** — same branch, commit
  `1aa6670`, pushed. Grade observable, friction not; Route C terminator
  made surface-relative; benchmark frozen.
- **C2-M2.1 (benchmark + verdict): COMPLETE** — same branch, commit
  `7796d04`, pushed. **C2-M2 is closed.**
- **C2-M3 (mission executive): NOT STARTED.** Current milestone,
  unblocked.
- C2-M4…C2-M9: not started. See `docs/ROADMAP.md`.

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
   benchmark.

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
   it as a defect.

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
| **Last verified commit** | `7796d04` — *C2-M2.1: the benchmark ran, and the bar is the result* — on `coco2-m1-observability`, **pushed to `jazzy2`** |
| Previous checkpoint | `1aa6670` — *C2-M2.0: grade is observable, friction is not* |
| Trunk head | `6c06c45`, pushed to `origin/jazzy-harmonic-port` |

---

## TESTS RUN (2026-08-19)

Per package, **cwd set to the package directory**, against the branch's
overlay build. Run before *and* after the changes, every milestone.

| package | C2-M1.6 | C2-M2.0 | **C2-M2.1 after** |
|---|---|---|---|
| `coco_config` | 70 | 70 | 70 |
| `custom_teleop` | 67 | 67 | 67 |
| `coco_rl` | 109 | **152** | **164** |
| `coco_perception` | 44 | 44 | 44 |
| `gazebo_models` | 41 | 41 | 41 |
| `coco_moveit_config` | 12 | 5 (+7 skipped) | 12 |
| `coco_sim` | 55 | 55 | 55 |
| `coco_mission` | 37 | 37 | 37 |
| **total** | **435** | **471** | **490** |

Zero failing, every time. On the **trunk**, `coco_mission` does not exist
yet; the rest arrive with the merge.

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

## EXPERIMENTS RUN (2026-08-17)

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

The files C2-M3 will touch first:

- `gazebo_models/scripts/traverse_demo.py` — the blocking script being
  replaced
- `coco_mission/` — where the executive belongs (CLAUDE.md §6: anything
  composing `move_group` lives here, not in `gazebo_models`)
- `custom_teleop/custom_teleop/cmd_vel_arbiter.py` — **read, do not
  change.** Sole publisher to the controller topic
- `coco_mission/scripts/mission_hud.py` — `/mission/state`'s current
  producer
- `gazebo_models/scripts/ros_clean.sh` — **anything added to a launch
  file must get a pattern here**

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

**Begin C2-M3 — the mission executive — by reading `docs/ROADMAP.md`'s
C2-M3 block, NOT by editing `traverse_demo.py`.**

The milestone is states with an entry condition, an action, a success
condition, a timeout, a failure condition, diagnostics and recovery —
each expressed as a ROS action/service/event rather than a step in one
blocking script. `/mission/state` (C2-M1) already publishes a step label
and is a **stepping stone, not a substitute**: no state it reports has an
entry condition, a timeout or a recovery.

The existing arbiter architecture is to be **preserved**:
`cmd_vel_arbiter` stays the sole publisher to the controller topic. That
invariant was re-measured live in C2-M2.1 (publisher count 1, before and
after adding a node) and it must survive C2-M3.

Two known problems below are C2-M3/C2-M5 material and are the natural
first targets: **nav home has failed in 2 of 4 recorded legs by two
distinct mechanisms** (KNOWN PROBLEMS 1), and **the scripted descent timed
out in both C2-M1.6 traverse runs** (KNOWN PROBLEMS 3b) with a
control-loop confound that was never isolated. Neither is a rate — four
runs and two runs respectively — and the standing figure is M6's 19/20.

```bash
# watch the mission while working on the executive
ros2 launch coco_mission mission.launch.py policy:=<zip>                       # clean
ros2 launch coco_mission mission.launch.py policy:=<zip> rviz_config:=mission_debug

# C2-M2 is closed, but its artifacts re-report without re-running anything
python3 -m coco_rl.terrain_benchmark --report docs/data/c2m2_benchmark.json
python3 docs/data/c2m2_analysis.py
```

**Do not re-open the friction question.** It is settled and measured
twice over: τ − tan(grade) is −0.0012 / −0.0034 / +0.0043 across 1,440
episodes in MuJoCo, and 0.3248 vs tan(18°)=0.3249 live in Gazebo. Read
`DESIGN_DECISIONS.md`, "What a robot can know about the ground it is on",
before building anything that claims to estimate friction — including the
two formulations that were wrong in ways that *looked like* the result
being sought.

---

## FILES TO READ FIRST IN THE NEXT SESSION

1. `PROJECT_STATE.md` (this file)
2. `docs/ROADMAP.md` — the **C2-M3** block, which is the current milestone
3. `docs/STATE_PROTOCOL.md` — which branch owns which file
4. `docs/SESSION_LOG.md`, the **2026-08-19 C2-M2.1** entry at the tail,
   and the **C2-M2.0** entry immediately before it
5. `docs/RESULTS.md`, section **"C2-M2.1 the terrain benchmark"** — every
   measured number behind the verdict above, including the caveat that
   the verdict must be read with
6. `docs/DESIGN_DECISIONS.md`, the **three** entries at the tail — what a
   robot can know about the ground it is on; why an estimator runs on the
   sensor's clock; and why the decision rule was not moved after the
   result arrived
7. `docs/data/c2m2_benchmark.json` is the raw 1,440 episodes;
   `c2m2_analysis.py` and `c2m2_plots.py` re-report it without re-running
   anything
8. For C2-M3 specifically: `coco_mission/` and `gazebo_models/scripts/
   traverse_demo.py` (the blocking script being replaced), plus
   `custom_teleop/custom_teleop/cmd_vel_arbiter.py` — the invariant that
   must survive

---

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
