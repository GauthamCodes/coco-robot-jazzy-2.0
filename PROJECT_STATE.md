# PROJECT_STATE.md

**Authoritative snapshot. A fresh agent reads this first.**
Lives on the trunk (`jazzy-harmonic-port`) and is edited **only** there —
see `docs/STATE_PROTOCOL.md`.

Last updated: 2026-08-17, after C2-M1.5.

---

## BRANCH MAP — READ BEFORE `git status` CONFUSES YOU

The trunk does **not** contain all completed work. This table is what
makes a trunk-only state file honest.

| Branch | Contains | Merged? |
|---|---|---|
| `jazzy-harmonic-port` | **the trunk.** Everything through M7 Phase 3, plus this state layer | — |
| `coco2-m1-observability` | **C2-M1 and C2-M1.5 complete**: `mission_hud`, `mission.rviz`, `/mission/state`, `pitch_probe.py`, the pitch fix, 67 tests | **NO — unmerged** |
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
| **C2-M1 … C2-M9** | The **COCO 2.0** plan. The active track. | C2-M1 + C2-M1.5 done (unmerged) |

"M2" is ambiguous. **Always write `C2-M2` for the COCO 2.0 plan** and
plain `M2` only for the historical v1 milestone.

---

## CURRENT MILESTONE

**C2-M2 — Terrain control experiment.** Not started. **Gate cleared.**

## CURRENT STATUS

**C2-M1.5 (runtime integrity gate) is COMPLETE**, and its verdict is that
**C2-M2 is READY to begin**. The signal C2-M2's grade estimator would
have been built on was measured, found to be stale, and fixed.

## CURRENT OBJECTIVE

Finish the terrain-control research before adding any RL:
tip-termination correction, classical baseline re-evaluation, a grade
estimator, a friction estimator, an observer-driven controller.

**Decision rule, fixed in advance:** expand RL *only* if the
observer-driven controller stays **more than 10 percentage points below**
the privileged controller on a measured task. If the observer closes the
gap, **that is the successful result** and RL is not added.

## MILESTONE STATUS

- **C2-M1 (observability): COMPLETE and verified** — branch
  `coco2-m1-observability`, not merged.
- **C2-M1.5 (runtime integrity gate): COMPLETE** — same branch, commit
  `1c99415`, pushed. See COMPLETED / NOT COMPLETED below.
- **C2-M2 (terrain control): NOT STARTED.** Current milestone, unblocked.
- C2-M3…C2-M9: not started. See `docs/ROADMAP.md`.

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

4. **The user's `~/ros2_ws` may still have a stale `coco_sim` build.**
   Signature: 29 `coco_rl` tests failing with `FileNotFoundError` on
   `build/coco_sim/worlds/yard_params.yaml`. Fix, **measured**:
   ```bash
   cd ~/ros2_ws && colcon build --packages-select coco_sim
   ```
   Not seen this session — the branch's overlay build gives 106/0 → now
   109/0.

5. **Run pytest from inside each package directory.** From the repo root
   the `coco_rl/` *directory* shadows the installed module.

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
| **Last verified commit** | `1c99415` — *fix(mission): ROBOT PITCH was a fossil, and it would have passed C2-M2* — on `coco2-m1-observability`, **pushed to `jazzy2`** |
| Trunk head | `6c06c45`, pushed to `origin/jazzy-harmonic-port` |

---

## TESTS RUN (2026-08-17)

Per package, **cwd set to the package directory**, against the branch's
overlay build. Run before *and* after the changes.

| package | before | after |
|---|---|---|
| `coco_config` | 70 | 70 |
| `custom_teleop` | 67 | 67 |
| `coco_rl` | 106 | **109** |
| `coco_perception` | 44 | 44 |
| `gazebo_models` | 20 | 20 |
| `coco_moveit_config` | 12 | 12 |
| `coco_sim` | 55 | 55 |
| `coco_mission` | 30 | **37** |
| **total** | **404** | **414** |

Zero failing, both times. On the **trunk**, `coco_mission` does not exist
yet, so the trunk total is **374**; the rest arrive with the merge.

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

---

## CURRENT FILES

Nothing is mid-edit. Changed by C2-M1.5:

- `coco_rl/coco_rl/ramp_driver.py` — `pitch=--` while idle, `live_pitch()`
- `coco_mission/scripts/mission_hud.py` — `ROBOT PITCH` from `/imu`
- `gazebo_models/scripts/pitch_probe.py` — **new**, the instrument
- `gazebo_models/rviz/mission.rviz` — view framing
- `gazebo_models/CMakeLists.txt`, `gazebo_models/scripts/ros_clean.sh`
- `coco_mission/package.xml`, and the two test files

The files C2-M2 will touch first:

- `coco_rl/coco_rl/baselines.py` — B0/B1/B2 and the reference path
- `coco_rl/coco_rl/baseline_eval.py` — the runner and failure taxonomy
- `coco_rl/coco_rl/ramp_env.py` — where `TIP_LIMIT` is consumed
- `coco_config/` — wherever `TIP_LIMIT` is defined (shared)
- `docs/M7_DESIGN.md`, `docs/M7_PHASES.md` — the spec and phase blocks

---

## UNRESOLVED QUESTIONS

**Three open decisions gate M7 Phase 4** (carried forward, unchanged):

1. **Deck convergence geometry** — 1.95 m lateral shift in 1.80 m of
   travel before a 0.65 m bridge, against a 0.40 m turn radius. B1
   reaches the deck 99% of the time then falls off the bridge 105 times
   in 120. **Nothing changed.**
2. **Route B viability** — best success 8%; **39.3% of episodes have
   mu < tan(grade) and are physically unclimbable**. Four options costed
   in `RESULTS.md`. **None chosen.**
3. **Route C's tip terminator** — 101/120 tips are pitch events, 0 of 101
   roll-dominated. `TIP_LIMIT` is 0.6 rad **absolute**; the 16.3 deg
   grade consumes 16.3 of it while true static rear-over is **54.5 deg**.
   **Instrumentation, not control.** Fix not applied because `TIP_LIMIT`
   is shared with `ramp_env`, the v1 curriculum and the shipped policy.

**C2-M2 begins with decision 3** — "terrain tip-termination correction"
is the same problem.

---

## NEXT EXACT ACTION

**Take the Route C tip-terminator decision** (unresolved question 3).
It is C2-M2's first item and M7 Phase 4's third gate, it is the same
problem stated twice, and it needs a decision rather than a measurement.
The relevant evidence is already in `docs/RESULTS.md` (M7 Phase 3
close-out): `TIP_LIMIT = 0.6` rad is absolute, the 16.3° grade eats 16.3°
of it, and true static rear-over is 54.5°.

Read `coco_config` for where `TIP_LIMIT` lives and `coco_rl/ramp_env.py`
for how it is consumed before proposing anything — it is shared with the
v1 curriculum and the shipped policy, which is why the fix was not
applied when it was found.

Then C2-M2 proper. **Use `/imu` for attitude, not `/ramp/status`** — that
is the whole point of C2-M1.5 — and `pitch_probe.py` is already the
instrument for the grade/friction comparison:

```bash
ros2 run gazebo_models pitch_probe.py --out /tmp/pitch.csv --hz 10
```

---

## FILES TO READ FIRST IN THE NEXT SESSION

1. `PROJECT_STATE.md` (this file)
2. `docs/ROADMAP.md` — the C2-M2 completion criteria and decision rule
3. `docs/STATE_PROTOCOL.md` — which branch owns which file
4. `docs/SESSION_LOG.md`, the **2026-08-17 C2-M1.5** entry at the tail
5. `docs/RESULTS.md`, section **"C2-M1.5 runtime integrity"** — every
   measured number behind the claims above
6. `docs/DESIGN_DECISIONS.md`, the two entries at the tail — the stale
   -field lesson, and why `/approach/target` was left alone
7. `docs/RESULTS.md`, M7 Phase 3 close-out — the evidence for the Route C
   decision that is next
8. `coco_config/coco_config/robot.py` and `coco_rl/coco_rl/ramp_env.py`
   — `TIP_LIMIT` and `RAMP_ANGLE_DEG`

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
  `base_footprint` fixed frame is correct *there*.
- The training environment (`coco_rl/coco_rl/mujoco_env.py` and
  everything it touches) **must never import `rclpy`**.
- **`/approach/target` stays VOLATILE.** Investigated in C2-M1.5 and the
  current semantics are correct; see `docs/DESIGN_DECISIONS.md`.

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
