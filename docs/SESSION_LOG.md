# Session log

One entry per working session, newest at the bottom. Append, never rewrite.

The point of this file is that a session ending mid-phase should be resumable
by someone with no memory of it — including you, three weeks later. The "next
command" line is the most important line in every entry; if it is vague the
entry has failed.

**Format:**

```
## YYYY-MM-DD — <phase>, <one-line summary>

**Built:**      what changed, by file or package
**Measured:**   numbers produced from runs IN THIS SESSION only
**Unverified:** written but not observed working
**Open:**       questions, blockers, decisions deferred
**Next:**       the exact command to run
```

`Measured` and `Unverified` are separate fields on purpose. Anything that has
not been observed working goes in `Unverified`, no matter how confident the
code looks. M6 is currently open precisely because a fix was written into the
approach window without a run behind it.

---

## 2026-08-06 — state at the start of M7

**Built:**
M0–M6 complete as source. Eight packages. `coco_mission` composes the full
stack. `traverse_demo.py` sequences the seven-step fetch.

**Measured:**
- M0: sim RTF ≈ 1.0, every sensor at nominal rate, in sim time
- M2: Nav2 + SmacPlanner2D, 10/10 goals, mean 34.7 s, 36.3 m, home to 12 cm,
  paths 6.2 % shorter than Dijkstra
- M3: `arm_ik` 20,000/20,000 round trips, max error 1.7e-16 m, 1.5 µs/solve;
  MoveIt pick-and-place 4/4 at the tuned target
- M3: `--target` re-targeting 5/14 with the magnet grasp; failures split
  cleanly on x, every point ≥ 0.1505 completes, every point ≤ 0.1468 rejected
- M4: five-stage curriculum, 10/10 deterministic at both 18° and 24°,
  126–127 steps, returns 69.5–69.9; re-verified 10/10 after ramp rebuild
  without retraining
- M4: `--fast` A/B, same seed and config — with: 531/533 tipped, eval 0/10;
  without: 0/533 tipped, eval 10/10, and faster (8.7 vs 8.2 steps/s)
- M5: perception 16/16 lane × station cells within ±2 mm vs `gz model -p`
- M6: bare policy at yaw 0 drifts +0.03 m over 2.5 m in every lane
- M6: `lateral_hold` at K_Y 3.0 / K_YAW 2.5 takes worst-case drift to
  0.053 m, 8/8 summits, no retraining
- Tests: 250, 0 failures
- Training throughput ceiling: ~8.6–8.7 env-steps/s

**Unverified:**
- **M6 end-to-end fetch has never completed.** Best run reached step 4 and
  failed at grasp approach, stopping at base-x 0.1443 — inside the measured
  self-collision bound of 0.150.
- The corrected approach window `[0.1510, 0.1565]` is **written and unit
  tested but never run in simulation**.
- CI workflow and Dockerfile have never executed (no Docker or runner on
  this machine).

**Open:**
- ~111 commits unpushed; `origin` has only `main`.
- `FUTURE_WORK.md` 9(b): the 12° full-distance stage evaluates 0/10 alone —
  a greedy stall at 4.34 m, reproducible to within 0.02 of return.
  `MIN_LIN = 0.15` sitting between a 0.10 m/s timeout and a 0.17 m/s finish
  is the leading suspect. Unexplained.
- `gazebo_models/scripts/` and `coco_moveit_config/scripts/` have no linters;
  ~118 docstring and import-order findings remain.

**Next:**
Phase 0. One blue fetch on the v1 world, fresh simulator:

```bash
ros2 launch gazebo_models full_world_robo.launch.py traverse:=true gui:=false
ros2 launch coco_mission mission.launch.py \
    policy:=~/coco_rl_runs/curriculum_20260726_211008/phase5_24deg_s0p0.zip
ros2 run gazebo_models traverse_demo.py --colour blue
```

Report the base-x the creep phase achieved and whether `/grasp/pick` planned.

---

## 2026-08-06 — Phase 0, M6 closes: the fetch completes end to end

**Built:**
No source changes. Docs only: the end-to-end result in `docs/RESULTS.md`,
the `policy:=` command corrected to an absolute path in `docs/RUNNING.md`,
the M6 row in `README.md`, and this entry.

**Measured** (all from one run in this session, v1 wedge world, fresh
simulator, `gui:=false`, never `--fast`):
- **`FETCH COMPLETE — blue delivered`.** All seven steps, 230.9 s from the
  first log line to the last, home to within 0.06 m of the start.
- **base-x 0.1544** reported by `approach_server` against the window
  `[0.1510, 0.1565]` — +3.4 mm above the near bound, −2.1 mm below the far
  one, +0.7 mm off the 0.15375 centre. The previous attempt's 0.1443 was
  5.7 mm *below* `GRASP_SELF_COLLISION_X`; this is 4.8 mm above it.
- **base-x 0.1548 by Gazebo ground truth** — robot at world
  (3.89573, 0.26346) yaw −0.08396, `target_blue` at (4.049990, 0.250000),
  giving (0.1548, −0.0005) in `base_footprint`. Agrees with the
  dead-reckoned estimate to 0.45 mm in x, 0.48 mm in y.
- **`/grasp/pick` planned and held**: `outcome=held`, `lifted=1`, grasp
  `[0.2728, 0.5052]`, hover `[-0.1054, 0.2935]`.
- **Lift 34.8 mm** (z 0.7288 → 0.7636), read from Gazebo. Place confirmed
  at z 0.0790, `target_blue` ending at world (−1.909110, −0.054373).
- Climb `outcome=goal`, 60 steps, progress 4.72, **lateral +0.09** with the
  lane hold on. Descent `outcome=goal`, 322 steps, progress 6.65.
- Arbiter trace `idle → nav → rl → idle → approach → idle → rl → nav →
  idle`, no double-publisher warning at any point.
- Tests: **250, 0 failures, 0 skipped** (57/67/50/44/20/12). A bare
  `colcon test-result` said 266 — the stale-XML inflation already noted in
  RESULTS.md; the per-package current files sum to 250.
- Bringup gates all passed first time: `verify_sim.py` all checks passed,
  four controllers active, all 4 magnets released, `bt_navigator` active.

**Unverified:**
- **Repeatability. This is 1/1 for blue, not a success rate.** No colour
  other than blue has been driven end to end, and no run has been repeated.
- CI workflow and Dockerfile still have never executed here.
- No video recorded — the run was headless. Still open from the M7_DESIGN
  precondition list.

**Open:**
- `ramp_driver` has **no `os.path.expanduser`** on its `model` parameter,
  and bash does not tilde-expand after `:=`. The documented
  `policy:=~/coco_rl_runs/...` (see the previous entry's Next block, left
  intact as the historical record) reaches `PPO.load` as a literal `~` and
  raises inside the climb worker, surfacing as a failed `/ramp/climb`. The
  docs now use the absolute path; the one-line code fix is NOT done.
- Untracked and therefore unpushed: `CLAUDE.md`, `docs/M7_DESIGN.md`,
  `docs/M7_PHASES.md`, `docs/README_BANNER_snippet.md`. They exist only on
  this machine. Committing them is a call for the repo owner.
- `FUTURE_WORK.md` 9(b) unchanged: 12° full-distance evaluates 0/10 alone,
  a reproducible greedy stall at 4.34 m.
- `gazebo_models/scripts/` and `coco_moveit_config/scripts/` still unlinted.

**Next:**
M6 is closed, so M7 is unblocked. Phase 1 of `docs/M7_PHASES.md` — the
MuJoCo throughput baseline. The figure to beat is 8.7 env-steps/s, and the
instruction is to stop and report if MuJoCo is not meaningfully faster.

Before anything else, re-read that block. Then:

```bash
source ~/ros2_ws/src/coco-robot-ros2/setup_env.sh
cd ~/ros2_ws && colcon test --packages-select coco_rl && colcon test-result
```

To re-run the M6 fetch instead (fresh simulator, absolute policy path):

```bash
bash ~/ros2_ws/src/coco-robot-ros2/gazebo_models/scripts/ros_clean.sh
ros2 launch gazebo_models full_world_robo.launch.py traverse:=true gui:=false
ros2 launch coco_mission mission.launch.py \
    policy:=/home/gautham/coco_rl_runs/curriculum_20260726_211008/phase5_24deg_s0p0.zip
ros2 run gazebo_models traverse_demo.py --colour blue
```

---

## 2026-08-06 — Phase 0.5, M6 consolidated: 19/20, and the drift explained

**Built:**
No source changes; docs only. Phase 0.5 block added to `M7_PHASES.md`;
`CLAUDE.md` and `M7_DESIGN.md` brought into the tree; `README_BANNER_snippet.md`
deleted; `FUTURE_WORK.md` 7b and 8b added. The 20-run harness, the
ground-truth/AMCL logger and the analysers live in the job scratch dir,
deliberately outside the repo.

**Measured** (20 runs, 5 per colour, fresh simulator each, headless, never
`--fast`):
- **19/20 complete.** red 5/5, green 5/5, yellow 5/5, blue 4/5.
- **Approach: 20/20 inside `[0.1510, 0.1565]`.** Ground truth 0.1534–0.1556,
  mean 0.1543, **sd 0.6 mm**. Reported 0.1530–0.1547. |truth − reported|
  0.05–0.92 mm, mean 0.53 mm.
- **Grasp held 20/20**, lift 33.9–35.9 mm from Gazebo ground truth.
- Vision confirmed the requested colour **20/20**.
- Run durations 116.7–322.5 s.
- **Drift at summit** +0.020 to +0.280 m, mean +0.081, sd 0.072. Exceeds the
  documented 0.053 m worst case in **9/20**; max is **5.3×** it. Positive in
  all 20 runs. Per lane: red +0.034, green +0.058, blue +0.086, yellow +0.146.
- **Entry heading at `/ramp/climb`** |yaw| 0.104–0.472 rad, mean 0.290,
  **outside Nav2's `yaw_goal_tolerance: 0.25` in 14/20** — settled, not
  transient (Δyaw over the next second = 0.0000 in all 20; stationary for the
  prior 2 s in 12 of the 14).
- **AMCL is not the cause**: |ground-truth yaw − AMCL yaw| 0.006–0.165 rad,
  mean 0.076; the two disagree about tolerance compliance in 1 run of 20.
- Drift vs entry heading: Pearson **r = +0.565** (r² = 0.32); fit
  drift = 0.132·yaw₀ + 0.073.
- **Lane offset at the summit** −0.012 to +0.301 m. Two runs finished more
  than a half-lane off centre: run 16 at y +1.0512 (**0.199 m from the
  platform edge**), run 19 at y +0.5041 (nearer yellow's lane than blue's,
  and colour-based selection is what kept it correct).
- **AMCL gap at descent end** 0.119–1.183 m, mean 0.378. Every run ≤ 0.470 m
  drove home; the single 1.183 m run did not.
- Tests: **250, 0 failures, 0 skipped** — unchanged.

**Unverified:**
- **Nothing is pushed.** `gh auth status` still reports no host despite the
  task stating `gh auth login` had been run; `~/.config/gh` does not exist.
- **No video.** `ffmpeg` is not installed and `sudo` needs a password.
- The mechanism behind the yaw-tolerance breach (FUTURE_WORK 8b) — candidates
  listed, none tested.
- The residual +y drift bias — observed in all 20 runs, unexplained.

**Open:**
- Two blockers above, both needing the operator: `gh auth login`, and
  `sudo apt install ffmpeg`.
- FUTURE_WORK 7b: 1/20 of the mission is lost to the deliberately unmapped
  corridor, after a successful pick. Mapping it is probably cheaper than
  tuning AMCL.
- `ramp_driver` still has no `os.path.expanduser` on its `model` parameter.
- The lane-hold gains were deliberately NOT retuned.

**Next:**
Land the two blocked deliverables, in this order:

```bash
gh auth login                                  # operator
git -C ~/ros2_ws/src/coco-robot-ros2 push -u origin jazzy-harmonic-port
git -C ~/ros2_ws/src/coco-robot-ros2 ls-remote --heads origin
```

Then the video (needs `sudo apt install ffmpeg`), then M7 Phase 1 — the
MuJoCo throughput baseline in `docs/M7_PHASES.md`. The figure to beat is
8.7 env-steps/s, and the instruction there is to stop and report if MuJoCo
is not meaningfully faster.

---

## 2026-08-07 — Phase 0.6: pushed, cross-track fixed, the +y bias is the policy

**Built:**
First source change of these phases, and it is confined to reporting:
`ramp_driver` now publishes `lateral` as signed distance from the **target
lane centreline** and keeps the old quantity as `disp`. It takes ground
truth from `/model/coco/odometry` and the lane from
`/mission/target_colour` via `coco_config`'s colour→lane table, with a
`lane_y` parameter for standalone runs. **`ramp_env` is untouched** —
`obs[1]` is a policy input and redefining it would have broken the shipped
policy and every number measured against it. `lateral_hold`'s control
input is unchanged and the gains were not retuned.

Also: `docs/RESULTS.md` and `docs/FUTURE_WORK.md` 7b/8b/9(a) amended;
diagnostics and the fetch-matrix harness live in the job scratch dir,
outside the repo.

**Measured** (this session):
- **Cross-track, recomputed over the 20 logged runs — no new simulation.**
  Recomputed `disp` reproduces the logged `lateral` to **0.0050 m**, which
  is the status line's own 2 dp quantisation, so the recomputation is
  sound. `disp` mean +0.0814 / max +0.2793; **cross-track mean +0.1203 /
  max +0.3012**. Mean error was understated by 0.039 m (~48 %).
- **The old metric ranked the lanes backwards.** By `disp`: red +0.0344
  (best) → yellow +0.1472. By cross-track: blue +0.0592 (best), red
  **+0.1249** (second worst), yellow +0.2099. Red arrives +0.127 m
  off-lane and then barely drifts.
- **The +y bias is the policy, not the machine.** Open loop (constant
  `linear.x`, `angular.z` = 0, no policy), 3 trials over **10.05 m**:
  lateral **+0.0000 m**, yaw change **0.00000 rad**. Bare policy, same
  lane: **+0.3115 / +0.3107 m over 6.13 m** (≈ +50.8 mm/m). Bare policy
  teleported to **exactly yaw 0** on the ramp: **+0.0452 / +0.0452 /
  +0.0438 / +0.0438 m** in lanes +0.75 / +0.25 / −0.25 / −0.75 — same sign
  and magnitude on both sides of the centreline. The bias follows the
  robot, not the lane.
- Tests: **253, 0 failures, 0 skipped** (up from 250; `coco_rl` 50 → 53).

**Corrected** (both errors were mine, in text committed last session):
- "Every run ≤ 0.470 m got home" was **circular** — 0.470066 m is simply
  the largest AMCL gap among the successes, so it was true by
  construction. The data brackets the threshold to **(0.470, 1.183) m with
  nothing sampled between**, and supports no stronger claim.
- The half-lane count was **three** runs, not two: run 20 (+0.2581) was
  missed alongside 16 (+0.3012) and 19 (+0.2541).

**Unverified:**
- **No demo video.** See below — it is a tooling gap, not a failed run.
- The bias rate is ~2.5× larger on the flat (50.8 mm/m) than on the grade
  (20.2 mm/m). Unexplained.
- The mechanism behind Nav2 finishing legs outside its own
  `yaw_goal_tolerance` (FUTURE_WORK 8b) is still untested.

**Open:**
- **The video needs a window-manager tool.** `wmctrl` and `xdotool` are
  both absent and `sudo` needs a password. Without one, the Gazebo GUI and
  RViz cannot be placed or raised, so they open behind the fullscreen
  terminal and `x11grab` records the terminal instead of the robot. The
  first attempt was aborted the moment a layout probe showed this, and the
  capture was deleted rather than kept. `~/.gz/sim/8/gui.config` was
  temporarily resized to 952×1000 and has been restored to its original
  1000×845.
- Pushed to **`GauthamCodes/coco-robot-jazzy-2.0` (private)**, a *new*
  repo, as instructed. `origin` (coco-robot-ros2) is untouched and still
  carries only `main` at 34f151c.
- `ramp_driver` still has no `os.path.expanduser` on its `model` parameter.

**Next:**
For the video, one of:

```bash
sudo apt install wmctrl        # then I can tile and raise both windows
```

— or arrange the Gazebo GUI and RViz side by side by hand and say when
they are placed. Otherwise, M7 Phase 1, the MuJoCo throughput baseline:

```bash
source ~/ros2_ws/src/coco-robot-ros2/setup_env.sh
sed -n '/## Phase 1/,/^```$/p' ~/ros2_ws/src/coco-robot-ros2/docs/M7_PHASES.md
```

The figure to beat is 8.7 env-steps/s, and that block says to stop and
report if MuJoCo is not meaningfully faster.

---

## 2026-08-07 — Phase 0.5/0.6 closed, history rewritten, published

**Built:**
No new features. History rewritten with `git-filter-repo`, the demo video
published as a release asset, README gains a video link and an attribution
line. Phases 0.5 and 0.6 are closed.

**The rewrite — what it did and did not touch.** Blobs only; **no commit
message was modified**. Two things removed: the 5 `.pyc` blobs that should
never have been committed, and `/home/akshayr2003` (a third party's home
path, present in history but not in the working tree), replaced with
`/home/user`. The superseded `gautham@gmail.com` was replaced with
`gauthamanil888@gmail.com` so the identity is consistent.

Deliberately **kept**: the 106 `Co-Authored-By` trailers and the 26
`Claude-Session:` URL trailers. Both were flagged before the rewrite and
the decision was to leave existing history alone and simply stop adding
trailers from now on. Anyone minding the session URLs being public should
know they are there.

**Verified before pushing** (all six, on the rewritten branch):
- `akshayr2003` anywhere in history: **0**
- authors: **only `GauthamCodes <gauthamanil888@gmail.com>`**
- `.pyc` anywhere in history: **0** (was 5)
- commit count: **128 → 128** (`--prune-empty=never`; no commit touched
  only `.pyc`, so none could have been pruned anyway)
- tracked content vs the pre-rewrite tip: **exactly 2 changed lines in 2
  files**, both `maintainer_email` — i.e. only the intended replacement.
  Note the HEAD *tree* SHA did change (`51af1444` → `137d38c9`), which is
  expected: the email replacement edits tracked files, so byte-identity
  was never achievable and "unchanged tree hash" was the wrong check.
- `colcon build` clean, tests **253 / 0 / 0**

**Published:**
- Public repo `coco-robot-ros2`, branch `jazzy-harmonic-port` at `82a2297`.
  **`main` untouched, still `34f151c`.**
- Private mirror `coco-robot-jazzy-2.0` force-updated to the same SHA;
  local, origin and jazzy2 all agree.
- Release `m6-fetch-demo` with `coco_fetch_demo.mp4` (1920×1004, 75.28 s,
  936 kbps, 8.8 MB). Not in git. Also at `~/Videos/coco_fetch_demo.mp4`.

**Backup — keep this.** `~/coco-backup-20260807-0543.bundle` (4,731,170
bytes), `git bundle verify` reported *"is okay"* and *"records a complete
history"* before anything was rewritten. Pre-rewrite ref state is beside it
in `~/coco-backup-20260807-0543.refs.txt`; the old tip was `d270e77`.

**Measured across Phases 0.5–0.6** (carried forward, all from those runs):
19/20 fetches complete; approach inside the 5.5 mm window **20/20**
(sd 0.6 mm); grasp held **20/20**, lifts 33.9–35.9 mm; cross-track at the
summit mean **+0.120 m**, max **+0.301 m**; entry heading outside Nav2's
own `yaw_goal_tolerance` in **14/20**; constant policy bias **+0.045 m**
with open-loop drive measuring **+0.0000 m** over 10.05 m.

**Unexplained, and this is the honest headline:** the **majority of mission
cross-track drift has no established cause**. The constant policy bias
accounts for ~15 % of the 0.301 m worst case; entry heading covers some
further part at r² = 0.32; the remainder — including the arrival offset of
up to +0.158 m the robot inherits at the ramp foot — is unattributed. Also
open: why Nav2 finishes legs outside its own yaw tolerance (not AMCL error
— estimate and truth agree to 0.076 rad), and why the policy bias rate is
~2.5× larger on the flat than on the grade.

**Next:**
M7 Phase 1 — the MuJoCo throughput baseline. `coco_config` does **not**
currently hold wheel radius, track or masses (they live in the xacro and
`coco_controllers.yaml`), so generating an MJCF "from coco_config" requires
adding them there first, with a test pinning them to the xacro.

```bash
sed -n '/## Phase 1/,/^```$/p' ~/ros2_ws/src/coco-robot-ros2/docs/M7_PHASES.md
```

---

## 2026-08-07 — M7 Phase 1: MuJoCo throughput and the fidelity gap

**Built:**
- `coco_config.robot` gains the base physics constants — `WHEEL_RADIUS`,
  `WHEEL_WIDTH`, `WHEEL_MASS`, `WHEEL_SEPARATION`, `WHEELBASE`,
  `CHASSIS_MASS`, `CHASSIS_SIZE`, `WHEEL_SEPARATION_MULTIPLIER` — each with
  its provenance. They were readable only from the xacro and
  `coco_controllers.yaml` before, which was tenable with one simulator and
  is not with two. `test_base_matches_urdf.py` pins them to both sources,
  including deriving the track from where the wheel joints actually sit
  rather than trusting the typed parameter.
- **`coco_sim`** (ament_python): generates the MJCF from those constants.
  `test_mjcf_traces_to_config` rebuilds with a monkeypatched constant and
  asserts the model changed, plus a guard asserting an *unused* constant
  leaves it unchanged — together they make "generated from coco_config" a
  fact rather than a comment.
- **`coco_rl/coco_rl/mujoco_env.py`**: Gymnasium env, shape-identical to
  `ramp_env` (`Box(-1,1,(2,))` action, 8-dim obs, `STEP_DT` 0.1,
  `MAX_LIN` 0.4 / `MAX_ANG` 0.5). Zero `rclpy`, enforced by a hostile test
  that strips ROS from `sys.modules` and poisons `__import__`, with a
  further test asserting the guard itself still raises.

**Measured** (this session, this machine):
- **Throughput**: 1 / 4 / 8 / 12 workers → **805 / 2,126 / 2,791 / 2,826**
  steps/s. Peak **2,826 = 325×** Gazebo's 8.7. Inside M7_DESIGN §5.1's
  2,000–6,000 target, at the low end.
- Scaling **saturates at 8 workers** (8 → 12 buys 1.3 % on 12 cores).
- `SubprocVecEnv` at 1 worker (805) is **slower** than in-process (1,026):
  IPC costs ~22 %, so it only pays from 2 workers up.
- **Attribution, not assertion**: raw `mj_step` = 100,401 physics/s =
  **1,004** control-step equivalents; full env step = **1,026**. They agree
  to ~2 %, so the env loop costs nothing measurable. Combined with the v1
  A/B (8.7 without `--fast`, 8.2 with — unlocking physics made it *worse*),
  the ~118× single-process gain is almost entirely **the removal of the ROS
  round trip**, not MuJoCo's solver. Multiprocessing adds the rest.
- **Fidelity**, identical open-loop sequence, 10 s, ground truth both sides:
  straight leg **0.0779 m error over 1.9874 m (3.9 %)** with yaw matched to
  **0.02°**; arc leg **1.0959 m** and **1.2015 rad (68.8°)**.

**Unverified / unexplained:**
- **Turning does not transfer, and the obvious explanation is wrong.** The
  `wheel_separation_multiplier: 1.10` predicts a yaw ratio of 1.10; the
  measured ratio is **2.902**. Both simulators under-turn a commanded
  2.5 rad (Gazebo 1.833, MuJoCo 0.632) as a skid-steer should, but disagree
  by 2.9×. The remaining ~2.6× is **unexplained**; contact modelling is the
  leading candidate per M7_DESIGN §5.3. **Not tuned** — Phase 1 states the
  divergence, §5.3's calibration is where it gets closed.
- Consequence for Phase 2: straight-line dynamics transfer well enough to
  train against; anything depending on commanded yaw tracking — including
  §4.3's cross-track reward term — will not, until contact is calibrated.
- The MJCF is base-only (no arm, no sensors, no meshes) and its inertias are
  primitive-shape approximations carrying the xacro's masses.
- Carried forward, still open: the majority of mission cross-track drift
  remains unattributed; why Nav2 finishes legs outside its own yaw
  tolerance.

**Open:**
- `mujoco` 3.11.0 and `git-filter-repo` 2.47.0 are `pip --user` installs,
  not in any package manifest. `coco_sim`/`mujoco_env` will not build on a
  machine without them.
- History backup bundle kept at `~/coco-backup-20260807-0543.bundle`.

**Next:**
M7 Phase 2 — The Yard, per `docs/M7_PHASES.md`. Before any policy training,
§5.3's contact calibration is now a stated precondition rather than an
optional step, because of the 2.9× yaw divergence above.

```bash
sed -n '/## Phase 2/,/^```$/p' ~/ros2_ws/src/coco-robot-ros2/docs/M7_PHASES.md
```

---

## 2026-08-07 — Phase 1.5: contact calibration, and a Phase 1 number corrected

**Corrected:** the "2.9× yaw divergence" reported in the Phase 1 entry was
roughly half harness error. `fidelity_mujoco.py` sent a **normalised**
action (0.5, scaled by `MAX_ANG` → 0.25 rad/s); `fidelity_gazebo.py`
published a **raw** twist (0.5 rad/s). The two simulators were driven at
different yaw rates. Compared against commanded rather than against each
other, the real gap was ~1.45×. Everything in Phase 1.5 commands both
sides in rad/s.

**Built:**
- Calibrated contact in `coco_sim/mjcf.py`: sliding friction 0.7 → **0.4**,
  `solref` 0.02 → **0.1**, `solimp` d0 0.9 → **0.5**.
- `mujoco_env` now applies `WHEEL_SEPARATION_MULTIPLIER` in its IK, which
  is parity with the deployed `diff_drive_controller` rather than a tuning
  knob — the same `cmd_vel` must mean the same motion in both.
- `M7_DESIGN.md` §2.5 gains a **yaw-gain randomisation term, 0.70–1.45**;
  §5.3 gains the line that transfer is bought by making the policy
  insensitive to steering authority, not by making the engines agree.
- `coco_sim` now declares `mujoco==3.11.0` in `setup.py` and records it in
  `package.xml` (no rosdep key exists). Pinned because the contact fit is
  against 3.11.0's solver.

**Measured:**
- Yaw sweep, 7 magnitudes × both signs, both simulators.
- Gap **worst at the smallest commands**: 1.711× at 0.05 rad, i.e. exactly
  the lane-hold band — and roughly constant proportional loss, not a slip
  nonlinearity (MuJoCo loses ~40 % even at 0.01 rad/s).
- **Calibrated: worst deviation 1.707× → 1.274×.** Target of 1.3× met.
- **Straight-line improved**: 4.1 % → **2.8 %** of distance over 5 s.
- Three hypotheses tested and two killed: anisotropic friction (refuted at
  source — the xacro is isotropic `mu1=mu2=0.7`, no `fdir1`, and warns
  against anisotropy in DART); torsional friction (`condim=3` moved
  achieved yaw 60.6 % → 60.7 %); actuator tracking (servos deliver 98.8 %
  of the commanded wheel-speed difference). The cause is skid-steer scrub,
  and **sliding friction is a weak lever on it** (0.2 → 1.5 moves
  efficiency only 59.5 % → 65.2 %) while contact softness is the strong one.
- **Gazebo is not self-consistent above 1 rad**: its own +/− asymmetry is
  ≤1.014 up to 1.0 rad, 1.174 at 1.5, and **1.361 at 2.5** — larger than
  the 1.3× tolerance being targeted. Comparisons use the magnitude average
  and say so.

**Unverified / open:**
- Residual 1.27×–0.86× is **not closed**, by choice: friction is a weak
  lever and the reference disagrees with itself at the top of the range, so
  further tuning would fit one yaw rate and degrade the model elsewhere.
  Handled by randomisation instead.
- Single Gazebo run per sweep point. At 1.5 and 2.5 rad the sign spread
  exceeds the difference being measured, so those rows are approximate;
  repeats not run.
- Calibration is on a flat plane only. The Yard's grades and heightfields
  are a different contact regime and are not covered by this fit.
- Carried forward: the majority of mission cross-track drift is still
  unattributed; Nav2 still finishes legs outside its own yaw tolerance.

**Next:**
M7 Phase 2 — The Yard, per `docs/M7_PHASES.md`. The contact calibration
that Phase 1 flagged as a precondition is now done for flat ground.

```bash
sed -n '/## Phase 2/,/^```$/p' ~/ros2_ws/src/coco-robot-ros2/docs/M7_PHASES.md
```

---

## 2026-08-09 — M7 Phase 2: The Yard in both simulators, and three closeout checks

**Built:**
- `coco_sim/worlds/yard_params.yaml` — the single source of Yard geometry.
  Every rescaled value carries `spec:`, `value:` and `derivation:`.
- `coco_sim/coco_sim/yard.py` — one generator, two engines. Analytic
  `height(x, y)` is the sole truth; both the MJCF and the SDF are emitted
  from the same `features()` list; heightfield STLs written on MuJoCo's own
  triangulation diagonal, which was **measured** rather than assumed.
- `coco_sim/coco_sim/probes.py` — where parity probes go and why there.
- `coco_rl/coco_rl/yard_env.py` — full §2.5 randomisation, applied to a
  compiled model **in place** (no per-episode recompile), reproducible from
  a seed alone.
- `gazebo_models/worlds/coco_yard.world` + `meshes/yard/*.stl`, generated.
  **`coco_world.world` untouched.**
- Tests: **335 passing** (was 250). `coco_sim` 42, `coco_rl` 93,
  `coco_config` 70. The 6 remaining failures are `flake8`/`pep257`/
  `copyright` in `custom_teleop` and `coco_perception` — **pre-existing**,
  verified by re-running them on a stashed tree; neither package was
  touched this session.

**Measured:**
- **Cross-engine parity 0.242 mm worst case** over 264 plumb-bob probes
  dropped in both engines, of which 0.197–0.201 mm is a *constant*
  compliance offset present on flat ground too — **geometric parity is
  0.138 mm**. Concave features genuinely entered: the bridge void drops the
  full 0.650 m in both engines; troughs, depressions and the under-deck
  cavity all agree.
- **Yard throughput at 8 workers: 2,287 / 2,222 / 751 steps/s** on routes
  A / B / C. **Route C is 3.0× more expensive** (the rubble heightfield);
  still above the 500 steps/s stop threshold.
- **Per-route feasibility:** A completable (24/25 at ≤0.65 throttle), B
  marginal (caps at 15/25, friction-limited), C completable but
  throttle-sensitive (23/25 at 0.35, 8/25 at full). A and C fall
  monotonically with throttle, B is flat — torque-limited vs
  friction-limited, cleanly separated.
- **The curb: the spec's 60 mm needs 1.00 m/s, which is 2.5× `MAX_LIN`.**
  Not mountable as this robot is commanded. The built 28 mm needs 0.35 m/s
  (88 % of maximum), and is unmountable at μ = 0.6 — the bottom of Route
  C's own range.
- **Calibration conditioning:** not flat (span 0.113 over μ ∈ [0.30, 0.50],
  9.3 % of the fitted score) and the fitted μ = 0.40 is **not** the
  optimum — μ = 0.30 scores better.

**Found and fixed (defects in already-committed work):**
- `CAMERA_MASS = 0.040` mislabelled; the extra 10 g was the **IMU**. Root
  cause was a test regex that did not handle self-closing `<link/>` tags
  and swallowed the next link's mass. Split, parser fixed, guard added.
- **Neither MuJoCo env limited acceleration**, while the deployed
  controller ramps at 2.0 m/s². Caused wheelies that read as "grippy
  ground is hard to climb". Wired in `yard_env`.
- `torque_scale` scaled `gainprm` but not `biasprm` — a **speed** scale,
  not a torque scale.
- A **curb overhang of my own design** that made the curb unclimbable at
  any speed. Found by the probes; removed.
- The **spawn transient**: only spawning at exactly the wheel radius is
  stable. Spawning 2 mm clear leaves the robot still descending 0.1 s
  later (0.25 s contact time constant, 11.8 mm overshoot); spawning at the
  settled depth throws it 85 mm in the air.

**Unverified / open:**
- **Check 1 is half done.** MuJoCo's yaw efficiency across μ is measured;
  **the Gazebo half was not**, because it needs a world variant per μ and
  `full_world_robo.launch.py` has no `world` argument while
  `coco_world.world` is do-not-touch. **The 0.70–1.45 question is
  unanswered.** Recommendation recorded (narrow the friction
  distribution rather than widen the gain range) but **not acted on**.
- **`refit.py`'s `solref` lever is disconnected** — three values return
  bit-for-bit identical scores, caught by `coco_sim.sweep`. The accepted
  calibration is **not reproducible from the committed harness** (1.211 vs
  the recorded 1.170), and `solimp = 0.9` scores better than the fitted
  0.5. Re-fit with all levers verified before reusing those numbers.
- Non-monotonicity of yaw efficiency in μ is **rate-dependent** and at
  0.50 rad/s, μ = 1.5 the sign inverts. **Hypothesis (labelled): mostly a
  solver artefact** — halving the timestep cuts the high-friction end by a
  third while leaving the low end alone, and a physical optimum does not
  move with the integrator.
- `mujoco_env` still has no acceleration limit; left alone deliberately so
  Phase 1.5's steady-state numbers stay valid. Unify in Phase 3.
- Nothing was trained. Deck traverse open-loop is 0/17, 3/9, 0/8.

**Next:**
Phase 3 — the classical baselines, per `docs/M7_PHASES.md`. Re-fit the
contact calibration first, with every lever asserted connected.

```bash
sed -n '/## Phase 3/,/^```$/p' ~/ros2_ws/src/coco-robot-ros2/docs/M7_PHASES.md
```

---

## 2026-08-09 (later) — Phase 2 aftermath: the harness, Check 1 finished, Route C options

**Built:**
- `build_mjcf()` now takes `friction` / `solref` / `solimp` / `timestep` /
  `kv` as **arguments**, defaulting to the committed constants. This is
  the structural fix for the disconnected lever: the old harness swept by
  string-replacing literals in the generated XML, so there is now no
  literal for a sweep to miss.
- `coco_sim/coco_sim/calibrate.py` — the calibration harness, in the
  package and under test, with `audit_levers()`. Reference data committed
  at `coco_sim/reference/yaw_gazebo_baseline.csv`, recomputed rather than
  transcribed. A test forbids `.replace(`/`re.sub(` in the harness source.
- **`world` launch argument** on `full_world_robo.launch.py` (bare name →
  package `worlds/`, absolute path → as given, default unchanged), so
  terrain can be swept without touching frozen files.
- Tests **335 → 348**; `coco_sim` 42 → 55.

**Measured:**
- **All four levers now live** (`friction` 0.2401, `solref` 0.0765,
  `sep_mult` 0.1480, `solimp` **0.0078** — weak but connected, which is a
  different statement from disconnected).
- **The committed calibration does not reproduce as recorded.** `mjcf.py`
  claims worst deviation 1.170×; the committed parameters actually score
  **1.2696 over all seven commands** and 1.2105 over the four the harness
  scores. 1.170 is reachable only over a **two-command subset** — and it
  was compared against Phase 1.5's 1.274×, which was explicitly over
  seven. Like-for-like, the re-fit moved 1.274 → **1.270**: a wash, not an
  improvement.
- **The committed parameters rank 26th of 60.** Best is the same
  solref/solimp at **friction 0.30 → 1.1714**, confirming Check 2's
  finding by an independent route.
- **Check 1 finished. The ratio does NOT stay inside 0.70–1.45** — it
  leaves at 4 of 15 combinations, reaching **0.526**, and sits at 0.709 at
  μ = 0.70 (inside by 1.3 %).
- **Gazebo cannot express terrain friction above 0.7.** Its yaw response
  is two plateaus with one step between μ 0.5 and 0.7, flat at 69.6 / 69.3
  / 69.6 % for μ 0.70 / 0.90 / 1.10 — the wheels are pinned at 0.7 in the
  xacro. So the μ ≥ 0.9 rows compare MuJoCo at 0.9–1.1 against Gazebo
  still at 0.7; that divergence is a definition mismatch, not an engine
  disagreement. Exact mirror of the MuJoCo max-rule bug the `<pair>`
  elements were added to fix.
- **Route C curb, minimum approach speed by height and μ** — 24 mm is
  mountable across the whole of Route C's friction range inside `MAX_LIN`
  (0.35 m/s at μ = 0.6); the built 28 mm needs 0.50 m/s at μ = 0.6.

**Reported, not acted on (awaiting decision):**
- **Route C**: four options with costs — raise `MAX_LIN` to 0.50 (breaks
  the shipped policy's action scale and the 10/10 and 19/20 measured with
  it, and argues against a measured v1 finding); shrink the curb to 24 mm
  (**my recommendation** — confined to Route C, preserves the momentum
  demand at 88 % of `MAX_LIN`); raise the friction floor to 0.8 (halves
  the route's adaptation demand); or drop the curb (removes the world's
  only discontinuity).
- **Friction definition**: fix what μ means in Gazebo *before* touching
  `YAW_GAIN_RANGE`. Raising the xacro's wheel μ is the correct fix and the
  expensive one; capping §2.5 at 0.35–0.70 is the cheap one and rewrites
  Routes A and C.
- **Re-fitting at friction 0.30**: not done. It would change the contact
  model every Phase 2 number was taken through — parity, throughput and
  per-route feasibility would all need re-running.

**Corrections recorded** in `DESIGN_DECISIONS.md`: the quasi-static
"60 mm is impossible" derivation (right regime, wrong question), and the
NavFn "terminates the fill early" explanation (both modes stop at the
start cell — `navfn_planner.cpp:272` passes `atStart=true`; the real
mechanism is `calcPath` abandoning gradient descent for a grid-locked step
whenever any of nine neighbourhood cells is unvisited).

**Unverified / open:**
- The 24 mm margin (0.35 against 0.40, **12 %**) was measured on a **flat
  run-up**, not over 2.17 m of heightfield. Not measured.
- Gazebo's ± yaw asymmetry is ~1.35× at 2.5 rad **at every friction**.
- `mujoco_env` still has no acceleration limit.

**Next:**
Decisions pending on Route C and on the friction definition. Phase 3 —
classical baselines — after those, per `docs/M7_PHASES.md`.

```bash
sed -n '/## Phase 3/,/^```$/p' ~/ros2_ws/src/coco-robot-ros2/docs/M7_PHASES.md
```

---

## 2026-08-09 (later still) — three decisions applied, and the state at restart

**Note on coverage.** The phases requested for this checkpoint already have
their own entries above and are not repeated: Phase 0.5/0.6 close-out and
the history rewrite (2026-08-07), M7 Phase 1 (2026-08-07), Phase 1.5
(2026-08-07), Phase 2 (2026-08-09), Phase 2 aftermath (2026-08-09). This
entry covers the decisions applied on top of them, and ends with a single
state-of-play block for picking the work back up.

**Decided and applied:**

1. **Route C curb 28 mm → 24 mm**, validated on the ACTUAL 2.17 m rubble
   run-up rather than flat ground. In situ it needs **0.50 of 1.00
   throttle** across Route C's range — **2× margin**, not the 12 % the
   flat measurement implied. The flat figure was **pessimistic**: the
   robot arrives already pitched nose-up by the 16° grade, which lifts the
   wheel's contact relative to the step. Constraint recorded: at μ = 0.35
   neither 24 nor 28 mm mounts at any throttle, so Route C's 0.50 floor is
   now load-bearing.
2. **Calibration NOT re-fitted.** Parameters stand at 0.4 / 0.25 / 0.5.
   `mjcf.py` and RESULTS.md now record **1.2696× over the seven measured
   commands** (inside the 1.3× target) with the scope stated. The old
   "1.170×, better than 1.274×" was scope-free and not a comparison —
   **like-for-like the re-fit was 1.274 → 1.270, a wash.** Friction 0.30
   at **1.1714** recorded as known-better-and-not-adopted, because
   re-fitting changes the contact model every Phase 2 number was taken
   through.
3. **§2.5 friction narrowed 0.35–1.10 → 0.35–0.70**, reasoning in
   M7_DESIGN §2.5. Per-route ranges **re-derived, not clipped** (A
   0.55–0.70, B 0.35–0.70, C 0.50–0.70) because Route A's old range lay
   entirely at or above the cap. **Check 1 re-run: 12 of 12 combinations
   inside 0.70–1.45**, span 0.709–1.142.

---

### State of play at restart

**MEASURED and standing:**
- MuJoCo throughput **3,712 steps/s at 8 workers = 427×** (flat model);
  Yard **2,287 / 2,222 / 751** on routes A / B / C — Route C 3× dearer.
- Cross-engine parity **0.242 mm** worst case over 264 settle probes;
  **0.138 mm geometric** once the 0.197 mm constant compliance offset is
  removed.
- Contact calibration **1.2696× worst over seven commands**, inside 1.3×.
- Per-route open-loop ascent: A completable (24/25 at ≤0.65 throttle), B
  marginal (15/25, friction-limited), C completable but throttle-sensitive
  (23/25 at 0.35 throttle, 8/25 at full).
- Curb: spec 60 mm needs **1.00 m/s** = 2.5× `MAX_LIN` (not reachable);
  built 24 mm needs **0.50 throttle in situ**.
- Yaw ratio across the narrowed friction range: **0.709 – 1.142**, inside
  `YAW_GAIN_RANGE`.
- **349 tests passing.**

**BROKEN:**
- Nothing known-broken in the harness. `refit.py`'s disconnected `solref`
  lever — three values returning bit-for-bit identical scores — was fixed
  at the cause in `5785b28`: `build_mjcf()` takes contact parameters as
  arguments, the harness is `coco_sim.calibrate` with `audit_levers()`,
  and a test forbids the string-replacement idiom. All four levers
  audited live.
- **Pre-existing and not ours:** 6 `flake8`/`pep257`/`copyright` failures
  in `custom_teleop` and `coco_perception`, confirmed on a stashed tree.

**UNMEASURED:**
- The 0.70–1.45 yaw-gain question is **answered** for the narrowed range
  (12/12 inside). What remains unmeasured: whether raising the xacro's
  wheel μ would let Gazebo express the original 0.35–1.10 — that needs
  v1's 10/10 and 19/20 re-checked on a different surface pairing.
- IMU noise σ: the xacro declares no `<noise>` element, so there is
  nothing to match. Sampler applies zero.
- Deck traverse beyond ascent: open loop is 0/17, 3/9, 0/8.
- `mujoco_env` still has no acceleration limit (deliberate — Phase 1.5's
  steady-state numbers were taken through it).

**UNDECIDED:**
- Nothing blocking. Route C is decided (24 mm, in-situ validated). The
  calibration is decided (not re-fitted). The friction range is decided
  (0.35–0.70).
- Open but not blocking: `YAW_GAIN_RANGE`'s floor sits **1.3 % above** the
  measured minimum ratio of 0.709. Widening it to ~0.60 would restore
  margin; not changed.

**Next:** Phase 3 — the classical baselines, `docs/M7_PHASES.md`
unchanged.

```bash
sed -n '/## Phase 3/,/^```$/p' ~/ros2_ws/src/coco-robot-ros2/docs/M7_PHASES.md
```

---

## 2026-08-09 (Phase 3) — the classical baselines, and one claim refuted

**Built:**
- `coco_rl/coco_rl/lateral.py` — `lateral_hold` and its gains, moved out of
  `ramp_driver` **unchanged**, so B1 can import the shipped function
  without dragging `rclpy` into the training environment.
  `test_ramp_driver.py` reaches it through `ramp_driver` and passes
  untouched.
- `coco_rl/coco_rl/baselines.py` — B0 / B1 / B2 and the shared reference
  path, with the **tuned** B2 schedule committed alongside.
- `coco_rl/coco_rl/baseline_eval.py` — the runner and a failure taxonomy
  that is *measured*: `slid back` and `high-centred` both look like a
  timeout if you only read the terminator, and they are what separates a
  friction failure from a geometry one.
- Tests **349 → 361**.

**Measured (120 episodes per cell, 1,080 total; B2 tuned on seeds
10000–10011, evaluated on 0–119, disjoint):**

| | A success | B success | C success |
|---|---|---|---|
| B0 open-loop | 0 % | 8 % | 0 % |
| B1 shipped PD | 0 % | 2 % | 0 % |
| **B2 scheduled PD** | **98 %** | 3 % | 15 % |

- **Claim 1 (camber needs adaptation) is REFUTED.** Measured on the ramp,
  where camber actually acts: B2 holds **1.26 cm mean / 6.66 cm worst**
  across camber 0–8°, four times inside the 5 cm falsifier, **with no
  trend in camber** (1.39 / 1.05 / 1.23 / 1.31 cm). Even B1, un-retuned,
  averages 3.79 cm. **This changes what M8 should be:** Route A's
  contribution is now the deck convergence and the bridge, not the camber,
  and 98 % is the number a policy has to beat there.
- Claim 2 (friction) **stands** — B1 gets 0 % below μ 0.55 and 9 % at the
  top, 2 % overall, against a ≥90 % falsifier.
- Claim 3 (curb) **stands for the 60 mm spec step** (needs 1.00 m/s =
  2.5× `MAX_LIN`) but is **refuted at the built 24 mm**, which B2's fixed
  schedule mounts across the whole friction range.
- Claim 4 (washboard) **stands** — constant throttle crosses only below
  ~0.14 m/s and tips at ≥0.22 m/s.
- Claim 5 (loaded descent) **not tested** — the Phase 3 task ends at the
  bay, so the descent is never exercised.

**Found and fixed:**
- **Bridge falls were being reported as tips.** The detector waited for
  z < 0.30 m, by which point the robot had rolled 43° on the way down and
  the tip terminator had fired — measured at z = 0.610, two control steps
  after it left the deck. Now positional. One of the five failure modes
  this phase must report, so it would have mislabelled a whole column.
- **B2 was under-tuned on the first pass and lost to B0 on Route B**
  (0 % vs 8 %), because the grid searched throttle only to 0.65 and never
  tried what a 26° chute needs. Re-searched to 1.0: A 88 → 98 %, B 0 → 3 %,
  C 7 → 15 %. Exactly the "a weak B2 makes the entire M8 result worthless"
  failure §3.1 warns about.

**Unverified / open:**
- Claim 4's measurement establishes that constant throttle fails above a
  speed threshold; it does **not** separate resonance from plain
  over-speed. Rows above 0.4 m/s are post-tip tumbling.
- Route C tips 101/120 under B2 at the **lowest** cross-track of any cell
  (0.035 m). Not a steering failure — the rubble pitches it over — and the
  mechanism is not isolated.
- Claim 5 needs the descent added to the task before it can be tested.
- Route B is unsolved by every baseline (best 8 %, by B0 of all things).

**Next:** Phase 4 — policy training, `docs/M7_PHASES.md`. Note that Phase
3 has narrowed what M8 can claim: camber is off the table.

```bash
sed -n '/## Phase 4/,/^```$/p' ~/ros2_ws/src/coco-robot-ros2/docs/M7_PHASES.md
```

---

## 2026-08-09 (Phase 3 close-out) — the two routes diagnosed, and what gates Phase 4

Written for a cold start: assume only the repo, no memory of this session.

**MEASURED — standing results**

- **Phase 3 baseline matrix**, 1,080 episodes (120 per baseline per route).
  B2 tuned on seeds 10000–10011, evaluated on 0–119, disjoint.

  | | Route A | Route B | Route C |
  |---|---|---|---|
  | B0 open-loop | 0 % | 8 % | 0 % |
  | B1 shipped PD | 0 % | 2 % | 0 % |
  | B2 scheduled PD (privileged) | **98 %** | 3 % | 15 % |

- **Claim 1 REFUTED.** On the ramp, where camber acts, a retuned PD holds
  **1.26 cm mean / 6.66 cm worst** across camber 0–8°, four times inside
  the 5 cm falsifier, **with no trend in camber**. Camber alone is not
  evidence for learning; Route A's contribution to M8 is now the deck
  convergence and the bridge, and 98 % is the bar.
- **Claim 3 REFUTED at 24 mm** (the height the world contains — B2's fixed
  schedule mounts it across the whole friction range); stands only at the
  60 mm spec step, and there only because 60 mm needs 2.5× `MAX_LIN` and
  is outside the action space.
- **Claims 2 and 4 stand.** Claim 2 with a wide margin: B1 gets 0 % below
  μ 0.55, 2 % overall, against ≥90 %.
- **Claim 5 not tested** — the Phase 3 task ends at the bay, so the loaded
  descent is never exercised.
- Earlier phases: MuJoCo throughput **3,712 steps/s at 8 workers (427×)**;
  cross-engine parity **0.242 mm** worst case, **0.138 mm geometric**;
  contact calibration **1.2696× over seven commands**, inside the 1.3×
  target; Yard throughput 2,287 / 2,222 / 751 on A / B / C.
- **361 tests passing.**

**BROKEN**

- Nothing known-broken in the code. The `refit.py` disconnected-lever
  defect was fixed at the cause (`5785b28`); all four calibration levers
  audited live.
- **Pre-existing, not ours:** 6 `flake8`/`pep257`/`copyright` failures in
  `custom_teleop` and `coco_perception`, confirmed on a stashed tree. Run
  tests **per package** — several packages share test module names and a
  single pytest invocation dies with `ImportPathMismatchError`.

**UNMEASURED**

- The tipped-vs-completed correlation on Route C. The diagnostic harness
  omitted the completion check `baseline_eval` uses, so it recorded 0
  completions where the matrix records 18; the tip *characterisation* is
  unaffected but the correlation was not obtained.
- The 24 % of Route C tips in the **first quarter** of the ramp — a
  separate population from the 65 % at the curb, not explained by the
  terminator mechanism, not diagnosed.
- Claim 4's measurement shows constant throttle fails above ~0.22 m/s but
  does **not** separate resonance from plain over-speed.
- Claim 5 needs the descent added to the task.
- Whether raising the xacro's wheel μ would let Gazebo express the
  original 0.35–1.10 friction range (would require re-checking v1's 10/10
  and 19/20 on a different surface pairing).

**UNDECIDED — all three gate Phase 4**

1. **The deck convergence geometry.** The deck demands up to **1.95 m of
   lateral shift in 1.80 m of travel** before a 0.65 m bridge, against a
   0.40 m minimum turn radius at 0.2 m/s. B1 tracks the lane well, reaches
   the deck 99 % of the time, and then **falls off the bridge 105 times in
   120**. B2 only clears it by slowing to 0.6 deck throttle. Options not
   explored: lengthen the deck before the bridge, move the routes closer
   in y, or widen the bridge. **Nothing changed.**
2. **Route B's viability.** Best success is 8 %, by B0. **39.3 % of its
   episodes have μ < tan(grade) and are physically unclimbable** — no
   controller can help, and it matches the observed `slid back` counts.
   Four options costed in RESULTS.md (reduce grade to 19–22°, widen — which
   does not address it, raise the friction floor to 0.55 at the cost of
   narrowing 2.00× → 1.27×, or drop the route and lose claim 2's only
   home). **None chosen.**
3. **Route C's tipping mechanism.** 101/120 tips are **pitch events, 0 of
   101 roll-dominated**, 65 % at the curb approach. `TIP_LIMIT` is 0.6 rad
   **absolute**; the 16.3° grade consumes 16.3° of it, leaving 18.1°, and
   the measured excursion is 20.6° — while the robot's **true static
   rear-over is 54.5°**. The terminator fires 34° short of falling over,
   on the very manoeuvre that mounts the curb. **This is instrumentation,
   not control.** The fix (measure tip relative to the local surface
   normal) is **not applied**, because `TIP_LIMIT` is shared with
   `ramp_env`, the v1 curriculum and the shipped policy's training
   conditions.

**Next:** these three decisions, then Phase 4 (policy training). Phase 3
has already narrowed what M8 can claim — camber is off the table, and two
of the three routes currently fail for reasons a policy cannot address.

```bash
sed -n '/## Phase 4/,/^```$/p' ~/ros2_ws/src/coco-robot-ros2/docs/M7_PHASES.md
```

---
## 2026-08-16 — COCO 2.0 M1: observability, and three defects only a live run could find

Written for a cold start: assume only the repo, no memory of this session.

**Context change.** Work continues under a new plan (COCO 2.0) whose
milestone 1 is visualisation and mission observability, ahead of any
further terrain-control or RL work. M7 Phase 4 is therefore **not**
started, and the three decisions gating it (deck convergence geometry,
Route B viability, Route C tip instrumentation) are **still open and
unchanged**.

**Built**

- `coco_mission/scripts/mission_hud.py` — subscribes 10 status topics and
  renders one block on `/mission/hud` at 2 Hz. Subscribe-only; publishes
  nothing any other node reads, so it cannot affect a run. Ages come from
  a steady clock, not `/clock`, so it keeps marking sources stale even if
  sim time stops.
- `gazebo_models/rviz/mission.rviz` — 14 displays, fixed frame **`map`**.
  A NEW file. `coco_robot.rviz` is deliberately untouched: it is loaded by
  `rsp.launch.py` where `base_footprint` is the only frame that exists.
- `/mission/state` from `traverse_demo` (step labels, previously stdout
  only) and `/mission/goal` from `mission_hud`.
- `coco_mission` gains a pytest suite: **30 new tests, all passing**.
  (The earlier commit message in this branch says "361 -> 391". That
  arithmetic assumed the documented 361 baseline still held. It does
  not — see below.)

**Measured**

- Two full fetch missions, fresh sim each, `--colour blue`.
  Run 1 **FAILED** at nav-home (vision unconfirmed, `found=0`,
  cross-track `+0.52 m` at climb end). Run 2 **FETCH COMPLETE**, approach
  arrived **0.4 mm** from window centre (base-x 0.1541 vs 0.1537), home
  to within **0.06 m**. **1 of 2 is not a success rate and is not offered
  as one** — the standing M6 figure remains 19/20 from a dedicated matrix.
- Every RViz display topic and every HUD input probed live. Full table in
  `RESULTS.md`, "M1 observability".
- `rviz2 -d mission.rviz` against the live stack: **zero** plugin, type or
  QoS errors; three occupancy grids created (`243x175` twice, `60x60`),
  which is evidence those displays received real data.
- AMCL covariance ~0 before motion (yaw **1.09e-13**), growing to
  **sigma x 0.229 m** while driving and **0.452 m** at the platform.

**The documented 361-test baseline does not currently hold**

Measured per package with cwd set to the package dir: **375 passing, 29
failing.** All 29 are in `coco_rl`, they reproduce **identically on an
unmodified checkout**, and every one is `FileNotFoundError:
.../ros2_ws/build/coco_sim/worlds/yard_params.yaml`. That directory does
not exist; the file is present in source. The workspace's `coco_sim`
build is stale. Fix, **not applied** (it is the user's workspace):

```bash
cd ~/ros2_ws && colcon build --packages-select coco_sim
```

Separately, three packages score **higher** than CLAUDE.md recorded —
`custom_teleop` 67 (not 64), `coco_perception` 44 (not 41),
`coco_moveit_config` 12 (not 5). The six "pre-existing"
flake8/pep257/copyright failures and the seven missing
`coco_moveit_config` tests were an artefact of invoking pytest from the
repo root, where the `coco_rl/` directory shadows the installed module.
With the correct cwd they pass. CLAUDE.md corrected.

**Found and fixed**

1. `mission_hud.py` lacked the executable bit. With
   `--symlink-install` that aborted all of `mission.launch.py`, which
   SIGINT'd six nodes mid-import and surfaced as a numpy/rclpy
   `ImportError` storm in healthy processes. Cause was a file permission.
2. **`ros_clean.sh` had no `mission_hud` pattern** — same trap its own
   header documents for `parameter_bridge`. Two HUDs published
   `/mission/hud` at once and the stale one won often enough to make a
   fixed field look unfixed. **Anything added to a launch file must be
   added to `ros_clean.sh`.**
3. `/goal_pose` is advertised and **never publishes** in an autonomous
   run — the sequencer uses the `NavigateToPose` action, and
   `/goal_pose` is RViz's own goal tool only. Replaced with
   `/mission/goal`, derived from the end of the global plan.
4. `LOCALIZATION` showed `STALE` and hid the sigmas while the robot was
   correctly localised, because AMCL publishes only after
   `update_min_d 0.25 m`. Age is no longer staleness for that field.

**Unverified / open**

- Run 1's `+0.52 m` climb cross-track: variance, regression, or
  `lateral_hold` not engaging. **Not diagnosed.**
- `ROBOT PITCH` read `-0.314 rad` during the platform approach, where the
  robot should be flat. Either genuine, or `/ramp/status`'s `pitch` is
  held from the climb while the driver is idle. **Not diagnosed**, and it
  matters for M2's grade estimator.
- The rendered RViz window has never been visually inspected or recorded.
- `rviz_2d_overlay_plugins` is not installed, so `_publish_overlay` has
  never executed. Install with
  `sudo apt install ros-jazzy-rviz-2d-overlay-plugins`.
- M7 Phase 4 and its three gating decisions remain untouched.

**Next:** decide whether to install the overlay plugin and record the
demo, or move to COCO 2.0 milestone 2 (terrain control: tip-termination
correction, grade and friction estimators, observer-driven controller).
Note that milestone 2's first item is the same Route C tip-instrumentation
decision M7 Phase 3 left open.

```bash
# reproduce the M1 verification
bash gazebo_models/scripts/ros_clean.sh
ros2 launch gazebo_models full_world_robo.launch.py traverse:=true gui:=false   # T1
ros2 launch coco_mission mission.launch.py \
    policy:=/home/gautham/coco_rl_runs/curriculum_20260726_211008/phase5_24deg_s0p0.zip \
    rviz:=true                                                                   # T2
ros2 run gazebo_models traverse_demo.py --colour blue                            # T3
ros2 topic echo /mission/hud --field data                                        # T4
```

---
## 2026-08-16 (checkpoint) — persistent state files added, C2-M1 closed

*This entry keeps this file's Built/Measured/Unverified/Open/Next fields
and adds the Objective/Commands/Interpretation fields the COCO 2.0
handoff protocol asks for. Both are satisfied; the file's format is not
broken.*

**Objective:** establish repository-authoritative state files so a fresh
agent with zero conversation memory can continue, and formally close
COCO 2.0 milestone 1. No hypothesis — this is bookkeeping, not an
experiment.

**Built:**
- `PROJECT_STATE.md` (new, repo root) — the authoritative snapshot.
- `docs/ROADMAP.md` (new) — all three milestone tracks with completion
  criteria and measured results.
- No source changes.

**Commands run:**

```bash
git status --short                # clean but for untracked .build_wt/ .install_wt/
git rev-parse --abbrev-ref HEAD   # worktree-coco2-m1-observability
git log --oneline -3              # dfcc49c, 0766781, 22c793c
# per-package pytest, cwd = package dir
```

**Measured — and this checkpoint produced a real result:**

Re-running the suite showed `coco_rl` at **106 passed, 0 failing**, not
the 77/29 recorded earlier the same day. The difference is that this
worktree's overlay build of `coco_sim` had since been created, which
produced the `worlds/` directory the tests were looking for. Re-running
against the unmodified main checkout still gives 77/29. So:

| `coco_sim` build | `coco_rl` |
|---|---|
| stale (the user's `~/ros2_ws`) | 77 passed, 29 failing |
| fresh (this branch's overlay) | **106 passed, 0 failing** |

That turns the earlier diagnosis from a hypothesis into a **measured
fix**, and takes the suite to **404 passing / 0 failing**. `CLAUDE.md`,
`RESULTS.md` and `PROJECT_STATE.md` updated from 375/29 to 404/0 with
the precondition stated.

**Unverified:** the rebuild has **not** been applied to the user's
`~/ros2_ws` — that is theirs to run. Until they do, their tree still
shows the 29.

**Interpretation / decisions:**

- **The session log stays at `docs/SESSION_LOG.md`.** The handoff
  protocol names a root-level `SESSION_LOG.md`, but this file already
  holds 1000+ lines and `CLAUDE.md` points here. A second log would
  fragment history, which is worse than a naming deviation.
  `PROJECT_STATE.md` states the location in its read-order section.
- **Milestone IDs are now namespaced `C2-`.** The COCO 2.0 plan's
  milestone numbers collide with the repo's existing M0–M7 — "M2" could
  mean the v1 world rebuild or COCO 2.0 terrain control. `PROJECT_STATE.md`
  and `docs/ROADMAP.md` both lead with this.
- **`docs/ROADMAP.md` carries all three tracks**, not just COCO 2.0,
  because M7 Phase 4's three gating decisions are the same decisions
  C2-M2 has to take. Splitting them across files would hide that.

**Failures:** none this checkpoint.

**Open (carried forward, all unresolved):** the stale `coco_sim` build
(29 failing tests, one colcon command, deliberately not applied because
it mutates the user's workspace); run 1's `+0.52 m` climb cross-track;
the `-0.314 rad` `ROBOT PITCH` during the platform approach; the three
M7 Phase 4 decisions.

**Next:** clear KNOWN PROBLEM #1, then diagnose #4, then take the
Route C tip-terminator decision — which is C2-M2's first item.

```bash
cd ~/ros2_ws && colcon build --packages-select coco_sim
cd ~/ros2_ws/src/coco-robot-ros2/coco_rl && python3 -m pytest test -q
```

---
## 2026-08-17 — the persistence layer moved to the trunk

**Objective:** fix the branch architecture of the state files. No C2-M1
implementation touched.

**Built:** nothing new. Two files *moved* branches.

**The defect:** `PROJECT_STATE.md` and `docs/ROADMAP.md` were committed
on this feature branch (`625a659`). Both describe the *project*, not the
branch, which broke the handoff protocol in two ways:

1. **A fresh agent checking out the trunk saw no state at all.** The
   whole point of the protocol is that clearing the conversation is safe.
   It was not — the state was hiding on a branch nobody had been told to
   check out. Flagged at the end of the previous session; this fixes it.
2. **They are singleton mutable snapshots.** Any second feature branch
   that checkpoints rewrites the same lines and conflicts on every merge,
   forever. Not a merge accident — the predictable result of
   version-controlling a "current value" on parallel branches.

**Result:**
- New commit `6c06c45` on branch `coco2-state`, based directly on the
  trunk (`33110a6`), carrying `PROJECT_STATE.md`, `docs/ROADMAP.md`, the
  new `docs/STATE_PROTOCOL.md`, a "State first" pointer at the top of
  `CLAUDE.md`, and the `.gitignore` entry. It **fast-forwards** onto
  `jazzy-harmonic-port`.
- This commit deletes those two files from this branch, so the two
  branches no longer both own them and the C2-M1 merge stays clean.

**Interpretation / decisions:**
- **The trunk, not a long-lived `state` branch.** A parallel state branch
  would have to be merged into every feature branch to be readable from
  them — strictly more work than keeping state where a fresh agent
  already lands, and more likely to go stale.
- **`PROJECT_STATE.md` gained a BRANCH MAP.** That table is what makes
  trunk-only state honest: the trunk does not *contain* the C2-M1 code,
  but it always *knows where it is*, and says so before a reader can
  mistake a missing `coco_mission` package for a bug.
- **`docs/SESSION_LOG.md` stays shared and append-only**, and is
  deliberately not touched on `coco2-state`. Two tails on two branches
  would manufacture exactly the conflict this work removes. Append-only
  files conflict only at the end and resolve as "keep both, in date
  order".
- **`CLAUDE.md` is edited on both branches on purpose, in different
  hunks** — "State first" at the very top here, the Tests baseline far
  below there — so git auto-merges them instead of conflicting.

**Measured:** none. No runs; no code changed. Tests not re-run because
no source, test or launch file was modified — `git diff --stat` against
the previous commit is two deletions, both Markdown.

**Unverified:** the merge itself. `coco2-state` fast-forwards onto the
trunk by inspection (one commit, direct descendant of `33110a6`), and
the C2-M1 merge is expected clean now that the overlap is gone, but
**neither merge has been performed** — merging is the repo owner's call.

**Open:** unchanged — the stale `coco_sim` build, run 1's `+0.52 m`
cross-track, the `-0.314 rad` `ROBOT PITCH`, and M7 Phase 4's three
decisions.

**Next:** land the state layer on the trunk, then decide on C2-M1.

```bash
cd ~/ros2_ws/src/coco-robot-ros2
git checkout jazzy-harmonic-port
git merge --ff-only coco2-state
git ls-tree --name-only HEAD PROJECT_STATE.md docs/ROADMAP.md   # both must list
```

---

---
## 2026-08-17 — C2-M1.5: the HUD's pitch was a fossil, and the failed fetch was two failures

**Objective:** a runtime-integrity and signal-semantics gate before C2-M2.
C2-M2's first deliverable is a grade estimator, and C2-M1 had left the one
field it would be built on undiagnosed. Diagnose only; fix only what the
diagnosis proves. No C2-M2 implementation, and none was started.

**Hypotheses under test, all pre-registered by the previous checkpoint:**
(A) what diverged first in the failed fetch of 2026-08-16; (B) what
`ROBOT PITCH = -0.314 rad` actually was; (C) whether `/approach/target`'s
one-shot VOLATILE publication is a defect.

**Built**

- `gazebo_models/scripts/pitch_probe.py` — new, subscribe-only. Puts every
  pitch-shaped signal in one CSV at 10 Hz with timestamps: `/ramp/status`'s
  `pitch`, `/imu`, ground-truth odometry orientation, `/mission/state`, and
  the number parsed back off `/mission/hud`. Two independent ground truths
  on purpose, so "the IMU is lying" and "the field is stale" stay
  separable. Installed in `CMakeLists.txt`; `pitch_prob[e]` added to
  `ros_clean.sh`.
- Ten new tests: `coco_rl` 106 -> **109**, `coco_mission` 30 -> **37**.

**Commands run**

```bash
# baseline, per package, cwd = the package dir
for p in coco_config custom_teleop coco_rl coco_perception gazebo_models \
         coco_moveit_config coco_sim coco_mission; do (cd $p && pytest test -q); done
# three live runs, fresh sim each, ros_clean between, gui:=false, never --fast
ros2 launch gazebo_models full_world_robo.launch.py traverse:=true gui:=false
ros2 launch coco_mission mission.launch.py policy:=<phase5_24deg_s0p0.zip> rviz:=true
ros2 run gazebo_models pitch_probe.py --out /tmp/pitch.csv --hz 10
ros2 run gazebo_models traverse_demo.py --colour blue            # exp 1
ros2 run gazebo_models traverse_demo.py --colour blue --no-grasp  # exp 2
ros2 topic info -v /approach/target                               # exp 1, live
```

**Measured — (B), the pitch. This was the gate and it is now closed.**

`ramp_driver` writes `self.pitch` only inside its climb and descend loops.
Between segments nothing assigns it, while the 5 Hz status timer keeps
publishing it. The message is never late; the number in it is minutes old.
Over one full fetch, 1,900 samples:

| step | `segment` | `/ramp/status` pitch | `/imu` | max diff |
|---|---|---|---|---|
| 2. RL climb | climb | −0.314 .. 0.000 | −0.315 .. 0.000 | 0.140 (sampling skew) |
| 3. approach the target | idle | −0.314 | −0.314 .. +0.000 | **0.314** |
| 4. pick it up | idle | −0.314 | +0.000 | **0.314** |
| 6. nav home | idle | −0.000 | −0.217 .. +0.004 | **0.217** |

`/ramp/status`'s pitch changed **21 times in 1,899 sample pairs**; `/imu`
changed 144. It held one value for **79.2 s** while the robot pitched to
−0.217. `/imu` and ground-truth odometry agreed to 3 dp everywhere, so the
sensor was never in question.

**Diagnosis: stale ramp-driver state** — option B of the five. Three things
matter more than the label:

1. `-0.314` is genuine *at its sample instant*. `RAMP_ANGLE_DEG = 18`, and
   18° = **0.31416 rad**. The reading is the ramp, exactly.
2. It is *always* the ramp grade, structurally: the climb stops
   `GOAL_MARGIN = 0.3` m short of the crest, so the last sample is taken
   on the uniform 18° face, quasi-statically, where body pitch and surface
   grade coincide. **A grade estimator built on this field would have
   matched ground truth on every metre of ramp and then reported 18° on
   flat ground forever.** It would have passed its own tests.
3. The C2-M1 note's premise was itself half wrong. The robot is *not* flat
   when the approach begins — `/imu` independently reads −0.314 there. It
   levels *during* the approach, and that is where the field stops
   tracking.

**Measured — (A), the failed fetch.** `~/.ros/log` still held both
2026-08-16 runs, with timestamps, so this needed no re-running.

*First divergence: inside the RL climb, and nothing before it.*
`climb finished: ... disp +0.51 m, cross-track +0.524 m`. Cross-track minus
disp is **+0.014 m** — Nav2 delivered the robot to within 14 mm of the blue
lane centreline and the entire 0.51 m accumulated during the climb.
`lateral_hold` was **on and reached its clamp** (peak 0.800 = exactly
`LATERAL_CLAMP`), so "lateral_hold not engaging" is **refuted**. It does
not follow that the clamp was binding: the 2026-08-17 run also peaked at
0.800 and finished at cross-track +0.036 m. Saturation happens on good
climbs too.

*`found=0` is a consequence.* Logged **3.0 s after** the climb ended, with
the robot 0.52 m off a lane grid of 0.5 m spacing, and `seen=blue,yellow`
is the wrong-lane signature `target_finder` documents itself. Blue was in
frame; `_locate` rejected it on either the 0.15–2.00 m range gate or the
`plausible_blob` width check — **which one is not determined**, because the
status line records `found=0` and not the reason.

*The step that actually ended run 1 was nav home, and it is independent.*
A 2026-08-17 run with a clean climb, vision CONFIRMED and a successful pick
(x=0.1537, dead on the window centre) **still failed at nav home**, and not
even the same way:

| | run 1 | run 3 (2026-08-17) |
|---|---|---|
| AMCL at leg start | map (8.07, −2.33) vs truth ≈(8.65, +0.84) → **≈3.2 m** | (9.11, 0.06) vs (8.66, 0.25) → 0.45 m |
| ending | `bt_navigator: Goal failed` at 76.1 s | client 240 s timeout, goal cancelled |
| symptoms | — | 11× `Failed to make progress`, 2× Spin timeout, repeated `collision_monitor: PolygonStop` |
| stopped at | (4.74, −2.90) | (0.53, 0.73), **2.59 m short**, stationary 49.7 s |

Run 1 is the AMCL-divergence family (M6 run 15; `M7_DESIGN.md` §2.7 item 1,
the EKF). Run 3 is not. **Confound stated:** run 3 logged `Control loop
missed its desired rate of 10.0000 Hz. Current loop rate is 4.8077 Hz`
with Gazebo, RViz, move_group and the probe all running. Not isolated.

Nav-home across the four recorded legs: FAILED, SUCCEEDED, FAILED,
SUCCEEDED (traverse-only, home to **0.10 m**). Carrying the cylinder splits
one-one across both outcomes. **Four runs are not a success rate and none
is offered** — the standing figure is M6's 19/20. What they do establish is
that nav home fails for reasons that are not downstream of the climb or of
vision. **That is C2-M5's** (localisation health and recovery), which
already names M6 run 15 as its benchmark.

**Measured — (C), `/approach/target`. No change made.** `ros2 topic info
-v` on the live stack: publisher `approach_server` RELIABLE/VOLATILE,
subscribers `grasp_server` and `rviz2`, QoS compatible, one message per
approach. TRANSIENT_LOCAL would be a **defect, not a fix**: the payload's
frame is `base_footprint`, so latching it hands a late joiner a coordinate
in a frame that has since moved. And there is no reliability hole to close
— both nodes start together from `mission.launch.py`, and `grasp_server`
gates on `APPROACH_FIX_MAX_AGE = 120 s` and otherwise warns and grasps at
the nominal stop pose. The `PROJECT_STATE.md` "future idea" to make it
TRANSIENT_LOCAL is **dropped, not deferred**. The real mismatch is that
this is the *result of* `/approach/run` and a `Trigger` response cannot
carry a point; that belongs to **C2-M3**, when actions replace the Trigger
services.

**Found and fixed**

1. **`ramp_driver` publishes `pitch=--` while idle**, matching the `--`
   that `lateral` already used for "no lane, so no cross-track". The
   segment-final sample moved into the `climb finished` / `descend
   finished` log line, so the datum is filed under a timestamp instead of
   broadcast as current.
2. **`mission_hud` takes `ROBOT PITCH` from `/imu`**, BEST_EFFORT, aged
   like every other field, printed in radians and degrees. Body attitude
   was never the ramp driver's to publish, and routing it through the node
   that runs the policy meant the field could only be alive during two of
   the mission's seven steps.

Verified live on a fresh traverse: `pitch=--` before any segment and after
both; `ROBOT PITCH +0.000 rad (+0.0 deg)` on the flat; `-0.315 .. +0.000`
during the climb and `-0.314 .. +0.314` during the descent, tracking `/imu`
to within one sample. `/ramp/status` held `--` for **148.7 s** of nav home,
the interval that used to carry a stale number.

**RViz — inspected for the first time.** Five screenshots of the rendered
window across live missions. Working by inspection: global plan (a legible
green line), camera framing of the target, goal arrow, laser scan, Global
Status Ok, 14 displays in 3 groups. The occupancy map's wall cells sit
under the global costmap's inflation, which is ordinary Nav2 appearance and
not a defect.

**One objective defect, fixed:** the robot leaves the viewport. At
`Distance: 9` / `Focal Point (1.5, 0)` it was near centre at startup,
clipped at the bottom-right on the outbound leg, and off-screen entirely
during the descent and the drive home. The focal point moved to the
**centre of the map** — `(3.956, -0.535)`, computed from
`maps/coco_world.yaml`, which the old value was not in either axis — and
`Distance` was swept against the rendered window on one live stack:
**14 overflows, 18 fits with margin, 22 is a postage stamp**. Shipped at
18. Acceptance test, a property of the config rather than of a run: the
whole occupancy map is inside the viewport, so every reachable pose is
visible without touching the mouse.

The first attempt (`Distance: 14`, focal point at the middle of the
*traverse* rather than of the *map*) was **worse than what it replaced**,
and only the screenshot caught it. Reasoning about a perspective camera's
ground coverage from two numbers and a yaw does not work; looking at the
window takes 25 seconds. Nothing else in the view was touched — this was
not a UI pass, and C2-M9 owns that.

**Unverified / open**

- Why *that* climb drifted 0.51 m when others peak at the same correction
  and stay on the lane. Clamp saturation is **not** established as the
  binding constraint.
- Which of the two gates in `_locate` rejected blue. The status line does
  not record it.
- Nav home: two distinct failure mechanisms in four legs, and the
  degraded-control-loop confound not isolated. **C2-M5.**
- `rviz_2d_overlay_plugins` still not installed, so
  `mission_hud._publish_overlay` has still never executed.
- M7 Phase 4's three gating decisions: untouched.

**Tests:** 404 -> **414 passing, 0 failing**, per package with cwd set to
the package directory, against this branch's overlay build. The stale
`coco_sim` question needed no re-investigation — `coco_rl` was already
106/0 here, the established signature of a fresh build.

**Next:** C2-M2 is **READY**. The pitch signal now has known semantics, a
known source, a known sign convention and a staleness contract, and the
field that would have poisoned the grade estimator no longer exists. Start
with the Route C tip-terminator decision, which is C2-M2's first item and
M7 Phase 4's third gate.

```bash
cd ~/ros2_ws && colcon build --packages-select coco_sim   # if 29 coco_rl tests are red
ros2 run gazebo_models pitch_probe.py --out /tmp/pitch.csv --hz 10  # the C2-M2 instrument
```

## 2026-08-17 — C2-M1.6: the map was fine, the overlay was not

**Objective:** answer two questions that look identical on a screen and
have opposite answers — *is the occupancy map poor* or *is the RViz
presentation cluttered* — then fix the second without touching anything
that could change the first. Presentation only. No SLAM, Nav2, planner,
controller, costmap, robot-model or perception change, and none was made.
No C2-M2 work, and none was started.

**The rule set in advance:** measure the map before touching the display,
and classify it explicitly. If the map had a real defect, document and
stop rather than change SLAM inside a visualization milestone.

**Built**

- `gazebo_models/rviz/mission_debug.rviz` — **new**, the engineering view.
  It is the C2-M1.5 `mission.rviz` preserved: byte-identical below the
  comment header, verified by diff. Everything on — TF, particle cloud,
  both costmaps, laser, the camera pane, the oblique Distance-18 camera.
- `gazebo_models/rviz/mission.rviz` — **rewritten** as the clean operating
  view. Same topics, fewer enabled, re-framed.
- `coco_mission/launch/mission.launch.py` — `rviz_config:=mission` (the
  default) or `mission_debug`, via `PathJoinSubstitution`. `os.path.join`
  would stringify the substitution object into the path.
- `gazebo_models/test/test_rviz_configs.py` — **new**, 21 tests.
- `docs/data/map_audit.py` — **new**, the instrument. Read-only, no ROS,
  not installed by CMakeLists: it is evidence, not a runtime tool.

**Commands run**

```bash
# the map audit, offline, reproducible
python3 docs/data/map_audit.py -o docs/images/c2m16_map_audit.png

# framing sweep: RViz reads the view at startup, so restart the VIEWER,
# not the simulator. map_server + rviz2 only, no Gazebo needed.
ros2 run nav2_map_server map_server --ros-args \
    -p yaml_filename:=gazebo_models/maps/coco_world.yaml
xwd -id <rviz window> -silent -out shot.xwd      # NOT x11grab; see below

# live: one fresh sim, one viewer at a time, both configs on the same run
ros2 launch gazebo_models full_world_robo.launch.py traverse:=true gui:=false
ros2 launch coco_mission mission.launch.py policy:=<zip> rviz:=false
ros2 run gazebo_models traverse_demo.py --colour blue --no-grasp

# the launch argument
ros2 launch coco_mission mission.launch.py --show-args
```

**Measured — the map. Question 1 is answered: GOOD.**

The decisive test is registration, because it is the one a drifted or
ghosted map cannot pass. Five free-standing objects in
`worlds/coco_world.world` have known poses; located independently in the
map they agree on a **single rigid offset (+2.0560, +0.0150) m** with
peak-to-peak **(0.0500, 0.0000) m** and a **worst residual of 25 mm —
half a cell**. Drift makes landmarks disagree; these do not.

- 186 occupied components, **156 of them ≤ 2 cells**. The eight largest
  are every structure that exists. **No ghost walls, no duplicates.**
- The ramp reads 0.575 m short at the up-ramp foot and 0.625 m short at
  the down-ramp foot. Those imply a scan plane at **186.8 mm and
  203.1 mm**, agreeing to **16.2 mm**, against `LIDAR_MOUNT_XYZ`
  z = 0.200 m. **Symmetric** — a defect would not be.
- Free 73.830 m²; the arena is **66.08 m² (89.5%)** of it in one
  component; **51.66 m² drivable** after a 0.2225 m erosion. Speckle is
  85 cells = 0.2125 m²; inflating all of it by 0.30 m costs **0.73%** of
  drivable space and the drivable region stays **one** component.
- Unknown 23.310 m², fully accounted: 15.143 m² outside the arena hull,
  7.625 m² the platform's own occluded interior.

**The one honest caveat.** The north and south walls have continuous gaps
of **0.55 m and 0.85 m**, which do exceed the robot's 0.297 m footprint.
They sit in the far east corners the mapping drive never entered —
**unobserved, not distorted** — and they are not navigable: they open onto
unknown cells, and `nav2_params.yaml` has `track_unknown_space: true` with
`allow_unknown: false` on both planners, so no plan can route through
them. **Recorded, not fixed.** Changing SLAM was out of scope and the
finding does not justify it.

**Measured — the framing.** Map bbox in pixels inside the 1220 × 806
render area of a 1600 × 900 window:

| Distance | Pitch | map bbox | margins L/R/T/B |
|---|---|---|---|
| 12 | 1.30 | 1092 × 691 | 64 / 64 / 91 / **24** |
| **13** | **1.45** | **949 × 652** | **135 / 136 / 90 / 64** |
| 14 | 1.30 | 922 × 591 | 149 / 149 / 132 / 83 |
| 16 | 1.30 | 798 × 516 | 211 / 211 / 164 / 126 |

Yaw 5.9 → **4.712389 = 3π/2**, which puts +x screen-right and +y
screen-up. Not cosmetic: the arena is 12.15 m along x and 8.75 m along y
in a window wider than it is tall, and yaw 5.9 laid the long axis down
the short axis of the window. Turning the map to match the window is what
let the distance come in. Pitch 1.45 beat 1.30 on measurement — less
foreshortening draws a **bigger** map at equal bottom margin.

**Net: the clean view draws the map 36% larger in linear terms than the
preserved C2-M1.5 camera** (949 px against 700 px, same rig, same window),
and both still fit the whole map inside the viewport. C2-M1.5's
"Distance 18 fits with margin" is **confirmed**, not corrected.

**Found and fixed, and only by looking**

1. **The robot lost the frame to its own costmap.** At local-costmap
   alpha 0.32 the two inflation blooms around the gate cubes read louder
   than the robot did. RViz cannot scale a `RobotModel` and the robot is
   frozen, so: alpha **0.22**, plus a saturated blue AMCL arrow at the
   robot in a colour nothing else uses.
2. **The laser was nearly invisible.** The light blue chosen to replace
   the original orange disappeared against the map's *white* free space.
   Recoloured to a mid-saturation teal.
3. **A claim written into the config was wrong, and measuring killed it.**
   The comment said the camera pane costs 3D render width. It does not.
   Measured side by side on one run: the render area is **1220 × 806 px
   in both** files. The pane stacks *above* the Displays tree and costs
   **304 px, 41% of the dock** — the display tree goes 740 px → 436 px.
   The pane still stays out of the clean view, but for the other reason:
   that view's premise is diagnostics-present-but-unticked, and a tree
   you have to scroll is a worse place to keep them.

**Three harness traps, all of which produced a wrong measurement first**

- **`x11grab` captures a screen region.** Another terminal window raised
  itself over RViz and was scored as a framing result.
  `xwd -id <win>` asks the X server for the window's own pixels and
  cannot be occluded.
- **Parking the mouse in a screen corner.** `xdotool mousemove 5 5` hits
  the desktop's top-left hot corner; several renders came back with the
  camera silently orbited away from the config under test. Park it
  somewhere neutral. The tell is the status bar reading
  "Left-Click: Rotate" instead of "RViz is ready".
- **Not killing the previous viewer.** `xdotool search --name RViz` then
  returns whichever window it finds last, and a screenshot gets scored
  against a config that was not the one under test.

RViz was checked and does **not** write the `-d` config back on exit; the
shipped values were verified intact after every sweep.

**Unverified / open**

- **Two traverse runs, `--no-grasp`, fresh sim each: neither completed.**
  Both climbed cleanly (`outcome=goal`, cross-track −0.01 m, disp
  +0.03 m) and confirmed blue at 1.159 m; both then **timed out in the
  scripted descent at 90.1 s** on the platform's far edge, world
  (4.50, 0.24). **No diagnosis attempted** — nothing this milestone
  changed can reach the controller. **Confound stated:** run 1 ran with
  two RViz instances alive, and C2-M1.5 already recorded a 4.8 Hz control
  loop against a 10 Hz target under Gazebo + RViz + move_group. Two runs
  are not a rate; the standing figure is M6's **19/20**.
- The AMCL arrow is drawn at z = 0 and `rviz_default_plugins/Pose` has no
  z-offset, so the robot model hides most of the shaft. Locator and
  heading indicator, not a beacon.
- `/perception/target` is published in `base_footprint`
  (`target_finder.py:566`), so the marker rides with the robot instead of
  pinning a world position. Perception is frozen; this is **C2-M4's**.
- The launch argument was verified by `--show-args` and by resolving the
  substitution to files that exist; RViz itself was started directly on
  each config rather than through `mission.launch.py rviz:=true`.
- `rviz_2d_overlay_plugins` still not installed, so
  `mission_hud._publish_overlay` has still never executed.
- M7 Phase 4's three gating decisions: untouched.

**Tests:** 414 → **435 passing, 0 failing**, per package with cwd set to
the package directory. All 21 new ones are in
`gazebo_models/test/test_rviz_configs.py` (20 → 41) and every one is a
silent-failure mode: a QoS mismatch, a wrong fixed frame, a topic nobody
publishes, a plugin that cannot subscribe to the message type it is
pointed at. RViz does not error on any of those — it draws nothing and
looks like a broken robot. Colours, alphas, widths and camera distance
are deliberately **not** asserted; they were judged against rendered
windows and pinning them would only make them harder to re-judge.

**Next:** C2-M2 remains next and its gate is still open. Nothing here
changed that. Start with the Route C tip-terminator decision, which is
C2-M2's first item and M7 Phase 4's third gate.

```bash
# the mission, either view
ros2 launch coco_mission mission.launch.py policy:=<zip>                      # clean
ros2 launch coco_mission mission.launch.py policy:=<zip> rviz_config:=mission_debug
# re-check the map claim without a simulator
python3 docs/data/map_audit.py
```

---
## 2026-08-19 — C2-M2.0: grade is observable, friction is not, and the tip terminator was measuring the wrong angle

**Objective:** the first of two sessions on C2-M2. Take the Route C
tip-terminator decision, audit the existing baselines, build the terrain
observer and its controller, test them, and leave C2-M2.1 a frozen
benchmark. No large sweep, no RL training. None was started.

**The Route C decision — option B, and it turned out to be nearly free.**

M7 Phase 3 diagnosed this and declined to fix it, on the stated grounds
that `TIP_LIMIT` was shared with `ramp_env`, the v1 curriculum and the
shipped policy. **It is not.** `TIP_LIMIT` is not in `coco_config`; it is
written out independently in four modules, and only `yard_env` is the
Yard. So the Yard's terminator was corrected without touching a number
any v1 result was measured against, and a test now asserts the other
three are still 0.6 rad absolute.

What changed is the **reference frame, not the threshold**: `|roll|` and
`|pitch|` are now measured from the local surface normal, with 0.6 rad
kept exactly, so no new tuning constant enters the repo. Two guards, both
from numbers that already existed — an absolute backstop at the measured
**54.5°** static rear-over, and the surface correction bounded by
`TIP_LIMIT` itself so a bad surface reading can at worst double the
effective absolute limit.

**Reproduced live before changing anything.** Route C seed 7, open loop:
terminated at step 184 at body pitch **−45.30°** on a **+20.16°**
surface — **25.14° surface-relative**, against a 54.5° rear-over. After
the change it fires at step **185**, at −54.51°, which is a genuine
rear-over. **The mechanism is fixed; whether the population of 101 Route
C tips changes is a C2-M2.1 measurement and is not yet measured.**

**The baseline audit produced one finding that shapes the whole
experiment.** In `TUNED_SCHEDULE`, `grade_k = 0.0` and
`lateral_lo == lateral_hi` on all three routes. So B2's entire privileged
advantage, as tuned, is **one number**: throttle interpolated on true μ,
over a range of 0.20 action units. Grade is in B2's interface and has no
effect. That is worth knowing before reading C2-M2.1's table.

**Built**

- `coco_rl/coco_rl/terrain_observer.py` — new, pure Python, no `rclpy`
  (it is reached from `baselines` and so from `yard_env`; CLAUDE.md §2 is
  structural here, not aspirational). `GradeEstimator`,
  `TractionEstimator`, `TerrainObserver`, and the `DeployableSignals` /
  `TerrainEstimate` types.
- `coco_rl/coco_rl/sensor_model.py` — new. **The information boundary, as
  code.** `deployable_signals()` builds what the robot could know;
  `ground_truth()` builds what only the simulator knows. They are
  different types sharing **no field name**, so feeding truth to the
  observer is a `TypeError` rather than a review miss. A 50 Hz
  `ImuSampler` — the rate `coco_robo2.xacro` declares — that reads
  `qpos`/`qvel` and writes nothing.
- `coco_rl/coco_rl/baselines.py` — `B3`, and `schedule_gains()` extracted
  from `B2.reset` so B3 reuses the privileged controller's *relationship*
  rather than a copy of it. A test pins the extracted function against
  B2's original arithmetic.
- `coco_rl/coco_rl/terrain_observer_node.py` — new. Publishes
  `/terrain/state` as `diagnostic_msgs/DiagnosticArray` (no custom
  message; `level` carries validity, `values` the numbers). **Adds no
  publisher to any `cmd_vel` topic** — `cmd_vel_arbiter` remains sole
  publisher to the controller.
- `coco_rl/coco_rl/terrain_benchmark.py` — new. **C2-M2.1's benchmark,
  frozen**: B0/B1/B2/B3 × routes A/B/C × seeds 0–119 = 1,440 episodes,
  metrics fixed, and the decision rule's task named **before** any result
  existed.
- `docs/data/c2m2_sanity.py` — new, the five sanity checks. Read-only, no
  ROS, deliberately not installed by any `CMakeLists.txt`, same shape as
  `map_audit.py`.
- `coco_rl/coco_rl/yard_env.py` — the terminator, the IMU sampler, and
  `flat_reference` captured inside the settle `_measure_rest_z` already
  did.
- `gazebo_models/scripts/ros_clean.sh` — `terrain_observe[r]` added
  **before it is ever launched**, per the rule `mission_hud` paid for.

**Measured**

- **Nose-up is NEGATIVE pitch.** Route A's uniform 12.000° face reads
  **−12.00°**. A `body_pitch → grade` rename would have been wrong in
  *sign* as well as in reference.
- Grade MAE, both axles on one plane: **A 0.106°, B 0.366°, C 1.433°**.
  Flat ground: worst 0.2057° against a true zero.
- **Friction is not identifiable.** τ equals tan(grade) to four decimal
  places at every μ — Route A spans **0.0003** across a μ span of 0.35.
  The encoders cannot see friction at all (wheel speed and servo lag
  identical to four decimals across μ), and an inertial body-velocity
  estimate lost 0.10–0.15 m/s in two seconds against a true 0.28.
- Instrumentation cost 1.5–4.1% of single-worker throughput; a test
  asserts the sampler cannot move the simulation.
- **Tests 428 → 471, 0 failing**, per package with cwd inside each.

**Three things that were wrong first and were caught by measuring**

1. The normal load modelled as `g·cos(grade)` instead of measured — the
   bound `τ ≤ μ` held on **27%** of Route B's samples.
2. The ratio taken in the body frame instead of the contact frame — broke
   on **47%**, *and produced a spurious monotone reading in μ that looked
   exactly like the result being sought*. The apparent signal was the
   error.
3. Both confidence thresholds guessed from filtered-signal behaviour and
   set below the **median** of the raw distribution they gate, so the
   observer disqualified itself and B3 ran in fallback 78–94% of the
   time. Re-set from measured distributions, chosen before B3's outcome
   was looked at.

**Unverified / open**

- **No benchmark was run.** The only multi-episode runs were 4- and
  6-seed smoke tests to prove the runner works; their numbers are **not**
  results and are not recorded as any.
- Whether Route C's 101-tip population changes under the new terminator.
- Whether B3 closes the 10-percentage-point gap. **Not yet measured** —
  that is C2-M2.1 and the whole point of freezing the config now.
- The bound `τ ≤ μ` has **two known exceptions**, both stated: a slope
  break (the robot straddles the ramp foot for one wheelbase with its
  rear axle still on the apron) and a vertical face (Route C's curb
  pushes back with a *normal* reaction). Neither is detectable from an
  IMU and encoders alone, so the benchmark reports `mu_bound_held` as a
  measured rate rather than asserting it.
- The simulated IMU is **noiseless** (`imu_noise_sigma:
  not_yet_measured`). No noise floor was invented. This is why nothing in
  the observer integrates, and it bounds what C2-M2.1 can claim about a
  real robot.
- `terrain_observer_node` has **never been run against a live Gazebo**.
  It is unit-tested through its pure core only; the ROS wiring is
  unexercised.
- M7 Phase 4's other two gating decisions: untouched.

**Not changed, deliberately:** Nav2, SLAM, AMCL, the map, perception, the
robot model, the terrain geometry, the action space, `cmd_vel_arbiter`,
the reward, the shipped policy, `GOAL_SUMMIT`/`GOAL_MARGIN`, and the v1
tip terminator in all three of its non-Yard homes.

**Next:** C2-M2.1 — run the frozen benchmark, then analyse and apply the
decision rule. The rule and its task were fixed here and must not move.

```bash
# the benchmark. 1,440 episodes, ~30-60 min at 8 workers.
python3 -m coco_rl.terrain_benchmark --out docs/data/c2m2_benchmark.json

# re-report an existing run without re-running it
python3 -m coco_rl.terrain_benchmark --report docs/data/c2m2_benchmark.json

# the implementation checks, before trusting any of it
python3 docs/data/c2m2_sanity.py
```

## 2026-08-19 — C2-M2.1: the benchmark ran, the observer cleared the bar, and the bar is the result

**Objective:** the second and final session of C2-M2. Validate the
observer live in Gazebo, run the frozen 1,440-episode benchmark, apply
the 10-percentage-point rule unchanged, and close the phase. No RL
training. None was started.

**The live gate found three defects, and every one was invisible to the
pure-core tests.** C2-M2.0 shipped `terrain_observer_node` having never
run it against a live Gazebo. It did not survive first contact:

1. `is_best_effort()` called with **no argument** — it takes the topic,
   and every other caller in the repo passes one. `TypeError` in the
   constructor: **the node could not start at all.**
2. The estimator was advanced from the **10 Hz publish timer**, so
   samples reached the observer exactly `MAX_AGE` apart and it withdrew
   itself on **431 of 431** — `stale input: 0.100 s > 0.100 s`, a full
   climb without one valid estimate. C2-M2.0 had fixed the observer rate
   at 50 Hz and `B3.observe` says why in as many words; the node put
   estimation and publication on the same clock. They are separate now.
3. `on_declared_flat` was **never passed**, so the flat reference could
   never be learned and `calibrated` was False forever — while the node's
   own comment claimed the opposite.

All three are wiring, not estimation, which is exactly the class a test
that drives the observer directly cannot reach. **12 new tests now
construct the real node**, because nothing off-line ever had.

**Live, after the fixes.** Fresh sim each, `gui:=false`, never `--fast`.

- `/imu` **49.1 Hz** (declared 50), `/terrain/state` **10.02 Hz**,
  422/422 estimates finite, stamps monotonic sim-time.
- Grade on the flat **0.0000°** at confidence **1.000**; on the 18° face
  MAE **0.672°**, and the settled tail sits **0.0035°** off the built
  18.000.
- `/diff_drive_controller/cmd_vel` publisher count **1** — the arbiter —
  before and after the observer started. The observer publishes
  `/terrain/state` and nothing else.
- On the Yard's Route B the bound established at t=3.10 s
  (μ_lower **0.3529**), **B3 engaged on 167 of 200** samples with
  throttle 0.638 / lateral 6.000, and on deliberate withdrawal fell to
  throttle **0.5** / lateral **3.0** — B1's shipped gains exactly.

**And the gate returned a physics result nobody asked it for.** τ settles
at **0.3248** against tan(18°) = 0.3249, and peaks at **0.4865** against
tan(26°) = 0.4877. C2-M2.0's equilibrium-pinning result was measured in
MuJoCo; it now holds in **Gazebo**, on two grades, in a different physics
engine.

**The benchmark: 1,440 intended, 1,440 completed, 0 runner errors.**
Nothing dropped, retried or re-seeded.

**The rule, applied unchanged.** Task `ascent`, margin 10 pp, both fixed
in C2-M2.0 before any result existed:

| route | B2 | B3 | gap |
|---|---|---|---|
| A | 99.2 % | 99.2 % | **+0.0 pp** |
| B | 34.2 % | 32.5 % | **+1.7 pp** |
| C | 65.8 % | 58.3 % | **+7.5 pp** |

**RL is justified on 0 of 3 routes. Additional learned control is NOT
justified by this benchmark.**

**The finding that matters more than the verdict, and it is not the
comfortable reading.** B3 ≈ B2 on ascent is a statement about the **task**,
not about the estimator.

On Route A, B3 fell back on **120 of 120** episodes — identical outcome on
every seed, identical cross-track to four decimals. **B3 is B1 there**, and
necessarily: tan(12°) = 0.213 is below the 0.35 a-priori friction floor,
so the bound can never become informative and the observer correctly
refuses to schedule on an assumption. It recovered **nothing**.

Meanwhile B2 **completed 97.5 % of Route A against B1's and B3's 0.0 %** —
a **97.5-point** difference bought by one number, throttle interpolated on
true μ. The ascent gap is 0.0 pp because ascent does not discriminate on
Route A (B0 through B3 all reach the deck 92–99 %), not because
estimation succeeded.

C2-M2.0 chose ascent for a stated reason: Phase 3 saw B1 reach the deck
99 % and then fall off the bridge 105 times in 120, so completion looked
like it was scoring deck geometry rather than terrain control. **This
benchmark weakens that premise** — B2 crosses the bridge 117 times in 120
on terrain-aware throttle alone, and a pure geometry problem would not
yield to terrain information.

The rule was applied unchanged and its verdict stands as recorded. Whether
`ascent` was the right task is a question for whoever sets the next rule,
and the evidence to decide it is now in `RESULTS.md`.

**Measured**

- Grade MAE by route: **A 0.057°, B 0.253°, C 2.681°** (worst 11.220°),
  convergence **0.94 / 2.73 / 10.10 s**. Route C's rubble is where body
  pitch stops representing the surface, and the tail runs to 20°.
- **τ − tan(grade) = −0.0012 / −0.0034 / +0.0043** over 1,440 episodes.
  τ is pinned by geometry and carries no information about μ. **No
  friction MAE is reported, and none exists to report.**
- The traction bound held on **100.0 %** of single-plane samples on all
  three routes — C2-M2.0 declined to assert this and reported it as a
  rate; measured, it holds.
- Scheduling-input gap on Route A: **0.280 against a μ range of 0.35**.
  Four fifths of the range, unrecovered.
- **Route C is where the observer costs something.** B3 ascends **58.3 %**
  against B1's **84.2 %** — 25.9 points worse than the baseline it falls
  back to — losing ascent on 32 seeds and gaining it on 1, while engaging
  on only 13 % of steps against a grade MAE of 2.681°.
- Route C tips: **B1 106, B3 116** under the surface-relative terminator,
  against Phase 3's 101 under the absolute one. **The population did not
  shrink.** What changed is that the terminator now fires at a genuine
  rear-over instead of 34° short of one. This entry does not claim the
  count improved.
- Tests **478 → 490**, 0 failing.

**Terminology corrected BEFORE the benchmark ran**, not after seeing a
result: `mu_mae`/`mu_bias` → `sched_mu_gap_mae`/`sched_mu_gap_bias`,
`mu_hat` on the wire → `mu_sched_input`, plus new `tau_mean` and
`tau_minus_tangrade_*` columns and a `note` field on `/terrain/state`
stating that true μ is not identifiable.

**On the test count, 471 → 478 before any change.** C2-M2.0's 471 was
measured without the user-space MoveIt prefix on the path, which **skips**
`coco_moveit_config`'s 7 `test_pick_poses` tests. `setup_env.sh` puts it
there; a hand-built environment omits it. Sourced, they pass. **471 was
reproduced exactly in this session on the unmodified tree** before the
prefix was added, so the delta is environmental and not a regression.
`gazebo_models` additionally needs `--ignore=test_integration`, whose
`launch_testing` suite is off by default and kills collection outright.

**Unverified / open**

- **Whether `ascent` is the right decision task.** The evidence above says
  it does not discriminate where the privileged advantage is largest.
  Not resolved here — changing the rule after seeing the result is the
  failure the freeze existed to prevent.
- Whether Route C's tips are avoidable by control at all. The terminator
  is now honest; the population is unchanged.
- Whether B3's Route C behaviour improves with a better grade channel on
  rubble. The correlation is suggestive (2.681° MAE, 10.10 s convergence,
  13 % engagement, worst ascent) and is **not** a demonstrated cause.
- The simulated IMU is still **noiseless**
  (`imu_noise_sigma: not_yet_measured`). Nothing here integrates, and this
  still bounds what any of it claims about a real robot.
- M7 Phase 4's other two gating decisions: untouched.

**Not changed, deliberately:** Nav2, SLAM, AMCL, the map, perception, the
robot model, the terrain geometry, the action space, `cmd_vel_arbiter`,
the reward, the shipped policy, `GOAL_SUMMIT`/`GOAL_MARGIN`, the v1 tip
terminator in all three non-Yard homes, **the tuned schedule, the routes,
the seeds, the decision task and the 10-point margin**. `baselines.py`,
`yard_env.py`, `terrain_observer.py` and `sensor_model.py` are
byte-identical to C2-M2.0, verified with `git diff` before the benchmark
ran.

**Next:** C2-M3, the mission executive. **Do not start it by editing
`traverse_demo.py`** — read `ROADMAP.md`'s C2-M3 block first; the
milestone is about states with entry conditions, timeouts and recovery,
and `/mission/state` is a stepping stone rather than a substitute.

```bash
# reproduce the whole benchmark (~25 min at 8 workers, 12 cores)
python3 -m coco_rl.terrain_benchmark --out docs/data/c2m2_benchmark.json

# re-report, analyse and plot WITHOUT re-running it
python3 -m coco_rl.terrain_benchmark --report docs/data/c2m2_benchmark.json
python3 docs/data/c2m2_analysis.py
python3 docs/data/c2m2_plots.py

# the implementation checks, before trusting any of it
python3 docs/data/c2m2_sanity.py

# the live gate, if the node is ever touched again
ros2 launch gazebo_models full_world_robo.launch.py gui:=false
ros2 run coco_rl terrain_observer --ros-args -p use_sim_time:=true \
    -p declare_flat:=true
ros2 run custom_teleop cmd_vel_arbiter --ros-args \
    -p use_sim_time:=true -p initial_mode:=rl
python3 docs/data/c2m2_live_gate.py /tmp/gate.csv 40 0.35
```

---

## 2026-08-20 — C2-M3.0, the mission is a state machine and it completed a fetch

**Built:**

- `coco_mission/scripts/mission_states.py` — **new**, the machine. Pure
  Python, no `rclpy`, no clock, no I/O: an `Observation` in, a
  `Directive` out. 18 states, a contract table (mode, owner, timeout,
  max retries, retry target, escalation), ~40 structured failure
  reasons, and one uniform failure path through `RECOVERY`.
- `coco_mission/scripts/mission_executive.py` — **new**, the ROS
  adapter. Subscriptions → `Observation`; one idempotent request out per
  state. Publishes `/mission/mode`, `/mission/state` and (only when it
  was told the colour) `/mission/target_colour`. Offers
  `/mission/start` and `/mission/abort`. **No velocity publisher.**
- `coco_mission/launch/mission.launch.py` — starts the executive
  (`executive:=true` by default, `mission_autostart:=false`).
- `coco_mission/scripts/mission_hud.py` — renders the new
  `/mission/state` line, and the `RECOVERY` row finally has a source.
  Both formats render, so `traverse_demo.py` stays readable.
- `gazebo_models/launch/nav.launch.py` — pins `autostart: 'true'` on the
  nav2_bringup include. Interface bug, see below.
- `gazebo_models/scripts/ros_clean.sh` — `mission_executiv[e]`.
- Tests: `test_mission_states.py` (**new**, 62) and
  `test_mission_executive.py` (**new**, 35, every one constructing the
  real node), plus 3 in `test_mission_hud.py`.

`traverse_demo.py` is **unchanged and kept**: it is the harness the
M4/M5/M6 numbers were measured with.

**Measured:**

- **One full fetch completed end to end through the executive**, blue,
  fresh simulator, `gui:=false`, RViz off, never `--fast`. All 15
  nominal transitions in order, `IDLE → COMPLETE`, `result=fetch`,
  **zero RECOVERY entries and zero retries** (`attempts={}`).
- **175.8 s** from `/mission/start` to `COMPLETE`. Per state:
  LOCALIZE 0.1, NAVIGATE_TO_RAMP 14.5, ALIGN_FOR_CLIMB 0.2, CLIMB 13.1,
  VERIFY_CLIMB 0.2, SEARCH_TARGET 0.2, STOW_ARM 3.2, APPROACH_TARGET
  13.1, GRASP 27.5, VERIFY_GRASP 0.2, DESCEND 16.5, RETURN_HOME 69.4,
  PLACE 17.4, VERIFY_PLACEMENT 0.2 seconds.
- **Home to 7 mm.** Final world pose `(-2.0008, +0.0070)` against a
  `(-2.0, 0.0)` goal.
- **Arbiter invariant held: publisher count on
  `/diff_drive_controller/cmd_vel` = 1**, measured before the mission
  started and again after it finished. One `mission_executive` on the
  graph.
- **The descent did NOT reproduce KNOWN PROBLEMS 3b.** `outcome=goal` in
  16.5 s against the 90.1 s timeout seen twice in C2-M1.6 — under light
  load and with RViz off, which is exactly the confound 3b named. One
  run is not a rate and 3b is not closed.
- **Nav home succeeded first time**, no repeat of KNOWN PROBLEMS 1. Also
  one run.
- **Pre-climb heading +0.281 rad (+16.1°)**, measured against ground
  truth at the ramp foot, gate off. An earlier run measured **+0.28**
  and, after re-driving the leg, **+0.26** — see below.
- Tests **490 → 589**, 0 failing. Per package: `coco_config` 70,
  `custom_teleop` 67, `coco_rl` 164, `coco_perception` 44,
  `coco_moveit_config` 12, `coco_sim` 55, `coco_mission` **136**,
  `gazebo_models` 41.

**Two defects the live runs found that no test could have:**

1. **`autostart` leaked into Nav2 and stopped the whole stack.**
   `mission.launch.py` declared a launch argument called `autostart`.
   Launch configurations are inherited by every include, and an
   inherited value shadows the included file's own
   `DeclareLaunchArgument` default — so `nav2_bringup`'s `autostart`
   (default `true`) became `false`. **Every Nav2 lifecycle node came up
   `unconfigured`**: `map_server`, `amcl`, `planner_server`,
   `controller_server`, `bt_navigator`. `/amcl_pose` had **0
   publishers**, and the mission aborted in `LOCALIZE` with
   `NO_LOCALIZATION` after its 40 s budget — correctly, and four layers
   from the cause. **Nothing in any log contained the word `autostart`;**
   it was found with
   `ros2 param get /lifecycle_manager_localization autostart`, which
   answered `False` against a params file that never mentions it. Fixed
   twice over: the mission's argument is now `mission_autostart`, and
   `nav.launch.py` pins Nav2's `autostart` explicitly rather than
   inheriting it. Two tests assert both.

2. **The heading gate was calibrated to the wrong reference and is now
   off by default.** `ALIGN_FOR_CLIMB` originally failed the mission if
   |yaw| exceeded 0.25 rad — nav2_params' own `yaw_goal_tolerance`. It
   fired: the leg arrived at **+0.28 rad** and, re-driven, at
   **+0.26 rad**, and the mission aborted with `ALIGN_HEADING`. Both are
   *inside* Nav2's checker, because **Nav2 judges yaw against the AMCL
   pose it is steering by while this check reads ground truth**, and the
   two differ by the localisation error. Re-driving cannot fix it
   either: the same goal through the same goal checker cannot beat the
   checker's own tolerance, so the retry was structurally futile. The
   mission it aborted is the mission that completes 19/20. Following
   C2-M1's precedent for the HUD's localization verdict, the threshold
   is **not asserted**: the heading is measured, logged and exposed, and
   the gate is off unless `yaw_tolerance` is set to a float.

**One bug found by the unit tests before any live run:** a `RECOVERY`
that timed out was handed to `_fail`, which re-entered `RECOVERY` and
reset its clock — the mission would have sat there for ever with the
robot possibly still moving. It escalates to `ABORT` now.

**Unverified:**

- **Every failure path.** The live run was clean, so `RECOVERY` fired
  only in the two aborted runs (`NAVIGATION_FAILED` ×3 and
  `ALIGN_HEADING` ×2, both retrying and then aborting as specified).
  `skip_grasp`, `CLOCK_STALLED`, `OPERATOR_ABORT`, every worker-outcome
  reason and every timeout are unit-tested and **have not run on the
  robot**.
- `--no-grasp` through the executive has not been run live.
- The HUD's `RECOVERY` row was not read end to end live: `ros2 topic
  echo` truncates the block. The `STATE` row was — it rendered
  `NAVIGATE_TO_RA...` and, in the aborted run, `RECOVERY   (0....`.
- `rviz_2d_overlay_plugins` is still not installed, so the overlay path
  has still never executed.

**Open:**

- **`ALIGN_FOR_CLIMB` has no calibrated threshold and therefore gates on
  nothing but the lane and the ramp foot.** Turning the heading gate on
  needs either a tighter goal checker for that leg (nav2_params already
  defines a `precise_goal_checker` at 0.05 m) or an aligner **behind the
  arbiter**, plus a threshold measured against climbs that actually
  failed. C2-M3.1.
- **One clean run is not a rate.** The standing figure is M6's 19/20 and
  nothing here changes it.
- `grasp_server` writes outcomes containing spaces (`failed at hover`)
  into a space-separated `key=value` line. The executive reads the first
  token and classifies correctly by accident. Not touched — it is a
  pre-existing quirk in a subsystem this milestone must not modify.
- Two publishers on `/mission/mode` is now possible by operator error
  (executive + `traverse_demo.py`). Documented in three places and
  guarded by `executive:=false`; nothing enforces it at runtime.

**Next:** C2-M3.1 — end-to-end mission and recovery behaviours. The
first concrete action is to exercise the failure paths on the robot
rather than only in the harness, starting with `OPERATOR_ABORT` mid-climb
(the one that proves `/ramp/stop` is really reached before the wheels
age out against the arbiter's watchdog).

```bash
# the mission, through the executive
ros2 launch gazebo_models full_world_robo.launch.py traverse:=true gui:=false
ros2 launch coco_mission mission.launch.py rviz:=false \
    policy:=/home/gautham/coco_rl_runs/curriculum_20260726_211008/phase5_24deg_s0p0.zip
ros2 service call /mission/start std_srvs/srv/Trigger
ros2 topic echo /mission/state
ros2 service call /mission/abort std_srvs/srv/Trigger     # the C2-M3.1 test

# the old blocking script, still reproducible
ros2 launch coco_mission mission.launch.py policy:=<abs> executive:=false
ros2 run gazebo_models traverse_demo.py --colour blue

# tests, from inside each package directory
cd coco_mission && python3 -m pytest test -q
cd gazebo_models && python3 -m pytest test -q --ignore=test_integration
```


## 2026-08-22 — C2-M3.1: the failure paths ran on the robot, and the machine did not need changing

**Built:** nothing in the robot. `mission_states.py` and
`mission_executive.py` are **byte-identical to C2-M3.0** — verified with
`git diff` — and that is the result of this milestone rather than an
omission. Five live missions, four deliberately broken, all behaved
exactly as their contracts specify.

The work was instrumentation and injection, all of it outside the repo:
a subscribe-only witness node recording `/mission/state`, `/ramp/status`,
`/cmd_vel_arbiter/status`, `/perception/status`, `/grasp/status`,
odometry, every controller command and the controller topic's publisher
count, stamped on both the simulation and a steady clock; and two witness
nodes that fire an injection on an **observed state**, never on a sleep.

Documentation changed: `docs/RESULTS.md` (new C2-M3.1 section),
`docs/DESIGN_DECISIONS.md` (three entries), `CLAUDE.md` (one trap row).

**Measured — five runs, fresh simulator each, `gui:=false`, RViz off,
never `--fast`:**

| Scenario | Trigger | Retries | Final | Result |
|---|---|---|---|---|
| Operator abort during `CLIMB` | `/mission/abort` on a moving robot | 0 | `ABORT` `OPERATOR_ABORT` | pass, x3 |
| Navigation failure | `--lane 5.0`, goal off the map | 2 | `ABORT` `NAVIGATION_FAILED` | pass |
| Perception failure | `target_blue` removed from the sim | 2 | `ABORT` `TARGET_NOT_FOUND` | pass |
| Manipulation failure | cylinder removed at `GRASP` entry | 2 | `ABORT` `GRASP_FAILED` | pass |
| Retry exhaustion | both escalation targets | max | `ABORT` | pass |

- **All four routes into `RECOVERY` now have a live run**: operator
  request, navigation action status, state timeout, and worker terminal
  outcome. Both escalations — `ESCALATE_ABORT` and
  `ESCALATE_SKIP_GRASP` — were reached.
- **Operator abort, three runs.** Fired only once `/mission/state`
  reported `CLIMB` *and* three consecutive odometry samples exceeded
  0.05 m/s, so the robot was provably climbing under RL control. Service
  replied in **24-32 ms**; last nonzero controller command at **+20 /
  +30 ms**; `CLIMB -> RECOVERY` at **+36 / +44 / +104 ms**; arbiter
  `active=none` at **+44 / +152 / +158 ms**; `RECOVERY -> ABORT` at
  **+180 / +204 / +304 ms**. Travel after the abort: **13.1 / 15.3 /
  23.6 mm**. Velocity below 2 mm/s at **+142 / +220 / +436 ms**.
- **The stop is commanded, not coasted.** Run 1c captured every message
  on the controller topic: **10 explicit zero commands over 0.88 s**
  after the last nonzero one — `cmd_vel_arbiter`'s
  `ZERO_HOLD_SECONDS = 1.0`.
- **No stale command resumed motion in any run.** After the last moving
  odometry sample, `max |vx| = 0.0` and `max |wz| = 0.0` across 50, 264
  and 482 further samples.
- **Retry counts are exact.** `attempts={'NAVIGATE_TO_RAMP': 2}`,
  `{'SEARCH_TARGET': 2}`, `{'GRASP': 2}` — read from the executive's own
  `MISSION ABORT` line, each equal to that state's `max_retries`.
- **Nav2's own words for the unreachable goal**, three times identically:
  `"Goal Coordinates of(2.500000, 5.000000) was outside bounds"`. The
  goal was chosen from the map, not guessed: free cells in
  `coco_world.pgm` span map-y `[-4.585, 3.565]` and the array ends at
  `3.840`. `IDLE -> ABORT` in **1.2 s**, robot never moved.
- **`SEARCH_TARGET` timed out three times at 15.09 / 15.00 / 15.09 s**
  against a 15.0 s contract, then `grasp abandoned; coming home`.
- **`GRASP` failed three times at 13.99 / 15.60 / 15.39 s** against a
  180 s timeout — a genuine worker outcome, not a timeout in disguise.
  `grasp_server` ran its whole unmodified sequence and reported
  `outcome=failed at magnet attach`.
- **No accidental COMPLETE.** Runs 3 and 4 descended, drove home (**120
  mm** and **63 mm** from home) and still ended `ABORT` carrying the
  original reason. A mission that did everything but the grasp reports
  failure.
- **cmd_vel invariant: 1,134 publisher-count samples across five runs,
  every one of them 1** — before each mission, through every recovery
  and retry, and after every abort.
- **0 states entered after `ABORT`** in 5 of 5 runs.
- Tests **589 passing / 0 failing**, unchanged. Per package:
  `coco_config` 70, `custom_teleop` 67, `coco_rl` 164,
  `coco_perception` 44, `coco_moveit_config` 12, `coco_sim` 55,
  `coco_mission` 136, `gazebo_models` 41.
- **Run the suite on a clean ROS graph.** Measured this session: with a
  live stack still up from a mission run, `coco_mission` gives
  **134 passed / 2 failed** — 35 of its tests construct the real node,
  and a populated graph is not the graph they assume. The same suite,
  after `ros_clean.sh`, gives **136 / 0**. The failures are graph
  pollution, not a regression; `ros_clean.sh` before `pytest` is the fix.

**One instrumentation defect, found the expensive way.**
`/diff_drive_controller/cmd_vel` reports **two** types —
`geometry_msgs/msg/Twist` and `geometry_msgs/msg/TwistStamped` — and the
arbiter publishes the second. The first recorder subscribed as `Twist`,
matched no publisher, and captured **zero** commands. That reads exactly
like "no stale command was ever issued", which is the conclusion the test
existed to reach. The run was repeated against `TwistStamped` and the
real answer is stronger than the empty file looked. Recorded in
`CLAUDE.md`'s trap table with the general form: **any check whose success
condition is "we saw nothing" must first prove it can see something.**

**One C2-M3.0 open item confirmed live.** `grasp_server` writes
`outcome=failed at magnet attach` into a space-separated `key=value`
line. `parse_kv` reads `outcome=failed`; the executive classifies
`GRASP_FAILED` **correctly**, but `at magnet attach` — the whole
diagnosis — never reaches the log or `/mission/state`. Not fixed:
`grasp_server` is a subsystem this milestone must not modify, and the
classification does not depend on it.

**Unverified — and this matters for how C2-M3.1 is described.** Four
representative branches ran live. **The following did not** and remain
unit-tested only: `CLOCK_STALLED`, `--no-grasp` through the executive,
`NAVIGATION_REJECTED`, `NAVIGATION_UNAVAILABLE`, `SERVICE_UNAVAILABLE`,
`SERVICE_REFUSED`, `RECOVERY_TIMEOUT`, every `ALIGN_*`, `CLIMB_TIPPED`,
every `DESCENT_*`, `RETURN_*`, `STOW_*`, `APPROACH_*`, `PLACE_*` and
`VERIFY_PLACEMENT`. The correct sentence is "live validation completed
for operator abort, navigation failure, perception failure and grasp
retry" — **not** "the recovery system is validated".

Also unverified: the **no stale completion** invariant. The token
mechanism was never made to race — no late worker reply arrived after a
cancel in any of the five runs — so that invariant is still argued from
the code and the unit tests rather than measured.

**Open:**

- `ALIGN_FOR_CLIMB` still has no calibrated heading threshold; the gate
  is still off and the number still only reported. Untouched by this
  milestone, which had no evidence to calibrate it with.
- **One run is still not a rate.** These five runs say the failure paths
  behave; they say nothing about how often the mission fails. The
  standing figure is M6's **19/20**.
- `KNOWN PROBLEMS 1` (nav home) did not reproduce in any of the three
  runs that drove home. Not closed.
- Two publishers on `/mission/mode` remains possible by operator error.

**Note for whoever runs this next.** The workspace checkout at
`~/ros2_ws/src/coco-robot-ros2` is on the **trunk**, which does not
contain the executive, so `~/ros2_ws/install` cannot run these tests.
This session built the worktree into a separate overlay at
`~/ros2_ws/c2m31_overlay` and sourced it on top, leaving the user's
`~/ros2_ws/install` untouched. `source ~/ros2_ws/c2m31_overlay/env.sh`
reproduces the environment; `bash ~/ros2_ws/c2m31_overlay/build.sh`
rebuilds it.

**Next:** C2-M4 — perception-driven manipulation.

```bash
# the environment these runs used
source ~/ros2_ws/c2m31_overlay/env.sh

# a failure run, end to end (fresh simulator, executive run directly so
# the documented --lane / --no-grasp parameters can be passed)
ros2 launch gazebo_models full_world_robo.launch.py traverse:=true gui:=false
ros2 launch coco_mission mission.launch.py rviz:=false executive:=false \
    policy:=/home/gautham/coco_rl_runs/curriculum_20260726_211008/phase5_24deg_s0p0.zip
ros2 run coco_mission mission_executive.py --colour blue --lane 5.0 \
    --ros-args -p use_sim_time:=true
ros2 service call /mission/start std_srvs/srv/Trigger

# operator abort, on a moving robot
ros2 service call /mission/abort std_srvs/srv/Trigger

# make a target unavailable, without touching coco_perception
gz service -s /world/coco_world/remove --reqtype gz.msgs.Entity \
    --reptype gz.msgs.Boolean --timeout 5000 --req 'name: "target_blue" type: MODEL'

# tests, from inside each package directory
cd coco_mission && python3 -m pytest test -q
```

---

## 2026-08-29 — C2-M4.0: the target pose is measured, and the depth gate has a radius

**Built.** The perception-to-pose half of C2-M4, as two files plus a
node beside `target_finder` rather than inside it:

- `coco_perception/coco_perception/target_pose.py` — **new**, pure. No
  `rclpy`, no `tf2`, no message types. The validity states, the target
  representation, the selection policy, the depth quality metrics, the
  deprojection and the reachability verdicts. Same split C2-M2 made
  between `terrain_observer` and its node, for the same reason: the
  C2-M2.1 live gate's three defects were all in the node and none in
  the arithmetic.
- `coco_perception/coco_perception/target_pose_node.py` — **new**, thin.
  Subscribes, calls `target_pose`, asks `tf2` for one transform,
  publishes. Holds no geometry.
- `coco_perception/test/test_target_pose.py` — **new**, 70 tests.
- `docs/data/c2m4_localisation.py` — **new**, the instrument. Also the
  C2-M4.1 benchmark runner; `--benchmark` is the full grid.
- `coco_perception/setup.py`, `package.xml`,
  `gazebo_models/scripts/ros_clean.sh` — wiring.

`target_finder.py` is **byte-identical**. `/perception/target` still
carries `PointStamped` in `base_footprint` and `approach_server`'s servo
mode still consumes it — the path M6's 20/20 approach ran through.

**New topics.** `/perception/target_pose`
(`vision_msgs/Detection3DArray`), `/perception/grasp_point`
(`geometry_msgs/PoseStamped`), `/perception/target_pose/status`
(`std_msgs/String`, 5 Hz, key=value).

**Measured.** Fresh simulator, clean graph, never `--fast`. Twenty
placements, four colours, five stand-offs, **240 of 240 frames
detected**, every one in `base_footprint` with a frame id and a
validity.

| stand-off 0.35-0.90 m, 16 placements | min | median | max |
|---|---|---|---|
| horizontal error | **1.1 mm** | **1.6 mm** | **2.1 mm** |
| vertical error | 0.7 mm | 1.1 mm | 1.7 mm |
| Euclidean error | 1.3 mm | 1.9 mm | 2.7 mm |

Colour-independent to within 0.8 mm. This is an independent
corroboration of the `~2.0 mm` perception residual
`GRASP_MAX_LATERAL`'s comment has carried since M5 as a budget line; it
is now a measurement.

**The residual is bias, not noise.** `spread_x` and `spread_y` — the
frame-to-frame range at a fixed pose — were **0.0000 m in all 20
placements**. Averaging would buy nothing.

**The estimate tracks a moving target.** The sweep moves the robot;
a second experiment moved the *target* with the robot parked, which a
pipeline that had latched a constant or was reading `lane_for_colour`
would fail. It moved **70.1 mm against 70 mm commanded in x** and
**100.9 mm against 100 mm in y**, and "home" repeated to the last digit
after an excursion.

**One defect found, diagnosed to arithmetic, and NOT fixed.**
`min_range` interacts with the target's own radius. At a 0.28 m
stand-off the camera is 0.155 m from the axis, so a cylinder's near face
sits at `0.155 - r` = 0.145/0.143/0.141/0.139 m — **all under the 0.15 m
gate**. `robust_depth` rejects them and the surviving median is biased
away:

| stand-off 0.28 m | default 0.15 | control 0.11 |
|---|---|---|
| red / green / blue / yellow `dx` | **+4.1 / +5.5 / +6.9 / +8.3 mm** | **−1.0 / −1.0 / −1.3 / −1.4 mm** |

The bias is proportional to radius, which is the signature; the control
changed one parameter and it collapsed to the far-field figure.

**The node announced this itself, without ground truth.**
`hypothesis.score` — the fraction of blob pixels carrying usable depth —
read **1.0000 from 0.35 m out** and **0.0423-0.0706 at 0.28 m**. A
consumer gating on `score` would have refused those measurements. The
quality field justified itself on its first run.

**Left at 0.15 deliberately**: it matches `target_finder`, the operating
envelope starts around 0.30 m anyway because the approach's last leg is
blind below `min_range` by construction, and retuning a gate on one
session's evidence is what the evidence discipline exists to slow down.
One parameter, data recorded, C2-M4.1's call.

**A second, independent close-range effect.** `dz` at 0.28 m was
−4.3 to −5.4 mm and **did not move** when the gate was lowered, so it is
not the gate: it is the framing effect `target_finder`'s docstring
predicted — the cylinder's top has left the frame and the visible
centroid rides down. It costs the grasp nothing: `grasp_point.z` is
`TARGET_GRASP_Z` from the arm's geometry and never comes from the
camera.

**The far-field `dx` bias is explained too.** `SURFACE_TO_AXIS = 0.8`
under-shoots the cylinder's true median offset of `r*sqrt(3)/2 = 0.866r`
by `0.066r` — −0.7 to −1.1 mm across the four diameters, which is what
the −0.4 to −1.5 mm residual is. Recorded, not tuned: it is under a
millimetre and `0.8` is the constant M6 was measured with.

**Reachability reaches the real solver.** `arm_ik` is resolved through
`ament_index` at start-up and injected, so `IK_UNAVAILABLE` is a state
rather than an ImportError. Two verdicts are published because one would
mislead: `reach` read `OUT_OF_WORKSPACE` on all 20 placements, which is
*correct* — the arm reaches base-x 0.157 and perception sees the target
at 0.28-0.90 m — and `reach_appr`, evaluated at `approach_stop_x` with
the measured lateral offset, read `REACHABLE` on all 20. Since the
approach drives straight forward, **perception's `dy` is the whole of
what decides post-approach feasibility**, against
`GRASP_MAX_LATERAL = 0.010`.

**Unverified / not done.** No grasp was attempted. No approach was
driven — the robot was placed with `gz set_pose`. Lateral offsets were
not swept (on-lane only); that is C2-M4.1's grid. The depth camera is
noiseless, so `spread = 0.0000` is a statement about gz and not about a
sensor. `min_range` is diagnosed, not fixed.

**Traps paid for.** The job scratch directory carried a previous
session's `numbers.py` and `trace.py`. Python puts a script's own
directory at `sys.path[0]`, so both shadowed stdlib modules: `numbers`
broke `numpy` at import inside `rclpy`'s parameter service, and `trace`
printed a previous run's mission trace into the middle of this one's
output. Run instruments from a directory you control.

**Tests.** `coco_perception` **41 -> 111 passing, 0 failing**, run from
inside the package directory. Its `flake8` and `pep257` baselines were
clean, so all 14 style errors the new files introduced were fixed rather
than counted against the pre-existing allowance.

**Next:** C2-M4.1 — four-colour benchmark, grasp integration, final
validation. The benchmark runner exists and is parameterised; the grid
is four colours x five stand-offs (0.30/0.40/0.55/0.70/0.90) x three
lateral offsets (0.0/−0.010/+0.030), 60 placements.

```bash
# environment
source ~/ros2_ws/c2m31_overlay/env.sh
bash   ~/ros2_ws/c2m31_overlay/build.sh          # rebuild the overlay

# T1 — fresh simulator, ALWAYS. traverse:=true spawns the targets.
ros2 launch gazebo_models full_world_robo.launch.py traverse:=true gui:=false

# T2 — the node under test. Nav2 and MoveIt are NOT needed to measure
#      the pose: robot_state_publisher alone supplies the TF chain.
ros2 run coco_perception target_pose_node \
    --ros-args -p use_sim_time:=true -p target_colour:=blue

# T3 — the C2-M4.1 benchmark, 60 placements
cd docs/data && python3 c2m4_localisation.py --benchmark \
    --frames 12 --out c2m4_benchmark.csv

# the min_range control that diagnosed the close-range bias
ros2 run coco_perception target_pose_node --ros-args \
    -p use_sim_time:=true -p min_range:=0.11
cd docs/data && python3 c2m4_localisation.py \
    --colours red green blue yellow --standoffs 0.28 0.35

# tests, from inside the package directory
cd coco_perception && python3 -m pytest test -q
```

## 2026-08-29 — C2-M4.1: the benchmark ran, the grasp is perception-driven, and the lateral budget has no headroom

**Built.** One parameter, one instrument, one analysis, and nothing else
touched:

- `coco_perception/coco_perception/target_pose_node.py` — added
  `point_topic`, **empty by default**. Set to `/perception/target` the
  node stands exactly where `target_finder` stood and the whole existing
  manipulation chain — servo, align, creep, `/approach/target`,
  `check_target_pose`, `arm_ik`, MoveIt, the magnet — runs unmodified on
  the C2-M4.0 estimate. That is the entire C2-M4.1 integration.
- `coco_perception/test/test_target_pose.py` — **+6 tests** (73 in the
  file, 117 in the package) pinning the seam: the default is off, the
  publisher is conditional, the **axis** point is published and not the
  grasp point, the stamp is the **image's**, and the publish sits inside
  the `is_valid` branch.
- `docs/data/c2m4_grasp.py` — **new**, the manipulation instrument. One
  perception-driven grasp per invocation, one fresh simulator per
  invocation, and a physical verdict read from gz independently of the
  server's own.
- `docs/data/c2m4_analysis.py` — **new**, post-processing. Reads the
  benchmark CSV, reads nothing live, re-derives the IK verdict from the
  *measured* pose with the same `coco_config` bounds the robot uses.
- `docs/data/c2m4_benchmark.csv`, `docs/data/c2m4_grasp.csv`,
  `docs/data/c2m4_scatter.png` — the data.
- `CLAUDE.md` — one trap row: the grasp and approach services are
  **asynchronous**.

**Not changed, deliberately:** `target_finder.py`, `approach_server.py`,
`grasp_server.py`, `arm_ik.py`, `arm_control.py`, MoveIt, the arbiter,
Nav2, AMCL, the map, the robot model, the world, the action space, the
shipped policy. `GRASP_MAX_LATERAL` and `min_range` were **not retuned**
— see below, both are deliberate.

**Measured — perception.** Fresh simulator, clean graph, sim time, never
`--fast`, configuration fixed before the first placement.

```bash
ros2 launch gazebo_models full_world_robo.launch.py traverse:=true gui:=false
ros2 run coco_perception target_pose_node \
    --ros-args -p use_sim_time:=true -p target_colour:=blue
cd docs/data && python3 c2m4_localisation.py --benchmark \
    --frames 12 --out c2m4_benchmark.csv
```

**60 of 60 placements measured. 720 of 720 frames detected. 0
wrong-colour selections.** Horizontal error **0.7 / 1.4 / 2.4 mm**
(min/median/max). Frame-to-frame spread **0.0000 m in all 60** — bias,
not noise, and a statement about gz's noiseless depth camera rather than
about any real sensor.

Colour-independent to within 0.47 mm of median (blue 1.47, green 1.21,
red 1.37, yellow 1.68 mm). No per-colour branch exists anywhere in the
pipeline and the benchmark says none is needed.

**The error grows with range and it grows in `dy`.** `|dy|` median runs
0.39 / 0.57 / 1.12 / 1.41 / 1.75 mm at 0.30 / 0.40 / 0.55 / 0.70 /
0.90 m while `dx` stays between −1.8 and −0.4 mm throughout.

**The lateral bias is sub-pixel and geometric.** On-lane, `dy` is
identical across all four colours to within 0.01 mm at every stand-off —
four diameters, four lanes, one number. As a bearing it is 1.30 to
1.95 mrad; at the image it is **0.29 to 0.43 pixels**. The node's own
`CameraInfo` log reads `cx=160.00` on a **320-pixel-wide** image, half a
pixel off the geometric centre under the pixel-centre convention, which
is the right sign and order — but the equivalent offset *rises* across
the sweep rather than holding flat, so that does not account for all of
it and the mechanism is **not claimed**.

**The operational consequence.** Because the bias grows with range, the
lateral estimate is best from close in. The approach's last visual fix
lands at ~0.29 m by construction, so the number that actually reaches
the grasp is the ~0.4 mm one, not the ~2 mm one — and that is what the
existing approach already does, with no change.

**`min_range`: decision B — no change, envelope documented instead.**
C2-M4.0 measured `dx` of +4.1 to +8.3 mm at 0.28 m, proportional to
radius. At the operating floor of 0.30 m the defect is **already gone**:
`dx` is −0.68 / −0.97 / −1.27 / −1.58 mm (the ordinary negative
far-field residual) and `qual` reads **0.9989 or better** against
**0.0423-0.0706** at 0.28 m. The gate is rejecting essentially nothing
where the robot actually works. It stays at 0.15 because that is what
`target_finder` uses, because the defect does not occur inside the
envelope, and — the reason that generalises — because **`qual`
announces the failure without ground truth**, so a consumer gating on it
is protected at stand-offs nobody has characterised.

**THE RESULT: the lateral budget has no headroom.**

| commanded lateral | true \|y\| | measured \|y\| min/med/max | feasible (measured) | feasible (truth) |
|---|---|---|---|---|
| 0.000 | 0.0 mm | 0.39 / 0.95 / 1.75 mm | **20 of 20** | 20 of 20 |
| **−0.010** | **10.0 mm = the budget** | 10.22 / 10.52 / 12.22 mm | **0 of 20** | 20 of 20 |
| +0.030 | 30.0 mm | 27.92 / 28.72 / 29.88 mm | **0 of 20** | 0 of 20 |

Three rows, three different reasons, and collapsing them would lose the
result:

- **+0.030 is geometry, not perception.** Three budgets out; measured and
  truth agree perfectly, 0 disagreements in 20. The arm is *planar* —
  both joints rotate about the base y-axis — so an off-plane target is
  unreachable at every joint angle and no sensor could fix it. The
  pipeline refuses on its own measurement, before any motion is planned.
- **0.000 works,** with 8.25 mm of margin at worst.
- **−0.010 is the finding.** The target sits *exactly* on
  `GRASP_MAX_LATERAL`, so `abs(y) > max_lateral` is a tie a perfect
  sensor wins by nothing. The residual is biased **outward**, so the
  measured value lands 0.22 to 2.22 mm over the limit in **20 of 20**.

That is not perception failing — 0.2-2.2 mm against a 10 mm budget is a
good sensor with zero headroom. **`GRASP_MAX_LATERAL` was not moved.**
Moving a decision rule after seeing the cases it rejected is the failure
`DESIGN_DECISIONS.md` already records for the terrain observer. What
C2-M4.1 owes the next session is the number, and the number is on the
record.

**Measured — the perception-driven grasp, live.** Eight runs, **one
fresh simulator each** (the gz `DetachableJoint` binds its child once),
never `--fast`, `target_finder` NOT running, publisher count on
`/perception/target` verified 1 before every run.

Integration is one parameter: `-p point_topic:=/perception/target`.
`approach_server`, `grasp_server`, `arm_ik`, `arm_control` and MoveIt
are **byte-identical**.

| | |
|---|---|
| perception VALID at the start | **8 of 8** |
| approach `arrived` | **8 of 8** |
| `check_target_pose` accepted the perception-derived fix | **8 of 8** |
| IK + MoveIt planned and executed | **8 of 8** |
| **grasp physically verified** (object rose, read from gz) | **8 of 8** |
| **placement physically verified** (object back on its deck) | **7 of 8** |
| fixes inside the window [0.1510, 0.1565] | **8 of 8**, 0.15341-0.15471 |
| median run | 71.0 s |

Four colours at 0.45 m, blue at 0.30 / 0.45 / 0.70 m, blue at laterals
0.000 / −0.010 / +0.030. No per-colour manipulation logic exists and
none was added.

**THE CORRECTION THE LIVE HALF MAKES TO THE STATIC HALF.** Both lateral
placements were judged `OFF_ARM_PLANE` by the static verdict and **both
grasped successfully**:

| lateral | perception `y` | static verdict | `y` delivered to the grasp | live |
|---|---|---|---|---|
| −0.010 | +10.2 mm | OFF_ARM_PLANE | **+1.68 mm** | **grasped, verified** |
| +0.030 | −29.2 mm | OFF_ARM_PLANE | **−3.0 mm** | **grasped, verified** |

`approach_server`'s `align` phase pivots until the bearing is nulled and
only then takes the fix the creep and grasp use, so the offset is
absorbed rather than carried. `reachability_after_approach` models the
approach as translation-only — its docstring says so — and therefore
**under-predicts** feasibility. That is the safe direction for a gate to
be wrong in, but it is a **lower bound, not a forecast**. Not changed;
measured and recorded. Both lateral runs are `n = 1` and 30 mm is the
largest offset tried, not a characterised limit.

**The one failure, and the gap it exposed.** `blue` at 0.30 m: grasp
succeeded, placement did not. With `PLATFORM_Z = 0.64984` and
`TARGET_HEIGHT = 0.158`, a standing cylinder's centre is at **0.72884**
and one lying on its side at `0.64984 + r` = **0.66384**. The instrument
read 0.72884 (standing) right after the approach; `grasp_server`'s own
pre-grasp read was **0.6638 — already down**. The target was **toppled
during the pick sequence**; which motion did it was **not isolated**
(1 of 1 at 0.30 m, 0 of 4 at 0.45 m, 0 of 1 at 0.70 m).

The magnet then welded to the fallen cylinder, lifted it 43.7 mm, and
**`check_lifted` passed** — correctly by its contract, because it did
come up. So:

> **`check_lifted` verifies the object moved up, not that it is
> upright.** A toppled cylinder is lifted, carried and delivered lying
> down, and every step reports success.

Not fixed: deciding what "upright" means for a grasp allowed to be
imperfect is a design decision, not a patch.

**A second unstated precondition, found the same way.**
`grasp_server.check_released` asserts the placed object stands at
`TARGET_HEIGHT / 2` — the floor **at home**. All eight runs place on the
platform, `PLATFORM_Z` higher, so all eight logged "not standing on the
ground (0.0790)" and `/grasp/place` returned failure — **including the
seven that released perfectly**. Correct in the M6 mission, where the
robot *is* at home; the precondition was simply never written down.
Recorded, not fixed. The instrument answers the physical question
against the deck the object actually started on.

**Tests: 662 passing / 0 failing**, up from 656, on a **clean ROS
graph**, run per package from inside each package directory.

```
coco_config 70   custom_teleop 67   coco_rl 164   coco_perception 117
gazebo_models 41  coco_moveit_config 12  coco_sim 55  coco_mission 136
```

**Still unverified.** The full mission through the executive was not
re-run on the new path — these eight runs are the perception -> approach
-> grasp chain in isolation, deliberately, to keep the Gazebo + RViz +
`move_group` confound out. The climb, the lane hold, the descent and the
delivery at home were not exercised. Eight runs is not a rate; the
standing mission figure is still M6's **19/20**.

**Next.** `point_topic` is opt-in and nothing launches with it yet —
`perception.launch.py` still starts `target_finder`, and that is
deliberate until the executive has run a full mission on the new path.
The next concrete step is exactly that: a full `mission.launch.py` fetch
with `target_pose_node` driving `/perception/target` in
`target_finder`'s place, which is the run that would let the default
move.

## 2026-08-29 — C2-M4.2: the swap needed a second topic, and the mission completed on it

**The task was an integration gate, not a milestone:** run one full
fetch through the real mission executive with `target_pose_node` in
`target_finder`'s place, and prove the C2-M4 pose survives the trip.
It did — but not with the handover C2-M4.1 left behind, and the missing
half was found by reading rather than by spending a run on it.

**The defect, found statically before the simulator was started.**
C2-M4.1's `point_topic` feeds `approach_server` through
`/perception/target`, and that is genuinely all the *manipulation* chain
needs. The *executive* needs something else:
`mission_states._check_search_target` gates `SEARCH_TARGET` on
**`/perception/status`** reading `found=1` with a matching `sel`.
`target_pose_node` publishes `/perception/target_pose/status`, a
different topic whose key set has no `found` in it at all.

So the obvious swap — kill `target_finder`, set `point_topic`, run —
fails like this: zero publishers on `/perception/status`,
`obs.perception.newer_than(entered_at)` never true, `SEARCH_TARGET`
never leaves RUNNING, and the mission dies on the state's 15 s timeout
with `TARGET_NOT_FOUND`. A topic-name problem wearing a perception
diagnosis. **First broken boundary: the subscriber assumption** — not
the message type, not the QoS, not the frame, all three of which were
already compatible (`geometry_msgs/PointStamped`, depth 10, RELIABLE,
`base_footprint`).

**Built.** Four files, and no algorithm in any of them:

- `coco_perception/coco_perception/target_pose.py` — new pure function
  `finder_status_fields(observation)`, mapping a `TargetObservation`
  onto `target_finder`'s `/perception/status` fields. Returns a dict, so
  the module stays free of any `target_finder` import and the *format*
  has exactly one definition, in `target_finder.format_status`, which
  the node calls with these fields. Geometry is gated on `is_valid`,
  mirroring `target_finder`: a compat line whose whole purpose is
  substitutability has to be substitutable in behaviour, not merely in
  key names. `lane` and `age` render `--` because this pipeline computes
  neither and a plausible invented number is the failure the `--`
  convention exists to prevent.
- `coco_perception/coco_perception/target_pose_node.py` —
  `status_compat_topic`, **empty by default**, exactly like
  `point_topic`. Set to `/perception/status` the node answers the
  vision gate with its own verdict, `found=1` iff `validity == VALID`.
  Published on the existing 5 Hz status timer, so it keeps arriving
  whether or not a frame did — the executive ages the topic against the
  state's entry time.
- `coco_perception/launch/perception.launch.py` — `target_source`,
  `target_finder` (default) or `target_pose`, dispatched in an
  **`OpaqueFunction`**. Not two `IfCondition`s: two conditions over one
  argument can both be false on a typo, which launches a mission with no
  perception at all, and both can be true if someone edits one and not
  the other, which is two estimates racing for `/perception/target` with
  the grasp taking whichever landed last. The function returns a
  one-element list and raises on an unknown value. It also sets **both**
  handover parameters together, because setting one without the other is
  precisely the defect above.
- `coco_mission/launch/mission.launch.py` — declares `target_source` and
  forwards it. That is the whole of the mission-side change.

**Not changed:** `target_finder.py`, `approach_server.py`,
`grasp_server.py`, `arm_ik.py`, `arm_control.py`, `mission_states.py`,
`mission_executive.py`, MoveIt, Nav2, AMCL, the arbiter, the map, the
robot model, the world, the action space, the policy. The default is
still `target_finder`, so the path M6's 19/20 was measured on is
untouched and still what a bare `mission.launch.py` starts.

**Measured — one full fetch, and it completed.** Fresh simulator, clean
graph, sim time, `rviz:=false`, never `--fast`, publisher counts checked
before *and* after.

```bash
ros2 launch gazebo_models full_world_robo.launch.py traverse:=true gui:=false
ros2 launch coco_mission mission.launch.py rviz:=false \
    target_source:=target_pose target_colour:=blue policy:=<zip>
ros2 service call /mission/start std_srvs/srv/Trigger
```

**COMPLETE, all 16 states, `retries=0`, `reason=--` at every sample,
178 s** from LOCALIZE to COMPLETE. `/perception/target` and
`/perception/status` each had **exactly one publisher, `target_pose_node`**,
before and after; `target_finder` never ran; one executive; `/amcl`
`active [3]`. Both legacy consumers of `/perception/status` —
`mission_executive` and `mission_hud` — took the compat line unchanged.

The chain, measured: first `found=1` was
`sel=blue found=1 u=168 v=162 area=30 w=5 h=6 range=1.378 x=1.503
y=-0.050 z=-0.189 lane=-- seen=green,blue,yellow age=--`, and
`SEARCH_TARGET` passed on the first sample after entry. **62 `found=1`
samples and 62 `validity=VALID` samples — the same number**, which is
the check that `found` is exactly `validity == VALID`. 190 points on
`/perception/target`. Approach `outcome=arrived`, travel 1.139 m,
bearing nulled to `-0.000`. Grasp `x=0.1540 lifted=1 outcome=held`, then
`outcome=placed` — **0.1540 is inside the 5.5 mm window
[0.1510, 0.1565]**, and it came from the camera.

`RETURN_HOME` succeeded in 59.9 s. That is KNOWN PROBLEMS 1's leg, and
it is now the second consecutive success under light load with RViz off.
**Three of six recorded legs have failed; six is still not a rate** and
it stays open for C2-M5.

**Tests: 684 passing, 0 failing** (was 662). All 22 new tests are in
`coco_perception`, which moves 117 → 139: twelve on the compat line
(`found=1` only when VALID, `found=0` for each of the five non-VALID
states, the key set is `target_finder`'s exactly, geometry withheld
unless valid, `range` is the axis and not the surface, `lane`/`age`
absent rather than invented), four on the parameter (off by default,
conditional publisher, separate from the node's own status topic,
published on the status timer), and six on the launch invariant (each
source builds **exactly one** node, the two are different executables,
an unknown value **raises**, `target_pose` sets **both** handover
parameters, and the default is still `target_finder`). Run per package,
cwd inside each, on a clean graph.

**What this is not.** One run. The standing mission figure is still
M6's **19/20**. This is an existence proof that the swap works through
the executive — not a rate, not a comparison against `target_finder` on
the same course, and no claim the new path is better. It is measured to
**work**, not to win.

**Two known verification limitations, deliberately untouched.**
`VERIFY_PLACEMENT` passed here, and that is a precondition holding, not
a fix: `check_released` asserts the floor height **at home**, and this
mission places at home. C2-M4.1's finding that it fails every correct
*platform* placement stands, and the platform figure stays **7 of 8**.
`check_lifted` still verifies the object moved **up**, not that it is
**upright**. Neither was changed; the gate did not require it.

**Next.** C2-M5 — localization health and recovery. `RETURN_HOME` and
M6's run 15 are both its benchmark. Read `docs/ROADMAP.md`'s C2-M5 block
first.

```bash
# reproduce this run
source ~/ros2_ws/c2m31_overlay/env.sh
ros2 launch gazebo_models full_world_robo.launch.py traverse:=true gui:=false
ros2 launch coco_mission mission.launch.py rviz:=false \
    target_source:=target_pose policy:="$COCO_POLICY"
ros2 topic info -v /perception/target | grep -i 'publisher count'  # must be 1
ros2 lifecycle get /amcl                                           # active [3]
ros2 service call /mission/start std_srvs/srv/Trigger
```

## 2026-08-31 — C2-M5.0: covariance is the wrong signal, and the wheel path has a loop

**Milestone:** C2-M5.0, localization health characterization. The first
of two C2-M5 sessions. **No recovery was implemented, deliberately** —
the rule for this session was OBSERVE → CLASSIFY → DEFINE, and only then
RECOVER.

**Branch:** `coco2-m1-observability`. State layer on `coco2-state`.

### What was built

* **`docs/data/c2m5_locrec.py`** — a subscribe-only recorder for the
  whole localization stack at 10 Hz: AMCL pose and covariance, `map->odom`
  and its age, wheel odometry, the four stages of the command chain,
  collision-monitor state, `navigate_to_pose` status, the plan, RTF, and
  a `gt_`-prefixed ground-truth block for **offline scoring only**. It
  also computes, from the map and the laser alone, the **likelihood field
  `nav2_amcl` scores particles against and never publishes**.
  `--topology` prints the command chain off the live graph.
* **`docs/data/c2m5_analysis.py`** — per-state scoring and the
  healthy-vs-bad range table.
* **`coco_mission/scripts/localization_health.py`** — the pure health
  core. **Imported by nothing**, by design. 30 unit tests.
* Added `c2m5_locre[c]` to `ros_clean.sh`.

### What was measured — five missions, fresh simulator each, never `--fast`

| run | injection | RETURN_HOME | outcome |
|---|---|---|---|
| `healthy1` | none | 80.3 s | **COMPLETE**, home to 0.078 m |
| `healthy2` | **none** | 12.0 s, 3 attempts | **ABORT** |
| `obstacle1` | a cylinder into the corridor | 50.0 s | **COMPLETE**, home to 0.079 m |
| `diverged1` | `/initialpose` −3 m in y, tight covariance, plus heading error | 131.5 s | **ABORT** `RETURN_FAILED` |
| `diverged2` | the same, heading preserved | 24.7 s | **ABORT** `RETURN_FAILED` |

**`healthy2` failed with no injection at all** — the spontaneous
return-home failure KNOWN PROBLEMS 1 describes, caught with
instrumentation running for the first time.

### The three findings

**1. AMCL's covariance does not detect a divergence, and points the wrong
way.** `sigma_xy` fell to **0.070 m** — below anything in either leg that
finished — at the instant the pose became 3 m wrong, and took **24.5 s**
(13.9 s on the second run) to pass the healthy maximum. On common ground
the run that was 3.14 m wrong had the **lowest** covariance of all five
(0.281 vs 0.370/0.389/0.372). `healthy2`, the uninjected failure, had the
lowest whole-leg median of all five. Part of the dip is imposed by the
injection; the time AMCL took to notice is not.

**2. The scan-vs-map likelihood detects it in 0.4 s, replicated on both
divergence runs**, and stayed outside the healthy envelope for 62.6% and
91.5% of those legs. It is computed from the map, the laser and TF — no
ground truth.

**3. The command chain loops, and the collision monitor's gating never
reaches the wheels.** `nav2_bringup` remaps `controller_server` and
`velocity_smoother` to `/cmd_vel_nav`; `nav.launch.py arbiter:=true`
points `cmd_vel_relay`'s **output** at the same topic. Confirmed on the
live graph: **7 publishers, 2 subscribers**. Measured at the wheels, the
robot receives **10.15–10.77 Hz more than the collision monitor
publishes** — exactly `controller_frequency: 10.0` — and during an active
SLOWDOWN, gated cap 0.090 m/s, wheel commands reached **0.300 m/s** on
84.2% of `obstacle1`'s slowdown samples. **A safety defect, not a
localization problem, and NOT fixed** — the wheel path is frozen and this
milestone's job was to characterize.

### What the evidence does not support

**No threshold was picked.** Class A separates at almost any value.
Class B does not separate: on common ground the gap between the worst leg
that finished and the best that failed is **0.054 m**. `Thresholds` in
`localization_health.py` therefore has **no defaults** and cannot be
constructed without naming every number; `classify()` returns `UNKNOWN`
rather than guess, and `UNKNOWN` is falsy so `if health:` cannot read it
as good news.

**Collision-monitor activity is not the discriminator, in either
direction.** `obstacle1` (finished) and `diverged1` (aborted) logged the
**same 36 PolygonLimit entries**. `diverged2` was 3.2 m wrong with the
monitor at `DO_NOTHING` for the entire leg. And
`/collision_monitor_state` is **edge-triggered**: `healthy1` received
**zero messages in 219.7 s**, so silence and "not running" are identical
to a subscriber.

**Not reproduced:** the 2026-08-17 `PolygonStop` stall, and the 4.8 Hz
control loop. RTF never fell below 0.818 and `/scan` held 10 Hz in all
five runs, with RViz off throughout — consistent with the degradation
being load-induced, and not establishing it. Both stay open.

### Two defects found in my own instrumentation, and what they cost

* **`/mission/state` is a whole `key=value` line, not a label.** Reading
  it raw made every 2 Hz republication look like a transition and meant
  `--stop-on-terminal` could never match. Fixed in the recorder and in
  the injector, where it would have meant the injection silently never
  fired.
* **The first recorder ran on the system clock, not sim time.** Every
  `*_age` column came out as the Unix epoch and `rtf` was
  d(wall)/d(wall) ≡ 1.000 — a number that looks like a healthy simulator
  and is a tautology. `use_sim_time` is now forced, with the tick timer
  on a steady clock so a stalled `/clock` is still recordable.
  `healthy1`'s age and RTF columns are excluded from the results; its
  other columns are unaffected and are used.

**And a frame trap worth the line it costs.** `/amcl_pose` is in the
**map** frame, `/model/coco/odometry` in Gazebo's **world** frame, and
map (0,0) is world (−2, 0) — `mission_states.WORLD_TO_MAP_X`, which
already existed. Subtracting them raw makes the healthy run read as
**2.2 m of localization error on a mission that finished 0.078 m from
home**.

**Tests: 714 passing, 0 failing** (was 684). All 30 new tests are in
`coco_mission`, 136 → 166. Run per package, cwd inside each, on a clean
graph.

### Next

**C2-M5.1 — localization recovery and mission resume.** The requirements
it inherits are in `RESULTS.md`, "Recovery requirements for C2-M5.1". The
first one is the awkward one: **the collision monitor cannot be relied on
to stop the robot**, so the stop must be the arbiter's and must be proved
at the arbiter.

```bash
# reproduce any run in this session
source ~/ros2_ws/c2m31_overlay/env.sh
ros2 launch gazebo_models full_world_robo.launch.py traverse:=true gui:=false
ros2 launch coco_mission mission.launch.py rviz:=false \
    target_source:=target_pose policy:="$COCO_POLICY"
ros2 lifecycle get /amcl                                    # active [3]
python3 docs/data/c2m5_locrec.py --topology                 # 7 pubs on /cmd_vel_nav
cd docs/data && python3 c2m5_locrec.py --out run.csv --events run_events.txt \
    --tag mytag --hz 10 --map ../../gazebo_models/maps/coco_world.yaml \
    --stop-on-terminal &
ros2 service call /mission/start std_srvs/srv/Trigger
python3 docs/data/c2m5_analysis.py docs/data/c2m5_*.csv --states --compare
```


## 2026-08-31 — C2-M5.1: localization health, recovery, and what it cannot fix

**Built.** `localization_monitor.py`, the ROS face of C2-M5.0's pure
`localization_health.py`; a `RELOCALIZE` state in `mission_states.py`
reached only from `RECOVERY` and only for a localization failure; the
`/reinitialize_global_localization` + Spin recovery in
`mission_executive.py`; `docs/data/c2m51_hrec.py` (health recorder) and
`docs/data/c2m51_inject.py` (the C2-M5.0 class-A injection, written down
for the first time); `HOW_TO_RUN.md`.

**Measured.**

* The threshold, from the committed C2-M5.0 CSVs and not from a search:
  `lik_mean_d > 0.40 m`, strictly above every gated sample on a leg that
  finished (largest 0.3851).
* Experiment 1, one healthy mission with the signal published and unread:
  **COMPLETE**, 1714 samples, **0 INCONSISTENT on mapped ground**.
* Experiment 4, the final nominal mission with everything on:
  **COMPLETE in 184 s, `attempts={}`, 0 triggers**, wheel-topic publisher
  count 1.
* Detection latency for the class-A injection: **3.33 s, 4.52 s, 82.9 s**
  across three runs. Highly variable, and that is a property of the
  signal.
* Safe stop: `RECOVERY → RELOCALIZE` in **0.30 s / 0.40 s**, proved at
  the arbiter (`active=none`), never by a dwell.
* Recovery duration, entry to health re-verified: **9.1 s to 33.9 s**.
* Tests **829 passing / 0 failing**, up from 714. All 115 new ones are in
  `coco_mission` (166 → 281).

**Five defects found live and fixed, each with a test.** The node built
its own `Thresholds` and silently kept a default; `amcl_age` is not a
staleness test on an event-driven topic; strict-contiguity persistence
discarded real evidence; the two latches could both be set; the
mapped-ground gate was x-only and blanked the corridor the robot drives
home through; the recovery shared a retry budget Nav2's own abort had
already spent; and the resume did not wait for the spin to finish.

**Unverified, and stated as such.** No live run produced degradation →
recovery → resume → **COMPLETE**. The recovery restores the health
signal but not reliably a pose Nav2 can plan from, for two measured
reasons: `recovery_alpha_fast/slow: 0.0` means AMCL cannot escape a
confident wrong mode, and global relocalization on this near-rectangular
map converged to world (2.60, −0.64) — inside the wedge — after which
the planner reported "Start occupied". Evidence in
`docs/data/c2m51_planner_after_recovery.txt`.

**Not touched, deliberately.** The `/cmd_vel_nav` loop and the collision
monitor's gating. AMCL's parameters — `recovery_alpha_*` is diagnosed,
not changed, because tuning AMCL to make one mission succeed is the thing
NEXT EXACT ACTION forbids.

**Next command to run:**

```bash
source ~/ros2_ws/c2m31_overlay/env.sh
cd ~/ros2_ws/src/coco-robot-ros2/coco_mission && python3 -m pytest test -q
```

---

## 2026-08-31 — Release: one repository, one branch, and the numbers re-measured

**What this session was.** No feature work, by instruction. Consolidate,
verify, document, release. COCO 2.0 is frozen at C2-M5.

**The consolidation.** The work was split across two branches by
`docs/STATE_PROTOCOL.md`: implementation on `coco2-m1-observability`,
and `PROJECT_STATE.md` / `docs/ROADMAP.md` / `docs/STATE_PROTOCOL.md` on
`coco2-state`. Both descend from the trunk at `33110a6`. Neither is
readable alone, so the release is their union, taken as a real merge
rather than a copy. The only conflict was `.gitignore` and both sides
were kept. Verified afterwards by diffing the merge against each parent:
**no file from either side is missing**, and both trunks are ancestors.
The ancient `main` (a Layer-1 stub) is also an ancestor, so
fast-forwarding it loses nothing.

**Measured this session, on the consolidated tree:**

* **Tests: 829 passing, 0 failing, 0 skipped.** Per package, cwd inside
  the package, clean ROS graph, against a fresh overlay built from the
  release tree: `coco_config` 70, `custom_teleop` 67, `coco_rl` 164,
  `coco_perception` 139, `gazebo_models` 41, `coco_moveit_config` 12,
  `coco_sim` 55, `coco_mission` 281. Run twice — before and after the
  documentation work — with the same result.
* **One nominal fetch mission: COMPLETE.** Fresh simulator,
  `gui:=false`, `rviz:=false`, never `--fast`,
  `target_source:=target_pose`, colour blue. All four pre-start
  invariants passed (AMCL `active [3]`, publisher count 1 on each of
  `/perception/target`, `/perception/status`, `/localization/health`,
  `/diff_drive_controller/cmd_vel`). All 16 nominal states,
  **`attempt=1` on every sample and `reason=--` throughout**, **186.7 s**.
  Grasp verified from Gazebo ground truth: **lifted 35.1 mm**
  (z 0.7288 → 0.7639), inside the M6 band of 33.9–35.9 mm.
  `place finished: placed`. Final line:
  `MISSION COMPLETE: result=fetch reason=-- attempts={}`.
* **Localization health on that mission: 0 triggers.** `degraded=0` on
  **all 5,784 samples**; 5,173 `CONSISTENT`/`OK` and 611
  `UNKNOWN`/`OFF_MAPPED_GROUND` — the ramp and platform, excluded by the
  mapped-ground gate by design. Committed as
  `docs/data/release_nominal_mission.txt`.

**One field that reads like a failure and is not.** The `/mission/state`
status line's `retries=` field is the contract's `max_retries` **budget**
(`contract.max_retries`), not a count; `attempt=` is the counter. A first
pass at scoring the run flagged 344 samples as retried because it matched
`retries=[1-9]`. The run used no retries at all.

**Corrections made to the documentation, each checked against source:**

* **The executive has 19 states, not 16.** 16 is the *nominal path*;
  `RECOVERY`, `RELOCALIZE` and `ABORT` are the other three. Both numbers
  now appear, each labelled with its frame.
* **Nine packages, not eight.** Eight carry test suites; `coco_web` has
  no `test/` directory.
* **The `CLAUDE.md` test baseline was 404**, four milestones stale.
* **`CLAUDE.md` contradicted itself** on the six "pre-existing"
  `flake8`/`pep257` failures: one paragraph explained they were a
  wrong-cwd artefact, the next still counted them as a standing
  allowance. Resolved in favour of the measurement.
* **`PROJECT_STATE.md` claimed "C2-M5 … NOT STARTED"** while C2-M5.0 and
  C2-M5.1 were both complete — exactly the drift the file exists to
  prevent.

**Documentation.** `README.md` rebuilt for a reader who has never seen
the milestone numbering: what the robot does, then measured results, then
**Known limitations** stated plainly, then the engineering lessons, and
only then the historical M0–M6 / M7 tracks. `HOW_TO_RUN.md` gained the
Clone section it lacked and lost the side-overlay instructions, since the
branch they worked around is now merged; every launch file, RViz config,
executable and `docs/data` script it names was checked to resolve against
a build of this tree. `PROJECT_STATE.md` frozen: 1764 lines to 1294, with
every section carrying measured evidence kept verbatim. `ROADMAP.md`
closed and C2-M6…C2-M9 relabelled *scoped, not undertaken*.
`STATE_PROTOCOL.md` marked historical — a clone of `main` is now
sufficient and no branch hides code.

**Two limitations are deliberately kept prominent** rather than softened,
in `README.md`, `PROJECT_STATE.md` and `CLAUDE.md`: severe confident AMCL
divergence is **detected but not reliably recovered** to a Nav2-plannable
pose, and the `/cmd_vel_nav` topic loop means **the collision monitor's
gating does not reach the wheels**. Neither was fixed. Both are
characterized with the runs that show them.

**Next command to run:**

```bash
source ~/ros2_ws/src/coco-robot-jazzy-2.0/setup_env.sh
cd ~/ros2_ws/src/coco-robot-jazzy-2.0/coco_mission && python3 -m pytest -q
```

---

## 2026-08-31 — Public release: one demo, and the policy ships with the repo

**What this session was.** No feature work, by instruction. Simplify the
public entry point so a stranger can clone, build, launch once and watch
the fetch. The autonomy was not touched.

**The problem with the previous guide.** It asked the reader to run four
demonstrations — fetch, terrain, perception, localization — each with its
own terminals, invariant checks and harness scripts. Those are components
of one mission, and presenting them separately made a finished robot read
as a workspace. `HOW_TO_RUN.md` is now one flagship demo: **540 lines to
235**, four demos to one, three commands.

**Three setup defects fixed, each of which blocked a documented command:**

* **`COCO_POLICY` is gone.** The trained ramp policy was a file on the
  author's machine that the reader had to find and export. It is 149 KB.
  It now ships at `coco_rl/policies/phase5_24deg_s0p0.zip` (md5
  `1421ce4af745a8f60f5591efedcdc485`, byte-identical to the curriculum
  artefact), installs to `share/coco_rl/policies/`, and is
  `mission.launch.py`'s `policy` default, resolved through the ament
  index so it carries no machine-specific path. `.gitignore` gains one
  negation; `*.zip` still ignores every other training artefact.
* **`rosdep install` now works.** `coco_sim/package.xml` line 13 contained
  `pip --user install` inside an XML comment, and `--` is not legal there,
  so rosdep refused the whole tree. **Measured both ways: exit 1 before,
  exit 0 after.** The guide can now document the dependency step instead
  of apologising for it.
* **`docs/RUNNING.md` no longer hard-codes `/home/gautham/`.** Five
  `policy:=` invocations lost their absolute paths; the policy defaults.

**Measured this session, on this tree:**

* **Flagship mission: COMPLETE.** Exactly the three commands the new guide
  documents, no policy argument, Gazebo GUI on, `rviz:=false`, fresh
  simulator, never `--fast`. All four pre-start invariants passed. All 16
  nominal states in order, **`attempt=1` and `reason=--` throughout**,
  **303 s**, grasp verified from Gazebo ground truth at **35.1 mm** lift
  (z 0.7288 → 0.7639), `place finished: placed`,
  `MISSION COMPLETE: result=fetch`. Committed as
  `docs/data/release_flagship_mission.txt`.
* **The run before it aborted, and it is in that file too.** With both
  renderers on it fetched correctly and then failed `RETURN_HOME`:
  `planner_server` refused every path because AMCL came off the descent at
  (6.14, **4.20**), outside the global costmap. Known failure class,
  reported rather than dropped.
* **Tests on the three changed packages: `coco_rl` 164, `coco_sim` 55,
  `coco_mission` 281 — all passing**, cwd inside each package, clean ROS
  graph. Unchanged from the 829 baseline.
* **Build: `Summary: 9 packages finished`, 0 errors.**

**Repository shape.** `release-consolidation` was merged into `main` as a
fast-forward and every other branch deleted, local and remote, after
verifying each tip is an ancestor of `main`. One branch, `main`, is now
the whole project.

**One thing deliberately not done.** 303 s is not a timing result — the
Gazebo window was rendering and `RETURN_HOME` took 161.7 s against the
headless 81.0 s. The measured nominal stays **186.7 s** from
`release_nominal_mission.txt`. The new guide says so.

**Next command to run:**

```bash
cd <clone> && source ./setup_env.sh
ros2 launch gazebo_models full_world_robo.launch.py traverse:=true
```

---

## 2026-09-01 — C2-NAV.0: diagnosing the wall/enclosure stall

**Branch `worktree-c2nav0-diagnosis`, off `main` at `ea66155`.
Diagnosis only — `nav2_params.yaml` was not touched, no test was edited,
no fix was applied.**

**What was built.** `gazebo_models/scripts/nav_bench.py`: a seven-leg
`NavigateToPose` tour that records the whole command chain
(`/cmd_vel_nav` → smoother → collision monitor → relay → wheels) against
Gazebo ground truth, plus DWB's `/evaluation`, which carries the
per-critic score of every sampled trajectory and the name of the critic
that rejected each illegal one. Ground truth is read for evaluation only.
Analysis lives in `docs/data/c2nav0_analysis.py` (`table`, `chain`,
`arith`) and `docs/data/c2nav0_footprint.py`.

**Measured this session, 42 legs across two topologies:**

* **Topology A** (`nav.launch.py` alone): **16/21 legs**, median transit
  speed **0.208 m/s** against `max_vel_x: 0.30`, DWB at **8.76 Hz**
  against `controller_frequency: 10.0`.
* **Topology B** (`arbiter:=true`, what `mission.launch.py` runs):
  **14/21**, **0.155 m/s**, **7.97 Hz**, 45 progress-checker failures
  against 27.
* `wall_adjacent` failed 2/3 and 3/3; **`enclosure_entry` failed 0/3 in
  both**.

**Three mechanisms, separated:**

1. **A third of every leg is spent rotating on the spot.** Every caller
   sends `orientation.w = 1.0`, the goal checker wants yaw within 0.25
   rad, and `FollowPath.xy_goal_tolerance` (0.05) disagrees with
   `goal_checker.xy_goal_tolerance` (0.25) by 5×, so between them there
   is no rotate-in-place mode at all. RotateToGoal is **50.7 %** of all
   trajectory rejections. `open_space` drove 2.2 m in 9 s cleanly, then
   stood still for 10 s.
2. **BaseObstacle dominates wherever clearance < `inflation_radius`.**
   The critics are scaled in different units — `MapGridCritic::getScale()`
   is `resolution * 0.5 * scale`, BaseObstacle's is `scale` on a 0–252
   cost — so advancing one cell toward the goal is worth **1.40** and
   advancing one cell into the inflation gradient costs **128–454**. With
   `sum_scores: false` the score is the trajectory's final pose, so the
   cheapest command in a rising cost field is zero. **Measured: the robot
   stopped 1.149 m short of the goal for 47.8 s with 777 of 819
   trajectories valid**, BaseObstacle **93.4 %** of the chosen
   trajectory's score, and the zero originating at `/cmd_vel_nav`.
3. **The collision-monitor zones are squares**, so `PolygonSlow` reaches
   0.566 m rather than 0.40 and `PolygonLimit` 0.778 m rather than 0.55.
   `wall_adjacent` held `SLOWDOWN` for 57.25 s with the nearest return at
   0.498 m. Aggravates; does not cause.

**Two things measured to be the opposite of the suspicion:**

* **`robot_radius: 0.20` is 5.1 mm too SMALL.** The circumscribed radius
  is **0.2051 m**, measured from live TF by transforming every collision
  box's corners and both rims of every wheel cylinder. The first attempt
  said 0.2199 m because it used bounding boxes for the cylinders; that
  was wrong by 15 mm and is corrected in the committed script.
* **The `/cmd_vel_nav` ownership loop is not the stall.** It is real —
  wheels exceed the collision monitor's output on **0.06 %** of samples
  without it and **14.0 %** with it, worst case 0.300 m/s against a
  commanded 0.0 — and it costs 25 % of transit speed, but
  `enclosure_entry` fails 0/3 either way.

**Incidental:** topology A drops **233** wheel commands as stale and
topology B drops **0**, because `cmd_vel_relay` republishes with the
original `header.stamp` and `cmd_vel_arbiter` re-stamps.

**Tests.** None run. `nav_bench.py` is a new standalone diagnostic script
with no importers; no existing source, launch file or test was modified,
so the 829 baseline is untouched and re-running it would prove nothing
about this change.

**Next command to run** — C2-NAV.1, and only proposal 1 first:

```bash
ros2 launch gazebo_models full_world_robo.launch.py gui:=false
ros2 launch gazebo_models nav.launch.py
ros2 run gazebo_models nav_bench.py --tag navA_goalyaw --repeats 3 --timeout 75
```

---

## 2026-09-01 — C2-NAV.1: the terminal yaw, tested alone

**Branch `worktree-c2nav0-diagnosis`, continuing from `8f05c45`. A
single-variable experiment.** One Nav2 parameter changed; no source
file, no launch file and no test was edited.

**The one change.** `gazebo_models/config/nav2_params.yaml`,
`controller_server.goal_checker.plugin`: `nav2_controller::SimpleGoalChecker`
→ **`nav2_controller::PositionGoalChecker`**, Nav2's own "only checks XY
position and ignores orientation" plugin, which ships in Jazzy.
`xy_goal_tolerance` stayed 0.25 and `stateful` stayed true, so the
arrival test — and every metric measured against it — is the one the
baseline used. `yaw_goal_tolerance` is not declared by the new plugin, so
the yaw requirement is *removed* rather than widened and there is no
tolerance left to tune.

Chosen over the scope's other option (raise `yaw_goal_tolerance` toward
π) for that reason. Nav2 launched with `params_file:=` pointing at the
worktree copy, because `install/share` symlinks to the trunk checkout.

**Verified single-variable, off the live node rather than the file.**
`Created goal checker : goal_checker of type
nav2_controller::PositionGoalChecker`;
`goal_checker.yaw_goal_tolerance` → **"Parameter not set"**;
`goal_checker.xy_goal_tolerance` 0.25; and `FollowPath.xy_goal_tolerance`
0.05, `RotateToGoal.scale` 32.0, `BaseObstacle.scale` 8.0, `sum_scores`
False, `vx_samples` 20, `vtheta_samples` 40, `sim_time` 1.5,
`controller_frequency` 10.0, `robot_radius` 0.20, `inflation_radius` 0.50
all unchanged.

**Measured — 21 legs, topology A, three repeats, same tour, same 75 s
timeout, fresh headless sim, robot verified at spawn:**

* **16/21 → 18/21.** Median leg **20.31 → 12.75 s (−37 %)**, median
  transit time flat at 11.42 → 11.20 s — the whole saving is after
  arrival. Median transit speed 0.208 → 0.228 m/s.
* **`RotateToGoal` rejections 465 063 → 0.** Total rejections −68 %,
  median DWB illegal fraction 0.170 → 0.004, progress aborts 27 → 13.
* **`wall_adjacent` 1/3 → 3/3**, 77.34 → 4.22 s.
* **Clearance improved**: median 0.419 → 0.486 m, worst **0.273 →
  0.331 m**. The speed was not bought from the obstacle margin.
* **`enclosure_entry` 0/3 → 0/3**, and its stall got *longer*: 47.8 →
  58.9 / 62.7 / 66.2 s, stopped 1.31–1.35 m short with 637–677 of 819
  trajectories still legal.

**Verdict: PARTIALLY CONFIRMED.** Terminal yaw was a large contributor
to leg time and to the `wall_adjacent` failure; it is **not** the cause
of the wall/enclosure stall.

**Two more suspects eliminated.** Post-change the robot stalls with the
nearest laser return at **0.545–0.567 m** (0.388 m in the baseline) and
the collision monitor at **`DO_NOTHING` 84–88 %** of the stall — more
free space, less gating, still zero velocity. With C2-NAV.0's
`/cmd_vel_nav` control, three of four candidate causes of
`enclosure_entry` are now ruled out by measurement. BaseObstacle
domination (still 49.3 % of the chosen trajectory's score) is what is
left, and is still only a hypothesis.

**Watched, not just logged** (brief §12). RViz mid-stall:
`Navigation: active`, `Localization: active`, `Distance remaining:
1.27 m`, `Time taken: 41 s`, `Recoveries: 4` — a completely healthy
stack, motionless. The fixed `wall_adjacent` leg for contrast:
`0.27 m`, `3 s`, `0 recoveries`. On a fresh sim, baseline `open_space`
spins **1.49 rad in its last 5 s at up to 1.037 rad/s** against
`max_vel_theta` 1.0; post-change 0.42 rad, peak 0.231 rad/s.

**Two costs, measured, neither closed.**

1. **Arrival accuracy**: ground-truth error 0.118 → **0.263 m** median;
   7 of 21 legs reached within 0.25 m by ground truth against 18 of 21.
   The terminal phase was not only spinning — `GoalDist` was closing the
   last ~0.15 m while it ran.
2. **Final heading now arbitrary**: median 0.449 → **1.583 rad**.
   `DESIGN_DECISIONS.md` records that 0.25 rad at the pre-ramp pose is
   0.64 m of lateral over a 2.5 m climb. **No ramp or mission run was
   attempted.** The change is measured but **must not be merged** until
   it is.

**One metric retired.** `xtrack_med_m` (0.571 → 1.227 m) is an artefact:
it measures distance to a stub final plan over time-uniform samples, and
the baseline parks **32.8 %** of its samples at the goal against
**0.0 %** now. Not a tracking regression. Recorded so nobody quotes it.

**Tests.** `gazebo_models` **41/41 passed**, from inside the package dir
on a clean graph with `--ignore=test/test_integration` — the CLAUDE.md
baseline for that package, unchanged. No other package was run: the
change is a Nav2 YAML value that no test reads (the three `coco_mission`
files matching `nav2_params` reference `amcl` and `behavior_server` in
comments only). `docs/data/c2nav0_analysis.py` gained a one-line
formatter guard — a whole scenario now has *no* collision-monitor state,
which `f'{None:>7}'` cannot format — and its output on the committed
baselines was diffed byte-for-byte before and after to prove the numbers
did not move.

**Next command to run** — C2-NAV.2, and the single most informative
thing left, because it is the only surviving hypothesis for the stall:

```bash
# T1 fresh simulator, headless. Never --fast.
ros2 launch gazebo_models full_world_robo.launch.py gui:=false
# T2 with BaseObstacle.scale 8.0 -> 2.0 as the ONLY further change
ros2 launch gazebo_models nav.launch.py arbiter:=false \
    params_file:=<repo>/gazebo_models/config/nav2_params.yaml
# T3 the enclosure leg alone first — 3 repeats, ~4 minutes, and it either
#    moves off 0/3 or it does not
ros2 run gazebo_models nav_bench.py --tag navA_baseobs --repeats 3 \
    --timeout 75 --only enclosure_entry
```

---

## 2026-09-02 — C2-NAV.2: the BaseObstacle scale, tested alone and rejected

**One variable**, and the last surviving hypothesis for the
`enclosure_entry` stall. `FollowPath.BaseObstacle.scale`, **8.0 → 2.0**.
No source file, launch file or test was touched. Full record:
`docs/RESULTS.md`, "C2-NAV.2 navigation BaseObstacle scale"; ranking of
what is left: `docs/ROADMAP.md`, "C2-NAV.3 candidates".

**The baseline is C2-NAV.0, not C2-NAV.1.** The worktree carried
C2-NAV.1's `PositionGoalChecker`, which would have made this a two-variable
experiment. `nav2_params.yaml` was restored from `8f05c45` first and then
edited in one place. Verified two ways: a comment-stripped diff against
`8f05c45` reduces to `-BaseObstacle.scale: 8.0 / +BaseObstacle.scale: 2.0`
and nothing else; and the **live** `controller_server` reports
`BaseObstacle.scale 2.0`, `sum_scores False`, goal checker
`nav2_controller::SimpleGoalChecker` with `xy`/`yaw` tolerance 0.25/0.25,
`PathAlign`/`PathDist` 32.0, `GoalAlign`/`GoalDist` 24.0, `RotateToGoal`
32.0, `vx_samples` 20, `vtheta_samples` 40, `sim_time` 1.5,
`controller_frequency` 10.0, `robot_radius` 0.20, `inflation_radius` 0.50,
`PolygonSlow.slowdown_ratio` 0.3.

**The params file must be passed explicitly.**
`install/gazebo_models/share/gazebo_models/config/nav2_params.yaml` is a
symlink to the trunk checkout, which is at `main` and still holds
`BaseObstacle.scale: 8.0`. Without `params_file:=<worktree>/...` this
experiment silently re-runs the baseline and reports it as the result.

**Result: REJECTED.** 3 repeats, topology A, 75 s timeout, fresh headless
sim, robot verified at the spawn, RTF 0.991.

| | C2-NAV.0 | C2-NAV.2 |
|---|---:|---:|
| success | 0/3 | **0/3** |
| longest stall (median) | 47.84 s | **64.21 s** |
| distance remaining at stall | 1.150 m | **1.322 m** |
| DWB best vx == 0 | 0.680 | **0.921** |
| `BaseObstacle` % of chosen score | 71.8 % | **0.0 %** |

**A rejection, not a null result.** The intervention did what it was meant
to do to the quantity it targeted — `BaseObstacle` went from 71.8 % of the
chosen trajectory's score to 0.0 % — and the stall got *longer*, the robot
got *less far*, and it selected zero *more* often.

**Measured deeper than success/failure**, with a probe on DWB's
`/evaluation` (`docs/data/c2n2_evalprobe.py`). Two stall poses, two
different failure reasons:

* **Pose A, 1.313 m out**: the robot sits in a **cost-0** cell with a
  **1.90 m** zero-cost band. Chosen `vx = 0.0` in 12 of 12 cycles. **8 of
  10 sampled forward speeds are scored to completion with `BaseObstacle`
  = 0.00 and still lose**, median gap 7.90, carried entirely by
  `PathAlign` +34.40, `GoalAlign` +29.40, `GoalDist` +18.00, `PathDist`
  +14.40 over 12 cycles. The total rises monotonically with commanded
  speed, 32.60 → 43.00, with `BaseObstacle` 0.00 throughout.
  **`BaseObstacle` is not a necessary condition for the stall.**
* **Pose B, 1.271 m out**, 0.165 m deeper: all 10 forward speeds aborted
  on `BaseObstacle` alone, 120.0–262.0 against a winning total of 34.0 —
  cell costs of **60–131** at scale 2.0.

**And the arithmetic that closes the knob.** With `sum_scores` false and
the MapGrid critics' effective weight `resolution * 0.5 * scale` = 0.60
per cell, the winning zero-velocity total is ≈ 33. Forward motion is
disqualified once `cost × scale` exceeds that: cost ≈ 17 at scale 2.0,
≈ 4 at scale 8.0. The pinch presents 60–131, so it would take
`scale < 0.26–0.57` — **below the 0.02 C2-NAV.0 forbade returning to**.
The scale cannot reach the behaviour without recreating the defect it was
raised to fix.

**The falsifier was already committed.** C2-NAV.0 repeat 2 stalled 48.21 s
with `BaseObstacle` at 0.0 % of the chosen score. The 93.4 % was one
instant in one repeat, never the population.

**The robot is rotating, not frozen**: 5.550 rad over the 64.21 s stall,
commanded `w` reaching `max_vel_theta` 1.0 rad/s against an actual median
of 0.027 rad/s, `/cmd_vel_nav` linear zero on 96.7 % of samples, collision
monitor `SLOWDOWN` 75.3 % / `DO_NOTHING` 16.3 %. The progress checker
aborts `follow_path` every 10 s because a rotating robot never translates
0.1 m — 5, 6 and 6 aborts across the three repeats.

**Two honesty notes.**

1. **A methodological difference from the baseline.** The baseline ran
   `enclosure_entry` as leg 6 of a 7-leg tour, approached from
   `corridor_gate` at ≈ (−2.58, −0.03). This ran `--only enclosure_entry`,
   so **only repeat 0 is a fresh approach**; repeats 1 and 2 start where
   repeat 0 stalled and are *escape* tests. Their `path_len_m` of 0.195 m
   and 0.494 m must not be compared with the baseline's 3.3–4.0 m. The
   comparable number is repeat 0's 1.320 m goal error against the
   baseline's 1.159–1.449 m — **inside** the baseline range. Repeats 1
   and 2 do establish separately that once stalled the robot does not
   recover: zero selected on 92.1 % and 100 % of cycles.
2. **The first pass of the critic decomposition was wrong and was
   corrected.** `short_circuit_trajectory_evaluation` is true, so an
   aborted trajectory carries only the critics scored before the abort and
   its `total` is a partial sum. Differencing it against a complete score
   as if the missing critics were 0.0 manufactured a spurious −195 for
   `GoalAlign`. `docs/data/c2n2_reanalyse.py` separates complete from
   aborted scores; every number above comes from the corrected pass.

**Tests.** `gazebo_models` **41/41 passed**, from inside the package dir on
a clean graph with `--ignore=test/test_integration` — the CLAUDE.md
baseline for that package, unchanged. No other package was run: the change
is a Nav2 YAML value no test reads.

**One infrastructure trap, paid for.** `ros_clean.sh` brackets every
`pkill` pattern except **`'nav2_'`**. A helper named `c2nav2_up.sh`
contains that substring, so the sweep it invoked killed the shell running
it; the run died at exit 144 before the simulator started, which reads
exactly like a bringup failure. Every C2-NAV.2 artefact is named `c2n2_*`
instead. **Bracketing it to `'nav[2]_'` is a one-character fix and was
deliberately NOT made in this commit**, which must carry one variable and
its documentation and nothing else. It is recorded in `docs/ROADMAP.md`
for whoever takes it.

**State of the worktree.** `BaseObstacle.scale: 2.0` is left in
`nav2_params.yaml` as the record of the experiment, commented as
EXPERIMENTAL and not approved. **It is worse than the baseline on every
movement metric measured and must not be merged.** Neither may C2-NAV.1,
which is still blocked on its own ramp verification.

**Next command to run** — C2-NAV.3, and it is a **diagnosis, not an
intervention**. Four candidate causes are now dead by measurement and the
open question is why the goal/path MapGrid prefers standing still in free
space. Measured inputs to it: the robot is 39.7° off the goal bearing but
only **11.8° off its own plan's heading** over the plan's first 0.30 m,
the plan is present and 25 poses long, and forward motion still increases
`GoalDist` 26 → 29 cells and `PathAlign` 0.00 → 4.00.

```bash
# First: revert the experiment, so C2-NAV.3 starts from C2-NAV.0 again.
cd <worktree> && git checkout 8f05c45 -- gazebo_models/config/nav2_params.yaml

# T1 fresh simulator, headless. Never --fast.
ros2 launch gazebo_models full_world_robo.launch.py gui:=false
# T2 baseline params, explicitly
ros2 launch gazebo_models nav.launch.py arbiter:=false \
    params_file:=<worktree>/gazebo_models/config/nav2_params.yaml
# T3 instrument the MapGrid itself: dump GoalDist/PathDist cell values
#    along the global plan and across the trajectory endpoints at the
#    stall, and establish whether the propagation is blocked at the
#    pinch, truncated by the 3 x 3 m local costmap window, or seeded
#    from a plan whose in-window portion ends short.
#    Start from docs/data/c2n2_evalprobe.py, which already captures
#    /evaluation, /plan and the pose at the stall.
```

## 2026-09-02 — C2-NAV.3: the MapGrid critics are not the cause, and what is

**A diagnosis, not a change.** No navigation parameter moved. Full record:
`docs/RESULTS.md`, "C2-NAV.3 navigation MapGrid diagnosis"; the next
experiment and its acceptance test: `docs/ROADMAP.md`, "C2-NAV.4".

**The question.** C2-NAV.2 left the four MapGrid critics — `GoalDist`,
`PathDist`, `PathAlign`, `GoalAlign` — as the only remaining suspects for
the `enclosure_entry` stall. Why do they prefer zero velocity 1.3 m short
of the goal?

**The answer: they do not.** In a controlled sweep at the captured stall
pose with `wz` held at exactly 0.0, `GoalDist` falls **29 → 24 cells** and
`GoalAlign` **30 → 24** as `vx` rises 0 → 0.30, while `PathAlign` and
`PathDist` never leave 0–1. All four reward forward motion or ignore it.
**`BaseObstacle` rises 0 → 66 within one cell of travel**, and 66 × 8.0 =
528 against a winning total of 36.20, so every forward trajectory is
short-circuited at critic **3 of 7** and `GoalDist` is never computed.

**Verdict: EXPLAINED.** The robot stands in the last cost-0 cell before an
inflation field that its **entire** global plan runs through — all 28 plan
poses at cost 60–164, **none at cost 0**, measured twice. Following the
plan costs ≥ 60 × 8.0 = 480 in `BaseObstacle`. The MapGrid critics can pay
at most **25.20** (bounded by `aggregation_type: last` and a 0.45 m
horizon = 9 cells), which `BaseObstacle` at scale 8.0 spends at a cell
cost of **3.15**. The gate is the cost field, not the weight on it.

**The baseline is C2-NAV.0.** `docs/data/c2nav3_baseline_params.yaml` is
`nav2_params.yaml` at `8f05c45` verbatim, sha256 `dbcee9ca…`, passed as
`params_file:=`. The live `controller_server` reports
`BaseObstacle.scale 8.0`, `SimpleGoalChecker`, `xy`/`yaw` 0.25/0.25,
`aggregation_type last` on both Dist critics, `forward_point_distance`
0.1, `min_vel_x` 0.0, `publish_cost_grid_pc` **False**. Dump in
`.navbench/logs/c2n3_params.txt`.

**Two fresh approaches, one leg each** — not `--repeats 3`, because
repeats 1 and 2 start from the stalled state. The stall reproduces at
(−2.1946, 2.5685) and (−2.2054, 2.5777), **1.3 cm apart**, 1.312 m and
1.299 m from the goal, at headings 50° apart.

**The rebuild is what makes this evidence.** `c2nav3_mapgrid.py`
reimplements the seeding, the L1 propagation and the scoring from
`dwb_critics` 1.3.11 source (verified byte-identical to the `jazzy` tip)
and reproduces **23/25** and **20/21** of DWB's published raw scores —
**all four MapGrid critics matched in both runs**. `c2nav3_probe.py`
regenerates trajectories and lands on DWB's own poses to **9–13 µm**.

**Five source facts worth keeping.**

1. `GoalDist` is **not** the distance to the goal. It seeds one cell — the
   last plan pose still inside the 3 × 3 m window — and measures the
   **Manhattan distance in cells** to it.
2. The propagation does **not** avoid obstacles.
   `MapGridQueue::validCellToQueue` returns `true` unconditionally; the
   header comment claiming otherwise is wrong about its own code.
3. `aggregation_type` is `last`: only the trajectory's **final** pose
   scores. With `sim_time` 1.5 s and `max_vel_x` 0.3 that is 9 cells, and
   it bounds every MapGrid critic.
4. `MapGridCritic::getScale()` is `resolution * 0.5 * scale`;
   `BaseObstacle` does not override it. The two families are on
   **incommensurable scales by construction** — cell counts against a
   0–252 cost.
5. `min_vel_x` is 0.0, so **reverse is never sampled**. DWB cannot back
   out because it never considers it.

**Reconciled with C2-NAV.2, which probed the same poses.** Its Pose A
(1.313 m) and this session's run A (1.312 m) are the same stall, and the
numbers agree. Its "8 of 10 forward speeds complete with `BaseObstacle`
0.00" is a **minimum over `wz`** at each `vx`, so the survivors are the
trajectories that turn hardest away from the wall. Hold `wz` at 0 and
every forward sample above 0.0158 m/s aborts on `BaseObstacle`. C2-NAV.2's
data and its arithmetic on the knob were right; the sentence
"`BaseObstacle` is not a necessary condition" over-read them.

**One instrument bug, found and fixed mid-session.** The first capture put
all six subscriptions in the node's default callback group. `/evaluation`
is 819 trajectories × up to 60 poses at 10 Hz, and under a
`MultiThreadedExecutor` a shared `MutuallyExclusive` group serialises
everything: `/model/coco/odometry` and `/cmd_vel_nav` went **51 s without
a single callback** while the robot actually drove 2.6 m, so the recorded
pose froze at the spawn. A starved subscription and a silent topic look
identical from the inside. Fixed with one callback group per subscription;
run A's timeline was discarded and run B's is the clean one. Run A's stall
*snapshot* is kept — its pose was cross-checked against an independent
read of the same topic from a separate process and agreed to 0.017 m.

**One infrastructure fix, committed separately** (`323471f`): the last
three unbracketed `ros_clean.sh` patterns — `nav2_`, `ros2_control_node`,
`rosbridge` — are now bracketed, restoring the invariant the file's own
header states. It is not part of the navigation result.

**Its commit message overstates it, and this corrects that.** Bracketing
stops a pattern matching its **own text**; `'nav[2]_'` and `'nav2_'`
match exactly the same strings, so `c2nav2_up.sh` — whose name genuinely
contains `nav2_` — is **still** killed by the sweep it invokes, as is any
`ros2 launch ... params_file:=<…>/nav2_params.yaml`. Measured both ways
with `.navbench/c2n3_bracketcheck.sh`. **The naming convention is still
load-bearing**: helpers are `c2n2_*` / `c2n3_*`, and C2-NAV.3's parameter
copy is `c2nav3_baseline_params.yaml`, not `*nav2_params.yaml`.

**Next command.**

```bash
cd ~/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis
# C2-NAV.4 acceptance test, BEFORE any drive: change one inflation value
# in a copy of c2nav3_baseline_params.yaml, bring the stack up on it, and
# ask whether a cheap corridor now exists at all.
bash .navbench/c2n3_capture.sh .navbench/results/c2n4
cd docs/data && python3 c2nav3_probe.py ../../.navbench/results/c2n4_stall.json 0
# read the last line: "cost along the transformed plan: min N".
# If N is not below about 3, the robot will not move, and no drive is
# needed to know it.
```

## Checkpoint — C2-NAV.4, the inflation cost field (2026-09-02)

**One variable: `local_costmap.inflation_layer.cost_scaling_factor`,
5.0 → 22 / 30 / 65.** Baseline is C2-NAV.0 exactly, verified off the live
nodes on every run. Full record: `docs/RESULTS.md`, "C2-NAV.4 navigation
inflation cost field".

### What was built

- `docs/data/c2nav4_costfield.py` — the pre-drive falsifier. It identifies
  the inflation lookup table against a captured costmap, remaps every cell
  to a different `cost_scaling_factor` **exactly** (ring by ring over
  integer cell offsets, not by inverting the exponential), rebuilds the
  four MapGrid critics through C2-NAV.3's `c2nav3_mapgrid.py`, and replays
  **all 819** `(vx, wz)` samples DWB evaluated, every one scored to
  completion so no short-circuit partial is read as a decomposition.
  `--base-csf` names the factor a capture was taken under; getting it
  wrong aborts the run rather than silently mis-scaling.
- `docs/data/c2nav4_report.py` — renders the static prediction, the
  `nav_bench` legs and the capture timelines. Computes nothing new.
- `docs/data/c2nav4_csf{22,30,65}_params.yaml` — one-line derivatives of
  `c2nav3_baseline_params.yaml` (the C2-NAV.0 baseline verbatim). Line 206
  only, the **local** costmap.
- `docs/data/c2nav4_{base,csf22}_stall.json`,
  `c2nav4_{base,csf22,csf30,csf65}_timeline.csv`, `c2nav4_bench.json` —
  the captures and all eight benchmark legs.
- `.navbench/c2n4_*.sh` — scratch helpers. `c2n4_verify.sh` reads every
  parameter back off the live nodes; `c2n4_bracketcheck.sh` proves no
  C2-NAV.4 command line can be matched by `ros_clean.sh`'s pattern list,
  and carries a positive control so "nothing matched" is a measurement.

### What was measured

- **The enclosure-entry leg SUCCEEDS for the first time**, at CSF 65:
  57.89 s / 0.056 m goal error on a 75 s budget, 78.33 s / 0.053 m on a
  150 s budget. Traverses 3/3 approaches, 2/2 benchmark legs SUCCEEDED.
  The baseline is 0/3 at both budgets, and C2-NAV.0's committed record is
  0/3.
- **The inflation layer's inscribed radius is 0.205879 m**, the apothem of
  the `robot_radius` 0.20 16-gon after `footprint_padding` **0.01**. Only
  that value reproduces all 34 distinct inflated costs in the captured
  grid; `robot_radius` misses 29. Confirmed on the live node.
- **The remap is one-to-one** at every factor tested, and the replay
  reproduces DWB's real command at CSF 5.0 in **all three** captured
  stalls, with regenerated trajectories on DWB's own poses to 9–13 µm.
- **The decision flips at CSF ≈ 21**, not at C2-NAV.3's "minimum plan cost
  below 3" screen — which passes CSF 15 and 20 (both still stall) and
  passes the **unmodified baseline**. The criterion is cost **exactly 0**
  at the trajectory's final pose: the realised MapGrid margin is 2.0–6.0
  points and `BaseObstacle.scale` 8.0 spends that on one unit of raw cost.
- **CSF 30 traverses 2 of 3 and never passes the goal checker.** At 150 s
  it reaches the goal position to 0.010 m and still TIMEOUTs, on the goal
  **yaw** — C2-NAV.1's mechanism, not this one.
- **CSF 22 is rejected**, 0 of 3 traverses. **Lowering** the factor is
  rejected statically: CSF 2.5 raises the cheapest plan cell 60 → 123.

### What remains unverified

`docs/RESULTS.md`, "C2-NAV.4 … NOT PROVEN". The short list: n = 3 is a
contrast, not a rate; **only `enclosure_entry` was run**, so nothing is
known about the other six tour legs under a near-binary cost field; and
minimum clearance at CSF 65 is 0.227–0.279 m against an inscribed radius
of 0.2059 m, a worst-case margin of 2.1 cm that was not assessed.

### Exact next command

```bash
# C2-NAV.5, step 1: turn the 3/3 into a rate, and the 0/3 with it.
# One fresh simulator per approach -- a repeat inside one simulator is an
# escape probe, not a trial.
cd ~/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis
bash .navbench/c2n4_bench.sh docs/data/c2nav4_csf65_params.yaml csf65_r1
bash .navbench/c2n4_bench.sh docs/data/c2nav3_baseline_params.yaml base_r1
# ... repeat with fresh tags, then:
cd docs/data && python3 c2nav4_report.py live ../../.navbench/results/*_r*.json
```

## 2026-09-02 — C2-NAV.5: CSF 65 validated on fresh simulators, and the fix opened a door it cannot close

**Branch** `worktree-c2nav0-diagnosis`. A **validation** pass, not a
tuning session. No sweep, no new candidate value, no parameter searched.
Exactly two configurations, differing in one line, on genuinely fresh
simulators throughout.

### What was built

- `docs/data/c2nav5_costprobe.py` — a cost-field probe that triggers on
  **geometry** rather than on a stall. C2-NAV.3's capture snapshots the
  field when it detects ten seconds of zero command; run against a
  configuration that works, it returns "no snapshots", which in the
  artifact is indistinguishable from an instrument that never
  subscribed. This one snapshots at a fixed ladder of distances to the
  goal and reuses C2-NAV.3's `Capture`/`snapshot`/`Costmap` and
  C2-NAV.4's `plan_costs`/`describe` **by import**, so its numbers are
  comparable to theirs by construction. Same positive control: it refuses
  to report until it has seen `/evaluation`, the costmap and the
  transformed plan.
- `docs/data/c2nav5_report.py` — the tables, plus a `collect` mode that
  folds the scratch directory's per-run JSONs into one committed
  `c2nav5_bench.json`. `enclosure` and `tour` read **either** source and
  were checked with `diff` to produce byte-identical output from both.
- `.navbench/c2n5_*` — the run harness (not committed, as with C2-NAV.0
  through C2-NAV.4), including `c2n5_bracketcheck.sh`, which passed
  against the eleven command lines this experiment put on the wire before
  any simulator started.

### What was measured

**Stage 1 — `enclosure_entry`, 10 fresh simulators, 5 per condition,
interleaved, 150 s each.**

| | traversed | SUCCEEDED | median duration | median final error |
|---|---|---|---|---|
| baseline CSF 5.0 | **0/5** | **0/5** | — | 1.298 m |
| candidate CSF 65 | **5/5** | **5/5** | 93.77 s | 0.064 m |

The baseline failure is deterministic: five stalls inside a
4.6 × 12.8 cm box, 1.240–1.324 m out, median commanded `vx` exactly 0.0,
crawl 90.5–90.8 s in four of five. Two of the five occur with the
collision monitor at `DO_NOTHING`, so gating is not the cause. CSF 65's
transit is far steadier than its total — 23.27–26.98 s to the tolerance
across all five, with 56.7–77.0 % of each leg spent settling the goal
yaw. RTF 0.972–0.987 throughout.

**Stage 2 — six fresh tours, 42 legs, topology A, 75 s per leg.**
Baseline **17/21 SUCCEEDED**, CSF 65 **18/21**. `wall_adjacent`
**2/3 → 3/3**; `wall_parallel` 3/3 in both but **56.10 → 18.97 s**
median. `open_space` unchanged. Clearance costs 3–8 cm on the open and
wall legs.

**Stage 3 — cost field, three fresh probes.** Baseline at 1.192 m:
**0 of 24** transformed-plan poses at cost 0 (min 59, median 164, max
230), `BaseObstacle` **456.00**, closest approach 1.1794 m — reproducing
C2-NAV.3's "60–164, none at cost 0" two sessions later. Its 1.3 m rung is
the knife-edge in one line: forward total **36.60** equals zero total
**36.60**, and DWB picks zero. CSF 65 at the same rungs: every pose cost
**0**, `BaseObstacle` **0.00**, forward beating zero by 1.8–6.8 points,
DWB never selecting zero.

**The regression, and it is not this knob's.** `enclosure_exit` is
**1/3** at CSF 65 against 3/3 at the baseline — but the baseline never
attempted the same leg, because its `enclosure_entry` always failed and
left the robot *outside* the pocket. On the two CSF 65 failures DWB
commands a median **0.2684 m/s** and selects zero on **0.000** of cycles,
while the **collision monitor holds STOP for 91.4 % and 94.1 %** of the
leg and the wheels see **0.0142 m/s**. The robot parks inside its own
`PolygonStop` circle and is gated from leaving. The escaping run had a
*closer* scan return (0.153 m vs 0.218 m) and never entered STOP, so the
trigger is `min_points: 4` inside the 0.25 m circle, not proximity.

Incidentally this is the first direct measurement in the series of the
collision monitor's gating **reaching the wheels** — in topology A, where
there is no `/cmd_vel_nav` loop.

**Verdict: PARTIALLY VALIDATED.** Six of seven acceptance criteria pass.
Criterion 5 (no unsafe proximity or instability) fails on `enclosure_exit`.

### What remains unverified

`docs/RESULTS.md`, "C2-NAV.5 … NOT PROVEN". The short list: N = 5 and
N = 3 are engineering-validation counts, not statistics; `enclosure_exit`
at CSF 65 has N = 3 and one success; the `min_points` explanation was
never tested by varying `min_points`; the worst driven-path clearance at
CSF 65 is **0.216 m** against a 0.2051 m circumscribed radius, a 1.1 cm
margin, though no run in either condition drove below it; and — the one
that matters most — **every run in C2-NAV.0 through C2-NAV.5 is topology
A, while `mission.launch.py` runs topology B. CSF 65 is unvalidated in
the configuration the robot ships in.**

Nothing is approved for merge. `gazebo_models/config/nav2_params.yaml` is
untouched and still carries C2-NAV.2's rejected `BaseObstacle.scale: 2.0`.

### Exact next command

```bash
# C2-NAV.6, step 1: is PolygonStop's min_points what traps the robot?
# One variable, everything else at the C2-NAV.5 candidate.
#
# ADDRESS THE LINE, NOT THE PATTERN. There are THREE `min_points: 4`
# lines in this file -- PolygonStop (425), PolygonSlow (439) and
# PolygonLimit (448) -- plus FootprintApproach's 6 at 460. A bare
# s/min_points: 4/.../ moves three variables and the experiment is no
# longer single-variable. 425 is PolygonStop's; check it before editing.
cd ~/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis
sed -n '416,428p' docs/data/c2nav4_csf65_params.yaml     # confirm 425
sed '425s/min_points: 4/min_points: 8/' \
    docs/data/c2nav4_csf65_params.yaml > docs/data/c2nav6_minpts8_params.yaml
diff docs/data/c2nav4_csf65_params.yaml docs/data/c2nav6_minpts8_params.yaml
# must be exactly one hunk, at line 425
# then the whole tour, fresh simulator, so enclosure_exit starts from
# inside the pocket exactly as it did here:
bash .navbench/c2n5_run.sh \
    "$PWD/docs/data/c2nav6_minpts8_params.yaml" c2n6_tour_mp8_r1 ALL 75
```

---

## 2026-09-02 — C2-NAV.6: the stop is real geometry, and the threshold is the wrong knob

### What was built

`docs/data/c2nav6_stopprobe.py` — a subscribe-only probe that rides
alongside `nav_bench.py` and answers the question C2-NAV.5 could not:
**how many laser returns are actually inside `PolygonStop` when the
monitor holds STOP.** It re-implements nothing by description. It applies
`nav2_collision_monitor` 1.3.11's own three decisions verbatim —
`scan.cpp`'s `r >= range_min && r <= range_max` (no `isfinite` test, and
a return below 0.15 m is *dropped*), `circle.cpp`'s **strict**
`x² + y² < radius²` about the origin of `base_frame_id` (**not** the
lidar), and the node's `getPointsInside(...) >= getMinPoints()`, so
`min_points: 4` means "four or more".

`docs/data/c2nav6_stopgeom.py` — dumps the *individual* returns from a
robot parked in the pose under diagnosis. A count cannot separate "small
obstacle" from "large obstacle, sliver inside the circle"; this can.

`docs/data/c2nav6_report.py` — `collect` / `legs` / `stop`, reading either
the `.navbench` scratch directory or the committed `c2nav6_bench.json`,
verified with `diff` to give byte-identical tables from both.

`docs/data/c2nav6_minpts7_params.yaml` — the candidate, a **line-addressed**
one-line derivative of `c2nav4_csf65_params.yaml` at line 425.

`ros_clean.sh` gained `'c2nav6_stopprob[e]'`, for the reason
`c2m5_locrec` and `c2m51_hrec` are there.

### What was measured

**The baseline failure reproduced on a fresh simulator.**
`enclosure_entry` SUCCEEDED (55.85 s, 4.229 m); `enclosure_exit` TIMEOUT
after 150.55 s having driven **0.263 m** with a median commanded
**0.2842 m/s** — against C2-NAV.5's 0.274 m / 0.220 m and 0.2684 m/s.

**The trigger is exactly 6 points**, on **1470 of 1470** STOP frames,
min = median = mean = max, zero variance. The six are **contiguous** beams
314–319 spanning **10.2 mm** of a **convex corner** that penetrates the
0.25 m circle by **5.5 mm**; at 1.87 mm of beam spacing that is 6 beams.
C2-NAV.5 was right that the mechanism is a sparse count.

**`min_points: 7` did what was predicted and did not fix anything.** The
6-point stop cleared, the robot moved — then advanced **4.4 cm** and STOP
re-armed at exactly **8** points on 1418 of 1418 frames, 9.3 mm inside
the circle over a 16.3 mm sliver. Exit still TIMEOUT, still 3.14 m from
the goal, driven 0.263 m → **0.307 m**.

**The count tracks penetration depth**: 5.5 mm → 6 beams, 9.3 mm → 8
beams, both matching sliver ÷ spacing to under one beam. A `min_points`
high enough to clear the escape path is a **radius reduction in
disguise**.

**The monitor is authoritative in topology A, to the frame.** Baseline
exit: **1470** frames with a wheel command of exactly 0.0 against
**1470** frames in STOP — the same integer, out of 1537.

**`STOP` zeroes all three axes**, so the −0.15 m/s reverse recorded on
both exit legs reached the wheels as 0.0. The escape manoeuvre is gated
by the rule that created the trap.

**Safety held and the change is still not free.** Nearest returns
0.2445 m and 0.2407 m, both well outside the **0.2051 m** circumscribed
radius (39.4 mm and 35.6 mm of margin); neither run approached below it.
But six returns is about **1 cm** of visible surface, and `min_points: 7`
stops stopping for anything smaller. **Not recommended for adoption.**

`gazebo_models` **41/41** on a clean ROS graph.

### Two corrections to the plan this session inherited

**The candidate value is 7, not the 8 the C2-NAV.5 log proposed** — and
the difference does not matter, which is itself the result. 8 was a guess
made before the count was known; the count is 6, so 7 is the smallest
value that suppresses the observed trigger, which is what "smallest
clearly diagnostic increase" means. And because the *second* stall sits
at exactly 8 points, `8 >= 8` would have fired there too: `min_points: 8`
would have produced the same TIMEOUT.

**The probe's positive control had to be split in two, and the first
attempt deadlocked.** `/collision_monitor_state` is published from
`cmdVelInCallback`, so with no goal running there is no
`/cmd_vel_smoothed`, no callback, and no state at all — measured, 581
scans and 2905 ground-truth messages against **0** monitor states over
60 s. Gating startup on it deadlocks the run that would have produced it.
`/scan`, ground truth and TF are now asserted **before** recording; the
monitor and the wheels are asserted **after**, over the recording, and a
run that never saw them exits non-zero and is discarded rather than read
as "the monitor never fired". Both runs recorded `control ok=True`.

### What remains unverified

`min_points` beyond 4 and 7 (the suppression curves say 10 clears both
*measured* poses and say nothing about the poses past them); whether the
exit is achievable from this pose **at all**; reproducibility of either
result (N=1 per condition, by design — this is a diagnosis, not a rate);
anything about **topology B**, which is still where the robot ships; and
that `PolygonStop.radius` would fix it, which is now merely the indicated
next knob.

Nothing is approved for merge. `gazebo_models/config/nav2_params.yaml` is
untouched and still carries C2-NAV.2's rejected `BaseObstacle.scale: 2.0`.

### Exact next command

```bash
# C2-NAV.7: is the enclosure_entry goal a pose the robot can be left in
# at all? C2-NAV.5 ranked this third; C2-NAV.6 promotes it to first,
# because the trap is NOT a sensing artefact a threshold can filter out.
# It is the robot parked with real geometry 3.5-3.9 cm from its hull
# inside a stop zone that extends 4.5 cm past the chassis.
#
# The arithmetic to satisfy: the nearest geometry must end up further
# than PolygonStop.radius (0.25 m) from the base_footprint origin, so the
# goal needs roughly 5-10 cm more stand-off than its current 0.35 m.
# This is a BENCHMARK edit, not a tuning knob: it changes nav_bench.py's
# TOUR, so the new goal is a new entry and C2-NAV.0's committed baselines
# for the old one still stand unaltered.
cd ~/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis
sed -n '126,142p' gazebo_models/scripts/nav_bench.py    # the TOUR
python3 -c "print('current entry goal world (-3.45, 2.95); stall put the "
"nearest return 0.2445 m from base origin, 0.0055 m inside the circle')"

# Then, unchanged in every other respect, at the C2-NAV.4/.5 candidate:
bash .navbench/c2n6_run.sh \
    "$PWD/docs/data/c2nav4_csf65_params.yaml" c2n7_base_r1 \
    enclosure_entry,enclosure_exit 150
# The probe rides along automatically and writes <tag>_stop.{csv,json};
# if the exit succeeds, the trap was the goal and NEITHER PolygonStop
# knob should move. Only if it still fails does radius become the
# experiment, with 0.2051 m stated as the hard floor.
```

---

## 2026-09-03 — C2-NAV.7: the goal was outside the stop circle and still on the wrong side of the only way out

### What was built

`docs/data/c2nav7_geom.py` — the pre-edit geometry, and the arbiter for a
disagreement three clearance numbers were having. It (1) **identifies**
the obstacle instead of assuming it, by transforming C2-NAV.6's six
inside-circle laser returns into world coordinates using the ground-truth
pose recorded in the same session; (2) computes the true stand-off at the
goal; (3) derives the **PolygonStop-free corridor** through the NW pinch;
and, in `track` mode, (4) computes exact minimum clearance along a
recorded run from world-file collision faces — 360° **and** unquantised,
unlike the map (5 cm grid) or the laser (240° FOV, blind behind).

`nav_bench.py` gained a **default-off `--goal NAME:X,Y` override**.
Moving a scenario goal is a benchmark change, not a tuning knob, so
`TOUR` stays byte-identical to `8f05c45`, every earlier experiment
reproduces without the flag, and the goal that ran is written into each
leg record as `goal_world`. An offline test asserts no-override
identity, single-leg isolation, and rejection of malformed specs before
anything launches.

### What was measured

**The brief's hypothesis was wrong in form.** The `enclosure_entry` goal
is **0.3606 m** from the nearest geometry — **111 mm OUTSIDE** the 0.25 m
stop circle, and 155 mm outside the 0.2051 m circumscribed radius. It was
never "too close". The "≈0.35 m" that had been carried forward is neither
accurate nor measured to a wall.

**The obstacle is `box_obstacle_1`'s NW corner (−3.25, 2.65).** All six
of C2-NAV.6's inside-circle returns land on it within **0.1 mm**, and the
stall pose is 0.2437 m from it against a measured 0.2445 m — 0.8 mm.

**The real constraint is a 0.150 m corridor.** The exit must cross the
pinch between `wall_west` (x = −3.900) and `box_obstacle_1` (x = −3.250).
Clearing `PolygonStop.radius` on both sides requires **x ∈ [−3.650,
−3.500]**. The goal at −3.450 is **50 mm east of it**; so is the
C2-NAV.6 stall pose. That, not the absolute stand-off, is what the goal
position controls.

**Candidate −3.5750, the corridor centre, ±75 mm.** Chosen over the
brief's 5–10 cm because 5 cm lands on the corridor edge with **zero**
margin and 10 cm leaves 50 mm, against a measured entry goal error of
80–96 mm.

**The exit works, 3 of 3.** Three fresh simulators, **5325 frames, 0 STOP
frames, 0 returns inside the circle on every leg** — against the
baseline's constant 6 for 1470 consecutive frames. `enclosure_exit`
SUCCEEDED 3/3, driving **4.228 / 3.461 / 3.495 m** in **41.42 / 33.19 /
33.27 s**, against TIMEOUT after **0.263 m** ending 3.139 m short. Median
`v_nav` 0.2842 → wheel **0.0853** (the 0.3 `slowdown_ratio`) where the
baseline's arrived as **0.0**. **No navigation parameter moved** — the
live read-back is byte-identical to C2-NAV.6's baseline.

**Two costs.** `enclosure_entry` SUCCEEDED **1 of 3** — though it
traversed 3/3 at 0.153 / 0.116 / 0.069 m, so the TIMEOUTs are C2-NAV.1's
terminal yaw — and ran **2–2.7× slower** (116.56 / 150.68 / 150.01 s vs
55.85 s), with the SLOWDOWN share rising 77.7 % → 84.3 / 86.8 / 86.7 %:
at 0.325 m from `wall_west` the robot is permanently inside
`PolygonSlow`, which scales **angular** velocity by 0.3. And **r3's entry
passed within 0.2 mm of re-triggering the stop** — the ±75 mm design
margin is not the achieved margin.

`gazebo_models` **41/41** on a clean ROS graph.

### Two corrections worth carrying forward

**`nav_bench`'s `min_clearance_m` is not a safety statistic in this
pocket.** It is quantised to the 5 cm map grid and disagreed with exact
world-file geometry by up to **106 mm in both directions**: it reported
0.201 m where the truth was 0.3066 m, and — worse — **0.339 m for the one
leg that genuinely entered the stop circle at 0.2437 m**, hiding the only
real incursion in the series. The laser and the exact geometry agree to
0.1–0.2 mm on five of six candidate legs. Use `c2nav7_geom.py track`.

**The shared-scratch-directory trap fired, exactly as the table
describes.** An offline test run from a directory containing a stray
`numbers.py` **and** `trace.py` shadowed the stdlib, broke `numpy` inside
the import chain, **and printed another run's output into this one** —
both halves of the documented failure, in one command. Both files were
present. Instruments are now run from a directory this experiment owns.

### What remains unverified

Whether −3.575 is the right *operational* goal (it fixes the exit and
costs entry time); entry reliability at the shifted goal (1 of 3, N=3);
whether a smaller shift would also work (−3.500 and −3.550 are inside the
corridor on paper, untested); whether the corridor argument generalises
to any other leg; the seven-leg tour, the fetch mission and grasping,
none of which were run; and **topology B**, where CSF 65 is still
unvalidated in the configuration the robot ships in.

Nothing is approved for merge. `gazebo_models/config/nav2_params.yaml` is
untouched and still carries C2-NAV.2's rejected `BaseObstacle.scale: 2.0`.

### Exact next command

```bash
# C2-NAV.8: does the shifted goal hold across the FULL TOUR?
# Same validated parameter file, same goal override, seven legs, 75 s
# each, several fresh simulators -- against C2-NAV.5's committed 18/21.
#
# Watch three things: enclosure_entry's SUCCEEDED rate (1 of 3 in
# C2-NAV.7), whether enclosure_exit holds 0 STOP frames when it starts
# from a tour-length approach rather than a two-leg one, and how often
# the entry path comes as close to box_obstacle_1 as r3's 0.2502 m did
# -- which was 0.2 mm outside the stop radius.
cd ~/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis
bash .navbench/c2n7_run.sh \
    "$PWD/docs/data/c2nav4_csf65_params.yaml" c2n8_tour_r1 ALL 75 \
    enclosure_entry:-3.575,2.95

# then, per run, the exact clearance -- NOT nav_bench's min_clearance_m,
# which is quantised to the 5 cm map grid and was wrong by up to 106 mm
# in both directions in C2-NAV.7:
python3 docs/data/c2nav7_geom.py track \
    .navbench/results/c2n8_tour_r1_stop.csv "C2-NAV.8 tour r1"
```

---

## 2026-09-03 — C2-NAV.8: the whole tour, and the trap moved to the other corner

**Branch** `worktree-c2nav0-diagnosis`. A **validation** pass. No
parameter searched, swept or changed; three complete seven-leg tours on
three genuinely fresh simulators, against C2-NAV.5's committed 18/21.

### What was built

- `docs/data/c2nav8_report.py` — `collect` / `legs` / `entry` / `exit` /
  `stop` / `clear` / `compare` / `all`, reading either the `.navbench`
  scratch directory or the committed `c2nav8_bench.json`, verified with
  `diff` across all six reading modes to give byte-identical tables from
  both. Three things in it are new because this is a SEVEN-leg tour
  where C2-NAV.6 and C2-NAV.7 ran two legs:
  **(1)** the probe CSV is re-segmented across all seven legs
  **offline**, leaving `c2nav6_stopprobe.py` byte-identical — its
  `LEG_GOALS` knows only the two enclosure legs, and a count taken by a
  changed instrument is not comparable to C2-NAV.6's by construction;
  the probe's own JSON stays as an independent cross-check and agrees on
  every run. **(2)** true clearance over the whole world, importing
  `c2nav7_geom.py`'s `nearest`/`BOXES`/`STOP_RADIUS`/`CIRCUMSCRIBED`
  rather than restating them. **(3)** the C2-NAV.5 comparison reads
  `c2nav5_bench.json`, so it is a file rather than a memory.
- `nav_bench.py` gained a **default-off `--leg-timeout NAME:SECONDS`**
  override, mirroring C2-NAV.7's `--goal`. A single `--timeout` cannot
  serve both constraints: the six ordinary legs must stay at C2-NAV.5's
  75 s to stay comparable, while C2-NAV.7's entry ran 116.56 / 150.68 /
  150.01 s against a 150 s cap with **two of three ending AT the cap**,
  so 150 s cannot separate slow convergence from failure.
  `enclosure_entry` alone gets **200 s** and every leg records the cap it
  ran under as `timeout_s`. An offline test asserts no-override
  identity, isolation, repeatability and rejection of malformed,
  non-numeric, non-positive and unknown-scenario specs — 22 checks,
  before anything launches.
- `.navbench/c2n8_*` — the run harness (not committed, as with C2-NAV.0
  through C2-NAV.7), including `c2n8_bracketcheck.sh`, which passed
  against the fourteen command lines with **three** positive controls.

### Two instruments were wrong and were caught before their numbers were used

**The report had to prove it could see a stop.** Its headline claim is
partly a count that should be zero, so the segmentation and clearance
were pointed at C2-NAV.6's and C2-NAV.7's **committed** CSVs first: they
reproduce **1470** STOP frames at **exactly 6** returns and **0.2437 m**
to `box_obstacle_1`, and C2-NAV.7's three zeros. Only then were they
pointed at anything new.

**The exact-geometry clearance metric was itself wrong.**
`c2nav7_geom.py`'s eight-box list is complete for the two enclosure legs,
which is all C2-NAV.7 used it for — a seven-leg tour also passes
`cylinder_obstacle`, two pilasters and the ramp/platform footprints.
Measured on tour r1 before the fix: `corridor_gate` scored **0.6254 m**
against the laser's **0.3795 m**, a 246 mm **overstatement** — the one
direction a clearance number must never err. With the full world in,
geometry and laser agree to **0.1–0.8 mm on all 21 legs**.

**And a number this session inherited was wrong.** C2-NAV.7's "5325
frames" is the sum of the two *labelled* legs; the CSVs hold **5384**
rows, the other **59** recorded before the first goal is accepted. Both
are now asserted, because that difference is exactly the off-by-a-segment
that would make a per-leg STOP count look clean by dropping frames.

### What was measured

**18 of 21 SUCCEEDED, 19 of 21 TRAVERSED**, three fresh simulators,
RTF 0.986–0.992, all three probe positive controls passed. Per tour
**5/7, 6/7, 7/7**.

The five ordinary legs are **15/15 with 0 STOP frames on 3016 frames**,
true clearance 0.3792–0.5160 m.

**`enclosure_entry` 1/3 SUCCEEDED, 2/3 TRAVERSED** — 201.42 / 200.22 /
123.67 s, final error 1.076 / 0.125 / 0.118 m. The two that arrived
reached the 0.25 m tolerance in **25.61 / 26.45 s** — faster than
C2-NAV.5's 74.91 s whole-leg median — then spent **174.61 / 97.23 s**
(87.2 % / 78.6 % of the leg) settling the goal yaw. That is C2-NAV.1's
mechanism, and on r2 it alone costs the SUCCEEDED.

**`enclosure_exit` 2/3 SUCCEEDED** — 34.28 and 47.71 s, driving 3.515 and
4.280 m with **0 STOP frames on 827 frames**, command chain `v_nav`
0.2684 → wheel **0.0853**, the 0.3 `slowdown_ratio`, reproducing
C2-NAV.7 to the digit. Throttled, not gated.

**The new failure, and a two-leg protocol could not have found it.**
r1's entry is a **269.5 s continuous `PolygonStop` deadlock** at
(−3.3009, +1.9100) — two poses **0.8 mm apart**, `v_wheel` exactly
**0.0 on all 2673 frames** while `v_nav` spans **−0.15 to +0.2526**. The
gate is `box_obstacle_1`'s **SOUTH-west corner** at 0.2453 m, **4.7 mm
inside** the circle, 5–6 returns; C2-NAV.6's trap was the **NORTH**-west
corner at 5.5 mm and 6 returns. Both enclosure legs are lost and the exit
drove **0.000 m**. `STOP` zeroes all three axes, so the −0.15 m/s
recovery reverse reached the wheels as 0.0 — C2-NAV.6's finding,
reproduced on the entry.

**Why C2-NAV.7 missed it is structural.** Its two-leg run started the
entry at the **spawn (−2.000, 0.000)**; in the tour the entry is leg 6
and starts where `corridor_gate` ended, ≈(−2.58, −0.02), 0.6 m further
west. Different approach, and it clips the SW corner **before** reaching
the corridor C2-NAV.7 derived. **That corridor is about where the robot
ENDS and says nothing about how it gets there.**

**Against C2-NAV.5, like for like:** total **18/21 both**; entry
2/3 → 1/3 and 74.91 → 200.22 s median; exit 1/3 → **2/3** and
77.14 → **47.71 s**. The shift trades one entry success for one exit
success and buys 125 s of entry for 29 s of exit.

**Safety.** Minimum true clearance over all 21 legs and 10 626 frames is
**0.2453 m**, **40.2 mm above** the 0.2051 m circumscribed radius;
nothing approached below it. `nav_bench`'s `min_clearance_m` was wrong by
**−157 mm** on one leg — worse than C2-NAV.7's 106 mm — and read 0.150 m
for a leg whose true minimum was 0.2558 m.

**Verdict: PARTIALLY VALIDATED.** Exit clean, safety intact, ordinary
legs untouched; but the tour total does not improve, entry regresses and
is 2.7× slower, and 1 fresh simulator in 3 immobilises the robot.

`gazebo_models` **41/41** on a clean ROS graph.

### What remains unverified

`docs/RESULTS.md`, "C2-NAV.8 … NOT PROVEN". The short list: N = 3 tours
is an engineering-validation count and "1 of 3" bounds nothing — the
deadlock's true frequency is unmeasured; what determines which of the two
entry modes a tour falls into is uncontrolled; whether any offset inside
the corridor avoids the SW corner on the approach is untested and
deliberately so; whether the terminal-yaw cost is removable; whether the
deadlock ever recovers given more than 269.5 s; the fetch mission and
grasping; and — still the one that matters most — **every run in
C2-NAV.0 through C2-NAV.8 is topology A, while `mission.launch.py` runs
topology B. CSF 65 AND the shifted goal are both unvalidated in the
configuration the robot ships in.**

Nothing is approved for merge. `gazebo_models/config/nav2_params.yaml` is
untouched and still carries C2-NAV.2's rejected `BaseObstacle.scale: 2.0`.

### Exact next command

```bash
# C2-NAV.9, step 1: the APPROACH corridor, offline, BEFORE any simulator.
# C2-NAV.7 computed the band of x that clears PolygonStop.radius from
# wall_west AND box_obstacle_1 for a robot PARKED at the goal. The
# deadlock is 0.6 m short of that band, on a NORTHBOUND approach, against
# box_obstacle_1's SOUTH-WEST corner (-3.250, +2.150) -- a corner that
# analysis never considered. Compute the band that clears BOTH, and if it
# is empty, say so: the goal is then not repairable by moving it again.
cd ~/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis
python3 - <<'PY'
import sys; sys.path.insert(0, 'docs/data')
from c2nav7_geom import BOXES, STOP_RADIUS, dist_to_box
# the deadlock pose, measured: 0.2453 m from box_obstacle_1's SW corner
b = [x for x in BOXES if x[0] == 'box_obstacle_1'][0]
print('box_obstacle_1 rect:', b)
print('deadlock:', dist_to_box(-3.3009, 1.9100, b))
print('stop radius:', STOP_RADIUS)
PY

# Then, and ONLY if the geometry says a path exists, the tour again --
# same parameter file, same override mechanism, no new knob:
bash .navbench/c2n8_all.sh 3 1
```

## 2026-09-03 — C2-NAV.9: the corridor is 76 mm wider than it needs to be, and DWB cannot tell

**An offline geometry/feasibility diagnosis, explicitly not an
intervention.** No navigation parameter searched, swept or changed; the
goal not moved; `PolygonStop` untouched; no simulator run except to
generate one deterministic PNG from already-committed data. Full record:
`docs/RESULTS.md`, "C2-NAV.9 navigation approach-corridor reconstruction".

**What was built.** `docs/data/c2nav9_corridor.py` — a self-testing
offline tool that (1) reproduces C2-NAV.6's 5.5 mm and C2-NAV.8's 4.7 mm
corner penetrations from committed CSV/JSON data before anything new is
trusted; (2) builds a 3 mm clearance field over the `corridor_gate` →
`enclosure_entry` region from `nearest_full` (C2-NAV.8's full-world fix,
not the incomplete eight-box list); (3) answers "how wide is the
corridor" as a maximum-bottleneck / widest-path problem via binary search
over `scipy.ndimage.label` connectivity — exact given the grid, not a
guess; (4) reproduces `nav2_collision_monitor`'s own circle/square logic
for arbitrary heading, closed-form for the squares (`reach = hw/cos(θ)`,
θ folded into `[0°,45°]`) and via a dense 720-heading sweep against a
continuous sampled obstacle surface; (5) correlates all of it against the
three committed C2-NAV.8 tours' actual ground-truth trajectories.

**What was measured.**

* **A fully `PolygonStop`-free route exists from every one of C2-NAV.8's
  three real `corridor_gate` exits to the current goal — bottleneck
  326.0 mm, 76 mm of margin over the 250 mm `PolygonStop` needs.** The
  SW-corner deadlock is not a geometric necessity.
* **But the SW corner is genuinely load-bearing on that widest path**,
  not an avoidable detour: the bottleneck's tightest point,
  `(-3.535,+1.998)` at 323 mm, is closest to `box_obstacle_1`'s SW
  corner, not the wall. A second, comparably-tight pinch sits in the
  classic NW wall/box gap at 325–329 mm.
* **r1's 269.6 s deadlock, reconstructed** from
  `c2nav8_tour_r1_stop.csv`: frozen at `(-3.3001,+1.9095)`, 0.2457 m /
  4.3 mm penetration of the SW corner, matching C2-NAV.8's own "269.5 s"
  and "4.7 mm" to within one CSV row's resolution.
* **`PolygonSlow` is mathematically unavoidable at the current goal for
  every heading** — its minimum possible reach (0.400 m, closed-form for
  a 0.8×0.8 m body-fixed square) exceeds the goal's own clearance to the
  nearest geometry (`wall_west`, 0.325 m). A 720-heading dense sweep
  confirms it algebraically (0% / 100%, never partial — as expected for
  `PolygonStop`, a circle, vs. `PolygonSlow`/`PolygonLimit`, squares).
  Matches C2-NAV.8's own recorded `SLOWDOWN` fractions (94.3% / 93.0%)
  exactly.
* **The unavoidable `SLOWDOWN` does not explain the observed duration.**
  A worst-case single in-place turn needs ≤π rad; C2-NAV.8's own
  `terminal_yaw_travel_rad` was **8.494 / 10.572 rad** — 2.7×/3.4× more,
  several net revolutions, a hunting signature layered on top of the
  geometrically-proven `SLOWDOWN` tax.
* **±0.30 m around the goal: 0.0% of the pocket is `PolygonSlow`-clear
  for ANY heading.** No nearby goal position escapes this by moving.
* **Correlation against the three real tours**: the offline field,
  evaluated at each run's actual GT samples, reproduces the committed
  `cm_action_frac` labels exactly — r1 90.6% of samples below
  `PolygonStop.radius` (matches its 96.2% `STOP` fraction), r2/r3 0%
  (matches their 0% `STOP`, 93–94% `SLOWDOWN`), passing at 257/259 mm —
  only 7–9 mm above the 250 mm threshold, despite 326 mm being available.

**Root-cause classification: B (a feasible path exists, Nav2 does not
reliably select it), compounded by D (the collision monitor's
`PolygonSlow` interacting with the terminal yaw controller).** INFERRED,
not directly instrumented this session: `local_costmap.cost_scaling_factor
= 65.0` was set in C2-NAV.4 specifically so cost reaches 0 at 0.291 m —
meaning DWB's `BaseObstacle` critic cannot distinguish 257 mm from
326 mm at all, so nothing in the local cost function rewards the wider,
safer route over one that skims the SW corner. Which one a given fresh
simulator's DWB sampling converges to is exactly the kind of run-to-run
variance three tours would show one of.

**Verdict.** The goal does not need to move again — nothing here shows a
different `(x,y)` would be safer by a comparable margin, and the
`PolygonSlow` finding shows no nearby position escapes it either. What is
missing is something between `corridor_gate` and the goal that gives DWB
a reason to prefer the 326 mm route over the 245–260 mm one it is
currently indifferent between.

**Tests.** None run — no source, launch file, parameter or test touched.
`gazebo_models/config/nav2_params.yaml` untouched, still carries
C2-NAV.2's rejected `BaseObstacle.scale: 2.0`. `docs/RSE_ASSIGNMENT_PLAN_V2.md`
untouched. `main` untouched (this branch is `worktree-c2nav0-diagnosis`,
still based on `main` at `ea66155`).

**What remains unverified.** `docs/RESULTS.md`, C2-NAV.9's own OBSERVED /
INFERRED / NOT PROVEN. Short list: whether a corridor-aligned waypoint
actually changes DWB's path selection (no simulator run this session);
the deadlock's true rate (still N=1 of 3); whether `/plan` itself already
threads the SW corner or DWB's local sampling diverges from a wider
global plan (no `/plan` capture exists to check); topology B, still
untouched by C2-NAV.0 through C2-NAV.9.

### Exact next command

```bash
# C2-NAV.10: does a single corridor-aligned waypoint change which path
# DWB actually samples as cheapest? C2-NAV.9 proved the corridor has
# 76 mm of geometric slack (326 mm vs. the 250 mm PolygonStop needs) and
# that BaseObstacle cannot see the difference above 291 mm (C2-NAV.4).
# This is a controller-behaviour question offline geometry cannot answer.
#
# NOT a goal coordinate change -- C2-NAV.7 already moved this goal once
# on a two-leg approach that did not generalise to the tour (C2-NAV.8),
# and the goal itself is not shown unsafe by C2-NAV.9's geometry.
cd ~/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis
python3 docs/data/c2nav9_corridor.py nofig   # re-derive the wide-band
    # waypoint region if the world file changes; currently x~-3.6,
    # y~1.2-1.5, >=0.45 m clearance per corridor()'s grid.

# Then, unchanged in every other respect except one intermediate pose
# on the approach to enclosure_entry (mechanism: coco_mission's approach
# sequencing or a nav_bench.py TOUR insertion -- pick one, do not do
# both), the full seven-leg tour, 3 fresh simulators:
bash .navbench/c2n8_all.sh 3 1
# Watch specifically: does any run's true minimum clearance to
# box_obstacle_1 (docs/data/c2nav7_geom.py track, NOT nav_bench's
# quantised min_clearance_m) still fall in the 245-260 mm band all three
# C2-NAV.8 tours landed in, or does it move toward 320+ mm?

## 2026-09-03 — C2-NAV.10: the waypoint reaches itself reliably, and stops mattering the moment it does

**A single-hypothesis intervention, tested and REJECTED, with a precise
mechanistic reason.** Full record: `docs/RESULTS.md`, "C2-NAV.10
navigation corridor-aligned waypoint". Hypothesis: an explicit
corridor-aligned intermediate waypoint before `enclosure_entry` causes
Nav2/DWB to select C2-NAV.9's geometrically wide 326 mm approach instead
of drifting toward the 245–260 mm SW-corner trap that deadlocked
C2-NAV.8's r1 tour for 269.5 s. No other variable moved: CSF 65.0/5.0,
`inflation_radius` 0.5, `BaseObstacle.scale` 8.0, `SimpleGoalChecker`,
`PolygonStop.radius` 0.25/`min_points` 4, final goal `(-3.575, 2.95)` —
all read back off the live nodes on every run.

**The waypoint: `(-3.40, 1.35)`, derived, not copied.** The brief's
illustrative `x≈-3.6` only measures 300 mm clearance from `wall_west` —
short of its own ≥450 mm target. `c2nav9_corridor.py`'s clearance grid
gives `x=-3.40` exactly 500 mm, a 326 mm bottleneck to the goal
(unchanged from the whole-corridor figure — no worse route forced), zero
`PolygonStop` interaction, and only a 22.6° turn at the waypoint. Static
validation in `.navbench/c2n10_static_report.py` passed every check
before any simulator ran.

**Implementation: one function in `nav_bench.py`, mirroring the file's
own established pattern.** `apply_waypoint_insert(tour, specs)`, wired to
a new default-off `--waypoint BEFORE:X,Y` flag, inserts one leg into a
*copy* of the tour immediately before a named scenario — `TOUR` itself
untouched, exactly as C2-NAV.7's `apply_goal_overrides` and C2-NAV.8's
`apply_leg_timeouts` already do it. No `NavigateThroughPoses` or
waypoint-follower exists anywhere in this repo, so a second independent
`NavigateToPose` call is the smallest architecture-compatible mechanism
per the brief's own §5 instruction.

**One real bug, caught before it corrupted a result.** `main()` applied
`--only` filtering *after* waypoint insertion, so `--only corridor_gate,
enclosure_entry` silently dropped the inserted `enclosure_entry_waypoint`
leg — its own generated name matched neither filter term. The first live
attempt consequently drove the unmodified C2-NAV.8 route and proved
nothing; its artifacts are kept as `VOID_ORDERBUG_c2n10_appr_r1*`, not
deleted. Fixed by reordering (goal overrides → `--only` filter by
original name → waypoint insert → leg timeouts) and regression-tested
offline for all four call shapes before re-running. This is exactly the
class of error the brief's §5 "verify the waypoint is materially
represented" requirement exists to catch.

**Three fresh-simulator runs, exact same waypoint, §11 obeyed —
two distinct failure modes, not one:**

| run | waypoint leg | final leg | whole-run STOP | closest to SW corner | closest to r1's deadlock pose |
|---|---|---|---:|---:|---:|
| r1 | SUCCEEDED | TIMEOUT — 6.909 rad terminal yaw, 88.0% of leg | 0.00% | 799 mm | 568 mm |
| r2 | SUCCEEDED | TIMEOUT — froze in transit, err 1.02 m | 74.23% | 241 mm (inside PolygonStop) | 51 mm |
| r3 | SUCCEEDED | ABORTED — froze in transit, err 1.11 m | 58.59% | 253 mm | 97 mm |

The `enclosure_entry_waypoint` leg SUCCEEDED cleanly in all three runs —
0% `PolygonStop`, 0% illegal-transit fraction, 57–73 mm terminal error.
**That part of the mechanism is confirmed 3/3.** But r2 and r3 froze
within 51–97 mm of C2-NAV.8 r1's own recorded deadlock pose, on the leg
immediately *after* the waypoint — `dwb_best_critic_mean.BaseObstacle`
stays 0.0 on the chosen trajectory the entire time, meaning the local
cost field genuinely does not register the SW-corner approach as costly,
exactly C2-NAV.9's diagnosis. **The mechanism is architectural, not a
bad coordinate:** each leg is an independent `NavigateToPose` call, so
Nav2 replans from scratch at the waypoint, and nothing about having
passed through it persists into the very leg where the pinch lives.

**Verdict: REJECTED**, brief §13's gate for seven-leg validation is not
met, and no waypoint coordinate was moved to try to fix it (§11). The
underlying C2-NAV.9 diagnosis stands — the wide route exists and gets
used reliably whenever the planner starts far enough from the pinch —
but a terminating via-pose is the wrong shape of intervention for the
segment where the failure actually occurs.

**Tests.** `.navbench/c2n10_logic_test.py` (nav_bench.py regression, no
ROS): 6/6 PASS. No `coco_mission`/`coco_rl`/etc. package test suite
touched — this experiment only edits `gazebo_models/scripts/nav_bench.py`
and adds `.navbench/c2n10_*` / `docs/data/c2nav10_report.py`.
`gazebo_models/config/nav2_params.yaml` untouched, still carries
C2-NAV.2's rejected `BaseObstacle.scale: 2.0`. `docs/RSE_ASSIGNMENT_PLAN_V2.md`
untouched. `main` untouched (branch `worktree-c2nav0-diagnosis`, still
based on `main` at `ea66155`).

**What remains unverified.** `docs/RESULTS.md`, C2-NAV.10's own OBSERVED
/ INFERRED / NOT PROVEN. Short list: whether a **continuous** multi-pose
global plan (no re-plan boundary at the waypoint) would carry the
wide-corridor preference across the SW-corner segment — not tested this
session; the true rate of either failure mode (N=3); whether r2/r3's
`/plan` itself threads near the SW corner or a wide `/plan` exists with
DWB's local sampling diverging from it (no `/plan` capture diffed);
topology B, still untouched by C2-NAV.0 through C2-NAV.10.

### Exact next command

```bash
# C2-NAV.11: does a CONTINUOUS multi-pose global plan -- not two
# independent NavigateToPose calls with a re-plan boundary between them
# -- carry the wide-corridor preference across the SW-corner segment
# that C2-NAV.10 showed a terminating via-pose cannot?
#
# First establish the mechanism exists in this Nav2 (Jazzy) build:
cd ~/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis
ros2 action list -t | grep -i navigate
ros2 interface show nav2_msgs/action/NavigateThroughPoses 2>&1 | head -20
# If it exists and is wired to the same DWB/costmap stack as
# NavigateToPose, the smallest test is one NavigateThroughPoses call with
# poses [waypoint, enclosure_entry] in place of C2-NAV.10's two chained
# NavigateToPose legs -- same waypoint (-3.40, 1.35), same goal, same
# params, 3 fresh simulators. Do NOT move the waypoint (still C2-NAV.9's
# derived coordinate) and do NOT tune DWB/CSF/PolygonStop.
#
# Falsifiable prediction: if C2-NAV.10's re-plan-boundary diagnosis is
# right, the second segment's BaseObstacle/PathAlign should now favour
# the wide corridor too, and the SW-corner freeze rate should drop below
# C2-NAV.10's 2/3. If it does not, the cost-field-blindness diagnosis
# itself needs revisiting, not the waypoint mechanism.
#
# Do NOT run the seven-leg tour until this establishes whether a
# continuous plan actually changes the outcome -- C2-NAV.10 already
# showed the chained-leg mechanism does not, and the brief's own gate
# (§13) says a negative result there adds no information from a full tour.
```

## 2026-09-03 — C2-NAV.11: the boundary was the mechanism, and removing it removes the deadlock

**A single-hypothesis intervention, tested and CONFIRMED, with a
mechanistic proof, not just a pass/fail count.** Full record:
`docs/RESULTS.md`, "C2-NAV.11 navigation continuous multi-pose enclosure
approach". Hypothesis: representing C2-NAV.9's waypoint `(-3.40, 1.35)`
and `enclosure_entry`'s own goal inside ONE continuous
`NavigateThroughPoses` request — instead of C2-NAV.10's two independent
`NavigateToPose` calls — removes the re-plan boundary C2-NAV.10 diagnosed
as the reason its waypoint mechanism failed on 2 of 3 runs. No other
variable moved: CSF 65.0/5.0, `inflation_radius` 0.5, `BaseObstacle.scale`
8.0, `SimpleGoalChecker`, `PolygonStop.radius` 0.25/`min_points` 4,
waypoint `(-3.40, 1.35)`, final goal `(-3.575, 2.95)` — all read back off
the live nodes on every run.

**First finding, before any live run: `NavigateThroughPoses` was
silently broken in this repo.** `bt_navigator.default_nav_through_poses_
bt_xml` in the validated baseline pointed at the SINGLE-pose behaviour
tree (`ComputePathToPose goal="{goal}"`), not the multi-pose one
(`ComputePathThroughPoses goals="{goals}"`) — a pre-existing
misconfiguration, unrelated to any earlier C2-NAV experiment, that would
have made a `NavigateThroughPoses` call silently plan straight to the
last pose with no error. Fixed with a ONE-LINE parameter change
(`docs/data/c2nav11_ntp_params.yaml`, sha256 `6f61e499…`, diffs from the
validated baseline `3d9623d6…` in exactly that line) pointing at Nav2's
own stock through-poses BT XML — not a DWB/costmap/PolygonStop tuning
change, a wiring correction to make the codebase's own declared
navigator do what its name says. Full byte-level evidence in
`.navbench/c2n11_iface_check.txt` / `.sh`.

**Implementation: `apply_through_poses()` in `nav_bench.py`, a different
shape than C2-NAV.10's `apply_waypoint_insert()`.** It does not splice a
leg into the tour — it builds `{name: [(x,y),...]}`, consulted by name at
dispatch, so `--through-pose` has none of C2-NAV.10's `--only`-ordering
hazard. A new `send_multi_leg()` sends one `NavigateThroughPoses` goal;
`nav_bench.py` also gained a `/plan` snapshot ring buffer so the FIRST
plan after goal acceptance can be captured directly. `.navbench/
c2n11_logic_test.py`: 8/8 PASS, no ROS.

**Proof of continuity — the load-bearing evidence.** In all 3 runs, the
first `/plan` after goal acceptance arrived **4–6 ms** later, 105–107
poses, ending **12 mm** from the FINAL goal, while the robot was still
3.2 m away at its leg-start pose. C2-NAV.10's mechanism cannot produce
this by construction (its second leg's plan doesn't exist until the
first leg's independent action call has already finished, tens of
seconds later). This is direct evidence the re-plan boundary is
structurally absent, not an inference from success alone.

**3 of 3 fresh-simulator runs SUCCEEDED on `enclosure_entry`** — against
C2-NAV.8's 1/3 and C2-NAV.10's 1/3 at the identical waypoint and cost
field:

| run | duration | final err | closest to SW corner | closest to C2-NAV.8 r1 deadlock pose | PolygonStop | true min clearance |
|---|---:|---:|---:|---:|---:|---:|
| r1 | 61.64 s | 0.038 m | 0.302 m | 0.206 m | 0/626 (0%) | 0.2698 m |
| r2 | 112.38 s | 0.073 m | 0.304 m | 0.201 m | 0/1155 (0%) | 0.2956 m |
| r3 | 156.37 s | 0.044 m | 0.273 m | 0.153 m | 0/1600 (0%) | 0.2739 m |

Never inside `PolygonStop`'s 0.25 m circle; true clearance (the collision
monitor's own live lidar-derived base-frame distance — NOT `nav_bench`'s
quantized `min_clearance_m`, which read 0.165–0.191 m here and is known
unreliable by up to 106 mm per C2-NAV.7) never fell below 0.2698 m, 19.4
mm above the STOP threshold. No safety regression.

**DWB mechanism matches the falsifiable prediction C2-NAV.10 itself
made.** `dwb_best_critic_mean.BaseObstacle` is nonzero on the chosen
trajectory in all 3 runs (0.74/0.18/10.5) — C2-NAV.10's frozen legs
showed it pinned at 0.0 throughout. `path_efficiency` 0.86–0.89 all
three; the robot is always closing distance, never standing dead against
the corner.

**Terminal yaw is untouched, exactly as predicted — orthogonal, not
fixed.** 4.2–11.5 rad, 60–83% of leg time, reproducing C2-NAV.9/10's
diagnosis almost exactly. This is why total duration (62–156 s) is 2–4×
transit time despite clean route selection in every run; this experiment
targeted route selection only.

**One infrastructure note, not a navigation finding:** `c2n11_appr_r2`'s
`nav_bench.py` segfaulted (exit 139) AFTER writing its results JSON and
after the PolygonStop probe reported TELEMETRY OK — an `rclpy`/DDS
teardown crash, verified not to have corrupted any recorded data. Did not
recur in r1 or r3.

**Verdict: CONFIRMED.** Removing the re-plan boundary — same waypoint,
same goal, same cost field, same DWB config as C2-NAV.10 — removes the
SW-corner deadlock. **Not yet cleared for seven-leg tour validation**:
this repo has never exercised `NavigateThroughPoses` before this
session, so its behaviour inside a CHAINED tour (six legs of accumulated
drift before `enclosure_entry`, rather than a fresh spawn) is unverified,
and N=3 is reproducibility, not a rate.

**Tests.** `.navbench/c2n11_logic_test.py`: 8/8 PASS (no ROS). C2-NAV.10's
own `c2n10_logic_test.py` re-run and still 6/6 PASS (no regression from
sharing `nav_bench.py`). No `coco_mission`/`coco_rl`/etc. package test
suite touched — this experiment only edits `gazebo_models/scripts/
nav_bench.py` and adds `docs/data/c2nav11_*` / `.navbench/c2n11_*`.
`gazebo_models/config/nav2_params.yaml` untouched.
`docs/RSE_ASSIGNMENT_PLAN_V2.md` untouched. `main` untouched (branch
`worktree-c2nav0-diagnosis`, still based on `main` at `ea66155`).

**What remains unverified.** `docs/RESULTS.md`, C2-NAV.11's own OBSERVED
/ INFERRED / NOT PROVEN. Short list: whether the `/plan` polyline itself
threads wide of the SW corner at every point, not just at its endpoint
(same gap C2-NAV.9/10 left open); the true SW-corner-avoidance rate at
scale (N=3); behaviour inside a chained seven-leg tour; topology B,
still untouched by C2-NAV.0 through C2-NAV.11.

### Exact next command

```bash
# C2-NAV.12: does the SAME --through-pose mechanism and waypoint hold up
# inside a full CHAINED seven-leg tour, not a fresh spawn straight into
# corridor_gate? C2-NAV.11 proved the continuity mechanism and got 3/3 on
# the enclosure-approach segment alone; this is the brief's own §15 next
# gate (a positive result licenses further validation, not a jump
# straight to declaring the mechanism mission-ready).
cd ~/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis
bash .navbench/c2n11_run.sh docs/data/c2nav11_ntp_params.yaml c2n12_tour_r1 \
    ALL 75 enclosure_entry:-3.575,2.95 enclosure_entry:200 \
    enclosure_entry:-3.40,1.35
# Do NOT move the waypoint (-3.40, 1.35) and do NOT tune CSF/inflation/
# BaseObstacle/PolygonStop/goal-checker/DWB. The only open question is
# whether a chained approach heading into corridor_gate changes anything
# about the continuity mechanism C2-NAV.11 already proved. 2 more fresh
# runs (3 total) if the first is clean, matching C2-NAV.11's own
# reproducibility count. If 3/3 (or close) holds, NavigateThroughPoses
# is a candidate to promote from benchmark-only to coco_mission's actual
# navigation interface -- a separate, larger decision, not this
# experiment's to make.
```

## 2026-09-04 — C2-NAV.12: the mechanism still works, but the tour does not reliably reach it

**A validation experiment, REJECTED.** Full record: `docs/RESULTS.md`,
"C2-NAV.12 navigation seven-leg tour with genuine continuous multi-pose
enclosure approach". Question: does C2-NAV.11's fix — one continuous
`NavigateThroughPoses` request through the waypoint `(-3.40, 1.35)` to
`enclosure_entry`'s own goal `(-3.575, 2.95)` — remain reliable across
the **complete seven-leg tour**, with the heading and AMCL state five
preceding legs actually leave behind, rather than C2-NAV.11's own
fresh-spawn two-leg start? No parameter moved: same
`docs/data/c2nav11_ntp_params.yaml` (sha256 `6f61e499…`), same goal, same
waypoint, same `PolygonStop`/CSF/BaseObstacle/goal-checker, same leg
timeouts (75 s ordinary, 200 s `enclosure_entry`, reused from C2-NAV.8
rather than picked blind).

**Answer: no.** 3 fresh seven-leg tours: **1/3 SUCCEEDED** on
`enclosure_entry`. r2 reproduced C2-NAV.8's own SW-corner `PolygonStop`
deadlock to within **51.8 mm** of its frozen pose — `PolygonStop`
engaged 92.5% of entry then 100% of the following exit, ~282 s total,
zero net displacement on the exit, exactly C2-NAV.8's own finding that a
failed entry costs the exit after it. r1 produced a mechanism no prior
C2-NAV session has seen: nine `Failed to make progress` cycles, two
failed recoveries, then `bt_navigator` **ABORTED** via a planner
`"Start occupied"` failure, wedged against `box_obstacle_1`'s EAST face
(never near the SW corner or NW pinch). Only r3 succeeded cleanly (0%
STOP both enclosure legs, throttled exit). **The seven-leg total, 17/21,
is measured WORSE than C2-NAV.8's own unfixed baseline (18/21) at the
identical goal in the identical tour.**

**The mechanism itself is not broken — confirmed by direct evidence in
all three runs, including the two that failed.** The early-`/plan`
continuity proof C2-NAV.11 introduced reproduces exactly: 4–8 ms after
acceptance, 100–102 poses, 12 mm from the final goal, in r1 and r2 too.
What breaks is a claim C2-NAV.11's fresh two-leg runs never had reason to
test: **"the waypoint is in the request" is not the same claim as "the
waypoint is reached."** Read out of the installed, unmodified Nav2 BT
XML: `RemovePassedGoals radius="0.7"` drops an intermediate pose once the
robot passes within 0.7 m of it — independent of
`goal_checker.xy_goal_tolerance` (0.25 m). Closest approach to the
waypoint: r1 0.551 m, r2 0.293 m, r3 0.006 m. Only r3 genuinely arrived;
r1 and r2 were close enough to be silently pruned from `{goals}` without
arriving, and the remaining single-goal replan was exposed to precisely
the re-plan-boundary failure C2-NAV.11 was built to remove — confirmed at
r1's final replan attempt, `GridBased plugin failed to plan ... to
(-1.58, 2.95)` (map frame), the FINAL goal alone.

**Why the approach passes the waypoint differently: heading, not
position.** `corridor_gate`'s stop pose matches C2-NAV.11's own fresh
runs to within 5 cm, but the entering YAW sign is reversed — roughly
+0.3 to +0.5 rad fresh (spawn → `corridor_gate` is the first leg) against
roughly -0.3 to -0.5 rad here (`obstacle_corner` → `corridor_gate` is the
fifth leg). A plausible proximate cause for the differing route, not
confirmed by a controlled sweep — two paired data points, not a
statistic.

**Safety: no threshold crossed.** Minimum true clearance across all 21
legs is r2's 0.2487 m, 43.6 mm above the 0.2051 m circumscribed radius
and only 1.3 mm inside `PolygonStop`'s trigger circle — why the monitor
engaged rather than a collision occurring. r1's worst approach, 0.2636 m,
never entered `PolygonStop` at all. The five ordinary legs: 15/15
SUCCEEDED, 0 STOP frames on 3016 frames across all three tours — no
regression from C2-NAV.8's own clean baseline.

**What was built.** `docs/data/c2nav12_report.py` — C2-NAV.8's own report
reused **by import** (`nearest_full`, `BOXES`, `LEGS`, `traversed`,
`_fmt`, all the seven-leg segmentation and true-clearance geometry, none
restated), extended with C2-NAV.11's waypoint-continuity check
(`closest-approach-to-waypoint`, `early_plan_*`). Verified to reproduce
every table from `docs/data/c2nav12_bench.json` alone (no `.navbench/`
needed) except the closest-approach-to-SW-corner/deadlock-pose columns,
which — like C2-NAV.11's own equivalent metric — need the per-leg trace
CSVs and are scratch-only, stated as such rather than silently
degraded. No `nav_bench.py` change was needed: C2-NAV.11's
`--through-pose` and C2-NAV.8's `--leg-timeout` already compose with a
bare `ALL` tour (`--only` omitted) with zero new code.

**Tests.** No `coco_mission`/`coco_rl`/etc. suite touched. No
`nav_bench.py` change, so C2-NAV.10's and C2-NAV.11's own offline logic
tests were not re-run (nothing they cover changed).
`gazebo_models/config/nav2_params.yaml` untouched.
`docs/RSE_ASSIGNMENT_PLAN_V2.md` untouched. `main` untouched (branch
`worktree-c2nav0-diagnosis`).

**Verdict: REJECTED**, specifically for the claim "C2-NAV.11's fix,
unchanged, reliably survives full-tour accumulated state" — not for the
underlying multi-pose mechanism, which still functions exactly as
characterised whenever the waypoint is genuinely reached. Per this
session's own brief, a repeated safety/deadlock failure — r2's is
C2-NAV.8's own, to within 52 mm — rules out anything stronger. **Not
proceeding to Topology B**: this validation's own premise did not hold.

### Exact next command

```bash
# C2-NAV.13: offline first, no simulator. Does the corridor_gate-exit
# heading reversal measured here (fresh +0.3-0.5 rad vs. tour
# -0.3-0.5 rad) actually determine how close the approach passes the
# waypoint? Use C2-NAV.9's own offline-geometry method before any live
# run. Do NOT move the waypoint (-3.40, 1.35), do NOT tune
# RemovePassedGoals' radius, and do NOT tune CSF/inflation/BaseObstacle/
# PolygonStop -- none of those has been asked for by a user decision,
# per this repo's rule 7 ("ask before assuming"). If a heading-dependent
# margin is confirmed, the next live question is a SECOND through-pose
# on the heading side of the approach, still a benchmark-level
# --through-pose addition, not a Nav2 parameter change -- a single
# hypothesis, tested once, matching this brief's own discipline.
cd ~/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis
python3 -P docs/data/c2nav12_report.py all   # start here: the full record already collected
```

## 2026-09-04 — C2-NAV.13: both mechanisms are real, and premature removal happens in every run, not just the failures

**Offline diagnosis only, no simulator, no Nav2 parameter touched.** Full
record: `docs/RESULTS.md`, "C2-NAV.13 navigation heading vs.
waypoint-removal diagnosis". Question: of C2-NAV.12's two candidate
mechanisms — (A) the tour's reversed entering heading, (B)
`RemovePassedGoals radius=0.7` pruning the waypoint before genuine
arrival — which actually drives the SW-corner failure, and how do they
interact? Built `docs/data/c2nav13_heading.py`, importing C2-NAV.9's
geometry and C2-NAV.12's constants **by import**, reading the raw
per-0.1 s traces C2-NAV.11/.12 left in `.navbench/` (uncommitted scratch,
never tracked in this repo) and cross-checking every pulled number
against a committed figure. Self-test: reproduces C2-NAV.9's 326.0 mm
bottleneck and C2-NAV.12's own three nearest-waypoint distances
(0.551/0.293/0.006 m) to within 1 mm from the raw trace alone — ALL PASS
before anything new was trusted.

**Finding 1: the heading reversal is real, quantified, and present at
t=0.** Position matches within 5 cm across all six runs studied (three
C2-NAV.11 fresh, three C2-NAV.12 tour); because the bearing FROM that
position TO the waypoint is therefore also nearly identical (117.8–120.1°
in all six), the required turn-to-face-the-waypoint differs by almost
exactly the yaw difference: the tour needs **36–58° more turn, same
rotational sense in every one of the three pairs** — not noise, not a
sign flip in the correction direction, a consistent larger turn. This
exists before the robot has moved a centimetre, strictly before any
`RemovePassedGoals` tick can act.

**Finding 2: `PolygonStop` cannot see heading at all — a closed-form,
not inferred, result.** A circle's distance to a fixed obstacle point has
no yaw term; confirmed algebraically at three test points including both
corridor_gate-end poses. `PolygonSlow`/`PolygonLimit` (squares) do vary
with heading at a fixed point (up to 41% swing in reach), consistent
with C2-NAV.9's own 720-heading sweep finding PolygonStop reads
0%/100%, never partial. So any heading effect on the outcome must act
through DWB path selection, not through an instantaneous clearance
change at a fixed position.

**Finding 3, the reframing this session actually produced: premature
`RemovePassedGoals` pruning happens in FIVE of six runs studied,
including all three of C2-NAV.11's clean successes — not just the two
tour failures.** Reconstructed the installed BT's exact mechanism first
— not from memory of generic Nav2 docs, but from the installed,
unmodified `navigate_through_poses_w_replanning_and_recovery.xml`
(`RemovePassedGoals radius="0.7"` gated by `RateController hz="0.333"`,
period 3.003 s, **not** the 1 Hz this session initially assumed), the
`remove_passed_goals_action.hpp` header, and `nav2_util`'s
`geometry_utils.hpp` (Euclidean 2D robot-base-to-goal distance, no
orientation term; the `.cpp` implementation itself is not shipped with
the binary `.deb` on this machine, so the erase-from-front vs. per-goal
algorithm detail is stated as not needed rather than guessed — it cannot
matter with exactly one via-pose). Simulating that exact tick schedule
against the raw distance-to-waypoint trace: the waypoint is pruned
*before* the trajectory's own true-nearest sample in 5 of 6 runs,
including C2-NAV.11 r2 (pruned at t=6.01 s, 0.69 m away, 2.1 s/0.58 m
before its own 0.113 m true-nearest pass — and that run still finished
0% `PolygonStop`) and C2-NAV.12 r3 itself, the one tour run everyone
(including this session's own first draft) called "the one that
genuinely arrived" — pruned by the tick model 1.3 s before its 0.007 m
true-nearest sample. **Premature pruning is the norm in this dataset, not
a defect unique to the failures — so it cannot by itself be the
differentiator.**

**Finding 4, a correction that mattered: only ONE of C2-NAV.12's three
tour runs is actually an SW-corner case.** An early version of the
SW-side-commitment check used only "south of the box," which
misclassified r1 (frozen 0.264 m EAST of `box_obstacle_1`'s east face,
inside the box's own y-span) as an SW-corner approach — caught by
cross-checking against r1's own logged frozen pose before trusting the
output, exactly the kind of self-check this repo's evidence discipline
exists to force. Tightened to require the west-side column
(x < -3.10); r1 then correctly reads NEVER. Only r2 reproduces the
classic deadlock (west-column entry t=12.70 s, frozen at (-3.249, 1.901),
51.8 mm from C2-NAV.8 r1's own pose) — **N=1**, matching C2-NAV.8's own
N=1. In r2, the heading swings 60° away from the waypoint bearing within
about one sample of the removal tick (t=9.01 s), but the position itself
does not visibly commit to the dangerous column until 3.7 s later
(t=12.70 s) — a tight temporal coincidence between the heading swing and
the prune, correlation not proof, stated as such rather than oversold.

**Verdict: both hypotheses PARTIALLY SUPPORTED, and the honest finding
is that they interact rather than compete.** `RemovePassedGoals` removes
the corridor-shaping constraint at a fixed, radius/tick-driven moment
regardless of trajectory quality; heading/accumulated state determines
whether the robot is already tracking the wide corridor closely enough,
at that moment, for the post-removal replan-to-final-goal-alone to keep
following it. Section 5's plan-geometry-before/after question is **NOT
PROVEN** — the committed record has one `/plan` capture per run, at
t≈0, not at either tick boundary; stated as a data gap, not
extrapolated past it.

**Recommended next test: heading (Hypothesis A), not because B is
rejected but because B fires in every run and cannot be the
differentiator alone.** A second, heading-correcting via-pose on the
approach to `corridor_gate` — same `--through-pose` mechanism C2-NAV.11
already built, not a Nav2 parameter change — tests whether normalising
the entering heading toward C2-NAV.11's fresh-start range moves the
SW-corner deadlock rate below C2-NAV.12's 1/3, with `RemovePassedGoals`,
CSF, inflation, `BaseObstacle` and `PolygonStop` all untouched.

**Tests.** No `coco_mission`/`coco_rl`/etc. suite touched — this is a
pure-offline geometry/trajectory script, no ROS. `gazebo_models/config/
nav2_params.yaml` untouched. `docs/RSE_ASSIGNMENT_PLAN_V2.md` untouched.
`main` untouched (branch `worktree-c2nav0-diagnosis`). No simulator run
this session.

**What remains unverified.** `docs/RESULTS.md`, C2-NAV.13's own OBSERVED
/ INFERRED / NOT PROVEN. Short list: whether heading is what actually
selects the SW-corner side in DWB's own rollout (no DWB-internal
trajectory-tree data available offline); plan geometry before/after the
removal tick (no mid-leg `/plan` capture exists); any rate for either
mechanism (N=1 genuine SW-corner case); topology B, still untouched.

### Exact next command

```bash
# C2-NAV.14 (not run this session): a heading-correcting via-pose on the
# obstacle_corner -> corridor_gate leg (or at corridor_gate's own goal),
# chosen so corridor_gate-exit heading moves toward C2-NAV.11's fresh
# range (+0.3 to +0.5 rad) instead of the tour's own -0.3 to -0.5 rad.
# Implemented via nav_bench.py's existing --through-pose mechanism
# (C2-NAV.11's own), default-off, NOT a Nav2 parameter change.
#
# Do NOT move the (-3.40, 1.35) waypoint. Do NOT tune RemovePassedGoals,
# CSF, inflation, BaseObstacle, or PolygonStop. 3 fresh seven-leg tours,
# same acceptance criteria as C2-NAV.12.
#
# Falsifiable prediction: if Hypothesis A gates the SW-corner failure,
# the deadlock rate should fall below C2-NAV.12's 1/3. If it does not,
# Hypothesis A is not dominant, and the next question is whether
# RemovePassedGoals' radius needs a user-authorised change -- not this
# experiment's decision to make.
cd ~/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis
python3 -P docs/data/c2nav13_heading.py all   # start here: the full record already collected
```

## 2026-09-04 — C2-NAV.14: the heading correction works for 2.3 seconds, and the robot still finds the same corner

**One live simulator run, one hypothesis, nothing tuned.** Full record:
`docs/RESULTS.md`, "C2-NAV.14 navigation heading-correcting through-pose".
Question: does a second, heading-correcting `--through-pose`, added
inside the `enclosure_entry` `NavigateThroughPoses` request between
`corridor_gate` and the existing waypoint `(-3.40, 1.35)`, make the
seven-leg tour reach that waypoint cleanly and avoid `box_obstacle_1`'s
SW corner? Offline first: `docs/data/c2nav14_heading_pose.py`
self-tests against C2-NAV.10's own committed waypoint figures (bearing
118.9°, clearance 500 mm, τ*=500/326 mm) before deriving
`HEADING_POSE = (-3.00, 0.625)` — the exact midpoint of the
`corridor_gate`-goal→waypoint line, chosen so the incoming and outgoing
bearings at the pose are identical (0° extra turn demanded before the
waypoint) — and confirms it STOP/Slow/Limit-clear at all 720 sampled
headings, 900 mm from the nearest obstacle, on a route that stays
≥326 mm clear end to end. `.navbench/c2n14_run.sh` (scratch, based on
C2-NAV.11's own runner) extends `--through-pose` to two flags for the
same scenario; verified offline that `nav_bench.py`'s own
`apply_through_poses`/`send_multi_leg` preserves flag order before
trusting the live request.

**Result: TIMEOUT at the 200 s cap, frozen 32.8 mm from the canonical
SW-corner deadlock pose — closer than C2-NAV.12 r2's own 51.8 mm, and
reached in 6.3 s instead of 12.7 s.** `docs/data/c2nav14_report.py`
self-tests the raw 0.1 s trace against `nav_bench.py`'s own committed
JSON (`end_world`, `final_goal_err_m` reproduced to <1 mm) before
reporting anything new. The request itself was verified correct at
runtime, not just by code inspection: the driver log shows both
`--through-pose` flags accepted in order, and `early_plan_ts_offset_
from_t0_s = 0.008` — a `/plan` message captured 8 ms after goal
acceptance already runs 99 poses to within 12 mm of the final goal,
the same continuity signature C2-NAV.11 established.

**The mechanism partially fired, then lost the argument.** By t=6.90 s
the robot reaches 13° of the desired heading (+118.9°, the bearing
from `HEADING_POSE` to the waypoint) — down from 147.8° off at t=0 —
the clearest evidence in this experiment chain that a nearby
through-pose's position genuinely pulls DWB's short-horizon heading
selection toward it. That alignment does not hold: by t=9.20 s, 2.3 s
later, it has swung 45.5° back away (to 60.4° off), the same
"heading swings ~60° near the removal tick" signature C2-NAV.13 found
in C2-NAV.12 r2, reproduced here despite the correction. **A finding
the offline geometry did not anticipate:** the robot enters the
dangerous west column (x < -3.10, C2-NAV.13's own test) at t=6.30 s —
0.6 s BEFORE its closest approach to `HEADING_POSE` itself, and at a
well-aligned heading (+112.1°, only 6.8° off desired) — so the drift
toward the SW corner is not explained by a heading error at that
instant. The executed path is already ~245 mm west of the direct
`corridor_gate → HEADING_POSE` line at that same y. Why is **NOT
PROVEN**: `nav_bench.py` only ever captured the early plan's endpoint,
never its full geometry, so whether this is the global planner's own
path shape or DWB's local cost trade-offs cannot be told apart from
this session's data — the same instrumentation gap C2-NAV.13 flagged
for the removal-tick boundary, now shown to matter earlier in the leg
too.

**A geometric side-effect neither prior experiment had reason to
surface:** `HEADING_POSE` and `WAYPOINT` are 828 mm apart — less than
2× `RemovePassedGoals`' own 700 mm radius — so both entered its removal
range within one 3.003 s tick of each other (t=6.006 s / t=9.009 s).
Whether the installed BT removes multiple queued poses per tick or
strictly front-first is still not determined (the `.cpp` is not
shipped with the installed `.deb`, same gap C2-NAV.13 recorded, now
more consequential with two via-poses instead of one).

**Verdict: REJECTED.** The brief's own criteria for CONFIRMED/PARTIALLY
CONFIRMED are not met: the waypoint was not reached cleanly (341 mm
closest approach), `PolygonStop` was not clean (97% of the 201 s leg,
`n_in_stop` constant at 5), and SW-corner interaction was not reduced —
it was reproduced tighter and faster than the prior worst case. Per
the brief's own §14 decision rule (repeats specified only after a
clear success), no additional live runs were made. **N=1**, matching
this chain's own evidence discipline for single-occurrence findings
(C2-NAV.8, C2-NAV.12 r2).

**Should the route proceed to full validation? No.** Neither
Hypothesis A (heading, this session) nor Hypothesis B (premature
`RemovePassedGoals` pruning, C2-NAV.13) alone has produced a
configuration that clears C2-NAV.12's 1/3 SW-corner rate — consistent
with, and somewhat strengthened by, C2-NAV.13's own read that the two
mechanisms interact rather than either being solely dominant.

**Tests.** No `coco_mission`/`coco_rl`/etc. suite touched. `gazebo_models/
config/nav2_params.yaml` untouched — the run used
`docs/data/c2nav11_ntp_params.yaml` unmodified (sha256 verified live
against the file on disk). `docs/RSE_ASSIGNMENT_PLAN_V2.md` untouched.
`main` untouched (branch `worktree-c2nav0-diagnosis`). One fresh
Gazebo + Nav2 simulator run this session (`c2n14_tour_r1`), `ros_clean.sh`
before it, no orphaned processes.

**What remains unverified.** `docs/RESULTS.md`, C2-NAV.14's own OBSERVED
/ INFERRED / NOT PROVEN. Short list: whether the early westward path
bow originates in the global plan or in DWB's local sampling (no
mid-leg `/plan` geometry captured, only its endpoint); whether
`RemovePassedGoals` processes multiple queued poses per tick or
strictly front-first; any failure rate for this configuration (N=1);
whether a heading pose placed BEFORE `corridor_gate` (C2-NAV.13's own
original suggestion) would behave differently — out of this session's
scope.

### Exact next command

```bash
# C2-NAV.15 (not run this session): instrument, don't tune. Extend
# nav_bench.py's plan_snapshots capture (already recording full /plan
# geometry in memory) to write 2-3 full-path snapshots per
# enclosure_entry leg -- leg start, and immediately after each modelled
# RemovePassedGoals tick -- instead of just the single early_plan
# endpoint already captured. This directly answers C2-NAV.14's own NOT
# PROVEN: whether the executed path's early westward bow originates in
# the global plan's shape or DWB's local sampling. No Nav2 parameter,
# RemovePassedGoals, CSF, inflation, BaseObstacle, PolygonStop,
# waypoint, or goal should move until that instrumentation exists.
cd ~/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis
python3 -P docs/data/c2nav14_report.py all   # start here: the full record already collected
```

## C2-NAV.15 checkpoint (2026-09-04)

**Built.** The plan-observability instrumentation C2-NAV.14 itself asked
for: `nav_bench.py`'s `send_multi_leg` already populated a full-geometry
`/plan` ring buffer (`self.plan_snapshots`, C2-NAV.11) but only ever
extracted the FIRST message (`early_plan`); C2-NAV.15 extends it to dump
EVERY snapshot across the whole leg to
`<tag>_planwindow_<leg>_rep<rep>.json`. No new subscription, no behaviour
change — confirmed by `.navbench/c2n11_logic_test.py` still passing
unchanged. `docs/data/c2nav15_planwindow.py` analyzes the capture:
self-tests against every known committed fact in this chain, computes
per-snapshot true clearance (`nearest_full`, whole world) and the
SW-column test (reused, not reimplemented, from `c2nav13_heading.py`),
identifies FIRST_BAD_PLAN, classifies CASE A/B, and renders
`docs/images/c2nav15_planwindow.png`.

**Measured.** ONE fresh seven-leg tour (`c2n15_tour_r1`), byte-identical
config to C2-NAV.14. `enclosure_entry` **SUCCEEDED** (64.41 s) — did NOT
reproduce the SW-corner deadlock. 23 `/plan` snapshots captured. The
global plan never enters the SW-side column at any point in any
snapshot; it does dip to 203.7 mm from `box_obstacle_1`'s **NW** corner
at one tick (t=17.6 s) — 1.4 mm inside the robot's own 205.1 mm
circumscribed radius — then opens back to 255 mm by the next tick,
before the robot's own GT track (which stayed 412 mm+ from that corner
the whole leg, TRUE minimum clearance 302 mm to `wall_west`) ever got
there. `corridor_gate`-exit heading (-25.9°) was within 3° and 2 cm of
C2-NAV.14's own failing entry state, yet this run took the box's
opposite (east, then north-wall) side entirely, never approaching either
via-pose closely (HEADING_POSE 328 mm, WAYPOINT 587 mm nearest
approach, both pruned by `RemovePassedGoals` after their own closest
pass). Full write-up: `docs/RESULTS.md` C2-NAV.15.

**What remains unverified.** The SW-corner mechanism itself — this
session's one live run did not reproduce it, so whether the global plan
bends toward the SW corner on a run that DOES fail there is still NOT
PROVEN. Whether the observed "brief bad tick, self-corrected before the
robot arrives" pattern holds when the robot is already close to the
tight region (C2-NAV.12 r2 / C2-NAV.14's own geometry) when the bad tick
fires. Any failure rate (N=1 this session, C2-NAV.12's own 1/3 at this
configuration is the only rate context that exists).

### Exact next command

```bash
# C2-NAV.16 (not run this session): same exact config, same
# instrumentation -- nothing to change, C2-NAV.15 proved it works. Run
# 2-3 more fresh seven-leg tours, stopping the moment one reproduces the
# SW-corner deadlock, then apply docs/data/c2nav15_planwindow.py's own
# analysis (retarget its TAG constant, or pass the tag explicitly) to
# THAT run's plan-window capture. This directly answers what C2-NAV.15
# itself could not: whether the global plan bends toward the SW corner
# on a run that fails there, or whether even a failing run's plan stays
# safe while DWB diverges from it. Do NOT tune anything to try to induce
# the failure -- capture the same uncontrolled variance this chain has
# already measured (C2-NAV.12: 1/3), with the new instrumentation
# pointed at it.
cd ~/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis
bash .navbench/c2n14_run.sh "$(pwd)/docs/data/c2nav11_ntp_params.yaml" \
    c2n16_tour_r1 ALL 75 "enclosure_entry:-3.575,2.95" "enclosure_entry:200" \
    "enclosure_entry:-3.00,0.625;enclosure_entry:-3.40,1.35"
```

## 2026-09-04 — C2-NAV.16: the same replan tick sends GOOD wide and BAD into the pinch, from states 15 cm apart

**A targeted capture experiment, not a tuning session.** Byte-identical
configuration to C2-NAV.14/.15: `docs/data/c2nav11_ntp_params.yaml`
(sha256 `6f61e499…`, verified live against the file on disk before and
after the run), goal override `enclosure_entry:-3.575,2.95`, leg timeout
`enclosure_entry:200`, through-poses `HEADING_POSE (-3.00, 0.625)` then
`WAYPOINT (-3.40, 1.35)`, `CSF 65/5`, `inflation_radius 0.5`,
`BaseObstacle.scale 8.0`, `SimpleGoalChecker`, `PolygonStop.radius
0.25`/`min_points 4`. No Nav2 parameter, BT file, goal, waypoint, or
`RemovePassedGoals` setting was touched. Full record:
`docs/data/c2nav16_bench.json`, `docs/data/c2nav16_compare.py`,
`docs/images/c2nav16_compare.png`.

**What was built.** `docs/data/c2nav16_compare.py` — reuses every
`c2nav15_planwindow.py` function **by import**, passing the tag
explicitly (that module's functions were already built to accept one;
nothing in it needed to change). Adds three things no prior C2-NAV
script computed: `dwb_command_window()` (reads the existing C2-NAV.6
stop-probe CSV — not a new subscription — for `v_nav`/`w_nav`/collision-
monitor action in a window around a chosen tick), `replan_gaps()`
(inter-`/plan`-snapshot timing), and `first_divergence()` (the session's
central question, built from the other two plus the per-snapshot table).
Self-test reproduces C2-NAV.15's own eight committed facts plus this
session's own three new ones (BAD's frozen-pose distance to the
canonical deadlock pose, and both runs' `FIRST_BAD_PLAN` timestamps)
before anything new is trusted.

### Runs

**1 fresh seven-leg tour, `c2n16_tour_r1`. The first one attempted
reproduced the SW-corner deadlock.** Per the brief's own stop condition,
the live campaign stopped there — no second or third run. `open_space`
through `corridor_gate` (five legs) all SUCCEEDED, matching C2-NAV.15's
own clean prefix. `enclosure_entry` TIMEOUT at 198.59 s (cap 200 s),
`enclosure_exit` TIMEOUT immediately after with 0.0 m net displacement —
the same "a lost entry costs the exit" pattern as C2-NAV.8 and
C2-NAV.12 r2. `gazebo_models` test suite not touched by anything this
session added; `TELEMETRY OK` on the stop probe (3768 rows, 29 monitor
states).

**The failure is genuinely the SW-corner mechanism, not a different
pocket.** Frozen at `(-3.1973, 1.9006)`, **103.2 mm** from C2-NAV.8/.12's
own `DEADLOCK_POSE (-3.3001, 1.9095)` — well inside the west-column /
south-of-box test C2-NAV.13 built specifically to exclude the *other*
pocket (C2-NAV.12 r1's east-face "Start occupied" mode). SW-side
commitment (the GT track) at **t=17.00 s**, after the WAYPOINT
`RemovePassedGoals` tick (**t=9.01 s**) — same ordering as C2-NAV.12 r2
and C2-NAV.13's own finding.

**GOOD = the committed `c2n15_tour_r1` (SUCCEEDED, 64.41 s, final error
0.066 m). BAD = `c2n16_tour_r1` (TIMEOUT, 198.59 s, final error 1.115 m,
`dwb_illegal_frac` 0.4796 → 0.8106).**

### FIRST_DIVERGENCE

**Both runs replan on the identical schedule.** `HEADING_POSE`'s
`RemovePassedGoals` tick fires at **t=6.006 s** in both; `WAYPOINT`'s at
**t=9.009 s** in both — confirmed directly from each run's own raw trace,
not assumed. Neither via-pose is ever genuinely reached in either run
(GOOD nearest 0.5867 m, BAD nearest 0.5728 m, both `> goal_xy_tolerance`
0.25 m) — the C2-NAV.13 finding that premature pruning is the norm, not
the exception, reproduces a third time.

**The two runs are close but not identical going into the tick.** Robot
GT pose at t=9.009 s: GOOD `(-2.5604, 1.6450)` at 56.10°, BAD
`(-2.6766, 1.5540)` at 46.12° — a **147.6 mm** / 9.98° pose delta, having
started from the same spawn and the same tour. The `/plan` snapshot
immediately preceding the tick is still statically similar in both
(GOOD t=5.924 s: 65 poses/3.469 m; BAD t=6.150 s: 82 poses/4.407 m — both
threading the wide, safe route, `min_clearance` 0.2924 m, no SW-column
entry in either).

**The very next captured `/plan` — the first one issued after the tick —
is where the routes split.** GOOD (t=9.028 s): 51 poses, 2.566 m,
`min_clearance` **unchanged at 0.2924 m**, never the SW column. BAD
(t=9.19 s): 48 poses, 2.428 m, `min_clearance` drops to **0.2306 m**,
**inside `PolygonStop`, inside the SW column** — this is `FIRST_BAD_PLAN`
**and** `FIRST_STOP_PLAN` simultaneously (the same snapshot satisfies
both conditions; no earlier snapshot satisfies either separately).

**A physical correlate, not just a geometric one.** In the ±1.5 s window
around the tick, GOOD's commanded `v_nav` never drops to 0 (steady
0.28–0.30 m/s throughout, `monitor_action` constant `DO_NOTHING`). BAD's
`v_nav` is pinned at **0.0 for ~1.5 s** while `w_nav` is commanded at
**0.64–1.0 rad/s** (near `max_vel_theta`) and `monitor_action` toggles
`DO_NOTHING`/`LIMIT` — the robot rotating in place, aligning toward the
new (bad) plan's heading, while its position is frozen at
`(-2.6766, 1.554)`. This is the same "heading swings ~60° near the
removal tick" signature C2-NAV.13/.14 found from GT yaw alone; this
session is the first to show a `/plan`-geometry correlate for it.

**`FIRST_DIVERGENCE = the first post-tick `/plan` message (t0+9.03 s
GOOD, t0+9.19 s BAD), not the tick itself, not a later replan, and not a
later robot-track commitment.** The tick fires identically in both runs;
what differs is the *content* SmacPlanner2D returns once the via-poses
are pruned from `{goals}` and it plans directly to the shifted goal from
whatever state the robot happens to be in at that instant. A 148 mm /
10° difference in that instant — itself accumulated from ordinary
run-to-run variance over the preceding ~9 s, not traced to any single
cause this session — is enough to flip which side of `box_obstacle_1`
the global plan takes.

### Classification (reusing C2-NAV.15's own CASE A/B test, unmodified)

* **BAD: CASE A.** The global plan itself enters the SW column /
  `PolygonStop` region at t=9.19 s, **7.81 s before** the robot's GT
  track physically commits there (t=17.00 s). The planner chooses the
  dangerous route; DWB (and, after the freeze, the collision monitor)
  only execute and then arrest it.
* **GOOD: CASE A (partial).** The global plan *also* briefly enters the
  `PolygonStop` band once, later (t=14.7 s, the NW-corner dip C2-NAV.15
  already reported at 203.7 mm, here reproduced as 0.2445 m at a
  slightly different tick), but the robot's GT track never committed to
  the SW column in this run — the leg reached the goal before the robot
  physically arrived at that part of the plan.
* Both classifications came from the same unmodified `classify()`
  function; the difference between "partial" and full CASE A is exactly
  the timing relationship the function was built to report, not a
  qualitative change in the test.

### Replanning timing (brief §10)

**No timing difference exists before the divergence.** Both runs replan
on the same ~2.8–3.1 s cadence (period 3.003 s) with **zero gaps** up to
and including the tick. **BAD develops four gaps of 12.4–17.7 s each**
starting at t=55 s — well after the freeze (t≈17.85 s) — coinciding with
repeated `Failed to make progress` errors in the captured Nav2 console
log (BAD: 18 occurrences; GOOD: 6, all during ordinary transit, none
during a stall). This is a **consequence** of the deadlock — recovery-
behaviour cycles interrupting the `RateController`'s normal cadence —
not a candidate cause of it, since it postdates `FIRST_DIVERGENCE` by
46 s.

### RemovePassedGoals correlation (brief §11)

**Removal precedes the bad route, in this run, unambiguously.** The
WAYPOINT removal tick (t=9.01 s) precedes `FIRST_BAD_PLAN` (t=9.19 s) by
0.18 s — the very next snapshot — and precedes the robot's own SW-column
commitment (t=17.00 s) by 7.99 s. This is consistent with, and sharpens,
C2-NAV.13's own reading that removal and heading/state interact rather
than either being solely dominant: removal is necessary (it is what
frees the planner to choose a route that no longer has to pass near the
WAYPOINT at all) but the specific route chosen once freed depends on the
robot's state at that exact instant, which is where GOOD and BAD
actually part ways.

### DWB / cost-field cross-check (brief §12–13): partially answered

**What was captured this session**: commanded `v_nav`/`w_nav` and the
collision-monitor's own gating state, from the existing C2-NAV.6 stop
probe (no new subscription). **What was NOT captured**: per-trajectory
DWB critic scores (`/evaluation`) and grid-quantized costmap cost along
either candidate route — neither the C2-NAV.11–15 chain's `nav_bench.py`
nor this session's own tooling subscribes to `/evaluation` during a
`NavigateThroughPoses` leg, and capturing the costmap itself would be a
new instrument, which the brief's §5 and this repo's rule 7 both counsel
against adding without being asked. The whole-world **exact geometry**
clearance (`nearest_full`, not the grid) is the only cost-adjacent signal
this session has: BAD's post-tick plan clearance (0.2306–0.1901 m) sits
well inside CSF 65's zero-cost floor of 0.291 m (C2-NAV.4/.9), so it is
**INFERRED, not measured this session**, that the BAD route carries
nonzero `BaseObstacle` cost along much of its length where GOOD's does
not — consistent with, but not new confirmation of, C2-NAV.9's "cost
field is blind above 291 mm, indifferent within it" diagnosis.

### Safety

No `PolygonStop` knob touched. The robot spent 3 stop episodes on this
leg (n_stops=3 in the leg record) settling exactly where prior sessions
measured real geometry — `box_obstacle_1`'s SW corner, 4–10 mm
penetrations of the 0.25 m circle, always ≥ 40 mm above the 0.2051 m
circumscribed radius. No new proximity floor established or approached.

### Visualization

`docs/images/c2nav16_compare.png` — the C2-NAV.9 clearance field with
both runs' complete `/plan` snapshot sets (thin green/red), both GT
tracks (bold green/red), both runs' `FIRST_BAD_PLAN` polylines
highlighted, and BAD's frozen pose marked. The two plan families overlay
almost exactly until the WAYPOINT, then visibly split: GOOD's bold trace
arcs east and north; BAD's cuts straight down the SW pinch and the GT
track follows it into the freeze.

### Root-cause classification

**COMBINATION — global planner route selection (CASE A, directly
measured) triggered by `RemovePassedGoals`/BT pruning (necessary
precondition, directly measured) acting on a small, unexplained
run-to-run state difference (148 mm / 10° at the tick, measured but not
traced to a cause).** Not DWB alone (BAD's plan itself is unsafe before
DWB ever executes it); not `RemovePassedGoals` alone (it fires
identically, at the identical tick, in the run that succeeds); not
"global planner" in isolation either, since an identical planner
invocation from GOOD's own tick-time state produces the safe route.

### OBSERVED / INFERRED / NOT PROVEN

**OBSERVED**: `FIRST_DIVERGENCE` at the first post-removal-tick `/plan`
(148 mm / 10° apart state, opposite route outcome); BAD is CASE A
(plan-bad-before-robot-committed, 7.81 s lead); GOOD is CASE A (partial)
(plan briefly unsafe, robot never got there); removal precedes
`FIRST_BAD_PLAN` by 0.18 s in BAD; zero replan-timing difference before
the divergence, four gaps after it coincident with recovery-behaviour
errors; BAD's DWB commands a 1.5 s in-place-rotation stall bracketing the
tick, GOOD commands none.

**INFERRED**: the BAD route likely carries nonzero `BaseObstacle` cost
where GOOD's does not, from exact-geometry clearance against the known
CSF 65 zero-cost floor — no live costmap capture this session.

**NOT PROVEN**: why the two runs' states differ by 148 mm / 10° at the
tick despite an apparently identical tour up to that point (Gazebo
physics/DDS timing jitter vs. DWB sampling stochasticity vs. something
else — not instrumented this session); whether SmacPlanner2D's own cost
function treats the SW-pinch route and the wide route as a near-tie
(would explain why small state deltas flip the choice) or a clear
preference that BAD's replan start pose happens to satisfy — no
per-candidate-path cost was computed; any rate for this specific
mechanism (N=1 clean SW-corner failure, N=1 GOOD, matching this chain's
own standard for single-occurrence findings since C2-NAV.8); whether the
mechanism replicates on a second fresh SW-corner capture — deliberately
not pursued this session per the brief's own stop condition.

**Tests.** No `coco_mission`/`coco_rl`/etc. suite touched. `nav_bench.py`
unmodified since C2-NAV.15 (byte-identical, confirmed by `git diff`
before this session's first run). `gazebo_models/config/nav2_params.yaml`
untouched. `main` untouched (still `ea66155`). One fresh Gazebo + Nav2
simulator run this session (`c2n16_tour_r1`), `ros_clean.sh` before it,
`TELEMETRY OK` after.

### Exact next command

```bash
# C2-NAV.17 (not run this session): the NOT PROVEN that matters most is
# whether SmacPlanner2D treats the SW-pinch route and the wide route as
# a genuine near-tie at CSF 65 -- which would explain why a 148 mm / 10
# degree difference in robot state at the removal tick is enough to flip
# the outcome, without needing any bug in the planner or the BT. Capture
# the GLOBAL costmap (not just exact whole-world geometry) at or just
# before the WAYPOINT removal tick, on both a GOOD-state and a BAD-state
# replan (staged offline: call SmacPlanner2D's own compute-path service,
# or the /global_costmap, directly from each run's own recorded tick-time
# pose, rather than trying to reproduce the live nondeterminism), and
# compare the two candidate paths' summed cost. Do NOT tune CSF,
# inflation, BaseObstacle, PolygonStop, RemovePassedGoals, the waypoint,
# or the goal to try to induce or prevent either outcome.
cd ~/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis
python3 -P docs/data/c2nav16_compare.py all   # start here: the full record already collected
```

## 2026-09-04 — C2-NAV.17: the near-tie hypothesis is REJECTED — a static reconstruction never finds BAD's route at all

**Offline only, as scoped. Zero simulator runs.** The brief's own question:
does SmacPlanner2D treat the SW-pinch route and the wide route as a
genuine near-tie at CSF 5 (global), so that the 147.6 mm / 9.98° state
difference C2-NAV.16 measured at the WAYPOINT removal tick is *itself*
sufficient to flip the outcome? Built `docs/data/c2nav17_routeselect.py`
to answer it by reconstruction rather than assertion: the actual Nav2
global-costmap cost field (not just clearance) from
`docs/data/c2nav11_ntp_params.yaml` — resolution 0.05 m, `robot_radius`
0.20 m, `inflation_radius` 0.5 m, global `cost_scaling_factor` 5.0 — over
`c2nav9_corridor.py`'s own verified box/cylinder geometry, plus an
8-connected grid A* built to match `nav2_smac_planner::Node2D` as closely
as this environment allows it to be checked.

### Section 4: exact replay is NOT possible — stated, not assumed

No prior C2-NAV session captured the per-cell global costmap at any
tick; only `/plan` output polylines (endpoints/summary stats, not full
points, in the committed JSON), GT poses, and a laser-range stop-probe
CSV are committed. **Exact replay of the real GOOD/BAD costmap content is
therefore not possible from committed artifacts.** This session builds
the brief's own fallback instead (§5): a principled *reconstruction* from
canonical world geometry and the documented Nav2 InflationLayer formula,
clearly separated throughout from anything source-verified (see next
section) — never presented as the real costmap.

### SmacPlanner2D source findings (§7): what could and could not be checked

`nav2_smac_planner` 1.3.11 ships headers only at
`/opt/ros/jazzy/include/nav2_smac_planner/*.hpp`; no `.cpp`, no
`deb-src` entry, and this environment has no outbound network access
(`curl` to raw.githubusercontent.com timed out — checked once, not
retried). Two facts came out **OBSERVED, directly from the header
files, not from memory or documentation**:

1. **`Node2D` has no heading state, at all.** `node_2d.hpp`'s
   `getCoords(index, width, angles)` throws `"Node type Node2D does not
   have a valid angle quantization"` unless `angles == 1`. This is not a
   tuning default — the 2D planner's search graph is a plain (x, y)
   grid. Consequence for §9 (heading sensitivity): the global route
   **cannot** depend on start yaw, structurally, for any costmap. The
   9.98° yaw difference C2-NAV.16 measured is provably not a candidate
   mechanism, independent of anything else this session found — and this
   is also why C2-NAV.14's heading-correcting through-pose was never
   going to fix the global-planner side of this, consistent with what
   C2-NAV.14 already measured.
2. **A* ties break by heap/container order, not by any secondary key.**
   `a_star.hpp`'s `NodeComparator` is `return a.first > b.first;` — a
   bare min-heap on total cost, nothing else. A genuine cost tie between
   two routes resolves however expansion order happens to land, which
   *is* sensitive to start cell — the one concrete mechanism by which a
   real near-tie could plausibly flip with a small start-pose change.

Everything else used below (the InflationLayer formula, `getTraversalCost`'s
general shape, the 8-connected default, `getHeuristicCost` as Euclidean
distance) is **DOCUMENTED, not verified against local source** — flagged
at every point it matters in the module docstring, and the normalising
denominator (252 vs 253) is disclosed as unconfirmed and used at 252.

### Self-test

Reproduces `c2nav9_corridor`'s 326.0 mm bottleneck, `box_obstacle_1`'s SW
corner, C2-NAV.16's own 147.6 mm / 9.98° state delta, a cross-check of
this session's reconstructed distance field against `nearest_full` at
`DEADLOCK_POSE` (0.2550 m vs 0.2457 m, within one grid cell), and that
`GOAL_SHIFTED` itself sits inside the inflation gradient but not blocked
(cost 120, `0 < 120 < 253`). **ALL PASS.**

### The central result: REJECTED, not confirmed

Running the reconstructed A* from the two REAL captured states
(`GOOD_START (-2.5604, 1.645)`, `BAD_START (-2.6766, 1.554)`, both to
`GOAL_SHIFTED`) does **not** reproduce the real split:

| | length | integrated cost | min clearance | enters PolygonStop |
|---|---|---|---|---|
| GOOD_START (synthetic) | 2.195 m | 3.651 | 0.300 m | **False** |
| BAD_START (synthetic) | 2.045 m | 3.501 | 0.300 m | **False** |
| GOOD (real, committed) | 2.566 m | — | 0.2924 m | False |
| BAD (real, committed) | 2.428 m | — | **0.2306 m** | **True** |

Both synthetic optima thread the exact same NW-pinch trunk corridor at a
300 mm offset from `box_obstacle_1`'s west face — the safe route,
matching GOOD's real 292.4 mm clearance closely. **Neither reproduces
BAD's real 230.6 mm intrusion inside `PolygonStop`.** Visual confirmation
in `docs/images/c2nav17_routeselect.png`: the two synthetic routes are
the same path within a few centimetres of their very first step.

**This is not a tuning artefact of the goal snap.** Real captured plans'
`last_pose` (`-3.519, 2.94`) sits ~60 mm from the nominal
`GOAL_SHIFTED (-3.575, 2.95)` — unresolved (`NOT PROVEN`, plausibly path
downsampling in the real planner's output, not investigated further).
Re-running both starts against the *observed* endpoint instead of the
nominal goal changes nothing: still 300 mm clearance, still never inside
`PolygonStop`, gap still 13.6 %/23.8 % (below).

**The cost gap is real, not marginal.** No committed real capture gives
a full SAFE-route polyline to cost directly (only summary stats), so the
SAFE route is priced by re-running A* with the region the natural
southward approach leg passes through *excluded* — the best available
detour — and compared to the unconstrained optimum:

| start | unconstrained cost | detour-forced cost | gap |
|---|---|---|---|
| GOOD_START | 3.6505 | 4.2753 | **+17.12 %** |
| BAD_START | 3.5005 | 4.4667 | **+27.60 %** |

17–28 % is not a near-tie under this reconstruction's own objective.

**Position sensitivity (§8, §10): no boundary anywhere tested.** Swept
±150 mm / 20 mm steps around the GOOD/BAD midpoint (256 starts) and
along the GOOD_START→BAD_START line extended 50 % past each end (41
points, ≈±750 mm beyond the real states): **`min_clearance` is exactly
0.300 m and `enters_polygon_stop` is `False` at every single sampled
start.** No route-class boundary crossing the real danger threshold
(0.25 m) exists anywhere near the measured states, nor well beyond them.

### Analysis validation (§16) — a tool defect caught and corrected for

`sw_column`, reused verbatim from `c2nav15_planwindow.py`, is designed
for ground-truth tracks that already start inside the pinch region.
Applied to a full point-to-point global-planner polyline, it also fires
on the harmless southward *approach leg* both classes share (visible in
`docs/images/c2nav17_routeselect.png`'s right panel: the entire sweep
region reads "SW" even though `min_clearance` never drops below 300 mm
anywhere in it). `enters_polygon_stop`/`min_clearance` — the field that
actually determines the real danger — was used as the operative
discriminator throughout this write-up instead, once this was caught.
Recorded here per the brief's own instruction to validate tooling before
trusting it, not smoothed over.

### OBSERVED / INFERRED / NOT PROVEN

**OBSERVED**: `Node2D` has no heading state (`node_2d.hpp`); A* ties
break by heap order with no secondary key (`a_star.hpp`); the
reconstructed static costmap+A* finds the SAFE route, by a 17–28 %
margin, from both real GOOD_START and BAD_START, and from every point
tested in a ±150 mm neighbourhood and along an extended ±750 mm line
through both; `min_clearance`/`enters_polygon_stop` never once matches
BAD's real 230.6 mm/inside-`PolygonStop` reading anywhere this session
searched.

**INFERRED**: since this session's reconstruction is insensitive to
start pose everywhere tested, the real GOOD/BAD split most likely traces
to a difference in the two runs' *actual live global-costmap content* at
the tick (transient `obstacle_layer`/`voxel_layer` marks, incomplete
propagation right after a costmap-clearing/leg-transition event, or
similar) rather than to robot pose alone — a **refinement** of C2-NAV.16's
open question, not a restatement of it.

**NOT PROVEN**: the exact `Node2D::getTraversalCost`/`getHeuristicCost`
algebraic form and the 252-vs-253 normalising constant (no `.cpp`, no
network); whether a materially different `cost_travel_multiplier`
normalisation would close the 17–28 % gap (not swept — brief §13
forbids tuning the *real* planner, and sweeping the *reconstruction's*
own unverified constant would not itself be evidence about the real
planner); why the real two runs' states differed by 147.6 mm/9.98° in
the first place (still open from C2-NAV.16); the ~60 mm goal-snap
discrepancy between nominal `GOAL_SHIFTED` and captured `last_pose`;
whether the real live global costmap actually differs in content
between a GOOD-repeat and a BAD-repeat at the tick — this is now the
load-bearing untested claim.

### Verdict

**REJECTED**, for the hypothesis as literally framed (a start-pose
perturbation alone flips a genuine near-tie in a *static* reconstruction
of the global costmap). The reconstruction is not near-tied at either
real state or anywhere nearby — it prefers the safe route everywhere,
by a substantial and stable margin, and never once reproduces BAD's real
behaviour. This does not close the investigation; it sharpens it. The
static-geometry model cannot explain BAD's plan, which means something
outside a static model must — most plausibly the real costmap's live
content, not the robot's pose, at the tick.

### Exact next live experiment

```bash
# C2-NAV.18 (not run this session): C2-NAV.17 shows a STATIC
# reconstruction of the global costmap never reproduces BAD's real
# 230.6 mm PolygonStop intrusion, from any start pose near GOOD_START or
# BAD_START -- so pose alone, against a static costmap, is not the
# mechanism. The next test isolates costmap CONTENT instead of pose:
# capture the REAL /global_costmap/costmap (nav_msgs/OccupancyGrid, or
# the costmap_raw service) at the WAYPOINT removal tick on a fresh
# GOOD-repeat and a fresh BAD-repeat, and diff the two live costmaps
# directly against each other and against this session's static
# reconstruction -- not just their resulting /plan outputs, which is all
# every prior session captured. If the two live costmaps are
# byte-identical in the SW-pinch region, this session's static model can
# be trusted and the mechanism is genuinely elsewhere (A* tie-breaking
# order, per the a_star.hpp finding above, is the next thing to suspect).
# If they differ, that difference IS the mechanism.
# Do NOT tune CSF, inflation, BaseObstacle, PolygonStop,
# RemovePassedGoals, the waypoint, cost_travel_multiplier, or the goal.
cd ~/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis
python3 -P docs/data/c2nav17_routeselect.py all   # start here: the full record already built
```

## 2026-09-04 — C2-NAV.18: live global-costmap capture built and validated on 3 real runs; GOOD-vs-BAD diff INCONCLUSIVE (0/3 BAD this session); SW-column framing revised

**A capture-and-diff experiment, not a tuning session.** Full write-up:
`docs/RESULTS.md` C2-NAV.18. Summary here.

**Built**: `gazebo_models/scripts/nav_bench.py` gained a
`/global_costmap/costmap` subscription (`_global_cb`) mirrored exactly
on the existing `/local_costmap/costmap` ring pattern, a
`costmap_window` (same `ts >= t0` filter `plan_window` already applies)
returned as a sixth element of `send_multi_leg`'s tuple, and a
`.npz` + `_meta.json` writer in `main()` (compressed stacked int8 grids;
a JSON array of the same data would run to hundreds of MB across a
200 s leg at this cadence). `docs/data/c2nav18_livecostmap.py` is the
offline analysis: `load_costmap_window` (validated, invalid-run
handling), `alignment` (T_PRUNE/FIRST_BAD_PLAN/nearest-snapshot, per run,
never assumed shared), `route_cost`/`diff_grids`/`region_diff`,
`onset_test` (brief's own temporal-ordering test with a MEASURED
significance threshold, not assumed), and — added once this session's
own runs turned out all-GOOD — `replicate_noise_floor`/
`visualize_replicates`. Self-test reproduces every `c2nav15_planwindow`
fact plus C2-NAV.17's own reconstruction-gap numbers. No Nav2 parameter,
BT, goal, waypoint, or `RemovePassedGoals` setting touched.

**Runs**: byte-identical config to C2-NAV.14/.15/.16/.17
(`c2nav11_ntp_params.yaml`, sha256 `6f61e49912765708e70470df967b23834338723176bcf7ae113f8b8c1e6bb950`).
Three fresh tours (`c2n18_tour_r1/r2/r3`), the brief's own 3-tour cap —
**all three SUCCEEDED** (enclosure_entry 64.93 s / 70.05 s / 103.76 s;
every leg of every tour, `TELEMETRY OK` all three). 0/3 BAD
reproductions this session, vs. C2-NAV.12's own measured 1/3 rate (not
statistically inconsistent at this N, but means the primary GOOD-vs-BAD
diff has nothing to compare against this session).

**Instrumentation validated**: 168/179/268 costmap snapshots captured
(r1/r2/r3), 0 shape mismatches, 0/615 total geometry drift, grid
243×175 @ 0.05 m identical across all three independently-launched
simulators. Measured mean publish interval ≈0.38 s (≈2.6 Hz) — well
below the configured 5.0 Hz `update_frequency`/`publish_frequency`, a
genuinely new measured fact (brief section 5 explicitly forbids
assuming this).

**The SW-corner mechanism is reframed, not confirmed or rejected.**
Applying the unmodified `pw.first_bad_plan`/`divergence_timing` test —
the same test that found C2-NAV.15's single GOOD run NEVER entered the
SW-side column — to these three new GOOD runs: **all three DID enter
it (both in `/plan` and GT track) and all three still SUCCEEDED.**
C2-NAV.15's "GOOD never enters the SW column" does not replicate at
N=3; it looks like the N=1 outlier, not the GOOD-run norm. The
discriminator between GOOD and BAD is therefore better framed as
*recovery from SW-column entry*, not entry itself. Also: **T_PRUNE is
NOT fixed by config alone**, contrary to C2-NAV.16's own framing — r3's
WAYPOINT removal tick fires at t=24.024 s (2.7× later than r1/r2's
9.009 s) because `RemovePassedGoals` only fires once the robot's own
trajectory first comes within 0.7 m of the WAYPOINT, itself downstream
of ordinary approach-leg variance.

**GOOD-vs-GOOD live-costmap noise floor measured** (the question this
session COULD answer, `replicate_noise_floor`): at each run's own
T_PRUNE-nearest snapshot, pairwise diffs of whole-grid 7.1–8.5%
(3013–3635 of 42525 cells), SW-corner-region 12.8–40.2% (58–182 of 453
cells), max\|Δ\|=100 (full cost-range span) in every pair — substantial,
measured baseline variation between two runs that BOTH succeeded. Any
future GOOD-vs-BAD diff needs to clear numbers at least this large in
the same region to be called diagnostic of a route difference rather
than ordinary `obstacle_layer`/`voxel_layer` observation-timing noise.

**Visualization**: `docs/images/c2nav18_replicates.png` — 3-panel, each
run's own live costmap + tick-nearest `/plan` + GT track, visibly
grazing the box's SW/west face before recovering north to the goal.

### Root-cause classification

INCONCLUSIVE for the primary hypothesis (LIVE COSTMAP CONTENT vs.
COSTMAP UPDATE TIMING vs. PLANNER STATE vs. PLANNER EXECUTION vs.
REMOVE-PASSED-GOALS/BT vs. OTHER) — no BAD capture exists this session
to classify against. The capture/analysis pipeline itself is CONFIRMED
working end to end on three independent live runs.

### OBSERVED / INFERRED / NOT PROVEN

OBSERVED: instrumentation correctness (0 shape mismatches/geometry
drift across 615 snapshots); measured cadence ≈2.6 Hz vs. configured
5.0 Hz; all 3 GOOD runs entered the SW column and recovered; T_PRUNE
varies 9.009–24.024 s across byte-identical config; GOOD-vs-GOOD
costmap diffs are nonzero and substantial (7–8.5% whole-grid, up to 40%
SW-region). INFERRED: C2-NAV.15's "GOOD avoids the SW column" was a
small-sample artefact, not a general property; grazing-and-recovering
is a real third pattern distinct from C2-NAV.15's avoid-entirely and
C2-NAV.16's permanent-freeze. NOT PROVEN: whether a live-costmap diff
specifically attributable to route OUTCOME (not ordinary noise) exists
— no BAD to compare against; what determines recovery vs. permanent
freeze after SW-column entry; whether T_PRUNE variability correlates
with outcome.

### Verdict

INCONCLUSIVE for the brief's central GOOD-vs-BAD question (no BAD
captured). Instrumentation CONFIRMED working. SW-corner mechanism
PARTIALLY REFRAMED: entry is common even in successful runs; recovery
is the open question.

**Tests.** No `coco_mission`/`coco_rl`/etc. suite touched.
`gazebo_models/config/nav2_params.yaml` untouched. `main` untouched.
Three fresh Gazebo + Nav2 simulator runs this session (`c2n18_tour_r1`,
`r2`, `r3`), `ros_clean.sh` before each (one stray orphaned instance
from a prior session was found running before this session started
anything and killed by process name, same pattern C2-NAV.15 already
documented), `TELEMETRY OK` after each.

### Exact next command

```bash
# C2-NAV.19 (not run this session): C2-NAV.18 built and validated the
# live /global_costmap/costmap capture + diff pipeline on 3 real runs,
# but all 3 SUCCEEDED (0/3 BAD) -- the primary GOOD-vs-BAD diff has
# nothing to compare against yet. Re-run this exact configuration and
# instrumentation (nothing to change) until a BAD (SW-corner
# deadlock/TIMEOUT) reproduces, then immediately run the already-built,
# already-self-tested analysis against the real tag pair -- zero new
# code needed.
cd ~/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis
bash .navbench/c2n14_run.sh "$(pwd)/docs/data/c2nav11_ntp_params.yaml" \
    c2n19_tour_r1 ALL 75 "enclosure_entry:-3.575,2.95" "enclosure_entry:200" \
    "enclosure_entry:-3.00,0.625;enclosure_entry:-3.40,1.35"
# Once a BAD tag exists, set GOOD/BAD at the top of
# docs/data/c2nav18_livecostmap.py and:
python3 -P docs/data/c2nav18_livecostmap.py all
python3 -P docs/data/c2nav18_livecostmap.py viz
python3 -P docs/data/c2nav18_livecostmap.py dump docs/data/c2nav18_bench.json
```

## 2026-09-05 — C2-NAV.19: the live costmap is not the variable, and the split is a 12.3 mm lidar margin

**A capture-and-compare experiment, not a tuning session.** C2-NAV.18
left exactly one thing outstanding: it had built and validated the
`/global_costmap/costmap` capture pipeline, but all three of its fresh
tours SUCCEEDED, so the GOOD-vs-BAD diff at the route-selection event
was INCONCLUSIVE. This session's brief was to obtain **one** valid BAD
run under the identical frozen configuration and run the comparison.

**The first fresh tour reproduced the deadlock.** Per the brief's stop
condition the live campaign ended there — no second or third tour. Zero
new instrumentation was needed: C2-NAV.18's pipeline worked unchanged on
the new capture, exactly as it predicted.

### Configuration (unchanged, verified twice)

`docs/data/c2nav11_ntp_params.yaml`, sha256 `6f61e499…`, checked on disk
before the run and against the live stack afterwards
(`c2n19_tour_r1_params_live.txt`). Local CSF 65 / global CSF 5,
`inflation_radius` 0.5, `BaseObstacle.scale` 8.0, `SimpleGoalChecker`,
`PolygonStop` 0.25 / `min_points` 4, C2-NAV.11 BT, `HEADING_POSE` →
`WAYPOINT` → `GOAL_SHIFTED` as one `NavigateThroughPoses` request via
`.navbench/c2n14_run.sh` (new tag only). Nothing was tuned.

### The run

`c2n19_tour_r1`: five prefix legs SUCCEEDED, `enclosure_entry` TIMEOUT at
201.36 s with **130.99 s of `PolygonStop`**, frozen at (−3.2214, 1.9018),
goal error 1.106 m; `enclosure_exit` then TIMED OUT with 0.0 m net
displacement. `TELEMETRY OK`, 3959 probe rows, **520** costmap snapshots.

The bench exited **139 (SIGSEGV) at teardown**. Every artifact was
already on disk — all seven legs in the bench JSON, the full costmap
window, the `.done` marker — and that was verified explicitly before any
analysis was trusted. A crash at process exit is not data loss, but it
should not be assumed to be harmless either; it was checked.

Frozen **79.1 mm** from `DEADLOCK_POSE` (C2-NAV.16's own BAD: 103.2 mm),
passing C2-NAV.13's west-column **and** south-of-box tests. Genuinely the
SW-corner mechanism.

### What was built

`docs/data/c2nav19_goodbad.py` — reuses `c2nav18_livecostmap` (the whole
costmap pipeline), `c2nav16_compare`, `c2nav15_planwindow`,
`c2nav13_heading` and `c2nav12_report` **by import**. It rebinds only
`lc.GOOD`/`lc.BAD`; C2-NAV.18's own file is left byte-unchanged so its
committed defaults still document C2-NAV.18. New here, because no prior
module computes it: `leg_rows` (stop-probe rows over the whole leg, not
just C2-NAV.16's ±1.5 s window), `approach_profile`, `latch_profile`,
`phase_windows`, `rpg_compare`. Self-test reproduces every C2-NAV.18
fact plus four of this session's own before anything new is believed.

### Findings

**1. The live costmap hypothesis is REJECTED.** At the matched tick the
GOOD-vs-BAD SW-corner diff is **183/453 cells (40.4 %)** against a
GOOD-vs-GOOD noise floor C2-NAV.18 had already measured at **40.0 % and
40.2 %** — i.e. it exceeds the worst GOOD-vs-GOOD pair by **one cell in
453**. Whole-grid: 9.4 % against a 7.1–8.5 % floor, 355 cells above.
Stated that exactly rather than rounded in the conclusion's favour: on a
strict `>` test both margins are positive, and neither is diagnostic —
a 1-in-453 excess in the decision-relevant region is noise by any
reading, and SW-corner `max|Δ|` is actually *lower* for GOOD-vs-BAD (39)
than for either GOOD-vs-GOOD pair involving r3 (63, 60). The
properly-thresholded test is the onset test, and it never fires.

**2. And it never becomes significant, anywhere in the leg.** The onset
test walks all **520** BAD snapshots against GOOD's nearest-in-time. Noise
floor 159 cells (measured from the first matched pair, while both runs
are still on the identical prefix); threshold 3× that = 477. Maximum
SW-corner difference over the entire 201 s leg: **216 cells**. Samples at
or above threshold: **0 of 520**. There is no costmap event to order
against the plan divergence — event A simply never happens.

**3. Route ordering does not reverse.** BAD's SW route is the cheaper of
the two under **both** live maps — 17.1 % cheaper under GOOD's own map,
34.8 % under BAD's. And GOOD's own post-tick plan *also* enters the SW
column (its `FIRST_BAD_PLAN` is at 6.072 s, before its own tick). Both
runs planned into the pinch; one deadlocked. C2-NAV.18's reframing is now
confirmed on a real GOOD/BAD pair rather than on three GOOD runs.

**4. The discriminator is 12.3 mm, and it is not in the costmap.**
`PolygonStop` fires on live `/scan` points inside a 0.25 m polygon; it
never reads the costmap. Closest lidar-to-base: **GOOD 0.2604 m
(+10.4 mm), BAD 0.2481 m (−1.9 mm)**. GOOD had **0 of 649** rows below
260 mm and never once put a single point in the stop polygon; BAD had
1314 of 2013 and reached 18 points.

**5. The stop is self-sustaining and blocks its own recovery.** After
BAD's latch at t = 70.266 s, across 1311 rows: the monitor released on
**0**, the wheels moved on **0**, `d_min` stayed inside a **0.3 mm band**
for 131 s, and points in the polygon never fell below 17. Meanwhile DWB
and the behaviour server kept commanding — `v_nav` from −0.15 to +0.2842
including **201 rows of commanded reverse** — and the nav log shows
`backup failed` ×2 and `spin failed` ×1. Wheels gated → geometry frozen →
points never leave → gate never lifts. In this benchmark's topology the
gating demonstrably *does* reach the wheels (`v_nav = −0.15` with
`v_out = v_wheel = 0.0` for 1311 consecutive rows), which is the opposite
of the `/cmd_vel_nav` loop limitation recorded for the C2-M5.0 mission
topology — measured here only for this topology, not a general claim.

**6. The BAD leg is three mechanisms, and only the third is the corner.**
It loses **42.84 s standing still** at (−2.681, 1.692) — the bench's own
`worst_crawl` — with `dwb_chosen_vx = 0.0`, **94 % of 819 trajectories
legal**, `scan_min = 0.456 m`, a fresh valid plan (age 0.67 s) and the
monitor only at `SLOWDOWN`. GOOD's worst crawl is 1.59 s. In the
*identical* window GOOD is **closer** to obstacles (0.260–0.398 m vs
0.458–0.482 m) and moving 82.0 % of the time, while BAD keeps half a
metre of clearance and is frozen 92.5 % of the time. Proximity is not
what stalls it. That is C2-NAV.3's zero-velocity DWB scoring pathology,
and it is what burns the budget and leaves the robot drifting into the
pocket 60 s later.

**7. RemovePassedGoals (read only, unchanged).** The tick fires at
**9.009 s in both runs**. GOOD is pruned 0.2245 m from the waypoint
(genuinely reached, inside the 0.25 m tolerance); BAD at **0.7177 m** —
essentially at the 0.7 m `RPG_RADIUS` boundary, having never come within
0.4487 m of it, with **3** threshold crossings rather than 1. A 566.1 mm
/ 71.33° state delta, far larger than C2-NAV.16's 147.6 mm / 9.98°. A
strong correlate; on N = 1 not established as the cause.

### A trap paid for again

`latch.py` in the shared scratch directory imported a sibling module,
which put that directory on `sys.path`, where a **stray `numbers.py`**
from an old session shadowed the stdlib and broke numpy inside the
analysis — *and* printed a completely different run's output into this
session's log. Both halves of the trap CLAUDE.md already documents. The
other scripts were unaffected because they run under `python3 -P` and
never add that directory. Fix: the analysis module is self-contained and
`python3 -P` is used everywhere.

### Verdict

**REJECTED** for the brief's central hypothesis. The live global costmap
is not the missing variable behind the GOOD/BAD split: a real BAD map was
captured and diffed against a real GOOD one at the exact
route-selection/recovery event, and the two are equivalent to within the
pre-existing measured noise floor while the outcomes differ completely.
The chain's question moves off the costmap and onto DWB's zero-velocity
scoring and `PolygonStop`'s latch.

### Housekeeping

Simulator stopped, ROS graph verified empty (`ros2 node list` returns
nothing; only the stateless CLI daemon respawns). No monitor, no
background process, no `/loop`, no scheduled wakeup, and none was created
by this task. `.navbench/` remains the untracked scratch directory, per
the established convention.

### Exact next command

```bash
cd ~/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis
python3 -P docs/data/c2nav19_goodbad.py all
python3 -P docs/data/c2nav19_goodbad.py viz
python3 -P docs/data/c2nav19_goodbad.py dump docs/data/c2nav19_bench.json
```

Then **C2-NAV.20**: instrument the DWB critic scores across the whole
42.84 s crawl window (not just the single `worst_crawl` sample the bench
already records) and answer why a zero-velocity trajectory outscores
every forward one when 94 % of 819 trajectories are legal and clearance
is 0.456 m. Reuse `c2nav16_compare.dwb_command_window` and the existing
stop-probe CSV — no new subscription. Do **not** tune CSF, inflation,
`BaseObstacle`, `PolygonStop`, `RemovePassedGoals`, the waypoint, the
goal, or DWB.

## 2026-09-05 — C2-NAV.20: zero velocity does not win the stall, it ties — and the tie-break always favours standing still

**A pure offline diagnosis. No simulator ran, no ROS node started, no
parameter moved.** The brief's own preference (its section 16) was zero
simulator runs, and that is what this was: every number comes from the
committed C2-NAV.19 BAD run, the three C2-NAV.18 GOOD runs, and
C2-NAV.3's raw `/evaluation` captures. Full write-up: `docs/RESULTS.md`
C2-NAV.20.

### Configuration

`docs/data/c2nav11_ntp_params.yaml`, sha256 `6f61e499…`, re-hashed by the
self-test; identical to C2-NAV.19's. Local CSF 65 / global CSF 5,
`BaseObstacle.scale` 8.0, `SimpleGoalChecker`, `PolygonStop` 0.25 /
`min_points` 4, C2-NAV.11 BT, the same three-pose
`NavigateThroughPoses`. Nothing was tuned; nothing could be.

### What was built

`docs/data/c2nav20_dwbstall.py` — reimplements DWB's sample iterator,
trajectory generator, `transformGlobalPlan`, `adjustPlanResolution` and
the four MapGrid critics from the 1.3.11 source, and reuses
`c2nav15_planwindow`, `c2nav16_compare`, `c2nav12_report` and
`c2nav13_heading` by import. Self-test = **37 checks**, all passing.

**The validation gate is C2-NAV.3's raw captures**, which — unlike
C2-NAV.19's — contain DWB's own transformed plan, the local costmap *with
its origin*, and all 819 per-trajectory critic scores together with their
critic **counts**. Reproduced exactly: `GoalDist` **1289/1289**,
`PathDist` **1289/1289**, `GoalAlign`/`PathAlign` **1288/1289**, the
complete/short/illegal splits **151/648/20** and **278/541/0**, the seed
cell **(3, 26)**, and the evaluation order (`best_index` → the captured
chosen twist) on every snapshot.

### Findings

**1. It is a tie, not a win.** Over 145 distinct DWB states in the
window, the best forward trajectory whose `BaseObstacle` is *provably*
zero scores within **−0.6 … +1.4** of the best zero-vx trajectory —
median **exactly 0.0** — on totals of 46–51. A median of **3**
trajectories (max 16) share the minimum *exactly*, every total is a
multiple of 0.2 (the gcd of the 0.6/0.8 MapGrid scales), and DWB's
tie-break is a strict `<`, so ties go to the first trajectory evaluated.
`OneDVelocityIterator::reset` starts at `min_vel_` and θ iterates
innermost, so **the 40-sample vx = 0 block is always scored first, always
completes, and therefore sets the short-circuit threshold every one of
the 779 forward trajectories is judged against.**

**2. `BaseObstacle` is ruled out by a bound, not an assumption.** Local
inflation is `floor(252·exp(−65·(d−0.20)))`, so cost is 0 beyond
**0.2851 m**; `d_min_base` is **0.4576–0.4824 m** for the whole window;
therefore **288–334 of the 779 forward trajectories are provably in
cost-0 cells at every state**. The chosen trajectory's `BaseObstacle` leg
mean is **exactly 0.00** over 1157 cycles, and since the critic is
non-negative it was 0 on every one.

**3. This is NOT C2-NAV.3's stall, and that matters.** At CSF 5,
**83.2 % / 69.4 %** of forward trajectories short-circuited at
`BaseObstacle` before `GoalDist` was ever computed, and the commanded
speed decayed *smoothly* 0.300 → 0.016 over 4.3 s. At CSF 65 nothing is
gated and the collapse is **one cycle wide** — 0.3000 → 0.0000, total
39.8 → 47.6, at t = 10.20 s. **But run this session's test on C2-NAV.3's
own capture and the surviving complete forward trajectories also tie
exactly (36.2 vs 36.2, 33.8 vs 33.8).** The tie was always there.
C2-NAV.4/5 raised CSF 5 → 65 and removed the gating C2-NAV.3 diagnosed;
the robot still stalls because gating was masking a degenerate score
landscape the cost change never touched.

**4. There is a discrete event, and it is the plan.** Between t = 8.20
and t = 8.70 the transformed plan's in-window endpoint moves **1.13 m**
and the heading error to it steps **+4.2° → +59.0°** as the 1.5 m clip
reaches past the bend round the SW corner. `RemovePassedGoals` fired at
**9.009 s** (C2-NAV.19's own measurement, BAD pruned 0.7177 m from a
waypoint it never reached); the replan landed at **9.140 s**; DWB
collapsed at **10.20 s**. The robot then sat **71–85° off** its own plan
for the whole 42.84 s.

**5. The turn it needed was below the resolution of the critics that
choose it.** Required turn **+2.7 … +84.9°, always positive**. DWB
commanded **negative** wz on **74.6 %** of 429 rows, and **72.3 %** of
`|wz|` were within the three smallest non-zero samples. It travelled
**150.7° of yaw** to net **+72.9°**. The reason is structural: at vx = 0
the endpoint is the robot's own cell, so `GoalDist`/`PathDist` are
identical for all 40 rotations, and only `GoalAlign`/`PathAlign`
separate them — read **0.1 m = 2 cells** ahead. Total rotational score
span **≤ 5.6** against a 46–51 total, quantised to whole cells.

**6. GOOD vs BAD has three clean discriminators, none of them
proximity.** BAD's chosen `GoalAlign`/`GoalDist` are **16.08/15.97**
(raw ≈ 27 cells) against GOOD's **4.45–6.55** (7–11 cells); BAD's
`BaseObstacle` mean is **0.00** where every GOOD run is non-zero
(1.37 / 3.16 / 11.32); `RotateToGoal` throws **90 000+** illegals in each
GOOD run and **779** in BAD, because GOOD reaches the goal region and BAD
never arrives. And BAD keeps **0.371 m** minimum scan range against
GOOD's **0.177–0.197 m** — **GOOD gets twice as close and recovers.**

**7. A refinement to C2-NAV.19's own phrasing.** The 42.84 s is a
*commanded*-crawl window (`|v_nav| < 0.05`); DWB's *selected* vx was
exactly 0.0 for **28.60 s of it (66.8 %)**, in two runs of 11.60 s and
28.60 s. In between it selected 0.0158 and 0.0316 m/s — the first two
forward samples, worth 24 and 47 mm over the horizon, **less than one
costmap cell**. Substance unchanged; the letter corrected.

### What this session could NOT do, stated plainly

The reconstruction reproduces C2-NAV.3 exactly but **does not reproduce
C2-NAV.19's published critic integers**: rebuilt `GoalDist` 41 vs
published 37, `GoalAlign` 44 vs 37, `PathDist` 1 vs 0. The seed sits 3
plan poses too far along, and **164 025** combinations of robot-pose
offset × costmap lattice phase reproduce none of them together — so it is
not a simple localisation offset and it was **left unexplained rather
than fitted away**. The C2-NAV.19 artifact has no transformed plan and no
costmap origin to pin it with.

Consequently only the zero-vs-forward *difference* is claimed, and
`seed_sensitivity()` walks the seed back 0–8 poses (more than double the
residual): the win/tie/loss split and the median margin are **bit-identical
at every backoff**. A constant seed error shifts every total equally and
cannot change which trajectories tie.

Also NOT PROVEN and not inferred: the complete/short-circuited split for
C2-NAV.19 — `nav_bench` does not record the critic count and it cannot be
recovered. The local costmap is likewise unrecoverable: `local_costmap`
runs obstacle + voxel + inflation with **no static layer**, so it is built
entirely from live `/scan`, which nothing captured.

### Verdict

**COMBINATION — TEMPORAL_STATE_CHANGE → SCORE_DOMINANCE.**
TRAJECTORY_VALIDITY ruled out (legal fraction 0.482–0.983, median 0.939).
CRITIC_GATING ruled out (288–334 provably ungated forward trajectories
per state). A discrete plan event puts the robot ~75° off its own path;
from there the MapGrid critics cannot tell moving from standing still,
and the tie-break does the rest.

### Housekeeping

No simulator was started, so none needed stopping; `ros2 node list`
returns nothing but the stateless CLI daemon. No monitor, no background
process, no `/loop`, no scheduled wakeup, and none was created by this
task. `.navbench/` remains the untracked scratch directory. Analysis ran
under `python3 -P` throughout, per the stray-`numbers.py` trap.

### Exact next command

```bash
cd ~/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis
python3 -P docs/data/c2nav20_dwbstall.py all
python3 -P docs/data/c2nav20_dwbstall.py viz
python3 -P docs/data/c2nav20_dwbstall.py dump docs/data/c2nav20_bench.json
```

Then **C2-NAV.21**, one parameter and one only: raise
**`GoalAlign.forward_point_distance` and `PathAlign.forward_point_distance`
from 0.1 to 0.325** — dwb's own default, which this params file overrode.
That moves the alignment lookahead from 2 cells to 6.5 and lifts the
rotational score span from ≤ 5.6 to ≤ 18.2, touching nothing else. Copy
the params file, edit those two lines, hash it, and run the same
`c2n14_run.sh` invocation with tag `c2n21_tour_r1`.

Read-out, in order: longest selected-zero run (BAD 28.60 s, GOOD
8.1–18.8 s); `dwb_best_vx_zero_frac` (BAD 0.607, GOOD 0.291–0.401);
`dwb_best_critic_mean.GoalAlign` (BAD 16.08, GOOD 4.45–6.55); and the
fraction of commanded `wz` carrying the sign of the required turn (BAD
23.8 %). **The falsifier: if the rotation cap is the mechanism, the
negative-wz fraction must fall well below 74.6 %.** If the stall shortens
but the wz sign does not improve, the diagnosis is wrong and the next
candidate is the 9-cell `aggregation_type: last` cap, not this knob.

BAD reproduces roughly 1 tour in 4 (C2-NAV.18 0/3, C2-NAV.19 1/1), so one
SUCCEEDED tour proves nothing — run until a reproduced BAD is available
for comparison, or report the sample inconclusive the way C2-NAV.18 did.
Do **not** tune CSF, inflation, `BaseObstacle`, `PolygonStop`,
`RemovePassedGoals`, the waypoint, the goal, the planner, or any velocity
or acceleration limit.

---

## C2-NAV.21 — every DWB mechanism scored offline, two tested live, both worse than baseline (2026-09-05)

**Branch/worktree.** `worktree-c2nav0-diagnosis` at
`.claude/worktrees/c2nav0-diagnosis`, pushed to `jazzy2`. **`main`
untouched.** Full write-up: `docs/RESULTS.md` "C2-NAV.21".

**What was built.**

* `docs/data/c2nav21_mechanism.py` — C2-NAV.20's validated
  reconstruction with every scoring constant turned into an argument:
  `aggregation_type` per critic (from
  `MapGridCritic::scoreTrajectory`, including the `start_index` shortcut
  that applies only under `Last && !stop_on_failure_`), separate
  forward-point distances applied in *both* places the source applies
  them, `sim_time`, sample counts, granularities, all four MapGrid scales,
  and the local costmap's finite extent so a nudged seed clips rather
  than being silently used.
* A **full-pose** trajectory generator, because `sum`/`product` score
  every pose and `getTimeSteps` makes the pose count velocity-dependent
  (`num_steps + 2`, 3 to 62 across the lattice).
* A **selection** validator, the gate C2-NAV.20 never had: with all seven
  critics and C2-NAV.3's captured costmap it picks the trajectory DWB
  picked, **6/6**, and reproduces the per-trajectory pose count 819/819.
* `docs/data/c2nav21_states.json` — the 145 states, 15 plan snapshots,
  stop-probe clearance and the leg's 10 Hz command trace, so every Stage 1
  table regenerates without the untracked `.navbench/` scratch.
* `gazebo_models/scripts/nav_bench.py` — per-cycle degeneracy in the
  `/evaluation` callback: the COMPLETE vs SHORT-CIRCUITED split via
  `len(score.scores)`, the zero-vx/forward margin, the exact-tie count,
  the rotation-block span, and illegals per cycle keyed by the throwing
  critic. Additive; every pre-existing field keeps its meaning.
* `docs/data/c2nav21_instrument_test.py` — 29 checks, no ROS graph. One
  exists because the `Oscillation` count's success condition is "we saw
  none", so it first feeds the callback a cycle that *does* carry a
  latched ban and refuses to pass unless the instrument reports it.
* `docs/data/c2nav21_live.py` — the run reader. Reproduces C2-NAV.18/.19's
  committed observations first; reads safety from the stop probe's true
  `d_min_base_m`, not nav_bench's quantised `min_clearance_m`; refuses to
  report a leg the robot never reached; and classifies **which subsystem**
  stopped the robot by ORDER, not occupancy.
* `.navbench/c2n21_matrix.sh`, `c2n21_run_b.sh`, `c2n21_up_nav_b.sh`.
* Figures `docs/images/c2nav21_mechanism.png`, `c2nav21_live.png`.
  Artifacts `c2nav21_bench.json`, `c2nav21_live.json`.

**Measured — offline, 145 states, no simulator.**

* **`aggregation_type` falsified.** The pose count is velocity-dependent,
  so a sum is dominated by how *many* poses a trajectory has. It makes
  forward win at all 145 states and picks the smallest non-zero vx with
  **zero rotation** — for a robot 71–85° off its plan that is a different
  failure, not a fix. `product` spans 100 decades.
* **The velocity lattice falsified.** vx 40 / vtheta 80 leave the margin
  bit-identical and make the landscape *more* degenerate (median
  trajectories at the minimum 3 → 8, max 16 → 32).
* **`forward_point_distance` 0.325**: rotation span ×3, trajectories at
  the minimum median 2 / max 5 — the least degenerate of any candidate —
  but **zero-vx wins rise 46 → 61**.
* **`sim_time` 2.5**: the only candidate that lowers the forward block's
  score *absolutely* (−1.20 against zero's −0.20), forward strictly
  winning at all 145 states. Also makes **12× as many trajectories
  illegal** on C2-NAV.3's captured costmaps (13 → 156), and that
  transfers across CSF because `isValidCost` keys on `INSCRIBED`, which
  comes from the inscribed *radius*.
* **Withdrawn**: every rotation-choice statistic for C2-NAV.19. The live
  command sits a median **3.2 score points** above the model's optimum
  with 96 of 819 better, so which individual trajectory wins is not
  reconstructible. The margin, tie count and span are kept — a constant
  seed error cannot change which totals tie.

**Measured — live, 13 tours, fresh simulator each, interleaved.**

| arm | tours | reached the leg | SUCCEEDED |
|---|---|---|---|
| baseline, topology A | 8 | 7 | **6** |
| baseline, **topology B** | 3 | 2 | **1** |
| `fpd` 0.325 | 3 | 3 | **0** |
| `sim_time` 2.5 | 3 | **0** | — |

* **The degeneracy reproduces.** Offline predicted a crawl-window margin
  of −0.6 / **0.00** / +1.4; live measured −0.4 / **0.00** / +1.4.
* **`fpd` 0.325 REJECTED.** It removes the tie (27 tied → **2**, hitting
  the offline 2/5 prediction) and replaces it with a decisive preference
  for standing still: margin 0.00 → **−4.20**, zero-vx wins 8.4 % →
  28.1 % of transit cycles, longest zero-vx run 39.2 s → 79.0 s.
  Mechanism: **a longer alignment radius is worth more to a rotation than
  to a translation.** Clearance was *better*, 0.314 m vs 0.249 m.
* **`sim_time` 2.5 REJECTED upstream.** 3/3 tours wedge at
  `obstacle_corner` within 6 cm of the same spot, the **global** planner
  refusing to plan. Not a collision: true clearance 0.270–0.278 m.
* **`Oscillation` measured.** It bans **exactly 400 of 819** samples (one
  wz sign); on `base_r1`, 374 of 1429 cycles, and in transit 84 000
  illegals against `BaseObstacle`'s 50 144. But `base_r1` SUCCEEDED at a
  26 % ban rate and the C2-NAV.19 BAD run failed at 4 % — **neither
  necessary nor sufficient**. An offline replay of its state machine did
  NOT separate and was rejected.
* **THE BIG ONE, and it was not what was being looked for.** Splitting
  the leg at the xy tolerance shows the **terminal yaw settle is 47–78 %
  of every enclosure leg that completes the approach**, with **200° to
  1124°** of yaw travelled to settle one heading. `c2n21_base_r4` spent
  **151.90 s turning through three full revolutions** and still counted
  as a success; its approach took 42.27 s. Of seven baseline tours that
  reached the leg, **one** failed in the approach — the mechanism the
  whole C2-NAV series has been chasing. The other six completed it and
  then spent the majority of the leg rotating on the spot.

**Unverified / NOT PROVEN.**

* That any DWB scoring parameter fixes the approach stall. Two were
  tested against a measured mechanism; both made things worse.
* That a smaller `sim_time` avoids the planner refusal — untested, and
  deliberately: there is no measured mechanism for the refusal.
* The C2-NAV.21 instrument **never captured a baseline approach
  failure**: the baseline succeeded on every tour that reached the leg in
  this session, so there is no GOOD/BAD contrast under the new metrics.
* Topology B: 3 tours, 2 reached the leg, **1 SUCCEEDED**. Both valid
  tours put the approach at **32.85 / 35.84 s** and the terminal
  rotation at **169.50 / 156.97 s — 83.8 % / 81.4 % of the leg**, with
  405° / 355° of yaw travelled. On the TIMEOUT the approach was healthy
  by every metric (transit margin median **+14.4**, zero-vx wins 11 of
  222, PolygonStop never entered) and the robot sat **0.069 m** from its
  goal. The only difference between the SUCCESS and the TIMEOUT was
  whether the rotation settled inside the 200 s cap.
* Not measured: three *DWB-limited* candidate tours per arm. The
  baseline succeeded on every topology-A tour that reached the leg in
  this session, so the new per-cycle metrics never captured a baseline
  approach FAILURE to contrast against.

**Two facts about the shipped robot, both measured from the repository.**

* **The shipping gap is two lines.** `mission.launch.py` includes
  `nav.launch.py` **without** `params_file`, so it reads the trunk's
  `gazebo_models/config/nav2_params.yaml`, where
  `default_nav_through_poses_bt_xml` still points at the **NavigateToPose**
  tree and local `cost_scaling_factor` is still **5.0**. The C2-NAV.11
  fix — the only CONFIRMED enclosure fix in the series — has never been
  in effect on the robot that ships.
* **Merge hazard.** This branch modifies that same file to
  `BaseObstacle.scale: 2.0`, C2-NAV.2's measured-and-REJECTED value, left
  deliberately as that experiment's record. Revert it before any merge:
  `git checkout 8f05c45 -- gazebo_models/config/nav2_params.yaml`.

**Tests.** `gazebo_models` **41/41**, matching the CLAUDE.md baseline —
the only package modified. No test imports `nav_bench.py`.

**The exact next command to run.** Not another DWB scoring knob. The
measured remaining cost is the terminal yaw settle, and it has never been
the subject of an experiment. The mechanism C2-NAV.1 named is still in
the parameter file, unchanged:

```bash
cd ~/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis
grep -n 'xy_goal_tolerance\|yaw_goal_tolerance\|RotateToGoal' \
     docs/data/c2nav11_ntp_params.yaml
# FollowPath.xy_goal_tolerance   0.05
# goal_checker.xy_goal_tolerance 0.25   <- 5x disagreement, no
# goal_checker.yaw_goal_tolerance 0.25     rotate-in-place mode between
```

Then one variable, with the same discipline this experiment used: predict
the signature offline from `terminal_yaw_travel_rad` and
`dwb_illegal_by_critic_terminal`, state the falsifier, and run three
fresh tours reading `t_terminal_s` and `terminal_frac_of_leg` — both of
which nav_bench already records.


## 2026-09-05 — C2-NAV.22: the terminal yaw settle explained — the rotate-to-goal critic is switched off for most of it, and the 0.05 m window is why

Diagnosis only. **No simulator was started, no ROS node was run, no
parameter was written.** `docs/data/c2nav11_ntp_params.yaml` is unchanged
at sha256 `6f61e499…1e6bb950`, the file C2-NAV.20 froze and C2-NAV.21
ran against. Branch `worktree-c2nav0-diagnosis`, worktree
`.claude/worktrees/c2nav0-diagnosis`.

### The question

C2-NAV.21 finished by measuring that the enclosure leg's terminal phase
is 47–84 % of the leg and that one final heading costs 200–1124° of yaw.
It named three candidates — `FollowPath.xy_goal_tolerance` 0.05 against
the goal checker's 0.25, the `Oscillation` critic, and an interaction —
and tested none.

### What was built

`docs/data/c2nav22_yaw.py`, which reads the committed traces and nothing
else, and `docs/data/c2nav22_yaw.json` (1.6 MB), which freezes the ten
enclosure traces, their leg records, the terminal plan-republication
periods and the ordinary-leg control so every table regenerates from
`docs/data/` alone. That was verified by re-running the whole suite with
the scratch directory pointed at a path that does not exist; the numbers
are bit-identical either way.

The gate runs first and the module reports nothing until it passes:
C2-NAV.21's own transit/terminal split reproduced from the raw 10 Hz
traces on all nine legs that have one — transit within **0.09 s**,
terminal fraction within **0.002**, yaw travel at **0.99–1.00×** (asserted
as ≥ 85 %, because the reconstruction resamples at 10 Hz and is a lower
bound by construction).

### The central result

**The terminal phase is not a heading failing to converge. It is a
rotate-in-place mode that keeps being switched off.**

Across 2300 rotate-in-place cycles on five tours and both topologies,
DWB turns *away* from the goal heading on **885 (38.5 %)** of them — and
every one of the 885 is in one of exactly two states: **694 (78.4 %)**
with `RotateToGoalCritic` inactive, **191 (21.6 %)** carrying a full
one-sign `Oscillation` ban, and **0 with neither**. There is no residual
mechanism left to find.

The loop, every step separately measured:

1. The BT republishes the path every **2.95–3.04 s** (`RateController
   hz="0.333"`). `DWBLocalPlanner::setPlan` resets **every** critic
   (dwb_local_planner.cpp:238-246) — the only caller of `reset()` there
   is — so `in_window_` clears.
2. It re-latches only if the controller's own distance to the plan
   endpoint is ≤ **0.05 m**, because `RotateToGoalCritic` reads
   `FollowPath.xy_goal_tolerance`, not the goal checker's 0.25
   (rotate_to_goal.cpp:61-64, :90).
3. With the latch off the critic scores 0.0 for everything — its scale of
   32.0 contributes nothing — so translating trajectories are legal again
   and `GoalAlign`/`PathAlign` choose the heading: the path direction,
   not the goal orientation.
4. DWB drives, and the robot leaves the window. Ground-truth distance
   rises from a median 17–27 mm while latched to **46–69 mm while
   unlatched**, out to 156 mm, with 46–73 % of unlatched cycles genuinely
   outside 0.05 m.
5. It must creep back in at **7–24 mm/s** before the latch can set.
6. Go to 1.

The critic is latched for only **0.0 % / 14.2 % / 15.5 % / 21.7 % /
44.5 % / 55.7 %** of the terminal window on `fpd_r3` / `base_r1` /
`base_r4` / `bbase_r2` / `base_r3` / `bbase_r3`, and **74.5–100 %** of
terminal yaw on the four worst legs is travelled with it off.

### And a premise of the question that was wrong

C2-NAV.21 split the leg at 0.25 m and called everything after it terminal
rotation. There is a third phase in between, and it is a crawl: every leg
spends **9.2–151.8 s creeping from 0.25 m to 0.05 m at 7.4–23.9 mm/s**.

**`c2n21_base_r4` — the 1124° leg — has no rotate-in-place phase at
all.** Its closest ground-truth approach was **0.071 m**, and all
**19.594 rad** was travelled while creeping: 151.80 s to drive **1.116 m
of path** inside a 0.25 m ball without ever getting within 71 mm. That is
the robot circling inside its own goal tolerance, not a heading failing
to settle.

**`c2n21_fpd_r3` is the cleanest case in the series.** C2-NAV.21 recorded
it arriving within 0.087 m and then spending 119.70 s failing to settle.
Its `dwb_ill_rot` is **0 on all 1197 terminal cycles**: RotateToGoal
never latched once, on a leg that reached 0.040 m in ground truth.

### The control that bounds every absolute heading number

The traces record **ground truth**; the goal checker and every critic are
fed `costmap_ros_->getRobotPose()`, the AMCL estimate. Across the **18
ordinary legs of three baseline tours** — legs that settle in 0.8–7.8 s
with no pathology at all — the controller stops commanding while the
ground-truth heading is still **0.194–0.492 rad** from the target, and
all 18 report SUCCEEDED. Only **3 of 21** SUCCEEDED legs end inside the
0.25 rad tolerance in ground truth.

So **heading arrival cannot be dated from committed artifacts** and is
reported as null rather than guessed. Yaw travel, sign reversals and turn
direction are frame-independent and carry the argument instead; whether
the controller believed itself inside 0.05 m is read from the critic's
own rejection count, which is strictly bimodal (779 or 0, no third value
in 7300 terminal samples).

### What else was settled

* **`PolygonSlow`: a ~4× time cost, not a feedback loop.** The monitor
  multiplies the whole twist including `tw` by `slowdown_ratio`
  (collision_monitor_node.cpp:543-551); measured
  |w_act|/|w_nav| is **0.173–0.293** against a configured 0.30, and
  **0.000** under `PolygonStop` — the control that proves the instrument
  separates the two actions. It cannot bias direction:
  `StandardTrajectoryGenerator::startNewIteration` passes **`sim_time_`**
  (1.5 s), not `sim_period`, as the acceleration horizon, so at
  `acc_lim_theta` 3.2 the reachable window is ±4.8 rad/s and the lattice
  spans the full kinematic range every cycle regardless of what the robot
  achieved.
* **Target-yaw drift: REJECTED.** The target is 0 rad, constant,
  established in four source paths; `use_final_approach_orientation:
  false` makes `plan.poses.back().pose.orientation =
  goal.pose.orientation`, `RemovePassedGoals` cannot remove the last
  goal, and the terminal plan pose sits a median 0.0117 m from the
  requested goal.
* **Angle wrapping: REJECTED in general, real on one leg.** Seven of nine
  legs never come within 0.2 rad of ±π and wrap zero times. `base_r4`
  arrives at **−3.106 rad (178°, the antipode)**, spends 10.8 % of its
  terminal there and wraps 8 times — and has 8 heading-error zero
  crossings against 0 or 1 everywhere else.
* **A genuine source defect, recorded because it is *not* the answer.**
  `OscillationCritic::resetAvailable` compares
  `pose_.theta - prev_stationary_pose_.theta` **unwrapped**
  (oscillation.cpp:190), so a ±π crossing clears the ban immediately.
  That is permissive — it shortens bans — so it cannot add rotation. It
  matters to anyone who later tunes `oscillation_reset_angle`.

### A defect in this session's own instrument, found and fixed

The first version decided whether a run carried the C2-NAV.21 per-cycle
columns by asking whether any row had a value in `dwb_ill_rot`. That
column is blank on every cycle the critic rejected nothing — so a leg
where RotateToGoal **never fired at all** is indistinguishable from a leg
recorded before the column existed. `c2n21_fpd_r3` is the former and was
silently dropped from the attribution table. The probe now reads the
trace's **schema**, and the recovered leg turned out to be the strongest
single case in the section. Two more of the same class were caught
earlier: the artifact dump wrote `value or None`, which nulls every
measured zero, and the pose filter checked only `yaw`.

### Verdict

* **SUPPORTED, dominant** — RotateToGoal inactive because `setPlan`
  resets its latch and the 0.05 m window is smaller than the stack's own
  position error.
* **SUPPORTED, but not as the brief framed it** — the 0.05/0.25
  mismatch. It does not create a long rotate-in-place phase; it creates
  the creep, and it is the window whose re-latch keeps failing.
* **SUPPORTED as time, REJECTED as feedback** — `PolygonSlow`.
* **PARTIALLY SUPPORTED, secondary** — `Oscillation`: 21.6 %, and 0 of
  191 wrong-way cycles on `c2n21_bbase_r3`.
* **REJECTED** — target-yaw drift, plan terminal orientation, angle
  wrapping in general, and any further controller-state mechanism.

**NOT PROVEN**: that the AMCL pose specifically is what breaks the
re-latch. `/amcl_pose` is subscribed by nav_bench but written to no trace
column and no leg field, so the controller's own distance-to-plan-endpoint
cannot be reconstructed. That is the one essential signal missing, and it
is what the next instrument must add. Also not proven: that raising
`FollowPath.xy_goal_tolerance` fixes the leg — that is a prediction, not
a result.

### Why no simulator was spent

The brief allows one fresh run if an essential signal is unrecoverable,
and one is. It was not spent because the attribution is already complete
without it (`NEITHER` = 0 over 2300 cycles) and both candidate causes of
a failed re-latch point at the **same single parameter** — so a run could
not have changed which experiment comes next.

### Exact next live experiment — C2-NAV.23

**Raise `FollowPath.xy_goal_tolerance` from 0.05 to 0.25**, matching
`goal_checker.xy_goal_tolerance`. One line, one mechanism. Verified
single-variable rather than assumed: across nav2 1.3.11 the only consumer
of `FollowPath.xy_goal_tolerance` is `RotateToGoalCritic`
(rotate_to_goal.cpp:61-64); `SimpleGoalChecker` reads its own namespaced
copy (simple_goal_checker.cpp:75-86) and is untouched.

Predicted and falsifiable: the unlatched fraction of the terminal window
goes to ≈ 0 (**primary read — if it does not move, drop the candidate
rather than retuning it**); wrong-way cycles fall from 38.5 % toward the
21.6 % `Oscillation`-only floor; the creep phase disappears; terminal yaw
travel falls from 2.0–7.1× the required heading toward 1×; terminal share
falls from 47–84 %.

**Risk to measure, not discover**: with `in_window_` latched at 0.25 m the
robot may not translate 250 mm in, so a short approach can no longer be
closed by driving. The run must report `final_goal_err_m` per leg and the
candidate **fails if any leg's final error exceeds 0.25 m** — a criterion
fixed now, before the run.

```bash
cd ~/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis

# 1. reproduce C2-NAV.22 offline first (no simulator, ~20 s)
python3 -P docs/data/c2nav22_yaw.py all

# 2. the candidate: copy the frozen baseline and change ONE line
cp docs/data/c2nav11_ntp_params.yaml docs/data/c2nav23_fpxy_params.yaml
#    FollowPath: xy_goal_tolerance: 0.05  ->  0.25
sha256sum docs/data/c2nav11_ntp_params.yaml docs/data/c2nav23_fpxy_params.yaml
diff docs/data/c2nav11_ntp_params.yaml docs/data/c2nav23_fpxy_params.yaml

# 3. interleaved, fresh simulator per tour, one Gazebo at a time
bash .navbench/c2n21_matrix.sh \
     "c2n23_base_rN:$PWD/docs/data/c2nav11_ntp_params.yaml" \
     "c2n23_fpxy_rN:$PWD/docs/data/c2nav23_fpxy_params.yaml"

# 4. topology B, which needs initial_mode:=nav (the runner enforces it)
bash .navbench/c2n21_matrix.sh \
     "c2n23_bfpxy_rN:$PWD/docs/data/c2nav23_fpxy_params.yaml:B"
```

Whether the `Oscillation` residual then needs its own experiment is
C2-NAV.24's question and is deliberately not pre-empted.

### Housekeeping

No simulator was started this session, so there is nothing to clean up;
`ros_clean.sh` was not needed and was not run. No monitor, loop, cron or
background process was created. nav2/dwb 1.3.11 sources were fetched to
the session scratch directory for reading only and are not part of the
repository.

## 2026-09-05 — C2-NAV.23: the one line ran live, and it is REJECTED — the tolerance is the radius at which the robot stops driving

Three fresh tours, fresh simulator each, one Gazebo at a time. Branch
`worktree-c2nav0-diagnosis`, worktree `.claude/worktrees/c2nav0-diagnosis`.
`docs/data/c2nav11_ntp_params.yaml` unchanged at sha256
`6f61e499…1e6bb950`; `main` untouched.

### The question

C2-NAV.22 predicted that raising `FollowPath.xy_goal_tolerance` from
0.05 to 0.25 would remove the terminal creep/relatch loop, and fixed the
falsifier in advance: **reject if any fresh leg ends more than 0.25 m
from its goal in ground truth, even if Nav2 reports SUCCESS.**

### What was run

One parsed YAML leaf differs from the frozen baseline —
`/controller_server/ros__parameters/FollowPath/xy_goal_tolerance`,
0.05 → 0.25 — and nothing else. Verified by flattening both files to
leaves rather than by `diff`, and verified **live** on all three runs by
reading the parameter back off the running `controller_server` (0.25,
0.25, 0.25).

That live readback was not optional. `c2n6_verify.sh` — the existing
runner's verifier — reads `goal_checker.xy_goal_tolerance` and **not**
FollowPath's, so nothing already in the harness could distinguish a file
that was edited from a file that was loaded. `.navbench/c2n23_liveparam.sh`
exists for that one check and starts nothing.

`c2n23_fpxy_r1`, `c2n23_fpxy_r2` (topology A) and `c2n23_bfpxy_r1`
(topology B, arbiter confirmed live in `nav` mode), through the unchanged
C2-NAV.11/.14 runner with the byte-identical C2-NAV.21 `nav_bench.py`.

### The result

**REJECTED on the falsifier, on 3 of 3 runs and both topologies.**

| arm | SUCCEEDED legs | over 0.25 m | median | max |
|---|---|---|---|---|
| baseline (frozen) | 39 | **0** | 0.085 m | 0.224 m |
| candidate (fresh) | 16 | **6** | 0.227 m | 0.349 m |

All six report SUCCEEDED. The baseline's own over-tolerance legs (8 of
49) are every one TIMEOUT or ABORTED, so this is not the pre-existing
ground-truth-vs-AMCL offset.

### Why — and the mechanism is confirmed, which is the point

`dwb_ill_rot` is `RotateToGoalCritic`'s own rejection count, so the first
cycle it bans the translating block is the cycle its latch set. Read on
`open_space`, the **first leg of the tour** and therefore the one leg
that cannot inherit a latch from its predecessor:

| arm | n | d_latch median | range | final err median | leg s |
|---|---|---|---|---|---|
| baseline | 5 | **0.074 m** | 0.049–0.097 | 0.087 m | 14.66 |
| candidate | 3 | **0.257 m** | 0.205–0.280 | 0.265 m | 12.66 |

`d_latch` tracks the parameter, and **`d_latch` ≈ final error in both
arms**. Once latched and stopped, `RotateToGoalCritic::scoreTrajectory`
throws `IllegalTrajectoryException` for any non-zero `vx`, so the
remaining distance cannot be closed by driving at all. The leg does get
faster — 14.66 → 12.66 s — and the trade is one-for-one: the terminal
settle is not fixed, it is converted into terminal position error.

**`FollowPath.xy_goal_tolerance` is not a free knob. It is the radius at
which the robot stops driving.**

### The compounding consequence

Every leg ends 0.15–0.20 m short, so the next starts there. Candidate
tours began the enclosure leg at x = −2.420…−2.467 against a baseline
−2.578…−2.626 (7 of 8 topology-A tours), into a 0.63 m pinch, and
**0 of 3 reached the enclosure goal ball** (baseline 6 of 8): TIMEOUT
1.140 m / TIMEOUT 1.070 m / ABORTED 1.451 m, with 173.2 / 182.5 / 6.5 s
of PolygonStop.

**Bounded honestly:** `c2n19_tour_r1` is a *baseline-params* run that
failed the same way (TIMEOUT, 1.106 m, 131.0 s PolygonStop), so the
enclosure deadlock exists at 0.05 m too. The start-pose displacement is
measured and attributable; the deadlock's dependence on the parameter is
consistent but **not isolated**, and is not claimed as more.

### What could NOT be measured

The pre-registered primary read — the unlatched fraction of the enclosure
leg's terminal window going to ≈ 0 — **has no value on the candidate
arm.** All three runs lost the leg before reaching the 0.25 m ball, so
there is no terminal window. `terminal`, `phases`, `latch` and
`attribution` print `never reached the 0.25 m outer tolerance` three
times. The correct statement is not "terminal yaw did not improve" but
"the candidate never got far enough to have a terminal phase on the leg
the experiment was about".

### Instrument

`docs/data/c2nav23_fpxy.py` (+ `docs/data/c2nav23_fpxy.json`, 624 KB).
It imports the frozen `c2nav22_yaw` rather than re-deriving anything, and
its gate refuses to report until the parsed leaf diff is exactly one, the
live readback is 0.25 on every run, and C2-NAV.22's own selftest passes
— which it still does, unchanged.

`falsifier`, `paired` and `arrival` — the three tables carrying the
verdict — were verified **bit-identical with the scratch tree pointed at
a path that does not exist**, so they regenerate from `docs/data/` alone.
Two defects in this session's own instrument were found that way and
fixed: `arrival()` fell through to an empty table when the leg records
resolved from the bundle but the traces did not, and `have()` reported a
completed run as absent for the same reason.

### Verdict

* **REJECTED** — the falsifier, 6 SUCCEEDED legs over 0.25 m against 0 of 39.
* **REJECTED** — material regression outside the objective: 0 of 3
  enclosure legs reached.
* **CONFIRMED** — C2-NAV.22's mechanism. The latch radius is measured
  live at 0.074 → 0.257 m.
* **NOT MEASURED** — whether a wider latch removes the creep.

This rules out the whole "widen the latch" family, `Oscillation` retunes
included: any fix has to keep translation **legal** while position error
is still being closed, which this parameter cannot do by construction.

### Housekeeping

`ros_clean.sh` run after the last tour: 10 matched, 0 still running. No
monitor, loop, cron or recurring wakeup was created. The `bench exit 139`
segfault on the two topology-A runs is pre-existing (baseline `base_r1`
and `bbase_r3` end the same way) and occurs **after** the JSON is
written; all telemetry probes reported `TELEMETRY OK`.

### Exact next step — C2-NAV.24

Do **not** try another tolerance. The question the rejection leaves is
narrow and answerable offline first: the creep exists because
`RotateToGoal` is off *and* `GoalAlign`/`PathAlign` pick the heading
while `GoalDist` is the only critic still pulling the robot in. The next
instrument should measure, on the frozen baseline traces, what the
translating block is actually being scored on during the 0.25 → 0.05 m
creep — i.e. whether the creep is slow because DWB *prefers* a slow
trajectory or because the collision monitor's `slowdown` is scaling a
normal one. C2-NAV.22 measured the monitor gain (0.173–0.293 against a
configured 0.30) but not the pre-gain choice.

```bash
cd ~/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis
python3 -P docs/data/c2nav22_yaw.py selftest
python3 -P docs/data/c2nav23_fpxy.py all
```

## 2026-09-06 — C2-NAV.24: the creep velocity is DWB's own choice first and PolygonSlow's multiplier second — offline diagnosis, no simulator

No simulator, no ROS node, no parameter written, no background job.
Branch `worktree-c2nav0-diagnosis`, worktree
`.claude/worktrees/c2nav0-diagnosis`. `docs/data/c2nav11_ntp_params.yaml`
unchanged at sha256 `6f61e499…1e6bb950`; `main` untouched at `ea66155`.

### The question

C2-NAV.23 ruled out the whole "widen the latch" family, so the next fix
has to come from somewhere else. The undecided question was where the
7–24 mm/s creep command is actually born:

* **A** DWB itself selects a tiny forward velocity, or
* **B** DWB selects a normal one and the collision monitor's PolygonSlow
  scales it down before the wheels see it.

Controller fix versus monitor fix. They could not be told apart from
anything C2-NAV.22 published, because C2-NAV.22's `chain_stats` measured
only the **angular** rate.

### What made it answerable

`nav_bench.py` has recorded five points on one linear command chain at
10 Hz since C2-NAV.21 and **no analysis had ever read them**:
`dwb_best_vx` (DWB's selected trajectory, straight off `/evaluation`),
`v_nav`, `v_smoothed`, `v_cmdvel` (collision-monitor output), `v_wheel`,
plus ground truth. The monitor sits exactly between `v_smoothed` and
`v_cmdvel`, so its contribution is one ratio and cannot be confused with
anything else on the chain.

`docs/data/c2nav24_chain.py` reconstructs it over the 0.25 → 0.05 m creep
window on **70 legs across 10 runs and both topologies**, and
`docs/data/c2nav24_chain.json` freezes every trace it reads so the
analysis reproduces from the repository alone — verified byte-identical
with the scratch tree pointed at a path that does not exist.

### The answer: BOTH, with DWB primary

| | DWB's own selection | collision monitor | residual |
|---|---|---|---|
| time-weighted, 66 legs / 873 creep s | **77.3 %** | **23.8 %** | −1.1 % |
| the 11 pathological legs (creep ≥ 20 s) | **73.2 %** | **26.2 %** | – |

Every chain stage is a verbatim pass-through except the monitor:

| stage | median gain |
|---|---|
| controller publish, smoother, arbiter → wheels | **1.000** each |
| collision monitor, no polygon active (n = 1520) | **1.000** |
| collision monitor, PolygonSlow active (n = 3268) | **0.300** |

`0.300` is the declared `slowdown_ratio` to three decimals, and the
`1.000` idle reading is what makes it a measurement rather than an
assumption — a genuine falsification opportunity the data passed. Stable
over a 20× range of the ratio floor.

And DWB's own command is already small **before** the monitor: on transit
it picks lattice index 18–19 of 19 (0.284–0.300 m/s); inside the creep it
picks index 1–7 (0.0158–0.1105 m/s) and selects `vx = 0` outright on up
to 86 % of cycles. On the worst leg — `c2n21_base_r4/enclosure_entry`,
151.8 s — DWB selected a mean of **5.0 mm/s** against its own 284 mm/s
transit command, a 57× self-imposed cut, *before* the monitor's 3.3×.

The decisive control: on legs PolygonSlow never touches, the monitor is a
measured pass-through (**1.011**) and the creep still runs at a DWB-chosen
47 mm/s. Removing the monitor entirely leaves **489 s of the 873 s**.

### Three things measured that were not known before

1. **Of 3980 vx = 0 creep cycles, 67.6 % were BANNED** — no complete
   legal forward trajectory existed at all — and 32.4 % were OUTSCORED
   (forward existed and scored worse). **0.0 % anomalies**, which is what
   licenses reading the margin fields this way.
2. **The pathological leg and the ordinary legs are stopped by different
   critics.** On ordinary legs `RotateToGoal` bans the translating block
   on 23–73 % of creep cycles; on `enclosure_entry` it does so on
   **0.0–15.5 %**, and **OscillationCritic** carries it instead
   (8.7–44.2 %). C2-NAV.22 saw the symptom (`c2n21_fpd_r3` never latched
   once in 1197 cycles) without separating the cause.
3. **PolygonSlow is active 92.4–100 % of every pathological creep
   window.** A 0.8 × 0.8 m slowdown box in a 0.63 m pinch cannot switch
   off; it is a constant speed derating, not a hazard response.

### Bounded honestly

* **PolygonStop never fired inside a creep window: 0 of 8795 cycles**
  (SLOWDOWN 5815, DO_NOTHING 2485, LIMIT 424). The creep is not a gating
  phenomenon.
* **AMCL distance to goal is UNAVAILABLE.** `nav_bench` subscribes to
  `/amcl_pose` and never writes it to any artifact, so requirement 8 of
  the brief cannot be answered from the frozen traces. Reported as
  unavailable, not substituted with ground truth.
* **The 77/24 split is claimed as a direction and a magnitude, not to
  that precision.** It uses each leg's own transit command as the
  reference for "normal", which is a modelling choice; per-leg shares on
  sub-5 s legs are noise-dominated. It survives restriction to the
  pathological legs (73/26) and is corroborated by an independent
  counterfactual (44 % of seconds).
* **A gate had to be replaced, not tuned.** The first `terminal_v_med`
  gate failed on 15 of 66 legs by up to 0.055 m/s. The cause is real and
  worth recording: `_gt_cb` timestamps ground truth with the **node clock
  at callback time**, not the message stamp, so the live series is
  sample-count weighted and bunches when the executor is idle — which
  over-represents the stopped tail. This module is time-uniform at 10 Hz.
  Two different weightings of two different series, not a disagreement
  about the robot. The gate was replaced with weighting-invariant ones
  (transit split, final goal error) and the discrepancy is now **printed
  as G5** rather than silently passed.

### Gates

`python3 docs/data/c2nav24_chain.py selftest` — all pass:
G1 transit split vs nav_bench live, 66 legs, worst 0.10 s;
G2 `final_goal_err_m` vs live, 70 legs, worst 0.0076 m (one ZOH cycle of
travel); G3 monitor idle 1.000 / PolygonSlow 0.300; G4 15267/15267
`dwb_best_vx` on the declared lattice; G5 the caveat above, reported.

### What this rules out

Hypothesis B alone; Hypothesis A alone as a complete account; the
smoother, arbiter and plant entirely; a PolygonStop/gating explanation;
and — combined with C2-NAV.23 — anything aimed at the RotateToGoal latch
as a fix for `enclosure_entry`, where that critic is barely involved.

### Exactly one next action

One live experiment, one leaf, prediction and falsifier fixed in advance:

```
collision_monitor/ros__parameters/PolygonSlow/slowdown_ratio : 0.3 -> 1.0
```

1.0 rather than deleting the polygon, so `cm_polygon` still reports
`PolygonSlow` and the traces stay comparable. `PolygonStop` (0.25 m
circle, action `stop`) and `PolygonLimit` untouched.

Chosen not because it is the larger cause — it is not — but because it is
the only lever whose effect is already known exactly (0.300, n = 3268, no
feedback path into DWB), which makes its prediction sharp enough to
**falsify this whole decomposition in one run**. If creep seconds do not
fall roughly as derived, the 77/24 split is wrong and no controller work
should be started on it. It also cannot reproduce the C2-NAV.23 failure
mode, because it moves no goal tolerance.

**Predicted (DERIVED):** total creep 873 s → ~489 s; the 151.8 s leg →
~45.4 s; monitor stage gain reads 1.000 under PolygonSlow.

**REJECT if:** any SUCCEEDED leg ends beyond 0.25 m ground truth (the
C2-NAV.23 criterion, carried forward); `min_clearance_m` drops below
`robot_radius` 0.20 m; PolygonStop activations rise; or creep seconds
fail to fall by at least 25 %.

Read `slowdown_ratio` back **off the running `collision_monitor`**, not
from the file — C2-NAV.23's lesson.

### Exact next command

```bash
cd ~/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis
python3 docs/data/c2nav24_chain.py all          # re-read the diagnosis
# then, for the live run, copy c2nav11_ntp_params.yaml, change the one
# leaf above, and drive it through the unchanged C2-NAV.11/.14 runner
# with a fresh simulator per tour and ros_clean.sh between them.
```

---

## 2026-09-06 — C2-NAV.25: the one leaf, live. Creep −59 %, one gate fires by 5 mm

**Built.** `docs/data/c2nav25_slow_params.yaml` (the candidate) and
`docs/data/c2nav25_slow.py` (pre-registration, analysis and gates). The
analysis **imports** `c2nav24_chain` rather than restating it, so the
creep window, the 5 mm/s ratio floor and the stage definitions are
identical between the frozen baseline arm and the fresh candidate arm.
`.navbench/c2n25_matrix.sh` bakes the three specs into the unchanged
C2-NAV.21 driver.

**The behavioural change.** Exactly one leaf of 323:

```
collision_monitor.ros__parameters.PolygonSlow.slowdown_ratio : 0.3 -> 1.0
```

asserted by flattening both YAML files, then re-checking fourteen named
non-negotiables **by value** so a typo in the expected key could not make
them pass silently. Read back off the running `/collision_monitor` on all
three runs: **1.0**, with `PolygonStop.radius` still 0.25.

**Measured.** Three fresh tours, fresh simulator each, `ros_clean.sh`
between, one Gazebo at a time. Topology A, A, B; topology B's arbiter
confirmed `initial_mode: nav`.

* creep per run **108.1 s → 44.2 s, −59.1 %** (gate needed −25 %)
* worst single leg **151.8 s → 19.3 s**
* whole-tour leg seconds per run **370.9 s → 150.8 s**
* legs SUCCEEDED **32/35 → 21/21**; `enclosure_entry` **4/5 → 3/3**,
  reached on every candidate run, so the terminal prediction was
  evaluable on all three
* cycle-weighted creep speed **9.4 → 23.4 mm/s**; achieved vx
  **13.4 → 31.2 mm/s** (2.33×) while DWB's own selection moved
  **22.7 → 26.6 mm/s** (1.17×)
* collision monitor SLOWDOWN **11489/18303 (62.77 %) → 0/4485**;
  PolygonStop **766 (4.19 %) → 47 (1.05 %)**, 15.32 → **1.58 s per run**
* final ground-truth error over SUCCEEDED legs: median 0.086 → 0.078,
  p90 0.137 → 0.150, max 0.224 → **0.255**

**Gates, applied as written.** 3 and 4 PASS (3 in the opposite direction
— PolygonStop *fell*). **1 FAILS** on one leg of 21:
`c2n25_slow_r2/wall_parallel` SUCCEEDED at 0.255 m against 0.25 m.
**2 FAILS**, and the reading was fixed and committed *before* the first
candidate simulator: six of the 35 baseline legs already sit below the
0.20 m absolute floor (lowest 0.151 m), so read absolutely the gate
rejects the control against itself. On the leg that is actually tight —
the enclosure pinch — the candidate is unchanged, 0.150 m against a
baseline 0.152 m.

**Verdict.** The performance claim is **SUPPORTED**; the candidate is
**REJECTED on the letter of the pre-registered gates**. Both are recorded
and neither absorbs the other.

**Two things learned that were not asked for.**

1. **A C2-NAV.24 prediction was wrong.** Setting the ratio to 1.0 was
   chosen so `cm_polygon` would still report `PolygonSlow`. It does not:
   `Velocity::operator<` in `nav2_collision_monitor/types.hpp` is a
   **strict** comparison on squared magnitude, so at ratio 1.0 the scaled
   velocity equals the incumbent and the polygon never claims the action.
   Zero SLOWDOWN cycles in 4485. The `.cpp` is not installed here, so
   that link is derived from two headers and confirmed by the count.
2. **The decomposition under-predicted its own effect** (−44 % derived,
   −59 % measured; worst leg ~45.4 s derived, 19.3 s measured).
   C2-NAV.24's "no feedback path from the monitor back into DWB" is right
   about the **lattice** and does not close the loop through the
   **plant**: an undivided robot is somewhere else next cycle, with a
   different costmap window and OscillationCritic history. Consistent
   with, **not established by**, three runs against five.

**Unverified / what this cannot show.** Not a paired design; the
pathological case reproduces roughly one tour in four; 21 legs cannot
bound a tail, and the single 0.255 m leg is exactly what this sample size
cannot distinguish from the baseline's own 0.224 m. **DWB remains the
primary unresolved cause** — 23.4 mm/s achieved against a 284–300 mm/s
transit command. C2-NAV.24's finding that OscillationCritic, not
RotateToGoal, does the banning on `enclosure_entry` is untouched here.

### Exactly one next action, and the exact next command

Do **not** stack a DWB change on this. Bound the terminal-error tail of
the identical candidate configuration with more fresh tours, then decide.

```bash
cd ~/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis
python3 docs/data/c2nav25_slow.py all      # re-read the result first

# then, per additional tour, one Gazebo at a time:
bash .navbench/c2n21_matrix.sh \
  "c2n25_slow_r3:$(pwd)/docs/data/c2nav25_slow_params.yaml:A"
bash gazebo_models/scripts/ros_clean.sh
# add the new tags to CAND in docs/data/c2nav25_slow.py, re-run `gates`
```

---

## 2026-09-06 — C2-NAV.26: six more tours of `slowdown_ratio = 1.0`

### What was built

`docs/data/c2nav26_robust.py` — the robustness read on C2-NAV.25's
candidate. It **imports** `c2nav25_slow.leg_row` and
`c2nav24_chain.windows` rather than restating them, so all three arms
(frozen baseline, frozen C2-NAV.25, fresh C2-NAV.26) are computed by one
copy of each definition. Commands: `selftest`, `legs`, `yawcheck`,
`failures`, `tail`, `byleg`, `monitor`, `compare`, `robust`, `dump`.

`.navbench/c2n26_matrix.sh` — six fresh tours through the **unchanged**
C2-NAV.21 driver, interleaved A/B/A/B/A/B so topology is not confounded
with the hour of machine state it ran in.

`docs/data/c2nav26_robust.json` — 6 records, 42 traces, 13,026 rows,
verified row-for-row against the scratch tree.

### What was measured

**The configuration was identical and proved so before the first
simulator**: exactly 1 of 323 leaves differs; `PolygonStop` and
`PolygonLimit` re-checked by value; params file byte-identical to the
committed C2-NAV.25 tree (sha256 `4c15893e…`); and the live readback off
the **running** `collision_monitor` byte-identical across all nine
candidate runs (sha256 `eec50fb3…`), with `slowdown_ratio = 1.0`
reported by every new run.

**The accuracy question is answered: the 0.255 m was an isolated tail
event.** 31 fresh SUCCEEDED legs, **none above 0.197 m**; median 0.083 m
against the baseline's 0.086 m. Across all 52 candidate legs, one
exceedance; the baseline's own worst is 0.224 m. `wall_parallel` tops
both frozen arms, so the tail belongs to that goal, not to the
parameter.

**The performance claim replicated**: creep per run 108.1 → 46.9 s
(−56.7 %, against −59.1 % on three runs), worst leg 151.8 → 37.1 s,
achieved vx 0.0134 → 0.0264 m/s, SLOWDOWN cycles 11489 → 0.

**Three other things did not.** 11 of 42 legs failed (3 primary, 8
cascaded) against 21/21 on the three-run arm. `PolygonStop` share rose
to 5.67 % of whole-tour cycles against the baseline's 4.19 % — C2-NAV.25
measured 1.05 % on that same window, so its headline stop reduction did
**not** replicate. Minimum true clearance fell on three legs, worst
per-leg regression 0.137 m.

4 of 8 pre-registered robustness checks pass.

### What was learned

The failure mechanism is `"Start occupied"`: the robot finishes
`wall_adjacent` 0.025 m *past* its goal at 0.259 m clearance (baseline
clusters 0.373–0.397 m), and the planner then refuses every later goal
because its own start cell is lethal. **It is not an AMCL divergence** —
localisation error at leg start peaks at 0.167 m across all arms,
measured by pairing `bt_navigator`'s stated estimated pose against
ground truth once the map/world offset (+2.0, 0.0) is applied. An early
read of that as a 2 m divergence was wrong for exactly that missing
offset.

Simulator degradation is ruled out: RTF 0.985–0.991 in all six runs, and
run 4 was a clean 7/7 after run 3 collapsed.

**The methodological lesson is the durable one: three runs were too
few.** C2-NAV.25's 21/21 and its 1.05 % stop share both looked like
properties of the configuration and were properties of a small sample.

### What remains unverified

Whether `slowdown_ratio = 1.0` **causes** the failures or merely fails
to prevent them. The primary-failure rate — 3 in 5 baseline runs against
3 in 6 candidate runs — does not distinguish them at this n. What
differs is character: the baseline's were late-tour timeouts that had
substantially arrived (0.069 m, 0.095 m); the candidate's include an
early abort that cost five further legs.

No statistical significance is claimed anywhere and none is computed.

### Exactly the next command to run

Diagnose the terminal overshoot from the **already-frozen** traces — no
new simulator, and no second parameter:

```bash
cd ~/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis
python3 docs/data/c2nav26_robust.py failures   # the three primaries
python3 docs/data/c2nav26_robust.py robust     # the eight checks
python3 docs/data/c2nav24_chain.py stages \
        --runs c2n26_slow_r2                   # the chain across the overshoot
```

---

## C2-NAV.27 — "Start occupied" diagnosed offline: it is a localisation term on the estimated pose, not a terminal overshoot, and not new (2026-09-06)

Offline only. No simulator, no ROS node, no parameter changed, nothing
re-run. Every number comes from artifacts frozen before the session.

New module: `docs/data/c2nav27_startocc.py`, eight subcommands —
`inventory geometry bound validate exposure decompose temporal cascade`.

### What C2-NAV.26 recorded, and what the traces actually say

C2-NAV.26 wrote the mechanism as "the robot **finishes** wall_adjacent
0.025 m past its goal … and the planner **then** refuses every later
goal". The sequence is wrong in two ways and the correction changes the
diagnosis.

1. **The wall_adjacent leg never finished.** `controller_server` never
   logged "Reached the goal!" for it and `bt_navigator` never logged
   "Goal succeeded". The *first* refusal is for the wall_adjacent goal
   **itself**, mid terminal yaw-settle, 18.5 s into the leg. 0.025 m is
   where the robot coasted to rest **after** the controller was
   cancelled (measured).
2. **The robot did not overshoot under command.** Peak ground-truth
   overshoot is **+0.076 m** at t = 16.6–17.0 s, and the robot is
   **already retreating** when the planner refuses — `v_act` turns
   negative at 17.1 s and past-goal distance falls 0.076 → 0.055 →
   0.031 m. Commanded `v_nav` never exceeds **0.032 m/s**; post-monitor
   `v_cmdvel` never exceeds **0.016 m/s**. `w_nav` is pinned at
   −1.000 rad/s: this is the yaw settle. `cm_polygon` is **PolygonLimit**
   throughout — never PolygonSlow, never PolygonStop (measured).

### The mechanism

`planner_server.GridBased` is `nav2_smac_planner::SmacPlanner2D`. It
refuses a start whose **global-costmap cell** is too close to a lethal
cell. The Nav2 `.cpp` is not installed here, so the predicate is
**bracketed from the artifacts** rather than read: refusals observed at
cell-to-lethal distance **≤ 0.150 m**, nearest acceptance **0.180 m**, so
the trip point is in **(0.150, 0.180] m**. The published costmap cannot
narrow it — nav2 maps raw 253 (INSCRIBED) to 99 and raw 254 (LETHAL) to
100, so one published 99 covers the whole 0–0.196 m band.

At the wall_adjacent goal the mapped wall's lethal cell row is at map
y = −3.335 and the goal at y = −3.000: **0.335 m of room**, of which
about **0.161 m** may be spent before the planner refuses (measured by
sweep on the reconstructed costmap, which `validate` checks against the
run's own live costmap to within 4 raw counts).

The estimated overshoot is a sum of two independent terms:

    est_past = gt_past + amcl_err

| arm | run | gt_past | amcl_err | est_past | margin | outcome |
|---|---|---|---|---|---|---|
| BASELINE | c2n21_base_r4 | −0.049 | **+0.139** | 0.090 | 0.071 | SUCCEEDED |
| C2-NAV.25 | c2n25_slow_r1 | −0.138 | **+0.178** | 0.040 | 0.121 | SUCCEEDED |
| **C2-NAV.26** | **c2n26_slow_r2** | **+0.025** | **+0.155** | **0.180** | **−0.019** | **REFUSED** |

**The localisation term is 0.155 m of the 0.180 m — 86 %.** And it is
**not larger in the candidate arm**: C2-NAV.25's `c2n25_slow_r1` records
**0.178 m**, larger than the failing run's, and survived only because it
had stopped 0.138 m short. The baseline's own `c2n21_base_r4` reached
est_past 0.090 m, **56 % of the way to the boundary**, at
`slowdown_ratio = 0.3`.

### "Start occupied" is not a new failure mode

It appears in **five earlier runs**, none at `slowdown_ratio = 1.0`:
`c2n21_bbase_r1`, `c2n21_sim_r1/r2/r3`, `c2n12_tour_r1`. `c2n21_bbase_r1`
matters most: its **live parameter readback is byte-identical to the
frozen baseline's** (sha256 `c16f4bcd…`, `slowdown_ratio` **0.3**), and it
produced three refusals with the identical cascade shape — every
subsequent goal refused. It is absent from the frozen arm only because
C2-NAV.21 voided it for never reaching the enclosure leg. C2-NAV.26's
check 8 ("a failure mode absent from both frozen arms") is true of the
five frozen runs and **false as a claim about the configuration**.

### Ground-truth position does not predict the refusal

Both refused legs had **zero** ground-truth exposure to the refusal band:
`c2n26_slow_r2/wall_adjacent` min d **0.259 m**, `c2n26_bslow_r3/
enclosure_entry` min d **0.269 m**. Legs that **SUCCEEDED** spent up to
**120.6 s** inside the band at 0.152 m (`c2n21_bbase_r2`, BASELINE).
Where the robot is does not decide it; where AMCL thinks it is does.

### The 0.259 m clearance is not a safe margin

It is a **centre-to-cell-centre** distance. At the deepest ground-truth
pose: centre → mapped wall face **0.234 m**, minus the measured
circumscribed radius **0.2051 m**, leaves **0.029 m** of body-to-wall
clearance (a lower bound; the physical surface may lie up to one 0.05 m
cell further, so read it as **0.029–0.079 m**).

### Why one refusal costs the tour

**Zero recovery behaviours ran after any refusal**, in any of the three
runs, although the BT is
`navigate_to_pose_w_replanning_and_recovery.xml` and the spin/backup
plugins are loaded and were used elsewhere in the same tours.
`bt_navigator` aborts ~40 ms after the refusal. *Why* the recovery
subtree does not run is **not established offline**; that it does not run
is observed.

### What remains unverified

Whether `slowdown_ratio = 1.0` contributes at all. One directional
signal exists and is weak: the baseline never crossed the wall_adjacent
goal line (5 of 5 stopped short), four of nine candidate runs crossed it,
by at most 0.076 m. No significance is claimed and none is computed. That
term is an order of magnitude too small on its own — the boundary needs
0.161 m of **estimated** overshoot.

C2-NAV.25's and C2-NAV.26's gates are not reopened, moved or recomputed
anywhere here.

### Exactly the next command to run

Still offline. Make the localisation term measurable instead of inferred
from two log lines per leg — `nav_bench.py` already subscribes
`/amcl_pose` (line 460) and never writes it out:

```bash
cd ~/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis
python3 docs/data/c2nav27_startocc.py decompose   # the two terms
python3 docs/data/c2nav27_startocc.py inventory   # the prior art
python3 docs/data/c2nav27_startocc.py bound       # the threshold bracket
```
