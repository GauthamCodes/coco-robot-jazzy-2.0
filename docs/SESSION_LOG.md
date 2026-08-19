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

- **`x11grab` captures a screen region.** Another Claude session's
  terminal raised itself over RViz and was scored as a framing result.
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
