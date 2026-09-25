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

## 2026-09-17 — C2-NAV.43: the command-path fix integrated from `main`, and an optional depth source measured

Branch `c2nav43-integration`, cut from `main` (`ea66155`), pushed to `jazzy2`,
**not merged**. Full report: `docs/agents/C2-NAV.43_RESULTS.md`.

**Built.**
- The C2-NAV.42 fix, integrated commit by commit after inspection: `d707327`
  wiring, `8bf1fe4` tests, `57f75d8` gz world-path quoting, the net of
  `ad2b8b8` (accepted nav2 defaults, sha256 `6f61e499…`), `9412719` docs,
  plus the tour tooling at `1235502` as files. No run data came across.
- `depth_cloud.launch.py`: `image_proc` resize x0.5 nearest, then
  `depth_image_proc` → `/camera/depth/points`, off by default
  (`nav.launch.py depth_cloud:=false`).
- A narrow `perception` experiment key and `verify-perception` in
  `nav_params_overlay.py`.
- Experiments `baseline_lidar_only.yaml` and `depth_fusion.yaml`, which
  differ only in `perception`.
- The instruments `docs/data/c2nav43_perception.py` (sensors | rates | capture
  | record | selftest), `c2nav43_compare.py` and `c2nav43_ramp.py`.

**Measured.**
- Tests 940 / 0 / 0 on the integrated tree and **975 / 0 / 0** on the final
  one.
- Live topology B: 13 / 13 chain links; a held raw 0.30 m/s gave wheels above
  the monitor 0 / 278, STOP held 0.249 m from the wall.
- Tours: A 7/7 (exceeded 0 / 2072); B 6/7 (bypass 0, stale drops 0).
- `/camera/points` is in the link convention under an optical frame_id
  (median error 0.0015–0.0020 m against the link projection, 0.64–0.73 m
  against optical).
- A full-resolution cloud (1.23 MB) reached a best-effort raw subscriber once
  in 12 s; the half-resolution one, 14.97 Hz.
- Capture: 0 phantoms at six poses in every arm; ramp coverage 0.086 → 0.904
  and 0.058 → 0.864.
- Part K (3 + 3 fresh, topology B): legs **16/21 → 18/21**, entry 1/3 → 2/3,
  exit 0/3 → 1/3, deadlocks 5 → 2, raw bypass 0 in all six. Off-geometry
  marks while driving **15,438 → 50,068**. Nav2 CPU 1.196 → 1.195 cores, plus
  0.125 for the depth nodes.

**Verdict.** The command-path fix is KEPT. Depth fusion is KEPT AS CANDIDATE,
not made default.

**Unverified.**
- M6 19/20 on the fixed path.
- Ramp navigation with fusion (no tour leg drives it).
- The cause of the 3.2× stale marks: consistent with a planar-LiDAR clearing
  asymmetry, not tested.
- The attribution of the trace residual (0.088–2.92 % per tour).
- N = 3 is not statistical.

**Traps paid for this session.**
- `ros_clean.sh`'s new `c2nav43_perceptio[n]` pattern matches any Bash call
  whose text names the instrument, so the session runner refused twice. Run
  the instrument in a command of its own.
- A shipped-params path containing `nav2_` in an instrument argument trips
  the same guard; use a copy named without it.
- `nav2_voxel_grid` marks a voxel with bits k AND k+16; the low bit alone
  means unknown. The first decoder put every column top at 0.8 m.
- A background tour sequence dies with the Claude session that launched it
  (`baseline_lidar_only_r03`, marked VOID). Launch long sequences with
  `setsid nohup`.

**Next command to run** (M6 on the fixed path, a fresh sim per run, both
terminals with `setup_env.sh` sourced):

```bash
ros2 launch gazebo_models full_world_robo.launch.py traverse:=true   # T1
ros2 launch coco_mission mission.launch.py rviz:=false              # T2
```

## 2026-09-17 — C2-NAV.44: M6 re-measured on the fixed command path

Branch `c2nav43-integration`, HEAD `c8a8206`, clean tree, no production file
changed. Full report: `docs/agents/C2-NAV.44_RESULTS.md`.

**Why.** `PROJECT_STATE.md` KNOWN LIMITATIONS 0 said M6's 19/20 was *not yet
measured* on the fixed path. It was measured with the `/cmd_vel_nav` loop in
place, so it could not be quoted as evidence for the shipping architecture.

**Built** (instruments only, all in `docs/data/`): `c2nav44_m6_run.sh` (the
C2-NAV.42 `live_mission.sh` with the fault injection removed — fresh sim,
bring-up checks, recorders, `/mission/start`, teardown),
`c2nav44_m6_run_traverse.sh` (the same with `executive:=false` +
`traverse_demo.py`), `c2nav44_m6_report.py` (offline, no ROS) and
`c2nav44_poserec.py` (`/amcl_pose` beside ground truth).

**Measured — six fresh executive-driven missions, one simulator each,
depth fusion off, never `--fast`:**
- **3 COMPLETE** (r01 red, r03 blue, r04 yellow) — 16 nominal states,
  `attempt=1`, `attempts={}`, 149.3 / 145.4 / 165.0 s sim.
- **3 ABORT, all green**, all `PRE_RAMP_POSE_OUT_OF_REGION`, before the climb.
- **Command path clean in all six:** raw-controller → wheel bypass **0**,
  wheels above the monitor **0** on Nav2-owned rows, stale command drops
  **0**, PolygonStop activations **0**, exactly one wheel publisher, live
  chain 12/12 OK, live nav2 parameter readback 0 mismatches.
- Grasp and approach held the historical bands: lift 35.3 / 34.6 / 35.6 mm,
  base-x 0.1540 / 0.1547 / 0.1546 — 3/3 inside the 5.5 mm window.

**The three aborts, diagnosed (not a command-path failure).**
`NAVIGATE_TO_RAMP` ended 0.3097 / 0.3115 / 0.3047 m from the pre-ramp goal by
ground truth, stopping within 6 mm of the same point across three fresh
simulators, while Nav2 logged `Reached the goal!` and `Goal succeeded` on
every attempt. Nav2's `SimpleGoalChecker` uses `xy_goal_tolerance: 0.25` on
the pose it steers by; the executive re-checks with `GOAL_XY_TOLERANCE =
0.25` on **ground truth**. Two 0.25 m tolerances from two different poses
leave zero margin. Both retries commanded **0.000 m/s** — the same
"structurally futile" retry the repo already documents for the yaw gate.
AMCL was not diverged: with `/amcl_pose` recorded, its error is 0.004–0.049 m
read right after each correction, `degraded=0` throughout. On red/blue/yellow
the estimate lags and the robot stops 0.056–0.147 m out; on green it leads.

**Comparability, and the harness run (`t02_green_traverse`).** 19/20 is
**not** a control: that matrix ran `traverse_demo.py`, whose `nav_to()`
returns on Nav2's `SUCCEEDED` alone and has no ground-truth arrival gate, so
this abort cannot occur in it. Run on this branch, same fixed command path,
fresh sim, `executive:=false` + `traverse_demo.py --colour green`: **all
seven steps, `outcome=held` base-x 0.1545, home to within 0.04 m, `FETCH
COMPLETE`, rc 0, 413 s wall** — the leg the executive rejected three times
was accepted and the mission finished. Command path on that run: bypass 0,
stale drops 0, STOP rows 0, wheels above the monitor 3/947 (0.32 %, inside
C2-NAV.43's unattributed residual). Its home leg took 190.7 s against the
historical single run's 103.6 s — N=1 vs N=1, reported, no causal claim.
**So the command-path fix does not break the fetch.**

**Unverified / not done.**
- Why the estimate's offset sign is lane-dependent. Not investigated: this
  sprint was forbidden from AMCL work, DWB tuning and goal changes.
- Three runs per lane is not a rate.
- No STOP hold occurred in any run, so this sprint adds no new PolygonStop
  evidence beyond C2-NAV.43's controlled test.
- Nothing was changed to improve the number: no gate, tolerance, goal,
  parameter or default was touched.

**Traps paid for.**
- `ros_clean.sh`'s `g[z] sim` pattern kills **every** Gazebo on the machine.
  An unrelated `eyantra_kepler_colony` simulator from `~/ros2_ws` started
  mid-run and the teardown killed it. The runner's refusal check only proves
  the machine was idle at the start.
- A 20 s watcher shelling out to `ros2 topic info` / `echo` creates DDS
  participants; with a second stack up, domain 0 ran out and the **next**
  process to start died with "Failed to find a free participant index for
  domain 0". That voided one run (`t01`), which was replaced.
- `/amcl_pose` publishes only on resample updates, so comparing it to ground
  truth at an arbitrary instant measures staleness, not localization error.

**Next command to run.** Reproduce the abort in ~25 s on one fresh simulator,
or re-run any colour:

```bash
bash docs/data/c2nav44_m6_run.sh ~/coco_nav_runs/c2nav44_m6/rNN_green green
```

---

## C2-NAV.45 — the pre-ramp arrival gate reports instead of retrying (2026-09-18)

Branch `c2nav43-integration`. Fix `56c324b`. A narrow mission-logic sprint:
C2-NAV.44 had already attributed its three green aborts to the executive's
arrival gate rather than to the command path, so this closed that one defect
and re-measured M6 on the green lane.

**What was built.** `mission_states._check_nav_leg` had one threshold and one
verdict, with `xy_tolerance` set to Nav2's own `xy_goal_tolerance` (0.25 m) so
as not to invent a second number — which is exactly what made it a gate with
**zero margin**: Nav2 stops when the pose it is *steering by* is inside 0.25 m,
the check then measured the *true* pose against the same 0.25 m, and any
localisation offset pointing away from the goal failed a leg the planner had
already declared finished. It now has three named bands: clean inside
`xy_tolerance`; **accepted, recorded in `arrival_discrepancy` and logged at
WARN** inside `xy_consistency`; hard failure beyond it, reason unchanged.
`mission_executive._log_event` prints the ground-truth error on every nav leg
and warns explicitly when Nav2 SUCCESS and ground truth disagree.

**The band is derived, not invented.** `GOAL_XY_CONSISTENCY = 2 x
GOAL_XY_TOLERANCE = 0.50 m`. Nav2 halts with its estimate inside 0.25 m, so a
true error past 0.50 m needs the estimate to be wrong by more than the whole
arrival window — a localisation failure, owned by the C2-M5 health monitor
already checked at the top of the same function. Run 15's 3.4 m divergence sits
far outside it. `xy_consistency == xy_tolerance` restores the old gate, and a
test asserts it. Applied in the shared helper, so `RETURN_HOME` gets it too:
the mechanism there is identical and leaving it out would knowingly keep a
futile retry behind.

**Measured — the old behaviour, re-derived this session.** Running the new
`docs/data/c2nav45_gate_report.py` over C2-NAV.44's committed run directories
reproduces its published numbers exactly, which is also the tool's validation:
true error **0.3097 / 0.3115 / 0.3047 m**, Nav2 `Reached the goal!` x3 in each,
7 region failures, 3 RECOVERY entries, 2 retries, and **max wheel speed after
the first stop 0.000 m/s in all three** — the retries provably could not move
the robot.

**Measured — three fresh green M6 missions** (one simulator each, headless,
never `--fast`, depth fusion off, no Nav2/goal/safety change, HEAD `56c324b`
with 0 dirty paths, C2-NAV.44's runner unmodified):

- **3 of 3 COMPLETE**, `result=fetch`, `reason=--`, `attempts={}`, all 16
  nominal states, **0 RECOVERY entries and 0 retries** in every run.
- **The original failure still occurs and is handled**: pre-ramp ground-truth
  error **0.348 / 0.315 / 0.316 m** — outside 0.25 m in all three, bracketing
  C2-NAV.44's range — accepted each time with an explicit WARN.
- Lift **36.0 / 35.4 / 34.9 mm**, `pick finished: held`, `place finished:
  placed`. Home to **0.070 / 0.038 / 0.022 m**, all clean at INFO.
- Pre-climb heading -0.160 / -0.209 / -0.132 rad, reported, gate still off.
- Command path: **bypass 0, wheels above the monitor 0, stale drops 0,
  PolygonStop 0** in all three. Runner 22 PASS / 0 FAIL each.
- Localization: **0 degraded samples, 0 relocalizations** over 6,317 samples.
- Wall time start to terminal: 825 / 450 / 424 s.

**Tests: 997 passing, 0 failing, 0 skipped.** Per package, cwd inside each
package, clean ROS graph. The branch's 975 plus **22** new in
`TestArrivalConsistency`, covering all three bands, the recorded r02/r05/r06
stop points, Nav2 failure with ground truth close, the absence of a second goal
after an accepted arrival, a full fetch from a discrepant arrival, and the
yaw/lane gates unchanged. Measured before and after the live sweep, identical.

**Unverified / not claimed.** AMCL is unchanged and the lane-dependent sign of
its offset is still undiagnosed. Localization recovery, the enclosure problem
and depth fusion are untouched. The collision monitor's 0.088-2.92 %
short-streak residual is not refuted by three clean runs. **Three runs of one
colour is not a rate** — this says the demonstrated abort no longer occurs, not
that M6 has a new success percentage. 19/20 remains not a like-for-like
comparison.

**Trap paid for.** Comparing Nav2's wall-clock log stamps against the
executive's *mission* clock made a healthy 165 s home leg look like a 398 s
overrun of a 240 s budget. `/mission/state` carries `elapsed=` and `timeout=`;
read those, not the difference between two nodes' log timestamps.

**Next command to run.** Re-measure the other three colours on the fixed gate,
to turn "the green abort is gone" into a fetch matrix:

```bash
bash docs/data/c2nav44_m6_run.sh ~/coco_nav_runs/c2nav45_m6/r04_red red
```

---

## 2026-09-18 — C2-NAV.46: the M6 fetch colour matrix

**Branch** `c2nav43-integration`, sweep at `ff98171`/`e73494d`, clean tree,
`dirty_paths=0` recorded in all nine new runs.

**What was built.** Nothing in the runtime. Two offline tools:
`docs/data/c2nav46_matrix_sweep.sh`, the nine-run driver, and
`docs/data/c2nav46_matrix_report.py`, a composer over the two existing
per-run reports (`c2nav44_m6_report.py`, `c2nav45_gate_report.py`), both
left untouched so C2-NAV.44's, C2-NAV.45's and this sprint's numbers stay
directly comparable. Colours were **interleaved by round**, not grouped, so
two hours of machine drift is not confounded with colour.

**What was measured.** Three fresh executive-driven missions each for red,
blue and yellow, fresh simulator per run, headless, never `--fast`, depth
fusion off, no Nav2/goal/planner/controller/safety change. Green's three
C2-NAV.45 runs carried over unchanged.

- **11 of 12 fetches. 12 valid runs, 0 void.** red 3/3, green 3/3,
  blue **2/3**, yellow 3/3.
- **The gate generalises.** All 12 passed the pre-ramp gate, the outer
  0.50 m band was **never reached**, and futile retries were **0 everywhere**.
- **The green discrepancy is lane-specific.** green 0.315-0.348 m uses the
  consistency band in all three runs; red 0.108-0.131 m, blue 0.066-0.085 m
  and yellow 0.030-0.047 m are **clean inside the original 0.25 m tolerance**
  in all nine. Red, blue and yellow would all have passed the *old* gate.
- **Command path clean in all 12:** bypass **0**, stale drops **0**.
- Wheels above the monitor **4 of 9744** nav-active samples = **0.0411 %**,
  worst gap 0.0316 m/s.
- Grasp **12/12** inside `[0.1510, 0.1565]` (0.1534-0.1547 m), lift
  34.5-36.4 mm, across four cylinder radii. Not colour-sensitive.
- Tests **997 passed, 0 failed, 0 skipped**, measured after the sweep.

**The one failure.** `r2_blue`, ABORT `RETURN_FAILED`, a **valid** run (22/22
checks, clean shutdown), classified **navigation**: pre-ramp gate was clean at
0.066 m, the pick succeeded (lift 36.2 mm), and the mission died on the way
home. PolygonStop held the robot **595.5 s**, 5920 rows inside `RETURN_HOME`;
`min_scan_m` 0.15 m; 40 `controller_failed_progress`, 38 costmap clears,
spin x9 / wait x9 / backup x6; ended at world (0.17, 0.35). AMCL was
**CONSISTENT 6873/7486, 0 degraded**, final gap 0.113 m, so not localisation;
bypass 0, so not the command path. The documented `enclosure_entry`/PolygonStop
deadlock class, on the return leg, **intermittent** — the same lane completed
cleanly twice with PolygonStop 0.

**Unverified / not claimed.** Three runs per colour is **not a rate**; 11/12
is not a 92 % reliability figure and blue 2/3 is one failure, not a
blue-specific failure rate. The `r2_blue` deadlock is **classified, not
diagnosed** — no root cause, no fix proposed. The wheels-above-monitor
residual stays **unattributed**. Depth fusion stayed off and is not mixed in.
The per-package test split (notably `gazebo_models` 171 vs the release
table's 41) does not match `CLAUDE.md`'s release baseline though the 997 total
does; not investigated.

**Trap paid for.** The matrix composer, run against the three known-good green
runs *before* any new data existed, scored all three VOID and one as
non-nominal. Both were the tool's fault: `process_died` also catches the
**teardown**, where every launched process dies by design, and the 10 Hz
`state_path_sim` sampler drops `LOCALIZE`, which lasts ~0.1 s. Validate a new
report against a known-good run before trusting it on new runs.

**Next command to run.** The matrix is clean, so the next step is integration,
not another M6 experiment. Review what the branch carries ahead of `main`:

```bash
cd ~/ros2_ws/src/coco-robot-ros2 && git log --oneline main..c2nav43-integration
```

## 2026-09-19 — C2-NAV.47: the C2-NAV.43–46 work prepared as the new main baseline

**Integration sprint, not an investigation.** No runtime code changed. Full
write-up in `docs/agents/C2-NAV.47_RESULTS.md`.

**Built.** Nothing in the runtime. This sprint adds the results document, the
missing `docs/data/README.md` index rows for evidence already committed
without them, this checkpoint, and its own regression readbacks under
`docs/data/c2nav47_live/`.

**One defect the merge itself would have introduced, fixed.**
`c2nav45_m6_sweep.sh` and `c2nav46_matrix_sweep.sh` hardcoded
`WT=<...>/.claude/worktrees/c2nav43-integration` — a path that stops existing
the moment the branch merges. Both now use the self-locating idiom
`c2nav44_m6_run.sh` already used, verified to resolve two directories up to
the repo root and to stay overridable by `COCO_WT`. Their `ROOT` defaults
moved from `/home/gautham/...` to `$HOME/...`.

**Still hardcoded, reported not changed.** `c2nav44_m6_run.sh:41` and
`c2nav44_m6_run_traverse.sh:46` set
`MV=/home/gautham/ros2_ws(personal)/moveit_prefix/...`. That path is the real
workspace's, not the worktree's, so it survives the merge and nothing breaks.
Deriving it would need a live run to validate, and these scripts are the
recorded provenance of committed evidence — not worth changing on a sprint
that is meant to change no behaviour.

**Measured.**
- **Ancestry, checked not assumed:** `main` is `ea66155` and **has not
  moved**; the merge base of `main` and `c2nav43-integration` **equals
  `main`**, so the branch is a strict descendant and **a fast-forward is
  available**.
- **Clean build 9/9, exit 0**, from a wiped `build/ install/ log/`.
- **Tests 997 passed, 0 failed, 0 skipped**, per package, cwd inside each.
- **M6 green: COMPLETE, `result=fetch`**, one fresh executive-driven mission
  at `aa5b968`, `dirty_paths=0`.
- **The arrival gate used both bands in that one run:** pre-ramp 0.290 m →
  WARN, accepted, **no retry**; `RETURN_HOME` 0.108 m → INFO, clean.
- **Command path, mission:** raw-controller → wheel bypass **0** (0/78 bypass
  rows, 0/498 nav-owned). Smoother followed on 90 of 109 rows where raw ≠
  smoothed, 1 raw-only. Monitor SLOWDOWN 18 / LIMIT 5, obeyed.
- **Command path, controlled STOP experiment:** `stop_held: true`, **69 STOP
  rows, 0 with the wheels driven**; smoother **9 of 9**, `wheel_eq_raw_only`
  **0**; monitor authority 252 samples **0 exceeded**. Robot halted with
  `min_scan_m` **0.342 m** while the probe still commanded a raw 0.30 m/s.
- **Arbiter is the sole wheel publisher:** `Publisher count: 1`,
  `cmd_vel_arbiter`, read off the live graph.
- **Depth fusion off, read back live:** no depth-cloud process, all four
  costmap observation sources `scan`.
- **The two production config lines** are the NavigateThroughPoses tree and
  local `cost_scaling_factor` 5.0 → 65.0. `BaseObstacle.scale` was **already
  8.0**; the **global** costmap stays at 5.0; the collision-monitor block is
  **byte-identical to `main`**.

**Resolved, from the last checkpoint's open list.** The per-package test split
*does* reconcile with `CLAUDE.md`'s release baseline — it was new tests, not a
discrepancy. `gazebo_models` 41 + **130** (`test_cmd_vel_wiring` 25,
`test_nav2_params_guard` 7, `test_nav_params_overlay` 63,
`test_perception_experiments` 35) = **171**; `custom_teleop` 67 + 8 = 75;
`coco_mission` 281 + 30 = 311. 829 + 168 = **997**. Nothing is missing.

**Also measured, and worth knowing before trusting a re-run.** The committed
run directories are **slimmed**: re-running `c2nav45_gate_report.py` against
`docs/data/c2nav45_live/r01_green` prints `no arrival recorded`. The report
*outputs* (`gate_report.json`, `matrix.json`, `matrix_report.txt`) are the
committed artefact. Now said plainly in `docs/data/README.md`.

**Unverified / not claimed.** The green pre-ramp discrepancy is now
**0.290–0.348 m over four runs** — this run's 0.290 m is *below* the
previously measured 0.315–0.348 m range. Four runs is **still not a rate**.
The `gated_zero_moving` residual (3 of 498 nav rows, 0.60 %, worst wheel
0.0158 m/s) sits inside the documented 0.088–2.92 % band and stays
**unattributed**. The `r2_blue` return-leg PolygonStop deadlock (595.5 s)
remains **classified, not diagnosed** — untouched here by instruction. Depth
fusion remains a **candidate**: not re-benchmarked, its ~3.2× stale-mark
result and unvalidated ramp driving stand. `coco_web` exits **5**, not the
**4** `CLAUDE.md` records; a documentation nit, left alone rather than edited
silently.

**Next command to run.** The branch is ready and `main` has not moved, so the
merge is a fast-forward. **The human performs it**; nothing here modified
`main`.

```bash
cd ~/ros2_ws\(personal\)/src/coco-robot-ros2
git checkout main
git merge --ff-only c2nav43-integration
```

---

## 2026-09-19 — C2-NAV.48: the branch merged to main, and the blue return-leg deadlock diagnosed

**The merge happened.** `main` `ea66155` → **`1425e6c`**, fast-forward, one
parent, **no merge commit**; `ea66155` verified as an ancestor. The previous
entry's "next command" was exactly this, and it is now done rather than
pending. Zero untracked collisions: the five tracked `C2-NAV.4x_RESULTS.md`
files have different names from the four local `docs/agents/` scaffolding
files, so `docs/agents/` now holds all nine. Local-only state (`.codex/` at
4529 files, `AGENTS.md`, `docs/RSE_ASSIGNMENT_PLAN_V2.md`, the four scaffolding
files and `tatus --short`) was md5-verified into
`~/coco_premerge_backup/20260919_033040/` first and verified intact after.
`tatus --short` is **not** junk — it is a captured `git diff` of the local
`CLAUDE.md` change, and it was kept.

**`CLAUDE.md` conflicted, and was reconciled rather than resolved one way.**
The stashed local version replaces the whole document with a ten-line pointer
to `AGENTS.md`; applied onto the new `main` it would have deleted 288 lines
including the entire C2-NAV.43–47 record. Kept both: the integrated content in
full **plus 14 lines, 0 deletions** adding a "Multi-agent protocol" section
pointing at `AGENTS.md` and `docs/agents/`. Left **uncommitted**, as it was
before the merge, and the stash was left on the stack (`apply`, never `pop`).

**Clean build 9 of 9, exit 0. Tests 997 → 1001, 0 failed, 0 skipped.** The
clean-build wipe was scoped to the nine packages under test rather than all of
`install/`: `<ws>/install` also holds `turtlebot3_*` prefixes whose
`local_setup.bash` is already missing and which `turtlebot3_node` cannot
rebuild, so a full wipe would have destroyed an unregenerable install. Every
package under test still built from scratch.

**C2-NAV.46's `r2_blue` deadlock is diagnosed.** Root cause: **the local
costmap and the collision monitor disagreed about which poses are navigable.**
`cylinder_obstacle` (a static model at (−0.2, 0.6), r 0.2, h 0.6) had its
surface **0.2486 m** from `base_footprint` — **1.4 mm inside** PolygonStop's
0.25 m circle, and **43 mm outside** the costmap's real inscribed radius of
**0.2060 m**. The planner scored that pose **15.8 of 254**; the monitor held the
wheels. The obstacle is identified to **0.9 mm**: predicted laser range to its
surface 0.3657 m against `scan_min` 0.3666 m, the difference explained entirely
by `LIDAR_MOUNT_XYZ = (-0.09, 0.10, 0.20)`, a 0.1345 m offset.

**Why nothing escaped, measured.** PolygonStop is a *circle* with
`action_type: stop`, so it is direction-agnostic. Across the 5955 held rows Nav2
commanded motion in **5423**, including **905** rows of `spin` (`w=+1.0`) and
**603** rows of `backup` (`v=−0.15`). **The wheels moved in 0.** Both escape
primitives were issued and both were vetoed identically. True chassis clearance
was **49 mm** — the robot was never in contact and could have moved.

**The first divergence is 23.8 mm.** `r3_blue` passed the same obstacle at
0.2724 m and completed; `r2_blue` passed at 0.2486 m and held 595.5 s.

**Fix, one parameter:** `local_costmap.robot_radius` **0.20 → 0.25**, so the
inflation layer's inscribed radius goes 0.205965 → **0.255004 m**, 5.0 mm past
the stop circle, and `r2_blue`'s pose becomes cost 253. Verified in Nav2's
source, not assumed: `BaseObstacleCritic::isValidCost` rejects
`INSCRIBED_INFLATED_OBSTACLE` and `scorePose` throws
`IllegalTrajectoryException`, and that critic scores the **centre** cell — the
same `base_footprint` origin PolygonStop measures from. **`PolygonStop` is
untouched**, honouring C2-NAV.6's ruling that neither of its knobs should move;
this generalises C2-NAV.7's accepted goal stand-off to the *transit* poses a
stand-off cannot reach. The inscribed-radius model reproduces **C2-NAV.0's
measured 0.205879 m to 0.086 mm**, and a test guards that.

**The global costmap is deliberately left at 0.20, and that is measured.** At
`cost_scaling_factor` 5.0 it already prices `r2_blue`'s pose at **203.6 of
254** and avoids the band unaided; the local costmap's 65.0 prices the identical
pose at **15.8**. The defect is local, so the fix is local. A test records it.

**Validated: 6 fresh missions, 6/6 `result=fetch`** — three blue plus red,
green and yellow smoke. **PolygonStop rows 0 in all six.** Closest approach to
`cylinder_obstacle` across them was **0.2905 m**, 40.5 mm outside the stop
circle. Command path: bypass **0** in all six, PolygonStop rows with wheels
driven **0** in all six. Mission regression went the *good* way:
`controller_failed_progress` **0** in all six against 40 in `r2_blue`, goals
aborted 0, RECOVERY entries 0. Arrival gate clean (0.094/0.108 m pre-ramp,
0.062/0.014 m return, **0** WARN-band entries); grasp base-x 0.1539 and 0.1545,
inside the measured window. 0 orphan processes, and all seven run directories
record `ros_clean: 0 matched, 0 still running`.

**Two harness defects had to be fixed before anything could run.** (1) The three
live-run scripts conflated the repo root with the colcon workspace root and
refused to start on `main`; worse, a **stale `<repo>/install` from 2026-07-27
holding one package, `coco_rl`**, did source successfully and layered a
seven-week-old build over the fresh one — the refusal is the only reason that
was not a silent wrong-overlay run. `WS` is now derived as `setup_env.sh`
derives it, overridable by `COCO_WS`. (2) **`<ws>/install` cannot launch Gazebo
at all**: its half-installed `turtlebot3_*` prefixes do not resolve, and
`ros_gz_sim`'s `GazeboRosPaths.get_paths()` enumerates every package in the
index, so one bad entry kills the launch. Measured: `<ws>/install` 1
unresolvable entry, an isolated overlay **0 of 490**. Runs used
`$HOME/c2nav48_overlay`.

**Unverified / not claimed.** **The deadlock was NOT reproduced on unmodified
runtime.** The base rate is 1 in 12, and the three runs that executed were
already on the fixed value — the overlay is `--symlink-install`, so the
installed `nav2_params.yaml` symlinks to source, which was edited at 22:30:50
UTC before Nav2 loaded parameters at ~22:31:36. So KEEP rests on the root cause
measured from C2-NAV.46's own trace, the mechanism verified in Nav2's source,
and six post-fix missions without regression — **not** on an A/B against a
reproduced failure. Six runs is **not a rate**. `robot_radius` is now in
`nav_params_overlay.py`'s `LIVE_CHECKS` for both costmaps so no future run has
to infer it. The **rasterisation residual stands**: at 0.05 m resolution the
inscribed boundary is about one cell accurate, so this is a large reduction in
exposure, not a proof. One run was **VOID** — `nav2_container` died in
bring-up to a SIGSEGV reported by ImageMagick's handler, caused by running the
runner under `env -i` (3 of 3 aborts, against 0 of 3 for the C2-NAV.46 worktree
runs, and bring-up slowed 19 s → 2 min 42 s); isolating only the ROS/colcon
variables fixed it. **Do not run these harnesses under `env -i`.**
`gated_zero_moving` (0–34 rows, worst wheel ≤ 0.0199 m/s) stays inside the
documented 0.088–2.92 % band and **remains unattributed**. One
`monitor exceeded` row appeared in two of six runs and 0–6 smoother raw-only
rows across them; reported, no mechanism claimed. `PROJECT_STATE.md` and
`CLAUDE.md` still say this deadlock is "classified, not diagnosed" and need
updating — not edited here because the tree carries an unrelated local
`CLAUDE.md` change.

**Next command to run.** Re-measure the colour matrix on the fixed
configuration, to replace C2-NAV.46's 11-of-12 with a figure measured on
`robot_radius` 0.25:

```bash
cd ~/ros2_ws\(personal\)/src/coco-robot-ros2
COCO_WS=$HOME/c2nav48_overlay \
  bash docs/data/c2nav46_matrix_sweep.sh ~/coco_nav_runs/c2nav48_matrix
```

---

## 2026-09-19 — C2-NAV.49: C2-NAV.48 integrated, and the colour matrix re-measured on `robot_radius` 0.25

**What was built.** Nothing new in the runtime. C2-NAV.48 was integrated onto
`main` and its fix validated across all four lanes. Branch
`c2nav49-integration` from `main` @ `1425e6c`, fast-forwarded to `04f9711`:
C2-NAV.48 is a strict two-commit descendant, `ea66155` is an ancestor, and
inspecting both commits showed nothing experiment-only in runtime code, so a
selective cherry-pick would have changed the tree for no reason. Added:
`docs/data/c2nav49_matrix_sweep.sh` (four colours x three rounds, interleaved),
`docs/data/c2nav49_clearance.py` (the deadlock-mechanism reader), the
`ros_clean.sh` scope fix and its three tests, and
`docs/agents/C2-NAV.49_RESULTS.md`.

**What was measured.** **12 of 12 fetches, 12 valid runs, 0 void — red 3/3,
green 3/3, blue 3/3, yellow 3/3.** All four colours re-run with none carried
over, because a changed costmap parameter invalidates every lane. Fresh
simulator per run, headless, never `--fast`, depth fusion off, `dirty_paths=0`,
**22 runner checks passed / 0 failed** in all twelve, 16 nominal states, 0
re-entries, Nav2 goals 2 succeeded / 0 failed / 0 aborted, 0 recoveries, 0
relocalizations, clean shutdown. Live readback confirmed
`local_costmap.robot_radius` **0.25** and `global_costmap.robot_radius`
**0.20** in every run, with PolygonStop 0.25 / 4, CSF 65 / 5, BaseObstacle
8.0 and the NavigateThroughPoses tree unchanged.

**The blue deadlock did not recur.** Over all twelve traces: PolygonStop rows
**0**, episodes **0**, duration **0.0 s**, stop-with-wheels-driven **0**,
`controller_failed_progress` **0** (C2-NAV.46's `r2_blue`: 40), costmap clears
**0**. Closest approach to `cylinder_obstacle` in any run **0.2894 m**
(`r1_blue`, return leg), **39.4 mm outside** the 0.25 m stop circle. Blue is
the exposed lane by geometry — **0.2894 / 0.4024 / 0.3335 m**, all three on
the return leg — against green 0.4054–0.5343, yellow 0.4532–0.4628 and red
never closer than **0.6214 m**. Grasp 12 of 12 inside `[0.1510, 0.1565]`
(0.1535–0.1547 m), lift 34.1–36.6 mm, mission 145.6–192.4 s sim. Tests
**1004 / 0 / 0** (`gazebo_models` 178). 9/9 packages built, colcon exit 0.

**Unverified / not claimed.** **The fix's mechanism was never exercised.** It
works by making the 0.2059–0.25 m band inscribed-lethal, and **no run entered
that band**: 0.2894 m is outside even the *old* 0.205879 m inscribed radius,
so none of these twelve would have deadlocked on 0.20 either. This matrix is
**not an A/B of the fix** — it shows no recurrence and no regression on the
corrected value, nothing more. KEEP still rests where C2-NAV.48 put it: a root
cause measured from C2-NAV.46's own trace and a mechanism verified in Nav2's
source. **Twelve runs is not a rate; 12/12 is not a 100 % figure.**
`min_scan_m` is useless for clearance here — it saturates at the 0.15 m LiDAR
floor in all twelve, so the ground-truth geometry is the only measure.
**Wheels above the monitor is 8 of 7,781 nav-active samples = 0.1028 %**,
worst gap 0.1250 m/s — *higher* than C2-NAV.46's 0.0411 %, inside the
documented 0.088–2.92 % band, still **unattributed** and **not** claimed fixed
or improved. Green's pre-ramp discrepancy **reproduces** (0.298 / 0.307 /
0.330 m against C2-NAV.46's 0.315–0.348) and is still **green's alone**; the
outer 0.50 m band was never reached and futile retries were 0 in all twelve.
`r1_red` ran at `3a22201` and the other eleven at `553f219`; the only
difference is an offline reader, no runtime change.

**Two environment findings, both re-measured rather than inherited.**
(1) `<ws>/install` still cannot launch Gazebo: **2** unresolvable ament-index
entries — `red_ball_nav` and `turtlebot3_teleop` — against **0 of 462** for an
isolated overlay. C2-NAV.48 measured 1; it is 2 today. Runs used
`$HOME/c2nav49_overlay`, selected with `COCO_WS`. This is the user's
environment and is reported, not changed. (2) `coco_world.world` is **not
well-formed XML** — its prose comments contain `--` (`--randomize`,
`--target`), which XML forbids — so ElementTree refuses the raw file while gz
parses it happily. `c2nav49_clearance.py` strips comments before parsing
rather than editing a world frozen as `world_v1`.

**`ros_clean.sh` no longer sweeps other people's simulators.**
`'g[z] sim'` → `'g[z] sim.*gazebo_models/worlds'`. C2-NAV.44 measured the cost
of the bare pattern: an unrelated `eyantra_kepler_colony` simulator from
`~/ros2_ws` started mid-run and the teardown killed it. **Scoping the sweep to
the current experiment was rejected**, not overlooked — this file exists to
kill orphans of *previous* runs, which are never in the current process group,
and a session-scoped sweep could not kill one of them. Every coco simulator
still matches, in both `gui` modes. Three tests assert **both** directions,
because a test that only checked the foreign simulator survives would also
pass on a pattern matching nothing — a typo that silently disarms the sweep.
Against the pre-fix script in an isolated copy: **2 failed, 1 passed**, the
pass being the positive control. Against the fixed script: **3 passed**.
**Not fixed, not claimed:** a hand-started `gz sim -g` carries no world path
and is not swept; no launch file here starts one.

**Stale artifacts removed.** `<repo>/install` (148K, 2 packages, newest file
2026-07-27 21:35) and `<repo>/build` (56K, 5 package dirs, newest 2026-07-28
16:35) in the main checkout — both `COLCON_IGNORE`d so colcon never refreshed
them, both holding a seven-week-old `coco_rl`, both existing only to be
sourced by mistake. **`<ws>/install` was NOT touched**: it carries the
unrelated turtlebot3 and `red_ball_nav` prefixes, 19 before and after. This
worktree's own `install/`+`build/` (2026-09-19 02:55) were left in place —
gitignored build output, not what C2-NAV.48 flagged, and nothing sources them
now that the runners take their overlay from `COCO_WS`.

**Next command to run.** The matrix is clean and the COCO core is stable
across all four lanes, so the next step is productization, not another
C2-NAV investigation. Merging is the owner's call:

```bash
cd ~/ros2_ws\(personal\)/src/coco-robot-ros2
git checkout main && git merge --ff-only c2nav49-integration
```

---

## 2026-09-20 — P0.2, the platform becomes usable

**Built:**

- `coco_web/mission_view.py` — the executive's state, translated. Reads
  all **twelve** fields `/mission/state` carries; P0.1 read two and
  looked for three (`colour`, `target`, `detail`) that the line has never
  contained, so the mission colour on the wire was permanently null.
  Carries both vocabularies: `phase` (ten product words) and `state` (the
  executive's own name). `RECOVERY`/`RELOCALIZE` keep the phase of the
  state they are retrying; `ABORT` with `OPERATOR_ABORT` is STOPPED, not
  FAILED. Constants duplicated from `coco_mission` (importing it would
  close a cycle) with an `ast` drift test in **both** directions.
- `telemetry.parse_grasp_status` — `/grasp/status` is the one status
  topic whose values contain spaces (`phase=pick:hover above target`), so
  `parse_kv` silently truncates it. Slices to the next known key instead.
- `coco_web/streams.py` — per-client subscriptions, rates and the bounded
  queue. `subscribe`/`unsubscribe`/`set_stream` honoured. Default set is
  P0.1's, which is what let the protocol stay `coco.v1`.
- `coco_web/binary.py`, `imaging.py` — self-describing binary frames for
  LiDAR, camera and depth. No ROS message on the wire; no topic in any
  header. Nine malformed shapes tested.
- `coco_web/metrics.py` — measured rates, drops, CPU, mission latency, at
  `/api/metrics` and in telemetry.
- `session.py` — a second axis: `lifecycle` (CREATED…FAILED) beside
  `state` (readiness), so `/healthz` stays 200 during a mission. Plus
  `connection`, and pilot/viewer drive arbitration.
- `web/index.html`, `app.js`, `style.css` — Play and Engineering modes
  rebuilt. The world view draws the real ramp, platform and target lanes
  from `coco_config`. The hard-coded phase list and interpolated
  percentage are **deleted**.
- `gazebo_models/scripts/ros_clean.sh` — gained `platform_serve[r]`.

**Measured (this session, one machine, one sitting):**

- **A complete green fetch driven entirely through the browser
  protocol**: all 16 states in order, `result=fetch`, **170.4 s**.
- Telemetry **1 714 frames, 0 dropped**, peak socket buffer **0 B**.
- Mission-state latency **18.4–82.7 ms** (1 714 samples in that run).
- LiDAR binary frame **668.8 bytes** mean, **10.0 Hz**.
- Camera **3 467 bytes** mean JPEG (q60, 320×240), **6.17 fps** under a
  10 fps cap. Depth **19 frames in 5 s** with `depth_topic` set, **0**
  without it.
- Platform CPU **67–75 % of one core**.
- `/diff_drive_controller/cmd_vel` **publisher count 1** (`cmd_vel_
  arbiter`); the platform's only velocity publisher is `/cmd_vel_teleop`;
  `-p teleop_topic:=/diff_drive_controller/cmd_vel` still refuses to
  start with `UnsafeTopicError`.
- Drive path: browser `drive` moved the wheels (40 commands, max
  0.15 m/s); `stop` zeroed them; a second client's `drive` refused
  `not_in_control` while its **STOP was honoured and reached the
  wheels**; disconnecting the last client ended stopped. Positive control
  honoured — the recorder saw **97** wheel commands.
- Compatibility, both directions: a text-only client received **51 JSON
  scans and 0 binary frames**; a binary client **50 binary frames and 0
  duplicate JSON**.
- Tests **1334 passing, 0 failing, 0 skipped** (was 1139). `coco_web`
  116 → 291, `gazebo_models` 178 → 181. Clean 9/9 build.

**Unverified:**

- **The browser was never driven.** The Chrome extension was not
  connected on this machine. The page is covered by static asset tests
  (73 element ids used, 73 present; every frame type it sends is in the
  server's schema; no ROS topic string in it) and by a WebSocket client
  exercising the same server paths. **Rendering, layout and interaction
  are unverified.**
- **Docker, still.** Not installed. No port and no dependency was added
  by P0.2. `docs/DOCKER.md` carries the exact procedure.
- Camera/depth behaviour with **two simultaneous viewers** — the
  shared-encode path is written and unit-tested but was never exercised
  by two real clients at different settings.

**Found, and fixed, by the live run:**

- **P0.2's own bug.** `wants()` gated binary delivery on
  `BINARY_STREAMS` (camera, depth) while `push_sensors` also framed
  **lidar** as binary — so a client declaring `binary: false` got binary
  lidar frames, which is exactly the compatibility guarantee the design
  claims. Surfaced as a `UnicodeDecodeError` in a probe calling
  `json.loads` on bytes. Split `BINARY_CAPABLE` from `BINARY_STREAMS`;
  six tests pin it.
- **A P0.1 gap.** `ros_clean.sh` had no `platform_server` pattern — the
  node was added to a launch file and not to the sweep, the same rule
  `mission_hud` already broke. An orphan holding :8080 was observed and
  the next launch died `Address already in use`.

**Open:**

- **One of two mission attempts aborted**, reaching `RETURN_HOME` (13 of
  16 states, including a verified grasp) and then failing
  `RETURN_FAILED` with `planner_server: GridBased plugin failed to plan
  from (7.97, 1.15) to (0.00, 0.00): "Start occupied"` — the robot's
  believed pose inside an occupied cell after the descent. A
  localisation outcome upstream of the web layer, which publishes no TF
  and no goals during a mission. **Two runs is not a rate** and this does
  not re-measure M6.
- **`/healthz` 200 does not mean localised.** Two missions started in
  that window aborted instantly with `NAVIGATION_FAILED` and
  `bt_navigator` logging *"Initial robot pose is not available"*. The
  bring-up script now waits for `map → odom`; the platform does not, and
  arguably should expose that wait.
- **AMCL dropped every scan** in one bring-up —
  *"timestamp earlier than all the data in the transform cache"* —
  alongside `robot_state_publisher: Moved backwards in time`. One
  `/clock` publisher, one bridge, sim time advancing, so **not** the
  documented stale-clock failure. Not diagnosed further; out of scope.
- `<ws>/install` still cannot launch gz — the half-installed
  `turtlebot3_teleop` (egg-link, no package marker) makes
  `GazeboRosPaths.get_paths()` throw. P0.2 worked around it with a fully
  isolated overlay at `/home/gautham/coco_p02_overlay`, built with
  `AMENT_PREFIX_PATH` unset first, because a login shell on this machine
  leaks another workspace onto the path and colcon bakes that chain into
  the overlay's own `setup.bash`.

**Next:**

```bash
# One command. It now sanitises AMENT_PREFIX_PATH first (a stray
# half-installed package on it kills every gz launch) and waits for
# map->odom after /healthz, because 200 does not mean localised:
cd <repo> && ./scripts/run_platform.sh --native

# then OPEN http://localhost:8080 IN A BROWSER and verify the UI --
# the one thing P0.2 could not check. Specifically:
#   Play mode draws the ramp, platform and four target lanes
#   the joystick drives and STOP halts
#   "show camera" starts frames; unticking stops them
#   picking a colour and pressing Start advances the step counter
#   switching to Engineering shows in/out rates and drops
```

If the simulator will not start, the cause is almost certainly a
half-installed package on `AMENT_PREFIX_PATH` rather than anything in
this repo — check with `printf '%s\n' "$AMENT_PREFIX_PATH" | tr : '\n'`
and look for a prefix whose `share/ament_index` is missing.

### Addendum, same session — two bring-up bugs found by running the entry point

Both pre-existing in P0.1's `scripts/run_platform.sh` and
`docker/entrypoint.sh`, both found by actually invoking the documented
command rather than reading it.

1. **`AMENT_PREFIX_PATH` was inherited.** ros_gz_sim's
   `GazeboRosPaths.get_paths()` enumerates every package on it, so one
   half-installed entry anywhere — here a stray `turtlebot3_teleop`, an
   egg-link with no package marker — killed every gz launch with
   *"package 'turtlebot3_teleop' not found"*. Two bring-ups were lost to
   it before the error was recognised, because it names a package this
   repo does not use.

2. **`| grep -q` under `set -o pipefail` fails when it MATCHES.**
   `grep -q` exits on the first match, closing the pipe; `ros2 topic
   info` then dies of EPIPE (Python exits **120**) and `pipefail`
   propagates that. **Measured: exit 0 without pipefail, exit 120 with
   it, on the same matching input.** So the readiness wait never broke
   out: `run_platform.sh --native` sat at `[coco] simulator…` for
   **13+ minutes** with a robot that had been publishing odometry the
   whole time. `docker/entrypoint.sh` had the identical line in the
   function that sequences the simulator before the mission stack.

   **After dropping `-q`: 15 seconds** from simulator launch to mission
   stack launch (09:58:59 → 09:59:14), then `/healthz` 200. The fix is
   `| grep PATTERN >/dev/null`, so grep drains the stream and the writer
   never sees EPIPE.

   The localisation wait added earlier in this same session had the bug
   too, copied from the line above it — which is the argument for
   `coco_rl/test/test_platform_scripts.py` asserting no `| grep -q`
   survives in a script that sets pipefail, rather than a comment.

**Verified after the fixes:** `./scripts/run_platform.sh --native`
launches the simulator, waits 15 s, launches the mission stack, reports
`drivable` at `/healthz` 200 and then waits for `map -> odom`.

## 2026-09-22 — Clean COCO runtime: `turtlebot3_teleop` was never a COCO dependency

Branch `coco-clean-runtime`, from `p02-browser-experience` @ `8991249`
(the platform the success condition needs — `mission.launch.py
platform:=` and `:8080` — exists only there; `main` d317d85 has the old
rosbridge panel). No Nav2, PolygonStop, DWB, costmap, AMCL, goal or
depth-fusion change.

**The symptom, reproduced first.** A `bash --noprofile --norc` started
from the developer's terminal, `source /opt/ros/jazzy/setup.bash`, the
main checkout's `setup_env.sh`, then the user's exact command:
`ros2 launch gazebo_models full_world_robo.launch.py traverse:=true
gui:=true` → exit **1**, `package 'turtlebot3_teleop' not found`.

**Root cause — environmental, not in this repo. Measured:**

1. **No COCO → TurtleBot edge exists.** Every `package.xml`, every launch
   file, `setup.py`, `CMakeLists.txt` and YAML value was audited; the
   only mentions are comments (nav2_params.yaml's provenance). Every
   package any COCO launch file looks up is declared.
2. **The trace:** `full_world_robo.launch.py:102` includes ros_gz_sim's
   `gz_sim.launch.py`, whose `launch_gz` (an `OpaqueFunction`, line 180)
   starts with `GazeboRosPaths.get_paths()`. That lists every package on
   `AMENT_PREFIX_PATH` (`ros2pkg.api.get_package_names` →
   `ament_index_python.get_resources`, `os.listdir`) and resolves each
   (`get_package_share_directory` → `get_resource`, `os.path.isfile`). A
   marker that is a **dangling symlink** is listed and not resolvable.
3. **`<ws>/install` put 2 such markers on the path** (colcon's own
   `_local_setup_util_sh.py`, asked directly): `turtlebot3_teleop` and
   `red_ball_nav`, both `--symlink-install` markers pointing into
   `/home/gautham/ros2_ws/build/...` — the workspace's pre-rename path.
   Installed 2026-07-29 / 07-21 and never rebuilt; the COCO packages were
   rebuilt 2026-09-19 and re-pointed. colcon orders `turtlebot3_teleop`
   first, so it is the one named. This matches C2-NAV.49's count of 2.
   (My first replay said 8; it added every install dir with a `share/`,
   which local_setup does not. Retracted.)
4. **Correction to the repo's notes:** "an egg-link with no package
   marker" was wrong. A prefix with NO marker is never listed and is
   harmless — `turtlebot3_node` / `turtlebot3_example` sat in the same
   install with none. Pinned in `test_no_turtlebot_dependency.py`.
5. **Two contamination paths from `$HOME/ros2_ws/install`** (now an
   unrelated e-Yantra workspace: `ur_description`,
   `eyantra_kepler_colony`, `ebot_description`, `algorithms`):
   `<ws>/install/setup.bash:25` froze it into the overlay's underlay
   chain at build time, and `~/.bashrc:149` exports it into every
   terminal — which `bash --noprofile --norc` does NOT clear.

**Built:**

- `setup_env.sh` — (a) removes every entry under an inherited non-ROS
  ament/colcon prefix from nine path-like variables before sourcing ROS
  (`COCO_PRESERVE_PATH=1` opts out, the knob `run_platform.sh` already
  had); (b) sources the overlay's `local_setup.bash`, not `setup.bash`;
  (c) finds `moveit_prefix` in the source workspace when `COCO_WS` points
  at an isolated overlay; (d) runs `scripts/check_ament_path.py`, which
  names every listed-but-unresolvable package. Warns; never edits.
- `scripts/build_overlay.sh [DEST]` — this repo only (`--base-paths`),
  clean package path, `--symlink-install`.
- Tests: `gazebo_models/test/test_no_turtlebot_dependency.py` (25) and
  `coco_rl/test/test_setup_env.py` (18). Swapping the old `setup_env.sh`
  back in fails 3 of the first 8 (underlay leak, no dangling report,
  MoveIt lost); the pre-sanitise version fails 8 of 18.

**A wrong turn, measured and reverted:** `build_overlay.sh` first
defaulted to a COPYING install (so a moved tree could not dangle).
44 `coco_rl` tests then failed (34 failed, 10 errors), all
`FileNotFoundError` on
`<install>/coco_sim/lib/python3.12/site-packages/worlds/yard_params.yaml`
— `coco_sim/yard.py:103` resolves `worlds/` from its source file.
`--symlink-install` is now required and tested.

**Measured:**

- Tests **1607 / 0 / 0** on `~/coco_ws_build`, per package, cwd inside,
  clean graph: coco_config 70, custom_teleop 75, coco_rl 216,
  coco_perception 139, gazebo_models 206, coco_moveit_config 12,
  coco_sim 55, coco_mission 317, coco_web 517. (1564 + 25 + 18.)
- Overlay `~/coco_ws_build`: 9 packages, 16.0 s, underlay chain
  `/opt/ros/jazzy` only, 0 dangling symlinks, 459 packages enumerated,
  0 unresolvable, `GazeboRosPaths.get_paths()` OK.
- Environment after, same inherited terminal (4 e-Yantra entries in):
  `AMENT_PREFIX_PATH` = MoveIt prefix + 9 COCO + `/opt/ros/jazzy`;
  0 entries under `$HOME/ros2_ws/` and 0 turtlebot entries across nine
  variables; `gazebo_models`, `coco_mission`, `coco_web` resolve from
  `~/coco_ws_build`.
- **Live, three fresh simulators, green, browser-driven** (headless
  Firefox, `scripts/browser_check/live.py`); evidence and the full table
  in `docs/data/clean_runtime/`:
  - Runs 1 and 2 — the user's exact commands, `gui:=true`: Gazebo server
    + GUI as one tree, controllers active 7 s / 10 s after launch,
    `/healthz` 200 10 s after the mission launch, Nav2 lifecycles active,
    0 turtlebot mentions and 0 `[ERROR]` in `sim.log`. Both climbed,
    found, approached and grasped (lift **35.8 / 34.8 mm**, ground truth),
    descended — and **both ABORTED `RETURN_FAILED`**: `planner_server`
    `"Start occupied"` ×3 from (8.00, 1.17) and (7.87, 1.25), the foot of
    the ramp, after ~17 s of return driving under PolygonSlow/Limit.
  - Run 3 — `live_run.sh` unmodified, `gui:=false`: **COMPLETE,
    `result=fetch`, `attempts={}`**, all 16 states on the page, lift
    35.2 mm, `place finished: placed`.
  - Safety, all three: STOP with W held → first zero 16.2 / 3.8 / 3.0 ms;
    browser SIGKILLed mid-drive → first zero 75.5 / 89.2 / 92.4 ms; moving
    commands > 600 ms after either: 0. One publisher on
    `/diff_drive_controller/cmd_vel` (`cmd_vel_arbiter`) throughout; the
    platform's only velocity publisher is `/cmd_vel_teleop`. 8/8 hostile
    frames refused. 0 dropped frames. Orphans after teardown, 55
    `ros_clean.sh` patterns: 0.

**Correction to P0.2's addendum** (above, not edited): its tip "look for a
prefix whose `share/ament_index` is missing" finds the harmless case. The
fatal one is a marker that EXISTS as a dangling symlink;
`python3 scripts/check_ament_path.py` finds it.

**NOT established:** why the GUI runs fail the return leg. GUI 0/2 vs
headless 1/1 is three runs, not a rate, and P0.2's first pass saw the
same `Start occupied` headless (1 of 2). It is Nav2 territory; nothing
was investigated or tuned.

**Unverified:** the user's own terminal (this ran from the job's shell,
which carries the same `~/.bashrc` exports); `<ws>/install` itself is
untouched and still cannot launch Gazebo; one completed fetch is not a
rate; the GUI return-leg failure is unexplained.

**Next command** (the configuration that completed; `gui:=true` also
launches cleanly but 0 of 2 fetches got home with it):

```bash
export COCO_WS="$HOME/coco_ws_build"
source <repo on coco-clean-runtime>/setup_env.sh
ros2 launch gazebo_models full_world_robo.launch.py traverse:=true gui:=false
ros2 launch coco_mission mission.launch.py platform:=true rviz:=false   # 2nd terminal
# browser: http://localhost:8080 -> pick a colour -> Start
```

## 2026-09-22 — P0.2 release pass: Claude + Codex integrated, release candidate

Branch `p02-release-candidate`, from `coco-clean-runtime` @ `b32539e`.
No robot, Nav2, arbiter, safety-allowlist or perception code changed;
`docs/RSE_ASSIGNMENT_PLAN_V2.md` untouched; protocol stays `coco.v1`.

**Built.**

- Codex: 10 of its 13 listed commits were already on the branch;
  `c0d2f11` cherry-picked (`-x`); `09a77aa` ported selectively (decoder
  strictness + stale-socket guard into `web/frame.js` / `app.js`, not the
  `Transport` class); `fcefc1b` (evidence/replay) and the handoff commit
  left on `codex/p02-hardening`.
- All ten of Codex's caller-side blockers resolved: stored lifecycle with
  explicit `LIFECYCLE_EDGES` (health never moves it; only COCO's simulator
  loss fails it); every browser write bounded (state streams superseded,
  4 MiB abandon, STOP at receipt); per-client `dropped`; close path
  tested; padded rows tested at the ROS boundary; MJPEG at
  `/video/<alias>` with `web_video_server` on loopback and 8080 the only
  published port; `mission.timing` with named clocks, `changed_at` null;
  telemetry kept pinned (explicit decision); keepalive 10/10 with a
  startup check; decoder + stale guard ported to the page.
- Harness: joystick drag, STOP hit-tests, MJPEG check, measured RTF,
  `COCO_LIVE_GUI`, a dedicated ROS domain (61), a session-sweep teardown.

**Measured.**

- Tests, per package, cwd inside, clean ROS graph on a quiet machine:
  coco_config 70, custom_teleop 75, coco_rl 218, coco_perception 139,
  gazebo_models 206, coco_moveit_config 12, coco_sim 55, coco_mission
  317, coco_web 575 = **1667 / 0 / 0**. Clean build 9/9.
- **Under load, one timing test fails**: after a foreign COCO stack
  (`~/c2nav49_overlay`, `gui:=true`, RViz) started at ~21:02, load average
  43 on 12 cores, `gazebo_models/test/test_cmd_vel_wiring.py::TestLiveGraph::
  test_the_relay_output_is_restamped_and_unaltered` failed 8 of 9 runs
  (9 of the 10 messages it needs inside its window; private DDS domain, so
  not the foreign graph). No code it tests changed. Not the documented
  `TestTheOldLoopIsDetected` flake.
- Slow client (tests): peer that stopped reading, 200 ticks, 3 runs —
  its buffer 74–89 kB, healthy neighbour 200/200 frames, longest tick
  40–42 ms, STOP from the stalled client reached the wheels.
- **Live, 5 / 5 browser-driven fetches COMPLETE**, fresh simulator each:
  red, blue, yellow, green headless; green `gui:=true`. Lift 35.6–36.0 mm,
  all `placed`, `attempts={}`; 16/16 states on the page; transitions
  9.6–101.7 ms to the DOM; RTF 0.455–0.506; joystick exercised (first
  wheel motion 72.8–161.2 ms); STOP with W held and browser SIGKILL both
  0 moving commands after; 1 wheel publisher; 8/8 hostile refused; 0
  drops; 0 JS errors. `docs/data/p02_release/`.
- GUI run: no divergence in timing or mission state; an unattributed
  `/mission/mode` + Nav2 goal (2.50, 2.00) on shared domain 0 moved the
  wheels at ≤ 0.012 m/s / 0.5 rad/s for 50 ms; `gz sim server` orphaned by
  the process-group teardown, killed by PID (ours: our session, our
  overlay, our cwd).

**Unverified.** Docker (not installed); touch-screen joystick; any
browser but Firefox; the source of the domain-0 goal; any rate.

**Next command** — the configuration that completed five times:

```bash
export COCO_WS="$HOME/coco_ws_build"
source <repo on p02-release-candidate>/setup_env.sh
scripts/build_overlay.sh "$COCO_WS"
scripts/browser_check/live_run.sh "$PWD" "$COCO_WS" out/live blue   # one fresh run
```

## 2026-09-24 — P0.3 stage B: the episode specification; the machine; Isaac Sim on this hardware

Branch `p03-episode-spec` from `p02-release-candidate` @ `c40098f`
(unchanged). Design and assessment: `docs/EPISODE_ARCHITECTURE.md`.

**Built.**

- `coco_sim/coco_sim/episode.py`: `generate_episode(seed, level, backend,
  world_variant, …)` → frozen `EpisodeSpec`; `manifest()` (privileged) vs
  `task_view()` = `{episode_id, requested_colour}` (robot); levels
  `fixed` (default, P0.2 pose for pose) · `colours` · `positions`;
  `validate_episode()` against an envelope DERIVED from `coco_config`;
  `ObstacleSpec` with motion fields, never generated; `EpisodeResult` +
  `check_reproducible()`; timing keys must name their clock. Stdlib +
  `coco_config` only.
- The approach-corridor rule (`16e575b`), after measuring its absence.
- Evidence: `docs/data/p03_episode_spec/`, `docs/data/isaac_foundation/`.

**Measured.**

- Tests, per package, cwd inside: coco_sim **55 → 128**, 0 failed, 0
  skipped; coco_config **70**, coco_rl **218** unchanged. ament_flake8 and
  ament_copyright clean on the new files (pep257 D213 only, the repo's
  existing style).
- Same seed → byte-identical manifest at every level; 10000 seeds × 3
  levels all pass `validate_episode`.
- Approach corridor blocked: `positions` **1012 / 10000** (276 on the
  requested target) before `16e575b`, **0 / 10000** after; `fixed` and
  `colours` 0 / 10000 both times.
- Home cleanup: **1.61 GiB** reclaimed (40384126976 → 42111000576 B
  free): a byte-identical backup of `.codex/worktrees/c2nav0-implementation`
  (`diff -rq` empty; its small unique files kept), `~/ros2_humble` (103
  upstream repos, 0 dirty, 0 unpushed), four `coco_ff_profile*` dirs.
- Isaac Sim: hardware below the 6.x minimum on four counts, 6.1 not
  downloaded. Existing 4.5.0 pip install: default start blocks forever in
  `_wait_for_viewport`; with `create_new_stage=False` it starts in 12.4 s
  and physics runs (60 s, 5114 steps, peak RSS 4.77 GB); rendering ends in
  `LLVM ERROR: out of memory`; Jazzy discovers Isaac's endpoints on domain
  77 but no message was delivered — 2/2 bridge-loaded runs aborted.

**Unverified.** No episode has been spawned in Gazebo or driven. The
`colours`/`positions` levels are geometric only, and the P0.2 mission
cannot complete a `colours` episode (it navigates by `lane_for_colour`).
Isaac ↔ Jazzy message exchange. Why the bridge-loaded runs abort. Docker.

**Needs the owner.** Keep or remove Isaac Sim 4.5 (13.2 G, physics-only
here); old `.claude/jobs/*/tmp` (1.18 G, one holds a rendered
`candidate.mp4`); `~/.local/share/Trash` (327 M); `.cache/codex-runtimes`
(1.8 G, re-downloads).

**Next command** — stage C, spawn from the manifest behind a switch that
defaults to today's layout:

```bash
cd coco_sim && python3 -c "from coco_sim.episode import generate_episode as g; print(g(seed=1827).to_json())"
```

## 2026-09-25 — Container runtime: built, tested, run; what the first real image found

Branch `claude/docker-reproducibility` from `p03-episode-spec` @ `b3c6598`
(P0.2 `c40098f` untouched; Codex's `p03-isaac-backend` untouched). Worktree
`~/coco-infra-worktree`. Docker Engine 29.8.1 / Compose v5.5.1 / Buildx
v0.37.1, containerd image store; the NVIDIA container toolkit is not
installed. Account: `docs/DOCKER.md`. Evidence, generated from the raw
runs by `scripts/container/condense_evidence.py`:
`docs/data/container_validation/`.

**Built.**

- Dockerfile: base pinned by digest; apt/pip downloads in BuildKit cache
  mounts; the WHOLE pip layer locked (`docker/pip-constraints.txt`, 17
  distributions, hard vs flexible pins argued) and enforced at build
  (`docker/check_pip_lock.py`); rosdep-complete (+depth/image_proc, xterm,
  nodejs), python3-scipy named, xvfb; runs as `coco` uid 1000; build-info
  (reproducibility metadata) in a layer, branch and wall clock kept out;
  dpkg/pip manifests in the image; `/etc/profile.d` so `bash -lc` finds ROS.
- entrypoint `info`/`test`/`COCO_MISSION_ARGS`, and a gate on an odometry
  MESSAGE; compose: host-loopback publish, no caps, no-new-privileges,
  `core: 0`, `COCO_IMAGE`. `full_world_robo.launch.py`: spawners
  `--switch-timeout 60`.
- `scripts/container/` (build, validate + failure bundles, platform/nav/ws
  probes, boot probe, mission regression + suspend detection, episode run,
  determinism, condense), `scripts/ci/run_package_tests.py`,
  `.github/workflows/container.yml`.

**Measured.**

- Original Dockerfile, empty Docker: builds, 2711.2 s (link 1.73 MB/s);
  torch drifted to 2.14.0+cpu; no mujoco → `coco_rl` 0 of 218 collected
  (module-level importorskip under launch_testing), `coco_sim` collection
  error; rosdep: 3 exec + 1 test unsatisfied. Its appliance: healthy in
  33.2 s, one boot.
- This image: 760.4 s with cold cache mounts, 229.7 s after an apt change,
  ~21 s for a source change, 870.3 s `--no-cache`; 8.36 GB of layers
  (5.96 GB merged, 1.83 GB compressed); test-only bytes ~124 MB (1.5 %).
- Tests: 1740/1740 in the container x3 serial, x3 `--jobs 9`, x2 on the
  final image; host 1740/1740 x7; the same 1740 ids, 0 outcome mismatches.
  Relay test (`test_cmd_vel_wiring … restamped`): 26/26 quiet, failed 3 of
  4 at load1 17.5 and once at 23.8 — load, not container.
- Appliance: 10 compose boots healthy in 26.4-33.2 s, RTF 0.31-0.43,
  6.6-8.1 cores, 1.35-1.59 GiB; one `cmd_vel` publisher every time; host
  sees 0 container topics; coco.v1 from the host: 0 leaks against 163 live
  topics.
- Activation race: before the fix, 2 of 2 boots with render-init-after-
  activation failed (0 of 10 with the other order); after it 10/10 active.
  The post-fix stalls were 4.12/4.57 s — under the old 5 s too.
- Nav2 with `executive:=false`: 3/3 SUCCEEDED, 0.698-0.714 m, 151-160 wheel
  commands; executive on: 0 wheel commands, Nav2 recoveries 8/8/10 (by
  design: the executive re-asserts `idle` at 2 Hz).
- Four-colour, `main`'s tree: valid runs green COMPLETE (568.9 s, 1
  recovery, 0.115 m); blue, red, yellow ABORT GRASP_FAILED — all after a
  real grasp, all at the carry move's 40 s wall-clock MoveIt wait (host
  archive carry 34.35 s; container green 39.80 s; grasp-phase RTF 0.28 for
  the one that completed, 0.23-0.27 for the three that did not; confounded
  by the image's newer ros2_control/JTC/gz-sim, see below). 3 VOID:
  host suspended twice (1280 s, 23848 s), disk full once.
- 12 GB of host core dumps (rviz2, Nav2 container) from three teardowns;
  `core=0` stopped them (verified).
- Determinism: all 12 COCO layers differ between a cached and a
  `--no-cache` build of one commit; content differs only in 8,251 `.pyc`
  and 3 build logs.
- Host ≠ container software: the image took September's ROS sync (Nav2
  1.3.13, ros2_controllers 4.42.1, controller_manager 4.48.0,
  gz_ros2_control 1.2.20, gz-sim vendor 0.0.13) against the host's
  April-June set (1.3.12, 4.39.0, 4.44.0, 1.2.17, 0.0.10); MoveIt 2.12.4
  both (host from a user-space prefix).
- Episode seed 7 (`fixed`): host/image manifests byte-identical; only
  `task_view()` entered the robot; COMPLETE 451.5 s; `EpisodeResult`
  reproducible from its seed.

**Unverified.** Any hosted-CI run (the workflow has never run on GitHub).
A GPU container (no toolkit). The activation fix against a >5 s stall. A
grasp-carry timeout fix (not made: mission code). Colours/positions
episodes (no manifest-built world). Bit-for-bit image reproducibility (not
achieved; the route is documented).

**Needs the owner.** `sudo rm` the 12 GB of cores in
`/var/lib/apport/coredump` (root-owned; exact command in the report).
Whether `arm_control`'s 40 s wall-clock MoveIt wait should become sim-time
or RTF-scaled. Whether to install the NVIDIA Container Toolkit. CLAUDE.md
still says the image was never built (not edited: owner's file).

**Next command** — the image, then everything but the simulator:

```bash
scripts/container/build.sh --ref HEAD
scripts/container/validate.sh --image coco-platform:$(git rev-parse --short=12 HEAD) --test-reps 3 --jobs 9
```
