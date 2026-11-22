
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
