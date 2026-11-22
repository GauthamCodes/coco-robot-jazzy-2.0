
---

## C2-NAV.2 navigation BaseObstacle scale — one change, measured (measured 2026-09-02)

**A single-variable experiment**, not a tuning session, and the third in
the C2-NAV series. C2-NAV.0 measured the baseline and named three
mechanisms; C2-NAV.1 removed the terminal yaw and eliminated it, plus the
collision monitor, as causes of the `enclosure_entry` stall. That left
**one standing hypothesis**, which C2-NAV.0 stated as mechanism 2:

> with `sum_scores: false`, the cheapest command in a rising `BaseObstacle`
> cost field is zero, and `BaseObstacle` was **93.4 %** of the score of the
> trajectory DWB chose at the stall.

**The hypothesis under test.** Reducing `BaseObstacle.scale` makes forward
motion competitive again, and the robot enters the enclosure.

**The verdict is REJECTED**, and the reason is quantitative rather than
rhetorical: the intervention did exactly what it was meant to do to the
score — `BaseObstacle`'s share of the chosen trajectory fell from
**71.8 % to 0.0 %** — and the stall got **longer**, the robot got
**less far**, and the zero-velocity fraction went **up**.

### The one variable

The baseline for C2-NAV.2 is **C2-NAV.0, not C2-NAV.1**. The C2-NAV.1
goal checker was deliberately reverted before the run so that one variable
moves; `nav2_params.yaml` was restored from commit `8f05c45` and then
edited in a single place.

| | C2-NAV.0 | C2-NAV.2 |
|---|---|---|
| `FollowPath.BaseObstacle.scale` | 8.0 | **2.0** |
| everything else | — | unchanged |

Diffed against `8f05c45` with comments and blank lines stripped, the
effective change is **two lines, one value**:

```
@@ -146 +146 @@
-      BaseObstacle.scale: 8.0
+      BaseObstacle.scale: 2.0
```

**Verified off the live node, not the file.** `install/gazebo_models`
symlinks its `config/` to the trunk checkout, which is at `main` and
still holds `BaseObstacle.scale: 8.0` — so without an explicit
`params_file:=` this experiment would have silently re-run the baseline
and reported it as the result. The running `controller_server` was
interrogated after bringup:

| parameter | live value |
|---|---|
| `FollowPath.BaseObstacle.scale` | **2.0** ← the change |
| `FollowPath.BaseObstacle.sum_scores` | False |
| `goal_checker.plugin` | `nav2_controller::SimpleGoalChecker` |
| `goal_checker.xy_goal_tolerance` / `yaw_goal_tolerance` | 0.25 / 0.25 |
| `FollowPath.xy_goal_tolerance` | 0.05 |
| `PathAlign` / `PathDist` `.scale` | 32.0 / 32.0 |
| `GoalAlign` / `GoalDist` `.scale` | 24.0 / 24.0 |
| `RotateToGoal.scale` / `Oscillation.scale` | 32.0 / 1.0 |
| `vx_samples` / `vtheta_samples` / `sim_time` | 20 / 40 / 1.5 |
| `max_vel_x` / `max_vel_theta` | 0.3 / 1.0 |
| `acc_lim_x` / `decel_lim_x` | 3.0 / −2.5 |
| `controller_frequency` / `failure_tolerance` | 10.0 / 0.3 |
| `progress_checker` radius / allowance | 0.1 / 10.0 |
| `local_costmap.robot_radius` | 0.20 |
| `inflation_radius` / `cost_scaling_factor` | 0.50 / 5.0 |
| `PolygonSlow.slowdown_ratio` | 0.3 |

and the node's own log line reads `Created goal checker : goal_checker of
type nav2_controller::SimpleGoalChecker` — the C2-NAV.0 plugin, not
C2-NAV.1's.

### The result — 0/3, and worse on every movement metric

Three `enclosure_entry` repeats, topology A, 75 s timeout, fresh headless
simulator, robot verified at the spawn (−2.0000, 0.0000), RTF 0.991.

| | C2-NAV.0 baseline | C2-NAV.2 | delta | interpretation |
|---|---:|---:|---:|---|
| success | 0/3 | **0/3** | 0 | no change |
| longest commanded stall (median) | 47.84 s | **64.21 s** | +34 % | **worse** |
| distance remaining at stall (median) | 1.150 m | **1.322 m** | +0.17 m | **worse** |
| final goal error (median) | 1.161 m | **1.320 m** | +0.16 m | **worse** |
| DWB best vx == 0 (median) | 0.680 | **0.921** | +0.24 | **worse** |
| DWB best vx mean (median) | 0.048 | **0.004** | −92 % | **worse** |
| `BaseObstacle` on chosen traj (median) | 101.37 | **0.00** | −100 % | the change worked |
| `BaseObstacle` % of chosen score (median) | 71.8 % | **0.0 %** | −71.8 pt | the change worked |
| min clearance (median) | 0.363 m | 0.435 m | +0.07 m | more space |
| nearest scan return (median) | 0.392 m | 0.513 m | +0.12 m | more space |
| illegal-trajectory fraction (median) | 0.122 | 0.072 | −41 % | fewer rejections |
| progress-checker aborts (median) | 4 | 6 | +2 | worse |
| DWB rate | 8.45 Hz | 7.86 Hz | −7 % | unattributed, as before |

**The intervention succeeded and the robot did not.** `BaseObstacle` stopped
contributing to the selected score entirely, the robot stalled in *more*
free space than the baseline, and it selected zero velocity *more* often.

### Per repeat

| run | status | start (world) | end (world) | driven | goal err | best vx == 0 | `BaseObstacle` on chosen | stall |
|---|---|---|---|---:|---:|---:|---:|---:|
| C2-NAV.0 r0 | TIMEOUT | (−2.611, −0.050) | (−2.197, 2.221) | 3.992 m | 1.449 m | 0.565 | 227.71 (86.5 %) | 47.84 s |
| C2-NAV.0 r1 | TIMEOUT | (−2.553, −0.056) | (−2.303, 2.788) | 3.305 m | 1.159 m | 0.680 | 101.37 (71.8 %) | 31.34 s |
| C2-NAV.0 r2 | TIMEOUT | (−2.566, −0.017) | (−2.293, 2.852) | 3.376 m | 1.161 m | 0.680 | 12.10 (26.7 %) | 48.21 s |
| C2-NAV.2 r0 | TIMEOUT | (−2.000, 0.000) | (−2.208, 2.504) | 2.811 m | 1.320 m | 0.843 | **0.00 (0.0 %)** | 64.21 s |
| C2-NAV.2 r1 | TIMEOUT | (−2.208, 2.504) | (−2.210, 2.501) | 0.195 m | 1.319 m | **1.000** | **0.00 (0.0 %)** | 74.33 s |
| C2-NAV.2 r2 | TIMEOUT | (−2.210, 2.501) | (−2.230, 2.318) | 0.494 m | 1.374 m | 0.921 | **0.00 (0.0 %)** | 55.05 s |

**A methodological difference that must be stated, not buried.** The
baseline ran `enclosure_entry` as leg 6 of a 7-leg tour, so each repeat
approached the pinch from the `corridor_gate` goal at ≈ (−2.58, −0.03).
C2-NAV.2 ran `--only enclosure_entry`, so **only repeat 0 is a fresh
approach** (from the spawn); repeats 1 and 2 begin where repeat 0 stalled
and are therefore *escape* tests, not approach tests. That is why their
`driven` figures are 0.195 m and 0.494 m and why the median `path_len_m`
must not be compared with the baseline's. The comparable numbers are
**C2-NAV.2 r0's 1.320 m goal error against the baseline's 1.159–1.449 m**
— inside the baseline range, not beyond it. Repeats 1 and 2 add a
separate finding: once stalled, the robot **cannot recover**, selecting
zero on 92.1 % and 100 % of cycles for the rest of the leg.

**Five independent reproductions of the stall** were observed under
C2-NAV.2: bench repeats 0, 1 and 2, plus two instrumented probe runs on
fresh simulators (below), stalling at 1.313 m and 1.271 m. None advanced.

### Why forward motion loses — read off DWB's own `/evaluation`

`nav_bench.py` records the chosen trajectory's critic breakdown, which is
enough to say `BaseObstacle` stopped dominating. It is not enough to say
what beat forward motion instead. `docs/data/c2n2_evalprobe.py`
subscribes to `/evaluation` directly and, at the stall, decomposes the
score gap between the chosen trajectory and the best *forward-moving*
one, critic by critic.

**`short_circuit_trajectory_evaluation` is true**, so DWB stops scoring a
trajectory the moment its running total exceeds the best complete score
so far. An aborted trajectory therefore carries only the critics
evaluated before the abort — with the order `RotateToGoal`, `Oscillation`,
`BaseObstacle`, `GoalAlign`, `PathAlign`, `PathDist`, `GoalDist` — and its
`total` is a **partial** sum. Differencing it against a complete score as
if the missing critics were 0.0 manufactures large negative `GoalDist` and
`GoalAlign` terms that are an artefact of the abort. The analysis below
separates the two cases; the first pass of this analysis did not, and
reported a spurious −195 for `GoalAlign`.

**Two stall poses were captured, and they fail for two different reasons.**

**Pose A — (−2.212, 2.513), 1.313 m out, 12 control cycles.** The robot
sits in a **cost-0** cell with a **1.90 m** zero-cost band across it. The
chosen trajectory is `vx = 0.0` in **12 of 12** cycles, total **32.6**,
with `BaseObstacle` **0.00**. Of the 10 sampled forward speeds
(`vx ≥ 0.15`), **8 are scored to completion with `BaseObstacle` = 0.00 and
still lose**, by a median of **7.90** points:

| critic | contribution to the gap, summed over 12 cycles |
|---|---:|
| `PathAlign` | **+34.40** |
| `GoalAlign` | **+29.40** |
| `GoalDist` | **+18.00** |
| `PathDist` | **+14.40** |
| `RotateToGoal` | 0.00 |
| **`BaseObstacle`** | **0.00** |
| `Oscillation` | 0.00 |

The objective as a function of commanded speed, one cycle, best legal
total per `vx` sample:

```
      vx    total  BaseObs  GoalDist  GoalAlign  PathDist  PathAlign
  0.0000    32.60     0.00     15.60      16.20      0.80       0.00
  0.0474    35.20     0.00     16.20      17.40      0.80       0.80
  0.0789    40.80     0.00     16.80      19.20      1.60       3.20
  0.1263    39.40     0.00     16.80      18.60      1.60       2.40
  0.1579    43.60     0.00     17.40      19.80      2.40       4.00
  0.2053    43.60     0.00     17.40      19.80      2.40       4.00
  0.2368    41.60     0.00     17.40      18.60      2.40       3.20
  0.2684    43.00     0.00     17.40      19.20      2.40       4.00
  0.2842   120.00   120.00      0.00       0.00      0.00       0.00   <- aborted
  0.3000   120.00   120.00      0.00       0.00      0.00       0.00   <- aborted
```

The total rises monotonically with commanded speed from 32.60 to 43.00
while **`BaseObstacle` is 0.00 at every one of those speeds**. At this
pose the critic under test is not merely non-dominant — it is *absent*
from the decision, and forward motion is rejected by the goal and path
critics on their own.

**Pose B — (−2.208, 2.678), 1.271 m out, 12 control cycles**, 0.165 m
further into the pinch. The robot is again in a cost-0 cell (chosen
total **34.0**, `BaseObstacle` **0.00**), but here **all 10 forward speeds
are aborted, every one of them on `BaseObstacle` alone**: 120.0–262.0,
against a winning total of 34.0. At scale 2.0 those are cell costs of
**60 to 131**.

**And that is the arithmetic that makes the whole approach unworkable.**
Because `sum_scores` is false and the MapGrid critics' effective weight is
`resolution * 0.5 * scale` = 0.60 per cell for `GoalDist`, the winning
zero-velocity total is only **≈ 33**. A forward trajectory is therefore
disqualified the moment `cost × scale` exceeds ≈ 33 — a cost of about
**17** at scale 2.0, against about **4** at scale 8.0. The pinch presents
cells at **60–131**. Admitting a cost-60 cell would need `scale < 0.57`;
admitting the measured worst at 131 would need `scale < 0.26`. Both are
**below the 0.02 that C2-NAV.0 explicitly forbade returning to**, and at
which obstacle avoidance carried 1/1600th the weight of path following.

So the scale knob cannot reach the required behaviour without recreating
the defect it was raised to fix. **`BaseObstacle.scale` is the wrong
control for this stall**, and 8.0 → 2.0 was never going to be enough:
it moves the admissible-cost bar from 4 to 17 in a place that presents 60
to 131.

### The falsifier was already in the committed baseline

`C2-NAV.0` repeat 2 stalled for **48.21 s** at 1.277 m with
`BaseObstacle` at **0.0 — 0.0 % of the chosen trajectory's score**. A
stall with no `BaseObstacle` contribution at all was sitting in
`c2nav0_baselineA.json` the whole time. The 93.4 % figure from C2-NAV.0's
mechanism 2 is real, and it is one instant in one repeat; it was never
the whole population.

### What the robot is actually doing during the stall

From `docs/data/c2n2_enclosure_stall.csv`, repeat 0's 64.21 s stall:

| | value |
|---|---|
| `/cmd_vel_nav` == 0 | **96.7 %** of samples |
| DWB best `vx` == 0 | **96.9 %** of samples |
| total yaw turned | **5.550 rad** |
| `w` commanded (median / p95 / max) | 0.128 / 1.000 / 1.000 rad/s |
| `w` actual (median / p95 / max) | **0.027** / 0.433 / 0.525 rad/s |
| collision monitor | `SLOWDOWN` 75.3 %, `DO_NOTHING` 16.3 %, `LIMIT` 8.4 % |
| nearest scan return | min 0.495 m, median 0.549 m |

It is **not frozen — it is rotating**, 5.55 rad over the stall, while
never translating. The commanded angular velocity reaches the
`max_vel_theta` of 1.0 rad/s and the actual median is 0.027 rad/s. The
progress checker then aborts `follow_path` every 10 s because a rotating
robot never translates 0.1 m: 5 aborts in repeat 0, 6 in each of 1 and 2.

### Verdict — REJECTED

**Reducing `BaseObstacle.scale` from 8.0 to 2.0 does not change the
enclosure-entry stall.**

* `enclosure_entry` is **0/3 before and 0/3 after**.
* The stall is **longer** (median 47.84 → 64.21 s), the robot stops
  **further out** (1.150 → 1.322 m), and DWB selects zero **more** often
  (0.680 → 0.921).
* The intervention is **not a null intervention** — `BaseObstacle` went
  from 71.8 % to 0.0 % of the chosen trajectory's score. The mechanism
  named in the hypothesis was removed and the symptom did not move.
* At stall pose A, forward trajectories are scored to completion with
  `BaseObstacle` = 0.00 and lose anyway, on `PathAlign`, `GoalAlign`,
  `GoalDist` and `PathDist`. **`BaseObstacle` is not even a necessary
  condition for the stall.**
* At stall pose B, `BaseObstacle` at scale 2.0 does still disqualify
  forward motion — which shows the reduction was insufficient, not that
  the hypothesis holds. The scale required is **< 0.26–0.57**, below the
  0.02-class ratio C2-NAV.0 forbade.

**`BaseObstacle.scale: 2.0` is NOT an approved value and must not be
merged.** It is worse than the baseline on every movement metric measured
and is retained in the worktree only as the record of the experiment.

### What this rules out, and what is left

* **`BaseObstacle` scale as the control for this stall. Ruled out.** Not
  by argument but by intervention, and with the required value bounded
  below the ratio the repo already rejected.
* Four of the original candidates are now eliminated by measurement: the
  `/cmd_vel_nav` ownership loop (C2-NAV.0, 0/3 either way), terminal yaw
  (C2-NAV.1, 0/3 either way), the collision monitor's square zones
  (C2-NAV.1, and here the stall runs with `DO_NOTHING` 16.3 % and 0.50 m
  of free space), and now `BaseObstacle` scaling.
* **What remains unexplained.** Why the goal and path critics score
  standing still better than every free-space forward trajectory at pose
  A. The measured facts are that the robot is 39.7° off the goal bearing
  but only **11.8° off its own global plan's heading** over that plan's
  first 0.30 m, that the plan is present and 25 poses long, and that
  moving forward nonetheless increases `GoalDist` (15.60 → 17.40,
  i.e. 26 → 29 cells) and `PathAlign` (0.00 → 4.00). **Why the MapGrid
  distance rises along a plan the robot is aligned with is not
  established by this experiment**, and is the next thing to measure.
* **Rates.** n = 3 repeats plus 2 probe runs. These are counts.

### Reproduce

```bash
# T1 fresh simulator, headless. Never --fast.
ros2 launch gazebo_models full_world_robo.launch.py gui:=false
# T2 Nav2, pointed at the WORKTREE params. Without params_file:= the
# installed config symlinks to the trunk and you re-run the baseline.
ros2 launch gazebo_models nav.launch.py arbiter:=false \
    params_file:=<worktree>/gazebo_models/config/nav2_params.yaml
# T3 -- nav_bench.py is not installed (it postdates main), so run the
# file, as C2-NAV.0 and C2-NAV.1 did:
python3 <worktree>/gazebo_models/scripts/nav_bench.py \
    --tag navA_baseobs --repeats 3 --timeout 75 --only enclosure_entry
# T3 -- the /evaluation decomposition, on a fresh sim
python3 docs/data/c2n2_evalprobe.py /tmp/c2n2_eval.json

cd docs/data
./c2n2_compare.py c2nav0_baselineA.json c2n2_navA_baseobs.json \
    c2n2_eval_probe.json
./c2n2_reanalyse.py c2n2_eval_probe.json        # pose A
./c2n2_reanalyse.py c2n2_eval_probe_pinch.json  # pose B
```

**One infrastructure trap, paid for in this session.** `ros_clean.sh`'s
pattern list is bracketed so that a pattern cannot match the process doing
the matching — except `'nav2_'`, which is not. Any helper script whose
name contains the substring `nav2_` is killed by the sweep it invokes:
`c2nav2_up.sh` matched, and the run died at exit 144 before the simulator
started. Every C2-NAV.2 script is therefore named `c2n2_*`, and the data
files with it, breaking the `c2nav0_*` / `c2nav1_*` convention on purpose.
**Bracketing that one pattern is a one-character fix and was deliberately
NOT made here**, because this commit is a single-variable experiment and
must not carry an unrelated source change.
