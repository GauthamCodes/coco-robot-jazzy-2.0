
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
