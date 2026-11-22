**C2-NAV.1 (terminal goal yaw): COMPLETE, one change, measured
2026-09-01.** Unmerged on `worktree-c2nav0-diagnosis` — see BRANCH MAP.
A single-variable experiment against the committed C2-NAV.0 baseline:
`controller_server.goal_checker.plugin` **`SimpleGoalChecker` →
`nav2_controller::PositionGoalChecker`** (Nav2's own "only checks XY
position and ignores orientation" plugin), `xy_goal_tolerance` left at
0.25 and every other Nav2 parameter verified unchanged off the live
node. Full A/B in `docs/RESULTS.md`.

**Hypothesis PARTIALLY CONFIRMED.** Terminal yaw is a large contributor
to leg time; it is **not** the cause of the wall/enclosure stall.

* **16/21 → 18/21 legs.** Median leg **20.31 → 12.75 s (−37 %)** while
  median *transit* time barely moved (11.42 → 11.20 s), so the whole
  saving is after arrival. `wall_adjacent` **1/3 → 3/3**, 77.34 → 4.22 s.
* **RotateToGoal rejections 465 063 → 0**, total rejections −68 %,
  progress-checker aborts 27 → 13.
* **It did not buy speed with clearance:** median min clearance
  0.419 → 0.486 m, worst case **0.273 → 0.331 m**.
* **`enclosure_entry` is 0/3 before and 0/3 after**, and the stall got
  *longer* — 47.8 → 58.9/62.7/66.2 s, stopped 1.31–1.35 m short with
  637–677 of 819 trajectories still legal.

**Two more suspects eliminated, by measurement rather than argument.**
Post-change the robot stalls with the nearest laser return at
**0.545–0.567 m** (against 0.388 m) and the collision monitor reporting
**`DO_NOTHING` 84–88 %** of the stall. It stops in *more* free space
with *less* gating and still selects zero. So neither terminal yaw nor
the collision monitor's square zones causes it. With C2-NAV.0's
`/cmd_vel_nav` control (0/3 either way), **three of four candidate
explanations for `enclosure_entry` are now ruled out**, leaving
BaseObstacle domination — still 49.3 % of the chosen trajectory's score,
and still only a hypothesis.

**THE CHANGE IS NOT SAFE TO MERGE AS IT STANDS.** Two costs are
measured and neither is closed:

1. **Arrival accuracy roughly halved.** Ground-truth error at the end of
   a leg 0.118 → 0.263 m median, and only **7 of 21** legs reached
   within 0.25 m of the goal by ground truth against 18 of 21. The
   terminal phase was not only spinning — GoalDist was closing the last
   ~0.15 m while it ran.
2. **The final heading is now arbitrary**: median |final heading|
   0.449 → **1.583 rad**, max 2.921. `docs/DESIGN_DECISIONS.md` records
   that 0.25 rad of leftover heading at the pre-ramp pose becomes
   **0.64 m of lateral over a 2.5 m climb**, and that the RL policy
   holds a line but cannot correct one. **No mission or ramp run was
   attempted this session.** The ramp leg must be verified before this
   goes anywhere near `main`.

`xtrack_med_m` (0.571 → 1.227 m) is an **artefact**, not a regression:
it measures distance to a stub final plan over time-uniform samples, and
the baseline parks 32.8 % of its samples at the goal against 0.0 % now.
Do not quote it as a tracking loss.
