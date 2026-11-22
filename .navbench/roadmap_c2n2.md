
---

## C2-NAV.2 — candidate 1 only, DONE and measured (2026-09-02)

One variable: `FollowPath.BaseObstacle.scale`, **8.0 → 2.0**, against the
**C2-NAV.0** baseline (`SimpleGoalChecker`, `yaw_goal_tolerance` 0.25).
The C2-NAV.1 goal-checker change was reverted first so that exactly one
value moves. Full record and every number: `docs/RESULTS.md`, "C2-NAV.2
navigation BaseObstacle scale".

**Verdict: REJECTED.** `enclosure_entry` is **0/3 before and 0/3 after**,
and every movement metric is worse — the stall is longer (median
47.84 → 64.21 s), the robot stops further out (1.150 → 1.322 m), and DWB
selects zero velocity more often (0.680 → 0.921).

This is a rejection rather than a null result, because the intervention
demonstrably worked on the quantity it targeted: `BaseObstacle`'s share of
the chosen trajectory's score fell from **71.8 % to 0.0 %**. The named
mechanism was removed and the symptom did not move.

**`BaseObstacle.scale: 2.0` is NOT approved and must not be merged.** It
is retained in the worktree only as the record of the experiment.

### What C2-NAV.2 settled

1. **`BaseObstacle` scale is the wrong control for this stall, with the
   required value bounded.** Because `sum_scores` is false and the MapGrid
   critics' effective weight is `resolution * 0.5 * scale` (0.60 per cell
   for `GoalDist`), the winning zero-velocity total is only ≈ 33. A
   forward trajectory is disqualified once `cost × scale` exceeds that —
   cost ≈ 17 at scale 2.0, ≈ 4 at scale 8.0. The pinch presents measured
   cell costs of **60–131**, so admitting them needs `scale < 0.26–0.57`,
   **below the 0.02-class ratio C2-NAV.0 forbade returning to**. The knob
   cannot reach the behaviour without recreating the defect it fixed.
2. **`BaseObstacle` is not a necessary condition for the stall.** At one
   captured stall pose, 8 of 10 sampled forward speeds are scored to
   completion with `BaseObstacle` = **0.00** and still lose to standing
   still, by a median of 7.90 points carried entirely by `PathAlign`
   (+34.40), `GoalAlign` (+29.40), `GoalDist` (+18.00) and `PathDist`
   (+14.40), summed over 12 cycles.
3. **The falsifier was already in the committed baseline.** C2-NAV.0
   repeat 2 stalled 48.21 s with `BaseObstacle` at 0.0 % of the chosen
   score. The 93.4 % figure was one instant in one repeat, not the
   population.
4. **The robot is rotating, not frozen** — 5.550 rad over a 64.21 s
   stall, commanded `w` reaching `max_vel_theta` 1.0 rad/s against an
   actual median of 0.027 rad/s, while `/cmd_vel_nav` linear is zero on
   96.7 % of samples.

### C2-NAV.3 candidates, re-ranked by what C2-NAV.2 eliminated

**Four of the original candidates are now dead by measurement**: the
`/cmd_vel_nav` ownership loop (C2-NAV.0, 0/3 either way), terminal yaw
(C2-NAV.1, 0/3 either way), the collision monitor's square zones
(C2-NAV.1, and C2-NAV.2 stalls with `DO_NOTHING` 16.3 % and 0.50 m of free
space), and `BaseObstacle` scaling (C2-NAV.2). The ranking that follows is
what survives.

1. **Why the goal/path MapGrid says forward is farther. THE OPEN
   QUESTION, and the next experiment.** Measured at the stall: the robot
   is 39.7° off the goal bearing but only **11.8° off its own global
   plan's heading** over that plan's first 0.30 m, the plan is present and
   25 poses long, and moving forward nonetheless *increases* `GoalDist`
   (15.60 → 17.40, i.e. 26 → 29 cells) and `PathAlign` (0.00 → 4.00).
   **Diagnosis before intervention**: instrument the `GoalDist` and
   `PathDist` grids themselves — dump the MapGrid cell values along the
   plan and across the trajectory endpoints — and establish whether the
   propagation is blocked at the pinch, truncated by the 3 × 3 m local
   costmap window, or seeded from a plan whose in-window portion ends
   short. No parameter change until that is known.
2. **`local_costmap.inflation_radius` 0.50 → ~0.35, or
   `cost_scaling_factor`.** Unchanged in rationale, and *promoted* by
   C2-NAV.2: the measured disqualifying costs of 60–131 are what the
   inflation produces in a 0.63 m pinch whose zero-cost band is 0.00 m.
   Attacking the cost field is the alternative to attacking the weight
   that multiplies it, and C2-NAV.2 showed the weight cannot do it.
   Validates by `enclosure_entry` success and `min_clearance_m` not
   falling below the 0.331 m C2-NAV.1 holds.
3. **`ObstacleFootprint` instead of `BaseObstacle`.** C2-NAV.0 listed it
   beside the scale change; C2-NAV.2 tested only the scale. It scores
   footprint collision rather than centre-cell cost, so it is not the same
   experiment and is not refuted by this one. Still ranks below (1)
   because it is an intervention and (1) is a diagnosis.
4. **The 3 × 3 m local costmap** against a goal 1.31 m away and a
   `sim_time` of 1.5 s. Named here for the first time: it is the window
   the MapGrid critics are computed in, and (1) may well end by
   implicating it.
5. **Restore arrival accuracy without restoring the spin.** Unchanged
   from C2-NAV.1's list. **This is what makes C2-NAV.1 mergeable**, and it
   is now the highest-ranked item that is *not* about the enclosure stall.
6. **Verify the ramp leg under a position-only goal.** Required before
   C2-NAV.1 can go to `main`, either way.
7. **A cross-track metric that survives a change in leg structure.**
   Unchanged; `xtrack_med_m` is retired until then.
8. **The control rate**, 7.86–8.45 Hz against a configured 10.0. Still
   unattributed across all three experiments.

### One infrastructure item, recorded and deliberately not fixed here

`gazebo_models/scripts/ros_clean.sh` brackets every `pkill` pattern so
that a pattern cannot match the process doing the matching — **except
`'nav2_'`**. Any helper whose name contains that substring is killed by
the sweep it invokes; it cost this session a run (`c2nav2_up.sh`, exit
144, before the simulator started). Every C2-NAV.2 artefact is therefore
named `c2n2_*` rather than `c2nav2_*`. Bracketing it to `'nav[2]_'` is a
one-character fix that preserves the pattern exactly, and it was **not**
made in the C2-NAV.2 commit because that commit must carry one variable
and its documentation, nothing else.
