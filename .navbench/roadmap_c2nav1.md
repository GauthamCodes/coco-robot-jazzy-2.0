
---

## C2-NAV.1 — proposal 1 only, DONE and measured (2026-09-01)

**The ranked list above is the scope as it was written. Only proposal 1
was run, deliberately: one variable, one experiment.** Proposals 2, 3 and
4 are untouched and remain candidates. The full A/B is in
`docs/RESULTS.md`, "C2-NAV.1 navigation terminal yaw".

**What was changed:** exactly one line of behaviour —
`controller_server.goal_checker.plugin`, `SimpleGoalChecker` →
`nav2_controller::PositionGoalChecker`. Not the "raise
`yaw_goal_tolerance` toward π" variant the scope offered: Nav2 ships a
plugin for a position-only goal, and using it means there is no yaw
tolerance left in the file to be tempted to tune later.

**Outcome: PARTIALLY CONFIRMED.** 16/21 → 18/21, median leg −37 %,
`wall_adjacent` 1/3 → 3/3, RotateToGoal rejections 465 063 → 0, progress
aborts 27 → 13, and min clearance *improved* (worst 0.273 → 0.331 m).
**`enclosure_entry` stayed 0/3** and its stall got longer. The predicted
"removes the `wall_adjacent` failure mode" was right; the hoped-for
enclosure fix was never on offer, because that leg never reaches the
goal tolerance and so has no terminal phase to remove.

**The risk the scope flagged is now measured and is real.** It said
"verify the climb still starts before adopting". Two costs:

* Ground-truth arrival error 0.118 → 0.263 m median; 7 of 21 legs
  reached within 0.25 m by ground truth against 18 of 21.
* Median |final heading| 0.449 → 1.583 rad.

**Neither the ramp nor a full fetch was run.** Until they are, this
change is measured-but-unadopted.

### C2-NAV.2 candidates, re-ranked by what C2-NAV.1 eliminated

Three of four candidate causes of the `enclosure_entry` stall are now
ruled out by measurement — the `/cmd_vel_nav` loop (C2-NAV.0, 0/3 either
way), terminal yaw (C2-NAV.1, 0/3 either way), and the collision monitor
(C2-NAV.1: the stall now happens with `DO_NOTHING` 84–88 % of the time
and 0.55 m of free space). That promotes what is left.

1. **`BaseObstacle` — now the only surviving hypothesis.** Either
   `BaseObstacle.scale` 8.0 → ~2.0, or swap to `ObstacleFootprint`.
   **Do not return to 0.02.** Validates by: `enclosure_entry` success,
   the score-share table, and `min_clearance_m` not falling below the
   0.331 m C2-NAV.1 now holds.
2. **`local_costmap.inflation_radius` 0.50 → ~0.35.** Unchanged in
   rationale from proposal 2 above; the 0.63 m and 0.75 m passages still
   have a 0.00 m zero-cost band.
3. **Restore arrival accuracy without restoring the spin.** The
   position-only goal gave away the late GoalDist correction along with
   the rotation. Options: a tighter `goal_checker.xy_goal_tolerance` now
   that there is no yaw settle to pay for, or the `precise_goal_checker`
   pattern `PROJECT_STATE.md` already describes. **This is what makes
   C2-NAV.1 mergeable**, so it ranks above cosmetic tuning.
4. **Verify the ramp leg under a position-only goal**, or keep
   `SimpleGoalChecker` for the pre-ramp leg specifically via a second
   goal-checker plugin and `FollowPath`'s `goal_checker_id`. Required
   before merge either way.
5. **A cross-track metric that survives a change in leg structure.**
   `xtrack_med_m` compares the whole driven track to the *last* global
   plan, so it silently rewards parking at the goal; it moved
   0.571 → 1.227 m for that reason alone. Compute it over the transit
   phase against a contemporaneous plan.
6. **The control rate** (8.31 Hz against a configured 10.0). Still
   unattributed, still to be tested one sampler at a time against the
   rate alone, as C2-NAV.0 said.
