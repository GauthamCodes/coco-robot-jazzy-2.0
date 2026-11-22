
---

## C2-NAV.1 — navigation tuning and validation (scoped 2026-09-01, NOT started)

C2-NAV.0 measured the baseline and ranked the causes; see
`docs/RESULTS.md`, "C2-NAV.0 navigation movement quality". This is the
follow-up it earned. **At most four changes, tested in controlled
combinations against the same seven-leg tour**, `nav_bench.py --repeats 3`
on both topologies, so every claim is comparable to the numbers already
recorded.

The baseline to beat, topology A: **16/21 legs**, median transit speed
**0.208 m/s**, terminal phase **35 %** of a leg, `enclosure_entry` **0/3**.

### Ranked proposals

**1. Stop requiring a goal yaw the mission does not need.**
*Change:* have callers send the goal with no orientation constraint, or
raise `goal_checker.yaw_goal_tolerance` toward π for the flat-ground
legs. `use_final_approach_orientation: false` is already set on the
planner with the comment "the goal has no meaningful heading here", so
the yaw requirement is unintended.
*Expected:* removes the terminal phase — a median 35 % of every leg — and
with it the `wall_adjacent` failure mode (transit 3.8 s, terminal 73.6 s).
*Risk:* **low for navigation, but not zero for the mission.** The ramp
approach and `ALIGN_FOR_CLIMB` may depend on arriving roughly head-on;
`PROJECT_STATE.md` records that the leg arrives at +0.26 to +0.28 rad
every time and that the heading gate is deliberately OFF. Verify the
climb still starts before adopting.
*Validates by:* terminal-phase seconds per leg, and a full fetch mission.

**2. Reduce the local costmap's `inflation_radius` to fit the arena.**
*Change:* `local_costmap.inflation_layer.inflation_radius` from 0.50
toward ~0.35, leaving the global costmap alone.
*Expected:* restores a zero-cost band in the 0.63 m and 0.75 m passages,
which currently have **none**, so BaseObstacle stops reading a large
absolute cost everywhere in them. This is the direct cause of the
`enclosure_entry` stall.
*Risk:* medium — less margin against the wall. Bounded by the fact that
`robot_radius` (0.20) still marks everything within 0.20 m as inscribed,
so it cannot plan into contact.
*Validates by:* `enclosure_entry` success, and `min_clearance_m` must not
fall below the 0.286 m already recorded for `obstacle_corner`.

**3. Lower `BaseObstacle.scale`, or move to `ObstacleFootprint`.**
*Change:* 8.0 → ~2.0, **or** swap the critic for `ObstacleFootprint`,
whose `getScale()` is `resolution * scale` and which checks the real
footprint rather than the centre cell.
*Expected:* BaseObstacle is currently 55 % of the chosen trajectory's
score against 2.4 % for PathDist + PathAlign; this rebalances it without
touching the cost field.
*Risk:* medium — this is the value C2-M1 raised from 0.02 deliberately.
**Do not return to 0.02.** The honest reading of that change is that it
corrected a real defect and overshot, because the two critics are scaled
in different units.
*Validates by:* the score-share table, driven `min_clearance_m`, and the
1.30 m Zone A gate still being taken.

**4. Make the progress checker survive a rotating robot.**
*Change:* raise `movement_time_allowance` above the time a 180° turn
takes at the throttled rate, or adopt `PoseProgressChecker`, which counts
rotation as progress.
*Expected:* removes the 27 (topology A) and 45 (topology B)
`Failed to make progress` aborts, each of which currently kills
`follow_path` and forces a BT replan.
*Risk:* low, but it masks genuine stalls — pair it with 1 so the rotation
it is forgiving is short.
*Validates by:* `n_progress_failures` per leg.

### Deliberately not in the first round

* **`vx_samples` / `vtheta_samples` (819 trajectories per cycle) and
  `publish_evaluation` / `publish_trajectories`.** The 8.76 Hz measured
  against a configured 10.0 Hz is confirmed; the attribution is not. Test
  these **one at a time against the rate alone** before touching
  behaviour, or the tuning above will be measured on a moving control
  rate.
* **`FollowPath.xy_goal_tolerance` 0.05 vs `goal_checker` 0.25.** A real
  5× disagreement, but proposal 1 may make it moot.
* **The collision monitor's square polygons.** Circles of the intended
  radius would stop `PolygonSlow` engaging at 0.566 m. Deferred because
  the monitor aggravates rather than causes, and because changing safety
  zones deserves its own session.

### Separately, and not part of the tuning

**C2-NAV.2 — the `/cmd_vel_nav` ownership loop.** Now measured against a
control: the wheels exceed the collision monitor's output on **0.06 % of
samples without the loop and 14.0 % with it**, worst case 0.300 m/s
against a commanded 0.0. The fix is the topic rename already described in
`PROJECT_STATE.md` KNOWN LIMITATIONS 0 — give the relay its own output
and point the arbiter at it — plus re-stamping in `cmd_vel_relay`, which
is why topology A drops 233 wheel commands as stale and topology B drops
none. This is a **safety** fix; it is not expected to fix the stalling,
because `enclosure_entry` fails 0/3 in both topologies.
