
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
