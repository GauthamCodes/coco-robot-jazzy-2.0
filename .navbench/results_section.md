
---

## C2-NAV.0 navigation movement quality — why it crawls near walls (measured 2026-09-01)

**Diagnosis only. Nothing in `nav2_params.yaml` was changed.** The
question was why the robot becomes slow, hesitant, oscillatory or stalled
near walls and enclosures, and the answer turned out to be **three
independent mechanisms**, only one of which is about walls.

### How it was measured

`gazebo_models/scripts/nav_bench.py` drives a fixed seven-leg tour
through the `NavigateToPose` action and records every stage of the
command chain plus DWB's own `/evaluation` topic, which carries the
per-critic score of every sampled trajectory **and the name of the critic
that rejected each illegal one**. That last field is the difference
between "DWB looks cautious" and "BaseObstacle was 93 % of the score of
the trajectory it picked".

```bash
# T1 — fresh simulator, headless. Never --fast.
ros2 launch gazebo_models full_world_robo.launch.py gui:=false
# T2 — topology A: Nav2 straight to the wheels
ros2 launch gazebo_models nav.launch.py
# T3
ros2 run gazebo_models nav_bench.py --tag baselineA --repeats 3 --timeout 75
```

Ground truth (`/model/coco/odometry`) is read for evaluation only and is
never published anywhere Nav2 can see it. Rates are per **simulation**
second; measured RTF was 0.99.

**Two topologies were run, on the identical tour**, because they differ
by one launch argument and that is a free controlled experiment:

| | topology A | topology B |
|---|---|---|
| launch | `nav.launch.py` | `nav.launch.py arbiter:=true` + `arbiter.launch.py` |
| used by | standalone Nav2 | **`coco_mission/launch/mission.launch.py`** |
| `/cmd_vel_nav` | 6 publishers, 1 subscriber | **7 publishers, 2 subscribers** |

Topology B is the shipped mission configuration and contains the
`/cmd_vel_nav` ownership loop of KNOWN LIMITATIONS 0.

### The scenarios, and the geometry they were chosen against

Clearances are measured from the shipped `coco_world.pgm`, not from the
world file's nominal numbers. `robot_radius` is 0.20 and
`inflation_radius` is 0.50, so the two thresholds that matter are 0.20 m
(cells become `INSCRIBED`, and DWB throws them out) and 0.50 m (cost
reaches zero).

| leg | goal (world) | goal clearance | what it probes |
|---|---|---|---|
| `open_space` | (−2.00, −2.20) | 1.15 m | control case |
| `wall_adjacent` | (−2.00, −3.00) | 0.35 m | goal inside the inflation band |
| `wall_parallel` | (0.50, −2.95) | 0.36 m | 2.5 m run alongside a wall |
| `obstacle_corner` | (0.30, −0.30) | 0.78 m | rounding `box_obstacle_2` |
| `corridor_gate` | (−2.60, −0.10) | 1.25 m | the 1.30 m Zone A gate |
| `enclosure_entry` | (−3.45, 2.95) | 0.35 m | the NW pinch, into the pocket |
| `enclosure_exit` | (−2.00, 0.00) | 0.82 m | back out of the pocket |

The NW pinch is the hard one: the gap between `box_obstacle_1` and the
west wall is **0.63 m**, of which only **0.30 m is non-inscribed**, and
its widest point is **0.316 m** of clearance — entirely inside the 0.50 m
inflation radius. There is no zero-cost cell anywhere in it.

### Baseline — topology A (3 repeats, 21 legs)

`t` and `transit`/`term` are simulation seconds; `v_tr` is mean transit
speed against a `max_vel_x` of 0.30; `gate%` is the time-weighted
fraction of the leg the collision monitor spent in a gating state.

```
scenario             ok    t_s transit   term    len  clear   v_tr stop  osc  gate% prog
open_space          3/3  14.06    8.65   5.51  2.251  0.517  0.239    2    1  0.031    0
wall_adjacent       1/3  77.34    4.22  73.02  1.066  0.461  0.193    2    9  0.862    6
wall_parallel       3/3  16.59     9.7   6.93  2.576  0.418  0.238    2    4   0.13    0
obstacle_corner     3/3  18.73   13.17   5.59  2.938  0.286   0.21    3    3  0.183    0
corridor_gate       3/3  22.11   13.96   6.66  3.134  0.454  0.202    2    1  0.113    0
enclosure_entry     0/3  77.32    None    0.0  3.376  0.363   None    7   14   0.61    4
enclosure_exit      3/3  20.06   13.96   4.95  2.798  0.329  0.187    2    4  0.215    0

  succeeded              16/21
  median transit         11.42 s
  median terminal         6.15 s   (35.0% of a leg)
  median transit speed    0.208 m/s   (max_vel_x 0.30)
  median DWB rate         8.76 Hz     (controller_frequency 10.0)
  total progress failures 27
  total loop-rate misses  13
  total stale-cmd drops   233
  rejections by critic:   RotateToGoal 465063 (50.7%), Oscillation 266760 (29.1%),
                          BaseObstacle 186259 (20.3%)
  score share of the CHOSEN trajectory: BaseObstacle 24.4 (55.0%), GoalAlign 9.2 (20.7%),
    GoalDist 9.1 (20.6%), PathDist 0.6 (1.3%), RotateToGoal 0.5 (1.2%), PathAlign 0.5 (1.1%)
```

`transit`/`term` is the split at the goal checker's 0.25 m
`xy_goal_tolerance`. It matters: **a third of every leg is spent after
the robot has arrived**, settling the goal yaw, and averaging that
together with the drive hides both halves.

### Baseline — topology B, the mission configuration (3 repeats, 21 legs)

```
scenario             ok    t_s transit   term    len  clear   v_tr stop  osc  gate% prog
open_space          3/3  20.31    8.91  11.42  2.444  0.513  0.231    4    7  0.013    0
wall_adjacent       0/3  77.25    5.44  72.04  1.052  0.458  0.149    5    2  0.757    5
wall_parallel       3/3  52.72  42.425   0.82  2.434  0.351  0.079    3    5  0.606    3
obstacle_corner     2/3  18.47  12.945   5.01  2.985  0.321   0.22    2    1  0.157    0
corridor_gate       3/3  22.62   15.71   7.59  3.118  0.431  0.189    3    1  0.166    0
enclosure_entry     0/3  77.32    None    0.0  3.336  0.356   None    3    6  0.715    4
enclosure_exit      3/3  42.55   33.09   5.41  2.858  0.334  0.087    3    2   0.45    1

  succeeded              14/21
  median transit speed    0.155 m/s
  median DWB rate         7.97 Hz
  total progress failures 45
  total loop-rate misses  23
  total stale-cmd drops   0
```

**The mission topology is measurably worse**: 14/21 against 16/21, median
transit speed **0.155 m/s against 0.208** (−25 %), and 45 progress-checker
failures against 27. But `enclosure_entry` fails 0/3 in **both**, so the
loop is not what stops the robot in the pinch.

### Mechanism 1 — a third of every leg is spent rotating on the spot

Every caller in this repository sends `orientation.w = 1.0` — a yaw-0
goal (`nav_round_trip.py:154`, `traverse_demo.py:345`,
`mission_executive.py:549`). The goal checker requires yaw within 0.25
rad, so on arrival the robot must rotate in place, and three things then
work against each other. Read live from the running controller:

| parameter | value | consequence |
|---|---|---|
| `goal_checker.xy_goal_tolerance` | 0.25 | the leg is "arrived" here |
| `FollowPath.xy_goal_tolerance` | **0.05** | but RotateToGoal only engages here |
| `FollowPath.trans_stopped_velocity` | 0.25 | "stopped" is 83 % of the speed range |
| `progress_checker.required_movement_radius` | 0.1 m / 10 s | a rotating robot never translates it |

Between 0.05 m and 0.25 m of the goal there is no rotate-in-place mode at
all — the two tolerances disagree by 5×. Inside 0.05 m, RotateToGoal
throws `IllegalTrajectoryException` for every trajectory with non-zero
linear velocity, which is **95 % of the 819-trajectory sample set**, and
it is the single largest source of rejections in the whole baseline
(50.7 %). Meanwhile the progress checker sees no translation and aborts
`follow_path` every 10 s; the BT replans and the cycle repeats.

Measured, `open_space` rep0: the robot drove 2.2 m in 9 s at a clean
0.28–0.30 m/s, then spent **10 s stationary** at (−1.99, −2.13) with
779/819 trajectories rejected, yaw hunting
−1.80 → −0.78 → −2.33 → +3.08 → +0.41 rad. `wall_adjacent` is the same
mechanism made fatal: transit 3.78 s, **terminal 73.6 s**, 6 progress
failures, 38 angular sign changes, leg timed out.

### Mechanism 2 — BaseObstacle dominates wherever clearance < inflation_radius

This is the wall/enclosure stall, and it is arithmetic rather than a bug.
From the Jazzy headers on this machine:

* `dwb_critics/map_grid.hpp:69` — `getScale() = resolution * 0.5 * scale`
* `dwb_core/trajectory_critic.hpp:177` — `getScale() = scale`, which is
  what `BaseObstacle` uses
* `BaseObstacle.sum_scores` is **false** (confirmed live), so the score is
  the cost of the trajectory's **final pose only**

So the effective weights are not the numbers in the yaml:

| critic | `scale` | effective |
|---|---|---|
| `BaseObstacle` | 8.0 | **8.00 per unit of cost**, cost running 0–252 |
| `PathDist` / `PathAlign` | 32.0 | 0.80 per 0.05 m cell |
| `GoalDist` / `GoalAlign` | 24.0 | 0.60 per 0.05 m cell |

Advancing one cell toward the goal is worth **1.40**. Advancing one cell
further into the inflation gradient costs:

```
  from d    to d    dCost  dBaseObstacle    ratio
   0.500   0.450     16.0          127.8      91x
   0.450   0.400     20.5          164.1     117x
   0.400   0.350     26.3          210.6     150x
   0.350   0.300     33.8          270.5     193x
   0.300   0.250     43.4          347.3     248x
   0.250   0.200     56.7          453.9     324x
```

DWB **minimises**, and with `sum_scores: false` a slower trajectory ends
in a nearer — therefore cheaper — cell. In any cost field that rises
along the direction of travel, **the BaseObstacle-optimal command is
zero**, and the goal critics are 91–324× too weak to outbid it.

**The stall, from `docs/data/c2nav0_enclosure_stall.csv`.**
`enclosure_entry` rep0, at t+13.43 s the robot stopped at world
(−2.305, 2.852) — **1.149 m short of the goal** — and did not move again
for **47.8 s**:

| signal | value |
|---|---|
| `/cmd_vel_nav` (controller out) | **0.0** |
| `/cmd_vel_smoothed` | 0.0 |
| `/cmd_vel` (monitor out) | 0.0 |
| `/diff_drive_controller/cmd_vel` | 0.0 |
| collision monitor | `SLOWDOWN` / `PolygonSlow` |
| trajectories sampled / rejected | 819 / **42 (5.1 %)** |
| chosen trajectory | vx **0.0**, wz 0.0256 |
| chosen-trajectory critics | **BaseObstacle 456.0**, GoalAlign 15.0, GoalDist 15.0, PathDist 1.6 |
| global plan | present, 0.5 s old |
| median cross-track error | 0.026 m |

**The zero originates at the controller.** DWB had 777 valid trajectories
and *chose* to stop; BaseObstacle was **93.4 %** of the score of the one
it picked. Two independent cross-checks agree: 456.0 ÷ 8.0 = cost 57,
which the inflation formula puts at 0.497 m of clearance, and the nearest
geometry to (−2.305, 2.852) is the `box_obstacle_1` NE corner at
**0.489 m**.

What it refused to enter was the 0.75 m channel between that box and the
north wall — non-inscribed band 0.35 m, **zero-cost band 0.00 m**. Best
achievable clearance at its centre is 0.375 m → cost 105 → BaseObstacle
840. Entering costs **+384**; the ~0.5 m of progress it buys is worth
**14**. The robot is behaving optimally for the objective it was given.

### Mechanism 3 — the collision-monitor zones are squares

`PolygonSlow` and `PolygonLimit` are axis-aligned squares, so their reach
is not their half-width:

| polygon | half-width | reach on the diagonal | action |
|---|---|---|---|
| `PolygonStop` | circle r 0.25 | 0.250 m | stop |
| `PolygonSlow` | 0.40 | **0.566 m** | `slowdown_ratio 0.3`, linear **and angular** |
| `PolygonLimit` | 0.55 | **0.778 m** | 0.4 m/s, 0.5 rad/s |

With `min_points: 4`, a flat wall supplies four returns easily.
Measured: `wall_adjacent` held `SLOWDOWN` for **57.25 s (80.9 % of the
leg)** with the closest laser return at **0.498 m** — between 0.40 and
0.566, exactly the band a square reaches and a circle would not. Because
slowdown scales angular velocity too, it throttles the very rotation
mechanism 1 requires, to 0.3 rad/s.

**This aggravates but does not cause.** Across topology A the monitor
reduced the command on only 10.5 % of samples, and at the enclosure stall
it was reading `SLOWDOWN` against a command that was already zero.

### The command chain, and what reaches the wheels

Comparing `/cmd_vel_smoothed` (monitor in) → `/cmd_vel` (monitor out) →
`/diff_drive_controller/cmd_vel` (wheels), row by row:

| | topology A | topology B (mission) |
|---|---|---|
| rows | 6 956 | 9 793 |
| monitor reduced the command | 733 (10.54 %) | 974 (9.95 %) |
| monitor output ≠ wheel command | **6 (0.09 %)** | **2 591 (26.46 %)** |
| wheels **exceeded** the gate | **4 (0.06 %)** | **1 371 (14.00 %)** |
| worst overshoot | 0.0158 m/s | **0.3000 m/s** (gate 0.0) |

**In topology A the collision monitor is authoritative** — the four
exceptions are 0.016 m/s resampling jitter at transitions. **In topology
B it is not**: 14 % of wheel commands exceed it, and the worst case is
the monitor commanding **0.0 m/s while the wheels receive 0.300**. This
is KNOWN LIMITATIONS 0, reproduced on this benchmark with a clean control
for the first time.

The **233 stale-command drops in topology A and 0 in topology B** have
the same root: `cmd_vel_relay` republishes with the original
`header.stamp`, so chain latency ages the message and
`diff_drive_controller` discards anything older than `cmd_vel_timeout`
(0.5 s). `cmd_vel_arbiter` re-stamps, which is why topology B has none.

### The footprint is not too conservative — it is 5 mm too small

Measured from live TF and the URDF collision geometry, transforming the
actual corners of every box and sampling both rims of every cylinder
(`docs/data/c2nav0_footprint.py`):

```
circumscribed radius : 0.2051 m   (driven by the wheels)
half-width           : 0.1415 m  -> full width 0.2830 m
length               : 0.3195 m  (x -0.1485 .. +0.1710)
nav2 robot_radius    : 0.2000 m  -> 5.1 mm SMALLER than the robot
```

The chassis cross-checks exactly against `CHASSIS_SIZE` (0.24 × 0.274 m).
**`robot_radius: 0.20` is realistic-to-slightly-permissive, not
conservative**, so reducing it is not an available fix. A polygon
footprint is not indicated either: this robot rotates in place at every
goal, so the swept disc is the correct model — it is simply 5 mm short.

### The controller does not run at 10 Hz

`controller_frequency` is 10.0. Measured DWB rate: **8.76 Hz** (topology
A) and **7.97 Hz** (B), with 13 and 23 explicit `Control loop missed its
desired rate` warnings and a minimum observed rate of **4.72 Hz**.

DWB samples `vx_samples: 20` × `vtheta_samples: 40` = **819 trajectories
per control cycle**. Separately, `publish_evaluation` and
`publish_trajectories` are both left at their `true` defaults, and
`dwb_core/publisher.hpp:85` gates recording on
`publish_evaluation_ || publish_trajectories_` **with no subscriber
check** — so the controller builds and publishes a full 819-trajectory
evaluation message every cycle whether or not anything is listening.
`/evaluation` was captured with 1 publisher and 0 subscribers before this
benchmark ever ran.

**The rate shortfall is confirmed. The attribution to trajectory count or
to the publishers is NOT** — neither was tested by changing them, which
is C2-NAV.1's job.

### Ruled out

* **Global planner geometry.** Healthy at every stall: a plan was present
  and ≤0.5 s old, median cross-track error 0.026–0.097 m.
* **The velocity smoother.** It upsamples 10 → 20 Hz correctly and
  `/cmd_vel_smoothed` tracked `/cmd_vel_nav`; it is not the source of any
  zero.
* **The collision monitor as the cause of the enclosure stall.** The
  command was already 0.0 at `/cmd_vel_nav`, upstream of it.
* **The `/cmd_vel_nav` ownership loop as the cause of the stall.**
  `enclosure_entry` fails 0/3 in both topologies. It is a safety defect
  and a 25 % speed tax, not the stall.
* **The footprint model.** Measured above.

### Not established

* **Localization error as a contributor.** Cross-track error was small
  and AMCL was healthy, but no divergence was injected and no
  AMCL-vs-ground-truth error budget was computed for these legs.
* **Which of the three publishers/samplers costs the controller its
  10 Hz.** Correlational only.
* **The `Oscillation` critic's exact role.** It is the second-largest
  source of rejections (29.1 %) and `oscillation_reset_dist` is 0.05 m,
  which a rotating robot does not travel — but `oscillation_reset_angle`
  is 0.2 rad and does reset it, so no livelock was isolated.
* **Rates.** n = 3 per scenario per topology. These are counts, not rates.

### Reproduce

```bash
ros2 run gazebo_models nav_bench.py --tag baselineA --repeats 3 --timeout 75
cd docs/data
./c2nav0_analysis.py table c2nav0_baselineA.json c2nav0_baselineB.json
./c2nav0_analysis.py chain '<out>/baselineA_traces/*.csv'
./c2nav0_analysis.py arith
python3 c2nav0_footprint.py          # needs the sim running
```
