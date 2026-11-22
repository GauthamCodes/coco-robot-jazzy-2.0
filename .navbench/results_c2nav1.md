
---

## C2-NAV.1 navigation terminal yaw — one change, measured (measured 2026-09-01)

**A single-variable experiment**, not a tuning session. C2-NAV.0 ranked
three mechanisms behind the crawl-and-stall behaviour and proposed four
changes; this tests **proposal 1 alone** and nothing else. No other Nav2
parameter, source file, launch file or test was touched, and the
verification that only one thing moved is recorded below.

### Hypothesis

The navigation stack spends a substantial part of every leg satisfying a
terminal goal **yaw** that the mission does not need. Every caller sends
`orientation.w = 1.0` — an identity quaternion, which means "no heading
was chosen", not "face map-x" — and the planner already declares the
heading meaningless (`use_final_approach_orientation: false`, commented
"the goal has no meaningful heading here").

### The one change

`gazebo_models/config/nav2_params.yaml`, `controller_server.goal_checker`:

| | before | after |
|---|---|---|
| `plugin` | `nav2_controller::SimpleGoalChecker` | **`nav2_controller::PositionGoalChecker`** |
| `xy_goal_tolerance` | 0.25 | 0.25 (**unchanged**) |
| `yaw_goal_tolerance` | 0.25 | **not declared by the plugin** |
| `stateful` | true | true (**unchanged**) |

`PositionGoalChecker` is Nav2's own plugin, shipped in Jazzy and
registered in `nav2_controller`'s `plugins.xml`: *"Goal checker that only
checks XY position and ignores orientation"*. Using it means the yaw
requirement is **removed** rather than widened, so there is no tolerance
left to tune — which is what makes this a single-variable test rather
than the start of a tuning loop.

**Everything else held.** Read back off the live `controller_server`
during the run, not off the file: `FollowPath.xy_goal_tolerance` 0.05,
`RotateToGoal.scale` 32.0, `.slowing_factor` 5.0, `.lookahead_time` -1.0,
`BaseObstacle.scale` 8.0, `.sum_scores` False, `Oscillation.scale` 1.0,
`vx_samples` 20, `vtheta_samples` 40, `sim_time` 1.5, `max_vel_x` 0.3,
`controller_frequency` 10.0, `failure_tolerance` 0.3,
`progress_checker.required_movement_radius` 0.1,
`.movement_time_allowance` 10.0, `robot_radius` 0.20,
`inflation_radius` 0.50, collision monitor untouched. The controller log
line is `Created goal checker : goal_checker of type
nav2_controller::PositionGoalChecker`, and
`ros2 param get /controller_server goal_checker.yaw_goal_tolerance`
answers **"Parameter not set"**.

### Method

Identical to the C2-NAV.0 baseline in every respect but the goal checker:
same seven-leg tour, same topology A (`nav.launch.py` alone, no arbiter),
same map, same 75 s timeout, three repeats, fresh headless simulator,
robot verified at the spawn (-2, 0) at yaw 0 before starting. Nav2 was
launched with `params_file:=` pointing at the worktree copy, because the
installed `share/` symlinks to the trunk checkout.

The baseline column below is **not transcribed**: `docs/data/c2nav1_ab.py`
reads the committed `c2nav0_baselineA.json` and reduces it with the same
functions `c2nav0_analysis.py table` uses, so both columns are produced
by one program from one definition.

### Result — the A/B

| metric | C2-NAV.0 baseline | C2-NAV.1 | change |
|---|---:|---:|---:|
| **legs succeeded** | **16/21** | **18/21** | +2 |
| median leg duration | 20.31 s | **12.75 s** | **−37 %** |
| median transit time | 11.42 s | 11.20 s | −2 % |
| median transit speed | 0.208 m/s | **0.228 m/s** | +9 % |
| median terminal time | 6.155 s | **0.89 s** | **−86 %** |
| terminal phase, % of leg | 35.0 % | **7.4 %** | −27.7 pt |
| median terminal yaw travel | 1.340 rad | **0.002 rad** | −100 % |
| **RotateToGoal rejections** | **465 063** | **0** | **−100 %** |
| BaseObstacle rejections | 186 259 | 176 509 | −5 % |
| Oscillation rejections | 266 760 | 120 400 | −55 % |
| total rejected trajectories | 918 082 | 296 909 | −68 % |
| median DWB illegal fraction | 0.170 | 0.004 | −97 % |
| median best-vx == 0 fraction | 0.284 | 0.041 | −86 % |
| progress-checker aborts | 27 | **13** | −52 % |
| median fraction cmd v < 0.05 | 0.389 | 0.081 | −79 % |
| median fraction actual v < 0.05 | 0.560 | 0.348 | −38 % |
| total stops | 66 | 52 | −21 % |
| median min clearance | 0.419 m | **0.486 m** | +16 % |
| **worst min clearance** | 0.273 m | **0.331 m** | +21 % |
| median path length | 2.798 m | 2.454 m | −12 % |
| median DWB rate | 8.76 Hz | 8.31 Hz | −5 % |
| BaseObstacle score share | 55.0 % | 49.3 % | −5.7 pt |
| **`enclosure_entry`** | **0/3** | **0/3** | **none** |
| `wall_adjacent` | 1/3 | **3/3** | +2 |
| median ground-truth arrival error | 0.118 m | **0.263 m** | **+123 %** |
| legs reaching GT 0.25 m of goal | 18/21 | **7/21** | −11 |
| median \|final heading\| | 0.449 rad | **1.583 rad** | +253 % |
| max \|final heading\| | 2.649 rad | 2.921 rad | +10 % |

Per scenario, median over three repeats:

| scenario | A ok | A leg s | B ok | B leg s |
|---|---:|---:|---:|---:|
| `open_space` | 3/3 | 14.06 | 3/3 | **8.54** |
| `wall_adjacent` | **1/3** | 77.34 | **3/3** | **4.22** |
| `wall_parallel` | 3/3 | 16.59 | 3/3 | 12.75 |
| `obstacle_corner` | 3/3 | 18.73 | 3/3 | 17.45 |
| `corridor_gate` | 3/3 | 22.11 | 3/3 | **11.87** |
| `enclosure_entry` | **0/3** | 77.32 | **0/3** | 77.19 |
| `enclosure_exit` | 3/3 | 20.06 | 3/3 | 13.79 |

### The terminal phase really did go, and the evidence is not circular

The terminal-phase metric is defined by crossing the same 0.25 m the
goal checker acts on, so on its own it would be close to tautological.
Three measurements that are **not**:

1. **RotateToGoal threw 465 063 rejections and now throws zero.** That
   critic only engages inside `FollowPath.xy_goal_tolerance` (0.05 m),
   which was never changed. Zero means the rotate-in-place mode was
   never entered at all.
2. **Median transit time is flat (11.42 → 11.20 s, −2 %) while median
   leg duration fell 37 %.** The entire saving is after arrival. Whatever
   the change did, it did not touch the drive.
3. **Watched, on a fresh simulator.** Baseline `open_space` turned
   **1.49 rad in its last 5 s**, exceeding 0.15 rad/s on 54.9 % of
   samples with a peak of **1.037 rad/s** — against `max_vel_theta` 1.0,
   i.e. a full-speed spin on the spot. Post-change the same leg turns
   **0.42 rad**, peak **0.231 rad/s**; `wall_adjacent` turns 0.27 rad,
   peak 0.182 rad/s. No spin-in-place remains.

### The enclosure stall is untouched — 0/3, and slightly worse

This is the result the hypothesis does not explain, and it was
predictable from the baseline: `enclosure_entry` never reaches the goal's
0.25 m at all, so it has **no terminal phase to delete**. It stalls
1.1–1.4 m out, in transit.

| | baseline rep0 | C2-NAV.1 rep0 | rep1 | rep2 |
|---|---:|---:|---:|---:|
| longest commanded stall | 47.80 s | **58.90 s** | 62.70 s | 66.20 s |
| % of the leg | 61.8 % | 76.5 % | 81.3 % | 85.6 % |
| distance remaining at stall | 1.193 m | 1.312 m | 1.345 m | 1.352 m |
| DWB best vx == 0 | 89.4 % | 94.7 % | 91.4 % | 94.4 % |
| trajectories still legal (of 819) | 696 | 677 | 637 | 651 |
| nearest scan return | 0.388 m | 0.559 m | 0.545 m | 0.567 m |
| collision monitor | SLOWDOWN 77 % | **DO_NOTHING 84 %** | SLOWDOWN 72 % | DO_NOTHING 88 % |

The stall is **longer** after the change, not shorter, in all three
repeats, and it happened again in both of the two extra observation runs
— **five consecutive `enclosure_entry` failures**.

**Two suspects are eliminated by this table rather than by argument.**
The robot now stalls with the nearest laser return at **0.545–0.567 m**
against 0.388 m in the baseline, and with the collision monitor reporting
`DO_NOTHING` for most of the stall rather than `SLOWDOWN`. It is stopping
in *more* free space, with *less* gating, and still selecting zero. So
neither the terminal yaw nor the collision monitor's square zones is what
holds it. What remains is C2-NAV.0 mechanism 2: with `sum_scores: false`,
the cheapest command in a rising `BaseObstacle` cost field is zero, and
BaseObstacle is still 49.3 % of the chosen trajectory's score.

**Watched live in RViz** on the nav2 default view, mid-stall:
`Navigation: active`, `Localization: active`, `Feedback: active`,
`Distance remaining: 1.27 m`, `Time taken: 41 s`, `Recoveries: 4`. The
stack is entirely healthy and the robot is not moving. For contrast, the
fixed `wall_adjacent` leg was caught at `Distance remaining: 0.27 m`,
`Time taken: 3 s`, `Recoveries: 0`.

### What the change costs, stated plainly

**1. Arrival is less accurate.** Ground-truth error at the end of a leg
roughly doubled, and only 7 of 21 legs got within 0.25 m of the goal by
ground truth at all (against 18 of 21):

| scenario | baseline | C2-NAV.1 |
|---|---:|---:|
| `open_space` | 0.145 m | 0.296 m |
| `wall_adjacent` | 0.132 m | 0.350 m |
| `wall_parallel` | 0.097 m | 0.249 m |
| `obstacle_corner` | 0.073 m | 0.258 m |
| `corridor_gate` | 0.065 m | 0.237 m |
| `enclosure_exit` | 0.102 m | 0.256 m |

The terminal phase was not *only* spinning: while RotateToGoal held the
robot at the goal, GoalDist kept closing the last ~0.15 m of position
error, and the goal checker judges against AMCL rather than ground truth.
Deleting the phase deletes that late correction too. Legs now stop the
moment AMCL believes it is inside 0.25 m.

**2. The final heading is now arbitrary**, by construction: median
\|final heading\| 0.449 → **1.583 rad**, max 2.921 rad (167°). This is
the thing that was deliberately given away, and it is exactly what makes
the change **unsafe for the mission as it stands** — see
`docs/DESIGN_DECISIONS.md`, "2.5 m of climb at 0.25 rad is 0.64 m of
lateral".

### One metric that does not survive the change, and must not be read as a regression

`xtrack_med_m` moved 0.571 → 1.227 m. **This is an artefact, not a
tracking regression.** It is the median over time-uniform samples of the
distance from the driven pose to the *last* global plan, and that plan is
a stub near the goal (`plan_len_m_last`: 0.05 m baseline, 0.383 m after).
A leg that parks at the goal contributes a block of samples at distance
≈ 0 and pulls the median down. Measured directly: the baseline parks
**32.8 %** of its samples within 0.25 m of the goal and C2-NAV.1 parks
**0.0 %**; median |track − goal| is 1.158 m and 1.626 m respectively.
The metric is measuring the absence of the terminal park. (The 32.8 %
is an independent corroboration of the baseline's "median 35 % of every
leg" finding, arrived at from the traces rather than the JSON.)

A real cross-track number needs to be computed over the transit phase
against a contemporaneous plan. That is a C2-NAV.2 tooling item.

### Verdict — PARTIALLY CONFIRMED (outcome B)

Terminal yaw was a **real and large contributor to leg time and to the
`wall_adjacent` failure mode, and is not the cause of the wall/enclosure
stall.**

* Confirmed: 37 % off the median leg, `wall_adjacent` 1/3 → 3/3,
  RotateToGoal rejections to zero, progress aborts halved, and — the
  check against buying speed with clearance — **min clearance improved,
  worst case 0.273 → 0.331 m**.
* Rejected as the cause of the stall: `enclosure_entry` is 0/3 before
  and 0/3 after, the stall is longer, and it now happens in more open
  space with the collision monitor idle.

### What this rules out

* Terminal yaw as the cause of the enclosure/wall stall. **Ruled out.**
* The collision monitor's square zones as the cause. **Ruled out for
  this stall** — it stalls with `DO_NOTHING` 84–88 % of the time.
* Together with C2-NAV.0's control on the `/cmd_vel_nav` loop (0/3 with
  and without), three of the four candidate explanations for
  `enclosure_entry` are now eliminated by measurement.

### What remains unexplained

* **Why DWB selects zero with 637–677 of 819 trajectories legal, a
  healthy plan, and 0.55 m of free space.** BaseObstacle domination is
  the standing hypothesis and is C2-NAV.2's first candidate; it is
  **not yet demonstrated by intervention**.
* **Whether the arrival-accuracy loss matters to the mission.** Not
  measured. No mission run was attempted this session.
* **The 8.31 Hz control rate** against a configured 10.0. Unchanged in
  character from the baseline's 8.76 Hz and still unattributed.
* **Rates.** n = 3 per scenario. These are counts, not rates.

### Reproduce

```bash
# T1 fresh simulator, headless. Never --fast.
ros2 launch gazebo_models full_world_robo.launch.py gui:=false
# T2 Nav2, pointed at the edited params
ros2 launch gazebo_models nav.launch.py arbiter:=false \
    params_file:=<repo>/gazebo_models/config/nav2_params.yaml
# T3
ros2 run gazebo_models nav_bench.py --tag navA_goalyaw --repeats 3 --timeout 75

cd docs/data
./c2nav1_ab.py c2nav0_baselineA.json c2nav1_navA_goalyaw.json \
    <out>/baselineA_traces <out>/navA_goalyaw_traces
./c2nav1_stall.py c2nav1_enclosure_stall.csv
./c2nav0_analysis.py table c2nav1_navA_goalyaw.json
```
