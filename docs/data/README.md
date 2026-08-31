# Raw measurement data

Backing data for the numbers in [../RESULTS.md](../RESULTS.md), committed
so the claims are checkable rather than asserted.

| File | What |
|---|---|
| `nav2_goals.json` | 10 Nav2 goals: target, result, seconds |
| `pick_place_runs.txt` | 8 pick-and-place runs: exit code, completion, gripper stall position, cylinder height afterwards |
| `*.monitor.csv` | SB3 Monitor logs behind the learning curve (below) |
| `map_audit.py` | the C2-M1.6 occupancy-map audit (below) |
| `c2m2_sanity.py` | the C2-M2.0 implementation checks — five deterministic experiments |
| `c2m2_benchmark.json` | **the C2-M2.1 benchmark: all 1,440 episodes, raw** |
| `c2m2_analysis.py` | tables, failure clusters and the decision rule, from that JSON |
| `c2m2_plots.py` | the four C2-M2.1 figures, from that JSON |
| `c2m2_live_gate.py` | the C2-M2.1 live Gazebo observer gate (below) |
| `c2m21_live_gate_*.csv` | the two live gate runs: v1 wedge 18°, Yard Route B 26° |
| `c2m5_locrec.py` | **the C2-M5.0 localization recorder** (below) |
| `c2m5_analysis.py` | per-state scoring of those recordings, and the healthy-vs-bad range table |
| `c2m5_*.csv` | the four C2-M5.0 runs, 10 Hz, every column raw |

## The C2-M5.0 localization recordings

Four missions, fresh simulator each, clean graph, sim time, `rviz:=false`,
never `--fast`. `c2m5_locrec.py` records the localization stack at 10 Hz
and computes, from the map and the laser alone, the scan-vs-map
likelihood that `nav2_amcl` scores particles against and never publishes.

```bash
# alongside a running mission stack, before /mission/start
python3 c2m5_locrec.py --out run.csv --tag healthy --hz 10 \
    --map ../../gazebo_models/maps/coco_world.yaml --stop-on-terminal
# what is wired to what, read from the live graph rather than the launch files
python3 c2m5_locrec.py --topology
# score it
python3 c2m5_analysis.py c2m5_*.csv --states --compare
```

**The ground-truth boundary is the point of the file.** Gazebo's
`/model/coco/odometry` is recorded, in columns that all begin `gt_`, and
it exists **only to score the others offline**. Nothing derived from a
`gt_` column may enter a deployable health signal, which is why the
prefix is uniform enough to grep for:

```bash
head -1 run.csv | tr ',' '\n' | grep -v '^gt_'   # what the robot can see
```

Two columns need reading before they are compared with anything.
`amcl_*` is in the **map** frame and `gt_*` is in Gazebo's **world**
frame; the map is anchored at the spawn, so map (0,0) is world (−2, 0)
and the analysis shifts by `WORLD_TO_MAP_X` before subtracting. Without
that shift the healthy run reads as 2.2 m of localization error on a
mission that finished 0.078 m from home. And `mo_age` is normally
**negative**: AMCL post-dates `map->odom` by its `transform_tolerance`,
so about −0.44 s is the healthy value and a climb through zero is the
tell that nothing is republishing it.

The verdict, the four runs and every number are in `RESULTS.md`,
"C2-M5.0 localization health".

## The C2-M2.1 terrain benchmark

`c2m2_benchmark.json` is the whole experiment: B0/B1/B2/B3 × routes
A/B/C × seeds 0–119, **1,440 episodes**, every per-episode row kept. The
configuration was frozen in C2-M2.0 before any of it ran.

```bash
python3 -m coco_rl.terrain_benchmark --report docs/data/c2m2_benchmark.json
python3 docs/data/c2m2_analysis.py     # + failure clusters, + the rule
python3 docs/data/c2m2_plots.py        # writes docs/images/c2m21_*.png
```

The verdict and every number behind it are in `RESULTS.md`, "C2-M2.1 the
terrain benchmark".

**Nothing in this file is a friction estimate.** C2-M2.0 measured that
true μ is not identifiable from this robot's IMU and wheel encoders;
`tau` is a **traction-demand ratio** pinned at tan(grade), and
`sched_mu_gap_*` is the distance between B3's scheduling input and B2's
privileged one — the information that is *not* recovered, not an
estimator error.

## The live observer gate

`c2m2_live_gate.py` drives the robot through the **existing**
`cmd_vel_arbiter` — it publishes to `/cmd_vel_rl`, an arbiter source,
never to the controller topic — and records `/terrain/state` against
ground-truth odometry while feeding the real B3 controller.

It is what caught three defects in `terrain_observer_node` that the
pure-core unit tests could not see, including one that made the observer
withdraw its own estimate on 431 of 431 samples. Run it whenever that node
is touched; the invocation is in `RESULTS.md` and in the file's docstring.

`c2m21_live_gate_wedge18.csv` is the 18° v1 wedge (422 rows), where the
traction bound *cannot* establish because tan(18°) = 0.325 is below the
0.35 a-priori floor. `c2m21_live_gate_yard_b26.csv` is the Yard's 26°
Route B (282 rows), where it does, and where B3 is seen engaging and then
falling back.

## The map-quality verdict

C2-M1.6 had to decide whether `maps/coco_world.pgm` was actually poor or
whether the RViz overlay was merely cluttered — the two look identical on
screen. `map_audit.py` is how that was settled without an opinion:

```bash
python3 docs/data/map_audit.py                        # the numbers
python3 docs/data/map_audit.py -o docs/images/c2m16_map_audit.png
```

It reads the committed map and the committed world and writes only the
figure. The decisive section is **registration**: every static object in
`worlds/coco_world.world` has a known pose, so if the map is coherent one
rigid `(dx, dy)` explains all of them, and if SLAM drifted they disagree.
Measured worst residual **25 mm, half a cell** — see `RESULTS.md`,
"C2-M1.6 map quality and the RViz split".

Needs `numpy`, `scipy`, `Pillow`, and `matplotlib` for `-o`. It is not a
ROS node and is deliberately not installed by any `CMakeLists.txt`: it is
evidence, not a runtime tool.

## Training data behind the published learning curve

`docs/images/ppo_learning_curve.png` is the evidence for the RL result
reported in the README (47,204 steps / 528 episodes, rolling-mean return
stuck at −11…−13, deterministic eval 0/10). These are the Stable-Baselines3
`Monitor` CSVs it was produced from, committed so the figure can be
regenerated and the numbers independently checked.

Regenerate it exactly:

```bash
python3 -m coco_rl.plot_curve \
    docs/data/ppo_run_part1_trimmed.monitor.csv \
    docs/data/ppo50k_b.monitor.csv \
    -o docs/images/ppo_learning_curve.png
```

That reproduces the committed PNG byte-for-byte
(md5 `dbd195d8926d8af2768220cdc7dbc64d`).

## Why two files, and why one is "trimmed"

The run was interrupted and resumed from the 25,000-step checkpoint.
`--resume` starts a fresh Monitor CSV, so the original file still contains
the episodes between the checkpoint and the interruption — episodes the
resumed run then repeats. Concatenating the two raw files double-counts
those steps and overstates the x-axis. `ppo_run_part1_trimmed.monitor.csv`
is the first CSV cut at the checkpoint; `ppo50k_b.monitor.csv` is the
continuation.

## C2-M4 target localisation

`c2m4_localisation.py` places the robot on the platform with
`gz set_pose`, samples `/perception/target_pose`, reads gz's own
ground truth, and subtracts — both sides reduced to **the target
cylinder's axis in `base_footprint`** before any comparison happens.

It is the C2-M4.0 sanity instrument and the C2-M4.1 benchmark runner;
the only difference is how many placements it is asked for.

**The ground-truth boundary is the point of the file.** Truth enters in
exactly two reads — `/model/coco/odometry` (gz's own
`world -> base_footprint`) and `/world/coco_world/pose/info` — and
leaves in none. Nothing it writes reaches `target_pose_node` except
`/mission/target_colour`, which is the operator's colour choice and an
input the real mission also supplies. Placing the robot is experiment
setup, not information: it decides where the robot stands exactly as
driving it there would.

Like `map_audit.py`, it is deliberately **not installed by any
`CMakeLists.txt`**. It is an instrument and must never end up on a
robot.

`c2m4_sanity_sweep.csv` is the 20-placement C2-M4.0 run — four colours
x five stand-offs (0.28/0.35/0.50/0.70/0.90 m), on-lane, 12 frames each.
`c2m4_minrange_probe.csv` is the 8-placement control that diagnosed the
close-range bias: the identical placements with `min_range:=0.11`
instead of the 0.15 default, which collapsed a +4.1 to +8.3 mm
radius-proportional range error to the far-field −1.0 to −1.4 mm. One
parameter changed; nothing else.

Both carry `spread_x`/`spread_y`, the frame-to-frame range of the
estimate at a fixed pose. They are 0.0000 throughout, which is what says
the residual is bias rather than noise — and also what says the
simulated depth camera is noiseless, so neither file bounds anything
about a real sensor.

The C2-M4.1 grid is `--benchmark`: four colours x five stand-offs
(0.30/0.40/0.55/0.70/0.90) x three lateral offsets
(0.0/−0.010/+0.030 m), 60 placements, in `c2m4_benchmark.csv`. The
laterals bracket `GRASP_MAX_LATERAL = 0.010`.

**Corrected by the C2-M4.1 live runs.** This paragraph used to say the
laterals matter "because the approach drives straight forward and
therefore leaves `y` alone". That is
`target_pose.reachability_after_approach`'s model, and it is not what
`approach_server` does: its `align` phase pivots until the bearing is
nulled and only then takes the fix the creep and the grasp use.
Measured, a 29.2 mm lateral offset at detection reached the grasp as
**3.0 mm** and a 10.2 mm offset as **1.68 mm**, and both grasps
succeeded. The static verdict is a **lower bound on feasibility**, not a
forecast. See `RESULTS.md`, "the static verdict is a lower bound".

`c2m4_analysis.py` post-processes `c2m4_benchmark.csv` — aggregates, the
IK verdict re-derived from the *measured* pose with `coco_config`'s own
bounds, and `c2m4_scatter.png`. It reads nothing live, so the analysis
is reproducible from the CSV with no simulator.

`c2m4_grasp.py` is the manipulation instrument: **one** perception-driven
grasp per invocation and **one fresh simulator per invocation**, because
the gz `DetachableJoint` binds its child once and a second grasp in the
same world reports success while welding nothing. `c2m4_grasp.csv` is
its eight runs. It reads gz ground truth for verification only — the
deployable path never sees it — and records `lift_verified` and
`place_verified` independently of the server's own verdicts, which is
what caught a toppled cylinder passing `check_lifted`.

## Format

SB3 `Monitor` output: one `#`-prefixed JSON header line, then CSV columns
`r` (episode return), `l` (episode length in steps), `t` (wall-clock
seconds since start). `coco_rl.plot_curve.load` accumulates `l` to get the
step axis; `coco_rl/test/test_plot_curve.py` covers that parsing.
