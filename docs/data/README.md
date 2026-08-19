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

## Format

SB3 `Monitor` output: one `#`-prefixed JSON header line, then CSV columns
`r` (episode return), `l` (episode length in steps), `t` (wall-clock
seconds since start). `coco_rl.plot_curve.load` accumulates `l` to get the
step axis; `coco_rl/test/test_plot_curve.py` covers that parsing.
