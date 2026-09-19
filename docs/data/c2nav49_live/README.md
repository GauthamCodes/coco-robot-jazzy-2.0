# C2-NAV.49 — the M6 colour matrix on `local_costmap.robot_radius` 0.25

Twelve fresh executive-driven missions, three per colour, all four colours.
**12 of 12 fetches, 12 valid, 0 void.** Full report:
`docs/agents/C2-NAV.49_RESULTS.md`.

Produced by:

```bash
COCO_WT=<this repo> COCO_WS=$HOME/c2nav49_overlay \
  bash docs/data/c2nav49_matrix_sweep.sh ~/coco_nav_runs/c2nav49_matrix
```

`COCO_WS` is not optional. `<ws>/install` cannot launch Gazebo — it holds 2
unresolvable ament index entries (`red_ball_nav`, `turtlebot3_teleop`) and
`ros_gz_sim`'s `GazeboRosPaths.get_paths()` enumerates every package, so one
bad entry kills the launch. An isolated overlay measured 0 of 462.

## What is here, and what is not

Per run, the **provenance and readback** files — the ones that say what was
actually running:

| file | what it settles |
|---|---|
| `meta.txt` | git HEAD, `dirty_paths`, colour, `nav2_params.yaml` sha256, start/end |
| `params_live.txt` | every guarded Nav2 parameter, file value vs **live** value |
| `topology_live.txt` | the command chain link by link, and who owns the wheels |
| `depth_off.txt` | depth fusion off as a **readback**, not an assumption |
| `lifecycle.txt` | all ten Nav2 lifecycle nodes active |
| `resolve.txt` | which prefix every package resolved out of |
| `final_state.txt` | the terminal `/mission/state` line |
| `cmdpath/summary.json` | bypass, `stop_breach`, monitor authority, `min_scan_m` |
| `cmdpath/meta.json` | recorder provenance |

Composed across all twelve:

- `c2nav49_matrix_report.json` — `c2nav46_matrix_report.py` output
- `c2nav49_clearance.json` — `c2nav49_clearance.py` output

**Deliberately NOT committed** (14 MB for the sweep): `cmdpath/trace.csv` and
`events.csv`, `mission.log`, `sim.log`, `hrec.csv`, `state_stream.txt`,
`runner.log`, `graph_chain.txt`. Same convention as `c2nav48_live/`.

**So the two report scripts do not re-run against this directory.**
`c2nav46_matrix_report.py` reads `mission.log` for the grasp base-x, and
`c2nav49_clearance.py` needs `trace.csv` — pointed here it prints
`NO TRACE ... the counts below are vacuous, not evidence`, which is correct
and is why it says it in those words. This directory is the **record**; the
JSON beside it is the reading. To regenerate from scratch, do a fresh sweep
and point the reports at *that* directory.

Full run trees for this sweep are at `~/coco_nav_runs/c2nav49_matrix/`, which
is outside the repository and is not backed up.
