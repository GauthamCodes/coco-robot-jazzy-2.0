# Raw measurement data

Backing data for the numbers in [../RESULTS.md](../RESULTS.md), committed
so the claims are checkable rather than asserted.

| File | What |
|---|---|
| `nav2_goals.json` | 10 Nav2 goals: target, result, seconds |
| `pick_place_runs.txt` | 8 pick-and-place runs: exit code, completion, gripper stall position, cylinder height afterwards |
| `*.monitor.csv` | SB3 Monitor logs behind the learning curve (below) |

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
