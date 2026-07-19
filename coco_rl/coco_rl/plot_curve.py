"""
plot_curve.py
=============
Render a PPO learning curve from SB3 Monitor CSVs.

Usage:
  python3 -m coco_rl.plot_curve run.monitor.csv [more.monitor.csv ...] out.png

Multiple CSVs are concatenated in order with cumulative step offsets
(useful after --resume runs). Faint dots are per-episode returns; the
line is a rolling mean.
"""

import argparse
import csv

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

BLUE = '#2a78d6'
INK = '#333639'
MUTED = '#6b7075'


def load(csvs):
    rewards, lengths = [], []
    for src in csvs:
        with open(src) as f:
            f.readline()  # json comment header
            for row in csv.DictReader(f):
                rewards.append(float(row['r']))
                lengths.append(int(row['l']))
    steps, total = [], 0
    for n in lengths:
        total += n
        steps.append(total)
    return steps, rewards


def rolling_mean(values, window):
    out = []
    for i in range(len(values)):
        lo = max(0, i - window + 1)
        out.append(sum(values[lo:i + 1]) / (i + 1 - lo))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csvs', nargs='+',
                    help='Monitor CSV(s) followed by the output PNG path')
    args = ap.parse_args()
    csvs, out = args.csvs[:-1], args.csvs[-1]

    steps, rewards = load(csvs)
    win = max(1, min(20, len(rewards) // 5))
    roll = rolling_mean(rewards, win)

    fig, ax = plt.subplots(figsize=(8, 4.5), dpi=150)
    ax.scatter(steps, rewards, s=6, color=BLUE, alpha=0.25, linewidths=0)
    ax.plot(steps, roll, color=BLUE, linewidth=2)
    ax.set_xlabel('environment steps', color=INK)
    ax.set_ylabel('episode reward', color=INK)
    ax.set_title(f'PPO ramp traversal — episode reward '
                 f'(rolling mean, window {win})', color=INK, fontsize=11)
    ax.grid(alpha=0.15, linewidth=0.5)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(MUTED)
    ax.tick_params(colors=MUTED)
    fig.tight_layout()
    fig.savefig(out, facecolor='white')
    print(f'saved {out} ({len(rewards)} episodes, {steps[-1]} steps)')


if __name__ == '__main__':
    main()
