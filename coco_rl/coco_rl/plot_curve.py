# Copyright 2026 Gautham Anil
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
plot_curve.py
=============
Render a PPO learning curve from SB3 Monitor CSVs.

Usage:
  python3 -m coco_rl.plot_curve run.monitor.csv [more.monitor.csv ...] -o out.png

Multiple CSVs are concatenated in order with cumulative step offsets
(useful after --resume runs). Faint dots are per-episode returns; the
line is a rolling mean.

The output path is an explicit -o/--out flag on purpose: an earlier
version took it as the last positional argument, so a single-argument
invocation wrote the PNG over the input Monitor CSV and destroyed the
training log before failing.
"""

import argparse
import csv
import sys

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
    ap.add_argument('csvs', nargs='+', metavar='MONITOR_CSV',
                    help='SB3 Monitor CSV(s), concatenated in order')
    ap.add_argument('-o', '--out', required=True, metavar='PNG',
                    help='output PNG path')
    args = ap.parse_args()
    csvs, out = args.csvs, args.out

    if not out.endswith('.png'):
        sys.exit(f'refusing to write {out}: --out must be a .png path')

    steps, rewards = load(csvs)
    if not rewards:
        sys.exit(f'no episodes found in {", ".join(csvs)} — nothing to plot')

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
