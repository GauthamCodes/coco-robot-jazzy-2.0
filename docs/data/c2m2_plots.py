#!/usr/bin/env python3
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

"""C2-M2.1 result figures, from ``docs/data/c2m2_benchmark.json``.

Four plots, each answering a question the milestone actually asks. No
decorative charts, every axis carries a unit, and nothing is drawn from a
number the benchmark did not measure.

Read-only. No ROS. Deliberately not installed by any ``CMakeLists.txt``,
same shape as ``map_audit.py`` and ``c2m2_sanity.py``::

    python3 docs/data/c2m2_plots.py

Reproduces every figure under ``docs/images/`` from the JSON alone.
"""

import json
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt          # noqa: E402
import numpy as np                       # noqa: E402


HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, 'c2m2_benchmark.json')
OUT = os.path.abspath(os.path.join(HERE, '..', 'images'))

# One colour per controller, used identically in every figure so the eye
# can carry a controller across plots.
COLOUR = {'B0': '#9e9e9e', 'B1': '#1f77b4', 'B2': '#d62728', 'B3': '#2ca02c'}
LABEL = {
    'B0': 'B0  open-loop',
    'B1': 'B1  fixed PD (deployable)',
    'B2': 'B2  privileged (true mu)',
    'B3': 'B3  observer (deployable)',
}
ROUTE_LABEL = {'a': 'Route A', 'b': 'Route B', 'c': 'Route C'}


def load(path=DATA):
    with open(path) as fh:
        return json.load(fh)


def cell(d, kind, route):
    return d['results'].get(f'{kind}/{route}', {})


def rows(d, kind, route):
    return cell(d, kind, route).get('rows', [])


def summary(d, kind, route):
    return cell(d, kind, route).get('summary', {})


def _save(fig, name):
    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, name)
    fig.savefig(p, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'  wrote {os.path.relpath(p, os.path.dirname(HERE))}')
    return p


# ── Plot 1: grade estimation error by route ──────────────────────────────
def plot_grade_error(d):
    """What the deployable estimator actually achieves, per route.

    Per-episode grade MAE, in degrees, scored on the ramp face only (both
    axles on one plane). Box per route plus the individual episodes, so
    the tail is visible rather than averaged away.
    """
    routes = d['config']['routes']
    data, labels = [], []
    for r in routes:
        v = [math.degrees(x['grade_mae']) for x in rows(d, 'B3', r)
             if x.get('grade_mae') is not None
             and not math.isnan(x.get('grade_mae', float('nan')))]
        if v:
            data.append(v)
            labels.append(f'{ROUTE_LABEL[r]}\n(n={len(v)})')

    fig, ax = plt.subplots(figsize=(7.5, 4.6))
    bp = ax.boxplot(data, labels=labels, showfliers=False,
                    medianprops=dict(color='black', lw=1.6),
                    boxprops=dict(lw=1.2), whiskerprops=dict(lw=1.2))
    for i, v in enumerate(data, start=1):
        x = np.random.default_rng(0).normal(i, 0.055, len(v))
        ax.plot(x, v, '.', ms=3.5, alpha=0.35, color='#1f77b4', zorder=1)
    ax.set_ylabel('grade estimation error, MAE (degrees)')
    ax.set_title('Plot 1  Grade estimation error by route\n'
                 'B3 observer, scored on the ramp face only '
                 '(both axles on one plane)', fontsize=10)
    ax.grid(axis='y', alpha=0.3)
    ax.set_axisbelow(True)
    for i, v in enumerate(data, start=1):
        ax.annotate(f'mean {np.mean(v):.2f}°',
                    (i, max(v) if v else 0), textcoords='offset points',
                    xytext=(0, 6), ha='center', fontsize=8)
    return _save(fig, 'c2m21_grade_error.png')


# ── Plot 2: the controller comparison the milestone is about ─────────────
def plot_controller_comparison(d):
    """B1 -> B2 -> B3 on the decision task, per route.

    Ascent is the frozen decision task. The 10-percentage-point rule is
    drawn against B2 so the verdict is readable off the figure.
    """
    routes = d['config']['routes']
    task = d['config']['decision_task']
    margin = d['config']['decision_margin_pp']
    kinds = ['B1', 'B2', 'B3']

    fig, ax = plt.subplots(figsize=(8.4, 4.8))
    w = 0.26
    xs = np.arange(len(routes))
    for i, k in enumerate(kinds):
        vals = [summary(d, k, r).get(task, float('nan')) * 100 for r in routes]
        ax.bar(xs + (i - 1) * w, vals, w, label=LABEL[k], color=COLOUR[k])
        for x, v in zip(xs + (i - 1) * w, vals):
            ax.annotate(f'{v:.1f}', (x, v), textcoords='offset points',
                        xytext=(0, 3), ha='center', fontsize=8)
    # the decision threshold, drawn per route against B2
    for j, r in enumerate(routes):
        b2 = summary(d, 'B2', r).get(task, float('nan')) * 100
        ax.plot([j - 1.5 * w, j + 1.5 * w], [b2 - margin] * 2,
                ls='--', lw=1.4, color='black', zorder=5,
                label='B2 minus 10 pp (RL threshold)' if j == 0 else None)
    ax.set_xticks(xs)
    ax.set_xticklabels([ROUTE_LABEL[r] for r in routes])
    ax.set_ylabel(f'{task} success rate (percent of 120 episodes)')
    ax.set_ylim(0, 108)
    ax.set_title('Plot 2  Controller comparison on the frozen decision task '
                 '(ascent)\nB3 below the dashed line on a route means RL is '
                 'justified there', fontsize=10)
    ax.legend(fontsize=8, loc='upper right')
    ax.grid(axis='y', alpha=0.3)
    ax.set_axisbelow(True)
    return _save(fig, 'c2m21_controller_comparison.png')


# ── Plot 3: cross-track distribution ─────────────────────────────────────
def plot_xtrack(d):
    """Path-holding, which is where the observer's gains actually act.

    Per-episode mean |cross-track|, metres, all four controllers, per
    route. This is the channel the lateral gain schedule touches.
    """
    routes = d['config']['routes']
    kinds = d['config']['controllers']
    fig, axes = plt.subplots(1, len(routes), figsize=(12.2, 4.4),
                             sharey=True)
    if len(routes) == 1:
        axes = [axes]
    for ax, r in zip(axes, routes):
        data, labels, colours = [], [], []
        for k in kinds:
            v = [x['xtrack_mean'] for x in rows(d, k, r)
                 if x.get('xtrack_mean') is not None
                 and not math.isnan(x.get('xtrack_mean', float('nan')))]
            if v:
                data.append(v)
                labels.append(k)
                colours.append(COLOUR[k])
        bp = ax.boxplot(data, labels=labels, showfliers=False,
                        patch_artist=True,
                        medianprops=dict(color='black', lw=1.5))
        for patch, c in zip(bp['boxes'], colours):
            patch.set_facecolor(c)
            patch.set_alpha(0.55)
        ax.set_title(ROUTE_LABEL[r], fontsize=10)
        ax.grid(axis='y', alpha=0.3)
        ax.set_axisbelow(True)
    axes[0].set_ylabel('mean |cross-track error| per episode (m)')
    fig.suptitle('Plot 3  Cross-track error distribution by controller '
                 '(lower is better)', fontsize=11)
    fig.tight_layout()
    return _save(fig, 'c2m21_xtrack.png')


# ── Plot 4: success rate by route and controller ─────────────────────────
def plot_success(d):
    """Ascent and completion side by side.

    Both are shown because the decision task is ascent and completion is
    dominated by the deck/bridge geometry, which is an M7 Phase 4
    decision and not a terrain-control result. Hiding completion would
    hide that; scoring the rule on it would measure the bridge.
    """
    routes = d['config']['routes']
    kinds = d['config']['controllers']
    fig, axes = plt.subplots(1, 2, figsize=(12.2, 4.5), sharey=True)
    for ax, key, title in ((axes[0], 'ascent', 'Ascent (reached the deck)'),
                           (axes[1], 'success',
                            'Completion (reached the bay)')):
        w = 0.2
        xs = np.arange(len(routes))
        for i, k in enumerate(kinds):
            vals = [summary(d, k, r).get(key, float('nan')) * 100
                    for r in routes]
            ax.bar(xs + (i - 1.5) * w, vals, w, label=LABEL[k],
                   color=COLOUR[k])
            for x, v in zip(xs + (i - 1.5) * w, vals):
                if not math.isnan(v):
                    ax.annotate(f'{v:.0f}', (x, v),
                                textcoords='offset points', xytext=(0, 2),
                                ha='center', fontsize=7)
        ax.set_xticks(xs)
        ax.set_xticklabels([ROUTE_LABEL[r] for r in routes])
        ax.set_title(title, fontsize=10)
        ax.grid(axis='y', alpha=0.3)
        ax.set_axisbelow(True)
        ax.set_ylim(0, 108)
    axes[0].set_ylabel('success rate (percent of 120 episodes)')
    # Below the axes: inside, it sat on top of B0's Route C bar label.
    axes[0].legend(fontsize=8, loc='upper center',
                   bbox_to_anchor=(1.03, -0.09), ncol=4, frameon=False)
    fig.suptitle('Plot 4  Success rate by route and controller '
                 '(120 seeds per cell)', fontsize=11)
    fig.tight_layout()
    return _save(fig, 'c2m21_success.png')


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DATA
    if not os.path.exists(path):
        print(f'no benchmark data at {path}', file=sys.stderr)
        return 1
    d = load(path)
    print(f'C2-M2.1 figures from {os.path.basename(path)}')
    plot_grade_error(d)
    plot_controller_comparison(d)
    plot_xtrack(d)
    plot_success(d)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
