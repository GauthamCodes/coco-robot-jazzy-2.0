#!/usr/bin/env python3
"""Draw the localization-health figure used in HOW_TO_RUN.md.

Reads two committed C2-M5.1 recorder CSVs and plots what the monitor
actually published, beside what the wheels were actually commanded:

  c2m51_nominal.csv          a healthy fetch, which ends COMPLETE
  c2m51_diverged_recover.csv the same mission with a 3 m pose error
                             injected at RETURN_HOME, which ends ABORT

Nothing here computes a new result. Every value drawn is a column the
recorder wrote during those runs; the figure exists so the shape of the
signal can be read without opening the CSV.

    python3 c2m51_plot.py                 # writes ../images/demo_localization.png
    python3 c2m51_plot.py --out other.png
"""
from __future__ import annotations

import argparse
import csv
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
INK = '#1c2430'
MUTED = '#6b7785'
GRID = '#dde3ea'
BAD = '#c0392b'
GOOD = '#2f7d63'
UNK = '#c9d0d8'
LINE = '#2f6f9f'

VERDICT_COLOUR = {'CONSISTENT': GOOD, 'INCONSISTENT': BAD, 'UNKNOWN': UNK}


def num(v):
    """Recorder columns carry '--' and 'nan' for 'no reading'."""
    try:
        return float(v)
    except (TypeError, ValueError):
        return math.nan


def load(name):
    with open(os.path.join(HERE, name), newline='') as fh:
        return list(csv.DictReader(fh))


def runs_of(rows, key):
    """Contiguous (t0, t1, value) spans of rows[key] over sim time."""
    out, start, cur = [], None, None
    for r in rows:
        t, v = num(r['t_sim']), r[key]
        if v != cur:
            if cur is not None:
                out.append((start, t, cur))
            start, cur = t, v
    if cur is not None:
        out.append((start, num(rows[-1]['t_sim']), cur))
    return out


def verdict_band(ax, rows, y, label):
    for t0, t1, v in runs_of(rows, 'verdict'):
        c = VERDICT_COLOUR.get(v)
        if c is None:                      # the leading '--' before first sample
            continue
        ax.barh(y, max(t1 - t0, 0.35), left=t0, height=0.52, color=c, lw=0)
    ax.text(-2.5, y, label, ha='right', va='center', fontsize=9, color=INK)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(HERE, '..', 'images',
                                                  'demo_localization.png'))
    args = ap.parse_args()

    nom = load('c2m51_nominal.csv')
    div = load('c2m51_diverged_recover.csv')

    fig, (ax0, ax1, ax2) = plt.subplots(
        3, 1, figsize=(9.6, 5.9), sharex=True,
        gridspec_kw={'height_ratios': [0.85, 1.55, 0.95], 'hspace': 0.22})
    fig.patch.set_facecolor('white')

    # ---- panel 0: the verdict the monitor published, run against run ----
    verdict_band(ax0, nom, 1, 'healthy run')
    verdict_band(ax0, div, 0, 'injected run')
    ax0.set_ylim(-0.6, 2.5)
    ax0.set_yticks([])
    ax0.set_title('Localization failure detection — the monitor’s verdict, its '
                  'signal, and what reached the wheels',
                  color=INK, fontsize=11.5, pad=26, loc='left')
    ax0.legend(handles=[Patch(facecolor=GOOD, label='CONSISTENT'),
                        Patch(facecolor=UNK, label='UNKNOWN (off mapped ground)'),
                        Patch(facecolor=BAD, label='INCONSISTENT')],
               loc='upper left', ncol=3, fontsize=8.2, frameon=False,
               labelcolor=INK, borderaxespad=0.1)

    # spans where the injected run was called INCONSISTENT, echoed below
    bad = [(t0, t1) for t0, t1, v in runs_of(div, 'verdict')
           if v == 'INCONSISTENT']
    for ax in (ax1, ax2):
        for t0, t1 in bad:
            ax.axvspan(t0, t1, color=BAD, alpha=0.12, lw=0)

    # ---- panel 1: the signal the verdict is computed from ---------------
    ax1.plot([num(r['t_sim']) for r in nom], [num(r['d']) for r in nom],
             color='#b9c1cb', lw=1.2, label='healthy run — ends COMPLETE')
    ax1.plot([num(r['t_sim']) for r in div], [num(r['d']) for r in div],
             color=LINE, lw=1.4, label='injected run — ends ABORT')
    ax1.set_ylabel('scan-vs-map\ndistance $d$ (m)', color=INK, fontsize=9.5)
    ax1.set_ylim(0, 0.92)
    ax1.legend(loc='upper left', fontsize=8.4, frameon=False, labelcolor=INK,
               ncol=1)

    # ---- panel 2: what the wheels were actually told --------------------
    ax2.plot([num(r['t_sim']) for r in div], [num(r['wheel_vx']) for r in div],
             color=INK, lw=1.0)
    ax2.set_ylabel('commanded\n$v_x$ (m/s)', color=INK, fontsize=9.5)
    ax2.set_xlabel('mission time (s, simulated)', color=INK, fontsize=10)
    ax2.set_xlim(-6, 182)

    # ---- the three moments ----------------------------------------------
    inj = next((num(r['t_sim']) for r in div if r['verdict'] == 'INCONSISTENT'),
               None)
    relo = next((num(r['t_sim']) for r in div if r['state'] == 'RELOCALIZE'),
                None)
    abort = next((num(r['t_sim']) for r in div if r['state'] == 'ABORT'), None)

    for t, txt, ha, off in ((inj, '3 m error injected,\ndetected', 'right', -7),
                            (relo, 'RECOVERY →\nRELOCALIZE', 'left', 6),
                            (abort, 'ABORT\nRETURN_FAILED', 'left', 6)):
        if t is None:
            continue
        for ax in (ax0, ax1, ax2):
            ax.axvline(t, color=BAD, lw=0.9, ls=':', alpha=0.85)
        ax1.annotate(txt, xy=(t, 0.80), xytext=(off, 0),
                     textcoords='offset points', color=BAD, fontsize=8.2,
                     va='top', ha=ha)

    for ax in (ax0, ax1, ax2):
        ax.grid(True, axis='x', color=GRID, lw=0.7)
        ax.set_axisbelow(True)
        for s in ('top', 'right'):
            ax.spines[s].set_visible(False)
        for s in ('left', 'bottom'):
            ax.spines[s].set_color(GRID)
        ax.tick_params(colors=MUTED, labelsize=8.8)
    ax1.grid(True, axis='y', color=GRID, lw=0.7)
    ax2.grid(True, axis='y', color=GRID, lw=0.7)
    ax0.spines['left'].set_visible(False)

    fig.text(0.012, 0.018,
             'Source: docs/data/c2m51_nominal.csv, c2m51_diverged_recover.csv '
             '— columns exactly as localization_monitor published them.\n'
             'The healthy run is INCONSISTENT 0 times in 175 s. The recovery '
             'restores the health signal; it does not restore a pose Nav2 can '
             'plan from, so the mission ends ABORT.',
             fontsize=7.5, color=MUTED, linespacing=1.5)

    fig.subplots_adjust(left=0.128, right=0.986, top=0.905, bottom=0.175)
    out = os.path.abspath(args.out)
    fig.savefig(out, dpi=132, facecolor='white')
    print(f'wrote {out}')


if __name__ == '__main__':
    main()
