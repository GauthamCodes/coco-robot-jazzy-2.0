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

"""
c2m4_analysis.py — turn the 60-placement benchmark into the C2-M4.1 answer.

Post-processing only. It reads `c2m4_benchmark.csv`, reads nothing live,
and writes tables and one plot. Running it twice on the same CSV gives
the same numbers, which is the property that makes the result
reproducible without a simulator.

WHAT THE IK COLUMN IS, and why it is honest
-------------------------------------------
The benchmark measures perception with the robot *placed*, at stand-offs
from 0.30 to 0.90 m. The arm reaches to base-x 0.157. So the target is
out of the arm's workspace in every single placement, and asking "is the
measured pose reachable *from here*" would answer OUT_OF_WORKSPACE 60
times and say nothing.

The question that matters is the one `target_pose.reachability_after_
approach` asks: **the approach drives straight forward**, so it sets x to
`approach_stop_x(colour)` — inside the grasp window by construction — and
leaves y untouched. Therefore the only thing perception's measurement
decides is y. This module re-derives that verdict from the *measured*
pose in the CSV, using the same `coco_config` bounds the robot uses, and
it is a deterministic function of a number perception produced. No
ground truth enters it.

That is the whole perception -> IK -> grasp chain for this geometry, and
it is why a 1-2 mm lateral error is not a rounding detail: the lateral
budget is GRASP_MAX_LATERAL = 10 mm, and the benchmark deliberately
places one of its three laterals exactly on that boundary.

Usage
-----
    python3 c2m4_analysis.py c2m4_benchmark.csv --plot c2m4_scatter.png
"""

import argparse
import csv
import math
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '..', 'coco_config'))
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '..',
    'coco_moveit_config', 'scripts'))

from coco_config.robot import (          # noqa: E402
    GRASP_HOVER_CLEARANCE, GRASP_MAX_LATERAL, TARGET_GRASP_Z,
    approach_stop_x, approach_window)

try:
    import arm_ik                        # noqa: E402
    IK = arm_ik.ik_or_none
except Exception:                        # pragma: no cover - reported, not hidden
    IK = None


# ── outcome vocabulary (section 10 of the C2-M4.1 brief) ────────────────
# Kept separate rather than collapsed into "failed", because these fail
# for different reasons and only one of them is perception's fault.
NOT_DETECTED = 'target not detected'
DEPTH_INVALID = 'depth invalid'
NO_TRANSFORM = 'transform unavailable'
OFF_ARM_PLANE = 'target outside workspace (lateral)'
OUT_OF_WORKSPACE = 'target outside workspace (range)'
IK_UNAVAILABLE = 'IK unavailable'
FEASIBLE = 'grasp-feasible'


def parse_status(text):
    """`key=value key=value ...` -> dict. '--' becomes None."""
    fields = {}
    for token in (text or '').split():
        if '=' not in token:
            continue
        key, _, value = token.partition('=')
        fields[key] = None if value == '--' else value
    return fields


def as_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def ik_verdict(colour, y):
    """
    Would the arm reach this target *after* the approach?

    x is `approach_stop_x(colour)` because the approach drives straight
    forward and stops there; y is what perception measured and the
    approach does not change. See the module docstring.
    """
    if IK is None:
        return IK_UNAVAILABLE
    stop = approach_stop_x(colour)
    if stop is None:
        return IK_UNAVAILABLE
    if abs(y) > GRASP_MAX_LATERAL:
        return OFF_ARM_PLANE
    if (IK(stop, TARGET_GRASP_Z) is None
            or IK(stop, TARGET_GRASP_Z + GRASP_HOVER_CLEARANCE) is None):
        return OUT_OF_WORKSPACE
    return FEASIBLE


def truth_verdict(colour, lateral):
    """The same verdict computed from the COMMANDED lateral.

    Evaluation only. This is what the answer would have been with a
    perfect sensor, and comparing it against `ik_verdict` is what
    isolates 'perception changed the decision' from 'the target was
    never in the workspace'.
    """
    if IK is None:
        return IK_UNAVAILABLE
    # The robot is placed at lane_y + lateral, so the target sits at
    # -lateral in base_footprint.
    return ik_verdict(colour, -lateral)


def median(values):
    ordered = sorted(values)
    n = len(ordered)
    if not n:
        return None
    mid = n // 2
    return ordered[mid] if n % 2 else (ordered[mid - 1] + ordered[mid]) / 2.0


def mm(value):
    return '--' if value is None else f'{value * 1000:.2f}'


def load(path):
    with open(path, newline='') as handle:
        rows = list(csv.DictReader(handle))
    for row in rows:
        row['status_fields'] = parse_status(row.get('status'))
        for key in ('standoff_cmd', 'lateral_cmd', 'dx', 'dy', 'dz',
                    'err_horizontal', 'err_vertical', 'err_norm',
                    'score', 'spread_x', 'spread_y', 'est_y'):
            row[key] = as_float(row.get(key))
        row['frames'] = int(row.get('frames') or 0)
        row['detected'] = int(row.get('detected') or 0)
    return rows


def classify(row):
    """One placement -> (perception outcome, IK verdict, truth verdict)."""
    fields = row['status_fields']
    validity = fields.get('validity')
    if row['result'] != 'OK':
        if validity == 'DEPTH_INVALID':
            return DEPTH_INVALID, None, None
        if validity in ('NO_TRANSFORM', 'STALE_TRANSFORM'):
            return NO_TRANSFORM, None, None
        return NOT_DETECTED, None, None
    measured_y = row['est_y']
    return ('detected',
            ik_verdict(row['colour'], measured_y),
            truth_verdict(row['colour'], row['lateral_cmd']))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('csv', help='c2m4_benchmark.csv')
    parser.add_argument('--plot', default=None, help='write a scatter here')
    args = parser.parse_args()

    rows = load(args.csv)
    for row in rows:
        row['outcome'], row['ik'], row['ik_truth'] = classify(row)

    ok = [r for r in rows if r['result'] == 'OK']
    print(f'placements: {len(rows)}   measured OK: {len(ok)}')
    print(f'IK solver: {"arm_ik loaded" if IK else "UNAVAILABLE"}')
    frames_total = sum(r['frames'] for r in rows)
    frames_det = sum(r['detected'] for r in rows)
    print(f'frames: {frames_det}/{frames_total} carried a detection')

    window = approach_window(rows[0]['colour']) if rows else None
    print(f'grasp window {window}, max lateral {GRASP_MAX_LATERAL * 1000:.0f} mm')

    # ── per-placement table ──────────────────────────────────────────
    print('\n== ALL PLACEMENTS (errors in mm) ==')
    header = (f"{'colour':>6} {'s/off':>6} {'lat':>7} {'det':>7} "
              f"{'dx':>7} {'dy':>7} {'dz':>7} {'|h|':>7} {'qual':>6} "
              f"{'cand':>4} {'reach_appr':>14} {'IK(measured)':>14}")
    print(header)
    print('-' * len(header))
    for row in rows:
        fields = row['status_fields']
        print(f"{row['colour']:>6} {row['standoff_cmd']:>6.2f} "
              f"{row['lateral_cmd']:>+7.3f} "
              f"{row['detected']:>3}/{row['frames']:<3} "
              f"{mm(row['dx']):>7} {mm(row['dy']):>7} {mm(row['dz']):>7} "
              f"{mm(row['err_horizontal']):>7} "
              f"{(fields.get('qual') or '--'):>6} "
              f"{(fields.get('cand') or '--'):>4} "
              f"{(fields.get('reach_appr') or '--'):>14} "
              f"{(row['ik'] or '--'):>14}")

    def summarise(label, groups):
        print(f'\n== {label} ==')
        print(f"{'group':>10} {'n':>3} {'det':>9} {'|h| min':>8} "
              f"{'|h| med':>8} {'|h| max':>8} {'|dy| med':>9} "
              f"{'|dy| max':>9}")
        for key in sorted(groups):
            members = groups[key]
            good = [r for r in members if r['result'] == 'OK']
            det = sum(r['detected'] for r in members)
            tot = sum(r['frames'] for r in members)
            if not good:
                print(f'{str(key):>10} {len(members):>3} {det:>4}/{tot:<4} '
                      f'{"--":>8}')
                continue
            horizontal = [r['err_horizontal'] for r in good]
            lateral = [abs(r['dy']) for r in good]
            print(f'{str(key):>10} {len(members):>3} {det:>4}/{tot:<4} '
                  f"{mm(min(horizontal)):>8} {mm(median(horizontal)):>8} "
                  f"{mm(max(horizontal)):>8} {mm(median(lateral)):>9} "
                  f"{mm(max(lateral)):>9}")

    by_colour = {}
    by_standoff = {}
    by_lateral = {}
    for row in rows:
        by_colour.setdefault(row['colour'], []).append(row)
        by_standoff.setdefault(row['standoff_cmd'], []).append(row)
        by_lateral.setdefault(row['lateral_cmd'], []).append(row)
    summarise('BY COLOUR', by_colour)
    summarise('BY STAND-OFF (m)', by_standoff)
    summarise('BY LATERAL (m)', by_lateral)

    # ── the perception -> IK correlation, which is the point ─────────
    print('\n== IK FEASIBILITY vs LATERAL ==')
    print(f"{'lateral':>8} {'n':>3} {'feasible(measured)':>19} "
          f"{'feasible(truth)':>16} {'disagreements':>14}")
    for key in sorted(by_lateral):
        members = [r for r in by_lateral[key] if r['result'] == 'OK']
        meas = sum(1 for r in members if r['ik'] == FEASIBLE)
        truth = sum(1 for r in members if r['ik_truth'] == FEASIBLE)
        disagree = sum(1 for r in members if r['ik'] != r['ik_truth'])
        print(f'{key:>+8.3f} {len(members):>3} {meas:>19} {truth:>16} '
              f'{disagree:>14}')

    print('\n== WHERE THE LATERAL DECISION SITS ==')
    print('|y_measured| against the 10.0 mm budget, per commanded lateral:')
    for key in sorted(by_lateral):
        members = [r for r in by_lateral[key] if r['result'] == 'OK']
        if not members:
            continue
        margins = [abs(r['est_y']) for r in members]
        print(f'  lateral {key:+.3f}: |y| min {mm(min(margins))} '
              f'median {mm(median(margins))} max {mm(max(margins))} mm '
              f'-> margin to budget '
              f'{mm(GRASP_MAX_LATERAL - max(margins))} mm at worst')

    # ── min_range behaviour at the envelope floor ────────────────────
    print('\n== MIN_RANGE AT THE OPERATING FLOOR (0.30 m) ==')
    floor = [r for r in rows if r['standoff_cmd'] == 0.30
             and r['result'] == 'OK']
    if floor:
        print(f"{'colour':>6} {'lat':>7} {'dx':>8} {'qual':>8}")
        for row in floor:
            print(f"{row['colour']:>6} {row['lateral_cmd']:>+7.3f} "
                  f"{mm(row['dx']):>8} "
                  f"{(row['status_fields'].get('qual') or '--'):>8}")
        dxs = [r['dx'] for r in floor]
        print(f'  dx at 0.30 m: min {mm(min(dxs))} median '
              f'{mm(median(dxs))} max {mm(max(dxs))} mm')
        print('  (C2-M4.0 measured +4.1 to +8.3 mm at 0.28 m with the same '
              'gate; the radius-proportional signature is what to look for)')

    print('\n== FRAME-TO-FRAME SPREAD ==')
    spreads = [max(r['spread_x'], r['spread_y']) for r in ok
               if r['spread_x'] is not None]
    if spreads:
        print(f'  max over all placements: {mm(max(spreads))} mm '
              f'({"bias, not noise" if max(spreads) == 0 else "some jitter"})')

    if args.plot:
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except Exception as exc:
            print(f'\nno plot: matplotlib unavailable ({exc})')
            return
        fig, axes = plt.subplots(1, 2, figsize=(12, 5))
        colours = {'red': '#d62728', 'green': '#2ca02c',
                   'blue': '#1f77b4', 'yellow': '#bcbd22'}

        for colour, members in by_colour.items():
            good = [r for r in members if r['result'] == 'OK']
            axes[0].plot([r['standoff_cmd'] for r in good],
                         [r['err_horizontal'] * 1000 for r in good],
                         'o', color=colours.get(colour, 'k'), label=colour,
                         alpha=0.7)
        axes[0].set_xlabel('commanded stand-off (m)')
        axes[0].set_ylabel('horizontal error (mm)')
        axes[0].set_title('Gazebo target-localization error vs range')
        axes[0].legend()
        axes[0].grid(alpha=0.3)

        # The decision plot: measured |y| against the arm's lateral budget.
        for row in ok:
            feasible = row['ik'] == FEASIBLE
            axes[1].plot(row['lateral_cmd'] * 1000, abs(row['est_y']) * 1000,
                         'o' if feasible else 'x',
                         color='#2ca02c' if feasible else '#d62728',
                         alpha=0.75)
        axes[1].axhline(GRASP_MAX_LATERAL * 1000, ls='--', color='k',
                        label=f'GRASP_MAX_LATERAL = '
                              f'{GRASP_MAX_LATERAL * 1000:.0f} mm')
        axes[1].set_xlabel('commanded lateral offset (mm)')
        axes[1].set_ylabel('|measured y| in base_footprint (mm)')
        axes[1].set_title('Lateral estimate vs the arm\'s budget\n'
                          'o = grasp-feasible, x = off the arm plane')
        axes[1].legend()
        axes[1].grid(alpha=0.3)

        fig.tight_layout()
        fig.savefig(args.plot, dpi=130)
        print(f'\nwrote {args.plot}')


if __name__ == '__main__':
    main()
