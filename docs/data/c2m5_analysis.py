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
c2m5_analysis — score the C2-M5.0 recordings, per state and per run.

Reads one or more CSVs written by ``c2m5_locrec.py`` and prints, for the
RETURN_HOME leg of each (or any state named with ``--state``):

  * the localization error, from ground truth. **Offline only.** It is
    the thing being predicted, never an input to the prediction.
  * every deployable signal beside it, so the two can be compared:
    covariance, map->odom step, the scan-vs-map likelihood, odom/AMCL
    displacement agreement, command rates, RTF, collision-monitor state.

The separation is the entire point of the milestone, so the report keeps
it typographically: ``gt_``/``err_`` columns are printed in their own
block, under a heading that says they are not available to the robot.

    python3 c2m5_analysis.py healthy1.csv diverged1.csv --state RETURN_HOME
    python3 c2m5_analysis.py *.csv --compare      # the discriminator table

What ``--compare`` answers
--------------------------
For each candidate signal, whether the healthy and bad distributions are
separated at all, reported as the two ranges side by side. It deliberately
does NOT emit a threshold. Two runs are two runs; a number picked to split
two samples is a number picked to split two samples, and this repo's
standing rule is that an unjustified threshold is worse than an admitted
gap in the evidence.
"""

import argparse
import csv
import math
import os
import sys

import numpy as np

# Columns the robot cannot see. Named once, so the boundary is checkable
# rather than remembered.
GROUND_TRUTH = ('gt_x', 'gt_y', 'gt_yaw', 'gt_age', 'err_xy', 'err_yaw',
                'gt_map_x', 'gt_map_y')

# map <- world. Taken from coco_config when it is importable, so this file
# cannot drift from the constant the mission steers by; the literal is the
# same value and exists only so the analysis runs without a sourced
# workspace.
try:                                                   # pragma: no cover
    from coco_config.robot import SPAWN_XY
    WORLD_TO_MAP_X = -SPAWN_XY[0]
    WORLD_TO_MAP_Y = -SPAWN_XY[1]
except Exception:                                      # pragma: no cover
    WORLD_TO_MAP_X, WORLD_TO_MAP_Y = 2.0, 0.0

# The deployable candidates, in the order the report prints them.
SIGNALS = [
    ('amcl_cxx', 'AMCL covariance xx', 'm^2'),
    ('amcl_cyy', 'AMCL covariance yy', 'm^2'),
    ('amcl_caa', 'AMCL covariance yaw', 'rad^2'),
    ('cov_sigma_xy', 'AMCL sigma_xy = sqrt(cxx+cyy)', 'm'),
    ('lik_mean_d', 'scan-vs-map mean endpoint distance', 'm'),
    ('lik_p90_d', 'scan-vs-map p90 endpoint distance', 'm'),
    ('lik_frac_near', 'scan endpoints within 0.10 m of an obstacle', '-'),
    ('mo_step', 'map->odom step per 0.1 s sample', 'm'),
    ('amcl_age', 'age of the latest /amcl_pose', 's'),
    ('drift_rate', '|d(map->odom)| over 1 s', 'm/s'),
    ('hz_cmd_nav', '/cmd_vel_nav rate', 'Hz'),
    ('hz_cmd_smoothed', '/cmd_vel_smoothed rate', 'Hz'),
    ('hz_cmd_out', '/cmd_vel rate', 'Hz'),
    ('hz_cmd_wheels', '/diff_drive_controller/cmd_vel rate', 'Hz'),
    ('hz_scan', '/scan rate', 'Hz'),
    ('hz_amcl', '/amcl_pose rate', 'Hz'),
    ('rtf', 'real-time factor d(sim)/d(wall)', '-'),
    ('speed_wheels', 'commanded |linear| at the wheels', 'm/s'),
    ('dist_to_goal', 'AMCL pose to the end of the current plan', 'm'),
]


def state_of(text):
    """The bare state label, whatever shape the column is in.

    The first recording was made before ``c2m5_locrec`` learned that
    ``/mission/state`` is a whole key=value line, so its state column
    holds the line rather than the label. Parsing both here keeps that
    run readable instead of re-running the simulator for a text bug.
    """
    text = (text or '').strip()
    for token in text.split():
        if token.startswith('state='):
            return token[len('state='):] or '--'
    return text.split()[0] if text.split() else '--'


def load(path):
    """Read a recorder CSV into a dict of numpy arrays, plus derived columns."""
    with open(path) as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise SystemExit(f'{path}: empty')
    cols = {}
    for key in rows[0]:
        raw = [r[key] for r in rows]
        try:
            cols[key] = np.array([float(v) if v not in ('', None) else np.nan
                                  for v in raw])
        except ValueError:
            cols[key] = np.array(raw, dtype=object)
    cols['state'] = np.array([state_of(r['state']) for r in rows], dtype=object)

    # ── derived, deployable ──────────────────────────────────────────────
    cols['cov_sigma_xy'] = np.sqrt(cols['amcl_cxx'] + cols['amcl_cyy'])
    cols['speed_wheels'] = np.abs(cols['wheel_vx'])
    # map->odom movement over a one-second window: the rate at which the
    # filter is correcting odometry, as opposed to its instantaneous step.
    n = len(cols['t_sim'])
    drift = np.full(n, np.nan)
    win = 10                                   # 10 samples at 10 Hz
    for i in range(win, n):
        dx = cols['mo_x'][i] - cols['mo_x'][i - win]
        dy = cols['mo_y'][i] - cols['mo_y'][i - win]
        dt = cols['t_sim'][i] - cols['t_sim'][i - win]
        if dt > 1e-6:
            drift[i] = math.hypot(dx, dy) / dt
    cols['drift_rate'] = drift

    # Distance from the pose Nav2 is steering by to the end of the plan it
    # is following. Both are in the map frame, so this is the robot's OWN
    # view of how far it has left — the quantity a progress check would
    # use, and deployable.
    cols['dist_to_goal'] = np.hypot(cols['goal_x'] - cols['amcl_x'],
                                    cols['goal_y'] - cols['amcl_y'])
    # Replans, counted as changes in the published plan between 10 Hz
    # samples. An UNDERCOUNT by construction: two replans inside one
    # sample interval, or a replan that returns an identically long path,
    # both read as one. Reported as a floor, not a count.
    plan_changed = np.zeros(n, dtype=bool)
    plan_changed[1:] = (np.abs(np.diff(cols['plan_len'])) > 1e-6) | (
        np.diff(cols['plan_n']) != 0)
    cols['plan_changed'] = plan_changed.astype(float)

    # ── derived, GROUND TRUTH — scoring only ─────────────────────────────
    # AMCL is in the MAP frame; gz is in the WORLD frame, and the two are
    # not the same origin. slam_toolbox anchored the map at the robot's
    # SLAM start, which is the Gazebo spawn at world (-2, 0), so map
    # (0,0) IS world (-2,0). mission_states.py already names the offset
    # WORLD_TO_MAP_X = -SPAWN_XY[0]; subtracting it here is the whole
    # difference between "AMCL is 2.2 m wrong on a mission that
    # succeeded" — which is what the raw columns say, and is nonsense —
    # and the real error, which is a few tens of centimetres.
    gt_map_x = cols['gt_x'] + WORLD_TO_MAP_X
    gt_map_y = cols['gt_y'] + WORLD_TO_MAP_Y
    cols['gt_map_x'], cols['gt_map_y'] = gt_map_x, gt_map_y
    cols['err_xy'] = np.hypot(cols['amcl_x'] - gt_map_x,
                              cols['amcl_y'] - gt_map_y)
    d = cols['amcl_yaw'] - cols['gt_yaw']
    cols['err_yaw'] = np.abs(np.arctan2(np.sin(d), np.cos(d)))
    cols['_path'] = path
    cols['_name'] = os.path.basename(path).replace('.csv', '')
    return cols


def mask_state(cols, state):
    return cols['state'] == state


def stats(v):
    v = np.asarray(v, dtype=float)
    v = v[np.isfinite(v)]
    if v.size == 0:
        return None
    return dict(n=v.size, min=v.min(), med=float(np.median(v)),
                mean=v.mean(), p90=float(np.percentile(v, 90)), max=v.max())


def fmt(s, width=9, prec=4):
    if s is None:
        return '     --  '
    return f'{s:{width}.{prec}f}'


def report_states(cols):
    """Per-state durations, so the leg boundaries are visible."""
    print(f'\n=== {cols["_name"]}: states ===')
    t = cols['t_sim']
    st = cols['state']
    print(f'{"state":<22}{"samples":>8}{"t_start":>10}{"t_end":>10}'
          f'{"dur_s":>9}')
    i = 0
    while i < len(st):
        j = i
        while j + 1 < len(st) and st[j + 1] == st[i]:
            j += 1
        print(f'{str(st[i]):<22}{j - i + 1:>8}{t[i]:>10.1f}{t[j]:>10.1f}'
              f'{t[j] - t[i]:>9.1f}')
        i = j + 1


def report_leg(cols, state):
    m = mask_state(cols, state)
    print(f'\n=== {cols["_name"]}: {state} ===')
    if not m.any():
        print(f'  no samples in {state}')
        return None
    t = cols['t_sim'][m]
    print(f'  {m.sum()} samples, {t[-1] - t[0]:.1f} s of simulation time')

    cm = cols['cm_action'][m]
    poly = cols['cm_polygon'][m]
    print('\n  collision monitor, by action:')
    for action in sorted(set(cm)):
        k = (cm == action)
        polys = sorted(set(poly[k]))
        print(f'    {str(action):<12} {k.sum():>5} samples '
              f'({100.0 * k.sum() / len(cm):5.1f}%)  polygons: {",".join(map(str, polys))}')

    nav = cols['nav_status'][m]
    print('\n  navigate_to_pose status, by sample:')
    for s in sorted(set(nav)):
        k = (nav == s)
        print(f'    {str(s):<12} {k.sum():>5} samples '
              f'({100.0 * k.sum() / len(nav):5.1f}%)')

    print('\n  DEPLOYABLE signals (the robot can compute all of these):')
    print(f'    {"signal":<44}{"unit":>7}{"n":>7}{"min":>10}{"median":>10}'
          f'{"p90":>10}{"max":>10}')
    out = {}
    for key, label, unit in SIGNALS:
        if key not in cols:
            continue
        s = stats(cols[key][m])
        out[key] = s
        if s is None:
            print(f'    {label:<44}{unit:>7}{"--":>7}')
            continue
        print(f'    {label:<44}{unit:>7}{s["n"]:>7}{s["min"]:>10.4f}'
              f'{s["med"]:>10.4f}{s["p90"]:>10.4f}{s["max"]:>10.4f}')

    print(f'\n  GROUND TRUTH — offline scoring only, NOT a health input.')
    print(f'  (gz world pose shifted into the map frame by '
          f'WORLD_TO_MAP_X={WORLD_TO_MAP_X:+.2f}, '
          f'WORLD_TO_MAP_Y={WORLD_TO_MAP_Y:+.2f}.)')
    for key, label, unit in (('err_xy', 'AMCL position error vs gz', 'm'),
                             ('err_yaw', 'AMCL yaw error vs gz', 'rad')):
        s = stats(cols[key][m])
        out[key] = s
        if s is None:
            print(f'    {label:<44}{unit:>7}{"--":>7}')
            continue
        print(f'    {label:<44}{unit:>7}{s["n"]:>7}{s["min"]:>10.4f}'
              f'{s["med"]:>10.4f}{s["p90"]:>10.4f}{s["max"]:>10.4f}')

    changes = int(np.nansum(cols['plan_changed'][m]))
    print(f'\n    plan updates observed during {state}: >= {changes} '
          f'(changes in the published plan between 10 Hz samples; an '
          f'undercount, see the code)')

    gx, gy = cols['gt_x'][m], cols['gt_y'][m]
    good = np.isfinite(gx) & np.isfinite(gy)
    if good.any():
        travelled = float(np.nansum(np.hypot(np.diff(gx[good]),
                                             np.diff(gy[good]))))
        print(f'\n    true distance travelled during {state}: '
              f'{travelled:.3f} m')
        print(f'    true pose at the end: ({gx[good][-1]:.3f}, '
              f'{gy[good][-1]:.3f})')
    return out


def compare(runs, state):
    """Healthy vs bad, signal by signal. Ranges, deliberately no threshold."""
    print(f'\n=== discriminator table, {state} ===')
    print('Ranges are min..max over the leg. NO threshold is proposed here:')
    print('see RESULTS.md for which of these separate and which do not.\n')
    names = [c['_name'] for c in runs]
    width = max(len(n) for n in names) + 2
    header = f'{"signal":<44}' + ''.join(f'{n:>{max(width, 26)}}' for n in names)
    print(header)
    print('-' * len(header))
    for key, label, unit in SIGNALS + [
            ('err_xy', 'GT position error (NOT deployable)', 'm')]:
        cells = []
        for cols in runs:
            m = mask_state(cols, state)
            s = stats(cols[key][m]) if (key in cols and m.any()) else None
            cells.append('--' if s is None
                         else f'{s["min"]:.3f}..{s["max"]:.3f}')
        print(f'{label:<44}' + ''.join(
            f'{c:>{max(width, 26)}}' for c in cells))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument('csv', nargs='+')
    ap.add_argument('--state', default='RETURN_HOME')
    ap.add_argument('--states', action='store_true',
                    help='print the per-state timeline for each run')
    ap.add_argument('--compare', action='store_true',
                    help='print the healthy-vs-bad range table')
    args = ap.parse_args()

    runs = [load(p) for p in args.csv]
    for cols in runs:
        if args.states:
            report_states(cols)
        report_leg(cols, args.state)
    if args.compare and len(runs) > 1:
        compare(runs, args.state)
    return 0


if __name__ == '__main__':
    sys.exit(main())
