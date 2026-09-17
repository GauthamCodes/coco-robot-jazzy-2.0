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

"""C2-NAV.42 -- what the residual 'wheels exceeded monitor' rows are.

After the /cmd_vel_nav fix, three fresh topology-B tours still show rows in
nav_bench's 10 Hz trace where |v_wheel| > |v_cmdvel| + 0.02 m/s (c2nav41's
monitor_authority), although the live graph has no path from the raw
controller to the wheels. Three offline analyses, no ROS:

trace     For each such nav_bench trace row: does the wheel value appear in
          the MONITOR's own output within +-0.5 s (5 rows)? Does it equal the
          raw controller at that row? Run on every arm with one definition.

stall     Tested and REJECTED as the explanation: are those rows nav_bench
          recorder stalls? A 'frozen' row repeats the previous row's ground
          truth pose and velocity exactly. Also the per-leg wheel/cmd_vel topic
          rate ratio nav_bench recorded.

messages  From c2nav42_cmdpath.py's events.csv (one row per wheel message,
          with the latest message on every upstream link as the recorder saw
          it): is each wheel command the latest monitor output, the latest
          relay output (a monitor message in transit), an EARLIER relay
          output (and how much earlier), or the raw controller's command?

  python3 -P docs/data/c2nav42_residual.py trace RUN_DIR [RUN_DIR ...]
  python3 -P docs/data/c2nav42_residual.py stall RUN_DIR [RUN_DIR ...]
  python3 -P docs/data/c2nav42_residual.py messages RECORDER_DIR
"""

import argparse
import csv
import glob
import json
import os
import statistics
import sys

TOL = 0.02          # c2nav41_topology.JITTER_TOL
WINDOW_ROWS = 5     # 0.5 s at 10 Hz
EXACT = 1e-6


def _f(row, key):
    try:
        return float(row[key])
    except (KeyError, ValueError, TypeError):
        return None


def _traces(run):
    tag = os.path.basename(os.path.normpath(run))
    return sorted(glob.glob(os.path.join(run, f'{tag}_traces', '*.csv')))


def trace(runs):
    total = {}
    per_run = []
    for run in runs:
        out = {'run': os.path.basename(os.path.normpath(run)), 'samples': 0, 'exceeded': 0,
               'wheel_in_monitor_within_0_5s': 0, 'only_later': 0, 'only_earlier': 0,
               'in_neither_window': 0, 'in_neither_window_and_eq_raw': 0}
        for path in _traces(run):
            rows = list(csv.DictReader(open(path)))
            for i, r in enumerate(rows):
                m, w = _f(r, 'v_cmdvel'), _f(r, 'v_wheel')
                if m is None or w is None:
                    continue
                out['samples'] += 1
                if abs(w) - abs(m) <= TOL:
                    continue
                out['exceeded'] += 1
                near = lambda xs: any(  # noqa: E731
                    _f(x, 'v_cmdvel') is not None and abs(_f(x, 'v_cmdvel') - w) <= TOL for x in xs)
                later = near(rows[i + 1:i + 1 + WINDOW_ROWS])
                earlier = near(rows[max(0, i - WINDOW_ROWS):i])
                if later or earlier:
                    out['wheel_in_monitor_within_0_5s'] += 1
                    out['only_later'] += later and not earlier
                    out['only_earlier'] += earlier and not later
                else:
                    out['in_neither_window'] += 1
                    raw = _f(r, 'v_nav')
                    out['in_neither_window_and_eq_raw'] += raw is not None and abs(w - raw) <= TOL
        per_run.append(out)
        for k, v in out.items():
            if isinstance(v, int):
                total[k] = total.get(k, 0) + v
    return {'runs': per_run, 'total': total}


def stall(runs):
    moving = frozen = exc = exc_frozen = 0
    ratio = []
    for run in runs:
        tag = os.path.basename(os.path.normpath(run))
        with open(os.path.join(run, f'{tag}.json')) as f:
            for leg in json.load(f)['legs']:
                a, b = leg.get('hz_cmd_vel'), leg.get('hz_diff_drive_controller/cmd_vel')
                if a and b:
                    ratio.append(b / a)
        for path in _traces(run):
            rows = list(csv.DictReader(open(path)))
            for prev, r in zip(rows, rows[1:]):
                va, m, w = _f(r, 'v_act'), _f(r, 'v_cmdvel'), _f(r, 'v_wheel')
                if None in (va, m, w) or abs(va) <= 0.01:
                    continue
                same = all(r.get(k) == prev.get(k) for k in ('x', 'y', 'yaw', 'v_act', 'w_act'))
                moving += 1
                frozen += same
                if abs(w) - abs(m) > TOL:
                    exc += 1
                    exc_frozen += same
    return {'moving_rows': moving,
            'frozen_frac_of_moving_rows': round(frozen / moving, 3) if moving else None,
            'exceeded_moving_rows': exc,
            'frozen_frac_of_exceeded_rows': round(exc_frozen / exc, 3) if exc else None,
            'median_wheel_hz_over_cmd_vel_hz': round(statistics.median(ratio), 3) if ratio else None,
            'legs_with_rates': len(ratio)}


def messages(rec_dir):
    def fl(x):
        return None if x in ('', None) else float(x)

    def eq(a, b):
        return abs(a[0] - b[0]) <= EXACT and abs(a[1] - b[1]) <= EXACT
    counts = {'wheel_messages': 0, 'eq_latest_monitor': 0, 'eq_latest_gated_only': 0,
              'eq_earlier_gated': 0, 'unattributed': 0, 'eq_raw_where_monitor_differs': 0}
    ages = []
    history = []
    with open(os.path.join(rec_dir, 'events.csv'), newline='') as f:
        for r in csv.DictReader(f):
            t = float(r['t'])
            wheel = (fl(r['v_wheel']), fl(r['w_wheel']))
            if fl(r['v_cmdvel']) is None or fl(r['v_gated']) is None:
                continue
            mon = (fl(r['v_cmdvel']), fl(r['w_cmdvel']))
            gated = (fl(r['v_gated']), fl(r['w_gated']))
            counts['wheel_messages'] += 1
            if eq(wheel, mon):
                counts['eq_latest_monitor'] += 1
            elif eq(wheel, gated):
                counts['eq_latest_gated_only'] += 1
            else:
                age = next((round(t - ts, 3) for ts, g in reversed(history) if eq(wheel, g)), None)
                if age is None:
                    counts['unattributed'] += 1
                else:
                    counts['eq_earlier_gated'] += 1
                    ages.append(age)
            if fl(r['v_nav']) is not None:
                raw = (fl(r['v_nav']), fl(r['w_nav']))
                if eq(wheel, raw) and not eq(wheel, mon) and not eq(wheel, gated):
                    counts['eq_raw_where_monitor_differs'] += 1
            history.append((t, gated))
            history = history[-200:]
    counts['earlier_gated_max_age_s'] = max(ages) if ages else None
    counts['earlier_gated_median_age_s'] = round(statistics.median(ages), 3) if ages else None
    return counts


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='command', required=True)
    for name in ('trace', 'stall'):
        p = sub.add_parser(name)
        p.add_argument('runs', nargs='+')
    m = sub.add_parser('messages')
    m.add_argument('rec_dir')
    args = ap.parse_args(argv)
    if args.command == 'trace':
        out = trace(args.runs)
    elif args.command == 'stall':
        out = stall(args.runs)
    else:
        out = messages(args.rec_dir)
    print(json.dumps(out, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
