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

"""C2-NAV.41 -- the accepted configuration driven through the arbiter.

THE QUESTION
------------
Every C2-NAV.0 ... C2-NAV.40 tour ran topology A: `nav.launch.py` alone,
`cmd_vel_relay` publishing the controller topic. The robot SHIPS in topology
B, where `mission.launch.py` points the relay at `/cmd_vel_nav` and
`cmd_vel_arbiter` becomes the sole publisher of the wheels. So the accepted
configuration -- local CSF 65, BaseObstacle 8.0, the multi-pose BT -- has
never been toured in the configuration it ships in. C2-NAV.40 checked
`mission.launch.py` BRING-UP only: no mission was started and no goal sent.

This module compares the two arms. It does NOT pick a topology: topology B
is not an alternative to A, it is the shipping path, and A is the harness
that every historical number was measured on.

THE SAFETY STATISTIC
--------------------
`/cmd_vel_nav` is already `controller_server`'s output into the velocity
smoother. Pointing the relay's output at the same topic closes a loop: the
collision monitor's GATED command is fed back into the smoother's input, and
the arbiter sees raw and gated commands interleaved on one topic. That is
PROJECT_STATE.md KNOWN LIMITATIONS 0, a characterised and unfixed safety
defect, and it is a property of topology B only.

`monitor_authority` counts it directly from the trace columns nav_bench.py
has written since C2-NAV.0:

    v_cmdvel   the collision monitor's output    (/cmd_vel)
    v_wheel    what the controller received      (/diff_drive_controller/cmd_vel)

A sample where `|v_wheel| > |v_cmdvel| + tol` is one the monitor did not
control.

WHY A TOLERANCE, AND WHY TOPOLOGY A IS THE CONTROL
--------------------------------------------------
The trace is a 10 Hz resample with a zero-order hold, so the two columns are
not synchronised samples of one instant and a one-row misalignment can
manufacture a gap. That is exactly why both arms are reported under the SAME
definition and why topology A is the control: C2-NAV.0 measured topology A at
4 of 6 956 samples (0.06 %), worst gap 0.016 m/s -- i.e. the artifact floor.
`JITTER_TOL` is set at 0.02 m/s, just above that measured floor. The count at
tol = 0 is reported alongside it so nothing is hidden by the choice.

EVIDENCE CLASS
--------------
OBSERVED   every trace column and leg record, for every leg of every run.
DERIVED    the authority counts, the speed medians, the per-arm totals.
ASSERTED   each run's topology, read from the manifest nav_tour_run.sh wrote
           and cross-checked against topology_live.txt, which was read off
           the live ROS graph. An arm labelled B whose runs did not actually
           run B is not evidence, so `arm` refuses to aggregate a mixed set.

WHAT THIS CANNOT SHOW
---------------------
Three tours per arm, one route set, legs that are not independent (a tour
carries error forward across leg boundaries). Counts and medians are printed;
no significance is computed. The speed statistic here is this module's own
definition and is NOT the 0.155 / 0.208 m/s pair in DESIGN_DECISIONS.md --
that definition is not recorded, so the two must not be compared. The A-vs-B
delta computed here is internally valid because both arms use this one.

  python3 -P docs/data/c2nav41_topology.py selftest
  python3 -P docs/data/c2nav41_topology.py compare \
      --a ~/coco_nav_runs/baseline/baseline_r0* \
      --b ~/coco_nav_runs/baseline_topology_b/baseline_topology_b_r0*
"""

import argparse
import glob
import importlib.util
import json
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def _load_report():
    """C2-NAV.39's report module, imported rather than restated -- a
    comparison whose two halves were computed by two copies of a definition
    is not a comparison (C2-NAV.26's rule)."""
    spec = importlib.util.spec_from_file_location(
        'c2nav39_tour_report', os.path.join(HERE, 'c2nav39_tour_report.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


REPORT = _load_report()

# Just above C2-NAV.0's measured topology-A worst gap of 0.016 m/s, which is
# resampling jitter rather than a real command difference.
JITTER_TOL = 0.02
# A wheel command this size or smaller is not motion worth calling motion.
MOVING = 0.01


def _f(row, key):
    value = row.get(key)
    if value is None or value == '':
        return None
    try:
        return float(value)
    except ValueError:
        return None


def monitor_authority(rows, tol=JITTER_TOL, moving=MOVING):
    """Does the collision monitor's command reach the wheels?

    Returns counts over every trace row carrying BOTH columns. `exceeded` is
    the headline: samples where the wheels got more than the monitor allowed,
    beyond the jitter tolerance. `gated_zero_moving` is the severe form --
    the monitor commanding a stop while the wheels were driven anyway.
    """
    samples = exceeded = raw = gated_zero = 0
    worst_gap = 0.0
    worst_pair = None
    worst_zero_wheel = 0.0
    for row in rows:
        monitor, wheel = _f(row, 'v_cmdvel'), _f(row, 'v_wheel')
        if monitor is None or wheel is None:
            continue
        samples += 1
        gap = abs(wheel) - abs(monitor)
        if gap > 0:
            raw += 1
        if gap > tol:
            exceeded += 1
            if gap > worst_gap:
                worst_gap, worst_pair = gap, (monitor, wheel)
        if abs(monitor) <= tol and abs(wheel) > moving:
            gated_zero += 1
            worst_zero_wheel = max(worst_zero_wheel, abs(wheel))
    return {
        'samples': samples,
        'exceeded': exceeded,
        'exceeded_frac': (exceeded / samples) if samples else None,
        'exceeded_raw_tol0': raw,
        'worst_gap_ms': round(worst_gap, 4) if worst_pair else None,
        'worst_gap_monitor_ms': round(worst_pair[0], 4) if worst_pair else None,
        'worst_gap_wheel_ms': round(worst_pair[1], 4) if worst_pair else None,
        'gated_zero_moving': gated_zero,
        'worst_wheel_while_gated_zero_ms': (round(worst_zero_wheel, 4)
                                            if gated_zero else None),
    }


def speeds(rows, moving=MOVING):
    """Measured forward speeds while the robot was actually moving.

    This module's OWN definition (see the header): |v_act| over rows where
    |v_act| > `moving`. Comparable between the arms here and nowhere else.
    """
    return [abs(v) for v in (_f(row, 'v_act') for row in rows)
            if v is not None and abs(v) > moving]


def iter_legs(run_dir):
    """[(leg record, trace rows), ...] for one nav_tour_run.sh run directory.

    Resolves the run JSON and trace layout exactly as
    c2nav39_tour_report.load_run_dir does, but keeps every trace COLUMN
    rather than only (x, y).
    """
    run_dir = os.path.expanduser(run_dir)
    tag = os.path.basename(os.path.normpath(run_dir))
    path = os.path.join(run_dir, f'{tag}.json')
    if not os.path.exists(path):
        found = [p for p in glob.glob(os.path.join(run_dir, '*.json'))
                 if 'legs' in json.load(open(p))]
        if len(found) != 1:
            raise SystemExit(f'{run_dir}: expected one nav_bench JSON, found {found}')
        path = found[0]
        tag = os.path.splitext(os.path.basename(path))[0]
    with open(path) as f:
        doc = json.load(f)
    out = []
    for leg in doc['legs']:
        trace = os.path.join(run_dir, f'{tag}_traces',
                             f"{leg['scenario']}_rep{leg.get('rep', 0)}.csv")
        out.append((leg, REPORT.load_trace_rows(trace) if os.path.exists(trace) else []))
    return out


def run_topology(run_dir):
    """(manifest topology, live-readback verdict) for one run directory.

    The manifest records what the experiment ASKED for; topology_live.txt
    records what `verify-topology` read off the running graph. Both, because
    they are different claims.
    """
    run_dir = os.path.expanduser(run_dir)
    manifest = os.path.join(run_dir, 'manifest.json')
    topology = None
    if os.path.exists(manifest):
        with open(manifest) as f:
            topology = json.load(f).get('topology')
    live = os.path.join(run_dir, 'topology_live.txt')
    verdict = 'absent'
    if os.path.exists(live):
        with open(live) as f:
            text = f.read()
        verdict = 'MISMATCH' if 'MISMATCH' in text else 'OK'
    return topology, verdict


def arm(run_dirs, label, expect=None):
    """Aggregate one arm, refusing a set whose runs did not all run it."""
    runs = []
    for run_dir in run_dirs:
        topology, verdict = run_topology(run_dir)
        runs.append({'dir': run_dir, 'topology': topology, 'live': verdict})
    claimed = {r['topology'] for r in runs}
    if expect is not None and claimed != {expect}:
        raise SystemExit(
            f'arm {label}: expected every run to be topology {expect}, '
            f'manifests say {sorted(str(c) for c in claimed)} -- refusing to '
            f'aggregate a mixed arm')
    items, auth_rows, speed_all = [], [], []
    for run_dir in run_dirs:
        for leg, rows in iter_legs(run_dir):
            items.append((leg, [(float(r['x']), float(r['y'])) for r in rows
                                if r.get('x') and r.get('y')]))
            auth_rows.extend(rows)
            speed_all.extend(speeds(rows))
    table = REPORT.arm_table(items)
    return {
        'label': label,
        'runs': runs,
        'n_runs': len(run_dirs),
        'table': table,
        'succeeded': table['TOTAL']['succeeded'],
        'attempts': table['TOTAL']['attempts'],
        'authority': monitor_authority(auth_rows),
        'median_speed_ms': (round(statistics.median(speed_all), 4)
                            if speed_all else None),
        'moving_samples': len(speed_all),
    }


def render(arms):
    lines = [REPORT.render({a['label']: a['table'] for a in arms}), '']
    lines.append('### Command-path safety: does the collision monitor reach the wheels?')
    lines.append('')
    lines.append('| arm | runs | legs | samples | wheels exceeded monitor | frac | '
                 'worst gap m/s | monitor 0 while driving | worst wheel then |')
    lines.append('|---|---|---|---|---|---|---|---|---|')
    for a in arms:
        x = a['authority']
        frac = '--' if x['exceeded_frac'] is None else f"{100 * x['exceeded_frac']:.2f} %"
        lines.append(
            f"| {a['label']} | {a['n_runs']} | {a['succeeded']}/{a['attempts']} | "
            f"{x['samples']} | {x['exceeded']} | {frac} | "
            f"{x['worst_gap_ms'] if x['worst_gap_ms'] is not None else '--'} | "
            f"{x['gated_zero_moving']} | "
            f"{x['worst_wheel_while_gated_zero_ms'] if x['gated_zero_moving'] else '--'} |")
    lines += ['', f'Tolerance {JITTER_TOL} m/s. Counts at tol = 0: '
              + ', '.join(f"{a['label']} {a['authority']['exceeded_raw_tol0']}"
                          for a in arms) + '.', '']
    lines.append('### Speed while moving (this module\'s definition; see the header)')
    lines.append('')
    lines.append('| arm | median m/s | moving samples |')
    lines.append('|---|---|---|')
    for a in arms:
        lines.append(f"| {a['label']} | "
                     f"{a['median_speed_ms'] if a['median_speed_ms'] is not None else '--'} | "
                     f"{a['moving_samples']} |")
    lines += ['', '### Topology, as asked for and as read off the live graph', '']
    lines.append('| arm | run | manifest | live readback |')
    lines.append('|---|---|---|---|')
    for a in arms:
        for r in a['runs']:
            lines.append(f"| {a['label']} | `{os.path.basename(r['dir'])}` | "
                         f"{r['topology']} | {r['live']} |")
    return '\n'.join(lines)


def mode_compare(argv):
    ap = argparse.ArgumentParser(prog='c2nav41_topology.py compare')
    ap.add_argument('--a', nargs='*', default=[], help='topology-A run directories')
    ap.add_argument('--b', nargs='*', default=[], help='topology-B run directories')
    ap.add_argument('--json', help='write the arms as JSON here')
    args = ap.parse_args(argv)
    arms = []
    if args.a:
        arms.append(arm(args.a, 'A', expect='A'))
    if args.b:
        arms.append(arm(args.b, 'B', expect='B'))
    if not arms:
        raise SystemExit('nothing to compare: pass --a and/or --b')
    print(render(arms))
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(arms, f, indent=1)
        print(f'\nwrote {args.json}')
    return 0


def mode_selftest():
    checks, failed = [], 0

    def check(name, ok, detail=''):
        nonlocal failed
        failed += 0 if ok else 1
        checks.append(f'{"ok  " if ok else "FAIL"} {name}{": " + detail if detail else ""}')

    def rows(pairs):
        return [{'v_cmdvel': str(m), 'v_wheel': str(w), 'v_act': str(w)}
                for m, w in pairs]

    # 1. a wheel command equal to the monitor's is not an exceedance
    a = monitor_authority(rows([(0.3, 0.3)] * 10))
    check('1 equal commands do not count', a['exceeded'] == 0 and a['samples'] == 10,
          str(a['exceeded']))

    # 2. jitter below the tolerance does not count, above it does
    a = monitor_authority(rows([(0.300, 0.316)]))
    b = monitor_authority(rows([(0.300, 0.330)]))
    check('2 the tolerance separates jitter from a real gap',
          a['exceeded'] == 0 and b['exceeded'] == 1, f"{a['exceeded']} {b['exceeded']}")

    # 3. tol=0 still sees the jitter, so the tolerance hides nothing
    check('3 tol=0 count reports what the tolerance excluded',
          a['exceeded_raw_tol0'] == 1, str(a['exceeded_raw_tol0']))

    # 4. the severe case: monitor gating to zero while the wheels drive
    a = monitor_authority(rows([(0.0, 0.3)] * 4))
    check('4 monitor at zero while the wheels drive',
          a['gated_zero_moving'] == 4 and a['worst_wheel_while_gated_zero_ms'] == 0.3,
          str(a))

    # 5. the monitor gating a STOPPED robot is not a defect
    a = monitor_authority(rows([(0.0, 0.0)] * 4))
    check('5 a gated, stopped robot is clean',
          a['gated_zero_moving'] == 0 and a['exceeded'] == 0, str(a))

    # 6. reverse motion compares magnitudes, not signs
    a = monitor_authority(rows([(-0.10, -0.30)]))
    check('6 magnitudes, not signs', a['exceeded'] == 1, str(a['exceeded']))

    # 7. blank cells are skipped rather than read as zero
    a = monitor_authority([{'v_cmdvel': '', 'v_wheel': '0.3'},
                           {'v_cmdvel': '0.3', 'v_wheel': ''},
                           {'v_cmdvel': '0.0', 'v_wheel': '0.3'}])
    check('7 blanks are skipped, not zero', a['samples'] == 1, str(a['samples']))

    # 8. null control: an arm compared with itself has zero delta
    same = rows([(0.3, 0.3), (0.2, 0.2)])
    check('8 null control', monitor_authority(same) == monitor_authority(same))

    # 9. the speed statistic ignores a stopped robot
    check('9 speeds exclude a stopped robot',
          speeds([{'v_act': '0.0'}, {'v_act': '0.2'}, {'v_act': '0.4'}]) == [0.2, 0.4])

    # 10. the imported report module is C2-NAV.39's, not a copy
    check('10 metric definitions are imported, not restated',
          hasattr(REPORT, 'arm_table') and hasattr(REPORT, 'load_trace_rows'))

    print('\n'.join(checks))
    print(f'\n{len(checks) - failed}/{len(checks)} checks passed')
    return 1 if failed else 0


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    modes = {'selftest': lambda _: mode_selftest(), 'compare': mode_compare}
    if not argv or argv[0] not in modes:
        raise SystemExit(f'usage: {sys.argv[0]} {{selftest|compare}} [args...]')
    return modes[argv[0]](argv[1:])


if __name__ == '__main__':
    sys.exit(main())
