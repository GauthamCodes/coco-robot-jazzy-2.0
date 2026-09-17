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
import tempfile

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


def stop_breach(rows, moving=MOVING):
    """While PolygonStop is ACTIVE, were the wheels still commanded to move?

    `cm_action == '1'` is the collision monitor's STOP state (the same test
    c2nav39_tour_report.stop_holds uses). A STOP row with |v_wheel| above
    `moving` is a stop the wheels did not obey.

    This is what separates two readings of "topology B had no PolygonStop
    deadlock": a robot that genuinely never got trapped, and a robot that
    was told to stop and drove anyway because the monitor does not own the
    wheels. They look identical in a success count and mean opposite things.
    """
    stop_rows = driven = 0
    worst = 0.0
    for row in rows:
        if row.get('cm_action') != '1':
            continue
        wheel = _f(row, 'v_wheel')
        if wheel is None:
            continue
        stop_rows += 1
        if abs(wheel) > moving:
            driven += 1
            worst = max(worst, abs(wheel))
    return {
        'stop_rows': stop_rows,
        'stop_rows_wheels_driven': driven,
        'worst_wheel_during_stop_ms': round(worst, 4) if driven else None,
    }


def bypass_source(rows, tol=JITTER_TOL, driven=0.2):
    """When the monitor commanded ~0 and the wheels were driven hard, what
    were the wheels actually following?

    Candidates are the two upstream stages nav_bench traces: `v_nav`, the
    controller's RAW output, and `v_smoothed`, the velocity smoother's. If
    the wheels track `v_nav` and never `v_smoothed`, both the smoother's
    acceleration limits and the monitor's gating are bypassed -- the
    arbiter is forwarding the controller's command straight to the wheels,
    which is what the /cmd_vel_nav loop predicts.

    `cm_action` is nav2_collision_monitor's ActionType: 0 none, 1 STOP,
    2 SLOWDOWN, 3 APPROACH, 4 LIMIT.
    """
    n = eq_raw = eq_smoothed = 0
    actions = {}
    raw, smoothed = [], []
    for row in rows:
        monitor, wheel = _f(row, 'v_cmdvel'), _f(row, 'v_wheel')
        if monitor is None or wheel is None or abs(monitor) > tol or abs(wheel) <= driven:
            continue
        n += 1
        key = row.get('cm_action') or 'blank'
        actions[key] = actions.get(key, 0) + 1
        nav, smooth = _f(row, 'v_nav'), _f(row, 'v_smoothed')
        if nav is not None:
            raw.append(nav)
            eq_raw += abs(wheel - nav) <= tol
        if smooth is not None:
            smoothed.append(smooth)
            eq_smoothed += abs(wheel - smooth) <= tol
    return {
        'rows': n,
        'cm_action': dict(sorted(actions.items())),
        'wheel_matches_raw_controller': eq_raw,
        'wheel_matches_smoother': eq_smoothed,
        'median_raw_controller_ms': round(statistics.median(raw), 4) if raw else None,
        'median_smoother_ms': round(statistics.median(smoothed), 4) if smoothed else None,
    }


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


ENCLOSURE = ('enclosure_entry', 'enclosure_exit')


def summarise(details, tour_secs):
    """The arm-level numbers the topology decision is made on, from
    C2-NAV.40 leg_detail records. Terminal errors are over SUCCEEDED legs
    only: a leg that never arrived has a 'final' error that measures where
    it gave up, not how well it converged."""
    def count(names, status='SUCCEEDED'):
        legs = [d for d in details if d['scenario'] in names]
        return sum(1 for d in legs if d['status'] == status), len(legs)

    ordinary = tuple({d['scenario'] for d in details} - set(ENCLOSURE))
    done = [d for d in details if d['status'] == 'SUCCEEDED']
    errs = [d['final_goal_err_m'] for d in done if d['final_goal_err_m'] is not None]
    yaws = [abs(d['final_yaw_err_rad']) for d in done if d['final_yaw_err_rad'] is not None]
    clear = [d['true_min_clearance_m'] for d in details if d['true_min_clearance_m'] is not None]
    statuses = {}
    for d in details:
        statuses[d['status']] = statuses.get(d['status'], 0) + 1
    return {
        'legs': count([d['scenario'] for d in details]),
        'ordinary': count(ordinary),
        'enclosure_entry': count(('enclosure_entry',)),
        'enclosure_exit': count(('enclosure_exit',)),
        'statuses': dict(sorted(statuses.items())),
        'tour_sim_s': tour_secs,
        'median_tour_sim_s': round(statistics.median(tour_secs), 2) if tour_secs else None,
        'true_min_clearance_m': min(clear) if clear else None,
        'stop_activations': sum(d['stop_activations'] for d in details),
        'polygon_stop_s': round(sum(d['polygon_stop_s'] or 0.0 for d in details), 2),
        'deadlocks': [d['scenario'] for d in details if d['stop_deadlock']],
        'median_goal_err_succeeded_m': round(statistics.median(errs), 3) if errs else None,
        'median_abs_yaw_err_succeeded_rad': round(statistics.median(yaws), 3) if yaws else None,
    }


def arm(run_dirs, label, expect=None, allow_legacy=False):
    """Aggregate one arm, refusing a set whose runs did not all run it.

    `allow_legacy` accepts a run whose manifest has NO topology key, which is
    every tour before C2-NAV.41 added it. Those runs are topology A -- at
    those commits nav_tour_run.sh hard-coded `arbiter:=false` and started no
    arbiter -- but that is an assertion about the code that ran, not a
    readback from the graph it ran on. So it is opt-in, and the run is
    labelled `legacy` in the output rather than quietly counted as if it had
    been verified.
    """
    runs = []
    for run_dir in run_dirs:
        topology, verdict = run_topology(run_dir)
        legacy = topology is None and allow_legacy and expect is not None
        runs.append({'dir': run_dir,
                     'topology': f'{expect} (legacy: no manifest key)' if legacy else topology,
                     'live': verdict,
                     'legacy': legacy})
    claimed = {expect if r['legacy'] else r['topology'] for r in runs}
    if expect is not None and claimed != {expect}:
        raise SystemExit(
            f'arm {label}: expected every run to be topology {expect}, '
            f'manifests say {sorted(str(c) for c in claimed)} -- refusing to '
            f'aggregate a mixed arm. Runs predating C2-NAV.41 have no topology '
            f'key; pass --{label.lower()}-legacy to accept them as {expect}, '
            f'which labels them legacy in the output.')
    items, auth_rows, speed_all = [], [], []
    details, tour_secs = [], []
    for run_dir in run_dirs:
        prev, secs = None, 0.0
        for leg, rows in iter_legs(run_dir):
            items.append((leg, [(float(r['x']), float(r['y'])) for r in rows
                                if r.get('x') and r.get('y')]))
            auth_rows.extend(rows)
            speed_all.extend(speeds(rows))
            # C2-NAV.40's per-leg definitions (STOP holds, deadlocks,
            # carried holds), imported, with the same prev-leg chaining
            prev = REPORT.leg_detail(leg, rows, prev)
            details.append(prev)
            secs += leg.get('duration_sim_s') or 0.0
        tour_secs.append(round(secs, 2))
    table = REPORT.arm_table(items)
    return {
        'label': label,
        'runs': runs,
        'n_runs': len(run_dirs),
        'table': table,
        'succeeded': table['TOTAL']['succeeded'],
        'attempts': table['TOTAL']['attempts'],
        'summary': summarise(details, tour_secs),
        'authority': monitor_authority(auth_rows),
        'stop_breach': stop_breach(auth_rows),
        'bypass_source': bypass_source(auth_rows),
        'median_speed_ms': (round(statistics.median(speed_all), 4)
                            if speed_all else None),
        'moving_samples': len(speed_all),
    }


def render(arms):
    lines = ['### Arm summary', '',
             '| arm | legs | ordinary | entry | exit | statuses | tour sim s (median) | '
             'min true clear m | STOP n / s | deadlocks | goal err m (succ, med) | '
             '\\|yaw err\\| rad (succ, med) |',
             '|---|---|---|---|---|---|---|---|---|---|---|---|']
    for a in arms:
        s = a['summary']
        frac = lambda pair: f'{pair[0]}/{pair[1]}'  # noqa: E731
        lines.append(
            f"| {a['label']} | {frac(s['legs'])} | {frac(s['ordinary'])} | "
            f"{frac(s['enclosure_entry'])} | {frac(s['enclosure_exit'])} | {s['statuses']} | "
            f"{s['tour_sim_s']} ({s['median_tour_sim_s']}) | {s['true_min_clearance_m']} | "
            f"{s['stop_activations']} / {s['polygon_stop_s']} | {s['deadlocks'] or '--'} | "
            f"{s['median_goal_err_succeeded_m']} | {s['median_abs_yaw_err_succeeded_rad']} |")
    lines += ['', REPORT.render({a['label']: a['table'] for a in arms}), '']
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
    lines.append('### While PolygonStop was active, did the wheels stop?')
    lines.append('')
    lines.append('| arm | STOP rows | STOP rows with wheels driven | worst wheel during STOP m/s |')
    lines.append('|---|---|---|---|')
    for a in arms:
        s = a['stop_breach']
        lines.append(f"| {a['label']} | {s['stop_rows']} | {s['stop_rows_wheels_driven']} | "
                     f"{s['worst_wheel_during_stop_ms'] if s['worst_wheel_during_stop_ms'] is not None else '--'} |")
    lines.append('')
    lines.append('### Monitor ~0 but wheels > 0.2 m/s: what were the wheels following?')
    lines.append('')
    lines.append('| arm | rows | wheel == raw controller | wheel == smoother | '
                 'median raw controller m/s | median smoother m/s | cm_action counts |')
    lines.append('|---|---|---|---|---|---|---|')
    for a in arms:
        b = a['bypass_source']
        lines.append(
            f"| {a['label']} | {b['rows']} | {b['wheel_matches_raw_controller']} | "
            f"{b['wheel_matches_smoother']} | "
            f"{b['median_raw_controller_ms'] if b['median_raw_controller_ms'] is not None else '--'} | "
            f"{b['median_smoother_ms'] if b['median_smoother_ms'] is not None else '--'} | "
            f"{b['cm_action'] or '--'} |")
    lines.append('')
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
    ap.add_argument('--a-legacy', action='store_true',
                    help='accept topology-A runs that predate the manifest key '
                         '(labelled legacy in the output)')
    ap.add_argument('--b-legacy', action='store_true',
                    help='accept topology-B runs that predate the manifest key')
    ap.add_argument('--json', help='write the arms as JSON here')
    args = ap.parse_args(argv)
    arms = []
    if args.a:
        arms.append(arm(args.a, 'A', expect='A', allow_legacy=args.a_legacy))
    if args.b:
        arms.append(arm(args.b, 'B', expect='B', allow_legacy=args.b_legacy))
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

    # 11. a run predating the manifest topology key is not silently counted
    with tempfile.TemporaryDirectory() as tmp:
        run = os.path.join(tmp, 'legacy_r01')
        os.makedirs(run)
        with open(os.path.join(run, 'manifest.json'), 'w') as f:
            json.dump({'run_id': 'legacy_r01'}, f)
        with open(os.path.join(run, 'legacy_r01.json'), 'w') as f:
            json.dump({'legs': [{'scenario': 'open_space', 'rep': 0,
                                 'status': 'SUCCEEDED'}]}, f)
        check('11 an absent manifest key reads as None, not as A',
              run_topology(run) == (None, 'absent'), str(run_topology(run)))
        try:
            arm([run], 'A', expect='A')
            refused = False
        except SystemExit:
            refused = True
        check('12 a legacy run is refused without the explicit opt-in', refused)
        got = arm([run], 'A', expect='A', allow_legacy=True)
        check('13 the opt-in labels it legacy instead of hiding it',
              got['runs'][0]['legacy'] and 'legacy' in got['runs'][0]['topology'],
              str(got['runs'][0]))
        # and a REAL topology value is never overwritten by the opt-in
        with open(os.path.join(run, 'manifest.json'), 'w') as f:
            json.dump({'run_id': 'legacy_r01', 'topology': 'B'}, f)
        try:
            arm([run], 'A', expect='A', allow_legacy=True)
            refused = False
        except SystemExit:
            refused = True
        check('14 the opt-in does not override a manifest that says otherwise',
              refused)

    # 15. a STOP the wheels obeyed is clean; a STOP they drove through is not
    obeyed = stop_breach([{'cm_action': '1', 'v_wheel': '0.0'}] * 5
                         + [{'cm_action': '0', 'v_wheel': '0.3'}] * 5)
    breached = stop_breach([{'cm_action': '1', 'v_wheel': '0.3'}] * 3
                           + [{'cm_action': '1', 'v_wheel': '0.0'}] * 2)
    check('15 a STOP the wheels obeyed is clean, and motion outside STOP is ignored',
          obeyed == {'stop_rows': 5, 'stop_rows_wheels_driven': 0,
                     'worst_wheel_during_stop_ms': None}, str(obeyed))
    check('16 a STOP the wheels drove through is counted',
          breached['stop_rows'] == 5 and breached['stop_rows_wheels_driven'] == 3
          and breached['worst_wheel_during_stop_ms'] == 0.3, str(breached))

    # 17. wheels following the raw controller past a gated monitor
    looped = bypass_source([{'v_cmdvel': '0.0', 'v_wheel': '0.25', 'v_nav': '0.25',
                             'v_smoothed': '0.0', 'cm_action': '2'}] * 4)
    check('17 the loop signature: wheels track the raw controller, not the smoother',
          looped['rows'] == 4 and looped['wheel_matches_raw_controller'] == 4
          and looped['wheel_matches_smoother'] == 0 and looped['cm_action'] == {'2': 4},
          str(looped))

    # 18. a correctly wired chain has no such rows at all
    wired = bypass_source([{'v_cmdvel': '0.0', 'v_wheel': '0.0', 'v_nav': '0.25',
                            'v_smoothed': '0.0', 'cm_action': '1'}] * 4)
    check('18 a gated wheel command produces no bypass rows', wired['rows'] == 0, str(wired))

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
