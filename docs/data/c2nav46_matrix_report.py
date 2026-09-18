#!/usr/bin/env python3
"""C2-NAV.46: the M6 fetch colour matrix, assembled from the per-run reports.

A composer, not a third measurement. Every number it prints comes from
`c2nav44_m6_report.py` (mission, command path, Nav2, manipulation) or
`c2nav45_gate_report.py` (what the arrival gate decided), both left
untouched so C2-NAV.44's, C2-NAV.45's and this sprint's runs stay directly
comparable. The only things read directly here are the two lines the
per-run reports do not carry -- the grasp `base-x` and the approach
distance, which live in the mission log.

No ROS. Reads only files the runner wrote.

    python3 -P c2nav46_matrix_report.py RUN_DIR [RUN_DIR ...] [--json OUT]

A run is VALID when the runner completed every check and reached a mission
terminal state. A run whose infrastructure failed -- a bring-up check that
did not pass, a launched process that died, a recorder that never saw the
terminal state -- is VOID: it measures the harness, not the mission, and it
is reported as VOID rather than as a mission failure.
"""
import json
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ANSI = re.compile(r'\x1b\[[0-9;]*m')
COLOURS = ('red', 'green', 'blue', 'yellow')
# mission_states.GOAL_XY_TOLERANCE / GOAL_XY_CONSISTENCY, for the header only.
XY_TOLERANCE = 0.25
XY_CONSISTENCY = 0.50
# The 16 nominal states of a clean fetch, in order.
NOMINAL = ('IDLE', 'LOCALIZE', 'NAVIGATE_TO_RAMP', 'ALIGN_FOR_CLIMB', 'CLIMB',
           'VERIFY_CLIMB', 'SEARCH_TARGET', 'STOW_ARM', 'APPROACH_TARGET',
           'GRASP', 'VERIFY_GRASP', 'DESCEND', 'RETURN_HOME', 'PLACE',
           'VERIFY_PLACEMENT', 'COMPLETE')


def sub_report(script, run_dirs):
    """Run one of the existing per-run reports and return its JSON."""
    out = os.path.join(os.environ.get('TMPDIR', '/tmp'),
                       f'c2nav46_{os.path.basename(script)}.json')
    subprocess.run([sys.executable, '-P', os.path.join(HERE, script),
                    *run_dirs, '--json', out],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                   check=False)
    try:
        with open(out) as f:
            return {r['run']: r for r in json.load(f)}
    except (OSError, ValueError, KeyError):
        return {}


def read(path):
    try:
        with open(path, errors='replace') as f:
            return ANSI.sub('', f.read())
    except OSError:
        return ''


def last_float(text, pattern):
    hits = re.findall(pattern, text)
    try:
        return float(hits[-1])
    except (IndexError, TypeError, ValueError):
        return None


def num(value, fmt='{:.3f}', dash='--'):
    return dash if value is None else fmt.format(value)


def one_run(run_dir, rep, gate):
    """Fold both per-run reports, plus the two mission-log lines, into a row."""
    name = os.path.basename(run_dir.rstrip('/'))
    r = rep.get(name, {})
    g = gate.get(name, {})
    meta = r.get('meta', {})
    final = r.get('final', {})
    cmdpath = r.get('cmdpath', {})
    nav_active = cmdpath.get('nav_active', {})
    mission_log = read(os.path.join(run_dir, 'mission.log'))
    runner_log = read(os.path.join(run_dir, 'runner.log'))

    checks = r.get('runner_checks', {})
    state = final.get('state')
    # VOID is an infrastructure verdict and is kept strictly separate from a
    # mission failure: a run only counts against a colour if the harness did
    # its job. `ros_clean_after` is the shutdown readback -- "would kill:"
    # with nothing after it means the graph came down clean.
    #
    # Process death is deliberately NOT a criterion here. The runner already
    # checks it at the two moments where it means something -- after bring-up
    # and after the mission -- and those verdicts are in `runner_checks`.
    # `process_died` additionally catches the teardown, where every launched
    # process dies by design: all three known-good C2-NAV.45 green runs carry
    # three such lines. Counting them would void a passing run.
    void_reasons = []
    if checks.get('fail'):
        void_reasons.append(f"{len(checks['fail'])} runner check(s) failed: "
                            + '; '.join(checks['fail']))
    if state not in ('COMPLETE', 'ABORT'):
        void_reasons.append(f'no mission terminal state (state={state})')
    if not r.get('runner_torn_down'):
        void_reasons.append('runner did not finish its teardown')
    incomplete = bool(re.search(r'INCOMPLETE', runner_log))
    if incomplete:
        void_reasons.append('runner logged INCOMPLETE')

    # The state sequence is taken from the executive's own logged transitions,
    # not from the 10 Hz `state_path_sim` sampler: LOCALIZE lasts ~0.1 s and
    # the sampler drops it about half the time, which would score an identical
    # clean fetch as non-nominal purely on sampling luck.
    trans = r.get('transitions') or []
    if trans:
        seq = [trans[0]['from']] + [t['to'] for t in trans]
    else:
        seq = [s for s, _, _ in r.get('state_path_sim', []) if s != '--']
    lifted = last_float('\n'.join(r.get('manipulation', {}).get('lifted', [])),
                        r'lifted\s+([0-9.]+)\s*mm')
    nav2 = r.get('nav2', {})
    shutdown = ' '.join(r.get('ros_clean_after', [])).strip()

    return {
        'run': name,
        'colour': meta.get('colour') or g.get('colour'),
        'head': meta.get('head'),
        'dirty_paths': meta.get('dirty_paths'),
        'valid': not void_reasons,
        'void_reasons': void_reasons,
        'result': state,
        'fetch_success': state == 'COMPLETE' and final.get('result') == 'fetch',
        'reason': final.get('reason'),
        'attempt': final.get('attempt'),
        # A state entered more than once is a retry; the nominal fetch enters
        # each of the 16 exactly once.
        'state_reentries': {s: n for s, n in r.get('state_entries', {}).items()
                            if n > 1},
        'state_sequence': seq,
        'sequence_nominal': tuple(seq) == NOMINAL,
        'recovery_count': r.get('recovery_entries'),
        'relocalization_count': len(r.get('localization_recovery_lines') or []),
        'mission_sim_s': r.get('mission_sim_s_first_active_to_terminal'),
        'wall_s': last_float('\n'.join(r.get('wall_start_to_terminal', [])),
                             r'terminal detection:\s*([0-9.]+)'),
        'lift_mm': lifted,
        'base_x_m': last_float(mission_log, r'base-x\s+([0-9.]+)'),
        'approach_m': last_float(mission_log, r'arrived after\s+([0-9.]+)\s*m'),
        'pre_ramp_goal': g.get('pre_ramp_goal'),
        'pre_ramp_gt_error_m': (g.get('pre_ramp') or {}).get('error_m'),
        'pre_ramp_gate_verdict': (g.get('pre_ramp') or {}).get('verdict'),
        'pre_ramp_warned': (g.get('pre_ramp') or {}).get('warned'),
        'home_gt_error_m': (g.get('home') or {}).get('error_m'),
        'home_gate_verdict': (g.get('home') or {}).get('verdict'),
        'nav2_goal_succeeded': nav2.get('bt_goal_succeeded'),
        'nav2_goal_failed': nav2.get('bt_goal_failed'),
        'nav2_goal_aborted': nav2.get('bt_goal_aborted_lines'),
        'warn_count': len(re.findall(r'\[WARN\]', mission_log)),
        # The three command-path safety numbers, over nav-active rows, which
        # is the window the C2-NAV.42 fix owns.
        'bypass_raw_controller': nav_active.get('bypass_wheel_eq_raw_controller'),
        'wheel_above_monitor': nav_active.get('monitor_exceeded'),
        'stale_cmd_drops': sum((r.get('stale_cmd_drops') or {}).values()),
        'polygon_stop_episodes': (r.get('polygon_stop') or {}).get('stop_episodes'),
        'runner_checks_pass': checks.get('pass'),
        'runner_checks_fail': checks.get('fail'),
        'shutdown_clean': shutdown == 'would kill:',
        'shutdown': shutdown,
        'final_gt_world': r.get('final_gt_world'),
        'incomplete_marker': incomplete,
        'process_died_lines': len(r.get('process_died') or []),
    }


def matrix(rows):
    """The per-colour matrix, runs in the order they were collected."""
    by_colour = {c: [r for r in rows if r['colour'] == c] for c in COLOURS}
    width = max([1] + [len(v) for v in by_colour.values()])
    head = '             ' + ''.join(f'Run {i + 1:<4}' for i in range(width))
    out = [head + '  Result']
    for colour in COLOURS:
        runs = by_colour[colour]
        cells = []
        for r in runs:
            if not r['valid']:
                cells.append('VOID')
            else:
                cells.append('FETCH' if r['fetch_success'] else (r['result'] or '?'))
        cells += ['--'] * (width - len(cells))
        valid = [r for r in runs if r['valid']]
        ok = sum(1 for r in valid if r['fetch_success'])
        out.append(f'{colour:<13}' + ''.join(f'{cell:<8}' for cell in cells)
                   + f'  {ok}/{len(valid)}')
    return '\n'.join(out), by_colour


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    json_out = None
    if '--json' in sys.argv:
        json_out = sys.argv[sys.argv.index('--json') + 1]
        args = [a for a in args if a != json_out]
    if not args:
        print(__doc__)
        return 2

    rep = sub_report('c2nav44_m6_report.py', args)
    gate = sub_report('c2nav45_gate_report.py', args)
    rows = [one_run(d, rep, gate) for d in args]

    print('C2-NAV.46 -- M6 fetch colour matrix')
    print(f'gate bands: clean <= {XY_TOLERANCE} m, '
          f'accepted <= {XY_CONSISTENCY} m, failure beyond\n')
    table, by_colour = matrix(rows)
    print(table)

    print('\ncolour       fetch success   (valid runs, void runs)')
    for c in COLOURS:
        runs = by_colour[c]
        valid = [r for r in runs if r['valid']]
        void = [r for r in runs if not r['valid']]
        ok = sum(1 for r in valid if r['fetch_success'])
        print(f'{c:<13}{ok}/{len(valid):<14}({len(valid)} valid, {len(void)} void)')

    valid = [r for r in rows if r['valid']]
    void = [r for r in rows if not r['valid']]

    def total(key):
        return sum(r[key] or 0 for r in valid)

    print(f'\ntotal fetches completed      {sum(1 for r in valid if r["fetch_success"])}')
    print(f'total valid runs             {len(valid)}')
    print(f'total void runs              {len(void)}')
    print(f'recovery count               {total("recovery_count")}')
    print(f'relocalization count         {total("relocalization_count")}')
    print(f'command-path bypass count    {total("bypass_raw_controller")}')
    print(f'wheels above the monitor     {total("wheel_above_monitor")}')
    print(f'stale command drops          {total("stale_cmd_drops")}')
    print(f'PolygonStop activations      {total("polygon_stop_episodes")}')

    print('\nper run')
    for r in rows:
        tag = 'VOID: ' + '; '.join(r['void_reasons']) if not r['valid'] else r['result']
        print(f"\n  {r['run']} ({r['colour']}) -- {tag}")
        if not r['valid']:
            continue
        print(f"    goal              {r['pre_ramp_goal']}")
        print(f"    pre-ramp GT error {num(r['pre_ramp_gt_error_m'])} m"
              f"   gate: {r['pre_ramp_gate_verdict']}")
        print(f"    home GT error     {num(r['home_gt_error_m'])} m"
              f"   gate: {r['home_gate_verdict']}")
        print(f"    Nav2 goals        succeeded {r['nav2_goal_succeeded']}, "
              f"failed {r['nav2_goal_failed']}, aborted {r['nav2_goal_aborted']}")
        print(f"    sequence          {'16 nominal states' if r['sequence_nominal'] else ' -> '.join(r['state_sequence'])}")
        print(f"    re-entries        {r['state_reentries'] or 'none'}")
        print(f"    duration          {num(r['mission_sim_s'], '{:.1f}')} s sim, "
              f"{num(r['wall_s'], '{:.0f}')} s wall")
        print(f"    lift / base-x     {num(r['lift_mm'], '{:.1f}')} mm / "
              f"{num(r['base_x_m'], '{:.4f}')} m")
        print(f"    recovery / reloc  {r['recovery_count']} / {r['relocalization_count']}")
        print(f"    bypass / above-cm {r['bypass_raw_controller']} / {r['wheel_above_monitor']}")
        print(f"    stale / polygon   {r['stale_cmd_drops']} / {r['polygon_stop_episodes']}")
        print(f"    warns             {r['warn_count']}")
        print(f"    checks / shutdown {r['runner_checks_pass']} passed / "
              f"{'clean' if r['shutdown_clean'] else r['shutdown']}")

    if json_out:
        with open(json_out, 'w') as f:
            json.dump(rows, f, indent=1)
        print(f'\nwrote {json_out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
