#!/usr/bin/env python3
"""One mission run's result, from its own ROS logs -- the same lines the
archived four-colour matrix was read from (main:docs/data/
navigation_world_final/results.json).

usage: extract_mission_result.py RUN_DIR COLOUR  -> RUN_DIR/result.json

RUN_DIR holds ros_log/ (ROS_LOG_DIR of the run) and run/ (the runner's
OUT_DIR). Nothing is inferred: a field whose line never appeared is null.
"""
import glob
import json
import os
import re
import sys

STAMP = re.compile(r'^\[(\w+)\] \[(\d+\.\d+)\] \[([^\]]+)\]: (.*)$')


def lines(run_dir, node):
    out = []
    for path in glob.glob(os.path.join(run_dir, 'ros_log', '**', '*.log'),
                          recursive=True):
        for raw in open(path, errors='replace'):
            m = STAMP.match(raw.rstrip('\n'))
            if m and m.group(3) == node:
                out.append((float(m.group(2)), m.group(1), m.group(4)))
    return sorted(out)


def first(rows, pattern):
    rx = re.compile(pattern)
    for t, _, text in rows:
        m = rx.search(text)
        if m:
            return t, m
    return None, None


def main():
    run_dir, colour = sys.argv[1], sys.argv[2]
    ex = lines(run_dir, 'mission_executive')
    gr = lines(run_dir, 'grasp_server')
    t_start, _ = first(ex, r'starting the fetch for ')
    t_end, term = first(ex, r'^MISSION (\w+): result=(\S+) reason=(\S+)')
    _, home = first(ex, r'RETURN_HOME arrived: ground truth ([\d.]+) m')
    _, ramp = first(ex, r'NAVIGATE_TO_RAMP arrived: ground truth ([\d.]+) m')
    _, lift = first(gr, r'Target lifted ([\d.]+) mm')
    t_det, _ = first(gr, r'Magnet detached')
    recoveries = sum(1 for _, _, text in ex if '-> RELOCALIZE' in text)
    transitions = [text for _, _, text in ex if ' -> ' in text]
    final_state = ''
    fs = os.path.join(run_dir, 'run', 'final_state.txt')
    if os.path.exists(fs):
        final_state = open(fs).read().strip()
    elapsed = re.search(r'elapsed=([\d.]+)', final_state)
    runner_log = os.path.join(run_dir, 'run', 'runner.log')
    checks = {'pass': [], 'fail': []}
    if os.path.exists(runner_log):
        for raw in open(runner_log, errors='replace'):
            m = re.search(r'm6_run: (PASS|FAIL) (.*) \(\d', raw)
            if m:
                checks['pass' if m.group(1) == 'PASS' else 'fail'].append(
                    m.group(2).strip())
    wall = round(t_end - t_start, 1) if t_start and t_end else None
    # /mission/state's `elapsed` is sim time in the CURRENT state, not the
    # mission: kept under that name, never divided into a mission RTF.
    state_elapsed = float(elapsed.group(1)) if elapsed else None
    result = {
        'colour': colour,
        'outcome': f'{term.group(1)}/{term.group(2)}' if term else None,
        'reason': term.group(3) if term else None,
        'wall_duration_s': wall,
        'final_state_elapsed_sim_s': state_elapsed,
        'localization_recoveries': recoveries,
        'home_arrival_error_m': float(home.group(1)) if home else None,
        'pre_ramp_arrival_error_m': float(ramp.group(1)) if ramp else None,
        'lift_mm': float(lift.group(1)) if lift else None,
        'magnet_detached': t_det is not None,
        'runner_checks_passed': len(checks['pass']),
        'runner_checks_failed': checks['fail'],
        'final_state': final_state,
        'transitions': transitions,
    }
    json.dump(result, open(os.path.join(run_dir, 'result.json'), 'w'),
              indent=1)
    print(json.dumps({k: result[k] for k in (
        'colour', 'outcome', 'wall_duration_s', 'localization_recoveries',
        'home_arrival_error_m', 'runner_checks_failed')}))


if __name__ == '__main__':
    main()
