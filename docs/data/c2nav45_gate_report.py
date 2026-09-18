#!/usr/bin/env python3
"""C2-NAV.45: what the pre-ramp arrival gate decided, per run.

An addendum to c2nav44_m6_report.py, not a replacement: that script is left
untouched so the two sprints' per-run numbers stay directly comparable. This
one answers only the questions C2-NAV.45 added, and it answers them from the
same files the runner already wrote. No ROS.

Per run it reports, for every nav leg:

  * what Nav2 said            (`Reached the goal!`, `Goal succeeded`)
  * what ground truth said    (the executive's own arrival line)
  * what the gate decided     (clean / discrepancy accepted / region failure)
  * whether a retry followed  (a second NAVIGATE_TO_RAMP goal, and the wheel
                               speed after the first stop -- the measurement
                               that showed C2-NAV.44's retries were futile)

The futility check is the important one and it is taken from the trace, not
from the logs: C2-NAV.44 established that the retries re-sent a goal a
stopped controller had already satisfied, so "was a retry issued" and "did
the robot move afterwards" are different questions and both are printed.

    python3 -P c2nav45_gate_report.py RUN_DIR [RUN_DIR ...] [--json OUT]
"""
import csv
import json
import math
import os
import re
import sys

ANSI = re.compile(r'\x1b\[[0-9;]*m')
# mission_states.PRE_RAMP_X and the four lane centres, world frame.
PRE_RAMP_X = 0.5
LANES = {'red': -0.75, 'green': -0.25, 'blue': 0.25, 'yellow': 0.75}
# mission_states.GOAL_XY_TOLERANCE / GOAL_XY_CONSISTENCY.
XY_TOLERANCE = 0.25
XY_CONSISTENCY = 0.50


def read(path):
    try:
        with open(path, errors='replace') as f:
            return ANSI.sub('', f.read())
    except OSError:
        return ''


def grep(text, pattern):
    return [ln.strip() for ln in text.splitlines() if re.search(pattern, ln)]


def rows(path):
    try:
        with open(path, newline='') as f:
            return list(csv.DictReader(f))
    except OSError:
        return []


def fnum(row, key):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return None


def first_stop(trace, state):
    """Where the wheels first stopped while `state` was current, and after.

    Returns the ground-truth pose at the first sample in `state` whose wheel
    speed has fallen back to zero after having been non-zero, plus the
    largest wheel speed seen anywhere after that instant while still in the
    state. A retry that cannot move the robot leaves that maximum at 0.000,
    which is exactly what C2-NAV.44 measured three times.
    """
    seen_moving = False
    stop_at = None
    v_after = 0.0
    for r in trace:
        if r.get('mission_state') != state:
            continue
        v = fnum(r, 'v_wheel')
        if v is None:
            continue
        if abs(v) > 1e-4:
            seen_moving = True
            if stop_at is not None:
                v_after = max(v_after, abs(v))
        elif seen_moving and stop_at is None:
            stop_at = (fnum(r, 'x'), fnum(r, 'y'), fnum(r, 't_rel'))
    return stop_at, round(v_after, 4)


def classify(error):
    if error is None:
        return 'no arrival recorded'
    if error <= XY_TOLERANCE:
        return 'clean (inside the tolerance)'
    if error <= XY_CONSISTENCY:
        return 'DISCREPANCY ACCEPTED (inside the consistency band)'
    return 'REGION FAILURE (past the consistency band)'


def report(run):
    name = os.path.basename(run.rstrip('/'))
    meta = dict(ln.split('=', 1) for ln in read(os.path.join(run, 'meta.txt')).splitlines()
                if '=' in ln)
    colour = meta.get('colour', '?')
    goal = (PRE_RAMP_X, LANES.get(colour, 0.0))
    mission = read(os.path.join(run, 'mission.log'))
    trace = rows(os.path.join(run, 'cmdpath', 'trace.csv'))

    out = {'run': name, 'colour': colour, 'head': meta.get('head'),
           'dirty_paths': meta.get('dirty_paths'), 'pre_ramp_goal': goal}

    # --- what the executive's gate logged --------------------------------
    arrivals = []
    for ln in grep(mission, r'arrived: ground truth'):
        m = re.search(r'([A-Z_]+) arrived: ground truth ([0-9.]+) m', ln)
        if m:
            arrivals.append({'leg': m.group(1), 'error_m': float(m.group(2)),
                             'warned': 'Nav2 reported SUCCESS but' in ln,
                             'line': ln.split(']: ', 1)[-1]})
    out['arrival_lines'] = arrivals
    pre = [a for a in arrivals if a['leg'] == 'NAVIGATE_TO_RAMP']
    home = [a for a in arrivals if a['leg'] == 'RETURN_HOME']
    out['pre_ramp'] = {
        'arrivals': pre,
        'error_m': pre[-1]['error_m'] if pre else None,
        'verdict': classify(pre[-1]['error_m'] if pre else None),
        'warned': bool(pre and pre[-1]['warned']),
    }
    out['home'] = {'arrivals': home,
                   'error_m': home[-1]['error_m'] if home else None,
                   'verdict': classify(home[-1]['error_m'] if home else None)}

    # --- what Nav2 said ---------------------------------------------------
    out['nav2'] = {
        'reached_the_goal': len(grep(mission, r'\[controller_server.*Reached the goal')),
        'goal_succeeded': len(grep(mission, r'\[bt_navigator.*Goal succeeded')),
        'goal_failed': len(grep(mission, r'\[bt_navigator.*Goal failed')),
    }

    # --- what the gate did about it --------------------------------------
    out['region_failures'] = len(grep(mission, r'PRE_RAMP_POSE_OUT_OF_REGION'))
    out['home_region_failures'] = len(grep(mission, r'HOME_POSE_OUT_OF_REGION'))
    goals = grep(mission, r'NavigateToPose -> world')
    out['nav_goals_sent'] = goals
    out['pre_ramp_goals_sent'] = sum(
        1 for g in goals
        if re.search(rf'world \({goal[0]:.2f}, {goal[1]:+.2f}\)'.replace('+', r'[+-]?'), g)
        or f'{goal[0]:.2f}' in g and f'{abs(goal[1]):.2f}' in g)
    out['retries'] = {
        'recovery_entries': len(grep(mission, r'-> RECOVERY')),
        'nav_retry_lines': grep(mission, r'RECOVERY -> NAVIGATE_TO_RAMP'),
    }

    # --- the futility measurement, from the trace ------------------------
    stop, v_after = first_stop(trace, 'NAVIGATE_TO_RAMP')
    out['pre_ramp_trace'] = {
        'first_stop_world': None if not stop else [round(stop[0], 4), round(stop[1], 4)],
        'first_stop_t_rel': None if not stop else round(stop[2], 2),
        'true_error_at_first_stop_m': (
            None if not stop
            else round(math.hypot(stop[0] - goal[0], stop[1] - goal[1]), 4)),
        'max_wheel_speed_after_first_stop_ms': v_after,
        'futile_retry_observed': bool(out['retries']['nav_retry_lines']) and v_after < 1e-3,
    }

    # --- did the rest of M6 happen ---------------------------------------
    final = read(os.path.join(run, 'final_state.txt')).strip().splitlines()
    fs = dict(tok.split('=', 1) for tok in (final[0].split() if final else []) if '=' in tok)
    out['final'] = {k: fs.get(k) for k in ('state', 'reason', 'result')}
    out['mission_line'] = grep(mission, r'\]: MISSION ')
    out['pre_climb_heading'] = grep(mission, r'pre-climb heading')
    out['grasp'] = grep(mission, r'lifted=1|Target lifted')[-2:]
    out['base_x'] = grep(mission, r'x=0\.1[0-9]+')[-2:]
    return out


def main(argv):
    out = None
    if '--json' in argv:
        i = argv.index('--json')
        out, argv = argv[i + 1], argv[:i] + argv[i + 2:]
    reports = [report(d) for d in argv]
    text = json.dumps(reports, indent=1)
    if out:
        with open(out, 'w') as f:
            f.write(text + '\n')
    print(text)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
