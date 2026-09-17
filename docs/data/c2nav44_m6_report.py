#!/usr/bin/env python3
"""C2-NAV.44: offline per-run report for an M6 fetch recorded by m6_run.sh.

No ROS. Reads only the files the runner wrote. Command-path metrics are the
recorder's own summary.json (C2-NAV.41 functions, unchanged); this script
adds only counting over logs and the trace.

    python3 -P m6_report.py RUN_DIR [RUN_DIR ...] [--json OUT]
"""
import csv
import json
import math
import os
import re
import sys
from collections import Counter

WORLD_TO_MAP_X = 2.0          # mission_states.WORLD_TO_MAP_X = -SPAWN_XY[0], SPAWN_XY (-2.0, 0.0)
NAV_STATES = ('NAVIGATE_TO_RAMP', 'RETURN_HOME')
ANSI = re.compile(r'\x1b\[[0-9;]*m')


def read(path):
    try:
        with open(path, errors='replace') as f:
            return ANSI.sub('', f.read())
    except OSError:
        return ''


def kv(line):
    return dict(tok.split('=', 1) for tok in line.split() if '=' in tok)


def grep(text, pattern):
    return [ln for ln in text.splitlines() if re.search(pattern, ln)]


def state_durations(hrec_rows):
    """Consecutive-state segments from hrec (10 Hz, sim time)."""
    segs = []
    for r in hrec_rows:
        try:
            t = float(r['t_sim'])
        except (TypeError, ValueError):
            continue
        st = r.get('state') or '--'
        if not segs or segs[-1][0] != st:
            segs.append([st, t, t])
        else:
            segs[-1][2] = t
    # a segment ends where the next begins
    out = []
    for i, (st, t0, t1) in enumerate(segs):
        end = segs[i + 1][1] if i + 1 < len(segs) else t1
        out.append((st, round(t0, 2), round(end - t0, 2)))
    return out


def polygon_stop(trace):
    episodes, rows, by_state = 0, 0, Counter()
    prev = None
    for r in trace:
        a = r.get('cm_action')
        if a == '1':
            rows += 1
            by_state[r.get('mission_state') or '--'] += 1
            if prev != '1':
                episodes += 1
        prev = a
    slow = sum(1 for r in trace if r.get('cm_action') == '2')
    return {'stop_episodes': episodes, 'stop_rows': rows,
            'stop_seconds_10hz': round(rows / 10.0, 1),
            'stop_rows_by_state': dict(by_state), 'slowdown_rows': slow}


def amcl_final(text):
    m = re.search(r'position:\s*\n\s*x:\s*(\S+)\s*\n\s*y:\s*(\S+)', text)
    if not m:
        return None
    return float(m.group(1)), float(m.group(2))


def report(run):
    r = {'run': os.path.basename(run.rstrip('/'))}
    meta = dict(ln.split('=', 1) for ln in read(os.path.join(run, 'meta.txt')).splitlines() if '=' in ln)
    r['meta'] = meta
    runner = read(os.path.join(run, 'runner.log'))
    r['runner_checks'] = {'pass': len(grep(runner, r'm6_run: PASS')),
                          'fail': grep(runner, r'm6_run: FAIL')}
    r['runner_torn_down'] = grep(runner, r'torn down')
    r['wall_start_to_terminal'] = grep(runner, r'wall seconds from start call')

    final = read(os.path.join(run, 'final_state.txt')).strip().splitlines()
    fs = kv(final[0]) if final else {}
    r['final'] = {k: fs.get(k) for k in ('state', 'prev', 'reason', 'result', 'attempt')}

    mission = read(os.path.join(run, 'mission.log'))
    sim = read(os.path.join(run, 'sim.log'))
    trans = []
    for ln in mission.splitlines():
        m = re.search(r'\[mission_executive\]: ([A-Z_]+) -> ([A-Z_]+)(?: \[([^\]]*)\])?(?:: (.*))?$', ln)
        if m:
            trans.append({'from': m.group(1), 'to': m.group(2),
                          'reason': m.group(3), 'detail': m.group(4)})
    r['transitions'] = trans
    r['mission_line'] = grep(mission, r'\]: MISSION ')
    r['state_entries'] = dict(Counter(t['to'] for t in trans))
    r['recovery_entries'] = sum(1 for t in trans if t['to'] in ('RECOVERY', 'RELOCALIZE'))
    r['abort_entries'] = sum(1 for t in trans if t['to'] == 'ABORT')
    r['nav_goals_sent'] = grep(mission, r'NavigateToPose -> world')
    r['pre_climb_heading'] = grep(mission, r'pre-climb heading')
    r['localization_recovery_lines'] = grep(mission, r'localization recovery')

    nav = {
        'bt_goal_succeeded': len(grep(mission, r'\[bt_navigator.*Goal succeeded')),
        'bt_goal_failed': len(grep(mission, r'\[bt_navigator.*Goal failed')),
        'bt_goal_canceled': len(grep(mission, r'\[bt_navigator.*Goal canceled')),
        'bt_goal_aborted_lines': len(grep(mission, r'\[bt_navigator.*[Aa]bort')),
        'controller_failed_progress': len(grep(mission, r'Failed to make progress')),
        'controller_reached_goal': len(grep(mission, r'\[controller_server.*Reached the goal')),
        'planner_failed': len(grep(mission, r'\[planner_server.*(Failed|failed|No valid path|Start occupied|Goal occupied)')),
        'behavior_running': dict(Counter(re.search(r'Running (\w+)', ln).group(1)
                                         for ln in grep(mission, r'\[behavior_server.*Running \w+'))),
        'clear_costmap': len(grep(mission, r'[Cc]lear(ing)?.*[Cc]ostmap')),
        'dwb_no_valid_trajectories': len(grep(mission, r'No valid trajectories')),
        'controller_missed_rate': len(grep(mission, r'\[controller_server.*missed its desired rate')),
    }
    r['nav2'] = nav
    r['stale_cmd_drops'] = {'sim_log': len(grep(sim, r'Ignoring the received message')),
                            'mission_log': len(grep(mission, r'Ignoring the received message'))}
    r['process_died'] = grep(mission + '\n' + sim, r'process has died|exited with code [1-9]')

    r['manipulation'] = {
        'lifted': grep(mission, r'Target lifted'),
        'place': grep(mission, r'place finished|placed'),
        'climb': grep(mission, r'\[ramp_driver.*outcome=')[-3:],
        'approach': grep(mission, r'\[approach_server.*outcome=')[-3:],
    }

    # health + per-state sim durations
    rows = []
    try:
        with open(os.path.join(run, 'hrec.csv'), newline='') as f:
            rows = list(csv.DictReader(f))
    except OSError:
        pass
    segs = state_durations(rows)
    r['state_path_sim'] = segs
    active = [s for s in segs if s[0] not in ('IDLE', '--', '', 'COMPLETE', 'ABORT')]
    if active:
        term = next((s for s in segs if s[0] in ('COMPLETE', 'ABORT')), None)
        t_end = term[1] if term else (active[-1][1] + active[-1][2])
        r['mission_sim_s_first_active_to_terminal'] = round(t_end - active[0][1], 2)
    r['health'] = {
        'samples': len(rows),
        'verdict': dict(Counter(x.get('verdict') for x in rows)),
        'reason': dict(Counter(x.get('reason') for x in rows)),
        'degraded_1': sum(1 for x in rows if x.get('degraded') == '1'),
    }
    if rows:
        last = rows[-1]
        gt = (float(last['gt_x']), float(last['gt_y']), float(last['gt_yaw']))
        r['final_gt_world'] = [round(v, 4) for v in gt]
        am = amcl_final(read(os.path.join(run, 'final_amcl_pose.txt')))
        if am:
            r['final_amcl_map'] = [round(v, 4) for v in am]
            r['final_amcl_gap_m'] = round(math.hypot(am[0] - (gt[0] + WORLD_TO_MAP_X), am[1] - gt[1]), 4)

    # command path
    summ = {}
    try:
        with open(os.path.join(run, 'cmdpath', 'summary.json')) as f:
            summ = json.load(f)
    except (OSError, ValueError):
        pass
    trace = []
    try:
        with open(os.path.join(run, 'cmdpath', 'trace.csv'), newline='') as f:
            trace = list(csv.DictReader(f))
    except OSError:
        pass
    if summ:
        nar = summ.get('nav_active_rows', {})
        r['cmdpath'] = {
            'record_stopped_by': summ.get('record_stopped_by'),
            'trace_rows': summ.get('trace_rows'),
            'wheel_messages': summ.get('wheel_messages'),
            'nav_active': {
                'rows': nar.get('rows'),
                'excluded_switch_guard': nar.get('rows_excluded_within_switch_guard'),
                'monitor_exceeded': (nar.get('monitor_authority') or {}).get('exceeded'),
                'monitor_samples': (nar.get('monitor_authority') or {}).get('samples'),
                'monitor_worst_gap_ms': (nar.get('monitor_authority') or {}).get('worst_gap_ms'),
                'gated_zero_moving': (nar.get('monitor_authority') or {}).get('gated_zero_moving'),
                'bypass_rows': (nar.get('bypass_source') or {}).get('rows'),
                'bypass_wheel_eq_raw_controller': (nar.get('bypass_source') or {}).get('wheel_matches_raw_controller'),
                'bypass_wheel_eq_smoother': (nar.get('bypass_source') or {}).get('wheel_matches_smoother'),
                'stop_rows': (nar.get('stop_breach') or {}).get('stop_rows'),
                'stop_rows_wheels_driven': (nar.get('stop_breach') or {}).get('stop_rows_wheels_driven'),
                'worst_wheel_during_stop_ms': (nar.get('stop_breach') or {}).get('worst_wheel_during_stop_ms'),
            },
            'all_rows': {
                'monitor_exceeded': summ['monitor_authority']['exceeded'],
                'monitor_samples': summ['monitor_authority']['samples'],
                'bypass_rows': summ['bypass_source']['rows'],
                'bypass_cm_action': summ['bypass_source']['cm_action'],
                'bypass_wheel_eq_raw_controller': summ['bypass_source']['wheel_matches_raw_controller'],
                'bypass_wheel_eq_smoother': summ['bypass_source']['wheel_matches_smoother'],
                'stop_rows': summ['stop_breach']['stop_rows'],
                'stop_rows_wheels_driven': summ['stop_breach']['stop_rows_wheels_driven'],
            },
            'smoother_events': summ.get('smoother'),
            'cm_action_rows': summ.get('cm_action_rows'),
            'relocalize_rows': (summ.get('relocalize_rows') or {}).get('rows'),
            'max_wheel_v_ms': summ.get('max_wheel_v_ms'),
            'max_raw_v_ms': summ.get('max_raw_v_ms'),
            'min_scan_m': summ.get('min_scan_m'),
        }
        if trace:
            by_src = Counter(row.get('arbiter_active') or 'blank' for row in trace)
            r['cmdpath']['arbiter_active_rows'] = dict(by_src)
    if trace:
        r['polygon_stop'] = polygon_stop(trace)
        r['final_gt_trace'] = [trace[-1].get('x'), trace[-1].get('y'), trace[-1].get('yaw')]

    # graph
    topo = read(os.path.join(run, 'topology_live.txt'))
    r['topology'] = {'ok': len(grep(topo, r'^OK')), 'mismatch': grep(topo, r'^MISMATCH')}
    params = read(os.path.join(run, 'params_live.txt'))
    r['params_live_mismatch'] = len(grep(params, r'^MISMATCH'))
    r['depth_off'] = read(os.path.join(run, 'depth_off.txt')).strip().splitlines()
    wp = read(os.path.join(run, 'wheel_publishers.txt')).strip().splitlines()
    counts = [ln.split('publishers=')[1].split()[0] for ln in wp if 'publishers=' in ln]
    r['wheel_publisher_polls'] = {'polls': len(wp), 'counts': dict(Counter(counts)),
                                  'nodes': dict(Counter(ln.split('nodes=')[1].split()[0] for ln in wp if 'nodes=' in ln))}
    r['ros_clean_after'] = read(os.path.join(run, 'ros_clean_after.txt')).strip().splitlines()
    return r


def main(argv):
    out = None
    if '--json' in argv:
        i = argv.index('--json')
        out = argv[i + 1]
        argv = argv[:i] + argv[i + 2:]
    reports = [report(d) for d in argv]
    text = json.dumps(reports, indent=1)
    if out:
        with open(out, 'w') as f:
            f.write(text + '\n')
    print(text)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
