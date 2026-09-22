# Copyright 2026 Gautham Anil -- Apache-2.0.
"""
Join the browser's action timeline with what reached the wheels.

    python3 analyse_live.py <outdir>

Reads <outdir>/actions.json (live.py), recorder.jsonl (wheel_recorder.py)
and metrics.jsonl (metrics_sampler.py). Prints one JSON verdict document.
Every number is computed from those files; nothing is assumed.
"""

import json
import os
import sys

OUT = sys.argv[1]
EPS = 1e-3


def load_jsonl(name):
    rows = []
    path = os.path.join(OUT, name)
    if not os.path.exists(path):
        return rows
    with open(path) as handle:
        for line in handle:
            line = line.strip()
            if line:
                try:
                    rows.append(json.loads(line))
                except ValueError:
                    pass
    return rows


with open(os.path.join(OUT, 'actions.json')) as handle:
    actions = json.load(handle)
rec = load_jsonl('recorder.jsonl')
metrics = [r for r in load_jsonl('metrics.jsonl') if 'm' in r]


def at(name):
    for a in actions:
        if a['action'] == name:
            return a
    return None


def rows(kind, t0, t1):
    return [r for r in rec if r['k'] == kind and t0 <= r['t'] <= t1]


def moving(r):
    return abs(r['lin']) > EPS or abs(r['ang']) > EPS


def window(label, t0, t1, sign):
    wheel = rows('wheel', t0, t1 + 1.0)
    teleop = rows('teleop', t0, t1 + 1.0)
    first = next((r for r in wheel if moving(r)), None)
    lin = [r['lin'] for r in wheel]
    return {
        'label': label,
        'teleop_msgs': len(teleop),
        'wheel_msgs': len(wheel),
        'wheel_moving_msgs': sum(1 for r in wheel if moving(r)),
        'max_wheel_lin': max(lin, default=None) if sign > 0
        else min(lin, default=None),
        'key_to_first_wheel_motion_ms': (
            round((first['t'] - t0) * 1000, 1) if first else None),
    }


def stop_after(label, t_event, horizon=2.5):
    """Last moving wheel command after `t_event`, and zeros that follow."""
    after = rows('wheel', t_event, t_event + horizon)
    last_moving = None
    for r in after:
        if moving(r):
            last_moving = r
    first_zero = next((r for r in after if not moving(r)), None)
    moving_later = [r for r in after
                    if moving(r) and r['t'] > t_event + 0.6]
    return {
        'label': label,
        'wheel_msgs_in_window': len(after),
        'first_zero_after_ms': (round((first_zero['t'] - t_event) * 1000, 1)
                                if first_zero else None),
        'last_moving_after_ms': (round((last_moving['t'] - t_event) * 1000, 1)
                                 if last_moving else None),
        'moving_commands_later_than_600ms': len(moving_later),
    }


report = {'outdir': OUT}
for name in ('stop_hit_at_open', 'page_ready', 'stop_hit_ready',
             'annotated_mjpeg', 'telemetry_and_lidar', 'camera_first_frame',
             'depth_first_frame', 'back_to_play', 'localised',
             'colour_confirmed', 'stop_hit_mission_running',
             'js_errors_before_disconnect', 'js_errors_mission'):
    a = at(name)
    if a:
        report[name] = {k: v for k, v in a.items() if k not in ('t',)}

fd, fu = at('drive_forward_down'), at('drive_forward_up')
bd, bu = at('drive_back_down'), at('drive_back_up')
if fd and fu:
    report['manual_forward'] = window('W held', fd['t'], fu['t'], +1)
if bd and bu:
    report['manual_back'] = window('S held', bd['t'], bu['t'], -1)

# The joystick: a real pointer drag on the nipplejs pad.
jd, ju = at('joy_forward_down'), at('joy_forward_up')
kd, ku = at('joy_back_down'), at('joy_back_up')
if jd and ju:
    report['joystick_forward'] = window('stick dragged up', jd['t'], ju['t'],
                                        +1)
    # The window ends at the next drag: the second drag starts ~1.5 s after
    # the first release, and counting its commands as "after release" was
    # an artifact of run 1's first analysis (0 moving, measured directly).
    report['joystick_release'] = stop_after(
        'stick released', ju['t'],
        horizon=min(2.0, kd['t'] - ju['t']) if kd else 2.0)
if kd and ku:
    report['joystick_back'] = window('stick dragged down', kd['t'], ku['t'],
                                     -1)
    report['joystick_back_release'] = stop_after('stick released',
                                                 ku['t'], horizon=1.4)

sd, sc = at('stop_test_w_down'), at('stop_clicked_w_still_held')
if sd and sc:
    report['stop_test_driving'] = window('W held before STOP', sd['t'],
                                         sc['t'], +1)
    report['stop_test'] = stop_after('STOP clicked, W still held', sc['t'],
                                     horizon=2.4)

dd, kill = at('disconnect_test_w_down'), at('browser_killed_while_driving')
if dd and kill:
    report['disconnect_driving'] = window('W held before kill', dd['t'],
                                          kill['t'], +1)
    report['disconnect_stop'] = stop_after('browser SIGKILLed', kill['t'],
                                           horizon=3.0)

# Mission: the executive's own transitions vs what the page showed.
start = at('mission_start_clicked')
end = at('mission_end')
if start:
    t_end = end['t'] if end else float('inf')
    ros = []
    for r in rec:
        if r['k'] == 'mission' and start['t'] - 1 <= r['t'] <= t_end + 1:
            fields = dict(p.split('=', 1) for p in r['line'].split()
                          if '=' in p)
            ros.append((r['t'], fields.get('state'), fields.get('event'),
                        fields.get('result'), fields.get('reason')))
    changes = []
    last = None
    for t, state, event, result, reason in ros:
        if state != last:
            changes.append({'t': t, 'state': state, 'result': result,
                            'reason': reason})
            last = state
    # Real-time factor, measured: the executive's own `elapsed` (ROS clock)
    # at the last line of each state, against the wall time that state
    # lasted on the recorder's clock. States under 2 s of wall are left
    # out -- the recorder's arrival jitter dominates them.
    elapsed_by_change = []
    for index, change in enumerate(changes[:-1]):
        t_next = changes[index + 1]['t']
        sims = []
        for r in rec:
            if (r['k'] == 'mission' and change['t'] <= r['t'] < t_next):
                for part in r['line'].split():
                    if part.startswith('elapsed='):
                        try:
                            sims.append(float(part.split('=', 1)[1]))
                        except ValueError:
                            pass
        wall = t_next - change['t']
        if sims and wall >= 2.0:
            elapsed_by_change.append((change['state'], max(sims), wall))
    sim_total = sum(s for _n, s, _w in elapsed_by_change)
    wall_total = sum(w for _n, _s, w in elapsed_by_change)
    page = [a for a in actions if a['action'] == 'mission_view']
    # Prefer the in-page MutationObserver log: every state the page
    # rendered, stamped by the page itself. The poll misses short states.
    logged = at('page_state_log')
    exact = bool(logged and logged.get('states'))
    if exact:
        page = [{'t': s['t'], 'state': s['state'], 'badge': s['badge']}
                for s in logged['states']]
    lags = []
    for c in changes:
        seen = next((p for p in page if p.get('state') == c['state']
                     and p['t'] >= c['t'] - 0.05), None)
        if seen:
            lags.append(round((seen['t'] - c['t']) * 1000, 1))
    page_states = []
    for p in page:
        if p.get('state') and (not page_states or
                               page_states[-1] != p['state']):
            page_states.append(p['state'])
    report['mission'] = {
        'ros_states': [c['state'] for c in changes],
        'page_states': page_states,
        'page_badges': [p['badge'] for p in page],
        'ros_final': changes[-1] if changes else None,
        'page_final': end.get('final') if end else None,
        'duration_s': (round(changes[-1]['t'] - changes[0]['t'], 1)
                       if len(changes) > 1 else None),
        'page_lag_ms_min_max': ([min(lags), max(lags)] if lags else None),
        'page_lag_ms_median': (sorted(lags)[len(lags) // 2] if lags
                               else None),
        'page_lag_note': (
            'ROS recorder arrival -> DOM change, both on this machine\'s '
            'wall clock (in-page MutationObserver)' if exact else
            'upper bound: the page is polled every 0.25 s, so this '
            'includes up to 250 ms of polling'),
        'every_ros_state_seen_on_page': (
            all(c['state'] in page_states for c in changes)
            if changes else None),
        'real_time_factor': (round(sim_total / wall_total, 3)
                             if wall_total else None),
        'rtf_basis': {'states': len(elapsed_by_change),
                      'sim_s': round(sim_total, 1),
                      'wall_s': round(wall_total, 1)},
        'state_timing': [{'state': n, 'sim_s': round(s, 1),
                          'wall_s': round(w, 1)}
                         for n, s, w in elapsed_by_change],
    }

graph = [r for r in rec if r['k'] == 'graph']
report['wheel_publishers_seen'] = sorted(
    {tuple(r['wheel_pubs']) for r in graph if r['wheel_pubs']})
report['max_wheel_publishers'] = max(
    (len(r['wheel_pubs']) for r in graph), default=None)
report['teleop_publishers_seen'] = sorted(
    {tuple(r['teleop_pubs']) for r in graph if r['teleop_pubs']})

if metrics:
    cpu = [m['m']['cpu_percent'] for m in metrics]
    report['platform_cpu_percent_one_core'] = {
        'min': min(cpu), 'max': max(cpu),
        'mean': round(sum(cpu) / len(cpu), 1), 'samples': len(cpu)}
    streams = {}
    for m in metrics:
        for name, s in m['m'].get('streams', {}).items():
            d = streams.setdefault(name, {'in_hz_max': 0, 'out_hz_max': 0,
                                          'dropped_max': 0})
            d['in_hz_max'] = max(d['in_hz_max'], s['in_hz'])
            d['out_hz_max'] = max(d['out_hz_max'], s['out_hz'])
            d['dropped_max'] = max(d['dropped_max'], s['dropped'])
    report['streams'] = streams
    lat = [m['m']['mission_latency_ms'] for m in metrics
           if m['m'].get('mission_latency_ms') is not None]
    report['mission_latency_ms'] = ([min(lat), max(lat)] if lat else None)
    report['peak_buffer_bytes'] = max(
        m['m'].get('peak_buffer_bytes', 0) for m in metrics)

probe = os.path.join(OUT, 'safety_probe.json')
if os.path.exists(probe):
    try:
        with open(probe) as handle:
            data = json.load(handle)
        report['safety_probe'] = {
            'all_refused': data.get('all_refused'),
            'codes': {a['attempt']: a['reply'].get('code')
                      for a in data.get('attempts', [])}}
    except ValueError:
        report['safety_probe'] = 'unparseable'

print(json.dumps(report, indent=1, default=str))
