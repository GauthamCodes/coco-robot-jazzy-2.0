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

"""C2-NAV.42 -- the command path, exercised live, after the /cmd_vel_nav fix.

Run against a LIVE topology-B graph (sim + `arbiter.launch.py
initial_mode:=nav` + `nav.launch.py arbiter:=true`). Two controlled
experiments, each recording every link of the chain:

    /cmd_vel_nav        v_nav       the controller's raw command (and the
                                    behavior_server's, and this probe's)
    /cmd_vel_smoothed   v_smoothed  velocity_smoother
    /cmd_vel            v_cmdvel    collision_monitor
    /cmd_vel_gated      v_gated     cmd_vel_relay (C2-NAV.42)
    /diff_drive_controller/cmd_vel  v_wheel  cmd_vel_arbiter

spin   The C2-M5.1 relocalization spin exactly as mission_executive sends
       it: a nav2_msgs/Spin goal with target_yaw = 2*pi and nothing else,
       to behavior_server. Asks: does the turn still reach the wheels, and
       through which stages?

stop   A stand-in controller. This probe turns the robot to face the west
       wall by publishing raw commands on /cmd_vel_nav, then publishes a
       CONSTANT raw 0.30 m/s forward and keeps publishing it. Nothing but
       the safety chain can stop the robot. Asks: does the collision
       monitor's slowdown/stop reach the wheels while the raw command says
       drive? 0.30 m/s is the worst wheel command C2-NAV.41 measured
       leaking past a monitor at 0, and it is above that metric's 0.2 m/s
       "driven" threshold, so a leak would be counted.

THE METRICS ARE NOT REDEFINED. The trace is resampled at 10 Hz sim time
with the zero-order hold nav_bench.write_trace uses, and `monitor_authority`,
`bypass_source` and `stop_breach` are imported from
docs/data/c2nav41_topology.py and applied unchanged.

Message-level columns are kept as well (`events.csv`): for each wheel
command, the most recent message on every upstream link. The smoother check
reads those, because at 2.5 m/s^2 its ramp lasts about two 20 Hz cycles and
a 10 Hz resample can hide it.

record A passive recorder for a whole mission (mission.launch.py): the same
       columns plus /mission/state, until SIGINT, --duration wall seconds, or
       (--until-terminal) the mission reaching COMPLETE or ABORT. The rl and
       approach sources legitimately drive the wheels while Nav2's monitor
       is idle, so the metrics are ALSO reported over only the rows where the
       arbiter's active source was `nav` -- the rows the Nav2 safety chain
       owns -- and over the RELOCALIZE rows.

  python3 -P docs/data/c2nav42_cmdpath.py spin --out DIR
  python3 -P docs/data/c2nav42_cmdpath.py stop --out DIR
  python3 -P docs/data/c2nav42_cmdpath.py record --out DIR [--duration S] [--until-terminal]
  python3 -P docs/data/c2nav42_cmdpath.py summarise DIR      # offline
"""

import argparse
import bisect
import csv
import importlib.util
import json
import math
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))

CHAIN = (('nav', '/cmd_vel_nav'), ('smoothed', '/cmd_vel_smoothed'),
         ('cmdvel', '/cmd_vel'), ('gated', '/cmd_vel_gated'),
         ('wheel', '/diff_drive_controller/cmd_vel'))
TRACE_HZ = 10.0
RAW_DRIVE = 0.30          # m/s, see the module docstring
RAW_TURN = 0.5            # rad/s, facing the wall
PUBLISH_HZ = 20.0         # the controller's own rate class
WEST = math.pi
TURN_TOL = 0.05           # rad
STOP_HOLD_S = 5.0         # sim seconds of STOP with the wheels still
DRIVE_BUDGET_S = 40.0
SPIN_BUDGET_S = 90.0
STILL = 0.01              # m/s -- c2nav41_topology.MOVING
TOL = 0.02                # m/s -- c2nav41_topology.JITTER_TOL
# /cmd_vel_arbiter/status is 2 Hz, so a row's `arbiter_active` can lag a real
# source switch by up to 0.5 s in either direction; rows this close to a switch
# are left out of the nav-owned subset (and counted) rather than attributed.
SWITCH_GUARD_S = 1.0


def _topology_module():
    spec = importlib.util.spec_from_file_location(
        'c2nav41_topology', os.path.join(HERE, 'c2nav41_topology.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# ── offline: trace construction and summary (no ROS) ───────────────────────
def last_at(ts, vs, t):
    """nav_bench.write_trace's zero-order hold: the last sample at or before t."""
    i = bisect.bisect_right(ts, t)
    return vs[i - 1] if i else None


def build_trace(series, t0, t1):
    """Rows at TRACE_HZ, nav_bench column names where nav_bench has them."""
    cols = {k: ([s[0] for s in series[k]], [s[1] for s in series[k]])
            for k in series}
    rows = []
    n = int(math.floor((t1 - t0) * TRACE_HZ + 1e-9))
    for i in range(n + 1):
        t = t0 + i / TRACE_HZ
        row = {'t_rel': round(t - t0, 2)}
        for key, _ in CHAIN:
            v = last_at(*cols[key], t)
            row[f'v_{key}'] = '' if v is None else round(v[0], 4)
            row[f'w_{key}'] = '' if v is None else round(v[1], 4)
        cm = last_at(*cols['cm'], t)
        row['cm_action'] = '' if cm is None else str(cm[0])
        row['cm_polygon'] = '' if cm is None else cm[1]
        gt = last_at(*cols['gt'], t)
        for j, name in enumerate(('x', 'y', 'yaw', 'v_act', 'w_act')):
            row[name] = '' if gt is None else round(gt[j], 4)
        sc = last_at(*cols['scan'], t)
        row['scan_min'] = '' if sc is None else round(sc, 4)
        arb = last_at(*cols['arbiter'], t)
        row['arbiter_active'] = '' if arb is None else arb
        if 'mission' in cols:
            st = last_at(*cols['mission'], t)
            row['mission_state'] = '' if st is None else st
        rows.append(row)
    return rows


def build_events(series):
    """One row per wheel message: the latest message on every upstream link."""
    cols = {k: ([s[0] for s in series[k]], [s[1] for s in series[k]])
            for k, _ in CHAIN}
    out = []
    for t, (v, w) in series['wheel']:
        row = {'t': round(t, 4), 'v_wheel': v, 'w_wheel': w}
        for key, _ in CHAIN[:-1]:
            last = last_at(*cols[key], t)
            row[f'v_{key}'] = None if last is None else last[0]
            row[f'w_{key}'] = None if last is None else last[1]
        out.append(row)
    return out


def _f(row, key):
    value = row.get(key)
    if value is None or value == '':
        return None
    return float(value)


def angular_authority(rows, tol=0.05):
    """monitor_authority's definition on the ANGULAR columns, for the spin.

    Not one of C2-NAV.41's metrics -- it had none for rotation -- so reported
    separately and labelled as this module's.
    """
    samples = exceeded = 0
    worst = 0.0
    for row in rows:
        monitor, wheel = _f(row, 'w_cmdvel'), _f(row, 'w_wheel')
        if monitor is None or wheel is None:
            continue
        samples += 1
        gap = abs(wheel) - abs(monitor)
        if gap > tol:
            exceeded += 1
            worst = max(worst, gap)
    return {'samples': samples, 'exceeded': exceeded,
            'worst_gap_rad_s': round(worst, 4) if exceeded else None,
            'tol_rad_s': tol}


def smoother_evidence(events, tol=1e-6):
    """Wheel messages whose value equals the smoother's latest output but NOT
    the raw command's -- the smoother demonstrably between the two -- and
    wheel messages equal to the raw command where the smoother differed.
    """
    through = raw_only = differing = 0
    for e in events:
        if None in (e['v_nav'], e['v_smoothed']):
            continue
        for axis in ('v', 'w'):
            nav, sm, wh = e[f'{axis}_nav'], e[f'{axis}_smoothed'], e[f'{axis}_wheel']
            if abs(nav - sm) <= tol:
                continue
            differing += 1
            if abs(wh - sm) <= tol:
                through += 1
            elif abs(wh - nav) <= tol:
                raw_only += 1
    return {'rows_raw_ne_smoothed': differing,
            'wheel_eq_smoothed': through,
            'wheel_eq_raw_only': raw_only}


def summarise_rows(rows, events, meta):
    top = _topology_module()
    actions = {}
    for row in rows:
        key = row.get('cm_action') or 'blank'
        actions[key] = actions.get(key, 0) + 1
    speeds = [abs(_f(r, 'v_wheel')) for r in rows if _f(r, 'v_wheel') is not None]
    ang = [abs(_f(r, 'w_wheel')) for r in rows if _f(r, 'w_wheel') is not None]
    raw_speeds = [abs(_f(r, 'v_nav')) for r in rows if _f(r, 'v_nav') is not None]
    summary = dict(meta)
    summary.update({
        'trace_rows': len(rows),
        'wheel_messages': len(events),
        'monitor_authority': top.monitor_authority(rows),
        'bypass_source': top.bypass_source(rows),
        'stop_breach': top.stop_breach(rows),
        'angular_authority': angular_authority(rows),
        'smoother': smoother_evidence(events),
        'cm_action_rows': dict(sorted(actions.items())),
        'max_wheel_v_ms': round(max(speeds), 4) if speeds else None,
        'max_wheel_w_rad_s': round(max(ang), 4) if ang else None,
        'max_raw_v_ms': round(max(raw_speeds), 4) if raw_speeds else None,
    })
    scan = [_f(r, 'scan_min') for r in rows if _f(r, 'scan_min') is not None]
    if scan:
        summary['min_scan_m'] = round(min(scan), 4)
    if any('mission_state' in r for r in rows):
        switches = [float(b['t_rel']) for a, b in zip(rows, rows[1:])
                    if a.get('arbiter_active') != b.get('arbiter_active')]

        def near_switch(row):
            t = float(row['t_rel'])
            i = bisect.bisect_left(switches, t - SWITCH_GUARD_S)
            return i < len(switches) and switches[i] <= t + SWITCH_GUARD_S
        labelled_nav = [r for r in rows if r.get('arbiter_active') == 'nav']
        nav_rows = [r for r in labelled_nav if not near_switch(r)]
        labelled_reloc = [r for r in rows if r.get('mission_state') == 'RELOCALIZE']
        reloc = [r for r in labelled_reloc if not near_switch(r)]
        states = []
        for r in rows:
            st = r.get('mission_state') or '--'
            if not states or states[-1][0] != st:
                states.append([st, r['t_rel']])
        summary['mission_states'] = states
        summary['nav_active_rows'] = {
            'rows': len(nav_rows),
            'rows_excluded_within_switch_guard': len(labelled_nav) - len(nav_rows),
            'switch_guard_s': SWITCH_GUARD_S,
            'monitor_authority': top.monitor_authority(nav_rows),
            'bypass_source': top.bypass_source(nav_rows),
            'stop_breach': top.stop_breach(nav_rows),
        }
        summary['relocalize_rows'] = {
            'rows': len(reloc),
            'rows_excluded_within_switch_guard': len(labelled_reloc) - len(reloc),
            'monitor_authority': top.monitor_authority(reloc),
            'bypass_source': top.bypass_source(reloc),
            'angular_authority': angular_authority(reloc),
            'max_wheel_w_rad_s': (round(max(abs(_f(r, 'w_wheel')) for r in reloc
                                            if _f(r, 'w_wheel') is not None), 4)
                                  if any(_f(r, 'w_wheel') is not None for r in reloc)
                                  else None),
            'arbiter_active': sorted({r.get('arbiter_active') for r in reloc}),
            'cm_action_rows': {k: sum(1 for r in reloc if (r.get('cm_action') or 'blank') == k)
                               for k in sorted({(r.get('cm_action') or 'blank') for r in reloc})},
        }
    return summary


def write_csv(path, rows):
    if not rows:
        open(path, 'w').close()
        return
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def mode_summarise(out_dir):
    with open(os.path.join(out_dir, 'meta.json')) as f:
        meta = json.load(f)
    with open(os.path.join(out_dir, 'trace.csv'), newline='') as f:
        rows = list(csv.DictReader(f))
    with open(os.path.join(out_dir, 'events.csv'), newline='') as f:
        events = [{k: (None if v == '' else (float(v) if k != 't' else float(v)))
                   for k, v in r.items()} for r in csv.DictReader(f)]
    summary = summarise_rows(rows, events, meta)
    print(json.dumps(summary, indent=1))
    return 0


# ── live ────────────────────────────────────────────────────────────────────
def run_live(mode, out_dir, duration=0.0, until_terminal=False):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data

    from geometry_msgs.msg import TwistStamped
    from nav2_msgs.action import Spin
    from nav2_msgs.msg import CollisionMonitorState
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String

    os.makedirs(out_dir, exist_ok=True)
    rclpy.init()
    node = Node('c2nav42_cmdpath',
                parameter_overrides=[Parameter('use_sim_time', value=True)])
    lock = threading.Lock()
    series = {k: [] for k, _ in CHAIN}
    series.update({'cm': [], 'gt': [], 'scan': [], 'arbiter': []})
    if mode == 'record':
        series['mission'] = []

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    def add(key, value):
        with lock:
            series[key].append((now(), value))

    for key, topic in CHAIN:
        node.create_subscription(
            TwistStamped, topic,
            (lambda k: lambda m: add(k, (m.twist.linear.x, m.twist.angular.z)))(key), 50)
    node.create_subscription(
        CollisionMonitorState, '/collision_monitor_state',
        lambda m: add('cm', (int(m.action_type), str(m.polygon_name))), 10)

    def on_gt(m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        add('gt', (m.pose.pose.position.x, m.pose.pose.position.y, yaw,
                   m.twist.twist.linear.x, m.twist.twist.angular.z))
    node.create_subscription(Odometry, '/model/coco/odometry', on_gt, 20)

    def on_scan(m):
        good = [r for r in m.ranges if m.range_min <= r <= m.range_max and math.isfinite(r)]
        add('scan', min(good) if good else float('inf'))
    node.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data)

    def on_arbiter(m):
        active = next((tok.split('=', 1)[1] for tok in m.data.split()
                       if tok.startswith('active=')), '')
        add('arbiter', active)
    node.create_subscription(String, '/cmd_vel_arbiter/status', on_arbiter, 10)
    if mode == 'record':
        def on_mission(m):
            state = next((tok.split('=', 1)[1] for tok in m.data.split()
                          if tok.startswith('state=')), '')
            add('mission', state)
        node.create_subscription(String, '/mission/state', on_mission, 10)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    def latest(key):
        with lock:
            return series[key][-1][1] if series[key] else None

    def wait_for(cond, secs):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            if cond():
                return True
            time.sleep(0.05)
        return False

    meta = {'mode': mode, 'started_wall_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}
    ok = wait_for(lambda: latest('gt') is not None and latest('scan') is not None
                  and node.get_clock().now().nanoseconds > 0, 30.0)
    meta['inputs_seen'] = ok
    if not ok:
        print('c2nav42_cmdpath: no ground truth / scan / clock -- refusing', file=sys.stderr)
        rclpy.shutdown()
        return 4
    t0 = now()
    gt0 = latest('gt')
    meta['start_pose'] = [round(v, 4) for v in gt0[:3]]

    raw_pub = node.create_publisher(TwistStamped, '/cmd_vel_nav', 10)

    def publish_raw(v, w):
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        raw_pub.publish(msg)

    def publish_for(v, w, secs, until=None):
        """Publish at PUBLISH_HZ for up to `secs` WALL seconds or until `until`."""
        end = time.monotonic() + secs
        while time.monotonic() < end:
            publish_raw(v, w)
            if until and until():
                return True
            time.sleep(1.0 / PUBLISH_HZ)
        return False

    if mode == 'record':
        import signal
        stop = threading.Event()
        signal.signal(signal.SIGINT, lambda *_: stop.set())
        signal.signal(signal.SIGTERM, lambda *_: stop.set())
        end = time.monotonic() + duration
        while not stop.is_set() and time.monotonic() < end:
            if until_terminal and latest('mission') in ('COMPLETE', 'ABORT'):
                time.sleep(3.0)
                break
            time.sleep(0.2)
        meta['record_stopped_by'] = ('signal' if stop.is_set() else
                                     'terminal' if latest('mission') in ('COMPLETE', 'ABORT')
                                     else 'duration')
        meta['final_mission_state'] = latest('mission')
    elif mode == 'spin':
        client = ActionClient(node, Spin, 'spin')
        meta['spin_server'] = client.wait_for_server(timeout_sec=20.0)
        goal = Spin.Goal()
        # Exactly what mission_executive._send_spin sets, and nothing else.
        goal.target_yaw = 2.0 * math.pi
        t_send = now()
        future = client.send_goal_async(goal)
        wait_for(future.done, 20.0)
        handle = future.result() if future.done() else None
        meta['spin_accepted'] = bool(handle and handle.accepted)
        status = None
        if meta['spin_accepted']:
            result = handle.get_result_async()
            wait_for(result.done, SPIN_BUDGET_S)
            status = result.result().status if result.done() else None
        t_done = now()
        meta['spin_status'] = status          # 4 SUCCEEDED, 5 CANCELED, 6 ABORTED
        meta['spin_duration_sim_s'] = round(t_done - t_send, 3)
        time.sleep(2.0)
        # Total rotation from ground truth over the goal, unwrapped.
        with lock:
            yaws = [(t, v[2]) for t, v in series['gt'] if t_send <= t <= t_done]
        turned = sum(wrap(b[1] - a[1]) for a, b in zip(yaws, yaws[1:]))
        meta['spin_turned_rad'] = round(turned, 4)
    else:
        # 1. Face the west wall, through the chain like any other command.
        turned = publish_for(0.0, RAW_TURN, 30.0,
                             until=lambda: abs(wrap(latest('gt')[2] - WEST)) < TURN_TOL)
        publish_for(0.0, 0.0, 2.0)
        meta['faced_west'] = turned
        meta['yaw_after_turn'] = round(latest('gt')[2], 4)
        t_drive = now()
        meta['t_drive_rel'] = round(t_drive - t0, 3)
        hold = {'since': None}

        def stop_held():
            cm, wheel = latest('cm'), latest('wheel')
            if cm is None or wheel is None:
                return False
            if cm[0] == 1 and abs(wheel[0]) <= STILL:
                hold['since'] = hold['since'] if hold['since'] is not None else now()
                return now() - hold['since'] >= STOP_HOLD_S
            hold['since'] = None
            return False
        # 2. Raw forward, held, until STOP has held the wheels for STOP_HOLD_S.
        meta['stop_held'] = publish_for(RAW_DRIVE, 0.0, DRIVE_BUDGET_S, until=stop_held)
        t_end_drive = now()
        meta['drive_sim_s'] = round(t_end_drive - t_drive, 3)
        gt1 = latest('gt')
        meta['end_pose'] = [round(v, 4) for v in gt1[:3]]
        meta['wall_face_x'] = -3.90   # wall_west: pose x -4.0, box 0.2 thick
        # 3. Release.
        publish_for(0.0, 0.0, 2.0)

    t1 = now()
    executor.shutdown()
    with lock:
        frozen = {k: list(v) for k, v in series.items()}
    rows = build_trace(frozen, t0, t1)
    events = build_events(frozen)
    meta['messages'] = {k: len(v) for k, v in frozen.items()}
    write_csv(os.path.join(out_dir, 'trace.csv'), rows)
    write_csv(os.path.join(out_dir, 'events.csv'), events)
    with open(os.path.join(out_dir, 'meta.json'), 'w') as f:
        json.dump(meta, f, indent=1)
    summary = summarise_rows(rows, events, meta)
    with open(os.path.join(out_dir, 'summary.json'), 'w') as f:
        json.dump(summary, f, indent=1)
    print(json.dumps(summary, indent=1))
    node.destroy_node()
    rclpy.shutdown()
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='command', required=True)
    for name in ('spin', 'stop', 'record'):
        p = sub.add_parser(name)
        p.add_argument('--out', required=True)
        if name == 'record':
            p.add_argument('--duration', type=float, default=1800.0)
            p.add_argument('--until-terminal', action='store_true')
    s = sub.add_parser('summarise')
    s.add_argument('dir')
    args = ap.parse_args(argv)
    if args.command == 'summarise':
        return mode_summarise(args.dir)
    return run_live(args.command, args.out,
                    duration=getattr(args, 'duration', 0.0),
                    until_terminal=getattr(args, 'until_terminal', False))


if __name__ == '__main__':
    sys.exit(main())
