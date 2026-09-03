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
"""C2-NAV.6 PolygonStop probe: how many laser returns are actually inside
the stop circle when the collision monitor holds STOP.

C2-NAV.5 measured the *consequence* -- `PolygonStop` held 70.38 s of
77.16 s while DWB commanded 0.2684 m/s and the wheels received 0.0142 --
and named `min_points: 4` as the likely trigger without varying it. That
is a hypothesis about a COUNT, and no run so far has recorded the count.
This records it.

It is not a re-implementation by description. `nav2_collision_monitor`
decides in three places and this copies all three verbatim:

  scan.cpp        Scan::getData()
                    if (r >= range_min && r <= range_max)
                      p_s = (r cos a, r sin a);  p_b = tf * p_s
                  -- note there is no isfinite() test and no NaN test;
                     the comparison rejects both, and a range BELOW
                     range_min (0.15 m here) is DROPPED, not clamped.
  circle.cpp      Circle::getPointsInside()
                    p.x*p.x + p.y*p.y < radius_squared_
                  -- STRICT, and the circle is centred on the ORIGIN of
                     base_frame_id, not on the lidar.
  collision_monitor_node.cpp
                    if (getPointsInside(...) >= getMinPoints())
                  -- so min_points: 4 means "four or more".

Two geometric facts follow from the URDF and matter for reading the
numbers. The lidar sits at base_link (-0.09, +0.10), i.e. 0.13454 m from
the base_footprint origin the circle is centred on, so a return can be
inside a 0.25 m circle while lying anywhere from 0.1155 m to 0.3845 m
from the sensor. And the scan is 480 samples over 240 deg, an increment
of 0.008745 rad, so at 0.25 m adjacent beams are 2.2 mm apart -- the
density the count has to be read against.

`base_shift_correction: true` means the monitor transforms from
lidar_link at the scan stamp to base_footprint at the CURRENT time
through odom. This probe uses the static lidar_link -> base_footprint
transform, so the two agree to the distance the robot moved during one
scan latency. At the 0.0142 m/s the stall runs at that is under 2 mm; the
recorded `v_wheel` column is what lets a reader check that for themselves
rather than take it on trust.

Changes nothing: subscribe-only, publishes no topic, sends no goal. It
rides alongside `nav_bench.py`, which does the driving.

Usage:
  python3 c2nav6_stopprobe.py <out_prefix> [duration_s]
writes  <out_prefix>_stop.csv    one row per /scan
        <out_prefix>_stop.json   summary, incl. the min_points threshold
                                 curve over the frames the monitor
                                 actually held STOP
reads   <out_prefix>.done        touch it to end the recording early;
                                 duration_s is only the backstop
exit    0 ok / 1 not alive / 2 no rows / 3 the monitor or the wheels were
        never seen, which makes the recording unusable rather than quiet
"""
import csv
import json
import math
import os
import statistics
import sys
import threading
import time
from collections import Counter

from geometry_msgs.msg import TwistStamped

from nav2_msgs.msg import CollisionMonitorState

from nav_msgs.msg import Odometry, Path

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan

from tf2_ros import Buffer, TransformListener

# nav2_params.yaml collision_monitor. Recorded here so the summary can
# state what it classified against instead of implying it.
STOP_RADIUS = 0.25
SLOW_HALF = 0.40
LIMIT_HALF = 0.55
BASE_FRAME = 'base_footprint'

# Map frame is the spawn; world = map - (2, 0). The two legs this probe
# exists for, as their MAP-frame goals, so a row can be labelled by which
# leg was driving without the probe having to talk to the action server.
LEG_GOALS = {'enclosure_entry': (-1.45, 2.95),
             'enclosure_exit': (0.00, 0.00)}

ACTION_NAMES = {0: 'DO_NOTHING', 1: 'STOP', 2: 'SLOWDOWN',
                3: 'APPROACH', 4: 'LIMIT'}

# The candidate ladder the summary reports a suppression curve over. Not
# a sweep to run -- a curve to read, so the single value that IS run can
# be chosen from measurement rather than from taste.
LADDER = [4, 5, 6, 8, 10, 12, 15, 20, 25, 30, 40, 50, 75, 100, 150, 200,
          300, 400]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class StopProbe(Node):

    def __init__(self):
        super().__init__('c2nav6_stopprobe')
        self.lock = threading.Lock()
        self.rows = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_lidar = None          # (tx, ty, cos, sin) base <- lidar
        self.tf_frame = None
        self.state = None             # (action_type, polygon_name)
        self.gt = None                # (x, y, yaw) world, ground truth
        self.goal = None              # (x, y) map, tail of /plan
        self.v_nav = None
        self.v_smooth = None
        self.v_out = None
        self.v_wheel = None
        self.n_scan = 0
        self.n_state = 0
        self.n_gt = 0
        self.t0 = time.time()

        self.create_subscription(
            LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(
            CollisionMonitorState, '/collision_monitor_state',
            self._state_cb, 10)
        self.create_subscription(
            Odometry, '/model/coco/odometry', self._gt_cb,
            qos_profile_sensor_data)
        self.create_subscription(Path, '/plan', self._plan_cb, 10)
        for topic, attr in (('/cmd_vel_nav', 'v_nav'),
                            ('/cmd_vel_smoothed', 'v_smooth'),
                            ('/cmd_vel', 'v_out'),
                            ('/diff_drive_controller/cmd_vel', 'v_wheel')):
            self.create_subscription(
                TwistStamped, topic, self._make_vel_cb(attr), 10)

    # ---- callbacks -------------------------------------------------
    def _make_vel_cb(self, attr):
        def cb(msg):
            with self.lock:
                setattr(self, attr,
                        (msg.twist.linear.x, msg.twist.angular.z))
        return cb

    def _state_cb(self, msg):
        with self.lock:
            self.state = (int(msg.action_type), msg.polygon_name)
            self.n_state += 1

    def _gt_cb(self, msg):
        p = msg.pose.pose
        with self.lock:
            self.gt = (p.position.x, p.position.y, yaw_of(p.orientation))
            self.n_gt += 1

    def _plan_cb(self, msg):
        if not msg.poses:
            return
        p = msg.poses[-1].pose.position
        with self.lock:
            self.goal = (p.x, p.y)

    def _lookup(self, src):
        """base_footprint <- src, as (tx, ty, cos, sin).

        Static, so it is looked up once and cached; a failure is
        reported, never guessed at from the URDF.
        """
        if self.tf_lidar is not None:
            return self.tf_lidar
        try:
            tr = self.tf_buffer.lookup_transform(
                BASE_FRAME, src, rclpy.time.Time())
        except Exception:                                  # noqa: BLE001
            return None
        t = tr.transform.translation
        y = yaw_of(tr.transform.rotation)
        self.tf_lidar = (t.x, t.y, math.cos(y), math.sin(y))
        self.tf_frame = src
        return self.tf_lidar

    def _scan_cb(self, msg):
        tf = self._lookup(msg.header.frame_id)
        if tf is None:
            return
        tx, ty, ca, sa = tf
        rmin, rmax = msg.range_min, msg.range_max
        a = msg.angle_min
        inc = msg.angle_increment
        r2_stop = STOP_RADIUS * STOP_RADIUS
        n_in = n_slow = n_lim = 0
        n_valid = 0
        d_min = float('inf')      # nearest return, from the BASE origin
        r_min = float('inf')      # nearest return, from the LIDAR
        for r in msg.ranges:
            # scan.cpp's own test. NaN fails both comparisons, inf fails
            # the upper one; a return below range_min is dropped.
            if rmin <= r <= rmax:
                n_valid += 1
                xs = r * math.cos(a)
                ys = r * math.sin(a)
                xb = tx + ca * xs - sa * ys
                yb = ty + sa * xs + ca * ys
                d2 = xb * xb + yb * yb
                if d2 < r2_stop:                    # circle.cpp, STRICT
                    n_in += 1
                if abs(xb) < SLOW_HALF and abs(yb) < SLOW_HALF:
                    n_slow += 1
                if abs(xb) < LIMIT_HALF and abs(yb) < LIMIT_HALF:
                    n_lim += 1
                d = math.sqrt(d2)
                if d < d_min:
                    d_min = d
                if r < r_min:
                    r_min = r
            a += inc
        with self.lock:
            st = self.state
            gt = self.gt
            goal = self.goal
            row = {
                't_s': round(time.time() - self.t0, 3),
                'stamp': round(msg.header.stamp.sec
                               + msg.header.stamp.nanosec * 1e-9, 3),
                'gt_x': None if gt is None else round(gt[0], 4),
                'gt_y': None if gt is None else round(gt[1], 4),
                'gt_yaw': None if gt is None else round(gt[2], 4),
                'goal_map_x': None if goal is None else round(goal[0], 3),
                'goal_map_y': None if goal is None else round(goal[1], 3),
                'n_valid': n_valid,
                'n_in_stop': n_in,
                'n_in_slow': n_slow,
                'n_in_limit': n_lim,
                'd_min_base_m': (None if d_min == float('inf')
                                 else round(d_min, 4)),
                'r_min_lidar_m': (None if r_min == float('inf')
                                  else round(r_min, 4)),
                'monitor_action': (None if st is None
                                   else ACTION_NAMES.get(st[0], str(st[0]))),
                'monitor_polygon': None if st is None else st[1],
                'v_nav': (None if self.v_nav is None
                          else round(self.v_nav[0], 4)),
                'w_nav': (None if self.v_nav is None
                          else round(self.v_nav[1], 4)),
                'v_smoothed': (None if self.v_smooth is None
                               else round(self.v_smooth[0], 4)),
                'v_out': (None if self.v_out is None
                          else round(self.v_out[0], 4)),
                'v_wheel': (None if self.v_wheel is None
                            else round(self.v_wheel[0], 4)),
            }
            self.rows.append(row)
            self.n_scan += 1


def leg_of(row):
    """Which tour leg this row belongs to, from the tail of /plan."""
    if row['goal_map_x'] is None:
        return None
    for name, (gx, gy) in LEG_GOALS.items():
        if math.dist((row['goal_map_x'], row['goal_map_y']), (gx, gy)) < 0.3:
            return name
    return 'other'


def summarise(rows):
    out = {'n_rows': len(rows)}
    legs = {}
    for r in rows:
        legs.setdefault(leg_of(r), []).append(r)
    out['legs'] = {}
    for name, rs in legs.items():
        stop = [r for r in rs if r['monitor_action'] == 'STOP']
        counts = [r['n_in_stop'] for r in stop]
        acts = Counter(r['monitor_action'] for r in rs)
        entry = {
            'n_rows': len(rs),
            't_span_s': (round(rs[-1]['t_s'] - rs[0]['t_s'], 2)
                         if rs else 0.0),
            'monitor_actions': dict(acts),
            'stop_frac': round(len(stop) / len(rs), 4) if rs else None,
            'n_stop_rows': len(stop),
        }
        if counts:
            entry['n_in_stop_when_STOP'] = {
                'min': min(counts), 'median': statistics.median(counts),
                'mean': round(statistics.fmean(counts), 2),
                'max': max(counts)}
            # The whole point: for each candidate min_points, how many of
            # the frames that DID hold STOP would still hold it. A value
            # is diagnostic only where this reaches 0.
            entry['suppression_curve'] = {
                str(k): sum(1 for c in counts if c >= k) for k in LADDER}
        for key in ('n_in_stop', 'd_min_base_m', 'r_min_lidar_m',
                    'v_nav', 'v_out', 'v_wheel'):
            vals = [r[key] for r in rs if r[key] is not None]
            if vals:
                entry[f'{key}_median'] = round(statistics.median(vals), 4)
                entry[f'{key}_min'] = round(min(vals), 4)
                entry[f'{key}_max'] = round(max(vals), 4)
        pts = [(r['gt_x'], r['gt_y']) for r in rs if r['gt_x'] is not None]
        if len(pts) > 1:
            entry['gt_start'] = list(pts[0])
            entry['gt_end'] = list(pts[-1])
            entry['gt_path_len_m'] = round(
                sum(math.dist(pts[i], pts[i + 1])
                    for i in range(len(pts) - 1)), 4)
            entry['gt_net_displacement_m'] = round(
                math.dist(pts[0], pts[-1]), 4)
        out['legs'][str(name)] = entry
    return out


def main():
    prefix = sys.argv[1] if len(sys.argv) > 1 else '/tmp/c2nav6'
    budget = float(sys.argv[2]) if len(sys.argv) > 2 else 400.0
    os.makedirs(os.path.dirname(os.path.abspath(prefix)), exist_ok=True)
    rclpy.init()
    node = StopProbe()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    # Positive control, and it is not optional here. This probe's central
    # claim is a COUNT, and "the count was low" and "the probe never saw
    # the scan, the monitor or the wheels" produce the same quiet CSV.
    # The trap list says it in one line: any check whose success
    # condition is "we saw nothing" must first prove it can see
    # something.
    #
    # It is in TWO parts, because the four streams are not available at
    # the same time. `/scan`, ground truth and the TF are live as soon as
    # the simulator is, so the probe refuses to record until it has all
    # three. `/collision_monitor_state` is NOT: the monitor publishes its
    # state from `cmdVelInCallback`, so with no goal running there is no
    # `/cmd_vel_smoothed`, no callback and no state at all -- measured on
    # a live stack, silent for 60 s while `/scan` published throughout.
    # Gating startup on it deadlocks the run that would have produced it.
    # So the monitor half of the control is asserted at the END instead,
    # over the recording, and a run that never saw a state message is
    # rejected there rather than reported as "the monitor never fired".
    snap = (0, 0, 0, None)
    t_end = time.time() + 60.0
    while time.time() < t_end:
        with node.lock:
            ok = (node.n_scan > 0 and node.n_gt > 0
                  and node.tf_lidar is not None)
            snap = (node.n_scan, node.n_state, node.n_gt, node.tf_lidar)
        if ok:
            print(f'[stopprobe] alive: scans={snap[0]} monitor={snap[1]} '
                  f'gt={snap[2]} tf {BASE_FRAME}<-{node.tf_frame}='
                  f'({snap[3][0]:.5f}, {snap[3][1]:.5f}) '
                  f'|d|={math.hypot(snap[3][0], snap[3][1]):.5f} m '
                  '(monitor state is asserted at the end, not here)',
                  flush=True)
            break
        time.sleep(0.5)
    else:
        print(f'[stopprobe] INSTRUMENT NOT ALIVE: scans={snap[0]} '
              f'gt={snap[2]} tf={snap[3]}')
        return 1

    # Reset the clock so t=0 is the first row a live instrument produced.
    with node.lock:
        node.rows.clear()
        node.t0 = time.time()
    # The probe rides alongside nav_bench.py and has to outlive it, but
    # not by a minute of dead air: the run script touches <prefix>.done
    # when the bench exits, and the budget is only the backstop for a
    # bench that died without touching it.
    done = f'{prefix}.done'
    deadline = time.time() + budget
    last = 0
    while time.time() < deadline and not os.path.exists(done):
        time.sleep(5.0)
        with node.lock:
            n = len(node.rows)
            tail = node.rows[-1] if node.rows else None
        if tail is not None:
            print(f'[stopprobe] t={tail["t_s"]:.0f}s rows={n} '
                  f'({n - last} in 5 s) n_in_stop={tail["n_in_stop"]} '
                  f'monitor={tail["monitor_action"]} '
                  f'v_nav={tail["v_nav"]} v_wheel={tail["v_wheel"]}',
                  flush=True)
        last = n

    with node.lock:
        rows = list(node.rows)
        n_state = node.n_state
        n_wheel_seen = sum(1 for r in rows if r['v_wheel'] is not None)
    # The second half of the positive control. A recording in which the
    # monitor never spoke, or the wheels never did, cannot support any
    # claim about what the monitor did to the wheels -- and it looks
    # exactly like a monitor that stayed at DO_NOTHING. Say so in the
    # artifact and exit non-zero rather than let it be read as a result.
    control = {'collision_monitor_state_msgs': n_state,
               'rows_with_wheel_cmd': n_wheel_seen,
               'ok': bool(n_state > 0 and n_wheel_seen > 0)}
    if not control['ok']:
        print('[stopprobe] CONTROL FAILED: '
              f'collision_monitor_state msgs={n_state}, '
              f'rows with a wheel command={n_wheel_seen}. '
              'This recording cannot support a claim about the monitor.')
    summary = summarise(rows)
    summary['positive_control'] = control
    summary['stop_radius_m'] = STOP_RADIUS
    summary['base_frame'] = BASE_FRAME
    summary['lidar_frame'] = node.tf_frame
    summary['tf_base_from_lidar'] = [round(node.tf_lidar[0], 6),
                                     round(node.tf_lidar[1], 6)]
    summary['lidar_offset_from_base_origin_m'] = round(
        math.hypot(node.tf_lidar[0], node.tf_lidar[1]), 6)
    summary['ladder'] = LADDER
    with open(f'{prefix}_stop.json', 'w') as f:
        json.dump(summary, f, indent=1)
    keys = list(rows[0].keys()) if rows else []
    with open(f'{prefix}_stop.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    print(f'[stopprobe] wrote {prefix}_stop.json and {prefix}_stop.csv '
          f'({len(rows)} rows, {n_state} monitor states, '
          f'{n_wheel_seen} rows with a wheel command)')
    ex.shutdown()
    if not rows:
        return 2
    return 0 if control['ok'] else 3


if __name__ == '__main__':
    sys.exit(main())
