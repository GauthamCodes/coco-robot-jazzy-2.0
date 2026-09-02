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
"""C2-NAV.3 capture: one fresh enclosure-entry stall, with everything the
four MapGrid critics are computed FROM.

C2-NAV.2 established that at the stall BaseObstacle is 0.00, forward
trajectories are legal and scored to completion, and they still lose to
zero velocity on PathAlign / GoalAlign / GoalDist / PathDist. It could not
say why, because it recorded the critic SCORES and not the grids those
scores are read out of.

This records the inputs as well:

  /evaluation                 every trajectory DWB scored, its critic
                              breakdown, and -- for a probe subset -- the
                              trajectory POSES that were scored
  /transformed_global_plan    the plan the critics actually receive,
                              already pruned and already in the local
                              costmap's frame. Published by DWB itself
                              (publish_transformed_plan defaults true), so
                              nothing has to be switched on to get it
  /local_costmap/costmap_raw  the raw uint8 costs of the same costmap the
                              MapGrid critics index into
  /plan                       the global plan, map frame, for reference
  TF map->odom, odom->base_footprint

Given those four, docs/data/c2nav3_mapgrid.py rebuilds each critic's
MapGrid from the Nav2 source and checks the rebuilt raw scores against the
raw scores in this capture. The rebuild is only believable if it matches.

Changes nothing. Subscribe-only, plus the NavigateToPose goal that makes
the controller run at all. No parameter is set, no plugin is swapped.

Usage:
  python3 c2nav3_capture.py <out_prefix>
writes  <out_prefix>_stall.json   the stall snapshot
        <out_prefix>_timeline.csv the whole approach, one row per cycle
"""
import csv
import json
import math
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from dwb_msgs.msg import LocalPlanEvaluation
from geometry_msgs.msg import TwistStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry, Path
import tf2_ros

# The enclosure_entry goal, copied from nav_bench.py's TOUR. The TOUR is in
# WORLD coordinates; the map origin sits +2.0 m in x from the world's, and
# nav_bench adds that offset before sending the action goal.
GOAL_WORLD = (-3.45, 2.95)
WORLD_TO_MAP_X = 2.0
WORLD_TO_MAP_Y = 0.0
GOAL_MAP = (GOAL_WORLD[0] + WORLD_TO_MAP_X, GOAL_WORLD[1] + WORLD_TO_MAP_Y)

# A stall is /cmd_vel_nav at rest for this long while still this far out.
# The 2.0 m ceiling keeps this on the ENCLOSURE stall: without it the few
# seconds of zero before the robot first accelerates at the spawn is
# caught instead, 3.3 m out, which is a startup pause and not the stall.
STALL_ZERO_S = 10.0
STALL_DIST_LO = 0.5
STALL_DIST_HI = 2.0

# Trajectory poses are only kept for a probe subset, or the snapshot is
# ~800 trajectories x ~60 poses. These are the (vx, wz) the probe asks
# for; the nearest ACTUALLY EVALUATED sample to each is what gets kept,
# so every row of the probe table is a trajectory DWB really scored.
PROBE_REQUESTS = [
    ('A_zero', 0.000, 0.00),
    ('B_small', 0.079, 0.00),
    ('C_medium', 0.158, 0.00),
    ('D_large', 0.300, 0.00),
    ('E_fwd_left', 0.158, 0.20),
    ('F_fwd_right', 0.158, -0.20),
    ('G_reverse', -0.100, 0.00),
    ('H_spin_left', 0.000, 0.50),
    ('I_spin_right', 0.000, -0.50),
]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def ang_norm(a):
    return math.atan2(math.sin(a), math.cos(a))


def critic_list(ts):
    """[(name, raw_score, scale)] exactly as the message carries it."""
    return [(cs.name, float(cs.raw_score), float(cs.scale))
            for cs in ts.scores]


def score_dict(ts):
    return {cs.name: float(cs.raw_score) * float(cs.scale) for cs in ts.scores}


class Capture(Node):

    def __init__(self):
        super().__init__('c2n3_capture',
                         parameter_overrides=[
                             Parameter('use_sim_time', value=True)])
        self.lock = threading.Lock()
        self.eval_msgs = []      # (wall_t, LocalPlanEvaluation)
        self.costmap = None      # (wall_t, Costmap)
        self.tplan = None        # (wall_t, Path in odom)
        self.plan = None         # (wall_t, Path in map)
        self.gt = None           # (x, y, yaw, vx, wz)
        self.v_nav = None
        self.t_zero_since = None
        self.rows = []           # the timeline

        be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)
        tl = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        history=HistoryPolicy.KEEP_LAST)

        # One callback group PER subscription. With the node's single
        # default MutuallyExclusive group, every callback serialises even
        # under a MultiThreadedExecutor -- and /evaluation is enormous:
        # 819 trajectories x up to 60 Pose2D each, deserialised in Python
        # at 10 Hz. Measured: with a shared group, /model/coco/odometry
        # and /cmd_vel_nav went 51 s without a single callback while the
        # robot actually drove 2.6 m, so the recorded pose froze at the
        # spawn and the stall detector never saw the robot arrive. A
        # starved subscription and a silent topic look identical from the
        # inside; this is the same trap as "any check whose success
        # condition is 'we saw nothing' must first prove it can see
        # something".
        def grp():
            return MutuallyExclusiveCallbackGroup()

        self.create_subscription(LocalPlanEvaluation, '/evaluation',
                                 self.on_eval, 10, callback_group=grp())
        self.create_subscription(Path, '/transformed_global_plan',
                                 self.on_tplan, 10, callback_group=grp())
        self.create_subscription(Path, '/plan', self.on_plan, 10,
                                 callback_group=grp())
        self.create_subscription(Costmap, '/local_costmap/costmap_raw',
                                 self.on_costmap, tl, callback_group=grp())
        self.create_subscription(Odometry, '/model/coco/odometry',
                                 self.on_gt, be, callback_group=grp())
        self.create_subscription(TwistStamped, '/cmd_vel_nav',
                                 self.on_nav, 10, callback_group=grp())

        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    # -- subscriptions --------------------------------------------------
    def on_gt(self, m):
        p, o = m.pose.pose.position, m.pose.pose.orientation
        t = m.twist.twist
        with self.lock:
            self.gt = (p.x, p.y, yaw_of(o), t.linear.x, t.angular.z)

    def on_nav(self, m):
        v = m.twist.linear.x
        now = time.time()
        with self.lock:
            self.v_nav = (v, m.twist.angular.z)
            if abs(v) < 0.02:
                if self.t_zero_since is None:
                    self.t_zero_since = now
            else:
                self.t_zero_since = None

    def on_tplan(self, m):
        with self.lock:
            self.tplan = (time.time(), m)

    def on_plan(self, m):
        with self.lock:
            self.plan = (time.time(), m)

    def on_costmap(self, m):
        with self.lock:
            self.costmap = (time.time(), m)

    def on_eval(self, m):
        now = time.time()
        with self.lock:
            self.eval_msgs.append((now, m))
            if len(self.eval_msgs) > 40:
                self.eval_msgs.pop(0)
            gt = self.gt
            zsince = self.t_zero_since
        # A one-line summary per control cycle, cheap enough to keep for
        # the whole approach.
        if not (0 <= m.best_index < len(m.twists)):
            return
        best = m.twists[m.best_index]
        legal = [t for t in m.twists if t.total >= 0.0]
        row = {
            't': round(now - self.t0, 3),
            'best_vx': round(float(best.traj.velocity.x), 4),
            'best_wz': round(float(best.traj.velocity.theta), 4),
            'best_total': round(float(best.total), 3),
            'n_traj': len(m.twists),
            'n_legal': len(legal),
            'n_crit_best': len(best.scores),
            'zero_for': (round(now - zsince, 2) if zsince else 0.0),
        }
        for name, raw, scale in critic_list(best):
            row[f'{name}_raw'] = round(raw, 4)
            row[f'{name}'] = round(raw * scale, 4)
        if gt:
            row['gt_x'], row['gt_y'], row['gt_yaw'] = (round(gt[0], 4),
                                                       round(gt[1], 4),
                                                       round(gt[2], 4))
            row['gt_vx'], row['gt_wz'] = round(gt[3], 4), round(gt[4], 4)
            row['dist_goal'] = round(math.dist(gt[:2], GOAL_WORLD), 4)
        with self.lock:
            self.rows.append(row)

    # -- helpers --------------------------------------------------------
    def tf_now(self, target, source):
        try:
            tr = self.tf_buf.lookup_transform(
                target, source, rclpy.time.Time(),
                timeout=Duration(seconds=0.5))
            t, q = tr.transform.translation, tr.transform.rotation
            return {'x': t.x, 'y': t.y, 'z': t.z,
                    'yaw': yaw_of(q),
                    'q': [q.x, q.y, q.z, q.w]}
        except Exception as e:                             # noqa: BLE001
            return {'error': str(e)}


def pick_probes(m):
    """Nearest actually-evaluated trajectory to each PROBE_REQUEST.

    Selecting from what was evaluated, rather than asking for a velocity
    and hoping it was sampled, is what makes the probe table a
    measurement. A request the sampler cannot produce (negative vx, with
    min_vel_x 0.0) comes back with the nearest thing it CAN produce and is
    flagged, rather than silently reported as if it were the request.
    """
    out = {}
    for label, vx, wz in PROBE_REQUESTS:
        best, bestd = None, None
        for i, t in enumerate(m.twists):
            d = (float(t.traj.velocity.x) - vx) ** 2 + \
                (float(t.traj.velocity.theta) - wz) ** 2
            if bestd is None or d < bestd:
                best, bestd = i, d
        if best is None:
            continue
        t = m.twists[best]
        got_vx = float(t.traj.velocity.x)
        got_wz = float(t.traj.velocity.theta)
        out[label] = {
            'requested': [vx, wz],
            'evaluated': [round(got_vx, 6), round(got_wz, 6)],
            'exact': abs(got_vx - vx) < 1e-6 and abs(got_wz - wz) < 1e-6,
            'index': best,
            'total': float(t.total),
            'n_critics': len(t.scores),
            'critics': critic_list(t),
            'poses': [[round(p.x, 5), round(p.y, 5), round(p.theta, 5)]
                      for p in t.traj.poses],
        }
    return out


def snapshot(node, m, t_eval):
    """Everything about ONE control cycle, plus the grids it read."""
    with node.lock:
        gt = node.gt
        cm = node.costmap
        tp = node.tplan
        pl = node.plan
    legal = [t for t in m.twists if t.total >= 0.0]
    best = m.twists[m.best_index]

    snap = {
        'eval_header': {'frame_id': m.header.frame_id,
                        'stamp': m.header.stamp.sec +
                        m.header.stamp.nanosec * 1e-9},
        'n_traj': len(m.twists),
        'n_legal': len(legal),
        'best_index': int(m.best_index),
        'worst_index': int(m.worst_index),
        'chosen': {'vx': float(best.traj.velocity.x),
                   'wz': float(best.traj.velocity.theta),
                   'total': float(best.total),
                   'n_critics': len(best.scores),
                   'critics': critic_list(best),
                   'poses': [[round(p.x, 5), round(p.y, 5), round(p.theta, 5)]
                             for p in best.traj.poses]},
        # Every trajectory's velocity, total and critic breakdown. No
        # poses -- those are in `probes`, for the controlled subset.
        'all': [{'vx': round(float(t.traj.velocity.x), 6),
                 'wz': round(float(t.traj.velocity.theta), 6),
                 'total': round(float(t.total), 4),
                 'n_critics': len(t.scores),
                 'n_poses': len(t.traj.poses),
                 'critics': [[c.name, round(float(c.raw_score), 4),
                              round(float(c.scale), 6)] for c in t.scores]}
                for t in m.twists],
        'probes': pick_probes(m),
    }
    if gt:
        snap['gt'] = {'x': gt[0], 'y': gt[1], 'yaw': gt[2],
                      'vx': gt[3], 'wz': gt[4]}
        snap['dist_to_goal_world'] = math.dist(gt[:2], GOAL_WORLD)
        b = math.atan2(GOAL_WORLD[1] - gt[1], GOAL_WORLD[0] - gt[0])
        snap['bearing_to_goal_world'] = b
        snap['heading_error_to_goal_deg'] = math.degrees(ang_norm(b - gt[2]))
    snap['tf'] = {
        'map_from_odom': node.tf_now('map', 'odom'),
        'odom_from_base': node.tf_now('odom', 'base_footprint'),
        'map_from_base': node.tf_now('map', 'base_footprint'),
    }
    if cm:
        t_cm, c = cm
        md = c.metadata
        snap['costmap'] = {
            'age_s_vs_eval': round(t_cm - t_eval, 3),
            'frame_id': c.header.frame_id,
            'resolution': md.resolution,
            'size_x': int(md.size_x), 'size_y': int(md.size_y),
            'origin': [md.origin.position.x, md.origin.position.y],
            'origin_yaw': yaw_of(md.origin.orientation),
            'data': list(c.data),
        }
    if tp:
        t_tp, p = tp
        snap['transformed_plan'] = {
            'age_s_vs_eval': round(t_tp - t_eval, 3),
            'frame_id': p.header.frame_id,
            'poses': [[round(ps.pose.position.x, 6),
                       round(ps.pose.position.y, 6),
                       round(yaw_of(ps.pose.orientation), 6)]
                      for ps in p.poses],
        }
    if pl:
        t_pl, p = pl
        snap['global_plan'] = {
            'age_s_vs_eval': round(t_pl - t_eval, 3),
            'frame_id': p.header.frame_id,
            'n': len(p.poses),
            'poses': [[round(ps.pose.position.x, 6),
                       round(ps.pose.position.y, 6)] for ps in p.poses],
        }
    snap['goal_world'] = list(GOAL_WORLD)
    snap['goal_map'] = list(GOAL_MAP)
    return snap


def main():
    prefix = sys.argv[1] if len(sys.argv) > 1 else '/tmp/c2nav3'
    rclpy.init()
    node = Capture()
    node.t0 = time.time()
    ex = MultiThreadedExecutor(num_threads=8)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    print('[capture] waiting for navigate_to_pose')
    if not node.ac.wait_for_server(timeout_sec=40.0):
        print('[capture] NO ACTION SERVER')
        return 1
    g = NavigateToPose.Goal()
    g.pose.header.frame_id = 'map'
    g.pose.header.stamp = node.get_clock().now().to_msg()
    g.pose.pose.position.x = GOAL_MAP[0]
    g.pose.pose.position.y = GOAL_MAP[1]
    g.pose.pose.orientation.w = 1.0
    print(f'[capture] goal world {GOAL_WORLD} = map {GOAL_MAP}')
    node.t0 = time.time()
    fut = node.ac.send_goal_async(g)
    dl = time.time() + 20.0
    while not fut.done() and time.time() < dl:
        time.sleep(0.05)
    if not fut.done():
        print('[capture] NO ACK')
        return 1
    handle = fut.result()
    print(f'[capture] accepted={handle.accepted}')
    if not handle.accepted:
        return 1

    # Prove the instrument can SEE something before believing a quiet
    # result. A capture that reports "no stall" and a capture whose
    # /evaluation subscription never matched look identical otherwise.
    t_end = time.time() + 40.0
    while time.time() < t_end:
        with node.lock:
            n = len(node.eval_msgs)
            has_cm = node.costmap is not None
            has_tp = node.tplan is not None
        if n and has_cm and has_tp:
            print(f'[capture] alive: {n} /evaluation msgs, costmap yes, '
                  'transformed plan yes')
            break
        time.sleep(0.5)
    else:
        with node.lock:
            print('[capture] INSTRUMENT NOT ALIVE: '
                  f'eval={len(node.eval_msgs)} '
                  f'costmap={node.costmap is not None} '
                  f'tplan={node.tplan is not None}')
        return 1

    snaps = []
    deadline = time.time() + 150.0
    while time.time() < deadline:
        time.sleep(0.5)
        with node.lock:
            z = node.t_zero_since
            gt = node.gt
            n = len(node.eval_msgs)
        if gt is None or z is None or n == 0:
            continue
        stalled_for = time.time() - z
        dist = math.dist(gt[:2], GOAL_WORLD)
        if (stalled_for >= STALL_ZERO_S
                and STALL_DIST_LO < dist < STALL_DIST_HI):
            print(f'[capture] STALL: {stalled_for:.1f}s at '
                  f'({gt[0]:.3f}, {gt[1]:.3f}), {dist:.3f} m from goal')
            with node.lock:
                recent = list(node.eval_msgs[-3:])
            for (t_eval, m) in recent:
                if 0 <= m.best_index < len(m.twists):
                    s = snapshot(node, m, t_eval)
                    s['stalled_for_s'] = round(stalled_for, 2)
                    snaps.append(s)
            break

    with node.lock:
        rows = list(node.rows)
    print(f'[capture] {len(snaps)} stall snapshots, {len(rows)} timeline rows')

    with open(f'{prefix}_stall.json', 'w') as f:
        json.dump({'snapshots': snaps,
                   'probe_requests': PROBE_REQUESTS,
                   'stall_criteria': {'zero_s': STALL_ZERO_S,
                                      'dist_lo': STALL_DIST_LO,
                                      'dist_hi': STALL_DIST_HI}}, f)
    keys = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(f'{prefix}_timeline.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    print(f'[capture] wrote {prefix}_stall.json and {prefix}_timeline.csv')

    try:
        handle.cancel_goal_async()
    except Exception:                                      # noqa: BLE001
        pass
    time.sleep(2.0)
    ex.shutdown()
    return 0 if snaps else 2


if __name__ == '__main__':
    sys.exit(main())
