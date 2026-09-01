#!/usr/bin/env python3
"""C2-NAV.2 evidence probe: WHY does DWB pick zero at the enclosure stall?

nav_bench.py records the CHOSEN trajectory's critic breakdown, which is
enough to say BaseObstacle stopped dominating the score. It is not enough
to say what beat forward motion instead. This subscribes to DWB's
/evaluation directly and, at the stall, decomposes the score gap between
the chosen trajectory and the best FORWARD-MOVING legal trajectory,
critic by critic. If BaseObstacle scoring is what rejects forward motion,
BaseObstacle is the dominant term in that gap. If it is not, this says
which critic is.

Changes nothing. Subscribe-only, plus the NavigateToPose goal that makes
the controller run at all.
"""
import bisect
import json
import math
import sys
import threading
import time
from collections import Counter, defaultdict

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from dwb_msgs.msg import LocalPlanEvaluation
from geometry_msgs.msg import TwistStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry

# The enclosure_entry goal, copied from nav_bench.py's TOUR. The TOUR is
# in WORLD coordinates and nav_bench adds the world->map offset before it
# sends the action goal; the map origin sits +2.0 m in x from the world's.
# Sending the world number straight through aims 2 m away, off the map.
GOAL = (-3.45, 2.95)
WORLD_TO_MAP_X = 2.0
WORLD_TO_MAP_Y = 0.0
GOAL_MAP = (GOAL[0] + WORLD_TO_MAP_X, GOAL[1] + WORLD_TO_MAP_Y)


class Probe(Node):

    def __init__(self):
        super().__init__('c2n2_evalprobe',
                         parameter_overrides=[
                             Parameter('use_sim_time', value=True)])
        self.lock = threading.Lock()
        self.cycles = []          # (t, LocalPlanEvaluation) kept raw
        self.pose = None
        self.v_nav = None
        self.t_zero_since = None
        be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(LocalPlanEvaluation, '/evaluation',
                                 self.on_eval, 10)
        self.create_subscription(Odometry, '/model/coco/odometry',
                                 self.on_gt, be)
        self.create_subscription(TwistStamped, '/cmd_vel_nav',
                                 self.on_nav, 10)
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def on_gt(self, m):
        p = m.pose.pose.position
        with self.lock:
            self.pose = (p.x, p.y)

    def on_nav(self, m):
        v = m.twist.linear.x
        now = time.time()
        with self.lock:
            self.v_nav = v
            if abs(v) < 0.02:
                if self.t_zero_since is None:
                    self.t_zero_since = now
            else:
                self.t_zero_since = None

    def on_eval(self, m):
        with self.lock:
            self.cycles.append((time.time(), m))
            if len(self.cycles) > 400:
                self.cycles.pop(0)


def breakdown(ts):
    """{critic: scaled score} for one TrajectoryScore."""
    return {cs.name: float(cs.raw_score) * float(cs.scale) for cs in ts.scores}


def analyse(m):
    """One control cycle -> the comparison that matters."""
    n = len(m.twists)
    if not (0 <= m.best_index < n):
        return None
    chosen = m.twists[m.best_index]
    ch_vx = float(chosen.traj.velocity.x)
    ch = breakdown(chosen)

    legal = [t for t in m.twists if t.total >= 0.0]
    illegal = [t for t in m.twists if t.total < 0.0]
    # Which critic threw each illegal one out (first negative raw_score).
    why_illegal = Counter()
    illegal_fwd = 0
    for t in illegal:
        if float(t.traj.velocity.x) >= 0.15:
            illegal_fwd += 1
        for cs in t.scores:
            if cs.raw_score < 0.0:
                why_illegal[cs.name] += 1
                break

    # The best LEGAL trajectory that actually moves forward.
    fwd = [t for t in legal if float(t.traj.velocity.x) >= 0.15]
    best_fwd = min(fwd, key=lambda t: t.total) if fwd else None

    # Objective as a function of commanded vx: min total per vx sample.
    by_vx = defaultdict(lambda: None)
    for t in legal:
        vx = round(float(t.traj.velocity.x), 4)
        if by_vx[vx] is None or t.total < by_vx[vx].total:
            by_vx[vx] = t

    out = {
        'n_traj': n,
        'n_legal': len(legal),
        'n_illegal': len(illegal),
        'n_fwd_legal': len(fwd),
        'n_fwd_illegal': illegal_fwd,
        'why_illegal': dict(why_illegal),
        'chosen': {'vx': ch_vx, 'wz': float(chosen.traj.velocity.theta),
                   'total': float(chosen.total), 'critics': ch},
    }
    if best_fwd is not None:
        bf = breakdown(best_fwd)
        gap = {k: round(bf.get(k, 0.0) - ch.get(k, 0.0), 3)
               for k in set(bf) | set(ch)}
        out['best_fwd'] = {
            'vx': float(best_fwd.traj.velocity.x),
            'wz': float(best_fwd.traj.velocity.theta),
            'total': float(best_fwd.total),
            'critics': bf,
        }
        out['gap_fwd_minus_chosen'] = gap
        out['gap_total'] = round(float(best_fwd.total) - float(chosen.total), 3)
        # Rank the critics by how much each contributes to the gap.
        out['gap_ranked'] = sorted(gap.items(), key=lambda kv: -kv[1])
    out['by_vx'] = {
        str(vx): {'total': round(float(t.total), 3),
                  'critics': {k: round(v, 3)
                              for k, v in breakdown(t).items()}}
        for vx, t in sorted(by_vx.items())
    }
    return out


def main():
    rclpy.init()
    node = Probe()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()

    print('[probe] waiting for action server')
    if not node.ac.wait_for_server(timeout_sec=30.0):
        print('[probe] NO ACTION SERVER')
        return 1
    g = NavigateToPose.Goal()
    g.pose.header.frame_id = 'map'
    g.pose.header.stamp = node.get_clock().now().to_msg()
    g.pose.pose.position.x = GOAL_MAP[0]
    g.pose.pose.position.y = GOAL_MAP[1]
    g.pose.pose.orientation.w = 1.0
    print(f'[probe] sending goal world {GOAL} = map {GOAL_MAP}')
    fut = node.ac.send_goal_async(g)
    dl = time.time() + 20.0
    while not fut.done() and time.time() < dl:
        time.sleep(0.05)
    if not fut.done():
        print('[probe] NO ACK')
        return 1
    handle = fut.result()
    print(f'[probe] goal accepted={handle.accepted}')
    if not handle.accepted:
        return 1
    t_start = time.time()

    # Wait for a sustained stall: /cmd_vel_nav at zero for 6 s while the
    # robot is still far from the goal.
    captured = []
    deadline = t_start + 90.0
    while time.time() < deadline:
        time.sleep(0.5)
        with node.lock:
            z = node.t_zero_since
            pose = node.pose
            ncyc = len(node.cycles)
        if pose is None:
            print('[probe] no ground truth yet')
            continue
        if z is None:
            continue
        if ncyc == 0:
            print('[probe] /evaluation is silent')
            continue
        stalled_for = time.time() - z
        dist = math.dist(pose, GOAL)
        if stalled_for >= 6.0 and dist > 0.5:
            print(f'[probe] stalled {stalled_for:.1f}s at '
                  f'({pose[0]:.3f}, {pose[1]:.3f}), {dist:.3f} m to goal')
            with node.lock:
                cyc = list(node.cycles[-12:])
            for (t, m) in cyc:
                a = analyse(m)
                if a:
                    a['pose'] = [round(pose[0], 3), round(pose[1], 3)]
                    a['dist_to_goal'] = round(dist, 3)
                    captured.append(a)
            if len(captured) >= 10:
                break

    print(f'[probe] captured {len(captured)} cycles')
    out = sys.argv[1] if len(sys.argv) > 1 else '/tmp/c2n2_eval.json'
    with open(out, 'w') as f:
        json.dump(captured, f, indent=1)
    print(f'[probe] wrote {out}')

    try:
        handle.cancel_goal_async()
    except Exception:
        pass
    time.sleep(2.0)
    ex.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
