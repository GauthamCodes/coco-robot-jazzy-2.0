#!/usr/bin/env python3
"""Nav2 on a live COCO graph, measured: lifecycle, plan, command chain,
wheels, odometry, recoveries.

usage: nav_smoke.py [--reach 1.0] [--timeout 240]   -> JSON on stdout

Runs inside a container where the appliance is up (platform_smoke.sh
copies it in). It changes nothing it does not restore, except the robot's
position and the arbiter's mode:

  1. lifecycle state of every Nav2 node navigation_world_run.sh checks;
  2. arbiter to `nav` (/mission/mode) -- Nav2's commands reach the wheels
     only in that mode;
  3. a REACHABLE goal --reach metres ahead of the current AMCL pose (in
     the map frame, so no world/map offset is assumed), counting messages
     and non-zero commands on every link of the command chain, /plan, and
     the odometry displacement;
  4. an UNREACHABLE goal (40 m beyond the map edge), recording what Nav2
     does: its own feedback's number_of_recoveries and the final status;
  5. arbiter back to `idle`.
"""
import argparse
import json
import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
import tf2_ros

NAV_NODES = ['/bt_navigator', '/controller_server', '/planner_server',
             '/amcl', '/map_server', '/local_costmap/local_costmap',
             '/global_costmap/global_costmap', '/velocity_smoother',
             '/collision_monitor', '/behavior_server']
CHAIN = ['/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel', '/cmd_vel_gated',
         '/diff_drive_controller/cmd_vel']
STATUS = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
          GoalStatus.STATUS_ABORTED: 'ABORTED',
          GoalStatus.STATUS_CANCELED: 'CANCELED'}


def twist_of(msg):
    return msg.twist if hasattr(msg, 'twist') else msg


class Probe(Node):
    def __init__(self):
        super().__init__('coco_nav_smoke')
        self.counts, self.nonzero = {}, {}
        self.plan_msgs, self.plan_len = 0, 0
        self.odom, self.amcl, self.arb = None, None, ''
        self.recoveries, self.distance_remaining = 0, None
        self.mode_pub = self.create_publisher(String, '/mission/mode', 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def wire(self):
        types = dict(self.get_topic_names_and_types())
        for t in CHAIN:
            if t in types:
                self.counts[t] = self.nonzero[t] = 0
                self.create_subscription(get_message(types[t][0]), t,
                                         lambda m, t=t: self._cmd(t, m),
                                         qos_profile_sensor_data)
        if '/plan' in types:
            self.create_subscription(get_message(types['/plan'][0]), '/plan',
                                     self._plan, qos_profile_sensor_data)
        self.create_subscription(get_message(types['/diff_drive_controller/odom'][0]),
                                 '/diff_drive_controller/odom', self._odom,
                                 qos_profile_sensor_data)
        # The pose comes from TF (map -> base_footprint), not /amcl_pose:
        # AMCL publishes that only on filter updates, so a stationary robot
        # can go minutes without one (measured: 60 s, three runs of three).
        # /amcl_pose is kept as a secondary reading, with the
        # transient-local QoS AMCL publishes it with.
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(get_message(types['/amcl_pose'][0]), '/amcl_pose',
                                 self._amcl, latched)
        self.create_subscription(String, '/cmd_vel_arbiter/status',
                                 lambda m: setattr(self, 'arb', m.data), 10)
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

    def map_pose(self):
        """(x, y, yaw) of base_footprint in the map frame, from TF; or None."""
        try:
            t = self.tf_buf.lookup_transform('map', 'base_footprint',
                                             rclpy.time.Time()).transform
        except Exception:  # noqa: BLE001
            return None
        yaw = 2 * math.atan2(t.rotation.z, t.rotation.w)
        return (t.translation.x, t.translation.y, yaw)

    def _cmd(self, t, m):
        self.counts[t] += 1
        tw = twist_of(m)
        if abs(tw.linear.x) > 1e-4 or abs(tw.angular.z) > 1e-4:
            self.nonzero[t] += 1

    def _plan(self, m):
        self.plan_msgs += 1
        self.plan_len = len(m.poses)

    def _odom(self, m):
        self.odom = (m.pose.pose.position.x, m.pose.pose.position.y)

    def _amcl(self, m):
        p = m.pose.pose
        yaw = 2 * math.atan2(p.orientation.z, p.orientation.w)
        self.amcl = (p.position.x, p.position.y, yaw)

    def spin_for(self, s, until=None):
        end = time.monotonic() + s
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if until and until():
                return True
        return False

    def lifecycle(self):
        out = {}
        for n in NAV_NODES:
            cli = self.create_client(GetState, f'{n}/get_state')
            if not cli.wait_for_service(timeout_sec=5.0):
                out[n] = 'no service'
                continue
            fut = cli.call_async(GetState.Request())
            self.spin_for(5.0, lambda: fut.done())
            out[n] = fut.result().current_state.label if fut.done() else 'timeout'
        return out

    def set_mode(self, mode):
        for _ in range(6):
            self.mode_pub.publish(String(data=mode))
            self.spin_for(0.25)

    def goal(self, x, y, yaw, timeout):
        self.recoveries, self.distance_remaining = 0, None
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = 'map'
        g.pose.pose.position.x, g.pose.pose.position.y = x, y
        g.pose.pose.orientation.z = math.sin(yaw / 2)
        g.pose.pose.orientation.w = math.cos(yaw / 2)

        def fb(msg):
            self.recoveries = max(self.recoveries, msg.feedback.number_of_recoveries)
            self.distance_remaining = round(msg.feedback.distance_remaining, 3)
        send = self.nav.send_goal_async(g, feedback_callback=fb)
        self.spin_for(10.0, lambda: send.done())
        if not send.done() or not send.result().accepted:
            return {'accepted': False}
        res = send.result().get_result_async()
        t0 = time.monotonic()
        finished = self.spin_for(timeout, lambda: res.done())
        out = {'accepted': True, 'wall_s': round(time.monotonic() - t0, 1),
               'finished': finished,
               'status': STATUS.get(res.result().status, res.result().status)
               if finished else 'TIMEOUT (not finished)',
               'error_code': getattr(res.result().result, 'error_code', None)
               if finished else None,
               'number_of_recoveries': self.recoveries,
               'last_distance_remaining': self.distance_remaining}
        if not finished:
            send.result().cancel_goal_async()
            self.spin_for(3.0)
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--reach', type=float, default=1.0)
    ap.add_argument('--timeout', type=float, default=240.0)
    a = ap.parse_args()
    rclpy.init()
    p = Probe()
    p.spin_for(4.0)
    p.wire()
    rep = {'lifecycle': p.lifecycle()}
    rep['action_server'] = p.nav.wait_for_server(timeout_sec=30.0)
    p.spin_for(60.0, lambda: p.map_pose() is not None and p.odom is not None)
    start = p.map_pose()
    rep['map_pose_before_tf'] = start
    rep['amcl_pose_latched'] = p.amcl
    if start is None:
        rep['error'] = 'no map->base_footprint transform within 60 s'
        print(json.dumps(rep, indent=1))
        return
    p.set_mode('nav')
    rep['arbiter_status_after_mode'] = p.arb
    x, y, yaw = start
    odom0 = p.odom
    reach = p.goal(x + a.reach * math.cos(yaw), y + a.reach * math.sin(yaw),
                   yaw, a.timeout)
    reach['odom_displacement_m'] = round(math.dist(odom0, p.odom), 3)
    reach['map_pose_after_tf'] = p.map_pose()
    reach['amcl_pose_after'] = p.amcl
    reach['chain_messages'] = dict(p.counts)
    reach['chain_nonzero'] = dict(p.nonzero)
    reach['plan_messages'], reach['plan_poses_last'] = p.plan_msgs, p.plan_len
    rep['reachable_goal'] = reach
    for t in p.counts:
        p.counts[t] = p.nonzero[t] = 0
    p.plan_msgs = 0
    odom1 = p.odom
    unreach = p.goal(x + 40.0, y + 40.0, 0.0, a.timeout)
    unreach['odom_displacement_m'] = round(math.dist(odom1, p.odom), 3)
    unreach['chain_nonzero'] = dict(p.nonzero)
    rep['unreachable_goal'] = unreach
    p.set_mode('idle')
    rep['arbiter_status_end'] = p.arb
    print(json.dumps(rep, indent=1))
    p.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
