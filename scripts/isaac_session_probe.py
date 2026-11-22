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

"""ROS-side readiness, navigation diagnostics and safe stop for one isolated session."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import TwistStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry, Path as NavPath
from nav2_msgs.action import NavigateToPose, ComputePathToPose, FollowPath
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, JointState, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


class Probe(Node):
    def __init__(self, output):
        super().__init__('coco_isaac_session_probe')
        self.output = output
        self.started = time.monotonic()
        self.counts = Counter()
        self.nonzero = Counter()
        self.last = {}
        self.received = {}
        self.events = []
        self.trace = []
        self.clock_steps = []
        self.clock_gaps = []
        self.statuses = {}
        self.result = {'passed': False, 'failure_class': None}
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.handles = []
        self.goal_handle = None
        self.stage = 'startup failure'
        self.sub(Clock, '/clock', 'clock', self.clock)
        self.sub(Odometry, '/diff_drive_controller/odom', 'odom', self.odom)
        self.sub(JointState, '/joint_states', 'joints')
        self.sub(LaserScan, '/scan', 'scan')
        self.sub(Image, '/camera/image_raw', 'rgb')
        self.sub(Image, '/camera/depth/image_raw', 'depth')
        self.sub(String, '/cmd_vel_arbiter/status', 'arbiter')
        self.sub(NavPath, '/plan', 'plan', lambda msg: self.event('plan', poses=len(msg.poses)))
        self.command_topics = [
            '/cmd_vel_nav',
            '/cmd_vel_smoothed',
            '/cmd_vel',
            '/cmd_vel_gated',
            '/diff_drive_controller/cmd_vel',
        ]
        for topic in self.command_topics:
            self.sub(TwistStamped, topic, topic, lambda msg, topic=topic: self.command(topic, msg))
        qos = QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for action in ('navigate_to_pose', 'compute_path_to_pose', 'follow_path'):
            self.handles.append(
                self.create_subscription(
                    GoalStatusArray,
                    f'/{action}/_action/status',
                    lambda msg, action=action: self.status(action, msg),
                    qos,
                )
            )
        self.action_clients = {
            name: ActionClient(self, kind, '/' + name)
            for name, kind in (
                ('navigate_to_pose', NavigateToPose),
                ('compute_path_to_pose', ComputePathToPose),
                ('follow_path', FollowPath),
            )
        }
        self.mode = self.create_publisher(String, '/mission/mode', 10)
        self.zero = self.create_publisher(TwistStamped, '/cmd_vel_teleop', 10)

    def event(self, event, **fields):
        row = {
            'event': event,
            'wall_monotonic_s': time.monotonic(),
            'elapsed_wall_s': time.monotonic() - self.started,
            'time_sim_s': self.sim_time(),
            **fields,
        }
        self.events.append(row)
        with self.output.with_suffix('.jsonl').open('a') as stream:
            stream.write(json.dumps(row) + '\n')

    def sim_time(self):
        msg = self.last.get('clock')
        return msg.clock.sec + msg.clock.nanosec * 1e-9 if msg else None

    def sub(self, kind, topic, key, callback=None):
        def receive(msg):
            self.counts[key] += 1
            if callback:
                callback(msg)
            self.last[key] = msg
            self.received[key] = time.monotonic()

        self.handles.append(self.create_subscription(kind, topic, receive, 10))

    def clock(self, msg):
        now = msg.clock.sec + msg.clock.nanosec * 1e-9
        previous = self.sim_time()
        if previous is not None:
            self.clock_steps.append(now - previous)
            self.clock_gaps.append(time.monotonic() - self.received['clock'])

    def command(self, topic, msg):
        if abs(msg.twist.linear.x) + abs(msg.twist.angular.z) > 0.001:
            self.nonzero[topic] += 1

    def status(self, action, msg):
        for item in msg.status_list:
            key = action + ':' + bytes(item.goal_info.goal_id.uuid).hex()
            if self.statuses.get(key) != item.status:
                self.statuses[key] = item.status
                stamp = item.goal_info.stamp
                self.event(
                    'action_status',
                    action=action,
                    goal_id=key.split(':')[1],
                    status=item.status,
                    goal_stamp_sim_s=stamp.sec + stamp.nanosec * 1e-9,
                )

    def odom(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.trace and stamp - self.trace[-1]['time_sim_s'] < 0.1:
            return
        row = {
            'time_sim_s': stamp,
            'wall_monotonic_s': time.monotonic(),
            'xy': [msg.pose.pose.position.x, msg.pose.pose.position.y],
            'vx': msg.twist.twist.linear.x,
            'wz': msg.twist.twist.angular.z,
        }
        try:
            tf = self.buffer.lookup_transform('map', 'base_footprint', Time())
            row['map_xy'] = [tf.transform.translation.x, tf.transform.translation.y]
            row['map_tf_available'] = True
        except Exception:
            row['map_tf_available'] = False
        self.trace.append(row)

    def wait(self, predicate, timeout, stage, detail):
        self.stage = stage
        self.event('wait_begin', condition=detail)
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise RuntimeError(detail + ' timed out')
            rclpy.spin_once(self, timeout_sec=0.02)
        self.event('ready', condition=detail)

    def state_ready(self):
        self.wait(
            lambda: self.counts['clock'] >= 5 and self.sim_time() > 0,
            45,
            'clock readiness failure',
            'progressing clock',
        )
        self.wait(
            lambda: self.counts['odom'] >= 5 and self.counts['joints'] > 0,
            30,
            'startup failure',
            'robot odometry and joints',
        )
        for frame in ('base_footprint', 'lidar_link', 'camera_optical_frame'):
            self.wait(
                lambda frame=frame: self.buffer.can_transform('odom', frame, Time()),
                30,
                'TF readiness failure',
                'odom to ' + frame,
            )
        self.wait(
            lambda: all(self.counts[x] >= 2 for x in ('scan', 'rgb', 'depth')),
            45,
            'ROS discovery failure',
            'sensor messages',
        )
        if any(step <= 0 for step in self.clock_steps):
            raise RuntimeError('nonprogressing clock')
        self.result['startup_ready'] = True

    def nav_ready(self):
        for name in (
            'map_server',
            'amcl',
            'planner_server',
            'controller_server',
            'bt_navigator',
            'velocity_smoother',
            'collision_monitor',
        ):
            service = self.create_client(GetState, '/' + name + '/get_state')
            self.wait(service.service_is_ready, 45, 'Nav2 lifecycle failure', name + ' service')
            deadline = time.monotonic() + 90
            while True:
                future = service.call_async(GetState.Request())
                self.wait(future.done, 5, 'Nav2 lifecycle failure', name + ' state reply')
                state = future.result().current_state
                self.event('lifecycle', node=name, state=state.id, label=state.label)
                if state.id == 3:
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError(name + ' never active')
                rclpy.spin_once(self, timeout_sec=0.1)
        self.wait(
            lambda: self.buffer.can_transform('map', 'base_footprint', Time()),
            30,
            'TF readiness failure',
            'map to base_footprint',
        )
        for name, client in self.action_clients.items():
            self.wait(client.server_is_ready, 30, 'action server unavailable', name)
        self.result['action_server_ready'] = True

    def planner_probe(self, count):
        samples = []
        for _ in range(count):
            goal = ComputePathToPose.Goal()
            goal.goal.header.frame_id = 'map'
            goal.goal.pose.position.x, goal.goal.pose.position.y = 1.5, 0.5
            goal.goal.pose.orientation.w = 1.0
            goal.planner_id = 'GridBased'
            start, sim = time.monotonic(), self.sim_time()
            future = self.action_clients['compute_path_to_pose'].send_goal_async(goal)
            self.wait(future.done, 5, 'planner failure', 'diagnostic planner acknowledgement')
            handle = future.result()
            ack = time.monotonic() - start
            ack_sim = self.sim_time() - sim
            if not handle.accepted:
                raise RuntimeError('diagnostic planner goal rejected')
            future = handle.get_result_async()
            self.wait(future.done, 10, 'planner failure', 'diagnostic planner result')
            reply = future.result()
            row = {
                'ack_wall_s': ack,
                'ack_sim_s': ack_sim,
                'status': reply.status,
                'poses': len(reply.result.path.poses),
                'error_code': reply.result.error_code,
            }
            samples.append(row)
            self.event('planner_probe', **row)
            if reply.status != 4 or not row['poses']:
                raise RuntimeError('diagnostic planner failed')
        self.result['planner_probes'] = samples

    def navigate(self, x=1.5, y=0.5):
        self.mode.publish(String(data='nav'))
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.w = 1.0
        start, sim = time.monotonic(), self.sim_time()
        self.event('goal_send', x=x, y=y)
        future = self.action_clients['navigate_to_pose'].send_goal_async(goal)
        self.wait(future.done, 15, 'goal rejection', 'NavigateToPose acknowledgement')
        self.goal_handle = future.result()
        self.result['goal_accepted'] = self.goal_handle.accepted
        self.event('goal_acknowledged', accepted=self.goal_handle.accepted)
        if not self.goal_handle.accepted:
            raise RuntimeError('NavigateToPose rejected goal')
        future = self.goal_handle.get_result_async()
        self.wait(future.done, 150, 'controller failure', 'NavigateToPose result')
        action = future.result()
        tf = self.buffer.lookup_transform('map', 'base_footprint', Time())
        xyz = tf.transform.translation
        error = math.hypot(xyz.x - x, xyz.y - y)
        owners = [e.node_name for e in self.get_publishers_info_by_topic(self.command_topics[-1])]
        self.result.update(
            status=action.status,
            error_code=action.result.error_code,
            final_pose_map_xy=[xyz.x, xyz.y],
            final_error_map_m=error,
            goal_elapsed_wall_s=time.monotonic() - start,
            goal_elapsed_sim_s=self.sim_time() - sim,
            wheel_publishers=owners,
        )
        self.result.setdefault('goals', []).append(
            {
                key: self.result[key]
                for key in (
                    'goal_accepted',
                    'status',
                    'error_code',
                    'final_pose_map_xy',
                    'final_error_map_m',
                    'goal_elapsed_wall_s',
                    'goal_elapsed_sim_s',
                    'wheel_publishers',
                )
            }
        )
        self.event('goal_result', **self.result['goals'][-1])
        if action.status != 4:
            raise RuntimeError('NavigateToPose status ' + str(action.status))
        if error >= 0.35 or owners != ['cmd_vel_arbiter']:
            raise RuntimeError('final pose/command ownership acceptance failed')
        if not all(self.nonzero[t] for t in self.command_topics):
            raise RuntimeError('incomplete command chain')

    def safe_stop(self):
        self.stage = 'shutdown failure'
        self.event('stop_requested')
        self.mode.publish(String(data='idle'))
        cancel = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        if cancel.wait_for_service(timeout_sec=1):
            future = cancel.call_async(CancelGoal.Request())
            self.wait(future.done, 5, 'shutdown failure', 'navigation cancel response')
            self.event('cancel_response', return_code=future.result().return_code)
        stable = None
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            self.mode.publish(String(data='idle'))
            zero = TwistStamped()
            if 'clock' in self.last:
                zero.header.stamp = self.last['clock'].clock
            zero.header.frame_id = 'base_footprint'
            self.zero.publish(zero)
            rclpy.spin_once(self, timeout_sec=0.02)
            cmd, odom = self.last.get(self.command_topics[-1]), self.last.get('odom')
            fresh = all(
                time.monotonic() - self.received.get(k, 0) < 0.5
                for k in (self.command_topics[-1], 'odom', 'clock')
            )
            if cmd and odom and fresh:
                values = [
                    cmd.twist.linear.x,
                    cmd.twist.angular.z,
                    odom.twist.twist.linear.x,
                    odom.twist.twist.angular.z,
                ]
                zero = abs(values[0]) + abs(values[1]) < 1e-9
                stopped = abs(values[2]) < 0.02 and abs(values[3]) < 0.03
                if zero and stopped:
                    stable = self.sim_time() if stable is None else stable
                    if self.sim_time() - stable >= 0.5:
                        self.result['zero_motion'] = {
                            'command_vx_wz': values[:2],
                            'measured_vx_wz': values[2:],
                            'stable_sim_s': self.sim_time() - stable,
                        }
                        self.event('zero_motion_verified', **self.result['zero_motion'])
                        return
                else:
                    stable = None
        raise RuntimeError('zero command and stationary odometry not observed')

    def save(self):
        self.result.update(
            events=self.events,
            trace=self.trace,
            counts=dict(self.counts),
            nonzero=dict(self.nonzero),
            action_statuses=self.statuses,
            clock_steps_sim_s=self.clock_steps,
            clock_gaps_wall_s=self.clock_gaps,
            elapsed_wall_s=time.monotonic() - self.started,
        )
        self.output.write_text(json.dumps(self.result, indent=2))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    parser.add_argument('--phase', choices=('state', 'nav', 'stop'), default='nav')
    parser.add_argument('--planner-probes', type=int, default=0)
    parser.add_argument('--goals', type=int, default=1)
    args = parser.parse_args()
    rclpy.init()
    probe = Probe(args.output)
    try:
        if args.phase == 'stop':
            probe.safe_stop()
        else:
            probe.state_ready()
            if args.phase == 'nav':
                probe.nav_ready()
                probe.planner_probe(args.planner_probes)
                for index in range(args.goals):
                    probe.navigate(*((1.5, 0.5) if index % 2 == 0 else (0.0, 0.0)))
                probe.safe_stop()
        probe.result['passed'] = True
    except BaseException as exc:
        probe.result.update(error=repr(exc), failure_class=probe.stage)
        probe.event('failure', error=repr(exc), failure_class=probe.stage)
        if args.phase == 'nav':
            try:
                probe.safe_stop()
            except BaseException as stop_error:
                probe.result['stop_error'] = repr(stop_error)
    finally:
        probe.save()
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if probe.result['passed'] else 1)


if __name__ == '__main__':
    main()
