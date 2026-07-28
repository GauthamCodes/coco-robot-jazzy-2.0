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

"""
nav_round_trip.py
=================
Drive Nav2 legs through the NavigateToPose ACTION and measure the result.

The web panel sends goals on `/goal_pose`, a bare topic: no feedback, no
result code, no cancel. That is fine for a human clicking a map and
useless for a mission, which has to know whether a leg succeeded before
it starts the next one. This is the action-based client the fetch mission
sequencer is built from, and on its own it is the "there and back" demo.

Default route is home -> the mission's pre-ramp pose in lane +0.75 ->
home. The outbound leg matters because it cannot be driven straight: the
Zone A gate sits between them, offset from y=0 on purpose, so a planner
that works is visibly different from one that does not.

Per leg it reports the result code, wall time, straight-line vs driven
distance, and the **minimum clearance of the DRIVEN path** to the nearest
occupied cell. That last number is the one that shows whether DWB is
actually avoiding obstacles rather than tracking the global path through
them — `BaseObstacle.scale` against `PathAlign/PathDist` decides it, and
the global path alone will not reveal the difference.

Poses are given in WORLD coordinates. The map frame is anchored where
slam_toolbox first saw odometry, i.e. the spawn pose, so
map = world + (2.0, 0).

Usage (with sim + nav.launch.py running):
  ros2 run gazebo_models nav_round_trip.py
  ros2 run gazebo_models nav_round_trip.py --waypoints 0.5 0.75 -2.0 0.0
"""
import argparse
import math
import sys
import time

from action_msgs.msg import GoalStatus
from coco_config.robot import lane_for_colour
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

WORLD_TO_MAP_X = 2.0
WORLD_TO_MAP_Y = 0.0

# Home is the spawn pose; the outbound goal is the mission's pre-ramp pose
# for the yellow target's lane (coco_config.robot.TARGETS).
DEFAULT_WAYPOINTS = [(0.5, lane_for_colour('yellow')), (-2.0, 0.0)]

STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
}


class RoundTrip(Node):
    """NavigateToPose client that records what the robot actually did."""

    def __init__(self):
        super().__init__('nav_round_trip')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pose = None
        self.track = []
        self.grid = None
        self.create_subscription(
            Odometry, '/model/coco/odometry', self._odom_cb, 10)
        # /map is latched TRANSIENT_LOCAL and published once at
        # activation; a VOLATILE subscriber joining later sees nothing.
        self.create_subscription(
            OccupancyGrid, '/map', self._map_cb,
            QoSProfile(depth=1,
                       history=QoSHistoryPolicy.KEEP_LAST,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)

    def _map_cb(self, msg):
        self.grid = msg

    def _occupied(self):
        info = self.grid.info
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        return [(ox + (i % info.width) * res, oy + (i // info.width) * res)
                for i, v in enumerate(self.grid.data) if v > 65]

    def min_clearance(self):
        """Closest the driven path came to an occupied cell, in metres.

        Driven poses are world coordinates and the map's cells are map
        coordinates, so the track is shifted before comparing.
        """
        if self.grid is None or not self.track:
            return None
        occupied = self._occupied()
        if not occupied:
            return None
        best = float('inf')
        for wx, wy in self.track:
            px, py = wx + WORLD_TO_MAP_X, wy + WORLD_TO_MAP_Y
            for cx, cy in occupied:
                d = math.hypot(cx - px, cy - py)
                if d < best:
                    best = d
        return best

    def wait_ready(self, timeout=30.0):
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose is not None and self.grid is not None:
                return True
        self.get_logger().error(
            f'not ready after {timeout:.0f}s '
            f'(odom={self.pose is not None}, map={self.grid is not None})')
        return False

    def leg(self, wx, wy, timeout=180.0):
        """Drive one leg. Returns (status_name, seconds, driven_metres)."""
        if not self._client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error(
                'navigate_to_pose unavailable — is nav.launch.py running?')
            return 'NO-SERVER', 0.0, 0.0

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = wx + WORLD_TO_MAP_X
        goal.pose.pose.position.y = wy + WORLD_TO_MAP_Y
        goal.pose.pose.orientation.w = 1.0

        start_pose = self.pose
        driven = 0.0
        last = self.pose
        t0 = time.time()

        send = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=20.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            return 'REJECTED', time.time() - t0, 0.0

        result = handle.get_result_async()
        while not result.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose is not None:
                if last is not None:
                    driven += math.hypot(self.pose[0] - last[0],
                                         self.pose[1] - last[1])
                last = self.pose
                self.track.append(self.pose)
            if time.time() - t0 > timeout:
                # Cancel rather than abandon: an orphaned goal keeps
                # driving the robot after this process exits.
                self.get_logger().warn('leg timed out — cancelling')
                cancel = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel, timeout_sec=10.0)
                return 'TIMEOUT', time.time() - t0, driven

        elapsed = time.time() - t0
        res = result.result()
        status = STATUS_NAMES.get(
            getattr(res, 'status', None), f'status={getattr(res, "status", "?")}')
        straight = 0.0
        if start_pose is not None and self.pose is not None:
            straight = math.hypot(self.pose[0] - start_pose[0],
                                  self.pose[1] - start_pose[1])
        self._last_straight = straight
        return status, elapsed, driven


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--waypoints', nargs='+', type=float,
                        help='flat list of WORLD x y pairs to visit in turn')
    parser.add_argument('--timeout', type=float, default=180.0)
    args, _ = parser.parse_known_args()

    if args.waypoints:
        if len(args.waypoints) % 2:
            print('--waypoints needs an even number of values', file=sys.stderr)
            sys.exit(2)
        pts = list(zip(args.waypoints[0::2], args.waypoints[1::2]))
    else:
        pts = DEFAULT_WAYPOINTS

    rclpy.init()
    node = RoundTrip()
    rc = 1
    try:
        if not node.wait_ready():
            sys.exit(1)
        start = node.pose
        print(f'\nstart (world): ({start[0]:.2f}, {start[1]:.2f})')
        print(f'{"leg":<6}{"goal (world)":<18}{"result":<12}'
              f'{"s":>7}{"driven m":>10}')
        print('-' * 53)
        ok = True
        for i, (wx, wy) in enumerate(pts, 1):
            status, secs, driven = node.leg(wx, wy, args.timeout)
            ok &= (status == 'SUCCEEDED')
            print(f'{i:<6}{f"({wx:.2f}, {wy:.2f})":<18}{status:<12}'
                  f'{secs:>7.1f}{driven:>10.2f}')
        end = node.pose
        home_err = math.hypot(end[0] - start[0], end[1] - start[1])
        clear = node.min_clearance()
        print(f'\nend (world):   ({end[0]:.2f}, {end[1]:.2f})')
        print(f'returned to within {home_err:.2f} m of the start')
        if clear is not None:
            print(f'min clearance of the DRIVEN path to an obstacle: '
                  f'{clear:.3f} m')
        print('ALL LEGS SUCCEEDED' if ok else 'SOME LEGS FAILED')
        rc = 0 if ok else 1
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
