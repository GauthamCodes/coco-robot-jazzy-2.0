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
plan_compare.py
===============
Ask each configured global planner for the same path and measure it.

"I implemented A*" is a claim, and this is the thing that answers it with
numbers instead. It drives nothing: `/compute_path_to_pose` is the
planner's action, so this asks for a plan and measures what comes back.

For each `planner_id` in nav2_params.yaml's `planner_plugins` it reports
path length, planning time, waypoint count, and the minimum clearance
from the path to the nearest occupied cell of the static map. That last
one is the interesting column — a shorter path that shaves corners is not
a better path for a robot with a 0.20 m radius, and it is exactly what
`cost_travel_multiplier` trades against.

Poses are given in WORLD coordinates and converted to the map frame here.
The map frame is anchored where slam_toolbox first saw odometry, i.e. the
spawn pose, so map = world + (2.0, 0) — see docs/RESULTS.md.

Usage (with sim + nav.launch.py running):
  ros2 run gazebo_models plan_compare.py
  ros2 run gazebo_models plan_compare.py --from -2.0 0.0 --to 0.5 0.75
  ros2 run gazebo_models plan_compare.py --planners GridBased NavFn
"""
import argparse
import math
import sys
import time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

# The map frame is anchored at the robot's spawn pose, so a world
# coordinate is this much larger in x than its map coordinate.
WORLD_TO_MAP_X = 2.0
WORLD_TO_MAP_Y = 0.0


def to_map(wx, wy):
    """World (x, y) -> map-frame (x, y)."""
    return wx + WORLD_TO_MAP_X, wy + WORLD_TO_MAP_Y


def path_length(poses):
    """Total path length in metres."""
    total = 0.0
    for a, b in zip(poses, poses[1:]):
        total += math.hypot(
            b.pose.position.x - a.pose.position.x,
            b.pose.position.y - a.pose.position.y)
    return total


class PlanCompare(Node):
    """Requests one path per planner and measures each."""

    def __init__(self):
        super().__init__('plan_compare')
        self._client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose')
        self.grid = None
        # /map is latched TRANSIENT_LOCAL by map_server and published once
        # at activation, so a VOLATILE subscriber joining later gets
        # nothing at all.
        self.create_subscription(
            OccupancyGrid, '/map', self._map_cb,
            QoSProfile(depth=1,
                       history=QoSHistoryPolicy.KEEP_LAST,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

    def _map_cb(self, msg):
        self.grid = msg

    def wait_for_map(self, timeout=15.0):
        deadline = time.time() + timeout
        while self.grid is None and time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.grid is not None

    def min_clearance(self, poses):
        """Smallest distance from any path pose to an occupied cell.

        Brute force over occupied cells: this map is 254x199 with ~2700 of
        them, so it costs milliseconds and needs no scipy.
        """
        if self.grid is None:
            return None
        info = self.grid.info
        res, ox, oy = info.resolution, info.origin.position.x, \
            info.origin.position.y
        occupied = [
            (ox + (i % info.width) * res, oy + (i // info.width) * res)
            for i, v in enumerate(self.grid.data) if v > 65
        ]
        if not occupied:
            return None
        best = float('inf')
        for p in poses:
            px, py = p.pose.position.x, p.pose.position.y
            for cx, cy in occupied:
                d = math.hypot(cx - px, cy - py)
                if d < best:
                    best = d
        return best

    def plan(self, planner_id, start, goal):
        """Request one path. Returns (poses, seconds) or (None, None)."""
        if not self._client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error(
                'compute_path_to_pose action server unavailable — is '
                'nav.launch.py running?')
            return None, None

        msg = ComputePathToPose.Goal()
        msg.planner_id = planner_id
        msg.use_start = True
        for field, (x, y) in (('start', start), ('goal', goal)):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x, pose.pose.position.y = to_map(x, y)
            pose.pose.orientation.w = 1.0
            setattr(msg, field, pose)

        t0 = time.time()
        send = self._client.send_goal_async(msg)
        rclpy.spin_until_future_complete(self, send, timeout_sec=20.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f'{planner_id}: goal rejected')
            return None, None
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=30.0)
        elapsed = time.time() - t0
        if result.result() is None:
            self.get_logger().error(f'{planner_id}: no result')
            return None, None
        poses = result.result().result.path.poses
        return (poses, elapsed) if poses else (None, None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--from', dest='start', nargs=2, type=float,
                        default=[-2.0, 0.0], metavar=('X', 'Y'),
                        help='start, in WORLD coordinates (default: spawn)')
    parser.add_argument('--to', dest='goal', nargs=2, type=float,
                        default=[0.5, 0.75], metavar=('X', 'Y'),
                        help='goal, in WORLD coordinates (default: the '
                             'mission pre-ramp pose in lane +0.75, which '
                             'is only reachable around the Zone A gate)')
    parser.add_argument('--planners', nargs='+', default=['GridBased', 'NavFn'],
                        help='planner_ids from nav2_params.yaml')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = PlanCompare()
    rc = 1
    try:
        if not node.wait_for_map():
            node.get_logger().warn(
                'no /map — clearance column will be blank')
        print(f'\nworld {tuple(args.start)} -> {tuple(args.goal)}  '
              f'(map {to_map(*args.start)} -> {to_map(*args.goal)})\n')
        print(f'{"planner":<12}{"length m":>10}{"plan ms":>10}'
              f'{"poses":>8}{"min clear m":>13}')
        print('-' * 53)
        any_ok = False
        for planner_id in args.planners:
            poses, elapsed = node.plan(planner_id, args.start, args.goal)
            if poses is None:
                print(f'{planner_id:<12}{"FAILED":>10}')
                continue
            any_ok = True
            clear = node.min_clearance(poses)
            clear_s = '—' if clear is None else f'{clear:.3f}'
            print(f'{planner_id:<12}{path_length(poses):>10.3f}'
                  f'{elapsed * 1000:>10.1f}{len(poses):>8}{clear_s:>13}')
        print()
        rc = 0 if any_ok else 1
    except (KeyboardInterrupt, ExternalShutdownException):
        # rclpy invalidates the context from its own SIGINT handler before
        # Python sees the signal, so Ctrl-C arrives as the latter.
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
