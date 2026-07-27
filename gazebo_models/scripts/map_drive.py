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
map_drive.py
============
Closed-loop waypoint drive to map coco_world with slam_toolbox.

Usage (with sim + slam.launch.py already running):
  python3 map_drive.py

Follows a waypoint loop around the whole arena — south lane, east half
behind the ramp, north lane, back to spawn — using the ground-truth
odometry published by the gz OdometryPublisher plugin
(/model/coco/odometry). A P-controller steers toward each waypoint;
translation-dominant motion keeps the slam_toolbox scan matcher locked
(skid-steer wheel odometry alone drifts badly during in-place turns).

Waypoints with a nonzero spin angle end with a slow turn so the
240-degree front lidar fills in coverage behind the robot; loop closure
comes mainly from retracing each lane in both directions.
"""
import math
import time

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

FWD = 0.25         # cruise speed, m/s
TURN_MAX = 0.3     # steering clamp, rad/s — faster turns break scan matching
ARRIVE = 0.3       # waypoint capture radius, m
SPIN_RATE = 0.25   # rad/s during coverage spins
HEADING_GATE = 0.6  # rad of heading error above which we turn in place


def steer(px, py, yaw, x, y):
    """Velocity command to drive from (px, py, yaw) toward (x, y).

    Returns (linear, angular, distance). Pure — no ROS, no state — so the
    wrap-around case is testable: a robot facing +179 deg with a target at
    -179 deg must turn 2 deg the short way, not 358 deg the long way.
    Drives forward only once roughly aimed, so it arcs onto the line
    instead of sweeping a wide curve that would blur the scan match.
    """
    dist = math.hypot(x - px, y - py)
    err = math.atan2(y - py, x - px) - yaw
    err = math.atan2(math.sin(err), math.cos(err))   # wrap to (-pi, pi]
    angular = max(-TURN_MAX, min(TURN_MAX, 0.8 * err))
    linear = FWD if abs(err) < HEADING_GATE else 0.0
    return linear, angular, dist

# (x, y, spin_after_radians). Arena walls: x -4.0..8.0, y -3.5..3.5, inner
# faces 0.1 m in. With traverse:=true the ramp assembly occupies
# x 1.0..6.5, y +/-1.25 — up-slope 1.0..3.0, platform 3.0..4.5, down-slope
# 4.5..6.5. Obstacles: boxes at (0.5, 1.2) and (0.8, -1.4), cylinder at
# (-0.2, 0.6), gate cubes at (-1.1, +1.05) and (-1.1, -0.75), wall
# pilasters at (7.72, +2.0) and (7.72, -1.4).
#
# Out-and-back lanes re-observe the same walls from both directions, which
# gives the pose graph loop closures without in-place 360s (pure rotation
# is the scan matcher's worst case).
#
# The east corridor (x>6.5) is no longer skipped. It used to be: two long
# parallel surfaces left the matcher unconstrained along the corridor
# axis. But the fetch mission descends the far side of the ramp and has to
# come home through it, and `track_unknown_space: true` makes anything
# unmapped untraversable — an unmapped corridor is an unusable return leg.
# The two wall pilasters are what make it mappable, and the drive down it
# is at x=7.05: clear of the down-ramp foot (6.5) and of the pilasters
# (which start at 7.57), with ~0.5 m either side.
#
# Nothing here may enter x 1.0..6.5 with |y| < 1.75 — that drives onto the
# ramp.
ROUTE = [
    (-2.2, -2.5, 0.0),        # into the south lane
    (1.0, -2.7, 0.0),         # east along the south lane
    (4.5, -2.6, 0.0),         # past the ramp's south side (no spin: corridor!)
    (6.6, -2.5, 0.0),         # approach the east corridor
    (7.05, -1.2, 0.0),        # into the corridor, south end
    (7.05, 1.2, 0.0),         # north behind the down-ramp — the return leg
    (6.6, 2.5, 0.0),          # out into the north lane
    (4.5, 2.6, 0.0),          # west along the north lane
    (1.0, 2.7, 0.0),          # continue west
    (-2.2, 2.5, 0.0),         # north-west corner
    (-3.2, 0.0, 0.0),         # west end, off the west wall
    (-2.2, -2.5, 0.0),        # back to the south lane (loop closure)
    (1.0, -2.7, 0.0),         # retrace east
    (-1.0, -2.4, 0.0),        # retrace west
    (0.2, 0.0, 2 * math.pi),  # ramp foot + the mission's pre-ramp poses;
                              #   one slow turn, in open ground not a corridor
    (-2.0, 0.0, 2 * math.pi),  # home; one slow turn to close the loop
]


class WaypointDriver(Node):
    """Drive a waypoint route using ground-truth odometry feedback."""

    def __init__(self):
        super().__init__('map_drive')
        self.pub = self.create_publisher(
            TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.pose = None
        self.create_subscription(
            Odometry, '/model/coco/odometry', self._odom_cb, 10)

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)

    def _cmd(self, lx, az):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.twist.linear.x = lx
        m.twist.angular.z = az
        self.pub.publish(m)

    def _spin_for(self, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds and rclpy.ok():
            self._cmd(0.0, SPIN_RATE)
            rclpy.spin_once(self, timeout_sec=0.05)

    def goto(self, x, y, timeout=90.0):
        t0 = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            # Deadline is checked before the pose guard: an odometry
            # dropout must not turn this into an unbounded spin.
            if time.time() - t0 > timeout:
                self.get_logger().warn(f'timeout heading to ({x}, {y})')
                self._cmd(0.0, 0.0)
                return False
            if self.pose is None:
                continue
            px, py, yaw = self.pose
            lx, az, dist = steer(px, py, yaw, x, y)
            if dist < ARRIVE:
                self.get_logger().info(f'reached ({x:.1f}, {y:.1f})')
                return True
            self._cmd(lx, az)

    def run(self):
        """Drive the route. Returns True only if every waypoint was reached.

        Stops the robot on every exit path where that is still possible.
        On Ctrl-C it is not: rclpy invalidates the context from its own
        SIGINT handler before we regain control, so the stop cannot be
        published and the controller's `cmd_vel_timeout` (0.5 s) is what
        actually halts the robot. Verified: velocity settles to ~1e-5 m/s
        within a second of the interrupt.
        """
        try:
            # wait for ground-truth odometry
            t0 = time.time()
            while self.pose is None and time.time() - t0 < 15.0 and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.2)
            if self.pose is None:
                self.get_logger().error(
                    'no /model/coco/odometry — is the sim up?')
                return False
            for i, (x, y, spin) in enumerate(ROUTE, 1):
                # A stuck robot silently burns the timeout at every remaining
                # waypoint and still reports "route done", leaving a partial
                # map that looks like a success. Abort instead.
                if not self.goto(x, y):
                    self.get_logger().error(
                        f'waypoint {i}/{len(ROUTE)} ({x}, {y}) not reached — '
                        f'aborting route; do not trust a map saved from this run')
                    return False
                if spin > 0.0:
                    self._spin_for(spin / SPIN_RATE)
            self.get_logger().info('route done')
            return True
        finally:
            if rclpy.ok():
                self._cmd(0.0, 0.0)


def main():
    rclpy.init()
    node = WaypointDriver()
    ok = False
    try:
        ok = node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        # rclpy's SIGINT handler tears the context down first, so Ctrl-C
        # arrives as ExternalShutdownException rather than KeyboardInterrupt.
        # Exit quietly instead of dumping a traceback on an expected action.
        print('\ninterrupted — robot stops on the controller watchdog')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
