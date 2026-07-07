#!/usr/bin/env python3
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
from rclpy.node import Node

FWD = 0.25         # cruise speed, m/s
TURN_MAX = 0.3     # steering clamp, rad/s — faster turns break scan matching
ARRIVE = 0.3       # waypoint capture radius, m
SPIN_RATE = 0.25   # rad/s during coverage spins

# (x, y, spin_after_radians) — the ramp occupies x 1.1..5.5, y -1.3..1.3;
# boxes sit at (0.5, 1.2) and (0.8, -1.4), cylinder at (-0.8, 0.3).
# Out-and-back lanes re-observe the same walls from both directions, which
# gives the pose graph loop closures without in-place 360s (pure rotation
# is the scan matcher's worst case). The corridor behind the ramp
# (x>5.5) is skipped: two long parallel walls leave the matcher
# unconstrained along the corridor axis.
ROUTE = [
    (-2.0, -2.3, 0.0),        # into the south lane
    (2.0, -2.5, 0.0),         # east along the south lane
    (4.3, -2.4, 0.0),         # ramp's south wall (no spin: corridor!)
    (0.0, -2.4, 0.0),         # retrace the lane (loop closure)
    (0.5, -0.4, 0.0),         # ramp's low west face, from open ground
    (-2.4, -0.5, 0.0),        # back near spawn
    (-2.2, 2.0, 0.0),         # into the north lane
    (2.5, 2.5, 0.0),          # east along the north lane
    (4.3, 2.2, 0.0),          # ramp's north wall (no spin)
    (1.0, 2.4, 0.0),          # retrace
    (-2.2, 1.2, 0.0),
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
            if self.pose is None:
                continue
            px, py, yaw = self.pose
            dist = math.hypot(x - px, y - py)
            if dist < ARRIVE:
                self.get_logger().info(f'reached ({x:.1f}, {y:.1f})')
                return True
            if time.time() - t0 > timeout:
                self.get_logger().warn(f'timeout heading to ({x}, {y})')
                return False
            err = math.atan2(y - py, x - px) - yaw
            err = math.atan2(math.sin(err), math.cos(err))
            az = max(-TURN_MAX, min(TURN_MAX, 0.8 * err))
            lx = FWD if abs(err) < 0.6 else 0.0
            self._cmd(lx, az)

    def run(self):
        # wait for ground-truth odometry
        t0 = time.time()
        while self.pose is None and time.time() - t0 < 15.0 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.pose is None:
            self.get_logger().error('no /model/coco/odometry — is the sim up?')
            return
        for x, y, spin in ROUTE:
            self.goto(x, y)
            if spin > 0.0:
                self._spin_for(spin / SPIN_RATE)
        self._cmd(0.0, 0.0)
        self.get_logger().info('route done')


def main():
    rclpy.init()
    node = WaypointDriver()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
