#!/usr/bin/env python3
"""
map_drive.py
============
Scripted drive pattern to map coco_world with slam_toolbox.

Usage (with sim + slam.launch.py already running):
  python3 map_drive.py

Drives a spin/forward pattern through the west half of the arena so the
240-degree front lidar sweeps all the walls and obstacles, then stops.
"""
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

# (linear_x m/s, angular_z rad/s, seconds)
# Gentle, translation-dominant loop: slow turns (0.3 rad/s) keep the lidar
# scan matcher locked while skid-steer odometry would otherwise drift. The
# robot drives a rough rectangle around the arena so every wall is seen from
# a moving viewpoint (scan matching is far better with translation than with
# pure in-place rotation).
FWD = 0.25
TURN = 0.3
PATTERN = [
    (0.0,  TURN,  6.0),   # slow half-turn to look around at spawn
    (0.0, -TURN,  6.0),   # back
    (FWD,  0.0,   8.0),   # drive east across arena
    (FWD,  TURN,  5.5),   # arc left (toward north wall)
    (FWD,  0.0,   6.0),   # drive north
    (FWD,  TURN,  5.5),   # arc left (toward west)
    (FWD,  0.0,   8.0),   # drive west
    (FWD,  TURN,  5.5),   # arc left (toward south)
    (FWD,  0.0,   6.0),   # drive south
    (FWD,  TURN,  5.5),   # arc left (close the loop)
    (FWD,  0.0,   6.0),   # drive back toward centre
    (0.0,  TURN, 11.0),   # one slow full turn for loop closure
    (0.0,  0.0,   1.0),   # stop
]


def main():
    rclpy.init()
    node = Node('map_drive')
    pub = node.create_publisher(
        TwistStamped, '/diff_drive_controller/cmd_vel', 10)
    time.sleep(2.0)  # discovery
    for lx, az, dur in PATTERN:
        node.get_logger().info(f'cmd lx={lx} az={az} for {dur}s')
        t0 = time.time()
        while time.time() - t0 < dur and rclpy.ok():
            m = TwistStamped()
            m.header.stamp = node.get_clock().now().to_msg()
            m.twist.linear.x = lx
            m.twist.angular.z = az
            pub.publish(m)
            time.sleep(0.05)
    node.get_logger().info('pattern done')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
