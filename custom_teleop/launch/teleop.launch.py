"""
teleop.launch.py
================
Launches both teleop nodes simultaneously in separate terminals.

  - teleop_arm_node   : controls the manipulator arm
  - teleop_wheels_node: controls the 4-wheel differential drive base

Each node reads from stdin, so they are run with 'prefix' set to launch
in a new terminal window.

Usage:
  ros2 launch custom_teleop teleop.launch.py

You can also run individual nodes directly:
  ros2 run custom_teleop teleop_arm_node
  ros2 run custom_teleop teleop_wheels_node
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # xterm prefix so each node gets its own interactive terminal window
    xterm = 'xterm -e'

    arm_node = Node(
        package='custom_teleop',
        executable='teleop_arm_node',
        name='teleop_arm',
        output='screen',
        prefix=xterm,
    )

    wheels_node = Node(
        package='custom_teleop',
        executable='teleop_wheels_node',
        name='teleop_wheels',
        output='screen',
        prefix=xterm,
    )

    return LaunchDescription([
        arm_node,
        wheels_node,
    ])
