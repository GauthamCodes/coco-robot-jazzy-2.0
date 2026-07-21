"""
Launch both teleop nodes simultaneously in separate terminals.

  - teleop_arm_node   : controls the manipulator arm
  - teleop_wheels_node: controls the 4-wheel differential drive base

Both nodes read keypresses from stdin, so each needs a terminal of its
own — hence the 'prefix', which wraps the node in a terminal emulator.
The default is xterm (declared as an exec_depend and installed in the
Dockerfile), but any emulator that takes a trailing '-e <command>' works:

  ros2 launch custom_teleop teleop.launch.py terminal:='gnome-terminal --'
  ros2 launch custom_teleop teleop.launch.py terminal:='konsole -e'

Usage:
  ros2 launch custom_teleop teleop.launch.py

You can also run individual nodes directly, in which case the terminal
you launch them from is the one they read from:
  ros2 run custom_teleop teleop_arm_node
  ros2 run custom_teleop teleop_wheels_node
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('custom_teleop'), 'config', 'teleop.yaml')

    terminal_arg = DeclareLaunchArgument(
        'terminal',
        default_value='xterm -e',
        description='Terminal emulator prefix; each teleop node reads stdin '
                    'and so needs its own window. Override for a machine '
                    "without xterm, e.g. 'gnome-terminal --'.",
    )
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='YAML of teleop parameters (step sizes, velocity caps, '
                    'input watchdog). Defaults to the values compiled into '
                    'the nodes.',
    )
    terminal = LaunchConfiguration('terminal')
    params_file = LaunchConfiguration('params_file')

    arm_node = Node(
        package='custom_teleop',
        executable='teleop_arm_node',
        name='teleop_arm',
        output='screen',
        prefix=terminal,
        parameters=[params_file],
    )

    wheels_node = Node(
        package='custom_teleop',
        executable='teleop_wheels_node',
        name='teleop_wheels',
        output='screen',
        prefix=terminal,
        parameters=[params_file],
    )

    return LaunchDescription([
        terminal_arg,
        params_arg,
        arm_node,
        wheels_node,
    ])
