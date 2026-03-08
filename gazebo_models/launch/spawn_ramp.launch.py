"""
spawn_ramp.launch.py
====================
Spawns only the ramp platform into a running Gazebo instance.

Requires Gazebo to already be running (e.g. started via full_world_robo.launch.py).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ramp_urdf = os.path.join(
        get_package_share_directory('gazebo_models'), 'urdf', 'abs.urdf'
    )

    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_ramp',
            arguments=[
                '-entity', 'ramp',
                '-file', ramp_urdf,
                '-x', '1.0', '-y', '0.0', '-z', '0.0',
            ],
            output='screen',
        )
    ])
