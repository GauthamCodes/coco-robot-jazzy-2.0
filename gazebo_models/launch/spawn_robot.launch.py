"""
spawn_robot.launch.py
=====================
Spawns only the Coco robot into a running Gazebo instance.

Requires Gazebo to already be running with robot_state_publisher active.
For a complete simulation, use full_world_robo.launch.py instead.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    coco_urdf = os.path.join(
        get_package_share_directory('gazebo_models'), 'urdf', 'coco_robo2.urdf'
    )

    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_coco',
            arguments=[
                '-entity', 'coco',
                '-file', coco_urdf,
                '-x', '0.0', '-y', '0.0', '-z', '0.3',
            ],
            output='screen',
        )
    ])
