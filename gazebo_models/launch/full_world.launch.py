"""
full_world.launch.py
====================
Lightweight launch file: starts Gazebo and spawns the ramp and robot
without setting up ros2_control or robot_state_publisher.

Useful for quick world-only testing.
Prefer full_world_robo.launch.py for a complete simulation session.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    pkg_path = get_package_share_directory('gazebo_models')

    ramp_urdf = os.path.join(pkg_path, 'urdf', 'abs.urdf')
    coco_urdf = os.path.join(pkg_path, 'urdf', 'coco_robo2.urdf')

    return LaunchDescription([
        # Start Gazebo server and client separately
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py')
            )
        ),
        # Spawn ramp after 3 s to ensure Gazebo is ready
        TimerAction(
            period=3.0,
            actions=[
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
            ],
        ),
        # Spawn robot after 5 s
        TimerAction(
            period=5.0,
            actions=[
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
            ],
        ),
    ])
