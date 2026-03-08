"""
rsp.launch.py
=============
Launches only the robot_state_publisher for the Coco robot.

Useful for visualising the robot in RViz without a full Gazebo simulation.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock (set true when running with Gazebo)',
    )

    urdf_path = os.path.join(
        get_package_share_directory('gazebo_models'), 'urdf', 'coco_robo2.urdf'
    )
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        rsp_node,
    ])
