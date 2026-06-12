"""
rsp.launch.py
=============
Launches only the robot_state_publisher for the Coco robot (from the
coco_robo2.xacro model). Useful for visualising the robot in RViz without
a full Gazebo simulation:

  ros2 launch gazebo_models rsp.launch.py use_sim_time:=false
  rviz2 -d $(ros2 pkg prefix gazebo_models)/share/gazebo_models/rviz/coco_robot.rviz
"""

import os

import xacro
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

    xacro_path = os.path.join(
        get_package_share_directory('gazebo_models'), 'urdf', 'coco_robo2.xacro'
    )
    robot_description = xacro.process_file(xacro_path).toxml()

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
