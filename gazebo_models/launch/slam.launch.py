"""
slam.launch.py
==============
Online async SLAM with slam_toolbox for the Coco robot.

Run the simulation first, then this, then drive the robot around
(teleop or web panel) to build the map:

  ros2 launch gazebo_models full_world_robo.launch.py
  ros2 launch gazebo_models slam.launch.py
  ros2 run custom_teleop teleop_wheels_node

Save the finished map into the package:

  ros2 run nav2_map_server map_saver_cli -f \
      ~/ros2_ws/src/coco-robot-ros2/gazebo_models/maps/coco_world
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('gazebo_models')
    slam_params = os.path.join(pkg_share, 'config', 'slam_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
