# Copyright 2026 Gautham Anil
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
