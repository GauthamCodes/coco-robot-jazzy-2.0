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
robot_state_publisher for the Coco robot (from coco_robo2.xacro), plus
RViz preloaded with the project's config.

Standalone — inspect the model with no simulator running:

  ros2 launch gazebo_models rsp.launch.py

Alongside a running simulation — full_world_robo.launch.py already starts
its own robot_state_publisher, so start RViz only and take the clock from
the simulation:

  ros2 launch gazebo_models rsp.launch.py rsp:=false use_sim_time:=true

Or skip RViz and publish TF only:

  ros2 launch gazebo_models rsp.launch.py rviz:=false
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock (set true when running with Gazebo)',
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz with the project config',
    )
    declare_rsp = DeclareLaunchArgument(
        'rsp', default_value='true',
        description='Start robot_state_publisher. Set false alongside '
                    'full_world_robo.launch.py, which starts its own — two '
                    'publishers on /robot_description fight over TF.',
    )

    pkg_share = get_package_share_directory('gazebo_models')
    xacro_path = os.path.join(pkg_share, 'urdf', 'coco_robo2.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'coco_robot.rviz')
    robot_description = xacro.process_file(xacro_path).toxml()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rsp')),
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz,
        declare_rsp,
        rsp_node,
        rviz_node,
    ])
