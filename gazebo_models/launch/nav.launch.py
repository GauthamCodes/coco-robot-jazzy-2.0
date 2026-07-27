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
nav.launch.py
=============
Nav2 autonomous navigation for the Coco robot on the saved coco_world map.

Run the simulation first:

  ros2 launch gazebo_models full_world_robo.launch.py
  ros2 launch gazebo_models nav.launch.py

Then send a goal from RViz ("2D Goal Pose") or the CLI:

  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
      "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 1.0}}}}"

AMCL is auto-initialised at the spawn pose (-2, 0). Nav2's final velocity
command (/cmd_vel, TwistStamped) is relayed to the DiffDriveController by
the cmd_vel_relay node started here.

During a mission the controller has a single publisher, cmd_vel_arbiter,
so Nav2 must not write to it directly:

  ros2 launch custom_teleop arbiter.launch.py
  ros2 launch gazebo_models nav.launch.py arbiter:=true

which points the relay at /cmd_vel_nav instead. Without arbiter:=true the
relay drives the controller exactly as it always has.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('gazebo_models')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    # With the arbiter running it, not Nav2, owns the controller topic.
    relay_output = PythonExpression([
        "'/cmd_vel_nav' if '", LaunchConfiguration('arbiter'),
        "'.lower() in ('true', '1') else '/diff_drive_controller/cmd_vel'"])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'arbiter', default_value='false',
            description='Send Nav2 through cmd_vel_arbiter (/cmd_vel_nav) '
                        'instead of straight to the controller. Requires '
                        'custom_teleop arbiter.launch.py to be running.'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_share, 'maps', 'coco_world.yaml')),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_yaml,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
            }.items(),
        ),

        Node(
            package='custom_teleop',
            executable='cmd_vel_relay',
            name='cmd_vel_relay',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'output_topic': relay_output}],
        ),
    ])
