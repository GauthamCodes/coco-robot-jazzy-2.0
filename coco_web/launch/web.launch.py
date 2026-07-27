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
web.launch.py
=============
Browser control panel for the Coco robot.

Starts:
  - rosbridge_websocket (ws://<host>:9090) — topics over websocket
  - rosapi              — introspection services for roslibjs
  - web_video_server    (http://<host>:8081) — MJPEG camera streams
  - a static HTTP server (http://<host>:8000) serving the panel

Usage (with the simulation already running):
  ros2 launch coco_web web.launch.py
  # then open http://<robot-ip>:8000 from any device on the same network

The panel's map view + click-to-goal needs Nav2 (nav.launch.py) or
slam_toolbox running so /map is published.

The panel's joystick publishes /cmd_vel_teleop, so this file also starts
cmd_vel_arbiter (arbiter:=false to opt out) — without it nothing forwards
the stick to the wheels. When Nav2 runs alongside, start it with
`nav.launch.py arbiter:=true` so its relay feeds /cmd_vel_nav instead of
publishing to the controller in parallel with the arbiter.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    web_root = os.path.join(get_package_share_directory('coco_web'), 'web')
    arbiter_launch = os.path.join(
        get_package_share_directory('custom_teleop'),
        'launch', 'arbiter.launch.py')
    use_sim_time = LaunchConfiguration('use_sim_time')

    bridge_port = LaunchConfiguration('bridge_port')
    video_port = LaunchConfiguration('video_port')
    http_port = LaunchConfiguration('http_port')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # Ports are arguments so two people (or two sims) can share a
        # machine. The panel defaults to these same numbers and accepts
        # overrides as query parameters, e.g.
        #   http://<host>:8010/?ws=9091&video=8082
        DeclareLaunchArgument(
            'bridge_port', default_value='9090',
            description='rosbridge websocket port'),
        DeclareLaunchArgument(
            'video_port', default_value='8081',
            description='web_video_server MJPEG port'),
        DeclareLaunchArgument(
            'http_port', default_value='8000',
            description='static HTTP port serving the control panel'),
        # Default true: the panel publishes /cmd_vel_teleop, so without an
        # arbiter the joystick would move nothing at all. Set false only
        # when some other launch file already started one.
        DeclareLaunchArgument(
            'arbiter', default_value='true',
            description='start cmd_vel_arbiter, which forwards the panel '
                        'joystick to the wheel controller'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(arbiter_launch),
            condition=IfCondition(LaunchConfiguration('arbiter')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'port': bridge_port}],
        ),
        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'port': video_port}],
        ),
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', http_port,
                 '--bind', '0.0.0.0', '--directory', web_root],
            output='screen',
        ),
    ])
