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
platform.launch.py — the COCO web platform.

Starts the versioned WebSocket/HTTP server, the MJPEG camera server, and
(by default) the velocity arbiter the browser's joystick feeds.

  ros2 launch coco_web platform.launch.py
  # then open http://localhost:8080

This is the appliance entry point. It replaces web.launch.py's
rosbridge + static-http pair with one node that speaks a closed protocol;
web.launch.py still exists and still starts the legacy rosbridge panel,
for one release, for anyone with a bookmark.

Ports
-----
  8080  HTTP + WebSocket (the UI, /ws, /healthz, /api/session) and the
        retained MJPEG view at /video/<alias>
  8081  web_video_server, bound to 127.0.0.1: only the platform talks to
        it. Its URLs take a topic in the query string, so on the LAN it
        would let any browser request any image topic on the graph.

arbiter:=false when something else already started cmd_vel_arbiter --
mission.launch.py does, which is why it passes arbiter:=false itself.
Two arbiters on one graph is not a cosmetic clash: both publish the wheel
topic, and the robot then tracks the average of two decisions.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description for the web platform."""
    arbiter_launch = os.path.join(
        get_package_share_directory('custom_teleop'),
        'launch', 'arbiter.launch.py')
    use_sim_time = LaunchConfiguration('use_sim_time')
    http_port = LaunchConfiguration('http_port')
    video_port = LaunchConfiguration('video_port')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'http_port', default_value='8080',
            description='HTTP + WebSocket port serving the UI and /ws'),
        DeclareLaunchArgument(
            'video_port', default_value='8081',
            description='web_video_server MJPEG port'),
        DeclareLaunchArgument(
            'bind', default_value='0.0.0.0',
            description='address to bind; 127.0.0.1 to refuse the LAN'),
        DeclareLaunchArgument(
            'depth_topic', default_value='/camera/depth/image_raw',
            description='depth IMAGE shown in the browser, opt-in per '
                        'client; empty disables it. This is display only '
                        'and is NOT depth fusion, which is '
                        'nav.launch.py depth_cloud:= and stays off'),
        DeclareLaunchArgument(
            'expected_components', default_value='lidar',
            description='optional components whose absence makes the '
                        'session health DEGRADED (comma-separated: lidar, '
                        'navigation, perception, mission). The launch that '
                        'starts them should declare them'),
        DeclareLaunchArgument(
            'arbiter', default_value='true',
            description='start cmd_vel_arbiter, which forwards the '
                        'browser joystick to the wheel controller'),
        DeclareLaunchArgument(
            'video', default_value='true',
            description='start web_video_server for the camera streams'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(arbiter_launch),
            condition=IfCondition(LaunchConfiguration('arbiter')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        Node(
            package='coco_web',
            executable='platform_server',
            name='coco_web_platform',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'http_port': http_port,
                'video_port': video_port,
                'bind': LaunchConfiguration('bind'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'expected_components':
                    LaunchConfiguration('expected_components'),
            }],
        ),

        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen',
            condition=IfCondition(LaunchConfiguration('video')),
            # Loopback only. The browser reaches MJPEG through the
            # platform's /video/<alias>; a LAN-facing web_video_server
            # would take any topic a browser typed into its URL.
            parameters=[{'use_sim_time': use_sim_time, 'port': video_port,
                         'address': '127.0.0.1'}],
        ),
    ])
