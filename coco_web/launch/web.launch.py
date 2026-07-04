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
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    web_root = os.path.join(get_package_share_directory('coco_web'), 'web')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'port': 9090}],
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
            parameters=[{'use_sim_time': use_sim_time, 'port': 8081}],
        ),
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', '8000',
                 '--bind', '0.0.0.0', '--directory', web_root],
            output='screen',
        ),
    ])
