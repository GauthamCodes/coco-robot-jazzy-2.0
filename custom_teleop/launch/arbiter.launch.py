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
Start cmd_vel_arbiter, the sole publisher of /diff_drive_controller/cmd_vel.

  ros2 launch custom_teleop arbiter.launch.py

Once this is running, nothing else should publish to the controller. Point
the other sources at the arbiter's inputs instead:

  ros2 launch gazebo_models nav.launch.py arbiter:=true
  ros2 run custom_teleop teleop_wheels_node --ros-args
      -p cmd_vel_topic:=/cmd_vel_teleop

The web panel publishes /cmd_vel_teleop already, and the RL environment
takes its topic as a constructor argument.

The mode comes from /mission/mode, which the panel asserts at 2 Hz. With no
publisher the arbiter sits in idle and only teleop can move the robot, so a
standalone Nav2 demo needs:

  ros2 launch custom_teleop arbiter.launch.py initial_mode:=nav

Watch what it is doing:

  ros2 topic echo /cmd_vel_arbiter/status
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description for the velocity arbiter."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    initial_mode = LaunchConfiguration('initial_mode')
    source_timeout = LaunchConfiguration('source_timeout')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'initial_mode', default_value='idle',
            description='Mode before anything publishes /mission/mode. '
                        'idle lets only teleop drive, which is the safe '
                        'default; use nav to run Nav2 without the panel.'),
        DeclareLaunchArgument(
            'source_timeout', default_value='0.3',
            description='Seconds after the last message from a source '
                        'ARRIVES before it counts as stale and the robot '
                        'is stopped.'),

        Node(
            package='custom_teleop',
            executable='cmd_vel_arbiter',
            name='cmd_vel_arbiter',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'initial_mode': initial_mode,
                'source_timeout': source_timeout,
            }],
        ),
    ])
