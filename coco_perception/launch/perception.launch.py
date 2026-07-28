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
Start target_finder, which identifies the fetch mission's platform target.

  ros2 launch coco_perception perception.launch.py

Stands alone rather than being folded into full_world_robo.launch.py: an
edge from gazebo_models to this package would close a dependency cycle
(gazebo_models -> custom_teleop -> coco_config), and colcon refuses to
order a workspace containing one.

Which target it looks for comes from /mission/target_colour, which the
web panel asserts at 2 Hz. Set the starting value for a standalone run:

  ros2 launch coco_perception perception.launch.py target_colour:=red

Watch what it sees:

  ros2 topic echo /perception/status
  ros2 topic echo /perception/target

The annotated frame goes out on /perception/annotated, which
web_video_server discovers on its own — no change to coco_web needed:

  http://<host>:8081/stream?topic=/perception/annotated&type=mjpeg
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description for the perception node."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    target_colour = LaunchConfiguration('target_colour')
    params = os.path.join(
        get_package_share_directory('coco_perception'),
        'config', 'target_finder.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'target_colour', default_value='blue',
            description='Which target to look for until /mission/target_'
                        'colour says otherwise: red, green, blue or yellow.'),
        DeclareLaunchArgument(
            'publish_annotated', default_value='true',
            description='Publish /perception/annotated for the phone. '
                        'Costs one image copy per frame; turn it off if '
                        'the camera rate sags.'),

        Node(
            package='coco_perception',
            executable='target_finder',
            name='target_finder',
            output='screen',
            parameters=[params, {
                'use_sim_time': use_sim_time,
                'target_colour': target_colour,
                'publish_annotated': LaunchConfiguration('publish_annotated'),
            }],
        ),
    ])
