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
Everything the fetch mission needs, except the simulator.

Before this existed the mission was six terminals and a docs page that
listed three of them. Following RUNNING.md's Demo 7 got you the sim,
perception and the web panel — and then a sequencer that hung, because
Nav2, the arbiter and ramp_driver were never started, and move_group (which
the grasp needs) was in no mission launch file at all.

This lives in its own package because of the dependency graph rather than
for tidiness: coco_moveit_config already depends on gazebo_models for the
robot description, so a launch file in gazebo_models that starts move_group
closes a cycle and colcon refuses to order the workspace at all.

The simulator is deliberately NOT included. Only one Gazebo can run on this
machine at a time and the mission needs a fresh one per run — the
DetachableJoint binds to each target once, on first spawn — so it stays a
separate, deliberate command:

  # T1, and restart it between runs:
  ros2 launch gazebo_models full_world_robo.launch.py traverse:=true gui:=false
  # T2:
  ros2 launch coco_mission mission.launch.py policy:=<path to the .zip>
  # T3, optional — the phone picks the colour and shows the camera:
  ros2 launch coco_web web.launch.py
  # T4:
  ros2 run gazebo_models traverse_demo.py --colour blue

What is started, and why each is load-bearing
---------------------------------------------
nav.launch.py (arbiter:=true)  Nav2 for the two flat-ground legs. The
      argument only repoints cmd_vel_relay at /cmd_vel_nav; it does NOT
      start the arbiter, which is why the arbiter is a separate entry here.
arbiter.launch.py              cmd_vel_arbiter, the sole publisher to the
      wheels. Without it four sources interleave and the robot tracks their
      average instead of obeying one.
perception.launch.py           target_finder: confirms the colour is in
      front and feeds /perception/target to the approach.
move_group.launch.py           MoveIt. The grasp cannot plan without it.
ramp_driver                    the PPO climb and the scripted descent.
approach_server                the 1.198 m from the end of the climb to the
      grasp pose, which nothing else can drive.
grasp_server                   stow, pick, place.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description for the whole mission stack."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    policy = LaunchConfiguration('policy')
    target_colour = LaunchConfiguration('target_colour')

    gazebo_models = get_package_share_directory('gazebo_models')
    custom_teleop = get_package_share_directory('custom_teleop')
    coco_perception = get_package_share_directory('coco_perception')
    coco_moveit = get_package_share_directory('coco_moveit_config')

    def include(package_share, name, arguments=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_share, 'launch', name)),
            launch_arguments=(arguments or {}).items())

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'policy', default_value='',
            description='Path to the trained PPO .zip. Without it '
                        'ramp_driver refuses /ramp/climb, which is the '
                        'right failure — a mission that drives to the ramp '
                        'and then discovers it has no policy has already '
                        'wasted the run.'),
        DeclareLaunchArgument(
            'target_colour', default_value='blue',
            description='Initial colour for target_finder, until the panel '
                        'or the sequencer publishes /mission/target_colour. '
                        'approach_server and grasp_server deliberately have '
                        'no default: stopping in the wrong window, or '
                        'welding the wrong object, is worse than refusing.'),
        DeclareLaunchArgument(
            'lateral_hold', default_value='true',
            description='Correct the RL policy toward the lane centreline '
                        'during the climb. false reproduces the bare '
                        'policy, which drifts 0.58 m from a Nav2-legal '
                        'start heading and takes the outer lanes to within '
                        '70 mm of the platform edge.'),

        include(gazebo_models, 'nav.launch.py',
                {'use_sim_time': use_sim_time, 'arbiter': 'true'}),
        include(custom_teleop, 'arbiter.launch.py',
                {'use_sim_time': use_sim_time}),
        include(coco_perception, 'perception.launch.py',
                {'use_sim_time': use_sim_time,
                 'target_colour': target_colour}),
        include(coco_moveit, 'move_group.launch.py',
                {'use_sim_time': use_sim_time}),

        Node(
            package='coco_rl',
            executable='ramp_driver',
            name='ramp_driver',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'model': policy,
                # NOT the controller topic. ramp_env builds its node
                # directly, so a launch-file remapping is accepted and then
                # silently ignored; the topic has to be a parameter.
                'cmd_vel_topic': '/cmd_vel_rl',
                'lateral_hold': LaunchConfiguration('lateral_hold'),
            }],
        ),
        Node(
            package='custom_teleop',
            executable='approach_server',
            name='approach_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='coco_moveit_config',
            executable='grasp_server.py',
            name='grasp_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
