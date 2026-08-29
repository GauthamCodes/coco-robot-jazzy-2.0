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
Start the perception node that tells the mission where its target is.

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

WHICH NODE OWNS THE TARGET — `target_source` (C2-M4.2)
------------------------------------------------------
Two nodes can answer "where is the target", and exactly one may run:

  target_source:=target_finder   the default. The M6 path, measured
        19/20 through the full fetch and 20/20 on the approach window.
        Publishes /perception/target, /perception/status and
        /perception/annotated.

  target_source:=target_pose     the C2-M4 path. `target_pose_node`
        with `point_topic` and `status_compat_topic` both set, so it
        stands where target_finder stood for BOTH consumers: the
        approach (/perception/target) and the executive's SEARCH_TARGET
        vision gate (/perception/status). It also publishes its own
        richer /perception/target_pose, /perception/grasp_point and
        /perception/target_pose/status, which target_finder has no
        vocabulary for. Measured in C2-M4: 60/60 placements, horizontal
        error 0.7/1.4/2.4 mm min/median/max, 8/8 live grasps.

        It does NOT publish /perception/annotated, so the web panel's
        camera pane is dark on this path. Nothing in the mission reads
        it; it is an operator convenience, and this is recorded here
        rather than left to be discovered during a demo.

**The selection is an if/else in Python, not two conditions.** Two
`IfCondition`s over the same argument can both be false on a typo — a
mission that launches with no perception at all and diagnoses as "the
camera is broken" — and both can be true if someone edits one and not
the other, which is two estimates racing for /perception/target with
the grasp taking whichever landed last. `OpaqueFunction` makes exactly
one node exist by construction, and an unknown value raises at launch
time rather than producing a silently half-built graph.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# The two answers `target_source` accepts. A module constant so the test
# that asserts "exactly one publisher of /perception/target" enumerates
# the same set this file dispatches on, rather than a copy that drifts.
TARGET_SOURCES = ('target_finder', 'target_pose')

# The topics the mission consumes, and which this file guarantees have
# exactly one publisher whichever source is selected.
TARGET_TOPIC = '/perception/target'
STATUS_TOPIC = '/perception/status'


def perception_node(context, *args, **kwargs):
    """
    Build the one perception node `target_source` names.

    An OpaqueFunction rather than two IfConditions — see the module
    docstring. It returns a single-element list, and that is the whole
    of the duplicate-publisher argument: there is no code path here
    that returns two nodes.
    """
    source = LaunchConfiguration('target_source').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    target_colour = LaunchConfiguration('target_colour')

    if source not in TARGET_SOURCES:
        raise RuntimeError(
            f'target_source:={source!r} is not one of '
            f'{", ".join(TARGET_SOURCES)}. Refusing to launch rather than '
            f'starting no perception at all — a mission with no target '
            f'publisher fails in SEARCH_TARGET 15 s later and reads as a '
            f'camera fault.')

    if source == 'target_finder':
        params = os.path.join(
            get_package_share_directory('coco_perception'),
            'config', 'target_finder.yaml')
        return [Node(
            package='coco_perception',
            executable='target_finder',
            name='target_finder',
            output='screen',
            parameters=[params, {
                'use_sim_time': use_sim_time,
                'target_colour': target_colour,
                'publish_annotated': LaunchConfiguration('publish_annotated'),
            }],
        )]

    # target_pose. Both handover parameters are set HERE and together:
    # point_topic alone feeds the approach a good fix the executive
    # never lets it use, because SEARCH_TARGET gates on the status
    # topic. Setting one without the other is the C2-M4.2 defect, and
    # is why this is one launch argument rather than two parameters an
    # operator is trusted to remember.
    return [Node(
        package='coco_perception',
        executable='target_pose_node',
        name='target_pose_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_colour': target_colour,
            'point_topic': TARGET_TOPIC,
            'status_compat_topic': STATUS_TOPIC,
        }],
    )]


def generate_launch_description():
    """Build the launch description for the perception node."""
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
                        'the camera rate sags. target_finder only — '
                        'target_pose_node does not draw an overlay.'),
        DeclareLaunchArgument(
            'target_source', default_value='target_finder',
            choices=list(TARGET_SOURCES),
            description='Which node owns /perception/target and '
                        '/perception/status. target_finder is the M6 path '
                        'and the default; target_pose is the C2-M4 '
                        'depth+TF pipeline. Exactly one runs — see the '
                        'module docstring.'),

        OpaqueFunction(function=perception_node),
    ])
