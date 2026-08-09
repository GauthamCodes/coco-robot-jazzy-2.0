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
full_world_robo.launch.py — Jazzy + Gazebo Harmonic (Layer 1)
=============================================================
Starts:
  1. Gazebo Harmonic with coco_world (walled arena + obstacles)
  2. Coco robot (coco_robo2.xacro -> URDF), spawned upright on a z-up
     base_link — no more roll-90 spawn hack
  3. Static ramp
  4. ros_gz_bridge (/clock)
  5. ros2_control: diff_drive_controller (all 4 wheels),
     arm_controller + gripper_controller (JointTrajectoryController —
     holds position on activation, so the arm no longer free-swings
     at spawn and no "home publisher" hack is needed)

Usage:
  ros2 launch gazebo_models full_world_robo.launch.py
  ros2 launch gazebo_models full_world_robo.launch.py gui:=false
"""

import math
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from coco_config.robot import (PLATFORM_LEN, RAMP_ANGLE_DEG, RAMP_FOOT_X,
                               RAMP_RUN, RAMP_SUMMIT_X, RAMP_WIDTH, SPAWN_XY,
                               SPAWN_Z, TARGET_MASS, TARGET_ROW_X, TARGETS)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui').perform(context).lower() in ('true', '1')
    ramp_angle = int(float(LaunchConfiguration('ramp_angle').perform(context)))
    traverse = LaunchConfiguration('traverse').perform(context).lower() \
        in ('true', '1', 'yes')

    pkg_share  = get_package_share_directory('gazebo_models')
    xacro_path = os.path.join(pkg_share, 'urdf', 'coco_robo2.xacro')
    ramp_path  = os.path.join(pkg_share, 'urdf', 'ramp.sdf')
    # The world is selectable so terrain properties can be swept WITHOUT
    # editing coco_world.world, which is frozen: v1's 10/10 traverse and
    # 19/20 fetch matrix are measurements against that exact file. The
    # default is unchanged, so every existing command line behaves
    # identically.
    #
    # A bare name resolves inside the package's worlds/ directory; an
    # absolute path is taken as-is, which is what a generated variant in
    # /tmp needs. Missing files fail here rather than inside gz, where the
    # symptom is an empty world and a robot falling forever.
    world_arg = LaunchConfiguration('world').perform(context)
    world_file = (world_arg if os.path.isabs(world_arg)
                  else os.path.join(pkg_share, 'worlds', world_arg))
    if not os.path.exists(world_file):
        raise RuntimeError(f'world file not found: {world_file}')
    mesh_uri   = 'file://' + os.path.join(pkg_share, 'meshes') + '/'

    # Robot description: package:// URIs for RViz/robot_state_publisher,
    # absolute file:// URIs for Gazebo spawning.
    robot_xml = xacro.process_file(xacro_path).toxml()
    robot_xml_gz = robot_xml.replace('package://gazebo_models/meshes/', mesh_uri)

    # ramp.sdf ships the default (18 deg) wedge; swap in the curriculum grade
    # by filename. Each curriculum angle has a committed
    # meshes/ramp_wedge_<deg>.stl (gen_ramp.py); fail loudly rather than
    # spawn a missing mesh.
    wedge_stl = f'ramp_wedge_{ramp_angle}.stl'
    if not os.path.exists(os.path.join(pkg_share, 'meshes', wedge_stl)):
        raise RuntimeError(
            f'no wedge mesh for ramp_angle={ramp_angle} ({wedge_stl}). '
            f'Generate one: ros2 run gazebo_models gen_ramp.py '
            f'--angle-deg {ramp_angle} --run {RAMP_RUN} --width {RAMP_WIDTH} '
            f'--out <pkg>/meshes/{wedge_stl}')
    with open(ramp_path) as f:
        ramp_xml = (f.read()
                    .replace('ramp_wedge_18.stl', wedge_stl)
                    .replace('package://gazebo_models/meshes/', mesh_uri))

    gz_args = ('-r -v2 ' if gui else '-r -s -v2 ') + world_file
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_xml,
            'use_sim_time': use_sim_time,
        }],
    )

    spawn_coco = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_coco',
        arguments=[
            '-name', 'coco',
            '-string', robot_xml_gz,
            '-x', str(SPAWN_XY[0]), '-y', str(SPAWN_XY[1]), '-z', str(SPAWN_Z),
        ],
        output='screen',
    )

    # The wedge's local origin is its foot edge (x=0, z=0), rising +x. Spawn it
    # so the foot meets the ground at world x=RAMP_FOOT_X centred on y=0; the
    # summit sits at RAMP_SUMMIT_X (=3.0, inside the east wall), leaving the
    # west half of the arena free for driving and SLAM.
    spawn_ramp = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_ramp',
        arguments=[
            '-name', 'ramp',
            '-string', ramp_xml,
            '-x', str(RAMP_FOOT_X), '-y', '0.0', '-z', '0.0',
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_share, 'config', 'bridge.yaml'),
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager-timeout', '120'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        )
        for name in [
            'joint_state_broadcaster',
            'diff_drive_controller',
            'arm_controller',
            'gripper_controller',
        ]
    ]

    # ── optional traverse: up-slope + flat top + down-slope ──────────────────
    # The wedge is a right triangular prism, so past the crest there is a
    # VERTICAL drop (0.53 m at 12 deg, 1.11 m at 24 deg). That is fine for a
    # climb task -- the RL episode deliberately finishes GOAL_MARGIN short of
    # the crest for exactly this reason -- but it makes "climb it and come back
    # down" impossible: there is nothing to descend, and turning round means a
    # 180 deg skid-steer pivot on a 2 m ledge above the drop.
    #
    # traverse:=true adds a flat platform at the crest and a mirrored wedge
    # descending the far side, giving a continuous up-over-down route:
    #
    #     foot            crest      platform      crest       foot
    #     x=1.0 ---------- 3.0 ========= 4.5 ---------------- 6.5
    #            up-slope         flat           down-slope
    #
    # The far foot at x=6.5 clears the east wall at x=8.0 by 1.5 m. The four
    # fetch targets stand on the platform at TARGET_ROW_X = 4.05.
    #
    # Note GOAL_MARGIN's reasoning does NOT apply in this configuration:
    # there is no drop past the crest here, only 1.5 m of level platform. The
    # RL goal still stops at world x=2.700 because the policy was verified
    # against that goal, and the mission's approach_server owns everything
    # from there to the target row rather than the goal being moved.
    extra = []
    if traverse:
        rise = RAMP_RUN * math.tan(math.radians(ramp_angle))
        plat_len = PLATFORM_LEN
        plat_x = RAMP_SUMMIT_X + plat_len / 2.0
        # Flat top, so the robot crests on level ground instead of pivoting over
        # a knife edge where the two slopes would otherwise meet.
        platform_sdf = f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="ramp_platform">
    <static>true</static>
    <link name="link">
      <collision name="c"><geometry><box>
        <size>{plat_len} {RAMP_WIDTH} {rise}</size></box></geometry></collision>
      <visual name="v"><geometry><box>
        <size>{plat_len} {RAMP_WIDTH} {rise}</size></box></geometry>
        <material><ambient>0.6 0.6 0.62 1</ambient>
                  <diffuse>0.6 0.6 0.62 1</diffuse></material></visual>
    </link>
  </model>
</sdf>'''
        extra.append(Node(
            package='ros_gz_sim', executable='create', name='spawn_platform',
            arguments=['-name', 'ramp_platform', '-string', platform_sdf,
                       '-x', str(plat_x), '-y', '0.0', '-z', str(rise / 2.0)],
            output='screen'))
        # The four fetch targets, in LANES ACROSS Y on the platform (which
        # spans x 3.0..4.5). Not a row along x: the robot arrives travelling
        # +x, so reaching the third object in a row would mean driving
        # THROUGH the first two — 40-100 mm tall against a chassis with no
        # such clearance — and no ordering lets the operator pick freely.
        # With lanes the sequencer maps colour to a lane, sends Nav2 to a
        # flat-ground pre-ramp pose (0.5, lane_y), and the policy climbs
        # straight into the right lane. Zero pivoting on the ledge.
        #
        # Outer lane to platform edge: 0.50 m. Between lanes: 0.50 m.
        # (Both were 0.29/0.33 while the ramp was 2.0 m wide; it is 2.5 m
        # now, and the stale figures outlived the change.)
        #
        # These are 158 mm TALL, not the 60 mm they started at, and that
        # is a reach fix rather than a cosmetic one. The arm reaches to
        # base-x 0.1299 at the 30 mm height a 60 mm cylinder grasps at,
        # while the chassis ends at 0.120 — so the 24 mm and 30 mm
        # targets had a NEGATIVE approach window and could not be grasped
        # at all, and the other two had under 4 mm. At 158 mm the grasp
        # band lands at coco_config's TARGET_GRASP_Z, which is exactly
        # pick_place.py's verified pinch point, and every window is ~27 mm.
        # See coco_config/test/test_reach.py.
        for target in TARGETS:
            radius = target.diameter / 2.0
            height = target.height
            # Solid cylinder about its centre of mass. The old literals
            # (8e-6/8e-6/3e-6) were sized for a 0.02 kg, 60 mm object and
            # would understate ixx by ~13x at this height, which reads as
            # an implausibly twitchy object rather than as a wrong number.
            i_xx = TARGET_MASS * (3.0 * radius ** 2 + height ** 2) / 12.0
            i_zz = TARGET_MASS * radius ** 2 / 2.0
            target_sdf = f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{target.model}">
    <link name="link">
      <inertial><mass>{TARGET_MASS}</mass>
        <inertia><ixx>{i_xx:.6e}</ixx><iyy>{i_xx:.6e}</iyy>
                 <izz>{i_zz:.6e}</izz>
                 <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <collision name="c"><geometry><cylinder>
        <radius>{radius}</radius><length>{height}</length></cylinder></geometry>
        <surface><friction><ode><mu>1.5</mu><mu2>1.5</mu2></ode></friction></surface>
      </collision>
      <visual name="v"><geometry><cylinder>
        <radius>{radius}</radius><length>{height}</length></cylinder></geometry>
        <material><ambient>{target.rgb} 1</ambient>
                  <diffuse>{target.rgb} 1</diffuse></material></visual>
    </link>
  </model>
</sdf>'''
            extra.append(Node(
                package='ros_gz_sim', executable='create',
                name=f'spawn_{target.model}',
                arguments=['-name', target.model, '-string', target_sdf,
                           '-x', str(TARGET_ROW_X), '-y', str(target.lane_y),
                           '-z', str(rise + height / 2.0)],
                output='screen'))

        # Release all four immediately. The DetachableJoint plugin attaches
        # its child the instant the model appears — there is no SDF option
        # to start detached — so without this the robot spawns welded to
        # four objects six metres away and CANNOT TURN: measured, a
        # commanded -0.3 rad/s for 6 s moved yaw 0.000 -> 0.000 welded
        # versus 0.000 -> -1.342 detached. Translation still works, which
        # is what makes it such a confusing failure. See magnet_release.py.
        extra.append(Node(
            package='gazebo_models', executable='magnet_release.py',
            name='magnet_release', output='screen',
            arguments=['--models'] + [t.model for t in TARGETS]))

        # Mirrored wedge: yaw pi flips its local +x, so placing its foot at
        # far_foot puts its crest back at the platform's far edge.
        far_foot = RAMP_SUMMIT_X + plat_len + RAMP_RUN
        extra.append(Node(
            package='ros_gz_sim', executable='create', name='spawn_ramp_down',
            arguments=['-name', 'ramp_down', '-string', ramp_xml,
                       '-x', str(far_foot), '-y', '0.0', '-z', '0.0',
                       '-Y', str(math.pi)],
            output='screen'))

    return [gz_sim, rsp, spawn_coco, spawn_ramp, bridge] + extra + spawners


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use Gazebo simulation clock'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Set false for headless/server-only mode'),
        DeclareLaunchArgument(
            'traverse', default_value='false',
            description='add a flat crest platform and a mirrored down-slope, '
                        'turning the climb into an up-over-down traverse. The '
                        'plain wedge ends in a vertical drop, so this is what '
                        'makes "carry something back down" physically possible.'),
        DeclareLaunchArgument(
            'world', default_value='coco_world.world',
            description='World file: a bare name resolves in the package '
                        'worlds/ directory, an absolute path is used as '
                        'given. Exists so terrain properties can be swept '
                        'without editing the frozen coco_world.world.'),
        DeclareLaunchArgument('ramp_angle', default_value=str(RAMP_ANGLE_DEG),
                              description='Ramp grade in degrees; selects '
                                          'meshes/ramp_wedge_<deg>.stl '
                                          '(curriculum: 12, 18, 24)'),
        OpaqueFunction(function=launch_setup),
    ])
