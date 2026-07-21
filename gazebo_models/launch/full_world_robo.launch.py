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

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from coco_config.robot import SPAWN_XY, SPAWN_Z
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui').perform(context).lower() in ('true', '1')

    pkg_share  = get_package_share_directory('gazebo_models')
    xacro_path = os.path.join(pkg_share, 'urdf', 'coco_robo2.xacro')
    ramp_path  = os.path.join(pkg_share, 'urdf', 'ramp.sdf')
    world_file = os.path.join(pkg_share, 'worlds', 'coco_world.world')
    mesh_uri   = 'file://' + os.path.join(pkg_share, 'meshes') + '/'

    # Robot description: package:// URIs for RViz/robot_state_publisher,
    # absolute file:// URIs for Gazebo spawning.
    robot_xml = xacro.process_file(xacro_path).toxml()
    robot_xml_gz = robot_xml.replace('package://gazebo_models/meshes/', mesh_uri)

    with open(ramp_path) as f:
        ramp_xml = f.read().replace('package://gazebo_models/meshes/', mesh_uri)

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

    # The ramp mesh extends 4.4 m in -x and 2.6 m in +y from its own origin;
    # this pose puts the structure at x 1.1..5.5 centred on y=0, keeping the
    # west half of the arena free for driving and SLAM.
    spawn_ramp = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_ramp',
        arguments=[
            '-name', 'ramp',
            '-string', ramp_xml,
            '-x', '5.5', '-y', '-1.3', '-z', '0.0',
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

    return [gz_sim, rsp, spawn_coco, spawn_ramp, bridge] + spawners


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use Gazebo simulation clock'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Set false for headless/server-only mode'),
        OpaqueFunction(function=launch_setup),
    ])
