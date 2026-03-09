"""
full_world_robo.launch.py  -  Layer 1 final
============================================
Uses the working string-patch approach (no Xacro) + world file.
Patches coco_robo2.urdf at runtime:
  - Resolves $(find ...) controller yaml path
  - Converts package:// mesh URIs to file:// for Gazebo Classic

Usage:
  ros2 launch gazebo_models full_world_robo.launch.py
  ros2 launch gazebo_models full_world_robo.launch.py gui:=false
"""

import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock',
    )
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Set false for headless mode',
    )

    pkg_path    = get_package_share_directory('gazebo_models')
    urdf_path   = os.path.join(pkg_path, 'urdf', 'coco_robo2.urdf')
    ramp_urdf   = os.path.join(pkg_path, 'urdf', 'abs.urdf')
    world_file  = os.path.join(pkg_path, 'worlds', 'coco_world.world')
    mesh_path   = os.path.join(pkg_path, 'meshes')
    ctrl_yaml   = os.path.join(pkg_path, 'urdf', 'coco_arm_controller.yaml')

    # Patch robot URDF: resolve controller yaml path + fix mesh URIs
    with open(urdf_path) as f:
        robot_xml = f.read()
    robot_xml = robot_xml.replace(
        '$(find gazebo_models)/urdf/coco_arm_controller.yaml', ctrl_yaml
    ).replace(
        'package://gazebo_models/meshes/', 'file://' + mesh_path + '/'
    )
    tmp_robot = tempfile.NamedTemporaryFile(
        mode='w', suffix='_coco.urdf', delete=False
    )
    tmp_robot.write(robot_xml); tmp_robot.flush(); tmp_robot.close()

    # Patch ramp URDF mesh paths
    with open(ramp_urdf) as f:
        ramp_xml = f.read()
    ramp_xml = ramp_xml.replace(
        'file:///home/user/ros2_ws/src/gazebo_models/meshes/',
        'file://' + mesh_path + '/'
    ).replace(
        'package://gazebo_models/meshes/', 'file://' + mesh_path + '/'
    )
    tmp_ramp = tempfile.NamedTemporaryFile(
        mode='w', suffix='_ramp.urdf', delete=False
    )
    tmp_ramp.write(ramp_xml); tmp_ramp.flush(); tmp_ramp.close()

    # Gazebo with structured world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py',
            )
        ),
        launch_arguments={'world': world_file, 'gui': gui}.items(),
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
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_coco',
        arguments=[
            '-entity', 'coco',
            '-file', tmp_robot.name,
            '-x', '-2.0', '-y', '0.0', '-z', '0.15',
            '-R', '1.5707963', '-P', '0.0', '-Y', '0.0',
        ],
        output='screen',
    )

    spawn_ramp = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_ramp',
        arguments=[
            '-entity', 'ramp',
            '-file', tmp_ramp.name,
            '-x', '3.0', '-y', '0.0', '-z', '0.0',
        ],
        output='screen',
    )

    controller_names = [
        'joint_state_broadcaster',
        'm_link1_controller',
        'm_link2_controller',
        'm_link3_controller',
        'm_link3_Revolute_9_controller',
    ]

    arm_home = [
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once',
                 '/m_link1_controller/commands',
                 'std_msgs/msg/Float64MultiArray', '{data: [0.0]}'],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once',
                 '/m_link2_controller/commands',
                 'std_msgs/msg/Float64MultiArray', '{data: [0.0]}'],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once',
                 '/m_link3_controller/commands',
                 'std_msgs/msg/Float64MultiArray', '{data: [0.0]}'],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once',
                 '/m_link3_Revolute_9_controller/commands',
                 'std_msgs/msg/Float64MultiArray', '{data: [0.0]}'],
            output='screen',
        ),
    ]

    delayed_startup = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_coco,
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[
                        Node(
                            package='controller_manager',
                            executable='spawner',
                            arguments=[name],
                            output='screen',
                        )
                        for name in controller_names
                    ],
                ),
                TimerAction(period=9.0, actions=arm_home),
            ],
        )
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_gui,
        gazebo,
        rsp,
        spawn_coco,
        spawn_ramp,
        delayed_startup,
    ])
