"""
full_world_robo.launch.py
=========================
Main launch file for the Coco robot simulation.

Starts Gazebo, robot_state_publisher, spawns coco robot + ramp platform,
and activates all ros2_control arm controllers.

Usage:
  ros2 launch gazebo_models full_world_robo.launch.py
  ros2 launch gazebo_models full_world_robo.launch.py gui:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
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
        description='Use simulation (Gazebo) clock',
    )
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Launch Gazebo with GUI',
    )

    pkg_path = get_package_share_directory('gazebo_models')
    coco_urdf = os.path.join(pkg_path, 'urdf', 'coco_robo2.urdf')
    ramp_urdf = os.path.join(pkg_path, 'urdf', 'abs.urdf')
    mesh_path = os.path.join(pkg_path, 'meshes')

    # Tell Gazebo where to find mesh files
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=mesh_path,
    )

    # ── Gazebo ──────────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py',
            )
        ),
        launch_arguments={'gui': gui}.items(),
    )

    # ── Robot State Publisher ───────────────────────────────────────────────
    with open(coco_urdf, 'r') as f:
        robot_description_content = f.read()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }],
    )

    # ── Spawn robot ─────────────────────────────────────────────────────────
    spawn_coco = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_coco',
        arguments=[
            '-entity', 'coco',
            '-file', coco_urdf,
            '-x', '0.0', '-y', '0.0', '-z', '0.3',
        ],
        output='screen',
    )

    # ── Spawn ramp ──────────────────────────────────────────────────────────
    spawn_ramp = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_ramp',
        arguments=[
            '-entity', 'ramp',
            '-file', ramp_urdf,
            '-x', '1.0', '-y', '0.0', '-z', '0.0',
        ],
        output='screen',
    )

    # ── ros2_control arm controllers ────────────────────────────────────────
    controller_names = [
        'joint_state_broadcaster',
        'm_link1_controller',
        'm_link2_controller',
        'm_link3_controller',
        'm_link3_Revolute_9_controller',
    ]

    delayed_controllers = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_coco,
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[
                        Node(
                            package='controller_manager',
                            executable='spawner',
                            arguments=[controller],
                            output='screen',
                        )
                        for controller in controller_names
                    ],
                )
            ],
        )
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_gui,
        set_gazebo_model_path,
        gazebo,
        rsp,
        spawn_coco,
        spawn_ramp,
        delayed_controllers,
    ])
