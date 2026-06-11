"""
full_world_robo.launch.py  —  Jazzy + Gazebo Harmonic port
===========================================================
Replaces the Classic gazebo_ros stack with ros_gz_sim + gz_ros2_control.

Changes from Humble/Classic:
  - gz sim launched via ros_gz_sim/launch/gz_sim.launch.py
  - Robots spawned with ros_gz_sim create (not spawn_entity.py)
  - ros_gz_bridge bridges /cmd_vel and /odom between ROS and gz
  - URDF plugin: gz_ros2_control-system (not libgazebo_ros2_control.so)
  - URDF plugin: gz-sim-diff-drive-system (not libgazebo_ros_diff_drive.so)

Usage:
  ros2 launch gazebo_models full_world_robo.launch.py
  ros2 launch gazebo_models full_world_robo.launch.py gui:=false
"""

import os
import tempfile
import atexit
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
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
        description='Set false for headless/server-only mode',
    )

    pkg_path  = get_package_share_directory('gazebo_models')
    urdf_path = os.path.join(pkg_path, 'urdf', 'coco_robo2.urdf')
    ramp_urdf = os.path.join(pkg_path, 'urdf', 'abs.urdf')
    world_file = os.path.join(pkg_path, 'worlds', 'coco_world.world')
    mesh_path  = os.path.join(pkg_path, 'meshes')
    ctrl_yaml  = os.path.join(pkg_path, 'urdf', 'coco_arm_controller.yaml')

    # Patch robot URDF: resolve controller yaml path + fix mesh URIs
    with open(urdf_path) as f:
        robot_xml = f.read()
    robot_xml = robot_xml.replace(
        '$(find gazebo_models)/urdf/coco_arm_controller.yaml', ctrl_yaml
    ).replace(
        'package://gazebo_models/meshes/', 'file://' + mesh_path + '/'
    )
    tmp_robot = tempfile.NamedTemporaryFile(mode='w', suffix='_coco.urdf', delete=False)
    tmp_robot.write(robot_xml); tmp_robot.flush(); tmp_robot.close()
    atexit.register(lambda: os.unlink(tmp_robot.name) if os.path.exists(tmp_robot.name) else None)

    # Patch ramp URDF mesh paths
    with open(ramp_urdf) as f:
        ramp_xml = f.read()
    ramp_xml = ramp_xml.replace(
        'package://gazebo_models/meshes/', 'file://' + mesh_path + '/'
    )
    tmp_ramp = tempfile.NamedTemporaryFile(mode='w', suffix='_ramp.urdf', delete=False)
    tmp_ramp.write(ramp_xml); tmp_ramp.flush(); tmp_ramp.close()
    atexit.register(lambda: os.unlink(tmp_ramp.name) if os.path.exists(tmp_ramp.name) else None)

    # Gazebo Harmonic via ros_gz_sim
    # -r = run simulation on start; omit -s to include GUI (add -s for headless)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            'gz_args': world_file + ' -r',
            'on_exit_shutdown': 'true',
        }.items(),
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

    # Spawn coco robot via ros_gz_sim create
    spawn_coco = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_coco',
        arguments=[
            '-entity', 'coco',
            '-file', tmp_robot.name,
            '-x', '-2.0', '-y', '0.0', '-z', '0.15',
            '-R', '1.5707963', '-P', '0.0', '-Y', '0.0',
        ],
        output='screen',
    )

    # Spawn ramp
    spawn_ramp = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_ramp',
        arguments=[
            '-entity', 'ramp',
            '-file', tmp_ramp.name,
            '-x', '3.0', '-y', '0.0', '-z', '0.0',
        ],
        output='screen',
    )

    # Bridge: /cmd_vel (ROS→gz), /odom (gz→ROS), /clock (gz→ROS)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
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

    # Delay controller spawning to let gz_ros2_control initialise
    spawn_controllers = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[name],
                output='screen',
            )
            for name in controller_names
        ],
    )

    arm_home = TimerAction(
        period=13.0,
        actions=[
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
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_gui,
        gz_sim,
        rsp,
        spawn_coco,
        spawn_ramp,
        bridge,
        spawn_controllers,
        arm_home,
    ])
