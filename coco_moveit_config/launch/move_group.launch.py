"""
move_group.launch.py
====================
MoveIt2 move_group for the Coco arm, executing through the ros2_control
JointTrajectoryControllers started by full_world_robo.launch.py.

  ros2 launch gazebo_models full_world_robo.launch.py      # sim first
  ros2 launch coco_moveit_config move_group.launch.py
  ros2 run coco_moveit_config pick_place.py                # scripted demo
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    xacro_path = os.path.join(
        get_package_share_directory('gazebo_models'), 'urdf', 'coco_robo2.xacro')

    moveit_config = (
        MoveItConfigsBuilder('coco', package_name='coco_moveit_config')
        .robot_description(file_path=xacro_path)
        .robot_description_semantic(file_path='config/coco.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                moveit_config.to_dict(),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
