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
depth_cloud.launch.py
=====================
An optional PointCloud2 built from the robot's EXISTING rgbd_camera, for a
costmap observation source (C2-NAV.43). No sensor is added: the cloud is
computed from the bridged depth image and camera_info by the installed
depth_image_proc::PointCloudXyzNode.

  /camera/depth/image_raw (32FC1) + /camera/camera_info
      -> depth_image_proc point_cloud_xyz_node -> /camera/depth/points

Why not the bridged /camera/points, which already exists: measured on the live
simulator (C2-NAV.43, docs/agents/C2-NAV.43_RESULTS.md), gz-sim's rgbd point
cloud carries frame_id camera_optical_frame but its points are in the
x-forward LINK convention -- 1.5 mm median error against that projection,
0.66 m against the optical one. A costmap transforming it through the optical
frame puts floor points up to 1.6 m in the air. It also arrived at under 5 Hz
against the depth image's 15. The depth image projected through camera_info
in camera_optical_frame lands the floor at |z| <= 5.3 mm and the ramp on its
wedge surface to 5 mm, so that is what this node publishes.

camera_info: image_transport derives the info topic from the image topic,
/camera/depth/camera_info, which nothing publishes; the bridge publishes the
camera's single camera_info on /camera/camera_info. That absolute name is
remapped rather than bridging the same message twice.

The node subscribes lazily -- only while something subscribes to its output --
so it costs nothing unless a costmap source is configured to read it.

Started by `nav.launch.py depth_cloud:=true`; standalone:

  ros2 launch gazebo_models depth_cloud.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Must equal nav_params_overlay.DEPTH_CLOUD_TOPIC; a test compares them.
DEPTH_CLOUD_TOPIC = '/camera/depth/points'
DEPTH_IMAGE_TOPIC = '/camera/depth/image_raw'
CAMERA_INFO_TOPIC = '/camera/camera_info'


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='depth_image_proc',
            executable='point_cloud_xyz_node',
            name='depth_cloud_xyz',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('image_rect', DEPTH_IMAGE_TOPIC),
                ('/camera/depth/camera_info', CAMERA_INFO_TOPIC),
                ('points', DEPTH_CLOUD_TOPIC),
            ],
        ),
    ])
