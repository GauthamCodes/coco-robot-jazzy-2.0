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
costmap observation source (C2-NAV.43). No sensor is added: two installed
image_pipeline nodes turn the bridged depth image into a cloud.

  /camera/depth/image_raw (32FC1, 320x240) + /camera/camera_info
      -> image_proc resize_node (x0.5, nearest)  -> /camera/depth/half/image_raw
                                                   /camera/depth/half/camera_info
      -> depth_image_proc point_cloud_xyz_node   -> /camera/depth/points

All three decisions below were measured on the live simulator
(docs/agents/C2-NAV.43_RESULTS.md, Part I):

1. Not the bridged /camera/points. gz-sim's rgbd cloud is stamped
   camera_optical_frame but its points are in the x-forward LINK convention
   (1.5 mm median error against that projection, 0.66 m against optical); a
   costmap transforming it through the optical frame puts floor points up to
   1.6 m in the air. The depth image projected through camera_info in
   camera_optical_frame lands the floor at |z| <= 5.3 mm and the ramp on its
   wedge surface to 5 mm.

2. Half resolution. At full resolution the cloud is 1.23 MB, and a
   best-effort subscriber -- the policy nav2_costmap_2d's ObstacleLayer uses
   -- received it once in 12 s: a sample that large fragments and is dropped
   whole. The 307 KB depth image arrived at 15.0 Hz on the same transport. At
   x0.5 the cloud is 19200 points, the depth image's size, and its spacing at
   the costmap's 2.5 m obstacle range is ~20 mm, under the 50 mm cell.
   Nearest-neighbour, so no depth is interpolated across an edge; resize_node
   scales the intrinsics with the image.

3. camera_info. image_transport derives an info topic from the image topic,
   /camera/depth/camera_info, which nothing publishes; the bridge publishes the
   camera's one camera_info on /camera/camera_info. That absolute name is
   remapped rather than bridging the same message twice.

Both nodes subscribe lazily -- only while something reads their output -- so
this costs nothing unless a costmap source is configured to read the cloud.

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
HALF_IMAGE_TOPIC = '/camera/depth/half/image_raw'
HALF_INFO_TOPIC = '/camera/depth/half/camera_info'
SCALE = 0.5
INTER_NEAREST = 0  # cv::InterpolationFlags


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='image_proc',
            executable='resize_node',
            name='depth_cloud_resize',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'use_scale': True,
                         'scale_width': SCALE,
                         'scale_height': SCALE,
                         'interpolation': INTER_NEAREST}],
            remappings=[
                ('image/image_raw', DEPTH_IMAGE_TOPIC),
                ('/camera/depth/camera_info', CAMERA_INFO_TOPIC),
                ('resize/image_raw', HALF_IMAGE_TOPIC),
                ('resize/camera_info', HALF_INFO_TOPIC),
            ],
        ),
        Node(
            package='depth_image_proc',
            executable='point_cloud_xyz_node',
            name='depth_cloud_xyz',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('image_rect', HALF_IMAGE_TOPIC),
                ('points', DEPTH_CLOUD_TOPIC),
            ],
        ),
    ])
