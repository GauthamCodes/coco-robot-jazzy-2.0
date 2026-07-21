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
robot.py
========
Facts about the Coco robot and its sensors that more than one package
needs to agree on: where the robot spawns, and how fast each sensor is
configured to publish.

These are physical properties of the model, not tuning knobs, so they are
plain constants rather than ROS parameters. Kept free of ROS imports so
they can be read from launch files, plain scripts and unit tests alike.

Source of truth for the rates is the <update_rate> tags in
gazebo_models/urdf/coco_robo2.xacro (and the controller_manager
update_rate in coco_controllers.yaml for /joint_states).
"""

# Where full_world_robo.launch.py places the robot, in the world frame.
# The arena is walled; this spot faces +x with the ramp ahead and the arm
# pointing back toward the west wall.
SPAWN_XY = (-2.0, 0.0)

# Height used when *spawning* into an empty world: a few cm of clearance so
# the wheels settle onto the ground plane rather than starting interpenetrated.
SPAWN_Z = 0.05

# Nominal publish rate (Hz) and whether the publisher is best-effort.
# Best-effort here means the gz->ROS bridge republishes sensor data with
# sensor QoS, so a reliable subscriber would never match.
SENSOR_TOPICS = {
    # topic: (nominal Hz, best_effort)
    '/joint_states': (100.0, False),   # controller_manager update_rate
    '/scan': (10.0, True),             # lidar <update_rate>
    '/imu': (50.0, True),
    '/camera/image_raw': (15.0, True),
    '/diff_drive_controller/odom': (50.0, False),
    '/model/coco/odometry': (50.0, False),   # gz OdometryPublisher (ground truth)
}


def nominal_hz(topic):
    """Configured publish rate for `topic`, or None if it is not a sensor."""
    entry = SENSOR_TOPICS.get(topic)
    return None if entry is None else entry[0]


def is_best_effort(topic):
    """Whether `topic` needs a best-effort subscription to match."""
    entry = SENSOR_TOPICS.get(topic)
    return False if entry is None else entry[1]
