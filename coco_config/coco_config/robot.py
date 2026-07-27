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

# Ramp geometry — the single source of truth shared by the launch file (where
# to spawn the wedge) and coco_rl (where the summit is, so the RL goal is the
# real top of the climb rather than a bare distance). The mesh is a clean
# parametric wedge from gazebo_models/scripts/gen_ramp.py; foot at RAMP_FOOT_X
# rising +x to the summit. RAMP_ANGLE_DEG is the default/nominal grade; the
# curriculum relaunches with ramp_angle:=12|18|24 (each has a committed mesh).
#
# Keep RAMP_ANGLE_DEG below ~30: on the ramp the robot pitches nose-up by
# roughly the grade, and coco_rl's tip-over terminator fires at 0.6 rad
# (~34 deg), so a steeper wedge would read the climb itself as a fall.
# RAMP_SUMMIT_X stays inside the east wall (x=8).
RAMP_FOOT_X = 1.0        # world x where the ramp foot meets the ground (z=0)
# RUN was 2.5 and WIDTH 2.0. Both changed for the fetch mission's
# up-over-down traverse: the crest carries a 1.5 m platform now, and with a
# 2.5 m run the mirrored down-ramp's far foot would land at x=7.5, leaving
# 0.5 m to the east wall — not enough for the robot (0.297 m x-footprint) to
# turn around after descending. At 2.0 m the far foot is 6.5 m and there is
# 1.5 m of room. The width went up because the four target objects sit in
# lanes across y on the platform.
RAMP_RUN = 2.0           # horizontal length of the wedge (m)
RAMP_WIDTH = 2.5         # width across the wedge (m), centred on y=0
RAMP_ANGLE_DEG = 18      # default grade; matches meshes/ramp_wedge_18.stl
RAMP_SUMMIT_X = RAMP_FOOT_X + RAMP_RUN   # world x of the crest (= 3.0)

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
