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
needs to agree on: where the robot spawns, how fast each sensor is
configured to publish, where the camera looks from, and what the four
fetch targets are.

These are physical properties of the model, not tuning knobs, so they are
plain constants rather than ROS parameters. Kept free of ROS imports so
they can be read from launch files, plain scripts and unit tests alike.

Source of truth for the rates is the <update_rate> tags in
gazebo_models/urdf/coco_robo2.xacro (and the controller_manager
update_rate in coco_controllers.yaml for /joint_states).
"""

import math
from typing import NamedTuple

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

# Flat crest between the up-slope and the mirrored down-slope. The robot
# has to stand on this to reach the targets, so it is 1.5 m rather than the
# 0.5 m it started at. Spans world x RAMP_SUMMIT_X .. RAMP_SUMMIT_X+PLATFORM_LEN
# (= 3.0 .. 4.5) and the full RAMP_WIDTH across y.
PLATFORM_LEN = 1.5


class Target(NamedTuple):
    """One fetch target: what it looks like, how big it is, which lane."""

    colour: str      # perception key; what the phone publishes
    model: str       # gz model name; needs a magnet macro in coco_robo2.xacro
    diameter: float  # m
    height: float    # m
    rgb: str         # SDF <ambient>/<diffuse> triple
    lane_y: float    # world y of this target's lane


# The fetch mission's four platform targets, in LANES ACROSS Y. Everything
# that needs to agree about them reads this: the spawner builds them,
# magnet_release detaches them by name, the sequencer maps a chosen colour
# to a lane, and the perception node classifies by colour. A model name
# here with no matching <xacro:magnet model="..."> in coco_robo2.xacro
# would spawn with no magnet at all, so a test asserts the two agree.
#
# Lanes are 0.5 m apart and the outermost is 0.5 m from the platform edge
# (RAMP_WIDTH/2 = 1.25).
TARGET_ROW_X = 4.05      # world x of the row, 0.45 m from the platform's far edge
TARGETS = (
    Target('red',    'target_red',    0.012, 0.06, '0.85 0.10 0.10', -0.75),
    Target('green',  'target_green',  0.018, 0.06, '0.10 0.70 0.15', -0.25),
    Target('blue',   'target_blue',   0.024, 0.06, '0.10 0.25 0.85',  0.25),
    Target('yellow', 'target_yellow', 0.030, 0.06, '0.90 0.80 0.10',  0.75),
)

TARGET_COLOURS = tuple(t.colour for t in TARGETS)


def target_by_colour(colour):
    """The Target with this colour, or None if it is not one of ours."""
    for target in TARGETS:
        if target.colour == colour:
            return target
    return None


def lane_for_colour(colour):
    """World y of the lane holding `colour`, or None.

    This is the whole of the mission's colour->lane mapping: the sequencer
    sends Nav2 to the flat-ground pre-ramp pose in this lane BEFORE the
    climb, and the policy then drives straight up into it. Nothing on the
    platform is visible from the flat (the crest edge occludes it), so the
    lane cannot be chosen by looking — it has to come from this table.
    """
    target = target_by_colour(colour)
    return None if target is None else target.lane_y


def colour_for_lane(lane_y, tol=0.1):
    """Which target's lane `lane_y` is, or None if it is between lanes."""
    for target in TARGETS:
        if abs(target.lane_y - lane_y) <= tol:
            return target.colour
    return None


# ── camera ───────────────────────────────────────────────────────────────
# Pose of camera_link in base_footprint, composing coco_robo2.xacro's
# camera_joint (0.125 0 0.055 on base_link) with base_footprint_joint
# (0 0 0.0135). A test asserts this composition still matches the xacro.
#
# The rpy is (0, 0, 0) DELIBERATELY. An earlier plan called pitching the
# camera down "the highest-value single change in the project"; it is not.
# Positive pitch is nose-down in URDF, and the visible band's far limit is
# infinite only while |pitch| <= half-vfov (0.496 rad). A 0.6 rad pitch
# inserts a cutoff at 0.37 m for a target on the ground plane, which is
# inside the range the mission classifies at, and buys nothing: the arm
# reaches to base-x 0.1617 while the nearest visible ground at that pitch
# is 0.160, so "see the grasp workspace" is a ~1 mm sliver. Classify at
# range and approach open-loop; see docs/DESIGN_DECISIONS.md.
CAMERA_XYZ = (0.125, 0.0, 0.0685)
CAMERA_RPY = (0.0, 0.0, 0.0)
CAMERA_WH = (320, 240)
CAMERA_HFOV = 1.25          # rad, <horizontal_fov> in the xacro
CAMERA_DEPTH_CLIP = (0.1, 8.0)   # m, <depth_camera><clip> in the xacro


def camera_intrinsics():
    """Pinhole (fx, fy, cx, cy) in pixels, derived from the xacro's FOV.

    gz builds a square-pixel camera from <horizontal_fov> and the image
    size, so fy == fx. Use /camera/camera_info when it is available; this
    is the offline fallback that lets geometry be unit-tested without a
    running simulator.
    """
    width, height = CAMERA_WH
    fx = (width / 2.0) / math.tan(CAMERA_HFOV / 2.0)
    return fx, fx, width / 2.0, height / 2.0


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
