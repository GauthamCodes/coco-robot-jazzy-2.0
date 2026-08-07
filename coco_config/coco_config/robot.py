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
# The arena is walled; this spot faces +x with the ramp ahead. The arm is
# MOUNTED at the rear (shoulder pivot at base-x -0.075) but works forward:
# arm_ik.fk puts the fingertip pinch at base-x +0.152 in the verified grasp
# pose, on the same side as the camera and the ramp.
SPAWN_XY = (-2.0, 0.0)

# Height used when *spawning* into an empty world: a few cm of clearance so
# the wheels settle onto the ground plane rather than starting interpenetrated.
SPAWN_Z = 0.05

# ── the base, as physics sees it ──────────────────────────────────────────
# These were previously readable only out of the xacro and the controller
# yaml, which was fine while Gazebo was the only simulator. M7 adds a second
# one, and two hand-maintained robot models diverge within a week — the
# divergence then presents as a mysterious sim-to-sim transfer gap rather
# than as the transcription error it actually is. So they live here, once,
# and coco_sim generates the MJCF from them.
#
# test_base_matches_urdf.py parses the xacro and coco_controllers.yaml and
# fails if any of these drifts from its source.
#
#   WHEEL_RADIUS      coco_robo2.xacro <xacro:property name="wheel_radius">
#                     and coco_controllers.yaml diff_drive wheel_radius
#   WHEEL_WIDTH       coco_robo2.xacro <xacro:property name="wheel_width">
#   WHEEL_MASS        coco_robo2.xacro <xacro:property name="wheel_mass">.
#                     0.4 kg is deliberate: the CAD auto-density gave 3.74 kg
#                     per wheel, 94% of the robot's mass, which is not a
#                     printed wheel.
#   WHEEL_SEPARATION  the track. base_Revolute-1 sits at chassis-frame
#                     z = +0.057 and -3 at z = -0.217; the difference is
#                     0.274, and coco_controllers.yaml says the same.
#   WHEELBASE         front-to-rear spacing: base_Revolute-1 at x = -0.03
#                     against -2 at x = -0.21.
#   CHASSIS_MASS      the <mass> on base_link in the xacro.
#   CHASSIS_SIZE      the chassis collision box, expressed in the BASE frame.
#                     The xacro writes it as 0.24 x 0.06 x 0.274 in the
#                     chassis frame, which chassis_joint's pi/2 roll maps to
#                     (x 0.24, y 0.274, z 0.06) here — the same rotation that
#                     puts the box's front face at CHASSIS_FRONT_X = 0.120.
WHEEL_RADIUS = 0.0585        # m
WHEEL_WIDTH = 0.04           # m
WHEEL_MASS = 0.4             # kg, per wheel; there are four
WHEEL_SEPARATION = 0.274     # m, left-right track
WHEELBASE = 0.18             # m, front-rear
CHASSIS_MASS = 1.041325435922023          # kg
CHASSIS_SIZE = (0.24, 0.274, 0.06)        # m, in the base frame

# THE NUMBER THAT DECIDES WHETHER AN OBSTACLE IS DRIVABLE, and it was not
# written down anywhere until M7 Phase 2 needed it. Derived:
#
#   chassis_joint rolls the CAD frame by +pi/2 about x and translates by
#   (0.12, -0.08, 0), which maps the collision box origin (-0.12, 0.03,
#   -0.08) to base_link (0, 0, 0.030) and its 0.24 x 0.06 x 0.274 extent to
#   (x 0.24, y 0.274, z 0.06). Half-height 0.030, so the UNDERSIDE sits at
#   base_link z = 0.000.
#   The four wheel joints land at base_link (+/-0.09, +/-0.137, 0.045)
#   -- the xacro says so in a comment -- and the wheels are 0.0585 in
#   radius, so the ground plane is base_link z = -0.0135.
#
# Underside minus ground = 13.5 mm. For scale: M7_DESIGN's original
# 40 mm washboard would pitch the chassis nose 48.7 mm down into this.
CHASSIS_GROUND_CLEARANCE = 0.0135         # m

# Everything that is not a wheel, lumped by where it actually sits, because
# the CoM height is what decides tipping on a cambered route. Summing the
# xacro's <mass> tags: chassis 1.041325 + arm chain 0.210179 + lidar 0.080
# + camera 0.040 = 1.371505 kg, and with 4 x 0.4 of wheel the whole robot
# is 2.971505 kg. A base-only model is 11 % light and light in exactly the
# wrong places -- the arm is rearward, the lidar is on a mast.
#
# Positions are the joint origins in base_link, from the xacro:
#   arm     m_link1_Revolute-6  (-0.075, 0.0075, 0.075)
#   lidar   lidar_joint         (-0.09,  0.10,   0.20)
#   camera  camera_joint        ( 0.125, 0,      0.055)
ARM_CHAIN_MASS = 0.210179     # m_link1+m_link2+m_link3+grip1+grip2
LIDAR_MASS = 0.080
CAMERA_MASS = 0.040           # camera_link + camera_optical_frame
ARM_MOUNT_XYZ = (-0.075, 0.0075, 0.075)
LIDAR_MOUNT_XYZ = (-0.09, 0.10, 0.20)
CAMERA_MOUNT_XYZ = (0.125, 0.0, 0.055)

# Axle height above base_link, i.e. the offset between base_link and the
# frame a MuJoCo model naturally roots at (wheel centres). Kept explicit so
# the MJCF does not re-derive it.
AXLE_Z_IN_BASE_LINK = 0.045

TOTAL_MASS = (CHASSIS_MASS + ARM_CHAIN_MASS + LIDAR_MASS + CAMERA_MASS
              + 4 * WHEEL_MASS)          # 2.971505 kg

# The diff_drive controller does NOT command the physical track. It applies
# wheel_separation_multiplier to it, so a commanded yaw rate is computed
# against an effective 0.3014 m. That is a skid-steer fudge: four wheels
# scrubbing sideways turn slower than a two-wheel differential model
# predicts, and the multiplier compensates. It matters here because a
# second simulator built to the PHYSICAL track will not reproduce Gazebo's
# yaw response unless it either applies the same multiplier or is compared
# on straight-line motion only.
WHEEL_SEPARATION_MULTIPLIER = 1.10

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


# ── where the arm can actually work ──────────────────────────────────────
# The grasp geometry the pick-and-place demo measured 32-40 mm lifts at,
# and the two bounds that make it the only one available.
#
# arm_ik puts the fingertip pinch at (0.152, 0.128) for joint angles
# (0.30, 0.58) — the demo's verified grasp pose — and the arm's forward
# reach at that height is base-x 0.1608. The chassis collision box ends
# at base-x 0.120 (its 0.24 x 0.06 x 0.274 box maps through chassis_joint's
# pi/2 rotation to x in [-0.120, +0.120]), so a target's axis has to stop
# somewhere in between:
#
#     [CHASSIS_FRONT_X + radius,  x_max(grasp_z)]
#
# That window is ~27 mm at grasp_z = 0.128. At grasp_z = 0.030 — where a
# 60 mm cylinder standing on the platform puts its centre — reach falls to
# 0.1299 and the window is +3.9 mm for the 12 mm target, +0.9 mm for the
# 18 mm one, and NEGATIVE for the 24 mm and 30 mm ones. The four mission
# targets were, as originally spawned, two-thirds impossible to grasp and
# one-third a coin flip. test_reach.py is what stops that recurring.
CHASSIS_FRONT_X = 0.120   # base-x of the chassis collision box's front face
GRASP_APPROACH_X = 0.152  # base-x of the pinch point in the verified pose
TARGET_GRASP_Z = 0.128    # base-z of the same, i.e. where the magnet binds

# Daylight between the palm and the chassis nose, on top of the target's
# own radius. Measured rather than guessed: in the pedestal case every
# MoveIt goal at x >= 0.1505 planned and every one at x <= 0.1468 was
# rejected for palm-vs-pedestal contact, and that boundary lands within
# 0.5 mm of CHASSIS_FRONT_X + 0.025 + 0.005.
PALM_MARGIN = 0.005

# THE BOUND THAT ACTUALLY BITES, and it is not about the target at all.
# The arm has no wrist, so reaching a pinch point closer to the base means
# curling the forearm back over the chassis. Probed against move_group's
# /check_state_validity at the grasp height, solving arm_ik at 1 mm steps:
#
#     x <= 0.1440   chassis_link/m_link2 AND chassis_link/m_link3
#     x <= 0.1490   chassis_link/m_link2
#     x >= 0.1500   valid
#
# pick_place.py's POSES comment has always said "anything deeper than about
# [0.30, 0.58] clips m_link2/m_link3 into the chassis box"; this is that
# sentence with an x on it. It matters because the first end-to-end fetch
# stopped at 0.1443 — comfortably inside the window computed from chassis
# clearance and reach alone — and had its grasp rejected before physics ran.
#
# SELF_COLLISION_MARGIN is 1 mm because the probe's resolution is 1 mm.
GRASP_SELF_COLLISION_X = 0.150
SELF_COLLISION_MARGIN = 0.001

# The grasp descends vertically onto the target from this far above it.
GRASP_HOVER_CLEARANCE = 0.07

# How far off the arm's y=0 working plane the target may be and still be
# worth grasping. The arm is planar — it cannot reach sideways at all — so
# this is less a tolerance than a refusal to weld something the robot is
# not actually in front of.
#
# It lives here rather than in grasp_server.py because the APPROACH is
# what has to hit it and the GRASP is what enforces it, and those are two
# packages. 10 mm is the sum of the approach's three blind-leg error
# sources, with margin:
#
#     perception residual, measured in M5          ~2.0 mm
#     ALIGN_EPS over the 0.166 m blind creep        3.3 mm
#     wheel odometry at ~1% over the same leg       1.7 mm
#                                                  -------
#                                                   7.0 mm
#
# Note this is the LATERAL budget and is unrelated to the 5.5 mm approach
# window above, which is along-axis. Conflating the two is easy and
# wrong: a heading error at the end of the align moves the target
# sideways, and costs only 0.03 mm of range.
GRASP_MAX_LATERAL = 0.010

# Furthest forward the pinch point reaches, and the number that bounds the
# approach. It is the reach at the HOVER height, not at the grasp height:
# arm_ik reaches base-x 0.16085 at TARGET_GRASP_Z but only 0.15651 at
# TARGET_GRASP_Z + GRASP_HOVER_CLEARANCE, and the descent needs BOTH ends
# in the envelope. Using the grasp-height figure would advertise 4.3 mm of
# window that cannot actually be entered — a plan that fails at its start
# state, which MoveIt reports identically to an unreachable goal.
# test_reach.py pins this against arm_ik by bisection.
GRASP_REACH_X_MAX = 0.1565

# The targets are TALL rather than pedestal-mounted so the grasp band
# lands at TARGET_GRASP_Z with nothing else nearby: a plinth would put a
# static obstacle in the lane the robot has to drive through on the
# up-over-down descent, and would reinstate the palm-vs-pedestal planning
# collision that item 5b in FUTURE_WORK is about. A cylinder leaves with
# the robot.
TARGET_HEIGHT = 0.158     # 30 mm of body above the grasp band
TARGET_MASS = 0.05        # kg; 0.118 N.m at full reach vs a 10 N.m limit

# The fetch mission's four platform targets, in LANES ACROSS Y. Everything
# that needs to agree about them reads this: the spawner builds them,
# magnet_release detaches them by name, the sequencer maps a chosen colour
# to a lane, and the perception node classifies by colour. A model name
# here with no matching <xacro:magnet model="..."> in coco_robo2.xacro
# would spawn with no magnet at all, so a test asserts the two agree.
#
# Lanes are 0.5 m apart and the outermost is 0.5 m from the platform edge
# (RAMP_WIDTH/2 = 1.25).
#
# Diameters were 12/18/24/30 mm. They are 20/24/28/32 now: the small end
# rose because a 12 mm x 158 mm cylinder tips at 4.3 deg and would not
# survive the robot arriving beside it, and the whole set shifted up
# because the approach window grows as the target gets thinner only until
# the chassis bound bites. All four stay inside the gripper's 6-32 mm
# band, and 28 mm is exactly pick_place.py's pick_target, so the measured
# lift stays the reference. 32 mm is the tightest descent clearance
# (~5.5 mm interpolated against the 28 mm case's verified 7.76 mm) and is
# the one to drop to 30 mm if the gripper knocks it over.
TARGET_ROW_X = 4.05      # world x of the row, 0.45 m from the platform's far edge
TARGETS = (
    Target('red',    'target_red',    0.020, TARGET_HEIGHT,
           '0.85 0.10 0.10', -0.75),
    Target('green',  'target_green',  0.024, TARGET_HEIGHT,
           '0.10 0.70 0.15', -0.25),
    Target('blue',   'target_blue',   0.028, TARGET_HEIGHT,
           '0.10 0.25 0.85',  0.25),
    Target('yellow', 'target_yellow', 0.032, TARGET_HEIGHT,
           '0.90 0.80 0.10',  0.75),
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


def approach_window(colour):
    """
    Base-x range the target's axis may occupy for a graspable approach.

    Returns ``(near, far)`` in metres, or None for an unknown colour.

    Three constraints, and the third dominates all four targets:

    - the cylinder must clear the chassis box: ``CHASSIS_FRONT_X + radius
      + PALM_MARGIN``, which is 0.135-0.141 depending on thickness;
    - the arm must reach it from the hover above it, ``GRASP_REACH_X_MAX``;
    - **the forearm must not curl into the chassis**,
      ``GRASP_SELF_COLLISION_X``, measured at 0.150.

    So the window is [0.151, 0.1565] — **5.5 mm, and identical for all
    four colours**. The diameters stopped mattering the moment the
    self-collision bound was measured, which is worth knowing before
    anyone tunes something per-colour: at the working depth this arm has
    one grasp pose, not four.
    """
    target = target_by_colour(colour)
    if target is None:
        return None
    near = max(CHASSIS_FRONT_X + target.diameter / 2.0 + PALM_MARGIN,
               GRASP_SELF_COLLISION_X + SELF_COLLISION_MARGIN)
    return near, GRASP_REACH_X_MAX


def approach_stop_x(colour):
    """
    Where the base should stop: the CENTRE of the approach window.

    Not GRASP_APPROACH_X, though the two are within 2 mm now that the
    self-collision bound is in the window — which is itself the point.
    0.152 was never arbitrary; it is a pose that happens to sit inside a
    5.5 mm band, and it is worth stopping in the middle of that band
    rather than 1 mm from its near edge, because the approach's last
    0.13 m is blind (perception's minimum range) and the margin is needed
    on both sides.

    Nothing is lost by not landing exactly on 0.152: the grasp pose is
    solved from wherever the target actually ends up (arm_ik of the
    measured x), exactly as pick_place.retarget() already does.
    """
    window = approach_window(colour)
    return None if window is None else (window[0] + window[1]) / 2.0


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
