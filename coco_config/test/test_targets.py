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
The fetch targets and the camera must agree with the URDF that ships them.

TARGETS moved here from full_world_robo.launch.py, which CMake installs to
share/ and is therefore not importable — so the lane values had been
copied as bare literals into three separate scripts. One table now, and
these tests are what stop it drifting from the model again.

Two couplings are silent when broken and expensive to debug:

- a target whose model name has no <xacro:magnet model="..."> spawns with
  no magnet at all, so the grasp reports success and lifts nothing;
- CAMERA_XYZ/RPY are what the perception node deprojects through. Wrong
  by the 13.5 mm base_footprint offset and every reported target position
  is wrong by the same amount, with nothing in the output looking odd.

The xacro is located by repo layout rather than the ament index on
purpose: a test_depend on gazebo_models would close a dependency cycle
(gazebo_models exec_depends custom_teleop, which depends on coco_config)
and colcon would refuse to order the packages. Same reasoning as
test_limits_match_urdf.py.
"""
from pathlib import Path
import re

from coco_config.robot import (CAMERA_HFOV, CAMERA_RPY, CAMERA_WH, CAMERA_XYZ,
                               PLATFORM_LEN, RAMP_SUMMIT_X, RAMP_WIDTH,
                               TARGET_COLOURS, TARGET_ROW_X, TARGETS,
                               camera_intrinsics, colour_for_lane,
                               lane_for_colour, target_by_colour)

import pytest

# .../coco_config/test/this_file -> parents[2] is the repo root
URDF = (Path(__file__).resolve().parents[2]
        / 'gazebo_models' / 'urdf' / 'coco_robo2.xacro')

# Half the robot's x-footprint (wheel joints at +-0.09, radius 0.0585).
HALF_FOOTPRINT_X = 0.1485


def _urdf_text():
    assert URDF.is_file(), f'URDF not found at {URDF}'
    return URDF.read_text()


def _joint_origin(text, joint):
    """Return (xyz, rpy) tuples for a fixed joint's <origin>."""
    match = re.search(
        r'<joint\s+name="%s"[^>]*>\s*<origin\s+xyz="([^"]+)"\s+rpy="([^"]+)"'
        % re.escape(joint), text)
    assert match, f'no <origin> found for joint {joint}'
    return (tuple(float(v) for v in match.group(1).split()),
            tuple(float(v) for v in match.group(2).split()))


# ── the table itself ─────────────────────────────────────────────────────
def test_colours_and_models_are_unique():
    assert len(set(TARGET_COLOURS)) == len(TARGETS)
    assert len({t.model for t in TARGETS}) == len(TARGETS)


def test_lanes_are_distinct_and_evenly_spaced():
    """
    Even spacing is what makes a lane miss diagnosable.

    The RL policy's measured lateral drift is reported against these, so
    an uneven row would make "one lane off" mean different things in
    different places.
    """
    lanes = sorted(t.lane_y for t in TARGETS)
    gaps = [b - a for a, b in zip(lanes, lanes[1:])]
    assert gaps == pytest.approx([gaps[0]] * len(gaps))
    assert gaps[0] > 0.0


def test_every_lane_fits_on_the_platform():
    """
    A lane the robot cannot stand in is not a lane.

    The platform is RAMP_WIDTH across, so the robot's half-footprint has
    to clear the edge in the outermost lane.
    """
    limit = RAMP_WIDTH / 2.0 - HALF_FOOTPRINT_X
    for target in TARGETS:
        assert abs(target.lane_y) < limit, (
            f'{target.colour} lane {target.lane_y:+.2f} leaves '
            f'{limit - abs(target.lane_y):.3f} m to the platform edge')


def test_the_target_row_is_on_the_platform():
    """The row has to be somewhere the robot can drive up to and stop."""
    assert RAMP_SUMMIT_X < TARGET_ROW_X < RAMP_SUMMIT_X + PLATFORM_LEN


def test_lane_lookup_round_trips():
    for colour in TARGET_COLOURS:
        assert colour_for_lane(lane_for_colour(colour)) == colour


def test_an_unknown_colour_is_none_not_a_default_lane():
    """
    Returning a lane for a colour we do not have would drive somewhere.

    The sequencer sends Nav2 to whatever this returns, so a silent
    fallback to lane 0 would look like a working mission that fetched the
    wrong thing.
    """
    assert lane_for_colour('purple') is None
    assert target_by_colour('purple') is None


def test_a_y_between_lanes_is_not_claimed_by_either():
    midpoint = (TARGETS[0].lane_y + TARGETS[1].lane_y) / 2.0
    assert colour_for_lane(midpoint) is None


# ── couplings to the URDF ────────────────────────────────────────────────
def test_every_target_has_a_magnet_in_the_urdf():
    """
    A model with no magnet macro grasps nothing, silently.

    DetachableJoint binds by model name at spawn; a name only present in
    one of the two files produces an object that can never be picked up
    and a grasp that reports success.
    """
    text = _urdf_text()
    magnets = set(re.findall(r'<xacro:magnet\s+model="([^"]+)"', text))
    assert magnets, 'no <xacro:magnet> instances found — parser is stale'
    missing = {t.model for t in TARGETS} - magnets
    assert not missing, f'targets with no magnet in the URDF: {sorted(missing)}'


def test_camera_pose_matches_the_urdf_chain():
    """
    CAMERA_XYZ is camera_joint composed with base_footprint_joint.

    Both joints are fixed with zero rotation, so the composition is a
    plain sum — but it is a sum, and dropping the 13.5 mm base_footprint
    term biases every deprojected target by exactly that much.
    """
    text = _urdf_text()
    cam_xyz, cam_rpy = _joint_origin(text, 'camera_joint')
    base_xyz, base_rpy = _joint_origin(text, 'base_footprint_joint')
    assert base_rpy == pytest.approx((0.0, 0.0, 0.0)), (
        'base_footprint_joint has rotated; CAMERA_XYZ is no longer a sum')
    composed = tuple(a + b for a, b in zip(cam_xyz, base_xyz))
    assert CAMERA_XYZ == pytest.approx(composed)
    assert CAMERA_RPY == pytest.approx(cam_rpy)


def test_the_camera_is_deliberately_unpitched():
    """
    Guard the decision, not just the value.

    An earlier plan called pitching the camera down the highest-value
    change in the project. It is not: positive pitch is nose-DOWN in
    URDF, and the visible band gains a finite far cutoff as soon as the
    pitch exceeds the half-vertical-FOV, which is inside the range the
    mission classifies at. If someone changes this, they should have to
    change this test and read why.
    """
    import math
    _, fy, _, cy = camera_intrinsics()
    half_vfov = math.atan(cy / fy)
    assert abs(CAMERA_RPY[1]) <= half_vfov, (
        f'camera pitch {CAMERA_RPY[1]:+.3f} exceeds the half-vfov '
        f'{half_vfov:.3f}, which puts a hard far limit on detection')


def test_camera_intrinsics_match_the_urdf_sensor():
    """fx is derived from <horizontal_fov> and <width>; keep them in step."""
    text = _urdf_text()
    sensor = re.search(r'<sensor\s+name="camera".*?</sensor>', text, re.S)
    assert sensor, 'no camera <sensor> block found'
    body = sensor.group(0)
    hfov = float(re.search(r'<horizontal_fov>([^<]+)</horizontal_fov>',
                           body).group(1))
    width = int(re.search(r'<width>([^<]+)</width>', body).group(1))
    height = int(re.search(r'<height>([^<]+)</height>', body).group(1))
    assert CAMERA_HFOV == pytest.approx(hfov)
    assert CAMERA_WH == (width, height)

    fx, fy, cx, cy = camera_intrinsics()
    assert fx == pytest.approx(fy), 'gz builds square pixels'
    assert (cx, cy) == pytest.approx((width / 2.0, height / 2.0))
