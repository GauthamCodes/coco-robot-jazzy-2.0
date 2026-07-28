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
Every fetch target must sit somewhere the arm can actually reach.

This is the test that would have caught the M5 blocker at design time
instead of at M6. The four targets were spawned as 60 mm cylinders
standing on the platform, which puts their grasp band at base-z 0.030.
The arm reaches to base-x 0.1299 at that height and the chassis
collision box ends at 0.120, so the approach window
``[CHASSIS_FRONT_X + radius, x_max(grasp_z)]`` was:

    red    12 mm   +3.9 mm
    green  18 mm   +0.9 mm
    blue   24 mm   -2.1 mm   impossible
    yellow 30 mm   -5.1 mm   impossible

Two of the mission's four objects could not be picked up at all, and the
other two needed the base to stop within 4 mm. Nothing said so: the
world spawned, the camera would have seen them, and MoveIt would simply
have reported an unreachable goal at the very end of the mission.

Nothing about that is visible from either file alone — the reach comes
from coco_moveit_config's arm_ik, the geometry from coco_config, and the
chassis bound from the URDF. It only shows up when the three are
multiplied together, which is what this does.

arm_ik is imported by repo path rather than as a package: it ships as an
installed PROGRAM (see coco_moveit_config/CMakeLists.txt), and a
test_depend on that package would point coco_config at something that
depends on gazebo_models, which depends back on coco_config. Same
reasoning as test_limits_match_urdf.py.
"""
from pathlib import Path
import sys

from coco_config.robot import (CHASSIS_FRONT_X, GRASP_APPROACH_X,
                               TARGET_GRASP_Z, TARGET_HEIGHT, TARGETS)

import pytest

# .../coco_config/test/this_file -> parents[2] is the repo root
_SCRIPTS = (Path(__file__).resolve().parents[2]
            / 'coco_moveit_config' / 'scripts')
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

arm_ik = pytest.importorskip('arm_ik')

# pick_place.py approaches from HOVER_CLEARANCE above the grasp pose.
HOVER_CLEARANCE = 0.07

# The base has to stop with the target's axis inside the window. 15 mm is
# not generous for a diff-drive stopping tolerance; it is the floor below
# which the geometry stops being the thing worth arguing about.
MIN_WINDOW = 0.015

# Daylight between the palm and the chassis nose. The measured MoveIt
# boundary in the pedestal case (every point at x >= 0.1505 planned, every
# point at x <= 0.1468 was rejected for palm-vs-pedestal contact) lands
# within 0.5 mm of CHASSIS_FRONT_X + 0.025 + 0.005, which is where this
# comes from.
PALM_MARGIN = 0.005


def x_max(z, lo=-0.30, hi=0.50, iterations=200):
    """Furthest forward base-x the pinch point can reach at height `z`."""
    for _ in range(iterations):
        mid = (lo + hi) / 2.0
        if arm_ik.ik(mid, z):
            lo = mid
        else:
            hi = mid
    return lo


def window(target, grasp_z=TARGET_GRASP_Z):
    """(near, far) base-x the target's axis may occupy, in metres."""
    near = CHASSIS_FRONT_X + target.diameter / 2.0 + PALM_MARGIN
    return near, x_max(grasp_z)


def test_the_verified_grasp_pose_is_still_reachable():
    """
    (0.152, 0.128) is the one grasp geometry with measured lifts.

    arm_ik's own docstring pins fk(0.30, 0.58) to it, and pick_place.py
    solves its hover from it. If this fails, the arm model changed and
    every measured grasp result in docs/RESULTS.md is void.
    """
    assert arm_ik.ik(GRASP_APPROACH_X, TARGET_GRASP_Z), (
        f'({GRASP_APPROACH_X}, {TARGET_GRASP_Z}) is unreachable')
    q_shoulder, q_elbow = arm_ik.ik(GRASP_APPROACH_X, TARGET_GRASP_Z)[0]
    assert arm_ik.fk(q_shoulder, q_elbow) == pytest.approx(
        (GRASP_APPROACH_X, TARGET_GRASP_Z), abs=1e-4)


def test_the_approach_is_reachable_from_the_hover_above_it():
    """
    The descent is vertical, so both ends have to be in the envelope.

    A reachable grasp under an unreachable hover is a plan that cannot be
    started, which MoveIt reports the same way as an unreachable grasp.
    """
    assert arm_ik.ik(GRASP_APPROACH_X, TARGET_GRASP_Z + HOVER_CLEARANCE)


@pytest.mark.parametrize('target', TARGETS, ids=lambda t: t.colour)
def test_every_target_has_a_usable_approach_window(target):
    near, far = window(target)
    assert far - near >= MIN_WINDOW, (
        f'{target.colour} (d={target.diameter * 1000:.0f} mm): window '
        f'[{near:.4f}, {far:.4f}] is {(far - near) * 1000:+.1f} mm')


@pytest.mark.parametrize('target', TARGETS, ids=lambda t: t.colour)
def test_the_demo_approach_x_lies_inside_every_window(target):
    """
    One approach pose has to work for all four, or the demo needs four.

    pick_place.py drives to a single GRASP_APPROACH_X; if a target's
    window excluded it, that target would need its own tuned stop
    distance and the "one grasp server, parameterised by width" plan for
    M6 would not hold.
    """
    near, far = window(target)
    assert near <= GRASP_APPROACH_X <= far, (
        f'{target.colour}: {GRASP_APPROACH_X} outside [{near:.4f}, {far:.4f}]')


def test_targets_standing_on_the_platform_would_be_unreachable():
    """
    Pin the regression this file exists for.

    A 60 mm cylinder on the platform grasps at base-z 0.030, where reach
    collapses to 0.1299 — behind the chassis nose for anything 24 mm or
    thicker. If someone shortens the targets back down, this fails and
    says why rather than leaving it to be discovered during a grasp.
    """
    short_grasp_z = 0.030
    assert x_max(short_grasp_z) < CHASSIS_FRONT_X + 0.012, (
        'reach at the old 60 mm grasp height is no longer the constraint '
        'it was — re-derive TARGET_HEIGHT rather than trusting this file')


def test_the_grasp_band_sits_below_the_top_of_the_target():
    """
    The magnet binds the palm to the body, not to a rim.

    A grasp band at or above the top would have the palm meeting the
    cylinder's end face, and the descent would knock it over rather than
    slide alongside it.
    """
    assert TARGET_GRASP_Z < TARGET_HEIGHT
    assert TARGET_HEIGHT - TARGET_GRASP_Z >= 0.02, (
        f'only {(TARGET_HEIGHT - TARGET_GRASP_Z) * 1000:.0f} mm of body '
        f'above the grasp band')


def test_the_hover_clears_the_top_of_the_target():
    """The open gripper must start above the object, not beside it."""
    assert TARGET_GRASP_Z + HOVER_CLEARANCE > TARGET_HEIGHT
