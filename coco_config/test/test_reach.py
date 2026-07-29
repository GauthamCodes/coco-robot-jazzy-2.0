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

The window itself now lives in ``coco_config.robot.approach_window`` so
the mission's approach controller and this test cannot disagree about it,
and its far bound is the reach at the HOVER height rather than at the
grasp height — see GRASP_REACH_X_MAX, and the test that pins it.

The near bound turned out not to be about the target at all. Probing
move_group's ``/check_state_validity`` at 1 mm steps found the forearm
folding into the chassis below base-x 0.150, which is 9-15 mm outside
every radius-derived bound above — so all four colours share ONE 5.5 mm
window now. Two tests below pin that, because "the window is the same
for every target" is exactly the kind of fact someone re-derives
per-colour tuning against.

arm_ik is imported by repo path rather than as a package: it ships as an
installed PROGRAM (see coco_moveit_config/CMakeLists.txt), and a
test_depend on that package would point coco_config at something that
depends on gazebo_models, which depends back on coco_config. Same
reasoning as test_limits_match_urdf.py.
"""
from pathlib import Path
import sys

from coco_config.robot import (approach_stop_x, approach_window,
                               CHASSIS_FRONT_X, GRASP_APPROACH_X,
                               GRASP_HOVER_CLEARANCE, GRASP_REACH_X_MAX,
                               GRASP_SELF_COLLISION_X, SELF_COLLISION_MARGIN,
                               TARGET_GRASP_Z, TARGET_HEIGHT, TARGETS)

import pytest

# .../coco_config/test/this_file -> parents[2] is the repo root
_SCRIPTS = (Path(__file__).resolve().parents[2]
            / 'coco_moveit_config' / 'scripts')
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

arm_ik = pytest.importorskip('arm_ik')

# The base has to stop with the target's axis inside the window.
#
# This was 15 mm, chosen when the near bound was CHASSIS_FRONT_X + radius
# and the window looked 15-21 mm wide. It is 5 mm because the real near
# bound is GRASP_SELF_COLLISION_X — the arm folding into its own chassis,
# measured at 0.150 — which leaves 5.5 mm for every colour. Widening this
# constant back out cannot be done by editing it: it would need a wrist,
# a shorter forearm, or a chassis that ends further back.
#
# 5.5 mm is hittable only because the approach creeps the last leg at
# 0.03 m/s and leads its stop by the braking distance; see CREEP_SPEED
# and CREEP_LEAD in custom_teleop/approach_server.py, whose own tests
# assert against this same window.
MIN_WINDOW = 0.005


def x_max(z, lo=-0.30, hi=0.50, iterations=200):
    """Furthest forward base-x the pinch point can reach at height `z`."""
    for _ in range(iterations):
        mid = (lo + hi) / 2.0
        if arm_ik.ik(mid, z):
            lo = mid
        else:
            hi = mid
    return lo


def window(target):
    """(near, far) base-x the target's axis may occupy, in metres."""
    return approach_window(target.colour)


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
    assert arm_ik.ik(GRASP_APPROACH_X,
                     TARGET_GRASP_Z + GRASP_HOVER_CLEARANCE)


def test_the_reach_bound_is_the_hover_height_not_the_grasp_height():
    """
    GRASP_REACH_X_MAX is a hard-coded number; this is what makes it true.

    Two things are asserted, and the second is the one that matters. The
    constant must match arm_ik by bisection, AND it must be the reach at
    the HOVER height rather than at the grasp height — the arm reaches
    4.3 mm further at 0.128 than at 0.198, and a window sized on the
    former would include stops whose descent cannot even be started.
    """
    reach_at_grasp = x_max(TARGET_GRASP_Z)
    reach_at_hover = x_max(TARGET_GRASP_Z + GRASP_HOVER_CLEARANCE)
    assert reach_at_hover < reach_at_grasp, (
        'the hover is no longer the binding height — re-derive '
        'GRASP_REACH_X_MAX rather than trusting this file')
    assert GRASP_REACH_X_MAX == pytest.approx(reach_at_hover, abs=1e-4), (
        f'GRASP_REACH_X_MAX={GRASP_REACH_X_MAX} but arm_ik reaches '
        f'{reach_at_hover:.5f} at the hover height')


@pytest.mark.parametrize('target', TARGETS, ids=lambda t: t.colour)
def test_both_ends_of_the_descent_are_reachable_at_the_stop_pose(target):
    """
    The whole point of stopping at the window centre.

    A stop is only useful if the arm can hover above the target AND reach
    down to it from there. Asserted at the pose the approach actually
    aims for, not at the nominal one.
    """
    stop = approach_stop_x(target.colour)
    assert arm_ik.ik(stop, TARGET_GRASP_Z), f'{target.colour}: no grasp IK'
    assert arm_ik.ik(stop, TARGET_GRASP_Z + GRASP_HOVER_CLEARANCE), (
        f'{target.colour}: no hover IK above the stop pose')


@pytest.mark.parametrize('target', TARGETS, ids=lambda t: t.colour)
def test_the_stop_pose_is_centred_in_its_window(target):
    """
    Margin on both sides, because the last 0.13 m of the approach is blind.

    Perception's minimum range ends ~0.13 m before the stop, so the creep
    that closes it is dead reckoning. Centring converts the lopsided
    -1.0/+4.5 mm budget at GRASP_APPROACH_X into a symmetric +/-2.75 mm.
    """
    near, far = window(target)
    stop = approach_stop_x(target.colour)
    assert stop - near == pytest.approx(far - stop, abs=1e-9)
    assert stop - near >= MIN_WINDOW / 2.0


@pytest.mark.parametrize('target', TARGETS, ids=lambda t: t.colour)
def test_every_target_has_a_usable_approach_window(target):
    near, far = window(target)
    assert far - near >= MIN_WINDOW, (
        f'{target.colour} (d={target.diameter * 1000:.0f} mm): window '
        f'[{near:.4f}, {far:.4f}] is {(far - near) * 1000:+.1f} mm')


@pytest.mark.parametrize('target', TARGETS, ids=lambda t: t.colour)
def test_the_window_clears_the_arms_self_collision_bound(target):
    """
    The bound that actually bites, and the one nothing else would catch.

    The first end-to-end fetch stopped at base-x 0.1443 — comfortably
    inside a window computed from chassis clearance and reach alone — and
    had its grasp REJECTED before any physics ran, because reaching that
    close curls m_link2 into the chassis box. Probed against
    /check_state_validity at 1 mm steps:

        x <= 0.1440   chassis_link/m_link2 AND chassis_link/m_link3
        x <= 0.1490   chassis_link/m_link2
        x >= 0.1500   valid

    arm_ik cannot see this: it solves a 2-link chain and knows nothing
    about the chassis it is bolted to. So this is asserted against the
    measured constant rather than re-derived, and re-measuring it means
    re-running the probe against a live move_group.
    """
    near, _ = window(target)
    assert near >= GRASP_SELF_COLLISION_X, (
        f'{target.colour}: near bound {near:.4f} is inside the measured '
        f'self-collision limit {GRASP_SELF_COLLISION_X}')
    assert near >= GRASP_SELF_COLLISION_X + SELF_COLLISION_MARGIN, (
        f'{target.colour}: near bound {near:.4f} leaves less than the '
        f'{SELF_COLLISION_MARGIN * 1000:.0f} mm probe resolution')


def test_every_colour_shares_one_window():
    """
    At the working depth this arm has one grasp pose, not four.

    The diameters stopped mattering the moment the self-collision bound
    was measured: 0.150 is 9 mm outside the thickest target's chassis
    bound, so max() picks it for all four. This exists so that a future
    attempt to tune the approach per colour fails loudly here instead of
    quietly doing nothing — every branch of such a change would still
    produce four identical numbers.
    """
    windows = {t.colour: window(t) for t in TARGETS}
    stops = {t.colour: approach_stop_x(t.colour) for t in TARGETS}
    assert len(set(windows.values())) == 1, (
        f'windows have diverged by colour: {windows}')
    assert len(set(stops.values())) == 1, (
        f'stops have diverged by colour: {stops}')

    near, far = next(iter(windows.values()))
    assert near == pytest.approx(
        GRASP_SELF_COLLISION_X + SELF_COLLISION_MARGIN, abs=1e-9)
    assert far == pytest.approx(GRASP_REACH_X_MAX, abs=1e-9)


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
    assert TARGET_GRASP_Z + GRASP_HOVER_CLEARANCE > TARGET_HEIGHT
