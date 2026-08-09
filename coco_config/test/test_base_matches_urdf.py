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

"""The base constants must stay in step with the xacro and the controller.

Same argument as test_limits_match_urdf.py, and now it earns its keep
twice over: M7 adds a SECOND simulator generated from these numbers, so a
drift here no longer shows up as a wrong URDF — it shows up as a
sim-to-sim transfer gap, which is a far more expensive thing to debug.
M7_DESIGN 5.2 says the MJCF must be generated from coco_config precisely
so there is one place to be wrong.

Parses the xacro and coco_controllers.yaml directly. No ROS environment,
no xacro CLI: every value checked here is a literal in its source file.
The files are located through the repo layout rather than the ament index
because a test_depend on gazebo_models would close a dependency cycle —
see the note in test_limits_match_urdf.py.
"""
from pathlib import Path
import re

from coco_config.robot import (
    CHASSIS_MASS, WHEEL_MASS, WHEEL_RADIUS, WHEEL_SEPARATION,
    WHEEL_SEPARATION_MULTIPLIER, WHEEL_WIDTH, WHEELBASE)

import pytest

ROOT = Path(__file__).resolve().parents[2]
URDF = ROOT / 'gazebo_models' / 'urdf' / 'coco_robo2.xacro'
CONTROLLERS = ROOT / 'gazebo_models' / 'urdf' / 'coco_controllers.yaml'


def _xacro_property(name):
    """Value of <xacro:property name="..." value="..."/>, as a float."""
    assert URDF.is_file(), f'URDF not found at {URDF}'
    match = re.search(
        rf'<xacro:property\s+name="{name}"\s+value="([^"]+)"', URDF.read_text())
    assert match, f'no xacro property {name!r} in {URDF.name}'
    return float(match.group(1))


def _controller_value(key):
    """Value of `key: <number>` in the diff-drive controller yaml."""
    assert CONTROLLERS.is_file(), f'controllers yaml not found at {CONTROLLERS}'
    match = re.search(rf'^\s*{key}:\s*([0-9.]+)\s*$',
                      CONTROLLERS.read_text(), re.M)
    assert match, f'no {key!r} in {CONTROLLERS.name}'
    return float(match.group(1))


def _wheel_joint_origins():
    """{joint: (x, y, z)} for the four continuous wheel joints."""
    text = URDF.read_text()
    found = {}
    for match in re.finditer(
            r'<joint\s+name="(base_Revolute-[1-4])"\s+type="continuous"\s*>'
            r'\s*<origin\s+xyz="([^"]+)"', text):
        found[match.group(1)] = tuple(
            float(v) for v in match.group(2).split())
    return found


def test_the_parsers_find_something():
    """Guard the regexes: a parser that found nothing would make every
    comparison below vacuously pass, which is the failure mode this whole
    file exists to prevent."""
    assert _xacro_property('wheel_radius') > 0
    assert _controller_value('wheel_radius') > 0
    assert len(_wheel_joint_origins()) == 4, (
        f'expected 4 wheel joints, parsed {sorted(_wheel_joint_origins())}')


def test_wheel_radius_matches_both_sources():
    """The xacro draws the wheel; the controller does odometry with it.
    They must agree with each other AND with coco_config."""
    assert WHEEL_RADIUS == pytest.approx(_xacro_property('wheel_radius'))
    assert WHEEL_RADIUS == pytest.approx(_controller_value('wheel_radius'))


def test_wheel_width_and_mass_match_the_xacro():
    assert WHEEL_WIDTH == pytest.approx(_xacro_property('wheel_width'))
    assert WHEEL_MASS == pytest.approx(_xacro_property('wheel_mass'))


def test_wheel_separation_matches_the_controller():
    assert WHEEL_SEPARATION == pytest.approx(
        _controller_value('wheel_separation'))


def test_wheel_separation_matches_the_joint_geometry():
    """Derive the track from where the wheels actually are.

    The controller value is a parameter someone typed; this is the
    geometry. base_Revolute-1 and -3 are the front pair, and in the
    chassis frame they are separated along z.
    """
    origins = _wheel_joint_origins()
    front_right = origins['base_Revolute-1']
    front_left = origins['base_Revolute-3']
    assert WHEEL_SEPARATION == pytest.approx(
        abs(front_left[2] - front_right[2]), abs=1e-9)


def test_wheelbase_matches_the_joint_geometry():
    origins = _wheel_joint_origins()
    front = origins['base_Revolute-1']
    rear = origins['base_Revolute-2']
    assert WHEELBASE == pytest.approx(abs(rear[0] - front[0]), abs=1e-9)


def test_chassis_mass_matches_the_xacro():
    """base_link's <mass>, which is the first one in the file."""
    match = re.search(r'<mass\s+value="([^"]+)"', URDF.read_text())
    assert match, 'no <mass> found in the xacro'
    assert CHASSIS_MASS == pytest.approx(float(match.group(1)))


def test_the_separation_multiplier_is_recorded():
    """Not a fudge factor to forget about.

    A second simulator built to the physical track will not reproduce
    Gazebo's yaw response, because the controller commands against
    separation * multiplier. Recording it is what makes that difference a
    known quantity rather than a surprise in the transfer table.
    """
    assert WHEEL_SEPARATION_MULTIPLIER == pytest.approx(
        _controller_value('wheel_separation_multiplier'))


# ── added in M7 Phase 2 ──────────────────────────────────────────────────
# Ground clearance and total mass were not written down anywhere until a
# world full of obstacles needed them, and the MJCF had drifted to 2.1x the
# real clearance and 11% light without anything noticing. These pin both to
# the xacro so that cannot recur.

def _all_link_masses():
    """{link_name: mass} for every link in the xacro, wheels resolved.

    The `[^/]` before the closing bracket matters. Without it the pattern
    also matched SELF-CLOSING links -- `<link name="camera_optical_frame"/>`
    -- and, finding no `</link>` of their own, ran on to the next one and
    swallowed the following link's body. That attributed imu_link's 10 g
    to camera_optical_frame, which is why coco_config carried
    CAMERA_MASS = 0.040 commented "camera_link + camera_optical_frame"
    for a whole phase with a passing test underneath it. The total stayed
    correct, so only the ATTRIBUTION was wrong -- and attribution is the
    entire point of placing these lumps separately in the MJCF.
    """
    text = URDF.read_text()
    found = {}
    for match in re.finditer(r'<link name="([^"]+)"[^/]*?>(.*?)</link>',
                             text, re.S):
        name, body = match.groups()
        m = re.search(r'<mass value="([^"]+)"', body)
        if not m:
            continue
        raw = m.group(1)
        found[name] = (WHEEL_MASS if 'wheel_mass' in raw else float(raw))
    return found


def test_the_mass_parser_does_not_run_past_a_self_closing_link():
    """Guard the guard. camera_optical_frame has no <inertial> at all, so
    it must not appear here; imu_link has one, so it must."""
    m = _all_link_masses()
    assert 'camera_optical_frame' not in m
    assert m['imu_link'] == pytest.approx(0.010, abs=1e-9)
    assert m['camera_link'] == pytest.approx(0.030, abs=1e-9)


def test_total_mass_matches_the_sum_of_urdf_links():
    """The whole robot, not just the bits the MJCF happens to model.

    A base-only model is 11% light and light in the wrong places: the arm
    is rearward and the lidar is on a mast, and this is the mass that sets
    normal force, friction and the tipping moment.
    """
    from coco_config.robot import TOTAL_MASS
    masses = _all_link_masses()
    # The xacro declares one wheel link via a macro; the robot has four.
    total = sum(masses.values()) + 3 * WHEEL_MASS
    assert TOTAL_MASS == pytest.approx(total, abs=1e-6), (
        f'coco_config TOTAL_MASS {TOTAL_MASS} vs xacro sum {total}')


def test_the_non_wheel_masses_are_accounted_for_individually():
    """Each lumped mass must equal the links it stands in for."""
    from coco_config.robot import (ARM_CHAIN_MASS, CAMERA_MASS, IMU_MASS,
                                   LIDAR_MASS)
    m = _all_link_masses()
    arm = sum(m[k] for k in
              ('m_link1', 'm_link2', 'm_link3', 'grip1', 'grip2'))
    assert ARM_CHAIN_MASS == pytest.approx(arm, abs=1e-6)
    assert LIDAR_MASS == pytest.approx(m['lidar_link'], abs=1e-6)
    assert CAMERA_MASS == pytest.approx(m['camera_link'], abs=1e-6)
    assert IMU_MASS == pytest.approx(m['imu_link'], abs=1e-6)


def test_ground_clearance_derives_from_the_chassis_box_and_the_axles():
    """13.5 mm, and every obstacle in M7_DESIGN is measured against it.

    Re-derived here from the xacro rather than trusted: the collision box
    origin maps through chassis_joint's +pi/2 roll to base_link z = 0.030
    with half-height 0.030, so its underside is at base_link 0.000; the
    axles are at base_link 0.045 and the wheels are WHEEL_RADIUS, so the
    ground is at base_link -WHEEL_RADIUS + 0.045.
    """
    from coco_config.robot import (AXLE_Z_IN_BASE_LINK,
                                   CHASSIS_GROUND_CLEARANCE, CHASSIS_SIZE)
    text = URDF.read_text()
    box = re.search(
        r'<collision name="chassis_collision">\s*<origin xyz="([^"]+)"'
        r'.*?<box size="([^"]+)"', text, re.S)
    assert box, 'chassis collision box not found in the xacro'
    ox, oy, oz = (float(v) for v in box.group(1).split())
    sx, sy, sz = (float(v) for v in box.group(2).split())

    # chassis_joint: roll +pi/2 about x maps (x, y, z) -> (x, -z, y),
    # then translate by (0.12, -0.08, 0).
    centre_z_in_base_link = oy
    half_height = sy / 2.0          # the roll swaps the box's y and z
    underside = centre_z_in_base_link - half_height
    ground = AXLE_Z_IN_BASE_LINK - WHEEL_RADIUS

    assert CHASSIS_GROUND_CLEARANCE == pytest.approx(
        underside - ground, abs=1e-9), (
        f'clearance {CHASSIS_GROUND_CLEARANCE} disagrees with the xacro '
        f'({underside - ground})')
    assert CHASSIS_SIZE[2] == pytest.approx(sy, abs=1e-9)


def test_axle_height_matches_the_wheel_joint_origins():
    from coco_config.robot import AXLE_Z_IN_BASE_LINK
    # chassis-frame (x, y, z) -> base_link z is the joint's y component.
    origins = _wheel_joint_origins()
    for name, xyz in origins.items():
        assert AXLE_Z_IN_BASE_LINK == pytest.approx(xyz[1], abs=1e-9), (
            f'{name} axle at base_link z {xyz[1]}, config says '
            f'{AXLE_Z_IN_BASE_LINK}')
