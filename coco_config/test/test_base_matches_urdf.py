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
