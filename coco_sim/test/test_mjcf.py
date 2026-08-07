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

"""The MJCF must be generated from coco_config, not typed alongside it.

"Generated from coco_config" is easy to claim and easy to violate: paste
one literal into the template and the model still builds, still runs, and
silently stops tracking the robot. test_mjcf_traces_to_config is the
check that would catch that — it rebuilds with a changed constant and
asserts the output moved.
"""
import re

from coco_config.robot import (
    CHASSIS_MASS, WHEEL_MASS, WHEEL_RADIUS, WHEEL_SEPARATION, WHEELBASE)

from coco_sim import mjcf

import pytest


def test_the_model_is_well_formed_xml():
    from xml.etree import ElementTree
    root = ElementTree.fromstring(mjcf.build_mjcf())
    assert root.tag == 'mujoco'


def test_there_are_four_wheels_and_four_actuators():
    """A three-wheeled robot would still load and still drive."""
    text = mjcf.build_mjcf()
    assert len(re.findall(r'type="hinge"', text)) == 4
    assert len(re.findall(r'<velocity ', text)) == 4


def test_wheel_positions_derive_from_wheelbase_and_track():
    positions = dict((name, (x, y)) for name, x, y, _ in
                     mjcf.wheel_positions())
    assert positions['front_right'][0] == pytest.approx(WHEELBASE / 2.0)
    assert positions['rear_right'][0] == pytest.approx(-WHEELBASE / 2.0)
    assert positions['front_left'][1] == pytest.approx(WHEEL_SEPARATION / 2.0)
    assert positions['front_right'][1] == pytest.approx(
        -WHEEL_SEPARATION / 2.0)


def test_masses_are_the_config_masses():
    text = mjcf.build_mjcf()
    assert f'mass="{CHASSIS_MASS:.6f}"' in text
    assert f'mass="{WHEEL_MASS:.6f}"' in text


def test_wheel_radius_is_the_config_radius():
    text = mjcf.build_mjcf()
    assert f'size="{WHEEL_RADIUS:.6f}' in text


@pytest.mark.parametrize('constant,probe', [
    ('WHEEL_RADIUS', 0.0777),
    ('WHEEL_SEPARATION', 0.4242),
    ('WHEELBASE', 0.3131),
    ('WHEEL_MASS', 0.6161),
    ('CHASSIS_MASS', 2.7182),
])
def test_mjcf_traces_to_config(monkeypatch, constant, probe):
    """Change the constant, and the generated model must change with it.

    This is the test that makes "generated from coco_config" a fact
    rather than a comment. A pasted literal passes every other test in
    this file and fails this one.

    It asserts the model *changed*, not that the probe appears verbatim.
    Some constants reach the MJCF derived rather than raw — a track of
    0.4242 becomes wheel offsets of +/-0.2121 — so demanding the literal
    would fail on a correctly-traced value. The property under test is
    traceability, and inequality is exactly that property.
    """
    baseline = mjcf.build_mjcf()
    monkeypatch.setattr(mjcf, constant, probe)
    changed = mjcf.build_mjcf()
    assert changed != baseline, (
        f'{constant} is not actually used to build the MJCF — the model is '
        f'identical after changing it, so something is hard-coded')


def test_a_constant_the_model_does_not_use_would_not_fool_the_trace_test():
    """Guard the guard.

    test_mjcf_traces_to_config only means something if an UNUSED constant
    really does leave the model unchanged. If this fails, that test has
    become vacuous and would pass for a fully hard-coded generator.
    """
    baseline = mjcf.build_mjcf()
    mjcf.__dict__['NOT_USED_BY_THE_MODEL'] = 1.234
    try:
        assert mjcf.build_mjcf() == baseline
    finally:
        del mjcf.__dict__['NOT_USED_BY_THE_MODEL']
