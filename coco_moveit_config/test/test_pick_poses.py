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

"""Pick-and-place waypoints must stay inside the URDF joint limits."""
import copy
import os
import sys

from coco_config.joint_limits import ARM_LIMITS

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import arm_ik  # noqa: E402

# pick_place needs rclpy and moveit_msgs, which are absent wherever the
# workspace is built without the MoveIt prefix — including under
# `colcon test`, where this package's tests run with only /opt/ros and the
# overlay sourced.
#
# This used to be two module-level `pytest.importorskip` calls, and that
# was quietly costing the whole package its test coverage: an importorskip
# that fires during collection aborts the SESSION, not just this file, so
# `pytest test` reported "collected 0 items / 1 skipped" and
# test_arm_ik.py's five tests never ran at all. Worse, pytest exits 5 on an
# empty collection, so ctest recorded coco_moveit_config as FAILED on every
# `colcon test` — a red package whose real cause was a missing optional
# dependency in a different file.
#
# Import defensively instead and mark only the tests that need it. Nothing
# is skipped silently: the reason carries the import error.
try:
    import rclpy                                          # noqa: F401
    import moveit_msgs                                    # noqa: F401
    import pick_place
    _IMPORT_ERROR = None
except ImportError as exc:                                # pragma: no cover
    pick_place = None
    _IMPORT_ERROR = str(exc)

pytestmark = pytest.mark.skipif(
    pick_place is None,
    reason=f'pick_place needs rclpy + moveit_msgs: {_IMPORT_ERROR}')

# Gripper finger limits, from the same table the runtime clamps against
# rather than re-typed here — coco_config's test_limits_match_urdf checks
# that table against coco_robo2.xacro.
GRIP1_LIMITS = ARM_LIMITS['m_link3_Revolute-8']
GRIP2_LIMITS = ARM_LIMITS['m_link3_Revolute-9']

# The target the module ships with, captured at import before any test
# can call retarget().
SHIPPED_TARGET_POS = (tuple(pick_place.TARGET_POS)
                      if pick_place is not None else ())

# Module state that retarget() rewrites in place.
_MUTATED = ('POSES', 'PEDESTAL_SIZE', 'PEDESTAL_POS', 'TARGET_POS',
            'PEDESTAL_SDF')


@pytest.fixture(autouse=True)
def restore_module_state():
    """Undo retarget()'s global mutation after every test.

    retarget() rewrites POSES, PEDESTAL_SIZE, PEDESTAL_POS, TARGET_POS and
    PEDESTAL_SDF in place. Nothing used to put them back, so every test
    that ran after test_retarget_updates_scene_consistently saw a module
    describing a 0.14/0.15 target instead of the shipped one. The suite
    passed only because pytest happened to collect those tests earlier in
    the file; reordering it would have broken them.
    """
    if pick_place is None:                                # pragma: no cover
        yield
        return
    saved = {name: copy.deepcopy(getattr(pick_place, name))
             for name in _MUTATED}
    yield
    for name, value in saved.items():
        setattr(pick_place, name, value)


def test_arm_poses_within_limits():
    for name, (q1, q2) in pick_place.POSES.items():
        assert arm_ik.SHOULDER_LIMITS[0] <= q1 <= arm_ik.SHOULDER_LIMITS[1], name
        assert arm_ik.ELBOW_LIMITS[0] <= q2 <= arm_ik.ELBOW_LIMITS[1], name


def test_gripper_commands_within_limits():
    for g1, g2 in (pick_place.GRIP_OPEN, pick_place.GRIP_CLOSED):
        assert GRIP1_LIMITS[0] <= g1 <= GRIP1_LIMITS[1]
        assert GRIP2_LIMITS[0] <= g2 <= GRIP2_LIMITS[1]


def test_grasp_pose_reaches_target():
    """The scripted grasp pose must put the pinch point on the spawned
    cylinder (same numbers pick_place.py uses to place the Gazebo object)."""
    x, z = arm_ik.fk(*pick_place.POSES['grasp'])
    assert abs(x - pick_place.TARGET_POS[0]) < 0.005
    assert abs(z - pick_place.TARGET_POS[2]) < 0.005


def test_place_matches_grasp():
    assert pick_place.POSES['place'] == pick_place.POSES['grasp']


def test_retarget_updates_scene_consistently():
    ok = pick_place.retarget(0.14, 0.15)
    assert ok
    x, z = arm_ik.fk(*pick_place.POSES['grasp'])
    assert abs(x - 0.14) < 1e-6 and abs(z - 0.15) < 1e-6
    assert pick_place.TARGET_POS == [0.14, 0.0, 0.15]
    # pedestal top must sit at the cylinder's lower end
    ped_top = pick_place.PEDESTAL_POS[2] + pick_place.PEDESTAL_SIZE[2] / 2
    assert abs(ped_top - (0.15 - pick_place.TARGET_SIZE[2] / 2)) < 1e-6


def test_retarget_rejects_unreachable():
    assert not pick_place.retarget(1.0, 1.0)


def test_retarget_does_not_leak_into_later_tests():
    """The shipped target must be visible even after a retarget.

    This is the regression guard for the fixture above: without it, this
    test's assertions depend on whether it runs before or after
    test_retarget_updates_scene_consistently.
    """
    assert pick_place.TARGET_POS == list(SHIPPED_TARGET_POS)
    x, z = arm_ik.fk(*pick_place.POSES['grasp'])
    assert abs(x - SHIPPED_TARGET_POS[0]) < 0.005
    assert abs(z - SHIPPED_TARGET_POS[2]) < 0.005
