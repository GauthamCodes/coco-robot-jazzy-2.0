"""ARM_LIMITS must stay in step with the URDF it claims to mirror.

joint_limits.ARM_LIMITS is the Python-side source of truth: the teleop
node clamps to it and the joint-state monitor warns from it. The URDF is
the source of truth for the robot. Nothing enforced that those two agreed,
and they had already drifted — teleop_arm_node re-typed the gripper range
as (-0.3, 1.047) against the URDF's (-0.35, 1.1).

This parses the xacro directly rather than going through the xacro CLI:
the <limit> tags on the arm joints carry literal numbers, so no property
substitution is involved, and the test needs no ROS environment at all.

The URDF is located through the repo layout rather than the ament index
on purpose. Declaring a test_depend on gazebo_models would close a
dependency cycle — gazebo_models exec_depends on custom_teleop (for
cmd_vel_relay in nav.launch.py), which depends on coco_config — and
colcon then refuses to order the packages at all. Both packages always
ship in this repo, so the relative path is stable.
"""
from pathlib import Path
import re

from coco_config.joint_limits import ARM_LIMITS

import pytest

# .../coco_config/test/this_file -> parents[2] is the repo root
URDF = (Path(__file__).resolve().parents[2]
        / 'gazebo_models' / 'urdf' / 'coco_robo2.xacro')


def _urdf_limits():
    """Return {joint_name: (lower, upper)} for every joint with a range."""
    assert URDF.is_file(), f'URDF not found at {URDF}'
    text = URDF.read_text()

    found = {}
    for match in re.finditer(
            r'<joint\s+name="([^"]+)"\s+type="([^"]+)"(.*?)</joint>',
            text, re.S):
        name, _type, body = match.groups()
        limit = re.search(
            r'<limit[^>]*lower="([^"]+)"[^>]*upper="([^"]+)"', body)
        if limit:
            found[name] = (float(limit.group(1)), float(limit.group(2)))
    return found


def test_urdf_parse_finds_the_arm_joints():
    """Guard the regex itself.

    A parser that silently found nothing would make every comparison
    below vacuously pass.
    """
    urdf = _urdf_limits()
    assert set(ARM_LIMITS) <= set(urdf), (
        f'joints in ARM_LIMITS missing from the URDF: '
        f'{sorted(set(ARM_LIMITS) - set(urdf))}')


@pytest.mark.parametrize('joint', sorted(ARM_LIMITS))
def test_limits_match_urdf(joint):
    lower, upper = _urdf_limits()[joint]
    assert ARM_LIMITS[joint] == pytest.approx((lower, upper)), (
        f'{joint}: joint_limits.py says {ARM_LIMITS[joint]}, '
        f'coco_robo2.xacro says {(lower, upper)}')


def test_gripper_joints_are_exact_mirrors():
    """Check the gripper ranges are exact negations of each other.

    teleop_arm_node publishes [grip, -grip] to Revolute-8/-9, which is
    only safe while that holds.
    """
    lo8, hi8 = ARM_LIMITS['m_link3_Revolute-8']
    lo9, hi9 = ARM_LIMITS['m_link3_Revolute-9']
    assert (lo9, hi9) == pytest.approx((-hi8, -lo8))
