"""Pick-and-place waypoints must stay inside the URDF joint limits."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import arm_ik  # noqa: E402

# pick_place imports rclpy/moveit_msgs; skip cleanly where ROS isn't sourced
pytest.importorskip('rclpy')
pytest.importorskip('moveit_msgs')
import pick_place  # noqa: E402

# Gripper finger limits from the URDF (m_link3_Revolute-8 / -9)
GRIP1_LIMITS = (-0.35, 1.1)
GRIP2_LIMITS = (-1.1, 0.35)


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
