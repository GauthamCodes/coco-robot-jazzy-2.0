"""Joint-limit maths (pure functions — no ROS/Gazebo needed)."""
from coco_config.joint_limits import ARM_LIMITS, limit_margin

SHOULDER = 'm_link1_Revolute-6'      # (-3.84, 1.0)
ELBOW = 'm_link2_Revolute-7'         # (-1.6, 1.6)


def test_mid_range_joint_is_clear_of_limits():
    margin, near, beyond = limit_margin(ELBOW, 0.0)
    assert margin == 1.6
    assert not near and not beyond


def test_joint_near_upper_limit_warns():
    _, near, beyond = limit_margin(ELBOW, 1.59)
    assert near and not beyond


def test_joint_near_lower_limit_warns():
    _, near, beyond = limit_margin(SHOULDER, -3.83)
    assert near and not beyond


def test_joint_beyond_limit_is_flagged_and_margin_goes_negative():
    margin, near, beyond = limit_margin(ELBOW, 1.75)
    assert margin < 0
    assert beyond and near     # beyond implies near


def test_unknown_joints_are_ignored():
    # /joint_states also carries the four wheel joints, which are
    # continuous and must not be limit-checked.
    assert limit_margin('wheel_front_left_joint', 123.4) == (None, False, False)


def test_margin_is_symmetric_about_range_centre():
    lower, upper = ARM_LIMITS[ELBOW]
    centre = (lower + upper) / 2
    left, _, _ = limit_margin(ELBOW, centre - 0.5)
    right, _, _ = limit_margin(ELBOW, centre + 0.5)
    assert abs(left - right) < 1e-12
