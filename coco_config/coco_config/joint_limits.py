"""
joint_limits.py
===============
Joint limits from coco_robo2.xacro, plus the pure predicate used to
decide when a joint is close enough to a limit to warn about.

Kept free of ROS imports so it is unit-testable without a simulator, and
so the numbers live in exactly one place on the Python side. They must
stay in step with the <limit> tags in
gazebo_models/urdf/coco_robo2.xacro — the URDF remains the source of
truth for the robot itself.
"""

# joint name -> (lower, upper) in radians
ARM_LIMITS = {
    'm_link1_Revolute-6': (-3.84, 1.0),    # shoulder
    'm_link2_Revolute-7': (-1.6, 1.6),     # elbow
    'm_link3_Revolute-8': (-0.35, 1.1),    # finger
    'm_link3_Revolute-9': (-1.1, 0.35),    # finger (mirrored)
}

# Warn once a joint is within this fraction of its range of either end.
MARGIN_FRACTION = 0.02


def limit_margin(name, position, limits=None, margin_fraction=MARGIN_FRACTION):
    """Distance (rad) to the nearer limit, and whether that is too close.

    Returns (margin, near_limit, beyond_limit). `margin` is negative when
    the joint is outside its declared range. Unknown joints return
    (None, False, False) so wheel joints on /joint_states are ignored.
    """
    limits = ARM_LIMITS if limits is None else limits
    if name not in limits:
        return None, False, False
    lower, upper = limits[name]
    margin = min(position - lower, upper - position)
    threshold = (upper - lower) * margin_fraction
    return margin, margin <= threshold, margin < 0.0
