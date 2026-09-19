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
Which topics the browser is allowed to reach, and which it never may.

This module is the command-safety boundary of the web platform, and it is
deliberately a *pure* module: no rclpy, no sockets, no I/O. The whole of
the policy is data plus three predicates, so the guarantee "the browser
cannot become a second wheel publisher" is proved by unit tests that need
no ROS graph, exactly as ``cmd_vel_arbiter.select_source`` is.

The command path this protects, measured and fixed across C2-NAV.42/43::

    Nav2      -> /cmd_vel        -> cmd_vel_relay -> /cmd_vel_gated ->|
    browser   -> /cmd_vel_teleop --------------------------------->  |
    RL policy -> /cmd_vel_rl     --------------------------------->  +-> arbiter
    approach  -> /cmd_vel_approach ------------------------------->  |
                                                                     |
                                    arbiter -> /diff_drive_controller/cmd_vel
                                                    -> collision monitor
                                                    -> velocity smoother
                                                    -> wheels

``cmd_vel_arbiter`` is the ONLY node permitted to publish the wheel topic.
It says so itself and warns when a second publisher appears, but a warning
is not a boundary: by the time it fires, two sources are already
interleaving and the robot tracks their average. C2-NAV.41 measured that
exact failure reaching the wheels 21.31 % of the time. So the web layer
refuses structurally instead, in three independent ways:

1. The wire protocol has **no field naming a topic**. A browser cannot ask
   for a topic because the vocabulary has no word for one (see
   ``protocol.py``). This is the difference between this server and the
   rosbridge panel it replaces, where any tab could publish anything.
2. Every topic this server may publish is listed in ``PUBLISH_ALLOWLIST``
   below, and each entry is checked against ``WHEEL_TOPICS`` by
   ``assert_publish_safe`` at node construction -- so a typo, a bad launch
   argument or a future edit fails loudly at startup rather than quietly
   at 20 Hz.
3. Velocity is clamped before it is published, so a hostile or buggy
   client cannot command more than the panel's own limits.
"""

# Topics that reach the wheels. Publishing any of these from the web layer
# would defeat arbitration, so nothing here may ever appear in
# PUBLISH_ALLOWLIST.
#
# /diff_drive_controller/cmd_vel is the arbiter's output and the real wheel
# topic. It carries BOTH Twist and TwistStamped (CLAUDE.md's trap table) --
# which is precisely why a second publisher is so hard to notice by hand:
# `ros2 topic info` still reads healthy.
#
# /cmd_vel is Nav2's raw output, consumed by cmd_vel_relay. It is not the
# wheel topic, but publishing it from a browser injects commands into the
# navigation lane upstream of the relay, which is the same class of defect:
# a control source that no arbiter selected. C2-NAV.42 fixed a loop on this
# topic; nothing should widen it again.
WHEEL_TOPICS = frozenset({
    '/diff_drive_controller/cmd_vel',
    '/cmd_vel',
    '/cmd_vel_smoothed',
    '/cmd_vel_nav',
})

# The single velocity channel the browser is allowed to drive. This is an
# arbiter *input*: teleop preempts every mode, and the arbiter still
# decides whether it reaches the wheels.
TELEOP_TOPIC = '/cmd_vel_teleop'

# Logical command name -> (topic, ROS message type as a type string).
#
# The message types are strings rather than imported classes so this module
# stays importable without rclpy, which is what lets the safety tests run
# on a bare interpreter. platform_server.py resolves them once at startup.
PUBLISH_ALLOWLIST = {
    'drive': (TELEOP_TOPIC, 'geometry_msgs/msg/TwistStamped'),
    'mode': ('/mission/mode', 'std_msgs/msg/String'),
    'target_colour': ('/mission/target_colour', 'std_msgs/msg/String'),
    'nav_goal': ('/goal_pose', 'geometry_msgs/msg/PoseStamped'),
    'arm': ('/arm_controller/joint_trajectory',
            'trajectory_msgs/msg/JointTrajectory'),
    'gripper': ('/gripper_controller/joint_trajectory',
                'trajectory_msgs/msg/JointTrajectory'),
}

# Services the browser may call. Same closed-vocabulary rule: the client
# names an action, never a service path.
CALL_ALLOWLIST = {
    'mission_start': ('/mission/start', 'std_srvs/srv/Trigger'),
    'mission_abort': ('/mission/abort', 'std_srvs/srv/Trigger'),
}

# Panel velocity limits, matching the joystick the old panel shipped. The
# robot's own limits live in the controller and the velocity smoother;
# these only bound what the *browser* may ask for.
MAX_LINEAR = 0.5     # m/s
MAX_ANGULAR = 1.2    # rad/s


class UnsafeTopicError(RuntimeError):
    """
    Raised when the web layer is configured to publish a wheel topic.

    Deliberately fatal. A web server that has been pointed at the wheel
    topic is not degraded, it is dangerous: it silently defeats the
    collision monitor, the velocity smoother and the arbiter all at once.
    """


def is_wheel_topic(topic):
    """Whether `topic` is a channel that reaches the wheels."""
    return _normalise(topic) in WHEEL_TOPICS


def _normalise(topic):
    """
    Canonical form of a topic name, so '//cmd_vel' cannot slip past.

    Only leading-slash and trailing-slash noise is normalised. Remapping
    and namespacing are ROS's job; this is a string guard, not a resolver.
    """
    if not isinstance(topic, str):
        return ''
    collapsed = '/'.join(part for part in topic.split('/') if part)
    return '/' + collapsed if collapsed else ''


def assert_publish_safe(topics):
    """
    Raise UnsafeTopicError if any of `topics` reaches the wheels.

    Called by platform_server at construction with every topic it is about
    to create a publisher for -- including values that came from ROS
    parameters, which is the path a bad launch argument would take.
    """
    offenders = sorted({_normalise(t) for t in topics if is_wheel_topic(t)})
    if offenders:
        raise UnsafeTopicError(
            'the web layer may not publish ' + ', '.join(offenders) +
            '; only cmd_vel_arbiter may publish the wheel topic. Point the '
            'web layer at ' + TELEOP_TOPIC + ' (an arbiter input) instead.')


def allowlisted_topics():
    """Every topic the web layer may publish, as a sorted list."""
    return sorted(topic for topic, _ in PUBLISH_ALLOWLIST.values())


def clamp_velocity(linear, angular):
    """
    Clamp a requested velocity to the panel's limits.

    Returns ``(linear, angular)`` as floats. Non-numeric input becomes 0.0
    rather than raising: this runs on a message that already passed schema
    validation, and a stop is the safe reading of nonsense.
    """
    return (_clamp(linear, MAX_LINEAR), _clamp(angular, MAX_ANGULAR))


def _clamp(value, limit):
    """Clamp `value` into [-limit, limit], mapping junk and NaN to 0.0."""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0.0
    # NaN fails every comparison, so it would slip through min/max and
    # reach the controller as a NaN twist. Compare it against itself.
    if number != number:
        return 0.0
    return max(-limit, min(limit, number))
