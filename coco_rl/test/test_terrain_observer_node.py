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

"""The ROS wrapper's own tests. Added by C2-M2.1, and here is why.

C2-M2.0 unit-tested :mod:`coco_rl.terrain_observer` thoroughly through its
pure core and never once **constructed the node**. Three defects lived in
the gap, and every one of them was invisible to a test that drives the
observer directly:

1. ``is_best_effort()`` called with no argument. It takes the topic.
   ``TypeError`` in ``__init__`` — the node could not start at all.
2. The estimator was advanced from the 10 Hz publish timer, so every
   sample arrived exactly ``MAX_AGE`` apart and the observer withdrew
   itself with ``stale input: 0.100 s > 0.100 s`` on every single one.
   The pure core cannot see this because tests feed it at 50 Hz.
3. ``on_declared_flat`` was never passed, so the flat-ground reference
   could never be learned and ``calibrated`` was False forever.

All three are wiring, not estimation. These tests construct the real node
against fake messages so that wiring is exercised off-line.

This file imports ``rclpy``. That is allowed and does not touch
CLAUDE.md's rule 2, which is about the TRAINING environment:
``coco_rl.mujoco_env`` and what it reaches. ``terrain_observer_node`` is
already the ROS face and is reached by nothing the trainer imports —
``test_mujoco_env_has_no_ros.py`` is the test that pins that, and it is
untouched.
"""

import math

from geometry_msgs.msg import TwistStamped

import pytest

rclpy = pytest.importorskip('rclpy')

from sensor_msgs.msg import Imu, JointState        # noqa: E402

from coco_rl.terrain_observer_node import (        # noqa: E402
    TerrainObserverNode, WHEEL_JOINTS)


G = 9.81


@pytest.fixture
def node():
    rclpy.init()
    n = TerrainObserverNode()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _imu(stamp, pitch=0.0, pitch_rate=0.0):
    """An IMU sample at `stamp` seconds, nose-up NEGATIVE as measured."""
    m = Imu()
    m.header.stamp.sec = int(stamp)
    m.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
    # quaternion for a pure pitch
    m.orientation.w = math.cos(pitch / 2.0)
    m.orientation.x = 0.0
    m.orientation.y = math.sin(pitch / 2.0)
    m.orientation.z = 0.0
    m.angular_velocity.y = pitch_rate
    # specific force: at rest it reads +g on z, not zero
    m.linear_acceleration.x = -G * math.sin(pitch)
    m.linear_acceleration.y = 0.0
    m.linear_acceleration.z = G * math.cos(pitch)
    return m


def _joints(speed=1.0):
    m = JointState()
    m.name = list(WHEEL_JOINTS)
    m.velocity = [speed] * len(WHEEL_JOINTS)
    return m


def test_the_node_constructs_at_all(node):
    """Defect 1. `is_best_effort()` needs its topic; this caught nothing
    off-line and the node died on the first live launch."""
    assert node is not None


def test_imu_subscription_is_best_effort(node):
    """The camera/IMU trap: a RELIABLE subscriber never matches a
    BEST_EFFORT publisher and the node goes SILENTLY blind."""
    from rclpy.qos import ReliabilityPolicy
    subs = {s.topic_name: s for s in node.subscriptions}
    imu = subs['/imu']
    assert imu.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
    js = subs['/joint_states']
    assert js.qos_profile.reliability == ReliabilityPolicy.RELIABLE


def test_estimator_runs_at_the_imu_rate_not_the_publish_rate(node):
    """Defect 2, the one that mattered most.

    Feeding at 50 Hz must produce VALID estimates. The old node advanced
    the observer from its 10 Hz timer, which put every sample exactly
    MAX_AGE apart and made the observer withdraw on all of them.
    """
    node._joints = _joints()
    t = 100.0
    for i in range(50):
        node._on_imu(_imu(t + i * 0.02))       # 50 Hz, the IMU's rate
    assert node._est is not None
    assert node._est.grade_valid, node._est.reason
    assert 'stale' not in node._est.reason


def test_ten_hz_feeding_is_what_used_to_break_it(node):
    """The same node fed at the publish rate withdraws. Pinned so the
    regression is described by a test rather than by a comment.

    The spacing here is 0.102 s and not 0.100, because that is what the
    old code actually produced and the difference is the whole defect.
    ``MAX_AGE`` is 0.1 s and the test is ``dt > MAX_AGE``, so an exact
    0.100 does NOT trip it. The 10 Hz timer picked up whichever /imu
    message was latest, and /imu arrives in ~0.0204 s quanta (measured
    live: 49.1 Hz), so five quanta land at ~0.102 s — just past the
    threshold, on essentially every tick. Measured live: 431 of 431
    samples reported ``stale input: 0.100 s > 0.100 s``.
    """
    node._joints = _joints()
    t = 200.0
    for i in range(20):
        node._on_imu(_imu(t + i * 0.102))      # what the 10 Hz timer gave
    assert node._est is not None
    assert not node._est.grade_valid
    assert 'stale' in node._est.reason


def test_publish_does_not_advance_the_estimator(node):
    """Publication is on the consumer's clock, estimation on the
    sensor's. `_publish` must be a pure read."""
    node._joints = _joints()
    t = 300.0
    for i in range(10):
        node._on_imu(_imu(t + i * 0.02))
    before = node._est
    node._publish()
    node._publish()
    assert node._est is before


def test_declare_flat_is_wired_through(node):
    """Defect 3. Left false the reference is never taken; set true, on
    quiet flat ground, it is."""
    node._joints = _joints(speed=0.0)
    node._cmd = (0.0, 0.0)
    t = 400.0
    for i in range(60):
        node._on_imu(_imu(t + i * 0.02))
    assert not node._est.grade_calibrated

    node.set_parameters([rclpy.parameter.Parameter(
        'declare_flat', rclpy.Parameter.Type.BOOL, True)])
    t = 500.0
    for i in range(60):
        node._on_imu(_imu(t + i * 0.02))
    assert node._est.grade_calibrated


def test_it_publishes_no_velocity_command(node):
    """CLAUDE.md rule 4: cmd_vel_arbiter is the SOLE publisher to the
    controller. The observer publishes and does not drive."""
    # /parameter_events and /rosout come with every rclpy node; the
    # question is what this node adds on top of them.
    builtin = ('/parameter_events', '/rosout')
    topics = [p.topic_name for p in node.publishers
              if p.topic_name not in builtin]
    assert topics == ['/terrain/state']
    assert not any('cmd_vel' in t for t in topics)


def test_it_subscribes_to_the_command_and_never_publishes_it(node):
    subs = [s.topic_name for s in node.subscriptions]
    assert '/diff_drive_controller/cmd_vel' in subs
    pubs = [p.topic_name for p in node.publishers]
    assert '/diff_drive_controller/cmd_vel' not in pubs


def test_waiting_states_are_explicit_not_plausible_numbers(node):
    """No inputs yet: STALE with a reason, never a comfortable zero."""
    from diagnostic_msgs.msg import DiagnosticStatus
    node._publish()
    st = node._waiting('waiting for /imu')
    assert all(s.level == DiagnosticStatus.STALE for s in st)
    assert all(s.message for s in st)


def test_the_traction_payload_does_not_claim_a_friction_estimate(node):
    """C2-M2.0 measured that true mu is not identifiable. Nothing on the
    wire may be called a friction estimate."""
    node._joints = _joints()
    t = 600.0
    for i in range(30):
        node._on_imu(_imu(t + i * 0.02))
    keys = [kv.key for kv in node._traction_status(node._est).values]
    assert 'tau_traction_demand' in keys
    assert 'mu_lower_bound' in keys
    assert 'mu_sched_input' in keys
    # the old, misleading names must be gone
    assert 'mu_hat' not in keys
    assert not any(k in ('friction', 'mu', 'mu_estimate') for k in keys)


def test_cmd_vel_is_read_from_a_twiststamped(node):
    m = TwistStamped()
    m.twist.linear.x = 0.4
    m.twist.angular.z = -0.2
    node._on_cmd(m)
    assert node._cmd == (0.4, -0.2)


def test_missing_wheel_names_are_not_read_as_zeros(node):
    """A missing encoder and a stationary wheel are different facts."""
    js = JointState()
    js.name = ['not_a_wheel']
    js.velocity = [0.0]
    node._joints = js
    assert node._wheel_speeds() is None
