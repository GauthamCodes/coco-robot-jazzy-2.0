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
C2-NAV.42: the relay re-stamps, and the arbiter reads the relay, not Nav2.

The relay and the arbiter are constructed as real nodes but never spun:
the callback is called directly and the publisher is replaced by a list,
so nothing here waits on DDS. The end-to-end graph behaviour -- raw
commands never reaching the wheels, a STOP that does -- is in
gazebo_models/test/test_cmd_vel_wiring.py, next to the launch file that
wires it.
"""

from builtin_interfaces.msg import Time

from custom_teleop.cmd_vel_arbiter import CmdVelArbiter
from custom_teleop.cmd_vel_relay import CmdVelRelay, GATED_TOPIC

from geometry_msgs.msg import TwistStamped

import pytest

import rclpy

# The controller's raw command into the velocity smoother. nav2_bringup
# remaps controller_server, behavior_server and the smoother's input here.
RAW_NAV_TOPIC = '/cmd_vel_nav'
WHEEL_TOPIC = '/diff_drive_controller/cmd_vel'


class _Capture:
    """Stands in for a publisher; keeps what would have gone out."""

    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


@pytest.fixture
def context():
    """Own a ROS context for one test, torn down whatever happens."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def relay(context):
    """Build a relay whose publisher is a list."""
    node = CmdVelRelay()
    node._pub = _Capture()
    yield node
    node.destroy_node()


def _command(stamp_sec=0, stamp_nanosec=0):
    msg = TwistStamped()
    msg.header.stamp = Time(sec=stamp_sec, nanosec=stamp_nanosec)
    msg.header.frame_id = 'base_footprint'
    msg.twist.linear.x = 0.1234
    msg.twist.linear.y = -0.0005
    msg.twist.angular.z = -0.5678
    return msg


def _ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


# ── re-stamping ─────────────────────────────────────────────────────────────
def test_the_relay_replaces_the_stamp_with_its_own_clock(relay):
    # The collision monitor's stamp is the one that got wheel commands
    # dropped as stale. A zero stamp is the extreme case and is what the
    # web panel sends, so it is the input that proves a re-stamp.
    before = relay.get_clock().now().nanoseconds
    relay._cb(_command(stamp_sec=0))
    after = relay.get_clock().now().nanoseconds
    assert len(relay._pub.sent) == 1
    stamp = _ns(relay._pub.sent[0].header.stamp)
    assert stamp != 0
    assert before <= stamp <= after


def test_an_old_stamp_is_replaced_not_kept(relay):
    relay._cb(_command(stamp_sec=12, stamp_nanosec=345))
    stamp = relay._pub.sent[0].header.stamp
    assert (stamp.sec, stamp.nanosec) != (12, 345)


def test_only_the_stamp_changes(relay):
    # The relay sits after the collision monitor. Altering a velocity here
    # would be a second, unreviewed gate on the command.
    relay._cb(_command())
    out = relay._pub.sent[0]
    expected = _command()
    assert out.header.frame_id == expected.header.frame_id
    assert out.twist == expected.twist


def test_every_message_is_forwarded_exactly_once(relay):
    for _ in range(5):
        relay._cb(_command())
    assert len(relay._pub.sent) == 5


# ── topics ──────────────────────────────────────────────────────────────────
def test_the_gated_topic_is_not_the_raw_controller_topic():
    assert GATED_TOPIC != RAW_NAV_TOPIC
    assert GATED_TOPIC.lstrip('/') != RAW_NAV_TOPIC.lstrip('/')


def test_the_gated_topic_survives_ros_clean():
    # ros_clean.sh kills by the substring 'nav2_'; a topic name is on no
    # command line today, but a remap or a parameter would put it on one.
    assert 'nav2_' not in GATED_TOPIC


def test_the_relay_defaults_still_drive_the_controller(relay):
    # Topology A -- nav.launch.py without the arbiter -- is unchanged.
    assert relay.get_parameter('input_topic').value == '/cmd_vel'
    assert relay.get_parameter('output_topic').value == WHEEL_TOPIC


def test_the_arbiter_reads_the_relay_by_default(context):
    node = CmdVelArbiter()
    try:
        assert node.get_parameter('nav_topic').value == GATED_TOPIC
        # What it actually subscribed to, not what the parameter says.
        topics = [sub.topic_name for sub in node.subscriptions]
        assert GATED_TOPIC in topics
        assert RAW_NAV_TOPIC not in topics
        publishers = [pub.topic_name for pub in node.publishers]
        assert WHEEL_TOPIC in publishers
    finally:
        node.destroy_node()
