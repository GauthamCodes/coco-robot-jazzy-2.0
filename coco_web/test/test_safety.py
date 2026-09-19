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
The command-safety boundary: the browser cannot reach the wheels.

These are the tests TASK 3 asks for. They run on a bare interpreter with
no ROS graph, which is the point: the guarantee is a property of the
allowlist and the schema, not of a running system that happened to behave
during one sweep.
"""

import math

from coco_web import protocol, safety

import pytest


# ── the allowlist may not contain a wheel topic ────────────────────────

def test_no_allowlisted_topic_reaches_the_wheels():
    """The core invariant: no publishable topic is a wheel topic."""
    for command, (topic, _type) in safety.PUBLISH_ALLOWLIST.items():
        assert not safety.is_wheel_topic(topic), (
            f'command {command!r} publishes {topic}, which reaches the '
            f'wheels; only cmd_vel_arbiter may do that')


def test_allowlist_and_wheel_topics_are_disjoint():
    """Stated as a set operation, so a future edit to either side fails."""
    allowed = set(safety.allowlisted_topics())
    assert allowed & set(safety.WHEEL_TOPICS) == set()


def test_the_only_velocity_topic_is_the_arbiter_input():
    """Exactly one velocity channel, and it is an arbiter input."""
    velocity = [topic for topic, msg in safety.PUBLISH_ALLOWLIST.values()
                if 'Twist' in msg]
    assert velocity == [safety.TELEOP_TOPIC]


def test_the_arbiter_output_is_named_as_a_wheel_topic():
    """
    Guards the guard: the real wheel topic must be in WHEEL_TOPICS.

    If this list ever stopped naming the arbiter's output, every other
    test here would still pass while protecting nothing.
    """
    assert '/diff_drive_controller/cmd_vel' in safety.WHEEL_TOPICS


def test_wheel_topics_match_the_arbiter_default():
    """
    The wheel topic is the one cmd_vel_arbiter actually publishes.

    Read from custom_teleop rather than re-typed, so a rename there fails
    here instead of silently opening the band. Skipped rather than failed
    when custom_teleop is not importable: that is a stripped environment,
    not a defect.
    """
    arbiter = pytest.importorskip('custom_teleop.cmd_vel_arbiter')
    source = arbiter.__doc__ or ''
    del source
    # The default is declared as a ROS parameter; find it in the source.
    import inspect
    text = inspect.getsource(arbiter)
    assert "'output_topic', '/diff_drive_controller/cmd_vel'" in text, (
        'cmd_vel_arbiter no longer defaults to the wheel topic this '
        'module guards; update safety.WHEEL_TOPICS')


# ── assert_publish_safe is the startup gate ────────────────────────────

def test_assert_publish_safe_accepts_the_allowlist():
    """The shipped allowlist passes its own gate."""
    safety.assert_publish_safe(safety.allowlisted_topics())


@pytest.mark.parametrize('topic', sorted(safety.WHEEL_TOPICS))
def test_assert_publish_safe_rejects_every_wheel_topic(topic):
    """Each wheel topic is refused, one test per topic so failures name it."""
    with pytest.raises(safety.UnsafeTopicError):
        safety.assert_publish_safe([topic])


def test_assert_publish_safe_rejects_a_bad_parameter_override():
    """
    A launch argument cannot turn the web layer into a wheel publisher.

    This is the realistic attack: not malice, but
    `-p teleop_topic:=/diff_drive_controller/cmd_vel` in a hurry.
    """
    with pytest.raises(safety.UnsafeTopicError):
        safety.assert_publish_safe(
            safety.allowlisted_topics() + ['/diff_drive_controller/cmd_vel'])


def test_assert_publish_safe_normalises_before_comparing():
    """Slash noise cannot smuggle a wheel topic past the guard."""
    for spelling in ('//diff_drive_controller/cmd_vel',
                     '/diff_drive_controller/cmd_vel/',
                     '/diff_drive_controller//cmd_vel'):
        with pytest.raises(safety.UnsafeTopicError):
            safety.assert_publish_safe([spelling])


def test_the_error_names_the_offending_topic():
    """An operator must be able to fix it from the message alone."""
    with pytest.raises(safety.UnsafeTopicError) as caught:
        safety.assert_publish_safe(['/cmd_vel'])
    message = str(caught.value)
    assert '/cmd_vel' in message
    assert safety.TELEOP_TOPIC in message


# ── velocity clamping ──────────────────────────────────────────────────

@pytest.mark.parametrize('linear,angular,want_lin,want_ang', [
    (0.0, 0.0, 0.0, 0.0),
    (0.2, -0.4, 0.2, -0.4),
    (99.0, 99.0, safety.MAX_LINEAR, safety.MAX_ANGULAR),
    (-99.0, -99.0, -safety.MAX_LINEAR, -safety.MAX_ANGULAR),
])
def test_clamp_velocity(linear, angular, want_lin, want_ang):
    """Requests are clamped to the panel's limits in both directions."""
    assert safety.clamp_velocity(linear, angular) == (want_lin, want_ang)


def test_clamp_velocity_maps_nan_to_zero():
    """
    A NaN must not reach the controller as a NaN twist.

    NaN fails every comparison, so a naive min/max clamp passes it
    straight through. A stop is the safe reading of nonsense.
    """
    assert safety.clamp_velocity(float('nan'), float('nan')) == (0.0, 0.0)


def test_clamp_velocity_bounds_infinity():
    """Infinity clamps to the limit rather than passing through."""
    lin, ang = safety.clamp_velocity(float('inf'), float('-inf'))
    assert lin == safety.MAX_LINEAR
    assert ang == -safety.MAX_ANGULAR
    assert math.isfinite(lin) and math.isfinite(ang)


def test_clamp_velocity_maps_junk_to_zero():
    """Non-numeric input stops the robot instead of raising mid-command."""
    assert safety.clamp_velocity('fast', None) == (0.0, 0.0)


# ── the protocol cannot express a topic at all ─────────────────────────

def test_no_client_frame_accepts_a_topic_field():
    """
    The schema has no word for a topic, so a client cannot ask for one.

    This is the structural half of the guarantee: the allowlist stops a
    misconfigured *server*, and this stops a hostile *client*.
    """
    for kind, keys in protocol._CLIENT_SCHEMA.items():
        forbidden = {'topic', 'msg_type', 'type_name', 'service', 'op', 'msg'}
        assert keys & forbidden == set(), (
            f'frame type {kind!r} accepts a field that could name a ROS '
            f'entity: {sorted(keys & forbidden)}')


def test_a_rosbridge_publish_frame_is_rejected():
    """
    The exact frame the old panel's bridge would have accepted.

    Under rosbridge this published to the wheels. Here it must not even
    parse, and the error must say so rather than failing vaguely.
    """
    rosbridge_frame = (
        '{"op":"publish","topic":"/diff_drive_controller/cmd_vel",'
        '"msg":{"twist":{"linear":{"x":9.0}}}}')
    with pytest.raises(protocol.ProtocolError) as caught:
        protocol.decode(rosbridge_frame)
    assert caught.value.code in ('no_type', 'unknown_type')


def test_a_drive_frame_carrying_a_topic_is_rejected_not_ignored():
    """Extra keys are refused, so a confused client learns it was refused."""
    with pytest.raises(protocol.ProtocolError) as caught:
        protocol.decode(
            '{"type":"drive","linear":0.1,"angular":0.0,'
            '"topic":"/diff_drive_controller/cmd_vel"}')
    assert caught.value.code == 'unexpected_fields'
    assert 'topic' in str(caught.value)


def test_decoded_drive_is_already_clamped():
    """Clamping happens at the boundary, so no caller can forget it."""
    frame = protocol.decode('{"type":"drive","linear":50,"angular":-50}')
    assert frame['linear'] == safety.MAX_LINEAR
    assert frame['angular'] == -safety.MAX_ANGULAR
