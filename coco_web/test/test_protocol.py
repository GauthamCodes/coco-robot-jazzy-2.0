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

"""The coco.v1 wire protocol: validation, refusal and framing."""

import json

from coco_web import protocol

import pytest


# ── malformed input ────────────────────────────────────────────────────

@pytest.mark.parametrize('raw,code', [
    ('not json at all', 'bad_json'),
    ('', 'bad_json'),
    ('[1,2,3]', 'bad_frame'),
    ('"a string"', 'bad_frame'),
    ('123', 'bad_frame'),
    ('{}', 'no_type'),
    ('{"type":42}', 'no_type'),
    ('{"type":"teleport"}', 'unknown_type'),
])
def test_malformed_frames_are_refused_with_a_code(raw, code):
    """Every refusal carries a stable code the UI can branch on."""
    with pytest.raises(protocol.ProtocolError) as caught:
        protocol.decode(raw)
    assert caught.value.code == code


def test_a_refusal_keeps_the_correlation_id():
    """A client that sent an id gets it back, so it can match the error."""
    with pytest.raises(protocol.ProtocolError) as caught:
        protocol.decode('{"type":"set_mode","mode":"warp","id":"abc"}')
    assert caught.value.frame_id == 'abc'


def test_a_bad_id_type_is_refused():
    """The id is echoed into JSON, so it must be a scalar."""
    with pytest.raises(protocol.ProtocolError) as caught:
        protocol.decode('{"type":"stop","id":{"nested":1}}')
    assert caught.value.code == 'bad_id'


# ── hello and versioning ───────────────────────────────────────────────

def test_hello_accepts_the_current_version():
    """The happy path names the version this server speaks."""
    frame = protocol.decode(
        json.dumps({'type': 'hello', 'protocol': protocol.PROTOCOL_VERSION}))
    assert frame['protocol'] == protocol.PROTOCOL_VERSION


def test_hello_refuses_a_different_version():
    """
    A version mismatch is an error, not a warning.

    Silently serving a client that expects a different contract is how a
    field that changed meaning becomes a mystery bug in someone's UI.
    """
    with pytest.raises(protocol.ProtocolError) as caught:
        protocol.decode('{"type":"hello","protocol":"coco.v99"}')
    assert caught.value.code == 'protocol_mismatch'


def test_hello_without_a_version_assumes_the_current_one():
    """Omitting the version is allowed; claiming a wrong one is not."""
    assert protocol.decode('{"type":"hello"}')['protocol'] == \
        protocol.PROTOCOL_VERSION


def test_client_name_is_truncated():
    """A client cannot push an unbounded string into the session log."""
    frame = protocol.decode(json.dumps(
        {'type': 'hello', 'client': 'x' * 5000}))
    assert len(frame['client']) <= 120


# ── drive ──────────────────────────────────────────────────────────────

def test_drive_defaults_to_zero():
    """A drive frame with no fields is a stop, not an error."""
    frame = protocol.decode('{"type":"drive"}')
    assert frame == {'type': 'drive', 'id': None,
                     'linear': 0.0, 'angular': 0.0}


@pytest.mark.parametrize('raw', [
    '{"type":"drive","linear":"fast"}',
    '{"type":"drive","linear":true}',
    '{"type":"drive","angular":null}',
    '{"type":"drive","linear":[1]}',
])
def test_drive_refuses_non_numeric_velocity(raw):
    """Strings, booleans, null and lists are all refused, not coerced."""
    with pytest.raises(protocol.ProtocolError) as caught:
        protocol.decode(raw)
    assert caught.value.code == 'bad_velocity'


def test_drive_refuses_nan_and_infinity():
    """
    Non-finite velocity is refused at the boundary.

    json.loads accepts bare NaN/Infinity even though neither is JSON, so
    this is a real wire case rather than a theoretical one.
    """
    for raw in ('{"type":"drive","linear":NaN}',
                '{"type":"drive","angular":Infinity}'):
        with pytest.raises(protocol.ProtocolError) as caught:
            protocol.decode(raw)
        assert caught.value.code == 'bad_velocity'


# ── modes, colours, missions, goals ────────────────────────────────────

@pytest.mark.parametrize('ui,arbiter', [
    ('teleop', 'teleop'), ('auto', 'nav'), ('stop', 'idle')])
def test_mode_translation(ui, arbiter):
    """The UI's words map onto the arbiter's, and only onto those."""
    assert protocol.mode_to_arbiter(ui) == arbiter


def test_every_ui_mode_translates():
    """No UI mode may be offered that the arbiter cannot be told about."""
    for mode in protocol.UI_MODES:
        assert protocol.mode_to_arbiter(mode) is not None


def test_unknown_mode_does_not_translate():
    """An unmapped mode returns None rather than passing through."""
    assert protocol.mode_to_arbiter('nav') is None


def test_select_target_uses_the_supplied_colour_table():
    """Colours are validated against the mission's table, not a copy."""
    frame = protocol.decode('{"type":"select_target","colour":"red"}',
                            colours=('red', 'green'))
    assert frame['colour'] == 'red'
    with pytest.raises(protocol.ProtocolError) as caught:
        protocol.decode('{"type":"select_target","colour":"blue"}',
                        colours=('red', 'green'))
    assert caught.value.code == 'bad_colour'


def test_fallback_colours_match_coco_config():
    """The fallback list must not drift from the mission's own table."""
    robot = pytest.importorskip('coco_config.robot')
    assert tuple(robot.TARGET_COLOURS) == protocol.FALLBACK_COLOURS


@pytest.mark.parametrize('action', ['start', 'abort'])
def test_mission_actions(action):
    """Only start and abort exist."""
    assert protocol.decode(
        json.dumps({'type': 'mission', 'action': action}))['action'] == action


def test_mission_refuses_anything_else():
    """A third action is refused rather than treated as one of the two."""
    with pytest.raises(protocol.ProtocolError) as caught:
        protocol.decode('{"type":"mission","action":"pause"}')
    assert caught.value.code == 'bad_action'


def test_nav_goal_accepts_a_reasonable_pose():
    """A plain map-frame goal passes through as floats."""
    frame = protocol.decode('{"type":"nav_goal","x":1.5,"y":-2.25}')
    assert (frame['x'], frame['y']) == (1.5, -2.25)


@pytest.mark.parametrize('raw', [
    '{"type":"nav_goal","x":1.0}',
    '{"type":"nav_goal","x":1.0,"y":NaN}',
    '{"type":"nav_goal","x":1e9,"y":0.0}',
    '{"type":"nav_goal","x":"here","y":0.0}',
])
def test_nav_goal_refuses_junk(raw):
    """Missing, non-finite, absurd and non-numeric goals are all refused."""
    with pytest.raises(protocol.ProtocolError) as caught:
        protocol.decode(raw)
    assert caught.value.code == 'bad_goal'


# ── subscribe ──────────────────────────────────────────────────────────

def test_subscribe_accepts_known_streams_and_dedupes():
    """Known streams are kept, in order, without duplicates."""
    frame = protocol.decode(
        '{"type":"subscribe","streams":["telemetry","lidar","telemetry"]}')
    assert frame['streams'] == ['telemetry', 'lidar']


def test_subscribe_refuses_an_unknown_stream():
    """An unknown stream is an error, not a silently dropped request."""
    with pytest.raises(protocol.ProtocolError) as caught:
        protocol.decode('{"type":"subscribe","streams":["telemetry","brain"]}')
    assert caught.value.code == 'unknown_stream'
    assert 'brain' in str(caught.value)


# ── server frames ──────────────────────────────────────────────────────

def test_welcome_advertises_the_contract():
    """Welcome carries the version and the full command vocabulary."""
    frame = protocol.welcome({'id': 'abc'}, {}, {})
    assert frame['protocol'] == protocol.PROTOCOL_VERSION
    assert 'drive' in frame['commands']
    assert 'stop' in frame['commands']


def test_every_advertised_command_actually_decodes():
    """Welcome must not advertise a command the server cannot parse."""
    advertised = protocol.welcome({}, {}, {})['commands']
    assert set(advertised) == set(protocol._CLIENT_SCHEMA)


def test_encode_scrubs_non_finite_floats():
    """
    One NaN from a sensor must not break the whole client stream.

    json.dumps emits a bare NaN by default, which JSON.parse rejects, so
    the browser would lose the entire frame -- including the mission
    state -- because one range reading was bad.
    """
    payload = protocol.encode(
        {'type': 'telemetry', 'v': float('nan'),
         'nested': {'list': [1.0, float('inf')]}})
    assert 'NaN' not in payload and 'Infinity' not in payload
    parsed = json.loads(payload)
    assert parsed['v'] is None
    assert parsed['nested']['list'] == [1.0, None]


def test_error_frame_shape():
    """Errors carry code, message and the client's id."""
    frame = protocol.error('bad_mode', 'nope', 7)
    assert frame == {'type': 'error', 'id': 7,
                     'code': 'bad_mode', 'message': 'nope'}


def test_telemetry_frame_carries_a_sequence_number():
    """Sequence numbers let a client detect its own dropped frames."""
    frame = protocol.telemetry(5, 1.0, {}, {}, {}, {}, {})
    assert frame['seq'] == 5
    assert frame['type'] == 'telemetry'
