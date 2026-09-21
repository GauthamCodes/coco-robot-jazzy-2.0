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
The versioned WebSocket protocol spoken between the browser and COCO.

Pure module: parsing and validation only, no rclpy and no sockets, so the
whole schema is unit-tested without a ROS graph.

Design rule, and the reason this file exists at all
---------------------------------------------------
The panel this replaces spoke *rosbridge*, a generic ROS-over-WebSocket
bridge. rosbridge is an excellent debugging tool and a poor product
boundary: it lets any browser tab publish any topic, call any service and
enumerate the graph through rosapi. The panel's HTML happened to be
well-behaved, but nothing made it so -- a second tab, a stale cache or a
curious user could publish /diff_drive_controller/cmd_vel directly and
become a second wheel publisher.

So this protocol is a **closed vocabulary**. A client names an intent
("drive", "start the mission"); it never names a topic, a service, or a
message type, because no message in the schema has a field that could
carry one. Extra keys are not merely ignored, they are rejected, so a
client that *thinks* it is talking rosbridge gets a clear error instead of
a silent no-op. Mapping intent onto ROS is ``safety.PUBLISH_ALLOWLIST``'s
job, and it is checked against ``safety.WHEEL_TOPICS`` at startup.

Versioning
----------
``PROTOCOL_VERSION`` is sent in every ``welcome`` frame and may be sent by
the client in ``hello``. It is a *wire contract* version, bumped when an
existing field changes meaning or disappears -- not when a field is added,
which clients must tolerate. Telemetry carries a monotonic ``seq`` so a
client can detect its own dropped frames.
"""

import json
import math

from coco_web import safety
from coco_web import streams as streams_mod

#: Wire contract version. See the module docstring for the bump rule.
PROTOCOL_VERSION = 'coco.v1'

#: Colours the fetch mission understands. Sourced from coco_config at
#: runtime by platform_server; duplicated here only as the fallback for a
#: bare-interpreter import, and a test asserts the two agree.
FALLBACK_COLOURS = ('red', 'green', 'blue', 'yellow')

#: Drive modes the arbiter understands, in the panel's spelling. 'auto' is
#: the arbiter's 'nav' and 'stop' is its 'idle'; the mapping lives in
#: ``mode_to_arbiter`` so the browser never has to know arbiter spelling.
UI_MODES = ('teleop', 'auto', 'stop')

_MODE_TO_ARBITER = {'teleop': 'teleop', 'auto': 'nav', 'stop': 'idle'}

#: Every client->server frame type, and the keys each one allows. A frame
#: carrying any other key is rejected: see the module docstring.
_CLIENT_SCHEMA = {
    'hello': {'protocol', 'client', 'binary'},
    'ping': {'t'},
    'drive': {'linear', 'angular'},
    'stop': set(),
    'set_mode': {'mode'},
    'select_target': {'colour'},
    'mission': {'action'},
    'nav_goal': {'x', 'y'},
    'set_arm': {'shoulder', 'elbow'},
    'set_gripper': {'grip'},
    'subscribe': {'streams'},
    'unsubscribe': {'streams'},
    'set_stream': {'stream', 'fps', 'quality', 'scale'},
}

#: Optional on every client frame: an opaque correlation id echoed back in
#: the ack or error. Not part of any frame's own key set.
_ID_KEY = 'id'

#: Telemetry stream names a client may ask for. Unknown names are an
#: error rather than being ignored, for the same reason extra keys are.
#: The list lives in ``streams.py`` beside the policy that honours it;
#: re-exported here because it is part of the wire contract.
STREAMS = streams_mod.STREAMS

#: What a client that never sends ``subscribe`` receives. This is the
#: P0.1 set on purpose -- honouring subscriptions must not change what an
#: existing client sees, so only the NEW expensive streams are opt-in.
DEFAULT_STREAMS = streams_mod.DEFAULT_STREAMS


#: Refusals that get their own stable code rather than plain 'refused'.
#: A UI branches on these: "someone else is driving" resolves itself and
#: deserves a gentle note, while a generic refusal does not.
REFUSAL_CODES = {
    'not_in_control': 'another browser is driving; press STOP to take '
                      'over, or wait for it to let go',
}


class ProtocolError(ValueError):
    """
    A client frame that cannot be honoured, with a machine-readable code.

    Carries ``code`` (a stable, lowercase slug the UI may branch on) and
    ``frame_id`` (the client's correlation id, when it sent one) so the
    server can answer with a well-formed ``error`` frame.
    """

    def __init__(self, code, message, frame_id=None):
        super().__init__(message)
        self.code = code
        self.frame_id = frame_id


def mode_to_arbiter(ui_mode):
    """
    Translate a UI mode into the arbiter's spelling.

    The browser says 'auto' and 'stop'; cmd_vel_arbiter latches 'nav' and
    'idle'. Keeping the translation here is what lets the UI stay in
    product language while the ROS side keeps its own.
    """
    return _MODE_TO_ARBITER.get(ui_mode)


def _unique_object(pairs):
    """Reject duplicate keys rather than selecting an ambiguous command."""
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f'duplicate field {key!r}')
        result[key] = value
    return result


def _finite(value):
    """Check a JSON number without overflowing on a large integer."""
    try:
        return math.isfinite(value)
    except (TypeError, ValueError, OverflowError):
        return False


def decode(raw, colours=FALLBACK_COLOURS):
    """
    Parse and validate one client frame.

    Returns a dict with a ``type`` key plus that type's validated fields,
    and an ``id`` key which is None when the client sent no correlation id.
    Raises ProtocolError for anything malformed -- which, on this protocol,
    includes a frame that would have been perfectly valid rosbridge.
    """
    try:
        frame = json.loads(raw, object_pairs_hook=_unique_object)
    except (TypeError, ValueError, RecursionError) as exc:
        raise ProtocolError('bad_json', f'not valid JSON: {exc}') from exc
    if not isinstance(frame, dict):
        raise ProtocolError('bad_frame', 'frame must be a JSON object')

    frame_id = frame.get(_ID_KEY)
    if frame_id is not None and type(frame_id) not in (str, int):
        raise ProtocolError('bad_id', 'id must be a string or integer')

    kind = frame.get('type')
    if not isinstance(kind, str):
        raise ProtocolError('no_type', 'frame needs a string "type"', frame_id)
    if kind not in _CLIENT_SCHEMA:
        raise ProtocolError(
            'unknown_type',
            f'unknown frame type {kind!r}; this server speaks '
            f'{PROTOCOL_VERSION}, not the rosbridge protocol',
            frame_id)

    allowed = _CLIENT_SCHEMA[kind] | {'type', _ID_KEY}
    extra = sorted(set(frame) - allowed)
    if extra:
        # Rejected rather than ignored on purpose. A client sending
        # {"op": "publish", "topic": ...} is speaking rosbridge at a server
        # that is not one, and silently dropping those keys would leave it
        # believing it had just driven the robot.
        raise ProtocolError(
            'unexpected_fields',
            f'frame type {kind!r} does not accept {", ".join(extra)}',
            frame_id)

    out = _VALIDATORS[kind](frame, colours, frame_id)
    out['type'] = kind
    out[_ID_KEY] = frame_id
    return out


def _v_hello(frame, _colours, frame_id):
    """Validate a hello frame; a version mismatch is an error, not a warning."""
    wanted = frame.get('protocol', PROTOCOL_VERSION)
    if not isinstance(wanted, str):
        raise ProtocolError('bad_protocol', 'protocol must be a string', frame_id)
    if wanted != PROTOCOL_VERSION:
        raise ProtocolError(
            'protocol_mismatch',
            f'client speaks {wanted!r}, server speaks {PROTOCOL_VERSION!r}',
            frame_id)
    client = frame.get('client', '')
    if not isinstance(client, str):
        raise ProtocolError('bad_client', 'client must be a string', frame_id)
    # Binary support is DECLARED, never assumed. A client that does not
    # say it can parse a binary frame is never sent one, so adding binary
    # transport cannot break a client written before it existed --
    # which, for a protocol whose whole point is a stable contract, is
    # the difference between adding a feature and breaking one.
    binary = frame.get('binary', False)
    if not isinstance(binary, bool):
        raise ProtocolError('bad_binary', 'binary must be true or false',
                            frame_id)
    return {'protocol': wanted, 'client': client[:120], 'binary': binary}


def _v_ping(frame, _colours, frame_id):
    """Validate a ping frame, echoing whatever timestamp the client chose."""
    stamp = frame.get('t', 0)
    if (not isinstance(stamp, (int, float)) or isinstance(stamp, bool)
            or not _finite(stamp)):
        raise ProtocolError('bad_t', 't must be a number', frame_id)
    return {'t': stamp}


def _v_drive(frame, _colours, frame_id):
    """
    Validate a drive frame and clamp it to the panel's velocity limits.

    Clamping happens here, at the boundary, rather than in the publisher:
    every path that produces a velocity goes through decode(), so there is
    exactly one place where an out-of-range request can enter.
    """
    for key in ('linear', 'angular'):
        value = frame.get(key, 0.0)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ProtocolError('bad_velocity', f'{key} must be a number', frame_id)
        if not _finite(value):
            raise ProtocolError('bad_velocity', f'{key} must be finite', frame_id)
    linear, angular = safety.clamp_velocity(
        frame.get('linear', 0.0), frame.get('angular', 0.0))
    return {'linear': linear, 'angular': angular}


def _v_stop(_frame, _colours, _frame_id):
    """Validate a stop frame, which carries nothing at all."""
    return {}


def _v_set_mode(frame, _colours, frame_id):
    """Validate a mode change against the three modes the UI offers."""
    mode = frame.get('mode')
    if mode not in UI_MODES:
        raise ProtocolError(
            'bad_mode', f'mode must be one of {", ".join(UI_MODES)}', frame_id)
    return {'mode': mode}


def _v_select_target(frame, colours, frame_id):
    """Validate a target colour against the mission's own colour table."""
    colour = frame.get('colour')
    if colour not in tuple(colours):
        raise ProtocolError(
            'bad_colour',
            f'colour must be one of {", ".join(colours)}', frame_id)
    return {'colour': colour}


def _v_mission(frame, _colours, frame_id):
    """Validate a mission action; only start and abort exist."""
    action = frame.get('action')
    if action not in ('start', 'abort'):
        raise ProtocolError(
            'bad_action', 'action must be "start" or "abort"', frame_id)
    return {'action': action}


def _v_nav_goal(frame, _colours, frame_id):
    """
    Validate a map-frame navigation goal.

    Bounded generously rather than tightly: the map is the authority on
    what is reachable and Nav2 will refuse an unreachable pose itself. The
    bound exists so a NaN or a 1e300 cannot reach the planner.
    """
    out = {}
    for key in ('x', 'y'):
        value = frame.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ProtocolError('bad_goal', f'{key} must be a number', frame_id)
        if not _finite(value) or abs(value) > 1000.0:
            raise ProtocolError(
                'bad_goal', f'{key} must be finite and within 1000 m', frame_id)
        out[key] = float(value)
    return out


def _v_set_arm(frame, _colours, frame_id):
    """
    Validate shoulder/elbow angles; the server clamps to URDF limits.

    Only finiteness is enforced here. The real bound is
    ``coco_config.joint_limits.ARM_LIMITS``, which is checked against the
    URDF by coco_config's own tests -- so the server clamps against that
    rather than this module re-typing the numbers, which is exactly the
    drift teleop_arm_node's comment warns about and the old panel's HTML
    had already suffered.
    """
    out = {}
    for key in ('shoulder', 'elbow'):
        value = frame.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ProtocolError('bad_arm', f'{key} must be a number', frame_id)
        if not _finite(value):
            raise ProtocolError('bad_arm', f'{key} must be finite', frame_id)
        out[key] = float(value)
    return out


def _v_set_gripper(frame, _colours, frame_id):
    """Validate a gripper opening; the server clamps to URDF limits."""
    value = frame.get('grip')
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError('bad_grip', 'grip must be a number', frame_id)
    if not _finite(value):
        raise ProtocolError('bad_grip', 'grip must be finite', frame_id)
    return {'grip': float(value)}


def _v_streams(frame, _colours, frame_id):
    """
    Validate a stream list against STREAMS, for subscribe and unsubscribe.

    An unknown name is an error rather than a silent drop: a client that
    asks for "cameras" and is quietly given nothing will conclude the
    camera is broken, and go looking in the wrong place.
    """
    streams = frame.get('streams')
    if not isinstance(streams, list) or not all(
            isinstance(s, str) for s in streams):
        raise ProtocolError(
            'bad_streams', 'streams must be a list of strings', frame_id)
    unknown = streams_mod.known(streams)
    if unknown:
        raise ProtocolError(
            'unknown_stream',
            f'unknown stream(s) {", ".join(unknown)}; '
            f'known streams are {", ".join(STREAMS)}',
            frame_id)
    return {'streams': list(dict.fromkeys(streams))}


def _v_set_stream(frame, _colours, frame_id):
    """
    Validate a per-stream rate/quality request.

    Values are NOT clamped here, because the bounds are per stream and
    live in ``streams.LIMITS``; the server clamps when it applies them.
    What is checked here is that the stream is tunable at all and that
    the numbers are numbers -- the same split as ``set_arm``, whose real
    bounds live in ``coco_config``.
    """
    stream = frame.get('stream')
    if stream not in streams_mod.TUNABLE_STREAMS:
        raise ProtocolError(
            'bad_stream',
            f'stream must be one of '
            f'{", ".join(streams_mod.TUNABLE_STREAMS)}', frame_id)
    out = {'stream': stream}
    for key in ('fps', 'quality', 'scale'):
        if key not in frame:
            out[key] = None
            continue
        value = frame[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ProtocolError(
                'bad_stream_config', f'{key} must be a number', frame_id)
        if not _finite(value):
            raise ProtocolError(
                'bad_stream_config', f'{key} must be finite', frame_id)
        out[key] = value
    return out


_VALIDATORS = {
    'hello': _v_hello,
    'ping': _v_ping,
    'drive': _v_drive,
    'stop': _v_stop,
    'set_mode': _v_set_mode,
    'select_target': _v_select_target,
    'mission': _v_mission,
    'nav_goal': _v_nav_goal,
    'set_arm': _v_set_arm,
    'set_gripper': _v_set_gripper,
    'subscribe': _v_streams,
    'unsubscribe': _v_streams,
    'set_stream': _v_set_stream,
}


# ── server -> client frames ────────────────────────────────────────────
# Builders rather than free-form dicts so every frame that leaves the
# server has one definition, and the docs can be generated from one place.

def welcome(session, streams, limits, subscriptions=None, world=None):
    """
    Build the first frame: contract, session, limits and subscriptions.

    ``streams`` is the MJPEG descriptor map and ``subscriptions`` is the
    WebSocket stream document. The two words collide unhappily and the
    older one is load-bearing for existing clients, so both are kept and
    ``WEB_API.md`` says plainly which is which.
    """
    frame = {
        'type': 'welcome',
        'protocol': PROTOCOL_VERSION,
        'session': session,
        'streams': streams,
        'limits': limits,
        'commands': sorted(_CLIENT_SCHEMA),
    }
    if subscriptions is not None:
        frame['subscriptions'] = subscriptions
    if world is not None:
        frame['world'] = world
    return frame


def subscription(document):
    """Confirm a client's stream set after it changed."""
    return {'type': 'subscription', **document}


def ack(frame_id, command):
    """Confirm that a command was accepted and acted on."""
    return {'type': 'ack', 'id': frame_id, 'command': command, 'ok': True}


def error(code, message, frame_id=None):
    """Report a refused frame, with a stable code the UI may branch on."""
    return {'type': 'error', 'id': frame_id, 'code': code, 'message': message}


def pong(stamp):
    """Answer a ping, echoing the client's own timestamp for RTT."""
    return {'type': 'pong', 't': stamp}


def telemetry(seq, stamp, robot, mission, nav, sensors, platform):
    """
    Build one telemetry frame.

    ``seq`` is monotonic per connection so a client can detect drops; the
    server never re-sends, because stale telemetry is worse than a gap.
    """
    return {
        'type': 'telemetry',
        'seq': seq,
        't': stamp,
        'robot': robot,
        'mission': mission,
        'nav': nav,
        'sensors': sensors,
        'platform': platform,
    }


def map_frame(width, height, resolution, origin_x, origin_y, data_b64):
    """
    Build the occupancy-grid frame.

    Sent on connect and then only when the grid changes, never in the
    telemetry tick. A 320x320 grid is 102 400 cells; as a JSON array of
    integers at 10 Hz that would be several megabytes a second to redraw
    a picture that does not move. ``data_b64`` is base64 of the raw int8
    cells, which the browser decodes once with ``atob``.
    """
    return {
        'type': 'map',
        'width': width,
        'height': height,
        'resolution': resolution,
        'origin': {'x': origin_x, 'y': origin_y},
        'data': data_b64,
    }


def encode(frame):
    """
    Serialise a server frame to a JSON string.

    ``allow_nan=False`` on purpose: json.dumps emits bare ``NaN`` by
    default, which is not JSON and which ``JSON.parse`` rejects, so a
    single NaN leaking in from a sensor would break the client's whole
    stream rather than one field. Anything non-finite is scrubbed first.
    """
    return json.dumps(_scrub(frame), allow_nan=False, separators=(',', ':'))


def _scrub(value):
    """Recursively replace non-finite floats with None so encode cannot fail."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: _scrub(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_scrub(v) for v in value]
    return value
