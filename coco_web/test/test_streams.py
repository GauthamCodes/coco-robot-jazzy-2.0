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

"""Per-client subscriptions, rate limits and the bounded frame queue."""

from coco_web import protocol
from coco_web import streams as st

import pytest


# ── the default set, and why it is what it is ──────────────────────────

def test_a_new_client_gets_the_p01_default_set():
    """
    Honouring subscriptions must not change what an existing client sees.

    P0.1 sent every client telemetry, lidar, map and path. A client
    written against that release never sends `subscribe`, so that set has
    to remain the default or P0.2 silently breaks it.
    """
    sub = st.Subscription()
    for stream in ('telemetry', 'mission', 'lidar', 'map', 'path'):
        assert sub.wants(stream), stream


def test_the_expensive_streams_are_opt_in():
    """Camera and depth are never sent to a client that did not ask."""
    sub = st.Subscription(binary=True)
    assert not sub.wants('camera')
    assert not sub.wants('depth')


def test_subscribing_adds_without_dropping_the_defaults():
    """Asking for the camera must not cost a client its map."""
    sub = st.Subscription(binary=True)
    sub.subscribe(['camera'])
    assert sub.wants('camera')
    assert sub.wants('map')


def test_unsubscribing_removes_only_what_was_named():
    """Dropping lidar leaves the rest of the default set alone."""
    sub = st.Subscription()
    sub.unsubscribe(['lidar'])
    assert not sub.wants('lidar')
    assert sub.wants('map')
    assert sub.wants('path')


def test_telemetry_cannot_be_unsubscribed():
    """
    A client with no telemetry cannot find out that it is broken.

    The tick carries session state and readiness -- the things that tell
    a user WHY nothing is happening. Dropping it would leave a page that
    looks connected and is blind.
    """
    sub = st.Subscription()
    sub.unsubscribe(['telemetry'])
    assert sub.wants('telemetry')


# ── binary is declared, never assumed ──────────────────────────────────

def test_binary_streams_need_declared_binary_support():
    """
    A client that never said it speaks binary is never sent a Blob.

    Otherwise adding binary transport breaks every client written before
    it existed, which for a versioned contract is the whole thing it is
    supposed to prevent.
    """
    sub = st.Subscription(binary=False)
    sub.subscribe(['camera'])
    assert 'camera' in sub.streams
    assert sub.wants('camera') is False


def test_declaring_binary_unlocks_the_stream():
    """With support declared, the same subscription delivers."""
    sub = st.Subscription(binary=True)
    sub.subscribe(['camera'])
    assert sub.wants('camera') is True


# ── unknown names are refused, not ignored ─────────────────────────────

def test_unknown_stream_names_are_reported():
    """A typo must surface as an error, not as a silently dead stream."""
    assert st.known(['camera', 'cameras']) == ['cameras']
    assert st.known(['camera', 'lidar']) == []


def test_the_protocol_refuses_an_unknown_stream():
    """
    A client asking for 'cameras' is told, rather than given nothing.

    Silently dropping it makes the client conclude the camera is broken
    and go looking in the wrong place.
    """
    with pytest.raises(protocol.ProtocolError) as caught:
        protocol.decode('{"type":"subscribe","streams":["cameras"]}')
    assert caught.value.code == 'unknown_stream'


def test_unsubscribe_validates_the_same_way():
    """Both frames share a validator, so both refuse the same names."""
    with pytest.raises(protocol.ProtocolError) as caught:
        protocol.decode('{"type":"unsubscribe","streams":["nope"]}')
    assert caught.value.code == 'unknown_stream'


def test_unsubscribe_is_advertised():
    """The welcome frame must not omit a command the server accepts."""
    frame = protocol.welcome({}, {}, {})
    assert 'unsubscribe' in frame['commands']
    assert 'set_stream' in frame['commands']


# ── hello declares binary support ──────────────────────────────────────

def test_hello_defaults_to_no_binary():
    """Omitting the flag means text-only, which is the safe reading."""
    assert protocol.decode('{"type":"hello"}')['binary'] is False


def test_hello_accepts_declared_binary():
    """A client that says it parses binary frames is believed."""
    assert protocol.decode(
        '{"type":"hello","binary":true}')['binary'] is True


def test_hello_refuses_a_non_boolean_binary():
    """A truthy string must not quietly enable binary transport."""
    with pytest.raises(protocol.ProtocolError) as caught:
        protocol.decode('{"type":"hello","binary":"yes"}')
    assert caught.value.code == 'bad_binary'


# ── set_stream ─────────────────────────────────────────────────────────

def test_set_stream_only_accepts_tunable_streams():
    """The map has no frame rate; asking to set one is a mistake."""
    with pytest.raises(protocol.ProtocolError) as caught:
        protocol.decode('{"type":"set_stream","stream":"map","fps":5}')
    assert caught.value.code == 'bad_stream'


def test_set_stream_refuses_non_finite_numbers():
    """A non-finite rate must not reach a frame-interval division."""
    with pytest.raises(protocol.ProtocolError) as caught:
        protocol.decode('{"type":"set_stream","stream":"camera","fps":1e999}')
    assert caught.value.code == 'bad_stream_config'


def test_an_over_limit_rate_is_clamped_not_refused():
    """
    Asking for 60 fps means 'as fast as you can', not 'fail'.

    Same rule as velocity: clamp at the boundary rather than refusing,
    because a stick dragged past the pad should drive at full speed.
    """
    sub = st.Subscription()
    config = sub.configure('camera', fps=60.0)
    assert config['fps'] == st.LIMITS['camera']['fps_max']


def test_the_rate_cap_sits_below_the_sensor_rate():
    """
    /camera/image_raw publishes at 15 Hz, so 15 is the honest ceiling.

    Advertising more would only be a promise to drop.
    """
    assert st.LIMITS['camera']['fps_max'] <= 15.0


def test_quality_and_scale_are_clamped_too():
    """Every tunable has bounds, not just the rate."""
    sub = st.Subscription()
    config = sub.configure('camera', quality=1000, scale=99.0)
    assert config['quality'] == st.LIMITS['camera']['quality_max']
    assert config['scale'] == st.LIMITS['camera']['scale_max']


def test_junk_config_falls_back_to_the_default():
    """Nonsense becomes the default rather than raising mid-stream."""
    sub = st.Subscription()
    assert sub.configure('camera', fps='fast')['fps'] == \
        st.LIMITS['camera']['fps']


def test_one_clients_rate_does_not_affect_another():
    """Rate limiting is per client, or one slow phone slows every tab."""
    slow, fast = st.Subscription(), st.Subscription()
    slow.configure('camera', fps=1.0)
    assert slow.config['camera']['fps'] == 1.0
    assert fast.config['camera']['fps'] == st.LIMITS['camera']['fps']


# ── rate limiting ──────────────────────────────────────────────────────

def test_a_stream_is_not_due_again_immediately():
    """At 10 fps a second frame 10 ms later is early."""
    sub = st.Subscription()
    sub.configure('camera', fps=10.0)
    sub.mark_sent('camera', now=100.0)
    assert sub.due('camera', now=100.01) is False
    assert sub.due('camera', now=100.11) is True


# ── backpressure: the bounded queue ────────────────────────────────────

class _Pending:
    """A write future that resolves only when the test says so."""

    def __init__(self):
        self.finished = False

    def done(self):
        """Report whether the write has flushed."""
        return self.finished


def test_a_second_frame_is_dropped_while_the_first_is_in_flight():
    """
    The queue is one frame deep, per stream, per client.

    This is the bound that stops a slow browser growing ROS memory: the
    producer is a 15 Hz camera and the consumer may be a phone on a bad
    connection, so an unbounded queue grows until the node dies -- and
    the node is what holds the STOP button.
    """
    sub = st.Subscription()
    pending = _Pending()
    assert sub.ready('camera') is True
    sub.mark_sent('camera', pending)
    assert sub.ready('camera') is False
    pending.finished = True
    assert sub.ready('camera') is True


def test_a_dropped_frame_is_counted_so_the_client_can_be_told():
    """Loss must be visible, not silent."""
    sub = st.Subscription()
    assert sub.mark_dropped('camera') == 1
    assert sub.mark_dropped('camera') == 2
    assert sub.as_dict()['dropped']['camera'] == 2


def test_a_stalled_socket_blocks_even_a_first_frame():
    """
    A write can stay 'in flight' forever on a stalled TCP connection.

    So there is a second bound, on the transport's own buffer rather
    than on our bookkeeping.
    """
    sub = st.Subscription()
    assert sub.ready('camera', buffered=st.MAX_BUFFERED_BYTES + 1) is False
    assert sub.ready('camera', buffered=0) is True


def test_the_buffer_bound_is_a_real_limit():
    """A cap so large it never fires would not be a bound at all."""
    assert 0 < st.MAX_BUFFERED_BYTES <= 8 << 20


# ── the document a client is sent ──────────────────────────────────────

def test_the_subscription_document_is_json_safe():
    """It rides the same JSON encoder as every other frame."""
    protocol.encode(protocol.subscription(st.Subscription().as_dict()))


def test_the_document_advertises_what_is_available():
    """A client feature-detects instead of guessing at stream names."""
    document = st.Subscription().as_dict()
    assert set(document['available']) == set(st.STREAMS)
    assert set(document['default']) == set(st.DEFAULT_STREAMS)


def test_protocol_and_streams_agree_on_the_vocabulary():
    """
    One list, re-exported -- not two that can drift.

    The wire contract and the policy that honours it must name the same
    streams or a client can subscribe to something nothing will send.
    """
    assert protocol.STREAMS is st.STREAMS
    assert protocol.DEFAULT_STREAMS is st.DEFAULT_STREAMS


# ── the lidar dual-form bug, found live ────────────────────────────────
# `wants()` gated binary delivery on BINARY_STREAMS, which is only
# (camera, depth) -- but push_sensors also frames LIDAR as binary. So a
# client that said `binary: false` was sent binary lidar frames anyway.
# It surfaced as a UnicodeDecodeError in a probe calling json.loads on a
# byte string, which is exactly how a P0.1 client would have met it.

def test_a_text_client_is_never_sent_a_binary_lidar_frame():
    """
    The compatibility promise, for the stream that has two forms.

    lidar is BINARY_CAPABLE but not BINARY_ONLY: a client that declared
    no binary support still gets the scan, as JSON, in the telemetry
    tick. What it must never get is the frame.
    """
    sub = st.Subscription(binary=False)
    assert sub.wants('lidar') is True           # it still gets the scan
    assert sub.wants_binary('lidar') is False   # but not as a frame
    assert sub.wants_json_lidar() is True       # it comes in the tick


def test_a_binary_client_gets_the_frame_and_not_the_json_copy():
    """
    Sending both would double the cost binary framing exists to avoid.

    A 240-point scan is 480 bytes packed and roughly 2 kB as JSON;
    shipping both to the same client is worse than shipping neither
    format well.
    """
    sub = st.Subscription(binary=True)
    assert sub.wants_binary('lidar') is True
    assert sub.wants_json_lidar() is False


def test_unsubscribing_lidar_stops_both_forms():
    """Neither the frame nor the JSON block survives an unsubscribe."""
    for binary in (True, False):
        sub = st.Subscription(binary=binary)
        sub.unsubscribe(['lidar'])
        assert sub.wants_binary('lidar') is False
        assert sub.wants_json_lidar() is False


def test_every_binary_capable_stream_is_gated_on_declared_support():
    """
    The general form of the bug: no binary frame without a declaration.

    Asserted across the whole list rather than per stream, so a stream
    added to BINARY_CAPABLE later cannot quietly skip the gate.
    """
    text_only = st.Subscription(binary=False)
    text_only.subscribe(list(st.BINARY_CAPABLE))
    for stream in st.BINARY_CAPABLE:
        assert text_only.wants_binary(stream) is False, stream


def test_binary_capable_is_a_superset_of_binary_only():
    """
    A binary-ONLY stream must also be binary-CAPABLE.

    Structural: the two lists disagreeing in this direction is what let
    the lidar case through.
    """
    assert set(st.BINARY_STREAMS) <= set(st.BINARY_CAPABLE)


def test_every_stream_the_server_frames_is_binary_capable():
    """
    binary.py knows three streams; all three must be gated.

    A stream that binary.py can frame but streams.py does not list is
    one that bypasses the declaration check entirely -- which is the bug
    this file is about, in its general form.
    """
    from coco_web import binary as binary_mod
    assert set(binary_mod.STREAM_IDS) == set(st.BINARY_CAPABLE)


# ── per-client telemetry filtering ─────────────────────────────────────

def _frame():
    """Build a telemetry frame with every filterable section populated."""
    return {
        'mission': {'state': 'CLIMB'},
        'sensors': {'lidar': {'ranges': [1.0, 2.0]}, 'perception': {}},
        'nav': {'path': [[1.0, 2.0]], 'online': True},
    }


def test_a_default_text_client_sees_everything_it_did_before():
    """The P0.1 view, unchanged, for a client that never subscribed."""
    view = st.filter_telemetry(_frame(), st.Subscription(binary=False))
    assert view['sensors']['lidar'] is not None
    assert view['mission'] is not None
    assert view['nav']['path'] == [[1.0, 2.0]]


def test_a_binary_client_loses_only_the_duplicate_json_scan():
    """It has the scan already, as a frame. Everything else stays."""
    view = st.filter_telemetry(_frame(), st.Subscription(binary=True))
    assert view['sensors']['lidar'] is None
    assert view['mission'] is not None
    assert view['nav']['path'] == [[1.0, 2.0]]


def test_unsubscribing_blanks_the_section_rather_than_removing_it():
    """
    None is already the meaning before the first message arrives.

    Removing the key instead would make a client branch on existence,
    which is how a page ends up throwing on undefined mid-render.
    """
    sub = st.Subscription(binary=False)
    sub.unsubscribe(['mission', 'path'])
    view = st.filter_telemetry(_frame(), sub)
    assert 'mission' in view and view['mission'] is None
    assert view['nav']['path'] == []


def test_filtering_does_not_mutate_the_shared_frame():
    """
    The frame is built ONCE per tick and filtered per client.

    Mutating it in place would corrupt every client filtered after the
    first in the same tick -- presenting as one browser's LiDAR vanishing
    because a different browser unsubscribed.
    """
    frame = _frame()
    st.filter_telemetry(frame, st.Subscription(binary=True))
    assert frame['sensors']['lidar'] is not None
    assert frame['nav']['path'] == [[1.0, 2.0]]
    assert frame['mission'] is not None


def test_two_clients_in_one_tick_get_independent_views():
    """The property the previous test protects, stated end to end."""
    frame = _frame()
    text = st.filter_telemetry(frame, st.Subscription(binary=False))
    binary = st.filter_telemetry(frame, st.Subscription(binary=True))
    assert text['sensors']['lidar'] is not None
    assert binary['sensors']['lidar'] is None


def test_no_subscription_passes_the_frame_through():
    """A client with no subscription object is not a reason to crash."""
    frame = _frame()
    assert st.filter_telemetry(frame, None) is frame
