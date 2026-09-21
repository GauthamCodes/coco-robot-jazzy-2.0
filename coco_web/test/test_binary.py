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

"""The binary sensor frame: round trips, and every malformed shape."""

import json
import struct

from coco_web import binary

import pytest


def _frame(stream, header, payload=b''):
    """Provide complete metadata while testing the existing envelope."""
    fields = {'seq': 1, 't': 0.0, 'dropped': 0}
    if stream == 'lidar':
        fields.update(count=len(payload) // 2, angle_min=0, angle_step=0,
                      scale=1000, no_return=0)
    else:
        fields.update(w=320, h=240, enc='jpeg')
        if stream == 'depth':
            fields.update(min_m=0.1, max_m=8.0)
    fields.update(header)
    return binary.encode_frame(stream, fields, payload)


# ── round trips ────────────────────────────────────────────────────────

def test_a_frame_round_trips():
    """What the encoder writes, the reference reader reads back."""
    blob = _frame('camera', {'seq': 7, 'w': 320}, b'\xff\xd8')
    stream, header, payload = binary.decode_frame(blob)
    assert stream == 'camera'
    assert header['seq'] == 7
    assert header['w'] == 320
    assert payload == b'\xff\xd8'


def test_the_header_names_its_own_stream():
    """
    A client reads the stream from the header, not from the id table.

    The numeric id exists only to keep the fixed prefix a fixed size; the
    name is the contract, so a client never has to keep a copy of
    STREAM_IDS in sync with the server.
    """
    _stream, header, _payload = binary.decode_frame(
        _frame('depth', {'seq': 1}))
    assert header['stream'] == 'depth'


def test_an_empty_payload_is_legal():
    """A header-only frame is a valid frame, not a truncated one."""
    stream, _header, payload = binary.decode_frame(
        _frame('lidar', {'seq': 1}))
    assert stream == 'lidar'
    assert payload == b''


def test_the_length_field_is_big_endian():
    """
    DataView.getUint16 is big-endian by default, so this is too.

    A little-endian length would work in Python and need an explicit
    flag plus a comment in every browser that reads it.
    """
    blob = _frame('camera', {'seq': 1})
    declared = struct.unpack('>H', blob[6:8])[0]
    assert declared == len(blob) - binary.PREFIX_SIZE


def test_encoding_an_unknown_stream_is_refused():
    """Only the three binary streams may be framed."""
    with pytest.raises(binary.BinaryFrameError) as caught:
        _frame('telemetry', {})
    assert caught.value.code == 'unknown_stream'


# ── malformed frames ───────────────────────────────────────────────────
# The half that matters: a format whose failure modes are untested is a
# format that fails in someone's browser console instead.

def test_a_frame_without_the_magic_is_refused():
    """Four magic bytes, so a misrouted frame cannot parse as nonsense."""
    blob = bytearray(_frame('camera', {'seq': 1}))
    blob[0:4] = b'XXXX'
    with pytest.raises(binary.BinaryFrameError) as caught:
        binary.decode_frame(bytes(blob))
    assert caught.value.code == 'bad_magic'


def test_a_frame_shorter_than_the_prefix_is_refused():
    """A few stray bytes must not index past the end of the buffer."""
    with pytest.raises(binary.BinaryFrameError) as caught:
        binary.decode_frame(b'CO')
    assert caught.value.code == 'short_frame'


def test_an_empty_frame_is_refused():
    """Zero bytes is the degenerate case of the same check."""
    with pytest.raises(binary.BinaryFrameError) as caught:
        binary.decode_frame(b'')
    assert caught.value.code == 'short_frame'


def test_a_future_format_version_is_refused():
    """
    A client must not read a layout it does not know.

    The version covers the LAYOUT, not the header fields: adding a
    header key is additive and must not bump it, exactly as in the JSON
    protocol.
    """
    blob = bytearray(_frame('camera', {'seq': 1}))
    blob[4] = binary.FORMAT_VERSION + 1
    with pytest.raises(binary.BinaryFrameError) as caught:
        binary.decode_frame(bytes(blob))
    assert caught.value.code == 'bad_version'


def test_a_header_length_past_the_buffer_is_refused():
    """
    The length field indexes into the buffer, so it is checked.

    Unchecked it silently yields a short slice, and the failure surfaces
    as a JSON error somewhere much less informative than here.
    """
    blob = bytearray(_frame('camera', {'seq': 1}, b'body'))
    blob[6:8] = struct.pack('>H', 4000)
    with pytest.raises(binary.BinaryFrameError) as caught:
        binary.decode_frame(bytes(blob))
    assert caught.value.code == 'truncated_header'


def test_an_absurd_header_length_is_refused_before_slicing():
    """A 16-bit length can claim 65 535 bytes; a real header is hundreds."""
    blob = bytearray(_frame('camera', {'seq': 1}))
    blob[6:8] = struct.pack('>H', 0xFFFF)
    with pytest.raises(binary.BinaryFrameError) as caught:
        binary.decode_frame(bytes(blob))
    assert caught.value.code == 'header_too_large'


def test_a_non_json_header_is_refused():
    """Junk where the header should be fails with a code, not a traceback."""
    prefix = struct.pack(
        binary.HEADER_STRUCT, binary.MAGIC, binary.FORMAT_VERSION, 2, 5)
    with pytest.raises(binary.BinaryFrameError) as caught:
        binary.decode_frame(prefix + b'{{{{{')
    assert caught.value.code == 'bad_header_json'


def test_a_non_utf8_header_is_refused():
    """Bytes that are not text fail as an encoding error, distinctly."""
    prefix = struct.pack(
        binary.HEADER_STRUCT, binary.MAGIC, binary.FORMAT_VERSION, 2, 4)
    with pytest.raises(binary.BinaryFrameError) as caught:
        binary.decode_frame(prefix + b'\xff\xfe\xfd\xfc')
    assert caught.value.code == 'bad_header_encoding'


def test_a_header_that_is_not_an_object_is_refused():
    """A JSON array parses, but is not a header."""
    body = json.dumps([1, 2, 3]).encode('utf-8')
    prefix = struct.pack(
        binary.HEADER_STRUCT, binary.MAGIC, binary.FORMAT_VERSION, 2,
        len(body))
    with pytest.raises(binary.BinaryFrameError) as caught:
        binary.decode_frame(prefix + body)
    assert caught.value.code == 'bad_header'


def test_an_unknown_stream_id_is_refused():
    """An id this build does not know must not decode as a guess."""
    prefix = struct.pack(
        binary.HEADER_STRUCT, binary.MAGIC, binary.FORMAT_VERSION, 99, 2)
    with pytest.raises(binary.BinaryFrameError) as caught:
        binary.decode_frame(prefix + b'{}')
    assert caught.value.code == 'unknown_stream'


def test_an_oversized_header_is_refused_at_encode_time():
    """The bound is enforced on the way out as well as the way in."""
    with pytest.raises(binary.BinaryFrameError) as caught:
        _frame('camera', {'pad': 'x' * (binary.MAX_HEADER_BYTES)})
    assert caught.value.code == 'header_too_large'


# ── LiDAR frames ───────────────────────────────────────────────────────

SCAN = {
    'angle_min': -2.0944,
    'angle_step': 0.0175,
    'ranges': [1.234, None, 0.15, 12.0],
    'floor': 0.15,
}


def test_lidar_ranges_survive_the_millimetre_round_trip():
    """
    A millimetre is finer than the sensor's own 0.01 m resolution.

    So nothing observable is lost by sending uint16 mm instead of floats.
    """
    _stream, header, payload = binary.decode_frame(
        binary.lidar_frame(1, 100.0, SCAN))
    values = struct.unpack(f'>{header["count"]}H', payload)
    assert values[0] == 1234
    assert values[2] == 150
    assert values[3] == 12000


def test_a_lidar_no_return_is_zero_not_a_range():
    """
    None means no return, and must not be drawn as a wall.

    Zero is unambiguous because the sensor never reports below its own
    0.15 m floor.
    """
    _stream, _header, payload = binary.decode_frame(
        binary.lidar_frame(1, 100.0, SCAN))
    values = struct.unpack('>4H', payload)
    assert values[1] == binary.LIDAR_NO_RETURN == 0


def test_the_lidar_header_carries_everything_needed_to_plot_it():
    """A client must not need a second source to place the returns."""
    _stream, header, _payload = binary.decode_frame(
        binary.lidar_frame(4, 100.0, SCAN))
    assert header['angle_min'] == pytest.approx(-2.0944)
    assert header['angle_step'] == pytest.approx(0.0175)
    assert header['count'] == 4
    assert header['scale'] == binary.LIDAR_SCALE_MM
    assert header['floor'] == 0.15
    assert header['no_return'] == binary.LIDAR_NO_RETURN


def test_the_lidar_frame_is_far_smaller_than_its_json():
    """
    480 bytes of payload against roughly two kilobytes of JSON.

    This is the whole reason the format exists, so it is asserted rather
    than assumed.
    """
    scan = {'angle_min': -2.0944, 'angle_step': 0.0175, 'floor': 0.15,
            'ranges': [1.234] * 240}
    blob = binary.lidar_frame(1, 100.0, scan)
    as_json = json.dumps({'type': 'lidar', 'seq': 1, **scan})
    assert len(blob) < len(as_json) / 2


def test_an_out_of_range_reading_is_clamped_not_wrapped():
    """
    A range past 65.535 m would wrap to a short one, which draws a wall.

    The sensor maxes at 12 m so this cannot happen today; it is clamped
    anyway because a wrapped value is indistinguishable from a real
    obstacle.
    """
    scan = dict(SCAN, ranges=[999.0])
    _stream, _header, payload = binary.decode_frame(
        binary.lidar_frame(1, 100.0, scan))
    assert struct.unpack('>H', payload)[0] == 0xFFFF


def test_an_empty_scan_frames_cleanly():
    """No scan yet is a state, not an exception."""
    _stream, header, payload = binary.decode_frame(
        binary.lidar_frame(1, 100.0, None))
    assert header['count'] == 0
    assert payload == b''


def test_drops_are_reported_in_the_next_frame():
    """
    Loss is visible to the client rather than silent.

    A client that missed frames should be able to say so instead of
    showing stale data as though it were current.
    """
    _stream, header, _payload = binary.decode_frame(
        binary.lidar_frame(1, 100.0, SCAN, dropped=12))
    assert header['dropped'] == 12


# ── image frames ───────────────────────────────────────────────────────

def test_an_image_frame_carries_its_own_dimensions():
    """A client must not have to remember what it negotiated."""
    _stream, header, payload = binary.decode_frame(
        binary.image_frame('camera', 3, 100.0, 320, 240, b'\xff\xd8',
                           quality=60))
    assert (header['w'], header['h']) == (320, 240)
    assert header['enc'] == 'jpeg'
    assert header['quality'] == 60
    assert payload == b'\xff\xd8'


def test_a_depth_frame_carries_the_metre_range_it_mapped():
    """
    Without the range, a depth picture is an unlabelled gradient.

    `extra` is how the stream says what its shades mean.
    """
    _stream, header, _payload = binary.decode_frame(
        binary.image_frame('depth', 1, 100.0, 320, 240, b'\xff\xd8',
                           extra={'min_m': 0.1, 'max_m': 8.0}))
    assert header['min_m'] == 0.1
    assert header['max_m'] == 8.0


def test_no_frame_header_can_carry_a_topic():
    """
    The binary path names no ROS topic, in either direction.

    The MJPEG descriptor in `welcome` still does, because a
    web_video_server URL requires one -- but nothing here needs it, so
    nothing here has it.
    """
    for blob in (binary.lidar_frame(1, 100.0, SCAN),
                 binary.image_frame('camera', 1, 100.0, 320, 240, b'')):
        _stream, header, _payload = binary.decode_frame(blob)
        for key, value in header.items():
            assert 'topic' not in key
            if isinstance(value, str):
                assert not value.startswith('/')
