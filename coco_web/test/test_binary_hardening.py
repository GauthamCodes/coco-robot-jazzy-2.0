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

"""Validate sensor metadata and lengths, not just the binary prefix."""

import json
import math
import struct

from coco_web import binary

import pytest


def raw_frame(stream, header, payload=b''):
    """Construct unchecked wire bytes to exercise the decoder boundary."""
    encoded = json.dumps(header).encode()
    return struct.pack(binary.HEADER_STRUCT, binary.MAGIC, 1,
                       binary.STREAM_IDS[stream], len(encoded)) + encoded + payload


def camera_header(**fields):
    """Build a valid camera header with one controlled mutation."""
    return dict(stream='camera', seq=1, t=2.0, w=320, h=240,
                enc='jpeg', dropped=0, payload_bytes=4, **fields)


@pytest.mark.parametrize('key,value', [
    ('stream', 'depth'), ('seq', -1), ('seq', True), ('seq', 1.5),
    ('t', math.nan), ('t', math.inf), ('t', 'now'), ('w', 0),
    ('h', -1), ('w', True), ('enc', 'cdr'), ('payload_bytes', 3),
    ('payload_bytes', -1), ('payload_bytes', True), ('dropped', -2),
])
def test_bad_metadata_is_refused(key, value):
    """Plausible JSON can still be an impossible sensor frame."""
    header = camera_header()
    header[key] = value
    with pytest.raises(binary.BinaryFrameError):
        binary.decode_frame(raw_frame('camera', header, b'jpeg'))


@pytest.mark.parametrize('key', ['seq', 't', 'w', 'h', 'enc', 'stream'])
def test_required_metadata_is_required(key):
    """The decoder must be able to describe the payload independently."""
    header = camera_header()
    del header[key]
    with pytest.raises(binary.BinaryFrameError):
        binary.decode_frame(raw_frame('camera', header, b'jpeg'))


@pytest.mark.parametrize('delta', [-1, 1])
def test_sensor_payload_length_is_exact(delta):
    """Both truncation and appended garbage invalidate a sized payload."""
    blob = binary.image_frame('camera', 1, 2, 320, 240, b'jpeg')
    bad = blob[:delta] if delta < 0 else blob + b'x'
    with pytest.raises(binary.BinaryFrameError):
        binary.decode_frame(bad)


def test_lidar_payload_count_is_exact_even_for_legacy_frames():
    """The ray count bounds every uint16 read in the browser."""
    blob = binary.lidar_frame(7, 2.5, {'ranges': [1.0] * 480})
    stream, header, payload = binary.decode_frame(blob)
    assert len(payload) == 960
    assert header['seq'] == 7 and header['t'] == 2.5
    header.pop('payload_bytes', None)
    with pytest.raises(binary.BinaryFrameError):
        binary.decode_frame(raw_frame(stream, header, payload[:-2]))


def test_invalid_lidar_returns_do_not_crash_or_draw_near_obstacles():
    """NaN, infinity, negatives and absent returns all encode as zero."""
    blob = binary.lidar_frame(1, 2, {
        'ranges': [None, math.nan, math.inf, -math.inf, -1.0, 0.0, 1.25]})
    _, header, payload = binary.decode_frame(blob)
    assert header['count'] == 7
    assert struct.unpack('>7H', payload) == (0, 0, 0, 0, 0, 0, 1250)


def test_depth_range_cannot_be_inverted():
    """A false metric legend must be refused."""
    with pytest.raises(binary.BinaryFrameError):
        binary.image_frame('depth', 1, 2, 8, 8, b'jpeg',
                           extra={'min_m': 8, 'max_m': 1})


def test_extra_metadata_cannot_override_image_identity():
    """Depth additions cannot silently replace sequence or encoding."""
    with pytest.raises(binary.BinaryFrameError):
        binary.image_frame('camera', 1, 2, 8, 8, b'jpeg',
                           extra={'seq': 99})


def test_metadata_cannot_name_ros_access():
    """A generic envelope is not an escape hatch for ROS serialization."""
    with pytest.raises(binary.BinaryFrameError):
        binary.encode_frame('camera',
                            dict(camera_header(), topic='/camera/raw'), b'jpeg')


@pytest.mark.parametrize('value', [None, 128, {}, 'COCO'])
def test_binary_input_requires_a_byte_buffer(value):
    """An integer must never be interpreted as an allocation size."""
    with pytest.raises(binary.BinaryFrameError) as caught:
        binary.decode_frame(value)
    assert caught.value.code == 'bad_buffer'


def test_payload_limit_is_enforced_before_copying():
    """A producer cannot frame an oversized sensor payload."""
    with pytest.raises(binary.BinaryFrameError):
        binary.image_frame('camera', 1, 2, 8, 8,
                           b'x' * (binary.MAX_PAYLOAD_BYTES + 1))
