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
The binary WebSocket frame sensor data travels in.

Pure module: no rclpy, no sockets, no OpenCV. It builds and parses bytes,
so the whole wire format is unit-tested on a bare interpreter -- including
every malformed shape, which is the half that matters.

Why binary at all
-----------------
JSON is the right carrier for commands and state: it is self-describing,
diffable, and a human can read it out of a browser console at three in
the morning. It is the wrong carrier for pixels. A 320x240 JPEG is about
ten kilobytes; base64 inside a JSON string is thirteen, and every one of
those bytes has to be escaped, parsed and garbage-collected on a phone.

So control and metadata stay JSON text frames, and sensor payloads move
as binary frames. The split is along a real seam: text frames are things
a person might want to read, binary frames are things a machine decodes.

The format
----------
::

    off  size  field
    0    4     magic  b'COCO'
    4    1     format version
    5    1     stream id
    6    2     header length, big-endian uint16
    8    H     UTF-8 JSON header
    8+H  ...   payload

Big-endian because ``DataView.getUint16(offset)`` is big-endian by
default: the browser reads this with no flag and no comment explaining a
flag.

**Every frame is self-describing.** The JSON header carries the stream
name, sequence number and timestamp, plus whatever that stream needs to
be interpreted -- image dimensions and encoding, or the LiDAR's angular
step and scale factor. A client never has to know a frame's shape in
advance, and never has to guess: if the header does not say it, it is not
assumed.

What is deliberately NOT here
-----------------------------
A serialized ROS message. The payload is always something this module
constructed -- JPEG bytes, or an array packed here -- never a
``sensor_msgs`` blob passed through. Putting CDR on a public socket would
make the browser a ROS client, which is exactly the boundary the closed
command vocabulary exists to hold: a client that can deserialize a ROS
message is one library away from producing one.

The header also names no topic. The MJPEG descriptor in ``welcome`` still
does, because a ``web_video_server`` URL requires one, but nothing in
this path does -- and no field here could carry one.
"""

import json
import struct

#: Frame marker. Four bytes, so a truncated or misrouted frame is
#: rejected rather than parsed into nonsense.
MAGIC = b'COCO'

#: Format version, bumped when the LAYOUT changes -- not when a header
#: field is added, which clients must tolerate exactly as they must in
#: the JSON protocol.
FORMAT_VERSION = 1

#: Fixed-size prefix: magic, version, stream id, header length.
HEADER_STRUCT = '>4sBBH'
PREFIX_SIZE = struct.calcsize(HEADER_STRUCT)

#: Stream name -> id on the wire. Ids rather than names in the fixed
#: prefix so the prefix stays a fixed size; the name is in the JSON
#: header anyway, so a client never has to keep this table.
STREAM_IDS = {
    'lidar': 1,
    'camera': 2,
    'depth': 3,
}

STREAM_NAMES = {value: key for key, value in STREAM_IDS.items()}

#: Largest header this parser will believe. A 16-bit length can claim
#: 65 535 bytes; a legitimate header is a few hundred.
MAX_HEADER_BYTES = 8192

#: LiDAR ranges are sent as uint16 millimetres. 12 m of range needs
#: 12 000 counts, so a 65 535 ceiling leaves room to spare, and a
#: millimetre is already finer than the sensor's own 0.01 m resolution.
LIDAR_SCALE_MM = 1000.0

#: The uint16 that means "no return". Zero is unambiguous: a real range
#: below the 0.15 m floor is not reported by the sensor at all.
LIDAR_NO_RETURN = 0


class BinaryFrameError(ValueError):
    """A binary frame that cannot be parsed, with a machine-readable code."""

    def __init__(self, code, message):
        """Carry a stable slug alongside the human-readable message."""
        super().__init__(message)
        self.code = code


def encode_frame(stream, header, payload=b''):
    """
    Build one binary frame.

    ``header`` is enriched with the stream name so the payload can be
    interpreted without consulting STREAM_IDS -- the id in the prefix is
    a transport detail, the name is the contract.
    """
    stream_id = STREAM_IDS.get(stream)
    if stream_id is None:
        raise BinaryFrameError(
            'unknown_stream', f'{stream!r} is not a binary stream')
    body = dict(header)
    body['stream'] = stream
    blob = json.dumps(body, separators=(',', ':')).encode('utf-8')
    if len(blob) > MAX_HEADER_BYTES:
        raise BinaryFrameError(
            'header_too_large',
            f'header is {len(blob)} bytes, limit is {MAX_HEADER_BYTES}')
    prefix = struct.pack(
        HEADER_STRUCT, MAGIC, FORMAT_VERSION, stream_id, len(blob))
    return prefix + blob + bytes(payload)


def decode_frame(blob):
    """
    Parse one binary frame into ``(stream, header, payload)``.

    Ships beside the encoder so the format has a reference reader: every
    malformed shape below is a test, and a format whose failure modes are
    untested is a format that fails in a browser console instead.
    """
    data = bytes(blob)
    if len(data) < PREFIX_SIZE:
        raise BinaryFrameError(
            'short_frame',
            f'frame is {len(data)} bytes, prefix alone is {PREFIX_SIZE}')
    magic, version, stream_id, header_len = struct.unpack(
        HEADER_STRUCT, data[:PREFIX_SIZE])
    if magic != MAGIC:
        raise BinaryFrameError(
            'bad_magic', f'frame does not start with {MAGIC!r}')
    if version != FORMAT_VERSION:
        raise BinaryFrameError(
            'bad_version',
            f'frame is format {version}, this build speaks '
            f'{FORMAT_VERSION}')
    if header_len > MAX_HEADER_BYTES:
        raise BinaryFrameError(
            'header_too_large',
            f'header claims {header_len} bytes, limit is '
            f'{MAX_HEADER_BYTES}')
    end = PREFIX_SIZE + header_len
    if end > len(data):
        # The length field is attacker-or-bug controlled and indexes into
        # the buffer. Unchecked, it silently yields a short slice and the
        # JSON parse fails somewhere much less informative.
        raise BinaryFrameError(
            'truncated_header',
            f'header claims {header_len} bytes but only '
            f'{len(data) - PREFIX_SIZE} remain')
    try:
        header = json.loads(data[PREFIX_SIZE:end].decode('utf-8'))
    except UnicodeDecodeError as exc:
        raise BinaryFrameError(
            'bad_header_encoding', f'header is not UTF-8: {exc}') from exc
    except ValueError as exc:
        raise BinaryFrameError(
            'bad_header_json', f'header is not JSON: {exc}') from exc
    if not isinstance(header, dict):
        raise BinaryFrameError(
            'bad_header', 'header must be a JSON object')
    stream = STREAM_NAMES.get(stream_id)
    if stream is None:
        raise BinaryFrameError(
            'unknown_stream', f'stream id {stream_id} is not known')
    return stream, header, data[end:]


def lidar_frame(seq, stamp, scan, dropped=0):
    """
    Build a LiDAR frame from a downsampled scan payload.

    ``scan`` is ``telemetry.downsample_scan``'s output, so the reduction
    still happens once at the source rather than per client. The ranges
    become uint16 millimetres: 240 points is 480 bytes against roughly
    two kilobytes of JSON, and a millimetre is finer than the sensor's
    own 0.01 m range resolution, so nothing observable is lost.
    """
    ranges = (scan or {}).get('ranges') or []
    packed = bytearray()
    for value in ranges:
        if value is None:
            packed += struct.pack('>H', LIDAR_NO_RETURN)
            continue
        millimetres = int(round(float(value) * LIDAR_SCALE_MM))
        packed += struct.pack('>H', max(0, min(0xFFFF, millimetres)))
    header = {
        'seq': seq,
        't': stamp,
        'count': len(ranges),
        'angle_min': (scan or {}).get('angle_min', 0.0),
        'angle_step': (scan or {}).get('angle_step', 0.0),
        'scale': LIDAR_SCALE_MM,
        'floor': (scan or {}).get('floor'),
        'no_return': LIDAR_NO_RETURN,
        'dropped': dropped,
    }
    return encode_frame('lidar', header, bytes(packed))


def image_frame(stream, seq, stamp, width, height, jpeg, quality=None,
                dropped=0, extra=None):
    """
    Build a camera or depth frame around already-encoded JPEG bytes.

    ``extra`` carries whatever that stream needs to be read honestly --
    for depth, the metre range the greyscale ramp spans, so a client can
    label a legend instead of showing an unscaled picture.
    """
    header = {
        'seq': seq,
        't': stamp,
        'w': width,
        'h': height,
        'enc': 'jpeg',
        'dropped': dropped,
    }
    if quality is not None:
        header['quality'] = quality
    if extra:
        header.update(extra)
    return encode_frame(stream, header, jpeg)
