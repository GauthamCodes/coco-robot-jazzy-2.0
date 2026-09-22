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

r"""
The retained MJPEG view, re-served under a name the platform owns.

Pure module: no rclpy, no sockets, no tornado.

Why this exists
---------------
P0.1 sent the browser a web_video_server URL:
``http://host:8081/stream?topic=/perception/annotated&type=mjpeg``. That
is a ROS topic name on the wire, which coco.v1 says never happens -- and
a browser that can edit the query string can ask web_video_server for ANY
image topic on the graph. Codex found it (handoff blocker 6); it was the
one recorded exception to "no ROS names on the wire".

The platform now serves ``/video/<alias>`` itself, on its own origin. The
alias is one of a fixed few; the topic behind it is a server parameter
the browser never sees; and the platform fetches from web_video_server on
loopback and re-frames the parts. MJPEG is retained exactly as before --
same encoder, same quality -- it just stops leaking.

Re-framed, not relayed byte for byte, because a relay cannot drop: a
slow viewer would buffer web_video_server's whole output in this
process. Parsing whole JPEG parts lets the handler keep one part in
flight per viewer and drop the rest, the same rule the binary streams
follow.

Wire format read (web_video_server, MultipartStream)::

    --boundarydonotcross\r\n
    Content-type: image/jpeg\r\n
    X-Timestamp: 1234.567890\r\n
    Content-Length: N\r\n
    \r\n
    <N bytes of JPEG>\r\n--boundarydonotcross\r\n
    Content-type: ...

The parser keys on ``Content-Length`` and treats boundary lines as
separators, so the exact boundary string does not matter.
"""

#: The aliases a browser may name. Each maps to a server parameter, never
#: to a topic the browser supplies.
ALIASES = {
    'camera': 'camera_topic',
    'annotated': 'annotated_topic',
    'depth': 'depth_topic',
}

#: Our own multipart boundary.
BOUNDARY = 'cocoframe'

#: The response Content-Type for a re-framed stream.
CONTENT_TYPE = f'multipart/x-mixed-replace;boundary={BOUNDARY}'

#: Refuse a part bigger than this. A 320x240 JPEG is ~3.5 kB; this is the
#: binary protocol's payload bound, so neither path accepts more.
MAX_PART_BYTES = 8 << 20

#: Refuse a header block longer than this: web_video_server's is ~100 B.
MAX_HEADER_BYTES = 16 << 10


class MjpegError(ValueError):
    """The upstream stream is not the MJPEG this module reads."""


def descriptor(alias, port):
    """
    Build one stream descriptor for welcome and telemetry.

    ``path`` is on the platform's OWN origin. ``port`` is the platform's
    HTTP port, kept so a P0.1 client that builds ``host:port + path``
    still works; a client should resolve ``path`` against the page's
    origin, which also survives a mapped port.
    """
    return {
        'encoding': 'mjpeg',
        'path': f'/video/{alias}',
        'port': port,
    }


def upstream_request(topic, quality=60):
    """
    Build the loopback HTTP request to web_video_server for `topic`.

    Server side only: this string is the one place the topic travels,
    and it travels to web_video_server on 127.0.0.1, never to a browser.
    """
    from urllib.parse import quote
    return (f'GET /stream?topic={quote(topic, safe="/")}&type=mjpeg'
            f'&quality={int(quality)} HTTP/1.0\r\n'
            f'Host: 127.0.0.1\r\nConnection: close\r\n\r\n').encode('ascii')


def part(jpeg):
    """Frame one JPEG as a part of our multipart response."""
    return (f'--{BOUNDARY}\r\nContent-Type: image/jpeg\r\n'
            f'Content-Length: {len(jpeg)}\r\n\r\n').encode('ascii') \
        + bytes(jpeg) + b'\r\n'


def response_status(head):
    """Return the status code from an HTTP response head, or raise."""
    first = head.split(b'\r\n', 1)[0].decode('latin-1')
    fields = first.split()
    if len(fields) < 2 or not fields[0].startswith('HTTP/'):
        raise MjpegError('upstream did not answer HTTP')
    try:
        return int(fields[1])
    except ValueError as exc:
        raise MjpegError('upstream status is not a number') from exc


class MjpegParser:
    """Split a multipart MJPEG byte stream into whole JPEG parts."""

    def __init__(self):
        """Start between parts, with nothing buffered."""
        self._buffer = bytearray()
        self._need = None          # body bytes still to read, or None

    def feed(self, chunk):
        """
        Consume `chunk`; return the JPEG parts it completed, in order.

        Buffers at most one header block or one part: never more than
        MAX_PART_BYTES plus one chunk.
        """
        self._buffer.extend(chunk)
        parts = []
        while True:
            if self._need is None:
                end = self._buffer.find(b'\r\n\r\n')
                if end < 0:
                    if len(self._buffer) > MAX_HEADER_BYTES:
                        raise MjpegError('part header too long')
                    return parts
                self._need = self._length(bytes(self._buffer[:end]))
                del self._buffer[:end + 4]
                continue
            if len(self._buffer) < self._need:
                return parts
            parts.append(bytes(self._buffer[:self._need]))
            del self._buffer[:self._need]
            self._need = None

    @staticmethod
    def _length(block):
        """Read Content-Length from one part-header block."""
        for line in block.decode('latin-1').split('\r\n'):
            line = line.strip()
            if not line or line.startswith('--'):
                continue            # blank separator, or a boundary line
            name, _sep, value = line.partition(':')
            if name.strip().lower() != 'content-length':
                continue
            try:
                length = int(value.strip())
            except ValueError as exc:
                raise MjpegError('Content-Length is not a number') from exc
            if not 0 < length <= MAX_PART_BYTES:
                raise MjpegError('part size out of bounds')
            return length
        raise MjpegError('part without Content-Length')
