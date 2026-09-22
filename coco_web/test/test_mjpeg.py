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

"""The MJPEG re-framer: aliases out, topics never; parts whole, bounded."""

from coco_web import mjpeg

import pytest


def _upstream(parts, boundary=b'boundarydonotcross'):
    """Encode `parts` the way web_video_server's MultipartStream does."""
    out = b'--' + boundary + b'\r\n'
    for jpeg in parts:
        out += (b'Content-type: image/jpeg\r\nX-Timestamp: 12.500000\r\n'
                b'Content-Length: %d\r\n\r\n' % len(jpeg) + jpeg
                + b'\r\n--' + boundary + b'\r\n')
    return out


PARTS = [b'\xff\xd8' + bytes([i]) * (10 + i) + b'\xff\xd9' for i in range(5)]


def test_whole_parts_come_out_in_order():
    """Everything web_video_server sends, and only that."""
    assert mjpeg.MjpegParser().feed(_upstream(PARTS)) == PARTS


@pytest.mark.parametrize('size', [1, 2, 3, 7, 64])
def test_any_chunking_gives_the_same_parts(size):
    """TCP delivers arbitrary slices; a part split anywhere still parses."""
    data = _upstream(PARTS)
    parser = mjpeg.MjpegParser()
    got = []
    for start in range(0, len(data), size):
        got.extend(parser.feed(data[start:start + size]))
    assert got == PARTS


def test_a_body_containing_the_boundary_is_not_split():
    """Content-Length decides, not a search for the boundary."""
    tricky = b'\xff\xd8\r\n--boundarydonotcross\r\n\r\n\xff\xd9'
    assert mjpeg.MjpegParser().feed(_upstream([tricky])) == [tricky]


def test_the_boundary_string_does_not_matter():
    """Boundary lines are separators, whatever they say."""
    assert mjpeg.MjpegParser().feed(
        _upstream(PARTS, b'somethingelse')) == PARTS


def test_our_own_parts_parse_back():
    """The re-framed output is itself well-formed MJPEG."""
    data = b''.join(mjpeg.part(jpeg) for jpeg in PARTS)
    assert mjpeg.MjpegParser().feed(data) == PARTS


@pytest.mark.parametrize('header', [
    b'Content-type: image/jpeg\r\n\r\n',                  # no length
    b'Content-Length: nine\r\n\r\n',                      # not a number
    b'Content-Length: 0\r\n\r\n',                         # empty part
    b'Content-Length: %d\r\n\r\n' % (mjpeg.MAX_PART_BYTES + 1),
])
def test_malformed_or_oversized_parts_are_refused(header):
    """A part the parser cannot bound is an error, never a huge buffer."""
    with pytest.raises(mjpeg.MjpegError):
        mjpeg.MjpegParser().feed(b'--b\r\n' + header)


def test_an_endless_header_is_refused_not_buffered():
    """No Content-Length ever arriving must not grow memory."""
    parser = mjpeg.MjpegParser()
    with pytest.raises(mjpeg.MjpegError):
        for _ in range(100):
            parser.feed(b'X-Junk: ' + b'a' * 1000 + b'\r\n')


def test_the_descriptor_names_an_alias_and_no_topic():
    """What the browser is told: a same-origin path, nothing ROS."""
    for alias in mjpeg.ALIASES:
        descriptor = mjpeg.descriptor(alias, 8080)
        assert descriptor == {'encoding': 'mjpeg',
                              'path': f'/video/{alias}', 'port': 8080}
        assert 'topic' not in repr(descriptor)


def test_only_the_loopback_request_carries_the_topic():
    """The topic travels once, to web_video_server, encoded."""
    request = mjpeg.upstream_request('/perception/annotated')
    assert request.startswith(
        b'GET /stream?topic=/perception/annotated&type=mjpeg&quality=60 ')
    assert b'Host: 127.0.0.1' in request
    # a topic cannot smuggle a second parameter or a header
    hostile = mjpeg.upstream_request('/a&topic=/b\r\nX: y')
    assert b'\r\nX: y' not in hostile
    assert hostile.count(b'&topic=') == 0


def test_the_upstream_status_line_is_read_or_refused():
    """A 200 is a stream; anything unreadable is an error."""
    assert mjpeg.response_status(b'HTTP/1.0 200 OK\r\nX: y\r\n\r\n') == 200
    assert mjpeg.response_status(b'HTTP/1.1 404 Not Found\r\n\r\n') == 404
    for bad in (b'SSH-2.0-OpenSSH\r\n\r\n', b'HTTP/1.0 abc\r\n\r\n', b''):
        with pytest.raises(mjpeg.MjpegError):
            mjpeg.response_status(bad)


def test_every_alias_maps_to_a_parameter_not_a_topic():
    """The alias table holds parameter NAMES; topics live in parameters."""
    assert mjpeg.ALIASES == {'camera': 'camera_topic',
                             'annotated': 'annotated_topic',
                             'depth': 'depth_topic'}
