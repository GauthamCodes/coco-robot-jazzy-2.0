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
The page's binary decoder, run in Node against the Python encoder's bytes.

Codex's cross-language approach (test_frontend_transport.py on
codex/p02-hardening), pointed at the decoder the page actually loads --
web/frame.js -- rather than a standalone module nothing loads.
"""

import base64
import json
import os
from pathlib import Path
import shutil
import struct
import subprocess

from coco_web import binary

import pytest

HERE = Path(__file__).resolve().parent


def _b64(blob):
    """Encode bytes for the JSON fixture file."""
    return base64.b64encode(blob).decode('ascii')


def _fixtures():
    """Build valid frames with expected values, and broken ones by name."""
    lidar = binary.lidar_frame(
        7, 12.5, {'ranges': [1.234, None, 2.0], 'angle_min': -1.0,
                  'angle_step': 0.5, 'floor': 0.15}, dropped=3)
    camera = binary.image_frame('camera', 8, 12.5, 8, 6,
                                b'\xff\xd8jpeg\xff\xd9', quality=60)
    depth = binary.image_frame(
        'depth', 9, 12.5, 8, 6, b'\xff\xd8jpeg\xff\xd9',
        extra={'min_m': 0.1, 'max_m': 8.0, 'palette': 'grey_near_bright'},
        dropped=11)
    valid = {
        'lidar': {'blob': _b64(lidar), 'stream': 'lidar', 'seq': 7,
                  'dropped': 3, 'payload_bytes': 6,
                  'ranges': [1.234, None, 2.0]},
        'camera': {'blob': _b64(camera), 'stream': 'camera', 'seq': 8,
                   'dropped': 0, 'payload_bytes': 8},
        'depth': {'blob': _b64(depth), 'stream': 'depth', 'seq': 9,
                  'dropped': 11, 'payload_bytes': 8},
    }

    def with_header(blob, **changes):
        """Rewrite one frame's JSON header, keeping its prefix and body."""
        stream, header, payload = binary.decode_frame(blob)
        header = dict(header, **changes)
        text = json.dumps(header).encode()
        return blob[:6] + struct.pack('>H', len(text)) + text + payload

    wrong_kind = bytearray(camera)
    wrong_kind[5] = 3                       # says depth, header says camera
    invalid = {
        'empty': b'',
        'magic only': b'COCO',
        'bad magic': b'XOXO' + camera[4:],
        'version 2': camera[:4] + b'\x02' + camera[5:],
        'unknown kind': camera[:5] + b'\x09' + camera[6:],
        'kind disagrees with header': bytes(wrong_kind),
        'header longer than frame': camera[:6] + b'\xff\xff' + camera[8:],
        'camera truncated': camera[:-1],
        'depth padded': depth + b'x',
        'lidar short a ray': lidar[:-2],
        'lidar count lies': with_header(lidar, count=40, payload_bytes=6),
        'depth range inverted': with_header(depth, min_m=5.0, max_m=1.0),
        'negative seq': with_header(camera, seq=-1),
        'topic in metadata': with_header(camera, topic='/camera/image_raw'),
        'not jpeg': with_header(camera, enc='png'),
    }
    return {'valid': valid,
            'invalid': {name: _b64(blob) for name, blob in invalid.items()}}


@pytest.mark.skipif(shutil.which('node') is None,
                    reason='nodejs is a declared test_depend; install it')
def test_the_page_decoder_agrees_with_the_python_encoder(tmp_path):
    """Node runs web/frame.js over real encoder output; all must pass."""
    fixtures = tmp_path / 'frames.json'
    fixtures.write_text(json.dumps(_fixtures()))
    result = subprocess.run(
        ['node', '--test', str(HERE / 'frame_decode.test.js')],
        capture_output=True, text=True, timeout=60,
        env={**os.environ, 'COCO_FRAME_FIXTURES': str(fixtures)})
    assert result.returncode == 0, result.stdout + result.stderr
    assert '# pass 3' in result.stdout, result.stdout


def test_every_broken_fixture_is_broken_for_python_too():
    """The fixtures are real refusals, not decoder disagreements."""
    for name, blob in _fixtures()['invalid'].items():
        raw = base64.b64decode(blob)
        if name in ('lidar count lies', 'depth range inverted',
                    'negative seq', 'topic in metadata', 'not jpeg',
                    'kind disagrees with header'):
            with pytest.raises(binary.BinaryFrameError):
                binary.decode_frame(raw)


def test_the_page_loads_the_decoder_before_the_client():
    """frame.js defines window.cocoFrame, which app.js calls."""
    index = (HERE.parent / 'web' / 'index.html').read_text()
    assert index.index('src="frame.js"') < index.index('src="app.js"')
    assert 'window.cocoFrame.decodeFrame' in (
        HERE.parent / 'web' / 'app.js').read_text()
