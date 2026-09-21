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

"""Raw image layout and invalid metadata must have deterministic outcomes."""

from coco_web import imaging

import numpy as np

import pytest


@pytest.mark.parametrize('width,height', [(0, 8), (-1, 8), (8, 0),
                                          (True, 8), (8.5, 8), (9000, 1)])
@pytest.mark.parametrize('depth', [False, True])
def test_bad_dimensions_raise_image_error(width, height, depth):
    """No malformed shape should escape as an OpenCV or reshape exception."""
    call = imaging.depth_jpeg if depth else imaging.colour_jpeg
    with pytest.raises(imaging.ImageError):
        call(width, height, '32FC1' if depth else 'rgb8', b'x' * 1024)


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -1, 0, 'tiny'])
def test_bad_scale_is_refused(value):
    """Negotiated metadata must not cause an uncaught resize error."""
    with pytest.raises(imaging.ImageError):
        imaging.colour_jpeg(8, 8, 'rgb8', b'x' * 192, scale=value)


def test_rgb_row_padding_is_not_interpreted_as_pixels():
    """Each row starts at Image.step, including non-pixel padding bytes."""
    pixels = bytes([255, 0, 0]) * 8
    packed = pixels * 8
    padded = (pixels + b'padding!') * 8
    expected = imaging.colour_jpeg(8, 8, 'rgb8', packed)
    assert imaging.colour_jpeg(8, 8, 'rgb8', padded, step=32) == expected


def test_depth_endianness_and_padding_are_honoured():
    """Opposite endian buffers describing the same metres render identically."""
    pixels = np.linspace(0.2, 7.8, 64).reshape(8, 8)
    little = pixels.astype('<f4').tobytes()
    big = b''.join(row.astype('>f4').tobytes() + b'pad!'
                   for row in pixels)
    expected = imaging.depth_jpeg(8, 8, '32FC1', little,
                                  clip=(0.1, 8), is_bigendian=False)
    actual = imaging.depth_jpeg(8, 8, '32FC1', big, clip=(0.1, 8),
                                step=36, is_bigendian=True)
    assert actual == expected


@pytest.mark.parametrize('step', [1, -1, True, 23, 24.5])
def test_invalid_row_stride_is_refused(step):
    """A stride smaller than a row cannot be used to index the next row."""
    with pytest.raises(imaging.ImageError):
        imaging.colour_jpeg(8, 8, 'rgb8', b'x' * 256, step=step)
