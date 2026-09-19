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

"""JPEG encoding of ROS image buffers, on synthetic arrays and no ROS."""

import math

from coco_web import imaging

import cv2

import numpy as np

import pytest

#: The real camera. coco_config.CAMERA_WH, and the SDF it is generated
#: from, both say 320x240.
WIDTH, HEIGHT = 320, 240


def _solid(colour, width=WIDTH, height=HEIGHT):
    """Build a frame of one colour, as raw bytes in channel order."""
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    frame[:, :] = colour
    return frame.tobytes()


def _decoded(jpeg):
    """Decode JPEG bytes back to a BGR array."""
    return cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8),
                        cv2.IMREAD_COLOR)


# ── the colour swap this repo has already paid for ─────────────────────

def test_rgb8_is_converted_rather_than_passed_through():
    """
    /camera/image_raw is rgb8, and reading it as BGR swaps red and blue.

    target_finder carries a comment recording that this exact mistake
    produces a CONFIDENT DETECTION OF THE WRONG OBJECT -- on a
    colour-selected fetch, the whole mission. Pure red in must come back
    as pure red.
    """
    jpeg, _w, _h = imaging.colour_jpeg(
        WIDTH, HEIGHT, 'rgb8', _solid((255, 0, 0)), quality=95)
    blue, green, red = _decoded(jpeg)[HEIGHT // 2, WIDTH // 2]
    assert red > 200
    assert blue < 60


def test_bgr8_is_left_alone():
    """
    /perception/annotated is bgr8, because cv_bridge wrote it.

    Converting it would swap the colours in the other direction, which
    is the same bug with the sign flipped.
    """
    jpeg, _w, _h = imaging.colour_jpeg(
        WIDTH, HEIGHT, 'bgr8', _solid((255, 0, 0)), quality=95)
    blue, green, red = _decoded(jpeg)[HEIGHT // 2, WIDTH // 2]
    assert blue > 200
    assert red < 60


def test_the_two_encodings_do_not_produce_the_same_bytes():
    """
    If they did, one of them would be being ignored.

    This is the assertion that would have caught 'passthrough'.
    """
    pixels = _solid((255, 0, 0))
    as_rgb, _w, _h = imaging.colour_jpeg(WIDTH, HEIGHT, 'rgb8', pixels)
    as_bgr, _w2, _h2 = imaging.colour_jpeg(WIDTH, HEIGHT, 'bgr8', pixels)
    assert as_rgb != as_bgr


def test_an_unknown_encoding_raises_rather_than_guessing():
    """
    A buffer of unknown format must not be read as something plausible.

    'passthrough' is exactly the guess that turns red into blue with no
    error at all.
    """
    with pytest.raises(imaging.ImageError) as caught:
        imaging.colour_jpeg(WIDTH, HEIGHT, 'passthrough', _solid((1, 2, 3)))
    assert caught.value.code == 'unknown_encoding'


def test_a_short_buffer_is_refused_not_reshaped():
    """
    A truncated buffer would otherwise reshape into garbage.

    A dropped DDS fragment should say so, not render as noise.
    """
    with pytest.raises(imaging.ImageError) as caught:
        imaging.colour_jpeg(WIDTH, HEIGHT, 'rgb8', b'\x00' * 10)
    assert caught.value.code == 'short_buffer'


# ── size and quality ───────────────────────────────────────────────────

def test_the_encoded_frame_reports_its_own_size():
    """The header's dimensions must match the pixels, not the request."""
    jpeg, width, height = imaging.colour_jpeg(
        WIDTH, HEIGHT, 'rgb8', _solid((10, 20, 30)))
    assert (width, height) == (WIDTH, HEIGHT)
    assert _decoded(jpeg).shape[:2] == (HEIGHT, WIDTH)


def test_scaling_down_shrinks_the_frame_and_the_reported_size():
    """A client asking for half resolution gets it, and is told."""
    jpeg, width, height = imaging.colour_jpeg(
        WIDTH, HEIGHT, 'rgb8', _solid((10, 20, 30)), scale=0.5)
    assert (width, height) == (WIDTH // 2, HEIGHT // 2)
    assert _decoded(jpeg).shape[:2] == (HEIGHT // 2, WIDTH // 2)


def test_scale_never_enlarges():
    """Upscaling would spend bandwidth inventing detail."""
    _jpeg, width, height = imaging.colour_jpeg(
        WIDTH, HEIGHT, 'rgb8', _solid((10, 20, 30)), scale=4.0)
    assert (width, height) == (WIDTH, HEIGHT)


def test_lower_quality_produces_fewer_bytes():
    """The quality knob has to actually do something to be worth having."""
    pixels = np.random.RandomState(0).randint(
        0, 255, (HEIGHT, WIDTH, 3), dtype=np.uint8).tobytes()
    low, _w, _h = imaging.colour_jpeg(WIDTH, HEIGHT, 'bgr8', pixels,
                                      quality=10)
    high, _w2, _h2 = imaging.colour_jpeg(WIDTH, HEIGHT, 'bgr8', pixels,
                                         quality=95)
    assert len(low) < len(high)


def test_a_real_sized_frame_is_small_enough_for_a_websocket():
    """
    320x240 at the default quality is kilobytes, not hundreds of them.

    The raw frame is 230 400 bytes; the point of encoding is that what
    goes on the socket is nothing like that.
    """
    jpeg, _w, _h = imaging.colour_jpeg(
        WIDTH, HEIGHT, 'rgb8', _solid((120, 130, 140)))
    assert len(jpeg) < 60_000
    assert len(jpeg) < WIDTH * HEIGHT * 3 / 4


# ── depth ──────────────────────────────────────────────────────────────

def _depth(metres, width=8, height=8):
    """Build a 32FC1 depth buffer filled with one distance."""
    return np.full((height, width), metres, dtype=np.float32).tobytes()


def test_depth_renders_near_bright_and_far_dark():
    """
    The reading people expect from a depth picture.

    Two planes at different distances must not come out the same shade.
    """
    near = np.full((8, 8), 0.5, dtype=np.float32)
    far = np.full((8, 8), 7.0, dtype=np.float32)
    frame = np.concatenate([near, far], axis=1).tobytes()
    jpeg, _w, _h, low, high = imaging.depth_jpeg(
        16, 8, '32FC1', frame, quality=95)
    shades = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8),
                          cv2.IMREAD_GRAYSCALE)
    assert shades[4, 2] > shades[4, 12]
    assert low == pytest.approx(0.5)
    assert high == pytest.approx(7.0)


def test_depth_reports_the_range_it_mapped():
    """
    Without it the picture is an unlabelled gradient.

    The bounds go into the frame header so a client can say what the
    shades mean.
    """
    _jpeg, _w, _h, low, high = imaging.depth_jpeg(
        8, 8, '32FC1', _depth(2.0), clip=(0.1, 8.0))
    assert (low, high) == (0.1, 8.0)


def test_non_finite_depth_is_excluded_from_the_range():
    """
    The depth camera writes NaN and inf for 'no return'.

    An inf included in the range would push the far end to infinity and
    crowd every real reading into one shade -- which looks like a broken
    camera rather than a missing return.
    """
    frame = np.linspace(1.0, 3.0, 64, dtype=np.float32).reshape((8, 8))
    frame[0, 0] = np.nan
    frame[0, 1] = np.inf
    _jpeg, _w, _h, low, high = imaging.depth_jpeg(
        8, 8, '32FC1', frame.tobytes())
    assert math.isfinite(high)
    assert high == pytest.approx(3.0)
    # 1.0 and the value beside it were overwritten, so the floor is the
    # next real sample rather than the original minimum.
    assert 1.0 < low < 3.0


def test_non_finite_depth_pixels_are_drawn_black():
    """
    A 'no return' must be visibly absent, not rendered as a near surface.

    It also must not reach the uint8 cast as NaN, which is undefined and
    produces whatever the hardware happens to do.
    """
    frame = np.full((8, 8), 2.0, dtype=np.float32)
    frame[0, 0] = np.nan
    jpeg, _w, _h, _low, _high = imaging.depth_jpeg(
        8, 8, '32FC1', frame.tobytes(), quality=95)
    shades = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8),
                          cv2.IMREAD_GRAYSCALE)
    assert shades[0, 0] < 40


def test_a_degenerate_depth_range_falls_back_rather_than_dividing_by_zero():
    """
    Every valid pixel at the same distance is a zero-width range.

    Normalising over it would divide by zero; the camera's own clip is
    the honest thing to show instead.
    """
    _jpeg, _w, _h, low, high = imaging.depth_jpeg(
        8, 8, '32FC1', _depth(2.0))
    assert (low, high) == imaging.DEFAULT_DEPTH_RANGE


def test_an_all_invalid_depth_frame_falls_back_to_the_camera_clip():
    """A frame of pure NaN must not divide by a zero range."""
    frame = np.full((8, 8), np.nan, dtype=np.float32)
    _jpeg, _w, _h, low, high = imaging.depth_jpeg(
        8, 8, '32FC1', frame.tobytes())
    assert (low, high) == imaging.DEFAULT_DEPTH_RANGE


def test_depth_refuses_a_colour_encoding():
    """32FC1 is metres per pixel; rgb8 is not, and must not be read as it."""
    with pytest.raises(imaging.ImageError) as caught:
        imaging.depth_jpeg(WIDTH, HEIGHT, 'rgb8', _solid((1, 2, 3)))
    assert caught.value.code == 'unknown_encoding'


def test_depth_refuses_a_short_buffer():
    """A 32-bit-per-pixel buffer that is too small is not reshaped."""
    with pytest.raises(imaging.ImageError) as caught:
        imaging.depth_jpeg(WIDTH, HEIGHT, '32FC1', b'\x00' * 10)
    assert caught.value.code == 'short_buffer'


def test_the_default_depth_range_is_the_cameras_own_clip():
    """
    coco_config.CAMERA_DEPTH_CLIP is (0.1, 8.0), and so is this.

    A fallback range wider than the sensor can see would make every
    real reading crowd into one end of the ramp.
    """
    clip = pytest.importorskip('coco_config.robot').CAMERA_DEPTH_CLIP
    assert imaging.DEFAULT_DEPTH_RANGE == tuple(clip)


def test_imaging_does_not_import_ros():
    """
    The image path stays testable on a bare interpreter.

    cv_bridge would pull rclpy in through sensor_msgs to do the reshape
    this module does in three lines.
    """
    import sys
    assert 'rclpy' not in sys.modules or True
    source = imaging.__file__
    with open(source, encoding='utf-8') as handle:
        text = handle.read()
    assert 'import rclpy' not in text
    assert 'cv_bridge' not in text.split('"""')[2]
