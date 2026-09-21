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
Turning ROS image buffers into JPEG, without importing ROS.

Every function takes plain values -- width, height, encoding string, raw
bytes -- rather than a ``sensor_msgs/Image``. That is what lets the whole
of the image path be unit-tested on synthetic arrays with no ROS graph,
and it is the same shape ``telemetry.py`` uses for status lines.

cv_bridge is deliberately not used. It would pull rclpy in through
``sensor_msgs``, and all it does here is the reshape below.

Encodings, named explicitly, never guessed
-------------------------------------------
``/camera/image_raw`` is **rgb8**. The gz bridge delivers the sensor's
``R8G8B8`` straight through, and this repo has already paid for getting
that wrong once: ``target_finder`` carries a comment recording that
treating it as BGR and converting to HSV swaps red and blue, and produces
a *confident detection of the wrong object*. On a colour-selected fetch
that is the whole mission.

``/perception/annotated`` is **bgr8**, because cv_bridge wrote it.
``/camera/depth/image_raw`` is **32FC1** -- metres, one float per pixel,
with non-finite values for "no return".

So the encoding is always a parameter and never inferred from the buffer,
and an unknown one raises rather than being read as something plausible.

Depth is a VISUALISATION
-------------------------
``depth_image()`` produces a greyscale JPEG and reports the metre range
it mapped, so a client can label what it is showing. It is a picture of
the depth camera, not a navigation input. Nothing here feeds a costmap,
and C2-NAV.43 left depth fusion a candidate that is OFF -- the browser
must not be the thing that quietly turns it on.
"""

import math

import cv2

import numpy as np

#: Bytes per pixel for each colour encoding this module accepts.
CHANNELS = {'rgb8': 3, 'bgr8': 3, 'mono8': 1}

#: Encodings that carry metric depth rather than colour.
DEPTH_ENCODINGS = ('32FC1',)

#: Clip for the depth ramp when the frame carries nothing finite. The
#: camera's own far clip is 8.0 m (coco_config.CAMERA_DEPTH_CLIP).
DEFAULT_DEPTH_RANGE = (0.1, 8.0)


class ImageError(ValueError):
    """An image buffer that cannot be decoded, with a stable code."""

    def __init__(self, code, message):
        """Carry a slug alongside the message, as ProtocolError does."""
        super().__init__(message)
        self.code = code


def _layout(width, height, pixel_bytes, data, step):
    """Validate bounded dimensions and extract rows without their padding."""
    if (type(width) is not int or type(height) is not int
            or not 1 <= width <= 8192 or not 1 <= height <= 8192
            or width * height > 8 << 20):
        raise ImageError('bad_dimensions', 'invalid image dimensions')
    row_bytes = width * pixel_bytes
    step = row_bytes if step is None else step
    if type(step) is not int or not row_bytes <= step <= 32 << 20:
        raise ImageError('bad_step', 'invalid row stride')
    expected = step * height
    if expected > 32 << 20:
        raise ImageError('bad_dimensions', 'image buffer exceeds limit')
    if len(data) < expected:
        raise ImageError('short_buffer', 'buffer shorter than image layout')
    rows = np.frombuffer(data, dtype=np.uint8, count=expected)
    return rows.reshape(height, step)[:, :row_bytes].copy().tobytes()


def _array(width, height, encoding, data, step=None):
    """Reshape a raw image buffer, respecting explicit row padding."""
    channels = CHANNELS.get(encoding)
    if channels is None:
        raise ImageError('unknown_encoding', 'unsupported colour encoding')
    packed = _layout(width, height, channels, data, step)
    return np.frombuffer(packed, dtype=np.uint8).reshape(
        (height, width, channels))


def _settings(quality, scale):
    """Reject malformed encoder settings before calling OpenCV."""
    try:
        valid = (not isinstance(quality, bool) and 1 <= quality <= 100
                 and math.isfinite(quality)
                 and (scale is None or
                      (not isinstance(scale, bool) and scale > 0
                       and math.isfinite(scale))))
    except (TypeError, ValueError, OverflowError):
        valid = False
    if not valid:
        raise ImageError('bad_settings', 'invalid JPEG quality or scale')


def _scaled(frame, scale):
    """Resize by `scale`, never up, and never below one pixel."""
    if scale is None or scale >= 0.999:
        return frame
    height, width = frame.shape[:2]
    return cv2.resize(
        frame,
        (max(1, int(width * scale)), max(1, int(height * scale))),
        interpolation=cv2.INTER_AREA)


def _jpeg(frame, quality):
    """Encode a BGR or greyscale array as JPEG bytes."""
    ok, buffer = cv2.imencode(
        '.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise ImageError('encode_failed', 'cv2.imencode refused the frame')
    return buffer.tobytes()


def colour_jpeg(width, height, encoding, data, quality=60, scale=1.0,
                *, step=None):
    """
    Encode a colour camera frame as JPEG. Returns ``(jpeg, w, h)``.

    The encoding is honoured rather than assumed: rgb8 is converted, bgr8
    is already what cv2 wants. Getting this backwards does not raise --
    it silently swaps red and blue, which on a colour-selected fetch
    means a confident picture of the wrong cylinder.
    """
    _settings(quality, scale)
    frame = _array(width, height, encoding, data, step)
    if encoding == 'rgb8':
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    elif encoding == 'mono8':
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    frame = _scaled(frame, scale)
    return _jpeg(frame, quality), frame.shape[1], frame.shape[0]


def depth_jpeg(width, height, encoding, data, quality=60, scale=1.0,
               clip=None, *, step=None, is_bigendian=False):
    """
    Render a depth frame as a greyscale JPEG. Returns ``(jpeg, w, h, lo, hi)``.

    Near is bright and far is dark, which is the reading people expect
    from a depth picture. ``lo``/``hi`` are the metre bounds the ramp
    actually spans and go into the frame header, so the client can say
    what the shades mean instead of showing an unlabelled gradient.

    Non-finite pixels -- the depth camera's "no return" -- are rendered
    black and excluded from the range, so one NaN cannot flatten the
    whole picture to a single shade.
    """
    if encoding not in DEPTH_ENCODINGS:
        raise ImageError(
            'unknown_encoding',
            f'{encoding!r} is not a depth encoding; known: '
            f'{", ".join(DEPTH_ENCODINGS)}')
    _settings(quality, scale)
    if type(is_bigendian) is not bool:
        raise ImageError('bad_endianness', 'is_bigendian must be boolean')
    packed = _layout(width, height, 4, data, step)
    metres = np.frombuffer(
        packed, dtype='>f4' if is_bigendian else '<f4').reshape((height, width))
    valid = np.isfinite(metres) & (metres > 0.0)
    if clip:
        low, high = float(clip[0]), float(clip[1])
        valid &= (metres >= low) & (metres <= high)
    elif valid.any():
        low, high = float(metres[valid].min()), float(metres[valid].max())
    else:
        low, high = DEFAULT_DEPTH_RANGE
    if not math.isfinite(low) or not math.isfinite(high) or high <= low:
        low, high = DEFAULT_DEPTH_RANGE
    # Invalid pixels are replaced BEFORE the arithmetic, not masked
    # after: NaN survives np.clip, and casting NaN to uint8 is undefined
    # (numpy warns and produces whatever the hardware does). They are
    # painted black below regardless, so the value here is arbitrary.
    filled = np.where(valid, metres, low)
    # Near bright, far dark. np.clip so a reading outside the range
    # cannot wrap when it is cast.
    normalised = (np.clip(filled, low, high) - low) / (high - low)
    shades = ((1.0 - normalised) * 255.0).astype(np.uint8)
    shades[~valid] = 0
    shades = _scaled(shades, scale)
    return (_jpeg(shades, quality), shades.shape[1], shades.shape[0],
            low, high)
