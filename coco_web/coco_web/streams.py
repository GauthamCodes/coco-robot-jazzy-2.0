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
What one client is subscribed to, and how much of it it may be sent.

Pure module: no rclpy, no sockets, no tornado. A ``Subscription`` is one
connected client's view of the telemetry it asked for, the rate it agreed
to, and how far behind it is. Keeping that policy here rather than inside
the WebSocket handler is what lets "a slow browser must not grow ROS
memory" be proved by a unit test with a fake socket, exactly as
``safety.py`` proves the command boundary without a ROS graph.

Why subscriptions exist at all
------------------------------
P0.1 accepted a ``subscribe`` frame and did nothing with it -- every
client received every stream. That was honest for four small JSON
sections. It stops being honest the moment camera frames exist: a phone
watching the map should not be sent 320x240 JPEGs fifteen times a second
because a laptop in another tab asked for them.

So the expensive streams are **opt-in**, and the cheap ones a P0.1 client
already receives stay on by default. That is the whole compatibility
rule: a client written against P0.1 that never sends ``subscribe`` sees
exactly what it saw before, and cannot be surprised by a binary frame it
has no code to parse.

Backpressure, and why the queue is one frame deep
--------------------------------------------------
A WebSocket write that is not yet flushed is a frame sitting in memory.
If the producer is a 15 Hz camera and the consumer is a phone on a bad
connection, an unbounded queue grows until something dies -- and the
thing that dies is the ROS node, which is also the thing holding the STOP
button.

The queue here is therefore **one frame per stream per client**: while a
frame is still in flight, the next one for that stream is dropped and
counted. Dropping is the correct answer for a sensor -- a LiDAR frame
from two seconds ago is not worth showing, and the client is told how
many it missed so it can say so rather than silently lie.

**Control frames are never dropped.** ``ack``, ``error``, ``pong`` and
the telemetry tick always write. Starving the path that carries mission
state and the STOP acknowledgement in order to keep video smooth would
be exactly the wrong trade.
"""

import time

#: Streams a client may ask for. Unknown names are an error rather than
#: being ignored, for the same reason unexpected frame keys are.
STREAMS = (
    'telemetry',   # pose, velocity, session, arbiter -- the core tick
    'mission',     # the executive's state, inside the telemetry frame
    'lidar',       # the downsampled scan
    'map',         # the occupancy grid, on change only
    'path',        # the current Nav2 plan
    'camera',      # binary JPEG frames
    'depth',       # binary colourised depth frames
)

#: What a client gets without asking. Deliberately the P0.1 set: a client
#: written against that release and never sending `subscribe` must keep
#: working unchanged. The new, expensive streams are the opt-in ones.
DEFAULT_STREAMS = ('telemetry', 'mission', 'lidar', 'map', 'path')

#: Streams delivered as binary frames rather than inside the JSON tick.
#: A client that did not declare binary support in `hello` is never sent
#: one, and asking for these without that support is refused.
BINARY_STREAMS = ('camera', 'depth')

#: Streams whose rate and quality a client may negotiate.
TUNABLE_STREAMS = ('camera', 'depth')

#: Per-stream defaults and hard bounds. `fps` is capped BELOW the
#: sensor's own rate on purpose: /camera/image_raw publishes at 15 Hz,
#: and promising 15 to a browser only means promising to drop.
LIMITS = {
    'camera': {
        'fps': 10.0, 'fps_max': 15.0, 'fps_min': 1.0,
        'quality': 60, 'quality_max': 95, 'quality_min': 10,
        'scale': 1.0, 'scale_min': 0.25, 'scale_max': 1.0,
    },
    'depth': {
        'fps': 5.0, 'fps_max': 15.0, 'fps_min': 1.0,
        'quality': 60, 'quality_max': 95, 'quality_min': 10,
        'scale': 1.0, 'scale_min': 0.25, 'scale_max': 1.0,
    },
}

#: Bytes of unflushed socket buffer past which even a first frame is
#: dropped. A stalled TCP connection can leave a write "in flight"
#: indefinitely; this is the second bound, on the transport rather than
#: on our own bookkeeping.
MAX_BUFFERED_BYTES = 1 << 20      # 1 MiB


def known(names):
    """Return the unknown names in `names`, sorted. Empty means all known."""
    return sorted(set(names) - set(STREAMS))


def clamp(stream, field, value):
    """
    Clamp one tunable to its documented bounds.

    Clamped rather than refused, for the same reason velocity is: a
    client asking for 30 fps wants "as fast as you can", and answering
    that with an error helps nobody.
    """
    limits = LIMITS.get(stream)
    if limits is None or f'{field}_min' not in limits:
        return None
    low, high = limits[f'{field}_min'], limits[f'{field}_max']
    try:
        number = float(value)
    except (TypeError, ValueError):
        return limits[field]
    if number != number:                     # NaN fails every comparison
        return limits[field]
    number = max(low, min(high, number))
    return int(number) if isinstance(limits[field], int) else number


class Subscription:
    """One client's streams, its negotiated rates, and its backlog."""

    def __init__(self, streams=None, binary=False):
        """Start from the default set unless the client named others."""
        self.streams = set(DEFAULT_STREAMS if streams is None else streams)
        self.binary = bool(binary)
        self.config = {name: dict(values) for name, values in LIMITS.items()}
        self.sent = {name: 0 for name in STREAMS}
        self.dropped = {name: 0 for name in STREAMS}
        self._last_sent = {name: 0.0 for name in STREAMS}
        self._inflight = {}

    # ── membership ─────────────────────────────────────────────────────
    def wants(self, stream):
        """
        Whether this client should be sent `stream` at all.

        A binary stream is refused to a client that never declared it can
        parse one, even if it managed to subscribe: sending a Blob to a
        client expecting text is a parse error in someone else's console,
        several layers from the cause.
        """
        if stream not in self.streams:
            return False
        if stream in BINARY_STREAMS and not self.binary:
            return False
        return True

    def subscribe(self, names):
        """Add streams. Returns the resulting set, sorted."""
        self.streams.update(names)
        return sorted(self.streams)

    def unsubscribe(self, names):
        """
        Drop streams. Returns the resulting set, sorted.

        ``telemetry`` cannot be dropped: it carries the session state and
        the readiness the UI needs to tell a user why nothing is
        happening, and a client with no telemetry has no way to find out
        that it is broken.
        """
        self.streams.difference_update(set(names) - {'telemetry'})
        return sorted(self.streams)

    def configure(self, stream, **fields):
        """Clamp and apply tunables for one stream. Returns the config."""
        target = self.config.setdefault(stream, {})
        for field, value in fields.items():
            if value is None:
                continue
            clamped = clamp(stream, field, value)
            if clamped is not None:
                target[field] = clamped
        return dict(target)

    # ── rate limiting ──────────────────────────────────────────────────
    def due(self, stream, now=None):
        """
        Whether enough time has passed to send `stream` again.

        The rate limit is applied per client, so one browser asking for
        2 fps does not slow the tab next to it down to 2 fps.
        """
        config = self.config.get(stream)
        if not config or not config.get('fps'):
            return True
        stamp = time.monotonic() if now is None else now
        return stamp - self._last_sent[stream] >= 1.0 / config['fps']

    # ── backpressure ───────────────────────────────────────────────────
    def ready(self, stream, buffered=0):
        """
        Whether a new frame for `stream` may be written now.

        False when the previous frame for that stream has not flushed, or
        when the socket itself is holding more than MAX_BUFFERED_BYTES.
        The caller drops rather than queues -- see the module docstring.
        """
        if buffered > MAX_BUFFERED_BYTES:
            return False
        pending = self._inflight.get(stream)
        if pending is None:
            return True
        return bool(getattr(pending, 'done', lambda: True)())

    def mark_sent(self, stream, pending=None, now=None):
        """Record a frame going out, holding its write future if given."""
        self.sent[stream] = self.sent.get(stream, 0) + 1
        self._last_sent[stream] = time.monotonic() if now is None else now
        self._inflight[stream] = pending

    def mark_dropped(self, stream):
        """Record a frame that was not sent, so the client can be told."""
        self.dropped[stream] = self.dropped.get(stream, 0) + 1
        return self.dropped[stream]

    # ── views ──────────────────────────────────────────────────────────
    def as_dict(self):
        """Build the subscription document sent in welcome and on change."""
        return {
            'streams': sorted(self.streams),
            'available': list(STREAMS),
            'default': list(DEFAULT_STREAMS),
            'binary': self.binary,
            'binary_streams': list(BINARY_STREAMS),
            'config': {name: dict(values)
                       for name, values in sorted(self.config.items())},
            'sent': dict(self.sent),
            'dropped': dict(self.dropped),
        }
