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

**Control frames are never dropped.** ``welcome``, ``ack``, ``error``,
``pong`` and ``subscription`` always write, immediately, in order.
Starving the path that carries the STOP acknowledgement in order to keep
video smooth would be exactly the wrong trade.

State streams: latest wins, never queued
-----------------------------------------
P0.2's first pass wrote every telemetry tick unconditionally, so a
browser that stopped reading grew the server's write buffer by one
telemetry frame every 100 ms, forever. Codex flagged it; it was the one
unbounded path left.

``telemetry`` and ``map`` are STATE: each frame is a complete picture
that makes every earlier one worthless. So each gets the sensor rule --
one write in flight -- with one difference: a frame that cannot go now
is not dropped but OWED, and a newer one replaces it (``superseded``).
The moment the socket drains, the client gets the latest state, never a
backlog of stale ones. A mission state lasting less than one flush can
therefore be missed by a client that is behind; the executive's `prev`
and `event` fields in the next frame still say what happened.

And one hard bound over everything: a client holding more than
MAX_CLIENT_BYTES unflushed is disconnected. Only a client that sends
requests and never reads can get there -- every stream is bounded
without it -- and disconnecting it runs the ordinary close path,
including the last-client STOP. STOP itself never depends on a write:
the zero is published on RECEIPT of the frame, before its ack is queued,
and TCP carries the browser's frames in however far behind the server's
are.
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

#: Streams that EXIST ONLY as binary frames. A client that did not
#: declare binary support in `hello` cannot have these at all, because
#: there is no JSON form of a JPEG worth sending.
BINARY_STREAMS = ('camera', 'depth')

#: Every stream that CAN be delivered as a binary frame. Wider than
#: BINARY_STREAMS because lidar has both forms: a binary frame for a
#: client that declared support, and the JSON `sensors.lidar` block in
#: the telemetry tick for one that did not.
#:
#: Keeping these two lists distinct is load-bearing, and conflating them
#: was a real bug: `wants()` gated only on BINARY_STREAMS, so a client
#: that said `binary: false` was still sent binary LIDAR frames -- the
#: precise compatibility guarantee this design claims to make. It
#: surfaced as a UnicodeDecodeError in a client calling json.loads on a
#: JPEG-adjacent byte string, which is exactly how a P0.1 client would
#: have met it.
BINARY_CAPABLE = ('lidar', 'camera', 'depth')

#: Streams whose rate and quality a client may negotiate.
TUNABLE_STREAMS = ('camera', 'depth')

#: Streams whose every frame supersedes the last: one write in flight,
#: and at most one more OWED, replaced rather than queued. See the module
#: docstring.
STATE_STREAMS = ('telemetry', 'map')

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

#: Unflushed bytes past which a client is DISCONNECTED rather than
#: written to. Above the sensor bound plus one telemetry frame and one
#: map by a wide margin, so no well-behaved client -- however slow its
#: link -- ever meets it; only one that floods requests without reading.
MAX_CLIENT_BYTES = 4 << 20        # 4 MiB


def known(names):
    """Return the unknown names in `names`, sorted. Empty means all known."""
    return sorted(set(names) - set(STREAMS))


def _validated_names(names):
    """Validate the whole request before mutating any client state."""
    if isinstance(names, (str, bytes)) or names is None:
        raise ValueError('streams must be an iterable of stream names')
    try:
        values = list(names)
    except TypeError as exc:
        raise ValueError('streams must be iterable') from exc
    if any(not isinstance(name, str) for name in values):
        raise ValueError('stream names must be strings')
    if known(values):
        raise ValueError('unknown stream name')
    return set(values)


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


def filter_telemetry(frame, subscription):
    """
    Blank the telemetry sections this client did not subscribe to.

    Sections become None (or [] for a list) rather than being removed.
    That is already their meaning before the first message arrives, so a
    client needs no new branch -- and a client that never sent
    `subscribe` holds the P0.1 default set and sees no change at all.

    **Returns a copy.** The frame is built once per tick and filtered per
    client, so mutating it in place would corrupt every client filtered
    after the first one in the same tick -- a bug that would present as
    one browser's LiDAR vanishing when a different browser unsubscribed.

    Lives here rather than in the server because it is policy, and
    policy in this module is testable without a ROS graph.
    """
    if subscription is None:
        return frame
    view = dict(frame)
    if not subscription.wants('mission'):
        view['mission'] = None
    # A binary client already received the scan as its own frame, so the
    # JSON copy is dropped rather than sent twice.
    if not subscription.wants_json_lidar():
        sensors = dict(view.get('sensors') or {})
        sensors['lidar'] = None
        view['sensors'] = sensors
    if not subscription.wants('path'):
        nav = dict(view.get('nav') or {})
        nav['path'] = []
        view['nav'] = nav
    # This client's own delivery counters. `perf` beside it is the
    # platform's total across every client; before this there was no way
    # for a page to tell its own losses from another tab's. Additive.
    if isinstance(view.get('platform'), dict):
        platform = dict(view['platform'])
        platform['delivery'] = subscription.delivery()
        view['platform'] = platform
    return view


class Subscription:
    """One client's streams, its negotiated rates, and its backlog."""

    def __init__(self, streams=None, binary=False):
        """Start from the default set unless the client named others."""
        self.streams = _validated_names(
            DEFAULT_STREAMS if streams is None else streams)
        self._closed = False
        self.binary = bool(binary)
        self.config = {name: dict(values) for name, values in LIMITS.items()}
        self.sent = {name: 0 for name in STREAMS}
        self.dropped = {name: 0 for name in STREAMS}
        #: STATE frames replaced by a newer one before they could be sent.
        #: Not drops: nothing was lost that the next frame does not carry.
        self.superseded = {name: 0 for name in STREAMS}
        self._last_sent = {name: 0.0 for name in STREAMS}
        self._inflight = {}
        self._owed = {}

    # ── membership ─────────────────────────────────────────────────────
    def wants(self, stream):
        """
        Report whether this client should be sent `stream` at all.

        A binary-only stream is refused to a client that never declared
        it can parse one, even if it managed to subscribe: sending a Blob
        to a client expecting text is a parse error in someone else's
        console, several layers from the cause.
        """
        if self._closed or stream not in STREAMS or stream not in self.streams:
            return False
        if stream in BINARY_STREAMS and not self.binary:
            return False
        return True

    def wants_binary(self, stream):
        """
        Report whether `stream` should go to this client AS a binary frame.

        Separate from ``wants`` because lidar has two forms. A client
        that declared binary support gets the compact frame; one that did
        not gets the JSON block in the telemetry tick, and must never
        receive the binary one.
        """
        return (self.binary and stream in BINARY_CAPABLE
                and self.wants(stream))

    def wants_json_lidar(self):
        """
        Report whether the telemetry tick should carry `sensors.lidar`.

        False for a binary client, which already received the scan as its
        own frame -- sending both would double the cost of the thing the
        binary format exists to make cheap.
        """
        return self.wants('lidar') and not self.binary

    def subscribe(self, names):
        """Add streams. Returns the resulting set, sorted."""
        if self._closed:
            raise ValueError('subscription is closed')
        self.streams.update(_validated_names(names))
        return sorted(self.streams)

    def unsubscribe(self, names):
        """
        Drop streams. Returns the resulting set, sorted.

        ``telemetry`` cannot be dropped: it carries the session state and
        the readiness the UI needs to tell a user why nothing is
        happening, and a client with no telemetry has no way to find out
        that it is broken.
        """
        self.streams.difference_update(_validated_names(names) - {'telemetry'})
        return sorted(self.streams)

    def configure(self, stream, **fields):
        """Clamp and apply tunables for one stream. Returns the config."""
        if stream not in TUNABLE_STREAMS or self._closed:
            raise ValueError('stream is not tunable or subscription is closed')
        target = self.config[stream]
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
        if self._closed or buffered >= MAX_BUFFERED_BYTES:
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

    # ── state streams: latest wins ─────────────────────────────────────
    def owe(self, stream, frame):
        """
        Hold `frame` for `stream` until its in-flight write completes.

        At most one per stream: an older owed frame is replaced, and
        counted as superseded. A closed subscription owes nothing.
        """
        if self._closed or stream not in STATE_STREAMS:
            return False
        if stream in self._owed:
            self.superseded[stream] += 1
        self._owed[stream] = frame
        return True

    def owed(self, stream):
        """Return whether a frame is being held for `stream`."""
        return stream in self._owed

    def take_owed(self, stream):
        """Remove and return the owed frame for `stream`, or None."""
        return self._owed.pop(stream, None)

    def supersede_owed(self, stream):
        """Discard the owed frame: a newer one is going out instead."""
        if self._owed.pop(stream, None) is not None:
            self.superseded[stream] += 1

    def should_send(self, stream):
        """Return subscription eligibility; rate/backpressure are separate."""
        return self.wants(stream)

    def queue_depth(self, stream):
        """
        Count writes outstanding for `stream`: in flight, plus one owed.

        Never more than two, and the owed one is a reference to the
        shared frame, not a copy.
        """
        pending = self._inflight.get(stream)
        in_flight = int(pending is not None and not pending.done())
        return in_flight + int(stream in self._owed)

    def close(self):
        """Release bookkeeping without cancelling transport-owned writes."""
        self._closed = True
        self.streams.clear()
        self._inflight.clear()
        self._owed.clear()

    def delivery(self):
        """
        Report what THIS client has been sent, dropped and superseded.

        Per client, from this client's own counters -- never the shared
        frame's -- for the telemetry tick. Streams with nothing to report
        are left out to keep the tick small.
        """
        return {name: {'sent': self.sent[name],
                       'dropped': self.dropped[name],
                       'superseded': self.superseded[name]}
                for name in STREAMS
                if self.sent[name] or self.dropped[name]
                or self.superseded[name]}

    # ── views ──────────────────────────────────────────────────────────
    def as_dict(self):
        """Build the subscription document sent in welcome and on change."""
        return {
            'streams': sorted(self.streams),
            'available': list(STREAMS),
            'default': list(DEFAULT_STREAMS),
            'binary': self.binary,
            'binary_streams': list(BINARY_STREAMS),
            'binary_capable': list(BINARY_CAPABLE),
            'config': {name: dict(values)
                       for name, values in sorted(self.config.items())},
            'sent': dict(self.sent),
            'dropped': dict(self.dropped),
            'superseded': dict(self.superseded),
            'queue_depth': {name: self.queue_depth(name) for name in STREAMS},
            'send_attempts': {name: self.sent[name] + self.dropped[name]
                              for name in STREAMS},
        }
