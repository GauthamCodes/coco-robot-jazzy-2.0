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
What the platform is actually doing, measured rather than assumed.

Pure module: no rclpy, no sockets, stdlib only. Rates are computed from
counters over a real elapsed interval, so every number here is something
that happened rather than something configured.

That distinction is the point. ``camera.fps`` is not the rate the client
negotiated, it is the rate frames were *sent*; ``in_hz`` is the rate they
*arrived* from ROS. When those two disagree the difference is the
dropping, and being able to see that without attaching a profiler is the
difference between "the video looks choppy" and "the encoder is keeping
up and the socket is not".

CPU comes from ``os.times()`` rather than psutil, which is a dependency
this package does not otherwise need. It measures this process only --
the simulator's CPU is Gazebo's business and is not attributed here.

Nothing in this module is written into a document. It is live
instrumentation; the numbers that end up in ``RESULTS.md`` come from a
measured run and are recorded by hand, with their provenance.
"""

import os
import time

#: Seconds between rate recomputations. One second is short enough to
#: see a stall and long enough that a 10 Hz tick is not quantised into
#: nonsense.
WINDOW_S = 1.0

#: Clock ticks per second, for turning os.times() into a fraction.
_CLOCK = os.sysconf('SC_CLK_TCK') if hasattr(os, 'sysconf') else 100


class Meter:
    """One named counter and the rate it is accumulating at."""

    def __init__(self):
        """Start empty, with no interval measured yet."""
        self.total = 0
        self.bytes = 0
        self._marked = 0
        self._marked_bytes = 0
        self._at = time.monotonic()
        self.rate = 0.0
        self.byte_rate = 0.0

    def count(self, size=0):
        """Record one event, optionally carrying a payload size."""
        self.total += 1
        self.bytes += size

    def sample(self, now=None):
        """Recompute the rate if the window has elapsed. Returns the rate."""
        stamp = time.monotonic() if now is None else now
        elapsed = stamp - self._at
        if elapsed < WINDOW_S:
            return self.rate
        self.rate = (self.total - self._marked) / elapsed
        self.byte_rate = (self.bytes - self._marked_bytes) / elapsed
        self._marked = self.total
        self._marked_bytes = self.bytes
        self._at = stamp
        return self.rate


class Metrics:
    """Every counter the platform publishes, and the CPU it is using."""

    def __init__(self, streams=()):
        """Create one in- and one out-meter per stream, plus totals."""
        self.inbound = {name: Meter() for name in streams}
        self.outbound = {name: Meter() for name in streams}
        self.frames_out = Meter()
        self.dropped = {name: 0 for name in streams}
        self.clients = 0
        self.peak_buffer = 0
        # Mission-state latency: monotonic when the ROS callback fired,
        # and the measured gap to the frame that carried it out. This is
        # the number that answers "is the UI behind the robot?".
        self.mission_stamp = None
        self.mission_latency_ms = None
        self._cpu_at = time.monotonic()
        self._cpu_used = self._process_seconds()
        self.cpu_percent = 0.0

    @staticmethod
    def _process_seconds():
        """CPU seconds this process has consumed, user plus system."""
        times = os.times()
        return times.user + times.system

    def observed(self, stream, size=0):
        """Record one message arriving from ROS."""
        meter = self.inbound.setdefault(stream, Meter())
        meter.count(size)

    def sent(self, stream, size=0):
        """Record one frame written to a client."""
        self.outbound.setdefault(stream, Meter()).count(size)
        self.frames_out.count(size)

    def dropped_frame(self, stream, count=1):
        """Record frames deliberately not sent, for backpressure."""
        self.dropped[stream] = self.dropped.get(stream, 0) + count

    def mission_seen(self, now=None):
        """Mark when a mission-state message arrived from ROS."""
        self.mission_stamp = time.monotonic() if now is None else now

    def mission_delivered(self, now=None):
        """Measure the gap from that arrival to the frame carrying it."""
        if self.mission_stamp is None:
            return None
        stamp = time.monotonic() if now is None else now
        self.mission_latency_ms = (stamp - self.mission_stamp) * 1000.0
        self.mission_stamp = None
        return self.mission_latency_ms

    def buffered(self, size):
        """Record the high-water mark of any client's socket buffer."""
        self.peak_buffer = max(self.peak_buffer, int(size or 0))

    def sample_cpu(self, now=None):
        """
        Recompute this process's CPU share over the elapsed interval.

        A percentage of ONE core: 100 means one core saturated, and on a
        twelve-core machine that is 8 % of the box. Reporting it per-core
        rather than per-machine is what makes "the encoder is the
        bottleneck" visible.
        """
        stamp = time.monotonic() if now is None else now
        elapsed = stamp - self._cpu_at
        if elapsed < WINDOW_S:
            return self.cpu_percent
        used = self._process_seconds()
        self.cpu_percent = max(0.0, (used - self._cpu_used) / elapsed * 100.0)
        self._cpu_used = used
        self._cpu_at = stamp
        return self.cpu_percent

    def sample(self, now=None):
        """Recompute every rate. Called once per telemetry tick."""
        for meter in self.inbound.values():
            meter.sample(now)
        for meter in self.outbound.values():
            meter.sample(now)
        self.frames_out.sample(now)
        self.sample_cpu(now)

    def as_dict(self):
        """
        Build the perf document, for telemetry and /api/metrics.

        ``in_hz`` is what ROS delivered and ``out_hz`` is what went to
        clients. The gap between them IS the dropping, which is why both
        are reported rather than one.
        """
        streams = {}
        for name in sorted(set(self.inbound) | set(self.outbound)):
            inbound = self.inbound.get(name)
            outbound = self.outbound.get(name)
            streams[name] = {
                'in_hz': round(inbound.rate, 2) if inbound else 0.0,
                'out_hz': round(outbound.rate, 2) if outbound else 0.0,
                'sent': outbound.total if outbound else 0,
                'dropped': self.dropped.get(name, 0),
                'bytes_per_s': (round(outbound.byte_rate, 1)
                                if outbound else 0.0),
            }
        return {
            'clients': self.clients,
            'frames_per_s': round(self.frames_out.rate, 2),
            'bytes_per_s': round(self.frames_out.byte_rate, 1),
            'cpu_percent': round(self.cpu_percent, 1),
            'peak_buffer_bytes': self.peak_buffer,
            'mission_latency_ms': (
                None if self.mission_latency_ms is None
                else round(self.mission_latency_ms, 1)),
            'streams': streams,
        }
