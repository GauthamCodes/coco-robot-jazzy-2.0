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

"""Instrumentation: measured rates, drops, latency and CPU."""

import json

from coco_web import metrics as met
from coco_web import streams as st

import pytest


def test_a_rate_is_measured_over_a_real_interval():
    """
    Ten events one second apart is 10 Hz, computed not configured.

    The whole point of this module is that every number is something
    that happened.
    """
    meter = met.Meter()
    meter._at = 0.0
    for _ in range(10):
        meter.count()
    assert meter.sample(now=1.0) == 10.0


def test_a_rate_is_not_recomputed_inside_the_window():
    """Sampling faster than the window returns the last value, not noise."""
    meter = met.Meter()
    meter._at = 0.0
    meter.count()
    assert meter.sample(now=0.1) == 0.0


def test_byte_rates_are_measured_too():
    """Frames per second does not say whether a link is saturated."""
    meter = met.Meter()
    meter._at = 0.0
    meter.count(1000)
    meter.count(1000)
    meter.sample(now=1.0)
    assert meter.byte_rate == 2000.0


def test_in_and_out_rates_are_reported_separately():
    """
    The gap between them IS the dropping.

    Reporting only one would leave "the video is choppy" unanswerable
    without attaching a profiler.
    """
    metrics = met.Metrics(st.STREAMS)
    metrics.inbound['camera']._at = 0.0
    metrics.outbound['camera']._at = 0.0
    for _ in range(15):
        metrics.observed('camera', 10_000)
    for _ in range(10):
        metrics.sent('camera', 10_000)
    metrics.sample(now=1.0)
    camera = metrics.as_dict()['streams']['camera']
    assert camera['in_hz'] == 15.0
    assert camera['out_hz'] == 10.0


def test_drops_are_counted_per_stream():
    """A client can be told how much it missed, and of what."""
    metrics = met.Metrics(st.STREAMS)
    metrics.dropped_frame('camera', 3)
    metrics.dropped_frame('lidar')
    document = metrics.as_dict()['streams']
    assert document['camera']['dropped'] == 3
    assert document['lidar']['dropped'] == 1


def test_mission_latency_measures_arrival_to_delivery():
    """
    The number that answers "is the UI behind the robot?".

    Measured from the ROS callback to the frame that carried the state
    out, not from anything configured.
    """
    metrics = met.Metrics(st.STREAMS)
    metrics.mission_seen(now=100.0)
    assert metrics.mission_delivered(now=100.05) == pytest.approx(50.0)


def test_mission_latency_is_none_before_anything_arrives():
    """No measurement is reported as absent, never as zero."""
    metrics = met.Metrics(st.STREAMS)
    assert metrics.mission_delivered() is None
    assert metrics.as_dict()['mission_latency_ms'] is None


def test_a_repeated_state_line_does_not_restart_the_clock():
    """
    Consuming the stamp is what makes the next tick not re-measure it.

    The executive re-asserts the same line at 2 Hz; measuring the delay
    to a repeat would report the tick interval instead of the lag.
    """
    metrics = met.Metrics(st.STREAMS)
    metrics.mission_seen(now=100.0)
    assert metrics.mission_delivered(now=100.05) == pytest.approx(50.0)
    assert metrics.mission_delivered(now=100.10) is None


def test_the_peak_socket_buffer_is_remembered():
    """A high-water mark survives the moment it happened in."""
    metrics = met.Metrics(st.STREAMS)
    metrics.buffered(5000)
    metrics.buffered(100)
    assert metrics.as_dict()['peak_buffer_bytes'] == 5000


def test_cpu_is_a_share_of_one_core():
    """
    100 means one core saturated, not the whole machine.

    Per-core is what makes "the JPEG encoder is the bottleneck" visible
    on a twelve-core box, where per-machine would read as 8 %.
    """
    metrics = met.Metrics(st.STREAMS)
    metrics._cpu_at = 0.0
    metrics._cpu_used = 0.0
    metrics._process_seconds = staticmethod(lambda: 0.5)
    assert metrics.sample_cpu(now=1.0) == 50.0


def test_cpu_never_goes_negative():
    """A clock that jumps backwards must not produce a negative share."""
    metrics = met.Metrics(st.STREAMS)
    metrics._cpu_at = 0.0
    metrics._cpu_used = 10.0
    metrics._process_seconds = staticmethod(lambda: 1.0)
    assert metrics.sample_cpu(now=1.0) == 0.0


def test_the_document_is_json_safe():
    """It rides the telemetry frame, so it must encode."""
    metrics = met.Metrics(st.STREAMS)
    metrics.observed('lidar')
    metrics.sent('lidar', 480)
    json.dumps(metrics.as_dict())


def test_every_stream_appears_in_the_document():
    """A stream missing from the report reads as a stream at zero."""
    document = met.Metrics(st.STREAMS).as_dict()['streams']
    assert set(document) == set(st.STREAMS)


def test_metrics_adds_no_new_dependency():
    """
    CPU comes from os.times(), not psutil.

    psutil is installed on this machine but is not a dependency of this
    package, and adding one for a single number would be a poor trade in
    an image that has to build offline.
    """
    with open(met.__file__, encoding='utf-8') as handle:
        source = handle.read()
    assert 'import psutil' not in source
    assert 'import rclpy' not in source
