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

"""TopicRate's sliding-window estimator, driven without a ROS graph.

TopicRate takes its timestamps from the node clock, so a fake node with a
controllable clock exercises the whole window/eviction path — including
the staleness edge cases that only occur when a sensor dies, which are
awkward to produce against a live sim.
"""
import pytest

pytest.importorskip('rclpy')
pytest.importorskip('diagnostic_msgs')

from coco_config.diagnostics_node import WINDOW, TopicRate  # noqa: E402


class _FakeClock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        outer = self

        class _T:
            nanoseconds = 0

            def __init__(self):
                self.nanoseconds = int(outer.t * 1e9)
        return _T()


class _FakeNode:
    """Just enough Node surface for TopicRate: a clock and a subscribe hook."""

    def __init__(self):
        self._clock = _FakeClock()
        self.subscriptions_made = []

    def get_clock(self):
        return self._clock

    def create_subscription(self, msg_type, topic, cb, qos):
        self.subscriptions_made.append((topic, msg_type, qos))
        return None


def _probe():
    node = _FakeNode()
    return node, TopicRate(node, '/scan', object, 10)


def test_subscribes_to_the_topic_it_is_given():
    node, _ = _probe()
    assert node.subscriptions_made[0][0] == '/scan'


def test_rate_is_zero_before_two_messages():
    """One sample cannot define an interval; it must not divide by zero."""
    node, probe = _probe()
    assert probe.rate() == 0.0
    probe._cb(None)
    assert probe.rate() == 0.0


def test_rate_matches_a_steady_stream():
    node, probe = _probe()
    for _ in range(11):
        probe._cb(None)
        node._clock.t += 0.1          # 10 Hz
    assert probe.rate() == pytest.approx(10.0, rel=1e-6)


def test_rate_uses_intervals_not_message_count():
    """n messages span n-1 intervals; using n would over-report by 1/(n-1)."""
    node, probe = _probe()
    probe._cb(None)
    node._clock.t += 1.0
    probe._cb(None)
    node._clock.t += 1.0
    probe._cb(None)
    assert probe.rate() == pytest.approx(1.0)


def test_old_samples_fall_out_of_the_window():
    """A burst then silence must decay, not be remembered forever."""
    node, probe = _probe()
    for _ in range(20):
        probe._cb(None)
        node._clock.t += 0.05
    node._clock.t += WINDOW * 2       # long gap
    probe._cb(None)
    # every pre-gap sample is now older than WINDOW
    assert probe.rate() == 0.0


def test_age_is_infinite_until_the_first_message():
    node, probe = _probe()
    assert probe.age(node._clock.t) == float('inf')


def test_age_tracks_the_most_recent_message():
    node, probe = _probe()
    probe._cb(None)
    node._clock.t += 3.25
    assert probe.age(node._clock.t) == pytest.approx(3.25)
