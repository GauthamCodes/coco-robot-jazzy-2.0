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
Tests for the localization health monitor node.

The C2-M2.1 lesson applies here too: a node nobody built is a node nobody
tested, and this one is constructed for real against a live rclpy context.

What is being pinned is not the arithmetic -- that is pure and lives in
``test_localization_health.py`` -- but the three things that can only go
wrong in the adapter:

1. it publishes and does not drive
2. the scan subscription is BEST_EFFORT, or the node is silently blind
3. the status line is the shape ``mission_states.parse_kv`` reads, and
   carries the latched flags the executive actually keys on
"""

import os
import sys

import pytest

import rclpy

from rclpy.qos import QoSReliabilityPolicy

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import localization_health as lh  # noqa: E402
import localization_monitor as lm  # noqa: E402
import mission_states as ms  # noqa: E402

HERE = os.path.dirname(__file__)
SCRIPTS = os.path.join(HERE, '..', 'scripts')
LAUNCH = os.path.join(HERE, '..', 'launch', 'mission.launch.py')
ROS_CLEAN = os.path.join(HERE, '..', '..', 'gazebo_models', 'scripts',
                         'ros_clean.sh')

WHEEL_TOPIC = '/diff_drive_controller/cmd_vel'


@pytest.fixture
def context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(context):
    monitor = lm.LocalizationMonitor()
    yield monitor
    monitor.destroy_node()


class TestItConstructs:

    def test_it_starts_at_all(self, node):
        assert node.get_name() == 'localization_monitor'

    def test_it_ships_the_measured_thresholds(self, node):
        assert node.thresholds.lik_mean_d_max == lh.LIK_MEAN_D_MAX
        assert node.thresholds.lik_frac_near_min == lh.LIK_FRAC_NEAR_DISABLED

    def test_it_carries_EVERY_shipped_threshold_not_just_the_exposed_ones(
            self, node):
        # Experiment 2 caught this live. The node used to name only the
        # two parameters it exposes, so `max_amcl_age` fell back to the
        # dataclass default of 5.0 and the C2-M5.1 decision to switch
        # that check off never reached the running node: the mission
        # degraded on POSE_STALE at startup and spun to recover from a
        # fault that did not exist. Any field added to C2M51_THRESHOLDS
        # in future must reach the node the same way.
        for field in ('max_amcl_age', 'max_map_odom_age', 'min_beams'):
            assert getattr(node.thresholds, field) \
                == getattr(lh.C2M51_THRESHOLDS, field), field

    def test_the_amcl_age_check_is_off_on_the_running_node(self, node):
        assert node.thresholds.max_amcl_age is None

    def test_the_two_holds_are_separate_objects(self, node):
        # One shared Persistence would make a resume clear the trigger's
        # own history, which is a subtle way to resume early.
        assert node.degraded is not node.healthy
        assert node.healthy.hold > node.degraded.hold

    def test_it_starts_with_no_field_and_no_scan(self, node):
        assert node.field is None
        assert node.scan is None


class TestItPublishesAndDoesNotDrive:
    """CLAUDE.md §4: cmd_vel_arbiter is the sole publisher to the wheels."""

    def test_no_publisher_to_the_wheel_controller(self, node):
        topics = [publisher.topic_name for publisher in node.publishers]
        assert WHEEL_TOPIC not in topics
        assert not any('cmd_vel' in topic for topic in topics)

    def test_it_publishes_only_its_status_topic(self, node):
        builtin = ('/rosout', '/parameter_events')
        topics = sorted(publisher.topic_name for publisher in node.publishers
                        if publisher.topic_name not in builtin)
        assert topics == ['/localization/health']

    def test_the_source_imports_no_velocity_type(self):
        body = open(os.path.join(SCRIPTS, 'localization_monitor.py')).read()
        assert 'Twist' not in body

    def test_it_takes_no_ground_truth(self, node):
        # The C2-M5.0 constraint, enforced rather than commented: the
        # simulator's own odometry is how the RECORDER scores this
        # signal, never an input to it. Checked against what the node
        # actually subscribed to, not against its source text -- the
        # source names the topic in a docstring precisely to say it is
        # absent, and a grep would fail on the explanation.
        topics = [sub.topic_name for sub in node.subscriptions]
        assert '/model/coco/odometry' not in topics
        assert not any('model/coco' in topic for topic in topics)

    def test_it_subscribes_only_to_deployable_inputs(self, node):
        builtin = ('/parameter_events',)
        topics = sorted(sub.topic_name for sub in node.subscriptions
                        if sub.topic_name not in builtin)
        assert topics == ['/amcl_pose', '/map', '/scan', '/tf', '/tf_static']


class TestTheScanSubscription:
    """The trap that makes a monitor silently blind."""

    def test_the_scan_subscription_is_best_effort(self, node):
        scans = [sub for sub in node.subscriptions
                 if sub.topic_name == '/scan']
        assert len(scans) == 1
        assert scans[0].qos_profile.reliability \
            == QoSReliabilityPolicy.BEST_EFFORT

    def test_an_unknown_scan_topic_still_subscribes_best_effort(self, context):
        # is_best_effort() returns False for a topic it does not know, so
        # deriving RELIABLE from it would go blind on a rename. The
        # fallback has to be the safe direction.
        monitor = lm.LocalizationMonitor()
        monitor.set_parameters([])
        try:
            subs = [sub for sub in monitor.subscriptions
                    if sub.topic_name == '/scan']
            assert subs[0].qos_profile.reliability \
                == QoSReliabilityPolicy.BEST_EFFORT
        finally:
            monitor.destroy_node()

    def test_the_map_subscription_is_latched(self, context):
        from rclpy.qos import QoSDurabilityPolicy
        monitor = lm.LocalizationMonitor()
        try:
            maps = [sub for sub in monitor.subscriptions
                    if sub.topic_name == '/map']
            assert len(maps) == 1
            # map_server publishes the map once. A VOLATILE subscriber
            # that starts late never sees it and the field is never built.
            assert maps[0].qos_profile.durability \
                == QoSDurabilityPolicy.TRANSIENT_LOCAL
        finally:
            monitor.destroy_node()


class TestTheLatchesAreCoherent:
    """Experiment 2: both flags read 1 at once, and the resume was bogus.

    Drives the node's real ``_tick`` with a scripted observation, so what
    is under test is the code that runs on the robot rather than a
    re-implementation of its rule.
    """

    def drive(self, node, obs, seconds, dt=0.1):
        """Feed one observation repeatedly, advancing a fake clock."""
        published = []
        node.status.publish = lambda msg: published.append(msg.data)
        node._observe = lambda: obs
        t = getattr(self, '_t', 0.0)
        for _ in range(int(seconds / dt)):
            node.now = lambda captured=t: captured
            node._tick()
            t += dt
        self._t = t
        return [ms.parse_kv(line) for line in published]

    def healthy_obs(self):
        return lh.Observation(lik_mean_d=0.05, lik_frac_near=0.9,
                              lik_beams=55, map_odom_age=-0.44)

    def bad_obs(self):
        return lh.Observation(lik_mean_d=0.49, lik_frac_near=0.23,
                              lik_beams=54, map_odom_age=-0.44)

    def test_degraded_and_healthy_are_never_both_set(self, node):
        self._t = 0.0
        fields = self.drive(node, self.healthy_obs(), 6.0)
        fields += self.drive(node, self.bad_obs(), 6.0)
        fields += self.drive(node, self.healthy_obs(), 8.0)
        both = [f for f in fields
                if f['degraded'] == '1' and f['healthy'] == '1']
        assert both == []

    def test_a_long_healthy_run_latches_healthy(self, node):
        self._t = 0.0
        fields = self.drive(node, self.healthy_obs(), 6.0)
        assert fields[-1]['healthy'] == '1'
        assert fields[-1]['degraded'] == '0'

    def test_a_degradation_clears_the_healthy_latch_at_once(self, node):
        # The bug: healthy=1 survived into RELOCALIZE and satisfied the
        # resume gate before the spin had turned the robot.
        self._t = 0.0
        self.drive(node, self.healthy_obs(), 6.0)
        fields = self.drive(node, self.bad_obs(), 4.0)
        latched = [f for f in fields if f['degraded'] == '1']
        assert latched, 'the degradation should latch within 4 s'
        assert all(f['healthy'] == '0' for f in latched)

    def test_health_must_be_re_earned_after_a_degradation(self, node):
        self._t = 0.0
        self.drive(node, self.healthy_obs(), 6.0)
        self.drive(node, self.bad_obs(), 6.0)
        # Immediately after the fault clears, health is NOT yet declared.
        soon = self.drive(node, self.healthy_obs(), 1.0)
        assert all(f['healthy'] == '0' for f in soon)
        # It comes back once the evidence has been rebuilt.
        later = self.drive(node, self.healthy_obs(), 8.0)
        assert later[-1]['healthy'] == '1'
        assert later[-1]['degraded'] == '0'


class TestTheStatusLine:

    def line(self, verdict, degraded, healthy, held, obs):
        return lm.format_status(verdict, degraded, healthy, held, obs)

    def test_it_parses_with_the_executive_s_own_parser(self):
        obs = lh.Observation(lik_mean_d=0.053, lik_frac_near=0.875,
                             lik_beams=57, cov_sigma_xy=0.376,
                             map_odom_age=-0.44)
        verdict = lh.classify(obs, lh.C2M51_THRESHOLDS)
        fields = ms.parse_kv(self.line(verdict, False, True, 9.0, obs))
        assert fields['verdict'] == lh.CONSISTENT
        assert fields['degraded'] == '0'
        assert fields['healthy'] == '1'

    def test_a_degraded_line_says_so_in_the_field_the_executive_reads(self):
        obs = lh.Observation(lik_mean_d=0.49, lik_frac_near=0.23,
                             lik_beams=54, map_odom_age=-0.44)
        verdict = lh.classify(obs, lh.C2M51_THRESHOLDS)
        assert verdict.verdict == lh.INCONSISTENT
        fields = ms.parse_kv(self.line(verdict, True, False, 2.1, obs))
        assert fields['degraded'] == '1'
        assert fields['healthy'] == '0'

    def test_missing_numbers_render_as_the_project_s_own_placeholder(self):
        obs = lh.Observation(on_mapped_ground=False)
        verdict = lh.classify(obs, lh.C2M51_THRESHOLDS)
        fields = ms.parse_kv(self.line(verdict, False, False, 0.0, obs))
        assert fields['d'] == '--'
        assert fields['near'] == '--'
        assert fields['mapped'] == '0'

    def test_covariance_is_reported_and_is_not_the_verdict(self):
        # The C2-M5.0 headline, kept enforceable: sigma appears in the
        # line for a human, and moving it does not move the verdict.
        low = lh.Observation(lik_mean_d=0.49, lik_frac_near=0.23,
                             lik_beams=54, cov_sigma_xy=0.070,
                             map_odom_age=-0.44)
        high = lh.Observation(lik_mean_d=0.49, lik_frac_near=0.23,
                              lik_beams=54, cov_sigma_xy=1.369,
                              map_odom_age=-0.44)
        a = lh.classify(low, lh.C2M51_THRESHOLDS)
        b = lh.classify(high, lh.C2M51_THRESHOLDS)
        assert a.verdict == b.verdict == lh.INCONSISTENT
        assert 'sigma=0.070' in self.line(a, True, False, 2.1, low)


class TestTheLaunchAndCleanupWiring:
    """Two files that have to be edited together, and were not, once."""

    def test_the_launch_file_starts_the_monitor(self):
        body = open(LAUNCH).read()
        assert 'localization_monitor.py' in body

    def test_ros_clean_kills_it(self):
        # CLAUDE.md: anything added to a launch file has to be added
        # here too. Two monitors would both publish /localization/health
        # and the executive ACTS on that topic.
        body = open(ROS_CLEAN).read()
        assert 'localization_monito[r]' in body

    def test_the_kill_pattern_is_bracketed(self):
        # A bash -c process's own command line contains the script text,
        # so an unbracketed pattern kills the cleaner itself. Only the
        # QUOTED pattern entries matter; the surrounding comment names
        # the node in plain text on purpose.
        body = open(ROS_CLEAN).read()
        assert "'localization_monito[r]'" in body
        assert "'localization_monitor'" not in body
