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
Tests that CONSTRUCT the mission executive node.

These exist because of what C2-M2.1 cost. The terrain observer's pure
core had thorough unit tests and its ROS adapter had none; three defects
lived in that gap, and one of them — a helper called with the wrong
number of arguments — meant the node could not start at all. Every test
in this file builds the real :class:`MissionExecutive`, on a real ROS
context, with no simulator and no other node on the graph.

The most important test here is the one that finds nothing:
``test_no_publisher_to_the_wheel_controller``. cmd_vel_arbiter is the
sole publisher to ``/diff_drive_controller/cmd_vel``, four control
paradigms hand the wheels back and forth through it, and an executive
that publishes velocity directly breaks that silently — the robot tracks
the average of two sources rather than obeying one.
"""

import os
import sys

import pytest

import rclpy

from std_srvs.srv import Trigger

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import mission_executive as mx  # noqa: E402
import mission_states as ms  # noqa: E402

HERE = os.path.dirname(__file__)
SCRIPTS = os.path.join(HERE, '..', 'scripts')
LAUNCH = os.path.join(HERE, '..', 'launch', 'mission.launch.py')
ROS_CLEAN = os.path.join(HERE, '..', '..', 'gazebo_models', 'scripts',
                         'ros_clean.sh')

WHEEL_TOPIC = '/diff_drive_controller/cmd_vel'


@pytest.fixture
def context():
    """A ROS context for one test, torn down whatever happens."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(context):
    """A constructed executive, destroyed afterwards."""
    executive = mx.MissionExecutive(colour='blue')
    yield executive
    executive.destroy_node()


class TestItConstructs:
    """The C2-M2.1 lesson: a node nobody built is a node nobody tested."""

    def test_it_starts_at_all(self, node):
        assert node.get_name() == 'mission_executive'
        assert node.machine.state == ms.IDLE

    def test_the_colour_picks_the_lane_from_the_table(self, node):
        assert node.colour == 'blue'
        assert node.plan.lane == pytest.approx(0.25)

    def test_it_starts_without_a_colour_and_says_so(self, context):
        executive = mx.MissionExecutive()
        try:
            assert executive.colour is None
            reply = executive._on_start(Trigger.Request(), Trigger.Response())
            assert not reply.success
            assert 'colour' in reply.message
        finally:
            executive.destroy_node()

    def test_the_tick_timer_survives_a_stopped_clock(self, node):
        # Both timers must run on a steady clock: a node-clock timer
        # under use_sim_time stops when /clock stops, and a stopped
        # executive cannot report that the clock stopped.
        from rclpy.clock import ClockType
        assert node._steady.clock_type == ClockType.STEADY_TIME
        for timer in node.timers:
            assert timer.clock.clock_type == ClockType.STEADY_TIME


class TestCmdVelInvariant:
    """cmd_vel_arbiter stays the sole publisher to the wheels."""

    def test_no_publisher_to_the_wheel_controller(self, node):
        topics = [publisher.topic_name for publisher in node.publishers]
        assert WHEEL_TOPIC not in topics
        assert not any('cmd_vel' in topic and topic.endswith('cmd_vel')
                       for topic in topics)

    def test_the_package_contains_no_twist_at_all(self):
        # A package that never imports a Twist cannot publish one, which
        # is a stronger statement than "no publisher on that topic today".
        for name in os.listdir(SCRIPTS):
            if not name.endswith('.py'):
                continue
            body = open(os.path.join(SCRIPTS, name)).read()
            # Twist and TwistStamped are the only velocity types on this
            # robot. A file that imports neither cannot publish one.
            assert 'Twist' not in body, name

    def test_it_publishes_only_the_three_mission_topics(self, node):
        builtin = ('/rosout', '/parameter_events')
        topics = sorted(publisher.topic_name
                        for publisher in node.publishers
                        if publisher.topic_name not in builtin)
        assert topics == ['/mission/mode', '/mission/state',
                          '/mission/target_colour']

    def test_it_selects_a_source_rather_than_driving(self, node):
        # /mission/mode is arbitration, not actuation: the arbiter still
        # decides whether the selected source is fresh enough to forward.
        for contract in ms.CONTRACTS.values():
            assert contract.mode in ('idle', 'nav', 'rl', 'approach')


class TestInterfaces:
    """It talks to what already exists, and nothing new."""

    def test_it_offers_the_operator_services(self, node):
        offered = {service.srv_name for service in node.services}
        assert {'/mission/start', '/mission/abort'} <= offered

    def test_it_has_a_client_for_every_subsystem_service(self, node):
        assert set(node.service_clients) == set(mx.SERVICES)
        for state, service in ms.STATE_SERVICE.items():
            assert service in node.service_clients, state
        # rclpy's Node owns `_clients`; shadowing it breaks the executor.
        assert all(hasattr(client, 'service_name')
                   for client in node.clients)

    def test_the_stop_services_are_the_ones_that_exist(self, node):
        # /grasp has no stop: an arm trajectory in flight is finished by
        # move_group, not interrupted from here.
        assert mx.STOP_SERVICES == ('/ramp/stop', '/approach/stop')
        for name in mx.STOP_SERVICES:
            assert name in node.service_clients

    def test_it_uses_the_standard_navigation_action(self, node):
        assert node._nav._action_name == 'navigate_to_pose'


class TestObservation:
    """The node's only job is to describe the world accurately."""

    def test_a_fresh_node_sees_nothing(self, node):
        obs = node.observe()
        assert obs.pose is None
        assert not obs.localized
        assert not obs.ramp.seen()
        assert obs.colour == 'blue'

    def test_a_status_line_lands_in_the_right_view(self, node):
        from std_msgs.msg import String
        node._on_status('ramp', String(data='segment=climb outcome=none'))
        obs = node.observe()
        assert obs.ramp.get('segment') == 'climb'
        assert obs.ramp.seen()

    def test_an_amcl_pose_only_records_presence(self, node):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        node._on_amcl(PoseWithCovarianceStamped())
        assert node.observe().localized
        # And nothing anywhere claims the estimate is any GOOD. That
        # threshold is C2-M5's and has never been calibrated.
        assert not hasattr(node, 'localization_quality')

    def test_odometry_becomes_a_world_pose_with_a_heading(self, node):
        from nav_msgs.msg import Odometry
        msg = Odometry()
        msg.pose.pose.position.x = 1.5
        msg.pose.pose.position.y = -0.25
        msg.pose.pose.orientation.z = 0.7071068
        msg.pose.pose.orientation.w = 0.7071068
        node._on_odom(msg)
        obs = node.observe()
        assert obs.pose[0] == pytest.approx(1.5)
        assert obs.pose[1] == pytest.approx(-0.25)
        assert obs.pose[2] == pytest.approx(1.5707963, abs=1e-5)
        assert obs.pose_stamp is not None


class TestOperatorControl:
    """Start and abort, and what they refuse."""

    def test_start_arms_the_machine(self, node):
        assert not node.started
        reply = node._on_start(Trigger.Request(), Trigger.Response())
        assert reply.success
        assert node.started

    def test_abort_is_always_accepted(self, node):
        reply = node._on_abort(Trigger.Request(), Trigger.Response())
        assert reply.success
        assert node.abort_requested

    def test_start_refuses_once_the_mission_has_finished(self, node):
        node.machine.state = ms.COMPLETE
        reply = node._on_start(Trigger.Request(), Trigger.Response())
        assert not reply.success
        assert 'finished' in reply.message

    def test_a_colour_change_mid_mission_is_refused(self, node):
        from std_msgs.msg import String
        node.machine.state = ms.CLIMB
        node._on_colour(String(data='green'))
        assert node.colour == 'blue'
        assert node.plan.lane == pytest.approx(0.25)

    def test_a_colour_from_the_panel_before_the_start_is_taken(self, context):
        from std_msgs.msg import String
        executive = mx.MissionExecutive()
        try:
            executive._on_colour(String(data='yellow'))
            assert executive.colour == 'yellow'
            assert executive.plan.lane == pytest.approx(0.75)
            # It did not ask for the colour, so it must not assert it —
            # two publishers on that topic at 2 Hz is how the mode topic
            # went wrong once already.
            assert not executive.announce
        finally:
            executive.destroy_node()

    def test_a_colour_it_was_given_is_re_asserted(self, node):
        # approach_server and grasp_server take the choice off the topic
        # and refuse to start without it; with no panel there would
        # otherwise be no publisher at all.
        assert node.announce

    def test_nonsense_colours_are_ignored(self, node):
        from std_msgs.msg import String
        node._on_colour(String(data='mauve'))
        assert node.colour == 'blue'


class TestRunning:
    """One real spin, with nothing else on the graph."""

    def test_it_publishes_its_state_while_idle(self, node):
        for _ in range(30):
            rclpy.spin_once(node, timeout_sec=0.05)
        assert node._last_published is not None
        fields = ms.parse_kv(node._last_published)
        assert fields['state'] == ms.IDLE
        assert fields['mode'] == 'idle'

    def test_it_does_not_move_the_robot_before_being_started(self, node):
        for _ in range(30):
            rclpy.spin_once(node, timeout_sec=0.05)
        assert node.machine.state == ms.IDLE
        assert node.requests_issued() == []

    def test_a_started_mission_waits_on_its_inputs(self, node):
        node._on_start(Trigger.Request(), Trigger.Response())
        for _ in range(30):
            rclpy.spin_once(node, timeout_sec=0.05)
        # No odometry, no ramp_driver, no AMCL: it must sit in LOCALIZE
        # rather than driving off on an assumption.
        assert node.machine.state == ms.LOCALIZE

    def test_the_exit_code_is_zero_only_for_a_complete_mission(self, node):
        node.machine.state = ms.COMPLETE
        assert node.exit_code() == 0
        node.machine.state = ms.ABORT
        assert node.exit_code() == 1


class TestWiring:
    """The launch file and the cleanup script know about the new node."""

    def test_the_launch_file_starts_the_executive(self):
        body = open(LAUNCH).read()
        assert "executable='mission_executive.py'" in body
        assert "DeclareLaunchArgument(\n            'executive'" in body

    def test_ros_clean_kills_it_by_process_name(self):
        # Anything added to a launch file has to be added here too: its
        # command line does not contain "mission.launch.py", so a sweep
        # by launch-file name leaves it running. And the pattern must be
        # bracketed, or a pkill matches its own command line.
        body = open(ROS_CLEAN).read()
        assert "'mission_executiv[e]'" in body

    def test_the_cmakelists_installs_both_files(self):
        body = open(os.path.join(HERE, '..', 'CMakeLists.txt')).read()
        assert 'scripts/mission_executive.py' in body
        # mission_states is a library, but it installs beside the node
        # because that is how `import mission_states` resolves at runtime.
        assert 'scripts/mission_states.py' in body


class TestPureHelpers:
    """The small things, checked directly."""

    def test_yaw_of_a_quarter_turn(self):
        class Q:
            x = y = 0.0
            z = 0.7071068
            w = 0.7071068
        assert mx.yaw_of(Q()) == pytest.approx(1.5707963, abs=1e-5)

    def test_yaw_of_the_identity_is_zero(self):
        class Q:
            x = y = z = 0.0
            w = 1.0
        assert mx.yaw_of(Q()) == pytest.approx(0.0)

    def test_the_cli_matches_traverse_demos(self):
        args = mx.parse_args(['--colour', 'green', '--no-grasp'])
        assert args.colour == 'green'
        assert args.no_grasp
        assert not args.autostart
