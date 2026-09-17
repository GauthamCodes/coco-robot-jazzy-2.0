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

"""C2-NAV.42: the wheels are reachable from Nav2 only through the safety chain.

    controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel_smoothed
      -> collision_monitor -> /cmd_vel -> cmd_vel_relay -> /cmd_vel_gated
      -> cmd_vel_arbiter -> /diff_drive_controller/cmd_vel

Until C2-NAV.42 `nav.launch.py arbiter:=true` pointed the relay at
/cmd_vel_nav -- the controller's own output and the smoother's input -- and
the arbiter read that topic, so the wheels followed the RAW controller while
the collision monitor commanded zero (C2-NAV.41_RESULTS.md section 6). No
test referenced cmd_vel_nav at the time.

Three layers, each asserting something the others cannot:

1. The launch files, RESOLVED: the substitutions are evaluated in a
   LaunchContext exactly as `ros2 launch` evaluates them, so a
   PythonExpression that reads right and resolves wrong is caught.
2. The Nav2 side of the chain, read from nav2_bringup's installed launch
   file and the shipped nav2_params.yaml.
3. A real ROS graph, in-process, on a private DDS domain: the relay and the
   arbiter are built from the parameters layer 1 resolved, a stand-in
   controller publishes a distinctive raw command on /cmd_vel_nav, a
   stand-in collision monitor publishes on /cmd_vel, and what arrives on the
   wheel topic is recorded. The old wiring is run through the SAME scenario
   as a positive control: a check whose pass condition is "the raw value was
   never seen" must first show it can see it (CLAUDE.md).

Nothing in layer 3 races a clock. Every wait polls a condition under a
generous deadline, and every "never" is judged only after the probe has
itself received the raw commands and the wheels have received later ones.
"""

import ast
import importlib.util
import os
import time

from ament_index_python.packages import get_package_share_directory

import pytest

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '..')
NAV_LAUNCH = os.path.join(PKG, 'launch', 'nav.launch.py')
PARAMS = os.path.join(PKG, 'config', 'nav2_params.yaml')

RAW_NAV_TOPIC = '/cmd_vel_nav'
WHEEL_TOPIC = '/diff_drive_controller/cmd_vel'
MONITOR_OUT_TOPIC = '/cmd_vel'

# Distinctive values: nothing else on the private graph produces them.
RAW = 0.777
GATED = 0.123


# ── layer 1: the launch files, resolved ────────────────────────────────────
def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _arbiter_launch_path():
    return os.path.join(get_package_share_directory('custom_teleop'),
                        'launch', 'arbiter.launch.py')


def resolved_node_parameters(path, executable, **configurations):
    """Evaluate one Node's parameters the way `ros2 launch` would.

    Declared arguments are visited first so their defaults apply, then the
    given configurations override them -- the order a parent launch file's
    include arguments take effect in.
    """
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters

    description = _load(path, 'launch_under_test').generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update(configurations)
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(context)
    nodes = [entity for entity in description.entities
             if isinstance(entity, Node) and entity.node_executable == executable]
    assert len(nodes) == 1, f'{path}: want one {executable}, got {len(nodes)}'
    merged = {}
    for params in evaluate_parameters(context, nodes[0]._Node__parameters):
        assert isinstance(params, dict), 'parameter files are not expected here'
        merged.update(params)
    return merged


def relay_output(arbiter):
    return resolved_node_parameters(
        NAV_LAUNCH, 'cmd_vel_relay', arbiter=arbiter)['output_topic']


def arbiter_nav_topic():
    return resolved_node_parameters(
        _arbiter_launch_path(), 'cmd_vel_arbiter')['nav_topic']


class TestLaunchWiring:
    """What nav.launch.py and arbiter.launch.py resolve to."""

    @pytest.mark.parametrize('arbiter', ['true', 'True', '1'])
    def test_in_arbiter_mode_the_relay_does_not_publish_the_raw_topic(
            self, arbiter):
        assert relay_output(arbiter) != RAW_NAV_TOPIC

    def test_in_arbiter_mode_the_relay_feeds_the_arbiters_nav_input(self):
        assert relay_output('true') == arbiter_nav_topic()

    def test_the_arbiter_does_not_read_the_raw_topic(self):
        assert arbiter_nav_topic() != RAW_NAV_TOPIC

    def test_the_launch_file_agrees_with_the_nodes_own_constant(self):
        # web.launch.py and mission.launch.py include arbiter.launch.py; a
        # bare `ros2 run custom_teleop cmd_vel_arbiter` gets the default.
        # Both must land on the same topic.
        from custom_teleop.cmd_vel_relay import GATED_TOPIC
        assert arbiter_nav_topic() == GATED_TOPIC
        assert relay_output('true') == GATED_TOPIC

    @pytest.mark.parametrize('arbiter', ['false', 'False', '0'])
    def test_without_the_arbiter_the_relay_still_drives_the_wheels(
            self, arbiter):
        # Topology A, the harness every C2-NAV tour before .41 ran on.
        assert relay_output(arbiter) == WHEEL_TOPIC

    def test_in_arbiter_mode_the_relay_does_not_publish_the_wheels(self):
        # With the arbiter up, a relay on the wheel topic would be a second
        # publisher and the robot would track the average of the two.
        assert relay_output('true') != WHEEL_TOPIC

    def test_the_arbiter_is_the_wheel_publisher(self):
        params = resolved_node_parameters(_arbiter_launch_path(),
                                          'cmd_vel_arbiter')
        # Not overridden, so the node's default, which is asserted by
        # custom_teleop's own test to be the wheel topic.
        assert params.get('output_topic', WHEEL_TOPIC) == WHEEL_TOPIC


# ── layer 2: the Nav2 half of the chain ────────────────────────────────────
def _bringup_cmd_vel_remaps():
    """{executable-or-plugin: [remap target, ...]} for ('cmd_vel', X) remaps.

    Read from the INSTALLED nav2_bringup, because that is what nav.launch.py
    includes, and walked as an AST so both the process nodes and the
    composable ones are seen.
    """
    path = os.path.join(get_package_share_directory('nav2_bringup'),
                        'launch', 'navigation_launch.py')
    tree = ast.parse(open(path).read())
    found = {}
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call):
            continue
        name = getattr(call.func, 'id', getattr(call.func, 'attr', None))
        if name not in ('Node', 'ComposableNode'):
            continue
        keywords = {k.arg: k.value for k in call.keywords}
        who = keywords.get('executable', keywords.get('plugin'))
        if not isinstance(who, ast.Constant):
            continue
        targets = []
        for node in ast.walk(call):
            if (isinstance(node, ast.Tuple) and len(node.elts) == 2
                    and all(isinstance(e, ast.Constant) for e in node.elts)
                    and node.elts[0].value == 'cmd_vel'):
                targets.append(node.elts[1].value)
        if targets:
            found.setdefault(who.value, []).extend(targets)
    return found


def _topic(name):
    return name if name.startswith('/') else '/' + name


class TestNav2Chain:
    """Nav2's own nodes, and where the relay joins them."""

    def test_the_raw_topic_is_where_nav2_puts_its_velocity(self):
        remaps = _bringup_cmd_vel_remaps()
        assert remaps, 'no cmd_vel remaps found: the parser saw nothing'
        targets = {_topic(t) for ts in remaps.values() for t in ts}
        assert targets == {RAW_NAV_TOPIC}

    def test_the_raw_topic_is_the_smoothers_input(self):
        remaps = _bringup_cmd_vel_remaps()
        smoother = [who for who in remaps if 'velocity_smoother' in who.lower()
                    or 'VelocitySmoother' in who]
        assert smoother
        for who in smoother:
            assert [_topic(t) for t in remaps[who]] == [RAW_NAV_TOPIC]

    def test_the_controller_publishes_the_raw_topic(self):
        remaps = _bringup_cmd_vel_remaps()
        controller = [who for who in remaps if 'controller' in who.lower()]
        assert controller
        for who in controller:
            assert [_topic(t) for t in remaps[who]] == [RAW_NAV_TOPIC]

    def test_nothing_in_nav2_publishes_the_relays_or_arbiters_topics(self):
        remaps = _bringup_cmd_vel_remaps()
        targets = {_topic(t) for ts in remaps.values() for t in ts}
        assert relay_output('true') not in targets
        assert arbiter_nav_topic() not in targets

    def test_the_collision_monitor_reads_the_smoother(self):
        with open(PARAMS) as f:
            monitor = yaml.safe_load(f)['collision_monitor']['ros__parameters']
        # nav2_velocity_smoother publishes cmd_vel_smoothed.
        assert _topic(monitor['cmd_vel_in_topic']) == '/cmd_vel_smoothed'

    def test_the_relay_reads_the_collision_monitor(self):
        with open(PARAMS) as f:
            monitor = yaml.safe_load(f)['collision_monitor']['ros__parameters']
        relay_in = resolved_node_parameters(
            NAV_LAUNCH, 'cmd_vel_relay', arbiter='true').get(
                'input_topic', MONITOR_OUT_TOPIC)
        assert _topic(monitor['cmd_vel_out_topic']) == relay_in
        assert relay_in == MONITOR_OUT_TOPIC


# ── layer 3: a real graph on a private domain ──────────────────────────────
DEADLINE = 15.0
ENOUGH = 10
PUBLISH_HZ = 20.0


def _domain(offset):
    # Private, so a live stack on this machine neither sees nor feeds it.
    return 100 + (os.getpid() + offset) % 100


class Graph:
    """The relay and the arbiter as built nodes, plus a probe around them."""

    def __init__(self, tmp_path, domain, relay_out, arbiter_nav):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from geometry_msgs.msg import TwistStamped

        from custom_teleop.cmd_vel_arbiter import CmdVelArbiter
        from custom_teleop.cmd_vel_relay import CmdVelRelay

        # Node-scoped parameters, as the launch files scope them. Wall
        # clock: there is no /clock on this graph. arbiter_nav None means
        # the launch file does not set it, so the node's default applies --
        # exactly what `ros2 launch` would run.
        arbiter_params = {'use_sim_time': False, 'initial_mode': 'nav'}
        if arbiter_nav is not None:
            arbiter_params['nav_topic'] = arbiter_nav
        params = {
            'cmd_vel_relay': {'ros__parameters': {
                'use_sim_time': False, 'output_topic': relay_out}},
            'cmd_vel_arbiter': {'ros__parameters': arbiter_params},
        }
        path = tmp_path / f'wiring_{domain}.yaml'
        path.write_text(yaml.safe_dump(params))

        self.rclpy = rclpy
        self.relay_out = relay_out
        self.nodes = []
        self.executor = None
        rclpy.init(args=['--ros-args', '--params-file', str(path)],
                   domain_id=domain)
        try:
            self.relay = CmdVelRelay()
            self.nodes.append(self.relay)
            self.arbiter = CmdVelArbiter()
            self.nodes.append(self.arbiter)
            self.probe = rclpy.create_node('wiring_probe')
            self.nodes.append(self.probe)
            self.executor = SingleThreadedExecutor()
            for node in self.nodes:
                self.executor.add_node(node)
        except BaseException:
            # A half-built graph must not leave the global context
            # initialised, or every later fixture fails in rclpy.init.
            self.close()
            raise

        self.wheel, self.raw_seen, self.gated_seen = [], [], []
        self.monitor_pub = self.probe.create_publisher(
            TwistStamped, MONITOR_OUT_TOPIC, 10)
        self.raw_pub = self.probe.create_publisher(
            TwistStamped, RAW_NAV_TOPIC, 10)
        self.probe.create_subscription(
            TwistStamped, WHEEL_TOPIC,
            lambda m: self.wheel.append(m.twist.linear.x), 10)
        self.probe.create_subscription(
            TwistStamped, RAW_NAV_TOPIC,
            lambda m: self.raw_seen.append(m.twist.linear.x), 10)
        if relay_out != RAW_NAV_TOPIC:
            self.probe.create_subscription(
                TwistStamped, relay_out, self.gated_seen.append, 10)
        self._msg = TwistStamped

    def close(self):
        try:
            if self.executor is not None:
                self.executor.shutdown()
            for node in self.nodes:
                node.destroy_node()
        finally:
            if self.rclpy.ok():
                self.rclpy.shutdown()

    def spin_until(self, condition, deadline=DEADLINE, publish=None):
        # Publishing is on a 20 Hz timer, the rate of a real source. Publishing
        # once per spin_once instead floods a single-threaded executor -- each
        # spin runs ONE callback -- and starves the arbiter (measured: the
        # probe logged 8755 raw messages while the wheels logged none).
        timer = (self.probe.create_timer(1.0 / PUBLISH_HZ, publish)
                 if publish else None)
        try:
            end = time.monotonic() + deadline
            while time.monotonic() < end:
                self.executor.spin_once(timeout_sec=0.02)
                if condition():
                    return True
            return False
        finally:
            if timer is not None:
                self.probe.destroy_timer(timer)

    def names(self, topic, kind):
        infos = (self.probe.get_publishers_info_by_topic(topic)
                 if kind == 'pub' else
                 self.probe.get_subscriptions_info_by_topic(topic))
        return sorted(i.node_name for i in infos if i.node_name != 'wiring_probe')

    def send(self, raw, gated):
        def publish():
            for topic_pub, value in ((self.raw_pub, raw),
                                     (self.monitor_pub, gated)):
                msg = self._msg()   # stamp left at zero on purpose
                msg.twist.linear.x = value
                topic_pub.publish(msg)
        return publish

    def matched(self):
        return (self.monitor_pub.get_subscription_count() >= 1
                and self.raw_pub.get_subscription_count() >= 1
                and self.probe.count_publishers(WHEEL_TOPIC) >= 1)


@pytest.fixture
def fixed(tmp_path):
    """The wiring the launch files resolve to today."""
    nav = resolved_node_parameters(_arbiter_launch_path(),
                                   'cmd_vel_arbiter').get('nav_topic')
    graph = Graph(tmp_path, _domain(0), relay_output('true'), nav)
    yield graph
    graph.close()


@pytest.fixture
def looped(tmp_path):
    """The pre-C2-NAV.42 wiring, rebuilt as a positive control."""
    graph = Graph(tmp_path, _domain(1), RAW_NAV_TOPIC, RAW_NAV_TOPIC)
    yield graph
    graph.close()


def _drive(graph, stop_after=True):
    """Gated and raw together, then a STOP from the monitor with raw still on.

    Returns the index into graph.wheel where the STOP began.
    """
    assert graph.spin_until(graph.matched), 'graph never matched'
    drove = graph.spin_until(
        lambda: (graph.wheel.count(GATED) >= ENOUGH
                 and graph.raw_seen.count(RAW) >= ENOUGH),
        publish=graph.send(RAW, GATED))
    assert drove, (f'wheels saw {graph.wheel.count(GATED)} gated, probe saw '
                   f'{graph.raw_seen.count(RAW)} raw')
    stop_at = len(graph.wheel)
    if stop_after:
        graph.spin_until(
            lambda: (graph.wheel[stop_at:].count(0.0) >= ENOUGH
                     and graph.wheel[-ENOUGH:] == [0.0] * ENOUGH
                     and graph.raw_seen.count(RAW) >= 2 * ENOUGH),
            publish=graph.send(RAW, 0.0))
    return stop_at


class TestLiveGraph:
    """The resolved wiring, on a real graph."""

    def test_exactly_one_wheel_publisher_and_it_is_the_arbiter(self, fixed):
        assert fixed.spin_until(lambda: fixed.names(WHEEL_TOPIC, 'pub'))
        assert fixed.names(WHEEL_TOPIC, 'pub') == ['cmd_vel_arbiter']

    def test_the_relay_publishes_the_arbiters_input_and_not_the_raw_topic(
            self, fixed):
        assert fixed.spin_until(
            lambda: 'cmd_vel_arbiter' in fixed.names(fixed.relay_out, 'sub'))
        assert fixed.names(fixed.relay_out, 'pub') == ['cmd_vel_relay']
        assert 'cmd_vel_relay' not in fixed.names(RAW_NAV_TOPIC, 'pub')

    def test_the_arbiter_does_not_subscribe_to_the_raw_topic(self, fixed):
        # Positive control first: the arbiter's subscriptions are visible.
        assert fixed.spin_until(
            lambda: 'cmd_vel_arbiter' in fixed.names(fixed.relay_out, 'sub'))
        assert 'cmd_vel_arbiter' not in fixed.names(RAW_NAV_TOPIC, 'sub')

    def test_the_raw_controller_command_never_reaches_the_wheels(self, fixed):
        _drive(fixed)
        assert fixed.raw_seen.count(RAW) >= ENOUGH   # it was on the graph
        assert GATED in fixed.wheel                  # and the path works
        assert RAW not in fixed.wheel

    def test_a_monitor_stop_reaches_the_wheels_and_holds(self, fixed):
        stop_at = _drive(fixed)
        after = fixed.wheel[stop_at:]
        assert after.count(0.0) >= ENOUGH
        assert fixed.wheel[-ENOUGH:] == [0.0] * ENOUGH
        # Raw kept publishing throughout the stop and never got through.
        assert fixed.raw_seen.count(RAW) >= 2 * ENOUGH
        assert RAW not in after

    def test_the_relay_output_is_restamped_and_unaltered(self, fixed):
        _drive(fixed, stop_after=False)
        gated = [m for m in fixed.gated_seen if m.twist.linear.x == GATED]
        assert len(gated) >= ENOUGH
        # Sent with a zero stamp; the relay's wall clock replaced it.
        assert all(m.header.stamp.sec > 0 for m in gated)


class TestTheOldLoopIsDetected:
    """The same scenario on the pre-fix wiring must FAIL the checks above."""

    def test_the_raw_command_reached_the_wheels(self, looped):
        _drive(looped, stop_after=False)
        assert RAW in looped.wheel

    def test_the_stop_did_not_hold(self, looped):
        stop_at = _drive(looped, stop_after=False)
        looped.spin_until(
            lambda: looped.wheel[stop_at:].count(RAW) >= ENOUGH,
            publish=looped.send(RAW, 0.0))
        assert looped.wheel[stop_at:].count(RAW) >= ENOUGH
