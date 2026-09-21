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
The real server, end to end over real WebSockets, with no ROS graph.

Every other coco_web test proves a POLICY in a pure module --
``Subscription.wants``, ``filter_telemetry``, ``decode``. None of them
proved that ``platform_server`` actually applies that policy: that the
handler which writes to the socket consults the subscription it was
given, that two connected clients hold two subscriptions, and that the
last close publishes a stop. This file does, by running ``make_app`` --
the same ``ControlSocket``, ``Platform`` and route table the node serves
-- on a local port and connecting real tornado WebSocket clients.

Only the ROS node is faked, and it is faked at exactly the seam the
server already has: ``Platform`` reads the robot through
``node.snapshot()`` and acts through ``node.publish_*``. The fake records
what would have been published instead of publishing it.
"""

import asyncio
import json
import os
import time

from coco_web import binary
from coco_web import metrics as metrics_mod
from coco_web import platform_server as ps
from coco_web import session as session_mod
from coco_web import streams as streams_mod

import pytest

from tornado.httpclient import AsyncHTTPClient, HTTPClientError
from tornado.httpserver import HTTPServer
from tornado.testing import bind_unused_port
from tornado.websocket import websocket_connect

WEB_ROOT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'web')


class _Logger:
    """Swallow log lines; the tests assert on behaviour, not on logs."""

    def info(self, *_a, **_k):
        """Discard."""

    def warn(self, *_a, **_k):
        """Discard."""

    def error(self, *_a, **_k):
        """Discard."""


class FakeNode:
    """
    Stand-in for CocoWebNode at the seam Platform already uses.

    ``snap``/``fresh`` are what the robot "is doing"; ``published`` is
    every intent the server tried to put on the graph, in order.
    """

    def __init__(self, sim=True, expected=('lidar',)):
        """Start with a converged, drivable robot unless told otherwise."""
        self.colours = ('red', 'green', 'blue', 'yellow')
        self.arm_limits = None
        self.grip_limits = None
        self.metrics = metrics_mod.Metrics(streams_mod.STREAMS)
        self.published = []
        self.wanted = set()
        self.sim = sim
        self.expected = expected
        self.snap = {
            'pose': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0},
            'velocity': {'linear': 0.0, 'angular': 0.0},
            'scan': None, 'path': None,
            'arbiter': 'mode=teleop active=-- teleop=-- nav=--',
            'mission': '', 'perception': '', 'grasp': '', 'colour': '',
            'camera': None, 'depth': None, 'map': None, 'map_seq': 0,
        }
        self.fresh = {
            'robot': sim, 'arbiter': sim, 'mission': False,
            'perception': False, 'navigation': False, 'grasp': False,
            'colour': False, 'sim': sim, 'lidar': sim,
        }

    # the observation seam
    def snapshot(self):
        """Return the same (snap, fresh) pair the real node returns."""
        return dict(self.snap), dict(self.fresh)

    def simulator_up(self):
        """Mirror the real rule: COCO model odometry must be arriving."""
        return self.sim

    def expected_components(self):
        """Return what the launch file would have declared."""
        return tuple(self.expected)

    def camera_streams(self):
        """No MJPEG server in a unit test."""
        return {'camera': None, 'annotated': None, 'depth': None}

    def world_geometry(self):
        """No world in a unit test."""
        return None

    def want_streams(self, wanted):
        """Record which optional sensor subscriptions would be open."""
        self.wanted = set(wanted)

    def set_image_config(self, config):
        """Accept the merged encode settings."""
        self.image_config = config

    def get_logger(self):
        """Return a logger that discards."""
        return _Logger()

    # the action seam: record, never publish
    def publish_drive(self, linear, angular):
        """Record a teleop velocity."""
        self.published.append(('drive', linear, angular))

    def publish_stop(self):
        """Record an explicit zero."""
        self.published.append(('stop',))

    def publish_mode(self, mode):
        """Record a mode change."""
        self.published.append(('mode', mode))
        return True

    def publish_colour(self, colour):
        """Record a target colour."""
        self.published.append(('colour', colour))
        return True

    def publish_goal(self, x, y):
        """Record a nav goal."""
        self.published.append(('goal', x, y))
        return True

    def publish_arm(self, shoulder, elbow):
        """Arm limits are absent here, as in a stripped image."""
        return False

    def publish_gripper(self, grip):
        """Gripper limits are absent here, as in a stripped image."""
        return False

    def call_mission(self, action):
        """Record a mission service call."""
        self.published.append(('mission', action))
        return True, f'/mission/{action} called'


def _image(seq, stream='camera'):
    """Build one encoded frame the way _encode_image stores it."""
    extra = ({'min_m': 0.1, 'max_m': 8.0, 'palette': 'grey_near_bright'}
             if stream == 'depth' else None)
    return {'seq': seq, 't': time.time(), 'w': 4, 'h': 3,
            'jpeg': b'\xff\xd8JPEG' + bytes([seq % 256]) + b'\xff\xd9',
            'extra': extra, 'quality': 60}


class Harness:
    """One server on a free port, plus helpers to talk to it."""

    def __init__(self, node):
        """Build the real app around `node`; call start() to listen."""
        self.node = node
        self.platform = ps.Platform(node)
        self.app = ps.make_app(self.platform, WEB_ROOT)
        self.server = None
        self.port = None

    async def start(self):
        """Bind to an unused local port."""
        sock, self.port = bind_unused_port()
        self.server = HTTPServer(self.app)
        self.server.add_sockets([sock])
        return self

    async def stop(self):
        """Close the listener and every connection it accepted."""
        self.server.stop()
        await self.server.close_all_connections()

    async def client(self, binary_ok=True):
        """Connect, consume welcome (and map), send hello, await its ack."""
        ws = await websocket_connect(f'ws://127.0.0.1:{self.port}/ws')
        welcome = json.loads(await ws.read_message())
        assert welcome['type'] == 'welcome'
        ws.welcome = welcome
        ws.write_message(json.dumps(
            {'type': 'hello', 'id': 'h', 'binary': binary_ok}))
        await self.until(ws, lambda f: f.get('type') == 'ack')
        return ws

    @staticmethod
    async def until(ws, predicate, timeout=2.0):
        """Read JSON frames until one satisfies `predicate`; return it."""
        deadline = time.monotonic() + timeout
        while True:
            left = deadline - time.monotonic()
            assert left > 0, 'expected frame never arrived'
            raw = await asyncio.wait_for(ws.read_message(), left)
            assert raw is not None, 'socket closed'
            if isinstance(raw, bytes):
                continue
            frame = json.loads(raw)
            if predicate(frame):
                return frame

    @staticmethod
    async def drain(ws, seconds=0.3):
        """Collect everything that arrives within `seconds`."""
        got = []
        deadline = time.monotonic() + seconds
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                return got
            try:
                raw = await asyncio.wait_for(ws.read_message(), left)
            except asyncio.TimeoutError:
                return got
            if raw is None:
                return got
            got.append(raw)

    async def request(self, ws, frame, reply_type):
        """Send `frame` and return the first reply of `reply_type`."""
        ws.write_message(json.dumps(frame))
        return await self.until(ws, lambda f: f.get('type') == reply_type)


def _streams_in(frames):
    """Name the stream of every binary frame in `frames`."""
    names = []
    for raw in frames:
        if isinstance(raw, bytes):
            stream, header, _payload = binary.decode_frame(raw)
            # The id in the prefix and the name in the header must agree.
            assert header['stream'] == stream
            names.append(stream)
    return names


def _run(coro):
    """Run one async test body on a fresh event loop."""
    return asyncio.run(coro)


# ── subscriptions, per client, through the real handler ────────────────

def test_camera_reaches_only_the_client_that_subscribed_to_it():
    """Client A subscribes to camera; client B never does and gets none."""
    async def body():
        h = await Harness(FakeNode()).start()
        a = await h.client()
        b = await h.client()
        await h.request(a, {'type': 'subscribe', 'streams': ['camera']},
                        'subscription')
        h.node.snap['camera'] = _image(1)
        h.platform.tick()
        got_a = _streams_in(await h.drain(a))
        got_b = _streams_in(await h.drain(b))
        await h.stop()
        return got_a, got_b
    got_a, got_b = _run(body())
    assert 'camera' in got_a
    assert 'camera' not in got_b


def test_depth_reaches_only_the_client_that_subscribed_to_it():
    """Depth is opt-in exactly as camera is, and independently of it."""
    async def body():
        h = await Harness(FakeNode()).start()
        a = await h.client()
        b = await h.client()
        await h.request(b, {'type': 'subscribe', 'streams': ['depth']},
                        'subscription')
        h.node.snap['camera'] = _image(1)
        h.node.snap['depth'] = _image(1, 'depth')
        h.platform.tick()
        got_a = _streams_in(await h.drain(a))
        got_b = _streams_in(await h.drain(b))
        await h.stop()
        return got_a, got_b
    got_a, got_b = _run(body())
    assert 'depth' in got_b and 'camera' not in got_b
    assert 'depth' not in got_a and 'camera' not in got_a


def test_a_default_client_gets_lidar_but_neither_camera_nor_depth():
    """The default set is P0.1's: telemetry, mission, lidar, map, path."""
    async def body():
        h = await Harness(FakeNode()).start()
        a = await h.client()
        h.node.snap['scan'] = {'angle_min': -2.09, 'angle_step': 0.0175,
                               'ranges': [1.0, None, 2.5], 'floor': 0.15}
        h.node.snap['camera'] = _image(1)
        h.node.snap['depth'] = _image(1, 'depth')
        h.platform.tick()
        got = await h.drain(a)
        await h.stop()
        return a.welcome, got
    welcome, got = _run(body())
    assert welcome['subscriptions']['streams'] == sorted(
        streams_mod.DEFAULT_STREAMS)
    assert _streams_in(got) == ['lidar']


def test_unsubscribing_stops_the_stream_for_that_client_only():
    """B dropping camera must not take it from A."""
    async def body():
        h = await Harness(FakeNode()).start()
        a = await h.client()
        b = await h.client()
        for ws in (a, b):
            await h.request(ws, {'type': 'subscribe', 'streams': ['camera']},
                            'subscription')
        await h.request(b, {'type': 'unsubscribe', 'streams': ['camera']},
                        'subscription')
        h.node.snap['camera'] = _image(7)
        h.platform.tick()
        got_a = _streams_in(await h.drain(a))
        got_b = _streams_in(await h.drain(b))
        await h.stop()
        return got_a, got_b
    got_a, got_b = _run(body())
    assert 'camera' in got_a
    assert 'camera' not in got_b


def test_an_unknown_stream_is_refused_and_changes_nothing():
    """`cameras` is an error with a code, not a silent no-op."""
    async def body():
        h = await Harness(FakeNode()).start()
        a = await h.client()
        reply = await h.request(
            a, {'type': 'subscribe', 'id': 3, 'streams': ['cameras']},
            'error')
        state = await h.request(a, {'type': 'subscribe', 'streams': []},
                                'subscription')
        await h.stop()
        return reply, state
    reply, state = _run(body())
    assert reply['code'] == 'unknown_stream'
    assert reply['id'] == 3
    assert 'cameras' not in state['streams']
    assert 'camera' not in state['streams']


def test_a_text_client_subscribed_to_camera_is_never_sent_binary():
    """A client that did not declare binary gets no Blob it cannot parse."""
    async def body():
        h = await Harness(FakeNode()).start()
        a = await h.client(binary_ok=False)
        await h.request(a, {'type': 'subscribe', 'streams': ['camera']},
                        'subscription')
        h.node.snap['camera'] = _image(2)
        h.platform.tick()
        got = await h.drain(a)
        await h.stop()
        return got
    got = _run(body())
    assert not [raw for raw in got if isinstance(raw, bytes)]


def test_the_ros_camera_subscription_follows_viewer_demand():
    """Open while someone watches; closed when the last viewer leaves."""
    async def body():
        h = await Harness(FakeNode()).start()
        a = await h.client()
        await h.request(a, {'type': 'subscribe', 'streams': ['camera']},
                        'subscription')
        while_watching = set(h.node.wanted)
        a.close()
        for _ in range(50):
            if not h.platform.clients:
                break
            await asyncio.sleep(0.02)
        after = set(h.node.wanted)
        await h.stop()
        return while_watching, after
    while_watching, after = _run(body())
    assert while_watching == {'camera'}
    assert after == set()


# ── the command boundary, through the real handler ─────────────────────

def test_a_rosbridge_publish_frame_is_refused_and_publishes_nothing():
    """The literal rosbridge wheel publish must not reach the node."""
    async def body():
        h = await Harness(FakeNode()).start()
        a = await h.client()
        reply = await h.request(a, {
            'op': 'publish', 'topic': '/diff_drive_controller/cmd_vel',
            'msg': {'linear': {'x': 1.0}}}, 'error')
        await h.stop()
        return reply, list(h.node.published)
    reply, published = _run(body())
    assert reply['code'] == 'no_type'
    assert published == []


def test_a_drive_frame_cannot_name_a_topic():
    """An extra `topic` field is rejected, not ignored."""
    async def body():
        h = await Harness(FakeNode()).start()
        a = await h.client()
        reply = await h.request(a, {
            'type': 'drive', 'linear': 0.2, 'angular': 0.0,
            'topic': '/diff_drive_controller/cmd_vel'}, 'error')
        await h.stop()
        return reply, list(h.node.published)
    reply, published = _run(body())
    assert reply['code'] == 'unexpected_fields'
    assert published == []


def test_drive_is_clamped_before_it_reaches_the_node():
    """10 m/s from a browser becomes the server's limit, not 10 m/s."""
    async def body():
        h = await Harness(FakeNode()).start()
        a = await h.client()
        await h.request(a, {'type': 'drive', 'id': 1, 'linear': 10.0,
                            'angular': -10.0}, 'ack')
        await h.stop()
        return list(h.node.published)
    published = _run(body())
    assert published[-1][0] == 'drive'
    assert abs(published[-1][1]) <= 0.5 + 1e-9
    assert abs(published[-1][2]) <= 1.2 + 1e-9


def test_a_second_client_cannot_drive_but_its_stop_is_honoured():
    """One stick; STOP from anyone."""
    async def body():
        h = await Harness(FakeNode()).start()
        a = await h.client()
        b = await h.client()
        await h.request(a, {'type': 'drive', 'linear': 0.1,
                            'angular': 0.0}, 'ack')
        refused = await h.request(b, {'type': 'drive', 'linear': 0.3,
                                      'angular': 0.0}, 'error')
        await h.request(b, {'type': 'stop'}, 'ack')
        await h.stop()
        return refused, list(h.node.published)
    refused, published = _run(body())
    assert refused['code'] == 'not_in_control'
    assert ('drive', 0.3, 0.0) not in published
    assert published[-1] == ('stop',)


def test_the_last_client_leaving_stops_the_robot_and_not_before():
    """First close: no stop. Last close: an explicit zero."""
    async def body():
        h = await Harness(FakeNode()).start()
        a = await h.client()
        b = await h.client()
        await h.request(a, {'type': 'drive', 'linear': 0.1,
                            'angular': 0.0}, 'ack')

        async def closed(count):
            for _ in range(100):
                if len(h.platform.clients) == count:
                    return
                await asyncio.sleep(0.02)
        a.close()
        await closed(1)
        after_first = list(h.node.published)
        b.close()
        await closed(0)
        after_last = list(h.node.published)
        await h.stop()
        return after_first, after_last
    after_first, after_last = _run(body())
    assert ('stop',) not in after_first
    assert after_last[-1] == ('stop',)


# ── health: COCO-specific evidence, and two separate axes ──────────────

async def _get(port, path):
    """GET `path`, returning (status, json) whatever the status."""
    client = AsyncHTTPClient()
    try:
        response = await client.fetch(f'http://127.0.0.1:{port}{path}')
    except HTTPClientError as exc:
        response = exc.response
    return response.code, json.loads(response.body)


def test_a_clock_without_coco_is_not_ready_and_not_healthy():
    """
    An unrelated Gazebo publishing /clock must not make COCO look up.

    ``simulator_up`` is False because COCO's own model odometry is not
    arriving; nothing else the robot needs is there either.
    """
    async def body():
        node = FakeNode(sim=False)
        h = await Harness(node).start()
        status, health = await _get(h.port, '/healthz')
        await h.stop()
        return status, health
    status, health = _run(body())
    assert status == 503
    assert health['ready'] is False
    assert health['health'] == session_mod.UNHEALTHY
    assert 'simulator' in health['missing']


def test_healthz_is_200_and_healthy_on_a_converged_coco():
    """Every required component up, LiDAR arriving: 200 and HEALTHY."""
    async def body():
        h = await Harness(FakeNode()).start()
        status, health = await _get(h.port, '/healthz')
        await h.stop()
        return status, health
    status, health = _run(body())
    assert status == 200
    assert health['health'] == session_mod.HEALTHY
    assert health['lifecycle'] == session_mod.LIFE_READY


def test_losing_the_lidar_degrades_health_but_keeps_healthz_200():
    """DEGRADED is drivable; a restart policy must not kill it."""
    async def body():
        node = FakeNode()
        node.fresh['lidar'] = False
        h = await Harness(node).start()
        status, health = await _get(h.port, '/healthz')
        await h.stop()
        return status, health
    status, health = _run(body())
    assert status == 200
    assert health['health'] == session_mod.HEALTH_DEGRADED
    assert health['degraded_by'] == ['lidar']


def test_the_launch_declared_components_reach_the_session():
    """mission.launch.py's expected list is what health is judged by."""
    async def body():
        node = FakeNode(expected=('lidar', 'navigation', 'mission'))
        h = await Harness(node).start()
        status, health = await _get(h.port, '/healthz')
        await h.stop()
        return status, health
    status, health = _run(body())
    assert status == 200
    assert health['health'] == session_mod.HEALTH_DEGRADED
    assert health['degraded_by'] == ['mission', 'navigation']


def test_telemetry_carries_lifecycle_health_and_connection_separately():
    """Three different questions, three fields; none derived on the page."""
    async def body():
        node = FakeNode()
        h = await Harness(node).start()
        a = await h.client()
        h.platform.tick()
        frame = await h.until(a, lambda f: f.get('type') == 'telemetry')
        await h.stop()
        return a.welcome, frame
    welcome, frame = _run(body())
    for doc in (welcome['session'], frame['platform']['session']):
        assert doc['lifecycle'] in session_mod.LIFECYCLE_STATES
        assert doc['health'] in session_mod.HEALTH_STATES
        assert doc['connection'] in session_mod.CONNECTION_STATES
    assert frame['platform']['health'] == session_mod.HEALTHY
    assert frame['platform']['connection'] == session_mod.CONN_CONNECTED


def test_no_topic_name_appears_in_any_frame_a_client_receives():
    """
    The public protocol carries no ROS topic, service or command target.

    MJPEG descriptors are the one recorded exception (a web_video_server
    URL needs a topic) and the fake node advertises none, so here the
    rule is absolute: no '/'-rooted ROS name anywhere on the socket.
    """
    async def body():
        node = FakeNode()
        node.snap['mission'] = ('state=IDLE prev=-- event=enter '
                                'elapsed=0.0 timeout=-- attempt=1 '
                                'retries=0 owner=-- mode=idle reason=-- '
                                'result=--')
        node.fresh['mission'] = True
        h = await Harness(node).start()
        a = await h.client()
        await h.request(a, {'type': 'subscribe',
                            'streams': ['camera', 'depth']}, 'subscription')
        node.snap['camera'] = _image(1)
        node.snap['depth'] = _image(1, 'depth')
        h.platform.tick()
        got = await h.drain(a)
        await h.stop()
        return [a.welcome] + got
    frames = _run(body())
    for raw in frames:
        if isinstance(raw, bytes):
            _stream, header, _payload = binary.decode_frame(raw)
            text = json.dumps(header)
        else:
            text = raw if isinstance(raw, str) else json.dumps(raw)
        for needle in ('/cmd_vel', '/diff_drive', '/mission/', '/camera/',
                       '/scan', '/amcl', '/goal_pose', 'topic'):
            assert needle not in text, (needle, text[:200])


def test_a_frame_the_encoder_refuses_costs_only_its_own_stream():
    """
    A depth frame that fails validation must not take LiDAR with it.

    The encoder validates what it sends (Codex's P0.2 hardening). Every
    frame of a tick is built before any is sent, so an unguarded refusal
    would drop that tick's LiDAR too -- on every tick the bad input lasted.
    """
    async def body():
        h = await Harness(FakeNode()).start()
        a = await h.client()
        await h.request(a, {'type': 'subscribe', 'streams': ['depth']},
                        'subscription')
        h.node.snap['scan'] = {'angle_min': -2.09, 'angle_step': 0.0175,
                               'ranges': [1.0, 2.0], 'floor': 0.15}
        bad = _image(1, 'depth')
        bad['extra'] = {'min_m': 5.0, 'max_m': 1.0,        # low > high
                        'palette': 'grey_near_bright'}
        h.node.snap['depth'] = bad
        h.platform.tick()
        got = _streams_in(await h.drain(a))
        await h.stop()
        return got
    got = _run(body())
    assert 'lidar' in got
    assert 'depth' not in got


def test_the_derived_lifecycle_only_ever_takes_legal_edges():
    """
    Walk a whole session and check every lifecycle change it makes.

    session.py DERIVES the lifecycle, so it has no transition table to
    get wrong -- but a derivation can still jump somewhere it should not
    (CREATED straight to RUNNING, FAILED back to READY). lifecycle.py
    (Codex) validates a change against an owner-supplied policy; this is
    that policy, and the walk covers bring-up, a mission, a component
    lost, recovery refused, and shutdown.
    """
    from coco_web import lifecycle
    edges = {
        # bring-up
        ('CREATED', 'STARTING'), ('STARTING', 'READY'),
        # missions
        ('READY', 'RUNNING'), ('RUNNING', 'READY'),
        # a required component lost after convergence...
        ('READY', 'FAILED'), ('RUNNING', 'FAILED'),
        # ...and RECOVERY, deliberately legal: a component whose status
        # went stale for 3 s under load and came back is not a reason to
        # force a restart. FAILED is recoverable; STOPPED is terminal.
        ('FAILED', 'READY'), ('FAILED', 'RUNNING'),
        # shutdown, from any live state
        ('CREATED', 'STOPPING'), ('STARTING', 'STOPPING'),
        ('READY', 'STOPPING'), ('RUNNING', 'STOPPING'),
        ('FAILED', 'STOPPING'),
        ('STOPPING', 'STOPPED'), ('STOPPING', 'FAILED'),
    }
    s = session_mod.SimulationSession()
    seen = [s.lifecycle()]

    def step(action):
        action()
        now = s.lifecycle()
        if now != seen[-1]:
            assert lifecycle.validate_transition(seen[-1], now, edges)
            seen.append(now)
        lifecycle.validate_axes(now, s.health_state())

    for name in session_mod.REQUIRED:
        step(lambda n=name: s.observe(n, True))
    step(lambda: s.observe(session_mod.LIDAR, True))
    step(lambda: setattr(s, 'mission_running', True))
    step(lambda: setattr(s, 'mission_running', False))
    step(lambda: s.observe(session_mod.ARBITER, False))    # lost
    step(lambda: s.observe(session_mod.ARBITER, True))     # back
    step(lambda: setattr(s, 'stopping', True))
    step(lambda: s.stop())
    assert seen == ['CREATED', 'STARTING', 'READY', 'RUNNING', 'READY',
                    'FAILED', 'READY', 'STOPPING', 'STOPPED']
    # And the policy rejects what the derivation must never do.
    for bad in (('CREATED', 'RUNNING'), ('STOPPED', 'READY'),
                ('STARTING', 'FAILED')):
        with pytest.raises(ValueError):
            lifecycle.validate_transition(bad[0], bad[1], edges)


@pytest.mark.parametrize('stream', ['camera', 'depth'])
def test_a_client_that_stays_behind_has_frames_dropped_not_queued(stream):
    """
    A frame still in flight means the next one is dropped and counted.

    Driven through ControlSocket.send_binary itself with a write future
    that never completes -- the stalled socket -- so the bound is the
    handler's, not a restatement of Subscription's.
    """
    class Stalled:

        def done(self):
            """Never flushes: the peer is not reading."""
            return False

    sock = ps.ControlSocket.__new__(ps.ControlSocket)
    sock.subscription = streams_mod.Subscription(binary=True)
    sock.subscription.subscribe([stream])
    sock.subscription.configure(stream, fps=15)
    written = []

    def write_message(blob, binary=False):
        written.append(blob)
        return Stalled()
    sock.write_message = write_message
    sock.buffered_bytes = lambda: 0

    first = sock.send_binary(stream, b'one')
    sock.subscription._last_sent[stream] = 0.0      # due again at once
    second = sock.send_binary(stream, b'two')
    assert first is True and second is False
    assert written == [b'one']
    assert sock.subscription.dropped[stream] == 1
