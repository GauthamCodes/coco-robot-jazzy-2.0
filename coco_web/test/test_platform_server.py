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


def _mission_line(state):
    """One /mission/state line, in the executive's twelve-field shape."""
    return (f'state={state} prev=-- event=enter elapsed=1.0 timeout=60.0 '
            f'attempt=1 retries=0 owner=nav mode=auto reason=-- result=--')


def test_the_whole_session_lifecycle_through_the_real_server():
    """
    Startup, ready, running, degrade, recover, fail, restart, stop.

    Driven the way production drives it -- the node's observations change,
    Platform.tick() runs, and a real WebSocket client reads the telemetry
    frame; /healthz is fetched over real HTTP -- with the three answers
    read independently each time. Every lifecycle change the server made
    must be an edge of session.LIFECYCLE_EDGES, checked by Codex's
    lifecycle.validate_transition.
    """
    from coco_web import lifecycle

    async def body():
        node = FakeNode(sim=False)
        node.fresh.update(robot=False, arbiter=False, lidar=False)
        h = await Harness(node).start()
        a = await h.client()
        seen = [('welcome', a.welcome['session']['lifecycle'], None, None)]

        async def look(label):
            h.platform.tick()
            frame = await h.until(a, lambda f: f.get('type') == 'telemetry')
            status, _health = await _get(h.port, '/healthz')
            doc = frame['platform']['session']
            lifecycle.validate_axes(doc['lifecycle'], doc['health'])
            seen.append((label, doc['lifecycle'], doc['health'], status))

        await look('startup')
        node.sim = True
        node.fresh.update(sim=True, robot=True, arbiter=True, lidar=True)
        await look('ready')
        node.snap['mission'] = _mission_line('NAVIGATE_TO_RAMP')
        node.fresh['mission'] = True
        await look('running')
        node.fresh['lidar'] = False
        await look('degraded')
        node.fresh['lidar'] = True
        await look('recovered')
        node.sim = False
        node.fresh['sim'] = False
        await look('failed')
        node.sim = True
        node.fresh['sim'] = True
        await look('restarted')
        node.snap['mission'] = _mission_line('COMPLETE')
        await look('mission over')
        h.platform.session.request_stop()
        await look('stopping')
        h.platform.session.stop()
        await look('stopped')
        edges = list(h.platform.session.transitions)
        await h.stop()
        return seen, edges

    seen, edges = _run(body())
    H, D, U = (session_mod.HEALTHY, session_mod.HEALTH_DEGRADED,
               session_mod.UNHEALTHY)
    assert seen == [
        ('welcome', 'CREATED', None, None),
        ('startup', 'STARTING', U, 503),
        ('ready', 'READY', H, 200),
        ('running', 'RUNNING', H, 200),
        # health moves; the lifecycle does not
        ('degraded', 'RUNNING', D, 200),
        ('recovered', 'RUNNING', H, 200),
        # COCO's simulator gone: the one component loss that is an event
        ('failed', 'FAILED', U, 503),
        # back through STARTING (inside one tick) to READY, then RUNNING
        ('restarted', 'RUNNING', H, 200),
        ('mission over', 'READY', H, 200),
        ('stopping', 'STOPPING', U, 503),
        ('stopped', 'STOPPED', U, 503),
    ]
    walked = [(a, b) for a, b, _t in edges]
    assert walked == [
        ('CREATED', 'STARTING'), ('STARTING', 'READY'),
        ('READY', 'RUNNING'), ('RUNNING', 'FAILED'),
        ('FAILED', 'STARTING'), ('STARTING', 'READY'),
        ('READY', 'RUNNING'), ('RUNNING', 'READY'),
        ('READY', 'STOPPING'), ('STOPPING', 'STOPPED')]
    for before, after in walked:
        assert lifecycle.validate_transition(
            before, after, session_mod.LIFECYCLE_EDGES)
    # And the policy refuses what the old derivation could do.
    for bad in (('FAILED', 'READY'), ('STARTING', 'RUNNING'),
                ('STOPPED', 'READY'), ('CREATED', 'RUNNING')):
        with pytest.raises(ValueError):
            lifecycle.validate_transition(
                bad[0], bad[1], session_mod.LIFECYCLE_EDGES)


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


# ── a slow browser: bounded everywhere, and STOP is never starved ───────
# Blocker 2 of Codex's handoff. Every test here uses a real socket whose
# peer has STOPPED READING: tornado's client holds one unread message
# (Queue(1)) and then leaves the rest in the kernel, and both kernel
# buffers are shrunk so the backlog lands in tornado's write buffer --
# the memory the bound is about -- within a few frames.

def _shrink(ws, handler):
    """Make the kernel hold as little as it will between server and `ws`."""
    import socket
    ws.protocol.stream.socket.setsockopt(
        socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
    handler.ws_connection.stream.socket.setsockopt(
        socket.SOL_SOCKET, socket.SO_SNDBUF, 4096)


async def _client_and_handler(h, binary_ok=True):
    """Connect a client and return it with its server-side handler."""
    before = set(h.platform.clients)
    ws = await h.client(binary_ok)
    (handler,) = set(h.platform.clients) - before
    return ws, handler


async def _until_true(predicate, timeout=3.0):
    """Poll `predicate` on the running loop; return whether it came true."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        await asyncio.sleep(0.01)
    return predicate()


def test_buffered_bytes_sees_a_client_that_stopped_reading():
    """
    The backpressure probe must first prove it can see something.

    It read tornado's ``_write_buffer_size``, which tornado 6.5 does not
    have, so it returned 0 always -- a bound that reads "nothing buffered"
    whether or not anything is, exactly the "we saw nothing" trap
    CLAUDE.md records. Written raw here, past every bound, on purpose.
    """
    async def body():
        h = await Harness(FakeNode()).start()
        ws, handler = await _client_and_handler(h)
        _shrink(ws, handler)
        before = handler.buffered_bytes()
        for _ in range(64):
            handler.write_message(b'x' * 65536, binary=True)
        await asyncio.sleep(0.05)
        after = handler.buffered_bytes()
        await h.stop()
        return before, after
    before, after = _run(body())
    assert before == 0
    assert after > streams_mod.MAX_BUFFERED_BYTES


def test_a_slow_browser_is_bounded_and_cannot_starve_stop():
    """
    A browser that stops reading costs a bounded amount, and nobody else.

    Two hundred ticks of heavy telemetry, LiDAR and camera at it: the
    server's buffer for it stays under the sensor bound plus a frame,
    telemetry is superseded rather than queued, sensor frames are dropped
    and counted, a second client keeps receiving every tick, no tick
    blocks -- and a STOP sent BY the stalled client still reaches the
    wheels, because STOP is acted on at receipt, not at acknowledgement.
    """
    async def body():
        node = FakeNode()
        node.snap['scan'] = {'angle_min': -2.09, 'angle_step': 0.0175,
                             'ranges': [1.5] * 240, 'floor': 0.15}
        # Heavy telemetry: ~40 kB a tick, so the bound is reached quickly.
        node.snap['path'] = [[i * 0.001, 0.0] for i in range(4000)]
        h = await Harness(node).start()
        fast, fast_handler = await _client_and_handler(h)
        slow, handler = await _client_and_handler(h)
        await h.request(slow, {'type': 'subscribe', 'streams': ['camera']},
                        'subscription')
        _shrink(slow, handler)
        received = {'telemetry': 0}

        async def read_fast():
            while True:
                raw = await fast.read_message()
                if raw is None:
                    return
                if isinstance(raw, str) and '"telemetry"' in raw[:40]:
                    received['telemetry'] += 1
        reader = asyncio.ensure_future(read_fast())
        peak, longest = 0, 0.0
        jpeg = b'\xff\xd8' + bytes(30000) + b'\xff\xd9'
        for i in range(1, 201):
            node.snap['camera'] = dict(_image(i), jpeg=jpeg)
            handler.subscription._last_sent['camera'] = 0.0   # always due
            started = time.monotonic()
            h.platform.tick()
            longest = max(longest, time.monotonic() - started)
            peak = max(peak, handler.buffered_bytes())
            await asyncio.sleep(0.002)
        stops_before = h.node.published.count(('stop',))
        slow.write_message(json.dumps({'type': 'stop', 'id': 's'}))
        stopped = await _until_true(
            lambda: h.node.published.count(('stop',)) > stops_before)
        sub = handler.subscription
        result = {
            'peak': peak, 'longest': longest, 'stopped': stopped,
            'superseded': sub.superseded['telemetry'],
            'owed_max': sub.queue_depth('telemetry'),
            'camera_dropped': sub.dropped['camera'],
            'lidar_dropped': sub.dropped['lidar'],
            'abandoned': handler.abandoned,
            'fast_telemetry': received['telemetry'],
            'fast_superseded': fast_handler.subscription.superseded[
                'telemetry'],
        }
        reader.cancel()
        await h.stop()
        return result
    r = _run(body())
    # the bound: the 1 MiB sensor threshold plus at most one frame each
    assert r['peak'] < streams_mod.MAX_BUFFERED_BYTES + (256 << 10), r
    assert r['peak'] < streams_mod.MAX_CLIENT_BYTES
    # telemetry superseded, never queued: at most one in flight + one owed
    assert r['superseded'] > 100, r
    assert r['owed_max'] <= 2
    assert r['camera_dropped'] > 100 and r['lidar_dropped'] > 100, r
    # a slow-but-quiet client is bounded, not disconnected
    assert r['abandoned'] is False
    # nobody else pays: the fast client got (nearly) every tick
    assert r['fast_telemetry'] >= 190, r
    # no tick blocked on the stalled socket
    assert r['longest'] < 0.25, r
    # and STOP from the stalled client still reached the wheels
    assert r['stopped'] is True


def test_a_client_that_floods_without_reading_is_disconnected_and_stopped(
        monkeypatch):
    """
    The one hard bound: a client that only sends is cut off, and stopped.

    Every stream is bounded on its own, so only requests with replies can
    grow a client's buffer. Past MAX_CLIENT_BYTES (lowered here so the
    test is fast) the client is abandoned -- and abandoning runs the
    normal close path, so the last client leaving publishes a zero.
    """
    monkeypatch.setattr(streams_mod, 'MAX_CLIENT_BYTES', 128 << 10)

    async def body():
        h = await Harness(FakeNode()).start()
        ws, handler = await _client_and_handler(h)
        await h.request(ws, {'type': 'drive', 'linear': 0.1,
                             'angular': 0.0}, 'ack')
        _shrink(ws, handler)
        from tornado.websocket import WebSocketClosedError
        for i in range(20000):
            try:
                ws.write_message(
                    json.dumps({'type': 'ping', 't': i, 'id': 'p'}))
            except WebSocketClosedError:
                break                    # the server cut us off: the point
            if i % 500 == 0:
                await asyncio.sleep(0)
        gone = await _until_true(lambda: not h.platform.clients, 5.0)
        published = list(h.node.published)
        await h.stop()
        return gone, handler.abandoned, published
    gone, abandoned, published = _run(body())
    assert abandoned is True
    assert gone is True
    assert published[0][0] == 'drive'
    assert published[-1] == ('stop',)


# ── per-client drop counts on the wire (Codex blocker 3) ──────────────

def test_each_client_is_told_its_own_drops():
    """
    A stalled client's header counts ITS losses; a healthy one reads 0.

    The first pass sent one frame, built with dropped=0, to everyone.
    """
    class Stalled:
        """A write that has not flushed, until released."""

        def __init__(self):
            self.released = False

        def done(self):
            """Report whether the peer has taken the frame."""
            return self.released

    async def body():
        h = await Harness(FakeNode()).start()
        good, good_handler = await _client_and_handler(h)
        bad, bad_handler = await _client_and_handler(h)
        for ws in (good, bad):
            await h.request(ws, {'type': 'subscribe', 'streams': ['camera']},
                            'subscription')
        writes = []
        stall = Stalled()
        real_write = bad_handler.write_message

        def recording_write(payload, binary=False):
            if not binary:
                return real_write(payload, binary=binary)
            writes.append(payload)
            return stall
        bad_handler.write_message = recording_write
        # Yield between ticks as the 10 Hz PeriodicCallback does: tornado
        # 6.5 resolves a write as a Task, which completes only when the
        # loop runs, so back-to-back ticks would stall the GOOD client too.
        for seq in range(1, 7):
            h.node.snap['camera'] = _image(seq)
            for handler in (good_handler, bad_handler):
                handler.subscription._last_sent['camera'] = 0.0
            h.platform.tick()
            await asyncio.sleep(0.01)
        stall.released = True
        h.node.snap['camera'] = _image(7)
        bad_handler.subscription._last_sent['camera'] = 0.0
        good_handler.subscription._last_sent['camera'] = 0.0
        h.platform.tick()
        got = await h.drain(good)
        await h.stop()
        return writes, got
    writes, got = _run(body())
    bad_headers = [binary.decode_frame(w)[1] for w in writes]
    assert [hd['seq'] for hd in bad_headers] == [1, 7]
    # seq 2..6 were withheld from the stalled client: five, and it is told
    assert [hd['dropped'] for hd in bad_headers] == [0, 5]
    good_headers = [binary.decode_frame(raw)[1] for raw in got
                    if isinstance(raw, bytes)
                    and binary.decode_frame(raw)[0] == 'camera']
    assert [hd['seq'] for hd in good_headers] == [1, 2, 3, 4, 5, 6, 7]
    assert {hd['dropped'] for hd in good_headers} == {0}


def test_the_telemetry_tick_carries_this_clients_own_delivery():
    """platform.delivery is per client; perf stays the platform total."""
    async def body():
        h = await Harness(FakeNode()).start()
        a, a_handler = await _client_and_handler(h)
        a_handler.subscription.mark_dropped('camera')
        h.platform.tick()
        frame = await h.until(a, lambda f: f.get('type') == 'telemetry')
        await h.stop()
        return frame
    frame = _run(body())
    delivery = frame['platform']['delivery']
    assert delivery['camera']['dropped'] == 1
    assert 'perf' in frame['platform']


# ── disconnect cleanup (Codex blocker 4) ───────────────────────────────

def test_disconnect_releases_the_subscription_and_stops_the_robot():
    """
    Closing a socket closes its Subscription, owed frames included.

    The in-flight write handles and any owed STATE frame are released at
    close, not when the handler is garbage collected; demand for the
    camera goes to zero so the ROS image subscription closes; and the
    last client leaving still publishes a zero.
    """
    async def body():
        h = await Harness(FakeNode()).start()
        ws, handler = await _client_and_handler(h)
        await h.request(ws, {'type': 'subscribe', 'streams': ['camera']},
                        'subscription')
        await h.request(ws, {'type': 'drive', 'linear': 0.1,
                             'angular': 0.0}, 'ack')
        demand_before = set(h.node.wanted)
        handler.subscription.owe('telemetry', {'type': 'telemetry'})
        ws.close()
        closed = await _until_true(lambda: not h.platform.clients)
        sub = handler.subscription
        result = (demand_before, closed, sub._closed, dict(sub._owed),
                  dict(sub._inflight), set(h.node.wanted),
                  h.platform.demand(), list(h.node.published),
                  h.platform.session.clients)
        await h.stop()
        return result
    (demand_before, closed, is_closed, owed, inflight, wanted, demand,
     published, session_clients) = _run(body())
    assert demand_before == {'camera'}
    assert closed and is_closed
    assert owed == {} and inflight == {}
    assert wanted == set() and demand == set()
    assert published[-1] == ('stop',)
    assert not session_clients


# ── image layout at the ROS boundary (Codex blocker 5) ─────────────────

def _boundary_stub():
    """Build the slice of CocoWebNode that _encode_image touches."""
    import threading
    import types
    stub = types.SimpleNamespace(
        _image_config={}, _depth_clip=(0.1, 8.0), _lock=threading.Lock(),
        _image_seq={'camera': 0, 'depth': 0},
        _snap={'camera': None, 'depth': None},
        metrics=metrics_mod.Metrics(streams_mod.STREAMS),
        warnings=[])
    stub.get_logger = lambda: types.SimpleNamespace(
        warn=lambda msg, **_k: stub.warnings.append(msg))
    for name in ('_image_quality', '_image_scale'):
        setattr(stub, name,
                types.MethodType(getattr(ps.CocoWebNode, name), stub))
    return stub


def test_a_padded_colour_row_is_read_by_its_step_at_the_ros_boundary():
    """
    A real sensor_msgs/Image with row padding encodes the right pixels.

    Through CocoWebNode._encode_image itself, so the test covers the
    boundary (msg.step reaching imaging), not just the helper. The
    control: the same bytes with the step withheld are NOT refused --
    imaging reads the first w*h*3 bytes as packed, so padding becomes
    pixels, silently -- and they encode a different picture. That silent
    misread is exactly what passing msg.step prevents.
    """
    from coco_web import imaging
    from sensor_msgs.msg import Image
    width, height, pad = 8, 6, 8
    packed = bytes((x * 31 + y * 17 + c * 7) % 256
                   for y in range(height) for x in range(width)
                   for c in range(3))
    rows = [packed[y * width * 3:(y + 1) * width * 3] + b'\xee' * pad
            for y in range(height)]
    msg = Image(width=width, height=height, encoding='rgb8',
                step=width * 3 + pad, data=b''.join(rows))
    stub = _boundary_stub()
    ps.CocoWebNode._encode_image(stub, 'camera', msg)
    assert stub.warnings == []
    expected, _w, _h = imaging.colour_jpeg(width, height, 'rgb8', packed)
    assert stub._snap['camera']['jpeg'] == expected
    misread, _w, _h = imaging.colour_jpeg(width, height, 'rgb8',
                                          bytes(msg.data))
    assert misread != expected


def test_a_padded_big_endian_depth_row_is_read_correctly_at_the_boundary():
    """Depth byte order stays explicit: is_bigendian reaches imaging."""
    import struct
    from coco_web import imaging
    from sensor_msgs.msg import Image
    width, height, pad = 4, 3, 4
    metres = [0.5 + 0.25 * (x + y * width) for y in range(height)
              for x in range(width)]
    little = struct.pack(f'<{len(metres)}f', *metres)
    rows = [struct.pack(f'>{width}f', *metres[y * width:(y + 1) * width])
            + b'\x00' * pad for y in range(height)]
    msg = Image(width=width, height=height, encoding='32FC1',
                step=width * 4 + pad, is_bigendian=1, data=b''.join(rows))
    stub = _boundary_stub()
    ps.CocoWebNode._encode_image(stub, 'depth', msg)
    assert stub.warnings == []
    expected = imaging.depth_jpeg(width, height, '32FC1', little,
                                  clip=(0.1, 8.0))[0]
    assert stub._snap['depth']['jpeg'] == expected
    # the control: read as little-endian, the same bytes are other depths
    misread = imaging.depth_jpeg(width, height, '32FC1', bytes(msg.data),
                                 clip=(0.1, 8.0), step=width * 4 + pad,
                                 is_bigendian=False)[0]
    assert misread != expected
