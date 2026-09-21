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

"""Exercise real localhost WebSockets with the ROS side replaced by a fake."""

import asyncio
import json
from unittest.mock import Mock

from coco_web import binary, metrics, protocol
from coco_web.platform_server import make_app, Platform

from tornado.testing import AsyncHTTPTestCase, gen_test
from tornado.websocket import websocket_connect


class TestTransport(AsyncHTTPTestCase):
    """Use the production handler and platform without publishing ROS commands."""

    def get_app(self):
        """Build an actual Tornado app around a recording node fixture."""
        self.node = Mock()
        self.node.colours = protocol.FALLBACK_COLOURS
        self.node.arm_limits = {}
        self.node.grip_limits = None
        self.node.metrics = metrics.Metrics()
        self.node.camera_streams.return_value = {}
        self.node.world_geometry.return_value = {}
        self.node.snapshot.return_value = ({}, {})
        self.platform = Platform(self.node)
        self.sockets = []
        return make_app(self.platform, '.')

    async def connect_client(self):
        """Consume welcome before returning a connected client."""
        socket = await websocket_connect(
            self.get_url('/ws').replace('http:', 'ws:'))
        self.sockets.append(socket)
        welcome = json.loads(await socket.read_message())
        assert welcome['type'] == 'welcome'
        assert welcome['protocol'] == 'coco.v1'
        return socket

    async def request(self, socket, body):
        """Read responses through the correlated ack or error."""
        body = dict(body, id='request')
        await socket.write_message(json.dumps(body))
        replies = []
        while True:
            reply = json.loads(await socket.read_message())
            replies.append(reply)
            if reply['type'] in ('ack', 'error'):
                return replies

    async def barrier(self, socket):
        """Round-trip ping to prove the server processed earlier writes."""
        replies = await self.request(socket, {'type': 'ping', 't': 123})
        assert replies[0] == {'type': 'pong', 't': 123}

    async def wait_clients(self, count):
        """Wait boundedly for the asynchronous close callback."""
        for _ in range(100):
            if len(self.platform.clients) == count:
                return
            await asyncio.sleep(0.005)
        raise AssertionError('close callback did not complete')

    def tearDown(self):
        """Close only sockets this fixture created."""
        for socket in self.sockets:
            socket.close()
        self.io_loop.run_sync(lambda: self.wait_clients(0))
        super().tearDown()

    @gen_test
    async def test_closed_vocabulary_and_recovery(self):
        """A malformed request gets an error; the same socket remains usable."""
        socket = await self.connect_client()
        for frame in ({'type': 'publish', 'topic': '/wheel/command'},
                      {'type': 'drive', 'target': '/wheel/command'},
                      {'type': 'subscribe', 'streams': ['/scan']}):
            replies = await self.request(socket, frame)
            assert replies[-1]['type'] == 'error'
        await socket.write_message('{"type":"ping","t":NaN}')
        assert json.loads(await socket.read_message())['type'] == 'error'
        await self.barrier(socket)
        self.node.publish_drive.assert_not_called()

    @gen_test
    async def test_opt_in_isolation_unsubscribe_and_binary_roundtrip(self):
        """Only the subscribed client receives bytes, for both image streams."""
        left, right = await self.connect_client(), await self.connect_client()
        await self.request(left, {'type': 'hello', 'binary': True})
        left_handler = next(c for c in self.platform.clients
                            if c.subscription.binary)
        right_handler = next(c for c in self.platform.clients
                             if not c.subscription.binary)
        for stream in ('camera', 'depth'):
            extra = {'min_m': 0.1, 'max_m': 8} if stream == 'depth' else None
            blob = binary.image_frame(stream, 1, 2, 8, 8, b'jpeg', extra=extra)
            assert not left_handler.send_binary(stream, blob)
            await self.request(left, {'type': 'subscribe', 'streams': [stream]})
            assert left_handler.send_binary(stream, blob)
            received = await left.read_message()
            assert binary.decode_frame(received)[0] == stream
            assert not right_handler.send_binary(stream, blob)
            await self.request(left, {'type': 'unsubscribe',
                                      'streams': [stream]})
            assert not left_handler.send_binary(stream, blob)
        # An unexpected binary frame would arrive before this pong and fail.
        await self.barrier(left)
        await self.barrier(right)

    @gen_test
    async def test_stop_and_last_disconnect(self):
        """STOP from a watcher and last-client disconnect use the existing path."""
        driver, watcher = await self.connect_client(), await self.connect_client()
        await self.request(driver, {'type': 'drive', 'linear': 0.1})
        self.node.publish_drive.assert_called_once()
        replies = await self.request(watcher, {'type': 'stop'})
        assert replies[-1]['type'] == 'ack'
        self.node.publish_stop.assert_called_once()
        driver.close()
        await self.wait_clients(1)
        assert self.node.publish_stop.call_count == 1
        watcher.close()
        await self.wait_clients(0)
        assert self.node.publish_stop.call_count == 2
        assert not self.platform.session.clients
        assert self.platform.demand() == set()

    @gen_test
    async def test_slow_write_is_bounded_without_blocking_other_clients(self):
        """A pending socket write is never replaced by more pending writes."""
        client = await self.connect_client()
        await self.request(client, {'type': 'hello', 'binary': True})
        handler = next(iter(self.platform.clients))
        pending = asyncio.get_running_loop().create_future()
        handler.write_message = Mock(return_value=pending)
        blob = binary.lidar_frame(1, 2, {'ranges': [1.0] * 480})
        assert handler.send_binary('lidar', blob)
        for _ in range(1000):
            assert not handler.send_binary('lidar', blob)
        assert handler.write_message.call_count == 1
        assert handler.subscription.queue_depth('lidar') == 1
        assert handler.subscription.dropped['lidar'] == 1000
        pending.set_result(None)
        assert handler.send_binary('lidar', blob)
