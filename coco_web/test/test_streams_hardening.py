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

"""Subscription helpers validate atomically and release bounded state."""

from concurrent.futures import Future

from coco_web import streams as st

import pytest


@pytest.mark.parametrize('names', [['camera', 'unknown'], ['/scan'],
                                   ['depth', []], 'camera', None])
@pytest.mark.parametrize('operation', ['subscribe', 'unsubscribe'])
def test_invalid_update_is_atomic(names, operation):
    """An invalid request must leave the client completely unchanged."""
    sub = st.Subscription(binary=True)
    before = sub.as_dict()
    with pytest.raises(ValueError):
        getattr(sub, operation)(names)
    assert sub.as_dict() == before


@pytest.mark.parametrize('names', [['unknown'], ['camera', {}], 'camera'])
def test_constructor_validates_names(names):
    """The pure class has the same vocabulary as the wire boundary."""
    with pytest.raises(ValueError):
        st.Subscription(streams=names)


def test_duplicate_membership_and_client_isolation():
    """Repeated requests are idempotent and only affect their own client."""
    left, right = st.Subscription(binary=True), st.Subscription(binary=True)
    left.subscribe(['camera', 'camera'])
    assert left.should_send('camera')
    assert not right.should_send('camera')
    left.unsubscribe(['camera', 'camera'])
    left.unsubscribe(['camera'])
    assert not left.should_send('camera')


def test_unknown_configuration_cannot_grow_state():
    """Arbitrary stream keys cannot allocate an unbounded config map."""
    sub = st.Subscription()
    before = sub.as_dict()
    with pytest.raises(ValueError):
        sub.configure('/camera/raw', fps=5)
    assert sub.as_dict() == before


def test_disconnect_is_terminal_and_releases_references():
    """A retained handler cannot deliver frames after close."""
    sub = st.Subscription(binary=True)
    pending = Future()
    sub.subscribe(['camera'])
    sub.mark_sent('camera', pending)
    sub.close()
    sub.close()
    assert not sub.should_send('telemetry')
    assert not sub.should_send('camera')
    assert sub.queue_depth('camera') == 0
    assert not pending.cancelled()
    with pytest.raises(ValueError):
        sub.subscribe(['camera'])


def test_slow_client_has_one_pending_write_per_stream():
    """Ten thousand candidate frames cannot add more retained futures."""
    slow, fast = st.Subscription(binary=True), st.Subscription(binary=True)
    pending = Future()
    slow.mark_sent('camera', pending, now=1)
    for _ in range(10000):
        assert not slow.ready('camera')
        slow.mark_dropped('camera')
    assert slow.queue_depth('camera') == 1
    assert fast.ready('camera')
    view = slow.as_dict()
    assert view['queue_depth']['camera'] == 1
    assert view['send_attempts']['camera'] == 10001
    assert view['dropped']['camera'] == 10000
    pending.set_result(None)
    assert slow.queue_depth('camera') == 0
    assert slow.ready('camera')


def test_unsubscribe_resubscribe_does_not_bypass_inflight_bound():
    """A stream toggle must not forget a write already owned by Tornado."""
    sub = st.Subscription(binary=True)
    sub.subscribe(['camera'])
    sub.mark_sent('camera', Future())
    sub.unsubscribe(['camera'])
    sub.subscribe(['camera'])
    assert not sub.ready('camera')


def test_exact_buffer_limit_refuses_a_new_frame():
    """A full transport buffer has no space for another sensor frame."""
    assert not st.Subscription().ready('lidar', st.MAX_BUFFERED_BYTES)
