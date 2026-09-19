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

"""The simulation-session model: lifecycle, readiness and health."""

from coco_web import session as S

import pytest


def _bring_up(session, components=S.REQUIRED):
    """Mark `components` up, as the telemetry loop would."""
    for name in components:
        session.observe(name, True, 'test')
    return session


# ── lifecycle ──────────────────────────────────────────────────────────

def test_a_new_session_is_starting():
    """Before anything reports, the session is starting, not degraded."""
    assert S.SimulationSession().state == S.STARTING


def test_a_new_session_has_every_component():
    """Every known component exists from birth, so the UI never sees a gap."""
    session = S.SimulationSession()
    assert set(session.components) == set(S.ALL_COMPONENTS)
    assert all(not c.up for c in session.components.values())


def test_partial_bringup_is_degraded_not_starting():
    """Some-but-not-all is the case an operator most needs to see."""
    session = S.SimulationSession()
    session.observe(S.ROS, True)
    assert session.state == S.DEGRADED
    assert S.SIMULATOR in session.missing_required()


def test_all_required_up_is_ready():
    """READY means every required component reported up."""
    assert _bring_up(S.SimulationSession()).state == S.READY


def test_optional_components_do_not_block_ready():
    """
    Manual driving works without Nav2, perception or the mission.

    The appliance must be usable before the whole mission stack is up, or
    the first thing a new user sees is a permanently red light.
    """
    session = _bring_up(S.SimulationSession())
    assert session.state == S.READY
    assert not session.components[S.NAVIGATION].up
    assert not session.components[S.MISSION].up


def test_a_component_going_down_leaves_ready():
    """Readiness is re-derived on every observation, not latched."""
    session = _bring_up(S.SimulationSession())
    session.observe(S.SIMULATOR, False, 'gone')
    assert session.state == S.DEGRADED


def test_stop_is_terminal():
    """A stopped session does not come back when components report again."""
    session = _bring_up(S.SimulationSession())
    session.stop()
    assert session.state == S.STOPPED
    session.observe(S.ROS, True)
    assert session.state == S.STOPPED


def test_stop_drops_clients():
    """Stopping releases the client set."""
    session = S.SimulationSession()
    session.attach('a')
    session.stop()
    assert session.clients == set()


def test_unknown_components_are_tolerated():
    """A future launch file reporting something new must not crash telemetry."""
    session = _bring_up(S.SimulationSession())
    session.observe('weather_station', True, 'hello')
    assert session.state == S.READY
    assert session.components['weather_station'].up


# ── clients ────────────────────────────────────────────────────────────

def test_attach_and_detach_are_idempotent():
    """Double attach or detach must not miscount clients."""
    session = S.SimulationSession()
    assert session.attach('a') == 1
    assert session.attach('a') == 1
    assert session.attach('b') == 2
    assert session.detach('a') == 1
    assert session.detach('a') == 1
    assert session.detach('b') == 0


# ── health ─────────────────────────────────────────────────────────────

def test_health_is_503_until_ready():
    """
    TASK 10: the platform must not report ready while Gazebo is booting.

    A health check that goes green early is worse than none, because
    orchestration then routes at a simulator that cannot answer.
    """
    session = S.SimulationSession()
    body, status = session.health()
    assert status == 503
    assert body['ready'] is False
    assert S.SIMULATOR in body['missing']


def test_health_is_200_when_ready():
    """Green only once every required component is up."""
    body, status = _bring_up(S.SimulationSession()).health()
    assert status == 200
    assert body['ready'] is True
    assert body['missing'] == []


def test_health_distinguishes_starting_from_degraded():
    """Both answer 503, but the body says which, which is what differs."""
    session = S.SimulationSession()
    assert session.health()[0]['status'] == S.STARTING
    session.observe(S.ROS, True)
    assert session.health()[0]['status'] == S.DEGRADED


def test_health_names_what_is_missing():
    """An operator must be able to act on the body alone."""
    session = _bring_up(S.SimulationSession())
    session.observe(S.ARBITER, False)
    body, status = session.health()
    assert status == 503
    assert body['missing'] == [S.ARBITER]


def test_session_dict_is_json_safe():
    """The session document must survive json.dumps unmodified."""
    import json
    session = _bring_up(S.SimulationSession())
    session.attach('a')
    parsed = json.loads(json.dumps(session.as_dict()))
    assert parsed['state'] == S.READY
    assert parsed['clients'] == 1
    assert parsed['components'][S.ROS]['up'] is True


# ── registry ───────────────────────────────────────────────────────────

def test_sole_creates_once_and_returns_the_same_session():
    """The server's convenience accessor is stable across calls."""
    registry = S.SessionRegistry()
    first = registry.sole()
    assert registry.sole() is first
    assert len(registry.all_sessions()) == 1


def test_the_single_session_cap_is_enforced_not_assumed():
    """
    P0.1 serves one session, and says so rather than sharing a robot.

    Two clients sharing one simulation is the correct P0.1 behaviour; two
    *sessions* would mean two clients silently driving the same wheels.
    """
    registry = S.SessionRegistry()
    registry.sole()
    with pytest.raises(RuntimeError) as caught:
        registry.create()
    assert 'isolation' in str(caught.value)


def test_drop_stops_and_removes():
    """Dropping a session stops it and frees the slot."""
    registry = S.SessionRegistry()
    session = registry.sole()
    assert registry.drop(session.id) is True
    assert registry.all_sessions() == []
    assert session.state == S.STOPPED
    assert registry.drop(session.id) is False


def test_get_by_id():
    """Sessions are addressed by id, so no call site needs a global."""
    registry = S.SessionRegistry()
    session = registry.sole()
    assert registry.get(session.id) is session
    assert registry.get('nope') is None
