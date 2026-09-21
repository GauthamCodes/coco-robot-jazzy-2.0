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


# ── the lifecycle axis (P0.2) ──────────────────────────────────────────
# `state` is readiness and drives /healthz; `lifecycle` is where the
# session is in its life. They are separate on purpose -- see the note
# beside LIFECYCLE_STATES.

def _ready_session():
    """Build a session with every required component reporting up."""
    session = S.SimulationSession()
    for name in S.REQUIRED:
        session.observe(name, True)
    return session


def test_a_brand_new_session_is_created():
    """Nothing observed at all, not even rclpy."""
    assert S.SimulationSession().lifecycle() == S.LIFE_CREATED


def test_a_partly_started_session_is_starting():
    """The rclpy context is up but the simulator is not yet."""
    session = S.SimulationSession()
    session.observe(S.ROS, True)
    assert session.lifecycle() == S.LIFE_STARTING


def test_a_converged_idle_session_is_ready():
    """Drivable, with no mission running."""
    assert _ready_session().lifecycle() == S.LIFE_READY


def test_a_session_running_a_mission_is_running():
    """The second axis is what distinguishes this from plain READY."""
    session = _ready_session()
    session.mission_running = True
    assert session.lifecycle() == S.LIFE_RUNNING


def test_running_a_mission_does_not_change_readiness():
    """
    The reason the two axes are not merged.

    /healthz returns 200 only for `ready`. If starting a mission moved
    the session out of `ready`, Docker's HEALTHCHECK would mark the
    container unhealthy for the whole fetch -- and a restart policy
    would then kill the robot mid-climb.
    """
    session = _ready_session()
    session.mission_running = True
    assert session.state == S.READY
    _body, status = session.health()
    assert status == 200


def test_losing_a_component_after_convergence_is_a_failure():
    """
    A stack that came up and fell over is not 'still starting'.

    Showing those identically sends someone debugging a simulator that
    is merely still booting.
    """
    session = _ready_session()
    session.observe(S.SIMULATOR, False)
    assert session.lifecycle() == S.LIFE_FAILED


def test_never_having_had_a_component_is_still_starting():
    """The same partial state, without the history, means something else."""
    session = S.SimulationSession()
    session.observe(S.ROS, True)
    session.observe(S.ROBOT, True)
    assert session.converged is False
    assert session.lifecycle() == S.LIFE_STARTING


def test_stopping_is_reported_before_stopped():
    """Shutdown requested is distinguishable from shutdown complete."""
    session = _ready_session()
    session.stopping = True
    assert session.lifecycle() == S.LIFE_STOPPING


def test_a_stopped_session_is_stopped_and_a_failed_one_is_failed():
    """Terminal states split on whether a reason was recorded."""
    clean = _ready_session()
    clean.stop()
    assert clean.lifecycle() == S.LIFE_STOPPED
    broken = _ready_session()
    broken.fail('simulator exited')
    broken.stop()
    assert broken.lifecycle() == S.LIFE_FAILED


def test_every_lifecycle_value_is_in_the_documented_set():
    """A state the wire contract does not list must not reach a client."""
    for session in (S.SimulationSession(), _ready_session()):
        assert session.lifecycle() in S.LIFECYCLE_STATES


# ── the connection axis ────────────────────────────────────────────────

def test_connection_reports_the_simulator_coming_up():
    """Reporting connected while Gazebo spawns would be a lie."""
    session = S.SimulationSession()
    session.observe(S.ROS, True)
    assert session.connection() == S.CONN_SIM_STARTING


def test_connection_distinguishes_simulator_up_from_robot_ready():
    """
    Simulator-up-robot-not-yet is a real and reportable state.

    /model/coco/odometry appears well before ros2_control finishes
    activating, which is exactly this window.
    """
    session = S.SimulationSession()
    session.observe(S.ROS, True)
    session.observe(S.SIMULATOR, True)
    assert session.connection() == S.CONN_SIM_READY


def test_connection_is_connected_when_everything_is_up():
    """The ordinary case."""
    assert _ready_session().connection() == S.CONN_CONNECTED


def test_connection_reports_a_running_mission():
    """The UI shows what the robot is busy with, not just that it is up."""
    session = _ready_session()
    session.mission_running = True
    assert session.connection() == S.CONN_MISSION_RUNNING


def test_connection_reports_an_error_after_a_component_is_lost():
    """A stack that fell over must not read as merely starting."""
    session = _ready_session()
    session.observe(S.ARBITER, False)
    assert session.connection() == S.CONN_ERROR


def test_the_server_never_claims_connecting_or_disconnected():
    """
    Those are socket facts, and only the client can observe them.

    A server reporting DISCONNECTED would be doing so down a connection.
    """
    assert 'CONNECTING' not in S.CONNECTION_STATES
    assert 'DISCONNECTED' not in S.CONNECTION_STATES


def test_both_axes_ride_the_session_document():
    """A client reads them together, so both must be present."""
    document = _ready_session().as_dict()
    assert document['state'] == S.READY
    assert document['lifecycle'] == S.LIFE_READY
    assert document['connection'] == S.CONN_CONNECTED


# ── the health axis (P0.2, second pass) ────────────────────────────────
# HEALTHY / DEGRADED / UNHEALTHY is a separate axis from the lifecycle.
# These tests pin both halves of that: the values, and that neither axis
# can be computed from the other.

def test_health_values_are_exactly_the_documented_three():
    """The product vocabulary is locked; nothing else may appear."""
    assert S.HEALTH_STATES == ('HEALTHY', 'DEGRADED', 'UNHEALTHY')


def test_health_and_lifecycle_are_different_enums():
    """Two axes, not one renamed. No value is shared between them."""
    assert not set(S.HEALTH_STATES) & set(S.LIFECYCLE_STATES)


def test_a_new_session_is_unhealthy_and_created():
    """Nothing is up yet; the lifecycle says why, health says it is so."""
    session = S.SimulationSession()
    assert session.health_state() == S.UNHEALTHY
    assert session.lifecycle() == S.LIFE_CREATED


def test_required_up_and_expected_up_is_healthy():
    """Everything the session should have, it has."""
    session = _bring_up(S.SimulationSession(), S.REQUIRED + (S.LIDAR,))
    assert session.health_state() == S.HEALTHY


def test_required_up_but_no_lidar_is_degraded_not_unhealthy():
    """
    The robot is drivable, so /healthz stays 200 -- but it is not healthy.

    The simulated robot always has a LiDAR; one that is silent leaves the
    collision monitor blind, which an operator needs to see.
    """
    session = _bring_up(S.SimulationSession())
    assert session.state == S.READY
    assert session.health_state() == S.HEALTH_DEGRADED
    assert session.degraded_by() == [S.LIDAR]
    assert session.health()[1] == 200


def test_ready_and_degraded_is_the_combination_one_enum_cannot_say():
    """Lifecycle READY and health DEGRADED together, which is the point."""
    session = S.SimulationSession(expected=(S.LIDAR, S.NAVIGATION))
    _bring_up(session, S.REQUIRED + (S.LIDAR,))
    assert session.lifecycle() == S.LIFE_READY
    assert session.health_state() == S.HEALTH_DEGRADED
    assert session.degraded_by() == [S.NAVIGATION]


def test_running_and_healthy_is_a_fetch_with_nothing_wrong():
    """A mission running is a lifecycle fact, never a health problem."""
    session = _bring_up(S.SimulationSession(), S.REQUIRED + (S.LIDAR,))
    session.mission_running = True
    assert session.lifecycle() == S.LIFE_RUNNING
    assert session.health_state() == S.HEALTHY


def test_a_component_that_was_up_and_died_degrades_even_if_undeclared():
    """
    Nobody declared navigation expected, but it was running and stopped.

    That is a loss, and reporting HEALTHY over it would be the lie.
    """
    session = _bring_up(S.SimulationSession(), S.REQUIRED + (S.LIDAR,))
    session.observe(S.NAVIGATION, True)
    assert session.health_state() == S.HEALTHY
    session.observe(S.NAVIGATION, False)
    assert session.health_state() == S.HEALTH_DEGRADED
    assert session.degraded_by() == [S.NAVIGATION]


def test_an_undeclared_component_never_seen_does_not_degrade():
    """A manual-driving appliance with no Nav2 is healthy, not degraded."""
    session = _bring_up(S.SimulationSession(), S.REQUIRED + (S.LIDAR,))
    assert not session.components[S.NAVIGATION].up
    assert session.health_state() == S.HEALTHY


def test_a_missing_required_component_is_unhealthy_whatever_else_is_up():
    """Required means required: nothing optional can make up for it."""
    session = _bring_up(S.SimulationSession(), S.ALL_COMPONENTS)
    session.observe(S.ARBITER, False)
    assert session.health_state() == S.UNHEALTHY


def test_stopped_and_failed_sessions_are_unhealthy():
    """A session that cannot serve is not healthy, however it ended."""
    stopped = _bring_up(S.SimulationSession(), S.ALL_COMPONENTS)
    stopped.stop()
    assert stopped.health_state() == S.UNHEALTHY
    failed = _bring_up(S.SimulationSession(), S.ALL_COMPONENTS)
    failed.fail('gz crashed')
    assert failed.health_state() == S.UNHEALTHY


def test_healthz_is_200_exactly_when_health_is_not_unhealthy():
    """
    The status code did not change meaning when the axis was added.

    Checked over every combination of the components, not a handful: the
    equivalence is the contract Docker's HEALTHCHECK relies on.
    """
    import itertools
    names = S.ALL_COMPONENTS
    for mask in itertools.product((False, True), repeat=len(names)):
        session = S.SimulationSession()
        for name, up in zip(names, mask):
            session.observe(name, up)
        status = session.health()[1]
        assert (status == 200) == (session.health_state() != S.UNHEALTHY)


def test_health_rides_the_session_document_and_healthz():
    """Both documents carry it, and the old fields keep their meaning."""
    session = _bring_up(S.SimulationSession())
    document = session.as_dict()
    assert document['health'] == S.HEALTH_DEGRADED
    assert document['degraded_by'] == [S.LIDAR]
    assert document['expected'] == [S.LIDAR]
    assert document['state'] == S.READY          # coco.v1, unchanged
    body, status = session.health()
    assert body['health'] == S.HEALTH_DEGRADED
    assert body['status'] == S.READY
    assert status == 200


def test_parse_expected_accepts_commas_and_spaces_and_drops_required():
    """One launch-file string; required names are not optional twice."""
    assert S.parse_expected('lidar, navigation mission') == (
        S.LIDAR, S.NAVIGATION, S.MISSION)
    assert S.parse_expected('robot,lidar,lidar') == (S.LIDAR,)
    assert S.parse_expected('') == ()
    assert S.parse_expected(None) == ()


def test_the_simulator_component_is_not_described_as_the_clock():
    """
    /clock alone is not evidence that COCO is running.

    An unrelated Gazebo publishes one on the same graph; that was measured
    on the development machine. The module must not document the weaker
    rule it deliberately does not implement.
    """
    import inspect
    source = inspect.getsource(S)
    assert 'Gazebo publishing the clock' not in source
