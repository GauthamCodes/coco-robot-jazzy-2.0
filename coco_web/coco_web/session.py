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
The simulation-session abstraction, and the readiness rules it enforces.

Pure module: no rclpy, no sockets. Readiness is a function of observed
facts, so "is the platform up?" is decided by the same code the tests
exercise.

Scope, deliberately
-------------------
P0.1 is SINGLE USER / SINGLE SESSION. ``SessionRegistry`` therefore holds
exactly one session and says so. It exists as a registry anyway because
the thing that makes multi-session hard later is not the container
scheduler -- it is code that assumes the session is a global. Every caller
here already asks a registry for a session by id, so growing to

    user A -> COCO session A
    user B -> COCO session B

is a change of *policy* inside this one class, not a rewrite of the
server. The scheduler itself is explicitly out of scope for P0.1.

Why readiness is not a boolean
------------------------------
A ROS + Gazebo stack does not start, it *converges*: rclpy is up in
milliseconds, the clock a second later, the controllers several seconds
after that, Nav2's lifecycle later still. Reporting "ready" the moment the
web server binds its port is how a developer ends up pressing Start
against a simulator that has not spawned the robot yet, then filing the
resulting timeout as a mission bug. So a session carries one
``ComponentState`` per subsystem, and is READY only when the components
that are *required* are all up. Everything else is reported, not waited
for.
"""

from dataclasses import dataclass, field
import time
import uuid

# ── session lifecycle ──────────────────────────────────────────────────
#: Nothing observed yet; the server has just started.
STARTING = 'starting'
#: Some required component is not up. The UI shows what is missing.
DEGRADED = 'degraded'
#: Every required component is up. Safe to drive and to start a mission.
READY = 'ready'
#: The session has been shut down and will not become ready again.
STOPPED = 'stopped'

SESSION_STATES = (STARTING, DEGRADED, READY, STOPPED)

# ── component names ────────────────────────────────────────────────────
# Each is a thing that can independently be up or down, and each is
# observed from the ROS graph rather than assumed from a launch file
# having been executed -- "we started it" is not "it is running".
ROS = 'ros'                    # rclpy context alive, node spinning
SIMULATOR = 'simulator'        # Gazebo publishing the clock
ROBOT = 'robot'                # controllers publishing odometry
ARBITER = 'arbiter'            # cmd_vel_arbiter publishing its status
MISSION = 'mission'            # mission executive publishing state
PERCEPTION = 'perception'      # target finder publishing status
NAVIGATION = 'navigation'      # Nav2 publishing a costmap or plan

#: Components that must be up before a session is READY.
#:
#: Navigation, perception and the mission executive are deliberately NOT
#: required: the appliance is useful for manual driving without them, and
#: `mission.launch.py` starts them separately. They are reported so the UI
#: can grey out the mission panel instead of lying about it.
REQUIRED = (ROS, SIMULATOR, ROBOT, ARBITER)

ALL_COMPONENTS = (ROS, SIMULATOR, ROBOT, ARBITER, MISSION, PERCEPTION,
                  NAVIGATION)


@dataclass
class ComponentState:
    """
    One subsystem's health, as last observed.

    ``last_seen`` is a monotonic timestamp rather than wall clock so a
    simulator whose /clock jumps -- which it does on every restart -- does
    not make a component look alive for hours.
    """

    name: str
    up: bool = False
    detail: str = ''
    last_seen: float = 0.0

    def as_dict(self):
        """JSON-safe view for the telemetry frame."""
        return {'name': self.name, 'up': self.up, 'detail': self.detail}


@dataclass
class SimulationSession:
    """One COCO simulation and the clients attached to it."""

    id: str = field(  # noqa: A003 - "id" is the wire field name
        default_factory=lambda: uuid.uuid4().hex[:12])
    created_at: float = field(default_factory=time.time)
    state: str = STARTING
    components: dict = field(default_factory=dict)
    clients: set = field(default_factory=set)
    active_mission: str = ''
    target_colour: str = ''

    def __post_init__(self):
        """Give the session one ComponentState per known subsystem."""
        for name in ALL_COMPONENTS:
            self.components.setdefault(name, ComponentState(name))

    # ── observation ────────────────────────────────────────────────────
    def observe(self, name, up, detail='', now=None):
        """
        Record that `name` is (or is not) up, and re-derive the state.

        Unknown component names are accepted rather than raising: a future
        launch file may report something this build has never heard of,
        and dropping that on the floor is friendlier than crashing the
        telemetry loop. It simply never becomes required.
        """
        stamp = time.monotonic() if now is None else now
        component = self.components.get(name)
        if component is None:
            component = ComponentState(name)
            self.components[name] = component
        component.up = bool(up)
        component.detail = detail
        if up:
            component.last_seen = stamp
        self._rederive()
        return component

    def _rederive(self):
        """Recompute `state` from the components. STOPPED is terminal."""
        if self.state == STOPPED:
            return
        missing = self.missing_required()
        if not missing:
            self.state = READY
        elif len(missing) == len(REQUIRED):
            # Nothing at all has reported yet: still coming up, not broken.
            self.state = STARTING
        else:
            self.state = DEGRADED

    def missing_required(self):
        """List required components that are not up, in REQUIRED order."""
        return [name for name in REQUIRED
                if not self.components[name].up]

    def stop(self):
        """Mark the session stopped; it will not become ready again."""
        self.state = STOPPED
        self.clients.clear()

    # ── clients ────────────────────────────────────────────────────────
    def attach(self, client_id):
        """Register a connected client. Returns the client count."""
        self.clients.add(client_id)
        return len(self.clients)

    def detach(self, client_id):
        """Drop a disconnected client. Returns the client count."""
        self.clients.discard(client_id)
        return len(self.clients)

    # ── views ──────────────────────────────────────────────────────────
    def as_dict(self):
        """Build the `session` object sent in welcome and telemetry frames."""
        return {
            'id': self.id,
            'state': self.state,
            'created_at': self.created_at,
            'uptime': max(0.0, time.time() - self.created_at),
            'clients': len(self.clients),
            'active_mission': self.active_mission,
            'target_colour': self.target_colour,
            'missing': self.missing_required(),
            'components': {name: c.as_dict()
                           for name, c in sorted(self.components.items())},
        }

    def health(self):
        """
        Build the /healthz body and its HTTP status.

        Returns ``(body, status)``. 200 only when READY: a health check
        that goes green while Gazebo is still spawning is worse than no
        health check, because orchestration will then route traffic at a
        simulator that cannot answer. 503 is the honest answer for both
        STARTING and DEGRADED, and they are distinguished in the body.
        """
        body = {
            'status': self.state,
            'ready': self.state == READY,
            'session': self.id,
            'missing': self.missing_required(),
            'components': {name: c.as_dict()
                           for name, c in sorted(self.components.items())},
        }
        return body, (200 if self.state == READY else 503)


class SessionRegistry:
    """
    Holds the sessions this server serves. P0.1: exactly one.

    Single-session is enforced rather than merely assumed, so the day a
    second one is wanted the failure is a clear error at the call site
    instead of two clients quietly sharing one robot.
    """

    #: P0.1 cap. Raising this alone does NOT make the platform multi-user;
    #: it also needs per-session ROS graph isolation (separate containers,
    #: separate ROS_DOMAIN_ID). See docs/PRODUCT_ARCHITECTURE.md.
    max_sessions = 1

    def __init__(self):
        """Create an empty registry."""
        self._sessions = {}

    def create(self, session_id=None):
        """Create and register a session, or raise if the cap is reached."""
        if len(self._sessions) >= self.max_sessions:
            raise RuntimeError(
                f'this build serves {self.max_sessions} simulation session; '
                f'multi-session needs per-session ROS graph isolation '
                f'(see docs/PRODUCT_ARCHITECTURE.md)')
        session = SimulationSession(**({'id': session_id} if session_id else {}))
        self._sessions[session.id] = session
        return session

    def get(self, session_id):
        """Return the session with this id, or None."""
        return self._sessions.get(session_id)

    def sole(self):
        """
        Return the single session, creating it on first use.

        The convenience the server actually uses. It is still expressed as
        a registry lookup so no call site hard-codes a global.
        """
        if not self._sessions:
            return self.create()
        return next(iter(self._sessions.values()))

    def all_sessions(self):
        """Return every registered session."""
        return list(self._sessions.values())

    def drop(self, session_id):
        """Stop and remove a session. Returns True if one was removed."""
        session = self._sessions.pop(session_id, None)
        if session is None:
            return False
        session.stop()
        return True
