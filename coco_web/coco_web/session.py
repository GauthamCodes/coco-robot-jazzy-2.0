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

# ── lifecycle: a SECOND axis, deliberately not merged with the above ───
# `state` answers "is the stack converged?" and is what /healthz reports.
# `lifecycle` answers "where is this session in its life?" -- which is a
# different question, and the two are orthogonal: a session can be READY
# and idle, or READY and running a mission.
#
# They are kept apart because merging them breaks something concrete.
# /healthz returns 200 only for `ready`; if RUNNING replaced `ready` the
# moment a mission started, Docker's HEALTHCHECK would mark the container
# unhealthy for the entire duration of a fetch, and a restart policy
# would then kill the robot mid-climb. Two axes cost one extra field and
# avoid that entirely.
LIFE_CREATED = 'CREATED'      # constructed; nothing observed yet
LIFE_STARTING = 'STARTING'    # components coming up
LIFE_READY = 'READY'          # drivable, no mission running
LIFE_RUNNING = 'RUNNING'      # a mission is executing
LIFE_STOPPING = 'STOPPING'    # shutdown requested, not yet complete
LIFE_STOPPED = 'STOPPED'      # terminal, clean
LIFE_FAILED = 'FAILED'        # terminal, or a required component lost

LIFECYCLE_STATES = (LIFE_CREATED, LIFE_STARTING, LIFE_READY, LIFE_RUNNING,
                    LIFE_STOPPING, LIFE_STOPPED, LIFE_FAILED)

# ── connection state, as the SERVER can see it ─────────────────────────
# CONNECTING and DISCONNECTED are deliberately absent: they are facts
# about a socket, which only the client can observe. A server that
# reported "DISCONNECTED" would be reporting it down a connection. The
# split is documented in WEB_API.md so nobody goes looking for them here.
CONN_CONNECTED = 'CONNECTED'
CONN_SIM_STARTING = 'SIMULATOR_STARTING'
CONN_SIM_READY = 'SIMULATOR_READY'
CONN_MISSION_RUNNING = 'MISSION_RUNNING'
CONN_ERROR = 'ERROR'

CONNECTION_STATES = (CONN_CONNECTED, CONN_SIM_STARTING, CONN_SIM_READY,
                     CONN_MISSION_RUNNING, CONN_ERROR)

# ── health: a SEPARATE axis from the lifecycle, and not a rename of it ─
# `lifecycle` answers "where is this session in its life?"; `health`
# answers "is everything this session is supposed to have actually
# working?". They disagree in exactly the cases an operator needs told
# apart: READY-but-DEGRADED is a robot you can drive whose navigation has
# died, and RUNNING-and-HEALTHY is a fetch in progress with nothing wrong.
# One enum cannot say either.
#
# HEALTHY    every required component up, and every EXPECTED one too.
# DEGRADED   every required component up -- the robot is drivable -- but
#            something expected is down: a component the launch declared
#            (`expected`), or one that was up earlier and has since gone.
# UNHEALTHY  a required component is down, or the session failed or
#            stopped. Nothing can be trusted to move the robot.
#
# The spelling DEGRADED collides with the readiness value 'degraded'
# above, which predates this axis and is load-bearing on the wire
# (coco.v1 may not change a field's meaning). Hence HEALTH_DEGRADED for
# the constant and the upper-case value on the wire, matching lifecycle.
HEALTHY = 'HEALTHY'
HEALTH_DEGRADED = 'DEGRADED'
UNHEALTHY = 'UNHEALTHY'

HEALTH_STATES = (HEALTHY, HEALTH_DEGRADED, UNHEALTHY)

# ── component names ────────────────────────────────────────────────────
# Each is a thing that can independently be up or down, and each is
# observed from the ROS graph rather than assumed from a launch file
# having been executed -- "we started it" is not "it is running".
ROS = 'ros'                    # rclpy context alive, node spinning
# COCO's own simulator, not "a" simulator. /clock alone is not evidence:
# an unrelated project's Gazebo publishes one on the same graph, and that
# was measured on the development machine. The server requires COCO's
# model odometry to be ARRIVING as well -- see simulator_up().
SIMULATOR = 'simulator'
ROBOT = 'robot'                # controllers publishing odometry
ARBITER = 'arbiter'            # cmd_vel_arbiter publishing its status
MISSION = 'mission'            # mission executive publishing state
PERCEPTION = 'perception'      # target finder publishing status
NAVIGATION = 'navigation'      # Nav2 publishing a costmap or plan
LIDAR = 'lidar'                # /scan arriving

#: Components that must be up before a session is READY.
#:
#: Navigation, perception and the mission executive are deliberately NOT
#: required: the appliance is useful for manual driving without them, and
#: `mission.launch.py` starts them separately. They are reported so the UI
#: can grey out the mission panel instead of lying about it.
REQUIRED = (ROS, SIMULATOR, ROBOT, ARBITER)

ALL_COMPONENTS = (ROS, SIMULATOR, ROBOT, ARBITER, MISSION, PERCEPTION,
                  NAVIGATION, LIDAR)

#: Optional components a session expects even when no launch file says
#: more. The simulated robot always carries its LiDAR, so a drivable robot
#: with no scan is DEGRADED -- the collision monitor is blind -- even
#: though manual driving still works and /healthz stays 200.
DEFAULT_EXPECTED = (LIDAR,)


def parse_expected(text):
    """
    Turn the ``expected_components`` parameter into a tuple of names.

    Comma- or space-separated, because a launch file passes one string.
    Required components are dropped: they are already required, and
    listing them twice would make a typo look like a second requirement.
    """
    names = []
    for part in str(text or '').replace(',', ' ').split():
        if part not in names and part not in REQUIRED:
            names.append(part)
    return tuple(names)


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
    mission_running: bool = False
    stopping: bool = False
    failed_reason: str = ''
    #: True once every required component has been up at least once. A
    #: component going down AFTER that is a failure; one that has never
    #: come up is still just starting, and those must not look alike.
    converged: bool = False
    #: Optional components whose absence makes the session DEGRADED. Set
    #: from the launch file (``expected_components``), because only the
    #: launch knows whether Nav2 was started at all: a manual-driving
    #: appliance with no Nav2 is HEALTHY, a mission stack whose Nav2 died
    #: is not.
    expected: tuple = DEFAULT_EXPECTED
    #: Every component that has been up at least once. One that was up
    #: and is now down is a loss whether or not anyone declared it
    #: expected -- that is the case the health axis exists to surface.
    seen: set = field(default_factory=set)

    def __post_init__(self):
        """Give the session one ComponentState per known subsystem."""
        for name in ALL_COMPONENTS:
            self.components.setdefault(name, ComponentState(name))
        self.expected = tuple(self.expected)

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
            self.seen.add(name)
        self._rederive()
        return component

    def _rederive(self):
        """Recompute `state` from the components. STOPPED is terminal."""
        if self.state == STOPPED:
            return
        missing = self.missing_required()
        if not missing:
            self.state = READY
            self.converged = True
        elif len(missing) == len(REQUIRED):
            # Nothing at all has reported yet: still coming up, not broken.
            self.state = STARTING
        else:
            self.state = DEGRADED

    def lifecycle(self):
        """
        Return where this session is in its life, on the second axis.

        Derived rather than stored, so it cannot disagree with the
        readiness it is derived from. The one piece of history it needs
        is ``converged``: losing a required component after the stack
        came up is a FAILURE, while never having had it is merely
        STARTING, and a UI that showed those identically would send
        someone debugging a simulator that is simply still booting.
        """
        if self.state == STOPPED:
            return LIFE_FAILED if self.failed_reason else LIFE_STOPPED
        if self.stopping:
            return LIFE_STOPPING
        if self.state == READY:
            return LIFE_RUNNING if self.mission_running else LIFE_READY
        if self.state == DEGRADED and self.converged:
            return LIFE_FAILED
        if self.state == STARTING and not self.components[ROS].up:
            return LIFE_CREATED
        return LIFE_STARTING

    def connection(self):
        """
        Report what the SERVER can say about this connection.

        Not whether a socket is open -- the client already knows that,
        and a server reporting DISCONNECTED would be doing so down a
        connection. This answers the question a user actually has: is
        there a robot at the other end of it yet?
        """
        if self.state == STOPPED or self.failed_reason:
            return CONN_ERROR
        if self.state == DEGRADED and self.converged:
            return CONN_ERROR
        if not self.components[SIMULATOR].up:
            return CONN_SIM_STARTING
        if self.missing_required():
            return CONN_SIM_READY
        if self.mission_running:
            return CONN_MISSION_RUNNING
        return CONN_CONNECTED

    def fail(self, reason):
        """Mark the session failed, with a reason the UI can show."""
        self.failed_reason = reason
        return self.lifecycle()

    def missing_required(self):
        """List required components that are not up, in REQUIRED order."""
        return [name for name in REQUIRED
                if not self.components[name].up]

    def degraded_by(self):
        """
        List optional components that should be up and are not, sorted.

        "Should be" is either of two facts: the launch declared it
        expected, or it has been up before in this session. The second is
        what catches a mission stack whose navigation died mid-run on an
        appliance that never declared anything.
        """
        wanted = (set(self.expected) | self.seen) - set(REQUIRED)
        return sorted(name for name in wanted
                      if name in self.components
                      and not self.components[name].up)

    def health_state(self):
        """
        Return HEALTHY, DEGRADED or UNHEALTHY -- the health axis.

        Derived, like ``lifecycle()``, so the two can never disagree about
        the facts they share. They differ in the question they answer; see
        the note beside HEALTH_STATES.
        """
        if self.state == STOPPED or self.failed_reason:
            return UNHEALTHY
        if self.missing_required():
            return UNHEALTHY
        if self.degraded_by():
            return HEALTH_DEGRADED
        return HEALTHY

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
            'lifecycle': self.lifecycle(),
            'health': self.health_state(),
            'degraded_by': self.degraded_by(),
            'expected': list(self.expected),
            'connection': self.connection(),
            'created_at': self.created_at,
            'uptime': max(0.0, time.time() - self.created_at),
            'clients': len(self.clients),
            'active_mission': self.active_mission,
            'mission_running': self.mission_running,
            'target_colour': self.target_colour,
            'failed_reason': self.failed_reason,
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
            # Reported, never used as the gate. A mission running is not
            # a health problem, and 200 must not depend on it -- see the
            # note beside LIFECYCLE_STATES for what merging the two axes
            # would do to a container mid-fetch.
            'lifecycle': self.lifecycle(),
            # The health axis rides along too. It does NOT set the status
            # code either: DEGRADED is a drivable robot, and a container
            # restart policy must not kill it for having lost its LiDAR
            # view. 200 is exactly "health is not UNHEALTHY", which is
            # "every required component is up" -- a test pins that.
            'health': self.health_state(),
            'degraded_by': self.degraded_by(),
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
