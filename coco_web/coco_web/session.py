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

from collections import deque
from dataclasses import dataclass, field
import time
import uuid

from coco_web import lifecycle as lifecycle_policy

# ── session readiness (coco.v1 `state`) ────────────────────────────────
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
# `state` answers "is the stack converged?" and /healthz reports it.
# `lifecycle` answers "where is this session in its life?" -- which is a
# different question, and the two are orthogonal: a session can be READY
# and idle, or READY and running a mission.
#
# They are kept apart because merging them breaks something concrete.
# /healthz needs `ready` for a 200; if RUNNING replaced `ready` the
# moment a mission started, Docker's HEALTHCHECK would mark the container
# unhealthy for the entire duration of a fetch, and a restart policy
# would then kill the robot mid-climb. Two axes cost one extra field and
# avoid that entirely.
#
# The lifecycle is STORED, not derived, and it moves only on explicit
# events, each checked against LIFECYCLE_EDGES below. P0.2's first
# version derived it from readiness on every read, and a derivation has
# no memory: Codex measured fail() on a READY session still reading
# READY, and a converged session that lost every component reading
# CREATED -- as though it had never started.
#
# Health never moves it. A stale arbiter status, a lost LiDAR or a dead
# Nav2 is a HEALTH fact (UNHEALTHY or DEGRADED) and leaves a running
# mission RUNNING, because the executive -- not this server -- decides
# whether the mission is still going. The one observation that IS a
# lifecycle event is COCO's simulator stopping after convergence: a gz
# that comes back is a different world (DetachableJoint binds once per
# spawn; CLAUDE.md "fresh simulator per mission run"), so the session
# FAILS, and the simulator's return RESTARTS it through STARTING rather
# than letting it jump back to READY as if nothing happened.
LIFE_CREATED = 'CREATED'      # constructed; nothing observed yet
LIFE_STARTING = 'STARTING'    # observing; required components coming up
LIFE_READY = 'READY'          # converged, no mission running
LIFE_RUNNING = 'RUNNING'      # converged, the executive reports a mission
LIFE_STOPPING = 'STOPPING'    # shutdown requested, not yet complete
LIFE_STOPPED = 'STOPPED'      # terminal
LIFE_FAILED = 'FAILED'        # fail(): explicit, or COCO's simulator lost

LIFECYCLE_STATES = (LIFE_CREATED, LIFE_STARTING, LIFE_READY, LIFE_RUNNING,
                    LIFE_STOPPING, LIFE_STOPPED, LIFE_FAILED)

#: Every legal lifecycle change, and nothing else. Supplied to Codex's
#: ``lifecycle.validate_transition``, which raises on anything absent.
#: Deliberately absent: FAILED -> READY/RUNNING (a failed session
#: re-converges through STARTING), STARTING -> RUNNING (no mission is
#: RUNNING on a stack that has not converged), and anything out of
#: STOPPED, which is terminal.
LIFECYCLE_EDGES = frozenset({
    (LIFE_CREATED, LIFE_STARTING),     # begin: the node is observing
    (LIFE_STARTING, LIFE_READY),       # converge: every REQUIRED is up
    (LIFE_READY, LIFE_RUNNING),        # the executive reports a mission
    (LIFE_RUNNING, LIFE_READY),        # ...and then that it ended
    # fail(): from anything live, including a shutdown that failed
    (LIFE_CREATED, LIFE_FAILED), (LIFE_STARTING, LIFE_FAILED),
    (LIFE_READY, LIFE_FAILED), (LIFE_RUNNING, LIFE_FAILED),
    (LIFE_STOPPING, LIFE_FAILED),
    (LIFE_FAILED, LIFE_STARTING),      # restart(): re-converge from zero
    # request_stop(): from anything not already ending
    (LIFE_CREATED, LIFE_STOPPING), (LIFE_STARTING, LIFE_STOPPING),
    (LIFE_READY, LIFE_STOPPING), (LIFE_RUNNING, LIFE_STOPPING),
    (LIFE_FAILED, LIFE_STOPPING),
    (LIFE_STOPPING, LIFE_STOPPED),     # stop(): complete
})

#: failed_reason recorded when COCO's simulator stops after convergence.
SIMULATOR_LOST = "COCO's simulator stopped stepping"

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
    failed_reason: str = ''
    #: True once every required component has been up at least once in
    #: this run. A component going down AFTER that is a loss; one that has
    #: never come up is still just starting, and those must not look
    #: alike. Reset by restart().
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
        # The lifecycle is stored and moved only by _transition(); see the
        # note beside LIFECYCLE_EDGES. Not a dataclass field, so nothing
        # can construct a session already RUNNING.
        self._life = LIFE_CREATED
        self._mission = False
        self._sim_lost = False
        #: Server wall-clock time of the last lifecycle change. It is the
        #: server's own act, so this is an observation, not a derivation.
        self.lifecycle_since = time.time()
        #: The last lifecycle changes as (from, to, wall time), newest
        #: last. Bounded: a session that flaps must not grow memory.
        self.transitions = deque(maxlen=64)

    # ── the lifecycle axis ─────────────────────────────────────────────
    def lifecycle(self):
        """Return where this session is in its life, on the second axis."""
        return self._life

    @property
    def mission_running(self):
        """Whether the executive last reported a mission in progress."""
        return self._mission

    def _transition(self, target):
        """
        Move the lifecycle to `target`, if LIFECYCLE_EDGES allows it.

        The only writer of the lifecycle. An illegal edge raises
        ValueError and changes nothing, so a bug here is loud rather than
        a session quietly reporting a state it could not have reached.
        """
        if lifecycle_policy.validate_transition(
                self._life, target, LIFECYCLE_EDGES):
            stamp = time.time()
            self.transitions.append((self._life, target, stamp))
            self._life = target
            self.lifecycle_since = stamp
        return self._life

    def _settle(self):
        """
        Apply the convergence rule: STARTING with every REQUIRED up.

        READY first, then RUNNING if the executive already reported a
        mission -- two legal edges, never the absent STARTING -> RUNNING.
        """
        if self._life == LIFE_STARTING and not self.missing_required():
            self.converged = True
            self._transition(LIFE_READY)
            if self._mission:
                self._transition(LIFE_RUNNING)

    def set_mission_running(self, running):
        """
        Record the executive's own view of whether a mission is running.

        Moves READY <-> RUNNING. In any other lifecycle state it is only
        remembered, and applied when the session converges: a platform
        restarted mid-fetch still shows the fetch.
        """
        self._mission = bool(running)
        if self._life == LIFE_READY and self._mission:
            self._transition(LIFE_RUNNING)
        elif self._life == LIFE_RUNNING and not self._mission:
            self._transition(LIFE_READY)
        return self._life

    def fail(self, reason):
        """
        Mark the session FAILED, with a reason the UI can show.

        Explicit: nothing about component health calls this except the
        loss of COCO's own simulator (see LIFECYCLE_EDGES). Failing a
        FAILED session updates the reason; failing a STOPPED one raises.
        """
        self.failed_reason = str(reason)
        return self._transition(LIFE_FAILED)

    def restart(self):
        """
        Leave FAILED and re-converge from nothing: FAILED -> STARTING.

        Everything this run learned is forgotten -- convergence, which
        components were seen -- because it was learned about a world that
        no longer exists. If every required component is already up the
        session converges again in the same call.
        """
        self._transition(LIFE_STARTING)
        self.failed_reason = ''
        self._sim_lost = False
        self.converged = False
        self.seen = {name for name, c in self.components.items() if c.up}
        self._rederive()
        self._settle()
        return self._life

    def request_stop(self):
        """Begin shutdown: -> STOPPING. A no-op when already ending."""
        if self._life not in (LIFE_STOPPING, LIFE_STOPPED):
            self._transition(LIFE_STOPPING)
        return self._life

    def stop(self):
        """Complete shutdown: -> STOPPED, terminal. Drops every client."""
        self.request_stop()
        self._transition(LIFE_STOPPED)
        self.state = STOPPED
        self.clients.clear()
        return self._life

    # ── observation ────────────────────────────────────────────────────
    def observe(self, name, up, detail='', now=None):
        """
        Record that `name` is (or is not) up; apply what that implies.

        Readiness (`state`) and health are re-derived. The lifecycle moves
        only on the events an observation can carry -- the first sign of
        life, convergence, and COCO's simulator going or coming back --
        each through a legal edge.

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
        self._advance(name, bool(up))
        return component

    def _advance(self, name, up):
        """Apply the lifecycle events one observation can carry."""
        if self._life in (LIFE_STOPPING, LIFE_STOPPED):
            return
        if self._life == LIFE_CREATED and up:
            self._transition(LIFE_STARTING)
        if (self._life in (LIFE_READY, LIFE_RUNNING)
                and name == SIMULATOR and not up):
            self._sim_lost = True
            self.fail(SIMULATOR_LOST)
            return
        if (self._life == LIFE_FAILED and self._sim_lost
                and name == SIMULATOR and up):
            self.restart()
            return
        self._settle()

    def _rederive(self):
        """
        Recompute readiness (`state`) from the components.

        STOPPED is terminal. After convergence a missing component is
        DEGRADED however many are missing: losing everything is not
        "still starting", which is what the derived version reported.
        """
        if self.state == STOPPED:
            return
        missing = self.missing_required()
        if not missing:
            self.state = READY
        elif len(missing) == len(REQUIRED) and not self.converged:
            # Nothing at all has reported yet: still coming up, not broken.
            self.state = STARTING
        else:
            self.state = DEGRADED

    def connection(self):
        """
        Report what the SERVER can say about this connection.

        Not whether a socket is open -- the client already knows that,
        and a server reporting DISCONNECTED would be doing so down a
        connection. This answers the question a user actually has: is
        there a robot at the other end of it yet?
        """
        if self._life in (LIFE_STOPPED, LIFE_FAILED):
            return CONN_ERROR
        if self.state == DEGRADED and self.converged:
            return CONN_ERROR
        if not self.components[SIMULATOR].up:
            return CONN_SIM_STARTING
        if self.missing_required():
            return CONN_SIM_READY
        if self._mission:
            return CONN_MISSION_RUNNING
        return CONN_CONNECTED

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

        Derived from the components on every read -- unlike the
        lifecycle, which is stored. The dependency runs one way only: a
        session that is FAILED or shutting down cannot serve, so it is
        UNHEALTHY, but no health value ever moves the lifecycle.
        """
        if self._life in (LIFE_FAILED, LIFE_STOPPING, LIFE_STOPPED):
            return UNHEALTHY
        if self.missing_required():
            return UNHEALTHY
        if self.degraded_by():
            return HEALTH_DEGRADED
        return HEALTHY

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
            # Server wall clock at the last lifecycle change. Additive.
            'lifecycle_since': self.lifecycle_since,
            'health': self.health_state(),
            'degraded_by': self.degraded_by(),
            'expected': list(self.expected),
            'connection': self.connection(),
            'created_at': self.created_at,
            'uptime': max(0.0, time.time() - self.created_at),
            'clients': len(self.clients),
            'active_mission': self.active_mission,
            'mission_running': self._mission,
            'target_colour': self.target_colour,
            'failed_reason': self.failed_reason,
            'missing': self.missing_required(),
            'components': {name: c.as_dict()
                           for name, c in sorted(self.components.items())},
        }

    def health(self):
        """
        Build the /healthz body and its HTTP status.

        Returns ``(body, status)``. 200 exactly when health is not
        UNHEALTHY: every required component up, and the session neither
        FAILED nor shutting down. A health check that goes green while
        Gazebo is still spawning is worse than no health check, because
        orchestration will then route traffic at a simulator that cannot
        answer. 503 is the honest answer for STARTING and DEGRADED
        readiness, and they are distinguished in the body.

        ``state == ready`` is necessary for 200 but no longer sufficient:
        an explicitly failed session with every component up used to
        answer 200 beside ``health: UNHEALTHY``.
        """
        healthy = self.health_state() != UNHEALTHY
        body = {
            'status': self.state,
            'ready': healthy,
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
        return body, (200 if healthy else 503)


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
