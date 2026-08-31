#!/usr/bin/env python3
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
mission_states — the fetch mission as an explicit finite state machine.

This module is **pure**: no rclpy, no clock, no topics, no I/O. It is
given an :class:`Observation` (a snapshot of everything the executive can
see, including the current time) and returns a :class:`Directive` (which
state it is in, which velocity source owns the wheels, and the one
request the node should issue). That split is the whole point — every
transition, timeout, retry and abort in this file is testable without a
simulator, a ROS graph or a wall clock, and the same event sequence
always produces the same transitions.

Why this exists
---------------
``traverse_demo.py`` runs the same mission correctly and has done since
M4. What it cannot do is *say* anything about itself. It is a list of
lambdas executed in order inside one blocking loop, and every one of the
following is either absent or implicit in it:

- **entry conditions.** Step 3 runs because step 2 returned True, not
  because the robot is anywhere in particular.
- **success conditions.** ``outcome in ('arrived', 'held', 'placed',
  'done')`` trusts the worker's own verdict; the pose the worker reached
  is never checked against where it was supposed to be. A climb that
  drifts 0.51 m off the lane still reports ``outcome=goal``.
- **structured failure.** Everything prints ``FAILED at: <label>`` and
  exits 1. A navigation abort, a navigation timeout and a grasp that
  never picked the object up are the same event to anything downstream.
- **retry.** There is none. Any failure ends the mission.
- **recovery.** ``abort()`` stops the wheels; nothing decides whether the
  failure was worth another attempt.

The states below are the same mission. What is added is the contract
around each one.

The state contract
------------------
Every state carries a :class:`StateContract`: which velocity source owns
the robot while it runs, how long it may take, how many times it may be
retried, where a retry restarts, and what happens when the retries run
out. The failure path is uniform — any failure enters ``RECOVERY``, which
stops the robot, records the reason, and then either restarts a state or
escalates to ``ABORT``.

Two design choices worth stating plainly, because both were available
and rejected:

**The executive never commands velocity.** It publishes ``/mission/mode``
and calls the existing services; ``cmd_vel_arbiter`` remains the sole
publisher to ``/diff_drive_controller/cmd_vel``. That is why
``ALIGN_FOR_CLIMB`` is a *verification* state and not an aligner: an
actuating align would need a new velocity source, and adding one to fix
a state machine is how the arbiter invariant dies.

**A failure on the platform does not abort the mission.** The states
between ``VERIFY_CLIMB`` and ``VERIFY_GRASP`` all sit on a 0.65 m raised
deck. Aborting there parks the robot where it needs a manual recovery,
which is a worse outcome than coming home empty and saying so. Those
states therefore exhaust into ``skip_grasp``: the descent and the drive
home still run, and the mission ends in ``ABORT`` with the reason
recorded once the robot is somewhere recoverable. This preserves exactly
what ``traverse_demo.verify_target`` already did, and generalises it to
every state that shares the hazard.

What this module deliberately does NOT decide
---------------------------------------------
Whether localization is *good*. ``LOCALIZE`` checks that a pose estimate
exists, not that it is trustworthy. C2-M1 withheld a GOOD/DEGRADED
verdict in the HUD because that threshold has never been calibrated
against a known-bad run, and C2-M5 is the milestone that measures it.
Inventing the threshold here to make a state look complete would be the
same mistake with a state machine wrapped around it.
"""

import math

from coco_config.robot import (
    RAMP_FOOT_X,
    RAMP_SUMMIT_X,
    SPAWN_XY,
    lane_for_colour,
)

# ── states ───────────────────────────────────────────────────────────────
IDLE = 'IDLE'
LOCALIZE = 'LOCALIZE'
NAVIGATE_TO_RAMP = 'NAVIGATE_TO_RAMP'
ALIGN_FOR_CLIMB = 'ALIGN_FOR_CLIMB'
CLIMB = 'CLIMB'
VERIFY_CLIMB = 'VERIFY_CLIMB'
SEARCH_TARGET = 'SEARCH_TARGET'
STOW_ARM = 'STOW_ARM'
APPROACH_TARGET = 'APPROACH_TARGET'
GRASP = 'GRASP'
VERIFY_GRASP = 'VERIFY_GRASP'
DESCEND = 'DESCEND'
RETURN_HOME = 'RETURN_HOME'
PLACE = 'PLACE'
VERIFY_PLACEMENT = 'VERIFY_PLACEMENT'
COMPLETE = 'COMPLETE'
RECOVERY = 'RECOVERY'
# C2-M5.1. Entered only from RECOVERY, and only for a localization
# failure: RECOVERY has already proved the robot stopped, and RELOCALIZE
# is the one state that deliberately moves it again before the mission
# resumes. It is not on NOMINAL_NEXT and cannot be reached from it.
RELOCALIZE = 'RELOCALIZE'
ABORT = 'ABORT'

TERMINAL_STATES = (COMPLETE, ABORT)

# ── structured failure reasons ───────────────────────────────────────────
# One name per distinguishable cause. `FAILED` is not on this list on
# purpose: a reason that does not say which of the four things went wrong
# is a log line, not a diagnosis.
NO_ODOMETRY = 'NO_ODOMETRY'
NO_RAMP_DRIVER = 'NO_RAMP_DRIVER'
NO_LOCALIZATION = 'NO_LOCALIZATION'
NO_TARGET_COLOUR = 'NO_TARGET_COLOUR'
NAVIGATION_UNAVAILABLE = 'NAVIGATION_UNAVAILABLE'
NAVIGATION_REJECTED = 'NAVIGATION_REJECTED'
NAVIGATION_FAILED = 'NAVIGATION_FAILED'
NAVIGATION_TIMEOUT = 'NAVIGATION_TIMEOUT'
PRE_RAMP_POSE_OUT_OF_REGION = 'PRE_RAMP_POSE_OUT_OF_REGION'
HOME_POSE_OUT_OF_REGION = 'HOME_POSE_OUT_OF_REGION'
ALIGN_OFF_LANE = 'ALIGN_OFF_LANE'
ALIGN_HEADING = 'ALIGN_HEADING'
ALIGN_NOT_ON_FLAT = 'ALIGN_NOT_ON_FLAT'
ALIGN_TIMEOUT = 'ALIGN_TIMEOUT'
SERVICE_UNAVAILABLE = 'SERVICE_UNAVAILABLE'
SERVICE_REFUSED = 'SERVICE_REFUSED'
CLIMB_FAILED = 'CLIMB_FAILED'
CLIMB_TIPPED = 'CLIMB_TIPPED'
CLIMB_TIMEOUT = 'CLIMB_TIMEOUT'
CLIMB_POSE_UNVERIFIED = 'CLIMB_POSE_UNVERIFIED'
CLIMB_OFF_LANE = 'CLIMB_OFF_LANE'
CLIMB_VERIFY_TIMEOUT = 'CLIMB_VERIFY_TIMEOUT'
TARGET_NOT_FOUND = 'TARGET_NOT_FOUND'
TARGET_COLOUR_MISMATCH = 'TARGET_COLOUR_MISMATCH'
STOW_FAILED = 'STOW_FAILED'
STOW_TIMEOUT = 'STOW_TIMEOUT'
APPROACH_FAILED = 'APPROACH_FAILED'
APPROACH_TIMEOUT = 'APPROACH_TIMEOUT'
GRASP_FAILED = 'GRASP_FAILED'
GRASP_TIMEOUT = 'GRASP_TIMEOUT'
GRASP_UNVERIFIED = 'GRASP_UNVERIFIED'
GRASP_VERIFY_TIMEOUT = 'GRASP_VERIFY_TIMEOUT'
DESCENT_FAILED = 'DESCENT_FAILED'
DESCENT_TIPPED = 'DESCENT_TIPPED'
DESCENT_TIMEOUT = 'DESCENT_TIMEOUT'
RETURN_FAILED = 'RETURN_FAILED'
RETURN_TIMEOUT = 'RETURN_TIMEOUT'
PLACE_FAILED = 'PLACE_FAILED'
PLACE_TIMEOUT = 'PLACE_TIMEOUT'
PLACEMENT_UNVERIFIED = 'PLACEMENT_UNVERIFIED'
PLACEMENT_VERIFY_TIMEOUT = 'PLACEMENT_VERIFY_TIMEOUT'
RECOVERY_TIMEOUT = 'RECOVERY_TIMEOUT'
OPERATOR_ABORT = 'OPERATOR_ABORT'
CLOCK_STALLED = 'CLOCK_STALLED'
# C2-M5.1. The scan stopped agreeing with the map, and stayed that way
# for DEGRADED_HOLD_S. Distinct from RETURN_FAILED on purpose: that one
# says Nav2 gave up, this one says the pose Nav2 was steering by is not
# where the robot is, and only the second is worth relocalizing for.
LOCALIZATION_DEGRADED = 'LOCALIZATION_DEGRADED'
LOCALIZATION_RECOVERY_FAILED = 'LOCALIZATION_RECOVERY_FAILED'
LOCALIZATION_RECOVERY_TIMEOUT = 'LOCALIZATION_RECOVERY_TIMEOUT'
LOCALIZATION_RECOVERY_UNAVAILABLE = 'LOCALIZATION_RECOVERY_UNAVAILABLE'

# ── outcomes of a per-state check ────────────────────────────────────────
RUNNING = 'running'
SUCCESS = 'success'
FAILURE = 'failure'

# ── what happens when a state's retries run out ──────────────────────────
ESCALATE_ABORT = 'abort'
# Come down off the platform first, then report the failure. See the
# module docstring: a robot parked on a 0.65 m deck is not recoverable
# without hands, and an empty-handed robot at home is.
ESCALATE_SKIP_GRASP = 'skip_grasp'

# ── geometry, all derived from coco_config ───────────────────────────────
# Where the RL climb terminates, in world x. ramp_env stops GOAL_MARGIN
# short of the crest so the episode never ends on the wedge's vertical
# back face; approach_server's docstring quotes the same 2.700.
#
# GOAL_MARGIN is duplicated rather than imported because coco_rl.ramp_env
# pulls in gymnasium and rclpy, and this module must stay importable with
# neither. test_mission_states asserts the two agree.
CLIMB_GOAL_MARGIN = 0.3
CLIMB_END_X = RAMP_SUMMIT_X - CLIMB_GOAL_MARGIN

# The flat-ground pose the mission climbs from, and home. Both in WORLD
# coordinates; the map frame is anchored at the spawn pose, so a Nav2
# goal adds WORLD_TO_MAP_X. Lifted unchanged from traverse_demo.
PRE_RAMP_X = 0.5
HOME = (SPAWN_XY[0], SPAWN_XY[1])
WORLD_TO_MAP_X = -SPAWN_XY[0]

# Half the lane spacing (coco_config puts the four targets 0.5 m apart).
# Past this the robot is nearer its neighbour's lane than its own, which
# is the point at which "off the lane" stops being a tolerance question.
LANE_TOLERANCE = 0.25

# Nav2's own general_goal_checker, gazebo_models/config/nav2_params.yaml:
# xy_goal_tolerance 0.25. Used as the region bound so the executive's
# independent check is calibrated to the same number the planner was told
# to achieve, rather than to a new one.
GOAL_XY_TOLERANCE = 0.25

# The heading gate is OFF by default, and that is a measurement, not
# timidity. nav2_params sets yaw_goal_tolerance 0.25 rad, so 0.25 is the
# obvious candidate — and it is wrong in a way only a live run shows:
# Nav2 judges yaw against the AMCL pose it is steering by, while this
# check reads ground truth, so the two differ by the localisation error
# and a gate AT Nav2's tolerance fires whenever that error points the
# wrong way. Measured, C2-M3.0, one live run: the leg arrived at
# **+0.28 rad** and, re-driven, at **+0.26 rad** — both inside Nav2's own
# checker, both outside a 0.25 rad ground-truth gate. The mission it
# aborted is the mission that completes 19/20.
#
# Re-driving cannot fix it either: the same goal through the same goal
# checker cannot produce a tighter yaw than the checker's own tolerance,
# so the retry is structurally futile. A real heading gate needs either a
# tighter goal checker for that leg or an aligner behind the arbiter, and
# a threshold measured against climbs that actually failed. Neither is
# C2-M3.0's to do.
#
# So the executive follows the precedent C2-M1 set for the HUD's
# localization verdict: **report the number, do not assert a threshold
# nobody measured.** Set `yaw_tolerance` to a float to turn the gate on.
GOAL_YAW_TOLERANCE = None
NAV2_YAW_GOAL_TOLERANCE = 0.25     # what nav2_params configures, for reference

# ── C2-M5.1, the relocalization spin ─────────────────────────────────────
# A FULL revolution, not a fraction of one. The point of the motion is to
# re-observe every bearing the robot could see, and how much new geometry
# a partial spin brings in depends on the heading error -- which is
# unknown by construction, because the robot is lost. 2*pi removes that
# dependence; nothing else about the number was chosen.
RELOCALIZE_SPIN_RAD = 2.0 * math.pi

# nav2_params.yaml, behavior_server: max_rotational_vel 1.0 rad/s,
# min_rotational_vel 0.4. A full turn at the SLOWEST configured rate is
# 2*pi/0.4 = 15.7 s. Doubled, because the spin's output passes through
# the velocity smoother and the collision monitor and C2-M5.0 measured
# the monitor throttling commands mid-leg; plus HEALTHY_HOLD_S, because
# the state does not end when the spin ends -- it ends when the health
# monitor has called the robot healthy for that long. 2*15.7 + 3 = 34.4,
# rounded up.
RELOCALIZE_TIMEOUT = 40.0


def parse_kv(line):
    """
    Split a ``key=value key=value`` status line into a dict.

    Every status topic in this project uses that shape —
    ``/ramp/status``, ``/approach/status``, ``/grasp/status``,
    ``/perception/status``, ``/cmd_vel_arbiter/status`` — so one parser
    reads all of them. Defensive by design: a malformed field is dropped
    rather than raised, because a status line is telemetry and a parser
    that throws inside a timer callback takes the executive down with it.

    mission_hud has its own copy of this function. The duplication is
    deliberate — this module must not import a ROS node to parse a
    string — and a test asserts the two agree on real status lines.
    """
    fields = {}
    for part in (line or '').split():
        key, sep, value = part.partition('=')
        if sep and key:
            fields[key] = value
    return fields


def missing(value):
    """True when a status field carries no measurement.

    The three servers spell "nothing here" three ways: ``--`` (approach,
    grasp), ``none`` (ramp_driver's outcome) and absent. Treating any of
    them as a value is how a state declares success on a field that was
    never written.
    """
    return value is None or value in ('', '--', 'none', 'None')


class WorkerView:
    """The last status line from one subsystem, and when it arrived.

    ``stamp`` is a mission clock reading, not a header stamp. Whether a
    status is *newer than the request that caused it* is an arrival
    question, exactly as it is for the arbiter's watchdog — and it is the
    only thing that stops the executive reading a worker's pre-request
    idle state as "already finished".
    """

    __slots__ = ('fields', 'stamp')

    def __init__(self, fields=None, stamp=None):
        self.fields = dict(fields or {})
        self.stamp = stamp

    @classmethod
    def from_line(cls, line, stamp):
        """Build a view from a raw status string."""
        return cls(parse_kv(line), stamp)

    def get(self, key, default=None):
        """One field, or `default`."""
        return self.fields.get(key, default)

    def seen(self):
        """True once any status line has arrived."""
        return self.stamp is not None

    def newer_than(self, when):
        """True if the last line arrived strictly after `when`."""
        return self.stamp is not None and self.stamp > when

    def __repr__(self):
        return f'WorkerView({self.fields!r}, stamp={self.stamp!r})'


class Observation:
    """Everything the executive can see at one instant.

    Constructed by the node from its subscriptions and clock; constructed
    by hand in the tests. Nothing in this module reads anything that is
    not on this object, which is what makes the machine deterministic.
    """

    __slots__ = (
        'ros_now', 'wall_now', 'started', 'abort_requested', 'colour',
        'pose', 'pose_stamp', 'localized', 'ramp', 'approach', 'grasp',
        'perception', 'arbiter', 'localization', 'request_token',
        'request_status',
    )

    def __init__(self, ros_now, wall_now=None, started=False,
                 abort_requested=False, colour=None, pose=None,
                 pose_stamp=None, localized=False, ramp=None, approach=None,
                 grasp=None, perception=None, arbiter=None,
                 localization=None, request_token=None,
                 request_status=None):
        self.ros_now = ros_now
        self.wall_now = ros_now if wall_now is None else wall_now
        self.started = started
        self.abort_requested = abort_requested
        self.colour = colour
        self.pose = pose                     # (x, y, yaw) in WORLD metres
        # A pose handed over without an arrival time is taken as current.
        # The node always sets it; the default only keeps a hand-written
        # observation in a test from reading as silently stale.
        self.pose_stamp = (pose_stamp if pose_stamp is not None
                           else (ros_now if pose is not None else None))
        self.localized = localized
        self.ramp = ramp or WorkerView()
        self.approach = approach or WorkerView()
        self.grasp = grasp or WorkerView()
        self.perception = perception or WorkerView()
        self.arbiter = arbiter or WorkerView()
        # /localization/health, from localization_monitor (C2-M5.1). An
        # unseen view means the monitor is not on the graph, and every
        # check below reads that as "no evidence" rather than "bad": a
        # mission launched without the monitor behaves exactly as it did
        # before C2-M5.1, which is what keeps the standing 19/20 figure
        # meaningful.
        self.localization = localization or WorkerView()
        # The node reports on at most one outstanding request at a time.
        # A status carrying a stale token is ignored, which is what stops
        # a late reply from a cancelled attempt satisfying the retry.
        self.request_token = request_token
        self.request_status = request_status


# Request kinds the node knows how to perform.
NAV_GOAL = 'nav_goal'
CALL_SERVICE = 'call_service'
STOP_ALL = 'stop_all'
# C2-M5.1. The localization recovery, which the node performs in two
# parts against two interfaces that already exist:
#
#   1. AMCL's `/reinitialize_global_localization` — redistribute the
#      particles over the map's free space.
#   2. nav2_msgs/action/Spin, 2*pi — turn on the spot so the likelihood
#      field has 360 degrees of scan to collapse them with.
#
# **Part 1 is not optional, and Experiment 2 measured why.** The spin
# alone recovered nothing: nav2_params sets `recovery_alpha_fast: 0.0`
# and `recovery_alpha_slow: 0.0`, so AMCL's augmented-MCL random-particle
# injection is switched OFF and the filter has no mechanism to escape a
# mode it is confident in. Its particles stay clustered around the wrong
# pose no matter how much new scan data arrives. Measured: the spin ran
# for 9.1 s, health came back long enough to resume, and the scan
# disagreed again 6.0 s later. Resetting the filter is what makes the
# turn worth taking.
#
# Neither part introduces a wheel-command publisher. `behavior_server` is
# ALREADY on /cmd_vel_nav's publisher list — c2m5_topology.txt recorded
# it there before any of this existed — and the arbiter already reads
# that topic. The executive still never commands velocity itself.
RELOCALIZE_GOAL = 'relocalize_goal'

# Request statuses the node reports back.
PENDING = 'pending'
ACCEPTED = 'accepted'
REJECTED = 'rejected'
UNAVAILABLE = 'unavailable'
SUCCEEDED = 'succeeded'
ABORTED = 'aborted'
CANCELED = 'canceled'


class Request:
    """One side effect the node should perform, exactly once.

    ``token`` is the idempotency key. The machine re-emits the same
    request every update until the state changes; the node compares
    tokens and issues only what it has not issued yet. Retrying a state
    mints a new token, so a retry really does re-send the goal.
    """

    __slots__ = ('kind', 'payload', 'token')

    def __init__(self, kind, payload, token):
        self.kind = kind
        self.payload = payload
        self.token = token

    def __eq__(self, other):
        return (isinstance(other, Request)
                and (self.kind, self.payload, self.token)
                == (other.kind, other.payload, other.token))

    def __repr__(self):
        return f'Request({self.kind}, {self.payload!r}, {self.token!r})'


class Event:
    """A transition, as it will be logged and published."""

    __slots__ = ('at', 'previous', 'state', 'reason', 'attempt', 'detail')

    def __init__(self, at, previous, state, reason=None, attempt=0,
                 detail=''):
        self.at = at
        self.previous = previous
        self.state = state
        self.reason = reason
        self.attempt = attempt
        self.detail = detail

    def __repr__(self):
        return (f'Event({self.previous} -> {self.state}, '
                f'reason={self.reason}, attempt={self.attempt})')


class Directive:
    """What the node should do with this update's result."""

    __slots__ = ('state', 'mode', 'request', 'events', 'terminal')

    def __init__(self, state, mode, request, events, terminal):
        self.state = state
        self.mode = mode
        self.request = request
        self.events = events
        self.terminal = terminal


class StateContract:
    """The contract one state is held to.

    ``timeout`` is in mission-clock seconds and ``None`` means the state
    cannot time out (only IDLE and the two terminals). ``retry_state`` is
    where RECOVERY restarts on a retry — not always the state that
    failed, because the useful recovery for a bad pre-climb pose is to
    drive the leg again, and for a lost target it is to look again.
    """

    __slots__ = ('name', 'mode', 'owner', 'timeout', 'max_retries',
                 'retry_state', 'on_exhausted', 'note')

    def __init__(self, name, mode, owner, timeout, max_retries=0,
                 retry_state=None, on_exhausted=ESCALATE_ABORT, note=''):
        self.name = name
        self.mode = mode
        self.owner = owner
        self.timeout = timeout
        self.max_retries = max_retries
        self.retry_state = retry_state or name
        self.on_exhausted = on_exhausted
        self.note = note


# ── the contract table ───────────────────────────────────────────────────
#
# Timeouts are traverse_demo's, unchanged, wherever traverse_demo had
# one: 240 s for a Nav2 leg, 180 s for a worker service, 40 s for
# start-up. Nothing here is a new budget for a motion that already had a
# measured one. The 10-15 s figures belong to the verification-only
# states, which command nothing and are waiting on a 5 Hz status topic —
# an observation window, not a motion budget.
CONTRACTS = {
    IDLE: StateContract(
        IDLE, 'idle', 'nobody', None,
        note='waits for a start request and a target colour'),
    LOCALIZE: StateContract(
        LOCALIZE, 'idle', 'nobody', 40.0,
        note='odometry, ramp_driver, an AMCL pose and a colour all '
             'present. Presence, NOT quality — C2-M5 owns that'),
    NAVIGATE_TO_RAMP: StateContract(
        NAVIGATE_TO_RAMP, 'nav', 'nav2', 240.0,
        max_retries=2,
        note='NavigateToPose to the pre-ramp pose in the target lane'),
    ALIGN_FOR_CLIMB: StateContract(
        ALIGN_FOR_CLIMB, 'idle', 'nobody', 10.0,
        max_retries=1, retry_state=NAVIGATE_TO_RAMP,
        note='verification only. Recovery is to drive the leg again, '
             'because the executive must not command velocity. The '
             'heading is REPORTED, not gated: see GOAL_YAW_TOLERANCE, '
             'and the live run that showed why'),
    CLIMB: StateContract(
        CLIMB, 'rl', 'ramp_driver', 180.0,
        note='no retry: re-entering a PPO episode from halfway up the '
             'wedge has never been measured, and guessing on a slope '
             'is how a robot ends up on its back'),
    VERIFY_CLIMB: StateContract(
        VERIFY_CLIMB, 'idle', 'nobody', 10.0,
        note="outcome=goal is the driver's verdict; this checks the "
             'pose it actually reached'),
    SEARCH_TARGET: StateContract(
        SEARCH_TARGET, 'idle', 'nobody', 15.0,
        max_retries=2, on_exhausted=ESCALATE_SKIP_GRASP,
        note='vision gate. Exhausting it comes home empty rather than '
             'parking on the deck'),
    STOW_ARM: StateContract(
        STOW_ARM, 'idle', 'grasp_server', 180.0,
        max_retries=1, on_exhausted=ESCALATE_SKIP_GRASP,
        note='at home the pinch sits inside the volume the target will '
             'occupy, so driving at it unstowed knocks it over'),
    APPROACH_TARGET: StateContract(
        APPROACH_TARGET, 'approach', 'approach_server', 180.0,
        max_retries=1, retry_state=SEARCH_TARGET,
        on_exhausted=ESCALATE_SKIP_GRASP,
        note='every recorded approach failure is a lost fix, so the '
             'retry re-acquires the target before re-running'),
    GRASP: StateContract(
        GRASP, 'idle', 'grasp_server', 180.0,
        max_retries=2, on_exhausted=ESCALATE_SKIP_GRASP,
        note='grasp_server re-stows itself if it has to, so a repeat '
             'call is the whole retry'),
    VERIFY_GRASP: StateContract(
        VERIFY_GRASP, 'idle', 'nobody', 10.0,
        max_retries=2, retry_state=GRASP,
        on_exhausted=ESCALATE_SKIP_GRASP,
        note='re-reads the lifted flag after the action returned idle'),
    DESCEND: StateContract(
        DESCEND, 'rl', 'ramp_driver', 180.0,
        note='no retry, for the same reason as CLIMB'),
    RETURN_HOME: StateContract(
        RETURN_HOME, 'nav', 'nav2', 240.0,
        max_retries=2,
        note='the leg that has failed 2 of 4 recorded times. A bounded '
             're-issue is all C2-M3.0 claims; the localization recovery '
             'behind those failures is C2-M5'),
    PLACE: StateContract(
        PLACE, 'idle', 'grasp_server', 180.0, max_retries=1),
    VERIFY_PLACEMENT: StateContract(
        VERIFY_PLACEMENT, 'idle', 'nobody', 10.0,
        max_retries=1, retry_state=PLACE,
        note='the object has to still be standing once the arm left'),
    RECOVERY: StateContract(
        RECOVERY, 'idle', 'nobody', 20.0,
        note='stop everything, then decide retry or abort'),
    # mode 'nav', because the spin is served by nav2's behavior_server and
    # reaches the wheels down the SAME path a Nav2 leg does. No new mode,
    # no new publisher.
    #
    # max_retries=1 means TWO relocalizations per mission, counted on
    # RELOCALIZE itself rather than charged to the leg that was
    # interrupted -- see _resolve_recovery for the run that decided that.
    # It is the only state entered from exactly one place, so the count
    # bounds the recovery loop completely.
    RELOCALIZE: StateContract(
        RELOCALIZE, 'nav', 'nav2', RELOCALIZE_TIMEOUT,
        max_retries=1,
        note='reset the filter, turn on the spot to re-observe, and wait '
             'for the health monitor to call it healthy again. Its own '
             'budget, so a leg that Nav2 aborted has not already spent it'),
    COMPLETE: StateContract(COMPLETE, 'idle', 'nobody', None),
    ABORT: StateContract(ABORT, 'idle', 'nobody', None),
}

# The nominal path. Written once, here, so the sequence is readable
# without tracing handlers — and so a state cannot be silently orphaned.
NOMINAL_NEXT = {
    IDLE: LOCALIZE,
    LOCALIZE: NAVIGATE_TO_RAMP,
    NAVIGATE_TO_RAMP: ALIGN_FOR_CLIMB,
    ALIGN_FOR_CLIMB: CLIMB,
    CLIMB: VERIFY_CLIMB,
    VERIFY_CLIMB: SEARCH_TARGET,
    SEARCH_TARGET: STOW_ARM,
    STOW_ARM: APPROACH_TARGET,
    APPROACH_TARGET: GRASP,
    GRASP: VERIFY_GRASP,
    VERIFY_GRASP: DESCEND,
    DESCEND: RETURN_HOME,
    RETURN_HOME: PLACE,
    PLACE: VERIFY_PLACEMENT,
    VERIFY_PLACEMENT: COMPLETE,
}

# The states that run on the raised platform. Reaching the end of the
# retry budget in any of these must not leave the robot up there.
PLATFORM_STATES = (SEARCH_TARGET, STOW_ARM, APPROACH_TARGET, GRASP,
                   VERIFY_GRASP)

# Services the executive calls, by state.
STATE_SERVICE = {
    CLIMB: '/ramp/climb',
    STOW_ARM: '/grasp/stow',
    APPROACH_TARGET: '/approach/run',
    GRASP: '/grasp/pick',
    DESCEND: '/ramp/descend',
    PLACE: '/grasp/place',
}


class MissionPlan:
    """The mission's parameters, resolved once before it starts.

    ``do_grasp`` false is ``traverse_demo --no-grasp``: the M4/M5
    traverse, kept runnable so those measurements stay reproducible. It
    skips the five platform states and ends in COMPLETE rather than
    reporting an unfetched object, because nothing was asked for.
    """

    def __init__(self, colour, lane=None, do_grasp=True,
                 pre_ramp_x=PRE_RAMP_X, home=HOME,
                 xy_tolerance=GOAL_XY_TOLERANCE,
                 yaw_tolerance=GOAL_YAW_TOLERANCE,
                 lane_tolerance=LANE_TOLERANCE,
                 climb_end_x=CLIMB_END_X,
                 localization_recovery=True):
        self.colour = colour
        resolved = lane if lane is not None else lane_for_colour(colour)
        self.lane = 0.0 if resolved is None else resolved
        self.do_grasp = do_grasp
        self.pre_ramp_x = pre_ramp_x
        self.home = home
        self.xy_tolerance = xy_tolerance
        self.yaw_tolerance = yaw_tolerance
        self.lane_tolerance = lane_tolerance
        self.climb_end_x = climb_end_x
        # C2-M5.1. False publishes the health signal and acts on nothing,
        # which is how the false-positive experiment was run and how a
        # mission is reproduced exactly as it ran before C2-M5.1.
        self.localization_recovery = localization_recovery

    @property
    def pre_ramp(self):
        """The pre-ramp goal in world coordinates."""
        return (self.pre_ramp_x, self.lane)


class MissionMachine:
    """The mission executive's brain: states in, one directive out.

    Usage is a single call per update::

        machine = MissionMachine(plan)
        directive = machine.update(observation)

    ``update`` is pure with respect to the observation — it reads nothing
    else — and its only side effects are on the machine's own fields.
    """

    def __init__(self, plan, contracts=None, stall_limit=10.0):
        self.plan = plan
        self.contracts = contracts or CONTRACTS
        # Wall-clock seconds the ROS clock may stand still before the
        # executive stops trusting it. A dead or orphaned simulator
        # freezes /clock, and a mission timing itself on a frozen clock
        # waits forever — the exact failure the arbiter's watchdog runs
        # on a steady clock to avoid.
        self.stall_limit = stall_limit

        self.state = IDLE
        self.previous = None
        self.entered_at = 0.0
        self.attempts = {}
        self.reason = None            # reason for the CURRENT recovery
        self.detail = ''
        self.failed_state = None
        self.degraded_reason = None   # set by skip_grasp; reported at the end
        self.result = None            # 'fetch' | 'traverse' | 'aborted'
        self.align_yaw = None         # ground-truth heading at the ramp foot
        self.escalate = False         # this failure skips the retry budget

        self._seq = 0
        self._token = None
        self._request = None
        self._accepted_at = None
        self._latched_outcome = None
        self._saw_busy = False
        self._ros_seen = None
        self._ros_still_since = None
        self._pending_events = []
        self._started_at = None

    # ── introspection ────────────────────────────────────────────────────
    @property
    def contract(self):
        """The contract for the state the machine is in."""
        return self.contracts[self.state]

    def attempt(self, state=None):
        """1-based attempt number for `state` (default: the current one)."""
        return self.attempts.get(state or self.state, 0) + 1

    def elapsed(self, now):
        """Seconds since the current state was entered."""
        return now - self.entered_at

    # ── the update ───────────────────────────────────────────────────────
    def update(self, obs):
        """Advance the machine by one observation and say what to do."""
        self._pending_events = []
        self._track_clock(obs)

        if self.state in TERMINAL_STATES:
            if self.state == ABORT:
                # ABORT keeps asking for the stop. Most aborts arrive
                # through RECOVERY, which has already stopped everything
                # — but CLOCK_STALLED goes straight here, and an abort
                # that only stops asserting a mode leaves the last twist
                # to age out against a watchdog instead of being
                # cancelled.
                self._want(STOP_ALL, None)
            return self._directive()

        if obs.abort_requested and self.state != RECOVERY:
            self._fail(obs, OPERATOR_ABORT, escalate=True)
            return self._directive()

        if self._clock_stalled(obs):
            # Straight to ABORT rather than through RECOVERY: RECOVERY's
            # completion condition is that the arbiter reports nothing
            # driving, and a system whose clock has stopped cannot be
            # relied on to report anything at all.
            self.reason = CLOCK_STALLED
            self.result = 'aborted'
            self._transition(ABORT, obs.ros_now, reason=CLOCK_STALLED,
                             detail='ROS time stopped advancing')
            return self._directive()

        status, reason, detail = self._check(obs)

        if status == SUCCESS:
            self._advance(obs)
        elif status == FAILURE:
            self._fail(obs, reason, detail=detail)
        else:
            contract = self.contract
            if (contract.timeout is not None
                    and self.elapsed(obs.ros_now) > contract.timeout):
                if self.state == RECOVERY:
                    # A RECOVERY that times out must NOT be handed to
                    # RECOVERY: _fail re-enters the state and resets its
                    # clock, and the mission then sits in RECOVERY for
                    # ever with the robot possibly still moving. It
                    # escalates instead, which is the only remaining move.
                    self.reason = RECOVERY_TIMEOUT
                    self.escalate = True
                    self._resolve_recovery(obs)
                else:
                    self._fail(
                        obs, self._timeout_reason(obs),
                        detail=f'no completion in {contract.timeout:.0f}s')

        return self._directive()

    # ── clock ────────────────────────────────────────────────────────────
    def _track_clock(self, obs):
        if self._ros_seen is None or obs.ros_now > self._ros_seen:
            self._ros_seen = obs.ros_now
            self._ros_still_since = obs.wall_now
        elif self._ros_still_since is None:
            self._ros_still_since = obs.wall_now

    def _clock_stalled(self, obs):
        if self.state in (IDLE, RECOVERY) or self._ros_still_since is None:
            return False
        return (obs.wall_now - self._ros_still_since) > self.stall_limit

    # ── transitions ──────────────────────────────────────────────────────
    def _transition(self, state, now, reason=None, detail=''):
        self.previous = self.state
        self.state = state
        self.entered_at = now
        self.detail = detail
        self._seq += 1
        self._token = f'{state}:{self._seq}'
        self._request = None
        self._accepted_at = None
        self._latched_outcome = None
        self._saw_busy = False
        self._pending_events.append(
            Event(now, self.previous, state, reason=reason,
                  attempt=self.attempt(state), detail=detail))

    def _advance(self, obs):
        """Move on along the nominal path, applying the mission's shape."""
        now = obs.ros_now
        if self.state == RECOVERY:
            self._resolve_recovery(obs)
            return

        if self.state == RELOCALIZE:
            # Health came back and held. Resume the state that was
            # interrupted -- through its own retry_state, so a leg whose
            # useful retry is somewhere earlier still gets it. The
            # attempt was already charged on the way in.
            resume = self.contracts[self.failed_state].retry_state
            self._transition(resume, now, reason=LOCALIZATION_DEGRADED,
                             detail=f'localization recovered; resuming '
                                    f'{resume}')
            return

        nxt = NOMINAL_NEXT[self.state]

        if self.state == IDLE:
            self._started_at = now

        # --no-grasp, and the skip_grasp escalation, both cut the same
        # five platform states out of the path.
        if nxt in PLATFORM_STATES and not self._grasping():
            nxt = DESCEND

        if self.state == RETURN_HOME and not self._grasping():
            # Nothing to put down. A traverse that was asked for ends
            # COMPLETE; one that lost its object ends ABORT carrying the
            # reason it lost it, which is what traverse_demo's
            # 'FETCH FAILED ... nothing picked up' exit code meant.
            if self.degraded_reason:
                self.result = 'aborted'
                self.reason = self.degraded_reason
                self._transition(ABORT, now, reason=self.degraded_reason,
                                 detail='home, but empty-handed')
            else:
                self.result = 'traverse'
                self._transition(COMPLETE, now, detail='traverse complete')
            return

        if nxt == COMPLETE:
            self.result = 'fetch'

        self._transition(nxt, now)

    def _grasping(self):
        """True while the grasp half of the mission is still live."""
        return self.plan.do_grasp and self.degraded_reason is None

    def _fail(self, obs, reason, detail='', escalate=False):
        """Record a failure and hand the mission to RECOVERY."""
        self.failed_state = self.state
        self.reason = reason
        self.escalate = escalate
        self._transition(RECOVERY, obs.ros_now, reason=reason, detail=detail)

    def _resolve_recovery(self, obs):
        """RECOVERY has stopped the robot; decide retry or escalate."""
        now = obs.ros_now
        failed = self.failed_state
        contract = self.contracts[failed] if failed else self.contract
        used = self.attempts.get(failed, 0)

        # C2-M5.1. A localization failure spends the RELOCALIZE budget,
        # not the interrupted leg's.
        #
        # It shared the leg's budget until Experiment 2 measured what that
        # costs. An injected divergence makes Nav2 abort its own goal
        # first — the pose jump invalidates the path — so the leg is
        # already down one retry before the health monitor has finished
        # accumulating its two seconds of evidence. Relocalizing then
        # spent the last one, and a single further Nav2 hiccup ended a
        # mission whose localization had just been verified repaired.
        # Measured: recovery succeeded, health was re-established, and
        # the mission aborted 2.2 s after resuming with nothing left.
        #
        # Two counters, both bounded, is what makes "the robot may try to
        # fix its localization twice" and "the leg may be re-driven
        # twice" independent statements. Neither can run away: RELOCALIZE
        # is entered at most max_retries+1 times per mission, and this is
        # the only place it is entered from.
        relocalized = self.attempts.get(RELOCALIZE, 0)
        allowed = self.contracts[RELOCALIZE].max_retries + 1
        if (self.reason == LOCALIZATION_DEGRADED and not self.escalate
                and relocalized < allowed):
            self.attempts[RELOCALIZE] = relocalized + 1
            self._transition(RELOCALIZE, now, reason=self.reason,
                             detail=f'relocalization {relocalized + 1}/'
                                    f'{allowed} before resuming {failed}')
            return

        if self.reason == LOCALIZATION_DEGRADED:
            # The budget is gone and the pose is still wrong. Re-driving
            # the leg here would send the robot at a goal computed from an
            # estimate this mission has already twice failed to repair,
            # so it escalates instead of falling through to a plain retry.
            self.escalate = True

        if not self.escalate and used < contract.max_retries:
            self.attempts[failed] = used + 1
            self._transition(contract.retry_state, now,
                             reason=self.reason,
                             detail=f'retry {used + 1}/'
                                    f'{contract.max_retries} after '
                                    f'{self.reason}')
            return

        if (not self.escalate
                and contract.on_exhausted == ESCALATE_SKIP_GRASP):
            # Give up on the object, keep the robot. The descent and the
            # drive home still run; the reason is carried to the end.
            self.degraded_reason = self.reason
            self._transition(DESCEND, now, reason=self.reason,
                             detail='grasp abandoned; coming home')
            return

        self.result = 'aborted'
        self._transition(ABORT, now, reason=self.reason,
                         detail=f'{failed} exhausted its retries')

    def _timeout_reason(self, obs):
        if self.state == LOCALIZE:
            # Name the input that never arrived. "LOCALIZE_TIMEOUT" would
            # send whoever reads the log to the wrong subsystem three
            # times out of four.
            if not obs.colour:
                return NO_TARGET_COLOUR
            if obs.pose is None:
                return NO_ODOMETRY
            if not obs.ramp.seen():
                return NO_RAMP_DRIVER
            return NO_LOCALIZATION
        return {
            NAVIGATE_TO_RAMP: NAVIGATION_TIMEOUT,
            ALIGN_FOR_CLIMB: ALIGN_TIMEOUT,
            CLIMB: CLIMB_TIMEOUT,
            VERIFY_CLIMB: CLIMB_VERIFY_TIMEOUT,
            # The specific reason IS "the colour never appeared in the
            # window", which says more than SEARCH_TIMEOUT would.
            SEARCH_TARGET: TARGET_NOT_FOUND,
            STOW_ARM: STOW_TIMEOUT,
            APPROACH_TARGET: APPROACH_TIMEOUT,
            GRASP: GRASP_TIMEOUT,
            VERIFY_GRASP: GRASP_VERIFY_TIMEOUT,
            DESCEND: DESCENT_TIMEOUT,
            RETURN_HOME: RETURN_TIMEOUT,
            PLACE: PLACE_TIMEOUT,
            VERIFY_PLACEMENT: PLACEMENT_VERIFY_TIMEOUT,
            RECOVERY: RECOVERY_TIMEOUT,
            # RELOCALIZE times out when the spin ran and the health
            # monitor never called the robot healthy again. That is the
            # failed-recovery path, and it escalates rather than
            # relocalizing a second time: _fail records RELOCALIZE as the
            # failed state, its contract carries no retries, and
            # _resolve_recovery therefore aborts. One spin per charged
            # attempt, and the attempt was charged against the leg.
            RELOCALIZE: LOCALIZATION_RECOVERY_TIMEOUT,
        }.get(self.state, RECOVERY_TIMEOUT)

    # ── the directive ────────────────────────────────────────────────────
    def _directive(self):
        return Directive(
            state=self.state,
            mode=self.contract.mode,
            request=self._request,
            events=self._pending_events,
            terminal=self.state in TERMINAL_STATES)

    def _want(self, kind, payload):
        """Ask the node for a side effect, idempotently."""
        self._request = Request(kind, payload, self._token)
        return self._request

    # ── per-state checks ─────────────────────────────────────────────────
    def _check(self, obs):
        """Run the current state's check: (status, reason, detail)."""
        handler = getattr(self, f'_check_{self.state.lower()}')
        result = handler(obs)
        if isinstance(result, tuple):
            return (result + (None, ''))[:3]
        return result, None, ''

    def _check_idle(self, obs):
        if not obs.started or not obs.colour:
            return RUNNING
        return SUCCESS

    def _check_localize(self, obs):
        # Presence, not quality. All four are things traverse_demo's
        # wait_ready checked or assumed; naming them means a start-up
        # failure says WHICH input never arrived.
        if not obs.colour:
            return RUNNING
        if obs.pose is None:
            return RUNNING
        if not obs.ramp.seen():
            return RUNNING
        if not obs.localized:
            return RUNNING
        return SUCCESS

    def _check_navigate_to_ramp(self, obs):
        return self._check_nav_leg(
            obs, self.plan.pre_ramp, PRE_RAMP_POSE_OUT_OF_REGION)

    def _check_return_home(self, obs):
        return self._check_nav_leg(
            obs, self.plan.home, HOME_POSE_OUT_OF_REGION,
            failed=RETURN_FAILED)

    def _check_nav_leg(self, obs, goal, region_reason,
                       failed=NAVIGATION_FAILED):
        # C2-M5.1. Checked FIRST, before the action's own status: a leg
        # steered by a pose that is three metres wrong will keep
        # reporting RUNNING right up until Nav2 gives up, and C2-M5.0
        # measured that taking 131.5 s on diverged1. The health monitor
        # left the healthy envelope 0.4 s after the divergence.
        #
        # Only the nav legs are guarded. The climb, the platform work and
        # the descent all happen off the mapped ground, where the scan
        # metric is UNKNOWN rather than bad -- see the gate in
        # localization_health -- so there is nothing to guard there and a
        # guard would be firing on noise.
        if self._localization_degraded(obs):
            return (FAILURE, LOCALIZATION_DEGRADED,
                    obs.localization.get('reason') or 'scan disagrees')
        self._want(NAV_GOAL, goal)
        status = self._request_status(obs)
        if status == UNAVAILABLE:
            return FAILURE, NAVIGATION_UNAVAILABLE, 'no navigate_to_pose'
        if status == REJECTED:
            return FAILURE, NAVIGATION_REJECTED, f'goal {goal} rejected'
        if status in (ABORTED, CANCELED):
            return FAILURE, failed, f'action {status}'
        if status != SUCCEEDED:
            return RUNNING
        # The action's own verdict is not the success condition. Nav2
        # judges arrival against the AMCL pose it is also steering by;
        # this checks the world pose the robot actually holds, which is
        # the check that catches a localisation that has drifted.
        pose = self._fresh_pose(obs)
        if pose is None:
            return RUNNING
        error = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
        if error > self.plan.xy_tolerance:
            return (FAILURE, region_reason,
                    f'{error:.2f} m from {goal}, tolerance '
                    f'{self.plan.xy_tolerance:.2f} m')
        return SUCCESS

    def _check_align_for_climb(self, obs):
        # Verification only: this state commands nothing. It exists
        # because the climb's start pose is not checked anywhere else,
        # and a lane error at the foot is a lane error all the way up.
        #
        # Two of the three checks are worth being precise about. The
        # LANE check is implied by NAVIGATE_TO_RAMP's region check at the
        # instant the leg finished — the pre-ramp goal *is* the lane
        # centre — so it only bites if the robot moved between arriving
        # and starting the climb, which a wind-down or a collision
        # monitor nudge can do. The HEADING is genuinely new information
        # — the region check ignores yaw entirely — but it is REPORTED
        # rather than gated, because no threshold for it has been
        # measured and the obvious candidate aborts good missions. See
        # GOAL_YAW_TOLERANCE.
        pose = self._fresh_pose(obs)
        if pose is None or not obs.ramp.newer_than(self.entered_at):
            return RUNNING
        x, y, yaw = pose
        if abs(y - self.plan.lane) > self.plan.lane_tolerance:
            return (FAILURE, ALIGN_OFF_LANE,
                    f'y={y:+.2f} vs lane {self.plan.lane:+.2f}')
        # Recorded whether or not it is gated on, so the number is in the
        # log and in the tests even when nothing fails on it.
        self.align_yaw = _wrap(yaw)
        if (self.plan.yaw_tolerance is not None
                and abs(self.align_yaw) > self.plan.yaw_tolerance):
            return (FAILURE, ALIGN_HEADING,
                    f'yaw={self.align_yaw:+.2f} rad, tolerance '
                    f'{self.plan.yaw_tolerance:.2f}')
        if x > RAMP_FOOT_X:
            return (FAILURE, ALIGN_NOT_ON_FLAT,
                    f'x={x:.2f} is past the ramp foot {RAMP_FOOT_X:.2f}')
        return SUCCESS

    def _check_climb(self, obs):
        return self._check_worker(
            obs, '/ramp/climb', obs.ramp, 'segment',
            good=('goal',),
            reasons={'tipped': CLIMB_TIPPED, 'timeout': CLIMB_TIMEOUT},
            default=CLIMB_FAILED)

    def _check_descend(self, obs):
        return self._check_worker(
            obs, '/ramp/descend', obs.ramp, 'segment',
            good=('goal',),
            reasons={'tipped': DESCENT_TIPPED, 'timeout': DESCENT_TIMEOUT},
            default=DESCENT_FAILED)

    def _check_stow_arm(self, obs):
        return self._check_worker(
            obs, '/grasp/stow', obs.grasp, 'phase',
            good=('done',), reasons={}, default=STOW_FAILED)

    def _check_approach_target(self, obs):
        return self._check_worker(
            obs, '/approach/run', obs.approach, 'phase',
            good=('arrived',), reasons={}, default=APPROACH_FAILED)

    def _check_grasp(self, obs):
        return self._check_worker(
            obs, '/grasp/pick', obs.grasp, 'phase',
            good=('held',), reasons={}, default=GRASP_FAILED)

    def _check_place(self, obs):
        return self._check_worker(
            obs, '/grasp/place', obs.grasp, 'phase',
            good=('placed',), reasons={}, default=PLACE_FAILED)

    def _check_verify_climb(self, obs):
        pose = self._fresh_pose(obs)
        if pose is None or not obs.ramp.newer_than(self.entered_at):
            return RUNNING
        x, y, _ = pose
        floor = self.plan.climb_end_x - self.plan.xy_tolerance
        if x < floor:
            return (FAILURE, CLIMB_POSE_UNVERIFIED,
                    f'x={x:.2f}, expected >= {floor:.2f}')
        # ramp_driver publishes cross-track against the target lane; fall
        # back to computing it from the world pose if no lane was known
        # to it, which is honest rather than assuming zero.
        lateral = obs.ramp.get('lateral')
        cross = (y - self.plan.lane if missing(lateral)
                 else _as_float(lateral))
        if cross is not None and abs(cross) > self.plan.lane_tolerance:
            return FAILURE, CLIMB_OFF_LANE, f'cross-track {cross:+.2f} m'
        return SUCCESS

    def _check_search_target(self, obs):
        if not obs.perception.newer_than(self.entered_at):
            return RUNNING
        selected = obs.perception.get('sel')
        if (not missing(selected) and self.plan.colour
                and selected != self.plan.colour):
            return (FAILURE, TARGET_COLOUR_MISMATCH,
                    f'vision is selecting {selected}, mission wants '
                    f'{self.plan.colour}')
        if obs.perception.get('found') == '1':
            return SUCCESS
        return RUNNING

    def _check_verify_grasp(self, obs):
        if not obs.grasp.newer_than(self.entered_at):
            return RUNNING
        lifted = obs.grasp.get('lifted')
        if lifted == '1':
            return SUCCESS
        if lifted == '0':
            return (FAILURE, GRASP_UNVERIFIED,
                    'the pick returned but lifted=0')
        return RUNNING

    def _check_verify_placement(self, obs):
        if not obs.grasp.newer_than(self.entered_at):
            return RUNNING
        lifted = obs.grasp.get('lifted')
        outcome = obs.grasp.get('outcome')
        if lifted == '0' and outcome == 'placed':
            return SUCCESS
        if lifted == '1':
            return (FAILURE, PLACEMENT_UNVERIFIED,
                    'the place returned but the object is still held')
        return RUNNING

    # ── C2-M5.1, localization ────────────────────────────────────────────
    def _localization_degraded(self, obs):
        """True only when the monitor has LATCHED a degradation.

        Three ways this is deliberately conservative:

        * ``localization_recovery`` off means the signal is published and
          not read. That is how Experiment 1 ran the monitor over a
          healthy mission with nothing able to act on it.
        * a monitor that is not on the graph reads as no evidence, never
          as bad news, so a stack launched without it behaves exactly as
          it did before C2-M5.1.
        * the latch itself is the monitor's, held for DEGRADED_HOLD_S.
          The executive never sees a single sample and so cannot trigger
          on one.
        """
        if not self.plan.localization_recovery:
            return False
        if not obs.localization.seen():
            return False
        return obs.localization.get('degraded') == '1'

    def _localization_healthy(self, obs):
        """True only when the monitor has LATCHED health.

        The resume gate, and the reason it is a separate latch: it is
        held for longer than the degradation latch, because resuming a
        robot that is still lost costs the mission while waiting costs a
        few seconds. Ground truth plays no part -- the executive's world
        pose is used to verify ARRIVAL, never to decide that
        localization recovered.
        """
        if not obs.localization.seen():
            return False
        return obs.localization.get('healthy') == '1'

    def _check_relocalize(self, obs):
        self._want(RELOCALIZE_GOAL, RELOCALIZE_SPIN_RAD)
        status = self._request_status(obs)
        if status == UNAVAILABLE:
            return (FAILURE, LOCALIZATION_RECOVERY_UNAVAILABLE,
                    'no spin action server')
        if status == REJECTED:
            return (FAILURE, LOCALIZATION_RECOVERY_FAILED, 'spin rejected')
        # The motion must FINISH before the mission may resume, and that
        # is a measured requirement rather than tidiness. Experiment 2
        # resumed the instant health latched, 6.4 s in, while the 2*pi
        # spin was still turning; re-issuing the nav goal cancelled the
        # spin mid-rotation, and Nav2 aborted the fresh goal 2.2 s later.
        # A robot that is still executing its recovery is not a robot
        # ready to be given somewhere to go.
        if status in (None, PENDING, ACCEPTED):
            return RUNNING

        # Then, and only then, the health monitor decides. The spin
        # finishing is NOT the success condition and neither is the spin
        # succeeding -- C2-M5.0's requirement 5 is that consistency be
        # re-established and HOLD, on mapped ground. A spin that returns
        # SUCCEEDED with the robot still lost leaves this state RUNNING
        # until it times out, which is the correct outcome: the motion is
        # a means, and the monitor is the verdict.
        if self._localization_healthy(obs):
            return SUCCESS
        if status in (ABORTED, CANCELED):
            # The motion itself failed and health never came back. Say
            # which, rather than letting it read as a timeout.
            return (FAILURE, LOCALIZATION_RECOVERY_FAILED,
                    f'spin {status}, health still '
                    f'{obs.localization.get("verdict") or "unseen"}')
        return RUNNING

    def _check_recovery(self, obs):
        self._want(STOP_ALL, None)
        status = self._request_status(obs)
        if status not in (SUCCEEDED, UNAVAILABLE):
            return RUNNING
        # The completion condition is the arbiter's own report that
        # nothing is driving the wheels — an existing, measured
        # observable, not a dwell. If the arbiter is not on the graph at
        # all, the stops having been acknowledged is all there is.
        if not obs.arbiter.seen():
            return SUCCESS
        if not obs.arbiter.newer_than(self.entered_at):
            return RUNNING
        active = obs.arbiter.get('active')
        return SUCCESS if missing(active) else RUNNING

    # COMPLETE and ABORT have no check: `update` short-circuits on a
    # terminal state before dispatching, which is what makes them
    # terminal rather than merely quiet.

    # ── shared machinery ─────────────────────────────────────────────────
    def _fresh_pose(self, obs):
        """The world pose, if one arrived after this state was entered.

        Verifying where the robot ended up against a pose sampled before
        the leg began is not a verification, and odometry arrives far
        faster than any state's timeout — so waiting for a new sample
        costs a tick and buys the check its meaning.
        """
        if obs.pose is None or obs.pose_stamp is None:
            return None
        return obs.pose if obs.pose_stamp > self.entered_at else None

    def _request_status(self, obs):
        """The node's report on THIS state's request, or None."""
        if obs.request_token != self._token:
            return None
        return obs.request_status

    def _check_worker(self, obs, service, view, phase_key, good, reasons,
                      default):
        """
        Call a Trigger service and wait for its worker to finish.

        The completion condition is not "the phase reads idle": all three
        servers read idle for a moment *after* the service returns and
        *before* the worker thread picks the job up, so a naive check
        declares success instantly. traverse_demo papered over that with
        a blind 2 s dwell. Two things must hold instead:

        1. a status line arrived after the service was accepted, and
        2. the worker was seen busy, or it published a terminal outcome
           different from the one latched when the call went out.

        The second half covers a worker that finishes between two 5 Hz
        status samples; the first stops a pre-request line satisfying it.
        """
        self._want(CALL_SERVICE, service)
        status = self._request_status(obs)
        if status == UNAVAILABLE:
            return FAILURE, SERVICE_UNAVAILABLE, f'{service} not there'
        if status == REJECTED:
            return FAILURE, SERVICE_REFUSED, f'{service} refused'
        if status != ACCEPTED:
            return RUNNING

        if self._accepted_at is None:
            self._accepted_at = obs.ros_now
            self._latched_outcome = view.get('outcome')

        if not view.newer_than(self._accepted_at):
            return RUNNING

        phase = view.get(phase_key)
        outcome = view.get('outcome')
        if not missing(phase) and phase != 'idle':
            self._saw_busy = True
            return RUNNING

        progressed = self._saw_busy or (
            not missing(outcome) and outcome != self._latched_outcome)
        if not progressed:
            return RUNNING

        if outcome in good:
            return SUCCESS
        if missing(outcome):
            return RUNNING
        return (FAILURE, reasons.get(outcome, default),
                f'{service} finished with outcome={outcome}')

    # ── reporting ────────────────────────────────────────────────────────
    def status_line(self, now, event='run'):
        """
        ``/mission/state``'s payload: one key=value line.

        The same shape every other status topic in this project uses, so
        the HUD, the panel and ``ros2 topic echo`` all read it with the
        parser they already have. It replaces the free-text step label
        C2-M1 published, which no consumer could do anything with beyond
        printing it.
        """
        contract = self.contract
        timeout = ('--' if contract.timeout is None
                   else f'{contract.timeout:.0f}')
        return (
            f'state={self.state} '
            f'prev={self.previous or "--"} '
            f'event={event} '
            f'elapsed={self.elapsed(now):.1f} '
            f'timeout={timeout} '
            f'attempt={self.attempt()} '
            f'retries={contract.max_retries} '
            f'owner={contract.owner} '
            f'mode={contract.mode} '
            f'reason={self.reason or "--"} '
            f'result={self.result or "--"}')


def _wrap(angle):
    """Fold an angle onto [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _as_float(text):
    """Parse a status field as a float, or None if it is not one."""
    try:
        return float(text)
    except (TypeError, ValueError):
        return None
