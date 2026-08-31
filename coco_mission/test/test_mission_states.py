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
Tests for the mission executive's state machine.

The machine is pure, so all of this runs with no ROS graph, no simulator
and no wall clock: :class:`Harness` below is a scripted world, and time
is whatever the test says it is. That is the point of the split — a
transition table you cannot test without Gazebo is a transition table
nobody tests.

The harness republishes every status line on every tick, at 5 Hz-ish, so
the tests exercise the same "has the worker picked the job up yet"
ambiguity the real servers create rather than a tidier version of it.
"""

import math
import os
import re
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import localization_health as lh  # noqa: E402
import mission_hud  # noqa: E402
import mission_states as ms  # noqa: E402


IDLE_LINES = {
    'ramp': 'segment=idle step=0 progress=0.00 lateral=-- disp=+0.00 '
            'pitch=-- outcome=none',
    'approach': 'phase=idle tx=-- ty=-- range=-- bearing=-- stop=0.154 '
                'travel=0.000 colour=blue outcome=--',
    'grasp': 'phase=idle colour=blue x=-- y=-- lifted=0 outcome=--',
    'perception': 'sel=blue found=0 u=-- v=-- area=0 seen=-- age=0.05',
    'arbiter': 'mode=idle active=none teleop=-- nav=-- rl=-- approach=--',
    # C2-M5.1. The monitor present and reporting a healthy robot, which
    # is what it reports for the whole of a nominal mission — so every
    # test that predates C2-M5.1 runs with the signal live and unchanged.
    'localization': 'verdict=CONSISTENT reason=OK degraded=0 healthy=1 '
                    'held=9.00 d=0.053 near=0.875 beams=57 mapped=1 '
                    'sigma=0.376 mo_age=-0.44',
}


class Harness:
    """A scripted world for one MissionMachine.

    Emulates exactly what the node does: build an observation, hand it to
    the machine, and perform whatever single request comes back. Requests
    are accepted by default; a test that wants a rejection or an abort
    says so.
    """

    def __init__(self, plan=None, stall_limit=10.0, dt=0.1):
        self.machine = ms.MissionMachine(
            plan or ms.MissionPlan('blue'), stall_limit=stall_limit)
        self.now = 0.0
        self.wall = 0.0
        self.dt = dt
        self.started = True
        self.colour = 'blue'
        self.abort_requested = False
        self.pose = (-2.0, 0.0, 0.0)
        self.localized = True
        self.lines = dict(IDLE_LINES)
        self.silent = set()      # topics with no publisher at all
        self.token = None
        self.status = None
        self.reply = {}          # payload -> status to report on send
        self.transitions = []
        self.requests = []
        self.freeze_ros = False

    # ── the world ────────────────────────────────────────────────────────
    def publish(self, key, line):
        """Replace one subsystem's status line."""
        self.lines[key] = line

    def views(self):
        return {key: (ms.WorkerView() if key in self.silent
                      else ms.WorkerView.from_line(line, self.now))
                for key, line in self.lines.items()}

    def observation(self):
        return ms.Observation(
            ros_now=self.now, wall_now=self.wall,
            started=self.started, abort_requested=self.abort_requested,
            colour=self.colour, pose=self.pose, pose_stamp=self.now,
            localized=self.localized,
            request_token=self.token, request_status=self.status,
            **self.views())

    # ── the loop ─────────────────────────────────────────────────────────
    def tick(self, count=1):
        """Advance `count` updates, performing whatever is asked."""
        directive = None
        for _ in range(count):
            directive = self.machine.update(self.observation())
            for event in directive.events:
                self.transitions.append(
                    (event.previous, event.state, event.reason))
            request = directive.request
            if request is not None and request.token != self.token:
                self.token = request.token
                self.requests.append((request.kind, request.payload))
                self.status = self.reply.get(
                    request.payload,
                    ms.SUCCEEDED if request.kind == ms.STOP_ALL
                    else ms.ACCEPTED)
            if not self.freeze_ros:
                self.now += self.dt
            self.wall += self.dt
        return directive

    @property
    def state(self):
        return self.machine.state

    def states(self):
        """Just the states entered, in order."""
        return [entered for _, entered, _ in self.transitions]

    def reasons(self):
        """Every failure reason recorded, in order."""
        return [reason for _, _, reason in self.transitions if reason]

    # ── scripted subsystems ──────────────────────────────────────────────
    def worker(self, key, phase_key, busy, outcome, extra='', ticks=3,
               pose=None):
        """Run a subsystem: go busy for a while, then finish.

        `pose` is where the robot ends up, applied with the completion
        line rather than after it. A subsystem that drives has moved the
        robot by the time it reports done, and the verification state
        that follows must see the pose it actually finished at.
        """
        self.publish(key, f'{phase_key}={busy} {extra} outcome=--'.strip())
        self.tick(ticks)
        if pose is not None:
            self.pose = pose
        self.publish(key, f'{phase_key}=idle {extra} outcome={outcome}')
        self.tick(ticks)

    def nav_arrives(self, x, y, yaw=0.0, ticks=2):
        """Report the goal reached, with the robot actually there."""
        self.pose = (x, y, yaw)
        self.status = ms.SUCCEEDED
        self.tick(ticks)

    def recover(self, ticks=4):
        """Let RECOVERY finish: the arbiter reports nothing driving."""
        self.publish('arbiter', 'mode=idle active=none')
        self.tick(ticks)

    # ── C2-M5.1, the localization signal ─────────────────────────────────
    def degrade(self, ticks=0):
        """The monitor latches a degradation.

        Note what this does NOT model: the persistence window. That lives
        in localization_health.Persistence and is tested there, against
        the measured excursion durations. What crosses into the executive
        is the already-latched flag, and a test that re-implemented the
        holding here would be testing its own copy of the rule.
        """
        self.publish('localization',
                     'verdict=INCONSISTENT reason=SCAN_DISAGREES '
                     'degraded=1 healthy=0 held=2.10 d=0.492 near=0.233 '
                     'beams=54 mapped=1 sigma=0.476 mo_age=-0.44')
        if ticks:
            self.tick(ticks)

    def heal(self, ticks=0):
        """The monitor latches health again."""
        self.publish('localization', IDLE_LINES['localization'])
        if ticks:
            self.tick(ticks)

    def unhealed(self, ticks=0):
        """Still bad, but no longer latched degraded.

        The state RELOCALIZE sits in while a spin is running and has not
        yet worked: neither flag set, so neither the trigger nor the
        resume gate fires.
        """
        self.publish('localization',
                     'verdict=INCONSISTENT reason=SCAN_DISAGREES '
                     'degraded=0 healthy=0 held=0.40 d=0.455 near=0.240 '
                     'beams=54 mapped=1 sigma=0.470 mo_age=-0.44')
        if ticks:
            self.tick(ticks)


def run_to_climb(harness, lane=0.25):
    """IDLE through a clean climb, leaving the machine in SEARCH_TARGET."""
    harness.tick(3)                                    # IDLE, LOCALIZE
    harness.nav_arrives(ms.PRE_RAMP_X, lane)           # NAVIGATE_TO_RAMP
    harness.tick(2)                                    # ALIGN_FOR_CLIMB
    harness.worker('ramp', 'segment', 'climb', 'goal',
                   extra='lateral=+0.02 disp=+0.01',
                   pose=(ms.CLIMB_END_X, lane, 0.0))
    harness.tick(2)                                    # VERIFY_CLIMB
    return harness


def run_nominal(harness, lane=0.25):
    """The whole fetch, every subsystem behaving."""
    run_to_climb(harness, lane)
    harness.publish('perception', 'sel=blue found=1 range=1.198 seen=blue')
    harness.tick(2)                                    # SEARCH_TARGET
    harness.worker('grasp', 'phase', 'stow', 'done', extra='lifted=0')
    harness.worker('approach', 'phase', 'servo', 'arrived')
    harness.worker('grasp', 'phase', 'pick:hover', 'held', extra='lifted=1')
    harness.tick(2)                                    # VERIFY_GRASP
    harness.worker('ramp', 'segment', 'descend', 'goal',
                   extra='lateral=+0.02 disp=+0.01')
    harness.nav_arrives(*ms.HOME)                      # RETURN_HOME
    harness.worker('grasp', 'phase', 'place:lift', 'placed', extra='lifted=0')
    harness.tick(2)                                    # VERIFY_PLACEMENT
    return harness


class TestContractTable:
    """The table itself, before any state runs."""

    def test_every_nominal_state_has_a_contract(self):
        for state in ms.NOMINAL_NEXT:
            assert state in ms.CONTRACTS
            assert ms.NOMINAL_NEXT[state] in ms.CONTRACTS

    def test_every_retry_target_is_a_real_state(self):
        for contract in ms.CONTRACTS.values():
            assert contract.retry_state in ms.CONTRACTS

    def test_retries_are_bounded_and_small(self):
        # An unbounded retry is an unbounded mission. Two is the largest
        # count any state was given, and it was given deliberately.
        for contract in ms.CONTRACTS.values():
            assert 0 <= contract.max_retries <= 2

    def test_every_acting_state_has_a_timeout(self):
        untimed = [name for name, contract in ms.CONTRACTS.items()
                   if contract.timeout is None]
        assert sorted(untimed) == sorted([ms.IDLE, ms.COMPLETE, ms.ABORT])

    def test_every_state_names_the_owner_of_the_wheels(self):
        for contract in ms.CONTRACTS.values():
            assert contract.owner
            assert contract.mode in ('idle', 'nav', 'rl', 'approach')

    def test_platform_states_come_home_rather_than_abort(self):
        # A robot parked on the 0.65 m deck needs hands. Every state that
        # runs up there must exhaust into the descent, not into ABORT.
        for state in ms.PLATFORM_STATES:
            assert (ms.CONTRACTS[state].on_exhausted
                    == ms.ESCALATE_SKIP_GRASP)

    def test_the_two_ramp_segments_are_never_retried(self):
        # Re-entering a PPO episode from halfway up has never been
        # measured. If that changes, it changes with a measurement.
        assert ms.CONTRACTS[ms.CLIMB].max_retries == 0
        assert ms.CONTRACTS[ms.DESCEND].max_retries == 0

    def test_the_climb_goal_margin_matches_ramp_env(self):
        """CLIMB_END_X is derived; ramp_env owns the margin it derives from."""
        source = os.path.join(
            os.path.dirname(__file__), '..', '..',
            'coco_rl', 'coco_rl', 'ramp_env.py')
        if not os.path.exists(source):
            pytest.skip('ramp_env.py is not beside this package')
        match = re.search(r'^GOAL_MARGIN = ([0-9.]+)',
                          open(source).read(), re.M)
        assert match, 'ramp_env no longer defines GOAL_MARGIN'
        assert float(match.group(1)) == ms.CLIMB_GOAL_MARGIN


class TestNominalSequence:
    """The mission when nothing goes wrong."""

    def test_the_full_fetch_reaches_complete(self):
        harness = run_nominal(Harness())
        assert harness.state == ms.COMPLETE
        assert harness.machine.result == 'fetch'
        assert harness.states() == [
            ms.LOCALIZE, ms.NAVIGATE_TO_RAMP, ms.ALIGN_FOR_CLIMB, ms.CLIMB,
            ms.VERIFY_CLIMB, ms.SEARCH_TARGET, ms.STOW_ARM,
            ms.APPROACH_TARGET, ms.GRASP, ms.VERIFY_GRASP, ms.DESCEND,
            ms.RETURN_HOME, ms.PLACE, ms.VERIFY_PLACEMENT, ms.COMPLETE]

    def test_it_asked_for_exactly_the_existing_interfaces(self):
        harness = run_nominal(Harness())
        assert harness.requests == [
            (ms.NAV_GOAL, (ms.PRE_RAMP_X, 0.25)),
            (ms.CALL_SERVICE, '/ramp/climb'),
            (ms.CALL_SERVICE, '/grasp/stow'),
            (ms.CALL_SERVICE, '/approach/run'),
            (ms.CALL_SERVICE, '/grasp/pick'),
            (ms.CALL_SERVICE, '/ramp/descend'),
            (ms.NAV_GOAL, ms.HOME),
            (ms.CALL_SERVICE, '/grasp/place'),
        ]

    def test_it_never_asks_for_a_velocity(self):
        # The executive orchestrates; cmd_vel_arbiter drives. There are
        # exactly three things it can ask for and none of them is a twist.
        harness = run_nominal(Harness())
        for kind, _ in harness.requests:
            assert kind in (ms.NAV_GOAL, ms.CALL_SERVICE, ms.STOP_ALL)

    def test_the_wheels_change_hands_in_the_documented_order(self):
        harness = Harness()
        modes = []
        run = run_nominal
        original = harness.machine.update

        def record(obs):
            directive = original(obs)
            modes.append(directive.mode)
            return directive

        harness.machine.update = record
        run(harness)
        # Duplicates removed: the sequence of OWNERS, not of ticks.
        collapsed = [mode for i, mode in enumerate(modes)
                     if i == 0 or mode != modes[i - 1]]
        assert collapsed == ['idle', 'nav', 'idle', 'rl', 'idle',
                             'approach', 'idle', 'rl', 'nav', 'idle']

    def test_traverse_only_skips_the_platform_and_completes(self):
        harness = Harness(ms.MissionPlan('blue', do_grasp=False))
        run_to_climb(harness)
        harness.worker('ramp', 'segment', 'descend', 'goal',
                       extra='lateral=+0.01 disp=+0.01')
        harness.nav_arrives(*ms.HOME)
        assert harness.state == ms.COMPLETE
        assert harness.machine.result == 'traverse'
        for state in ms.PLATFORM_STATES:
            assert state not in harness.states()


class TestEntryConditions:
    """A state does not run because the last one returned True."""

    def test_idle_will_not_start_without_a_colour(self):
        harness = Harness()
        harness.colour = None
        harness.tick(20)
        assert harness.state == ms.IDLE

    def test_idle_waits_for_the_start_request(self):
        harness = Harness()
        harness.started = False
        harness.tick(20)
        assert harness.state == ms.IDLE
        harness.started = True
        harness.tick(1)
        assert harness.state == ms.LOCALIZE

    def test_localize_names_the_input_that_never_arrived(self):
        harness = Harness()
        harness.localized = False
        harness.dt = 5.0
        harness.tick(12)                       # past the 40 s budget
        assert harness.machine.reason == ms.NO_LOCALIZATION

    def test_localize_blames_odometry_when_odometry_is_missing(self):
        harness = Harness()
        harness.pose = None
        harness.dt = 5.0
        harness.tick(12)
        assert harness.machine.reason == ms.NO_ODOMETRY

    def test_localize_blames_the_ramp_driver_when_it_is_absent(self):
        harness = Harness()
        harness.silent.add('ramp')
        harness.dt = 5.0
        harness.tick(12)
        assert harness.machine.reason == ms.NO_RAMP_DRIVER


class TestSuccessVerification:
    """An action returning success is not the same as the job being done."""

    def test_nav_success_with_the_robot_elsewhere_is_a_failure(self):
        harness = Harness()
        harness.tick(3)
        assert harness.state == ms.NAVIGATE_TO_RAMP
        harness.nav_arrives(ms.PRE_RAMP_X + 1.4, 0.25)
        assert harness.machine.reason == ms.PRE_RAMP_POSE_OUT_OF_REGION
        assert harness.state == ms.RECOVERY

    def test_a_climb_that_reports_goal_from_the_wrong_place_fails(self):
        harness = Harness()
        harness.tick(3)
        harness.nav_arrives(ms.PRE_RAMP_X, 0.25)
        harness.tick(2)
        # Still at the ramp foot, and the driver says it finished.
        harness.worker('ramp', 'segment', 'climb', 'goal',
                       extra='lateral=+0.02', pose=(1.2, 0.25, 0.0))
        harness.tick(2)
        assert harness.machine.reason == ms.CLIMB_POSE_UNVERIFIED

    def test_a_climb_that_drifts_off_the_lane_fails(self):
        # The 0.51 m drift that outcome=goal happily reported.
        harness = Harness()
        harness.tick(3)
        harness.nav_arrives(ms.PRE_RAMP_X, 0.25)
        harness.tick(2)
        harness.worker('ramp', 'segment', 'climb', 'goal',
                       extra='lateral=+0.51',
                       pose=(ms.CLIMB_END_X, 0.76, 0.0))
        harness.tick(2)
        assert harness.machine.reason == ms.CLIMB_OFF_LANE

    def test_a_pick_that_returns_held_without_lifting_fails(self):
        harness = run_to_climb(Harness())
        harness.publish('perception', 'sel=blue found=1 seen=blue')
        harness.tick(2)
        harness.worker('grasp', 'phase', 'stow', 'done', extra='lifted=0')
        harness.worker('approach', 'phase', 'servo', 'arrived')
        harness.worker('grasp', 'phase', 'pick:hover', 'held',
                       extra='lifted=0')
        harness.tick(2)
        assert harness.machine.reason == ms.GRASP_UNVERIFIED

    def test_a_place_that_leaves_the_object_held_fails(self):
        harness = Harness()
        run_to_climb(harness)
        harness.publish('perception', 'sel=blue found=1 seen=blue')
        harness.tick(2)
        harness.worker('grasp', 'phase', 'stow', 'done', extra='lifted=0')
        harness.worker('approach', 'phase', 'servo', 'arrived')
        harness.worker('grasp', 'phase', 'pick:hover', 'held',
                       extra='lifted=1')
        harness.tick(2)
        harness.worker('ramp', 'segment', 'descend', 'goal', extra='lateral=0')
        harness.nav_arrives(*ms.HOME)
        harness.worker('grasp', 'phase', 'place:lift', 'placed',
                       extra='lifted=1')
        harness.tick(2)
        assert harness.machine.reason == ms.PLACEMENT_UNVERIFIED

    def test_vision_selecting_a_different_colour_is_named(self):
        harness = run_to_climb(Harness())
        harness.publish('perception', 'sel=green found=1 seen=green')
        harness.tick(2)
        assert harness.machine.reason == ms.TARGET_COLOUR_MISMATCH

    def test_a_worker_reading_idle_before_it_starts_is_not_success(self):
        # Every server reads phase=idle between the service returning and
        # its worker thread picking the job up. traverse_demo covered
        # that with a blind 2 s dwell; this covers it with a condition.
        harness = Harness()
        harness.tick(3)
        harness.nav_arrives(ms.PRE_RAMP_X, 0.25)
        harness.tick(2)
        assert harness.state == ms.CLIMB
        harness.tick(20)          # ramp still says idle, outcome unchanged
        assert harness.state == ms.CLIMB


class TestFailureTimeoutAndRetry:
    """Structured reasons, bounded retries, and an escalation that ends."""

    def test_a_nav_abort_retries_the_leg(self):
        harness = Harness()
        harness.tick(3)
        harness.status = ms.ABORTED
        harness.tick(2)
        assert harness.state == ms.RECOVERY
        assert harness.machine.reason == ms.NAVIGATION_FAILED
        harness.recover()
        assert harness.state == ms.NAVIGATE_TO_RAMP
        assert harness.machine.attempt(ms.NAVIGATE_TO_RAMP) == 2

    def test_a_nav_leg_gives_up_after_two_retries(self):
        harness = Harness()
        harness.tick(3)
        for _ in range(3):
            harness.status = ms.ABORTED
            harness.tick(2)
            harness.recover()
        assert harness.state == ms.ABORT
        assert harness.machine.reason == ms.NAVIGATION_FAILED
        assert harness.machine.result == 'aborted'
        assert harness.machine.attempts[ms.NAVIGATE_TO_RAMP] == 2

    def test_a_nav_timeout_is_not_a_nav_failure(self):
        harness = Harness(dt=30.0)
        harness.tick(3)
        assert harness.state == ms.NAVIGATE_TO_RAMP
        harness.tick(10)                       # past 240 s
        assert ms.NAVIGATION_TIMEOUT in harness.reasons()

    def test_a_missing_service_is_reported_as_missing(self):
        harness = Harness()
        harness.reply['/ramp/climb'] = ms.UNAVAILABLE
        harness.tick(3)
        harness.nav_arrives(ms.PRE_RAMP_X, 0.25)
        harness.tick(3)
        assert harness.machine.reason == ms.SERVICE_UNAVAILABLE

    def test_a_refused_service_is_distinct_from_a_missing_one(self):
        harness = Harness()
        harness.reply['/ramp/climb'] = ms.REJECTED
        harness.tick(3)
        harness.nav_arrives(ms.PRE_RAMP_X, 0.25)
        harness.tick(3)
        assert harness.machine.reason == ms.SERVICE_REFUSED

    def test_a_tipped_climb_aborts_without_retrying(self):
        harness = Harness()
        harness.tick(3)
        harness.nav_arrives(ms.PRE_RAMP_X, 0.25)
        harness.tick(2)
        harness.worker('ramp', 'segment', 'climb', 'tipped')
        assert harness.machine.reason == ms.CLIMB_TIPPED
        harness.recover()
        assert harness.state == ms.ABORT

    def test_a_descent_timeout_carries_its_own_reason(self):
        # KNOWN PROBLEMS 3b: the descent timed out in both C2-M1.6 runs
        # and the log said only "FAILED at: 5. scripted descent".
        harness = run_to_climb(Harness(ms.MissionPlan('blue',
                                                      do_grasp=False)))
        harness.worker('ramp', 'segment', 'descend', 'timeout')
        assert harness.machine.reason == ms.DESCENT_TIMEOUT
        harness.recover()
        assert harness.state == ms.ABORT

    def test_a_bad_pre_climb_pose_re_drives_the_leg(self):
        harness = Harness()
        harness.tick(3)
        harness.nav_arrives(ms.PRE_RAMP_X, 0.25, ticks=1)
        assert harness.state == ms.ALIGN_FOR_CLIMB
        # Nav2 called it arrived; the robot then settled onto the lane
        # boundary, and the climb would start crooked.
        harness.pose = (ms.PRE_RAMP_X, 0.60, 0.0)
        harness.tick(3)
        assert harness.machine.reason == ms.ALIGN_OFF_LANE
        harness.recover()
        assert harness.state == ms.NAVIGATE_TO_RAMP

    def test_a_bad_heading_fails_only_when_the_gate_is_switched_on(self):
        plan = ms.MissionPlan('blue', yaw_tolerance=0.25)
        harness = Harness(plan)
        harness.tick(3)
        harness.nav_arrives(ms.PRE_RAMP_X, 0.25, yaw=0.9)
        harness.tick(3)
        assert harness.machine.reason == ms.ALIGN_HEADING

    def test_the_heading_gate_is_off_by_default(self):
        # Measured, C2-M3.0: a live leg arrived at +0.28 rad and, re-driven,
        # at +0.26 -- both inside Nav2's own yaw_goal_tolerance, both
        # outside a 0.25 rad ground-truth gate, and the mission it aborted
        # is the one that completes 19/20. No threshold has been measured,
        # so none is asserted.
        assert ms.GOAL_YAW_TOLERANCE is None
        assert ms.MissionPlan('blue').yaw_tolerance is None
        harness = Harness()
        harness.tick(3)
        harness.nav_arrives(ms.PRE_RAMP_X, 0.25, yaw=0.28)
        harness.tick(3)
        assert harness.state == ms.CLIMB
        assert harness.machine.reason is None

    def test_the_heading_is_reported_even_when_it_is_not_gated(self):
        harness = Harness()
        harness.tick(3)
        harness.nav_arrives(ms.PRE_RAMP_X, 0.25, yaw=0.28)
        harness.tick(3)
        assert harness.machine.align_yaw == pytest.approx(0.28)

    def test_an_unverified_grasp_retries_the_grasp_itself(self):
        harness = run_to_climb(Harness())
        harness.publish('perception', 'sel=blue found=1 seen=blue')
        harness.tick(2)
        harness.worker('grasp', 'phase', 'stow', 'done', extra='lifted=0')
        harness.worker('approach', 'phase', 'servo', 'arrived')
        harness.worker('grasp', 'phase', 'pick:hover', 'held',
                       extra='lifted=0')
        harness.tick(2)
        assert ms.GRASP_UNVERIFIED in harness.reasons()
        harness.recover()
        # RECOVERY sent it back to GRASP, not to VERIFY_GRASP: there is
        # nothing to re-verify until the pick has been tried again.
        assert harness.state == ms.GRASP
        assert harness.machine.attempt(ms.GRASP) == 1
        assert harness.machine.attempts[ms.VERIFY_GRASP] == 1

    def test_a_lost_target_comes_home_empty_rather_than_aborting(self):
        harness = run_to_climb(Harness())
        harness.dt = 5.0
        for _ in range(3):                     # three 15 s windows
            harness.tick(6)
            harness.recover(ticks=3)
        assert harness.machine.degraded_reason == ms.TARGET_NOT_FOUND
        assert harness.state == ms.DESCEND
        assert harness.machine.attempts[ms.SEARCH_TARGET] == 2

    def test_coming_home_empty_ends_in_abort_carrying_the_reason(self):
        harness = run_to_climb(Harness())
        harness.dt = 5.0
        for _ in range(3):
            harness.tick(6)
            harness.recover(ticks=3)
        harness.dt = 0.1
        harness.worker('ramp', 'segment', 'descend', 'goal', extra='lateral=0')
        harness.nav_arrives(*ms.HOME)
        assert harness.state == ms.ABORT
        assert harness.machine.reason == ms.TARGET_NOT_FOUND
        assert harness.machine.result == 'aborted'
        # And it did come down: PLACE was never entered, DESCEND was.
        assert ms.DESCEND in harness.states()
        assert ms.PLACE not in harness.states()

    def test_a_failing_grasp_gives_up_after_two_retries(self):
        harness = run_to_climb(Harness())
        harness.publish('perception', 'sel=blue found=1 seen=blue')
        harness.tick(2)
        harness.worker('grasp', 'phase', 'stow', 'done', extra='lifted=0')
        harness.worker('approach', 'phase', 'servo', 'arrived')
        for attempt in range(3):
            harness.worker('grasp', 'phase', 'pick:hover',
                           f'failed at hover {attempt}', extra='lifted=0')
            harness.recover()
        assert harness.machine.attempts[ms.GRASP] == 2
        assert harness.machine.degraded_reason == ms.GRASP_FAILED
        assert harness.state == ms.DESCEND


class TestRecovery:
    """RECOVERY stops the robot before it decides anything."""

    def test_recovery_asks_for_the_stop(self):
        harness = Harness()
        harness.tick(3)
        harness.status = ms.ABORTED
        harness.tick(2)
        assert (ms.STOP_ALL, None) in harness.requests

    def test_recovery_waits_for_the_arbiter_to_report_nothing_driving(self):
        harness = Harness()
        harness.tick(3)
        harness.status = ms.ABORTED
        harness.tick(2)
        harness.publish('arbiter', 'mode=nav active=nav')
        harness.tick(10)
        assert harness.state == ms.RECOVERY   # still something driving
        harness.publish('arbiter', 'mode=idle active=none')
        harness.tick(3)
        assert harness.state == ms.NAVIGATE_TO_RAMP

    def test_a_recovery_that_never_settles_aborts(self):
        harness = Harness(dt=3.0)
        harness.tick(3)
        harness.status = ms.ABORTED
        harness.tick(2)
        harness.publish('arbiter', 'mode=rl active=rl')
        harness.tick(12)                       # past the 20 s budget
        assert harness.state == ms.ABORT
        assert harness.machine.reason == ms.RECOVERY_TIMEOUT

    def test_an_operator_abort_skips_the_retry_budget(self):
        harness = Harness()
        harness.tick(3)
        assert harness.state == ms.NAVIGATE_TO_RAMP
        harness.abort_requested = True
        harness.tick(2)
        assert harness.state == ms.RECOVERY
        assert harness.machine.reason == ms.OPERATOR_ABORT
        harness.recover()
        assert harness.state == ms.ABORT

    def test_abort_keeps_asking_for_the_stop(self):
        harness = Harness()
        harness.tick(3)
        harness.abort_requested = True
        harness.tick(2)
        harness.recover()
        assert harness.state == ms.ABORT
        directive = harness.tick(1)
        assert directive.request.kind == ms.STOP_ALL
        assert directive.mode == 'idle'

    def test_a_frozen_simulation_clock_aborts(self):
        harness = Harness(stall_limit=5.0)
        harness.tick(3)
        assert harness.state == ms.NAVIGATE_TO_RAMP
        harness.freeze_ros = True              # /clock stops; wall does not
        harness.tick(80)
        assert harness.state == ms.ABORT
        assert harness.machine.reason == ms.CLOCK_STALLED

    def test_a_healthy_slow_run_is_not_a_stalled_clock(self):
        harness = Harness(stall_limit=5.0, dt=0.5)
        harness.tick(3)
        harness.tick(60)
        assert harness.machine.reason != ms.CLOCK_STALLED


class TestSafety:
    """Properties that must hold whatever the world does."""

    def test_a_world_where_everything_fails_still_terminates(self):
        harness = Harness(dt=2.0)
        harness.reply = {}
        harness.status = ms.ABORTED
        for _ in range(4000):
            harness.tick(1)
            if harness.status is not None and harness.status != ms.SUCCEEDED:
                harness.status = ms.ABORTED
            harness.publish('arbiter', 'mode=idle active=none')
            if harness.state in ms.TERMINAL_STATES:
                break
        assert harness.state in ms.TERMINAL_STATES

    def test_terminal_states_are_terminal(self):
        harness = run_nominal(Harness())
        assert harness.state == ms.COMPLETE
        before = list(harness.transitions)
        harness.tick(50)
        assert harness.transitions == before

    def test_no_state_can_be_entered_more_often_than_its_budget(self):
        harness = Harness(dt=2.0)
        harness.status = ms.ABORTED
        for _ in range(2000):
            harness.tick(1)
            harness.publish('arbiter', 'mode=idle active=none')
            if harness.state in ms.TERMINAL_STATES:
                break
        for state, contract in ms.CONTRACTS.items():
            if state in (ms.RECOVERY,) + ms.TERMINAL_STATES:
                continue
            entered = harness.states().count(state)
            assert entered <= contract.max_retries + 1, state


class TestDeterminism:
    """The same events must always produce the same transitions."""

    def test_the_nominal_run_is_reproducible(self):
        first = run_nominal(Harness())
        second = run_nominal(Harness())
        assert first.transitions == second.transitions
        assert first.requests == second.requests

    def test_a_failing_run_is_reproducible(self):
        def failing():
            harness = Harness()
            harness.tick(3)
            harness.status = ms.ABORTED
            harness.tick(2)
            harness.recover()
            harness.nav_arrives(ms.PRE_RAMP_X, 0.25)
            return harness
        assert failing().transitions == failing().transitions


class TestPublication:
    """/mission/state has to be machine-readable, not a printed label."""

    def test_the_line_parses_and_carries_the_contract(self):
        harness = Harness()
        harness.tick(3)
        fields = ms.parse_kv(harness.machine.status_line(harness.now))
        assert fields['state'] == ms.NAVIGATE_TO_RAMP
        assert fields['prev'] == ms.LOCALIZE
        assert fields['owner'] == 'nav2'
        assert fields['mode'] == 'nav'
        assert fields['timeout'] == '240'
        assert fields['attempt'] == '1'
        assert fields['retries'] == '2'
        assert fields['reason'] == '--'
        assert fields['result'] == '--'

    def test_a_recovery_line_carries_the_reason_and_the_attempt(self):
        harness = Harness()
        harness.tick(3)
        harness.status = ms.ABORTED
        harness.tick(2)
        fields = ms.parse_kv(harness.machine.status_line(harness.now))
        assert fields['state'] == ms.RECOVERY
        assert fields['reason'] == ms.NAVIGATION_FAILED
        assert fields['prev'] == ms.NAVIGATE_TO_RAMP

    def test_the_terminal_line_says_what_the_mission_achieved(self):
        harness = run_nominal(Harness())
        fields = ms.parse_kv(harness.machine.status_line(harness.now))
        assert fields['state'] == ms.COMPLETE
        assert fields['result'] == 'fetch'

    def test_the_event_field_distinguishes_a_transition(self):
        harness = Harness()
        harness.tick(3)
        line = harness.machine.status_line(harness.now, event='enter')
        assert ms.parse_kv(line)['event'] == 'enter'
        assert ms.parse_kv(
            harness.machine.status_line(harness.now))['event'] == 'run'

    def test_the_two_status_parsers_agree(self):
        # mission_states carries its own parse_kv so the pure core needs
        # no ROS node to read a string. Duplication is only safe while
        # the two agree.
        for line in list(IDLE_LINES.values()) + [
                'state=CLIMB prev=ALIGN_FOR_CLIMB reason=-- result=--',
                '', 'nonsense', 'a=1 b= =c d']:
            assert ms.parse_kv(line) == mission_hud.parse_kv(line)


class TestHudRendering:
    """The HUD has to read the new line without being redesigned."""

    def test_the_state_row_shows_the_state_and_its_budget(self):
        harness = Harness()
        harness.tick(3)
        rendered = mission_hud.format_mission_state(
            harness.machine.status_line(harness.now))
        assert rendered.startswith(ms.NAVIGATE_TO_RAMP)
        assert '240' in rendered

    def test_a_traverse_demo_label_still_renders(self):
        # traverse_demo.py is kept as the measured harness, and its free
        # text label has to stay readable on the same HUD.
        assert mission_hud.format_mission_state('2. RL climb') == '2. RL climb'

    def test_the_recovery_row_reads_none_until_something_fails(self):
        harness = Harness()
        harness.tick(3)
        row = mission_hud.format_recovery(
            harness.machine.status_line(harness.now), 'fallback')
        assert row.startswith('none')

    def test_the_recovery_row_names_the_reason(self):
        harness = Harness()
        harness.tick(3)
        harness.status = ms.ABORTED
        harness.tick(2)
        row = mission_hud.format_recovery(
            harness.machine.status_line(harness.now), 'fallback')
        assert ms.NAVIGATION_FAILED in row

    def test_the_recovery_row_falls_back_without_an_executive(self):
        assert mission_hud.format_recovery('2. RL climb', 'fallback') \
            == 'fallback'


class TestLocalizationRecovery:
    """C2-M5.1: degradation -> safe stop -> relocalize -> resume, or abort.

    Every one of these drives the machine through the SAME failure path
    every other failure uses. That is the point: C2-M5.1 added one state
    and one reason, and did not give localization a private escape route
    out of the contract table.
    """

    def to_return_home(self, harness=None):
        """A machine in RETURN_HOME with the object aboard."""
        harness = harness or Harness()
        run_to_climb(harness)
        harness.publish('perception', 'sel=blue found=1 range=1.198 seen=blue')
        harness.tick(2)
        harness.worker('grasp', 'phase', 'stow', 'done', extra='lifted=0')
        harness.worker('approach', 'phase', 'servo', 'arrived')
        harness.worker('grasp', 'phase', 'pick:hover', 'held', extra='lifted=1')
        harness.tick(2)
        harness.worker('ramp', 'segment', 'descend', 'goal',
                       extra='lateral=+0.02 disp=+0.01')
        assert harness.state == ms.RETURN_HOME
        return harness

    # ── the trigger ──────────────────────────────────────────────────────
    def test_a_healthy_monitor_never_interrupts_the_mission(self):
        harness = run_nominal(Harness())
        assert harness.state == ms.COMPLETE
        assert ms.RELOCALIZE not in harness.states()
        assert ms.LOCALIZATION_DEGRADED not in harness.reasons()

    def test_a_latched_degradation_fails_the_nav_leg(self):
        harness = self.to_return_home()
        harness.degrade(ticks=2)
        assert harness.state == ms.RECOVERY
        assert harness.machine.reason == ms.LOCALIZATION_DEGRADED
        assert harness.machine.failed_state == ms.RETURN_HOME

    def test_the_trigger_is_off_when_localization_recovery_is_off(self):
        plan = ms.MissionPlan('blue', localization_recovery=False)
        harness = self.to_return_home(Harness(plan=plan))
        harness.degrade(ticks=5)
        # The signal is published and read by nobody. This is exactly how
        # the C2-M5.1 false-positive experiment was run.
        assert harness.state == ms.RETURN_HOME
        assert ms.LOCALIZATION_DEGRADED not in harness.reasons()

    def test_a_monitor_that_is_not_running_never_triggers(self):
        harness = self.to_return_home()
        harness.silent.add('localization')
        harness.tick(5)
        assert harness.state == ms.RETURN_HOME

    def test_an_unlatched_bad_sample_does_not_trigger(self):
        # The persistence window is the monitor's, and the executive is
        # only ever handed the latched answer. A raw INCONSISTENT with
        # degraded=0 must do nothing at all.
        harness = self.to_return_home()
        harness.unhealed(ticks=5)
        assert harness.state == ms.RETURN_HOME

    def test_the_climb_is_not_guarded(self):
        # Off the mapped ground the scan metric is UNKNOWN, not bad. A
        # guard there would fire on noise for a third of the mission.
        harness = Harness()
        harness.tick(3)
        harness.nav_arrives(ms.PRE_RAMP_X, 0.25)
        harness.tick(2)
        assert harness.state == ms.CLIMB
        harness.degrade(ticks=5)
        assert harness.state == ms.CLIMB

    # ── the stop, and then the spin ──────────────────────────────────────
    def test_recovery_stops_the_robot_before_relocalizing(self):
        harness = self.to_return_home()
        harness.degrade(ticks=2)
        assert harness.state == ms.RECOVERY
        # RECOVERY's mode is idle: nothing drives the wheels, whatever
        # nav2 is still publishing.
        assert ms.CONTRACTS[ms.RECOVERY].mode == 'idle'
        kinds = [kind for kind, _ in harness.requests]
        assert ms.STOP_ALL in kinds
        # And it does not leave until the ARBITER says nothing is driving.
        harness.publish('arbiter', 'mode=nav active=nav')
        harness.tick(4)
        assert harness.state == ms.RECOVERY

    def test_recovery_hands_a_localization_failure_to_relocalize(self):
        harness = self.to_return_home()
        harness.degrade(ticks=2)
        harness.recover()
        assert harness.state == ms.RELOCALIZE

    def test_relocalize_asks_for_a_spin_and_nothing_else(self):
        harness = self.to_return_home()
        harness.degrade(ticks=2)
        harness.recover()
        harness.tick(2)
        spins = [(kind, payload) for kind, payload in harness.requests
                 if kind == ms.RELOCALIZE_GOAL]
        assert spins == [(ms.RELOCALIZE_GOAL, ms.RELOCALIZE_SPIN_RAD)]

    def test_the_spin_is_a_full_revolution(self):
        assert ms.RELOCALIZE_SPIN_RAD == pytest.approx(2.0 * math.pi)

    def test_relocalize_drives_through_the_nav_source(self):
        # The spin comes out of nav2's behavior_server, down the same
        # topic a nav leg uses. No new mode, and so no new publisher.
        assert ms.CONTRACTS[ms.RELOCALIZE].mode == 'nav'
        assert ms.CONTRACTS[ms.RELOCALIZE].mode \
            == ms.CONTRACTS[ms.RETURN_HOME].mode

    # ── the resume gate ──────────────────────────────────────────────────
    def test_a_finished_spin_is_not_enough_to_resume(self):
        harness = self.to_return_home()
        harness.degrade(ticks=2)
        harness.recover()
        harness.tick(2)
        harness.unhealed()
        harness.status = ms.SUCCEEDED       # the spin itself worked
        harness.tick(3)
        # The motion succeeded and the robot is still lost. Resuming here
        # would be resuming on the estimate that failed.
        assert harness.state == ms.RELOCALIZE

    def test_health_returning_resumes_the_interrupted_leg(self):
        harness = self.to_return_home()
        harness.degrade(ticks=2)
        harness.recover()
        harness.tick(2)
        harness.status = ms.SUCCEEDED
        harness.heal(ticks=3)
        assert harness.state == ms.RETURN_HOME
        assert harness.machine.result is None

    def test_the_resumed_leg_re_issues_its_goal(self):
        # C2-M5.0 requirement 7: navigate_to_pose must be re-issued, not
        # resumed. Re-entering the state mints a new token, so the node
        # really does send a fresh goal.
        harness = self.to_return_home()
        before = len([k for k, _ in harness.requests if k == ms.NAV_GOAL])
        harness.degrade(ticks=2)
        harness.recover()
        harness.tick(2)
        harness.status = ms.SUCCEEDED
        harness.heal(ticks=3)
        harness.tick(2)
        after = len([k for k, _ in harness.requests if k == ms.NAV_GOAL])
        assert after > before

    def test_the_mission_can_still_complete_after_a_recovery(self):
        harness = self.to_return_home()
        harness.degrade(ticks=2)
        harness.recover()
        harness.tick(2)
        harness.status = ms.SUCCEEDED
        harness.heal(ticks=3)
        harness.nav_arrives(*ms.HOME)
        harness.worker('grasp', 'phase', 'place:lift', 'placed',
                       extra='lifted=0')
        harness.tick(2)
        assert harness.state == ms.COMPLETE
        assert harness.machine.result == 'fetch'
        assert harness.machine.degraded_reason is None

    # ── the negative path ────────────────────────────────────────────────
    def test_a_recovery_that_never_restores_health_aborts(self):
        harness = self.to_return_home()
        harness.degrade(ticks=2)
        harness.recover()
        harness.tick(2)
        harness.unhealed()
        harness.status = ms.SUCCEEDED
        harness.tick(int(ms.RELOCALIZE_TIMEOUT / harness.dt) + 5)
        assert harness.state == ms.ABORT
        assert harness.machine.result == 'aborted'

    def test_the_abort_names_localization_and_not_the_nav_leg(self):
        harness = self.to_return_home()
        harness.degrade(ticks=2)
        harness.recover()
        harness.tick(2)
        harness.unhealed()
        harness.tick(int(ms.RELOCALIZE_TIMEOUT / harness.dt) + 5)
        assert harness.machine.reason == ms.LOCALIZATION_RECOVERY_TIMEOUT

    def test_a_failed_recovery_does_not_relocalize_again(self):
        # The loop that must not exist: RELOCALIZE -> RECOVERY ->
        # RELOCALIZE. RELOCALIZE carries no retries of its own, so the
        # escalation is an abort and the state is entered exactly once.
        harness = self.to_return_home()
        harness.degrade(ticks=2)
        harness.recover()
        harness.tick(2)
        harness.unhealed()
        harness.tick(int(ms.RELOCALIZE_TIMEOUT / harness.dt) + 40)
        assert harness.states().count(ms.RELOCALIZE) == 1
        assert harness.state == ms.ABORT

    def test_a_rejected_spin_aborts_rather_than_waiting(self):
        harness = self.to_return_home()
        harness.reply[ms.RELOCALIZE_SPIN_RAD] = ms.REJECTED
        harness.degrade(ticks=2)
        harness.recover()
        harness.tick(6)
        assert harness.state == ms.ABORT
        assert harness.machine.reason == ms.LOCALIZATION_RECOVERY_FAILED

    def test_an_abort_after_a_failed_recovery_keeps_asking_for_the_stop(self):
        harness = self.to_return_home()
        harness.degrade(ticks=2)
        harness.recover()
        harness.tick(2)
        harness.unhealed()
        harness.tick(int(ms.RELOCALIZE_TIMEOUT / harness.dt) + 10)
        assert harness.state == ms.ABORT
        assert harness.requests[-1][0] == ms.STOP_ALL

    def relocalize_once(self, harness):
        """Degrade, stop, spin, heal, resume — one whole cycle."""
        harness.degrade(ticks=2)
        harness.recover()
        harness.tick(2)
        harness.status = ms.SUCCEEDED
        harness.heal(ticks=3)

    def test_relocalize_has_its_own_budget_not_the_legs(self):
        # Experiment 2: an injected divergence makes Nav2 abort its own
        # goal first, so a shared budget was already down one retry
        # before the health monitor had finished accumulating evidence.
        # These are two independent counters now.
        harness = self.to_return_home()
        harness.status = ms.ABORTED          # Nav2 gives up on its own
        harness.tick(3)
        harness.recover()
        assert harness.machine.attempts.get(ms.RETURN_HOME) == 1
        self.relocalize_once(harness)
        assert harness.state == ms.RETURN_HOME
        # The relocalization did not spend the leg's remaining retry.
        assert harness.machine.attempts.get(ms.RETURN_HOME) == 1
        assert harness.machine.attempts.get(ms.RELOCALIZE) == 1

    def test_two_relocalizations_are_allowed_and_a_third_is_not(self):
        harness = self.to_return_home()
        for _ in range(2):
            self.relocalize_once(harness)
            assert harness.state == ms.RETURN_HOME
        harness.degrade(ticks=2)
        harness.recover()
        assert harness.states().count(ms.RELOCALIZE) == 2
        assert harness.state == ms.ABORT

    def test_the_third_degradation_aborts_rather_than_re_driving(self):
        # Re-driving would aim the robot at a goal computed from an
        # estimate the mission has already twice failed to repair.
        harness = self.to_return_home()
        for _ in range(2):
            self.relocalize_once(harness)
        harness.degrade(ticks=2)
        harness.recover()
        assert harness.machine.reason == ms.LOCALIZATION_DEGRADED
        assert harness.machine.result == 'aborted'

    # ── the shape of the change ──────────────────────────────────────────
    def test_relocalize_is_not_reachable_from_the_nominal_path(self):
        assert ms.RELOCALIZE not in ms.NOMINAL_NEXT
        assert ms.RELOCALIZE not in ms.NOMINAL_NEXT.values()

    def test_relocalize_has_a_finite_timeout(self):
        # A recovery state that cannot time out is a way to hang for ever
        # with the robot possibly still turning.
        assert ms.CONTRACTS[ms.RELOCALIZE].timeout is not None
        assert ms.CONTRACTS[ms.RELOCALIZE].timeout > 0

    def test_relocalize_carries_a_small_bounded_budget(self):
        # Bounded is the requirement; zero was the first guess and
        # Experiment 2 measured that it makes a successful recovery
        # useless, because Nav2's own abort has already spent the leg's.
        allowed = ms.CONTRACTS[ms.RELOCALIZE].max_retries + 1
        assert 1 <= allowed <= 3

    def test_relocalize_is_entered_from_exactly_one_place(self):
        # What makes the budget a complete bound on the recovery loop:
        # if a second call site appeared, the count would stop covering
        # every path into the state.
        source = open(os.path.join(
            os.path.dirname(__file__), '..', 'scripts',
            'mission_states.py')).read()
        entries = re.findall(r'_transition\(RELOCALIZE', source)
        assert len(entries) == 1

    def test_the_timeout_covers_a_full_spin_at_the_slowest_rate(self):
        # nav2_params: behavior_server.min_rotational_vel 0.4 rad/s.
        slowest_full_turn = ms.RELOCALIZE_SPIN_RAD / 0.4
        assert ms.RELOCALIZE_TIMEOUT > slowest_full_turn + lh.HEALTHY_HOLD_S
