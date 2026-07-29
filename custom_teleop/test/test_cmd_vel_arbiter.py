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
The arbitration policy, tested without a ROS graph.

select_source() is the whole of the policy, deliberately written as a
pure function of (mode, source ages) so the safety-critical rules —
teleop preempts, a stale source stops the robot — are asserted here
rather than inferred from a simulator run.
"""

from custom_teleop.cmd_vel_arbiter import (
    AUTONOMOUS_MODES, format_status, MODES, normalise_mode, select_source,
    SOURCE_TIMEOUT, SOURCES)

FRESH = 0.05          # comfortably inside SOURCE_TIMEOUT
STALE = 5.0           # comfortably outside it
# Derived from SOURCES rather than listed: a source added to the arbiter
# and forgotten here would silently drop out of every test below.
NEVER = {name: None for name in SOURCES}


def ages(**kw):
    """Source ages with everything unmentioned having never published."""
    return dict(NEVER, **kw)


def test_import():
    from custom_teleop import cmd_vel_arbiter
    assert hasattr(cmd_vel_arbiter, 'CmdVelArbiter')
    assert hasattr(cmd_vel_arbiter, 'main')


# ── mode parsing ────────────────────────────────────────────────────────────
def test_every_mode_normalises_to_itself():
    for mode in MODES:
        assert normalise_mode(mode) == mode


def test_panel_says_auto_but_the_source_is_called_nav():
    """index.html has said 'auto' since before the mission state machine."""
    assert normalise_mode('auto') == 'nav'
    assert normalise_mode('autonomous') == 'nav'
    assert normalise_mode('nav2') == 'nav'


def test_mode_parsing_is_forgiving_about_case_and_whitespace():
    assert normalise_mode('  TELEOP \n') == 'teleop'
    assert normalise_mode('Auto') == 'nav'


def test_unknown_mode_is_rejected_not_guessed():
    """
    None tells the node to warn and keep the mode the operator set.

    Folding an unrecognised string onto a default would silently move the
    robot into a state nobody asked for.
    """
    assert normalise_mode('autonmous') is None      # typo
    assert normalise_mode('drive') is None
    assert normalise_mode(None) == 'idle'
    assert normalise_mode('') == 'idle'


# ── teleop preemption ───────────────────────────────────────────────────────
def test_teleop_preempts_every_mode():
    """
    The point of the whole node.

    Four unmediated publishers interleaved ~50/50, so grabbing the stick
    during a policy run halved the speed rather than taking control.
    """
    for mode in MODES:
        assert select_source(
            mode, ages(teleop=FRESH, nav=FRESH, rl=FRESH)) == 'teleop'


def test_teleop_preempts_a_running_rl_climb():
    assert select_source('rl', ages(teleop=FRESH, rl=FRESH)) == 'teleop'


def test_teleop_preempts_from_idle():
    """A human reaching for the stick must not have to set a mode first."""
    assert select_source('idle', ages(teleop=FRESH)) == 'teleop'


def test_releasing_the_stick_returns_control_to_the_mode():
    """
    Preemption lasts only while teleop is live.

    The panel latches a lasting override by switching mode instead, which
    it then asserts at 2 Hz.
    """
    assert select_source('nav', ages(teleop=STALE, nav=FRESH)) == 'nav'


# ── mode selection ──────────────────────────────────────────────────────────
def test_mode_selects_its_own_source():
    assert select_source('nav', ages(nav=FRESH, rl=FRESH)) == 'nav'
    assert select_source('rl', ages(nav=FRESH, rl=FRESH)) == 'rl'


def test_unselected_sources_are_ignored_even_when_fresh():
    """
    A deselected source must not leak onto the wheels.

    Nav2 keeps publishing across the ramp handoff, while the RL policy
    owns the wheels.
    """
    assert select_source('rl', ages(nav=FRESH)) is None
    assert select_source('nav', ages(rl=FRESH)) is None


def test_idle_drives_nothing_autonomously():
    assert select_source('idle', ages(nav=FRESH, rl=FRESH)) is None


def test_mode_teleop_with_no_teleop_publisher_stops():
    """
    Teleop mode with an idle stick means stop.

    It must not fall through to whatever else happens to be publishing.
    """
    assert select_source('teleop', ages(nav=FRESH, rl=FRESH)) is None


# ── staleness ───────────────────────────────────────────────────────────────
def test_a_stale_source_stops_the_robot():
    assert select_source('nav', ages(nav=STALE)) is None
    assert select_source('rl', ages(rl=STALE)) is None
    assert select_source('teleop', ages(teleop=STALE)) is None


def test_a_source_that_never_published_is_not_fresh():
    """An age of None must not compare as 0 — the startup case."""
    assert select_source('nav', NEVER) is None
    assert select_source('teleop', NEVER) is None


def test_the_timeout_boundary_is_inclusive():
    assert select_source('nav', ages(nav=SOURCE_TIMEOUT)) == 'nav'
    assert select_source('nav', ages(nav=SOURCE_TIMEOUT + 1e-6)) is None


def test_timeout_is_a_parameter_not_a_constant():
    assert select_source('nav', ages(nav=1.0), timeout=2.0) == 'nav'
    assert select_source('nav', ages(nav=1.0), timeout=0.5) is None


def test_default_timeout_rides_out_dropped_frames_but_beats_the_controller():
    """
    The default timeout has to sit inside a specific window.

    0.3 s is three missed frames from a 10 Hz publisher, and stays inside
    the diff_drive_controller's own 0.5 s cmd_vel_timeout so the arbiter —
    not the controller watchdog — is what stops the robot.
    """
    assert 3 / 10.0 <= SOURCE_TIMEOUT < 0.5


# ── status line ─────────────────────────────────────────────────────────────
def test_status_reports_mode_active_and_every_source():
    line = format_status('nav', 'nav', ages(nav=0.05, teleop=STALE))
    fields = dict(part.split('=', 1) for part in line.split(' '))
    assert fields['mode'] == 'nav'
    assert fields['active'] == 'nav'
    assert fields['nav'] == '0.05'
    assert fields['rl'] == '--'        # never published


def test_status_says_none_rather_than_going_blank_when_stopped():
    """The panel prints this verbatim; an empty field reads as a bug."""
    fields = dict(p.split('=', 1)
                  for p in format_status('idle', None, NEVER).split(' '))
    assert fields['active'] == 'none'


def test_status_reports_every_source_it_arbitrates():
    """
    A source the status line cannot report is a source nobody can debug.

    This is the whole reason the approach controller got its own input
    instead of borrowing /cmd_vel_rl.
    """
    fields = dict(p.split('=', 1)
                  for p in format_status('idle', None, NEVER).split(' '))
    for name in SOURCES:
        assert name in fields


# ── the approach source ─────────────────────────────────────────────────────
def test_approach_is_a_mode_and_a_source():
    assert 'approach' in SOURCES
    assert 'approach' in MODES
    assert normalise_mode('approach') == 'approach'
    assert normalise_mode('  APPROACH ') == 'approach'


def test_approach_mode_selects_the_approach_source():
    assert select_source('approach', ages(approach=FRESH)) == 'approach'


def test_approach_does_not_leak_into_other_modes():
    """
    The mission runs rl -> approach -> idle back to back.

    If a stale approach command could still be forwarded during the RL
    descent, the handoff would be a race rather than a switch.
    """
    assert select_source('rl', ages(approach=FRESH)) is None
    assert select_source('nav', ages(approach=FRESH)) is None
    assert select_source('idle', ages(approach=FRESH)) is None


def test_teleop_preempts_the_approach_too():
    """The vision servo drives at a target; a human must be able to stop it."""
    assert select_source(
        'approach', ages(teleop=FRESH, approach=FRESH)) == 'teleop'


def test_a_stale_approach_stops_the_robot():
    assert select_source('approach', ages(approach=STALE)) is None


def test_every_autonomous_mode_can_select_its_own_source():
    """
    Mode names and source names have to stay one-to-one.

    They are separate tuples, so a mode added without its source (or the
    reverse) would select nothing and read as "the controller published
    but the robot did not move".
    """
    for mode in AUTONOMOUS_MODES:
        assert mode in SOURCES
        assert mode in MODES
        assert select_source(mode, ages(**{mode: FRESH})) == mode


def test_idle_drives_none_of_the_autonomous_sources():
    every = {name: FRESH for name in SOURCES if name != 'teleop'}
    assert select_source('idle', dict(NEVER, **every)) is None
