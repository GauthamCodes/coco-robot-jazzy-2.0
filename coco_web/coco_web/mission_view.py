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
The mission executive's state, translated for a browser.

Pure module: no rclpy, no sockets. Every function takes the plain
``key=value`` line ``/mission/state`` carries and returns plain Python,
so the whole translation is unit-tested without a ROS graph.

What ``/mission/state`` actually carries
----------------------------------------
``mission_states.status_line()`` emits TWELVE fields, always all of them,
with ``--`` for absent::

    state= prev= event= elapsed= timeout= attempt= retries=
    owner= mode= reason= result=

P0.1 read two of them. Worse, it looked for ``colour``, ``target`` and
``detail``, none of which that line has ever contained -- so the mission
colour on the wire was permanently null and the UI only ever showed one
because the browser echoed its own selection back at itself. The
authoritative colour is on ``/mission/target_colour``, which is a
different topic, and this module does not guess at it.

Why the platform phase is a translation and not a replacement
--------------------------------------------------------------
The executive owns nineteen states and they mean specific things. A
product UI should not make an operator learn ``VERIFY_PLACEMENT``, but
neither may it throw the distinction away: an engineer looking at the
same screen needs the real name. So both travel together -- ``phase`` is
the ten-word platform vocabulary, ``state`` is the executive's own word,
and the UI chooses which to show.

``RECOVERY`` and ``RELOCALIZE`` keep the phase of the state they are
recovering FROM, with ``recovering`` set. Flattening them to a generic
"recovering" would lose which leg is being retried, which is the only
interesting thing about them.

Progress, and what is deliberately absent
------------------------------------------
``step``/``steps`` come from ``NOMINAL_ORDER``, which is the executive's
own ``NOMINAL_NEXT`` chain -- a real ordinal, not a presentation-side
list. ``elapsed``/``timeout`` are the executive's own numbers, so a
within-state fraction is real too.

There is **no overall percentage and no ETA**, because there is no basis
for one. ``mission_executive`` sends its Nav2 goal with no
``feedback_callback``, so distance-remaining is not published on any
topic and progress *within* a navigation leg is unobservable. A bar that
interpolated one would be inventing it. P0.1's UI did exactly that, from
a hard-coded fifteen-entry list in the browser; it is gone.

Drift
-----
The constants below are duplicated from
``coco_mission/scripts/mission_states.py`` rather than imported.
``coco_mission`` already depends on ``coco_web`` -- ``mission.launch.py``
includes ``platform.launch.py`` -- so importing it back would close a
cycle, which colcon refuses to order and which has broken this workspace
twice. ``test_mission_view.py`` parses that file with ``ast`` and asserts
every list here still matches it, the same guard ``FALLBACK_COLOURS``
has against ``coco_config``.
"""

# ── the executive's states, duplicated (see the module docstring) ───────
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
RELOCALIZE = 'RELOCALIZE'
ABORT = 'ABORT'

#: Every state the executive can report. Nineteen.
STATES = (
    IDLE, LOCALIZE, NAVIGATE_TO_RAMP, ALIGN_FOR_CLIMB, CLIMB,
    VERIFY_CLIMB, SEARCH_TARGET, STOW_ARM, APPROACH_TARGET, GRASP,
    VERIFY_GRASP, DESCEND, RETURN_HOME, PLACE, VERIFY_PLACEMENT,
    COMPLETE, RECOVERY, RELOCALIZE, ABORT,
)

#: The two states a mission ends in and never leaves.
TERMINAL_STATES = (COMPLETE, ABORT)

#: The nominal path, in order, from ``NOMINAL_NEXT``. Sixteen entries.
#: This is the ONLY basis on which a step number is reported.
NOMINAL_ORDER = (
    IDLE, LOCALIZE, NAVIGATE_TO_RAMP, ALIGN_FOR_CLIMB, CLIMB,
    VERIFY_CLIMB, SEARCH_TARGET, STOW_ARM, APPROACH_TARGET, GRASP,
    VERIFY_GRASP, DESCEND, RETURN_HOME, PLACE, VERIFY_PLACEMENT,
    COMPLETE,
)

#: Every structured failure reason the executive can report. Forty-eight.
#: Carried so an unknown code is reported AS unknown rather than printed
#: to an operator as though it were a diagnosis.
REASONS = (
    'NO_ODOMETRY', 'NO_RAMP_DRIVER', 'NO_LOCALIZATION',
    'NO_TARGET_COLOUR', 'NAVIGATION_UNAVAILABLE',
    'NAVIGATION_REJECTED', 'NAVIGATION_FAILED',
    'NAVIGATION_TIMEOUT', 'PRE_RAMP_POSE_OUT_OF_REGION',
    'HOME_POSE_OUT_OF_REGION', 'ALIGN_OFF_LANE', 'ALIGN_HEADING',
    'ALIGN_NOT_ON_FLAT', 'ALIGN_TIMEOUT', 'SERVICE_UNAVAILABLE',
    'SERVICE_REFUSED', 'CLIMB_FAILED', 'CLIMB_TIPPED',
    'CLIMB_TIMEOUT', 'CLIMB_POSE_UNVERIFIED', 'CLIMB_OFF_LANE',
    'CLIMB_VERIFY_TIMEOUT', 'TARGET_NOT_FOUND',
    'TARGET_COLOUR_MISMATCH', 'STOW_FAILED', 'STOW_TIMEOUT',
    'APPROACH_FAILED', 'APPROACH_TIMEOUT', 'GRASP_FAILED',
    'GRASP_TIMEOUT', 'GRASP_UNVERIFIED', 'GRASP_VERIFY_TIMEOUT',
    'DESCENT_FAILED', 'DESCENT_TIPPED', 'DESCENT_TIMEOUT',
    'RETURN_FAILED', 'RETURN_TIMEOUT', 'PLACE_FAILED',
    'PLACE_TIMEOUT', 'PLACEMENT_UNVERIFIED',
    'PLACEMENT_VERIFY_TIMEOUT', 'RECOVERY_TIMEOUT',
    'OPERATOR_ABORT', 'CLOCK_STALLED', 'LOCALIZATION_DEGRADED',
    'LOCALIZATION_RECOVERY_FAILED', 'LOCALIZATION_RECOVERY_TIMEOUT',
    'LOCALIZATION_RECOVERY_UNAVAILABLE',
)

#: The one reason that means a person pressed Abort rather than the
#: mission failing. It is the difference between STOPPED and FAILED, and
#: reporting an operator's own decision back to them as a failure is
#: both wrong and alarming.
OPERATOR_ABORT = 'OPERATOR_ABORT'

# ── the platform vocabulary ────────────────────────────────────────────
#: What the browser is told. Ten words, none of them ROS.
PLATFORM_PHASES = (
    'IDLE', 'STARTING', 'SEARCHING', 'APPROACHING', 'GRASPING',
    'NAVIGATING', 'RETURNING', 'COMPLETED', 'FAILED', 'STOPPED',
)

#: Executive state -> platform phase. ABORT is resolved separately
#: because it depends on the reason, and RECOVERY/RELOCALIZE are
#: resolved from ``prev``.
PHASE_OF = {
    IDLE: 'IDLE',
    LOCALIZE: 'STARTING',
    NAVIGATE_TO_RAMP: 'NAVIGATING',
    ALIGN_FOR_CLIMB: 'NAVIGATING',
    CLIMB: 'NAVIGATING',
    VERIFY_CLIMB: 'NAVIGATING',
    SEARCH_TARGET: 'SEARCHING',
    STOW_ARM: 'APPROACHING',
    APPROACH_TARGET: 'APPROACHING',
    GRASP: 'GRASPING',
    VERIFY_GRASP: 'GRASPING',
    DESCEND: 'RETURNING',
    RETURN_HOME: 'RETURNING',
    PLACE: 'RETURNING',
    VERIFY_PLACEMENT: 'RETURNING',
    COMPLETE: 'COMPLETED',
}

#: Operator-facing wording, one per executive state. The state name
#: itself stays visible in Engineering mode; Play mode reads as English.
WORDS = {
    IDLE: 'Waiting to start',
    LOCALIZE: 'Working out where it is',
    NAVIGATE_TO_RAMP: 'Driving to the ramp',
    ALIGN_FOR_CLIMB: 'Lining up with the ramp',
    CLIMB: 'Climbing the ramp',
    VERIFY_CLIMB: 'Checking it made it up',
    SEARCH_TARGET: 'Looking for the cylinder',
    STOW_ARM: 'Stowing the arm',
    APPROACH_TARGET: 'Closing in on the cylinder',
    GRASP: 'Grasping',
    VERIFY_GRASP: 'Checking the grip',
    DESCEND: 'Coming back down',
    RETURN_HOME: 'Heading home',
    PLACE: 'Putting the cylinder down',
    VERIFY_PLACEMENT: 'Checking the placement',
    COMPLETE: 'Done -- cylinder delivered',
    RECOVERY: 'Recovering',
    RELOCALIZE: 'Working out where it is again',
    ABORT: 'Mission stopped',
}

#: Plain-English wording for the failure reasons an operator can act on.
#: Deliberately partial: a reason with no entry is shown as its own code
#: rather than given invented prose, which is how a diagnosis becomes
#: wrong on the way to the screen.
REASON_WORDS = {
    'NO_TARGET_COLOUR': 'No colour was chosen',
    'OPERATOR_ABORT': 'Stopped on request',
    'TARGET_NOT_FOUND': 'Could not see the cylinder',
    'TARGET_COLOUR_MISMATCH': 'Saw a different colour than the one asked for',
    'NAVIGATION_TIMEOUT': 'Navigation ran out of time',
    'NAVIGATION_FAILED': 'Navigation gave up',
    'NAVIGATION_UNAVAILABLE': 'The navigation stack is not running',
    'CLIMB_TIPPED': 'The robot tipped on the ramp',
    'DESCENT_TIPPED': 'The robot tipped coming down',
    'GRASP_UNVERIFIED': 'The grasp did not lift the cylinder',
    'LOCALIZATION_DEGRADED': 'The robot lost track of where it is',
    'CLOCK_STALLED': 'The simulator clock stopped',
}

#: How ``/mission/state`` spells "this field has no value".
_ABSENT = ('', '--', 'none', 'None')


def _value(fields, key):
    """Return a field's value, or None for the executive's absent spellings."""
    raw = fields.get(key)
    return None if raw is None or raw in _ABSENT else raw


def _number(fields, key):
    """Return a field as a float, or None when absent or unparsable."""
    raw = _value(fields, key)
    if raw is None:
        return None
    try:
        return float(raw)
    except (TypeError, ValueError):
        return None


def _integer(fields, key):
    """Return a field as an int, or None when absent or unparsable."""
    number = _number(fields, key)
    return None if number is None else int(number)


def phase_for(state, previous=None, reason=None):
    """
    Map an executive state onto the platform's ten-word vocabulary.

    ``ABORT`` splits on the reason: an operator's own Abort is STOPPED,
    anything else is FAILED. ``RECOVERY`` and ``RELOCALIZE`` resolve to
    the phase of the state they are retrying, because "recovering" on its
    own does not say which leg is being retried, and that is the only
    thing worth knowing about them.
    """
    if state == ABORT:
        return 'STOPPED' if reason == OPERATOR_ABORT else 'FAILED'
    if state in (RECOVERY, RELOCALIZE):
        if previous in (None, RECOVERY, RELOCALIZE):
            return 'STARTING'
        return phase_for(previous, None, reason)
    return PHASE_OF.get(state)


def step_for(state):
    """
    Return ``(step, steps)`` on the executive's own nominal path.

    ``(None, 16)`` for a state that is not on it -- RECOVERY, RELOCALIZE
    and ABORT are real states but not ordinals, and reporting a position
    for them would put the progress bar somewhere it does not belong.
    """
    total = len(NOMINAL_ORDER)
    if state in NOMINAL_ORDER:
        return NOMINAL_ORDER.index(state) + 1, total
    return None, total


def normalise(fields, colour=None, now=None):
    """
    Turn the parsed ``/mission/state`` fields into the telemetry payload.

    ``colour`` comes from ``/mission/target_colour``, a DIFFERENT topic:
    ``/mission/state`` has never carried one, and P0.1 looked for it
    there anyway.

    ``changed_at`` is derived rather than observed -- the executive
    reports how long it has been in this state, so the transition
    happened ``elapsed`` seconds before this line was read. That is a
    real timestamp from a real number, not a guess at when a frame
    arrived.
    """
    state = _value(fields, 'state')
    previous = _value(fields, 'prev')
    reason = _value(fields, 'reason')
    elapsed = _number(fields, 'elapsed')
    step, steps = step_for(state)
    changed_at = None
    if now is not None and elapsed is not None:
        changed_at = now - elapsed
    active = bool(state) and state not in TERMINAL_STATES and state != IDLE
    return {
        'online': bool(fields),
        'phase': phase_for(state, previous, reason),
        'state': state,
        'previous': previous,
        'known': state in STATES,
        'words': WORDS.get(state),
        'active': active,
        'recovering': state in (RECOVERY, RELOCALIZE),
        'colour': colour,
        'reason': reason,
        'reason_known': reason is None or reason in REASONS,
        'reason_words': REASON_WORDS.get(reason),
        'result': _value(fields, 'result'),
        'owner': _value(fields, 'owner'),
        'mode': _value(fields, 'mode'),
        'event': _value(fields, 'event'),
        'step': step,
        'steps': steps,
        'elapsed': elapsed,
        'timeout': _number(fields, 'timeout'),
        'attempt': _integer(fields, 'attempt'),
        'retries': _integer(fields, 'retries'),
        'changed_at': changed_at,
    }
