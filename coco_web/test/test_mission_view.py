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

"""Mission-state translation, and the drift guard against the executive."""

import ast
import os

from coco_web import mission_view as mv

import pytest


# ── the drift guard ────────────────────────────────────────────────────
# mission_view duplicates constants from coco_mission because importing
# them would close a package cycle (coco_mission already depends on
# coco_web through mission.launch.py). Duplication is only safe with a
# test that reads the original, so these do.

def _mission_states_path():
    """Locate coco_mission/scripts/mission_states.py from the repo root."""
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(os.path.dirname(here))
    return os.path.join(root, 'coco_mission', 'scripts', 'mission_states.py')


def _executive_constants():
    """
    Parse the executive's module with ast, without importing it.

    ast rather than import because mission_states.py lives in
    coco_mission/scripts/ (installed as an executable, reached through a
    sys.path shim) and imports coco_config at module scope. Reading it is
    the point; running it is not.
    """
    path = _mission_states_path()
    assert os.path.isfile(path), (
        f'the executive moved: {path} is not a file. mission_view '
        f'duplicates its constants and this test is the only thing '
        f'stopping them drifting.')
    with open(path, encoding='utf-8') as handle:
        tree = ast.parse(handle.read())
    values = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign) or len(node.targets) != 1:
            continue
        target = node.targets[0]
        if not isinstance(target, ast.Name):
            continue
        resolved = _resolve(node.value, values)
        if resolved is not _UNRESOLVED:
            values[target.id] = resolved
    return values


#: Sentinel for an expression this reader cannot evaluate. None is a
#: legitimate value in that module, so it cannot double as "no value".
_UNRESOLVED = object()


def _resolve(node, values):
    """
    Evaluate one constant expression against the names already seen.

    ``literal_eval`` is not enough on its own: the executive writes
    ``TERMINAL_STATES = (COMPLETE, ABORT)`` and ``NOMINAL_NEXT = {IDLE:
    LOCALIZE, ...}``, which are tuples and dicts of NAMES. Those are
    exactly the two constants worth guarding, so the reader resolves a
    name against the constants defined above it, as Python would.
    """
    if isinstance(node, ast.Constant):
        return node.value
    if isinstance(node, ast.Name):
        return values.get(node.id, _UNRESOLVED)
    if isinstance(node, (ast.Tuple, ast.List)):
        items = [_resolve(item, values) for item in node.elts]
        if any(item is _UNRESOLVED for item in items):
            return _UNRESOLVED
        return tuple(items)
    if isinstance(node, ast.Dict):
        out = {}
        for key_node, value_node in zip(node.keys, node.values):
            key = _resolve(key_node, values)
            value = _resolve(value_node, values)
            if key is _UNRESOLVED or value is _UNRESOLVED:
                return _UNRESOLVED
            out[key] = value
        return out
    return _UNRESOLVED


def _executive_expression(name):
    """
    Return the source text of one assignment in the executive's module.

    For constants the executive itself DERIVES rather than spells out,
    the expression is the thing worth pinning: a resolved value would
    only say the two agree today, while the expression says they are
    computed the same way.
    """
    with open(_mission_states_path(), encoding='utf-8') as handle:
        tree = ast.parse(handle.read())
    for node in tree.body:
        if (isinstance(node, ast.Assign) and len(node.targets) == 1
                and isinstance(node.targets[0], ast.Name)
                and node.targets[0].id == name):
            return ast.unparse(node.value)
    return None


def test_the_executive_module_is_where_we_think_it_is():
    """A moved file must fail loudly here, not silently stop guarding."""
    assert os.path.isfile(_mission_states_path())


def test_every_state_name_matches_the_executive():
    """All nineteen states, spelled exactly as the executive spells them."""
    constants = _executive_constants()
    for state in mv.STATES:
        assert constants.get(state) == state, (
            f'{state} is not a state constant in mission_states.py')
    assert len(mv.STATES) == 19


def test_terminal_states_match_the_executive():
    """COMPLETE and ABORT, from the executive's own tuple."""
    assert tuple(_executive_constants()['TERMINAL_STATES']) == \
        mv.TERMINAL_STATES


def test_nominal_order_matches_the_executive_chain():
    """
    NOMINAL_ORDER must be NOMINAL_NEXT walked from IDLE.

    This is the whole basis for the step number the UI shows. If the
    executive gains or reorders a state, "step 7 of 16" becomes a lie,
    and this is what catches it.
    """
    chain = _executive_constants()['NOMINAL_NEXT']
    walked = [mv.IDLE]
    while walked[-1] in chain:
        walked.append(chain[walked[-1]])
    assert tuple(walked) == mv.NOMINAL_ORDER
    assert len(mv.NOMINAL_ORDER) == 16


def test_every_reason_matches_the_executive():
    """All forty-eight failure reasons exist, spelled identically."""
    constants = _executive_constants()
    for reason in mv.REASONS:
        assert constants.get(reason) == reason, (
            f'{reason} is not a reason constant in mission_states.py')
    assert len(mv.REASONS) == 48


def test_no_executive_reason_is_missing_from_our_copy():
    """
    The guard in the other direction: a NEW reason must not slip past.

    Without this, the executive could gain a failure code and the UI
    would quietly report it as unknown forever.
    """
    constants = _executive_constants()
    known = set(mv.REASONS) | set(mv.STATES) | {
        'RUNNING', 'SUCCESS', 'FAILURE', 'ESCALATE_ABORT',
        'ESCALATE_SKIP_GRASP',
    }
    self_named = {name for name, value in constants.items()
                  if isinstance(value, str) and value == name}
    assert self_named - known == set(), (
        'mission_states.py has self-named constants mission_view has '
        'never heard of; if they are new failure reasons, add them to '
        'REASONS')


def test_the_world_to_map_offset_is_derivable_from_coco_config():
    """
    The browser's world geometry is shifted by -SPAWN_XY[0], not by a copy.

    coco_config's geometry is in WORLD coordinates; every pose the page
    draws against is in the MAP frame, whose origin is the spawn point.
    So the shift is exactly -SPAWN_XY[0]. mission_states spells the same
    number WORLD_TO_MAP_X, and platform_server derives rather than copies
    it -- this asserts the derivation still equals the executive's value.
    Getting it wrong draws the ramp two metres from where the robot
    climbs it, which looks exactly like broken localisation.
    """
    spawn = pytest.importorskip('coco_config.robot').SPAWN_XY
    assert -spawn[0] == 2.0
    # The executive derives it identically, which is why copying its
    # value would have been the wrong move: both read the same config.
    assert _executive_expression('WORLD_TO_MAP_X') == '-SPAWN_XY[0]'


# ── phase translation ──────────────────────────────────────────────────

@pytest.mark.parametrize('state, phase', [
    ('IDLE', 'IDLE'),
    ('LOCALIZE', 'STARTING'),
    ('NAVIGATE_TO_RAMP', 'NAVIGATING'),
    ('ALIGN_FOR_CLIMB', 'NAVIGATING'),
    ('CLIMB', 'NAVIGATING'),
    ('VERIFY_CLIMB', 'NAVIGATING'),
    ('SEARCH_TARGET', 'SEARCHING'),
    ('STOW_ARM', 'APPROACHING'),
    ('APPROACH_TARGET', 'APPROACHING'),
    ('GRASP', 'GRASPING'),
    ('VERIFY_GRASP', 'GRASPING'),
    ('DESCEND', 'RETURNING'),
    ('RETURN_HOME', 'RETURNING'),
    ('PLACE', 'RETURNING'),
    ('VERIFY_PLACEMENT', 'RETURNING'),
    ('COMPLETE', 'COMPLETED'),
])
def test_each_state_maps_to_a_platform_phase(state, phase):
    """Every nominal state has a phase, and it is the expected one."""
    assert mv.phase_for(state) == phase


def test_every_state_is_translatable():
    """No executive state may reach the browser without a phase."""
    for state in mv.STATES:
        assert mv.phase_for(state, previous='CLIMB') in mv.PLATFORM_PHASES


def test_every_state_has_operator_wording():
    """Play mode must never fall back to printing a ROS state name."""
    for state in mv.STATES:
        assert mv.WORDS.get(state), f'{state} has no operator wording'


def test_operator_abort_is_stopped_not_failed():
    """
    A person pressing Abort is not a mission failure.

    Reporting an operator's own decision back to them in red is both
    wrong and alarming, and it is the only thing distinguishing the two.
    """
    assert mv.phase_for('ABORT', reason='OPERATOR_ABORT') == 'STOPPED'


def test_a_real_failure_is_failed():
    """Any other abort reason is a genuine failure."""
    assert mv.phase_for('ABORT', reason='RETURN_FAILED') == 'FAILED'
    assert mv.phase_for('ABORT') == 'FAILED'


def test_recovery_keeps_the_phase_it_is_recovering_from():
    """
    RECOVERY is a retry OF something, and which leg is the point.

    Flattening it to a generic phase would lose the only interesting
    thing about the state.
    """
    assert mv.phase_for('RECOVERY', previous='GRASP') == 'GRASPING'
    assert mv.phase_for('RELOCALIZE', previous='RETURN_HOME') == 'RETURNING'


def test_recovery_without_a_previous_state_does_not_crash():
    """A RECOVERY as the first thing seen still yields a valid phase."""
    assert mv.phase_for('RECOVERY') in mv.PLATFORM_PHASES
    assert mv.phase_for('RECOVERY', previous='RECOVERY') in mv.PLATFORM_PHASES


def test_an_unknown_state_has_no_phase_rather_than_a_wrong_one():
    """A state this build has not heard of must not be guessed at."""
    assert mv.phase_for('WARP_DRIVE') is None


# ── step numbers ───────────────────────────────────────────────────────

def test_step_is_the_executives_own_ordinal():
    """IDLE is step 1 and COMPLETE is the last; both from NOMINAL_ORDER."""
    assert mv.step_for('IDLE') == (1, 16)
    assert mv.step_for('COMPLETE') == (16, 16)
    assert mv.step_for('GRASP') == (10, 16)


def test_off_path_states_report_no_step():
    """
    RECOVERY and ABORT are states, not positions.

    Reporting an ordinal for them would put the progress bar somewhere
    the mission is not.
    """
    for state in ('RECOVERY', 'RELOCALIZE', 'ABORT'):
        step, steps = mv.step_for(state)
        assert step is None
        assert steps == 16


# ── the real status line ───────────────────────────────────────────────

LINE = ('state=CLIMB prev=ALIGN_FOR_CLIMB event=enter elapsed=12.3 '
        'timeout=180 attempt=1 retries=0 owner=ramp_driver mode=rl '
        'reason=-- result=--')


def _fields(line):
    """Split a key=value line the way telemetry.parse_kv_line does."""
    out = {}
    for token in line.split():
        key, sep, value = token.partition('=')
        if sep and key:
            out[key] = value
    return out


def test_all_twelve_fields_survive_normalisation():
    """
    P0.1 consumed two of the twelve fields on this line.

    Each one below was on the wire and discarded before P0.2.
    """
    view = mv.normalise(_fields(LINE), colour='blue', receipt=_SIM_RECEIPT)
    assert view['state'] == 'CLIMB'
    assert view['previous'] == 'ALIGN_FOR_CLIMB'
    assert view['event'] == 'enter'
    assert view['elapsed'] == pytest.approx(12.3)
    assert view['timeout'] == pytest.approx(180.0)
    assert view['attempt'] == 1
    assert view['retries'] == 0
    assert view['owner'] == 'ramp_driver'
    assert view['mode'] == 'rl'
    assert view['reason'] is None
    assert view['result'] is None


def test_the_dash_sentinel_becomes_none_not_a_dash():
    """'--' is the executive's 'absent'; the UI must not print it."""
    view = mv.normalise(_fields(LINE))
    assert view['reason'] is None
    assert view['result'] is None


def test_colour_comes_from_the_argument_not_the_line():
    """
    /mission/state has never carried a colour, and P0.1 looked for one.

    The authoritative value is on /mission/target_colour, so it arrives
    as an argument. A line carrying no colour must not produce one.
    """
    assert 'colour' not in LINE
    assert mv.normalise(_fields(LINE), colour='yellow')['colour'] == 'yellow'
    assert mv.normalise(_fields(LINE))['colour'] is None


_SIM_RECEIPT = {'wall': 1.8e9, 'ros': 1000.0, 'first_wall': 1.8e9 - 30.0,
                'ros_is_sim': True}


def test_the_transition_is_derived_on_one_clock_only():
    """
    ros_changed = ros_received - elapsed: two numbers from the same clock.

    The executive's `elapsed` is ROS (sim) time, and so is the server's
    ROS clock at receipt when both use sim time.
    """
    view = mv.normalise(_fields(LINE), receipt=_SIM_RECEIPT)
    timing = view['timing']
    assert timing['elapsed_clock'] == 'ros'
    assert timing['ros_changed'] == pytest.approx(1000.0 - 12.3)
    assert timing['wall_received'] == 1.8e9
    assert timing['wall_first_seen'] == 1.8e9 - 30.0


def test_wall_time_is_never_subtracted_from_ros_time():
    """
    changed_at was wall_now - elapsed: wall clock minus sim time.

    At the real-time factor measured with a browser attached (~0.4) that
    was wrong by 60 % of the time in state. It stays on the wire for
    coco.v1's shape and is always None.
    """
    view = mv.normalise(_fields(LINE), receipt=_SIM_RECEIPT)
    assert view['changed_at'] is None
    assert view['timing']['ros_changed'] != pytest.approx(1.8e9 - 12.3)


def test_no_transition_is_derived_unless_the_clocks_are_one_clock():
    """A server not on sim time cannot place a sim-time `elapsed`."""
    receipt = dict(_SIM_RECEIPT, ros_is_sim=False)
    timing = mv.normalise(_fields(LINE), receipt=receipt)['timing']
    assert timing['ros_changed'] is None
    assert timing['ros_received'] == 1000.0      # reported, not combined


def test_no_receipt_means_no_timestamps_rather_than_epoch_zero():
    """Nothing observed, nothing claimed."""
    timing = mv.normalise(_fields(LINE))['timing']
    assert timing['ros_changed'] is None
    assert timing['wall_received'] is None
    assert timing['wall_first_seen'] is None


def test_a_receipt_earlier_than_elapsed_derives_nothing():
    """A restarted simulator's clock cannot place an older transition."""
    receipt = dict(_SIM_RECEIPT, ros=5.0)
    assert mv.normalise(_fields(LINE), receipt=receipt)['timing'][
        'ros_changed'] is None


def test_active_is_false_in_idle_and_both_terminal_states():
    """The question the UI asks is 'may I offer Start?'."""
    for state in ('IDLE', 'COMPLETE', 'ABORT'):
        line = LINE.replace('state=CLIMB', f'state={state}')
        assert mv.normalise(_fields(line))['active'] is False
    assert mv.normalise(_fields(LINE))['active'] is True


def test_recovering_is_flagged_separately_from_the_phase():
    """A client can show 'retrying the grasp', which needs both."""
    line = LINE.replace('state=CLIMB', 'state=RECOVERY')
    view = mv.normalise(_fields(line))
    assert view['recovering'] is True
    assert view['phase'] == 'NAVIGATING'   # prev=ALIGN_FOR_CLIMB


def test_an_unknown_reason_is_marked_unknown_not_printed_as_prose():
    """
    A code this build does not know must not be shown as a diagnosis.

    reason_known is what lets the UI say 'unrecognised code' instead of
    presenting an unknown string as though it explained something.
    """
    line = LINE.replace('reason=--', 'reason=SOMETHING_NEW')
    view = mv.normalise(_fields(line))
    assert view['reason'] == 'SOMETHING_NEW'
    assert view['reason_known'] is False


def test_a_known_reason_is_marked_known():
    """The forty-eight documented reasons pass the same check."""
    line = LINE.replace('reason=--', 'reason=CLIMB_TIPPED')
    view = mv.normalise(_fields(line))
    assert view['reason_known'] is True
    assert view['reason_words']


def test_an_empty_line_is_offline_with_every_key_present():
    """
    A client must branch on `online`, never on whether a key exists.

    An offline payload missing keys is how a UI ends up throwing on
    undefined halfway through a render.
    """
    offline = mv.normalise({})
    online = mv.normalise(_fields(LINE))
    assert offline['online'] is False
    assert set(offline) == set(online)


def test_no_percentage_is_reported():
    """
    There is deliberately no overall progress fraction.

    mission_executive sends its Nav2 goal with no feedback_callback, so
    distance-remaining is not published anywhere and progress within a
    leg is unobservable. P0.1's browser interpolated one from a
    hard-coded list; that is exactly what must not come back.
    """
    view = mv.normalise(_fields(LINE), receipt=_SIM_RECEIPT)
    for key, value in view.items():
        assert 'percent' not in key
        assert 'eta' not in key
        if isinstance(value, float):
            assert not 0.0 < value < 1.0 or key != 'progress'
    assert 'progress' not in view
