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

"""Telemetry shaping: status-line parsing, pose maths and downsampling."""

import math

from coco_web import telemetry as tele


# ── key=value status lines ─────────────────────────────────────────────

def test_parse_kv_line():
    """The shape both the arbiter and the executive emit."""
    assert tele.parse_kv_line('mode=nav active=nav teleop=--') == {
        'mode': 'nav', 'active': 'nav', 'teleop': '--'}


def test_parse_kv_line_tolerates_prose():
    """A stray word must not cost a telemetry frame."""
    assert tele.parse_kv_line('arbiter: mode=idle') == {'mode': 'idle'}


def test_parse_kv_line_on_junk():
    """Non-strings and empties parse to an empty dict, not an exception."""
    assert tele.parse_kv_line(None) == {}
    assert tele.parse_kv_line('') == {}


def test_arbiter_status_reports_the_active_source():
    """`active` is the field that says who owns the wheels."""
    parsed = tele.parse_arbiter_status(
        'mode=nav active=nav teleop=-- nav=0.05 rl=-- approach=--')
    assert parsed['online'] is True
    assert parsed['mode'] == 'nav'
    assert parsed['active'] == 'nav'
    assert parsed['ages']['nav'] == 0.05
    assert parsed['ages']['teleop'] is None


def test_arbiter_dash_becomes_none_not_a_dash():
    """'--' is the arbiter's 'never published'; the UI must not print it."""
    parsed = tele.parse_arbiter_status('mode=idle active=--')
    assert parsed['active'] is None


def test_arbiter_offline_when_nothing_was_heard():
    """An empty line means offline, and says so explicitly."""
    parsed = tele.parse_arbiter_status('')
    assert parsed['online'] is False
    assert parsed['active'] is None


def test_arbiter_keeps_the_raw_line():
    """When something is wrong, the raw line is what an engineer wants."""
    line = 'mode=nav active=nav'
    assert tele.parse_arbiter_status(line)['raw'] == line


def test_mission_state_active_flag():
    """The `active` flag answers: may the UI offer Start."""
    running = tele.parse_mission_state('state=CLIMB')
    assert running['state'] == 'CLIMB'
    assert running['active'] is True


def test_mission_state_does_not_invent_a_colour():
    """
    `/mission/state` has never carried a colour, so none may appear.

    This test replaces one that asserted `colour == 'red'` from the line
    `state=CLIMB colour=red`. That line is fiction:
    `mission_states.status_line()` emits twelve fields and `colour` is
    not among them, so the assertion only passed because the parser was
    reading a key nothing writes. On a real line it produced null, and
    the UI showed a colour only because the browser echoed its own
    selection back at itself.
    """
    real = ('state=CLIMB prev=ALIGN_FOR_CLIMB event=run elapsed=1.0 '
            'timeout=180 attempt=1 retries=0 owner=ramp_driver mode=rl '
            'reason=-- result=--')
    assert 'colour=' not in real
    assert tele.parse_mission_state(real)['colour'] is None


def test_mission_colour_comes_from_the_caller():
    """The authoritative colour is /mission/target_colour's, passed in."""
    parsed = tele.parse_mission_state('state=CLIMB', colour='blue')
    assert parsed['colour'] == 'blue'


def test_mission_detail_still_aliases_the_reason():
    """A P0.1 client reading `detail` must not lose its status line."""
    parsed = tele.parse_mission_state('state=ABORT reason=RETURN_FAILED')
    assert parsed['detail'] == 'RETURN_FAILED'
    assert parsed['reason'] == 'RETURN_FAILED'


def test_mission_state_carries_the_executives_own_fields():
    """The nine fields beyond `state` that P0.1 discarded."""
    line = ('state=GRASP prev=APPROACH_TARGET event=enter elapsed=4.5 '
            'timeout=180 attempt=2 retries=2 owner=grasp_server '
            'mode=idle reason=-- result=--')
    parsed = tele.parse_mission_state(line)
    assert parsed['previous'] == 'APPROACH_TARGET'
    assert parsed['event'] == 'enter'
    assert parsed['elapsed'] == 4.5
    assert parsed['timeout'] == 180.0
    assert parsed['attempt'] == 2
    assert parsed['retries'] == 2
    assert parsed['owner'] == 'grasp_server'
    assert parsed['phase'] == 'GRASPING'
    assert parsed['step'] == 10


# ── /grasp/status, the one topic with spaces in its values ─────────────

def test_grasp_phase_survives_spaces_in_the_label():
    """
    `/grasp/status` breaks the no-spaces invariant every parser assumes.

    grasp_server's step labels contain spaces ('hover above target'), so
    a key=value split yields `phase='pick:hover'` and silently drops the
    rest. parse_grasp_status slices to the next KNOWN key instead.
    """
    line = ('phase=pick:hover above target colour=blue x=0.1535 '
            'y=+0.0012 lifted=0 outcome=--')
    assert tele.parse_kv_line(line)['phase'] == 'pick:hover'
    assert tele.parse_grasp_status(line)['phase'] == 'pick:hover above target'


def test_grasp_outcome_survives_spaces_too():
    """Several outcome strings are whole sentences."""
    line = ('phase=idle colour=-- x=-- y=-- lifted=0 '
            'outcome=nothing held - nothing to place')
    parsed = tele.parse_grasp_status(line)
    assert parsed['outcome'] == 'nothing held - nothing to place'


def test_grasp_lifted_is_a_bool_from_the_wire_digit():
    """grasp_server renders bools as '1'/'0', not 'true'/'false'."""
    held = 'phase=pick colour=blue x=0.15 y=+0.00 lifted=1 outcome=held'
    down = 'phase=pick colour=blue x=0.15 y=+0.00 lifted=0 outcome=--'
    assert tele.parse_grasp_status(held)['lifted'] is True
    assert tele.parse_grasp_status(down)['lifted'] is False


def test_grasp_dash_becomes_none():
    """'--' is absent, and must not reach the UI as a dash."""
    line = 'phase=idle colour=-- x=-- y=-- lifted=0 outcome=--'
    parsed = tele.parse_grasp_status(line)
    assert parsed['outcome'] is None
    assert parsed['colour'] is None


def test_grasp_offline_is_reported_as_offline():
    """No grasp server is a state, not an exception."""
    parsed = tele.parse_grasp_status('')
    assert parsed['online'] is False
    assert parsed['phase'] is None


def test_mission_terminal_states_are_not_active():
    """IDLE, COMPLETE and ABORT all mean 'no mission is running'."""
    for state in ('IDLE', 'COMPLETE', 'ABORT'):
        parsed = tele.parse_mission_state(f'state={state}')
        assert parsed['active'] is False, state


def test_mission_offline_is_not_active():
    """No executive at all is also 'no mission running'."""
    parsed = tele.parse_mission_state('')
    assert parsed['online'] is False
    assert parsed['active'] is False


def test_mission_states_are_real_state_names():
    """Guards against the terminal list drifting from coco_mission."""
    import pytest
    pytest.importorskip('rclpy')
    import importlib.util
    import pathlib
    here = pathlib.Path(__file__).resolve().parents[2]
    states_py = here / 'coco_mission' / 'scripts' / 'mission_states.py'
    if not states_py.exists():
        pytest.skip('coco_mission sources not alongside this package')
    spec = importlib.util.spec_from_file_location('ms', states_py)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    for name in ('IDLE', 'COMPLETE', 'ABORT'):
        assert hasattr(module, name)
    assert set(module.TERMINAL_STATES) <= {'COMPLETE', 'ABORT'}


def test_perception_status_found_and_seen():
    """`seen` is what distinguishes 'blind' from 'looking at the wrong one'."""
    parsed = tele.parse_perception_status('found=0 seen=green')
    assert parsed['found'] is False
    assert parsed['seen'] == 'green'
    assert tele.parse_perception_status('found=1 seen=red')['found'] is True


# ── pose ───────────────────────────────────────────────────────────────

def test_yaw_from_identity_quaternion_is_zero():
    """The identity quaternion faces +x."""
    assert tele.yaw_from_quaternion(0.0, 0.0, 0.0, 1.0) == 0.0


def test_yaw_quarter_turn():
    """A 90 degree yaw round-trips."""
    half = math.sqrt(0.5)
    assert tele.yaw_from_quaternion(0.0, 0.0, half, half) == \
        pytest_approx(math.pi / 2)


def test_yaw_of_a_degenerate_quaternion_is_zero():
    """An all-zero quaternion must not raise or return NaN."""
    assert tele.yaw_from_quaternion(0.0, 0.0, 0.0, 0.0) == 0.0


def test_pose_payload_shape():
    """The pose object the browser draws from."""
    payload = tele.pose_payload((1.0, 2.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert payload == {'x': 1.0, 'y': 2.0, 'z': 0.0, 'yaw': 0.0}


def test_velocity_payload_shape():
    """The velocity readout object."""
    assert tele.velocity_payload(0.3, -0.1) == {
        'linear': 0.3, 'angular': -0.1}


# ── LiDAR downsampling ─────────────────────────────────────────────────

def test_downsample_returns_the_requested_point_count():
    """The output is bounded regardless of the scan's own resolution."""
    scan = tele.downsample_scan([1.0] * 1800, -math.pi, 0.0035, 10.0,
                                points=240)
    assert len(scan['ranges']) == 240


def test_downsample_never_upsamples():
    """A short scan is returned at its own length, not padded."""
    scan = tele.downsample_scan([1.0] * 10, 0.0, 0.1, 10.0, points=240)
    assert len(scan['ranges']) == 10


def test_downsample_keeps_the_nearest_return_in_each_bucket():
    """
    A thin obstacle must survive downsampling.

    Averaging a bucket against the empty space beside it is how a real
    obstacle disappears from the plot while still being there.
    """
    ranges = [9.0] * 100
    ranges[57] = 0.4
    scan = tele.downsample_scan(ranges, 0.0, 0.01, 10.0, points=10)
    assert min(r for r in scan['ranges'] if r is not None) == 0.4


def test_downsample_drops_non_finite_and_out_of_range():
    """No-return readings become gaps, not a false wall at max range."""
    ranges = [float('inf'), float('nan'), 50.0, 0.0]
    scan = tele.downsample_scan(ranges, 0.0, 0.1, 10.0, points=4)
    assert scan['ranges'] == [None, None, None, None]


def test_downsample_of_an_empty_scan():
    """No scan yet is an empty payload, not a crash."""
    scan = tele.downsample_scan([], 0.0, 0.0, 10.0)
    assert scan['ranges'] == []


def test_downsample_reports_the_lidar_floor():
    """
    The client needs the floor to read 0.15 as 'at or below', not 'at'.

    C2-NAV.49 measured this saturation, which is why min_scan_m is
    useless as a clearance metric.
    """
    scan = tele.downsample_scan([1.0] * 20, 0.0, 0.1, 10.0, points=5)
    assert scan['floor'] == tele.LIDAR_FLOOR_M


def test_downsample_angle_step_spans_the_original_arc():
    """The drawn arc must match the scan's, or the picture is rotated."""
    total, points = 1000, 250
    increment = 0.004
    scan = tele.downsample_scan([1.0] * total, -2.0, increment, 10.0,
                                points=points)
    span = scan['angle_step'] * points
    assert span == pytest_approx(increment * total)


# ── path ───────────────────────────────────────────────────────────────

def test_path_payload_passes_short_paths_through():
    """A short plan is not resampled."""
    assert tele.path_payload([(0.0, 0.0), (1.0, 1.0)]) == [[0.0, 0.0],
                                                           [1.0, 1.0]]


def test_path_payload_bounds_long_paths():
    """A long plan is strided down to the limit."""
    points = [(float(i), 0.0) for i in range(1000)]
    assert len(tele.path_payload(points, limit=120)) == 120


def test_path_payload_keeps_the_final_point():
    """The drawn path must end at the goal, not short of it."""
    points = [(float(i), 0.0) for i in range(1000)]
    assert tele.path_payload(points, limit=120)[-1] == [999.0, 0.0]


def test_path_payload_of_an_empty_plan():
    """No plan is an empty list."""
    assert tele.path_payload([]) == []


def pytest_approx(value):
    """Local approx helper, so this module needs no pytest import at top."""
    import pytest
    return pytest.approx(value, rel=1e-6, abs=1e-9)


# ── the robot drawn in the map frame (P0.2, second pass) ───────────────

def _close(a, b, tol=1e-6):
    """Compare two (x, y, yaw) tuples, yaw modulo 2 pi."""
    return (abs(a[0] - b[0]) < tol and abs(a[1] - b[1]) < tol
            and abs(math.atan2(math.sin(a[2] - b[2]),
                               math.cos(a[2] - b[2]))) < tol)


def test_compose_with_the_inverse_is_the_identity():
    """The algebra the tracker rests on."""
    for pose in ((1.0, 2.0, 0.3), (-4.0, 0.5, -2.9), (0.0, 0.0, math.pi)):
        assert _close(tele.compose2d(pose, tele.invert2d(pose)),
                      (0.0, 0.0, 0.0))
        assert _close(tele.compose2d(tele.invert2d(pose), pose),
                      (0.0, 0.0, 0.0))


def test_before_amcl_the_odometry_pose_is_reported_and_labelled_odom():
    """Exact at spawn, where the map origin is the spawn point."""
    tracker = tele.MapPoseTracker()
    assert _close(tracker.odom(1.0, 0.3, -0.2, 0.1), (0.3, -0.2, 0.1))
    assert tracker.frame == 'odom' and not tracker.localised


def test_the_live_failure_draws_the_robot_where_it_is():
    """
    Odometry drifted to (0.61, 3.71) on a robot that was home.

    Measured live at the end of a completed fetch: the page drew COCO
    outside the arena. With the correction from AMCL, the same odometry
    reading is drawn at AMCL's map pose, and later motion moves it by the
    odometry increment, rotated into the map.
    """
    tracker = tele.MapPoseTracker()
    drifted = (0.61, 3.71, -0.91)
    tracker.odom(100.0, *drifted)
    tracker.amcl(100.0, 0.02, -0.01, 0.10)
    assert tracker.frame == 'map'
    assert _close(tracker.odom(100.1, *drifted), (0.02, -0.01, 0.10))
    # One metre forward in odometry's own heading...
    ahead = tele.compose2d(drifted, (1.0, 0.0, 0.0))
    moved = tracker.odom(101.0, *ahead)
    # ...is one metre forward in the map heading AMCL reported.
    assert _close(moved, (0.02 + math.cos(0.10), -0.01 + math.sin(0.10),
                          0.10))


def test_the_correction_uses_the_odometry_at_the_amcl_stamp():
    """
    The localiser's pose is for a past scan: match odometry of that time.

    Matching it to the LATEST odometry would bake the robot's motion
    since then into the correction.
    """
    tracker = tele.MapPoseTracker()
    tracker.odom(1.0, 0.0, 0.0, 0.0)
    tracker.odom(2.0, 1.0, 0.0, 0.0)
    tracker.amcl(1.0, 5.0, 5.0, 0.0)       # the scan taken at t = 1
    assert _close(tracker.odom(2.0, 1.0, 0.0, 0.0), (6.0, 5.0, 0.0))


def test_the_odometry_history_is_bounded():
    """A long mission must not grow the tracker without limit."""
    tracker = tele.MapPoseTracker(history=10)
    for step in range(1000):
        tracker.odom(float(step), float(step), 0.0, 0.0)
    assert len(tracker._history) == 10
