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
The perception policy, tested without a camera or a ROS graph.

Everything that decides what the robot believes is a module-level pure
function, so the cases that actually go wrong in vision — red wrapping
hue zero, a 0.0 range deprojecting to a confident wrong answer, a
sign-flipped lateral axis sending the robot to the wrong lane — are
asserted here rather than inferred from watching a simulator.

The colour tests derive their HSV from the targets' SDF colours through
OpenCV's own conversion, under BOTH the linear and the gamma-encoded
hypothesis for what gz writes, and across a brightness sweep. That makes
them a test of the claim the bands rest on (hue is invariant to uniform
shading) rather than a restatement of four hardcoded numbers.
"""

import math

import cv2

import numpy as np

import pytest

tf = pytest.importorskip('coco_perception.target_finder')

# Target colours come from the shared table; the rest are the things in
# the arena that must NEVER classify, quoted from where they are defined.
GREYS = {
    'platform': '0.6 0.6 0.62',        # full_world_robo.launch.py
    'obstacle': '0.55 0.55 0.57',      # coco_world.world
    'ramp': '0.55 0.55 0.60',          # ramp.sdf
    'wall': '0.9 0.9 0.9',             # coco_world.world
}
# coco_robo2.xacro's coco_orange, on the forearm. The one saturated
# non-target colour on the robot, and it sits between the red and yellow
# bands.
FOREARM_ORANGE = '0.9 0.45 0.1'

# gz may write linear RGB or gamma-encode it; the bands have to hold
# either way, so every colour test runs under both.
GAMMAS = (1.0, 2.2)
# Fully shadowed face through to a face square-on to both lights.
BRIGHTNESS = (0.4, 0.7, 1.0)


def hsv_of(rgb, gamma=2.2, scale=1.0):
    """Render an SDF colour triple the way OpenCV would see it."""
    linear = [scale * float(c) for c in rgb.split()]
    encoded = [min(1.0, c) ** (1.0 / gamma) for c in linear]
    bgr = np.array([[[round(255 * encoded[2]),
                      round(255 * encoded[1]),
                      round(255 * encoded[0])]]], dtype=np.uint8)
    return tuple(int(v) for v in cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[0][0])


def targets():
    from coco_config.robot import TARGETS
    return TARGETS


# ── colour policy ────────────────────────────────────────────────────────
@pytest.mark.parametrize('gamma', GAMMAS)
@pytest.mark.parametrize('scale', BRIGHTNESS)
def test_every_target_classifies_as_itself(gamma, scale):
    """
    The bands must hold under both render hypotheses and all lighting.

    Hue survives uniform shading exactly: white lights scale all three
    channels together, and a power-law encoding turns a scale of k into
    a scale of k**g, which is still uniform. Hue depends only on channel
    ratios. If this fails, that argument is wrong and the bands need
    re-deriving from a sampled frame, not widening.
    """
    for target in targets():
        hue, sat, val = hsv_of(target.rgb, gamma, scale)
        assert tf.classify_hsv(hue, sat, val) == target.colour, (
            f'{target.colour} rendered to H={hue} S={sat} V={val} '
            f'(gamma={gamma}, scale={scale})')


def test_red_wraps_hue_zero():
    """
    Both sides of the wrap are red, or half the mask goes missing.

    A single inRange over [0, 9] still finds a blob — just half of one,
    with its centroid pulled toward whichever side survived. That is a
    wrong position that looks entirely well-formed in the status line,
    which is why it gets its own test rather than being covered by the
    round-trip above.
    """
    assert tf.classify_hsv(2, 200, 200) == 'red'
    assert tf.classify_hsv(176, 200, 200) == 'red'
    assert len(tf.HSV_BANDS['red']) == 2, 'red needs two hue ranges'


@pytest.mark.parametrize('gamma', GAMMAS)
@pytest.mark.parametrize('name,rgb', sorted(GREYS.items()))
def test_no_grey_in_the_arena_is_ever_a_target(name, rgb, gamma):
    """
    M2 recoloured the whole arena for this.

    The obstacles used to be brownish-red and blue, which collide with
    two target colours in HSV. Everything neutral is what makes a
    saturation gate sufficient, so it is worth asserting rather than
    assuming.
    """
    for scale in BRIGHTNESS:
        hue, sat, val = hsv_of(rgb, gamma, scale)
        assert tf.classify_hsv(hue, sat, val) is None, (
            f'{name} rendered to H={hue} S={sat} V={val}')


@pytest.mark.parametrize('gamma', GAMMAS)
def test_the_orange_forearm_is_not_a_target(gamma):
    """
    The robot's own arm must not be picked up as an object.

    coco_orange lands between the red and yellow bands under both
    hypotheses. It is out of frame at every mission pose, but "out of
    frame" is a claim about poses and this is a claim about colour.
    """
    for scale in BRIGHTNESS:
        hue, sat, val = hsv_of(FOREARM_ORANGE, gamma, scale)
        assert tf.classify_hsv(hue, sat, val) is None, (
            f'forearm orange rendered to H={hue} at gamma={gamma}')


def test_no_two_bands_overlap():
    """One hue cannot be two colours; ambiguity would be silent."""
    for hue in range(180):
        matches = [colour for colour, bands in tf.HSV_BANDS.items()
                   if any(lo <= hue <= hi for lo, hi in bands)]
        assert len(matches) <= 1, f'hue {hue} matches {matches}'


def test_unsaturated_is_never_a_colour():
    """The saturation gate outranks hue, at every hue."""
    for hue in range(180):
        assert tf.classify_hsv(hue, tf.MIN_SATURATION - 1, 255) is None


def test_dark_is_never_a_colour():
    """A black pixel has a hue; it does not have a colour."""
    for hue in range(180):
        assert tf.classify_hsv(hue, 255, tf.MIN_VALUE - 1) is None


def test_an_unknown_target_colour_is_none_not_a_default():
    """
    Defaulting would run the whole mission and fetch the wrong object.

    The node keeps its previous selection and warns; it must never
    silently substitute one.
    """
    assert tf.normalise_colour('purple') is None
    assert tf.normalise_colour('') is None
    assert tf.normalise_colour(None) is None
    assert tf.normalise_colour('  BLUE ') == 'blue'


# ── geometry ─────────────────────────────────────────────────────────────
def test_deprojection_round_trips_to_the_camera_mount():
    """
    A point dead centre at 1 m must land 1 m ahead of the camera.

    This is the single number that silently poisons every grasp: get the
    optical-frame relabelling or the base_footprint offset wrong and
    every reported target is off by a fixed amount, with nothing in the
    output looking odd.
    """
    from coco_config.robot import CAMERA_XYZ, camera_intrinsics
    fx, fy, cx, cy = camera_intrinsics()
    point = tf.optical_to_base(*tf.deproject(cx, cy, 1.0, fx, fy, cx, cy))
    assert point == pytest.approx(
        (CAMERA_XYZ[0] + 1.0, CAMERA_XYZ[1], CAMERA_XYZ[2]))


def test_a_pixel_left_of_centre_is_a_positive_y():
    """
    REP-103 sign check. A flip sends the robot to the wrong lane.

    The lanes are 0.5 m apart and this error is worth exactly one lane
    in the wrong direction, which looks like drift rather than like a
    sign bug.
    """
    from coco_config.robot import camera_intrinsics
    fx, fy, cx, cy = camera_intrinsics()
    left = tf.optical_to_base(*tf.deproject(cx - 40, cy, 0.7, fx, fy, cx, cy))
    right = tf.optical_to_base(*tf.deproject(cx + 40, cy, 0.7, fx, fy, cx, cy))
    assert left[1] > 0.0
    assert right[1] < 0.0
    assert left[1] == pytest.approx(-right[1])


def test_a_positive_pitch_is_nose_down():
    """
    Pin the sign the camera decision rests on.

    An earlier plan asked for rpy "0 -0.6 0" to pitch the camera DOWN.
    In URDF a positive pitch rotates the forward axis toward -z, so that
    value aims it 34 degrees up into the back wall. If this test ever
    fails, the convention changed and coco_config's CAMERA_RPY comment
    is wrong too.
    """
    level = tf.optical_to_base(0.0, 0.0, 1.0, rpy=(0.0, 0.0, 0.0))
    pitched = tf.optical_to_base(0.0, 0.0, 1.0, rpy=(0.0, 0.5, 0.0))
    assert pitched[2] < level[2], 'positive pitch should look downward'
    assert pitched[0] < level[0], 'and give up forward reach for it'


def test_the_camera_extrinsics_are_actually_used():
    """
    Guard against the rotation being optimised into a constant.

    optical_to_base takes its extrinsics as defaulted arguments so a
    future camera move is a config edit. That only holds if the
    arguments reach the arithmetic.
    """
    moved = tf.optical_to_base(0.0, 0.0, 1.0, xyz=(1.0, 2.0, 3.0),
                               rpy=(0.0, 0.0, 0.0))
    assert moved == pytest.approx((2.0, 2.0, 3.0))


def test_front_surface_correction_moves_away_from_the_camera():
    """
    The mask sees the near surface; the grasp wants the axis.

    Systematic and one-directional, so it never averages out: 11 mm on a
    28 mm target, against a 27 mm approach window.
    """
    corrected = tf.surface_to_axis(0.700, 0.014)
    assert corrected > 0.700
    assert corrected == pytest.approx(0.700 + tf.SURFACE_TO_AXIS * 0.014)


# ── depth ────────────────────────────────────────────────────────────────
def test_nan_inf_and_zero_are_all_discarded():
    """All four are things gz writes for a miss, and each has to go."""
    values = [math.nan, math.inf, -math.inf, 0.0,
              0.72, 0.73, 0.72, 0.73, 0.725, 0.725]
    assert tf.robust_depth(values, 0.1, 8.0) == pytest.approx(0.725)


def test_all_invalid_returns_none_rather_than_zero():
    """
    A 0.0 range is a confident, well-formed, completely wrong answer.

    It deprojects to the camera's own origin, so the mission would drive
    to a target it believes is inside the robot.
    """
    assert tf.robust_depth([math.nan] * 20, 0.1, 8.0) is None
    assert tf.robust_depth([0.0] * 20, 0.1, 8.0) is None


def test_values_outside_the_clip_planes_are_discarded():
    """The depth camera's own clip is 0.1-8.0 m; outside it is noise."""
    values = [0.05] * 10 + [9.0] * 10 + [0.5] * 8
    assert tf.robust_depth(values, 0.1, 8.0) == pytest.approx(0.5)


def test_too_few_valid_pixels_is_no_answer():
    """
    A handful of surviving pixels is a silhouette, not an object.

    At the working distance the blob is ~9 px wide; a median over three
    pixels is as likely to be background as target.
    """
    assert tf.robust_depth([0.7, 0.7], 0.1, 8.0) is None


# ── blobs ────────────────────────────────────────────────────────────────
def _mask_with(rects, shape=(240, 320)):
    mask = np.zeros(shape, dtype=np.uint8)
    for x, y, w, h in rects:
        mask[y:y + h, x:x + w] = 255
    return mask


def test_blobs_come_back_largest_first_with_usable_labels():
    """
    Two same-coloured blobs must stay separable through to the depth.

    Sampling depth over the whole mask instead of one component would
    median two objects' ranges together and put the answer between them
    — plausible, and nowhere.
    """
    mask = _mask_with([(50, 60, 8, 45), (200, 100, 20, 90)])
    blobs, labels = tf.blob_stats(mask)
    assert len(blobs) == 2
    assert blobs[0][0] > blobs[1][0], 'largest first'
    big = blobs[0]
    assert (big[3], big[4]) == (20, 90)
    assert big[1] == pytest.approx(209.5)
    assert int((labels == big[5]).sum()) == big[0]
    assert int((labels == blobs[1][5]).sum()) == blobs[1][0]


def test_specks_below_the_floor_are_dropped():
    """One stray pixel is compression noise, not a target."""
    mask = _mask_with([(10, 10, 1, 1)])
    blobs, _ = tf.blob_stats(mask)
    assert blobs == []


def test_width_gate_accepts_a_target_and_rejects_a_wall():
    """
    The gate exists to reject nonsense, not to tell the four apart.

    At 0.7 m the four targets span 6.3-10.1 px, so any tolerance loose
    enough to survive segmentation noise also admits all four. Claiming
    it discriminates size would be the overclaim worth avoiding.
    """
    from coco_config.robot import camera_intrinsics
    fx = camera_intrinsics()[0]
    assert tf.plausible_blob(9, 0.028, 0.7, fx)
    assert not tf.plausible_blob(120, 0.028, 0.7, fx)
    assert not tf.plausible_blob(1, 0.028, 0.7, fx)


def test_the_width_gate_cannot_separate_adjacent_diameters():
    """
    State the limitation as a test so nobody builds on it.

    Adjacent sizes are 4 mm apart, which is 1.3 px of blob width at the
    working distance — a 24 mm target passes the 20 mm gate outright.
    The extremes (20 vs 32 mm) do separate, but by 0.6 px, which is
    inside the segmentation noise on a blob 6-10 px wide. Either way
    this is a sanity check on the range, not an identifier: colour is
    what says which object it is.
    """
    from coco_config.robot import camera_intrinsics
    fx = camera_intrinsics()[0]
    at_24 = tf.expected_width_px(0.024, 0.7, fx)
    assert tf.plausible_blob(at_24, 0.020, 0.7, fx), (
        'a 24 mm target passes the 20 mm gate — sizes are not separable')

    at_32 = tf.expected_width_px(0.032, 0.7, fx)
    margin = at_32 - 1.5 * tf.expected_width_px(0.020, 0.7, fx)
    assert 0.0 < margin < 1.0, (
        f'the extremes separate by {margin:.2f} px, which is either now '
        f'a real discriminator or no longer a distinction at all')


def test_a_red_mask_covers_both_sides_of_the_wrap():
    """The two-band OR has to survive into the image path, not just classify."""
    hsv = np.zeros((10, 20, 3), dtype=np.uint8)
    hsv[:, :10] = (2, 200, 200)      # just above the wrap
    hsv[:, 10:] = (176, 200, 200)    # just below it
    mask = tf.colour_mask(hsv, 'red')
    assert int((mask > 0).sum()) == 200, 'half the mask went missing'


# ── stamp pairing ────────────────────────────────────────────────────────
def test_an_exact_stamp_beats_a_nearer_neighbour_list():
    """Same sensor, same stamp — take it rather than searching."""
    assert tf.match_depth(1000, [999, 1000, 1001]) == 1000


def test_the_nearest_frame_wins_when_there_is_no_exact_match():
    exact = int(0.02 * 1e9)
    assert tf.match_depth(0, [exact, 3 * exact]) == exact


def test_a_depth_frame_older_than_the_tolerance_is_refused():
    """
    Pairing a fresh image with a stale depth gives a wrong range.

    While the robot is moving that error is proportional to speed, so it
    is largest exactly when the approach matters most. "No answer" has
    to be reachable.
    """
    stale = int(0.5 * 1e9)
    assert tf.match_depth(0, [stale], tolerance=0.1) is None
    assert tf.match_depth(0, []) is None


# ── status line ──────────────────────────────────────────────────────────
def test_status_splits_on_space_into_key_equals_value():
    """The panel parses with split(' '); a stray space shifts every field."""
    line = tf.format_status(sel='blue', found=1, u=161, v=118, area=372,
                            w=9, h=47, range=0.724, x=0.859, y=0.013,
                            z=0.079, lane=0.25, seen=['green', 'blue'],
                            age=0.03)
    parts = line.split(' ')
    fields = dict(p.split('=', 1) for p in parts)
    assert len(fields) == len(parts), 'a value contained a space'
    assert fields['sel'] == 'blue'
    assert fields['found'] == '1'
    assert fields['seen'] == 'green,blue'
    assert fields['y'].startswith('+'), 'lateral offset must be signed'
    assert fields['lane'].startswith('+')


def test_every_key_is_present_even_when_nothing_was_found():
    """
    A missing key would shift the panel's parse, not blank one field.

    '--' rather than an empty string for the same reason: two adjacent
    spaces collapse differently across parsers.
    """
    line = tf.format_status(sel='red', found=0)
    fields = dict(p.split('=', 1) for p in line.split(' '))
    assert set(fields) == set(tf.STATUS_KEYS)
    assert fields['range'] == '--'
    assert '' not in fields.values()


def test_an_empty_seen_list_is_a_word_not_a_gap():
    line = tf.format_status(sel='red', found=0, seen=[])
    fields = dict(p.split('=', 1) for p in line.split(' '))
    assert fields['seen'] == 'none'
