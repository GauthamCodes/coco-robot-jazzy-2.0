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
C2-M4.0: the target pose pipeline, tested without a camera or a graph.

The cases here are the ones that produce a *confident wrong answer*
rather than an error — a depth of 0.0 deprojecting to the camera's own
origin, a lateral sign flip sending the arm to the mirror image of the
target, a blob chosen by where it happened to land in the image. Those
are the failures that survive a visual check of an RViz marker, which is
why C2-M4.0 is not allowed to be declared on one.

One test here is a *divergence* test rather than a behaviour test:
`test_optical_to_base_matches_the_urdf_chain` recomputes
`target_finder.optical_to_base` from the xacro's own joint rpy. That
function is a second copy of the robot's extrinsics, which CLAUDE.md
rule 3 forbids; C2-M4.0 replaces it with TF for the new pipeline but
cannot delete it without changing a measured approach. Pinning the two
together is what stops them drifting apart in the meantime.
"""

import importlib.util
import math
import os

from coco_config.robot import (approach_stop_x, approach_window,
                               camera_intrinsics, CAMERA_RPY, CAMERA_XYZ,
                               GRASP_MAX_LATERAL, target_by_colour,
                               TARGET_GRASP_Z)

import numpy as np

import pytest

tp = pytest.importorskip('coco_perception.target_pose')

FX, FY, CX, CY = camera_intrinsics()
INTRINSICS = (FX, FY, CX, CY)
NEAR, FAR = 0.15, 2.0
BLUE = target_by_colour('blue')


def blob(area, u, v, width, height, label):
    """Build a blob tuple in target_finder's order."""
    return (area, u, v, width, height, label)


def width_at(target, rng):
    """Compute the width a target of this diameter subtends."""
    return target.diameter * FX / rng


def axis_range(surface, target):
    """Surface range moved back to the axis, as the estimator does."""
    return surface + 0.8 * target.diameter / 2.0


# ═══════════════════════════════════════════════════════════════════════
# depth
# ═══════════════════════════════════════════════════════════════════════
class TestDepth:
    """gz writes four kinds of unusable depth and all four must go."""

    def test_valid_depths_give_the_median(self):
        est = tp.depth_statistics([0.50] * 10, NEAR, FAR, 0.014)
        assert est is not None
        assert est.surface == pytest.approx(0.50)

    def test_axis_correction_is_applied(self):
        # The mask covers the near face; the axis is r*pi/4 further.
        est = tp.depth_statistics([0.50] * 10, NEAR, FAR, 0.014)
        assert est.axis == pytest.approx(0.50 + 0.8 * 0.014)
        # And it is a real difference: 11.2 mm against a 5.5 mm window.
        assert est.axis - est.surface > 0.0055

    def test_all_nan_is_none_not_zero(self):
        # The whole point: a 0.0 range deprojects to the camera origin,
        # which is a well-formed, confident, completely wrong answer.
        assert tp.depth_statistics([float('nan')] * 20, NEAR, FAR,
                                   0.014) is None

    def test_all_inf_is_none(self):
        assert tp.depth_statistics([float('inf')] * 20, NEAR, FAR,
                                   0.014) is None
        assert tp.depth_statistics([float('-inf')] * 20, NEAR, FAR,
                                   0.014) is None

    def test_all_zero_is_none(self):
        assert tp.depth_statistics([0.0] * 20, NEAR, FAR, 0.014) is None

    def test_beyond_the_far_gate_is_none(self):
        assert tp.depth_statistics([5.0] * 20, NEAR, FAR, 0.014) is None

    def test_too_few_valid_is_none(self):
        values = [0.5] * 5 + [float('nan')] * 40
        assert tp.depth_statistics(values, NEAR, FAR, 0.014,
                                   min_valid=6) is None

    def test_enough_valid_survives_a_mostly_bad_blob(self):
        values = [0.5] * 6 + [float('nan')] * 40
        est = tp.depth_statistics(values, NEAR, FAR, 0.014, min_valid=6)
        assert est is not None
        assert est.surface == pytest.approx(0.5)
        assert est.valid_px == 6
        assert est.total_px == 46

    def test_median_resists_one_sided_background_contamination(self):
        # Silhouette pixels carry the background, which is FURTHER. A
        # mean would be dragged out; the median holds while the good
        # pixels are the majority.
        values = [0.50] * 30 + [1.20] * 14
        est = tp.depth_statistics(values, NEAR, FAR, 0.014)
        assert est.surface == pytest.approx(0.50)
        assert np.mean(values) > 0.65      # what a mean would have given

    def test_valid_fraction_and_spread_are_reported(self):
        values = [0.50] * 8 + [float('nan')] * 2
        est = tp.depth_statistics(values, NEAR, FAR, 0.014)
        assert est.valid_fraction == pytest.approx(0.8)
        assert est.spread == pytest.approx(0.0)

    def test_spread_is_a_median_absolute_deviation(self):
        # One far outlier must not dominate, which is the whole reason
        # this is a MAD and not a standard deviation.
        values = [0.50] * 20 + [1.50]
        est = tp.depth_statistics(values, NEAR, FAR, 0.014)
        assert est.spread == pytest.approx(0.0, abs=1e-9)
        assert np.std(values) > 0.2

    def test_noisy_depth_still_lands_near_the_truth(self):
        rng = np.random.default_rng(0)
        values = 0.50 + rng.normal(0.0, 0.002, 200)
        est = tp.depth_statistics(values, NEAR, FAR, 0.014)
        assert est.surface == pytest.approx(0.50, abs=0.001)


# ═══════════════════════════════════════════════════════════════════════
# selection
# ═══════════════════════════════════════════════════════════════════════
class TestSelection:
    """Which blob is the target, and why not the others."""

    def _depths(self, mapping):
        return lambda label: mapping[label]

    def test_no_blobs_is_not_detected(self):
        found, est, reason = tp.select_candidate(
            [], self._depths({}), BLUE, FX, NEAR, FAR)
        assert found is None and est is None
        assert reason == tp.NOT_DETECTED

    def test_blob_with_unusable_depth_is_depth_invalid(self):
        # DETECTED BUT DEPTH INVALID is a different fault from NOT
        # DETECTED and has to be distinguishable — §13.
        blobs = [blob(60, 160.0, 120.0, 12, 30, 1)]
        found, _est, reason = tp.select_candidate(
            blobs, self._depths({1: [float('nan')] * 60}), BLUE, FX,
            NEAR, FAR)
        assert found is None
        assert reason == tp.DEPTH_INVALID

    def test_blob_of_the_wrong_size_is_implausible_not_invalid(self):
        # Depth was measurable; the object simply is not a target.
        blobs = [blob(600, 160.0, 120.0, 120, 30, 1)]
        found, _est, reason = tp.select_candidate(
            blobs, self._depths({1: [0.5] * 600}), BLUE, FX, NEAR, FAR)
        assert found is None
        assert reason == tp.IMPLAUSIBLE_SIZE

    def test_a_good_blob_is_accepted(self):
        rng_axis = axis_range(0.5, BLUE)
        w = int(round(width_at(BLUE, rng_axis)))
        blobs = [blob(60, 160.0, 120.0, w, 30, 1)]
        found, est, reason = tp.select_candidate(
            blobs, self._depths({1: [0.5] * 60}), BLUE, FX, NEAR, FAR)
        assert reason == tp.VALID
        assert found is blobs[0]
        assert est.axis == pytest.approx(rng_axis)

    def test_largest_blob_wins_regardless_of_array_order(self):
        # connectedComponentsWithStats hands back components in raster
        # order of their labels, which is a property of where things sit
        # in the frame. Selecting on that is the silent, framing-
        # dependent choice §12 forbids.
        near_r = axis_range(0.40, BLUE)
        far_r = axis_range(0.80, BLUE)
        big = blob(200, 100.0, 120.0,
                   int(round(width_at(BLUE, near_r))), 40, 1)
        small = blob(50, 220.0, 120.0,
                     int(round(width_at(BLUE, far_r))), 20, 2)
        depths = self._depths({1: [0.40] * 200, 2: [0.80] * 50})
        for order in ([big, small], [small, big]):
            found, est, reason = tp.select_candidate(
                sorted(order, key=lambda b: -b[0]), depths, BLUE, FX,
                NEAR, FAR)
            assert reason == tp.VALID
            assert found is big
            # Largest is also nearest: identical cylinders, area ~ 1/r^2.
            assert est.axis == pytest.approx(near_r)

    def test_selection_falls_through_to_the_next_candidate(self):
        # A big blob with no depth must not veto a good smaller one.
        good_r = axis_range(0.5, BLUE)
        bad = blob(300, 50.0, 120.0, 40, 60, 1)
        good = blob(60, 160.0, 120.0,
                    int(round(width_at(BLUE, good_r))), 30, 2)
        depths = self._depths({1: [float('inf')] * 300, 2: [0.5] * 60})
        found, _est, reason = tp.select_candidate(
            [bad, good], depths, BLUE, FX, NEAR, FAR)
        assert reason == tp.VALID
        assert found is good

    def test_unknown_colour_never_detects(self):
        obs = tp.observe([], lambda label: [], 'magenta', INTRINSICS,
                         NEAR, FAR)
        assert obs.validity == tp.NOT_DETECTED
        assert obs.point is None

    @pytest.mark.parametrize('colour', ('red', 'green', 'blue', 'yellow'))
    def test_every_colour_runs_the_same_path(self, colour):
        # §11: one pipeline, configured by colour. No per-colour branch.
        target = target_by_colour(colour)
        rng_axis = axis_range(0.5, target)
        w = int(round(width_at(target, rng_axis)))
        blobs = [blob(60, 160.0, 120.0, w, 30, 1)]
        obs = tp.observe(blobs, lambda label: [0.5] * 60, colour,
                         INTRINSICS, NEAR, FAR)
        assert obs.validity == tp.VALID, colour
        assert obs.depth.axis == pytest.approx(rng_axis)


# ═══════════════════════════════════════════════════════════════════════
# geometry
# ═══════════════════════════════════════════════════════════════════════
class TestGeometry:
    """The pinhole model and, more importantly, its signs."""

    def test_centre_pixel_deprojects_onto_the_optical_axis(self):
        assert tp.deproject(CX, CY, 0.5, FX, FY, CX, CY) == (
            pytest.approx((0.0, 0.0, 0.5)))

    def test_right_of_centre_is_positive_x_optical(self):
        x, y, z = tp.deproject(CX + 20.0, CY, 0.5, FX, FY, CX, CY)
        assert x > 0.0 and y == pytest.approx(0.0) and z == 0.5

    def test_below_centre_is_positive_y_optical(self):
        # REP-103 optical frame: y is DOWN. Getting this backwards
        # mirrors the target vertically and is invisible in a top-down
        # marker view.
        _x, y, _z = tp.deproject(CX, CY + 20.0, 0.5, FX, FY, CX, CY)
        assert y > 0.0

    def test_known_pixel_and_depth_give_the_hand_computed_point(self):
        u, v, z = CX + 30.0, CY - 10.0, 0.60
        x, y, zz = tp.deproject(u, v, z, FX, FY, CX, CY)
        assert x == pytest.approx(30.0 * z / FX)
        assert y == pytest.approx(-10.0 * z / FY)
        assert zz == pytest.approx(z)

    def test_expected_width_falls_as_one_over_range(self):
        assert (tp.expected_width_px(0.028, 0.5, FX)
                == pytest.approx(2 * tp.expected_width_px(0.028, 1.0, FX)))

    def test_expected_width_rejects_a_zero_range(self):
        assert tp.expected_width_px(0.028, 0.0, FX) is None
        assert tp.plausible_width(10, 0.028, 0.0, FX) is False

    def test_optical_to_base_matches_the_urdf_chain(self):
        """
        Check `target_finder.optical_to_base` against the xacro's rpy.

        camera_optical_joint is rpy (-pi/2, 0, -pi/2) from camera_link.
        URDF rpy is fixed-axis XYZ, i.e. R = Rz(y)Ry(p)Rx(r), so the
        optical->link relabelling is R applied to the optical point.
        Recomputing it from the angles rather than quoting (z,-x,-y) is
        what makes this a check and not a restatement.
        """
        tf = pytest.importorskip('coco_perception.target_finder')

        def rot(axis, angle):
            c, s = math.cos(angle), math.sin(angle)
            if axis == 'x':
                return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
            if axis == 'y':
                return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
            return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

        r, p, y = -math.pi / 2, 0.0, -math.pi / 2
        chain = rot('z', y) @ rot('y', p) @ rot('x', r)
        for point in [(0.0, 0.0, 0.5), (0.1, -0.2, 0.6), (-0.05, 0.03, 0.9)]:
            in_link = chain @ np.asarray(point)
            expected = tuple(in_link + np.asarray(CAMERA_XYZ))
            assert tf.optical_to_base(*point) == pytest.approx(expected)

    def test_camera_rpy_is_still_zero(self):
        # Two other tests assert this; a -0.6 rad pitch was proposed and
        # is wrong in both sign and magnitude. If it ever changes, the
        # composition above stops being a pure relabelling.
        assert CAMERA_RPY == (0.0, 0.0, 0.0)

    def test_a_target_dead_ahead_lands_on_the_robot_x_axis(self):
        tf = pytest.importorskip('coco_perception.target_finder')
        x, y, z = tf.optical_to_base(*tp.deproject(CX, CY, 0.5,
                                                   FX, FY, CX, CY))
        assert x == pytest.approx(0.5 + CAMERA_XYZ[0])
        assert y == pytest.approx(0.0)
        assert z == pytest.approx(CAMERA_XYZ[2])

    def test_a_target_right_of_centre_is_negative_y_in_base(self):
        # Right in the image is the robot's RIGHT, which is -y in ROS.
        # This sign is what sends the approach to the wrong lane.
        tf = pytest.importorskip('coco_perception.target_finder')
        _x, y, _z = tf.optical_to_base(
            *tp.deproject(CX + 30.0, CY, 0.5, FX, FY, CX, CY))
        assert y < 0.0


# ═══════════════════════════════════════════════════════════════════════
# transform + output
# ═══════════════════════════════════════════════════════════════════════
class TestTransformAndOutput:
    """What a consumer actually receives, and can it be misread."""

    def _valid(self, colour='blue', surface=0.5):
        target = target_by_colour(colour)
        rng_axis = axis_range(surface, target)
        w = int(round(width_at(target, rng_axis)))
        blobs = [blob(60, CX, CY, w, 30, 1)]
        return tp.observe(blobs, lambda label: [surface] * 60, colour,
                          INTRINSICS, NEAR, FAR)

    def test_an_untransformed_observation_is_in_the_optical_frame(self):
        obs = self._valid()
        assert obs.frame_id == 'camera_optical_frame'

    def test_a_valid_observation_cannot_exist_without_a_frame(self):
        # The ambiguous output §4 forbids — x/y/z with no frame_id —
        # is not representable.
        with pytest.raises(ValueError):
            tp.TargetObservation('blue', tp.VALID, point=(1.0, 2.0, 3.0))
        with pytest.raises(ValueError):
            tp.TargetObservation('blue', tp.VALID, frame_id='base_footprint')

    def test_an_unknown_validity_is_rejected(self):
        with pytest.raises(ValueError):
            tp.TargetObservation('blue', 'PROBABLY_FINE')

    def test_transform_sets_the_destination_frame(self):
        obs = tp.transform_observation(
            self._valid(), lambda p: (0.6, 0.0, 0.07), 'base_footprint')
        assert obs.frame_id == 'base_footprint'
        assert obs.point == pytest.approx((0.6, 0.0, 0.07))

    def test_grasp_point_takes_its_height_from_the_arm_not_the_camera(self):
        # The vertical blob centroid is framing-dependent; the magnet
        # binds at a height the arm's geometry fixes.
        obs = tp.transform_observation(
            self._valid(), lambda p: (0.6, 0.012, 0.9), 'base_footprint')
        assert obs.grasp_point[2] == pytest.approx(TARGET_GRASP_Z)
        assert obs.grasp_point[:2] == pytest.approx((0.6, 0.012))

    def test_transform_leaves_an_invalid_observation_alone(self):
        obs = tp.TargetObservation('blue', tp.NOT_DETECTED)
        out = tp.transform_observation(obs, lambda p: (9, 9, 9), 'base')
        assert out.point is None
        assert out.frame_id is None

    def test_status_carries_every_key(self):
        line = tp.status_for(self._valid())
        keys = [field.split('=')[0] for field in line.split(' ')]
        assert keys == list(tp.STATUS_KEYS)

    def test_status_renders_missing_values_as_a_placeholder(self):
        # A blank between two spaces collapses when a panel splits on
        # ' ' and shifts every field after it.
        line = tp.status_for(tp.TargetObservation('blue', tp.NOT_DETECTED))
        assert 'validity=NOT_DETECTED' in line
        assert 'x=--' in line and 'range=--' in line
        assert '  ' not in line

    def test_no_status_value_contains_a_space(self):
        line = tp.status_for(self._valid())
        assert len(line.split(' ')) == len(tp.STATUS_KEYS)

    def test_status_reports_the_frame_it_is_in(self):
        obs = tp.transform_observation(
            self._valid(), lambda p: (0.6, 0.0, 0.07), 'base_footprint')
        assert 'frame=base_footprint' in tp.status_for(obs)

    @pytest.mark.parametrize('validity', (
        tp.NOT_DETECTED, tp.DEPTH_INVALID, tp.IMPLAUSIBLE_SIZE,
        tp.NO_TRANSFORM, tp.STALE_TRANSFORM))
    def test_every_non_valid_state_renders(self, validity):
        line = tp.status_for(tp.TargetObservation('blue', validity))
        assert f'validity={validity}' in line


# ═══════════════════════════════════════════════════════════════════════
# reachability
# ═══════════════════════════════════════════════════════════════════════
def _load_arm_ik():
    """Load arm_ik from source so the test uses the real solver."""
    for candidate in (
            os.path.join(os.path.dirname(__file__), '..', '..',
                         'coco_moveit_config', 'scripts', 'arm_ik.py'),):
        path = os.path.abspath(candidate)
        if os.path.exists(path):
            spec = importlib.util.spec_from_file_location('arm_ik', path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module
    return None


ARM_IK = _load_arm_ik()


class TestReachability:
    """Does perception's answer reach the manipulation interface."""

    def test_no_solver_is_a_reported_state_not_a_crash(self):
        assert tp.classify_reachability(0.152, 0.0, TARGET_GRASP_Z,
                                        ik=None) == tp.IK_UNAVAILABLE

    def test_off_the_arm_plane_is_rejected_before_ik_is_asked(self):
        # The arm is planar: both joints turn about base-y. A target off
        # y=0 is unreachable at ANY joint angle, and a planar solver
        # would answer confidently about a point the gripper will miss.
        def never_called(x, z):
            raise AssertionError('IK must not be consulted off-plane')

        assert tp.classify_reachability(
            0.152, GRASP_MAX_LATERAL + 0.001, TARGET_GRASP_Z,
            ik=never_called) == tp.OFF_ARM_PLANE

    @pytest.mark.skipif(ARM_IK is None, reason='arm_ik not found')
    def test_the_verified_grasp_pose_is_reachable(self):
        assert tp.classify_reachability(
            0.152, 0.0, TARGET_GRASP_Z,
            ik=ARM_IK.ik_or_none) == tp.REACHABLE

    @pytest.mark.skipif(ARM_IK is None, reason='arm_ik not found')
    def test_a_target_at_detection_range_is_out_of_the_workspace(self):
        # This is the honest answer at 0.5 m and must not be dressed up:
        # the arm reaches to base-x ~0.157 and the robot has not driven
        # yet.
        assert tp.classify_reachability(
            0.60, 0.0, TARGET_GRASP_Z,
            ik=ARM_IK.ik_or_none) == tp.OUT_OF_WORKSPACE

    @pytest.mark.skipif(ARM_IK is None, reason='arm_ik not found')
    def test_hover_and_grasp_are_both_required(self):
        # arm_ik reaches 0.16085 at the grasp height but only 0.15651 at
        # the hover; testing only the grasp height advertises 4.3 mm of
        # window that cannot be entered.
        ik = ARM_IK.ik_or_none
        assert ik(0.1600, TARGET_GRASP_Z) is not None
        assert tp.classify_reachability(
            0.1600, 0.0, TARGET_GRASP_Z, ik=ik) == tp.OUT_OF_WORKSPACE

    @pytest.mark.skipif(ARM_IK is None, reason='arm_ik not found')
    @pytest.mark.parametrize('colour', ('red', 'green', 'blue', 'yellow'))
    def test_after_the_approach_every_colour_is_reachable_on_axis(self,
                                                                  colour):
        assert tp.reachability_after_approach(
            0.6, 0.0, colour, ik=ARM_IK.ik_or_none) == tp.REACHABLE

    @pytest.mark.skipif(ARM_IK is None, reason='arm_ik not found')
    def test_after_the_approach_lateral_error_is_what_decides(self):
        # The approach drives straight forward, so it fixes x and leaves
        # y alone. Perception's y is therefore the whole of what decides
        # post-approach feasibility — the quantity C2-M4.1 must bound.
        ik = ARM_IK.ik_or_none
        assert tp.reachability_after_approach(
            0.6, GRASP_MAX_LATERAL - 0.001, 'blue', ik=ik) == tp.REACHABLE
        assert tp.reachability_after_approach(
            0.6, GRASP_MAX_LATERAL + 0.001, 'blue',
            ik=ik) == tp.OFF_ARM_PLANE

    def test_grasp_window_state_reports_the_drive_still_needed(self):
        inside, drive = tp.grasp_window_state(0.60, 'blue')
        assert inside is False
        assert drive == pytest.approx(0.60 - approach_stop_x('blue'))
        assert drive > 0.0                     # forward

    def test_grasp_window_state_accepts_the_window_centre(self):
        inside, drive = tp.grasp_window_state(approach_stop_x('blue'), 'blue')
        assert inside is True
        assert drive == pytest.approx(0.0)

    @pytest.mark.parametrize('colour', ('red', 'green', 'blue', 'yellow'))
    def test_the_window_is_the_same_for_every_colour(self, colour):
        # The self-collision bound at 0.150 dominates all four
        # diameters: at this depth the arm has one grasp pose, not four.
        assert approach_window(colour) == approach_window('blue')

    def test_unknown_colour_has_no_window(self):
        inside, drive = tp.grasp_window_state(0.15, 'magenta')
        assert inside is False and drive is None
        assert tp.reachability_after_approach(
            0.15, 0.0, 'magenta') == tp.REACH_UNKNOWN

    @pytest.mark.skipif(ARM_IK is None, reason='arm_ik not found')
    def test_transform_fills_in_both_reach_verdicts(self):
        target = target_by_colour('blue')
        rng_axis = axis_range(0.5, target)
        blobs = [blob(60, CX, CY, int(round(width_at(target, rng_axis))),
                      30, 1)]
        obs = tp.observe(blobs, lambda label: [0.5] * 60, 'blue',
                         INTRINSICS, NEAR, FAR)
        obs = tp.transform_observation(
            obs, lambda p: (0.62, 0.001, 0.068), 'base_footprint',
            ik=ARM_IK.ik_or_none)
        assert obs.reach == tp.OUT_OF_WORKSPACE       # 0.62 m away
        assert obs.reach_after_approach == tp.REACHABLE
        assert obs.drive_distance > 0.4
        line = tp.status_for(obs)
        assert 'reach=OUT_OF_WORKSPACE' in line
        assert 'reach_appr=REACHABLE' in line


# ═══════════════════════════════════════════════════════════════════════
# the bounds the verdict rests on
# ═══════════════════════════════════════════════════════════════════════
def test_bounds_are_the_measured_ones():
    """Pinned so a silent edit to coco_config shows up here."""
    assert tp.BOUNDS['self_collision_x'] == 0.150
    assert tp.BOUNDS['reach_x_max'] == 0.1565
    assert tp.BOUNDS['grasp_z'] == 0.128
    assert tp.BOUNDS['max_lateral'] == 0.010
    near, far = approach_window('blue')
    assert far - near == pytest.approx(0.0055, abs=1e-4)


# ═══════════════════════════════════════════════════════════════════════
# C2-M4.1: the opt-in PointStamped that lets this node drive the mission
# ═══════════════════════════════════════════════════════════════════════
# target_pose_node imports rclpy, cv_bridge and tf2, so it cannot be
# imported here the way target_pose can. These read its source instead —
# the same technique test_camera_rpy_is_still_zero uses on the xacro, and
# for the same reason: the property being protected is a contract, and a
# contract is worth pinning even when the object holding it will not load.
NODE_SOURCE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'coco_perception',
    'target_pose_node.py')


def _node_source():
    with open(NODE_SOURCE) as handle:
        return handle.read()


def _point_topic_default():
    """Read the declared default of `point_topic` out of the AST."""
    import ast
    tree = ast.parse(_node_source())
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'declare_parameter'
                and node.args
                and isinstance(node.args[0], ast.Constant)
                and node.args[0].value == 'point_topic'):
            return node.args[1].value
    raise AssertionError('point_topic is not declared at all')


class TestPointTopicContract:
    """The C2-M4.1 integration seam, and the ways it could go wrong."""

    def test_point_topic_defaults_to_off(self):
        # Not a preference. target_finder owns /perception/target on the
        # measured mission path; a default that published there would put
        # two estimates on one topic and let the race pick the grasp.
        assert _point_topic_default() == ''

    def test_the_publisher_is_conditional_on_the_parameter(self):
        source = _node_source()
        assert 'if point_topic else' in source
        assert 'self._point_pub = (' in source

    def test_the_axis_point_is_published_not_the_grasp_point(self):
        # grasp_point.z is TARGET_GRASP_Z from the arm's geometry, not a
        # measurement. Publishing it on the topic a visual servo reads
        # would hand the servo a number the camera never saw.
        source = _node_source()
        assert 'point.point.x = observation.point[0]' in source
        assert 'point.point.z = observation.point[2]' in source
        assert 'point.point.z = observation.grasp_point[2]' not in source

    def test_the_point_carries_the_image_stamp(self):
        # approach_server ages this stamp to decide the fix is fresh. A
        # `now` stamp would make a frozen pipeline look live.
        source = _node_source()
        assert 'point.header.stamp = stamp' in source
        assert 'point.header.frame_id = observation.frame_id' in source

    def test_nothing_is_published_when_the_observation_is_not_valid(self):
        # The publish sits inside the `if observation.is_valid:` branch,
        # so a DEPTH_INVALID or NO_TRANSFORM frame leaves a gap on the
        # topic rather than a confident wrong point.
        source = _node_source()
        valid_branch = source.index('if observation.is_valid:')
        publish = source.index('self._point_pub.publish(point)')
        array_publish = source.index('self._pose_pub.publish(array)')
        assert valid_branch < publish < array_publish

    def test_the_point_message_type_is_imported(self):
        assert 'from geometry_msgs.msg import PointStamped' in _node_source()


# ═══════════════════════════════════════════════════════════════════════
# C2-M4.2: the swap the mission executive actually needs
# ═══════════════════════════════════════════════════════════════════════
# point_topic (C2-M4.1) feeds the approach. It is not enough on its own:
# mission_states._check_search_target gates SEARCH_TARGET on
# /perception/status reading found=1, and that topic was target_finder's.
# These pin the second half of the handover, and the launch-file
# invariant that exactly one node can own either topic.
def _declared_default(name):
    """Read a declare_parameter default out of the node's AST."""
    import ast
    tree = ast.parse(_node_source())
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'declare_parameter'
                and node.args
                and isinstance(node.args[0], ast.Constant)
                and node.args[0].value == name):
            return node.args[1].value
    raise AssertionError(f'{name} is not declared at all')


def _finder_line(observation):
    """Render an observation the way the compat publisher does."""
    from coco_perception.target_finder import format_status
    return format_status(**tp.finder_status_fields(observation))


def _fields(line):
    """Split a key=value status line the way the executive does."""
    return dict(part.split('=', 1) for part in line.split(' '))


def _valid_observation():
    """Build a VALID observation with a blob, a depth and a point."""
    obs = tp.TargetObservation(
        'blue', tp.VALID,
        point=(0.1534, 0.0017, 0.128),
        grasp_point=(0.1534, 0.0017, 0.128),
        frame_id='base_footprint',
        blob=blob(64, 160.4, 120.2, 8.0, 40.0, 1),
        depth=tp.DepthEstimate(0.500, 0.5112, 60, 64, 0.001),
        seen=['blue'])
    return obs


class TestFinderStatusCompat:
    """The vision gate, answered by the new pipeline's own verdict."""

    def test_valid_reports_found_1(self):
        # The exact string _check_search_target compares against.
        assert _fields(_finder_line(_valid_observation()))['found'] == '1'

    @pytest.mark.parametrize('validity', [v for v in tp.VALIDITIES
                                          if v != tp.VALID])
    def test_every_non_valid_state_reports_found_0(self, validity):
        obs = tp.TargetObservation('blue', validity)
        assert _fields(_finder_line(obs))['found'] == '0'

    def test_the_gate_reads_the_colour_the_mission_asked_for(self):
        # A mismatch here is TARGET_COLOUR_MISMATCH, a mission failure
        # rather than a wrong object — which is the point of the check.
        assert _fields(_finder_line(_valid_observation()))['sel'] == 'blue'

    def test_the_keys_are_target_finders_exactly(self):
        # The compat line has to BE a /perception/status line, not
        # merely resemble one: mission_hud and traverse_demo.py split it
        # on ' ' by position-independent key, but a missing key reads as
        # None and a spurious one is undefined.
        from coco_perception.target_finder import STATUS_KEYS
        assert tuple(_fields(_finder_line(_valid_observation()))) \
            == STATUS_KEYS

    def test_no_value_contains_a_space(self):
        # A space inside a value shifts every field after it when the
        # panel splits, silently.
        for part in _finder_line(_valid_observation()).split(' '):
            assert part.count('=') >= 1

    def test_geometry_is_withheld_unless_valid(self):
        # target_finder emits u v area w h range x y z only with a fix.
        # A compat line whose job is substitutability must behave the
        # same, not merely carry the same key names.
        obs = tp.TargetObservation(
            'blue', tp.DEPTH_INVALID,
            blob=blob(64, 160.4, 120.2, 8.0, 40.0, 1), seen=['blue'])
        fields = _fields(_finder_line(obs))
        for key in ('u', 'v', 'area', 'w', 'h', 'range', 'x', 'y', 'z'):
            assert fields[key] == '--', key
        # but the diagnosis still gets through
        assert fields['seen'] == 'blue'

    def test_range_is_the_axis_not_the_surface(self):
        # target_finder passes surface_to_axis()'s output. Publishing
        # the near-face range instead would be an 11.2 mm error on a
        # 5.5 mm window — like-for-like, not merely same-named.
        obs = _valid_observation()
        assert _fields(_finder_line(obs))['range'] \
            == f'{obs.depth.axis:.3f}'

    def test_lane_and_age_are_absent_rather_than_invented(self):
        # This pipeline computes neither. '--' is the honest answer;
        # a plausible number would be the failure the convention exists
        # to prevent.
        fields = _fields(_finder_line(_valid_observation()))
        assert fields['lane'] == '--'
        assert fields['age'] == '--'


class TestStatusCompatTopicContract:
    """The parameter, and that it is off unless deliberately set."""

    def test_status_compat_topic_defaults_to_off(self):
        # Same reason point_topic does: target_finder owns
        # /perception/status on the measured path, and two publishers
        # on it is a vision gate decided by a race.
        assert _declared_default('status_compat_topic') == ''

    def test_the_publisher_is_conditional_on_the_parameter(self):
        source = _node_source()
        assert 'if compat_topic else' in source
        assert 'self._compat_pub = (' in source

    def test_it_is_a_separate_topic_from_the_nodes_own_status(self):
        # /perception/target_pose/status keeps its C2-M4.0 format. The
        # compat line is an addition, never a replacement — reusing one
        # topic for two formats would break whichever reader lost.
        assert _declared_default('status_topic') \
            == '/perception/target_pose/status'

    def test_the_compat_line_is_published_on_the_status_timer(self):
        # The executive ages this topic against the state's entry time,
        # so it must keep arriving whether or not a frame did.
        source = _node_source()
        body = source[source.index('def _publish_status'):]
        assert 'self._compat_pub.publish' in body


# ── the launch-file invariant ────────────────────────────────────────────
def _perception_launch():
    """Load perception.launch.py by path; it is not an importable module."""
    import importlib.util
    path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'launch',
        'perception.launch.py')
    spec = importlib.util.spec_from_file_location('perception_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _nodes_for(source):
    """Run the launch file's dispatch for one target_source value."""
    from launch import LaunchContext
    module = _perception_launch()
    context = LaunchContext()
    context.launch_configurations['target_source'] = source
    return module, module.perception_node(context)


def _literals(obj):
    """
    Collect every literal string inside a nested substitution structure.

    A launch_ros Node keeps its parameters as unresolved substitution
    objects until launch time, so `str()` on them prints object reprs
    and an `in` test against one passes or fails for the wrong reason.
    This walks the structure and returns only the text that is actually
    literal, which is what a topic name set by the launch file is.

    Parameter values arrive YAML-encoded, with a document-end marker on
    a line of its own after the value, so each literal is normalised to
    its first line. Comparing the raw string would fail for a reason
    that has nothing to do with the invariant being tested.
    """
    from launch.substitutions import TextSubstitution
    if isinstance(obj, TextSubstitution):
        return [obj.text.splitlines()[0].strip() if obj.text else obj.text]
    if isinstance(obj, str):
        return [obj.splitlines()[0].strip() if obj else obj]
    if isinstance(obj, dict):
        found = []
        for key, value in obj.items():
            found += _literals(key) + _literals(value)
        return found
    if isinstance(obj, (list, tuple, set)):
        found = []
        for item in obj:
            found += _literals(item)
        return found
    return []


class TestExactlyOneTargetPublisher:
    """
    The invariant the whole swap rests on: one publisher, always.

    Two publishers on /perception/target is two estimates racing and a
    grasp decided by whichever landed last. This is checked at the
    dispatch rather than on a live graph because a graph test can only
    find it after a run has already been spent.
    """

    @pytest.mark.parametrize('source', ['target_finder', 'target_pose'])
    def test_each_source_builds_exactly_one_node(self, source):
        _, nodes = _nodes_for(source)
        assert len(nodes) == 1

    def test_the_two_sources_are_different_executables(self):
        launch = pytest.importorskip('launch_ros.actions')
        assert launch is not None
        _, finder = _nodes_for('target_finder')
        _, pose = _nodes_for('target_pose')
        assert finder[0] is not pose[0]

    def test_an_unknown_source_raises_rather_than_starting_nothing(self):
        # The failure this prevents: no perception node at all, a
        # mission that dies in SEARCH_TARGET 15 s later, and a
        # diagnosis that reads as a camera fault four layers away.
        with pytest.raises(RuntimeError, match='target_source'):
            _nodes_for('target_findr')

    def test_target_pose_sets_both_handover_parameters(self):
        # point_topic alone feeds the approach a fix the executive
        # never lets it use. Setting one without the other IS the
        # C2-M4.2 defect, so the launch file sets them together.
        module, nodes = _nodes_for('target_pose')
        literals = _literals(nodes[0]._Node__parameters)
        assert module.TARGET_TOPIC in literals
        assert module.STATUS_TOPIC in literals

    def test_the_default_is_still_the_measured_path(self):
        # target_finder is what the standing 19/20 was measured with.
        # A default that moved would re-open a measured result.
        module = _perception_launch()
        description = module.generate_launch_description()
        declared = [action for action in description.entities
                    if getattr(action, 'name', None) == 'target_source']
        assert len(declared) == 1
        assert declared[0].default_value[0].text == 'target_finder'
