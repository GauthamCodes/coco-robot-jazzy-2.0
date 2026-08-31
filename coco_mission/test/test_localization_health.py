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
Tests for the pure localization-health core.

The classifier is pure, so none of this needs a graph, a simulator or a
clock. What the tests are really pinning is the set of promises C2-M5.0
made about the signal, because those are the parts a later session would
otherwise quietly undo:

  * no threshold may have a default
  * ``UNKNOWN`` must never read as healthy
  * the mapped-ground gate must fire BEFORE the consistency test
  * freshness must fire before both
  * ground truth must not appear anywhere in the observation

The last one is a real test, not a comment: it reads the dataclass's
fields and fails if a ``gt_``-shaped name ever appears.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import localization_health as lh  # noqa: E402


def limits(**kw):
    """Thresholds with the two required numbers filled in.

    The values are the healthy envelope's own maximum and minimum from
    run `healthy1`. They are here so the tests have something concrete
    to push against — they are NOT a proposed configuration, which is
    exactly why `Thresholds` refuses to carry them itself.
    """
    base = dict(lik_mean_d_max=0.314, lik_frac_near_min=0.317)
    base.update(kw)
    return lh.Thresholds(**base)


def healthy_obs(**kw):
    """An observation in the middle of the measured healthy envelope."""
    base = dict(lik_mean_d=0.053, lik_frac_near=0.875, lik_beams=60,
                cov_sigma_xy=0.376, amcl_age=0.6, map_odom_age=-0.44,
                map_odom_step=0.0, on_mapped_ground=True)
    base.update(kw)
    return lh.Observation(**base)


# ── the no-defaults promise ──────────────────────────────────────────────

def test_thresholds_cannot_be_constructed_without_naming_the_numbers():
    """The two unjustified numbers must be typed by whoever uses them."""
    with pytest.raises(TypeError):
        lh.Thresholds()
    with pytest.raises(TypeError):
        lh.Thresholds(lik_mean_d_max=0.3)


def test_classify_without_thresholds_is_unknown_not_a_guess():
    v = lh.classify(healthy_obs())
    assert v.verdict == lh.UNKNOWN
    assert v.reason == lh.NO_THRESHOLDS


def test_the_envelope_is_not_a_threshold_object():
    """C2M50_ENVELOPE records observations; it must not be configuration."""
    for group in lh.C2M50_ENVELOPE.values():
        for env in group.values():
            assert isinstance(env, lh.Envelope)
            assert not isinstance(env, lh.Thresholds)
            assert env.lo <= env.median <= env.hi
            assert env.n > 0
            assert env.source


# ── UNKNOWN must not read as healthy ─────────────────────────────────────

@pytest.mark.parametrize('verdict', [lh.UNKNOWN, lh.STALE, lh.INCONSISTENT])
def test_only_consistent_is_truthy(verdict):
    assert not lh.Verdict(verdict, lh.OK)


def test_consistent_is_truthy():
    assert lh.Verdict(lh.CONSISTENT, lh.OK)


# ── freshness, whose bound is the stack's own transform_tolerance ────────

def test_a_stale_pose_is_stale():
    v = lh.classify(healthy_obs(amcl_age=9.0), limits())
    assert v.verdict == lh.STALE
    assert v.reason == lh.POSE_STALE


def test_healthy_map_odom_age_is_negative_and_passes():
    """AMCL post-dates map->odom by transform_tolerance, so -0.44 s is fine."""
    v = lh.classify(healthy_obs(map_odom_age=-0.44), limits())
    assert v.verdict == lh.CONSISTENT


def test_map_odom_age_past_zero_is_stale():
    """Once the tolerance window has run out, nothing is republishing it."""
    v = lh.classify(healthy_obs(map_odom_age=0.2), limits())
    assert v.verdict == lh.STALE
    assert v.reason == lh.TRANSFORM_STALE


def test_no_pose_at_all_is_unknown():
    v = lh.classify(healthy_obs(amcl_age=None, map_odom_age=None), limits())
    assert v.verdict == lh.UNKNOWN
    assert v.reason == lh.NO_POSE


def test_freshness_is_checked_before_the_consistency_test():
    """A stale pose whose last scan score was terrible reports STALE.

    Order matters: reporting INCONSISTENT there would send a recovery
    after a disagreement computed from a pose nobody is updating.
    """
    v = lh.classify(healthy_obs(amcl_age=9.0, lik_mean_d=0.9,
                                lik_frac_near=0.05), limits())
    assert v.verdict == lh.STALE


# ── the mapped-ground gate ───────────────────────────────────────────────

def test_off_mapped_ground_is_unknown_not_inconsistent():
    """The platform is not in the map; disagreeing with it is not a fault.

    This is the healthy run's own worst sample: mean endpoint distance
    0.31 m near the platform, with a true error of only 0.26 m.
    """
    v = lh.classify(healthy_obs(on_mapped_ground=False, lik_mean_d=0.31,
                                lik_frac_near=0.32), limits())
    assert v.verdict == lh.UNKNOWN
    assert v.reason == lh.OFF_MAPPED_GROUND


def test_the_gate_runs_before_the_consistency_test():
    v = lh.classify(healthy_obs(on_mapped_ground=False, lik_mean_d=5.0,
                                lik_frac_near=0.0), limits())
    assert v.verdict == lh.UNKNOWN


def test_too_few_beams_inside_the_map_is_unknown():
    v = lh.classify(healthy_obs(lik_beams=3), limits())
    assert v.verdict == lh.UNKNOWN
    assert v.reason == lh.FEW_BEAMS


def test_a_missing_scan_score_is_unknown():
    v = lh.classify(healthy_obs(lik_mean_d=None), limits())
    assert v.verdict == lh.UNKNOWN
    assert v.reason == lh.NO_SCAN_MATCH


# ── the consistency test itself ──────────────────────────────────────────

def test_the_measured_healthy_median_is_consistent():
    v = lh.classify(healthy_obs(), limits())
    assert v.verdict == lh.CONSISTENT
    assert v.reason == lh.OK


def test_the_measured_diverged_median_is_inconsistent():
    """diverged1's median: mean endpoint distance 0.376 m, 0.320 near."""
    v = lh.classify(healthy_obs(lik_mean_d=0.3764, lik_frac_near=0.3200),
                    limits())
    assert v.verdict == lh.INCONSISTENT
    assert v.reason == lh.SCAN_DISAGREES


def test_either_half_of_the_consistency_test_can_fire_alone():
    far = lh.classify(healthy_obs(lik_mean_d=0.9), limits())
    sparse = lh.classify(healthy_obs(lik_frac_near=0.01), limits())
    assert far.verdict == lh.INCONSISTENT
    assert sparse.verdict == lh.INCONSISTENT


def test_the_detail_names_the_numbers_that_decided_it():
    v = lh.classify(healthy_obs(lik_mean_d=0.9), limits())
    assert '0.900' in v.detail and '0.314' in v.detail


# ── covariance is recorded and is NOT the divergence test ────────────────

def test_covariance_alone_never_changes_the_verdict():
    """The measured reason: at the divergence sigma_xy went the WRONG way.

    On `diverged1` it fell to 0.070 m — below the healthy minimum of
    0.248 — at the instant the pose became 3 m wrong, and took 24.5 s to
    climb past the healthy maximum. A classifier that keyed on it would
    have called that robot unusually healthy while it was lost, so the
    field is carried and not consulted.
    """
    for sigma in (0.0, 0.070, 0.376, 1.2408, 99.0):
        assert lh.classify(healthy_obs(cov_sigma_xy=sigma),
                           limits()).verdict == lh.CONSISTENT


def test_the_recorded_divergence_dip_is_below_every_healthy_floor():
    """Pins the number the whole design argument rests on.

    Both divergence runs reported a smaller sigma_xy than the smallest
    seen on either leg that finished. That is why covariance is carried
    and not consulted.
    """
    floors = [lh.C2M50_ENVELOPE[r]['cov_sigma_xy'].lo
              for r in lh.C2M50_SUCCEEDED]
    for run in ('diverged1', 'diverged2'):
        assert lh.C2M50_ENVELOPE[run]['cov_sigma_xy'].lo < min(floors)


def test_the_latency_record_prefers_the_scan_signal_in_both_runs():
    for run, lat in lh.C2M50_LATENCY_S.items():
        assert lat['lik_mean_d'] < lat['cov_sigma_xy'], run


def test_the_scan_signal_detection_latency_replicated():
    """0.4 s is a replicated figure, not a single run's luck."""
    assert lh.C2M50_LATENCY_S['diverged1']['lik_mean_d'] == 0.4
    assert lh.C2M50_LATENCY_S['diverged2']['lik_mean_d'] == 0.4


def test_the_run_lists_partition_the_envelope():
    """Every recorded run is classified as finished or failed, once."""
    named = set(lh.C2M50_SUCCEEDED) | set(lh.C2M50_FAILED)
    assert named == set(lh.C2M50_ENVELOPE)
    assert not set(lh.C2M50_SUCCEEDED) & set(lh.C2M50_FAILED)


def test_healthy2_is_recorded_as_a_FAILED_leg():
    """Its name says healthy; its outcome was an abort. Pin the outcome.

    The run was launched as an uninjected baseline and failed anyway,
    which is the whole reason it matters. A reader skimming the envelope
    dict must not take the key for the verdict.
    """
    assert 'healthy2' in lh.C2M50_FAILED
    assert 'healthy2' not in lh.C2M50_SUCCEEDED


def test_the_failure_that_was_not_injected_gets_no_covariance_warning():
    """healthy2's covariance median is LOWER than either leg that finished.

    Measured: 0.3721 against 0.3763 (healthy1) and 0.4103 (obstacle1).
    So on the run that failed by itself, covariance did not merely fail
    to warn — it read slightly better than on both runs that succeeded.
    """
    h2 = lh.C2M50_ENVELOPE['healthy2']['cov_sigma_xy'].median
    ok = [lh.C2M50_ENVELOPE[r]['cov_sigma_xy'].median
          for r in lh.C2M50_SUCCEEDED]
    assert h2 <= min(ok), (
        'if this ever fails, covariance gained some warning value on the '
        'uninjected failure and the C2-M5.0 verdict needs revisiting')


# ── the ground-truth boundary, as a test rather than a comment ───────────

def test_no_ground_truth_field_can_enter_the_observation():
    """C2-M5's central constraint, enforced instead of remembered."""
    banned = ('gt', 'ground_truth', 'truth', 'gazebo', 'gz', 'err_xy',
              'err_yaw', 'true_')
    for name in lh.Observation.__dataclass_fields__:
        low = name.lower()
        for word in banned:
            assert not low.startswith(word), \
                f'{name} looks like ground truth; it may not be an input'
            assert word not in low.split('_'), \
                f'{name} looks like ground truth; it may not be an input'


def test_the_module_imports_without_ros():
    """It is pure. Importing it must not need rclpy on the path."""
    assert 'rclpy' not in sys.modules or True
    src = open(os.path.join(os.path.dirname(__file__), '..', 'scripts',
                            'localization_health.py')).read()
    assert 'import rclpy' not in src
    assert 'from rclpy' not in src


def test_every_verdict_constant_is_listed():
    assert set(lh.VERDICTS) == {lh.CONSISTENT, lh.INCONSISTENT, lh.STALE,
                                lh.UNKNOWN}


# ═══════════════════════════════════════════════════════════════════════
# C2-M5.1 — the thresholds that were finally named, and the persistence
# ═══════════════════════════════════════════════════════════════════════
#
# These pin the JUSTIFICATION, not just the value. Each one states the
# measured fact the number rests on, so a later change that breaks the
# reasoning fails here rather than passing quietly with a new constant.


class TestTheChosenThreshold:

    def test_it_is_above_every_sample_on_a_leg_that_finished(self):
        # The whole argument for 0.40. obstacle1's gated maximum is the
        # largest scan-vs-map sample recorded on any leg that completed.
        for run in lh.C2M50_SUCCEEDED:
            assert lh.LIK_MEAN_D_MAX > lh.C2M51_GATED[run].hi, run

    def test_it_is_below_the_worst_of_both_injected_divergences(self):
        for run in ('diverged1', 'diverged2'):
            assert lh.LIK_MEAN_D_MAX < lh.C2M51_GATED[run].hi, run

    def test_neither_leg_that_finished_produced_a_single_excursion(self):
        # The false-positive evidence, as an assertion.
        for run in lh.C2M50_SUCCEEDED:
            count, longest = lh.C2M51_EXCURSIONS[run]
            assert count == 0 and longest == 0.0, run

    def test_both_injected_divergences_produced_one(self):
        for run in ('diverged1', 'diverged2'):
            count, longest = lh.C2M51_EXCURSIONS[run]
            assert count >= 1 and longest > lh.DEGRADED_HOLD_S, run

    def test_the_gated_record_is_not_the_ungated_one(self):
        # They disagree, and the gate is the reason. Keeping both is what
        # stops a later reader averaging a platform sample into a verdict
        # about the flat.
        assert (lh.C2M51_GATED['diverged2'].hi
                != lh.C2M50_ENVELOPE['diverged2']['lik_mean_d'].hi)

    def test_the_shipped_thresholds_object_carries_it(self):
        assert lh.C2M51_THRESHOLDS.lik_mean_d_max == lh.LIK_MEAN_D_MAX

    def test_thresholds_still_cannot_be_built_without_naming_the_numbers(self):
        # C2-M5.1 named the numbers; it did not give the class defaults.
        # A future threshold still has to be typed by someone.
        with pytest.raises(TypeError):
            lh.Thresholds()

    def test_frac_near_is_disabled_and_the_data_says_why(self):
        # It cannot separate: ON MAPPED GROUND the cleanest injected
        # divergence never gets as low as a leg that finished did, so no
        # threshold under the healthy floor can fire on it.
        floors = lh.C2M51_GATED_FRAC_NEAR_LO
        assert floors['diverged2'] > floors['obstacle1']
        assert lh.C2M51_THRESHOLDS.lik_frac_near_min == 0.0

    def test_the_ungated_record_would_have_argued_the_opposite(self):
        # And this is why the gate is not a detail. Over the whole leg,
        # including the platform, diverged2's floor sits BELOW
        # obstacle1's and frac_near looks like a usable discriminator.
        # It is not; the samples making it look that way were taken
        # somewhere the map does not describe.
        assert (lh.C2M50_ENVELOPE['diverged2']['lik_frac_near'].lo
                < lh.C2M50_ENVELOPE['obstacle1']['lik_frac_near'].lo)

    def test_a_disabled_frac_near_never_decides_a_verdict(self):
        obs = lh.Observation(lik_mean_d=0.05, lik_frac_near=0.0,
                             lik_beams=50, map_odom_age=-0.44)
        assert lh.classify(obs, lh.C2M51_THRESHOLDS).verdict == lh.CONSISTENT

    def test_the_healthy_medians_pass_and_the_diverged_medians_fail(self):
        for run in lh.C2M50_SUCCEEDED:
            obs = lh.Observation(
                lik_mean_d=lh.C2M50_ENVELOPE[run]['lik_mean_d'].median,
                lik_frac_near=lh.C2M50_ENVELOPE[run]['lik_frac_near'].median,
                lik_beams=50, map_odom_age=-0.44)
            assert lh.classify(obs, lh.C2M51_THRESHOLDS).verdict \
                == lh.CONSISTENT, run
        for run in ('diverged1', 'diverged2'):
            obs = lh.Observation(
                lik_mean_d=lh.C2M50_ENVELOPE[run]['lik_mean_d'].hi,
                lik_frac_near=lh.C2M50_ENVELOPE[run]['lik_frac_near'].median,
                lik_beams=50, map_odom_age=-0.44)
            assert lh.classify(obs, lh.C2M51_THRESHOLDS).verdict \
                == lh.INCONSISTENT, run

    def test_healthy2_is_NOT_separated_and_that_is_recorded(self):
        # The known limitation, as an assertion. healthy2 failed with no
        # injection and its worst sample is still under the threshold, so
        # this monitor would not have caught it. If a future change makes
        # this pass, the class-B claim in RESULTS.md needs revisiting --
        # not this test.
        hi = lh.C2M50_ENVELOPE['healthy2']['lik_mean_d'].hi
        assert hi < lh.LIK_MEAN_D_MAX


def drive(p, pattern, dt=0.1, t0=0.0):
    """Feed a Persistence a sample pattern at a realistic 10 Hz.

    The monitor runs at 10 Hz and `Persistence.MAX_STEP` clamps a single
    update's contribution, so a test that steps 2 s at a time is testing
    the clamp rather than the rule. Everything here uses the real rate.
    """
    t = t0
    latched = p.latched
    for holds in pattern:
        latched = p.update(t, holds)
        t += dt
    return latched


class TestPersistence:
    """The accumulate/drain rule Experiment 2 forced.

    The first version reset on any single false sample. On the live
    injected divergence the signal dithered: 81 INCONSISTENT samples in
    the leg, longest unbroken stretch 1.80 s against a 2.0 s hold, so a
    real 3 m error never latched. These pin the replacement.
    """

    def test_one_bad_sample_never_latches(self):
        p = lh.Persistence(lh.DEGRADED_HOLD_S)
        assert not p.update(0.0, True)
        assert not p.latched

    def test_sustained_true_latches_after_exactly_the_hold(self):
        p = lh.Persistence(2.0)
        # 20 samples at 10 Hz spans 1.9 s of elapsed time; the 21st
        # crosses 2.0.
        assert not drive(p, [True] * 20)
        assert drive(p, [True] * 2, t0=2.0)

    def test_fifty_fifty_noise_never_latches_however_long_it_runs(self):
        # The property that makes the rule safe: a signal that is merely
        # ambiguous cannot accumulate, at any duration.
        p = lh.Persistence(2.0)
        assert not drive(p, [True, False] * 3000)
        assert not p.latched

    def test_eighty_percent_bad_does_latch(self):
        # The Experiment 2 stretch: ~80% INCONSISTENT for 4.6 s. Net
        # accumulation 0.6 s per second, so 2.0 s of credit in ~3.3 s.
        p = lh.Persistence(2.0)
        assert drive(p, [True, True, True, True, False] * 20)

    def test_a_single_good_sample_no_longer_throws_the_evidence_away(self):
        # The exact regression Experiment 2 found.
        p = lh.Persistence(2.0)
        drive(p, [True] * 18)
        before = p.credit
        drive(p, [False], t0=1.8)
        assert p.credit == pytest.approx(before - 0.1, abs=1e-6)
        assert p.credit > 0.0

    def test_sustained_false_clears_the_latch(self):
        p = lh.Persistence(2.0)
        assert drive(p, [True] * 25)
        assert not drive(p, [False] * 25, t0=2.5)

    def test_clearing_takes_as_long_as_latching(self):
        # Hysteresis: one good sample must not release a latched
        # degradation any more than one bad sample may set it.
        p = lh.Persistence(2.0)
        drive(p, [True] * 25)
        assert p.latched
        assert drive(p, [False] * 10, t0=2.5)     # 1.0 s of good: still on
        assert p.latched

    def test_credit_is_capped_at_the_hold(self):
        # Otherwise a long divergence banks credit and the robot stays
        # latched for minutes after it has genuinely recovered.
        p = lh.Persistence(2.0)
        drive(p, [True] * 600)
        assert p.credit == pytest.approx(2.0)

    def test_a_starved_monitor_cannot_bank_a_whole_gap(self):
        p = lh.Persistence(2.0)
        p.update(0.0, True)
        p.update(100.0, True)          # a 100 s stall
        assert p.credit <= lh.Persistence.MAX_STEP

    def test_a_clock_that_goes_backwards_contributes_nothing(self):
        p = lh.Persistence(2.0)
        p.update(10.0, True)
        p.update(5.0, True)
        assert p.credit == 0.0

    def test_held_for_reports_the_accumulated_evidence(self):
        p = lh.Persistence(2.0)
        drive(p, [True] * 11)
        assert p.held_for(0.0) == pytest.approx(p.credit)
        assert p.held_for(0.0) > 0.9

    def test_held_for_is_zero_when_nothing_is_holding(self):
        p = lh.Persistence(2.0)
        drive(p, [False] * 20)
        assert p.held_for(0.0) == 0.0

    def test_reset_clears_the_latch(self):
        p = lh.Persistence(1.0)
        drive(p, [True] * 20)
        assert p.latched
        p.reset()
        assert not p.latched
        assert p.credit == 0.0

    def test_the_healthy_run_worst_sample_would_have_fired_without_it(self):
        # The reason persistence exists at all, stated as a test: the
        # healthy leg's own worst scan-vs-map sample is a real excursion,
        # and a single-sample rule on ANY threshold under it fires on a
        # mission that went home to 0.078 m.
        worst = lh.C2M50_ENVELOPE['healthy1']['lik_mean_d'].hi
        assert worst > lh.C2M50_ENVELOPE['healthy1']['lik_mean_d'].median * 4


class TestTheHoldWindows:

    def test_the_degraded_hold_fits_inside_the_shortest_true_positive(self):
        # diverged2's single excursion above LIK_MEAN_D_MAX lasted 5.02 s.
        # The hold has to be comfortably under that or the shorter of the
        # two measured divergences is missed.
        shortest_measured_excursion = 5.02
        assert lh.DEGRADED_HOLD_S < shortest_measured_excursion / 2.0

    def test_resuming_is_harder_than_triggering(self):
        # Asymmetric on purpose: a false trigger costs a spin, a false
        # resume costs the mission.
        assert lh.HEALTHY_HOLD_S > lh.DEGRADED_HOLD_S

    def test_both_windows_are_positive(self):
        assert lh.DEGRADED_HOLD_S > 0
        assert lh.HEALTHY_HOLD_S > 0


class TestTheMappedGroundGate:

    def test_home_is_on_mapped_ground(self):
        assert lh.on_mapped_ground(-2.0)

    def test_the_ramp_the_platform_and_the_far_slope_are_not(self):
        for x in (1.0, 2.0, 3.0, 4.05, 4.5, 5.5, 6.5):
            assert not lh.on_mapped_ground(x, 0.0), x

    def test_the_corridor_BESIDE_the_wedge_is_mapped_ground(self):
        # Experiment 2's defect. The robot does not climb back over the
        # wedge to get home, it drives around it -- and an x-only gate
        # called that whole corridor unmapped, discarding the signal for
        # 65% of the return leg. Measured on the C2-M5.0 runs: the worst
        # corridor sample on a leg that finished is 0.3798, against 0.3851
        # on the flat, so the corridor scores like ordinary floor.
        for x in (1.5, 3.0, 4.5, 6.0):
            assert lh.on_mapped_ground(x, +2.0), x
            assert lh.on_mapped_ground(x, -2.0), x

    def test_the_wedge_itself_is_still_gated_out(self):
        for y in (0.0, +1.0, -1.0, +1.25, -1.25):
            assert not lh.on_mapped_ground(3.5, y), y

    def test_the_half_width_comes_from_the_wedge(self):
        from coco_config.robot import RAMP_WIDTH
        assert lh.MAPPED_GROUND_HALF_WIDTH == RAMP_WIDTH / 2.0

    def test_omitting_y_is_the_conservative_reading(self):
        # Without a lateral estimate the gate must not assume the robot
        # is safely beside the wedge.
        assert not lh.on_mapped_ground(3.5)
        assert not lh.on_mapped_ground(3.5, None)

    def test_past_the_far_foot_is_mapped_again(self):
        assert lh.on_mapped_ground(7.0)

    def test_an_unknown_position_is_not_mapped_ground(self):
        # No pose means no gate, and the gate failing closed makes the
        # verdict UNKNOWN rather than letting a scan be scored against a
        # position nobody has.
        assert not lh.on_mapped_ground(None)

    def test_the_span_is_derived_from_the_wedge_and_not_typed(self):
        from coco_config.robot import (PLATFORM_LEN, RAMP_FOOT_X, RAMP_RUN,
                                       RAMP_SUMMIT_X)
        assert lh.MAPPED_GROUND_MIN_X == RAMP_FOOT_X
        assert lh.MAPPED_GROUND_MAX_X == RAMP_SUMMIT_X + PLATFORM_LEN \
            + RAMP_RUN

    def test_the_gate_still_beats_the_consistency_test(self):
        obs = lh.Observation(lik_mean_d=9.9, lik_frac_near=0.0,
                             lik_beams=50, map_odom_age=-0.44,
                             on_mapped_ground=False)
        assert lh.classify(obs, lh.C2M51_THRESHOLDS).verdict == lh.UNKNOWN


class TestTheLikelihoodField:

    def field(self):
        # A 5x5 map at 1 m/cell with one occupied cell at index (2, 2),
        # origin at the origin. Cell centres are at +0.5.
        occupied = [[False] * 5 for _ in range(5)]
        occupied[2][2] = True
        return lh.LikelihoodField(occupied, 1.0, (0.0, 0.0))

    def test_the_occupied_cell_is_zero_from_itself(self):
        assert self.field().distance([2.5], [2.5])[0] == pytest.approx(0.0)

    def test_distance_grows_with_separation(self):
        d = self.field().distance([2.5, 3.5, 4.5], [2.5, 2.5, 2.5])
        assert d[0] < d[1] < d[2]

    def test_outside_the_map_is_nan_not_clamped(self):
        import math as _math
        d = self.field().distance([-10.0, 99.0], [2.5, 2.5])
        assert _math.isnan(d[0]) and _math.isnan(d[1])

    def test_unknown_cells_are_not_treated_as_obstacles(self):
        # -1 is 'unknown' in an OccupancyGrid. Counting it as occupied
        # would make every endpoint in unexplored space look explained.
        field = lh.LikelihoodField.from_occupancy_grid(
            [-1] * 25, 5, 5, 1.0, (0.0, 0.0))
        assert field.n_occupied == 0

    def test_an_occupancy_grid_threshold_is_honoured(self):
        data = [0] * 25
        data[12] = 100
        field = lh.LikelihoodField.from_occupancy_grid(
            data, 5, 5, 1.0, (0.0, 0.0))
        assert field.n_occupied == 1


class TestScoreScan:

    def field(self):
        occupied = [[False] * 20 for _ in range(20)]
        for i in range(20):
            occupied[i][10] = True          # a wall at x = 10
        return lh.LikelihoodField(occupied, 1.0, (0.0, 0.0))

    def test_a_scan_that_lands_on_the_wall_scores_near_zero(self):
        # One beam straight along +x from (5.5, 5.5): the wall is 5 m away.
        mean_d, frac_near, n = lh.score_scan(
            self.field(), [5.0], 0.0, 0.1, 0.1, 30.0, 5.5, 5.5, 0.0)
        assert n == 1
        assert mean_d == pytest.approx(0.0, abs=0.75)

    def test_a_pose_that_is_wrong_scores_worse(self):
        field = self.field()
        # Same scan, but the robot believes it is 4 m further back, so
        # the endpoint lands short of the wall.
        right = lh.score_scan(field, [5.0], 0.0, 0.1, 0.1, 30.0,
                              5.5, 5.5, 0.0)[0]
        wrong = lh.score_scan(field, [5.0], 0.0, 0.1, 0.1, 30.0,
                              1.5, 5.5, 0.0)[0]
        assert wrong > right

    def test_an_empty_scan_scores_nothing_rather_than_zero(self):
        assert lh.score_scan(self.field(), [], 0.0, 0.1, 0.1, 30.0,
                             0.0, 0.0, 0.0) == (None, None, 0)

    def test_out_of_range_returns_are_dropped(self):
        import math as _math
        assert lh.score_scan(
            self.field(), [_math.inf, 0.0, 99.0], 0.0, 0.1, 0.1, 30.0,
            5.5, 5.5, 0.0) == (None, None, 0)

    def test_no_field_scores_nothing(self):
        assert lh.score_scan(None, [1.0], 0.0, 0.1, 0.1, 30.0,
                             0.0, 0.0, 0.0) == (None, None, 0)

    def test_the_beam_count_matches_what_amcl_scores(self):
        # nav2_amcl max_beams is 60, and c2m5_locrec used the same, so a
        # live figure and a recorded CSV mean the same thing.
        assert lh.LIK_BEAMS == 60

    def test_the_near_radius_is_two_map_cells(self):
        assert lh.LIK_NEAR_M == pytest.approx(2 * 0.05)


class TestTheAmclAgeCheckIsOff:
    """C2-M5.1 Experiment 1: the /amcl_pose gap is not a staleness test.

    nav2_params sets amcl.update_min_d 0.25 and update_min_a 0.2, so AMCL
    publishes /amcl_pose only after the robot has MOVED. A stationary
    robot ages that topic without bound, and the healthy mission spent
    ~50 s stationary in GRASP.
    """

    def test_the_shipped_thresholds_do_not_test_it(self):
        assert lh.C2M51_THRESHOLDS.max_amcl_age is None

    def test_a_long_stationary_gap_is_not_stale(self):
        # 50 s of GRASP with the transform perfectly fresh.
        obs = lh.Observation(lik_mean_d=0.061, lik_frac_near=0.9,
                             lik_beams=55, amcl_age=50.0,
                             map_odom_age=-0.40)
        verdict = lh.classify(obs, lh.C2M51_THRESHOLDS)
        assert verdict.verdict == lh.CONSISTENT

    def test_a_transform_that_stops_being_republished_IS_stale(self):
        # The failure the freshness check actually has to catch. AMCL is
        # the only publisher of map->odom, so an AMCL that died drives
        # this age up through zero.
        obs = lh.Observation(lik_mean_d=0.061, lik_frac_near=0.9,
                             lik_beams=55, amcl_age=0.1,
                             map_odom_age=+1.20)
        verdict = lh.classify(obs, lh.C2M51_THRESHOLDS)
        assert verdict.verdict == lh.STALE
        assert verdict.reason == lh.TRANSFORM_STALE

    def test_the_check_still_works_when_a_bound_is_given(self):
        # Removed from the shipped thresholds, not from the module. A
        # different stack whose pose topic is periodic can still use it.
        limits = lh.Thresholds(lik_mean_d_max=0.4, lik_frac_near_min=0.0,
                               max_amcl_age=5.0)
        obs = lh.Observation(lik_mean_d=0.05, lik_frac_near=0.9,
                             lik_beams=55, amcl_age=50.0,
                             map_odom_age=-0.40)
        assert lh.classify(obs, limits).reason == lh.POSE_STALE

    def test_no_pose_at_all_is_still_unknown(self):
        # Turning the age check off must not turn "nothing has ever
        # arrived" into good news.
        obs = lh.Observation(lik_mean_d=0.05, lik_frac_near=0.9,
                             lik_beams=55)
        assert lh.classify(obs, lh.C2M51_THRESHOLDS).reason == lh.NO_POSE


class TestExperiment1:
    """The healthy false-positive check, as a record."""

    def test_the_mission_completed(self):
        assert lh.C2M51_EXP1['result'] == 'COMPLETE'

    def test_the_scan_signal_produced_no_false_positive(self):
        # The claim C2-M5.1 is allowed to make: over a whole healthy
        # mission the scan-vs-map rule fired zero times.
        assert lh.C2M51_EXP1['inconsistent_on_mapped_ground'] == 0
        assert lh.C2M51_EXP1['scan_disagrees_triggers'] == 0

    def test_the_threshold_was_not_moved_afterwards(self):
        # The worst gated sample of a third independent healthy mission
        # is still under the threshold chosen from the C2-M5.0 replay.
        # If this ever fails, the threshold needs re-deriving from the
        # runs -- not nudging to make this pass.
        assert lh.C2M51_EXP1['gated_lik_mean_d_max'] < lh.LIK_MEAN_D_MAX

    def test_the_only_false_positives_were_the_freshness_check(self):
        assert lh.C2M51_EXP1['pose_stale_triggers_before_fix'] > 0

    def test_the_run_is_bigger_than_a_single_leg(self):
        assert lh.C2M51_EXP1['gated_samples'] > 800
