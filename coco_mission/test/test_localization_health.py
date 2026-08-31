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
