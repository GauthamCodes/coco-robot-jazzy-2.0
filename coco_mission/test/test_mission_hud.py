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
Tests for the mission HUD's pure functions.

Everything interesting in mission_hud is a pure function taking already
-received values and returning text, so the whole display is testable
without a ROS graph, a simulator or a clock. The node class is a thin
shell over these.

The point of most of these tests is not that the formatting is pretty. It
is that the HUD cannot invent a number: a stale source must read STALE, a
source that never published must read '--', and the three fields with no
measured source must keep saying so.
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import mission_hud  # noqa: E402


class TestParseKv:
    """The shared key=value status format, parsed defensively."""

    def test_parses_a_real_arbiter_line(self):
        fields = mission_hud.parse_kv(
            'mode=nav active=nav teleop=-- nav=0.05 rl=-- approach=--')
        assert fields['mode'] == 'nav'
        assert fields['active'] == 'nav'
        assert fields['nav'] == '0.05'
        assert fields['teleop'] == '--'

    def test_parses_a_real_perception_line(self):
        fields = mission_hud.parse_kv(
            'sel=blue found=1 u=320 v=240 area=1877 w=48 h=52 '
            'range=1.198 x=1.198 y=+0.004 z=0.061 lane=+0.00 '
            'seen=blue age=0.08')
        assert fields['sel'] == 'blue'
        assert fields['range'] == '1.198'
        assert fields['y'] == '+0.004'

    def test_empty_and_none_are_empty_dicts(self):
        assert mission_hud.parse_kv('') == {}
        assert mission_hud.parse_kv(None) == {}

    def test_bare_tokens_are_skipped_not_raised(self):
        # A future publisher adding a bare word to its status line must
        # not be able to take the display down.
        assert mission_hud.parse_kv('phase=lift lifted') == {'phase': 'lift'}

    def test_value_may_contain_equals(self):
        assert mission_hud.parse_kv('note=a=b') == {'note': 'a=b'}

    def test_leading_equals_is_not_a_field(self):
        assert mission_hud.parse_kv('=orphan phase=stow') == {'phase': 'stow'}


class TestFreshness:
    """A dead publisher must never render as a live value."""

    def test_never_published_reads_as_dashes(self):
        assert mission_hud.format_age(None) == mission_hud.NEVER
        assert mission_hud.is_live(None) is False

    def test_fresh_age_is_shown_in_seconds(self):
        assert mission_hud.format_age(0.14) == '0.1s'
        assert mission_hud.is_live(0.14) is True

    def test_at_the_limit_is_still_live(self):
        assert mission_hud.is_live(mission_hud.STALE_AFTER) is True

    def test_past_the_limit_is_stale_and_says_so(self):
        rendered = mission_hud.format_age(mission_hud.STALE_AFTER + 5.0)
        assert mission_hud.STALE in rendered
        assert mission_hud.is_live(mission_hud.STALE_AFTER + 5.0) is False

    def test_age_of_returns_none_for_a_source_never_seen(self):
        assert mission_hud.age_of(None, 100.0) is None
        assert mission_hud.age_of(97.5, 100.0) == 2.5


class TestPoseSigmas:
    """AMCL covariance, read off the diagonal, defensively."""

    @staticmethod
    def _cov(var_x=0.04, var_y=0.09, var_yaw=0.01):
        cov = [0.0] * 36
        cov[0], cov[7], cov[35] = var_x, var_y, var_yaw
        return cov

    def test_takes_the_diagonal_terms(self):
        sx, sy, syaw = mission_hud.pose_sigmas(self._cov())
        assert sx == pytest.approx(0.2)
        assert sy == pytest.approx(0.3)
        assert syaw == pytest.approx(0.1)

    def test_wrong_length_is_rejected_not_guessed(self):
        assert mission_hud.pose_sigmas([0.0] * 9) is None
        assert mission_hud.pose_sigmas(None) is None

    def test_negative_variance_is_rejected(self):
        # sqrt of a negative inside a timer callback would kill the node.
        assert mission_hud.pose_sigmas(self._cov(var_x=-1.0)) is None

    def test_nan_variance_is_rejected(self):
        assert mission_hud.pose_sigmas(self._cov(var_y=float('nan'))) is None


class TestLocalisation:
    """The signal is shown. The verdict is deliberately withheld."""

    SIGMAS = (0.2, 0.3, 0.1)

    def test_shows_the_measured_sigmas(self):
        out = mission_hud.format_localisation(self.SIGMAS, 0.1)
        assert '0.200' in out
        assert '0.300' in out
        assert f'{math.degrees(0.1):.1f}' in out

    def test_does_not_render_a_health_verdict(self):
        # M5 calibrates the threshold. Until then the HUD must not imply
        # one exists -- this is the whole reason the field is formatted
        # by a named function instead of an f-string at the call site.
        out = mission_hud.format_localisation(self.SIGMAS, 0.1)
        for verdict in ('GOOD', 'OK', 'DEGRADED', 'BAD', 'HEALTHY', 'LOST'):
            assert verdict not in out.upper().split()

    def test_an_old_pose_keeps_its_sigmas_and_states_its_age(self):
        # AMCL publishes on UPDATE, and nav2_params sets update_min_d to
        # 0.25 m, so a stationary robot emits nothing. Measured on the
        # first live run: 17 s with no update while perfectly localised
        # at the start pose. Blanking the pose there would render normal
        # standing still as a localisation failure.
        out = mission_hud.format_localisation(
            self.SIGMAS, mission_hud.STALE_AFTER + 10.0)
        assert '0.200' in out
        assert '0.300' in out
        assert '12s since update' in out
        assert mission_hud.STALE not in out

    def test_never_received_reads_as_dashes(self):
        assert mission_hud.format_localisation(None, None) == mission_hud.NEVER

    def test_malformed_covariance_is_named(self):
        out = mission_hud.format_localisation(None, 0.1)
        assert 'malformed' in out


class TestDistance:
    """Which sensor produced the range is part of the reading."""

    def test_approach_wins_while_the_servo_is_running(self):
        out = mission_hud.format_distance({'range': '0.152'},
                                          {'range': '1.198'})
        assert out.startswith('0.152')
        assert 'approach' in out

    def test_falls_back_to_vision_before_the_approach_starts(self):
        out = mission_hud.format_distance({}, {'range': '1.198'})
        assert out.startswith('1.198')
        assert 'vision' in out

    def test_a_dashed_range_is_not_a_reading(self):
        # Both publishers emit '--' for "no value", which must not be
        # rendered as though it were a distance.
        out = mission_hud.format_distance({'range': '--'},
                                          {'range': '1.198'})
        assert out.startswith('1.198')

    def test_no_source_at_all(self):
        assert mission_hud.format_distance({}, {}) == mission_hud.NEVER


class TestGoal:
    def test_renders_a_goal(self):
        out = mission_hud.format_goal((2.5, -0.5))
        assert '+2.50' in out
        assert '-0.50' in out

    def test_no_plan_yet(self):
        assert mission_hud.format_goal(None) == mission_hud.NEVER


class TestRender:
    """The block itself."""

    STATE = {
        'mission': 'fetch',
        'mission_state': '2. RL climb',
        'controller': 'rl   (mode rl)',
        'localisation': 'sigma x 0.200 m  y 0.300 m  yaw 5.7 deg',
        'target': 'blue',
        'distance': '1.198 m (vision)',
        'goal': 'x +2.50  y -0.50  (map)',
        'pitch': '0.284 rad',
    }

    def test_every_label_appears(self):
        out = mission_hud.render(self.STATE)
        for label in ('MISSION', 'STATE', 'ACTIVE CONTROLLER', 'LOCALIZATION',
                      'TARGET', 'DISTANCE TO TARGET', 'CURRENT GOAL',
                      'ROBOT PITCH', 'TERRAIN GRADE', 'EST. FRICTION',
                      'RECOVERY'):
            assert label in out

    def test_unmeasured_fields_say_so_with_their_milestone(self):
        out = mission_hud.render(self.STATE)
        assert 'not yet measured (M2)' in out     # grade and friction
        assert 'not implemented (M5)' in out      # recovery

    def test_pitch_is_not_labelled_terrain_grade(self):
        # The ramp driver measures the ROBOT's attitude. Calling that the
        # slope of the surface is the exact claim M2 exists to earn.
        out = mission_hud.render(self.STATE)
        pitch_line = next(ln for ln in out.splitlines()
                          if ln.startswith('ROBOT PITCH'))
        grade_line = next(ln for ln in out.splitlines()
                          if ln.startswith('TERRAIN GRADE'))
        assert '0.284 rad' in pitch_line
        assert '0.284' not in grade_line

    def test_values_are_carried_through_verbatim(self):
        out = mission_hud.render(self.STATE)
        for value in self.STATE.values():
            assert value in out
