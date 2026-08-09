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
The baselines have to be honest before their numbers mean anything.

The failure this guards against is not a crash. It is a baseline that
quietly differs from the controller it claims to be, or that reads state
it is not entitled to — either of which turns the M8 comparison into a
comparison with something nobody built.
"""

import math

from coco_rl import baselines
from coco_rl.baselines import B0, B1, B2, DEFAULT_SCHEDULE, reference_y
from coco_rl.lateral import HEADING_GAIN, LATERAL_CLAMP, LATERAL_GAIN

from coco_sim.yard import load_params, sample_yard

import pytest


@pytest.fixture(scope='module')
def params():
    return load_params()


def test_b1_uses_the_shipped_function_not_a_copy():
    """If B1 reimplemented lateral_hold it would drift from the shipped
    controller and stop being the baseline it claims to be."""
    import inspect
    src = inspect.getsource(baselines)
    assert 'from coco_rl.lateral import' in src
    assert 'def lateral_hold' not in src, 'B1 must import it, not define it'


def test_b1_uses_the_shipped_gains_unchanged(params):
    """M7_DESIGN 3.1 pins B1 to LATERAL_GAIN 3.0 / HEADING_GAIN 2.5 /
    LATERAL_CLAMP 0.8. A retuned B1 is B2 wearing B1's name."""
    assert (LATERAL_GAIN, HEADING_GAIN, LATERAL_CLAMP) == (3.0, 2.5, 0.8)
    b = B1(params=params, throttle=0.5)
    s = sample_yard(params, seed=0, randomise=False)
    b.reset(s, 'a')
    obs = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
    from coco_rl.lateral import lateral_hold
    y = s.routes['a'].y_centre + 0.20
    expected = lateral_hold([0.5, 0.0], y_err=0.20, yaw=0.0)
    assert b(obs, -1.0, y) == pytest.approx(expected)


def test_b0_ignores_every_observation(params):
    """B0 is the no-feedback control. If it reacts to anything it is not
    measuring what an uncontrolled traverse looks like."""
    b = B0(throttle=0.42)
    b.reset(sample_yard(params, seed=0, randomise=False), 'a')
    outs = {tuple(b([0, y, math.sin(t), math.cos(t), 0, 0, 0, 0], x, y))
            for x in (-2.0, 0.5, 2.0) for y in (-1.0, 0.0, 1.0)
            for t in (-0.3, 0.0, 0.3)}
    assert outs == {(0.42, 0.0)}


def test_b2_is_the_only_baseline_with_privileged_access():
    assert B0.privileged is False
    assert B1.privileged is False
    assert B2.privileged is True


def test_b2_actually_uses_the_privileged_friction(params):
    """A schedule that ignores its privileged input is B1 with extra
    steps, and would make the whole M8 comparison flattering."""
    cfg = dict(DEFAULT_SCHEDULE['b'])
    cfg.update(throttle_lo=0.30, throttle_hi=0.90)
    schedule = {'b': cfg}
    seen = set()
    for seed in range(12):
        s = sample_yard(params, seed=seed, randomise=True)
        b = B2(schedule=schedule, params=params)
        b.reset(s, 'b')
        seen.add(round(b.gains['throttle'], 6))
    assert len(seen) > 1, 'B2 ignored the episode friction'


def test_b2_reacts_to_the_privileged_grade(params):
    """grade_k is the other privileged channel; it must reach the output."""
    cfg = dict(DEFAULT_SCHEDULE['a'])
    cfg.update(grade_k=0.05)
    seen = set()
    for seed in range(12):
        s = sample_yard(params, seed=seed, randomise=True)
        b = B2(schedule={'a': cfg}, params=params)
        b.reset(s, 'a')
        seen.add(round(b.gains['throttle'], 6))
    assert len(seen) > 1


# ── the reference path ──────────────────────────────────────────────────
def test_the_reference_path_converges_before_the_bridge(params):
    """The bridge is 0.65 m wide and there is no room to correct on it, so
    the lane must already be the bridge centreline when it starts."""
    bx0 = params['deck']['sections']['bridge']['x'][0]
    for route_y in (1.95, 0.0, -1.70):
        assert reference_y(bx0, route_y, params) == pytest.approx(
            params['bridge']['y_centre'])
        assert reference_y(bx0 + 0.5, route_y, params) == pytest.approx(
            params['bridge']['y_centre'])


def test_the_reference_path_is_the_route_centreline_on_the_ramp(params):
    for route_y in (1.95, -1.70):
        assert reference_y(-2.0, route_y, params) == route_y
        assert reference_y(-0.01, route_y, params) == route_y


def test_the_reference_path_is_continuous(params):
    """A step in the reference is a step in the error, which a PD answers
    with a spike. Any discontinuity here would show up as a controller
    result and be attributed to the controller."""
    xs = [i * 0.01 - 4.0 for i in range(800)]
    for route_y in (1.95, 0.0, -1.70):
        ys = [reference_y(x, route_y, params) for x in xs]
        jumps = [abs(b - a) for a, b in zip(ys, ys[1:])]
        assert max(jumps) < 0.05, f'reference jumps by {max(jumps):.3f} m'


def test_all_baselines_return_actions_inside_the_action_space(params):
    """The action space is on the do-not-touch list; a baseline that
    saturates outside it is not driving the same robot."""
    s = sample_yard(params, seed=3, randomise=True)
    for b in (B0(throttle=1.0), B1(params=params, throttle=1.0),
              B2(schedule=DEFAULT_SCHEDULE, params=params)):
        b.reset(s, 'c')
        for y in (-2.0, 0.0, 2.0):
            for yaw in (-1.2, 0.0, 1.2):
                obs = [0, 0, math.sin(yaw), math.cos(yaw), 0, 0, 0, 0]
                act = b(obs, 1.0, y)
                assert -1.0 <= act[0] <= 1.0
                assert -1.0 <= act[1] <= 1.0


def test_the_tuned_schedule_covers_every_route(params):
    """A missing route would raise inside reset() mid-evaluation, after
    the run had already started."""
    from coco_rl.baselines import TUNED_SCHEDULE
    assert set(TUNED_SCHEDULE) == {'a', 'b', 'c'}
    required = {'throttle_lo', 'throttle_hi', 'deck_throttle', 'lateral_lo',
                'lateral_hi', 'heading', 'clamp', 'grade_k'}
    for route, cfg in TUNED_SCHEDULE.items():
        assert required <= set(cfg), f'route {route} missing {required - set(cfg)}'
        b = B2(schedule=TUNED_SCHEDULE, params=params)
        b.reset(sample_yard(params, seed=1, randomise=True), route)
        assert 0.0 < b.gains['throttle'] <= 1.0


def test_the_tuned_throttle_range_reaches_full(params):
    """The first grid capped throttle at 0.65 and B0 at 1.0 beat B2 on
    Route B. A schedule whose search never tried what the route needs is
    not a strong baseline, and M7_DESIGN 3.1 says a weak B2 makes the M8
    comparison worthless."""
    from coco_rl.baselines import TUNED_SCHEDULE
    assert max(max(c['throttle_lo'], c['throttle_hi'])
               for c in TUNED_SCHEDULE.values()) >= 0.65
