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

"""The Yard generator: is it traceable, and does it build what it claims?"""
import copy
import math

from coco_sim.yard import (build_yard_mjcf, build_yard_sdf, features, height,
                           load_params, sample_yard)

import pytest


@pytest.fixture
def params():
    return load_params()


@pytest.fixture
def yard(params):
    return sample_yard(params, seed=0, randomise=False)


def test_every_rescaled_value_records_its_spec_and_derivation(params):
    """A number that differs from M7_DESIGN must say so, and say why.

    The whole defence against quietly redesigning the world to be easy is
    that each change carries the spec value beside it.
    """
    rescaled = [
        params['curb']['height'],
        params['washboard']['amplitude'],
        params['bridge']['width'],
        params['routes']['c']['rubble']['rms'],
    ]
    for entry in rescaled:
        assert 'spec' in entry and 'value' in entry
        assert entry['value'] != entry['spec']
        assert len(entry['derivation'].split()) > 20


def test_route_runs_are_derived_from_grade_not_typed(yard):
    """run = rise / tan(grade), so grade jitter cannot detach the ramp."""
    deck = yard.params['deck']['z']
    curb = yard.params['curb']['height']['value']
    for key, route in yard.routes.items():
        rise = deck - (curb if key == 'c' else 0.0)
        assert route.run == pytest.approx(
            rise / math.tan(math.radians(route.grade_deg)), rel=1e-9)
        assert route.z_top == pytest.approx(rise)


def test_ramps_meet_the_deck_at_their_top(yard):
    """The join is the thing grade jitter is most likely to break."""
    for key, route in yard.routes.items():
        top = height(route.x_top - 1e-4, route.y_centre, yard)
        assert top == pytest.approx(route.z_top, abs=2e-3)


def test_the_bridge_void_is_a_hole_not_a_low_step(yard):
    """0.650 m down to the apron. A 2-D lidar cannot see this."""
    p = yard.params
    x = sum(p['deck']['sections']['bridge']['x']) / 2.0
    on = height(x, 0.0, yard)
    beside = height(x, p['bridge']['width']['value'] / 2.0 + 0.4, yard)
    assert on == pytest.approx(p['deck']['z'])
    assert beside == 0.0


def test_the_curb_is_a_clean_step_of_the_stated_height(yard):
    """No overhang. An overhung curb is unclimbable at any speed."""
    p = yard.params
    route_c = yard.routes['c']
    below = height(-0.005, route_c.y_centre, yard)
    above = height(+0.005, route_c.y_centre, yard)
    assert above - below == pytest.approx(
        p['curb']['height']['value'], abs=3e-3)
    # nothing juts out over the ramp: just behind the lip the surface is
    # the RAMP, not the deck
    assert below < p['deck']['z'] - p['curb']['height']['value'] / 2


def test_washboard_is_an_integer_number_of_half_wavelengths(params):
    """Otherwise it ends on a step, and every measurement across it is
    contaminated by that step rather than by the washboard."""
    x0, x1 = params['deck']['sections']['washboard']['x']
    n_half = (x1 - x0) / (params['washboard']['wavelength'] / 2.0)
    assert n_half == pytest.approx(round(n_half), abs=1e-6)


def test_deck_sections_tile_without_gaps(params):
    order = ['staging', 'washboard', 'transition', 'bridge', 'approach',
             'bay']
    sec = params['deck']['sections']
    for a, b in zip(order, order[1:]):
        assert sec[a]['x'][1] == pytest.approx(sec[b]['x'][0])
    assert sec[order[0]]['x'][0] == pytest.approx(params['deck']['x'][0])
    assert sec[order[-1]]['x'][1] == pytest.approx(params['deck']['x'][1])


def test_geometry_traces_to_the_yaml_and_is_not_hard_coded(params):
    """Guard the guard: move a YAML value, the model must move with it.

    Same pattern as test_mjcf_traces_to_config. Without it, a constant
    could be duplicated into yard.py and the YAML would become
    documentation that no longer drives anything.
    """
    base = build_yard_sdf(sample_yard(params, seed=0, randomise=False))
    moved = copy.deepcopy(params)
    moved['deck']['z'] += 0.05
    assert build_yard_sdf(
        sample_yard(moved, seed=0, randomise=False)) != base

    moved = copy.deepcopy(params)
    moved['bridge']['width']['value'] += 0.1
    assert build_yard_sdf(
        sample_yard(moved, seed=0, randomise=False)) != base


def test_both_emitters_read_the_same_feature_list(yard):
    """Cross-engine parity is structural, not a coincidence to be checked.

    Every box in features() must appear in both the MJCF and the SDF. If
    an emitter grew its own geometry this fails.
    """
    boxes, fields = features(yard)
    mjcf, _ = build_yard_mjcf(yard)
    sdf = build_yard_sdf(yard)
    for b in boxes:
        assert f'name="{b.name}"' in mjcf
        assert f'name="{b.name}"' in sdf
    for f in fields:
        assert f'hfield="{f.name}"' in mjcf
        assert f'{f.name}.stl' in sdf


def test_the_world_is_grey_because_perception_classifies_by_hue(yard):
    """A textured terrain would put competing hue in the frames the 16/16
    perception result was measured on."""
    sdf = build_yard_sdf(yard)
    for block in sdf.split('<material>')[1:]:
        body = block.split('</material>')[0]
        for tag in ('ambient', 'diffuse'):
            rgba = body.split(f'<{tag}>')[1].split(f'</{tag}>')[0].split()
            r, g, b = (float(v) for v in rgba[:3])
            assert r == g == b, f'non-grey {tag}: {rgba}'


def test_grade_jitter_moves_the_foot_and_never_the_deck(params):
    """The deck is the origin plane; only the ramp feet slide."""
    feet, tops = set(), set()
    for seed in range(6):
        s = sample_yard(params, seed=seed, randomise=True)
        feet.add(round(s.routes['a'].x_foot, 6))
        tops.add(round(s.routes['a'].x_top, 6))
    assert len(feet) > 1
    assert len(tops) == 1


def test_route_b_grade_jitter_is_capped_below_the_tip_terminator(params):
    """29 deg would leave 5.4 deg to coco_rl's 0.6 rad terminator, so a
    successful climb could be scored as a fall."""
    cap = params['routes']['b']['grade_jitter_cap_deg']
    worst = params['routes']['b']['grade_deg'] + cap
    assert worst < math.degrees(0.6) - 6.0
    for seed in range(25):
        s = sample_yard(params, seed=seed, randomise=True)
        assert s.routes['b'].grade_deg <= worst + 1e-9


def test_the_rubble_grid_shape_is_independent_of_grade_jitter(params):
    """mjModel.hfield_data is allocated at compile time, so a jitter that
    changed the grid shape could not be applied to a compiled model."""
    shapes = {sample_yard(params, seed=s, randomise=True).rubble.shape
              for s in range(8)}
    assert len(shapes) == 1


def test_rubble_is_tapered_so_the_curb_height_is_not_a_random_variable(
        params):
    """Untapered noise under the lip would make a 28 mm curb anything from
    4 mm to 52 mm depending on the seed, and Section I's question is at
    what height a curb is mountable."""
    for seed in range(5):
        s = sample_yard(params, seed=seed, randomise=True)
        assert abs(s.rubble[:, -1]).max() < 1e-9
        assert abs(s.rubble[:, 0]).max() < 1e-9
        assert abs(s.rubble).max() > 1e-4
