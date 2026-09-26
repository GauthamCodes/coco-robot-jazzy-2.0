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
Stage C: every target stands in a named region, and the episode says which.

What these pin, in the task's own terms:

- FIXED is the frozen P0.2 layout AND the frozen colour->lane table;
- COLOUR permutes colours among the regions, deterministically;
- POSITION moves targets only inside their region's placement area,
  which is one-sided along the lane and bounded across it by the largest
  lateral offset the approach has been measured to absorb;
- a target outside its region is refused, and so is a manifest that
  names no region;
- the mission is handed region NAMES (``compat_mission_inputs``), and
  ``task_view`` did not grow.
"""

from dataclasses import replace
import json
import math

from coco_config.robot import (FIXED_REGION_MAP, lane_for_colour,
                               parse_region_map, region_by_id, REGION_IDS,
                               resolve_lane, TARGET_COLOURS, TARGET_REGIONS,
                               TARGET_ROW_X, TARGETS)
from coco_sim.episode import (compat_mission_inputs, episode_from_json,
                              generate_episode, InvalidEpisode, LEVELS,
                              PLACEMENT_FILL, region_area,
                              REGION_LATERAL_LIMIT, resolve_episode,
                              target_x_bounds, validate_episode)
import pytest

SEEDS = range(2000)


# ── A. FIXED ─────────────────────────────────────────────────────────────
@pytest.mark.parametrize('seed', [0, 1, 1827, 99999])
def test_fixed_assigns_the_frozen_region_map(seed):
    spec = generate_episode(seed=seed, level='fixed')
    assert spec.region_map() == FIXED_REGION_MAP


def test_fixed_poses_are_the_region_nominal_poses():
    for t in generate_episode(seed=3, level='fixed').targets:
        region = region_by_id(t.region_id)
        assert (t.x, t.y) == (region.row_x, region.lane_y)
        assert (t.x, t.y) == (TARGET_ROW_X,
                              lane_for_colour(t.colour))


# ── B. COLOUR ────────────────────────────────────────────────────────────
def test_colour_level_permutes_regions_and_keeps_nominal_poses():
    maps = set()
    for seed in range(200):
        spec = generate_episode(seed=seed, level='colours')
        mapping = spec.region_map()
        assert sorted(mapping) == sorted(TARGET_COLOURS)
        assert sorted(mapping.values()) == sorted(REGION_IDS)
        for t in spec.targets:
            region = region_by_id(t.region_id)
            assert (t.x, t.y) == (region.row_x, region.lane_y)
        maps.add(tuple(sorted(mapping.items())))
    # 4! = 24 assignments; 200 seeds must reach more than a handful.
    assert len(maps) > 12


def test_colour_level_is_deterministic_per_seed():
    for seed in (0, 7, 1827):
        a = generate_episode(seed=seed, level='colours')
        b = generate_episode(seed=seed, level='colours')
        assert a.region_map() == b.region_map()
        assert a.to_json() == b.to_json()


def test_positions_and_colours_assign_the_same_regions_per_seed():
    """The permutation is the first draw at both levels."""
    for seed in range(100):
        assert (generate_episode(seed=seed, level='colours').region_map()
                == generate_episode(seed=seed,
                                    level='positions').region_map())


# ── C/D. POSITION, inside the region area ────────────────────────────────
def test_region_area_contains_the_nominal_pose_on_its_near_edge():
    for region in TARGET_REGIONS:
        for t in TARGETS:
            (x_low, x_high), (y_low, y_high) = region_area(region,
                                                           t.diameter)
            assert x_low == region.row_x < x_high
            assert y_low < region.lane_y < y_high
            assert y_high - region.lane_y == pytest.approx(
                REGION_LATERAL_LIMIT)


def test_region_area_never_reaches_past_the_platform_bound():
    for region in TARGET_REGIONS:
        for t in TARGETS:
            (_, x_high), _ = region_area(region, t.diameter)
            _, far = target_x_bounds(t.diameter)
            assert x_high <= far
            assert x_high == pytest.approx(
                region.row_x + (far - region.row_x) * PLACEMENT_FILL)


def test_the_lateral_bound_is_the_measured_absorbed_offset():
    """0.030 m: C2-M4.1's largest lateral offset, grasped. Guard it."""
    assert REGION_LATERAL_LIMIT == 0.030


@pytest.mark.parametrize('level', LEVELS)
def test_no_seed_places_a_target_outside_its_region(level):
    for seed in SEEDS:
        spec = generate_episode(seed=seed, level=level)
        for t in spec.targets:
            region = region_by_id(t.region_id)
            (x_low, x_high), (y_low, y_high) = region_area(region,
                                                           t.diameter)
            assert x_low <= t.x <= x_high, (seed, t)
            assert y_low <= t.y <= y_high, (seed, t)
            # never nearer the crest than the frozen row
            assert t.x >= TARGET_ROW_X, (seed, t)


def test_positions_actually_move_the_targets():
    xs, ys = [], []
    for seed in range(300):
        for t in generate_episode(seed=seed, level='positions').targets:
            region = region_by_id(t.region_id)
            xs.append(t.x - region.row_x)
            ys.append(t.y - region.lane_y)
    assert max(xs) > 0.2 and min(xs) >= 0.0
    assert max(ys) > 0.02 and min(ys) < -0.02


def test_positions_differ_between_seeds_and_repeat_within_one():
    a = generate_episode(seed=11, level='positions')
    assert a.to_json() == generate_episode(seed=11,
                                           level='positions').to_json()
    assert a.to_json() != generate_episode(seed=12,
                                           level='positions').to_json()


# ── F. invalid placement is refused ──────────────────────────────────────
@pytest.fixture
def spec():
    return generate_episode(seed=5, level='positions')


def _retarget(spec, colour, **changes):
    targets = tuple(replace(t, **changes) if t.colour == colour else t
                    for t in spec.targets)
    return replace(spec, targets=targets)


def test_a_target_nearer_the_crest_than_its_region_is_refused(spec):
    t = spec.targets[0]
    with pytest.raises(InvalidEpisode, match='outside region'):
        validate_episode(_retarget(spec, t.colour, x=TARGET_ROW_X - 0.01))


def test_a_target_off_its_lane_by_more_than_the_limit_is_refused(spec):
    t = spec.targets[0]
    lane = region_by_id(t.region_id).lane_y
    with pytest.raises(InvalidEpisode, match='outside region'):
        validate_episode(_retarget(
            spec, t.colour, y=lane + REGION_LATERAL_LIMIT + 0.001))


def test_a_target_with_no_region_is_refused(spec):
    t = spec.targets[0]
    with pytest.raises(InvalidEpisode, match='names no known region'):
        validate_episode(_retarget(spec, t.colour, region_id=''))


def test_two_targets_in_one_region_are_refused(spec):
    a, b = spec.targets[0], spec.targets[1]
    with pytest.raises(InvalidEpisode, match='share region'):
        validate_episode(_retarget(spec, b.colour, region_id=a.region_id))


def test_a_pre_region_manifest_loads_but_does_not_validate():
    """p03 manifests had no region_id: replayable data, not a legal run."""
    data = json.loads(generate_episode(seed=2, level='fixed').to_json())
    for t in data['targets']:
        del t['region_id']
    old = episode_from_json(json.dumps(data))
    assert all(t.region_id == '' for t in old.targets)
    with pytest.raises(InvalidEpisode, match='no known region'):
        validate_episode(old)


# ── G. target-region mapping reaches the mission as names only ──────────
@pytest.mark.parametrize('level', LEVELS)
def test_compat_inputs_resolve_every_colour_to_its_regions_lane(level):
    for seed in range(100):
        spec = generate_episode(seed=seed, level=level)
        inputs = compat_mission_inputs(spec)
        assert set(inputs) == {'target_colour', 'region_map'}
        assert inputs['target_colour'] == spec.requested_colour
        mapping = parse_region_map(inputs['region_map'])
        for t in spec.targets:
            assert resolve_lane(t.colour, mapping) == \
                region_by_id(t.region_id).lane_y


@pytest.mark.parametrize('level', LEVELS)
def test_compat_inputs_carry_no_coordinate(level):
    for seed in range(100):
        spec = generate_episode(seed=seed, level=level)
        text = json.dumps(compat_mission_inputs(spec))
        for t in spec.targets:
            for value in (t.x, t.y, t.z):
                assert repr(value) not in text
                assert f'{value:.2f}' not in text


def test_fixed_compat_inputs_resolve_exactly_like_the_frozen_table():
    spec = generate_episode(seed=0, level='fixed', requested_colour='blue')
    mapping = parse_region_map(compat_mission_inputs(spec)['region_map'])
    for colour in TARGET_COLOURS:
        assert resolve_lane(colour, mapping) == lane_for_colour(colour)


def test_the_task_view_did_not_grow():
    for level in LEVELS:
        view = generate_episode(seed=4, level=level).task_view()
        assert set(view) == {'episode_id', 'requested_colour'}
        assert 'region' not in json.dumps(view)
        assert 'lane' not in json.dumps(view)


# ── J. resolving and recording what a launch was asked for ───────────────
def test_resolve_episode_defaults_to_the_fixed_layout():
    spec = resolve_episode()
    assert spec.level == 'fixed' and spec.backend == 'gazebo'
    assert spec.region_map() == FIXED_REGION_MAP


def test_resolve_episode_takes_launch_strings():
    spec = resolve_episode(level='colours', seed='1827',
                           requested_colour='red')
    assert spec == generate_episode(seed=1827, level='colours',
                                    requested_colour='red')
    assert resolve_episode(level='colours', seed='7',
                           requested_colour='') == generate_episode(
        seed=7, level='colours')


def test_resolve_episode_refuses_a_non_integer_seed():
    with pytest.raises(InvalidEpisode, match='integer'):
        resolve_episode(level='fixed', seed='abc')


def test_a_recorded_manifest_is_replayed_exactly(tmp_path):
    spec = generate_episode(seed=31, level='positions',
                            requested_colour='green')
    path = tmp_path / 'manifest.json'
    path.write_text(spec.to_json())
    back = resolve_episode(level='fixed', seed='0',
                           manifest_path=str(path))
    assert back == spec              # the manifest wins over seed/level
    assert back.to_json() == spec.to_json()


def test_a_manifest_for_another_backend_is_refused(tmp_path):
    spec = generate_episode(seed=1, level='colours', backend='isaac')
    path = tmp_path / 'manifest.json'
    path.write_text(spec.to_json())
    with pytest.raises(InvalidEpisode, match='isaac episode'):
        resolve_episode(manifest_path=str(path), backend='gazebo')


def test_an_illegal_recorded_manifest_is_refused(tmp_path):
    data = json.loads(generate_episode(seed=1, level='positions').to_json())
    data['targets'][0]['y'] += 0.5
    path = tmp_path / 'manifest.json'
    path.write_text(json.dumps(data))
    with pytest.raises(InvalidEpisode):
        resolve_episode(manifest_path=str(path))


def test_layout_does_not_depend_on_the_requested_colour():
    """Pin any colour: the layout of a seed does not move.

    The world launch and the mission launch may pin different colours for
    the SAME seed and still agree on where everything is.
    """
    for level in LEVELS:
        for seed in range(50):
            poses = {
                colour: [(t.colour, t.x, t.y, t.region_id) for t in
                         generate_episode(seed=seed, level=level,
                                          requested_colour=colour).targets]
                for colour in TARGET_COLOURS}
            first = poses[TARGET_COLOURS[0]]
            assert all(p == first for p in poses.values())


def test_every_positions_seed_keeps_the_full_envelope():
    """The region area sits inside p03's envelope; check the corners."""
    for seed in range(500):
        spec = generate_episode(seed=seed, level='positions')
        validate_episode(spec)
        for i, a in enumerate(spec.targets):
            for b in spec.targets[i + 1:]:
                assert abs(a.y - b.y) > 0.4, (seed, a, b)
                assert math.hypot(a.x - b.x, a.y - b.y) > 0.4
