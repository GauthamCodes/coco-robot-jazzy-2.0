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
The episode specification: reproducible, valid, and not a cheat sheet.

Four properties, each load-bearing for everything built on top:

1. **Baseline identity.** ``level='fixed'`` is the frozen P0.2 layout,
   pose for pose, so the episode layer can coexist with the mission.
2. **Determinism.** Same seed, same manifest, byte for byte.
3. **Validity.** No generated target lies outside the envelope derived
   from ``coco_config`` — checked over many seeds, not one.
4. **The boundary.** The robot's view carries no geometry.
"""

from dataclasses import replace
import json
import math

from coco_config.robot import (approach_window, PLATFORM_LEN,
                               RAMP_SUMMIT_X, RAMP_WIDTH, SPAWN_XY,
                               TARGET_COLOURS, TARGET_ROW_X, TARGETS)
from coco_sim.episode import (BACKENDS, episode_from_json,
                              episode_from_manifest, EpisodeSpec,
                              generate_episode, HALF_FOOTPRINT_X,
                              InvalidEpisode, LEVELS, min_target_separation,
                              ObstacleSpec, PLATFORM_TOP_Z, rebind,
                              target_x_bounds, target_y_bounds,
                              validate_episode, WORLD_VARIANTS)

import pytest

#: Enough seeds that a rare invalid draw would show up, few enough to
#: keep the package's suite fast.
SEEDS = range(500)


# ── the envelope is DERIVED, and agrees with the pinned literals ─────────
def test_half_footprint_matches_the_frozen_lane_test():
    """The derivation reproduces test_targets.py's literal 0.1485."""
    assert HALF_FOOTPRINT_X == pytest.approx(0.1485, abs=1e-9)


def test_the_frozen_row_is_inside_the_derived_envelope():
    """If P0.2's own layout failed validation, the envelope would be wrong."""
    for target in TARGETS:
        near, far = target_x_bounds(target.diameter)
        assert near <= TARGET_ROW_X <= far
        low, high = target_y_bounds()
        assert low <= target.lane_y <= high


def test_the_envelope_sits_on_the_platform():
    near, far = target_x_bounds(max(t.diameter for t in TARGETS))
    assert RAMP_SUMMIT_X < near < far < RAMP_SUMMIT_X + PLATFORM_LEN
    low, high = target_y_bounds()
    assert -RAMP_WIDTH / 2.0 < low < 0.0 < high < RAMP_WIDTH / 2.0


def test_every_colour_has_a_graspable_window():
    for colour in TARGET_COLOURS:
        near, far = approach_window(colour)
        assert near < far


# ── baseline identity ────────────────────────────────────────────────────
@pytest.mark.parametrize('seed', [0, 1, 7, 12345])
def test_fixed_level_is_the_p02_layout_for_any_seed(seed):
    """The default level must not move a single spawned pose."""
    spec = generate_episode(seed=seed, requested_colour='blue')
    assert spec.level == 'fixed'
    assert len(spec.targets) == len(TARGETS)
    for got, frozen in zip(spec.targets, TARGETS):
        assert got.colour == frozen.colour
        assert got.model == frozen.model
        assert got.x == TARGET_ROW_X
        assert got.y == frozen.lane_y
        assert got.diameter == frozen.diameter
        assert got.height == frozen.height
        assert got.z == pytest.approx(PLATFORM_TOP_Z + frozen.height / 2.0)


def test_fixed_level_starts_the_robot_at_the_p02_spawn():
    start = generate_episode(seed=3).robot_start
    assert (start.x, start.y) == SPAWN_XY


def test_defaults_are_the_frozen_world_on_gazebo_with_no_obstacles():
    spec = generate_episode(seed=0)
    assert spec.backend == 'gazebo'
    assert spec.world_variant == 'coco_world'
    assert spec.obstacles == ()


# ── determinism ──────────────────────────────────────────────────────────
@pytest.mark.parametrize('level', LEVELS)
def test_same_seed_gives_a_byte_identical_manifest(level):
    for seed in (0, 1, 42, 2 ** 31 - 1):
        first = generate_episode(seed=seed, level=level).to_json()
        second = generate_episode(seed=seed, level=level).to_json()
        assert first == second


@pytest.mark.parametrize('level', ['colours', 'positions'])
def test_different_seeds_give_different_layouts(level):
    layouts = {
        tuple((t.colour, round(t.x, 9), round(t.y, 9))
              for t in generate_episode(seed=s, level=level).targets)
        for s in range(50)
    }
    # 4! = 24 colour permutations bound the 'colours' level.
    assert len(layouts) >= (20 if level == 'colours' else 50)


def test_colours_level_permutes_colours_but_keeps_the_lanes():
    frozen_lanes = sorted(t.lane_y for t in TARGETS)
    moved = 0
    for seed in range(50):
        spec = generate_episode(seed=seed, level='colours')
        assert sorted(t.y for t in spec.targets) == frozen_lanes
        assert all(t.x == TARGET_ROW_X for t in spec.targets)
        if any(t.y != TARGETS[TARGET_COLOURS.index(t.colour)].lane_y
               for t in spec.targets):
            moved += 1
    assert moved > 0, 'no seed moved any colour off its P0.2 lane'


def test_requested_colour_is_seeded_and_varies():
    asked = {generate_episode(seed=s).requested_colour for s in range(50)}
    assert asked == set(TARGET_COLOURS)


def test_episode_id_is_stable_and_distinguishes_episodes():
    a = generate_episode(seed=5, level='positions')
    assert a.episode_id == generate_episode(seed=5,
                                            level='positions').episode_id
    assert a.episode_id != generate_episode(seed=6,
                                            level='positions').episode_id
    assert a.episode_id.startswith('ep-')


# ── validity over many seeds ─────────────────────────────────────────────
@pytest.mark.parametrize('level', LEVELS)
def test_no_seed_generates_a_target_outside_the_envelope(level):
    low, high = target_y_bounds()
    for seed in SEEDS:
        spec = generate_episode(seed=seed, level=level)
        for t in spec.targets:
            near, far = target_x_bounds(t.diameter)
            assert near <= t.x <= far, (seed, t)
            assert low <= t.y <= high, (seed, t)
        for i, a in enumerate(spec.targets):
            for b in spec.targets[i + 1:]:
                assert (math.hypot(a.x - b.x, a.y - b.y)
                        >= min_target_separation(a, b)), (seed, a, b)


# ── invalid placement is rejected ────────────────────────────────────────
def _move(spec, colour, **pose):
    targets = tuple(replace(t, **pose) if t.colour == colour else t
                    for t in spec.targets)
    return replace(spec, targets=targets)


@pytest.fixture
def good():
    return generate_episode(seed=0, requested_colour='red')


def test_the_fixture_is_valid(good):
    validate_episode(good)


def test_a_target_too_close_to_the_crest_is_rejected(good):
    near, _ = target_x_bounds(good.target('red').diameter)
    with pytest.raises(InvalidEpisode, match='reachable row'):
        validate_episode(_move(good, 'red', x=near - 0.001))


def test_a_target_off_the_far_edge_is_rejected(good):
    with pytest.raises(InvalidEpisode, match='reachable row'):
        validate_episode(_move(good, 'red',
                               x=RAMP_SUMMIT_X + PLATFORM_LEN + 0.01))


def test_a_target_outside_the_standable_band_is_rejected(good):
    _, high = target_y_bounds()
    with pytest.raises(InvalidEpisode, match='standable band'):
        validate_episode(_move(good, 'red', y=high + 0.001))


def test_a_floating_target_is_rejected(good):
    with pytest.raises(InvalidEpisode, match='does not rest'):
        validate_episode(_move(good, 'red',
                               z=good.target('red').z + 0.05))


def test_two_targets_too_close_are_rejected(good):
    green = good.target('green')
    with pytest.raises(InvalidEpisode, match='closer than'):
        validate_episode(_move(good, 'red', x=green.x, y=green.y + 0.05))


# ── schema ───────────────────────────────────────────────────────────────
@pytest.mark.parametrize('field,value,message', [
    ('level', 'chaos', 'unknown level'),
    ('backend', 'unity', 'unknown backend'),
    ('world_variant', 'mars', 'unknown world variant'),
    ('seed', '7', 'seed must be an int'),
    ('requested_colour', 'purple', 'not present'),
    ('targets', (), 'no targets'),
])
def test_schema_violations_are_rejected(good, field, value, message):
    with pytest.raises(InvalidEpisode, match=message):
        validate_episode(replace(good, **{field: value}))


def test_duplicate_colours_are_rejected(good):
    red = good.target('red')
    dup = replace(good, targets=good.targets + (red,))
    with pytest.raises(InvalidEpisode, match='duplicate'):
        validate_episode(dup)


def test_an_unknown_level_is_rejected_before_generation():
    with pytest.raises(InvalidEpisode, match='unknown level'):
        generate_episode(seed=0, level='chaos')


# ── simulator field ──────────────────────────────────────────────────────
@pytest.mark.parametrize('backend', BACKENDS)
def test_every_backend_generates_the_same_world(backend):
    """The backend is recorded, and changes nothing about the layout."""
    here = generate_episode(seed=11, level='positions', backend=backend)
    ref = generate_episode(seed=11, level='positions', backend='gazebo')
    assert here.backend == backend
    assert here.targets == ref.targets
    assert here.robot_start == ref.robot_start


def test_backend_changes_the_episode_id():
    ids = {generate_episode(seed=1, backend=b).episode_id for b in BACKENDS}
    assert len(ids) == len(BACKENDS)


def test_rebind_recomputes_the_id_and_revalidates(good):
    moved = rebind(good, backend='isaac')
    assert moved.backend == 'isaac'
    assert moved.episode_id != good.episode_id
    assert moved.targets == good.targets
    with pytest.raises(InvalidEpisode):
        rebind(good, backend='unity')


@pytest.mark.parametrize('variant', WORLD_VARIANTS)
def test_world_variants_are_accepted(variant):
    assert generate_episode(seed=0, world_variant=variant).world_variant \
        == variant


# ── serialisation ────────────────────────────────────────────────────────
@pytest.mark.parametrize('level', LEVELS)
def test_json_round_trips_exactly(level):
    spec = generate_episode(seed=99, level=level,
                            metadata={'software_commit': 'c40098f'})
    back = episode_from_json(spec.to_json())
    assert back == spec
    assert back.to_json() == spec.to_json()


def test_obstacles_round_trip_including_tuples():
    wall = ObstacleSpec(name='crate', kind='box', x=0.5, y=-2.0, z=0.25,
                        size=(0.5, 0.5, 0.5))
    mover = ObstacleSpec(name='cart', kind='cylinder', x=-1.0, y=2.0, z=0.3,
                         size=(0.2, 0.6), motion='waypoint', enabled=False,
                         waypoints=((-1.0, 2.0), (1.0, 2.0)), speed=0.3)
    spec = generate_episode(seed=4, obstacles=(wall, mover))
    back = episode_from_manifest(json.loads(spec.to_json()))
    assert back.obstacles == (wall, mover)
    assert isinstance(back.obstacles[1].waypoints[0], tuple)


def test_dynamic_obstacles_are_represented_but_never_generated():
    for level in LEVELS:
        for seed in range(20):
            assert generate_episode(seed=seed, level=level).obstacles == ()


def test_the_manifest_is_complete():
    """Every field an evaluator needs to answer "what was episode N?"."""
    data = generate_episode(seed=8, level='positions').manifest()
    for key in ('episode_id', 'seed', 'level', 'backend', 'world_variant',
                'ramp_angle_deg', 'requested_colour', 'targets',
                'obstacles', 'robot_start', 'metadata'):
        assert key in data, key
    for target in data['targets']:
        assert set(target) == {'colour', 'model', 'x', 'y', 'z',
                               'diameter', 'height'}
    assert set(data['robot_start']) == {'x', 'y', 'z', 'yaw'}


def test_the_spec_is_immutable():
    spec = generate_episode(seed=0)
    with pytest.raises(Exception):
        spec.seed = 1


def test_the_manifest_is_json_native():
    """No numpy scalar or tuple survives into the JSON document."""
    text = generate_episode(seed=2, level='positions').to_json()
    assert json.loads(text)['seed'] == 2


# ── the privileged/observable boundary ──────────────────────────────────
@pytest.mark.parametrize('level', LEVELS)
def test_the_task_view_leaks_no_geometry(level):
    """The robot is told WHAT to find, never WHERE it is."""
    for seed in range(50):
        spec = generate_episode(seed=seed, level=level)
        view = spec.task_view()
        assert set(view) == {'episode_id', 'requested_colour'}
        text = json.dumps(view)
        for t in spec.targets:
            for value in (t.x, t.y, t.z):
                assert repr(value) not in text
                assert f'{value:.2f}' not in text
        assert 'lane' not in text and 'backend' not in text


def test_the_task_view_names_a_colour_that_is_in_the_world():
    for seed in range(50):
        spec = generate_episode(seed=seed, level='positions')
        assert spec.target(spec.task_view()['requested_colour']) is not None


def test_episode_spec_has_no_field_named_like_a_pose_in_its_view():
    """Belt and braces: the view's keys come from a fixed, audited set."""
    assert EpisodeSpec.task_view.__doc__
    assert set(generate_episode(seed=0).task_view()) == {
        'episode_id', 'requested_colour'}
