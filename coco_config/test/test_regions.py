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
Target regions: the lanes, named, with the colour taken off them.

The region table is what lets an episode say "red stands in lane_3"
without anyone on the robot side ever handling a coordinate. These tests
pin the two properties that make it safe to introduce: it is DERIVED
from TARGETS (so it cannot drift from the world that is spawned), and
with no region map every lookup is exactly the frozen colour->lane table.
"""

from coco_config.robot import (FIXED_REGION_MAP, format_region_map,
                               lane_for_colour, lane_for_region,
                               parse_region_map, region_by_id,
                               REGION_IDS, region_for_lane, resolve_lane,
                               TARGET_COLOURS, TARGET_PLATFORM,
                               TARGET_REGIONS, TARGET_ROW_X, TARGETS)

import pytest


def test_one_region_per_frozen_lane():
    assert sorted(r.lane_y for r in TARGET_REGIONS) == sorted(
        t.lane_y for t in TARGETS)
    assert len(set(REGION_IDS)) == len(TARGET_REGIONS) == len(TARGETS)


def test_regions_are_ordered_by_y_and_named_stably():
    """lane_1 is the most negative y; the names never depend on colour."""
    assert REGION_IDS == ('lane_1', 'lane_2', 'lane_3', 'lane_4')
    lanes = [r.lane_y for r in TARGET_REGIONS]
    assert lanes == sorted(lanes)


def test_every_region_is_on_the_crest_at_the_frozen_row():
    for region in TARGET_REGIONS:
        assert region.platform == TARGET_PLATFORM
        assert region.row_x == TARGET_ROW_X


def test_a_region_carries_no_colour():
    """Colour is identity; a region is geometry. Keep them apart."""
    for region in TARGET_REGIONS:
        assert not set(region._fields) & {'colour', 'model', 'rgb'}


def test_the_fixed_map_is_todays_colour_to_lane_table():
    assert set(FIXED_REGION_MAP) == set(TARGET_COLOURS)
    for colour, region_id in FIXED_REGION_MAP.items():
        assert lane_for_region(region_id) == lane_for_colour(colour)


def test_lookups_refuse_unknown_names_rather_than_defaulting():
    assert region_by_id('lane_9') is None
    assert lane_for_region('lane_9') is None
    assert region_for_lane(0.0) is None     # between lanes 2 and 3


@pytest.mark.parametrize('colour', TARGET_COLOURS)
def test_no_region_map_is_exactly_the_frozen_lookup(colour):
    """The compatibility promise: pass nothing, get P0.2."""
    assert resolve_lane(colour) == lane_for_colour(colour)
    assert resolve_lane(colour, {}) == lane_for_colour(colour)
    assert resolve_lane(colour, FIXED_REGION_MAP) == lane_for_colour(colour)


def test_a_region_map_moves_the_lane_with_the_colour():
    swapped = dict(FIXED_REGION_MAP, red='lane_4', yellow='lane_1')
    assert resolve_lane('red', swapped) == lane_for_region('lane_4')
    assert resolve_lane('yellow', swapped) == lane_for_region('lane_1')
    assert resolve_lane('red', swapped) != lane_for_colour('red')


def test_an_unknown_colour_resolves_to_none_with_or_without_a_map():
    assert resolve_lane('purple') is None
    assert resolve_lane('purple', FIXED_REGION_MAP) is None


def test_the_wire_form_round_trips_and_is_canonical():
    text = format_region_map(FIXED_REGION_MAP)
    assert text == ('blue=lane_3,green=lane_2,red=lane_1,yellow=lane_4')
    assert parse_region_map(text) == FIXED_REGION_MAP
    shuffled = ','.join(reversed(text.split(',')))
    assert format_region_map(parse_region_map(shuffled)) == text


def test_blank_text_is_no_map():
    assert parse_region_map('') == {}
    assert parse_region_map('   ') == {}
    assert parse_region_map(None) == {}


@pytest.mark.parametrize('text,message', [
    ('red', 'malformed'),
    ('red=lane_1,green=', 'malformed'),
    ('purple=lane_1', 'unknown colour'),
    ('red=lane_9', 'unknown region'),
    ('red=lane_1,red=lane_2', 'twice'),
    ('red=lane_1,green=lane_1,blue=lane_3,yellow=lane_4', 'share a region'),
    ('red=lane_1,green=lane_2,blue=lane_3', 'does not place'),
])
def test_an_illegal_map_is_refused_not_half_applied(text, message):
    """A partial map would send the robot up the wrong lane."""
    with pytest.raises(ValueError, match=message):
        parse_region_map(text)


def test_the_wire_form_carries_no_coordinate():
    """Region names only: the robot is never handed a number here."""
    text = format_region_map(FIXED_REGION_MAP)
    for region in TARGET_REGIONS:
        assert f'{region.lane_y:.2f}' not in text
        assert repr(region.lane_y) not in text
    assert repr(TARGET_ROW_X) not in text
