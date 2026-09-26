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
The climb's lane hold follows the episode's region map (stage C).

ramp_driver measures cross-track against a lane it takes from the
mission's chosen colour. If that lane came from the frozen table while
the mission sent the robot up an episode's lane, the lane hold would
steer the climb toward the WRONG line — and the executive's CLIMB check,
which prefers ramp_driver's published cross-track, would read the
result against it too. So the datum is resolved with the same function
and the same map the executive uses.

Pure: the method is called on a stub, as test_ramp_driver.py does for
``live_pitch``; no ROS graph, no policy.
"""

from types import SimpleNamespace

from coco_config.robot import (format_region_map, lane_for_colour,
                               lane_for_region, parse_region_map)
import pytest

driver = pytest.importorskip('coco_rl.ramp_driver')

ROTATED = {'red': 'lane_2', 'green': 'lane_3', 'blue': 'lane_4',
           'yellow': 'lane_1'}


class _Log:

    def info(self, *_):
        pass


def _stub(region_map):
    return SimpleNamespace(_region_map=region_map, _lane_y=None,
                           get_logger=lambda: _Log())


def _say(stub, colour):
    driver.RampDriver._on_colour(stub, SimpleNamespace(data=colour))


@pytest.mark.parametrize('colour', ['red', 'green', 'blue', 'yellow'])
def test_no_map_is_the_frozen_table(colour):
    stub = _stub({})
    _say(stub, colour)
    assert stub._lane_y == lane_for_colour(colour)


@pytest.mark.parametrize('colour', ['red', 'green', 'blue', 'yellow'])
def test_the_datum_is_the_episode_regions_lane(colour):
    stub = _stub(ROTATED)
    _say(stub, colour)
    assert stub._lane_y == lane_for_region(ROTATED[colour])


def test_the_map_round_trips_through_the_parameter_form():
    stub = _stub(parse_region_map(format_region_map(ROTATED)))
    _say(stub, ' Yellow \n')
    assert stub._lane_y == lane_for_region('lane_1')


def test_an_unknown_colour_leaves_the_datum_alone():
    stub = _stub(ROTATED)
    _say(stub, 'blue')
    _say(stub, 'purple')
    assert stub._lane_y == lane_for_region('lane_4')


def test_ramp_driver_declares_the_region_map_parameter():
    """Static guard: the launch file sets it, so the node must read it."""
    import inspect
    source = inspect.getsource(driver.RampDriver.__init__)
    assert "declare_parameter('region_map', '')" in source
    assert 'parse_region_map' in source
