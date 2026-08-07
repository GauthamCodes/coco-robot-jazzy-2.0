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

"""The disconnected-parameter canary must itself be hard to fool."""
import pytest

from coco_sim.sweep import DisconnectedParameter, assert_lever_is_connected


def test_a_connected_lever_passes_and_returns_its_spread():
    spread = assert_lever_is_connected(
        'solref', [(0.1, 59.5), (0.25, 65.2), (0.4, 72.0)])
    assert spread == pytest.approx(12.5)


def test_a_disconnected_lever_raises_and_names_itself():
    """The exact failure that cost this project a calibration."""
    with pytest.raises(DisconnectedParameter) as exc:
        assert_lever_is_connected(
            'solref', [(0.1, 1.547), (0.25, 1.547), (0.35, 1.547)])
    assert 'solref' in str(exc.value)


def test_a_weak_but_real_lever_still_passes():
    """Weak is not the same as disconnected, and the guard must not
    conflate them — sliding friction is a real lever that happens to be
    ~3.4x weaker than contact softness, and it must not trip this."""
    spread = assert_lever_is_connected(
        'friction', [(0.2, 100.9), (0.4, 100.9000001)])
    assert spread > 0


def test_default_tolerance_is_exactly_zero():
    """A genuinely weak lever still moves the last bits of a float; a
    disconnected one returns bit-for-bit the same value. Any non-zero
    default would quietly turn this wiring check into a sensitivity
    check, which is a different question."""
    with pytest.raises(DisconnectedParameter):
        assert_lever_is_connected('x', [(1, 1.0), (2, 1.0)])
    assert_lever_is_connected('x', [(1, 1.0), (2, 1.0 + 5e-16)])


def test_identical_inputs_are_rejected_as_meaningless():
    """Identical outputs prove nothing if the inputs were not distinct."""
    with pytest.raises(ValueError, match='not distinct'):
        assert_lever_is_connected('x', [(0.5, 1.0), (0.5, 1.0)])


def test_a_single_sample_is_rejected():
    with pytest.raises(ValueError, match='at least two'):
        assert_lever_is_connected('x', [(0.5, 1.0)])


def test_the_message_points_at_the_pair_shadowing_trap():
    """The error should teach the reader the mechanism, not just complain.

    Whoever hits this next is most likely to have been bitten by the same
    shadowing behaviour, so the message names it.
    """
    with pytest.raises(DisconnectedParameter) as exc:
        assert_lever_is_connected('solimp', [(0.2, 5.0), (0.5, 5.0)])
    assert 'pair' in str(exc.value).lower()
