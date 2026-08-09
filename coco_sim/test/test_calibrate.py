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
The calibration harness must not be able to sweep a parameter that is not
reaching the model.

That is not hypothetical. The previous harness patched the literal
``solref="0.1 1"`` in the generated MJCF; when the fitted value 0.25 was
written back into ``mjcf.py`` the pattern stopped matching, the lever
detached, and three distinct values produced bit-for-bit identical scores
while the sweep printed a normal-looking table. The calibration the world
currently ships was fitted through a harness in that state.
"""
import inspect
import math

from coco_sim import calibrate
from coco_sim.calibrate import (audit_levers, gazebo_reference, score)
from coco_sim.mjcf import (CONTACT_SOLIMP_D0, CONTACT_SOLREF, TIMESTEP,
                           WHEEL_FRICTION, build_mjcf)
from coco_sim.sweep import DisconnectedParameter

import pytest


# ── the levers, checked at the model level (no simulation) ──────────────
@pytest.mark.parametrize('kwargs', [
    {'friction': 0.9}, {'solref': 0.05}, {'solimp': 0.9},
    {'timestep': 0.001}, {'kv': 25},
])
def test_every_contact_parameter_changes_the_generated_model(kwargs):
    """The cheapest possible version of the check that was missing."""
    assert build_mjcf(**kwargs) != build_mjcf()


def test_defaults_reproduce_the_committed_constants():
    """Calling with no arguments must give exactly the shipped world."""
    xml = build_mjcf()
    assert f'solref="{CONTACT_SOLREF} 1"' in xml
    assert f'solimp="{CONTACT_SOLIMP_D0} 0.99 0.001"' in xml
    assert f'friction="{WHEEL_FRICTION} {WHEEL_FRICTION} 0.005' in xml
    assert f'timestep="{TIMESTEP}"' in xml


def test_the_harness_does_not_sweep_by_string_replacement():
    """The root cause, guarded directly.

    A harness that reaches into generated XML with str.replace is one
    edit away from silently detaching. Parameters go through the
    generator's signature or not at all.
    """
    src = inspect.getsource(calibrate)
    for banned in ('.replace(', 're.sub('):
        assert banned not in src, (
            f'{banned} in calibrate.py — sweep parameters through '
            f'build_mjcf() arguments, not by patching its output. That is '
            f'exactly how the solref lever came to be disconnected.')


# ── the reference ───────────────────────────────────────────────────────
def test_the_gazebo_reference_is_read_not_transcribed():
    ref = gazebo_reference()
    assert set(ref) == {0.05, 0.10, 0.25, 0.50, 1.00, 1.50, 2.50}
    # magnitude-averaged over the +/- pair, so nothing is far from 100 %
    # at the small commands where Gazebo tracks well
    assert ref[0.05] == pytest.approx(103.52, abs=0.05)
    assert ref[2.50] == pytest.approx(65.71, abs=0.05)


def test_the_reference_records_both_signs():
    """Gazebo disagrees with its own mirrored command by 1.36x at 2.5 rad.
    Averaging is only defensible if both signs were actually measured."""
    import csv
    with open(calibrate.REFERENCE) as handle:
        rows = list(csv.DictReader(handle))
    signs = {}
    for row in rows:
        cmd = round(float(row['cmd_yaw']), 4)
        signs.setdefault(abs(cmd), set()).add(math.copysign(1, cmd))
    for cmd, seen in signs.items():
        assert seen == {1.0, -1.0}, f'|cmd| {cmd} measured in one sign only'


# ── the levers, checked through the score (simulated) ───────────────────
def test_all_four_levers_are_live_through_the_score():
    """The audit that must pass before any fit is trusted.

    Deliberately cheap — one command, two values per lever — because its
    job is to detect a DISCONNECTED parameter, not to measure sensitivity.
    """
    spreads = audit_levers(
        levers={'friction': (0.25, 0.70), 'solref': (0.05, 0.40),
                'solimp': (0.20, 0.90), 'sep_mult': (1.00, 1.25)},
        commands=(1.00,))
    assert set(spreads) == {'friction', 'solref', 'solimp', 'sep_mult'}
    for name, spread in spreads.items():
        assert spread > 0.0, f'{name} did not move the score'


def test_the_audit_fails_on_a_lever_that_is_not_connected(monkeypatch):
    """Guard the guard: if this stops raising, the audit proves nothing.

    Simulates the real failure — DISTINCT inputs, identical outputs — by
    making the scorer ignore its argument, which is precisely what string
    replacement against a stale literal did.
    """
    monkeypatch.setattr(calibrate, 'score',
                        lambda **kw: (1.234567, {}))
    with pytest.raises(DisconnectedParameter):
        audit_levers(levers={'friction': (0.25, 0.70)}, commands=(1.00,))


def test_the_audit_rejects_a_sweep_whose_inputs_are_not_distinct():
    """Identical outputs prove nothing if the inputs were identical too."""
    with pytest.raises(ValueError):
        audit_levers(levers={'friction': (0.4, 0.4)}, commands=(1.00,))


def test_score_is_symmetric_in_over_and_under_steering():
    """A plain ratio would reward a model that over-rotates. Both
    directions must be penalised, or the fit drifts to the fast side."""
    ref = {1.00: 100.0}
    worst_under, _ = score(commands=(1.00,), reference={1.00: 50.0})
    worst_over, _ = score(commands=(1.00,), reference={1.00: 200.0})
    assert worst_under > 1.0 and worst_over > 1.0
    assert ref  # the fixture above is only here to name the intent
