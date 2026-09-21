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

"""Check axis vocabulary and policy-driven transition validation."""

from coco_web import lifecycle as life

import pytest


@pytest.mark.parametrize('state', sorted(life.LIFECYCLES))
@pytest.mark.parametrize('health', sorted(life.HEALTH))
def test_axes_are_independent(state, health):
    """Every lifecycle can be represented separately from health."""
    assert life.validate_axes(state, health) == (state, health)


@pytest.mark.parametrize('current,target', [
    ('CREATED', 'STARTING'), ('STARTING', 'READY'), ('READY', 'RUNNING'),
    ('RUNNING', 'READY'), ('RUNNING', 'STOPPING'), ('STOPPING', 'STOPPED'),
    ('RUNNING', 'FAILED'), ('FAILED', 'STARTING'), ('STOPPED', 'STARTING'),
])
def test_transition_requires_explicit_owner_policy(current, target):
    """Forward, failure and restart edges need the same explicit authority."""
    assert life.validate_transition(current, target, [(current, target)])
    with pytest.raises(ValueError):
        life.validate_transition(current, target, [])


@pytest.mark.parametrize('state', sorted(life.LIFECYCLES))
def test_duplicate_transition_is_idempotent(state):
    """Repeated notifications do not fabricate another transition."""
    assert life.validate_transition(state, state, []) is False


@pytest.mark.parametrize('target', ['CREATED', 'HEALTHY', 'DISCONNECTED',
                                    'BROKEN', None, []])
def test_backwards_or_wrong_axis_value_is_refused(target):
    """Socket disconnect is not a new lifecycle; backwards edges need policy."""
    with pytest.raises(ValueError):
        life.validate_transition('RUNNING', target, [('RUNNING', 'READY')])


def test_policy_must_not_mix_health_and_lifecycle():
    """A typo in owner policy must fail before processing a duplicate."""
    with pytest.raises(ValueError):
        life.validate_transition('READY', 'READY', [('READY', 'UNHEALTHY')])
