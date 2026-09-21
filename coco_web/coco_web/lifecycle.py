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
Validate the locked axes using an integration-owned transition policy.

This module does not derive lifecycle from component health or define a new
restart policy. The owner supplies allowed edges explicitly. It can be used
by a state holder or tests without importing or replacing session.py.
"""

LIFECYCLES = frozenset(('CREATED', 'STARTING', 'READY', 'RUNNING',
                       'STOPPING', 'STOPPED', 'FAILED'))
HEALTH = frozenset(('HEALTHY', 'DEGRADED', 'UNHEALTHY'))


def validate_axes(lifecycle, health):
    """Return the two validated axes without conflating their meanings."""
    if not isinstance(lifecycle, str) or lifecycle not in LIFECYCLES:
        raise ValueError('unknown lifecycle')
    if not isinstance(health, str) or health not in HEALTH:
        raise ValueError('unknown health')
    return lifecycle, health


def validate_transition(current, target, allowed_edges):
    """
    Return whether a legal transition changes state; duplicates are no-ops.

    Every edge is a pair of lifecycle names. Restart and failure edges must
    be supplied by the session owner, not inferred from readiness or a
    WebSocket disconnect. Invalid requests never mutate caller state.
    """
    for value in (current, target):
        if not isinstance(value, str) or value not in LIFECYCLES:
            raise ValueError('unknown lifecycle')
    edges = set()
    for edge in allowed_edges:
        if (not isinstance(edge, (tuple, list)) or len(edge) != 2
                or any(not isinstance(value, str) or value not in LIFECYCLES
                       for value in edge)):
            raise ValueError('invalid lifecycle policy edge')
        edges.add(tuple(edge))
    if current == target:
        return False
    if (current, target) not in edges:
        raise ValueError(f'illegal lifecycle transition: {current} -> {target}')
    return True
