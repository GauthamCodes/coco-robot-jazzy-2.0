# Copyright 2026 Gautham Anil
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Adversarial protocol inputs must produce refusals, never tracebacks."""

import json

from coco_web import protocol

import pytest


@pytest.mark.parametrize('raw', [
    '{"type":"stop","type":"drive"}',
    '{"type":"ping","t":NaN}',
    '{"type":"ping","t":Infinity}',
    '{"type":"ping","t":1e999}',
    '{"type":"stop","id":true}',
    '{"type":"drive","linear":' + '9' * 400 + '}',
    '[' * 1100 + '0' + ']' * 1100,
])
def test_ambiguous_or_unrepresentable_json_is_refused(raw):
    """No parser ambiguity or float conversion overflow crosses the boundary."""
    with pytest.raises(protocol.ProtocolError):
        protocol.decode(raw)


@pytest.mark.parametrize('kind', ['publish', 'call_service', 'command', 'ack',
                                  'error', 'advertise', 'rosapi'])
def test_only_existing_client_command_vocabulary_is_accepted(kind):
    """Server responses and generic ROS envelopes are not robot commands."""
    with pytest.raises(protocol.ProtocolError):
        protocol.decode(json.dumps({'type': kind}))


@pytest.mark.parametrize('field', ['topic', 'service', 'target', 'message_type'])
@pytest.mark.parametrize('command', [{'type': 'stop'},
                                     {'type': 'drive', 'linear': 0.0},
                                     {'type': 'mission', 'action': 'start'}])
def test_approved_commands_cannot_name_a_ros_target(field, command):
    """Adding a wheel destination to an approved intent is an error."""
    with pytest.raises(protocol.ProtocolError):
        protocol.decode(json.dumps({**command, field: '/wheel/command'}))
