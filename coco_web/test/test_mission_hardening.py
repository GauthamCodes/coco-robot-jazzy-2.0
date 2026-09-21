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

"""Malformed status numbers must not crash or fabricate mission progress."""

import json

from coco_web import mission_view as mv

import pytest


@pytest.mark.parametrize('value', ['NaN', 'inf', '-inf', '-1', '1e999'])
@pytest.mark.parametrize('key', ['elapsed', 'timeout', 'attempt', 'retries'])
def test_invalid_status_number_is_absent(key, value):
    """Keep genuine state/result/reason while rejecting invalid measurements."""
    fields = {'state': 'ABORT', 'reason': 'RETURN_FAILED', 'result': 'failed',
              key: value}
    result = mv.normalise(fields, colour='blue', now=100)
    assert result[key] is None
    assert result['state'] == 'ABORT'
    assert result['phase'] == 'FAILED'
    assert result['colour'] == 'blue'
    assert result['reason'] == 'RETURN_FAILED'
    assert result['result'] == 'failed'
    assert result['changed_at'] is None
    json.dumps(result, allow_nan=False)


def test_fractional_retry_count_is_not_invented():
    """Truncating an invalid count would report an event that never happened."""
    result = mv.normalise({'state': 'SEARCH_TARGET', 'attempt': '1.5'})
    assert result['attempt'] is None


def test_transition_timestamp_uses_observed_elapsed_only():
    """Preserve a derivable timestamp without manufacturing overall progress."""
    result = mv.normalise({'state': 'GRASP', 'elapsed': '2.5'},
                          colour='green', now=100.0)
    assert result['changed_at'] == 97.5
    assert result['phase'] == 'GRASPING'
    assert 'progress' not in result
    assert 'eta' not in result
