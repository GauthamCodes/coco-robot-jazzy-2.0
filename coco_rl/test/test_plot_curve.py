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

"""Monitor-CSV parsing and smoothing behind the published learning curve.

docs/images/ppo_learning_curve.png is a headline artifact — it is the
evidence for the RL result. These cover the two pure functions that
produce it, so a change to either fails here rather than silently
redrawing the figure.
"""
import textwrap

from coco_rl.plot_curve import load, rolling_mean

import pytest

# Stable-Baselines3 Monitor format: a '#'-prefixed JSON header line, then
# a CSV with r (episode return), l (episode length), t (wall clock).
MONITOR_CSV = textwrap.dedent("""\
    #{"t_start": 1700000000.0, "env_id": null}
    r,l,t
    -11.5,100,1.0
    -9.25,150,2.0
    -13.0,50,3.0
    """)


def _write(tmp_path, text, name='run.monitor.csv'):
    path = tmp_path / name
    path.write_text(text)
    return str(path)


def test_load_parses_returns_and_accumulates_steps(tmp_path):
    steps, rewards = load([_write(tmp_path, MONITOR_CSV)])
    assert rewards == [-11.5, -9.25, -13.0]
    # x-axis is the cumulative timestep, not the episode index
    assert steps == [100, 250, 300]


def test_load_skips_the_json_header(tmp_path):
    """The header line is a comment; parsing it as data would crash."""
    _, rewards = load([_write(tmp_path, MONITOR_CSV)])
    assert len(rewards) == 3


def test_load_concatenates_multiple_runs_continuing_the_step_count(tmp_path):
    """--resume produces a second CSV; steps must continue, not restart."""
    a = _write(tmp_path, MONITOR_CSV, 'a.monitor.csv')
    b = _write(tmp_path, MONITOR_CSV, 'b.monitor.csv')
    steps, rewards = load([a, b])
    assert len(rewards) == 6
    assert steps == [100, 250, 300, 400, 550, 600]
    assert steps == sorted(steps), 'cumulative steps must be monotonic'


def test_load_of_a_header_only_file_yields_nothing(tmp_path):
    """An interrupted run can leave a CSV with no episodes.

    main() turns this into a clean error instead of writing an empty PNG,
    so the empty result has to survive load() rather than raising.
    """
    path = _write(tmp_path, '#{"t_start": 0}\nr,l,t\n')
    steps, rewards = load([path])
    assert steps == [] and rewards == []


def test_rolling_mean_is_causal_and_full_length():
    values = [1.0, 2.0, 3.0, 4.0]
    out = rolling_mean(values, window=2)
    assert len(out) == len(values)
    # Leading entries average over a short window rather than being
    # dropped or back-filled, so the curve starts at step 0.
    assert out == pytest.approx([1.0, 1.5, 2.5, 3.5])


def test_rolling_mean_window_of_one_is_the_identity():
    values = [3.0, -1.0, 7.5]
    assert rolling_mean(values, window=1) == pytest.approx(values)


def test_rolling_mean_window_larger_than_input_is_a_running_average():
    out = rolling_mean([2.0, 4.0], window=100)
    assert out == pytest.approx([2.0, 3.0])


def test_rolling_mean_smooths_a_spike_without_shifting_it():
    """A single outlier must be damped, and must not move the curve left."""
    flat = [0.0] * 5
    spiked = list(flat)
    spiked[2] = 10.0
    out = rolling_mean(spiked, window=3)
    assert out[1] == 0.0            # nothing leaks backwards
    assert 0.0 < out[2] < 10.0      # damped at the spike
    assert out[4] == pytest.approx(10.0 / 3)
