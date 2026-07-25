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

"""Tests for pure helpers in ramp_env (no simulator needed)."""
import pytest

np = pytest.importorskip('numpy')
pytest.importorskip('rclpy')
pytest.importorskip('gymnasium')

from coco_rl import ramp_env  # noqa: E402


def test_sample_start_pose_within_ranges():
    rng = np.random.default_rng(0)
    for _ in range(200):
        x, y, z, yaw = ramp_env.sample_start_pose(rng)
        assert x == ramp_env.START_POSE[0]
        assert z == ramp_env.START_POSE[2]
        assert abs(y - ramp_env.START_POSE[1]) <= ramp_env.RAND_Y
        assert abs(yaw) <= ramp_env.RAND_YAW


def test_sample_start_pose_varies():
    rng = np.random.default_rng(1)
    ys = {round(ramp_env.sample_start_pose(rng)[1], 4) for _ in range(20)}
    assert len(ys) > 10  # actually random, not constant


# ── quat_to_rp: attitude drives the tip-over termination, so a sign or
# axis error here would silently mislabel episodes ──────────────────────

def _q_from_rpy(roll, pitch, yaw=0.0):
    """Build a quaternion (x, y, z, w) from roll/pitch/yaw."""
    import math
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def test_quat_to_rp_identity_is_level():
    roll, pitch = ramp_env.quat_to_rp(0.0, 0.0, 0.0, 1.0)
    assert roll == pytest.approx(0.0, abs=1e-9)
    assert pitch == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize('roll,pitch', [
    (0.3, 0.0), (-0.3, 0.0), (0.0, 0.4), (0.0, -0.4), (0.25, -0.35),
])
def test_quat_to_rp_round_trips(roll, pitch):
    got_r, got_p = ramp_env.quat_to_rp(*_q_from_rpy(roll, pitch))
    assert got_r == pytest.approx(roll, abs=1e-9)
    assert got_p == pytest.approx(pitch, abs=1e-9)


def test_quat_to_rp_is_yaw_invariant():
    """Driving in a circle on flat ground must not read as tilt."""
    import math
    for yaw in (0.0, 1.0, math.pi / 2, math.pi, -2.0):
        roll, pitch = ramp_env.quat_to_rp(*_q_from_rpy(0.0, 0.0, yaw))
        assert roll == pytest.approx(0.0, abs=1e-9)
        assert pitch == pytest.approx(0.0, abs=1e-9)


def test_quat_to_rp_pitch_is_clamped_at_the_singularity():
    """asin's argument can exceed 1 through rounding; it must not raise."""
    import math
    roll, pitch = ramp_env.quat_to_rp(*_q_from_rpy(0.0, math.pi / 2))
    assert pitch == pytest.approx(math.pi / 2, abs=1e-6)


def test_ramp_pitch_exceeds_the_tip_threshold_only_when_steep():
    """A nose-up attitude on the ramp must not read as a tip-over."""
    from coco_rl.reward import is_tipped
    _, gentle = ramp_env.quat_to_rp(*_q_from_rpy(0.0, 0.25))
    _, steep = ramp_env.quat_to_rp(*_q_from_rpy(0.0, 0.9))
    assert not is_tipped(0.0, gentle)
    assert is_tipped(0.0, steep)


# ── gz_service retry (the bug that killed a 180k-step curriculum) ───────────
# Each episode reset shells out to `gz service`. Under --fast the round trip
# occasionally overruns the timeout while the simulator is perfectly healthy,
# and reset() raises on a false return — so a single transient miss used to
# kill a multi-hour run. These pin the retry behaviour.

class _FakeRun:
    """Stand-in for subprocess.run returning a scripted sequence."""

    def __init__(self, results):
        self.results = list(results)
        self.calls = 0

    def __call__(self, *args, **kwargs):
        self.calls += 1
        r = self.results.pop(0)
        if isinstance(r, Exception):
            raise r
        return type('P', (), {'stdout': r, 'stderr': ''})()


def test_gz_service_retries_until_true(monkeypatch):
    fake = _FakeRun(['data: false\n', 'data: false\n', 'data: true\n'])
    monkeypatch.setattr(ramp_env.subprocess, 'run', fake)
    monkeypatch.setattr(ramp_env.time, 'sleep', lambda _s: None)
    assert ramp_env.gz_service('/s', 'q', 'r', 'req', attempts=5) is True
    assert fake.calls == 3      # stopped as soon as it succeeded


def test_gz_service_retries_a_timeout(monkeypatch):
    import subprocess
    fake = _FakeRun([subprocess.TimeoutExpired('gz', 1), 'data: true\n'])
    monkeypatch.setattr(ramp_env.subprocess, 'run', fake)
    monkeypatch.setattr(ramp_env.time, 'sleep', lambda _s: None)
    assert ramp_env.gz_service('/s', 'q', 'r', 'req', attempts=3) is True
    assert fake.calls == 2


def test_gz_service_gives_up_after_attempts(monkeypatch):
    fake = _FakeRun(['data: false\n'] * 4)
    monkeypatch.setattr(ramp_env.subprocess, 'run', fake)
    monkeypatch.setattr(ramp_env.time, 'sleep', lambda _s: None)
    assert ramp_env.gz_service('/s', 'q', 'r', 'req', attempts=4) is False
    assert fake.calls == 4      # no silent success, and no extra calls


def test_gz_service_defaults_to_a_single_attempt(monkeypatch):
    fake = _FakeRun(['data: false\n'])
    monkeypatch.setattr(ramp_env.subprocess, 'run', fake)
    assert ramp_env.gz_service('/s', 'q', 'r', 'req') is False
    assert fake.calls == 1      # callers opt in; no surprise latency


def test_gz_service_does_not_retry_a_missing_binary(monkeypatch):
    fake = _FakeRun([FileNotFoundError('gz')] * 3)
    monkeypatch.setattr(ramp_env.subprocess, 'run', fake)
    monkeypatch.setattr(ramp_env.time, 'sleep', lambda _s: None)
    with pytest.raises(RuntimeError, match='not found on PATH'):
        ramp_env.gz_service('/s', 'q', 'r', 'req', attempts=3)
    assert fake.calls == 1      # a missing gz will never appear mid-run
