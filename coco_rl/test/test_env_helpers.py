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
