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

"""The training environment must never import rclpy.

This is the one test in the repo that guards an ARCHITECTURAL property
rather than a behaviour, so it is worth saying plainly why it exists.

v1's most expensive bug was `--fast`. Unlocking Gazebo's real-time factor
made sim time outrun wall-clock ROS delivery; cmd_vel arrived late and
intermittently; diff_drive_controller's 0.5 s watchdog repeatedly halted
the wheels; the stop-start pumping reared the chassis over backwards. Same
seed, same config: 531 of 533 episodes tipped and evaluation scored 0/10,
against 0/533 and 10/10 without the flag — and the flag was SLOWER.

Every link in that chain is a ROS transport link. An environment with no
ROS in it cannot have that class of bug, rather than avoiding it by
remembering not to pass a flag. M7_DESIGN 5.2 states the rule; this
enforces it.

The test is deliberately hostile: it does not merely check that the module
does not `import rclpy` textually, it removes rclpy from sys.modules and
poisons the import machinery so that ANY transitive import fails loudly.
"""
import builtins
import sys

import pytest

BANNED = ('rclpy', 'rosidl_runtime_py', 'rmw', 'ament_index_python')


@pytest.fixture
def no_ros(monkeypatch):
    """Make importing anything ROS raise, then hand control back."""
    for name in list(sys.modules):
        if name.split('.')[0] in BANNED or name.startswith('coco_rl'):
            monkeypatch.delitem(sys.modules, name, raising=False)

    real_import = builtins.__import__

    def guarded(name, *args, **kwargs):
        if name.split('.')[0] in BANNED:
            raise AssertionError(
                f'mujoco_env pulled in {name!r}. The training environment '
                f'must be pure Python + Gymnasium + MuJoCo — see '
                f'docs/M7_DESIGN.md 5.2 and the --fast story in RESULTS.md.')
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, '__import__', guarded)
    yield


def test_mujoco_env_imports_without_ros(no_ros):
    """The load-bearing assertion of this file."""
    import coco_rl.mujoco_env  # noqa: F401


def test_the_guard_itself_works(no_ros):
    """A guard that cannot fail proves nothing.

    If this stops raising, the fixture has broken and the test above is
    passing vacuously.
    """
    with pytest.raises(AssertionError):
        import rclpy  # noqa: F401


def test_rclpy_is_not_in_sys_modules_after_import(no_ros):
    import coco_rl.mujoco_env  # noqa: F401
    leaked = [n for n in sys.modules if n.split('.')[0] in BANNED]
    assert not leaked, f'ROS modules present after import: {leaked}'
