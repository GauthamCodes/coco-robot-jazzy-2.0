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

"""Check entrypoint process ownership without Docker, ROS or a simulator."""

import os
from pathlib import Path
import subprocess


ENTRYPOINT = Path(__file__).resolve().parents[2] / 'docker/entrypoint.sh'


def test_launch_pid_is_a_child_the_entrypoint_can_wait_for(tmp_path):
    """A launched process must remain a direct child of the waiting shell."""
    fake_ros = tmp_path / 'ros2'
    fake_ros.write_text('#!/bin/sh\nexit 7\n')
    fake_ros.chmod(0o755)
    text = ENTRYPOINT.read_text()
    start = text.index('launch_bg() {')
    function = text[start:text.index('\n}', start) + 2]
    script = (function + '\nlaunch_bg "$1" fixture\n'
              'wait "$LAUNCHED_PID"\n')
    result = subprocess.run(
        ['bash', '-c', script, 'entrypoint-test', str(tmp_path / 'launch.log')],
        env={**os.environ, 'PATH': str(tmp_path) + ':' + os.environ['PATH']},
        capture_output=True, text=True, timeout=5)
    assert result.returncode == 7, result.stdout + result.stderr
    assert 'not a child' not in result.stderr


def test_launch_callers_do_not_create_a_command_substitution_subshell():
    """Capturing stdout moves process ownership into a short-lived subshell."""
    text = ENTRYPOINT.read_text()
    assert '$(launch_bg' not in text
    assert 'SIM_PID=$LAUNCHED_PID' in text
    assert 'STACK_PID=$LAUNCHED_PID' in text


def test_entrypoint_shell_syntax():
    """The container entrypoint must parse without executing any commands."""
    result = subprocess.run(['bash', '-n', str(ENTRYPOINT)],
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
