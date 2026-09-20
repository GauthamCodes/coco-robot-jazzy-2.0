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
The platform's bring-up scripts, and one bash trap they both fell into.

Lives beside ``test_docker_context.py`` because that is already where
this repo statically checks things it cannot run: the Docker image is
never built here, and the native launcher takes minutes and a simulator.

The trap, measured on the native path
-------------------------------------
``grep -q`` exits the instant it matches. That closes the pipe, the
writer upstream dies of EPIPE -- a Python process exits **120** -- and
under ``set -o pipefail`` the pipeline's status becomes that 120. So::

    set -o pipefail
    ros2 topic info /diff_drive_controller/odom | grep -q 'Publisher count: [1-9]'

reports **failure exactly when the pattern was found**. Measured on a
live graph: exit 0 without pipefail, exit 120 with it, on the same
matching input.

The consequence was not subtle once seen: ``run_platform.sh --native``
never broke out of its readiness wait, burning all 120 iterations while
printing nothing, with a robot that had been publishing odometry for ten
minutes. ``docker/entrypoint.sh`` had the same line in the function that
sequences the simulator before the mission stack.

Dropping ``-q`` fixes it: grep reads the stream to the end, so the
writer never sees EPIPE.
"""

import os
import re
import subprocess

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))

#: The two bring-up scripts. Experiment harnesses are out of scope: they
#: are run by hand by someone who will notice a stall.
SCRIPTS = (
    os.path.join(REPO, 'scripts', 'run_platform.sh'),
    os.path.join(REPO, 'docker', 'entrypoint.sh'),
)


@pytest.mark.parametrize('path', SCRIPTS)
def test_the_script_exists(path):
    """A moved script must fail loudly, not silently stop being checked."""
    assert os.path.isfile(path), f'{path} is not a file'


@pytest.mark.parametrize('path', SCRIPTS)
def test_the_script_parses(path):
    """`bash -n` catches the class of typo that only fires on one branch."""
    result = subprocess.run(['bash', '-n', path], capture_output=True,
                            text=True)
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize('path', SCRIPTS)
def test_no_grep_dash_q_in_a_pipeline_under_pipefail(path):
    """
    The trap this module documents, asserted rather than remembered.

    Any `| grep -q` in a script that sets pipefail reports failure
    exactly when it matches. Use `| grep PATTERN >/dev/null` instead, so
    grep drains the stream and the writer never sees EPIPE.
    """
    with open(path, encoding='utf-8') as handle:
        body = handle.read()
    if 'pipefail' not in body:
        pytest.skip(f'{os.path.basename(path)} does not set pipefail')
    offenders = [line.strip() for line in body.splitlines()
                 if re.search(r'\|\s*grep\s+(-\w*q|\S*\s+-q)\b', line)]
    assert not offenders, (
        f'{os.path.basename(path)} pipes into `grep -q` while pipefail is '
        f'set; the pipeline then FAILS when the pattern MATCHES: '
        f'{offenders}')


def test_the_native_path_starts_from_a_clean_package_path():
    """
    A stray package on AMENT_PREFIX_PATH kills every gz launch.

    ros_gz_sim's GazeboRosPaths.get_paths() enumerates every package on
    that path; one half-installed entry -- an egg-link with no package
    marker -- makes it throw, and the launch dies naming a package this
    repo has nothing to do with. Measured on the development machine.
    """
    with open(SCRIPTS[0], encoding='utf-8') as handle:
        body = handle.read()
    assert 'unset AMENT_PREFIX_PATH' in body
    assert 'COCO_PRESERVE_PATH' in body, (
        'the sanitising must be opt-out, for a deliberately layered '
        'overlay')


def test_the_native_path_waits_for_localisation_not_just_health():
    """
    /healthz 200 means the components are up, not that AMCL has a pose.

    A mission started in that window aborts instantly with
    NAVIGATION_FAILED while bt_navigator logs "Initial robot pose is not
    available", which reads as a mission bug and is not one.
    """
    with open(SCRIPTS[0], encoding='utf-8') as handle:
        body = handle.read()
    assert 'tf2_echo map odom' in body
    assert 'LOCALISED' in body


def test_the_entrypoint_sequences_the_simulator_before_the_stack():
    """
    Order is load-bearing, and the wait is what enforces it.

    Starting the mission stack before a /clock exists leaves every
    use_sim_time node waiting on it, and Nav2's lifecycle then times out
    in a way that reads like a Nav2 bug.
    """
    with open(SCRIPTS[1], encoding='utf-8') as handle:
        body = handle.read()
    assert 'wait_for_topic' in body
    assert body.index('wait_for_topic /model/coco/odometry') < \
        body.index('mission.launch.py')
