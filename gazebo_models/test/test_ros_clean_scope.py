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

"""C2-NAV.49: the sweep must not kill a simulator that is not ours.

MEASURED COST (C2-NAV.44, SESSION_LOG "Traps paid for"): `ros_clean.sh`'s
`g[z] sim` pattern matched an unrelated `eyantra_kepler_colony` simulator
from `~/ros2_ws` that started mid-run, and the teardown killed it. The
runner's refusal check only proves the machine was idle *at the start*, so
nothing catches a foreign simulator that appears later.

The narrowing is deliberately the SMALLEST one that keeps the tool able to
do its job. Scoping the sweep to "processes this experiment launched" was
rejected: `ros_clean.sh` exists to kill ORPHANS from PREVIOUS runs, which by
definition are not in the current process group, and the header records what
surviving orphans cost -- a stale /clock, TF buffers clearing, AMCL never
updating, and `bt_navigator` rejecting every goal four layers from the fault.
A session-scoped sweep would be structurally unable to kill any of that.

What distinguishes ours from theirs is the WORLD. Every coco simulator is
launched by `full_world_robo.launch.py`, which always passes a world file out
of `gazebo_models/worlds/` (`coco_world.world` or `coco_yard.world`), in both
`gui:=true` (`-r -v2 <world>`) and `gui:=false` (`-r -s -v2 <world>`) form.
So the pattern matches `gz sim` **and** that path, which still catches every
coco orphan from any overlay, worktree or past run.

BOTH DIRECTIONS ARE ASSERTED, and that is the point. CLAUDE.md: any check
whose success condition is "we saw nothing" must first prove it can see
something. A test that only asserts the foreign simulator survives would
also pass if the pattern matched nothing at all -- including a typo that
disarms the sweep completely, which is the more dangerous failure.

`--list` is used throughout: it reports and kills nothing.
"""
import os
import re
import subprocess
import sys
import time
import uuid

import pytest

SCRIPT = os.path.join(os.path.dirname(__file__), '..', 'scripts',
                      'ros_clean.sh')
# A coco simulator: gz sim, and a world out of the package's worlds/ dir.
COCO_WORLD = ('/opt/whatever/install/gazebo_models/share/gazebo_models'
              '/worlds/coco_world.world')
# Somebody else's simulator. This is the one C2-NAV.44 actually killed.
FOREIGN_WORLD = '/home/someone/ros2_ws/eyantra_kepler_colony/worlds/colony.world'


def _spawn(fake_cmdline):
    """A harmless process whose COMMAND LINE reads like a simulator.

    `pgrep -f` matches the full command line, so passing the text as an
    argument to a sleeping interpreter reproduces exactly what the sweep
    sees, without running a simulator.
    """
    return subprocess.Popen(
        [sys.executable, '-c', 'import time; time.sleep(120)', fake_cmdline],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _visible(pid, token, timeout=10.0):
    """Wait until pgrep can see the process, so absence never means 'not yet'."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        hit = subprocess.run(['pgrep', '-f', token], capture_output=True,
                             text=True)
        if str(pid) in hit.stdout.split():
            return True
        time.sleep(0.2)
    return False


def _would_kill():
    """The PIDs `ros_clean.sh --list` reports. It kills nothing."""
    out = subprocess.run(['bash', SCRIPT, '--list'], capture_output=True,
                         text=True, timeout=120).stdout
    return {m.group(1) for m in re.finditer(r'^\s+(\d+)\s', out, re.M)}


@pytest.fixture
def decoys():
    nonce = uuid.uuid4().hex[:8]
    ours = _spawn(f'gz sim -r -s -v2 {COCO_WORLD} --nonce {nonce}')
    theirs = _spawn(f'gz sim -r -s -v2 {FOREIGN_WORLD} --nonce {nonce}')
    try:
        assert _visible(ours.pid, nonce), 'coco decoy never became visible'
        assert _visible(theirs.pid, nonce), 'foreign decoy never became visible'
        yield ours, theirs
    finally:
        for proc in (ours, theirs):
            proc.kill()
            proc.wait()


def test_the_sweep_still_sees_a_coco_simulator(decoys):
    # The positive control. Without this, the test below would also pass on a
    # pattern that matches nothing -- i.e. on a sweep that has been disarmed.
    ours, _ = decoys
    assert str(ours.pid) in _would_kill(), (
        'ros_clean.sh no longer matches a coco simulator; the sweep is '
        'disarmed and orphaned gz processes will survive it')


def test_the_sweep_leaves_an_unrelated_simulator_alone(decoys):
    # C2-NAV.44: an eyantra_kepler_colony simulator was killed mid-run.
    _, theirs = decoys
    assert str(theirs.pid) not in _would_kill(), (
        'ros_clean.sh would kill a simulator that is not ours (C2-NAV.44 '
        'killed a real one); the gz pattern must name a gazebo_models world')


def test_no_pattern_matches_a_bare_gz_sim():
    # Structural, so the narrowing cannot be undone by accident. A pattern of
    # just `gz sim` is what killed somebody else's work.
    with open(SCRIPT) as f:
        body = f.read()
    patterns = re.findall(r"^\s*'([^']+)'\s*$", body, re.M)
    assert patterns, 'no patterns parsed out of ros_clean.sh'
    gz = [p for p in patterns if 'sim' in p and p.startswith('g[z]')]
    assert gz, 'ros_clean.sh no longer has a gz sim pattern at all'
    for pat in gz:
        assert 'gazebo_models/worlds' in pat, (
            f'gz pattern {pat!r} matches every simulator on the machine; it '
            'must name a gazebo_models world')
