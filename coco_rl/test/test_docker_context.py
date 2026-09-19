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
What the Docker build context must contain, and must not.

Pure file inspection -- no Docker required, which matters because the
machine this was written on has none, so the image has never been built.

The rule worth knowing before editing .dockerignore: Docker does NOT use
shell-glob semantics. It uses Go's ``filepath.Match``, where ``*`` matches
any run of characters EXCEPT the separator, so a bare ``*.zip`` excludes
``ppo_coco_ramp.zip`` at the context root and leaves
``coco_rl/policies/phase5_24deg_s0p0.zip`` alone. Python's ``fnmatch``
does the opposite -- its ``*`` happily crosses ``/`` -- so a test written
with fnmatch reports exclusions that Docker would never make. That is why
``_excluded`` below compiles the pattern itself rather than reaching for
the obvious standard-library call.

The shipped policy is the thing most worth pinning. mission.launch.py
resolves ``coco_rl/policies/phase5_24deg_s0p0.zip`` from the ament index
as the default ``policy``, and ramp_driver refuses /ramp/climb without
one -- so an exclusion that reached it would produce an image that builds
cleanly and then fails every mission at CLIMB.
"""

import os
import pathlib
import re

import pytest

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[1]
DOCKERIGNORE = REPO / '.dockerignore'
POLICY = pathlib.Path('coco_rl/policies/phase5_24deg_s0p0.zip')


def _patterns():
    """Non-comment, non-empty lines of .dockerignore."""
    if not DOCKERIGNORE.exists():
        pytest.skip('.dockerignore not present in this tree')
    return [line.strip() for line in
            DOCKERIGNORE.read_text().splitlines()
            if line.strip() and not line.strip().startswith('#')]


def _to_regex(pattern):
    """
    Compile one .dockerignore pattern the way Docker does.

    ``**`` crosses directories, ``*`` and ``?`` do not. A leading slash is
    stripped: every pattern is already relative to the context root.
    """
    text = pattern.lstrip('/')
    out = ''
    index = 0
    while index < len(text):
        if text[index:index + 2] == '**':
            out += '.*'
            index += 2
        elif text[index] == '*':
            out += '[^/]*'
            index += 1
        elif text[index] == '?':
            out += '[^/]'
            index += 1
        else:
            out += re.escape(text[index])
            index += 1
    return re.compile(out)


def _excluded(path, patterns):
    """Whether Docker would exclude `path` from the build context."""
    text = str(path)
    for pattern in patterns:
        if pattern.endswith('/'):
            # A directory pattern excludes the directory and everything
            # under it, at the depth the pattern names.
            folder = pattern.rstrip('/').lstrip('/')
            if text == folder or text.startswith(folder + '/'):
                return True
            continue
        compiled = _to_regex(pattern)
        if compiled.fullmatch(text):
            return True
        # A pattern that matches a leading directory excludes its contents.
        parts = text.split('/')
        for cut in range(1, len(parts)):
            if compiled.fullmatch('/'.join(parts[:cut])):
                return True
    return False


def _instructions(path):
    """A Dockerfile/compose file with comment lines removed.

    Comments are stripped before path checks so that a comment which
    NAMES the anti-pattern -- as the Dockerfile's does, to explain why it
    is avoided -- does not fail the test that enforces it.
    """
    return '\n'.join(
        line for line in path.read_text().splitlines()
        if not line.strip().startswith('#'))


# ── the matcher itself ─────────────────────────────────────────────────

def test_star_does_not_cross_a_directory_separator():
    """
    Pins the semantics the rest of this file depends on.

    Without this, a future simplification to fnmatch would make every
    exclusion test below silently wrong in the same direction.
    """
    assert not _excluded(POLICY, ['*.zip'])
    assert _excluded(pathlib.Path('ppo_coco_ramp.zip'), ['*.zip'])
    assert _excluded(POLICY, ['**/*.zip'])


# ── what must reach the image ──────────────────────────────────────────

def test_the_policy_file_is_actually_there():
    """Guards the guard: a missing policy would make the rest vacuous."""
    assert (REPO / POLICY).is_file(), (
        f'{POLICY} is missing; the ramp climb has no policy to load')


def test_the_shipped_policy_is_not_excluded_from_the_image():
    """The climb's policy must reach the container."""
    assert not _excluded(POLICY, _patterns()), (
        f'{POLICY} would be excluded from the Docker build context. '
        f'ramp_driver refuses /ramp/climb without a policy, so the image '
        f'would build and then fail every mission at CLIMB.')


@pytest.mark.parametrize('needed', [
    pathlib.Path('coco_web/web/index.html'),
    pathlib.Path('coco_web/web/app.js'),
    pathlib.Path('coco_web/web/vendor/nipplejs.min.js'),
    pathlib.Path('gazebo_models/worlds/coco_world.world'),
    pathlib.Path('gazebo_models/config/nav2_params.yaml'),
    pathlib.Path('docker/entrypoint.sh'),
])
def test_runtime_assets_are_not_excluded(needed):
    """The browser assets, the world and the nav config all ship."""
    assert not _excluded(needed, _patterns()), f'{needed} would be excluded'


# ── what must not ──────────────────────────────────────────────────────

@pytest.mark.parametrize('checkpoint', [
    pathlib.Path('ppo_coco_ramp.zip'),
    pathlib.Path('ppo_coco_ramp.monitor.csv'),
])
def test_training_artifacts_are_excluded(checkpoint):
    """RL training output does not belong in the image."""
    assert _excluded(checkpoint, _patterns()), (
        f'{checkpoint} would be copied into the image')


def test_worktrees_are_excluded():
    """
    Worktree checkouts must not multiply the build context.

    .claude/worktrees/ holds full checkouts of other branches. Copying
    them in would add one whole repo per open worktree, and colcon would
    then find and try to build every one of them.
    """
    patterns = _patterns()
    assert _excluded(pathlib.Path('.claude/worktrees/x/coco_rl/setup.py'),
                     patterns)
    assert _excluded(pathlib.Path('.codex/worktrees/y/setup.py'), patterns)


def test_the_build_tree_is_excluded():
    """A host build/ or install/ copied in would be built over."""
    patterns = _patterns()
    for path in ('build/coco_rl/x.py', 'install/coco_rl/x.py', 'log/x.log'):
        assert _excluded(pathlib.Path(path), patterns), path


# ── the Dockerfile itself ──────────────────────────────────────────────

def test_dockerfile_builds_every_package_the_mission_needs():
    """
    The image must contain the mission, not just the simulator.

    This was really broken: the first Dockerfile's colcon line selected
    gazebo_models, custom_teleop, coco_config, coco_moveit_config,
    coco_web and coco_rl -- six of the nine. coco_mission,
    coco_perception and coco_sim were absent, so the mission executive
    and the target finder were not in the "reproducible" image at all and
    `docker compose up` could not have run a fetch.
    """
    dockerfile = REPO / 'Dockerfile'
    if not dockerfile.exists():
        pytest.skip('Dockerfile not present in this tree')
    text = dockerfile.read_text()
    required = [
        'coco_config', 'coco_mission', 'coco_moveit_config',
        'coco_perception', 'coco_rl', 'coco_sim', 'coco_web',
        'custom_teleop', 'gazebo_models',
    ]
    missing = [name for name in required if name not in text]
    assert not missing, f'Dockerfile never builds: {", ".join(missing)}'


def test_the_image_uses_container_paths_only():
    """
    No developer workspace path may be baked into the image.

    The host this was developed on has the workspace at
    `~/ros2_ws(personal)` -- parentheses and all -- which is exactly the
    kind of path that breaks the build for everyone else. Checked against
    instructions only; a comment may name it to explain the rule.
    """
    for name in ('Dockerfile', 'docker-compose.yml'):
        path = REPO / name
        if not path.exists():
            continue
        text = _instructions(path)
        assert 'ros2_ws' not in text, f'{name} references a host workspace'
        assert os.path.expanduser('~') not in text, (
            f'{name} references a developer home directory')
