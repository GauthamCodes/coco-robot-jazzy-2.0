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
``setup_env.sh`` in a clean shell: what it puts on the package path.

Lives beside ``test_platform_scripts.py``, which checks the bring-up
scripts that source it.

What went wrong, measured 2026-09-22
------------------------------------
From ``bash --noprofile --norc``, sourcing ROS and then this file, then::

    ros2 launch gazebo_models full_world_robo.launch.py traverse:=true gui:=true
    [ERROR] package 'turtlebot3_teleop' not found

Two separate things reached a shell that had asked for neither:

1. ``<ws>/install/setup.bash`` chained ``$HOME/ros2_ws/install`` -- an
   unrelated workspace that ``~/.bashrc`` had on the path when the overlay
   was BUILT. colcon freezes that underlay into the file. Fixed by sourcing
   the overlay's ``local_setup.bash``, which adds only its own packages.
2. ``<ws>/install`` itself listed ``turtlebot3_teleop`` and ``red_ball_nav``
   with ament index markers that were dangling symlinks. ros_gz_sim lists
   every package and resolves each, so the launch died. That is the
   developer's build tree, not this repo, so it is REPORTED, never edited:
   ``scripts/check_ament_path.py`` names each one when the file is sourced.

Every test runs the real file in ``bash --noprofile --norc`` with an
environment of ``HOME`` and ``PATH`` only, against a fake overlay in
``tmp_path`` -- nothing here depends on what the developer has built.
"""

import os
import shutil
import subprocess

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
SETUP_ENV = os.path.join(REPO, 'setup_env.sh')
CHECKER = os.path.join(REPO, 'scripts', 'check_ament_path.py')
BUILD_OVERLAY = os.path.join(REPO, 'scripts', 'build_overlay.sh')
MARKERS = os.path.join('share', 'ament_index', 'resource_index', 'packages')

pytestmark = pytest.mark.skipif(
    not os.path.isfile('/opt/ros/jazzy/setup.bash'),
    reason='setup_env.sh sources /opt/ros/jazzy and refuses without it')


def _prefix(root, name, marker=True, dangling=False):
    """Make an isolated-install prefix for package ``name`` under ``root``."""
    prefix = os.path.join(root, name)
    index = os.path.join(prefix, MARKERS)
    os.makedirs(index, exist_ok=True)
    os.makedirs(os.path.join(prefix, 'share', name), exist_ok=True)
    if dangling:
        os.symlink(os.path.join(root, 'moved_away', name),
                   os.path.join(index, name))
    elif marker:
        open(os.path.join(index, name), 'w').close()
    return prefix


def _overlay(root, packages, underlay=None):
    """
    A fake colcon install dir: ``local_setup.bash`` adds ``packages``.

    ``setup.bash`` does what colcon's does -- first sources the underlay
    that was on the path when it was built, then ``local_setup.bash``.
    """
    install = os.path.join(root, 'install')
    os.makedirs(install, exist_ok=True)
    lines = [f'export AMENT_PREFIX_PATH="{p}:$AMENT_PREFIX_PATH"'
             for p in packages]
    with open(os.path.join(install, 'local_setup.bash'), 'w') as handle:
        handle.write('\n'.join(lines) + '\n')
    with open(os.path.join(install, 'setup.bash'), 'w') as handle:
        if underlay:
            handle.write(
                f'export AMENT_PREFIX_PATH="{underlay}:$AMENT_PREFIX_PATH"\n')
        handle.write(f'source "{install}/local_setup.bash"\n')
    return root


def _source(setup_env, coco_ws, show='AMENT_PREFIX_PATH', inherited=None):
    """
    Source ``setup_env`` in a clean bash; return (stdout, stderr, rc).

    ``inherited`` is what the terminal the shell was started from had
    already exported -- ``--noprofile --norc`` does not clear it.
    """
    script = (f'export COCO_WS="{coco_ws}"\n'
              f'source "{setup_env}" || exit 9\n'
              f'printf "%s" "${show}"\n')
    env = {'HOME': os.environ.get('HOME', '/tmp'),
           'PATH': '/usr/local/bin:/usr/bin:/bin'}
    env.update(inherited or {})
    result = subprocess.run(['bash', '--noprofile', '--norc', '-c', script],
                            capture_output=True, text=True, env=env,
                            timeout=60)
    return result.stdout, result.stderr, result.returncode


def _exported_by_bashrc(unrelated_root):
    """What ~/.bashrc sourcing another workspace leaves exported."""
    pkg = os.path.join(unrelated_root, 'eyantra_pkg')
    return {
        'AMENT_PREFIX_PATH': f'{pkg}:/opt/ros/jazzy',
        'COLCON_PREFIX_PATH': unrelated_root,
        'CMAKE_PREFIX_PATH': pkg,
        'PYTHONPATH': f'{pkg}/lib/python3.12/site-packages',
        'LD_LIBRARY_PATH': f'{pkg}/lib',
        'GZ_SIM_SYSTEM_PLUGIN_PATH': f'{pkg}/lib',
        'PATH': f'{pkg}/bin:/usr/local/bin:/usr/bin:/bin',
    }


PATH_VARS = ('AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH',
             'PYTHONPATH', 'LD_LIBRARY_PATH', 'GZ_SIM_SYSTEM_PLUGIN_PATH',
             'PATH')


@pytest.mark.parametrize('var', PATH_VARS)
def test_a_workspace_the_terminal_exported_does_not_survive(tmp_path, var):
    """
    `bash --noprofile --norc` inherits exports; setup_env.sh must not.

    The measured case: ~/.bashrc sourced $HOME/ros2_ws/install, and a
    "clean" shell opened from that terminal still carried it on every
    path-like variable.
    """
    unrelated = str(tmp_path / 'ros2_ws' / 'install')
    coco = _prefix(str(tmp_path / 'ws' / 'install'), 'coco_fake')
    ws = _overlay(str(tmp_path / 'ws'), [coco])

    out, err, rc = _source(SETUP_ENV, ws, show=var,
                           inherited=_exported_by_bashrc(unrelated))
    assert rc == 0, err
    assert unrelated not in out, f'{var} kept the other workspace: {out}'


def test_the_system_path_survives_the_sanitising(tmp_path):
    """Only entries under a foreign prefix go -- never /usr/bin."""
    unrelated = str(tmp_path / 'ros2_ws' / 'install')
    coco = _prefix(str(tmp_path / 'ws' / 'install'), 'coco_fake')
    ws = _overlay(str(tmp_path / 'ws'), [coco])

    out, err, rc = _source(SETUP_ENV, ws, show='PATH',
                           inherited=_exported_by_bashrc(unrelated))
    assert rc == 0, err
    assert {'/usr/bin', '/bin'} <= set(out.split(':'))


def test_coco_preserve_path_keeps_a_deliberate_underlay(tmp_path):
    """The opt-out: layering COCO over another overlay on purpose."""
    unrelated = str(tmp_path / 'ros2_ws' / 'install')
    coco = _prefix(str(tmp_path / 'ws' / 'install'), 'coco_fake')
    ws = _overlay(str(tmp_path / 'ws'), [coco])
    inherited = _exported_by_bashrc(unrelated)
    inherited['COCO_PRESERVE_PATH'] = '1'

    out, err, rc = _source(SETUP_ENV, ws, inherited=inherited)
    assert rc == 0, err
    entries = out.split(':')
    assert os.path.join(unrelated, 'eyantra_pkg') in entries
    assert coco in entries


def test_sourcing_twice_is_harmless(tmp_path):
    """Re-sourcing strips the previous COCO entries and adds them again."""
    setup_env = _copy_repo_files(
        str(tmp_path / 'src_ws' / 'src' / 'coco-robot-ros2'))
    coco = _prefix(str(tmp_path / 'ws' / 'install'), 'coco_fake')
    ws = _overlay(str(tmp_path / 'ws'), [coco])
    script = (f'export COCO_WS="{ws}"\n'
              f'source "{setup_env}" && source "{setup_env}" || exit 9\n'
              'printf "%s" "$AMENT_PREFIX_PATH"\n')
    result = subprocess.run(
        ['bash', '--noprofile', '--norc', '-c', script],
        capture_output=True, text=True, timeout=60,
        env={'HOME': os.environ.get('HOME', '/tmp'),
             'PATH': '/usr/local/bin:/usr/bin:/bin'})
    assert result.returncode == 0, result.stderr
    assert result.stdout.split(':') == [coco, '/opt/ros/jazzy']


def test_the_overlay_is_sourced_without_its_frozen_underlays(tmp_path):
    """
    An underlay baked into install/setup.bash must not reach the shell.

    The measured case: ``$HOME/ros2_ws/install`` -- someone else's
    workspace -- arrived in ``bash --noprofile --norc`` this way.
    """
    coco = _prefix(str(tmp_path / 'ws' / 'install'), 'coco_fake')
    unrelated = _prefix(str(tmp_path / 'unrelated'), 'someone_elses_pkg')
    ws = _overlay(str(tmp_path / 'ws'), [coco], underlay=unrelated)

    out, err, rc = _source(SETUP_ENV, ws)
    assert rc == 0, err
    entries = out.split(':')
    assert coco in entries, 'the overlay itself was not sourced'
    assert unrelated not in entries, (
        'setup_env.sh sourced install/setup.bash, and with it an underlay '
        'frozen in at build time')
    assert '/opt/ros/jazzy' in entries


def test_a_clean_shell_gets_ros_and_the_overlay_and_nothing_else(tmp_path):
    """
    The whole package path, pinned: ROS, then the overlay. No third thing.

    Run from a copy of the repo files so the source workspace is empty,
    whatever the developer has built beside the real one.
    """
    setup_env = _copy_repo_files(
        str(tmp_path / 'src_ws' / 'src' / 'coco-robot-ros2'))
    coco = _prefix(str(tmp_path / 'ws' / 'install'), 'coco_fake')
    ws = _overlay(str(tmp_path / 'ws'), [coco])

    out, err, rc = _source(setup_env, ws)
    assert rc == 0, err
    assert out.split(':') == [coco, '/opt/ros/jazzy'], (
        f'a clean shell acquired more than ROS and the overlay: {out}')


def test_a_dangling_marker_is_named_when_the_env_is_sourced(tmp_path):
    """
    The package that will kill the gz launch is named up front.

    And sourcing still succeeds: the preflight warns, it never edits the
    path and never fails the shell.
    """
    coco = _prefix(str(tmp_path / 'ws' / 'install'), 'coco_fake')
    stale = _prefix(str(tmp_path / 'ws' / 'install'), 'turtlebot3_teleop',
                    dangling=True)
    ws = _overlay(str(tmp_path / 'ws'), [coco, stale])

    out, err, rc = _source(SETUP_ENV, ws)
    assert rc == 0, err
    assert stale in out.split(':'), 'the preflight must not edit the path'
    assert 'turtlebot3_teleop' in err
    assert 'cannot be resolved' in err
    assert 'coco_fake' not in err, 'a healthy package was reported'


def test_a_healthy_overlay_sources_silently(tmp_path):
    """No preflight noise when nothing is wrong."""
    coco = _prefix(str(tmp_path / 'ws' / 'install'), 'coco_fake')
    ws = _overlay(str(tmp_path / 'ws'), [coco])

    _out, err, rc = _source(SETUP_ENV, ws)
    assert rc == 0, err
    assert 'cannot be resolved' not in err


def _copy_repo_files(dest_repo):
    os.makedirs(os.path.join(dest_repo, 'scripts'))
    shutil.copy(SETUP_ENV, dest_repo)
    shutil.copy(CHECKER, os.path.join(dest_repo, 'scripts'))
    return os.path.join(dest_repo, 'setup_env.sh')


def test_an_isolated_overlay_still_finds_moveit_in_the_source_workspace(
        tmp_path):
    """
    COCO_WS pointed elsewhere must not silently drop MoveIt.

    The user-space MoveIt/rosbridge prefix lives in the SOURCE workspace
    (``<ws>/moveit_prefix``). Before this, an overlay selected with COCO_WS
    got no MoveIt unless someone remembered to symlink it in.
    """
    src_ws = tmp_path / 'src_ws'
    setup_env = _copy_repo_files(str(src_ws / 'src' / 'coco-robot-ros2'))
    moveit = src_ws / 'moveit_prefix' / 'root' / 'opt' / 'ros' / 'jazzy'
    moveit.mkdir(parents=True)
    coco = _prefix(str(tmp_path / 'overlay' / 'install'), 'coco_fake')
    overlay = _overlay(str(tmp_path / 'overlay'), [coco])

    out, err, rc = _source(setup_env, overlay)
    assert rc == 0, err
    assert str(moveit) in out.split(':')


def test_the_overlays_own_moveit_prefix_wins(tmp_path):
    """An overlay that carries its own moveit_prefix keeps using it."""
    src_ws = tmp_path / 'src_ws'
    setup_env = _copy_repo_files(str(src_ws / 'src' / 'coco-robot-ros2'))
    theirs = src_ws / 'moveit_prefix' / 'root' / 'opt' / 'ros' / 'jazzy'
    theirs.mkdir(parents=True)
    coco = _prefix(str(tmp_path / 'overlay' / 'install'), 'coco_fake')
    overlay = _overlay(str(tmp_path / 'overlay'), [coco])
    ours = tmp_path / 'overlay' / 'moveit_prefix' / 'root' / 'opt' / 'ros' \
        / 'jazzy'
    ours.mkdir(parents=True)

    out, err, rc = _source(setup_env, overlay)
    assert rc == 0, err
    entries = out.split(':')
    assert str(ours) in entries
    assert str(theirs) not in entries


def test_build_overlay_confines_discovery_to_this_repo():
    """
    `colcon build` from <ws> is what put turtlebot3 in the COCO install.

    The overlay builder must discover this repository only, and start from
    a clean package path so no login-shell workspace is frozen into it.
    """
    with open(BUILD_OVERLAY, encoding='utf-8') as handle:
        body = handle.read()
    assert os.access(BUILD_OVERLAY, os.X_OK), 'build_overlay.sh not +x'
    assert '--base-paths "$REPO"' in body
    assert '--install-base "$DEST/install"' in body
    assert 'unset AMENT_PREFIX_PATH' in body
    assert 'install/setup.bash' not in body


def test_the_checker_runs_without_ros():
    """The preflight must work before anything ROS is importable."""
    with open(CHECKER, encoding='utf-8') as handle:
        body = handle.read()
    for module in ('rclpy', 'ament_index_python', 'ros2pkg', 'catkin_pkg'):
        assert f'import {module}' not in body
        assert f'from {module}' not in body
    result = subprocess.run(['python3', CHECKER], capture_output=True,
                            text=True, env={'PATH': '/usr/bin:/bin'},
                            timeout=30)
    assert result.returncode == 0, result.stderr
