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
The COCO simulation needs no TurtleBot package -- and why it looked as if it did.

The symptom, measured 2026-09-22 (and by C2-NAV.48 and .49 before it)::

    ros2 launch gazebo_models full_world_robo.launch.py traverse:=true gui:=true
    [ERROR] [launch]: package 'turtlebot3_teleop' not found

No file in this repository names that package. ``full_world_robo.launch.py``
includes ros_gz_sim's ``gz_sim.launch.py``, whose ``launch_gz`` begins with
``GazeboRosPaths.get_paths()``: it lists EVERY package on
``AMENT_PREFIX_PATH`` and resolves each one. The listing accepts an ament
index marker that is a dangling symlink; the lookup does not. The
developer's ``<ws>/install`` put two such markers on the path --
``turtlebot3_teleop`` and ``red_ball_nav``, left pointing into the
workspace's pre-rename directory -- and the launch died on the first.

So this module pins two things. The repository never grows a TurtleBot
dependency (static, every package). And the mechanism, run against the
real ros_gz_sim, with ``scripts/check_ament_path.py`` flagging exactly what
it dies on -- including the correction that a prefix with NO marker is
harmless, which the repo's own notes used to get wrong.
"""

import ast
import glob
import importlib.util
import os
import re
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
import pytest
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
MARKERS = os.path.join('share', 'ament_index', 'resource_index', 'packages')
FORBIDDEN = re.compile(r'turtlebot', re.IGNORECASE)

#: Every package in the repository, found rather than listed, so a new one
#: is checked without anyone remembering to add it here.
PACKAGES = sorted(os.path.dirname(p) for p in glob.glob(
    os.path.join(REPO, '*', 'package.xml')))


def _files(pattern):
    out = []
    for pkg in PACKAGES:
        for path in glob.glob(os.path.join(pkg, '**', pattern),
                              recursive=True):
            if f'{os.sep}test{os.sep}' not in path:
                out.append(path)
    return sorted(out)


def _declared(pkg_dir):
    root = ET.parse(os.path.join(pkg_dir, 'package.xml')).getroot()
    deps = {e.text.strip() for e in root if e.tag.endswith('depend') and e.text}
    return root.findtext('name'), deps


def _strings(path):
    """Every string constant in a Python file -- comments excluded."""
    with open(path, encoding='utf-8') as handle:
        tree = ast.parse(handle.read(), filename=path)
    return [n.value for n in ast.walk(tree)
            if isinstance(n, ast.Constant) and isinstance(n.value, str)]


_LOOKUP = re.compile(
    r"(?:get_package_share_directory|FindPackageShare|get_package_prefix)"
    r"\(\s*['\"]([A-Za-z0-9_]+)['\"]|package\s*=\s*['\"]([A-Za-z0-9_]+)['\"]")


def test_the_repository_has_packages():
    """A moved tree must fail loudly, not silently check nothing."""
    names = {os.path.basename(p) for p in PACKAGES}
    assert {'gazebo_models', 'coco_mission', 'coco_web'} <= names


@pytest.mark.parametrize('pkg_dir', PACKAGES, ids=os.path.basename)
def test_no_package_declares_a_turtlebot_dependency(pkg_dir):
    """The dependency graph COCO -> TurtleBot has no edge. Keep it so."""
    name, deps = _declared(pkg_dir)
    assert not [d for d in deps if FORBIDDEN.search(d)], name


def test_no_launch_file_names_a_turtlebot_package():
    """Not as an include, a node, a lookup or an argument default."""
    offenders = []
    for path in _files('*.launch.py'):
        offenders += [(os.path.relpath(path, REPO), s)
                      for s in _strings(path) if FORBIDDEN.search(s)]
    assert not offenders


def test_no_build_file_names_a_turtlebot_package():
    """setup.py and CMakeLists.txt, comments excluded."""
    offenders = []
    for path in _files('setup.py'):
        offenders += [path for s in _strings(path) if FORBIDDEN.search(s)]
    for path in _files('CMakeLists.txt'):
        with open(path, encoding='utf-8') as handle:
            code = [ln.split('#', 1)[0] for ln in handle]
        offenders += [path for ln in code if FORBIDDEN.search(ln)]
    assert not offenders


def _walk(node):
    if isinstance(node, dict):
        for key, value in node.items():
            yield str(key)
            yield from _walk(value)
    elif isinstance(node, list):
        for value in node:
            yield from _walk(value)
    elif isinstance(node, str):
        yield node


def test_no_config_value_names_a_turtlebot_package():
    """
    YAML keys and values, comments excluded.

    nav2_params.yaml's header comment says it began as turtlebot3's
    burger.yaml; that is provenance, not a dependency, and stays.
    """
    offenders = []
    for path in _files('*.yaml'):
        with open(path, encoding='utf-8') as handle:
            for doc in yaml.safe_load_all(handle):
                offenders += [(path, s) for s in _walk(doc)
                              if FORBIDDEN.search(s)]
    assert not offenders


@pytest.mark.parametrize('pkg_dir', PACKAGES, ids=os.path.basename)
def test_every_package_a_launch_file_looks_up_is_declared(pkg_dir):
    """
    A launch file may only resolve packages its package.xml declares.

    An undeclared lookup is how an optional tool quietly becomes a runtime
    requirement: nothing installs it, and the launch fails on a machine
    that lacks it with the same "package not found" as the turtlebot case.
    """
    name, deps = _declared(pkg_dir)
    used = set()
    for path in glob.glob(os.path.join(pkg_dir, '**', '*.launch.py'),
                          recursive=True):
        if f'{os.sep}test{os.sep}' in path:
            continue
        with open(path, encoding='utf-8') as handle:
            for match in _LOOKUP.finditer(handle.read()):
                used.add(match.group(1) or match.group(2))
    assert not sorted(used - deps - {name})


# --- the mechanism, against the real ros_gz_sim ------------------------


def _load_gz_sim():
    path = os.path.join(get_package_share_directory('ros_gz_sim'),
                        'launch', 'gz_sim.launch.py')
    spec = importlib.util.spec_from_file_location('coco_gz_sim_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_checker():
    path = os.path.join(REPO, 'scripts', 'check_ament_path.py')
    spec = importlib.util.spec_from_file_location('check_ament_path', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _stale_prefix(root, name, marker):
    """A prefix for ``name``: marker is 'dangling', 'none' or 'file'."""
    prefix = os.path.join(str(root), name)
    index = os.path.join(prefix, MARKERS)
    os.makedirs(index)
    os.makedirs(os.path.join(prefix, 'share', name))
    if marker == 'dangling':
        os.symlink(os.path.join(str(root), 'renamed_away', name),
                   os.path.join(index, name))
    elif marker == 'file':
        open(os.path.join(index, name), 'w').close()
    return prefix


@pytest.fixture
def ros_prefix():
    """The prefix ros_gz_sim itself is installed in, from the real index."""
    return get_package_prefix('ros_gz_sim')


@pytest.fixture
def gz_sim():
    return _load_gz_sim()


def test_one_dangling_marker_kills_the_gz_launch(tmp_path, monkeypatch,
                                                 ros_prefix, gz_sim):
    """The measured failure, reproduced: the package name and the class."""
    stale = _stale_prefix(tmp_path, 'turtlebot3_teleop', 'dangling')
    path = os.pathsep.join([stale, ros_prefix])
    monkeypatch.setenv('AMENT_PREFIX_PATH', path)

    with pytest.raises(PackageNotFoundError, match='turtlebot3_teleop'):
        gz_sim.GazeboRosPaths.get_paths()
    assert [b[0] for b in _load_checker().find_unresolvable(path)] == \
        ['turtlebot3_teleop']


def test_a_prefix_with_no_marker_is_harmless(tmp_path, monkeypatch,
                                             ros_prefix, gz_sim):
    """
    A half-built package with NO marker is never listed, so never fatal.

    The repo's notes used to blame "an egg-link with no package marker".
    Measured here: turtlebot3_node and turtlebot3_example sat in the same
    install with no marker and were never the problem.
    """
    stale = _stale_prefix(tmp_path, 'turtlebot3_node', 'none')
    path = os.pathsep.join([stale, ros_prefix])
    monkeypatch.setenv('AMENT_PREFIX_PATH', path)

    gz_sim.GazeboRosPaths.get_paths()
    assert _load_checker().find_unresolvable(path) == []


def test_a_dangling_marker_shadowed_by_a_good_one_is_harmless(
        tmp_path, monkeypatch, ros_prefix, gz_sim):
    """Resolution searches every prefix; the checker must agree."""
    stale = _stale_prefix(tmp_path / 'a', 'coco_fake', 'dangling')
    good = _stale_prefix(tmp_path / 'b', 'coco_fake', 'file')
    path = os.pathsep.join([stale, good, ros_prefix])
    monkeypatch.setenv('AMENT_PREFIX_PATH', path)

    gz_sim.GazeboRosPaths.get_paths()
    assert _load_checker().find_unresolvable(path) == []
