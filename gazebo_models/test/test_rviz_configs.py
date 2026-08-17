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

"""Invariants of the three RViz configs (C2-M1.6).

An RViz config is data, and the failure modes are silent by
construction: RViz does not error on a display whose QoS cannot match
its publisher, or whose fixed frame does not exist, or whose topic
nobody writes. It draws nothing and looks like a broken robot. Every
assertion below is one of those, and each has already cost a debugging
session somewhere in this repo's history.

What is deliberately NOT asserted: colours, alphas, line widths, camera
distance. Those are judgements made against a rendered window and
re-judging them here would only pin them in place. The record of how
they were measured is in the configs' own comments and in
docs/SESSION_LOG.md.
"""
import os

import pytest

import yaml

RVIZ_DIR = os.path.join(os.path.dirname(__file__), '..', 'rviz')

MISSION = 'mission.rviz'
DEBUG = 'mission_debug.rviz'
ROBOT_ONLY = 'coco_robot.rviz'


def load(name):
    """Parse one config, or fail the test with the parse error."""
    with open(os.path.join(RVIZ_DIR, name)) as f:
        return yaml.safe_load(f)


def displays(cfg):
    """Flatten the display tree to (path, dict), descending into groups."""
    out = []

    def walk(items, prefix=''):
        for d in items:
            name = d.get('Name', d.get('Class', '?'))
            if d.get('Class') == 'rviz_common/Group':
                walk(d.get('Displays', []), prefix + name + '/')
            else:
                out.append((prefix + name, d))

    walk(cfg['Visualization Manager']['Displays'])
    return out


def topic_of(d):
    t = d.get('Topic')
    if isinstance(t, dict):
        return t.get('Value')
    return t


# ── parse ────────────────────────────────────────────────────────────
@pytest.mark.parametrize('name', [MISSION, DEBUG, ROBOT_ONLY])
def test_config_is_valid_yaml_with_a_view(name):
    """RViz configs are YAML. A tab or a bad indent makes RViz open empty."""
    cfg = load(name)
    assert 'Visualization Manager' in cfg
    assert cfg['Visualization Manager']['Displays']
    assert cfg['Visualization Manager']['Views']['Current']['Class']


# ── fixed frame ──────────────────────────────────────────────────────
@pytest.mark.parametrize('name', [MISSION, DEBUG])
def test_mission_views_are_fixed_on_map(name):
    """Both mission views fix on map.

    With the fixed frame on the robot the map and both costmaps
    translate and rotate underneath a stationary robot, which is
    backwards from what a viewer needs and was the single biggest
    reason the pre-C2-M1 recordings did not read as navigation.
    """
    assert load(name)['Visualization Manager']['Global Options'][
        'Fixed Frame'] == 'map'


def test_coco_robot_config_is_left_alone():
    """coco_robot.rviz stays on base_footprint and is NOT a mission view.

    rsp.launch.py starts robot_state_publisher and nothing else, so
    base_footprint is the only frame that exists there. Pointing it at
    map would show an empty window and an error. It is listed as frozen
    in PROJECT_STATE.md.
    """
    assert load(ROBOT_ONLY)['Visualization Manager']['Global Options'][
        'Fixed Frame'] == 'base_footprint'


# ── QoS, the silent-failure class ────────────────────────────────────
BEST_EFFORT_TOPICS = {'/scan', '/camera/image_raw', '/particle_cloud'}
TRANSIENT_LOCAL_TOPICS = {'/map', '/global_costmap/costmap',
                          '/local_costmap/costmap', '/amcl_pose'}


@pytest.mark.parametrize('name', [MISSION, DEBUG])
def test_best_effort_publishers_have_best_effort_subscribers(name):
    """A RELIABLE subscriber never matches a BEST_EFFORT publisher.

    It does not error. The display sits empty and reads as a dead
    sensor. This is the same trap that made target_finder silently
    blind, and CLAUDE.md lists it.
    """
    for path, d in displays(load(name)):
        t = topic_of(d)
        if t in BEST_EFFORT_TOPICS:
            assert d['Topic']['Reliability Policy'] == 'Best Effort', (
                f'{name}: {path} subscribes {t} RELIABLE')


@pytest.mark.parametrize('name', [MISSION, DEBUG])
def test_latched_publishers_have_transient_local_subscribers(name):
    """Nav2 and map_server latch these; a Volatile subscriber that

    connects after the last publish shows nothing until the next one,
    and the costmaps only republish on change.
    """
    for path, d in displays(load(name)):
        t = topic_of(d)
        if t in TRANSIENT_LOCAL_TOPICS:
            assert d['Topic']['Durability Policy'] == 'Transient Local', (
                f'{name}: {path} subscribes {t} VOLATILE')


# ── topics that exist, and one that does not publish ─────────────────
@pytest.mark.parametrize('name', [MISSION, DEBUG])
def test_goal_display_does_not_use_goal_pose(name):
    """/goal_pose is advertised and never publishes.

    The sequencer drives Nav2 through the NavigateToPose ACTION, and
    /goal_pose is only ever written by RViz's own goal TOOL. A display
    on it sits dead for a whole run while the robot is visibly
    navigating. mission_hud republishes the end of the live global plan
    on /mission/goal instead. Measured on the first live mission.
    """
    for path, d in displays(load(name)):
        assert topic_of(d) != '/goal_pose', (
            f'{name}: {path} displays /goal_pose, which never publishes')


def test_particle_cloud_uses_the_nav2_plugin():
    """Nav2 publishes nav2_msgs/ParticleCloud, not geometry_msgs/PoseArray.

    rviz_default_plugins/PoseArray cannot subscribe to it at all. It
    does not error; it simply never matches, and an empty display looks
    like a localization failure.
    """
    for name in (MISSION, DEBUG):
        for path, d in displays(load(name)):
            if topic_of(d) == '/particle_cloud':
                assert d['Class'] == 'nav2_rviz_plugins/ParticleCloud', (
                    f'{name}: {path} uses {d["Class"]}')


# ── the C2-M1.6 split ────────────────────────────────────────────────
def test_debug_view_offers_every_topic_the_clean_view_does():
    """The split is about what is ENABLED, never about what is available.

    mission_debug.rviz is a superset: anything the clean view can show,
    the engineering view can show too.
    """
    clean = {topic_of(d) for _, d in displays(load(MISSION))} - {None}
    debug = {topic_of(d) for _, d in displays(load(DEBUG))} - {None}
    assert clean <= debug, f'only in mission.rviz: {sorted(clean - debug)}'


def test_clean_view_keeps_every_diagnostic_display_present():
    """Nothing was DELETED from the clean view to tidy it.

    TF, the particle cloud and the global costmap are all still in the
    tree with their topics and QoS intact; they are unticked. A viewer
    who wants them ticks the box, and no config edit is needed.
    """
    clean = dict(displays(load(MISSION)))
    for path in ('TF', 'Localization/Particle Cloud',
                 'Navigation/Global Costmap'):
        assert path in clean, f'{path} was removed from mission.rviz'
        assert clean[path].get('Enabled') is False, (
            f'{path} is enabled in the clean view')


def test_clean_view_enables_the_mission_hierarchy():
    """MAP + ROBOT + GLOBAL PATH + LOCAL PLAN + GOAL + TARGET, all on.

    This is the C2-M1.6 visual-quality bar restated as an assertion. If
    a later edit unticks one of these the clean view stops being able to
    answer the question it exists for.
    """
    clean = dict(displays(load(MISSION)))
    for path in ('RobotModel', 'Navigation/Map', 'Navigation/Global Plan',
                 'Navigation/Local Plan (DWB)', 'Navigation/Goal (from plan)',
                 'Navigation/Local Costmap', 'Perception/LaserScan',
                 'Perception/Perception Target'):
        assert path in clean, f'{path} is missing from mission.rviz'
        assert clean[path].get('Enabled') is True, f'{path} is disabled'


def test_debug_view_enables_its_diagnostics():
    """The engineering view really is everything-on.

    Its whole purpose is that an engineer does not have to hunt for a
    checkbox mid-incident.
    """
    dbg = dict(displays(load(DEBUG)))
    for path in ('TF', 'Localization/Particle Cloud',
                 'Navigation/Global Costmap', 'Navigation/Local Costmap',
                 'Perception/Camera'):
        assert path in dbg, f'{path} is missing from mission_debug.rviz'
        assert dbg[path].get('Enabled') is True, f'{path} is disabled'


def test_only_the_debug_view_docks_an_image_pane():
    """An Image display is not a 3D overlay; it takes its own dock pane.

    C2-M1.6 gave that width back to the navigation view. The camera is
    not removed from the system or from the engineering view.
    """
    clean_classes = [d['Class'] for _, d in displays(load(MISSION))]
    assert 'rviz_default_plugins/Image' not in clean_classes
    dbg_classes = [d['Class'] for _, d in displays(load(DEBUG))]
    assert 'rviz_default_plugins/Image' in dbg_classes


# ── framing ──────────────────────────────────────────────────────────
def test_clean_view_is_near_top_down_and_map_oriented():
    """Yaw 3*pi/2 puts +x screen-right and +y screen-up; pitch is steep.

    Not cosmetic. The arena is 12.15 m along x and 8.75 m along y in a
    window that is wider than it is tall, so laying the long axis down
    the short axis of the window is the worst of the four right-angle
    choices and forces the camera to back off. Both numbers were
    measured against rendered windows.
    """
    import math
    view = load(MISSION)['Visualization Manager']['Views']['Current']
    assert view['Class'] == 'rviz_default_plugins/Orbit'
    assert abs(view['Yaw'] - 3 * math.pi / 2) < 0.02, (
        'the clean view is no longer oriented +x right / +y up')
    assert view['Pitch'] > 1.2, 'the clean view is no longer near-top-down'
    assert view['Pitch'] < math.pi / 2, 'Orbit pitch must stay below pi/2'


def test_both_mission_views_focus_on_the_centre_of_the_map():
    """(3.956, -0.535), computed from maps/coco_world.yaml.

    origin (-2.119, -4.910) at 0.05 m/cell over 243 x 175 cells gives
    x -2.119..10.031 and y -4.910..3.840. The pre-C2-M1.5 value of
    (1.5, 0) was the centre of neither the map nor the traverse, which
    is why the frame ran out on one side before the other.
    """
    from PIL import Image
    yml = os.path.join(os.path.dirname(__file__), '..', 'maps',
                       'coco_world.yaml')
    with open(yml) as f:
        meta = yaml.safe_load(f)
    res = meta['resolution']
    ox, oy = meta['origin'][0], meta['origin'][1]
    pgm = os.path.join(os.path.dirname(yml), meta['image'])
    with Image.open(pgm) as im:
        w, h = im.size
    cx, cy = ox + w * res / 2.0, oy + h * res / 2.0

    for name in (MISSION, DEBUG):
        fp = load(name)['Visualization Manager']['Views']['Current'][
            'Focal Point']
        assert abs(fp['X'] - cx) < 0.05, f'{name}: focal X {fp["X"]} vs {cx}'
        assert abs(fp['Y'] - cy) < 0.05, f'{name}: focal Y {fp["Y"]} vs {cy}'


# ── the launch file offers both ──────────────────────────────────────
def test_every_config_the_launch_file_can_name_exists():
    """rviz_config:=<name> resolves to rviz/<name>.rviz at launch.

    A typo there fails at runtime with an RViz that opens on an empty
    default config and no error anyone reads.
    """
    for name in ('mission', 'mission_debug'):
        path = os.path.join(RVIZ_DIR, name + '.rviz')
        assert os.path.isfile(path), f'{path} does not exist'
