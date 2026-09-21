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
The browser assets, checked against the server they talk to.

These are static checks on text files, which is not much -- but it is the
class of bug that a Python test suite otherwise cannot see at all. A
``$("missingId")`` returns null and the failure is a TypeError in a
console nobody has open, several panels away from the cause.

The rule this file really enforces is the product boundary: the page
names intents, never topics. That is asserted structurally rather than
trusted, because the whole reason `coco.v1` exists is that the panel it
replaced could publish anything.
"""

import os
import re

from coco_web import mission_view, protocol, streams

import pytest

WEB = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   'web')


def _read(name):
    """Read one browser asset as text."""
    with open(os.path.join(WEB, name), encoding='utf-8') as handle:
        return handle.read()


@pytest.fixture(scope='module')
def app_js():
    """Return the client script."""
    return _read('app.js')


@pytest.fixture(scope='module')
def index_html():
    """Return the page."""
    return _read('index.html')


# ── the product boundary ───────────────────────────────────────────────

def test_the_client_names_no_ros_topic(app_js):
    """
    The page must not contain a topic name, in any frame it sends.

    This is the property that separates coco.v1 from the rosbridge panel
    it replaced, where the HTML's good manners were the only thing
    stopping a tab publishing to the wheels. Asserted structurally so it
    cannot rot.
    """
    for forbidden in ('/cmd_vel', '/diff_drive_controller', '/mission/state',
                      '/scan', '/amcl_pose', '/goal_pose', '/plan',
                      '/joint_trajectory'):
        assert forbidden not in app_js, (
            f'app.js mentions {forbidden}; the client names intents, not '
            f'topics')


def test_the_client_sends_only_frames_the_server_parses(app_js):
    """
    Every `type:` the page sends must be in the server's schema.

    A client sending a frame the server does not know gets an
    `unknown_type` error at runtime; this turns that into a test failure.
    """
    sent = set(re.findall(r'type:\s*"([a-z_]+)"', app_js))
    known = set(protocol.welcome({}, {}, {})['commands'])
    assert sent, 'no frames found in app.js; the regex has rotted'
    assert sent <= known, f'app.js sends unknown frames: {sorted(sent - known)}'


def test_the_client_only_subscribes_to_real_streams(app_js):
    """A stream name the server rejects would be a silently dead pane."""
    named = set(re.findall(r'push\("([a-z]+)"\)', app_js))
    assert named, 'no stream subscriptions found; the regex has rotted'
    assert named <= set(streams.STREAMS)


def test_the_client_declares_binary_support(app_js):
    """
    Binary frames are only sent to a client that declares it parses them.

    The page does parse them, so it must say so or its camera pane stays
    permanently blank with no error anywhere.
    """
    assert 'binary: true' in app_js


# ── the DOM the script reaches for ─────────────────────────────────────

def test_every_id_the_script_uses_exists_in_the_page(app_js, index_html):
    """
    A missing id is a TypeError in a console nobody has open.

    The symptom is one dead panel several layers from the typo, which is
    exactly the kind of thing a static check is good at and a human
    reading two files is not.
    """
    used = set(re.findall(r'\$\("([A-Za-z0-9_]+)"\)', app_js))
    present = set(re.findall(r'id="([A-Za-z0-9_]+)"', index_html))
    assert used, 'no element lookups found; the regex has rotted'
    assert used <= present, f'app.js reaches for missing ids: ' \
                            f'{sorted(used - present)}'


def test_the_page_loads_the_script_and_the_stylesheet(index_html):
    """The three assets setup.py installs are the three the page wants."""
    assert 'app.js' in index_html
    assert 'style.css' in index_html
    assert 'vendor/nipplejs.min.js' in index_html


# ── what P0.2 deleted, and must not come back ──────────────────────────

def test_the_browser_no_longer_owns_a_phase_list(app_js):
    """
    The executive owns the mission's states; the page used to guess.

    P0.1 hard-coded a fifteen-entry PHASES list here and computed a
    percentage from it. The server now sends the real step number from
    the executive's own chain, and the guess must not return.
    """
    assert 'const PHASES' not in app_js
    for state in ('NAVIGATE_TO_RAMP', 'VERIFY_PLACEMENT', 'ALIGN_FOR_CLIMB'):
        assert state not in app_js, (
            f'app.js names the executive state {state}; that state '
            f'machine belongs to the executive, and a copy here drifts')


def test_the_page_does_not_invent_a_percentage(app_js):
    """
    There is no basis for one, so there must be no bar claiming one.

    mission_executive sends its Nav2 goal with no feedback_callback, so
    progress within a leg is not published anywhere. The bar advances by
    whole steps and says which step it is on.
    """
    assert '% complete' not in app_js
    assert 'PHASES.length' not in app_js
    assert 'Step ${mission.step} of ${mission.steps}' in app_js


def test_the_page_labels_depth_as_visualisation_not_navigation(index_html):
    """
    DEPTH VISUALISATION != DEPTH NAVIGATION FUSION, said on the page.

    C2-NAV.43 left depth fusion a candidate that is OFF. A depth pane
    with no label invites the conclusion that the robot is using it.
    """
    lowered = index_html.lower()
    assert 'visualisation' in lowered or 'visualization' in lowered
    assert 'costmap' in lowered


# ── the operator vocabulary ────────────────────────────────────────────

def test_play_mode_speaks_english_not_ros(index_html):
    """
    The words a first-time user meets should not be ROS words.

    Engineering mode still shows the real state names; Play mode must
    not be where someone learns what VERIFY_PLACEMENT means.
    """
    play = index_html.split('eng-only')[0]
    for jargon in ('rclpy', 'topic', 'Nav2', 'AMCL', 'costmap', 'TwistStamped'):
        assert jargon not in play, f'Play mode says {jargon!r}'


def test_the_server_supplies_operator_wording_for_every_state():
    """
    The page shows `words` from the server, so every state needs one.

    A state with no wording would fall through to its ROS name in Play
    mode, which is the thing this is meant to avoid.
    """
    for state in mission_view.STATES:
        assert mission_view.WORDS.get(state)


def _platform_launch_entities():
    """Build platform.launch.py's real description and return its entities."""
    import importlib.util
    path = os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), 'launch', 'platform.launch.py')
    spec = importlib.util.spec_from_file_location('platform_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description().entities


def test_platform_launch_shows_the_depth_image_but_never_fuses_it():
    """
    The depth IMAGE is on by default for display; depth FUSION is absent.

    P0.2's first pass defaulted depth_topic to empty, treating the picture
    as if it were the costmap input, so the depth pane could never show
    anything. Fusion is nav.launch.py depth_cloud:=, a different package:
    this launch must include neither that file nor depth_cloud.launch.py,
    and start no node but the platform and the MJPEG server.
    """
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch_ros.actions import Node

    entities = _platform_launch_entities()
    defaults = {e.name: ''.join(getattr(p, 'text', '')
                                for p in e.default_value)
                for e in entities if isinstance(e, DeclareLaunchArgument)}
    assert defaults['depth_topic'] == '/camera/depth/image_raw'
    assert defaults['expected_components'] == 'lidar'

    for entity in entities:
        if isinstance(entity, IncludeLaunchDescription):
            location = getattr(entity.launch_description_source,
                               '_LaunchDescriptionSource__location', None)
            name = ''.join(getattr(p, 'text', '') for p in location or [])
            assert 'depth_cloud' not in name and 'nav.launch' not in name
        if isinstance(entity, Node):
            assert entity.node_package in ('coco_web', 'web_video_server')
