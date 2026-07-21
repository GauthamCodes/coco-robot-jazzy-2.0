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
Integration test: bring the simulation up and assert the graph is wired.

The unit tests deliberately cover pure functions only, so everything
ROS-shaped — controller activation, bridge topics, TF — was verified by
hand until now. This launches the real headless sim and checks the things
that break silently when a launch file, controller config or bridge entry
regresses:

  * all four controllers reach 'active'
  * the load-bearing topics actually publish, at their expected sim rates
  * base_footprint -> base_link resolves in TF

It needs a real Gazebo and takes ~2 min, so it is OFF by default and
lives outside test/ where the fast unit run cannot collect it. Enable it
explicitly:

  colcon build --packages-select gazebo_models \
      --cmake-args -DBUILD_SIM_INTEGRATION_TESTS=ON
  colcon test --packages-select gazebo_models
"""
import os
import unittest

from ament_index_python.packages import get_package_share_directory

import launch
import launch.actions
import launch_testing
import launch_testing.actions

import pytest

import rclpy
from rclpy.node import Node


CONTROLLERS = [
    'joint_state_broadcaster',
    'diff_drive_controller',
    'arm_controller',
    'gripper_controller',
]

# Generous: Gazebo start-up plus four spawners, on a loaded CI box.
BRINGUP_TIMEOUT = 120.0


@pytest.mark.launch_test
def generate_test_description():
    sim = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_models'),
                         'launch', 'full_world_robo.launch.py')),
        launch_arguments={'gui': 'false'}.items(),
    )
    return (
        launch.LaunchDescription([
            sim,
            launch_testing.actions.ReadyToTest(),
        ]),
        {},
    )


class TestSimBringup(unittest.TestCase):
    """Assertions against the live graph."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_sim_bringup')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _wait_for(self, predicate, timeout, description):
        """Spin until predicate() is true, or fail naming what was missing."""
        import time
        deadline = time.time() + timeout
        last = None
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            last = predicate()
            if last is True or (last and last is not False):
                return last
        self.fail(f'timed out after {timeout:.0f}s waiting for {description} '
                  f'(last result: {last})')

    def test_controllers_become_active(self):
        """A controller that fails to spawn leaves the robot inert."""
        from controller_manager_msgs.srv import ListControllers

        client = self.node.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self.assertTrue(
            client.wait_for_service(timeout_sec=BRINGUP_TIMEOUT),
            '/controller_manager/list_controllers never appeared')

        def all_active():
            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            response = future.result()
            if response is None:
                return False
            active = {c.name for c in response.controller
                      if c.state == 'active'}
            return active if set(CONTROLLERS) <= active else False

        active = self._wait_for(all_active, BRINGUP_TIMEOUT,
                                f'controllers {CONTROLLERS} to become active')
        for name in CONTROLLERS:
            self.assertIn(name, active)

    def test_load_bearing_topics_publish_at_rate(self):
        """Reuses verify_sim's own checks so the two cannot drift apart."""
        import sys
        sys.path.insert(0, os.path.join(
            get_package_share_directory('gazebo_models'), '..', '..',
            'lib', 'gazebo_models'))
        from verify_sim import CHECKS, run_checks

        # Give the bridge and controllers time to come up before measuring.
        self._wait_for(
            lambda: self.node.count_publishers('/joint_states') > 0,
            BRINGUP_TIMEOUT, '/joint_states to have a publisher')

        results = run_checks(CHECKS, node=self.node, verbose=True)
        failures = {topic: detail
                    for topic, (passed, _, detail) in results.items()
                    if not passed}
        self.assertEqual({}, failures, f'topic checks failed: {failures}')

    def test_base_footprint_transform_resolves(self):
        """Nav2 costmaps and AMCL both key off base_footprint."""
        from tf2_ros import Buffer, TransformListener

        buffer = Buffer()
        TransformListener(buffer, self.node)

        def can_transform():
            return buffer.can_transform('base_footprint', 'base_link',
                                        rclpy.time.Time())

        self._wait_for(can_transform, 30.0,
                       'TF base_footprint -> base_link')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self):
        """The sim must not have crashed during the run."""
        launch_testing.asserts.assertExitCodes(
            self.proc_info, allowable_exit_codes=[0, -2, -6, -15])
