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
Measure target_finder against ground truth, one station at a time.

The M5 result artifact. It teleports the robot to a grid of poses on the
platform, asks for each colour in turn, and compares what the node
reports with where Gazebo says the object actually is. The number that
matters is the lateral and forward error in millimetres, because the
grasp has a ~27 mm approach window to stop the base inside.

It teleports rather than climbing. The RL policy has a measured +0.61 m
of lateral drift over the climb, so driving up into an outer lane is
itself unreliable — using it as the transport for a vision measurement
would confound the two, and a failed climb would cost the whole run.
`set_pose` carries absolute values, so it is idempotent and safe to
retry, which is the same property ramp_env relies on.

Everything runs against ONE Gazebo instance. There is no restart
anywhere in the grid.

Usage (sim with traverse:=true, plus perception.launch.py, already up):
  ros2 run coco_perception vision_check
  ros2 run coco_perception vision_check --colours blue --stations 3.4
"""

import argparse
import math
import subprocess
import sys
import time

from coco_config.robot import (colour_for_lane, lane_for_colour, RAMP_RUN,
                               RAMP_SUMMIT_X, target_by_colour,
                               TARGET_COLOURS, TARGET_ROW_X)

from nav_msgs.msg import Odometry

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String

WORLD = 'coco_world'

# Where the robot is stationed, as base_footprint x in world coordinates.
# 3.20 puts the camera 0.725 m from the target row; 3.75 puts it at
# 0.175 m, just inside the depth camera's near clip. The robot is fully
# on the platform (x 3.0..4.5) from 3.1485 given its 0.297 m footprint.
DEFAULT_STATIONS = (3.20, 3.40, 3.60, 3.75)

# Error the grasp can absorb. The approach window is ~27 mm wide, so a
# third of it is the working target rather than a stretch goal.
TOLERANCE_MM = 8.0


def gz_service(service, reqtype, reptype, req, timeout_ms=5000, attempts=5):
    """
    Call a Gazebo transport service; True iff it replied true.

    Retried by default because each call spawns a short-lived process
    binding an ephemeral gz-transport node, and the round trip
    occasionally overruns. A 180k-step curriculum was lost to exactly
    that; see coco_rl.ramp_env.gz_service for the full account. Absolute
    poses are idempotent, so re-sending one that did land is harmless.
    """
    cmd = ['gz', 'service', '-s', service, '--reqtype', reqtype,
           '--reptype', reptype, '--timeout', str(timeout_ms), '--req', req]
    for attempt in range(max(1, attempts)):
        if attempt:
            time.sleep(0.5)
        try:
            out = subprocess.run(cmd, capture_output=True, text=True,
                                 timeout=max(15, timeout_ms / 1000 * 5))
        except FileNotFoundError:
            raise RuntimeError(
                '`gz` not found on PATH — source setup_env.sh first.'
            ) from None
        except subprocess.TimeoutExpired:
            continue
        if out.returncode == 0 and 'true' in out.stdout.lower():
            return True
    return False


def model_pose(name):
    """(x, y, z) of a Gazebo model in the world frame, or None."""
    try:
        out = subprocess.run(['gz', 'model', '-m', name, '-p'],
                             capture_output=True, text=True, timeout=10)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None
    if out.returncode != 0:
        return None
    lines = out.stdout.splitlines()
    for index, line in enumerate(lines):
        if 'Pose' in line and index + 1 < len(lines):
            parts = lines[index + 1].strip().strip('[]').split()
            if len(parts) == 3:
                try:
                    return tuple(float(v) for v in parts)
                except ValueError:
                    return None
    return None


def world_to_base(target_xyz, robot_xyz, yaw):
    """
    Express a world point in the robot's base_footprint frame.

    Ground truth for what target_finder deprojects. base_footprint is on
    the ground under the robot, so z is measured from the platform
    surface the robot is standing on.
    """
    dx = target_xyz[0] - robot_xyz[0]
    dy = target_xyz[1] - robot_xyz[1]
    return (dx * math.cos(yaw) + dy * math.sin(yaw),
            -dx * math.sin(yaw) + dy * math.cos(yaw),
            target_xyz[2] - robot_xyz[2])


def field_of(line, key):
    """Value of `key=` in a space-separated key=value line, or None."""
    for part in (line or '').split(' '):
        if part.startswith(f'{key}='):
            value = part.split('=', 1)[1]
            return None if value == '--' else value
    return None


def platform_top(ramp_angle_deg):
    """Height of the crest platform for a given grade."""
    return RAMP_RUN * math.tan(math.radians(ramp_angle_deg))


class VisionCheck(Node):
    """Stations the robot, asks for a colour, records what came back."""

    def __init__(self):
        super().__init__('vision_check')
        self.pose = None
        self.yaw = None
        self.status = ''
        self.create_subscription(
            Odometry, '/model/coco/odometry', self._odom_cb, 10)
        self.create_subscription(
            String, '/perception/status', self._status_cb, 10)
        self._colour_pub = self.create_publisher(
            String, '/mission/target_colour', 10)

    def _odom_cb(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.pose = (position.x, position.y, position.z)
        self.yaw = math.atan2(
            2.0 * (orientation.w * orientation.z
                   + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2))

    def _status_cb(self, msg):
        self.status = msg.data

    def spin_for(self, seconds):
        deadline = time.time() + seconds
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_ready(self, timeout=40.0):
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose is not None and self.status:
                return True
        self.get_logger().error(
            f'not ready (odom={self.pose is not None}, '
            f'perception={bool(self.status)}) — is the sim up with '
            f'traverse:=true, and perception.launch.py running?')
        return False

    def station(self, x, y, z, settle=2.5):
        """Teleport the robot and let the physics settle."""
        if not gz_service(
                f'/world/{WORLD}/set_pose', 'gz.msgs.Pose',
                'gz.msgs.Boolean',
                f'name: "coco", position: {{x: {x}, y: {y}, z: {z}}}, '
                f'orientation: {{z: 0.0, w: 1.0}}'):
            self.get_logger().error(f'set_pose failed for ({x}, {y})')
            return False
        self.spin_for(settle)
        return True

    def ask_for(self, colour, settle=1.5):
        """Assert a target colour and let the node act on it."""
        deadline = time.time() + settle
        while time.time() < deadline and rclpy.ok():
            self._colour_pub.publish(String(data=colour))
            rclpy.spin_once(self, timeout_sec=0.1)


def measure(node, colour, station_x, top):
    """One grid cell: station, ask, compare. Returns a result dict."""
    lane = lane_for_colour(colour)
    target = target_by_colour(colour)
    if not node.station(station_x, lane, top + 0.05):
        return None
    node.ask_for(colour)
    node.spin_for(1.0)

    truth_world = model_pose(target.model)
    if truth_world is None or node.pose is None:
        return None
    truth = world_to_base(truth_world, node.pose, node.yaw)

    found = field_of(node.status, 'found') == '1'
    result = {
        'colour': colour, 'station': station_x, 'found': found,
        'distance': TARGET_ROW_X - station_x,
        'seen': field_of(node.status, 'seen'),
        'truth': truth, 'status': node.status,
    }
    if found:
        got = tuple(float(field_of(node.status, key)) for key in 'xyz')
        result['reported'] = got
        result['error_mm'] = tuple(
            (g - t) * 1000.0 for g, t in zip(got, truth))
        result['width_px'] = int(field_of(node.status, 'w'))
        result['range'] = float(field_of(node.status, 'range'))
    return result


def report(results):
    """Print the result table and return True if it met the target."""
    print(f'\n{"colour":<8}{"base x":>8}{"d (m)":>8}{"found":>7}'
          f'{"w px":>6}{"dx mm":>8}{"dy mm":>8}  notes')
    print('-' * 78)
    passes = 0
    measured = 0
    for row in results:
        if row is None:
            print('  (station failed)')
            continue
        if not row['found']:
            print(f'{row["colour"]:<8}{row["station"]:>8.2f}'
                  f'{row["distance"]:>8.3f}{"no":>7}{"":>6}{"":>8}{"":>8}'
                  f'  seen={row["seen"]}')
            continue
        measured += 1
        dx, dy, _dz = row['error_mm']
        ok = abs(dx) < TOLERANCE_MM and abs(dy) < TOLERANCE_MM
        passes += ok
        print(f'{row["colour"]:<8}{row["station"]:>8.2f}'
              f'{row["distance"]:>8.3f}{"yes":>7}{row["width_px"]:>6}'
              f'{dx:>+8.1f}{dy:>+8.1f}  {"" if ok else "OUT OF TOLERANCE"}')

    total = len([r for r in results if r is not None])
    print('-' * 78)
    print(f'detected {measured}/{total}; '
          f'{passes}/{measured or 1} inside +-{TOLERANCE_MM:.0f} mm')
    return measured == total and passes == measured


def wrong_lane_check(node, top):
    """
    Station in one lane while asking for another lane's target.

    This is the signal that answers the RL policy's lateral drift: the
    neighbouring target stays inside the frame at this distance, so the
    node should report not-found WITH the colour it can actually see,
    rather than an unexplained silence.
    """
    print('\n=== wrong-lane signal ===')
    ok = True
    for colour in TARGET_COLOURS:
        neighbour = colour_for_lane(lane_for_colour(colour) + 0.5)
        if neighbour is None:
            continue
        node.station(3.40, lane_for_colour(neighbour), top + 0.05)
        node.ask_for(colour)
        node.spin_for(1.0)
        found = field_of(node.status, 'found') == '1'
        seen = field_of(node.status, 'seen') or ''
        diagnostic = (not found) and neighbour in seen
        ok = ok and diagnostic
        print(f'  asked {colour:<7} standing in {neighbour:<7} '
              f'-> found={"yes" if found else "no":<4} seen={seen:<24}'
              f'{"" if diagnostic else "  NO DIAGNOSIS"}')
    return ok


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--colours', nargs='+', default=list(TARGET_COLOURS),
                        choices=TARGET_COLOURS)
    parser.add_argument('--stations', nargs='+', type=float,
                        default=list(DEFAULT_STATIONS),
                        help='base_footprint x in world coordinates')
    parser.add_argument('--ramp-angle', type=float, default=18.0,
                        help='grade the sim was launched with; sets the '
                             'platform height to teleport onto')
    parser.add_argument('--skip-wrong-lane', action='store_true')
    cli, _ = parser.parse_known_args()

    top = platform_top(cli.ramp_angle)
    print(f'platform top z={top:.4f} (ramp_angle={cli.ramp_angle:g}), '
          f'target row x={RAMP_SUMMIT_X + 1.05:.2f}')

    rclpy.init(args=args)
    node = VisionCheck()
    rc = 1
    try:
        if not node.wait_ready():
            sys.exit(1)
        results = [measure(node, colour, station, top)
                   for colour in cli.colours
                   for station in cli.stations]
        ok = report(results)
        if not cli.skip_wrong_lane:
            ok = wrong_lane_check(node, top) and ok
        rc = 0 if ok else 1
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
