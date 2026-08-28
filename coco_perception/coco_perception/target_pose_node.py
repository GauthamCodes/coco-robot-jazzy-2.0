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
target_pose_node — the ROS face of `target_pose`.

C2-M4.0. This node holds no geometry. It subscribes, it calls
`target_pose`, it asks `tf2` for the one transform it needs, and it
publishes. Every number it reports was computed somewhere that can be
unit-tested without a graph. That split is the same one C2-M2 made
between `terrain_observer` and `terrain_observer_node`, for the same
reason: the C2-M2.1 live gate found three defects, and all three were in
the node — QoS, callback placement, a parameter that was declared and
never read. None were in the arithmetic.

Why this node exists next to `target_finder`
--------------------------------------------
`target_finder` is unchanged and still publishes `/perception/target`,
which `approach_server`'s servo mode consumes and which the M6 fetch was
measured with. Breaking it to add a frame would be trading a measured
20/20 approach for an unmeasured one.

What this node adds is the three things manipulation needs and that
topic cannot carry: an explicit validity, a pose whose frame came from
the robot's own TF tree rather than from arithmetic over two constants,
and a reachability verdict.

Topics
------
in   /camera/image_raw        sensor_msgs/Image       BEST_EFFORT
in   /camera/depth/image_raw  sensor_msgs/Image       BEST_EFFORT
in   /camera/camera_info      sensor_msgs/CameraInfo  BEST_EFFORT
in   /mission/target_colour   std_msgs/String
in   /tf, /tf_static

out  /perception/target_pose         vision_msgs/Detection3DArray
out  /perception/grasp_point         geometry_msgs/PoseStamped
out  /perception/target_pose/status  std_msgs/String  (5 Hz, key=value)

`/perception/target_pose` is empty — a zero-length `detections` array —
whenever the answer is not VALID. Empty means "no target this frame",
which is a statement; the status topic, published at 5 Hz regardless,
says *which* of the five non-valid states it is. A consumer that only
takes the Detection3DArray still cannot mistake silence for absence,
because an empty array still arrives.

`vision_msgs/Detection3DArray` rather than a custom message: it already
carries identity (`id`), class (`class_id`), a confidence (`score`), a
sized 3-D box, a pose, and a header with a frame and a stamp. That is
the whole of what §4 asked for, and a package that defines its own
message stops being `ament_python`.

BEST_EFFORT on the camera inputs is not optional; the flag comes from
`coco_config.robot.is_best_effort()` rather than being re-typed here.
See CLAUDE.md's trap table — a RELIABLE subscriber never matches the
bridge and the node goes silently blind.
"""

import importlib.util
import os

from coco_config.robot import (camera_intrinsics, is_best_effort,
                               target_by_colour, TARGET_COLOURS)

from coco_perception import target_pose as tp
from coco_perception.target_finder import (blob_stats, colour_mask,
                                           match_depth, normalise_colour)

import cv2

from cv_bridge import CvBridge

from geometry_msgs.msg import PoseStamped

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from sensor_msgs.msg import CameraInfo, Image

from std_msgs.msg import String

import tf2_ros

from vision_msgs.msg import (Detection3D, Detection3DArray,
                             ObjectHypothesisWithPose)


def load_arm_ik():
    """
    Import `arm_ik` from wherever colcon installed it, or return None.

    `arm_ik.py` lives in `coco_moveit_config/scripts/` and is installed
    with `install(PROGRAMS ...)` into `lib/coco_moveit_config/`. That
    makes it an executable, not an importable module — `pick_place.py`
    gets away with `import arm_ik` only because it is itself run out of
    that directory, so `sys.path[0]` is already there.

    Resolving it through `ament_index` rather than a relative path is
    what keeps this honest: there is exactly one installed copy of the
    arm's kinematics and this finds that one. Copying the twelve
    constants into `coco_perception` would be the "two hand-maintained
    models" failure CLAUDE.md rule 3 names, and they would diverge the
    first time a link length changed.

    Returning None rather than raising is deliberate. Perception is
    useful without a reachability verdict, and `IK_UNAVAILABLE` is a
    state `target_pose` can report. An ImportError here would take the
    whole pipeline down over a downstream convenience.
    """
    try:
        from ament_index_python.packages import get_package_prefix
        path = os.path.join(get_package_prefix('coco_moveit_config'),
                            'lib', 'coco_moveit_config', 'arm_ik.py')
        if not os.path.exists(path):
            return None
        spec = importlib.util.spec_from_file_location('arm_ik', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    except Exception:
        return None


class TargetPoseNode(Node):
    """Publishes where the selected target is, in a frame the arm uses."""

    def __init__(self):
        super().__init__('target_pose_node')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('info_topic', '/camera/camera_info')
        self.declare_parameter('colour_topic', '/mission/target_colour')
        self.declare_parameter('pose_topic', '/perception/target_pose')
        self.declare_parameter('grasp_topic', '/perception/grasp_point')
        self.declare_parameter('status_topic',
                               '/perception/target_pose/status')
        self.declare_parameter('target_frame', 'base_footprint')
        self.declare_parameter('min_range', 0.15)
        self.declare_parameter('max_range', 2.0)
        self.declare_parameter('width_tolerance', 0.5)
        self.declare_parameter('stamp_tolerance', 0.1)
        # How far the transform's own stamp may sit from the image's
        # before the answer is refused. camera_optical_frame is fixed to
        # base_footprint through two fixed joints, so this is expected
        # to read ~0 and is here to catch the case where it does not.
        self.declare_parameter('max_tf_age', 0.2)
        self.declare_parameter('status_hz', 5.0)
        self.declare_parameter('target_colour', 'blue')

        self.target_frame = self.get_parameter('target_frame').value
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.width_tolerance = float(
            self.get_parameter('width_tolerance').value)
        self.stamp_tolerance = float(
            self.get_parameter('stamp_tolerance').value)
        self.max_tf_age = float(self.get_parameter('max_tf_age').value)

        self.selected = normalise_colour(
            self.get_parameter('target_colour').value) or 'blue'
        self.intrinsics = camera_intrinsics()
        self.have_info = False
        self.bridge = CvBridge()
        self._depth = {}
        self._status = tp.format_status(sel=self.selected,
                                        validity=tp.NOT_DETECTED)

        # The solver, resolved once. None is a reportable state.
        arm_ik = load_arm_ik()
        self.ik = None if arm_ik is None else arm_ik.ik_or_none

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        image_topic = self.get_parameter('image_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        info_topic = self.get_parameter('info_topic').value

        sensor_qos = QoSProfile(
            depth=1,
            reliability=(ReliabilityPolicy.BEST_EFFORT
                         if is_best_effort(image_topic)
                         else ReliabilityPolicy.RELIABLE))

        self.create_subscription(Image, image_topic, self._on_image,
                                 sensor_qos)
        self.create_subscription(Image, depth_topic, self._on_depth,
                                 sensor_qos)
        self.create_subscription(CameraInfo, info_topic, self._on_info,
                                 sensor_qos)
        self.create_subscription(
            String, self.get_parameter('colour_topic').value,
            self._on_colour, 10)

        self._pose_pub = self.create_publisher(
            Detection3DArray, self.get_parameter('pose_topic').value, 10)
        self._grasp_pub = self.create_publisher(
            PoseStamped, self.get_parameter('grasp_topic').value, 10)
        self._status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10)

        self.create_timer(
            1.0 / float(self.get_parameter('status_hz').value),
            self._publish_status)

        self.get_logger().info(
            f'target_pose_node: {image_topic} + {depth_topic} '
            f'-> {self.target_frame}, looking for {self.selected!r}')
        self.get_logger().info(
            'IK: ' + ('arm_ik loaded' if self.ik else
                      'UNAVAILABLE — reachability will read IK_UNAVAILABLE'))

    # ── inputs ───────────────────────────────────────────────────────────
    def _on_info(self, msg):
        if self.have_info:
            return
        self.intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        self.have_info = True
        self.get_logger().info(
            f'camera_info: fx={msg.k[0]:.2f} fy={msg.k[4]:.2f} '
            f'cx={msg.k[2]:.2f} cy={msg.k[5]:.2f}')

    def _on_depth(self, msg):
        # '32FC1' explicitly, never 'passthrough'. gz's rgbd_camera
        # writes float metres; if that ever changes this should fail
        # loudly rather than reinterpret the bytes.
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        self._depth[stamp_ns] = depth
        for stale in sorted(self._depth)[:-5]:
            del self._depth[stale]

    def _on_colour(self, msg):
        colour = normalise_colour(msg.data)
        if colour is None:
            self.get_logger().warn(
                f'ignoring unknown target colour {msg.data!r} — staying on '
                f'{self.selected!r}. Known: {", ".join(TARGET_COLOURS)}')
            return
        if colour != self.selected:
            self.get_logger().info(f'target {self.selected} -> {colour}')
            self.selected = colour

    # ── the frame ────────────────────────────────────────────────────────
    def _lookup(self, source_frame, stamp):
        """
        Look up the camera->robot transform at `stamp`.

        Asked for at the IMAGE's stamp, not at 'latest'. The two are the
        same number here only because both frames hang off base_link
        through fixed joints; asking for latest would hide the day that
        stops being true, and this is the one place in the pipeline
        where the robot's own geometry enters.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, source_frame, stamp,
                timeout=Duration(seconds=0.05))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
                tf2_ros.TransformException) as exc:
            return None, str(exc)
        return tf, None

    @staticmethod
    def _apply(tf):
        """Build a callable applying a TransformStamped to a point."""
        t = tf.transform.translation
        q = tf.transform.rotation
        # Quaternion rotation without importing a transforms library:
        # v' = v + 2*w*(u x v) + 2*(u x (u x v)), u = (x, y, z) of q.
        u = np.array([q.x, q.y, q.z])
        w = q.w

        def apply(point):
            v = np.asarray(point, dtype=np.float64)
            uv = np.cross(u, v)
            rotated = v + 2.0 * w * uv + 2.0 * np.cross(u, uv)
            return (float(rotated[0] + t.x),
                    float(rotated[1] + t.y),
                    float(rotated[2] + t.z))
        return apply

    # ── the frame-by-frame answer ────────────────────────────────────────
    def _on_image(self, msg):
        # 'bgr8', never 'passthrough': the bridge delivers rgb8, and
        # treating that as BGR swaps red and blue — a confident
        # detection of the wrong object.
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec

        key = match_depth(stamp_ns, list(self._depth), self.stamp_tolerance)
        depth = None if key is None else self._depth[key]

        seen = [colour for colour in TARGET_COLOURS
                if blob_stats(colour_mask(hsv, colour))[0]]

        observation = self._observe(hsv, depth, seen)
        observation.stamp_ns = stamp_ns
        if observation.is_valid:
            observation = self._reframe(observation, msg.header.stamp)

        self._status = tp.status_for(observation)
        self._publish(observation, msg.header.stamp)

    def _observe(self, hsv, depth, seen):
        """Detect and range the target in the camera's own frame."""
        if depth is None:
            return tp.TargetObservation(self.selected, tp.DEPTH_INVALID,
                                        seen=seen)
        mask = colour_mask(hsv, self.selected)
        if mask is None:
            return tp.TargetObservation(self.selected, tp.NOT_DETECTED,
                                        seen=seen)
        if depth.shape[:2] != mask.shape[:2]:
            # RGB and depth come out of one gz rgbd_camera and are
            # registered by construction. If the shapes disagree they
            # are not the same camera and pairing them would deproject
            # one image's pixel with another's range.
            self.get_logger().warn(
                f'depth {depth.shape[:2]} does not match colour '
                f'{mask.shape[:2]} — cannot pair them')
            return tp.TargetObservation(self.selected, tp.DEPTH_INVALID,
                                        seen=seen)
        blobs, labels = blob_stats(mask)
        return tp.observe(
            blobs, lambda label: depth[labels == label], self.selected,
            self.intrinsics, self.min_range, self.max_range, seen=seen,
            tolerance=self.width_tolerance)

    def _reframe(self, observation, stamp):
        """Camera optical frame -> the manipulation frame, through tf2."""
        tf, reason = self._lookup(observation.frame_id, stamp)
        if tf is None:
            observation.validity = tp.NO_TRANSFORM
            observation.point = None
            self.get_logger().warn(f'no transform: {reason}',
                                   throttle_duration_sec=5.0)
            return observation

        age = abs(Time.from_msg(tf.header.stamp).nanoseconds
                  - Time.from_msg(stamp).nanoseconds) / 1e9
        if age > self.max_tf_age:
            observation.validity = tp.STALE_TRANSFORM
            observation.tf_age = age
            observation.point = None
            self.get_logger().warn(
                f'transform {age:.3f}s from the image, over the '
                f'{self.max_tf_age:.3f}s limit', throttle_duration_sec=5.0)
            return observation

        return tp.transform_observation(
            observation, self._apply(tf), self.target_frame,
            ik=self.ik, tf_age=age)

    # ── outputs ──────────────────────────────────────────────────────────
    def _publish(self, observation, stamp):
        array = Detection3DArray()
        array.header.stamp = stamp
        array.header.frame_id = (observation.frame_id if observation.is_valid
                                 else self.target_frame)

        if observation.is_valid:
            target = target_by_colour(observation.colour)
            det = Detection3D()
            det.header = array.header
            det.id = target.model
            det.bbox.center.position.x = observation.point[0]
            det.bbox.center.position.y = observation.point[1]
            det.bbox.center.position.z = observation.point[2]
            det.bbox.center.orientation.w = 1.0
            # The cylinder's known extent. Not measured from the image —
            # the blob is ~8 px wide at working range and its apparent
            # size carries no information the colour has not already
            # given. Publishing the table value keeps a consumer from
            # reading pixel noise as a dimension.
            det.bbox.size.x = target.diameter
            det.bbox.size.y = target.diameter
            det.bbox.size.z = target.height

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = observation.colour
            # The share of the blob's pixels that carried a usable
            # depth. It is a data-completeness figure and is labelled
            # that way in the docs; it is NOT a calibrated probability
            # and nothing downstream should treat it as one.
            hypothesis.hypothesis.score = (
                observation.depth.valid_fraction if observation.depth else 0.0)
            hypothesis.pose.pose = det.bbox.center
            det.results.append(hypothesis)
            array.detections.append(det)

            grasp = PoseStamped()
            grasp.header.stamp = stamp
            grasp.header.frame_id = observation.frame_id
            grasp.pose.position.x = observation.grasp_point[0]
            grasp.pose.position.y = observation.grasp_point[1]
            grasp.pose.position.z = observation.grasp_point[2]
            grasp.pose.orientation.w = 1.0
            self._grasp_pub.publish(grasp)

        self._pose_pub.publish(array)

    def _publish_status(self):
        self._status_pub.publish(String(data=self._status))


def main(args=None):
    rclpy.init(args=args)
    node = TargetPoseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
