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
target_finder — which coloured object is in front, and exactly where.

The fetch mission's last unknown. The phone picks a colour, the sequencer
has already driven to that colour's lane and climbed, and this node has
to confirm the right object is there and hand the grasp a position good
enough to stop the base inside a ~27 mm approach window.

Why colour, and why it is easy here
-----------------------------------
Every other object in the arena was recoloured neutral grey in M2 for
this node's benefit: the obstacles used to be brownish-red and blue,
which collide with two of the four target colours in HSV. The four
targets are now the only saturated things in the world, so a saturation
gate does most of the work before any hue threshold is involved. That
was worth more than any amount of tuning.

Hue itself is stable under lighting here for a concrete reason: the
scene is lit by white sources only, so shading scales all three channels
by the same factor, and a power-law display encoding turns a scale of
`k` into a scale of `k**g` — still uniform. Hue depends only on channel
ratios, so it survives exactly. What breaks it is specular highlight
(adds white, desaturates) and clipping at 255, which is what the
morphological close and the largest-component rule are for.

Why range comes from depth and not the lidar
--------------------------------------------
The lidar's scan plane is 0.2135 m up. The targets stand 0.158 m tall on
the platform, entirely underneath it. Depth is not a nicety here, it is
the only sensor that sees them.

What this node deliberately does NOT do
---------------------------------------
It does not survey the four lanes and choose. It cannot: the crest edge
occludes the whole platform from every point on the flat ground (a
camera at the pre-ramp pose sees nothing at the target row below
z=0.907, and the targets top out at 0.808), so the colour->lane decision
has to be a table lookup made before the climb. See
coco_config.robot.lane_for_colour. What this node adds once the robot is
up there is confirmation, a 3-D position, and — because a neighbouring
lane's target is still inside the frame at the working distance — a
signal for arriving in the wrong lane.

Topics
------
in   /camera/image_raw        sensor_msgs/Image       BEST_EFFORT
in   /camera/depth/image_raw  sensor_msgs/Image       BEST_EFFORT
in   /camera/camera_info      sensor_msgs/CameraInfo  BEST_EFFORT
in   /mission/target_colour   std_msgs/String
out  /perception/target       geometry_msgs/PointStamped  (base_footprint)
out  /perception/status       std_msgs/String  (5 Hz, key=value)
out  /perception/annotated    sensor_msgs/Image  (bgr8, for the phone)

BEST_EFFORT on the camera inputs is not optional: the gz->ROS bridge
republishes with sensor QoS, so a RELIABLE subscriber never matches and
the node sits silent with no error anywhere. The flag is read from
coco_config.robot.is_best_effort() rather than re-typed here.

Every topic name is a ROS parameter, so this node never needs a remap.
"""

import math

from coco_config.robot import (camera_intrinsics, CAMERA_RPY, CAMERA_XYZ,
                               is_best_effort, lane_for_colour,
                               target_by_colour, TARGET_COLOURS)

import cv2

from cv_bridge import CvBridge

from geometry_msgs.msg import PointStamped

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import CameraInfo, Image

from std_msgs.msg import String

# ── colour ───────────────────────────────────────────────────────────────
# OpenCV HSV: hue 0-179, saturation and value 0-255.
#
# Band centres implied by the targets' SDF colours are red 0, yellow ~27,
# green ~64, blue ~111 if gz gamma-encodes its output, and 0 / 26 / 63 /
# 114 if it writes linear RGB. These bands are deliberately wide enough
# to contain BOTH, because which one gz does is not worth asserting from
# a desk — confirm against a sampled frame during bring-up and tighten
# then.
#
# RED WRAPS ZERO and needs two ranges OR'd together. A single [0, 9] band
# is the classic version of this bug: it still finds a blob, just half of
# one, with a centroid offset toward whichever side survived. That is a
# wrong answer that looks entirely plausible in the status line.
#
# The one near-miss in the whole scene is coco_orange (0.9 0.45 0.1) on
# the forearm, at hue ~18 gamma-encoded / ~13 linear — between the red
# and yellow bands, which is why yellow starts at 22 rather than 15. It
# is out of frame at every mission pose (the forearm sits above a camera
# 68.5 mm off the ground), and MIN_RANGE is the second guard.
HSV_BANDS = {
    'red':    ((0, 9), (171, 179)),
    'yellow': ((22, 40),),
    'green':  ((52, 78),),
    'blue':   ((98, 126),),
}

# The arena greys (platform 0.60 0.60 0.62, obstacles 0.55 0.55 0.57,
# walls, ramp) all land under saturation 10; the targets sit above 150.
# 80 is the middle of a very wide gap, not a tuned number.
MIN_SATURATION = 80
MIN_VALUE = 40

# ── blob selection ───────────────────────────────────────────────────────
# Apparent width is diameter * fx / range. At the working distance the
# four targets span 6.3-10.1 px and adjacent diameters differ by 1.3 px,
# so this is a sanity check on the range and NOT a size identifier: a
# 24 mm target passes the 20 mm gate outright, and even the extremes
# separate by only 0.6 px, which is inside the segmentation noise on a
# blob this small. Colour says which object it is; this throws out a blob
# whose size is nothing like a target's at the measured range.
WIDTH_TOLERANCE = 0.5
MIN_BLOB_PX = 6

# ── depth ────────────────────────────────────────────────────────────────
# The mask covers the cylinder's near surface, not its axis. Averaged
# across the projected width the offset is r*pi/4; the median of the
# masked depths lands close to it. Skipping this leaves a systematic
# 8-13 mm bias, which is a third of the approach window.
SURFACE_TO_AXIS = 0.8
MIN_VALID_DEPTH_PX = 6

# The two depth streams are separate bridge subscriptions, so arrival
# order is not guaranteed even though both carry the same sensor's sim
# stamp. 0.1 s is 1.5 frames at 15 Hz.
STAMP_TOLERANCE = 0.1

STATUS_HZ = 5.0

# Fields of /perception/status, in order. Space-separated key=value so
# the browser can split it without a JSON parser; no value may itself
# contain a space.
STATUS_KEYS = ('sel', 'found', 'u', 'v', 'area', 'w', 'h', 'range',
               'x', 'y', 'z', 'lane', 'seen', 'age')
SIGNED_STATUS_KEYS = ('y', 'lane')


def normalise_colour(raw):
    """
    Fold a /mission/target_colour string onto one of the four colours.

    Returns None for anything unrecognised so the caller can warn and
    keep the previous selection. Silently defaulting would produce a
    mission that ran perfectly and fetched the wrong object.
    """
    colour = (raw or '').strip().lower()
    return colour if colour in TARGET_COLOURS else None


def classify_hsv(hue, saturation, value):
    """
    Which target colour a single HSV pixel is, or None.

    The saturation and value gates come first because they are what
    actually separates the targets from the arena; hue only has to tell
    the four apart from each other.
    """
    if saturation < MIN_SATURATION or value < MIN_VALUE:
        return None
    for colour, bands in HSV_BANDS.items():
        for low, high in bands:
            if low <= hue <= high:
                return colour
    return None


def colour_mask(hsv, colour):
    """
    Binary mask of `colour` in an HSV image, closed to fill highlights.

    Red is OR'd across two hue ranges; everything else has one. The
    close comes before component labelling so a specular highlight that
    clipped a channel punches a hole in the blob rather than splitting
    it in two.
    """
    bands = HSV_BANDS.get(colour)
    if bands is None:
        return None
    mask = None
    for low, high in bands:
        band = cv2.inRange(
            hsv,
            np.array([low, MIN_SATURATION, MIN_VALUE], dtype=np.uint8),
            np.array([high, 255, 255], dtype=np.uint8))
        mask = band if mask is None else cv2.bitwise_or(mask, band)
    kernel = np.ones((3, 3), np.uint8)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


def blob_stats(mask, min_area=MIN_BLOB_PX):
    """
    Find the connected components of a mask, as (blobs, labels).

    Each blob is (area, u, v, width, height, label) with (u, v) its
    centroid, sorted largest-first. `labels` comes back alongside so a
    caller can sample depth over ONE component rather than over the
    whole mask — with two same-coloured blobs in frame, a mask-wide
    median mixes their ranges and lands between two real objects.
    """
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(
        mask, connectivity=8)
    blobs = []
    for index in range(1, count):          # 0 is the background label
        area = int(stats[index, cv2.CC_STAT_AREA])
        if area < min_area:
            continue
        blobs.append((area,
                      float(centroids[index][0]), float(centroids[index][1]),
                      int(stats[index, cv2.CC_STAT_WIDTH]),
                      int(stats[index, cv2.CC_STAT_HEIGHT]),
                      index))
    blobs.sort(key=lambda b: -b[0])
    return blobs, labels


def expected_width_px(diameter, rng, fx):
    """Pixels a cylinder of this diameter subtends at this range."""
    if rng is None or rng <= 0.0:
        return None
    return diameter * fx / rng


def plausible_blob(width_px, diameter, rng, fx, tolerance=WIDTH_TOLERANCE):
    """
    Whether a blob's width matches the target's, at the measured range.

    Width rather than height or aspect: the cylinder is 158 mm tall
    against a camera 68.5 mm up, so its base leaves the bottom of the
    frame below 0.166 m and its pixel height clips. Width does not.
    """
    expected = expected_width_px(diameter, rng, fx)
    if expected is None:
        return False
    return abs(width_px - expected) <= tolerance * expected


def robust_depth(values, near, far, min_valid=MIN_VALID_DEPTH_PX):
    """
    Median of the valid depths in `values`, or None if too few.

    gz writes +/-inf past the clip planes, NaN where the shader missed,
    and 0.0 for some near-plane cases. All four have to go, and the
    result must be None rather than 0.0 when nothing survives: a 0.0
    range deprojects to the camera's own origin, which is a confident,
    well-formed, completely wrong answer.

    The median rather than the centroid pixel because the blob is only
    ~8 px wide at the working distance, and a centroid can land on a
    silhouette pixel carrying the background's depth.
    """
    array = np.asarray(values, dtype=np.float64).ravel()
    good = array[np.isfinite(array) & (array > near) & (array < far)]
    if good.size < min_valid:
        return None
    return float(np.median(good))


def surface_to_axis(depth, radius):
    """Move a near-surface range back to the cylinder's axis."""
    return depth + SURFACE_TO_AXIS * radius


def deproject(u, v, z, fx, fy, cx, cy):
    """Pixel plus range -> a point in the camera's optical frame."""
    return ((u - cx) * z / fx, (v - cy) * z / fy, z)


def _rotate_rpy(vec, rpy):
    """Apply an RPY (Z-Y-X intrinsic) rotation to a 3-vector."""
    roll, pitch, yaw = rpy
    x, y, z = vec
    # Rx
    y, z = (y * math.cos(roll) - z * math.sin(roll),
            y * math.sin(roll) + z * math.cos(roll))
    # Ry
    x, z = (x * math.cos(pitch) + z * math.sin(pitch),
            -x * math.sin(pitch) + z * math.cos(pitch))
    # Rz
    x, y = (x * math.cos(yaw) - y * math.sin(yaw),
            x * math.sin(yaw) + y * math.cos(yaw))
    return x, y, z


def optical_to_base(x_opt, y_opt, z_opt, xyz=CAMERA_XYZ, rpy=CAMERA_RPY):
    """
    Optical-frame point -> base_footprint, through the URDF chain.

    camera_optical_frame is the REP-103 frame (x right, y down, z
    forward) that camera_optical_joint's rpy of (-pi/2, 0, -pi/2)
    produces from camera_link, so the inverse is the relabelling below.
    Then camera_link's own pose on base_footprint is applied.

    The extrinsics are arguments defaulted from coco_config rather than
    constants baked in here, so moving the camera is a config edit and
    not a silent, uniformly-wrong offset on every reported target.
    """
    in_link = (z_opt, -x_opt, -y_opt)
    rotated = _rotate_rpy(in_link, rpy)
    return tuple(component + offset for component, offset in zip(rotated, xyz))


def match_depth(stamp_ns, available_ns, tolerance=STAMP_TOLERANCE):
    """
    Pick the depth frame to pair with a colour frame, or None.

    Exact stamp first, then the nearest inside `tolerance`. Pairing a
    fresh image with a stale depth while the robot is moving gives a
    confidently wrong range, so "no answer" has to be reachable.
    """
    if not available_ns:
        return None
    if stamp_ns in available_ns:
        return stamp_ns
    nearest = min(available_ns, key=lambda ns: abs(ns - stamp_ns))
    if abs(nearest - stamp_ns) > tolerance * 1e9:
        return None
    return nearest


def _status_value(key, value):
    if value is None:
        return '--'
    if isinstance(value, (list, tuple)):
        return ','.join(str(v) for v in value) if value else 'none'
    if isinstance(value, bool):
        return '1' if value else '0'
    if isinstance(value, float):
        return f'{value:+.3f}' if key in SIGNED_STATUS_KEYS else f'{value:.3f}'
    return str(value)


def format_status(**fields):
    """
    Build one line of /perception/status.

    Every key is always present, missing values render as '--' rather
    than as an empty string: a blank between two spaces collapses when
    the panel splits on ' ' and shifts every field after it.
    """
    return ' '.join(f'{key}={_status_value(key, fields.get(key))}'
                    for key in STATUS_KEYS)


class TargetFinder(Node):
    """Finds the selected coloured target and reports where it is."""

    def __init__(self):
        super().__init__('target_finder')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('info_topic', '/camera/camera_info')
        self.declare_parameter('colour_topic', '/mission/target_colour')
        self.declare_parameter('target_topic', '/perception/target')
        self.declare_parameter('status_topic', '/perception/status')
        self.declare_parameter('annotated_topic', '/perception/annotated')
        self.declare_parameter('min_range', 0.15)
        self.declare_parameter('max_range', 2.0)
        self.declare_parameter('width_tolerance', WIDTH_TOLERANCE)
        self.declare_parameter('stamp_tolerance', STAMP_TOLERANCE)
        self.declare_parameter('status_hz', STATUS_HZ)
        self.declare_parameter('publish_annotated', True)
        # Which target to look for until the mission says otherwise.
        self.declare_parameter('target_colour', 'blue')

        self._check_numpy()

        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.width_tolerance = float(
            self.get_parameter('width_tolerance').value)
        self.stamp_tolerance = float(
            self.get_parameter('stamp_tolerance').value)
        self.publish_annotated = bool(
            self.get_parameter('publish_annotated').value)

        self.selected = normalise_colour(
            self.get_parameter('target_colour').value) or 'blue'
        self.intrinsics = camera_intrinsics()
        self.have_info = False
        self.bridge = CvBridge()
        self._depth = {}           # stamp_ns -> depth image
        self._status = format_status(sel=self.selected, found=0)

        image_topic = self.get_parameter('image_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        info_topic = self.get_parameter('info_topic').value

        # The bridge republishes camera data with sensor QoS. A RELIABLE
        # subscription simply never matches it -- no error, no warning,
        # just a node that never sees a frame -- so take the answer from
        # the shared table rather than re-deciding it here.
        sensor_qos = QoSProfile(
            depth=1,
            reliability=(ReliabilityPolicy.BEST_EFFORT
                         if is_best_effort(image_topic)
                         else ReliabilityPolicy.RELIABLE))

        self.create_subscription(
            Image, image_topic, self._on_image, sensor_qos)
        self.create_subscription(
            Image, depth_topic, self._on_depth, sensor_qos)
        self.create_subscription(
            CameraInfo, info_topic, self._on_info, sensor_qos)
        self.create_subscription(
            String, self.get_parameter('colour_topic').value,
            self._on_colour, 10)

        self._target_pub = self.create_publisher(
            PointStamped, self.get_parameter('target_topic').value, 10)
        self._status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10)
        # RELIABLE: web_video_server subscribes reliably, and a reliable
        # publisher matches best-effort subscribers too.
        self._annotated_pub = self.create_publisher(
            Image, self.get_parameter('annotated_topic').value, 1)

        self.create_timer(
            1.0 / float(self.get_parameter('status_hz').value),
            self._publish_status)

        fx, _fy, _cx, _cy = self.intrinsics
        self.get_logger().info(
            f'watching {image_topic} '
            f'({"best-effort" if is_best_effort(image_topic) else "reliable"})'
            f' + {depth_topic}, looking for {self.selected!r}')
        self.get_logger().info(
            f'fx={fx:.1f} px from the URDF until {info_topic} arrives; '
            f'range gate {self.min_range:.2f}-{self.max_range:.2f} m')

    def _check_numpy(self):
        # requirements.txt pins numpy<2 because the Jazzy debs are built
        # against the 1.x ABI. numpy 2 breaks cv_bridge at import, and
        # the traceback names neither package.
        if int(np.__version__.split('.')[0]) >= 2:
            self.get_logger().error(
                f'numpy {np.__version__} will break cv_bridge — the ROS 2 '
                f'Jazzy debs are built against the 1.x ABI. '
                f'pip install "numpy<2" (see requirements.txt).')

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
        # '32FC1' explicitly, never 'passthrough': gz's rgbd_camera emits
        # float metres and the conversion should fail loudly if that ever
        # changes rather than reinterpret the bytes.
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

    def _on_image(self, msg):
        # 'bgr8', never 'passthrough'. The bridge delivers rgb8; treating
        # that as BGR and converting to HSV swaps red and blue, which
        # produces a confident detection of the wrong object.
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec

        key = match_depth(stamp_ns, list(self._depth), self.stamp_tolerance)
        depth = None if key is None else self._depth[key]
        age = None if key is None else abs(stamp_ns - key) / 1e9

        seen = [colour for colour in TARGET_COLOURS
                if blob_stats(colour_mask(hsv, colour))[0]]
        found = self._locate(hsv, depth, self.selected)

        if found is None:
            # `seen` IS the wrong-lane signal: arriving one lane over
            # gives sel=yellow found=0 seen=blue, which is a diagnosis
            # rather than a silence.
            self._status = format_status(
                sel=self.selected, found=0, seen=seen, age=age,
                lane=lane_for_colour(self.selected))
        else:
            (area, u, v, width, height, _label), rng, point = found
            self._status = format_status(
                sel=self.selected, found=1, u=round(u), v=round(v),
                area=area, w=width, h=height, range=rng,
                x=point[0], y=point[1], z=point[2],
                lane=lane_for_colour(self.selected), seen=seen, age=age)
            self._publish_target(msg.header.stamp, point)

        if self.publish_annotated:
            self._publish_annotated(msg.header, frame, found)

    # ── the actual measurement ───────────────────────────────────────────
    def _locate(self, hsv, depth, colour):
        """Best blob of `colour` as (blob, range, point), or None."""
        target = target_by_colour(colour)
        if target is None or depth is None:
            return None
        mask = colour_mask(hsv, colour)
        if depth.shape[:2] != mask.shape[:2]:
            self.get_logger().warn(
                f'depth {depth.shape[:2]} does not match colour '
                f'{mask.shape[:2]} — cannot pair them')
            return None
        fx, fy, cx, cy = self.intrinsics
        blobs, labels = blob_stats(mask)
        for blob in blobs:
            _area, u, v, width, _height, label = blob
            surface = robust_depth(depth[labels == label],
                                   self.min_range, self.max_range)
            if surface is None:
                continue
            rng = surface_to_axis(surface, target.diameter / 2.0)
            if not plausible_blob(width, target.diameter, rng, fx,
                                  self.width_tolerance):
                continue
            point = optical_to_base(*deproject(u, v, rng, fx, fy, cx, cy))
            return blob, rng, point
        return None

    def _publish_target(self, stamp, point):
        msg = PointStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'base_footprint'
        msg.point.x, msg.point.y, msg.point.z = point
        self._target_pub.publish(msg)

    def _publish_annotated(self, header, frame, found):
        annotated = frame.copy()
        if found is not None:
            (_area, u, v, width, height, _label), rng, _point = found
            top_left = (int(u - width / 2), int(v - height / 2))
            cv2.rectangle(annotated, top_left,
                          (top_left[0] + width, top_left[1] + height),
                          (0, 255, 0), 1)
            cv2.putText(annotated, f'{self.selected} {rng:.2f}m',
                        (max(0, top_left[0] - 10), max(10, top_left[1] - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        out = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out.header = header
        self._annotated_pub.publish(out)

    def _publish_status(self):
        self._status_pub.publish(String(data=self._status))


def main(args=None):
    rclpy.init(args=args)
    node = TargetFinder()
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
