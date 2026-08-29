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
target_pose — where the target is, how sure we are, and can the arm get there.

C2-M4.0. This module is the geometry and the policy; `target_pose_node`
is the ROS face. Nothing here imports `rclpy`, `tf2` or any message
type, for the same reason `mission_states.py` does not: the decisions
worth testing are decisions about numbers, and a test that needs a
running graph to reach them is a test nobody runs.

What this module adds over `target_finder`
------------------------------------------
`target_finder` already finds the blob, takes a robust depth over it and
deprojects. It is measured, it feeds the approach servo, and C2-M4.0
does not change it. What it does NOT do, and what manipulation needs:

1. **It has no way to say "detected but I could not measure it".**
   `/perception/target` is simply not published, and silence is
   indistinguishable from a dead node, a wrong colour, or a target in
   deep shadow. Four different faults, one symptom.
2. **It converts to `base_footprint` through a hard-coded transform.**
   `optical_to_base()` re-derives the URDF chain from two constants in
   `coco_config`. That was correct and cheap when the only consumer was
   an approach servo nulling a lateral error, but it is a *second* copy
   of robot geometry, which CLAUDE.md rule 3 exists to forbid. The
   robot already publishes the real chain on `/tf_static`.
3. **It says nothing about whether the arm can act on the answer.**

So: same detection, same depth estimator, TF instead of arithmetic, an
explicit validity, and a reachability verdict.

The target reference point — READ THIS BEFORE COMPARING ANY NUMBER
-------------------------------------------------------------------
A "target position" is meaningless without saying which point on the
target. The targets are vertical cylinders of known height (0.158 m)
and known per-colour diameter, standing on the platform. Two candidate
reference points, and they are not the same:

- **the axis** — the cylinder's vertical centre line. Well posed from
  the blob's *horizontal* centroid, because a cylinder's silhouette is
  symmetric about its axis under any view that sees it side-on. This is
  the point the grasp needs: the arm closes around the axis.
- **the centroid of the visible blob** — moves with framing. The
  cylinder is 158 mm tall against a camera 68.5 mm up, so at working
  range the blob is clipped at the bottom of the image and its vertical
  centroid rides *upward* as the robot closes in. `target_finder`'s
  docstring already records this ("its base leaves the bottom of the
  frame below 0.166 m and its pixel height clips"), which is why its
  plausibility gate uses width and not height.

This module therefore reports, and names, both:

    point       the axis at the height of the visible vertical centroid
    grasp_point the axis at TARGET_GRASP_Z, the band the magnet binds at

`point.x` and `point.y` are the axis and are the numbers to validate.
`point.z` is a *framing-dependent* height on that axis. Manipulation
consumes `grasp_point`, whose z comes from the arm's measured geometry
and not from the camera at all.

**Measured, C2-M4.0, 20 placements, four colours (see RESULTS.md).**
While the whole cylinder is in frame — camera range above ~0.166 m, so
a stand-off above ~0.29 m — the vertical residual is small and the
framing dependence does not bite: `dz` ran **+0.7 to +1.7 mm**. Below
that the cylinder's top leaves the frame, the visible centroid rides
down with it, and `dz` went **-4.3 to -5.7 mm** at a 0.28 m stand-off.
That is the predicted effect, measured, and it is the reason the grasp
takes its height from `TARGET_GRASP_Z` instead.

Depth
-----
The estimator is `target_finder.robust_depth` unchanged — the median of
the finite, in-range depths over the blob's own connected component,
requiring at least 6 of them, then moved from the near surface back to
the axis by `SURFACE_TO_AXIS * radius`. It is kept because it is the
one that was measured, and because the contamination it faces is
one-sided (silhouette pixels carry the *background*, which is further)
and a median resists that up to half the blob.

What is new is that the estimate now carries its own quality:
`valid_fraction` and `spread` (median absolute deviation). Neither
filters anything — they are reported so C2-M4.1 can decide, from data,
whether the median needs replacing. Inventing a trimmed estimator now,
before the residual has been measured once, would be tuning against an
imagined fault.

**The far-field range bias is the 0.8r factor, and it is sub-millimetre.**
For a cylinder the median masked depth sits `r*sqrt(3)/2 = 0.866r` in
front of the axis, not `0.785r`, so a 0.8r correction under-shoots by
0.066r: -0.7 mm for the 20 mm target and -1.1 mm for the 32 mm one.
Measured `dx` over stand-offs 0.35-0.90 m was **-0.4 to -1.5 mm** and
scaled with diameter exactly that way. Left alone: it is inside the
noise of everything downstream and changing it would move a constant
`target_finder` was measured with.

**`min_range` interacts with the target's own radius — measured.** The
gate rejects depths below `min_range`, and a cylinder's near face is a
full radius closer than its axis. At a 0.28 m stand-off (camera range
0.155 m) the near face falls under the 0.15 m default for every colour,
the surviving median is biased away, and `dx` became **+4.1 / +5.5 /
+6.9 / +8.3 mm for d = 20 / 24 / 28 / 32 mm** — a bias proportional to
radius, which is the signature. Re-running the identical placements
with `min_range:=0.11` collapsed it to **-1.0 / -1.0 / -1.3 / -1.4 mm**,
i.e. back to the far-field figure. The default is left at 0.15 to match
`target_finder`; the operating envelope starts at ~0.30 m anyway, and
lowering it is a one-parameter change with the evidence recorded rather
than a silent retune. See RESULTS.md, "C2-M4.0".

Validity
--------
`VALID` is one of five states and the only one that publishes a pose.
The other four are the faults that were previously all silence. There
is no covariance: the simulator gives no basis for calibrating one, and
a fabricated covariance is worse than an honest quality scalar because
downstream code will weight by it.
"""

import math

from coco_config.robot import (approach_stop_x, approach_window,
                               CHASSIS_FRONT_X, GRASP_HOVER_CLEARANCE,
                               GRASP_MAX_LATERAL, GRASP_REACH_X_MAX,
                               GRASP_SELF_COLLISION_X, PALM_MARGIN,
                               target_by_colour, TARGET_GRASP_Z)

import numpy as np


# ── validity ─────────────────────────────────────────────────────────────
# Every one of these was previously "the topic went quiet". They are
# strings rather than an IntEnum because they are published in a
# key=value status line and read by humans in a terminal; an integer
# there would need a decoder ring at exactly the moment something is
# already going wrong.
VALID = 'VALID'
NOT_DETECTED = 'NOT_DETECTED'            # no blob of the selected colour
DEPTH_INVALID = 'DEPTH_INVALID'          # blob found, depth unusable
IMPLAUSIBLE_SIZE = 'IMPLAUSIBLE_SIZE'    # blob + depth, width gate failed
NO_TRANSFORM = 'NO_TRANSFORM'            # TF lookup failed
STALE_TRANSFORM = 'STALE_TRANSFORM'      # TF older than the tolerance

VALIDITIES = (VALID, NOT_DETECTED, DEPTH_INVALID, IMPLAUSIBLE_SIZE,
              NO_TRANSFORM, STALE_TRANSFORM)

# ── reachability ─────────────────────────────────────────────────────────
REACHABLE = 'REACHABLE'
OUT_OF_WORKSPACE = 'OUT_OF_WORKSPACE'    # IK has no solution there
OFF_ARM_PLANE = 'OFF_ARM_PLANE'          # |y| past GRASP_MAX_LATERAL
IK_UNAVAILABLE = 'IK_UNAVAILABLE'        # no solver was supplied
REACH_UNKNOWN = 'UNKNOWN'                # nothing to test (no valid pose)

REACHABILITIES = (REACHABLE, OUT_OF_WORKSPACE, OFF_ARM_PLANE,
                  IK_UNAVAILABLE, REACH_UNKNOWN)

# Fields of /perception/target_pose/status, in order. Space-separated
# key=value, the convention already used by /perception/status,
# /mission/state and grasp_server, so the existing panel splitter works.
# No value may contain a space.
STATUS_KEYS = ('sel', 'validity', 'u', 'v', 'area', 'w',
               'range', 'x', 'y', 'z', 'gx', 'gy', 'gz',
               'qual', 'spread', 'cand', 'seen', 'frame', 'tf_age',
               'reach', 'reach_appr', 'drive', 'lateral')
SIGNED_STATUS_KEYS = ('y', 'gy', 'drive', 'lateral')


class DepthEstimate:
    """
    A robust range over one blob, carrying how much of it was real.

    `surface` is the median of the usable depths, i.e. the range to the
    cylinder's near face. `axis` is that moved back to the centre line.
    `valid_fraction` and `spread` are diagnostics and are never used to
    accept or reject here — see the module docstring.
    """

    __slots__ = ('surface', 'axis', 'valid_px', 'total_px', 'spread')

    def __init__(self, surface, axis, valid_px, total_px, spread):
        self.surface = surface
        self.axis = axis
        self.valid_px = valid_px
        self.total_px = total_px
        self.spread = spread

    @property
    def valid_fraction(self):
        """Share of the blob's pixels that carried a usable depth."""
        return 0.0 if not self.total_px else self.valid_px / self.total_px

    def __repr__(self):
        return (f'DepthEstimate(surface={self.surface}, axis={self.axis}, '
                f'valid={self.valid_px}/{self.total_px}, '
                f'spread={self.spread})')


class TargetObservation:
    """
    One frame's answer about the selected colour.

    `validity` is always set. `point` and `grasp_point` are None unless
    `validity == VALID`, so a consumer that forgets to check gets a
    TypeError rather than a plausible zero — the same reason
    `robust_depth` returns None instead of 0.0.

    `frame_id` is the frame `point` is expressed in and is filled in by
    whoever did the transform. A point without its frame is the defect
    this class exists to make impossible to represent.
    """

    __slots__ = ('colour', 'validity', 'point', 'grasp_point', 'frame_id',
                 'stamp_ns', 'blob', 'depth', 'n_candidates', 'seen',
                 'reach', 'reach_after_approach', 'drive_distance',
                 'tf_age')

    def __init__(self, colour, validity, point=None, grasp_point=None,
                 frame_id=None, stamp_ns=None, blob=None, depth=None,
                 n_candidates=0, seen=(), reach=REACH_UNKNOWN,
                 reach_after_approach=REACH_UNKNOWN, drive_distance=None,
                 tf_age=None):
        if validity not in VALIDITIES:
            raise ValueError(f'unknown validity {validity!r}')
        if validity == VALID and (point is None or frame_id is None):
            raise ValueError('a VALID observation needs a point and a frame')
        self.colour = colour
        self.validity = validity
        self.point = point
        self.grasp_point = grasp_point
        self.frame_id = frame_id
        self.stamp_ns = stamp_ns
        self.blob = blob
        self.depth = depth
        self.n_candidates = n_candidates
        self.seen = tuple(seen)
        self.reach = reach
        self.reach_after_approach = reach_after_approach
        self.drive_distance = drive_distance
        self.tf_age = tf_age

    @property
    def is_valid(self):
        return self.validity == VALID

    def __repr__(self):
        return (f'TargetObservation({self.colour}, {self.validity}, '
                f'point={self.point}, frame={self.frame_id!r}, '
                f'reach={self.reach})')


# ── depth ────────────────────────────────────────────────────────────────
def depth_statistics(values, near, far, radius, min_valid=6):
    """
    Robust range over a blob's depths, with its own quality attached.

    Returns a DepthEstimate, or None when fewer than `min_valid` pixels
    survive. Mirrors `target_finder.robust_depth` exactly for the median
    itself — the same rejection of non-finite and out-of-range samples,
    the same minimum count — and adds the two diagnostics.

    `spread` is the median absolute deviation, not the standard
    deviation: one silhouette pixel carrying the platform's range 0.4 m
    away would dominate an SD computed over ~50 pixels and make a
    perfectly good measurement look terrible.

    The four bad values gz actually writes are all excluded by the same
    comparison chain: +/-inf past the clip planes, NaN where the shader
    missed, 0.0 near the near plane, and anything beyond `far`. A NaN
    fails `>` and `<` silently in Python but not in numpy, so the
    finiteness test comes first and explicitly.
    """
    array = np.asarray(values, dtype=np.float64).ravel()
    total = int(array.size)
    good = array[np.isfinite(array) & (array > near) & (array < far)]
    if good.size < min_valid:
        return None
    surface = float(np.median(good))
    spread = float(np.median(np.abs(good - surface)))
    # Same correction target_finder applies: the mask covers the near
    # face, and averaged across the projected width the offset to the
    # axis is r*pi/4 ~= 0.785r. Skipping it leaves a systematic 8-13 mm
    # bias, which is more than twice the 5.5 mm approach window.
    axis = surface + 0.8 * radius
    return DepthEstimate(surface, axis, int(good.size), total, spread)


# ── geometry ─────────────────────────────────────────────────────────────
def deproject(u, v, z, fx, fy, cx, cy):
    """
    Pixel plus range -> a point in the REP-103 camera optical frame.

    x right, y DOWN, z forward — the frame `camera_optical_joint`'s rpy
    of (-pi/2, 0, -pi/2) produces from `camera_link`, and the frame gz
    stamps the rgbd_camera's output with (`<gz_frame_id>` in the xacro).
    This is deliberately the same three lines as
    `target_finder.deproject`; the point of C2-M4.0 is not to re-derive
    the pinhole model, it is to stop hand-rolling the frame change that
    comes after it.

    `z` here is the range to the point being reported, i.e. the
    *axis-corrected* range. Passing the raw surface range gives a point
    on the cylinder's skin, which is 8-13 mm nearer than the axis and
    the wrong thing to grasp at.
    """
    return ((u - cx) * z / fx, (v - cy) * z / fy, z)


def expected_width_px(diameter, rng, fx):
    """Pixels a cylinder of this diameter subtends at this range."""
    if rng is None or rng <= 0.0:
        return None
    return diameter * fx / rng


def plausible_width(width_px, diameter, rng, fx, tolerance=0.5):
    """
    Whether a blob's width matches the target's at the measured range.

    Width, never height: the cylinder clips at the bottom of the frame
    at working range, so its pixel height is a function of framing. The
    gate is a sanity check on the *range*, not a size classifier — at
    the working distance adjacent diameters differ by ~1.3 px, which is
    inside the segmentation noise on an 8 px blob. Colour says which
    object it is.
    """
    expected = expected_width_px(diameter, rng, fx)
    if expected is None:
        return False
    return abs(width_px - expected) <= tolerance * expected


# ── selection ────────────────────────────────────────────────────────────
def select_candidate(blobs, depth_for_label, target, fx,
                     near, far, tolerance=0.5, min_valid=6):
    """
    Choose which blob of the selected colour is *the* target.

    THE POLICY, stated once so it is not inferred from a loop:

        Among the connected components of the selected colour's mask,
        take them in order of decreasing pixel area, and accept the
        first whose depth is measurable and whose width matches the
        target's diameter at that measured depth.

    Why largest-area and not "first in the array": the array order out
    of `connectedComponentsWithStats` is raster order of the component
    labels, which is a property of where things happen to sit in the
    image. That is the silent, framing-dependent choice §12 warns
    about. Area is a property of the scene.

    Why largest-area is also *nearest* here, which is what a grasp
    wants: all cylinders of one colour are identical (one per colour,
    per `coco_config.TARGETS`), and apparent area of a fixed object
    falls as 1/range^2 monotonically. So for the objects this robot can
    actually meet, "largest" and "nearest" are the same ordering, and
    the ordering that survives a second same-coloured object appearing
    is the one that picks the closer of the two.

    Returns `(blob, DepthEstimate, reason)`. On failure `blob` is None
    and `reason` distinguishes the two ways a *present* blob is
    rejected, because "I can see it but cannot measure it" and "I
    cannot see it" are different faults with different fixes:

        NOT_DETECTED      no components at all
        DEPTH_INVALID     components, none with usable depth
        IMPLAUSIBLE_SIZE  usable depth, but nothing the right size

    `depth_for_label` is a callable taking a component label and
    returning that component's depth samples. Passing a callable rather
    than the depth image keeps this function free of numpy indexing
    conventions and, more usefully, makes it trivial to test with a
    dict.
    """
    if not blobs:
        return None, None, NOT_DETECTED

    radius = target.diameter / 2.0
    saw_depth = False
    for blob in blobs:
        _area, _u, _v, width, _height, label = blob
        estimate = depth_statistics(depth_for_label(label), near, far,
                                    radius, min_valid=min_valid)
        if estimate is None:
            continue
        saw_depth = True
        if not plausible_width(width, target.diameter, estimate.axis, fx,
                               tolerance):
            continue
        return blob, estimate, VALID
    return None, None, (IMPLAUSIBLE_SIZE if saw_depth else DEPTH_INVALID)


# ── reachability ─────────────────────────────────────────────────────────
def classify_reachability(x, y, z, ik=None,
                          hover=GRASP_HOVER_CLEARANCE,
                          max_lateral=GRASP_MAX_LATERAL):
    """
    Decide whether the arm can put its pinch point on this axis.

    `ik` is the solver, injected: `arm_ik.ik_or_none`, or None. It is an
    argument rather than an import because `arm_ik` lives in
    `coco_moveit_config/scripts/` and is installed as an executable, not
    as an importable module — and because IK_UNAVAILABLE has to be a
    state the pipeline can actually be in and report, rather than an
    ImportError at start-up that takes perception down with it.

    Order matters. The lateral test comes first because this arm is
    *planar*: both joints rotate about the base y-axis, so a target off
    the y=0 plane is not "hard to reach", it is unreachable at any joint
    angle, and asking a planar IK about it would get a confident answer
    for a point the gripper will miss sideways.

    Both the grasp height and the hover above it must solve. The
    descent needs both ends in the envelope, and `arm_ik` reaches base-x
    0.16085 at the grasp height but only 0.15651 at the hover — testing
    only the grasp height advertises 4.3 mm of window that cannot be
    entered, which MoveIt reports as an unreachable goal from a
    different cause. `GRASP_REACH_X_MAX` is exactly that measured
    difference and `coco_config`'s test_reach pins it.
    """
    if ik is None:
        return IK_UNAVAILABLE
    if abs(y) > max_lateral:
        return OFF_ARM_PLANE
    if ik(x, z) is None or ik(x, z + hover) is None:
        return OUT_OF_WORKSPACE
    return REACHABLE


def grasp_window_state(x, colour):
    """
    Where the measured axis sits relative to the graspable band.

    Returns `(inside, drive_distance)`. `drive_distance` is how far
    forward the base still has to travel to put the axis at the centre
    of the window — positive means drive forward, and it is the number
    the approach consumes.

    The window is [0.151, 0.1565] for every colour: the self-collision
    bound at 0.150 dominates all four diameters, which is why there is
    one grasp pose here and not four. See `approach_window`.
    """
    window = approach_window(colour)
    stop = approach_stop_x(colour)
    if window is None:
        return False, None
    return (window[0] <= x <= window[1]), x - stop


def reachability_after_approach(x, y, colour, ik=None,
                                grasp_z=TARGET_GRASP_Z,
                                max_lateral=GRASP_MAX_LATERAL):
    """
    Decide whether the approach would leave this target graspable.

    This is the verdict that actually matters at detection range, and
    saying so plainly avoids a misleading headline. Perception first
    sees the target from ~0.3-1.0 m; the arm reaches to base-x 0.157.
    So `classify_reachability` on the *measured* x is OUT_OF_WORKSPACE
    essentially always, and reporting that alone would read as a
    failure when it is just the robot not having driven yet.

    The approach is a straight forward creep, so it changes x and
    leaves y alone. Post-approach x is `approach_stop_x(colour)`, which
    is inside the window by construction. **Therefore the only thing
    perception's measurement decides is y** — whether the robot is
    actually in front of the thing. That makes this function a test of
    the lateral estimate, which is exactly the quantity C2-M4.1's
    benchmark has to bound.
    """
    stop = approach_stop_x(colour)
    if stop is None:
        return REACH_UNKNOWN
    return classify_reachability(stop, y, grasp_z, ik=ik,
                                 max_lateral=max_lateral)


# ── the whole computation, minus ROS ─────────────────────────────────────
def observe(blobs, depth_for_label, colour, intrinsics, near, far,
            seen=(), tolerance=0.5, min_valid=6, grasp_z=TARGET_GRASP_Z):
    """
    Blobs and depths in, a camera-optical-frame TargetObservation out.

    The caller still has to transform `point` into a robot frame and set
    `frame_id`; `transform_observation` does that once a rotation and
    translation are known. Splitting there is deliberate — everything
    above the transform is testable arithmetic, and everything below it
    needs a TF tree.
    """
    target = target_by_colour(colour)
    if target is None:
        return TargetObservation(colour, NOT_DETECTED, seen=seen)

    fx, fy, cx, cy = intrinsics
    blob, estimate, reason = select_candidate(
        blobs, depth_for_label, target, fx, near, far,
        tolerance=tolerance, min_valid=min_valid)
    if blob is None:
        return TargetObservation(colour, reason, seen=seen,
                                 n_candidates=len(blobs))

    _area, u, v, _width, _height, _label = blob
    point = deproject(u, v, estimate.axis, fx, fy, cx, cy)
    return TargetObservation(
        colour, VALID, point=point, frame_id='camera_optical_frame',
        blob=blob, depth=estimate, n_candidates=len(blobs), seen=seen,
        grasp_point=None)


def transform_observation(observation, transform, frame_id,
                          ik=None, grasp_z=TARGET_GRASP_Z, tf_age=None):
    """
    Move a VALID observation into `frame_id` and decide reachability.

    `transform` is any callable mapping a 3-tuple to a 3-tuple. In the
    node it wraps `tf2`'s lookup; in a test it is a lambda. This is the
    seam that keeps the geometry testable without a TF tree, and it is
    also the seam that stops a *second* copy of the robot's extrinsics
    existing in this package — there is nowhere here to put one.

    `grasp_point` is (axis x, axis y, grasp_z). Its z comes from
    `coco_config.TARGET_GRASP_Z`, measured against `/check_state_valid`
    and the verified pick pose, and NOT from the camera: the vertical
    blob centroid is framing-dependent (see the module docstring) and
    the magnet binds at a height the arm's geometry fixes anyway.
    """
    if not observation.is_valid:
        return observation

    x, y, z = transform(observation.point)
    inside, drive = grasp_window_state(x, observation.colour)
    observation.point = (x, y, z)
    observation.grasp_point = (x, y, grasp_z)
    observation.frame_id = frame_id
    observation.tf_age = tf_age
    observation.reach = classify_reachability(x, y, grasp_z, ik=ik)
    observation.reach_after_approach = reachability_after_approach(
        x, y, observation.colour, ik=ik, grasp_z=grasp_z)
    observation.drive_distance = drive
    return observation


# ── status line ──────────────────────────────────────────────────────────
def _status_value(key, value):
    if value is None:
        return '--'
    if isinstance(value, (list, tuple)):
        return ','.join(str(v) for v in value) if value else 'none'
    if isinstance(value, bool):
        return '1' if value else '0'
    if isinstance(value, float):
        if not math.isfinite(value):
            return '--'
        return f'{value:+.4f}' if key in SIGNED_STATUS_KEYS else f'{value:.4f}'
    return str(value)


def format_status(**fields):
    """
    One line of /perception/target_pose/status.

    Every key always present, missing values '--' rather than empty: a
    blank between two spaces collapses when a panel splits on ' ' and
    silently shifts every field after it. Same rule, and the same
    reason, as `target_finder.format_status`.
    """
    return ' '.join(f'{key}={_status_value(key, fields.get(key))}'
                    for key in STATUS_KEYS)


def status_for(observation):
    """Render a TargetObservation as its status line."""
    blob = observation.blob
    depth = observation.depth
    point = observation.point
    grasp = observation.grasp_point
    return format_status(
        sel=observation.colour,
        validity=observation.validity,
        u=None if blob is None else round(blob[1]),
        v=None if blob is None else round(blob[2]),
        area=None if blob is None else blob[0],
        w=None if blob is None else blob[3],
        range=None if depth is None else depth.axis,
        x=None if point is None else point[0],
        y=None if point is None else point[1],
        z=None if point is None else point[2],
        gx=None if grasp is None else grasp[0],
        gy=None if grasp is None else grasp[1],
        gz=None if grasp is None else grasp[2],
        qual=None if depth is None else depth.valid_fraction,
        spread=None if depth is None else depth.spread,
        cand=observation.n_candidates,
        seen=observation.seen,
        frame=observation.frame_id,
        tf_age=observation.tf_age,
        reach=observation.reach,
        reach_appr=observation.reach_after_approach,
        drive=observation.drive_distance,
        lateral=None if point is None else point[1])


def finder_status_fields(observation):
    """
    Map a TargetObservation onto `target_finder`'s /perception/status fields.

    C2-M4.2. This exists so `target_pose_node` can stand exactly where
    `target_finder` stood for the *mission executive*, not just for the
    approach. `mission_states._check_search_target` gates SEARCH_TARGET
    on `/perception/status` reading `found=1` with a matching `sel`, and
    that topic and those two keys are the whole of what it reads. The
    richer per-frame answer stays on `/perception/target_pose/status`,
    which this does not touch.

    The geometry is gated on `is_valid` deliberately, mirroring
    `target_finder`, which emits `u v area w h range x y z` only when it
    has a fix and leaves them `--` otherwise. A compat line whose whole
    purpose is to be substitutable should be substitutable in behaviour
    and not merely in field names.

    `lane` and `age` are always None, and so always render `--`.
    `target_finder` fills them from `lane_for_colour` and from the
    colour/depth stamp pairing; neither is a quantity this pipeline
    computes, and rendering a plausible number for one it does not have
    is the failure the `--` convention exists to prevent. Nothing reads
    either — the executive reads `sel` and `found`, the HUD reads `sel`
    and `range`.

    Returns a plain dict so this stays pure: the caller passes it to
    `target_finder.format_status`, which keeps exactly one definition of
    the /perception/status field order and rendering.
    """
    valid = observation.is_valid
    blob = observation.blob if valid else None
    depth = observation.depth if valid else None
    point = observation.point if valid else None
    return {
        'sel': observation.colour,
        # bool: format_status renders True as '1', which is what
        # _check_search_target compares against.
        'found': valid,
        'u': None if blob is None else round(blob[1]),
        'v': None if blob is None else round(blob[2]),
        'area': None if blob is None else blob[0],
        'w': None if blob is None else blob[3],
        'h': None if blob is None else blob[4],
        # `range` is the axis range in both pipelines: target_finder
        # passes surface_to_axis()'s output and DepthEstimate.axis is
        # the same correction. Like-for-like, not merely same-named.
        'range': None if depth is None else depth.axis,
        'x': None if point is None else point[0],
        'y': None if point is None else point[1],
        'z': None if point is None else point[2],
        'seen': observation.seen,
        'lane': None,
        'age': None,
    }


# Re-exported so a reader of this module can see the bounds the
# reachability verdict rests on without chasing them into coco_config.
BOUNDS = {
    'chassis_front_x': CHASSIS_FRONT_X,
    'palm_margin': PALM_MARGIN,
    'self_collision_x': GRASP_SELF_COLLISION_X,
    'reach_x_max': GRASP_REACH_X_MAX,
    'grasp_z': TARGET_GRASP_Z,
    'hover': GRASP_HOVER_CLEARANCE,
    'max_lateral': GRASP_MAX_LATERAL,
}
