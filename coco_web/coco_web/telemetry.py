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
Turning ROS observations into telemetry the browser can render.

Pure module: every function takes plain Python values and returns plain
Python values, so the whole of the telemetry shaping is unit-tested
without a ROS graph. platform_server.py does the subscribing; this does
the thinking.

Two things worth knowing before editing
---------------------------------------
``/cmd_vel_arbiter/status`` and ``/mission/state`` are both
space-separated ``key=value`` lines. That is not incidental -- both nodes
emit that shape specifically so a reader needs no parser, and the old
panel printed them verbatim. Printing them verbatim is fine for a
debugging dashboard and wrong for a product: "the operator should think
'drive COCO up the ramp', not 'publish a Twist'". So the lines are parsed
into fields here, and the UI renders meaning. The raw line is still
carried through, because when something is wrong the raw line is the
thing an engineer wants.

LiDAR is downsampled here rather than in the browser. A full
``sensor_msgs/LaserScan`` is up to ~1800 ranges; at 10 Hz that is a lot of
JSON for a picture roughly 200 points wide. Downsampling at the source is
also the honest place for it: the client cannot choose a rate the server
has not agreed to send.
"""

import math

from coco_web import mission_view

#: ``/grasp/status``'s field order, from ``grasp_server.STATUS_KEYS``.
#: Needed as a table rather than a split because that topic's values can
#: contain spaces -- see ``parse_grasp_status``.
GRASP_KEYS = ('phase', 'colour', 'x', 'y', 'lifted', 'outcome')

#: Ranges kept in a telemetry LiDAR frame. Chosen to be a little denser
#: than a 320 px-wide plot so the picture is not visibly quantised.
LIDAR_POINTS = 240

#: Below this the LiDAR reports nothing useful. C2-NAV.49 measured that
#: `min_scan_m` saturates at this floor in every run, which is why it is
#: useless as a clearance metric -- and why a 0.15 reading here means
#: "at or below the floor", not "0.15 m away".
LIDAR_FLOOR_M = 0.15


def parse_kv_line(line):
    """
    Parse a space-separated ``key=value`` status line into a dict.

    Values stay strings; callers coerce what they care about. Tokens
    without '=' are ignored rather than raising -- these lines are written
    for humans first, and a stray word should not cost a telemetry frame.
    """
    fields = {}
    if not isinstance(line, str):
        return fields
    for token in line.split():
        key, sep, value = token.partition('=')
        if sep and key:
            fields[key] = value
    return fields


def parse_arbiter_status(line):
    """
    Shape ``/cmd_vel_arbiter/status`` into telemetry.

    The interesting field is ``active``: which source currently owns the
    wheels, or None when the arbiter is holding them still. ``--`` is the
    arbiter's spelling for "this source has never published", and it
    becomes None rather than the string, so the UI does not print a dash
    as if it were a value.
    """
    fields = parse_kv_line(line)
    active = fields.get('active') or None
    if active in ('--', 'none', 'None'):
        active = None
    ages = {}
    for source in ('teleop', 'nav', 'rl', 'approach'):
        raw = fields.get(source)
        ages[source] = None if raw in (None, '--') else _as_float(raw)
    return {
        'online': bool(fields),
        'mode': fields.get('mode') or None,
        'active': active,
        'ages': ages,
        'raw': line if isinstance(line, str) else '',
    }


def parse_mission_state(line, colour=None, now=None):
    """
    Shape ``/mission/state`` into telemetry, via ``mission_view``.

    P0.1 read two of the twelve fields that line carries, and looked for
    three -- ``colour``, ``target``, ``detail`` -- that it has never
    carried at all. The colour is on ``/mission/target_colour``, a
    different topic, and arrives here as the ``colour`` argument.

    ``detail`` is kept as an alias of the structured ``reason`` so a
    client written against P0.1 does not lose its status line.
    """
    fields = parse_kv_line(line)
    payload = mission_view.normalise(fields, colour=colour, now=now)
    payload['detail'] = payload['reason']
    payload['raw'] = line if isinstance(line, str) else ''
    return payload


def parse_grasp_status(line):
    """
    Shape ``/grasp/status`` into telemetry, working around its space bug.

    ``/grasp/status`` is the one status topic in this project that
    violates the invariant every parser here relies on: *no value
    contains a space*. Its step labels do -- ``('hover above target',
    ...)``, ``('grasp approach', ...)``, ``('confirm it stayed down',
    ...)`` -- so the wire line reads::

        phase=pick:hover above target colour=blue x=0.15 ...

    and ``parse_kv_line`` yields ``phase='pick:hover'``, silently
    dropping ``above`` and ``target``. Several ``outcome=`` strings are
    the same shape (``'nothing held -- nothing to place'``).

    So ``phase`` and ``outcome`` are sliced out of the RAW line between
    their own key and the next known key, rather than split on spaces.
    Everything else parses normally, because everything else is numeric.
    """
    fields = parse_kv_line(line)
    return {
        'online': bool(fields),
        'phase': _span(line, 'phase', GRASP_KEYS),
        'outcome': _span(line, 'outcome', GRASP_KEYS),
        'colour': _present(fields.get('colour')),
        'lifted': fields.get('lifted') == '1',
        'raw': line if isinstance(line, str) else '',
    }


def _present(value):
    """Return a status-line value, or None for its absent spellings."""
    return None if value in (None, '', '--', 'none', 'None') else value


def _span(line, key, keys):
    """
    Slice one value out of a status line, allowing spaces inside it.

    Ends the value at the next ``<known key>=`` rather than at the next
    space, which is what makes a value containing spaces survive.
    """
    if not isinstance(line, str):
        return None
    marker = key + '='
    start = line.find(marker)
    if start < 0:
        return None
    start += len(marker)
    end = len(line)
    for other in keys:
        if other == key:
            continue
        found = line.find(' ' + other + '=', start)
        if 0 <= found < end:
            end = found
    value = line[start:end].strip()
    return None if value in ('', '--') else value


def parse_perception_status(line):
    """
    Shape ``/perception/status`` into telemetry.

    ``seen`` is the useful field when ``found`` is 0: it names which
    lane's object IS in frame, which is the difference between "the camera
    is blind" and "the camera is looking at the wrong cylinder".
    """
    fields = parse_kv_line(line)
    found = fields.get('found')
    return {
        'online': bool(fields),
        'found': found in ('1', 'true', 'True'),
        'seen': fields.get('seen') or None,
        'raw': line if isinstance(line, str) else '',
    }


def yaw_from_quaternion(x, y, z, w):
    """
    Compute yaw in radians from a quaternion; 0.0 if degenerate.

    Only yaw: the robot is a differential drive on a ramp, and the browser
    draws it from above. Roll and pitch matter to the climb, and the climb
    has its own instrumentation.
    """
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    if siny == 0.0 and cosy == 0.0:
        return 0.0
    return math.atan2(siny, cosy)


def pose_payload(position, orientation):
    """Build the ``robot.pose`` object from a position and quaternion."""
    x, y, z = position
    qx, qy, qz, qw = orientation
    return {
        'x': _as_float(x),
        'y': _as_float(y),
        'z': _as_float(z),
        'yaw': yaw_from_quaternion(
            _as_float(qx), _as_float(qy), _as_float(qz), _as_float(qw)),
    }


def _wrap(angle):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def compose2d(a, b):
    """Return the 2D rigid transform ``a`` followed by ``b``."""
    ax, ay, ath = a
    bx, by, bth = b
    c, s = math.cos(ath), math.sin(ath)
    return (ax + c * bx - s * by, ay + s * bx + c * by, _wrap(ath + bth))


def invert2d(a):
    """Return the inverse of a 2D rigid transform."""
    ax, ay, ath = a
    c, s = math.cos(ath), math.sin(ath)
    return (-(c * ax + s * ay), -(-s * ax + c * ay), _wrap(-ath))


class MapPoseTracker:
    """
    The robot's pose in the MAP frame, at odometry rate, without TF.

    Why this exists: the world view draws the map, the plan and the ramp
    in the map frame, and P0.1 drew the robot at whichever pose arrived
    last -- which, at odometry's rate, was nearly always the ODOMETRY
    pose. Wheel odometry slips on the ramp, so after a climb the robot was
    drawn metres from where it was: measured live at (0.61, 3.71) for a
    robot the executive had just verified home, outside the arena, with
    its LiDAR scattered off the walls.

    Each AMCL pose fixes the map->odom correction, computed against the
    odometry sample nearest the AMCL stamp (not the latest one, which is
    up to a filter-update late); every odometry update is then drawn
    through it. Before the first AMCL pose the correction is the
    identity, which is exact at spawn -- the map origin IS the spawn
    point -- and ``frame`` says ``odom`` so nobody mistakes it for a fix.

    Subscribing to /tf instead would be exact, but /tf carries every joint
    of the arm at high rate, and deserialising all of it in Python to
    draw one arrow is the wrong cost.
    """

    def __init__(self, history=200):
        """Keep the last `history` odometry samples for stamp matching."""
        self._history = []
        self._limit = history
        self._map_from_odom = (0.0, 0.0, 0.0)
        self.localised = False

    @property
    def frame(self):
        """Return 'map' once AMCL has fixed the correction, else 'odom'."""
        return 'map' if self.localised else 'odom'

    def odom(self, stamp, x, y, yaw):
        """Record an odometry pose; return it expressed in the map frame."""
        self._history.append((stamp, (x, y, yaw)))
        if len(self._history) > self._limit:
            del self._history[:len(self._history) - self._limit]
        return compose2d(self._map_from_odom, (x, y, yaw))

    def amcl(self, stamp, x, y, yaw):
        """Fix the correction from a map-frame AMCL pose; return that pose."""
        if self._history:
            _, odom = min(self._history, key=lambda s: abs(s[0] - stamp))
            self._map_from_odom = compose2d((x, y, yaw), invert2d(odom))
        self.localised = True
        return (x, y, _wrap(yaw))


def pose2d_payload(pose, z=0.0):
    """Build ``robot.pose`` from an (x, y, yaw) tuple."""
    x, y, yaw = pose
    return {'x': _as_float(x), 'y': _as_float(y), 'z': _as_float(z),
            'yaw': _as_float(yaw)}


def velocity_payload(linear_x, angular_z):
    """Build the ``robot.velocity`` telemetry object."""
    return {'linear': _as_float(linear_x), 'angular': _as_float(angular_z)}


def downsample_scan(ranges, angle_min, angle_increment, range_max,
                    points=LIDAR_POINTS):
    """
    Reduce a LaserScan to at most `points` (angle, range) samples.

    Takes the MINIMUM of each bucket rather than the mean or a stride
    sample. For an obstacle picture the nearest return in a bucket is the
    one that matters; averaging it against the empty space beside it is
    how a thin obstacle disappears from the plot while still being there.

    Non-finite ranges (a LaserScan's way of saying "no return") and
    anything past `range_max` become None, so the client can draw a gap
    instead of a false wall at max range.
    """
    if not ranges:
        return {'angle_min': 0.0, 'angle_step': 0.0, 'ranges': []}
    total = len(ranges)
    points = max(1, min(int(points), total))
    bucket = total / points
    out = []
    for index in range(points):
        start = int(index * bucket)
        end = max(start + 1, int((index + 1) * bucket))
        best = None
        for value in ranges[start:end]:
            number = _as_float(value, default=None)
            if number is None or not math.isfinite(number):
                continue
            if number <= 0.0 or (range_max and number > range_max):
                continue
            if best is None or number < best:
                best = number
        out.append(None if best is None else round(best, 3))
    step = angle_increment * bucket
    return {
        'angle_min': _as_float(angle_min),
        'angle_step': _as_float(step),
        'ranges': out,
        'floor': LIDAR_FLOOR_M,
    }


def path_payload(points, limit=120):
    """
    Reduce a nav_msgs/Path to at most `limit` (x, y) pairs.

    Evenly strided and always keeping the last point, so the drawn path
    ends where the plan ends rather than short of the goal.
    """
    if not points:
        return []
    if len(points) <= limit:
        return [[round(_as_float(x), 3), round(_as_float(y), 3)]
                for x, y in points]
    stride = len(points) / float(limit)
    out = []
    for index in range(limit):
        x, y = points[int(index * stride)]
        out.append([round(_as_float(x), 3), round(_as_float(y), 3)])
    last_x, last_y = points[-1]
    out[-1] = [round(_as_float(last_x), 3), round(_as_float(last_y), 3)]
    return out


def _as_float(value, default=0.0):
    """Coerce to float, returning `default` for anything that will not."""
    try:
        return float(value)
    except (TypeError, ValueError):
        return default
