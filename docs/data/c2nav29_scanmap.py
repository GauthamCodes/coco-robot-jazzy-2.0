#!/usr/bin/env python3
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

"""C2-NAV.29: is the wall-adjacent AMCL bias registration, or the estimator?

OFFLINE. Reads the committed map, the committed world and the C2-NAV.28
traces. Starts no simulator, publishes nothing, changes no parameter.

THE PROBLEM THIS TOOL HAD TO WORK AROUND. The question asked for the
recorded laser scan placed at the ground-truth pose and compared to the
shipped map. That test cannot be run as posed: no raw scan exists.
nav_bench.py subscribes /scan and keeps ``min(good_ranges)`` -- one
scalar per sweep, the ``scan_min`` column -- and there is no rosbag
anywhere in the workspace. c2nav6_stopgeom.py is the only tool that ever
serialised per-beam points and it writes only the <=6 beams inside the
0.25 m stop polygon, with no world pose attached.

What makes a real test possible anyway is that the Gazebo lidar has NO
<noise> block (coco_robo2.xacro), so /scan is a noise-free raycast of
exact world geometry. That geometry is ten axis-aligned boxes and one
cylinder, all full height at the 0.20 m scan plane. So the true scan can
be RECONSTRUCTED, and -- this is the part that keeps it honest -- the
reconstruction is CHECKED against the one real scan statistic that was
recorded. If predicted min-range at the ground-truth pose reproduces the
measured scan_min, then the extrinsics, the frames, the FOV and the
world model are all right, and the reconstructed scan can be trusted for
the parts that were not recorded.

Three tests, in order of how much they lean on reconstruction:

  A  VALIDATION. World-raycast min-range at GT vs measured scan_min.
     Uses only recorded sensor data plus exact world geometry. If this
     fails, nothing below means anything.

  B  MEASURED min-range residual against the MAP, at GT and at AMCL.
     Still uses the recorded scan_min. One number per sample, but it is
     a real measurement of a real surface.

  C  RECONSTRUCTED full-scan likelihood against the MAP, at GT, at AMCL,
     and over a search grid around GT. This is the decisive one: it asks
     where the shipped map's likelihood field puts a PERFECT scan taken
     from the true pose. If that optimum sits on the observed bias, the
     bias is a property of the map, not of the filter.

Test C is a simulation and is labelled as one throughout. It is not
circular: the scan comes from the WORLD, the field comes from the MAP,
and the two are independent artifacts built at different times.

    python3 docs/data/c2nav29_scanmap.py            # all tests
    python3 docs/data/c2nav29_scanmap.py --selftest

Needs numpy, scipy, Pillow. Not a ROS node, not installed by
CMakeLists: it is evidence, not a runtime tool.
"""
import argparse
import csv
import glob
import json
import math
import os
import sys

import numpy as np

from PIL import Image

from scipy.ndimage import distance_transform_edt

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..', '..'))
PGM = os.path.join(REPO, 'gazebo_models', 'maps', 'coco_world.pgm')
MAPYAML = os.path.join(REPO, 'gazebo_models', 'maps', 'coco_world.yaml')
NAV2YAML = os.path.join(REPO, 'gazebo_models', 'config', 'nav2_params.yaml')
SCRATCH = os.path.join(REPO, '.navbench', 'results')
FROZEN = os.path.join(HERE, 'c2nav29_scanmap.json')

# -- the frame convention nav_bench.py USES, and the one map_audit.py
#    MEASURES. The gap between them is a result, not a setting.
NAVBENCH_OFFSET = (2.0, 0.0)          # nav_bench.py WORLD_TO_MAP_X/Y
MEASURED_OFFSET = (2.0560, 0.0150)    # docs/data/map_audit.py, 5 landmarks

# -- lidar, from gazebo_models/urdf/coco_robo2.xacro
LIDAR_XY = (-0.09, 0.10)              # lidar_joint origin on base_link
LIDAR_Z = 0.20
N_BEAMS, A_MIN, A_MAX = 480, -2.0944, 2.0944
R_MIN, R_MAX = 0.15, 12.0
A_INC = (A_MAX - A_MIN) / (N_BEAMS - 1)

# -- world geometry, from gazebo_models/worlds/coco_world.world
# (name, cx, cy, sx, sy). Every one is full height at the 0.20 m scan
# plane: the walls rise to 1.0, the cubes to 0.5, the pilasters to 0.6.
BOXES = [
    ('wall_north', 2.0, 3.5, 12.0, 0.2),
    ('wall_south', 2.0, -3.5, 12.0, 0.2),
    ('wall_west', -4.0, 0.0, 0.2, 7.2),
    ('wall_east', 8.0, 0.0, 0.2, 7.2),
    ('box_obstacle_1', -3.0, 2.4, 0.5, 0.5),
    ('box_obstacle_2', 0.8, -1.4, 0.5, 0.5),
    ('gate_cube_north', -1.1, 1.05, 0.5, 0.5),
    ('gate_cube_south', -1.1, -0.75, 0.5, 0.5),
    ('feature_pilaster_north', 7.72, 2.0, 0.3, 0.5),
    ('feature_pilaster_south', 7.72, -1.4, 0.3, 0.5),
]
CYLINDERS = [('cylinder_obstacle', -0.2, 0.6, 0.2)]

# The ramp is spawned separately (full_world_robo.launch.py) from the
# coco_config constants: foot at x=1.0, run 2.0, width 2.5 centred on
# y=0, 18 deg, then 1.5 m of platform and a mirrored wedge down to 6.5.
# A scan plane at LIDAR_Z only sees the wedge beyond z/tan(18) from each
# foot -- map_audit.py measured that inset independently as +0.575 and
# +0.625 m, against the 0.615 m predicted here.
RAMP_SHADOW = LIDAR_Z / math.tan(math.radians(18.0))
_RX0, _RX1 = 1.0 + RAMP_SHADOW, 6.5 - RAMP_SHADOW
RAMP = ('ramp_at_scan_plane', (_RX0 + _RX1) / 2.0, 0.0, _RX1 - _RX0, 2.5)

# -- thresholds. Every one is fixed by resolution or by the sensor, and
#    is stated here, before any result is looked at.
# A map cell is 0.05 m, so the finest distinction the likelihood field
# can make is one cell: the EDT of a thresholded grid quantises to
# multiples of the cell and reads exactly 0 inside an occupied one.
NEAR_BAND = 0.050            # "explained by the map" = within one cell
# The Gazebo range resolution is 0.01 m and there is no noise, so a
# reconstruction agreeing with the recorded scan_min to better than this
# is agreeing to the limit of what was recorded.
RANGE_RES = 0.01
# The bias under investigation, from C2-NAV.28's four wall_adjacent runs.
REPORTED_BIAS_M = 0.12

SCENARIOS = ('wall_adjacent', 'wall_parallel', 'obstacle_corner',
             'open_space', 'corridor_gate', 'enclosure_entry',
             'enclosure_exit')


# == inputs, read rather than retyped ===================================
def read_map_yaml():
    meta = {}
    with open(MAPYAML) as fh:
        for line in fh:
            if ':' not in line:
                continue
            k, v = line.split(':', 1)
            meta[k.strip()] = v.strip()
    org = [float(v) for v in meta['origin'].strip('[]').split(',')]
    return (float(meta['resolution']), (org[0], org[1]),
            float(meta['occupied_thresh']), int(meta['negate']))


def read_amcl_params():
    """The amcl block as configured, for the record."""
    keys = ('laser_model_type', 'max_beams', 'min_particles',
            'max_particles', 'update_min_d', 'update_min_a', 'sigma_hit',
            'z_hit', 'z_rand', 'laser_likelihood_max_dist',
            'laser_max_range', 'laser_min_range', 'resample_interval',
            'alpha1', 'alpha2', 'alpha3', 'alpha4', 'alpha5',
            'robot_model_type', 'tf_broadcast')
    out, inamcl = {}, False
    with open(NAV2YAML) as fh:
        for line in fh:
            s = line.strip()
            if s.startswith('amcl:'):
                inamcl = True
                continue
            if line[:1] not in (' ', '\t', '\n', '#', '') and s.endswith(':'):
                inamcl = s.startswith('amcl')
            if inamcl:
                for k in keys:
                    if s.startswith(k + ':'):
                        out.setdefault(k, s.split(':', 1)[1].strip())
    return out


# == raycasting the WORLD ===============================================
def _ray_boxes(ox, oy, dx, dy, boxes):
    best = math.inf
    for _, cx, cy, sx, sy in boxes:
        t0, t1, ok = -math.inf, math.inf, True
        for o, d, lo, hi in ((ox, dx, cx - sx / 2.0, cx + sx / 2.0),
                             (oy, dy, cy - sy / 2.0, cy + sy / 2.0)):
            if abs(d) < 1e-12:
                if o < lo or o > hi:
                    ok = False
                    break
            else:
                ta, tb = (lo - o) / d, (hi - o) / d
                if ta > tb:
                    ta, tb = tb, ta
                t0, t1 = max(t0, ta), min(t1, tb)
        if not ok or t0 > t1:
            continue
        t = t0 if t0 > 0 else (t1 if t1 > 0 else None)
        if t is not None and t < best:
            best = t
    return best


def _ray_cyls(ox, oy, dx, dy, cyls):
    best = math.inf
    for _, cx, cy, r in cyls:
        fx, fy = ox - cx, oy - cy
        b = 2.0 * (fx * dx + fy * dy)
        disc = b * b - 4.0 * (fx * fx + fy * fy - r * r)
        if disc < 0:
            continue
        s = math.sqrt(disc)
        for t in ((-b - s) / 2.0, (-b + s) / 2.0):
            if 0 < t < best:
                best = t
                break
    return best


def scan_from_world(lx, ly, lyaw, with_ramp=True, n=N_BEAMS):
    """Noise-free sweep at a lidar pose in WORLD coords.

    inf where the ray leaves the arena or falls outside [R_MIN, R_MAX],
    which is what a real LaserScan reports as an invalid return.
    """
    boxes = BOXES + ([RAMP] if with_ramp else [])
    inc = (A_MAX - A_MIN) / (n - 1)
    out = np.empty(n)
    for i in range(n):
        a = lyaw + A_MIN + i * inc
        dx, dy = math.cos(a), math.sin(a)
        t = min(_ray_boxes(lx, ly, dx, dy, boxes),
                _ray_cyls(lx, ly, dx, dy, CYLINDERS))
        out[i] = t if R_MIN <= t <= R_MAX else math.inf
    return out


def lidar_pose(bx, by, byaw):
    """base_link pose -> lidar pose, same frame. Camera/lidar rpy is 0."""
    ox, oy = LIDAR_XY
    return (bx + ox * math.cos(byaw) - oy * math.sin(byaw),
            by + ox * math.sin(byaw) + oy * math.cos(byaw), byaw)


# == the map's likelihood field =========================================
class LikelihoodField:
    """Distance to the nearest occupied map cell.

    Built exactly as docs/data/c2m5_locrec.py builds it -- threshold the
    pgm, flip to y-up, Euclidean distance transform -- because that is
    the field nav2_amcl scores particles against under
    laser_model_type: likelihood_field.
    """

    def __init__(self, pgm=PGM):
        res, origin, occ_th, negate = read_map_yaml()
        pix = np.array(Image.open(pgm)).astype(np.float64)
        occ = pix / 255.0 if negate else (255.0 - pix) / 255.0
        self.occupied = np.flipud(occ > occ_th)
        self.h, self.w = self.occupied.shape
        self.res, self.x0, self.y0 = res, origin[0], origin[1]
        self.dist = distance_transform_edt(~self.occupied) * res
        self.n_occupied = int(self.occupied.sum())

    def distance(self, xs, ys):
        """Metres to the nearest occupied cell, for MAP-frame points."""
        i = np.floor((np.asarray(ys, float) - self.y0) / self.res)
        j = np.floor((np.asarray(xs, float) - self.x0) / self.res)
        out = np.full(np.shape(i), np.nan)
        ok = (i >= 0) & (i < self.h) & (j >= 0) & (j < self.w)
        out[ok] = self.dist[i[ok].astype(int), j[ok].astype(int)]
        return out

    def raycast(self, lx, ly, lyaw, n=N_BEAMS):
        """Raycast the MAP grid from a lidar pose in MAP coords."""
        inc = (A_MAX - A_MIN) / (n - 1)
        step = self.res * 0.5
        out = np.empty(n)
        for k in range(n):
            a = lyaw + A_MIN + k * inc
            dx, dy = math.cos(a), math.sin(a)
            t, hit = R_MIN, math.inf
            while t <= R_MAX:
                j = int((lx + dx * t - self.x0) / self.res)
                i = int((ly + dy * t - self.y0) / self.res)
                if not (0 <= i < self.h and 0 <= j < self.w):
                    break
                if self.occupied[i, j]:
                    hit = t
                    break
                t += step
            out[k] = hit
        return out

    def score(self, ranges, lx, ly, lyaw, max_beams=None):
        """Endpoint-to-nearest-surface stats for a scan placed at a pose.

        ``ranges`` is a sweep in the sensor's own frame; the pose is the
        lidar in MAP coords. Returns the median, the p90 and the
        fraction inside NEAR_BAND, over valid returns only.
        """
        n = len(ranges)
        idx = np.arange(n)
        if max_beams and max_beams < n:
            idx = np.linspace(0, n - 1, max_beams).astype(int)
        r = np.asarray(ranges, float)[idx]
        good = np.isfinite(r) & (r >= R_MIN) & (r <= R_MAX)
        nan = float('nan')
        if not good.any():
            return dict(n=0, med=nan, p90=nan, frac_near=nan, mean=nan)
        a = lyaw + A_MIN + idx[good] * ((A_MAX - A_MIN) / (n - 1))
        d = self.distance(lx + r[good] * np.cos(a),
                          ly + r[good] * np.sin(a))
        d = d[np.isfinite(d)]
        if d.size == 0:
            return dict(n=0, med=nan, p90=nan, frac_near=nan, mean=nan)
        return dict(n=int(d.size), med=float(np.median(d)),
                    p90=float(np.percentile(d, 90)),
                    frac_near=float((d <= NEAR_BAND).mean()),
                    mean=float(d.mean()))


# == the C2-NAV.28 traces ===============================================
def load_samples(runs=('c2n28_a_r1', 'c2n28_a_r2', 'c2n28_b_r1',
                       'c2n28_focus_r1')):
    """Rows carrying BOTH a fresh /amcl_pose sample and a scan_min.

    The AMCL columns are deliberately not forward-filled by nav_bench
    (see C2-NAV.28): a blank means "no sample in this row's bucket",
    never "the previous sample again". Only rows with a real sample are
    used, so nothing here differences a stale estimate against a moving
    ground truth.
    """
    out = []
    for run in runs:
        for path in sorted(glob.glob(os.path.join(
                SCRATCH, run + '_traces', '*.csv'))):
            scen = os.path.basename(path).rsplit('_rep', 1)[0]
            with open(path) as fh:
                for row in csv.DictReader(fh):
                    if not row.get('amcl_x') or not row.get('scan_min'):
                        continue
                    try:
                        sm = float(row['scan_min'])
                        rec = dict(
                            run=run, scenario=scen,
                            t=float(row['t_rel']),
                            gt_x=float(row['x']), gt_y=float(row['y']),
                            gt_yaw=float(row['yaw']),
                            amcl_x=float(row['amcl_x']),
                            amcl_y=float(row['amcl_y']),
                            amcl_yaw=float(row['amcl_yaw']),
                            scan_min=sm)
                    except ValueError:
                        continue
                    if not math.isfinite(sm):
                        continue
                    out.append(rec)
    return out


def amcl_error(rec, offset):
    """AMCL minus ground truth, in MAP metres, under a stated offset."""
    ex = rec['amcl_x'] - (rec['gt_x'] + offset[0])
    ey = rec['amcl_y'] - (rec['gt_y'] + offset[1])
    return ex, ey, math.hypot(ex, ey)


def med(v):
    return float(np.median(v)) if len(v) else float('nan')


def p90(v):
    return float(np.percentile(v, 90)) if len(v) else float('nan')


def map_min_range(field, lx, ly, lyaw, n=N_BEAMS):
    """First range at which ANY beam meets an occupied map cell.

    Marching t outward and stopping at the first hit gives the minimum
    over the sweep directly, because t only increases. That is the map's
    prediction of the quantity nav_bench actually recorded, ``scan_min``.
    """
    a = lyaw + A_MIN + np.arange(n) * ((A_MAX - A_MIN) / (n - 1))
    ca, sa = np.cos(a), np.sin(a)
    step = field.res * 0.5
    t = R_MIN
    while t <= R_MAX:
        j = np.floor((lx + ca * t - field.x0) / field.res).astype(int)
        i = np.floor((ly + sa * t - field.y0) / field.res).astype(int)
        ok = (i >= 0) & (i < field.h) & (j >= 0) & (j < field.w)
        if ok.any() and field.occupied[i[ok], j[ok]].any():
            return float(t)
        if not ok.any():
            break
        t += step
    return float('inf')


# nav2_amcl's own likelihood-field weight, with the shipped parameters.
AMCL_SIGMA_HIT = 0.2
AMCL_Z_HIT = 0.5
AMCL_Z_RAND = 0.5
AMCL_MAX_BEAMS = 60
AMCL_MAX_DIST = 2.0          # laser_likelihood_max_dist


def amcl_weight(field, ranges, lx, ly, lyaw, max_beams=AMCL_MAX_BEAMS,
                sigma=AMCL_SIGMA_HIT, z_hit=AMCL_Z_HIT,
                z_rand=AMCL_Z_RAND, max_range=R_MAX,
                max_dist=AMCL_MAX_DIST):
    """The particle weight nav2_amcl would assign this scan at this pose.

    LikelihoodFieldModel: per beam pz = z_hit*exp(-d^2/2*sigma^2) +
    z_rand/max_range, accumulated as sum(pz**3). Higher is better. The
    field is truncated at laser_likelihood_max_dist, as amcl truncates
    it. Beams are subsampled to max_beams the way amcl subsamples them.
    """
    n = len(ranges)
    idx = np.linspace(0, n - 1, min(max_beams, n)).astype(int)
    r = np.asarray(ranges, float)[idx]
    good = np.isfinite(r) & (r >= R_MIN) & (r <= R_MAX)
    if not good.any():
        return float('nan')
    a = lyaw + A_MIN + idx[good] * ((A_MAX - A_MIN) / (n - 1))
    d = field.distance(lx + r[good] * np.cos(a), ly + r[good] * np.sin(a))
    d = np.where(np.isfinite(d), d, max_dist)
    d = np.minimum(d, max_dist)
    pz = z_hit * np.exp(-(d * d) / (2.0 * sigma * sigma)) + z_rand / max_range
    return float(np.sum(pz ** 3))


def best_pose(field, ranges, gx, gy, gyaw, span=0.30, step=0.01,
              yaw_span=0.0, yaw_step=0.02):
    """Where the MAP's likelihood field puts this scan, searching around
    a nominal MAP-frame base_link pose. Returns (dx, dy, dyaw, weight).

    The nominal pose is the ground truth. If the optimum sits away from
    it, the map and the scan disagree about where the robot is -- and
    that disagreement is what AMCL is estimating, not a filter defect.
    """
    offs = np.arange(-span, span + step / 2, step)
    yaws = (np.arange(-yaw_span, yaw_span + yaw_step / 2, yaw_step)
            if yaw_span > 0 else np.array([0.0]))
    best = (0.0, 0.0, 0.0, -math.inf)
    for dyaw in yaws:
        for dx in offs:
            for dy in offs:
                lx, ly, lyaw = lidar_pose(gx + dx, gy + dy, gyaw + dyaw)
                w = amcl_weight(field, ranges, lx, ly, lyaw)
                if w > best[3]:
                    best = (float(dx), float(dy), float(dyaw), w)
    return best


# == the selftest: every constant re-derived from its source ============
def _xacro_lidar():
    """lidar_joint origin and the gpu_lidar block, out of the xacro."""
    import re
    path = os.path.join(REPO, 'gazebo_models', 'urdf', 'coco_robo2.xacro')
    txt = open(path).read()
    m = re.search(r'<joint name="lidar_joint".*?<origin xyz="([^"]+)"'
                  r'\s+rpy="([^"]+)"', txt, re.S)
    xyz = [float(v) for v in m.group(1).split()]
    rpy = [float(v) for v in m.group(2).split()]
    blk = re.search(r'<sensor name="lidar".*?</sensor>', txt, re.S).group(0)

    def one(tag):
        return float(re.search(r'<%s>([^<]+)</%s>' % (tag, tag), blk).group(1))
    return dict(xyz=xyz, rpy=rpy, samples=int(one('samples')),
                a_min=one('min_angle'), a_max=one('max_angle'),
                r_min=one('min'), r_max=one('max'),
                noise=('<noise' in blk))


def _world_boxes():
    """(cx, cy, sx, sy) for every static box in the world file."""
    import re
    txt = open(os.path.join(REPO, 'gazebo_models', 'worlds',
                            'coco_world.world')).read()
    out = {}
    for m in re.finditer(r'<model name="([^"]+)">(.*?)</model>', txt, re.S):
        name, body = m.group(1), m.group(2)
        p = re.search(r'<pose>([^<]+)</pose>', body)
        b = re.search(r'<box><size>([^<]+)</size></box>', body)
        c = re.search(r'<cylinder><radius>([^<]+)</radius>', body)
        if p is None:
            continue
        pv = [float(v) for v in p.group(1).split()]
        if b:
            sv = [float(v) for v in b.group(1).split()]
            out[name] = (pv[0], pv[1], sv[0], sv[1])
        elif c:
            out[name] = (pv[0], pv[1], float(c.group(1)))
    return out


def selftest():
    """Offline. Asserts against committed artifacts, not against me."""
    ok = []

    def chk(label, cond, detail=''):
        ok.append(bool(cond))
        print(f"  [{'PASS' if cond else 'FAIL'}] {label}"
              + (f'   {detail}' if detail else ''))

    print('1  lidar geometry, re-read from coco_robo2.xacro')
    lz = _xacro_lidar()
    chk('lidar_joint xyz matches LIDAR_XY/LIDAR_Z',
        tuple(lz['xyz'][:2]) == LIDAR_XY and lz['xyz'][2] == LIDAR_Z,
        f"xacro {lz['xyz']}")
    chk('lidar rpy is (0,0,0) -- no mounting rotation to model',
        lz['rpy'] == [0.0, 0.0, 0.0], f"xacro {lz['rpy']}")
    chk('480 samples over [-2.0944, +2.0944]',
        (lz['samples'], lz['a_min'], lz['a_max'])
        == (N_BEAMS, A_MIN, A_MAX))
    chk('range [0.15, 12.0]', (lz['r_min'], lz['r_max']) == (R_MIN, R_MAX))
    chk('sensor declares NO <noise> -- the scan is a clean raycast',
        not lz['noise'])

    print('2  map metadata, re-read from coco_world.yaml')
    res, org, occ_th, negate = read_map_yaml()
    fld = LikelihoodField()
    chk('resolution 0.05, origin (-2.119, -4.910), negate 0',
        (res, org, negate) == (0.05, (-2.119, -4.91), 0))
    chk('field is 243 x 175 with a nonzero occupied set',
        (fld.w, fld.h) == (243, 175) and fld.n_occupied > 0,
        f'{fld.n_occupied} occupied')
    chk('EDT reads 0 inside an occupied cell',
        float(np.nanmin(fld.dist)) == 0.0)

    print('3  world geometry, re-read from coco_world.world')
    wb = _world_boxes()
    bad = [n for n, cx, cy, sx, sy in BOXES
           if n not in wb or tuple(np.round(wb[n], 6)) != (cx, cy, sx, sy)]
    chk('every modelled box matches the world file', not bad, str(bad))
    chk('the cylinder matches too',
        abs(wb['cylinder_obstacle'][2] - CYLINDERS[0][3]) < 1e-9)
    # ground_plane carries no <pose> and so never enters wb at all; it is
    # a horizontal plane and is invisible to a horizontal scan plane.
    extra = set(wb) - {n for n, *_ in BOXES} - {'cylinder_obstacle'}
    chk('nothing static and posed in the world is left unmodelled',
        extra == set(), str(sorted(extra)))

    print('4  the raycaster, against geometry worked out by hand')
    # From world (0, -3.0) at yaw 0: wall_south's inner face is at
    # y = -3.4, so the beam pointing straight down (-pi/2, inside the
    # 240 deg FOV) must read exactly 0.400 m, and be the sweep minimum.
    s = scan_from_world(0.0, -3.0, 0.0, with_ramp=False)
    k = int(round((-math.pi / 2 - A_MIN) / A_INC))
    chk('beam at -90 deg reads the 0.400 m to wall_south',
        abs(s[k] - 0.4) < 1e-6, f'got {s[k]:.6f}')
    chk('and that wall is the sweep minimum',
        abs(np.nanmin(s) - 0.4) < 1e-6)
    lp = lidar_pose(0.0, 0.0, 0.0)
    chk('lidar_pose at yaw 0 is the raw offset',
        (round(lp[0], 9), round(lp[1], 9)) == LIDAR_XY)
    lp = lidar_pose(0.0, 0.0, math.pi / 2)
    chk('lidar_pose at yaw 90 rotates the offset',
        abs(lp[0] + LIDAR_XY[1]) < 1e-9 and abs(lp[1] - LIDAR_XY[0]) < 1e-9)

    print('5  the map raycaster agrees with the field it is built from')
    lx, ly, lyaw = lidar_pose(0.056, -2.985, 0.0)
    r = fld.raycast(lx, ly, lyaw, n=60)
    g = np.isfinite(r)
    a = lyaw + A_MIN + np.linspace(0, N_BEAMS - 1, 60)[g] * A_INC
    d = fld.distance(lx + r[g] * np.cos(a), ly + r[g] * np.sin(a))
    chk('map-raycast endpoints land on map surfaces',
        np.nanmax(d) <= res * math.sqrt(2) + 1e-9,
        f'worst {np.nanmax(d):.4f} m')

    print('6  the frozen artifact reproduces the headline numbers')
    if os.path.exists(FROZEN):
        doc = json.load(open(FROZEN))
        w = doc['summary']['wall_adjacent']
        chk('wall_adjacent n = 90', w['n'] == 90)
        chk('GT wins every wall_adjacent sample',
            w['C_gt_wins_frac'] == 1.0)
        chk('median endpoint distance at GT is 0.000 m',
            w['C_gt_med'] == 0.0)
        chk('and is materially worse at AMCL',
            w['C_amcl_med'] >= 0.05, f"{w['C_amcl_med']} m")
        chk('the likelihood optimum is nowhere near the observed bias',
            abs(w['C_opt_dy_med']) < 0.5 * abs(w['amcl_dy_med']),
            f"opt dy {w['C_opt_dy_med']} vs observed {w['amcl_dy_med']}")
    else:
        chk('frozen artifact present', False, FROZEN)

    print()
    print(f'selftest: {sum(ok)}/{len(ok)} passed')
    return 0 if all(ok) else 1


def report():
    """Print every table the C2-NAV.29 write-up cites, from the frozen
    artifact, so this survives .navbench being scratch."""
    if not os.path.exists(FROZEN):
        print('no frozen artifact; run the analysis first', file=sys.stderr)
        return 1
    doc = json.load(open(FROZEN))
    print('C2-NAV.29  scan vs map, at ground truth and at AMCL')
    print(f"  frame offset used: {tuple(doc['offset_used'])} "
          f"(nav_bench uses {tuple(doc['offset_navbench'])})")
    print(f"  near band {doc['near_band_m']} m = one map cell; amcl "
          f"{doc['amcl_params'].get('laser_model_type')}, max_beams "
          f"{doc['amcl_params'].get('max_beams')}")
    print()
    hdr = (f"{'scenario':18s} {'n':>4s} | {'A resid':>8s} {'A<1cm':>6s} | "
           f"{'B GT':>6s} {'B AM':>6s} | {'C GTmed':>7s} {'C AMmed':>7s} "
           f"{'GTnear':>6s} {'AMnear':>6s} {'GTwins':>6s} | "
           f"{'optdy':>6s} {'AMCLdy':>7s}")
    print(hdr)
    print('-' * len(hdr))
    for scen in ['ALL'] + list(SCENARIOS):
        s = doc['summary'].get(scen)
        if not s:
            continue
        print(f"{scen:18s} {s['n']:4d} | "
              f"{s['A_world_gt_minus_scanmin_med']:+8.4f} "
              f"{s['A_frac_within_1cm']:6.3f} | "
              f"{s['B_map_gt_resid_absmed']:6.4f} "
              f"{s['B_map_amcl_resid_absmed']:6.4f} | "
              f"{s['C_gt_med']:7.4f} {s['C_amcl_med']:7.4f} "
              f"{s['C_gt_frac_near']:6.3f} {s['C_amcl_frac_near']:6.3f} "
              f"{s['C_gt_wins_frac']:6.3f} | "
              f"{s['C_opt_dy_med']:+6.3f} {s['amcl_dy_med']:+7.3f}")
    print()
    print('  A  world-raycast min at GT minus measured scan_min (validation)')
    print('  B  |map-raycast min minus measured scan_min|, median')
    print('  C  reconstructed GT scan scored on the map likelihood field')
    print('  optdy  where the map puts a PERFECT scan taken from GT')
    print()
    print('SELECTED CASES')
    hdr = (f"{'case':10s} {'run':15s} {'t_s':>6s} {'err':>6s} {'dy':>7s} | "
           f"{'GTmed':>6s} {'AMmed':>6s} {'GTp90':>6s} {'AMp90':>6s} "
           f"{'GTnear':>6s} {'AMnear':>6s} {'nval':>5s} | {'wins':>4s}")
    print(hdr)
    print('-' * len(hdr))
    for c in doc['cases']:
        print(f"{c['case']:10s} {c['run']:15s} {c['t_s']:6.1f} "
              f"{c['amcl_err_m']:6.3f} {c['amcl_dy_m']:+7.3f} | "
              f"{c['gt_med']:6.3f} {c['amcl_med']:6.3f} "
              f"{c['gt_p90']:6.3f} {c['amcl_p90']:6.3f} "
              f"{c['gt_frac_near']:6.3f} {c['amcl_frac_near']:6.3f} "
              f"{c['n_valid']:5d} | {c['better']:>4s}")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--selftest', action='store_true')
    args = ap.parse_args()
    return selftest() if args.selftest else report()


if __name__ == '__main__':
    sys.exit(main())
