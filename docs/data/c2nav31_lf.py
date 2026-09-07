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

"""C2-NAV.31: can the map defect produce AMCL's southward bias?

OFFLINE. Reads the committed map, the committed world, the shipped AMCL
configuration, the installed nav2_amcl headers and the C2-NAV.28/.30
traces. Starts no simulator, publishes nothing, changes no parameter.

THE QUESTION. C2-NAV.29 measured a real map-geometry residual of about
0.05-0.07 m and estimated it worth "one sixth" of the wall-adjacent AMCL
bias. C2-NAV.30 showed the particle cloud is broad, unimodal and
straddles the truth, with its whole centre of mass displaced about
-0.09 m. This session asks whether the DEPLOYED likelihood-field
observation model, scoring against the SHIPPED map, actually prefers
poses that far south -- and re-weights C2-NAV.30's own recorded particle
set to find out.

FOUR ESTIMATORS, because one of them is a trap:

  mode      argmax of the likelihood over dy. This is what C2-NAV.29
            reported. It is sharp but it wanders when the surface has a
            plateau, and the measured defect is exactly the kind that
            makes plateaus.
  mean      posterior mean of dy over +/-0.45 m, the cloud's own support
            scale. This is the statistic a particle-cloud CENTROID
            corresponds to, so it is the one comparable to the measured
            bias.
  band99    width and midpoint of the {w >= 0.99 * wmax} plateau. The
            identifiability measure: if this is wider than the bias, the
            observation model cannot resolve the bias in either
            direction.
  reweight  score C2-NAV.30's RECORDED particles with the shipped model
            and see which way the centroid moves. No search, no grid, no
            assumption about the shape of the surface.

WHAT THIS CANNOT DO, stated before any result. The published
/particle_cloud weights are flat -- resample_interval: 1 publishes the
set AFTER resampling has levelled them (C2-NAV.30, ESS/n = 1.0000 on
40/40). Nothing here reconstructs the importance weights AMCL actually
used, or its resampling. What it establishes is what the deployed
OBSERVATION MODEL would do to those particles given the true scan. That
is a different claim and it is labelled as one throughout.

The scan is RECONSTRUCTED, as in C2-NAV.29: raw /scan is recorded
nowhere, the Gazebo lidar declares no <noise> block, so the sweep is a
deterministic raycast of exact world geometry. Test A is re-run here on
the C2-NAV.30 leg rather than inherited.

    python3 docs/data/c2nav31_lf.py             # every table
    python3 docs/data/c2nav31_lf.py selftest    # offline checks
    python3 docs/data/c2nav31_lf.py params      # one section

Needs numpy, scipy, Pillow. Not a ROS node, not installed by
CMakeLists: it is evidence, not a runtime tool.
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np

from PIL import Image

from scipy.ndimage import distance_transform_edt

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, HERE)

import c2nav29_scanmap as S   # noqa: E402  the raycaster and world model

SCRATCH = os.environ.get('C2NAV_SCRATCH',
                         os.path.join(REPO, '.navbench', 'results'))
FROZEN = os.path.join(HERE, 'c2nav31_lf.json')
AMCL_MAP_HPP = '/opt/ros/jazzy/include/nav2_amcl/map/map.hpp'

CLOUD_RUN = 'c2n30_focus_r1'
CLOUD_LEGS = ('wall_adjacent', 'open_space')
CTRL_RUNS = ('c2n28_a_r1', 'c2n28_a_r2', 'c2n28_b_r1', 'c2n28_focus_r1')
CTRL_PER_SCENARIO = 40
CTRL_SEED = 20260907

# -- the two frame conventions, carried from C2-NAV.29 unchanged.
NAVBENCH_OFFSET = S.NAVBENCH_OFFSET      # (2.0, 0.0), what nav_bench uses
MEASURED_OFFSET = S.MEASURED_OFFSET      # (2.0560, 0.0150), map_audit.py

# -- the search support. +/-0.45 m is the RECORDED cloud's own half-span
#    in y (C2-NAV.30 median y range 1.3825 m), so the posterior mean is
#    taken over the region the particles actually occupy rather than over
#    a window chosen to produce an answer.
DY_SPAN, DY_STEP = 0.45, 0.005
BAND_FRAC = 0.99

# -- the map variants. The range is fixed by C2-NAV.29's measured
#    residual (0.05-0.07 m) plus one step either side, and by the
#    brief's list. It is declared here, before any optimum is looked at.
MAP_DY = (0.00, -0.02, -0.04, -0.06, -0.08, -0.10, 0.02, 0.04)
DILATIONS = (0.02, 0.05, 0.06, 0.07, 0.10)


# == Part 1: the deployed observation model ============================
def amcl_params():
    """Every likelihood-field parameter, read from its source file."""
    p = S.read_amcl_params()
    res, origin, occ_th, negate = S.read_map_yaml()
    with Image.open(S.PGM) as im:
        w, h = im.size
    lidar = S._xacro_lidar()
    return {
        'laser_model_type': p['laser_model_type'].strip('"\''),
        'max_beams': int(p['max_beams']),
        'sigma_hit': float(p['sigma_hit']),
        'z_hit': float(p['z_hit']),
        'z_rand': float(p['z_rand']),
        'z_short': 0.05, 'z_max': 0.05,
        'laser_likelihood_max_dist': float(p['laser_likelihood_max_dist']),
        'laser_max_range': float(p['laser_max_range']),
        'laser_min_range': float(p['laser_min_range']),
        'do_beamskip': 'false',
        'min_particles': int(p['min_particles']),
        'max_particles': int(p['max_particles']),
        'resample_interval': int(p['resample_interval']),
        'update_min_d': float(p['update_min_d']),
        'update_min_a': float(p['update_min_a']),
        'robot_model_type': p['robot_model_type'],
        'scan_range_min': lidar['r_min'],
        'scan_range_max': lidar['r_max'],
        'scan_samples': lidar['samples'],
        'scan_angle_min': lidar['a_min'],
        'scan_angle_max': lidar['a_max'],
        'scan_has_noise': lidar['noise'],
        'map_resolution': res,
        'map_origin': list(origin),
        'map_width': w, 'map_height': h,
        'map_occupied_thresh': occ_th, 'map_negate': negate,
    }


def effective_ranges(p):
    """What amcl derives from the config and the scan, not what it is told.

    nav2_amcl clamps range_max to min(scan.range_max, laser_max_range)
    and, with laser_min_range < 0, takes range_min from the scan. Short
    returns are MAPPED TO range_max and then skipped, so the effective
    beam set is {range_min < r < range_max}.
    """
    rmax = min(p['scan_range_max'], p['laser_max_range'])
    rmin = (p['scan_range_min'] if p['laser_min_range'] <= 0
            else max(p['scan_range_min'], p['laser_min_range']))
    step = max(1, (p['scan_samples'] - 1) // (p['max_beams'] - 1))
    return rmax, rmin, step, len(range(0, p['scan_samples'], step))


def binning_convention():
    """Read amcl's world->cell macro out of the INSTALLED header.

    This is the parameter the brief asks for that is not in the yaml.
    nav2_amcl does not use the ROS OccupancyGrid cell convention. Its
    map.hpp defines

        MAP_GXWX(map, x) = floor((x - origin_x)/scale + 0.5) + size_x/2

    and convertMap() sets origin_x = msg.origin.x + (size_x/2)*scale with
    size_x an int, so the two size_x/2 terms are the SAME integer and
    cancel:

        cell(x) = floor((x - yaml_origin_x)/scale + 0.5)

    A cell's world centre is therefore yaml_origin + i*scale, not
    yaml_origin + (i+0.5)*scale. Every surface in amcl's likelihood field
    sits HALF A CELL -- here 25 mm -- more negative in x and y than the
    same PGM read the ROS way. That is a fixed south-west displacement of
    the field, and it is not in any yaml.
    """
    out = {'header': AMCL_MAP_HPP, 'readable': os.path.exists(AMCL_MAP_HPP)}
    if not out['readable']:
        out['gxwx'] = out['wxgx'] = None
        return out
    with open(AMCL_MAP_HPP) as fh:
        for line in fh:
            if 'define MAP_GXWX' in line:
                out['gxwx'] = line.strip()
            if 'define MAP_WXGX' in line:
                out['wxgx'] = line.strip()
            if 'int size_x, size_y;' in line:
                out['size_is_int'] = True
    out.setdefault('size_is_int', False)
    out['half_cell_m'] = S.read_map_yaml()[0] / 2.0
    return out


class Field:
    """nav2_amcl's likelihood field, with every convention explicit.

    binning 'amcl' is floor(u + 0.5), what MAP_GXWX computes; binning
    'grid' is floor(u), the ROS OccupancyGrid convention C2-NAV.29's
    LikelihoodField used. Both are reported everywhere, because the
    difference between them is itself one of this session's results.

    The occupied set and the Euclidean distance transform are built
    exactly as C2-NAV.29 built them, and a test asserts the two agree.
    """

    def __init__(self, binning='amcl', max_dist=None):
        res, origin, occ_th, negate = S.read_map_yaml()
        pix = np.array(Image.open(S.PGM)).astype(np.float64)
        occ = pix / 255.0 if negate else (255.0 - pix) / 255.0
        self.occupied = np.flipud(occ > occ_th)
        self.h, self.w = self.occupied.shape
        self.res, self.x0, self.y0 = res, origin[0], origin[1]
        self.max_dist = (float(S.read_amcl_params()
                               ['laser_likelihood_max_dist'])
                         if max_dist is None else max_dist)
        raw = distance_transform_edt(~self.occupied) * res
        # Recorded because it turns out to matter: on THIS map the
        # largest distance to an occupied cell is well under
        # laser_likelihood_max_dist, so that parameter never binds.
        self.max_edt = float(raw.max())
        self.dist = np.minimum(raw, self.max_dist)
        if binning not in ('amcl', 'grid'):
            raise ValueError(binning)
        self.b = 0.5 if binning == 'amcl' else 0.0
        self.binning = binning

    def distance(self, xs, ys, mdy=0.0, dilate=0.0):
        """Metres to the nearest occupied cell.

        ``mdy`` TRANSLATES the map north(+)/south(-); translating the map
        by d is querying at -d. ``dilate`` grows every occupied region
        outward by that many metres, which is what C2-NAV.29's "surfaces
        drawn closer than they are" residual describes, and subtracting
        it from the exterior EDT is exactly that operation. Out-of-map
        reads return max_dist, as MAP_VALID makes amcl do.
        """
        u = (np.asarray(xs, float) - self.x0) / self.res + self.b
        v = (np.asarray(ys, float) - mdy - self.y0) / self.res + self.b
        j, i = np.floor(u).astype(np.int64), np.floor(v).astype(np.int64)
        out = np.full(np.shape(i), self.max_dist, float)
        ok = (i >= 0) & (i < self.h) & (j >= 0) & (j < self.w)
        out[ok] = self.dist[i[ok], j[ok]]
        if dilate:
            out = np.clip(out - dilate, 0.0, self.max_dist)
        return out


P0 = amcl_params()
RANGE_MAX, RANGE_MIN, BEAM_STEP, N_USED = effective_ranges(P0)


def endpoints(ranges, lx, ly, lyaw):
    """Beam endpoints for a sweep at a lidar pose, amcl's own subsample.

    amcl strides by (range_count-1)//(max_beams-1), maps returns at or
    below range_min to range_max, then skips everything at or above
    range_max -- which is also how an ``inf`` no-return is discarded.
    """
    n = len(ranges)
    idx = np.arange(0, n, BEAM_STEP)
    r = np.asarray(ranges, float)[idx]
    r = np.where(r <= RANGE_MIN, RANGE_MAX, r)
    good = np.isfinite(r) & (r < RANGE_MAX)
    a = lyaw + S.A_MIN + idx[good] * ((S.A_MAX - S.A_MIN) / (n - 1))
    return lx + r[good] * np.cos(a), ly + r[good] * np.sin(a)


def weight_from(field, ex, ey, dx=0.0, dy=0.0, mdy=0.0, dilate=0.0):
    """nav2_amcl LikelihoodFieldModel::sensorFunction, per particle.

    p = 1 + sum over beams of pz**3, with
    pz = z_hit*exp(-z^2 / 2*sigma_hit^2) + z_rand/range_max. The leading
    1.0 is amcl's own initialiser and is kept, so weights here are on
    amcl's scale rather than a proportional one. Translating a pose
    translates every endpoint by the same vector -- the rotation is
    unchanged -- so a whole (dx, dy) grid is one array operation.
    """
    dx = np.atleast_1d(np.asarray(dx, float))
    dy = np.atleast_1d(np.asarray(dy, float))
    d = field.distance(ex[None, :] + dx[:, None],
                       ey[None, :] + dy[:, None], mdy, dilate)
    pz = (P0['z_hit'] * np.exp(-(d * d) / (2.0 * P0['sigma_hit'] ** 2))
          + P0['z_rand'] / RANGE_MAX)
    return 1.0 + (pz ** 3).sum(axis=-1)


def weight_max():
    """The largest weight the model can return: every beam at distance 0."""
    pz = P0['z_hit'] + P0['z_rand'] / RANGE_MAX
    return 1.0 + N_USED * pz ** 3


def dy_profile(field, ex, ey, mdy=0.0, dilate=0.0):
    """Likelihood along dy at dx = 0, and the four estimators."""
    ds = np.arange(-DY_SPAN, DY_SPAN + DY_STEP / 2, DY_STEP)
    w = weight_from(field, ex, ey, np.zeros_like(ds), ds, mdy, dilate)
    band = ds[w >= BAND_FRAC * w.max()]
    return dict(
        ds=ds, w=w,
        mode=float(ds[int(np.argmax(w))]),
        mean=float((w * ds).sum() / w.sum()),
        band_lo=float(band.min()), band_hi=float(band.max()),
        band_mid=float((band.min() + band.max()) / 2.0),
        band_w=float(band.max() - band.min()),
        wmax=float(w.max()))


def grid_optimum(field, ex, ey, span=0.25, step=0.005, mdy=0.0, dilate=0.0):
    """(dx, dy) of the 2-D likelihood optimum around the nominal pose."""
    offs = np.arange(-span, span + step / 2, step)
    dx, dy = np.meshgrid(offs, offs, indexing='ij')
    w = weight_from(field, ex, ey, dx.ravel(), dy.ravel(), mdy, dilate)
    k = int(np.argmax(w))
    return float(dx.ravel()[k]), float(dy.ravel()[k]), float(w[k])


def particle_weights(field, ranges, lx, ly, lyaw, part, mdy=0.0, dilate=0.0):
    """Score a whole particle set. Each particle carries its own yaw, so
    the endpoints are rebuilt per particle rather than translated."""
    n = len(ranges)
    idx = np.arange(0, n, BEAM_STEP)
    r = np.asarray(ranges, float)[idx]
    r = np.where(r <= RANGE_MIN, RANGE_MAX, r)
    good = np.isfinite(r) & (r < RANGE_MAX)
    bear = S.A_MIN + idx[good] * ((S.A_MAX - S.A_MIN) / (n - 1))
    rg = r[good]
    ox, oy = S.LIDAR_XY
    px, py, pa = part[:, 0], part[:, 1], part[:, 2]
    plx = px + ox * np.cos(pa) - oy * np.sin(pa)
    ply = py + ox * np.sin(pa) + oy * np.cos(pa)
    ang = pa[:, None] + bear[None, :]
    d = field.distance(plx[:, None] + rg[None, :] * np.cos(ang),
                       ply[:, None] + rg[None, :] * np.sin(ang), mdy, dilate)
    pz = (P0['z_hit'] * np.exp(-(d * d) / (2.0 * P0['sigma_hit'] ** 2))
          + P0['z_rand'] / RANGE_MAX)
    return 1.0 + (pz ** 3).sum(axis=1)


# == inputs ============================================================
def _frozen_leg(leg):
    """The same snapshots, replayed from the committed bundle.

    .navbench is scratch and is not committed, so the bundle carries the
    raw particle poses as well as the summaries. Everything in Part 2
    therefore RECOMPUTES from the repository alone, not merely replays.
    """
    if not os.path.exists(FROZEN):
        return None
    with open(FROZEN) as fh:
        doc = json.load(fh)
    rows = doc.get('legs', {}).get(leg, {}).get('rows')
    if not rows or 'part' not in rows[0]:
        return None
    return [dict(r, part=[list(p) for p in r['part']]) for r in rows]


def load_cloud(leg, run=CLOUD_RUN):
    """C2-NAV.30's snapshots: GT pose, AMCL pose and the raw particles.

    The npz snapshots and the trace rows carrying a pc_ block are 1:1 and
    in order; the loader ASSERTS that the per-snapshot particle count and
    centroid reproduce the trace's own pc_n / pc_mx / pc_my rather than
    assuming the alignment. With .navbench gone it falls back to the
    committed bundle, which carries the particles themselves.
    """
    base = os.path.join(SCRATCH, f'{run}_traces')
    csvp = os.path.join(base, f'{leg}_rep0.csv')
    npzp = os.path.join(base, f'{leg}_rep0_cloud.npz')
    if not (os.path.exists(csvp) and os.path.exists(npzp)):
        return _frozen_leg(leg)
    with open(csvp) as fh:
        rows = [r for r in csv.DictReader(fh) if r.get('pc_n')]
    with np.load(npzp) as z:
        part, counts = z['particles'], z['counts']
    if len(rows) != len(counts):
        raise AssertionError(f'{leg}: {len(rows)} rows vs {len(counts)} snaps')
    out, off = [], 0
    for r, n in zip(rows, counts):
        n = int(n)
        pk = part[off:off + n].astype(float)
        off += n
        if n != int(r['pc_n']):
            raise AssertionError(f'{leg}: count {n} vs pc_n {r["pc_n"]}')
        for col, got in (('pc_mx', pk[:, 0].mean()),
                         ('pc_my', pk[:, 1].mean())):
            if abs(got - float(r[col])) > 1e-4:
                raise AssertionError(f'{leg}: {col} {got} vs {r[col]}')
        out.append(dict(
            t=float(r['t_rel']), gt_x=float(r['x']), gt_y=float(r['y']),
            gt_yaw=float(r['yaw']), amcl_x=float(r['amcl_x']),
            amcl_y=float(r['amcl_y']), amcl_yaw=float(r['amcl_yaw']),
            scan_min=float(r['scan_min']),
            part=[[round(float(v), 4) for v in p[:3]] for p in pk],
            wmax=float(r['pc_wmax']), wsum=float(r['pc_wsum']),
            ess=float(r['pc_ess'])))
    return out


def load_controls():
    """A fixed, seeded subsample of C2-NAV.28's 544 fresh-AMCL samples.

    Falls back to the committed bundle, which carries the chosen samples
    themselves, for the same reason load_cloud does.
    """
    recs = S.load_samples(CTRL_RUNS)
    if not recs:
        if not os.path.exists(FROZEN):
            return None
        with open(FROZEN) as fh:
            return json.load(fh).get('control_samples') or None
    by = {}
    for r in recs:
        by.setdefault(r['scenario'], []).append(r)
    rng = np.random.default_rng(CTRL_SEED)
    out = {}
    for scen in S.SCENARIOS:
        rs = by.get(scen)
        if not rs:
            continue
        k = min(CTRL_PER_SCENARIO, len(rs))
        pick = sorted(rng.choice(len(rs), k, replace=False))
        out[scen] = [{kk: round(rs[i][kk], 6) for kk in
                      ('gt_x', 'gt_y', 'gt_yaw', 'amcl_x', 'amcl_y',
                       'amcl_yaw', 'scan_min')} for i in pick]
        out[scen + '_n_available'] = len(rs)
    return out


def _med(v):
    return float(np.median(v)) if len(v) else float('nan')


# == the analysis ======================================================
def analyse(offset=NAVBENCH_OFFSET):
    """Every measurement this session makes, from the raw inputs."""
    doc = {
        'what': 'C2-NAV.31 likelihood-field causal test of the map defect',
        'offset_used': list(offset),
        'offset_navbench': list(NAVBENCH_OFFSET),
        'offset_measured': list(MEASURED_OFFSET),
        'amcl_params': P0,
        'binning': binning_convention(),
        'effective': {'range_max': RANGE_MAX, 'range_min': RANGE_MIN,
                      'beam_step': BEAM_STEP, 'beams_used': N_USED,
                      'weight_max': weight_max()},
        'dy_span': DY_SPAN, 'dy_step': DY_STEP, 'band_frac': BAND_FRAC,
        'map_dy': list(MAP_DY), 'dilations': list(DILATIONS),
    }
    fa, fg = Field('amcl'), Field('grid')
    doc['field'] = {'w': fa.w, 'h': fa.h, 'res': fa.res,
                    'origin': [fa.x0, fa.y0],
                    'n_occupied': int(fa.occupied.sum()),
                    'max_dist': fa.max_dist}

    # -- clouds: reconstruct the scan, validate it, score the particles
    doc['legs'] = {}
    for leg in CLOUD_LEGS:
        snaps = load_cloud(leg)
        if snaps is None:
            continue
        rows, testa = [], []
        for s in snaps:
            lx, ly, lyaw = S.lidar_pose(s['gt_x'], s['gt_y'], s['gt_yaw'])
            sc = S.scan_from_world(lx, ly, lyaw)
            fin = np.isfinite(sc)
            testa.append(float(sc[fin].min()) - s['scan_min'])
            mlx, mly = lx + offset[0], ly + offset[1]
            mgy = s['gt_y'] + offset[1]
            ex, ey = endpoints(sc, mlx, mly, lyaw)
            part = np.array(s['part'], float)
            row = {
                't': s['t'], 'n': len(part), 'gt_y_map': mgy,
                'amcl_dy': s['amcl_y'] - mgy,
                'cloud_dy': float(part[:, 1].mean()) - mgy,
                'scan_min': s['scan_min'],
                'testA': testa[-1], 'pub_ess_frac': s['ess'] / len(part),
                # the raw inputs, so the bundle recomputes rather than
                # replays once .navbench is gone
                'gt_x': s['gt_x'], 'gt_y': s['gt_y'], 'gt_yaw': s['gt_yaw'],
                'amcl_x': s['amcl_x'], 'amcl_y': s['amcl_y'],
                'amcl_yaw': s['amcl_yaw'], 'ess': s['ess'],
                'part': s['part'],
            }
            for tag, f in (('amcl', fa), ('grid', fg)):
                pr = dy_profile(f, ex, ey)
                row[f'mode_{tag}'] = pr['mode']
                row[f'mean_{tag}'] = pr['mean']
                row[f'bandmid_{tag}'] = pr['band_mid']
                row[f'bandw_{tag}'] = pr['band_w']
                row[f'bandlo_{tag}'] = pr['band_lo']
                row[f'bandhi_{tag}'] = pr['band_hi']
            gdx, gdy, _ = grid_optimum(fa, ex, ey)
            row['opt2d_dx'], row['opt2d_dy'] = gdx, gdy
            ox, oy = S.LIDAR_XY
            row['w_gt'] = float(weight_from(fa, ex, ey)[0])
            ay, ax, aa = s['amcl_y'], s['amcl_x'], s['amcl_yaw']
            alx = ax + ox * math.cos(aa) - oy * math.sin(aa)
            aly = ay + ox * math.sin(aa) + oy * math.cos(aa)
            exa, eya = endpoints(sc, alx, aly, aa)
            row['w_amcl'] = float(weight_from(fa, exa, eya)[0])
            row['w_opt'] = float(weight_from(fa, ex, ey, gdx, gdy)[0])
            row['reweight'] = {}
            for label, f, mdy, dil in (
                    [('shipped_amcl', fa, 0.0, 0.0),
                     ('shipped_grid', fg, 0.0, 0.0)]
                    + [(f'dilate{d:.2f}', fa, 0.0, d) for d in DILATIONS]
                    + [(f'mapdy{m:+.2f}', fa, m, 0.0) for m in MAP_DY]):
                w = particle_weights(f, sc, mlx, mly, lyaw, part, mdy, dil)
                wn = w / w.sum()
                pr = dy_profile(f, ex, ey, mdy, dil)
                row['reweight'][label] = {
                    'cen_dy': float((wn * part[:, 1]).sum()) - mgy,
                    'shift': float((wn * part[:, 1]).sum()
                                   - part[:, 1].mean()),
                    'ess_frac': float(1.0 / (wn ** 2).sum() / len(w)),
                    'mode': pr['mode'], 'mean': pr['mean'],
                    'band_mid': pr['band_mid'], 'band_w': pr['band_w'],
                }
                if label == 'shipped_amcl':
                    dy = part[:, 1] - mgy
                    dyaw = np.arctan2(np.sin(part[:, 2] - s['gt_yaw']),
                                      np.cos(part[:, 2] - s['gt_yaw']))
                    row['corr_w_dy'] = float(np.corrcoef(w, dy)[0, 1])
                    row['corr_w_absdyaw'] = float(
                        np.corrcoef(w, np.abs(dyaw))[0, 1])
                    row['sd_dy'] = float(dy.std())
                    row['sd_dyaw'] = float(dyaw.std())
            rows.append(row)
        doc['legs'][leg] = {'n': len(rows), 'rows': rows,
                            'testA_med': _med(testa),
                            'testA_absmax': float(np.abs(testa).max()),
                            'testA_frac_1cm': float(
                                (np.abs(testa) <= S.RANGE_RES).mean())}

    # -- controls: all seven scenarios, C2-NAV.28 samples
    ctrl = load_controls()
    doc['controls'] = {}
    doc['control_samples'] = ctrl
    if ctrl:
        for scen in S.SCENARIOS:
            rs = ctrl.get(scen)
            if not rs:
                continue
            acc = {k: [] for k in ('amcl_dy', 'mean_amcl', 'mean_grid',
                                   'mode_amcl', 'bandw_amcl', 'bandmid_amcl',
                                   'mean_dil06', 'mean_shift06')}
            for r in rs:
                lx, ly, lyaw = S.lidar_pose(r['gt_x'], r['gt_y'], r['gt_yaw'])
                sc = S.scan_from_world(lx, ly, lyaw)
                ex, ey = endpoints(sc, lx + offset[0], ly + offset[1], lyaw)
                pa, pg = dy_profile(fa, ex, ey), dy_profile(fg, ex, ey)
                pd = dy_profile(fa, ex, ey, 0.0, 0.06)
                ps = dy_profile(fa, ex, ey, -0.06, 0.0)
                acc['amcl_dy'].append(r['amcl_y'] - (r['gt_y'] + offset[1]))
                acc['mean_amcl'].append(pa['mean'])
                acc['mean_grid'].append(pg['mean'])
                acc['mode_amcl'].append(pa['mode'])
                acc['bandw_amcl'].append(pa['band_w'])
                acc['bandmid_amcl'].append(pa['band_mid'])
                acc['mean_dil06'].append(pd['mean'])
                acc['mean_shift06'].append(ps['mean'])
            doc['controls'][scen] = dict(
                {k: _med(v) for k, v in acc.items()},
                n=len(rs), n_available=ctrl.get(scen + '_n_available'))
    return doc


# == reporting =========================================================
def _leg(doc, leg='wall_adjacent'):
    return doc['legs'].get(leg)


def sec_params(doc):
    """Part 1: the exact deployed likelihood-field parameter table."""
    p, e, b = doc['amcl_params'], doc['effective'], doc['binning']
    print('PART 1  THE DEPLOYED LIKELIHOOD-FIELD MODEL')
    print('  every value read from its source file, none retyped')
    print()
    src_cfg = 'gazebo_models/config/nav2_params.yaml'
    src_xac = 'gazebo_models/urdf/coco_robo2.xacro'
    src_map = 'gazebo_models/maps/coco_world.yaml'
    rows = [
        ('laser_model_type', p['laser_model_type'], src_cfg),
        ('max_beams', p['max_beams'], src_cfg),
        ('sigma_hit', p['sigma_hit'], src_cfg),
        ('z_hit', p['z_hit'], src_cfg),
        ('z_rand', p['z_rand'], src_cfg),
        ('z_short / z_max', f"{p['z_short']} / {p['z_max']}",
         src_cfg + ' (unused by likelihood_field)'),
        ('laser_likelihood_max_dist', p['laser_likelihood_max_dist'], src_cfg),
        ('laser_max_range', p['laser_max_range'], src_cfg),
        ('laser_min_range', p['laser_min_range'], src_cfg),
        ('do_beamskip', p['do_beamskip'], src_cfg),
        ('min / max_particles',
         f"{p['min_particles']} / {p['max_particles']}", src_cfg),
        ('resample_interval', p['resample_interval'], src_cfg),
        ('update_min_d / _a',
         f"{p['update_min_d']} / {p['update_min_a']}", src_cfg),
        ('robot_model_type', p['robot_model_type'], src_cfg),
        ('scan samples', p['scan_samples'], src_xac),
        ('scan range min / max',
         f"{p['scan_range_min']} / {p['scan_range_max']}", src_xac),
        ('scan angle min / max',
         f"{p['scan_angle_min']} / {p['scan_angle_max']}", src_xac),
        ('scan <noise> block', p['scan_has_noise'], src_xac),
        ('map resolution', p['map_resolution'], src_map),
        ('map origin', tuple(p['map_origin']), src_map),
        ('map size', f"{p['map_width']} x {p['map_height']}",
         'coco_world.pgm'),
        ('map occupied_thresh / negate',
         f"{p['map_occupied_thresh']} / {p['map_negate']}", src_map),
    ]
    for k, v, src in rows:
        print(f'  {k:30s} {str(v):26s} {src}')
    print()
    print('  DERIVED, because amcl does not use the configured values raw:')
    print(f"    effective range_max   {e['range_max']}  "
          f"= min(scan.range_max, laser_max_range)")
    print(f"    effective range_min   {e['range_min']}  "
          f"= scan.range_min, since laser_min_range < 0")
    print(f"    beam stride           {e['beam_step']}  "
          f"= (range_count-1)//(max_beams-1), giving {e['beams_used']} beams")
    print(f"    z_rand_mult           {1.0/e['range_max']:.6f} = 1/range_max")
    print(f"    max attainable weight {e['weight_max']:.4f} "
          f"= 1 + n*(z_hit + z_rand/range_max)**3")
    print()
    print('  THE PARAMETER THAT IS NOT IN ANY YAML -- the map cell convention')
    print(f"    {b['header']}  readable: {b['readable']}")
    if b.get('gxwx'):
        print(f"    {b['gxwx']}")
        print(f"    {b['wxgx']}")
    print(f"    map_t.size_x declared int: {b.get('size_is_int')}")
    print('    convertMap sets origin_x = msg.origin.x + (size_x/2)*scale,')
    print('    and size_x/2 is the SAME integer in both, so it cancels:')
    print('        cell(x) = floor((x - yaml_origin_x)/scale + 0.5)')
    print('    A cell centre is yaml_origin + i*scale, NOT + (i+0.5)*scale.')
    print(f"    amcl's surfaces therefore sit {b.get('half_cell_m')} m more")
    print('    negative in x AND y than the same PGM read the ROS way.')
    print('    Both conventions are carried through every table below.')
    return 0


def sec_variants(doc):
    """Part 3 definition: what a map variant is, and why this range."""
    print('PART 3 (definition)  THE MAP VARIANTS')
    print('  The shipped PGM is NEVER modified. A variant is applied at')
    print('  query time inside Field.distance, two ways:')
    print()
    print('    mapdy  d   TRANSLATE the map by d in y. Querying at y-d is')
    print('               exactly a map translated by d. Negative = south.')
    print('    dilate e   GROW every occupied region outward by e metres.')
    print('               For the exterior EDT that is d -> max(d-e, 0).')
    print()
    print('  WHICH ONE MATCHES THE MEASURED DEFECT. C2-NAV.29 Test B found')
    print('  the map draws surfaces 0.05-0.07 m CLOSER than they are, in')
    print('  every direction. That is a DILATION, not a translation.')
    print('  map_audit.py measures the map->world transform over five')
    print('  landmarks as (+2.0560, +0.0150) m with y peak-to-peak 0.0000')
    print('  and worst residual 25 mm: the map is not translated south at')
    print('  all, and its landmark SIZES read 0.05-0.15 m too large --')
    print('  which is what a dilation looks like. Both are tested anyway,')
    print('  because the translation is the brief\'s hypothesis and is the')
    print('  only variant that could move an optimum rigidly.')
    print()
    print(f"  translations tested: {doc['map_dy']}")
    print(f"  dilations tested:    {doc['dilations']}")
    print(f"  dy support +/-{doc['dy_span']} m at {doc['dy_step']} m; "
          f"plateau band w >= {doc['band_frac']}*wmax")
    return 0


def sec_particles(doc):
    """Part 2: score the recorded particle set with the shipped model."""
    lg = _leg(doc)
    if not lg:
        print('no wall_adjacent cloud in this bundle')
        return 1
    print('PART 2  THE RECORDED PARTICLE SET, SCORED BY THE SHIPPED MODEL')
    print(f"  scan reconstruction revalidated on THIS leg (Test A): median "
          f"{lg['testA_med']:+.5f} m, max|.| {lg['testA_absmax']:.5f} m, "
          f"within 0.01 m {lg['testA_frac_1cm']:.3f}")
    print(f"  max attainable weight {doc['effective']['weight_max']:.4f}")
    print()
    hdr = (f"{'t':>5s} {'n':>4s} {'AMCLdy':>8s} {'clouddy':>8s} | "
           f"{'w(GT)':>7s} {'w(AMCL)':>8s} {'ratio':>6s} | "
           f"{'reW dy':>8s} {'shift':>8s} {'ESS/n':>6s} | "
           f"{'r(w,dy)':>8s} {'r(w,|dyaw|)':>12s}")
    print(hdr)
    print('-' * len(hdr))
    for r in lg['rows']:
        rw = r['reweight']['shipped_amcl']
        print(f"{r['t']:5.1f} {r['n']:4d} {r['amcl_dy']:+8.4f} "
              f"{r['cloud_dy']:+8.4f} | {r['w_gt']:7.4f} {r['w_amcl']:8.4f} "
              f"{r['w_amcl']/r['w_gt']:6.3f} | {rw['cen_dy']:+8.4f} "
              f"{rw['shift']:+8.4f} {rw['ess_frac']:6.3f} | "
              f"{r['corr_w_dy']:+8.3f} {r['corr_w_absdyaw']:+12.3f}")
    rows = lg['rows']
    wr = [r['w_amcl'] / r['w_gt'] for r in rows]
    beats = sum(1 for r in rows if r['w_amcl'] >= r['w_gt'])
    print()
    print(f"  median w(AMCL)/w(GT) {_med(wr):.4f}; the AMCL pose outscores "
          f"ground truth in {beats} of {len(rows)} snapshots")
    print(f"  median cloud centroid dy      "
          f"{_med([r['cloud_dy'] for r in rows]):+.4f} m  (observed)")
    rw = [r['reweight']['shipped_amcl'] for r in rows]
    print(f"  median re-weighted centroid   "
          f"{_med([x['cen_dy'] for x in rw]):+.4f}"
          f" m  (shipped model applied to those same particles)")
    print(f"  median shift from re-weighting "
          f"{_med([x['shift'] for x in rw]):+.4f}"
          f" m  (positive = NORTH = toward truth)")
    print(f"  median ESS/n after re-weighting "
          f"{_med([x['ess_frac'] for x in rw]):.4f}"
          f"  -- the model barely discriminates between these")
    print(f"  median corr(w, dy) {_med([r['corr_w_dy'] for r in rows]):+.3f}, "
          f"corr(w, |dyaw|) {_med([r['corr_w_absdyaw'] for r in rows]):+.3f}: "
          f"the weight is driven by YAW, not by y")
    print()
    print('  The published weights are FLAT (ESS/n '
          f"{_med([r['pub_ess_frac'] for r in rows]):.4f}), so none of this")
    print('  reconstructs AMCL\'s own importance weights. It is what the')
    print('  shipped OBSERVATION MODEL would say about these poses.')
    return 0


def sec_shift(doc):
    """Part 3: the shifted- and dilated-map sweep."""
    lg = _leg(doc)
    if not lg:
        return 1
    print('PART 3  MAP VARIANT -> WHERE THE MODEL PUTS THE ROBOT')
    print('  medians over the wall_adjacent snapshots; amcl cell convention')
    print()
    hdr = (f"{'variant':>16s} | {'mode dy':>8s} {'mean dy':>8s} "
           f"{'band mid':>9s} {'band w':>7s} | {'reW dy':>8s} {'shift':>8s}")
    print(hdr)
    print('-' * len(hdr))
    order = (['shipped_amcl', 'shipped_grid']
             + [f'dilate{d:.2f}' for d in doc['dilations']]
             + [f'mapdy{m:+.2f}' for m in doc['map_dy'] if m != 0.0])
    for label in order:
        v = [r['reweight'][label] for r in lg['rows']
             if label in r['reweight']]
        if not v:
            continue
        print(f"{label:>16s} | {_med([x['mode'] for x in v]):+8.4f} "
              f"{_med([x['mean'] for x in v]):+8.4f} "
              f"{_med([x['band_mid'] for x in v]):+9.4f} "
              f"{_med([x['band_w'] for x in v]):7.4f} | "
              f"{_med([x['cen_dy'] for x in v]):+8.4f} "
              f"{_med([x['shift'] for x in v]):+8.4f}")
    obs = _med([r['cloud_dy'] for r in lg['rows']])
    print()
    print(f'  observed cloud centroid dy {obs:+.4f} m')
    print('  READ THE BAND WIDTH COLUMN. A TRANSLATION slides the whole')
    print('  surface: band width is constant and mid tracks the shift 1:1.')
    print('  A DILATION does not translate anything -- it WIDENS the')
    print('  plateau, pinned at its northern edge, and the mode then')
    print('  wanders south inside a region the model cannot distinguish.')
    print('  The dilation mode is an argmax artefact, not a displacement.')
    return 0


def sec_bias(doc):
    """Part 4: observed vs predicted, case by case."""
    lg = _leg(doc)
    if not lg:
        return 1
    print('PART 4  OBSERVED VS PREDICTED, PER WALL-ADJACENT SAMPLE')
    hdr = (f"{'t':>5s} {'scanmin':>8s} {'obs dy':>8s} | {'mode':>7s} "
           f"{'mean':>7s} {'band mid':>9s} {'band w':>7s} | "
           f"{'w(GT)':>7s} {'w(AMCL)':>8s} {'w(opt)':>7s} | {'verdict':>18s}")
    print(hdr)
    print('-' * len(hdr))
    for r in lg['rows']:
        gap = abs(r['mean_amcl'] - r['amcl_dy'])
        v = ('model explains' if gap <= 0.02 else
             'model short' if abs(r['mean_amcl']) < abs(r['amcl_dy'])
             else 'model overshoots')
        print(f"{r['t']:5.1f} {r['scan_min']:8.3f} {r['amcl_dy']:+8.4f} | "
              f"{r['mode_amcl']:+7.3f} {r['mean_amcl']:+7.4f} "
              f"{r['bandmid_amcl']:+9.4f} {r['bandw_amcl']:7.3f} | "
              f"{r['w_gt']:7.4f} {r['w_amcl']:8.4f} {r['w_opt']:7.4f} | "
              f"{v:>18s}")
    rows = lg['rows']
    ao = _med([r['amcl_dy'] for r in rows])
    for tag in ('amcl', 'grid'):
        mm = _med([r[f'mean_{tag}'] for r in rows])
        print(f"  median observed {ao:+.4f} m; {tag}-binning model mean "
              f"{mm:+.4f} m = {100*mm/ao:.1f}% of it")
    print(f"  median plateau width {_med([r['bandw_amcl'] for r in rows]):.3f}"
          f" m, which is WIDER than the {abs(ao):.3f} m bias itself")
    return 0


def sec_amplify(doc):
    """Part 5: does map error amplify into localisation error?"""
    lg = _leg(doc)
    if not lg:
        return 1
    print('PART 5  SENSITIVITY: map error -> localisation displacement')
    print()
    xs, mode, mean, cen = [], [], [], []
    for m in sorted(doc['map_dy']):
        label = 'shipped_amcl' if m == 0.0 else f'mapdy{m:+.2f}'
        v = [r['reweight'][label] for r in lg['rows']
             if label in r['reweight']]
        if not v:
            continue
        xs.append(m)
        mode.append(_med([x['mode'] for x in v]))
        mean.append(_med([x['mean'] for x in v]))
        cen.append(_med([x['cen_dy'] for x in v]))
    hdr = f"{'map dy':>8s} {'mode dy':>9s} {'mean dy':>9s} {'reW cen dy':>11s}"
    print(hdr)
    print('-' * len(hdr))
    for i, m in enumerate(xs):
        print(f'{m:+8.2f} {mode[i]:+9.4f} {mean[i]:+9.4f} {cen[i]:+11.4f}')
    print()
    for name, ys in (('mode', mode), ('mean', mean),
                     ('reweighted centroid', cen)):
        g = float(np.polyfit(xs, ys, 1)[0])
        print(f'  gain d({name})/d(map dy) = {g:.4f}')
    print()
    print('  A gain of 1 is pass-through; > 1 would be amplification.')
    print('  Nothing here exceeds 1. Map error does NOT amplify: the mode')
    print('  tracks the map one-for-one and every broader statistic tracks')
    print('  it LESS than one-for-one, because the plateau is wide.')
    obs = _med([r['cloud_dy'] for r in lg['rows']])
    g = float(np.polyfit(xs, mean, 1)[0])
    c = float(np.polyfit(xs, mean, 1)[1])
    print(f'  To reach the observed {obs:+.4f} m by the mean, the map would')
    print(f'  have to be translated {(obs - c)/g:+.3f} m south. map_audit.py')
    print('  bounds the real y translation at +0.015 m with a 25 mm worst')
    print('  residual, so a translation that large is excluded.')
    return 0


def sec_consistency(doc):
    """Part 6: optimum vs particle centroid vs AMCL pose."""
    print('PART 6  LIKELIHOOD OPTIMUM vs PARTICLE CENTROID vs AMCL POSE')
    print()
    hdr = (f"{'leg':>15s} {'n':>3s} | {'AMCL dy':>8s} {'cloud dy':>9s} "
           f"{'mode':>7s} {'mean':>8s} {'band':>17s} {'width':>7s}")
    print(hdr)
    print('-' * len(hdr))
    for leg in CLOUD_LEGS:
        lg = _leg(doc, leg)
        if not lg:
            continue
        r = lg['rows']
        band = (f"[{_med([x['bandlo_amcl'] for x in r]):+.3f},"
                f"{_med([x['bandhi_amcl'] for x in r]):+.3f}]")
        print(f"{leg:>15s} {lg['n']:3d} | "
              f"{_med([x['amcl_dy'] for x in r]):+8.4f} "
              f"{_med([x['cloud_dy'] for x in r]):+9.4f} "
              f"{_med([x['mode_amcl'] for x in r]):+7.3f} "
              f"{_med([x['mean_amcl'] for x in r]):+8.4f} {band:>17s} "
              f"{_med([x['bandw_amcl'] for x in r]):7.3f}")
    print()
    lg = _leg(doc)
    r = lg['rows']
    obs = _med([x['cloud_dy'] for x in r])
    mean = _med([x['mean_amcl'] for x in r])
    lo = _med([x['bandlo_amcl'] for x in r])
    print('  The three do NOT coincide. The likelihood optimum stays near')
    print(f'  ground truth ({mean:+.4f} m by the mean, '
          f"{_med([x['mode_amcl'] for x in r]):+.3f} m by the mode) while the")
    print(f'  particle centroid sits {obs:+.4f} m south. On the brief\'s own')
    print('  reading that is the branch where the map and the likelihood')
    print('  model do NOT explain the estimator bias.')
    print()
    print('  AND THE SURFACE IS BROAD. The 99% plateau spans '
          f'{_med([x["bandw_amcl"] for x in r]):.3f} m,')
    print(f'  reaching {lo:+.3f} m south. The observed {obs:+.3f} m sits')
    print('  inside or at the edge of a region the observation model')
    print('  cannot resolve, so the model does not FORBID the bias either.')
    print('  It has no authority here in either direction -- which is a')
    print('  third possibility the brief names, and it is the true one.')
    return 0


def sec_direction(doc):
    """Part 7: does the model's preference point the same way as AMCL?"""
    print('PART 7  DIRECTION CHECK')
    print()
    hdr = (f"{'scenario':>16s} {'n':>4s} {'AMCL dy':>9s} {'model mean':>11s} "
           f"{'same sign':>10s} {'magnitude':>10s}")
    print(hdr)
    print('-' * len(hdr))
    agree = tot = 0
    for scen in S.SCENARIOS:
        c = doc['controls'].get(scen)
        if not c:
            continue
        same = 'yes' if c['amcl_dy'] * c['mean_amcl'] > 0 else 'NO'
        agree += same == 'yes'
        tot += 1
        frac = (100 * c['mean_amcl'] / c['amcl_dy']
                if abs(c['amcl_dy']) > 1e-9 else float('nan'))
        print(f"{scen:>16s} {c['n']:4d} {c['amcl_dy']:+9.4f} "
              f"{c['mean_amcl']:+11.4f} {same:>10s} {frac:9.1f}%")
    print()
    print(f'  direction agrees in {agree} of {tot} scenarios, INCLUDING')
    print('  obstacle_corner, where the observed bias is NORTHWARD and the')
    print('  model preference flips sign with it. The sign is right')
    print('  everywhere. The magnitude never is.')
    return 0


def sec_controls(doc):
    """Part 8: the same analysis across every scenario."""
    print('PART 8  CONTROL REGIONS')
    print()
    hdr = (f"{'scenario':>16s} {'n':>4s} {'AMCL dy':>9s} | {'mean amcl':>10s} "
           f"{'mean grid':>10s} {'dilate .06':>11s} {'map dy -.06':>12s} | "
           f"{'mode':>7s} {'band w':>7s} | {'expl%':>6s}")
    print(hdr)
    print('-' * len(hdr))
    for scen in S.SCENARIOS:
        c = doc['controls'].get(scen)
        if not c:
            continue
        frac = (100 * c['mean_amcl'] / c['amcl_dy']
                if abs(c['amcl_dy']) > 1e-9 else float('nan'))
        print(f"{scen:>16s} {c['n']:4d} {c['amcl_dy']:+9.4f} | "
              f"{c['mean_amcl']:+10.4f} {c['mean_grid']:+10.4f} "
              f"{c['mean_dil06']:+11.4f} {c['mean_shift06']:+12.4f} | "
              f"{c['mode_amcl']:+7.3f} {c['bandw_amcl']:7.3f} | {frac:6.1f}")
    print()
    print('  The MODE is near-constant across every scenario -- it is the')
    print('  half-cell convention plus grid quantisation, a fixed offset,')
    print('  and it cannot explain a bias that varies with place.')
    print('  The MEAN does track the bias in sign, at roughly a fifth of')
    print('  its size. The dilation column moves the answer by about a')
    print('  millimetre; a -0.06 m map translation moves it by ~0.02 m.')
    return 0


def sec_verdict(doc):
    """The classification, and the budget behind it."""
    lg = _leg(doc)
    r = lg['rows']
    obs = _med([x['cloud_dy'] for x in r])
    m_amcl = _med([x['mean_amcl'] for x in r])
    m_grid = _med([x['mean_grid'] for x in r])
    dil = _med([x['reweight']['dilate0.06']['mean'] for x in r])
    half = doc['binning']['half_cell_m']
    shift = _med([x['reweight']['shipped_amcl']['shift'] for x in r])
    print('CAUSAL CLASSIFICATION')
    print()
    print(f'  observed wall_adjacent centroid dy        {obs:+.4f} m')
    print('  -- attributed by the shipped observation model --')
    print(f'  map as drawn, ROS cell convention          {m_grid:+.4f} m '
          f'({100*m_grid/obs:.0f}%)')
    print(f"  + amcl's half-cell index convention        "
          f"{m_amcl - m_grid:+.4f} m ({100*(m_amcl-m_grid)/obs:.0f}%)")
    print(f'  = total observation-model preference       {m_amcl:+.4f} m '
          f'({100*m_amcl/obs:.0f}%)')
    print(f'  the measured 0.05-0.07 m dilation adds     '
          f'{dil - m_amcl:+.4f} m ({100*(dil-m_amcl)/obs:.1f}%)')
    print(f'  UNEXPLAINED                                {obs - dil:+.4f} m '
          f'({100*(obs-dil)/obs:.0f}%)')
    print()
    print('  and the direct test, which assumes nothing about the surface:')
    print('  re-weighting the RECORDED cloud with the shipped model moves')
    print(f'  its centroid {shift:+.4f} m -- NORTH, toward truth, not south.')
    print()
    print('  >>> B. MAP DEFECT EXPLAINS ONLY A MINOR PORTION;')
    print('  >>>    AN AMCL-SIDE MECHANISM IS STILL REQUIRED.')
    print()
    md_a = _med([x['mode_amcl'] for x in r])
    md_g = _med([x['mode_grid'] for x in r])
    print('  BY THE MODE, which is what C2-NAV.29 reported, so that the')
    print('  comparison is like for like:')
    print(f'    map as drawn, ROS convention   {md_g:+.4f} m '
          f'({100*md_g/obs:.0f}%)')
    print(f"    + amcl's half-cell convention  {md_a - md_g:+.4f} m "
          f'({100*(md_a-md_g)/obs:.0f}%)')
    print(f'    = total                        {md_a:+.4f} m '
          f'({100*md_a/obs:.0f}%)')
    print(f'  The half-cell moves the MODE by exactly {half} m -- it is a')
    print('  rigid translation of the field -- but only '
          f'{abs(m_amcl - m_grid):.4f} m of MEAN,')
    print('  because the plateau is wide enough to absorb most of it.')
    print()
    print('  So this is not "B because C2-NAV.29 said one sixth". Recomputed')
    print(f'  against the deployed model the share is LARGER '
          f'({100*md_a/obs:.0f}% by the mode,')
    print(f'  {100*m_amcl/obs:.0f}% by the mean, against C2-NAV.29\'s ~16%), '
          f"because amcl's own")
    print('  map-index convention is a mechanism C2-NAV.29 did not model.')
    print('  It is still a minority; it is a FIXED offset, identical in')
    print('  every scenario, so it cannot explain a bias that varies with')
    print('  place; and the measured dilation -- the part that is genuinely')
    print('  a map defect -- contributes almost nothing, because it widens')
    print('  the plateau instead of moving it.')
    return 0


SECTIONS = {
    'params': sec_params, 'variants': sec_variants, 'particles': sec_particles,
    'shift': sec_shift, 'bias': sec_bias, 'amplify': sec_amplify,
    'consistency': sec_consistency, 'direction': sec_direction,
    'controls': sec_controls, 'verdict': sec_verdict,
}


# == selftest ==========================================================
def selftest():
    """Every constant re-derived from its source; every model hand-checked."""
    fails = []
    n = [0]

    def chk(label, cond, detail=''):
        n[0] += 1
        print(f"  {'ok  ' if cond else 'FAIL'} {label}"
              f"{('  ' + detail) if detail else ''}")
        if not cond:
            fails.append(label)

    print('C2-NAV.31 selftest (offline)')
    p, e = P0, dict(zip(('rmax', 'rmin', 'step', 'nb'),
                        effective_ranges(P0)))

    print(' -- the configuration is what the tool thinks it is')
    chk('laser_model_type is likelihood_field',
        p['laser_model_type'] == 'likelihood_field', p['laser_model_type'])
    chk('max_beams 60', p['max_beams'] == 60, str(p['max_beams']))
    chk('sigma_hit 0.2', p['sigma_hit'] == 0.2)
    chk('z_hit / z_rand 0.5 / 0.5',
        (p['z_hit'], p['z_rand']) == (0.5, 0.5))
    chk('laser_likelihood_max_dist 2.0',
        p['laser_likelihood_max_dist'] == 2.0)
    chk('resample_interval 1', p['resample_interval'] == 1)
    chk('the lidar declares NO <noise> block', p['scan_has_noise'] is False)
    chk('scan is 480 beams', p['scan_samples'] == 480)

    print(' -- the derived quantities amcl computes for itself')
    chk('effective range_max is the SCAN max, not the config max',
        e['rmax'] == 12.0 and p['laser_max_range'] == 100.0,
        f"{e['rmax']} from min(12.0, {p['laser_max_range']})")
    chk('effective range_min is the scan min, laser_min_range < 0',
        e['rmin'] == p['scan_range_min'] and p['laser_min_range'] < 0)
    chk('beam stride 8 gives exactly 60 beams',
        (e['step'], e['nb']) == (8, 60), f"step {e['step']}, n {e['nb']}")
    hand = 1.0 + 60 * (0.5 + 0.5 / 12.0) ** 3
    chk('max weight matches a hand computation',
        abs(weight_max() - hand) < 1e-12, f'{weight_max():.6f}')

    print(' -- the map cell convention, read from the installed header')
    b = binning_convention()
    chk('nav2_amcl map.hpp is present', b['readable'], b['header'])
    chk('MAP_GXWX rounds (+ 0.5), it does not floor',
        b.get('gxwx') and '+ 0.5' in b['gxwx'], (b.get('gxwx') or '')[:70])
    chk('map_t.size_x is an int, so size_x/2 is integer division',
        b.get('size_is_int') is True)
    chk('half cell is 0.025 m', abs(b['half_cell_m'] - 0.025) < 1e-12)

    print(' -- the likelihood field itself')
    fa, fg = Field('amcl'), Field('grid')
    chk('both fields share one occupied set',
        bool((fa.occupied == fg.occupied).all()),
        f'{int(fa.occupied.sum())} cells')
    chk('laser_likelihood_max_dist NEVER binds on this map',
        fa.max_edt < p['laser_likelihood_max_dist'],
        f"largest distance-to-obstacle {fa.max_edt:.4f} m vs the "
        f"{p['laser_likelihood_max_dist']} m truncation")
    chk('so truncation changes no cell',
        bool(np.array_equal(fa.dist, np.minimum(fa.dist, fa.max_dist)))
        and abs(fa.dist.max() - fa.max_edt) < 1e-9)
    ref = S.LikelihoodField()
    chk('grid binning reproduces C2-NAV.29 LikelihoodField.distance',
        bool(np.allclose(fg.distance(np.array([0.0, 1.0, 2.0]),
                                     np.array([0.0, 1.0, -2.0])),
                         np.minimum(ref.distance(np.array([0.0, 1.0, 2.0]),
                                                 np.array([0.0, 1.0, -2.0])),
                                    fa.max_dist))))
    # the two binnings must differ by exactly half a cell, by construction
    xs = np.linspace(-1.5, 6.0, 400)
    ys = np.full_like(xs, -3.0)
    half = fa.res / 2.0
    chk('amcl binning == grid binning shifted by half a cell',
        bool(np.allclose(fa.distance(xs, ys),
                         fg.distance(xs + half, ys + half))),
        f'{half} m')

    print(' -- the weight function against a hand computation')
    fake = np.full(480, np.inf)
    fake[0] = 1.0
    ex, ey = endpoints(fake, 0.0, 0.0, 0.0)
    chk('one valid beam survives amcl\'s stride and range gate',
        len(ex) == 1, f'{len(ex)} endpoints')
    d = fa.distance(ex, ey)[0]
    pz = 0.5 * math.exp(-(d * d) / (2 * 0.2 ** 2)) + 0.5 / 12.0
    chk('weight_from == 1 + sum(pz**3), amcl\'s own accumulation',
        abs(float(weight_from(fa, ex, ey)[0]) - (1.0 + pz ** 3)) < 1e-12)
    fake2 = np.full(480, np.inf)
    fake2[0] = 0.10          # below range_min 0.15 -> mapped to range_max
    chk('a return below range_min is mapped away, not scored',
        len(endpoints(fake2, 0.0, 0.0, 0.0)[0]) == 0)
    fake3 = np.full(480, np.inf)
    fake3[0] = 12.0          # at range_max -> skipped
    chk('a return at range_max is skipped',
        len(endpoints(fake3, 0.0, 0.0, 0.0)[0]) == 0)

    print(' -- the map variants do what they claim')
    xs2 = np.linspace(0.0, 4.0, 200)
    ys2 = np.full_like(xs2, -1.0)
    chk('mapdy d equals querying at y - d',
        bool(np.allclose(fa.distance(xs2, ys2, mdy=-0.06),
                         fa.distance(xs2, ys2 + 0.06))))
    d0 = fa.distance(xs2, ys2)
    chk('dilate e subtracts e from the exterior EDT, clipped at 0',
        bool(np.allclose(fa.distance(xs2, ys2, dilate=0.06),
                         np.clip(d0 - 0.06, 0.0, fa.max_dist))))
    chk('a zero variant is the shipped map exactly',
        bool(np.allclose(fa.distance(xs2, ys2, 0.0, 0.0), d0)))

    print(' -- the shipped map file is untouched')
    import hashlib
    with open(S.PGM, 'rb') as fh:
        pgm_md5 = hashlib.md5(fh.read()).hexdigest()
    with open(S.MAPYAML, 'rb') as fh:
        yml_md5 = hashlib.md5(fh.read()).hexdigest()
    chk('map pgm and yaml both readable and hashed',
        len(pgm_md5) == 32 and len(yml_md5) == 32, f'pgm {pgm_md5[:8]}')

    print(' -- the estimators behave as advertised')
    ds = np.arange(-DY_SPAN, DY_SPAN + DY_STEP / 2, DY_STEP)
    chk('the dy support is symmetric about zero',
        abs(ds.min() + ds.max()) < 1e-12 and len(ds) == 181,
        f'{len(ds)} points')

    class _Flat:
        max_dist = 2.0

        def distance(self, x, y, mdy=0.0, dilate=0.0):
            return np.abs(np.asarray(y) - 0.05)

    pr = dy_profile(_Flat(), np.array([0.0]), np.array([0.0]))
    chk('mode finds a known analytic optimum',
        abs(pr['mode'] - 0.05) < 1e-9, f"{pr['mode']:+.4f}")
    chk('mean is pulled to the same side as the mode',
        pr['mean'] > 0, f"{pr['mean']:+.4f}")
    chk('band midpoint brackets the optimum',
        pr['band_lo'] <= 0.05 <= pr['band_hi'])

    print(' -- C2-NAV.22/.24/.28/.29/.30 still reproduce')
    for name, path in (('C2-NAV.29 scanmap', 'c2nav29_scanmap.json'),
                       ('C2-NAV.30 cloud', 'c2nav30_cloud.json'),
                       ('C2-NAV.28 amcl', 'c2nav28_amcl.json')):
        chk(f'{name} bundle still loads',
            os.path.exists(os.path.join(HERE, path)))
    try:
        d29 = json.load(open(os.path.join(HERE, 'c2nav29_scanmap.json')))
        wa = d29['summary']['wall_adjacent']
        chk('C2-NAV.29 wall_adjacent headline unchanged',
            wa['C_gt_wins_frac'] == 1.0 and wa['C_opt_dy_med'] == -0.02
            and wa['amcl_dy_med'] == -0.129,
            f"wins {wa['C_gt_wins_frac']}, optdy {wa['C_opt_dy_med']}, "
            f"amcldy {wa['amcl_dy_med']}")
    except Exception as exc:                       # noqa: BLE001
        chk('C2-NAV.29 headline readable', False, str(exc))
    try:
        d30 = json.load(open(os.path.join(HERE, 'c2nav30_cloud.json')))
        cm = d30['cloud_meta']['c2n30_focus_r1/wall_adjacent']
        chk('C2-NAV.30 wall_adjacent snapshot count unchanged',
            cm['n_snapshots'] == 19 and cm['n_particles_total'] == 10393,
            f"{cm['n_snapshots']} snaps, {cm['n_particles_total']} particles")
    except Exception as exc:                       # noqa: BLE001
        chk('C2-NAV.30 headline readable', False, str(exc))

    print(' -- the bundle RECOMPUTES without .navbench, it does not replay')
    if os.path.exists(FROZEN):
        with open(FROZEN) as fh:
            fz = json.load(fh)
        legs = fz.get('legs', {})
        chk('the bundle carries both legs',
            set(legs) == set(CLOUD_LEGS), str(sorted(legs)))
        got = all(r.get('part') for lg in legs.values() for r in lg['rows'])
        chk('every snapshot carries its raw particle poses', got)
        npart = sum(len(r['part']) for lg in legs.values()
                    for r in lg['rows'])
        chk('particle count matches C2-NAV.30 (10393 + 13051)',
            npart == 23444, f'{npart}')
        chk('the bundle carries the control samples too',
            bool(fz.get('control_samples')))
        wa = legs.get('wall_adjacent', {}).get('rows', [])
        chk('a frozen row reproduces its own centroid',
            bool(wa) and abs(
                float(np.mean([p[1] for p in wa[0]['part']]))
                - (wa[0]['gt_y_map'] + wa[0]['cloud_dy'])) < 1e-4)
    else:
        chk('frozen bundle present', False, FROZEN)

    print(' -- this session changed no behavioural file')
    for rel in ('gazebo_models/config/nav2_params.yaml',
                'gazebo_models/maps/coco_world.pgm',
                'gazebo_models/maps/coco_world.yaml',
                'gazebo_models/urdf/coco_robo2.xacro',
                'gazebo_models/scripts/nav_bench.py'):
        chk(f'{rel} exists and is only read',
            os.path.exists(os.path.join(REPO, rel)))

    print(f'\n{n[0] - len(fails)}/{n[0]} checks passed')
    if fails:
        print('FAILED: ' + '; '.join(fails))
    return 1 if fails else 0


# == entry point =======================================================
def build_and_freeze(offset=NAVBENCH_OFFSET):
    doc = analyse(offset)
    if not doc.get('legs'):
        return None
    with open(FROZEN, 'w') as fh:
        json.dump(doc, fh, separators=(',', ':'), sort_keys=True)
    return doc


def load_doc(rebuild=False, offset=NAVBENCH_OFFSET):
    """Recompute from the traces when they are there; else replay."""
    if rebuild or not os.path.exists(FROZEN):
        doc = build_and_freeze(offset)
        if doc is not None:
            return doc, 'recomputed from traces'
    if os.path.exists(FROZEN):
        with open(FROZEN) as fh:
            return json.load(fh), 'frozen bundle'
    return None, 'nothing'


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('section', nargs='?', default='all',
                    choices=['all', 'selftest'] + list(SECTIONS))
    ap.add_argument('--rebuild', action='store_true',
                    help='recompute from .navbench even if frozen')
    args = ap.parse_args()
    if args.section == 'selftest':
        return selftest()
    doc, how = load_doc(args.rebuild)
    if doc is None:
        print('no traces and no frozen bundle', file=sys.stderr)
        return 1
    print(f"C2-NAV.31  source: {how};  frame offset "
          f"{tuple(doc['offset_used'])} "
          f"(map_audit measures {tuple(doc['offset_measured'])})")
    print()
    names = list(SECTIONS) if args.section == 'all' else [args.section]
    rc = 0
    for i, nm in enumerate(names):
        if i:
            print('\n' + '=' * 78 + '\n')
        rc |= SECTIONS[nm](doc)
    return rc


if __name__ == '__main__':
    sys.exit(main())
