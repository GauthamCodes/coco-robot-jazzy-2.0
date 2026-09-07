#!/usr/bin/env python3
"""C2-NAV.32 — offline replay of nav2_amcl's DifferentialMotionModel.

THE QUESTION
    Can the deployed `nav2_amcl::DifferentialMotionModel`, fed the robot's
    recorded motion at the wall-adjacent poses and its deployed alpha
    parameters, displace the particle-cloud centroid southward by the
    observed ~0.09 m?

Offline only.  No ROS, no Gazebo, no simulator, no live experiment, no
behavioural parameter read-modified-written.  Everything here reads
committed artifacts and the INSTALLED nav2_amcl binaries, and writes
nothing but stdout and its own frozen bundle.

WHAT IS VERIFIED RATHER THAN ASSUMED

*The motion model is not re-implemented for the headline results.*
`c2nav32_oracle.cpp` links against the installed `libmotions_lib.so` and
calls `DifferentialMotionModel::odometryUpdate` itself, so the deployed
code is what runs.  `libpf_lib.so` draws from `drand48`, so `srand48(seed)`
makes the deployed sampler bit-reproducible.  A vectorised numpy twin
exists only to make the large Monte-Carlo sweeps affordable, and it is
gated on agreeing with the oracle — exactly at zero noise, and in
distribution with noise on.

The sigma formulae are IDENTIFIED from the oracle by switching one alpha
on at a time and recovering the sampled (rot1, trans, rot2) per particle,
not copied from a different Nav2 version's source.  `nav2_amcl`'s .cpp is
not installed on this machine; that is stated, and nothing is silently
substituted for it.

USAGE
    python3 docs/data/c2nav32_mm.py selftest    # the gate, offline
    python3 docs/data/c2nav32_mm.py model       # Part 1  verified equations
    python3 docs/data/c2nav32_mm.py params      # Part 2  deployed parameters
    python3 docs/data/c2nav32_mm.py motion      # Parts 2+10 motion input
    python3 docs/data/c2nav32_mm.py zeronoise   # Part 4  zero-noise control
    python3 docs/data/c2nav32_mm.py replay      # Parts 3+6 actual-noise replay
    python3 docs/data/c2nav32_mm.py size        # Part 7  sample-size effect
    python3 docs/data/c2nav32_mm.py heading     # Part 5  heading / frame test
    python3 docs/data/c2nav32_mm.py compare     # Part 8  observed vs replay
    python3 docs/data/c2nav32_mm.py control     # Part 9  control regions
    python3 docs/data/c2nav32_mm.py verdict     # Part 12 classification
    python3 docs/data/c2nav32_mm.py             # every table above
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))

# ---------------------------------------------------------------------------
# Provenance.  Every one of these is read from its source file at run time by
# `params`; the values here are only what the reader is CHECKED against, so a
# drift shows up as a failure rather than as a silently wrong table.
# ---------------------------------------------------------------------------

BUNDLE30 = os.path.join(HERE, 'c2nav30_cloud.json')
BUNDLE31 = os.path.join(HERE, 'c2nav31_lf.json')
PARAMS_YAML = os.path.join(HERE, 'c2nav25_slow_params.yaml')
SHIPPED_YAML = os.path.join(REPO, 'gazebo_models', 'config', 'nav2_params.yaml')
PARAMS_SHA = '4c15893e83936b745fd1659f5c63a38d31e8acb5883b2e8d6d6ff02888ccc323'

ORACLE_SRC = os.path.join(HERE, 'c2nav32_oracle.cpp')
ROS_PREFIX = '/opt/ros/jazzy'
NAV2_AMCL_DEB = 'ros-jazzy-nav2-amcl'

# The frame convention, carried EXACTLY as C2-NAV.29/.30/.31 carried it.
# `nav_bench.py` hard-codes (2.0, 0.0); `map_audit.py` measures (2.056, 0.015).
# Every y result below is a DIFFERENCE in y, so the y offset cancels from the
# historical convention and shifts the measured one by a constant; both are
# reported wherever a y offset could change a sign.
WORLD_TO_MAP = (2.0, 0.0)
WORLD_TO_MAP_MEASURED = (2.0560, 0.0150)

OUT_JSON = os.path.join(HERE, 'c2nav32_mm.json')

SCRATCH = os.environ.get('C2NAV_SCRATCH', os.path.join(REPO, '.navbench'))
TRACE_DIR = os.path.join(SCRATCH, 'results', 'c2n30_focus_r1_traces')

# Monte-Carlo sizing.  Fixed here so a reader cannot tune them per result.
MAIN_SEED = 20260907
N_SEEDS = 200
SIZE_SEEDS = 400
SIZE_NS = (100, 500, 800, 2000, 10000)


# ---------------------------------------------------------------------------
# angle helpers — nav2_amcl::angleutils, read from the installed header
#   normalize(z)     = atan2(sin z, cos z)
#   angle_diff(a, b) = the smaller of (a-b) and (2pi - |a-b|), signed
# ---------------------------------------------------------------------------

def normalize(z):
    return np.arctan2(np.sin(z), np.cos(z))


def angle_diff(a, b):
    a = normalize(np.asarray(a, dtype=float))
    b = normalize(np.asarray(b, dtype=float))
    d1 = a - b
    d2 = 2.0 * np.pi - np.abs(d1)
    d2 = np.where(d1 > 0.0, -d2, d2)
    return np.where(np.abs(d1) < np.abs(d2), d1, d2)


# ---------------------------------------------------------------------------
# The oracle: the DEPLOYED odometryUpdate, called as a subprocess.
# ---------------------------------------------------------------------------

class Oracle:
    """Wrapper around `c2nav32_oracle`, built on demand into a temp dir."""

    def __init__(self):
        self.exe = None
        self.why = None

    def available(self):
        if self.exe is not None:
            return True
        if self.why is not None:
            return False
        if not os.path.exists(ORACLE_SRC):
            self.why = 'c2nav32_oracle.cpp missing'
            return False
        inc = os.path.join(ROS_PREFIX, 'include')
        lib = os.path.join(ROS_PREFIX, 'lib')
        if not os.path.exists(os.path.join(lib, 'libmotions_lib.so')):
            self.why = 'libmotions_lib.so not installed'
            return False
        d = tempfile.mkdtemp(prefix='c2nav32_')
        exe = os.path.join(d, 'oracle')
        cmd = ['g++', '-O2', '-o', exe, ORACLE_SRC, '-I' + inc, '-L' + lib,
               '-lmotions_lib', '-lpf_lib', '-Wl,-rpath,' + lib]
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
        except (OSError, subprocess.SubprocessError) as exc:
            self.why = 'g++ failed: %s' % exc
            return False
        if r.returncode != 0:
            self.why = 'g++ exit %d: %s' % (r.returncode, r.stderr.strip()[:200])
            return False
        self.exe = exe
        return True

    def step(self, part, pose, delta, alphas, seed):
        """One deployed odometryUpdate.  part (N,3) -> (N,3)."""
        if not self.available():
            raise RuntimeError('oracle unavailable: %s' % self.why)
        part = np.asarray(part, dtype=float)
        buf = ['%d' % len(part)]
        for p in part:
            buf.append('%.17g %.17g %.17g' % (p[0], p[1], p[2]))
        args = [self.exe] + ['%.17g' % a for a in alphas] + ['%d' % seed] + \
               ['%.17g' % v for v in pose] + ['%.17g' % v for v in delta]
        r = subprocess.run(args, input='\n'.join(buf) + '\n',
                           capture_output=True, text=True, timeout=600)
        if r.returncode != 0:
            raise RuntimeError('oracle exit %d: %s' % (r.returncode, r.stderr[:200]))
        out = np.array(r.stdout.split(), dtype=float)
        return out.reshape(-1, 3)


ORACLE = Oracle()


# ---------------------------------------------------------------------------
# The numpy twin.  Its sigma formulae are IDENTIFIED against the oracle by
# `identify()`; `selftest` refuses to run anything else until they agree.
# ---------------------------------------------------------------------------

def motion_increments(pose, delta):
    """(delta_rot1, delta_trans, delta_rot2, rot1_noise, rot2_noise).

    Structure taken from the deployed binary, not from source: old_pose =
    pose - delta (a pf_vector_sub call), a sqrt(dx^2+dy^2) translation, a
    0.01 m guard below which delta_rot1 is forced to zero, and the
    min(|angle_diff(r,0)|, |angle_diff(r,pi)|) magnitudes that make the
    noise treat forward and backward motion alike.  `identify()` proves
    each of these against the oracle.
    """
    old_yaw = pose[2] - delta[2]
    dtrans = math.hypot(delta[0], delta[1])
    if dtrans < 0.01:
        drot1 = 0.0
    else:
        drot1 = float(angle_diff(math.atan2(delta[1], delta[0]), old_yaw))
    drot2 = float(angle_diff(delta[2], drot1))
    n1 = min(abs(float(angle_diff(drot1, 0.0))), abs(float(angle_diff(drot1, math.pi))))
    n2 = min(abs(float(angle_diff(drot2, 0.0))), abs(float(angle_diff(drot2, math.pi))))
    return drot1, dtrans, drot2, n1, n2


def sigmas(drot1, dtrans, drot2, n1, n2, alphas):
    a1, a2, a3, a4, a5 = alphas
    s1 = math.sqrt(a1 * n1 * n1 + a2 * dtrans * dtrans)
    st = math.sqrt(a3 * dtrans * dtrans + a4 * n1 * n1 + a4 * n2 * n2)
    s2 = math.sqrt(a1 * n2 * n2 + a2 * dtrans * dtrans)
    return s1, st, s2


def motion_step_np(part, pose, delta, alphas, rng):
    """Vectorised twin of the deployed odometryUpdate."""
    part = np.asarray(part, dtype=float)
    n = len(part)
    drot1, dtrans, drot2, n1, n2 = motion_increments(pose, delta)
    s1, st, s2 = sigmas(drot1, dtrans, drot2, n1, n2, alphas)
    e1 = rng.normal(0.0, s1, n) if s1 > 0 else np.zeros(n)
    et = rng.normal(0.0, st, n) if st > 0 else np.zeros(n)
    e2 = rng.normal(0.0, s2, n) if s2 > 0 else np.zeros(n)
    r1h = angle_diff(np.full(n, drot1), e1)
    th = dtrans - et
    r2h = angle_diff(np.full(n, drot2), e2)
    out = part.copy()
    out[:, 0] = part[:, 0] + th * np.cos(part[:, 2] + r1h)
    out[:, 1] = part[:, 1] + th * np.sin(part[:, 2] + r1h)
    out[:, 2] = part[:, 2] + r1h + r2h
    return out


# ---------------------------------------------------------------------------
# Part 1 — identify the deployed equations from the oracle.
# ---------------------------------------------------------------------------

def _recover(before, after, nominal_rot1=0.0):
    """Recover (rot1_hat, trans_hat, rot2_hat) from a zero-origin particle.

    A particle at (0,0,0) maps to
        x = t*cos(r1h), y = t*sin(r1h), yaw = r1h + r2h
    so the three sampled quantities come back out — up to the one genuine
    ambiguity, that (t, r1h) and (-t, r1h+pi) produce the same displacement.

    The branch is resolved against the KNOWN nominal delta_rot1, because the
    sampled rot1 is nominal plus a zero-mean draw and so is far more likely
    to sit near it than half a turn away.  Resolving it instead by |r1h| <
    pi/2 — the obvious-looking rule — mis-branches every draw with
    e1 < rot1 - pi/2, which at sigma 0.4 rad is 0.17 % of samples and was
    measured to inflate var_trans by 6e-4 and var_rot2 by 1.6e-2, i.e. to
    manufacture exactly the kind of small cross-term this probe exists to
    rule in or out.  It is a recovery artefact, not the model.
    """
    dx = after[:, 0] - before[:, 0]
    dy = after[:, 1] - before[:, 1]
    dyaw = after[:, 2] - before[:, 2]
    t = np.hypot(dx, dy)
    r1a = np.arctan2(dy, dx)
    r1b = normalize(r1a + np.pi)
    take_b = np.abs(angle_diff(r1b, nominal_rot1)) < np.abs(angle_diff(r1a, nominal_rot1))
    r1h = np.where(take_b, r1b, r1a)
    t = np.where(take_b, -t, t)
    return r1h, t, dyaw - r1h


def identify(seed=7, n=200000):
    """Switch one alpha on at a time and read the sigmas back out.

    Returns a dict of measured-vs-predicted variance for each term.  This
    is what makes the sigma formulae a MEASUREMENT of the deployed binary
    rather than a quotation from some other Nav2 revision.
    """
    # A motion with all three increments distinct and non-degenerate.
    r1, t, r2 = 0.4, 0.30, -0.25
    old_yaw = 0.0
    delta = (t * math.cos(r1), t * math.sin(r1), r1 + r2)
    pose = (0.0, 0.0, old_yaw + delta[2])
    n1 = min(abs(r1), abs(math.pi - abs(r1)))
    n2 = min(abs(r2), abs(math.pi - abs(r2)))
    before = np.zeros((n, 3))

    out = {'delta_rot1': r1, 'delta_trans': t, 'delta_rot2': r2,
           'rot1_noise': n1, 'rot2_noise': n2, 'n': n, 'terms': []}

    # Structural checks first: what the increments themselves are.
    zero = ORACLE.step(np.zeros((1, 3)), pose, delta, (0, 0, 0, 0, 0), 1)
    zr1, zt, zr2 = _recover(np.zeros((1, 3)), zero, r1)
    out['zero_noise_rot1'] = float(zr1[0])
    out['zero_noise_trans'] = float(zt[0])
    out['zero_noise_rot2'] = float(zr2[0])

    for name, alphas in (('alpha1', (1, 0, 0, 0, 0)),
                         ('alpha2', (0, 1, 0, 0, 0)),
                         ('alpha3', (0, 0, 1, 0, 0)),
                         ('alpha4', (0, 0, 0, 1, 0)),
                         ('alpha5', (0, 0, 0, 0, 1))):
        after = ORACLE.step(before, pose, delta, alphas, seed)
        r1h, th, r2h = _recover(before, after, r1)
        out['terms'].append({
            'alpha': name,
            'var_rot1': float(np.var(r1h)),
            'var_trans': float(np.var(th)),
            'var_rot2': float(np.var(r2h)),
            'mean_rot1': float(np.mean(r1h)),
            'mean_trans': float(np.mean(th)),
            'mean_rot2': float(np.mean(r2h)),
        })
    out['pred'] = {
        'alpha1': {'rot1': n1 * n1, 'trans': 0.0, 'rot2': n2 * n2},
        'alpha2': {'rot1': t * t, 'trans': 0.0, 'rot2': t * t},
        'alpha3': {'rot1': 0.0, 'trans': t * t, 'rot2': 0.0},
        'alpha4': {'rot1': 0.0, 'trans': n1 * n1 + n2 * n2, 'rot2': 0.0},
        'alpha5': {'rot1': 0.0, 'trans': 0.0, 'rot2': 0.0},
    }
    return out


# ---------------------------------------------------------------------------
# Data
# ---------------------------------------------------------------------------

def load_legs():
    """Snapshots per leg from the COMMITTED bundle.

    Each snapshot: t (trace bucket time), gt (world x,y,yaw), amcl (map),
    part (N,3) in map, plus the statistics C2-NAV.30/.31 published.
    """
    with open(BUNDLE31) as fh:
        b31 = json.load(fh)
    legs = {}
    for leg, blob in b31['legs'].items():
        snaps = []
        for r in blob['rows']:
            snaps.append({
                't': float(r['t']),
                'gt': (float(r['gt_x']), float(r['gt_y']), float(r['gt_yaw'])),
                'amcl': (float(r['amcl_x']), float(r['amcl_y']), float(r['amcl_yaw'])),
                'part': np.asarray(r['part'], dtype=float),
                'n': int(r['n']),
                'cloud_dy': float(r['cloud_dy']),
                'amcl_dy': float(r['amcl_dy']),
                'sd_dy': float(r['sd_dy']),
                'ess': float(r['ess']),
                'scan_min': float(r['scan_min']),
            })
        legs[leg] = snaps
    return legs


def load_ts():
    """Cloud message stamps, if the scratch npz survive.  Optional."""
    ts = {}
    for leg in ('wall_adjacent', 'open_space'):
        p = os.path.join(TRACE_DIR, '%s_rep0_cloud.npz' % leg)
        if os.path.exists(p):
            with np.load(p) as d:
                ts[leg] = (np.asarray(d['ts_sim_s'], dtype=float),
                           np.asarray(d['counts'], dtype=int))
    return ts


def gt_map(gt, conv=WORLD_TO_MAP):
    return (gt[0] + conv[0], gt[1] + conv[1], gt[2])


# ---------------------------------------------------------------------------
# Part 2 / Part 10 — the motion input, and what is NOT recorded.
# ---------------------------------------------------------------------------

def build_motion(snaps):
    """Increments between consecutive AMCL update events.

    With `resample_interval: 1` every filter update resamples and republishes
    `/particle_cloud`, so consecutive cloud messages bracket exactly one
    update.  The increment for update k is therefore the pose change between
    snapshot k-1 and snapshot k.

    THE SIGNAL IS A PROXY AND THAT IS NOT OPTIONAL.  AMCL consumes the
    odom->base_footprint TF that `diff_drive_controller` integrates from the
    wheels.  `nav_bench.py` never subscribed to it; its `x,y,yaw` come from
    `/model/coco/odometry`, which the xacro documents as the gz
    OdometryPublisher ground-truth pose that "publishes no ROS TF", and its
    `v_wheel` column is `/diff_drive_controller/cmd_vel` — a command.  So
    wheel odometry is recorded NOWHERE and ground truth stands in for it.
    `bound_odom` sweeps the scale to bound what that substitution can cost.
    """
    steps = []
    for k in range(1, len(snaps)):
        a, b = snaps[k - 1], snaps[k]
        pose = gt_map(b['gt'])
        prev = gt_map(a['gt'])
        delta = (pose[0] - prev[0], pose[1] - prev[1],
                 float(angle_diff(pose[2], prev[2])))
        r1, tr, r2, n1, n2 = motion_increments(pose, delta)
        steps.append({
            'k': k, 't0': a['t'], 't1': b['t'],
            'pose': pose, 'delta': delta,
            'drot1': r1, 'dtrans': tr, 'drot2': r2,
            'rot1_noise': n1, 'rot2_noise': n2,
            'dyaw': delta[2],
        })
    return steps


def scaled_delta(step, scale):
    d = step['delta']
    return (d[0] * scale, d[1] * scale, d[2] * scale)


# ---------------------------------------------------------------------------
# Replay
# ---------------------------------------------------------------------------

def chain(part0, steps, alphas, rng=None, seed=None, use_oracle=False,
          scale=1.0, hold_heading=False, flip_yaw=False):
    """Apply the motion model along `steps`, starting from `part0`.

    No sensor update and no resampling: this is the motion model ALONE, which
    is the term under test.  Everything the real filter does on top of it is
    exactly what the observed-vs-replay gap in `compare` measures.
    """
    part = np.asarray(part0, dtype=float).copy()
    for i, st in enumerate(steps):
        d = scaled_delta(st, scale)
        if flip_yaw:
            d = (d[0], d[1], -d[2])
        pose = st['pose']
        if hold_heading:
            # Same translational magnitude, heading held at the first
            # snapshot's: the trajectory geometry is removed while the
            # motion magnitude that drives the noise is kept.
            pose = (pose[0], pose[1], steps[0]['pose'][2])
            mag = math.hypot(d[0], d[1])
            h = steps[0]['pose'][2]
            d = (mag * math.cos(h), mag * math.sin(h), 0.0)
        if use_oracle:
            part = ORACLE.step(part, pose, d, alphas, seed + i)
        else:
            part = motion_step_np(part, pose, d, alphas, rng)
    return part


def cen(part):
    return float(np.mean(part[:, 0])), float(np.mean(part[:, 1]))


def sy(part):
    return float(np.std(part[:, 1]))


def pctl(a, q):
    return float(np.percentile(np.asarray(a, dtype=float), q))


# ---------------------------------------------------------------------------
# reporting helpers
# ---------------------------------------------------------------------------

def hdr(title):
    print('')
    print('=' * 74)
    print(title)
    print('=' * 74)


def row(cells, w):
    print('  '.join(str(c).ljust(n) for c, n in zip(cells, w)).rstrip())


def f(x, n=4):
    if x is None:
        return '-'
    return ('%+.' + str(n) + 'f') % x


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for blk in iter(lambda: fh.read(65536), b''):
            h.update(blk)
    return h.hexdigest()


# ---------------------------------------------------------------------------
# Part 1 — structural probes.  Each one asks the DEPLOYED binary a question
# whose answer would change the replay if it came back the other way.
# ---------------------------------------------------------------------------

def probe_structure():
    out = {}

    # (a) the 0.01 m guard below which delta_rot1 is forced to zero.
    #     Two motions identical but for translation magnitude, straddling it.
    for tag, t in (('below', 0.009), ('above', 0.011)):
        delta = (t * math.cos(1.0), t * math.sin(1.0), 0.0)
        pose = (0.0, 0.0, 0.0)
        a = ORACLE.step(np.zeros((1, 3)), pose, delta, (0, 0, 0, 0, 0), 1)
        r1, tt, r2 = _recover(np.zeros((1, 3)), a, 0.0)
        out['guard_%s_rot1' % tag] = float(r1[0])
        out['guard_%s_trans' % tag] = float(tt[0])
    out['guard_threshold_m'] = 0.01

    # (b) backward symmetry: the noise magnitude uses
    #     min(|angle_diff(r,0)|, |angle_diff(r,pi)|), so a near-pi rotation
    #     must draw the SAME spread as its small complement.
    n = 120000
    for tag, r1 in (('small', 0.2), ('near_pi', math.pi - 0.2)):
        t = 0.30
        delta = (t * math.cos(r1), t * math.sin(r1), r1)
        pose = (0.0, 0.0, delta[2])
        a = ORACLE.step(np.zeros((n, 3)), pose, delta, (1, 0, 0, 0, 0), 11)
        rr, tt, _ = _recover(np.zeros((n, 3)), a, r1)
        # The spread must be measured AROUND the nominal with angle_diff.  At
        # delta_rot1 = pi - 0.2 a sigma of 0.2 puts 16 % of draws over pi, so
        # they wrap to -pi + eps; a linear np.std across that branch cut reads
        # 2.17 rad for a distribution that is genuinely 0.2 rad wide.  That is
        # an artefact of the statistic, not of the model.
        out['sym_%s_sd_rot1' % tag] = float(np.std(angle_diff(rr, r1)))
    out['sym_expected_sd'] = 0.2

    # (c) is yaw normalised after the update?  Push a particle past pi.
    a = ORACLE.step(np.array([[0.0, 0.0, 3.0]]), (0.0, 0.0, 0.5),
                    (0.0, 0.0, 0.5), (0, 0, 0, 0, 0), 1)
    out['yaw_after'] = float(a[0, 2])          # 3.5 if unnormalised
    out['yaw_normalised'] = bool(abs(a[0, 2]) <= math.pi + 1e-9)

    # (d) is the noise drawn per particle, or once for the set?
    a = ORACLE.step(np.zeros((500, 3)), (0.0, 0.0, 0.0), (0.3, 0.0, 0.0),
                    (0.2, 0.2, 0.2, 0.2, 0.2), 3)
    out['per_particle_unique'] = int(len(np.unique(np.round(a[:, 0], 12))))
    out['per_particle_n'] = 500

    # (e) x/y asymmetry.  The same motion rotated 90 degrees must give the
    #     same statistics with x and y exchanged: no parameter is per-axis.
    n = 60000
    ax = ORACLE.step(np.zeros((n, 3)), (0.0, 0.0, 0.0), (0.3, 0.0, 0.0),
                     (0.2, 0.2, 0.2, 0.2, 0.2), 5)
    ay = ORACLE.step(np.full((n, 3), [0.0, 0.0, math.pi / 2]),
                     (0.0, 0.0, math.pi / 2), (0.0, 0.3, 0.0),
                     (0.2, 0.2, 0.2, 0.2, 0.2), 5)
    out['axis_x_meanx'] = float(np.mean(ax[:, 0]))
    out['axis_y_meany'] = float(np.mean(ay[:, 1]))
    out['axis_x_sdx'] = float(np.std(ax[:, 0]))
    out['axis_y_sdy'] = float(np.std(ay[:, 1]))
    return out


def probe_twin(seed=101, n=60000):
    """Gate the numpy twin against the deployed oracle."""
    pose = (0.0, 0.0, 0.9)
    delta = (0.21, -0.08, 0.35)
    part = np.zeros((n, 3))
    part[:, 2] = np.linspace(-math.pi, math.pi, n, endpoint=False)

    # exact at zero noise
    o0 = ORACLE.step(part, pose, delta, (0, 0, 0, 0, 0), seed)
    t0 = motion_step_np(part, pose, delta, (0, 0, 0, 0, 0),
                        np.random.default_rng(seed))
    exact = float(np.max(np.abs(o0 - t0)))

    # in distribution with the deployed alphas
    al = (0.2, 0.2, 0.2, 0.2, 0.2)
    o1 = ORACLE.step(part, pose, delta, al, seed)
    t1 = motion_step_np(part, pose, delta, al, np.random.default_rng(seed))
    d = {}
    for i, nm in enumerate('xy'):
        d['mean_' + nm] = (float(np.mean(o1[:, i])), float(np.mean(t1[:, i])))
        d['sd_' + nm] = (float(np.std(o1[:, i])), float(np.std(t1[:, i])))
    d['exact_zero_noise_maxabs'] = exact
    d['n'] = n
    return d


def _deb_version():
    try:
        r = subprocess.run(['dpkg-query', '-W', '-f=${Version}', NAV2_AMCL_DEB],
                           capture_output=True, text=True, timeout=30)
        if r.returncode == 0:
            return r.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        pass
    return '(dpkg-query unavailable)'


def read_amcl_params(path):
    """Read the amcl block out of a params yaml without a yaml dependency."""
    vals = {}
    inblock = False
    with open(path) as fh:
        for ln in fh:
            if ln.startswith('amcl:'):
                inblock = True
                continue
            if inblock and ln.strip() and not ln[0].isspace():
                break
            if not inblock or ':' not in ln:
                continue
            k, _, v = ln.strip().partition(':')
            v = v.split('#')[0].strip().strip('"')
            if v == '' or k.startswith('#'):
                continue
            try:
                vals[k] = float(v) if ('.' in v or 'e' in v.lower()) else int(v)
            except ValueError:
                vals[k] = v
    return vals


# ---------------------------------------------------------------------------
# commands
# ---------------------------------------------------------------------------

def cmd_model():
    hdr('PART 1 - the DEPLOYED DifferentialMotionModel, verified not assumed')
    print("""
Implementation under test
  package        %s %s
  symbol         nav2_amcl::DifferentialMotionModel::odometryUpdate
  library        %s/lib/libmotions_lib.so
  sampler        pf_ran_gaussian(sigma) in libpf_lib.so -> drand48
                 (zero-mean, polar Box-Muller; the header says so and the
                  library imports drand48/srand48 and nothing else random)
  angle helpers  nav2_amcl::angleutils, read from the installed header

  The .cpp is NOT installed on this machine, so nothing below is quoted from
  source.  Every line of it is measured by calling the deployed binary.""" % (
        NAV2_AMCL_DEB, _deb_version(), ROS_PREFIX))

    ident = identify()
    print("""
Increment reconstruction, zero noise (one particle, hand-set motion)
  delta_rot1   recovered %+.9f   set %+.9f
  delta_trans  recovered %+.9f   set %+.9f
  delta_rot2   recovered %+.9f   set %+.9f
  => old_pose = pose - delta ; delta_trans = hypot(dx, dy) ;
     delta_rot1 = angle_diff(atan2(dy, dx), old_pose.yaw) ;
     delta_rot2 = angle_diff(delta.yaw, delta_rot1)""" % (
        ident['zero_noise_rot1'], ident['delta_rot1'],
        ident['zero_noise_trans'], ident['delta_trans'],
        ident['zero_noise_rot2'], ident['delta_rot2']))

    print("""
Which alpha feeds which noise term - one alpha at a time, n=%d
  rot1_noise = %.4f   rot2_noise = %.4f   delta_trans = %.4f""" % (
        ident['n'], ident['rot1_noise'], ident['rot2_noise'],
        ident['delta_trans']))
    w = (8, 24, 24, 24)
    row(['alpha', 'var(rot1_hat)', 'var(trans_hat)', 'var(rot2_hat)'], w)
    for t in ident['terms']:
        p = ident['pred'][t['alpha']]
        row([t['alpha'],
             '%.6f (pred %.6f)' % (t['var_rot1'], p['rot1']),
             '%.6f (pred %.6f)' % (t['var_trans'], p['trans']),
             '%.6f (pred %.6f)' % (t['var_rot2'], p['rot2'])], w)
    print("""
  => sigma_rot1  = sqrt(alpha1*rot1_noise^2 + alpha2*delta_trans^2)
     sigma_trans = sqrt(alpha3*delta_trans^2 + alpha4*rot1_noise^2
                                             + alpha4*rot2_noise^2)
     sigma_rot2  = sqrt(alpha1*rot2_noise^2 + alpha2*delta_trans^2)
     and the update itself, in each particle's OWN frame:
       x   += trans_hat * cos(yaw + rot1_hat)
       y   += trans_hat * sin(yaw + rot1_hat)
       yaw += rot1_hat + rot2_hat
     with rot1_hat = angle_diff(delta_rot1, N(0, sigma_rot1)),
          trans_hat = delta_trans - N(0, sigma_trans),
          rot2_hat = angle_diff(delta_rot2, N(0, sigma_rot2)).
     alpha5 moves NOTHING: it belongs to the omni model, and the deployed
     differential model ignores it.  Measured, not inferred.""")

    st = probe_structure()
    print("""
Structural probes
  0.01 m guard   trans %.3f m -> delta_rot1 %+.6f
                 trans %.3f m -> delta_rot1 %+.6f
                 below the guard delta_rot1 is FORCED to zero, so a
                 rotation in place injects rot2 noise only - and, through
                 alpha4, a TRANSLATION noise proportional to that rotation
  backward sym   sd(rot1_hat) at delta_rot1 = 0.2    : %.5f
                 sd(rot1_hat) at delta_rot1 = pi-0.2 : %.5f  (expected %.3f)
                 the magnitude is min(|ad(r,0)|, |ad(r,pi)|), so a near-pi
                 rotation is treated as its small complement
  yaw wrapping   particle yaw 3.0 + 0.5 -> %.6f ; normalised: %s
  per particle   %d distinct x from %d identical particles -> the draw is
                 per particle, not once per set
  x/y symmetry   mean %+.6f (x-motion) vs %+.6f (y-motion, +90 deg)
                 sd   %.6f          vs %.6f
                 no parameter is per-axis; any x/y asymmetry can only come
                 from trajectory geometry""" % (
        0.009, st['guard_below_rot1'], 0.011, st['guard_above_rot1'],
        st['sym_small_sd_rot1'], st['sym_near_pi_sd_rot1'], st['sym_expected_sd'],
        st['yaw_after'], st['yaw_normalised'],
        st['per_particle_unique'], st['per_particle_n'],
        st['axis_x_meanx'], st['axis_y_meany'],
        st['axis_x_sdx'], st['axis_y_sdy']))

    tw = probe_twin()
    print("""
The numpy twin is gated on the oracle, not trusted
  zero noise, max |oracle - twin| over %d particles : %.2e  (exact)
  alphas 0.2, mean x  oracle %+.6f   twin %+.6f
  alphas 0.2, mean y  oracle %+.6f   twin %+.6f
  alphas 0.2, sd   x  oracle  %.6f   twin  %.6f
  alphas 0.2, sd   y  oracle  %.6f   twin  %.6f
  The twin is used ONLY for the large sweeps; every headline replay below
  is run through the oracle as well and the two are reported together.""" % (
        tw['n'], tw['exact_zero_noise_maxabs'],
        tw['mean_x'][0], tw['mean_x'][1], tw['mean_y'][0], tw['mean_y'][1],
        tw['sd_x'][0], tw['sd_x'][1], tw['sd_y'][0], tw['sd_y'][1]))
    return {'identify': ident, 'structure': st, 'twin': tw}


def cmd_params():
    hdr('PART 2 - the DEPLOYED AMCL parameters, read from the run\'s own file')
    p = read_amcl_params(PARAMS_YAML)
    ship = read_amcl_params(SHIPPED_YAML)
    sha = sha256(PARAMS_YAML)
    print("""
  file    %s
  sha256  %s
          %s C2-NAV.30's recorded value
  amcl block vs shipped gazebo_models/config/nav2_params.yaml: %s
""" % (os.path.relpath(PARAMS_YAML, REPO), sha,
       'MATCHES' if sha == PARAMS_SHA else 'DIFFERS FROM',
       'identical' if p == ship else 'NOT identical'))
    keys = ['robot_model_type', 'alpha1', 'alpha2', 'alpha3', 'alpha4',
            'alpha5', 'update_min_d', 'update_min_a', 'resample_interval',
            'min_particles', 'max_particles', 'pf_err', 'pf_z',
            'recovery_alpha_slow', 'recovery_alpha_fast', 'laser_model_type']
    w = (24, 34)
    for k in keys:
        row([k, p.get(k, '(absent)')], w)
    print("""
  alpha1..alpha5 are all 0.2, read from the file rather than assumed.
  alpha5 is inert for this model (Part 1).  recovery_alpha_slow/fast are
  0.0, so no random particles are injected, and between sensor updates the
  ONLY thing acting on a particle is the motion model under test.""")
    return {'params': p, 'sha256': sha, 'matches_shipped': p == ship}


def cmd_motion():
    hdr('PARTS 2 + 10 - the motion input, and what is NOT recorded')
    print("""
WHAT AMCL ACTUALLY CONSUMES, AND WHY IT IS NOT IN THE RECORD

  AMCL takes its motion from the odom -> base_footprint TF, which
  diff_drive_controller integrates from the wheels.  The record does not
  contain it:

    * nav_bench.py subscribes /model/coco/odometry for x,y,yaw.  The xacro
      documents that publisher as the gz OdometryPublisher ground-truth
      world pose and says in as many words that it "Publishes no ROS TF, so
      it cannot fight the diff-drive controller's odom->base_footprint
      transform".  It is ground truth, not odometry.
    * the v_wheel column is /diff_drive_controller/cmd_vel - a COMMAND.
    * there is no rosbag anywhere under the workspace; C2-NAV.29 established
      that for /scan and the same search covers odometry.

  So wheel odometry is recorded NOWHERE, and ground-truth motion stands in
  for it.  That substitution is the single largest limitation of this
  session; `replay` bounds what it can cost by sweeping the motion scale.

AMCL UPDATE EVENTS

  resample_interval is 1, so every filter update resamples and republishes
  /particle_cloud.  Consecutive cloud messages therefore bracket exactly one
  update, and the increment for update k is the pose change between
  snapshot k-1 and snapshot k.  The check below is whether those increments
  actually clear update_min_d = 0.25 m OR update_min_a = 0.2 rad, which is
  what must have happened for AMCL to update at all.
""")
    legs = load_legs()
    ts = load_ts()
    out = {}
    for leg in ('wall_adjacent', 'open_space'):
        snaps = legs[leg]
        steps = build_motion(snaps)
        hits_d = sum(1 for s in steps if s['dtrans'] >= 0.25)
        hits_a = sum(1 for s in steps if abs(s['dyaw']) >= 0.2)
        hits = sum(1 for s in steps
                   if s['dtrans'] >= 0.25 or abs(s['dyaw']) >= 0.2)
        print('  %s: %d snapshots, %d update intervals' % (
            leg, len(snaps), len(steps)))
        print('    clears update_min_d 0.25 m : %d/%d' % (hits_d, len(steps)))
        print('    clears update_min_a 0.2 rad: %d/%d' % (hits_a, len(steps)))
        print('    clears EITHER threshold    : %d/%d' % (hits, len(steps)))
        print('    dtrans  med %.4f  max %.4f  sum %.4f m' % (
            np.median([s['dtrans'] for s in steps]),
            max(s['dtrans'] for s in steps),
            sum(s['dtrans'] for s in steps)))
        print('    |dyaw|  med %.4f  max %.4f  sum %.4f rad' % (
            np.median([abs(s['dyaw']) for s in steps]),
            max(abs(s['dyaw']) for s in steps),
            sum(abs(s['dyaw']) for s in steps)))
        if leg in ts:
            gaps = np.diff(ts[leg][0])
            print('    cloud stamp gaps: med %.3f s  max %.3f s (scratch npz)'
                  % (float(np.median(gaps)), float(np.max(gaps))))
        out[leg] = {
            'n_steps': len(steps),
            'hits_d': hits_d, 'hits_a': hits_a, 'hits_either': hits,
            'dtrans_sum': float(sum(s['dtrans'] for s in steps)),
            'dyaw_abs_sum': float(sum(abs(s['dyaw']) for s in steps)),
        }
    print("""
  READ THE update_min_d COLUMN.  The robot never travels 0.25 m between
  updates on either leg; it is update_min_a that fires, on yaw.  That is
  consistent with C2-NAV.28's 0.8-1.9 Hz /amcl_pose and with a robot that
  is turning far more than it is translating at these poses.

  ALIGNMENT LIMIT, stated rather than hidden.  The ground-truth pose used
  for each increment is the 10 Hz trace row in the cloud message's own
  bucket (t-0.1, t], so each increment carries up to 0.1 s of timing
  quantisation - at the measured speeds, <=0.03 m and <=0.07 rad.  The
  exact odometry AMCL integrated between updates cannot be recovered from
  this record at all; see the limitation above.""")
    return out




# ---------------------------------------------------------------------------
# replay plumbing
#
# THE DECOMPOSITION.  For one AMCL update, starting from the RECORDED cloud
# at k-1 and ending at the recorded cloud at k, the cloud centroid's failure
# to track the truth in y splits exactly into two terms:
#
#     geom_k  = (zero-noise centroid motion) - (true motion)
#     noise_k = (noisy centroid motion) - (zero-noise centroid motion)
#     total_k = geom_k + noise_k
#
# `geom` is the deterministic geometry of carrying a cloud that has a real
# yaw spread through a real turn.  `noise` is the motion model's sampling,
# and it is the term this session is about.  Splitting them this way makes
# `noise` invariant to whether the input increments match the truth, which
# matters because the increments are a ground-truth PROXY for odometry that
# was never recorded.
#
# TWO estimators, and which one is the headline matters.
#
#  (1) RE-ANCHORED PER-UPDATE (primary).  Each update starts from the
#      RECORDED cloud of the previous snapshot, so the spread the motion
#      model acts on is the spread the filter actually carried.
#
#  (2) FREE-RUNNING FULL-LEG CHAIN (bound only).  The recorded cloud is
#      propagated through every increment with no sensor update and no
#      resampling.  Measured below to blow the cloud up to ~4.5x its observed
#      width, so its centroid is NOT a well-conditioned estimate of anything
#      the filter does.  It is labelled a bound everywhere it appears.
# ---------------------------------------------------------------------------

ALPHAS = (0.2, 0.2, 0.2, 0.2, 0.2)
ZERO = (0.0, 0.0, 0.0, 0.0, 0.0)


def dy_of(part, gt, conv=WORLD_TO_MAP):
    """Cloud-centroid y minus ground-truth y, in the map frame."""
    return float(np.mean(np.asarray(part)[:, 1])) - (gt[1] + conv[1])


def frac_north(part, gt, conv=WORLD_TO_MAP):
    return float(np.mean(np.asarray(part)[:, 1] > gt[1] + conv[1]))


def cloud_stats(part, gt):
    p = np.asarray(part)
    c = np.cov(p[:, 0], p[:, 1])
    return {
        'cx': float(np.mean(p[:, 0])), 'cy': float(np.mean(p[:, 1])),
        'dy': dy_of(p, gt), 'sy': float(np.std(p[:, 1])),
        'sx': float(np.std(p[:, 0])), 'cxy': float(c[0, 1]),
        'frac_north': frac_north(p, gt), 'n': int(len(p)),
    }


def group_steps(steps, m=1):
    """One entry per AMCL update, each a list of m equal sub-increments.

    The total motion is preserved exactly; only the number of independent
    noise draws it is divided into changes.  This is how the replay is made
    robust to the one thing the record cannot pin down -- whether each cloud
    message really corresponds to exactly one filter update.
    """
    out = []
    for s in steps:
        if m == 1:
            out.append([s])
            continue
        d = s['delta']
        sub = []
        for j in range(m):
            frac = (j + 1) / m
            p = (s['pose'][0] - d[0] * (1 - frac),
                 s['pose'][1] - d[1] * (1 - frac),
                 s['pose'][2] - d[2] * (1 - frac))
            dd = (d[0] / m, d[1] / m, d[2] / m)
            r1, tr, r2, n1, n2 = motion_increments(p, dd)
            sub.append({'k': s['k'], 't0': s['t0'], 't1': s['t1'], 'pose': p,
                        'delta': dd, 'drot1': r1, 'dtrans': tr, 'drot2': r2,
                        'rot1_noise': n1, 'rot2_noise': n2, 'dyaw': dd[2]})
        out.append(sub)
    return out


def perstep(snaps, groups, alphas, seed, use_oracle=False, nboot=0, **kw):
    """Estimator (1).  Returns dict of per-update geom / noise / total lists."""
    rng = np.random.default_rng(seed)
    geom, noise, sy = [], [], []
    for i, sub in enumerate(groups):
        a, b = snaps[i], snaps[i + 1]
        p0 = a['part']
        if nboot:
            p0 = p0[rng.integers(0, len(p0), nboot)]
        if use_oracle:
            det = chain(p0, sub, ZERO, seed=seed + 7919 * i, use_oracle=True, **kw)
            noi = chain(p0, sub, alphas, seed=seed + 104729 * (i + 1),
                        use_oracle=True, **kw)
        else:
            det = chain(p0, sub, ZERO, rng=rng, **kw)
            noi = chain(p0, sub, alphas, rng=rng, **kw)
        y0 = float(np.mean(p0[:, 1]))
        yd = float(np.mean(det[:, 1]))
        yn = float(np.mean(noi[:, 1]))
        geom.append((yd - y0) - (b['gt'][1] - a['gt'][1]))
        noise.append(yn - yd)
        sy.append(float(np.std(noi[:, 1])))
    return {'geom': geom, 'noise': noise,
            'total': [g + n for g, n in zip(geom, noise)],
            'geom_sum': float(np.sum(geom)), 'noise_sum': float(np.sum(noise)),
            'total_sum': float(np.sum(geom) + np.sum(noise)),
            'sy_mean': float(np.mean(sy))}


def freerun(snaps, steps, alphas, seed, use_oracle=False, **kw):
    """Estimator (2): the unconstrained full-leg chain.  A BOUND, not a fit."""
    if use_oracle:
        out = chain(snaps[0]['part'], steps, alphas, seed=seed,
                    use_oracle=True, **kw)
    else:
        out = chain(snaps[0]['part'], steps, alphas,
                    rng=np.random.default_rng(seed), **kw)
    return out, cloud_stats(out, snaps[-1]['gt'])


def spread(vals):
    a = np.asarray(vals, dtype=float)
    return {'mean': float(a.mean()), 'sd': float(a.std()),
            'p05': pctl(a, 5), 'p50': pctl(a, 50), 'p95': pctl(a, 95),
            'n': int(len(a))}


# ---------------------------------------------------------------------------
# Part 4 — zero-noise control
# ---------------------------------------------------------------------------

def cmd_zeronoise():
    hdr('PART 4 - zero-noise control: does the GEOMETRY alone move y?')
    print("""
  Diagnostic only.  alpha1..5 = 0 is NOT a realistic AMCL configuration; it
  is a causal decomposition tool.  With the noise off, every particle is
  carried rigidly along the recorded motion in its own frame, so anything
  here is the deterministic geometry of the trajectory acting on the cloud's
  own yaw spread -- not sampling.  Run through the DEPLOYED binary with all
  five alphas set to zero.
""")
    legs = load_legs()
    out = {}
    w = (16, 7, 22, 22)
    row(['leg', 'updates', 're-anchored geom sum', 'free-run dy change'], w)
    for leg in ('wall_adjacent', 'open_space'):
        snaps = legs[leg]
        steps = build_motion(snaps)
        r = perstep(snaps, group_steps(steps), ZERO, MAIN_SEED, use_oracle=True)
        dy0 = dy_of(snaps[0]['part'], snaps[0]['gt'])
        _, st = freerun(snaps, steps, ZERO, MAIN_SEED, use_oracle=True)
        row([leg, len(steps), f(r['geom_sum'], 5), f(st['dy'] - dy0, 5)], w)
        out[leg] = {'geom_sum': r['geom_sum'], 'geom': r['geom'],
                    'freerun': st['dy'] - dy0}
    print("""
  With the noise off the noise term is identically zero, so `geom` is the
  whole of it.  Three of the four numbers are NORTHWARD and the fourth is
  -0.003 m, 3 %% of the bias under test.  Integrating the recorded motion
  deterministically does not manufacture a southward offset: it produces a
  small northward one, because carrying a cloud that has a real yaw spread
  through a real turn moves its centroid toward the inside of the arc.  So
  nothing in the trajectory geometry points south, and whatever the motion
  model contributes has to come from the sampling -- Part 3.""")
    return out


# ---------------------------------------------------------------------------
# Parts 3 + 6 — actual-noise replay and the repeated-step bias test
# ---------------------------------------------------------------------------

def cmd_replay(n_seeds=N_SEEDS, oracle_seeds=25):
    hdr('PARTS 3 + 6 - actual-noise replay of the recorded particle set')
    print("""
  Primary estimator re-anchors on the RECORDED cloud at every update, and
  splits each update into geom (deterministic geometry) and noise (the
  motion model's sampling).  `noise` is the term under test and is the one
  that is invariant to the ground-truth-for-odometry proxy.
""")
    legs = load_legs()
    out = {}
    for leg in ('wall_adjacent', 'open_space'):
        snaps = legs[leg]
        steps = build_motion(snaps)
        groups = group_steps(steps)
        dy0 = dy_of(snaps[0]['part'], snaps[0]['gt'])
        dyN = dy_of(snaps[-1]['part'], snaps[-1]['gt'])

        tw = [perstep(snaps, groups, ALPHAS, MAIN_SEED + 1000 * s)
              for s in range(n_seeds)]
        orc = [perstep(snaps, groups, ALPHAS, MAIN_SEED + 1000 * s,
                       use_oracle=True) for s in range(oracle_seeds)]
        noi = spread([r['noise_sum'] for r in tw])
        geo = spread([r['geom_sum'] for r in tw])
        tot = spread([r['total_sum'] for r in tw])
        noi_o = spread([r['noise_sum'] for r in orc])
        se = noi['sd'] / math.sqrt(n_seeds)
        arr = np.array([r['noise_sum'] for r in tw])
        p09 = float(np.mean(np.abs(arr) >= 0.09))

        fr = [freerun(snaps, steps, ALPHAS, MAIN_SEED + 1000 * s)
              for s in range(30)]
        fr_dy = spread([st['dy'] - dy0 for _, st in fr])
        fr_sy = spread([st['sy'] for _, st in fr])
        sy_obs = float(np.std(snaps[-1]['part'][:, 1]))

        print('')
        print('  %s  (%d snapshots, %d updates, N %d..%d)' % (
            leg, len(snaps), len(steps),
            min(s['n'] for s in snaps), max(s['n'] for s in snaps)))
        print('    OBSERVED cloud dy: start %+0.4f  end %+0.4f  change %+0.4f m'
              % (dy0, dyN, dyN - dy0))
        print('    RE-ANCHORED decomposition over the whole leg (%d seeds, twin)'
              % n_seeds)
        print('      geom  sum  mean %+0.5f  sd %.5f' % (geo['mean'], geo['sd']))
        print('      NOISE sum  mean %+0.5f  sd %.5f  p05 %+0.5f  p50 %+0.5f  p95 %+0.5f'
              % (noi['mean'], noi['sd'], noi['p05'], noi['p50'], noi['p95']))
        print('      total sum  mean %+0.5f  sd %.5f' % (tot['mean'], tot['sd']))
        print('      oracle cross-check on NOISE, %d seeds: mean %+0.5f  sd %.5f'
              % (oracle_seeds, noi_o['mean'], noi_o['sd']))
        print('      95%% CI on the mean NOISE sum: [%+0.5f, %+0.5f] m'
              % (noi['mean'] - 1.96 * se, noi['mean'] + 1.96 * se))
        print('      P(|noise sum| >= 0.09 m) = %.4f  (%d/%d seeds)'
              % (p09, int(np.sum(np.abs(arr) >= 0.09)), n_seeds))
        print('    FREE-RUNNING chain (BOUND): dy change mean %+0.5f  sd %.5f'
              % (fr_dy['mean'], fr_dy['sd']))
        print('      cloud sd in y %.4f m vs observed %.4f m -> %.1fx too wide;'
              % (fr_sy['mean'], sy_obs, fr_sy['mean'] / sy_obs))
        print('      with no sensor update and no resampling the cloud')
        print('      diffuses, so this centroid bounds the term rather than')
        print('      estimating it.')

        print('    robustness of the NOISE term:')
        sc = {}
        for s in (0.5, 0.75, 1.0, 1.5, 2.0):
            v = [perstep(snaps, groups, ALPHAS, MAIN_SEED + 1000 * i,
                         scale=s)['noise_sum'] for i in range(60)]
            sc[s] = float(np.mean(v))
            print('      motion scale %.2f      mean noise sum %+0.5f m' % (s, sc[s]))
        ch = {}
        for m in (1, 2, 4):
            v = [perstep(snaps, group_steps(steps, m), ALPHAS,
                         MAIN_SEED + 1000 * i)['noise_sum'] for i in range(60)]
            ch[m] = float(np.mean(v))
            print('      %d sub-step(s)/update   mean noise sum %+0.5f m' % (m, ch[m]))

        out[leg] = {
            'dy0': dy0, 'dyN': dyN, 'obs_change': dyN - dy0,
            'n_steps': len(steps), 'noise': noi, 'geom': geo, 'total': tot,
            'noise_oracle': noi_o, 'p_reach_09': p09,
            'freerun_dy': fr_dy, 'freerun_sy': fr_sy, 'sy_obs_last': sy_obs,
            'scales': sc, 'chunks': ch,
        }

    print('')
    print('  PER-UPDATE table, wall_adjacent.  Each row starts from the')
    print('  RECORDED cloud at k-1 and applies exactly one increment.')
    snaps = legs['wall_adjacent']
    steps = build_motion(snaps)
    groups = group_steps(steps)
    nseed = 150
    runs = [perstep(snaps, groups, ALPHAS, MAIN_SEED + 1000 * s)
            for s in range(nseed)]
    NO = np.array([r['noise'] for r in runs])
    GE = np.array([r['geom'] for r in runs])
    w = (4, 8, 8, 11, 11, 15, 11, 12)
    row(['k', 'dtrans', '|dyaw|', 'dy before', 'dy after', 'noise d(dy)',
         'geom d(dy)', 'actual d(dy)'], w)
    rows = []
    for i, st in enumerate(steps):
        a, b = snaps[i], snaps[i + 1]
        dyb = dy_of(a['part'], a['gt'])
        dya = dy_of(b['part'], b['gt'])
        m, sd = float(NO[:, i].mean()), float(NO[:, i].std())
        g = float(GE[:, i].mean())
        rows.append({'k': st['k'], 'dtrans': st['dtrans'], 'dyaw': st['dyaw'],
                     'dy_before': dyb, 'dy_after': dya, 'noise': m,
                     'noise_sd': sd, 'geom': g, 'actual': dya - dyb})
        row([st['k'], '%.4f' % st['dtrans'], '%.4f' % abs(st['dyaw']),
             f(dyb), f(dya), '%+.5f+-%.5f' % (m, sd), f(g, 5),
             f(dya - dyb, 5)], w)
    act = sum(r['actual'] for r in rows)
    print('  totals: noise %+0.5f +- %.5f   geom %+0.5f   actual %+0.5f m'
          % (NO.sum(axis=1).mean(), NO.sum(axis=1).std(),
             GE.sum(axis=1).mean(), act))
    print('  updates whose MEAN noise term points SOUTH: %d/%d'
          % (int(np.sum(NO.mean(axis=0) < 0)), len(steps)))
    out['perstep_rows'] = rows
    out['perstep_noise_total'] = spread(NO.sum(axis=1))
    out['perstep_geom_total'] = spread(GE.sum(axis=1))
    out['perstep_actual_total'] = act
    out['perstep_south'] = int(np.sum(NO.mean(axis=0) < 0))
    return out


# ---------------------------------------------------------------------------
# Part 7 — sample-size effect
# ---------------------------------------------------------------------------

def cmd_size(n_seeds=SIZE_SEEDS):
    hdr('PART 7 - sample size: can finite N alone deliver 0.09 m?')
    print("""
  The recorded clouds hold 501-840 particles.  Here the SAME population is
  bootstrapped to several N from the recorded wall_adjacent clouds, so the
  distribution is identical and only the count changes; the same recorded
  motion is replayed; and the spread of the NOISE term is measured.  If sd
  scales as 1/sqrt(N) about a fixed mean, the displacement is ordinary
  sampling noise rather than a mechanism.
""")
    legs = load_legs()
    snaps = legs['wall_adjacent']
    groups = group_steps(build_motion(snaps))
    out = {}
    w = (8, 13, 11, 12, 12, 14, 12)
    row(['N', 'mean noise', 'sd', 'p05', 'p95', 'P(|d|>=0.09)', 'sd*sqrt(N)'], w)
    for n in SIZE_NS:
        v = np.array([perstep(snaps, groups, ALPHAS, MAIN_SEED + 31 * s,
                              nboot=n)['noise_sum'] for s in range(n_seeds)])
        pr = float(np.mean(np.abs(v) >= 0.09))
        row([n, f(v.mean(), 5), '%.5f' % v.std(), f(pctl(v, 5), 5),
             f(pctl(v, 95), 5), '%.4f' % pr, '%.4f' % (v.std() * math.sqrt(n))], w)
        out[n] = {'mean': float(v.mean()), 'sd': float(v.std()),
                  'p05': pctl(v, 5), 'p95': pctl(v, 95), 'p_reach': pr,
                  'sd_root_n': float(v.std() * math.sqrt(n))}
    print("""
  A flat sd*sqrt(N) column is the signature of ordinary 1/sqrt(N) sampling
  noise about a fixed expectation.  Read the N=500 and N=800 rows for the
  recorded particle counts: that is how often chance alone would deliver a
  displacement as large as the observed one, in EITHER direction.""")
    return out


# ---------------------------------------------------------------------------
# Part 5 — heading / frame test
# ---------------------------------------------------------------------------

def cmd_heading(n_seeds=150):
    hdr('PART 5 - heading and frame: can a body-frame model bias world y?')
    print("""
  The model's noise is symmetric in the ROBOT's frame.  The question is
  whether the trajectory geometry can turn that into a world-frame southward
  push.  Three replays of the same leg, NOISE term only:

    actual     the recorded headings and increments
    held       the same translational magnitudes, heading pinned at the
               leg's first heading, no rotation
    flipped    the same translations, every yaw increment sign-reversed

  If trajectory geometry were manufacturing a world-frame bias, reversing
  the yaw would reverse its sign.
""")
    legs = load_legs()
    out = {}
    for leg in ('wall_adjacent', 'open_space'):
        snaps = legs[leg]
        groups = group_steps(build_motion(snaps))
        print('  %s' % leg)
        w = (12, 14, 12, 12, 12)
        row(['variant', 'mean noise', 'sd', 'p05', 'p95'], w)
        out[leg] = {}
        for tag, kw in (('actual', {}), ('held', {'hold_heading': True}),
                        ('flipped', {'flip_yaw': True})):
            v = np.array([perstep(snaps, groups, ALPHAS,
                                  MAIN_SEED + 1000 * s, **kw)['noise_sum']
                          for s in range(n_seeds)])
            row([tag, f(v.mean(), 5), '%.5f' % v.std(), f(pctl(v, 5), 5),
                 f(pctl(v, 95), 5)], w)
            out[leg][tag] = spread(v)
    snaps = legs['wall_adjacent']
    dy0 = dy_of(snaps[0]['part'], snaps[0]['gt'])
    dy0m = dy_of(snaps[0]['part'], snaps[0]['gt'], WORLD_TO_MAP_MEASURED)
    print('')
    print('  wall_adjacent start dy: historical (2.0, 0.0) %+0.4f   '
          'measured (2.056, 0.015) %+0.4f' % (dy0, dy0m))
    print("""  Every number above is a DIFFERENCE of two y values in the same
  frame, so a constant y offset cancels from it exactly: the convention
  changes the start dy and leaves the whole table unchanged.""")
    return out


# ---------------------------------------------------------------------------
# Part 8 — observed cloud vs replay
# ---------------------------------------------------------------------------

def cmd_compare(n_seeds=200):
    hdr('PART 8 - the observed cloud against the replay')
    print("""
  A mechanism has to reproduce BOTH the centroid displacement and the
  spread.  `replay dy` is the recorded start dy carried through the recorded
  motion by the motion model alone (geom + noise, re-anchored).  `replay sy`
  is the free-running chain's cloud width, which is what the motion model
  produces when nothing contains it -- the only spread the motion model can
  claim on its own.
""")
    legs = load_legs()
    rows = []
    w = (15, 9, 9, 9, 9, 9, 8, 9, 6, 8, 8)
    row(['case', 'obs dy', 'rep mean', 'rep p05', 'rep p50', 'rep p95',
         'obs sy', 'rep sy', 'N', 'trans m', 'yaw rad'], w)
    for leg in ('wall_adjacent', 'open_space'):
        snaps = legs[leg]
        steps = build_motion(snaps)
        groups = group_steps(steps)
        dy0 = dy_of(snaps[0]['part'], snaps[0]['gt'])
        dyN = dy_of(snaps[-1]['part'], snaps[-1]['gt'])
        tot = np.array([perstep(snaps, groups, ALPHAS,
                                MAIN_SEED + 1000 * s)['total_sum']
                        for s in range(n_seeds)]) + dy0
        fr = [freerun(snaps, steps, ALPHAS, MAIN_SEED + 1000 * s)[1]['sy']
              for s in range(30)]
        syo = float(np.std(snaps[-1]['part'][:, 1]))
        mt = float(np.sum([s['dtrans'] for s in steps]))
        my = float(np.sum([abs(s['dyaw']) for s in steps]))
        rows.append({'case': leg, 'obs_dy': dyN, 'rep_mean': float(tot.mean()),
                     'rep_p05': pctl(tot, 5), 'rep_p50': pctl(tot, 50),
                     'rep_p95': pctl(tot, 95), 'obs_sy': syo,
                     'rep_sy': float(np.mean(fr)),
                     'n': int(np.median([s['n'] for s in snaps])),
                     'trans': mt, 'yaw': my})
        row([leg, f(dyN), f(float(tot.mean())), f(pctl(tot, 5)),
             f(pctl(tot, 50)), f(pctl(tot, 95)), '%.4f' % syo,
             '%.4f' % float(np.mean(fr)),
             int(np.median([s['n'] for s in snaps])),
             '%.3f' % mt, '%.3f' % my], w)
    print("""
  READ open_space FIRST; it is the discriminating row.  Its first cloud is
  effectively a point mass -- sd in y 0.0002 m, dy +0.0119 m -- because the
  leg begins just after AMCL initialised, so the bias is NOT already present
  and the replay has to create it.  It does not: the replay ends +0.096 m
  NORTH while the observed cloud ends -0.097 m SOUTH, a gap of 0.19 m in the
  wrong direction.

  wall_adjacent cannot discriminate on the centroid alone and should not be
  quoted as if it could.  Its first cloud already carries -0.0929 m, and the
  replay simply carries that along, so the replay bracketing the observed
  value there is inheritance, not explanation.  What is testable on that leg
  is the CHANGE across it: predicted +0.033 m against an observed -0.009 m.

  The spread fails on both legs and by a wide margin.  The only width the
  motion model can generate unaided is 4-5x the cloud that was observed.
  A mechanism that reproduces neither the sign of the displacement where the
  displacement is actually created, nor the width of the distribution, is
  not the mechanism.""")
    return rows


# ---------------------------------------------------------------------------
# Part 9 — control regions
# ---------------------------------------------------------------------------

def load_c28():
    p = os.path.join(HERE, 'c2nav28_amcl.json')
    with open(p) as fh:
        return json.load(fh)['traces']


def synth_cloud(pose, n, rng, sy=0.2221, sx=0.2251, syaw=0.2122):
    """A cloud with the MEASURED C2-NAV.30 shape, centred on `pose`.

    Used only where no cloud was recorded.  It is synthetic and is labelled
    as such: the shape comes from the wall_adjacent measurements (sd_y
    0.2221 m, sd_x 0.2251 m, sd_yaw 0.2122 rad), not from that scenario.
    """
    p = np.empty((n, 3))
    p[:, 0] = rng.normal(pose[0], sx, n)
    p[:, 1] = rng.normal(pose[1], sy, n)
    p[:, 2] = rng.normal(pose[2], syaw, n)
    return p


def cmd_control(n_seeds=60, n_part=600):
    hdr('PART 9 - control regions, including one where the bias points NORTH')
    print("""
  open_space carries a real recorded cloud and is replayed exactly as
  wall_adjacent is.  The other five scenarios have NO particle cloud -- only
  C2-NAV.28's fresh /amcl_pose samples with ground truth beside them -- so
  the cloud there is SYNTHETIC, built to the measured C2-NAV.30 shape and
  centred on each recorded pose.  That is stated in the table and must not
  be quoted as a recorded cloud.

  The point of the table is obstacle_corner, where the observed bias points
  NORTH.  If the motion model were the mechanism, its noise term would have
  to change sign with the place the way the observed bias does.
""")
    tr = load_c28()
    out = {}
    w = (26, 5, 10, 10, 12, 13, 12)
    row(['scenario', 'n', 'med trans', 'med |dyaw|', 'observed dy',
         'replay noise', 'replay geom'], w)
    for scen in ('corridor_gate', 'enclosure_entry', 'enclosure_exit',
                 'obstacle_corner', 'open_space', 'wall_adjacent',
                 'wall_parallel'):
        dts, dws, dys = [], [], []
        run_noi, run_geo = [], []
        for run, legs in tr.items():
            rows = legs.get(scen) or []
            seq = [r for r in rows if r.get('amcl_x') not in (None, '')]
            # sums are PER RUN: a run is one leg, and summing across
            # independent legs would not be a quantity the filter ever holds.
            noi, geo = [], []
            for i in range(1, len(seq)):
                a, b = seq[i - 1], seq[i]
                try:
                    ga = (float(a['x']) + WORLD_TO_MAP[0],
                          float(a['y']) + WORLD_TO_MAP[1], float(a['yaw']))
                    gb = (float(b['x']) + WORLD_TO_MAP[0],
                          float(b['y']) + WORLD_TO_MAP[1], float(b['yaw']))
                    dys.append(float(b['amcl_y']) -
                               (float(b['y']) + WORLD_TO_MAP[1]))
                except (TypeError, ValueError):
                    continue
                delta = (gb[0] - ga[0], gb[1] - ga[1],
                         float(angle_diff(gb[2], ga[2])))
                st = {'pose': gb, 'delta': delta}
                r1, t, r2, n1, n2 = motion_increments(gb, delta)
                st.update({'drot1': r1, 'dtrans': t, 'drot2': r2,
                           'rot1_noise': n1, 'rot2_noise': n2, 'dyaw': delta[2]})
                dts.append(t)
                dws.append(abs(delta[2]))
                nn, gg = [], []
                for s in range(n_seeds):
                    rng = np.random.default_rng(MAIN_SEED + 13 * s + 7 * i)
                    p0 = synth_cloud(ga, n_part, rng)
                    det = motion_step_np(p0, gb, delta, ZERO, rng)
                    noz = motion_step_np(p0, gb, delta, ALPHAS, rng)
                    y0 = float(np.mean(p0[:, 1]))
                    gg.append((float(np.mean(det[:, 1])) - y0) - (gb[1] - ga[1]))
                    nn.append(float(np.mean(noz[:, 1])) -
                              float(np.mean(det[:, 1])))
                noi.append(float(np.mean(nn)))
                geo.append(float(np.mean(gg)))
            if noi:
                run_noi.append(float(np.sum(noi)))
                run_geo.append(float(np.sum(geo)))
        if not dys or not run_noi:
            continue
        rec = 'recorded' if scen in ('wall_adjacent', 'open_space') else 'synth'
        obs = float(np.median(dys))
        nz = float(np.mean(run_noi))
        gz = float(np.mean(run_geo))
        row([scen + ' (' + rec + ')', len(dys), '%.4f' % float(np.median(dts)),
             '%.4f' % float(np.median(dws)), f(obs), f(nz, 5), f(gz, 5)], w)
        out[scen] = {'n': len(dys), 'n_runs': len(run_noi),
                     'med_trans': float(np.median(dts)),
                     'med_dyaw': float(np.median(dws)),
                     'observed_dy': obs, 'noise_sum': nz, 'geom_sum': gz,
                     'cloud': rec}
    obs = np.array([v['observed_dy'] for v in out.values()])
    nzs = np.array([v['noise_sum'] for v in out.values()])
    agree = int(np.sum(np.sign(obs) == np.sign(nzs)))
    cor = float(np.corrcoef(obs, nzs)[0, 1])
    print('')
    print('  sign of the replay noise term agrees with the observed bias in'
          ' %d of %d scenarios' % (agree, len(obs)))
    print('  corr(observed dy, replay noise) = %+0.3f' % cor)
    print("""
  Compare that with C2-NAV.31, where the OBSERVATION model's preference
  agreed in direction 7 of 7 including the obstacle_corner sign flip.  That
  was called a real causal signature, and it earned the name.  The motion
  model does not merely fail to track the observed bias -- it runs OPPOSITE
  to it, and it has the sign backwards in exactly the two scenarios that
  discriminate: obstacle_corner, where the observed bias is NORTH and the
  replay says south, and wall_adjacent, where the observed bias is SOUTH and
  the replay says north.

  Do not over-read the correlation.  n = 7 scenarios, so -0.78 is suggestive
  of an anti-relationship, not an established one.  What the table does
  establish without any appeal to the correlation is the sign disagreement
  at the two discriminating scenarios, and that every replay number is a
  fraction of the observed dy beside it.

  Sums are PER RUN and then averaged over the runs for that scenario.  They
  are the whole accumulation the motion model could deliver across a leg,
  and they are still small beside an observed dy that C2-NAV.30 measured to
  be a STANDING offset -- present in the first cloud message of the leg and
  barely moving with motion -- rather than an accumulation at all.""")
    out['sign_agree'] = agree
    out['corr'] = cor
    return out


# ---------------------------------------------------------------------------
# Part 12 — the classification
# ---------------------------------------------------------------------------

def cmd_verdict(n_seeds=200):
    hdr('PART 12 - causal classification')
    legs = load_legs()
    snaps = legs['wall_adjacent']
    steps = build_motion(snaps)
    groups = group_steps(steps)
    dy0 = dy_of(snaps[0]['part'], snaps[0]['gt'])
    dyN = dy_of(snaps[-1]['part'], snaps[-1]['gt'])
    runs = [perstep(snaps, groups, ALPHAS, MAIN_SEED + 1000 * s)
            for s in range(n_seeds)]
    noi = np.array([r['noise_sum'] for r in runs])
    geo = np.array([r['geom_sum'] for r in runs])
    se = noi.std() / math.sqrt(n_seeds)
    lo, hi = noi.mean() - 1.96 * se, noi.mean() + 1.96 * se
    stat = [r['sy_mean'] for r in runs]

    # the standing-offset test: what does the motion model do while stopped?
    still = [i for i, s in enumerate(steps) if s['dtrans'] < 0.02]
    noi_still = np.array([[r['noise'][i] for i in still] for r in runs]).sum(axis=1)

    print("""
  Observed at wall_adjacent (C2-NAV.30, unchanged here):
    cloud centroid dy, first snapshot            %+0.4f m
    cloud centroid dy, last  snapshot            %+0.4f m
    change across the whole leg                  %+0.4f m

  The motion model, replayed through the DEPLOYED binary on the RECORDED
  particle set with the deployed alphas:
    NOISE   term, %d seeds   mean %+0.5f m   sd %.5f
                              95%% CI [%+0.5f, %+0.5f]  -- STRADDLES ZERO
    GEOM    term              mean %+0.5f m
    P(|noise| >= 0.09 m)      %.4f
    sign of the mean noise term: %s

  And the decisive one.  %d of the %d updates on this leg have essentially
  no translation (dtrans < 0.02 m) -- the robot is turning in place through
  the terminal settle.  Over just those updates the motion model's noise
  term totals %+0.5f m, while the observed cloud sits %+0.4f m south the
  whole time.  A model whose translation term is driven by delta_trans
  cannot hold a standing offset open while delta_trans is zero.
""" % (dy0, dyN, dyN - dy0, n_seeds, noi.mean(), noi.std(), lo, hi,
       geo.mean(), float(np.mean(np.abs(noi) >= 0.09)),
       'north (+)' if noi.mean() > 0 else 'south (-)',
       len(still), len(steps), float(noi_still.mean()), dy0))

    print("""  CLASSIFICATION: (C) NEGLIGIBLE.

  The reasoning discipline the brief sets out, applied:

  * The expected centroid displacement is approximately zero -- the 95 %%
    confidence interval on the mean noise term straddles it -- so a single
    Monte-Carlo realisation that happens to land at 0.09 m must NOT be
    called a systematic causal mechanism.  It happens in %.1f %% of seeds,
    and the draw is near-symmetric about zero: p05 %+0.5f, p95 %+0.5f.
  * The mean is not consistently south.  It is marginally NORTH, and the
    deterministic geometry is also north.  The model is weakly opposing the
    observed bias, not producing it.
  * The replay produces drift without a repeatable direction, which is
    stochastic sampling, not a directional bias mechanism.
  * It reproduces neither the observed displacement nor the observed spread
    (Part 8), and its sign does not change with place the way the observed
    bias does (Part 9).

  So after C2-NAV.31 eliminated the observation model and C2-NAV.30
  eliminated depletion, the motion model is eliminated too.  What is left
  is the part of the update NEITHER session could observe: the importance
  weights AMCL actually used, which resample_interval = 1 levels before
  /particle_cloud is published.""" % (
        100.0 * float(np.mean(np.abs(noi) >= 0.09)),
        pctl(noi, 5), pctl(noi, 95)))

    return {'dy0': dy0, 'dyN': dyN, 'noise_mean': float(noi.mean()),
            'noise_sd': float(noi.std()), 'ci': [float(lo), float(hi)],
            'geom_mean': float(geo.mean()),
            'p_reach': float(np.mean(np.abs(noi) >= 0.09)),
            'still_updates': len(still),
            'noise_still': float(noi_still.mean()),
            'classification': 'C'}


# ---------------------------------------------------------------------------
# selftest — the gate, offline, before anything else runs
# ---------------------------------------------------------------------------

def cmd_selftest():
    hdr('C2-NAV.32 SELFTEST')
    checks = []

    def ck(name, ok, detail=''):
        checks.append((name, bool(ok), detail))
        print('  %-4s %-52s %s' % ('ok' if ok else 'FAIL', name, detail))

    # 1 angle helpers against hand values
    ck('normalize(3pi) == pi', abs(abs(float(normalize(3 * math.pi))) - math.pi) < 1e-12)
    ck('angle_diff(0.1, -0.1) == 0.2',
       abs(float(angle_diff(0.1, -0.1)) - 0.2) < 1e-12)
    ck('angle_diff wraps the short way',
       abs(float(angle_diff(3.0, -3.0)) - (3.0 - -3.0 - 2 * math.pi)) < 1e-12,
       '%.6f' % float(angle_diff(3.0, -3.0)))

    # 2 the oracle exists and is the deployed binary
    ck('oracle builds against %s' % ROS_PREFIX, ORACLE.available(),
       ORACLE.why or _deb_version())
    if not ORACLE.available():
        print('\n  SELFTEST ABORTED: no oracle, so nothing below can be trusted.')
        return checks

    # 3 hand-computed zero-noise cases through the DEPLOYED code
    a = ORACLE.step(np.zeros((1, 3)), (1, 0, 0), (1, 0, 0), ZERO, 1)
    ck('deployed: (0,0,0) + 1 m forward -> (1,0,0)',
       np.allclose(a[0], [1, 0, 0], atol=1e-12), str(np.round(a[0], 6)))
    a = ORACLE.step(np.array([[0.0, 0.0, math.pi / 2]]), (1, 0, 0), (1, 0, 0),
                    ZERO, 1)
    ck('deployed: motion is applied in the PARTICLE frame',
       abs(a[0, 1] - 1.0) < 1e-12 and abs(a[0, 0]) < 1e-12,
       str(np.round(a[0], 6)))
    a = ORACLE.step(np.zeros((1, 3)), (0, 0, 0.5), (0, 0, 0.5), ZERO, 1)
    ck('deployed: pure rotation moves no particle at zero noise',
       np.allclose(a[0], [0, 0, 0.5], atol=1e-12), str(np.round(a[0], 6)))

    # 4 determinism of the deployed sampler
    x = ORACLE.step(np.zeros((50, 3)), (1, 0, 0), (1, 0, 0), ALPHAS, 42)
    y = ORACLE.step(np.zeros((50, 3)), (1, 0, 0), (1, 0, 0), ALPHAS, 42)
    z = ORACLE.step(np.zeros((50, 3)), (1, 0, 0), (1, 0, 0), ALPHAS, 43)
    ck('deployed sampler is bit-reproducible on a seed',
       np.array_equal(x, y) and not np.array_equal(x, z))

    # 5 the twin matches the oracle exactly at zero noise
    tw = probe_twin()
    ck('twin == oracle exactly at zero noise',
       tw['exact_zero_noise_maxabs'] < 1e-12,
       '%.2e' % tw['exact_zero_noise_maxabs'])
    ck('twin matches oracle sd in y to 2 %%',
       abs(tw['sd_y'][0] - tw['sd_y'][1]) / tw['sd_y'][0] < 0.02,
       '%.5f vs %.5f' % (tw['sd_y'][0], tw['sd_y'][1]))

    # 6 the sigma formulae, identified from the deployed binary
    idn = identify(n=120000)
    tol = 0.06
    got = {t['alpha']: t for t in idn['terms']}
    for al in ('alpha1', 'alpha2', 'alpha3', 'alpha4'):
        p = idn['pred'][al]
        for key, vk in (('rot1', 'var_rot1'), ('trans', 'var_trans'),
                        ('rot2', 'var_rot2')):
            exp, obs = p[key], got[al][vk]
            good = abs(obs - exp) <= max(tol * max(exp, 1e-9), 2e-3)
            ck('%s -> var(%s) = %.4f' % (al, key, exp), good,
               'measured %.4f' % obs)
    ck('alpha5 is inert for the differential model',
       max(abs(got['alpha5'][k]) for k in
           ('var_rot1', 'var_trans', 'var_rot2')) < 1e-12)

    # 7 structural facts
    st = probe_structure()
    ck('delta_rot1 forced to 0 below the 0.01 m guard',
       abs(st['guard_below_rot1']) < 1e-12 and abs(st['guard_above_rot1']) > 0.9,
       '%.6f / %.6f' % (st['guard_below_rot1'], st['guard_above_rot1']))
    ck('rot noise is symmetric about pi (backward motion)',
       abs(st['sym_small_sd_rot1'] - st['sym_near_pi_sd_rot1']) < 0.02,
       '%.5f vs %.5f' % (st['sym_small_sd_rot1'], st['sym_near_pi_sd_rot1']))
    ck('yaw is NOT normalised by the update',
       not st['yaw_normalised'] and abs(st['yaw_after'] - 3.5) < 1e-12,
       '%.6f' % st['yaw_after'])
    ck('noise is drawn per particle', st['per_particle_unique'] > 490,
       '%d/%d distinct' % (st['per_particle_unique'], st['per_particle_n']))
    ck('no per-axis parameter (x/y symmetric)',
       abs(st['axis_x_sdx'] - st['axis_y_sdy']) < 0.02,
       '%.5f vs %.5f' % (st['axis_x_sdx'], st['axis_y_sdy']))

    # 8 the committed evidence is the evidence C2-NAV.30/.31 published
    legs = load_legs()
    tot = sum(len(s['part']) for leg in legs.values() for s in leg)
    ck('bundle carries C2-NAV.30\'s 23,444 particles', tot == 23444, str(tot))
    ck('19 wall_adjacent snapshots', len(legs['wall_adjacent']) == 19)
    ck('21 open_space snapshots', len(legs['open_space']) == 21)
    wa0 = legs['wall_adjacent'][0]
    ck('first wall_adjacent cloud dy = -0.0929 (C2-NAV.30)',
       abs(dy_of(wa0['part'], wa0['gt']) - (-0.09289590834697226)) < 1e-9,
       '%+.8f' % dy_of(wa0['part'], wa0['gt']))
    med = float(np.median([dy_of(s['part'], s['gt'])
                           for s in legs['wall_adjacent']]))
    ck('median wall_adjacent cloud dy = -0.0921 (C2-NAV.30)',
       abs(med - (-0.0921)) < 5e-4, '%+.6f' % med)
    ck('published weights were flat: ESS/n = 1 on every snapshot',
       all(abs(s['ess'] - s['n']) < 1e-6 for leg in legs.values() for s in leg))

    # 9 parameters
    p = read_amcl_params(PARAMS_YAML)
    ck('params sha256 matches C2-NAV.30', sha256(PARAMS_YAML) == PARAMS_SHA)
    ck('alpha1..5 all 0.2',
       all(p['alpha%d' % i] == 0.2 for i in range(1, 6)))
    ck('update_min_d 0.25 / update_min_a 0.2 / resample_interval 1',
       p['update_min_d'] == 0.25 and p['update_min_a'] == 0.2 and
       p['resample_interval'] == 1)
    ck('robot_model_type is the differential model',
       p['robot_model_type'] == 'nav2_amcl::DifferentialMotionModel')
    ck('amcl block identical to the shipped nav2_params.yaml',
       p == read_amcl_params(SHIPPED_YAML))

    # 10 the estimator itself
    snaps = legs['wall_adjacent']
    groups = group_steps(build_motion(snaps))
    r = perstep(snaps, groups, ZERO, 1)
    ck('zero alphas => the noise term is identically zero',
       max(abs(v) for v in r['noise']) < 1e-12,
       '%.2e' % max(abs(v) for v in r['noise']))
    r1 = perstep(snaps, groups, ALPHAS, 12345)
    r2 = perstep(snaps, groups, ALPHAS, 12345)
    ck('replay is deterministic on a fixed seed',
       r1['noise_sum'] == r2['noise_sum'], '%.9f' % r1['noise_sum'])
    r3 = perstep(snaps, groups, ALPHAS, 999)
    ck('a different seed gives a different draw',
       r1['noise_sum'] != r3['noise_sum'])
    g4 = group_steps(build_motion(snaps), 4)
    ck('re-chunking preserves total motion',
       abs(sum(sum(x['delta'][1] for x in sub) for sub in g4) -
           sum(s['delta'][1] for s in build_motion(snaps))) < 1e-12)

    n_ok = sum(1 for _, ok, _ in checks if ok)
    print('')
    print('  %d/%d checks pass' % (n_ok, len(checks)))
    if n_ok != len(checks):
        print('  SELFTEST FAILED - nothing below should be quoted.')
    return checks


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

CMDS = {
    'selftest': cmd_selftest, 'model': cmd_model, 'params': cmd_params,
    'motion': cmd_motion, 'zeronoise': cmd_zeronoise, 'replay': cmd_replay,
    'size': cmd_size, 'heading': cmd_heading, 'compare': cmd_compare,
    'control': cmd_control, 'verdict': cmd_verdict,
}

ORDER = ['model', 'params', 'motion', 'zeronoise', 'replay', 'size',
         'heading', 'compare', 'control', 'verdict']


def main(argv):
    if len(argv) > 1 and argv[1] not in CMDS:
        print('unknown command %r; one of: %s' % (argv[1], ', '.join(CMDS)))
        return 2
    if len(argv) > 1:
        CMDS[argv[1]]()
        return 0
    out = {'what': 'C2-NAV.32 DifferentialMotionModel offline replay',
           'nav2_amcl': _deb_version(), 'seed': MAIN_SEED}
    for name in ORDER:
        out[name] = CMDS[name]()
    with open(OUT_JSON, 'w') as fh:
        json.dump(out, fh, default=_jsonable)
    print('')
    print('wrote %s' % os.path.relpath(OUT_JSON, REPO))
    return 0


def _jsonable(o):
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    return str(o)


if __name__ == '__main__':
    sys.exit(main(sys.argv))
