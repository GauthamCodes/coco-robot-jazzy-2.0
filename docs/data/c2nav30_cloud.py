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

"""
c2nav30_cloud.py
================
C2-NAV.30. The instrument and the gate for ONE question:

    Does the AMCL particle cloud itself COLLAPSE toward the repeatable
    wall-adjacent bias, or does it still contain support around the true
    ground-truth pose?

WHY THIS HAS TO BE LIVE, AND WHAT CAME BEFORE
---------------------------------------------
C2-NAV.28 established that the ~0.12 m southward AMCL error at
`wall_adjacent` is a PLACE, not an event: it is present before the
terminal yaw in 8/8 legs that diverge materially, it SHRANK (0.182 ->
0.076 m) through the worst terminal-yaw case in the session, and it
points a different way at `obstacle_corner`, so it is not a global
odometry bias.

C2-NAV.29 then ruled out the two explanations that did not need the
filter. The recorded scan agrees with the true world at ground truth
(median +0.0001 m over 544 samples), and the reconstructed scan scores
higher on the map's own likelihood field at ground truth than at the
AMCL pose in 90 of 90 wall-adjacent samples. A perfect ground-truth scan
optimises against the shipped map at dy = -0.020 m, against an observed
AMCL dy = -0.129 m: map geometry owns about a SIXTH of the bias and
sensor extrinsics own none of it.

What neither could reach is WHICH part of the estimator. Both worked
from the filter's OUTPUT. This module works from its INTERNAL STATE --
`/particle_cloud`, the hypothesis set itself -- which exists only while
the filter is running. Hence one live run.

WHAT /particle_cloud ACTUALLY IS
--------------------------------
`nav2_msgs/ParticleCloud`, READ FROM THE INSTALLED INTERFACE rather than
assumed:

    std_msgs/Header header
    Particle[] particles          # geometry_msgs/Pose pose
                                  # float64 weight

So the weights ARE a field of the message. They are not reconstructed
here and nothing below invents them. Whether they are INFORMATIVE is a
separate question the instrument measures rather than assumes -- see
`pc_ess` in `nav_bench.cloud_summary` and the DATA QUALITY section of
`avail`. With `resample_interval: 1` AMCL may publish a set whose
weights were flattened by the resampling step that just ran, in which
case the effective sample size equals the particle count exactly and the
weights say nothing about the weighting step. That is a limitation to
REPORT, not to assume in either direction.

The publisher is `rclcpp::SensorDataQoS` -- BEST_EFFORT. A RELIABLE
subscriber matches nothing, receives nothing and raises nothing. That is
the silent-blindness class CLAUDE.md lists and that
`test_rviz_configs.py` already asserts for this exact topic, and
`nav_bench.main` prints a SEEN/NOT SEEN line before any leg runs so an
empty cloud column can never be reported as a measurement.

WHAT IS RECORDED, AND WHERE
---------------------------
Two representations, deliberately:

* `pc_*` columns in the 10 Hz per-leg trace -- `cloud_summary()` per
  message, sampled with the SAME half-open bucket rule and the SAME
  no-forward-fill rule C2-NAV.28 established for the AMCL columns. Full
  time resolution, fixed set of statistics.
* `<leg>_rep<n>_cloud.npz` beside the trace -- the FULL particle sets
  for the leg window, `(x, y, yaw, weight)` per particle, with a
  `counts` array because KLD sampling varies the particle count between
  messages. Full distribution, so a shape question nobody thought of
  before the run can still be answered after it.

The summary is never presented as equivalent to the cloud. Everything in
`support` and `modes` reads the particles.

FRAME CONVENTION -- NOT CHANGED, AND REPORTED BOTH WAYS
-------------------------------------------------------
`nav_bench.py` hard-codes `WORLD_TO_MAP = (2.0, 0.0)`; `map_audit.py`
measures (2.056, 0.015). C2-NAV.29 found that disagreement and left it
in place because it is a behavioural constant behind twenty-eight
commits of results. C2-NAV.30 does not change it either -- the brief is
explicit -- and, like C2-NAV.29, computes every headline under BOTH
conventions so no conclusion rests on the choice.

    python3 docs/data/c2nav30_cloud.py selftest   # the Part A gate
    python3 docs/data/c2nav30_cloud.py avail      # data quality first
    python3 docs/data/c2nav30_cloud.py table      # the central table
    python3 docs/data/c2nav30_cloud.py support    # the geometric test
    python3 docs/data/c2nav30_cloud.py modes      # multimodality
    python3 docs/data/c2nav30_cloud.py regions    # wall vs control
    python3 docs/data/c2nav30_cloud.py temporal   # when the bias appears
    python3 docs/data/c2nav30_cloud.py verdict    # the classification
    python3 docs/data/c2nav30_cloud.py dump       # freeze the bundle
"""
import argparse
import bisect
import csv
import json
import math
import os
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
WT = os.path.dirname(os.path.dirname(HERE))          # docs/data -> root
NAV_BENCH = os.path.join(WT, 'gazebo_models', 'scripts', 'nav_bench.py')
SCRATCH = os.environ.get('C2NAV_SCRATCH', os.path.join(WT, '.navbench'))
RESULTS = os.path.join(SCRATCH, 'results')

# The historical convention. NOT changed here -- see the module
# docstring. `MEASURED_*` is what map_audit.py measured, carried so every
# headline can be printed both ways.
WORLD_TO_MAP_X = 2.0
WORLD_TO_MAP_Y = 0.0
MEASURED_TO_MAP_X = 2.0560
MEASURED_TO_MAP_Y = 0.0150

# The 28-column pre-C2-NAV.28 schema, read out of the frozen bundle
# rather than retyped -- same source C2-NAV.28's own gate used.
FROZEN_BUNDLE = os.path.join(HERE, 'c2nav22_yaw.json')
# The three C2-NAV.28 columns, imported from that session's own module
# rather than retyped here, so a change there cannot silently pass.
AMCL_MODULE = os.path.join(HERE, 'c2nav28_amcl.py')

# The commit this session STARTED from -- C2-NAV.29. Compared against
# this and not HEAD: once C2-NAV.30 is committed a HEAD-relative diff is
# empty and the check would pass for the wrong reason.
BASE = '91e5f3a'

# C2-NAV.30's one live run. `open_space` is TOUR's own designated
# control case ("goal 1.15 m from anything: the control case") and
# `wall_adjacent` the treatment ("goal 0.35 m from the south wall"),
# both inside ONE simulator instance -- which is what makes the
# comparison control for the instance rather than straddle two of them.
RUN = 'c2n30_focus_r1'
WALL_LEG = 'wall_adjacent'
CONTROL_LEG = 'open_space'

BUNDLE = os.path.join(HERE, 'c2nav30_cloud.json')
_bundle = None


# ------------------------------------------------------------- utils
def hdr(t):
    print()
    print('=' * 78)
    print(t)
    print('=' * 78)


def fl(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def _git(*args):
    return subprocess.run(('git', '-C', WT) + args,
                          capture_output=True, text=True)


def _load_nav_bench():
    """Import the REAL nav_bench.py under test, by path."""
    import importlib.util
    spec = importlib.util.spec_from_file_location('nb_under_test', NAV_BENCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _load_amcl_module():
    import importlib.util
    spec = importlib.util.spec_from_file_location('c2nav28', AMCL_MODULE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _frozen_schema():
    with open(FROZEN_BUNDLE) as f:
        b = json.load(f)
    got = {tuple(v['schema']) for v in b['traces'].values()}
    return list(max(got, key=len))


def med(xs):
    xs = sorted(x for x in xs if x is not None)
    if not xs:
        return None
    n = len(xs)
    return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


def pct(xs, q):
    xs = sorted(x for x in xs if x is not None)
    if not xs:
        return None
    return float(np.percentile(np.asarray(xs, dtype=float), q))


# --------------------------------------------------------------- I/O
def bundle():
    """The frozen C2-NAV.30 evidence, or None.

    `.navbench/` is scratch and is not committed, so every number here
    would become unreproducible the moment it is cleared. `dump` freezes
    what the analyses actually read into a committed JSON bundle and
    every reader below falls back to it. Live scratch always wins when
    present, so a re-run is never masked by the bundle.
    """
    global _bundle
    if _bundle is None:
        if os.path.exists(BUNDLE):
            with open(BUNDLE) as f:
                _bundle = json.load(f)
        else:
            _bundle = {}
    return _bundle


def trace_path(tag, leg, rep=0):
    return os.path.join(RESULTS, f'{tag}_traces', f'{leg}_rep{rep}.csv')


def cloud_path(tag, leg, rep=0):
    return os.path.join(RESULTS, f'{tag}_traces', f'{leg}_rep{rep}_cloud.npz')


def read_trace(tag, leg, rep=0):
    """Rows of the 10 Hz trace, live scratch first then the bundle."""
    p = trace_path(tag, leg, rep)
    if os.path.exists(p):
        with open(p) as f:
            return list(csv.DictReader(f))
    b = bundle().get('traces', {}).get(f'{tag}/{leg}')
    if not b:
        return []
    return [dict(zip(b['schema'], r)) for r in b['rows']]


def read_clouds(tag, leg, rep=0):
    """[(t_sim, (n,4) array)] for the leg, or [] if not available.

    Live scratch only: the full particle sets are far too large for a
    committed bundle, so `dump` freezes DERIVED per-snapshot statistics
    instead (see `dump`) and the commands that need raw particles say so
    when they cannot run.
    """
    p = cloud_path(tag, leg, rep)
    if not os.path.exists(p):
        return []
    z = np.load(p)
    parts, counts, ts = z['particles'], z['counts'], z['ts_sim_s']
    out, i = [], 0
    for k, c in enumerate(counts):
        out.append((float(ts[k]), parts[i:i + int(c)]))
        i += int(c)
    return out


def cloud_meta(tag, leg, rep=0):
    p = cloud_path(tag, leg, rep).replace('.npz', '_meta.json')
    if os.path.exists(p):
        with open(p) as f:
            return json.load(f)
    return bundle().get('cloud_meta', {}).get(f'{tag}/{leg}')


def legs_of(tag):
    p = os.path.join(RESULTS, f'{tag}.json')
    if os.path.exists(p):
        with open(p) as f:
            return {leg['scenario']: leg for leg in json.load(f)['legs']}
    return bundle().get('legs', {}).get(tag, {})


# --------------------------------------------------- derived samples
def samples(tag, leg, rep=0, measured_frame=False):
    """Rows carrying BOTH a fresh /particle_cloud and a ground truth.

    A row is used only when its `pc_n` cell is non-empty -- i.e. a cloud
    message genuinely landed in that row's 0.1 s bucket. Nothing is
    forward-filled: a held cloud is indistinguishable from a live one
    once printed, and the whole point of the exercise is to see what the
    filter was holding WHEN it was holding it.

    `amcl_*` on the same row may be blank, because /amcl_pose and
    /particle_cloud are separate publications; the AMCL fields are then
    None and the cloud fields are still valid.
    """
    ox = MEASURED_TO_MAP_X if measured_frame else WORLD_TO_MAP_X
    oy = MEASURED_TO_MAP_Y if measured_frame else WORLD_TO_MAP_Y
    out = []
    for r in read_trace(tag, leg, rep):
        if not r.get('pc_n'):
            continue
        gx, gy = fl(r.get('x')), fl(r.get('y'))
        if gx is None or gy is None:
            continue
        d = {'t': fl(r['t_rel']),
             'gt_x_map': gx + ox, 'gt_y_map': gy + oy,
             'gt_yaw': fl(r.get('yaw')),
             'v_act': fl(r.get('v_act')), 'w_act': fl(r.get('w_act')),
             'v_nav': fl(r.get('v_nav')), 'w_nav': fl(r.get('w_nav')),
             'scan_min': fl(r.get('scan_min')),
             'cm_polygon': r.get('cm_polygon') or '',
             'amcl_x': fl(r.get('amcl_x')), 'amcl_y': fl(r.get('amcl_y')),
             'amcl_yaw': fl(r.get('amcl_yaw'))}
        for k in ('pc_n', 'pc_mx', 'pc_my', 'pc_myaw', 'pc_wmx', 'pc_wmy',
                  'pc_sx', 'pc_sy', 'pc_cxy', 'pc_a1', 'pc_a2', 'pc_ang',
                  'pc_xlo', 'pc_x05', 'pc_x50', 'pc_x95', 'pc_xhi',
                  'pc_ylo', 'pc_y05', 'pc_y50', 'pc_y95', 'pc_yhi',
                  'pc_wsum', 'pc_wmax', 'pc_ess', 'pc_nuniq', 'pc_pgap'):
            d[k] = fl(r.get(k))
        # Signed lateral error toward the SOUTH wall. Negative = south,
        # which is the direction C2-NAV.28 measured the bias in.
        if d['amcl_y'] is not None:
            d['amcl_dy'] = d['amcl_y'] - d['gt_y_map']
            d['amcl_dx'] = d['amcl_x'] - d['gt_x_map']
            d['amcl_err'] = math.hypot(d['amcl_dx'], d['amcl_dy'])
        else:
            d['amcl_dy'] = d['amcl_dx'] = d['amcl_err'] = None
        d['cloud_dy'] = d['pc_my'] - d['gt_y_map']
        d['cloud_dx'] = d['pc_mx'] - d['gt_x_map']
        d['cloud_err'] = math.hypot(d['cloud_dx'], d['cloud_dy'])
        if d['pc_wmy'] is not None:
            d['wcloud_dy'] = d['pc_wmy'] - d['gt_y_map']
            d['wcloud_err'] = math.hypot(d['pc_wmx'] - d['gt_x_map'],
                                         d['wcloud_dy'])
        else:
            d['wcloud_dy'] = d['wcloud_err'] = None
        # The one geometric question: does the POPULATION extend across
        # the true pose in y?
        d['gt_inside_y'] = (d['pc_ylo'] <= d['gt_y_map'] <= d['pc_yhi'])
        d['gt_inside_y90'] = (d['pc_y05'] <= d['gt_y_map'] <= d['pc_y95'])
        if d['amcl_y'] is not None:
            d['amcl_inside_y'] = (d['pc_ylo'] <= d['amcl_y'] <= d['pc_yhi'])
        else:
            d['amcl_inside_y'] = None
        out.append(d)
    return out


# ------------------------------------------------------ 1. selftest
def _fake_trace(amcl_samples, cloud_samples, t0=100.0, t1=100.5,
                gt_hz=20.0):
    """Run the REAL write_trace against a synthetic node. Returns rows.

    `cloud_samples` is [(t_sim, summary_dict)] -- what `self.cloud`
    holds, i.e. already through `cloud_summary`.
    """
    mod = _load_nav_bench()

    class S:
        def __init__(self, pairs):
            self.t = [p[0] for p in pairs]
            self.v = [p[1] for p in pairs]

        def window(self, a, b):
            i = bisect.bisect_left(self.t, a)
            j = bisect.bisect_right(self.t, b)
            return self.t[i:j], self.v[i:j]

    class FakeLock:
        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

    n = int((t1 - t0) * gt_hz) + 1
    gt_rows = [(t0 + i / gt_hz, (0.1 * i, 0.2 * i, 0.3, 0.4, 0.5))
               for i in range(n)]

    class FakeNode:
        _lock = FakeLock()
        gt = S(gt_rows)
        amcl = S(amcl_samples)
        cloud = S(cloud_samples)
        nav = S([])
        smooth = S([])
        out = S([])
        wheel = S([])
        cmstate = S([])
        scanmin = S([])
        evals = S([])

    fd, path = tempfile.mkstemp(suffix='.csv')
    os.close(fd)
    try:
        mod.write_trace(FakeNode(), path, t0, t1)
        with open(path) as f:
            return list(csv.DictReader(f))
    finally:
        os.unlink(path)


def _check(ok, label, detail=''):
    print('  %-4s %s%s' % ('OK' if ok else 'FAIL', label,
                           ('  -- ' + detail) if detail else ''))
    return 0 if ok else 1


def cmd_selftest(_args):
    """The Part A gate. Offline. No simulator, no ROS graph, no run."""
    hdr('C2-NAV.30 selftest -- the instrumentation, before any run')
    bad = 0
    try:
        nb = _load_nav_bench()
    except Exception as e:                        # noqa: BLE001
        print('  FAIL could not import nav_bench.py: %r' % (e,))
        print('  (it imports rclpy at module scope; this needs a sourced')
        print('   ROS 2 environment.)')
        return 1
    cs, PC = nb.cloud_summary, list(nb.PC_FIELDS)

    # ---- check 1: schema ------------------------------------------
    print('1. trace schema: 28 frozen + 3 C2-NAV.28 + the pc_ group')
    frozen = _frozen_schema()
    amcl_cols = _load_amcl_module().AMCL_COLS
    rows = _fake_trace([(100.05, (1.0, 2.0, 0.5))], [])
    got = list(rows[0].keys())
    pre = frozen + list(amcl_cols)
    bad += _check(got[:len(frozen)] == frozen,
                  'the %d C2-NAV.0..27 columns are unchanged, in order'
                  % len(frozen),
                  'first difference at index %s' % next(
                      (i for i, (a, b) in enumerate(zip(got, frozen))
                       if a != b), 'none'))
    bad += _check(got[len(frozen):len(pre)] == list(amcl_cols),
                  'the 3 C2-NAV.28 columns still follow them, in order',
                  'got %r' % (got[len(frozen):len(pre)],))
    bad += _check(got[len(pre):] == PC,
                  'exactly the %d pc_ columns appended, and nothing else'
                  % len(PC),
                  'got %r' % (got[len(pre):],))
    bad += _check(len(got) == len(pre) + len(PC),
                  'column count %d -> %d' % (len(pre), len(got)))

    # ---- check 2: C2-NAV.28's AMCL rule is UNCHANGED ---------------
    # Re-asserted here at the NEW schema. C2-NAV.28's own gate cannot do
    # it any more: its check 1 asserts "exactly three appended", which
    # C2-NAV.30 makes false by construction, and its check 4 diffs
    # against 88ecb05 with a C2-NAV.28-only allow-list that C2-NAV.29
    # already broke. The BEHAVIOUR those checks protected is protected
    # here instead, and is not weakened.
    print()
    print('2. the C2-NAV.28 AMCL sampling rule still holds exactly')
    rows = _fake_trace([(100.05, (1.0, 2.0, 0.5)),
                        (100.25, (3.0, 4.0, 0.6))], [])
    seq = [(r['t_rel'], r['amcl_x'], r['amcl_y'], r['amcl_yaw'])
           for r in rows]
    want = [('0.0', '', '', ''), ('0.1', '1.0', '2.0', '0.5'),
            ('0.2', '', '', ''), ('0.3', '3.0', '4.0', '0.6'),
            ('0.4', '', '', ''), ('0.5', '', '', '')]
    bad += _check(seq == want, 'each AMCL sample lands in exactly one row',
                  'got %r' % (seq,))
    rows = _fake_trace([(100.12, (1.0, 1.0, 0.1)),
                        (100.18, (2.0, 2.0, 0.2))], [])
    seq = [(r['t_rel'], r['amcl_x']) for r in rows]
    bad += _check(seq[2] == ('0.2', '2.0') and seq[1] == ('0.1', ''),
                  'a crowded AMCL bucket keeps the LATEST sample')
    rows = _fake_trace([], [])
    bad += _check(all(r['amcl_x'] == '' for r in rows),
                  'a leg with no /amcl_pose at all writes blanks')

    # ---- check 3: the pc_ bucket rule ------------------------------
    print()
    print('3. pc_ sampling: bucket (t-0.1, t], no forward fill')
    a = cs([1.0, 1.0], [2.0, 2.0], [0.0, 0.0], [0.5, 0.5])
    b = cs([9.0, 9.0], [8.0, 8.0], [0.0, 0.0], [0.5, 0.5])
    rows = _fake_trace([], [(100.05, a), (100.25, b)])
    seq = [(r['t_rel'], r['pc_mx'], r['pc_n']) for r in rows]
    want = [('0.0', '', ''), ('0.1', '1.0', '2'), ('0.2', '', ''),
            ('0.3', '9.0', '2'), ('0.4', '', ''), ('0.5', '', '')]
    bad += _check(seq == want, 'each cloud lands in exactly one row',
                  'got %r' % (seq,))
    bad += _check(all(all(r[k] == '' for k in PC) for r in rows
                      if r['t_rel'] in ('0.0', '0.2', '0.4', '0.5')),
                  'an empty bucket is %d BLANKS, not zeros and not the '
                  'previous row' % len(PC))
    rows = _fake_trace([], [(100.12, a), (100.18, b)])
    seq = [(r['t_rel'], r['pc_mx']) for r in rows]
    bad += _check(seq[2] == ('0.2', '9.0') and seq[1] == ('0.1', ''),
                  'a crowded cloud bucket keeps the LATEST message')
    rows = _fake_trace([], [])
    bad += _check(all(all(r[k] == '' for k in PC) for r in rows),
                  'a leg with no /particle_cloud at all writes blanks')
    # tiling, the property that actually matters -- row times accumulate
    # as t += 0.1 so a sample within a float epsilon of a boundary may
    # fall either side of it; what must hold is that every message
    # appears exactly once, in the row at or just after its own stamp.
    msgs = [(100.0 + 0.11 * k,
             cs([float(k)], [0.0], [0.0], [1.0])) for k in range(1, 5)]
    rows = _fake_trace([], msgs, t0=100.0, t1=100.5)
    seen = [(fl(r['t_rel']), fl(r['pc_mx'])) for r in rows if r['pc_mx']]
    bad += _check(sorted(v for _, v in seen) == [1.0, 2.0, 3.0, 4.0],
                  'every cloud message in the window appears exactly once',
                  'got %r' % (sorted(v for _, v in seen),))
    offs = [(100.0 + t) - (100.0 + 0.11 * v) for t, v in seen]
    bad += _check(bool(offs) and all(-0.02 <= o < 0.12 for o in offs),
                  'each lands in the row at or just after its own stamp',
                  'offsets %r' % [round(o, 3) for o in offs])

    # ---- check 4: cloud_summary, against hand-computed answers -----
    print()
    print('4. cloud_summary on synthetic clouds with KNOWN answers')
    bad += _check(cs([], [], [], []) is None,
                  'an EMPTY cloud is None, not a row of zeros')
    # Four particles on a 2x2 square centred at (10, 5), side 2.
    x = [9.0, 11.0, 9.0, 11.0]
    y = [4.0, 4.0, 6.0, 6.0]
    d = cs(x, y, [0.0] * 4, [0.25] * 4)
    bad += _check(d['pc_n'] == 4, 'pc_n counts the particles')
    bad += _check(abs(d['pc_mx'] - 10.0) < 1e-12
                  and abs(d['pc_my'] - 5.0) < 1e-12,
                  'unweighted mean is the centroid (10, 5)',
                  '(%r, %r)' % (d['pc_mx'], d['pc_my']))
    bad += _check(abs(d['pc_sx'] - 1.0) < 1e-12
                  and abs(d['pc_sy'] - 1.0) < 1e-12,
                  'population sd is 1.0 on each axis (ddof=0)',
                  '(%r, %r)' % (d['pc_sx'], d['pc_sy']))
    bad += _check(abs(d['pc_cxy']) < 1e-12, 'a symmetric square has zero xy '
                  'covariance', '%r' % d['pc_cxy'])
    bad += _check(abs(d['pc_ylo'] - 4.0) < 1e-12
                  and abs(d['pc_yhi'] - 6.0) < 1e-12,
                  'pc_ylo/pc_yhi are the true y extrema')
    bad += _check(d['pc_nuniq'] == 4, 'four distinct positions -> nuniq 4')
    bad += _check(abs(d['pc_ess'] - 4.0) < 1e-9,
                  'uniform weights -> ESS == n exactly (the flat-weight '
                  'tell)', '%r' % d['pc_ess'])
    # Duplicated particles: the depletion signature.
    d2 = cs([1.0] * 6 + [2.0] * 4, [0.0] * 10, [0.0] * 10, [0.1] * 10)
    bad += _check(d2['pc_n'] == 10 and d2['pc_nuniq'] == 2,
                  'n=10 over 2 distinct positions -> nuniq 2 (depletion is '
                  'visible in nothing else recorded)')
    # Weighted mean must differ from the unweighted one when it should.
    d3 = cs([0.0, 10.0], [0.0, 0.0], [0.0, 0.0], [0.9, 0.1])
    bad += _check(abs(d3['pc_mx'] - 5.0) < 1e-12
                  and abs(d3['pc_wmx'] - 1.0) < 1e-12,
                  'weighted mean 1.0 vs unweighted 5.0 -- the two are '
                  'recorded SEPARATELY and neither stands in for the other')
    bad += _check(abs(d3['pc_ess'] - (1.0 / 0.82)) < 1e-9,
                  'ESS = (sum w)^2 / sum(w^2) = 1/0.82 for (0.9, 0.1)',
                  '%r' % d3['pc_ess'])
    # Weights that cannot produce a weighted mean.
    d4 = cs([1.0, 2.0], [3.0, 4.0], [0.0, 0.0], [0.0, 0.0])
    bad += _check(d4['pc_wmx'] is None and d4['pc_wmy'] is None
                  and d4['pc_ess'] is None,
                  'all-zero weights -> weighted fields None, NOT the '
                  'unweighted mean silently standing in')
    bad += _check(abs(d4['pc_mx'] - 1.5) < 1e-12,
                  'and the 24 unweighted fields are still valid on that '
                  'same cloud')
    rows = _fake_trace([], [(100.05, d4)])
    r = [r for r in rows if r['t_rel'] == '0.1'][0]
    bad += _check(r['pc_wmx'] == '' and r['pc_wmy'] == '' and r['pc_ess'] == ''
                  and r['pc_mx'] == '1.5' and r['pc_n'] == '2',
                  'and it writes as three blanks amid valid columns, '
                  'per-field',
                  '%r' % {k: r[k] for k in ('pc_n', 'pc_mx', 'pc_wmx')})
    # Principal axes on a deliberately anisotropic, rotated cloud.
    ang = 0.4
    t = np.linspace(-1, 1, 101)
    px = (3.0 * t * math.cos(ang)).tolist()
    py = (3.0 * t * math.sin(ang)).tolist()
    d5 = cs(px, py, [0.0] * 101, [1.0 / 101] * 101)

    def _dmodpi(a, b):
        """Smallest angle between two AXES -- an eigenvector and its
        negation describe the same axis, so the comparison is mod pi."""
        d = (a - b) % math.pi
        return min(d, math.pi - d)

    bad += _check(_dmodpi(d5['pc_ang'], ang) < 1e-6,
                  'major axis bearing recovers the true 0.4 rad (mod pi)',
                  '%r' % d5['pc_ang'])
    bad += _check(d5['pc_a1'] > 20 * max(d5['pc_a2'], 1e-12),
                  'a collinear cloud has a1 >> a2')
    # pgap: one blob vs two separated modes.
    one = cs(np.linspace(0, 1, 200).tolist(), [0.0] * 200,
             [0.0] * 200, [0.005] * 200)
    two = cs((np.linspace(0, 0.1, 100).tolist()
              + np.linspace(0.9, 1.0, 100).tolist()), [0.0] * 200,
             [0.0] * 200, [0.005] * 200)
    bad += _check(one['pc_pgap'] < 0.01 < two['pc_pgap'],
                  'pc_pgap separates one blob (%.4f) from two modes (%.4f)'
                  % (one['pc_pgap'], two['pc_pgap']))
    # Circular yaw mean must not average +pi and -pi to 0.
    d6 = cs([0.0, 0.0], [0.0, 0.0], [3.1, -3.1], [0.5, 0.5])
    bad += _check(abs(abs(d6['pc_myaw']) - math.pi) < 0.05,
                  'yaw mean is CIRCULAR: +3.1 and -3.1 average to ~pi, '
                  'not 0', '%r' % d6['pc_myaw'])

    # ---- check 5: rounding keeps weights legible -------------------
    print()
    print('5. weight columns survive rounding')
    small = cs([0.0] * 2000, [0.0] * 2000, [0.0] * 2000,
               [1.0 / 2000] * 2000)
    rows = _fake_trace([], [(100.05, small)])
    r = [r for r in rows if r['t_rel'] == '0.1'][0]
    bad += _check(fl(r['pc_wmax']) not in (None, 0.0),
                  'a 1/2000 weight does not round to zero in the trace',
                  'pc_wmax=%r' % r['pc_wmax'])
    bad += _check(abs(fl(r['pc_ess']) - 2000.0) < 1e-3,
                  'ESS of 2000 flat weights reads 2000, not noise',
                  'pc_ess=%r' % r['pc_ess'])

    # ---- check 6: the earlier analyses still reproduce -------------
    print()
    print('6. frozen analyses reproduce (they read committed bundles, so')
    print('   this proves the instrumentation did not disturb them)')
    import re
    r = subprocess.run([sys.executable,
                        os.path.join(HERE, 'c2nav22_yaw.py'), 'selftest'],
                       capture_output=True, text=True)
    bad += _check(r.returncode == 0 and 'SELFTEST PASSED' in r.stdout,
                  'C2-NAV.22 selftest passes',
                  'rc=%d tail=%r' % (r.returncode, r.stdout[-160:]))
    # C2-NAV.29 reports "selftest: N/N passed" rather than the wording
    # C2-NAV.22 uses. Matched as N-of-N rather than against a hard-coded
    # 21, so adding a check there is not a failure here -- but a check
    # that stops passing IS.
    r = subprocess.run([sys.executable,
                        os.path.join(HERE, 'c2nav29_scanmap.py'),
                        '--selftest'], capture_output=True, text=True)
    m = re.search(r'selftest: (\d+)/(\d+) passed', r.stdout)
    bad += _check(r.returncode == 0 and m is not None
                  and m.group(1) == m.group(2) and int(m.group(2)) >= 21,
                  'C2-NAV.29 selftest passes (all checks, >= the 21 it '
                  'shipped with)',
                  'rc=%d got=%r' % (r.returncode,
                                    m.group(0) if m else None))
    r = subprocess.run([sys.executable,
                        os.path.join(HERE, 'c2nav24_chain.py'), 'stages'],
                       capture_output=True, text=True)
    mon = [ln for ln in r.stdout.splitlines()
           if ln.startswith('COLLISION MONITOR')]
    ok = bool(mon) and mon[0].split()[2:5] == ['3854', '0.301', '0.297']
    bad += _check(ok, 'C2-NAV.24 monitor stage still n=3854 median=0.301',
                  'got %r' % (mon[0] if mon else None,))
    # C2-NAV.28's own ANALYSES are untouched by this session even though
    # its session-scoped gate cannot pass any more; assert that directly.
    r = subprocess.run([sys.executable, AMCL_MODULE, 'ordering'],
                       capture_output=True, text=True)
    bad += _check(r.returncode == 0 and 'BEFORE' in r.stdout,
                  'C2-NAV.28 ordering analysis still runs',
                  'rc=%d' % r.returncode)

    # ---- check 7: nothing behavioural moved ------------------------
    print()
    print('7. no behavioural configuration was modified')
    allow = {'gazebo_models/scripts/nav_bench.py',
             'docs/data/c2nav28_amcl.py',
             'docs/data/c2nav30_cloud.py',
             'docs/data/c2nav30_cloud.json',
             'docs/data/c2nav30_extra.py',
             'docs/data/c2nav30_matrix.sh',
             'docs/SESSION_LOG.md'}
    changed = {ln for ln in _git('diff', '--name-only', BASE).stdout.split()
               if ln}
    print('  (vs %s, the C2-NAV.29 commit this session started from)' % BASE)
    bad += _check(bool(changed) and changed <= allow,
                  'every file this session touched is instrumentation, '
                  'analysis or notes',
                  'changed=%r unexpected=%r'
                  % (sorted(changed), sorted(changed - allow)))
    old = _git('show', BASE + ':gazebo_models/scripts/nav_bench.py').stdout
    with open(NAV_BENCH) as f:
        new = f.read()
    # The TOUR, the goals, the action plumbing and every constant above
    # it are the behaviour this benchmark IS. Assert that block byte for
    # byte rather than trusting the diff to be read by a human.
    # The block that IS this benchmark's behaviour: every constant, the
    # frame convention, the robot geometry and the whole TOUR. Asserted
    # byte for byte rather than left to a human reading a diff.
    #
    # NOT "everything before class Series", which was the first form of
    # this check and was wrong: that span also contains the import line
    # C2-NAV.30 legitimately edits, so it could only ever fail. The
    # import is pinned separately and exactly, just below.
    start, stop = 'WORLD_TO_MAP_X = 2.0', 'def apply_goal_overrides'
    bad += _check(old.split(start)[1].split(stop)[0]
                  == new.split(start)[1].split(stop)[0],
                  'the constants block -- frame convention, radii, '
                  'frequencies and the whole TOUR -- is byte-identical')
    o_imp = [ln for ln in old.splitlines() if ln.startswith('from ')
             or ln.startswith('import ')]
    n_imp = [ln for ln in new.splitlines() if ln.startswith('from ')
             or ln.startswith('import ')]
    bad += _check(
        [ln for ln in n_imp if ln not in o_imp]
        == ['from nav2_msgs.msg import CollisionMonitorState, ParticleCloud']
        and [ln for ln in o_imp if ln not in n_imp]
        == ['from nav2_msgs.msg import CollisionMonitorState'],
        'the only import change is ParticleCloud, added to an existing '
        'line',
        'added=%r removed=%r' % ([ln for ln in n_imp if ln not in o_imp],
                                 [ln for ln in o_imp if ln not in n_imp]))
    bad += _check(old.count('TOUR = [') == new.count('TOUR = [') == 1
                  and old.split('TOUR = [')[1].split(']')[0]
                  == new.split('TOUR = [')[1].split(']')[0],
                  'the TOUR -- every goal and waypoint -- is unchanged')
    # And the C2-NAV.28 diff-shape check, widened to the three regions
    # C2-NAV.30 legitimately touches and nothing else.
    import difflib
    ops = [op for op in difflib.SequenceMatcher(
        None, old.splitlines(), new.splitlines()).get_opcodes()
        if op[0] != 'equal']
    bad += _check(all(op[0] == 'insert' or
                      (op[0] == 'replace' and op[2] - op[1] <= 2)
                      for op in ops),
                  'the nav_bench.py diff is INSERTIONS (plus <=2-line '
                  'edits at the import and header lines), never deletions',
                  '%r' % [(o[0], o[1], o[2]) for o in ops])
    for name in ('c2nav11_ntp_params.yaml', 'c2nav25_slow_params.yaml'):
        p = os.path.join(HERE, name)
        d = _git('diff', '--name-only', BASE, '--',
                 os.path.relpath(p, WT)).stdout.strip()
        bad += _check(os.path.exists(p) and not d,
                      '%s unchanged vs %s' % (name, BASE))

    print()
    print('SELFTEST %s' % ('PASSED' if bad == 0 else 'FAILED (%d)' % bad))
    return 1 if bad else 0


# --------------------------------------------------------- 2. avail
def cmd_avail(args):
    """PARTICLE-CLOUD DATA QUALITY. Run this before believing anything."""
    hdr('C2-NAV.30 -- particle-cloud coverage and data quality')
    tag = args.run or RUN
    legs = legs_of(tag)
    if not legs:
        print('no record for %s (looked in %s and the bundle)'
              % (tag, RESULTS))
        return 1
    print('rows    = 10 Hz trace rows in the leg')
    print('fresh   = rows carrying a NEW /particle_cloud message')
    print('hz      = fresh / leg seconds: the FILTER\'s own cadence')
    print('maxgap  = longest run of rows with no new cloud, seconds')
    print('amcl    = rows carrying a fresh /amcl_pose, for comparison')
    print('n       = particle count (KLD varies it: min 500, max 2000)')
    print('ESS/n   = Kish effective sample size over particle count.')
    print('          1.000 exactly means the published weights are FLAT')
    print('          and say nothing about the weighting step.')
    print()
    print('%-16s%6s%7s%6s%8s%7s%14s%14s'
          % ('leg', 'rows', 'fresh', 'hz', 'maxgap', 'amcl', 'n min..max',
             'ESS/n med'))
    print('-' * 78)
    from_bundle = not os.path.exists(trace_path(tag, next(iter(legs))))
    for name, leg in legs.items():
        rows = read_trace(tag, name)
        if not rows:
            print('%-16s(no trace)' % name)
            continue
        fresh = [r for r in rows if r.get('pc_n')]
        amcl = [r for r in rows if r.get('amcl_x')]
        dur = fl(rows[-1]['t_rel']) or 0.0
        gap, cur = 0, 0
        for r in rows:
            cur = 0 if r.get('pc_n') else cur + 1
            gap = max(gap, cur)
        # Read from the bundle, `rows` is already decimated to the rows
        # that carry a sample, so the counts above describe the ARTIFACT
        # and not the leg. Prefer the values frozen at dump time.
        b = bundle().get('traces', {}).get('%s/%s' % (tag, name), {})
        if from_bundle and b.get('n_rows_total'):
            rows_n = b['n_rows_total']
            dur = b.get('duration_s') or dur
            gap = b.get('max_cloud_gap_rows', gap)
        else:
            rows_n = len(rows)
        ns = [fl(r['pc_n']) for r in fresh]
        ess = [fl(r['pc_ess']) / fl(r['pc_n']) for r in fresh
               if fl(r.get('pc_ess')) and fl(r['pc_n'])]
        print('%-16s%6d%7d%6.2f%8.1f%7d%14s%14s'
              % (name, rows_n, len(fresh),
                 (len(fresh) / dur) if dur else 0.0, gap * 0.1, len(amcl),
                 ('%d..%d' % (min(ns), max(ns))) if ns else '-',
                 ('%.4f' % med(ess)) if ess else '-'))
    print()
    if from_bundle:
        print('(read from the committed bundle; rows/hz/maxgap come from')
        print(' values frozen at dump time, not recomputed from the')
        print(' decimated rows the bundle carries.)')
        print()
    for name in legs:
        m = cloud_meta(tag, name)
        if m:
            print('%-16s frame_id=%-6s snapshots=%-5d particles=%-8d '
                  'ring_truncated=%s'
                  % (name, m.get('frame_id'), m.get('n_snapshots', 0),
                     m.get('n_particles_total', 0),
                     m.get('ring_truncated')))
    print()
    print('MISSING SAMPLES ARE EXPLICIT. A blank pc_ cell means no cloud')
    print('message landed in that 0.1 s bucket. Nothing below forward-')
    print('fills it, so every statistic is computed on rows where the')
    print('filter genuinely published.')
    return 0


# --------------------------------------------------------- 3. table
def cmd_table(args):
    """The central GT vs AMCL vs PARTICLE-CLOUD table."""
    tag = args.run or RUN
    for leg in (args.leg,) if args.leg else (CONTROL_LEG, WALL_LEG):
        S = samples(tag, leg)
        hdr('C2-NAV.30 -- %s / %s  (%d fresh cloud samples)'
            % (tag, leg, len(S)))
        if not S:
            print('no fresh /particle_cloud samples in this leg')
            continue
        print('all positions MAP frame, historical convention '
              '(world + %.1f, %.1f)' % (WORLD_TO_MAP_X, WORLD_TO_MAP_Y))
        print('dy is SIGNED toward the south wall: negative = southward')
        print('supGT/supAM = is GT / the AMCL pose inside the particle')
        print('              population\'s y range [pc_ylo, pc_yhi]')
        print()
        print('%6s %7s %7s %7s %7s %7s %6s %5s %7s %7s %6s %6s %4s %4s'
              % ('t', 'gt_x', 'gt_y', 'am_x', 'am_y', 'am_err', 'am_dy',
                 'pc_n', 'pc_mx', 'pc_my', 'pc_sy', 'pc_dy', 'sGT', 'sAM'))
        print('-' * 78)
        every = max(1, len(S) // args.rows)
        for i, d in enumerate(S):
            if i % every and i != len(S) - 1:
                continue
            print('%6.1f %7.3f %7.3f %7s %7s %7s %6s %5d %7.3f %7.3f '
                  '%6.3f %6.3f %4s %4s'
                  % (d['t'], d['gt_x_map'], d['gt_y_map'],
                     '%.3f' % d['amcl_x'] if d['amcl_x'] is not None else '-',
                     '%.3f' % d['amcl_y'] if d['amcl_y'] is not None else '-',
                     '%.3f' % d['amcl_err'] if d['amcl_err'] is not None
                     else '-',
                     '%.3f' % d['amcl_dy'] if d['amcl_dy'] is not None
                     else '-',
                     int(d['pc_n']), d['pc_mx'], d['pc_my'], d['pc_sy'],
                     d['cloud_dy'],
                     'Y' if d['gt_inside_y'] else 'n',
                     ('Y' if d['amcl_inside_y'] else 'n')
                     if d['amcl_inside_y'] is not None else '-'))
        print()
        _summarise(leg, S)
    return 0


def _summarise(leg, S):
    with_amcl = [d for d in S if d['amcl_err'] is not None]
    print('SUMMARY %s' % leg)
    print('  fresh cloud samples            %d' % len(S))
    print('  of which also carry AMCL       %d' % len(with_amcl))
    if with_amcl:
        print('  median AMCL error              %.4f m'
              % med([d['amcl_err'] for d in with_amcl]))
        print('  median AMCL dy (south neg)     %+.4f m'
              % med([d['amcl_dy'] for d in with_amcl]))
    print('  median cloud-MEAN error        %.4f m'
          % med([d['cloud_err'] for d in S]))
    print('  median cloud-MEAN dy           %+.4f m'
          % med([d['cloud_dy'] for d in S]))
    w = [d['wcloud_dy'] for d in S if d['wcloud_dy'] is not None]
    if w:
        print('  median cloud-WEIGHTED dy       %+.4f m' % med(w))
    print('  median particle count          %d' % med([d['pc_n'] for d in S]))
    print('  median sd along y              %.4f m'
          % med([d['pc_sy'] for d in S]))
    print('  median major/minor axis sd     %.4f / %.4f m'
          % (med([d['pc_a1'] for d in S]), med([d['pc_a2'] for d in S])))
    print('  GT inside particle y-range     %d / %d  (%.1f %%)'
          % (sum(1 for d in S if d['gt_inside_y']), len(S),
             100.0 * sum(1 for d in S if d['gt_inside_y']) / len(S)))
    print('  GT inside central 90 %% in y    %d / %d  (%.1f %%)'
          % (sum(1 for d in S if d['gt_inside_y90']), len(S),
             100.0 * sum(1 for d in S if d['gt_inside_y90']) / len(S)))
    if with_amcl:
        print('  AMCL inside particle y-range   %d / %d'
              % (sum(1 for d in with_amcl if d['amcl_inside_y']),
                 len(with_amcl)))
        print('  median |cloud mean - AMCL|     %.4f m'
              % med([math.hypot(d['pc_mx'] - d['amcl_x'],
                                d['pc_my'] - d['amcl_y'])
                     for d in with_amcl]))
    ess = [d['pc_ess'] / d['pc_n'] for d in S
           if d['pc_ess'] is not None and d['pc_n']]
    if ess:
        flat = sum(1 for e in ess if abs(e - 1.0) < 1e-6)
        print('  median ESS / n                 %.4f  (flat on %d of %d)'
              % (med(ess), flat, len(ess)))
    uq = [d['pc_nuniq'] / d['pc_n'] for d in S if d['pc_n']]
    print('  median distinct / total        %.4f' % med(uq))


# ------------------------------------------------------- 4. support
def cmd_support(args):
    """THE CRITICAL GEOMETRIC TEST, on the raw particles.

    Does the particle POPULATION physically extend across the true pose,
    or does it sit entirely on the wall side of it?
    """
    tag = args.run or RUN
    hdr('C2-NAV.30 -- cloud support around GT and around AMCL')
    print('Read from the FULL particle sets, not the summary columns.')
    print('frac_N = fraction of particles NORTH of ground truth in y')
    print('         (away from the south wall). 0.50 = the population is')
    print('         centred on truth; 0.00 = every particle is south of')
    print('         it, which is collapse to the wrong side.')
    print('d_near = distance from GT to the NEAREST particle.')
    print()
    any_leg = False
    for leg in (args.leg,) if args.leg else (CONTROL_LEG, WALL_LEG):
        clouds = read_clouds(tag, leg)
        if not clouds:
            print('%-16s no raw cloud file (needs .navbench; the bundle '
                  'carries derived stats only)' % leg)
            continue
        any_leg = True
        # Align each cloud snapshot to the nearest trace sample by sim
        # time. The trace stores t_rel, the snapshots store absolute sim
        # time, so recover t0 from the meta.
        m = cloud_meta(tag, leg) or {}
        t0 = m.get('t0_sim_s')
        rows = [(fl(r['t_rel']), r) for r in read_trace(tag, leg)
                if fl(r.get('x')) is not None]
        print()
        print('--- %s : %d snapshots ---' % (leg, len(clouds)))
        print('%6s %5s %8s %8s %8s %8s %8s %8s %8s'
              % ('t', 'n', 'gt_y', 'pc_ylo', 'pc_yhi', 'frac_N', 'd_near',
                 'wfrac_N', 'amcl_dy'))
        recs = []
        every = max(1, len(clouds) // args.rows)
        for k, (ts, arr) in enumerate(clouds):
            trel = (ts - t0) if t0 is not None else ts
            near = (min(rows, key=lambda rr: abs(rr[0] - trel))
                    if rows else None)
            if near is None or abs(near[0] - trel) > 0.5:
                continue
            r = near[1]
            gy = fl(r['y']) + WORLD_TO_MAP_Y
            gx = fl(r['x']) + WORLD_TO_MAP_X
            py, px, pw = arr[:, 1], arr[:, 0], arr[:, 3]
            frac_n = float((py > gy).mean())
            wsum = float(pw.sum())
            wfrac = (float(pw[py > gy].sum() / wsum) if wsum > 0
                     else float('nan'))
            dnear = float(np.min(np.hypot(px - gx, py - gy)))
            amdy = (fl(r['amcl_y']) - gy) if r.get('amcl_y') else None
            rec = {'t': round(trel, 2), 'n': int(len(arr)),
                   'gt_y': gy, 'ylo': float(py.min()), 'yhi': float(py.max()),
                   'frac_north': frac_n, 'wfrac_north': wfrac,
                   'd_near': dnear, 'amcl_dy': amdy}
            recs.append(rec)
            if k % every and k != len(clouds) - 1:
                continue
            print('%6.1f %5d %8.3f %8.3f %8.3f %8.3f %8.4f %8.3f %8s'
                  % (trel, len(arr), gy, rec['ylo'], rec['yhi'], frac_n,
                     dnear, wfrac,
                     '%+.3f' % amdy if amdy is not None else '-'))
        if not recs:
            continue
        print()
        print('SUPPORT SUMMARY %s over %d snapshots' % (leg, len(recs)))
        fr = [r['frac_north'] for r in recs]
        print('  frac of particles north of GT   median %.4f  '
              'min %.4f  max %.4f' % (med(fr), min(fr), max(fr)))
        print('  snapshots with ZERO particles north of GT   %d / %d'
              % (sum(1 for v in fr if v == 0.0), len(fr)))
        print('  snapshots with GT inside [ylo, yhi]         %d / %d'
              % (sum(1 for r in recs if r['ylo'] <= r['gt_y'] <= r['yhi']),
                 len(recs)))
        print('  nearest particle to GT          median %.4f m  max %.4f m'
              % (med([r['d_near'] for r in recs]),
                 max(r['d_near'] for r in recs)))
        wf = [r['wfrac_north'] for r in recs
              if not math.isnan(r['wfrac_north'])]
        if wf:
            print('  WEIGHTED frac north of GT       median %.4f' % med(wf))
        print('  median particle y-range         %.4f m'
              % med([r['yhi'] - r['ylo'] for r in recs]))
    if not any_leg:
        print()
        print('No raw particle files found under %s.' % RESULTS)
        return 1
    return 0


# --------------------------------------------------------- 5. modes
def cmd_modes(args):
    """Explicit multimodality detection on the raw particle sets.

    A mean is NOT presented as equivalent to the distribution. This
    splits each cloud on its own major axis and reports whether the
    population is one blob or several, with the gap that separates them.
    """
    tag = args.run or RUN
    hdr('C2-NAV.30 -- multimodality, measured not assumed')
    print('Each cloud is projected onto its own major axis and cut at')
    print('every gap wider than `--gap` metres (default one map cell,')
    print('0.05 m). A single blob yields one cluster. Clusters are')
    print('reported with their size and their mean y, so a second mode')
    print('sitting on the truth side of the wall would be visible.')
    print()
    for leg in (args.leg,) if args.leg else (CONTROL_LEG, WALL_LEG):
        clouds = read_clouds(tag, leg)
        if not clouds:
            print('%-16s no raw cloud file' % leg)
            continue
        m = cloud_meta(tag, leg) or {}
        t0 = m.get('t0_sim_s')
        counts = {}
        worst = None
        shares = []        # dominant cluster's share of the particles
        seconds = []       # SECOND-largest cluster's share
        for ts, arr in clouds:
            x, y = arr[:, 0].astype(float), arr[:, 1].astype(float)
            if len(x) < 2:
                continue
            cov = np.cov(np.vstack([x, y]), ddof=0)
            ev, evec = np.linalg.eigh(cov)
            ax = evec[:, int(np.argmax(ev))]
            proj = np.sort(x * ax[0] + y * ax[1])
            cuts = np.nonzero(np.diff(proj) > args.gap)[0]
            k = len(cuts) + 1
            counts[k] = counts.get(k, 0) + 1
            bounds = [0] + [int(c) + 1 for c in cuts] + [len(proj)]
            sizes = sorted((b - a for a, b in zip(bounds[:-1], bounds[1:])),
                           reverse=True)
            shares.append(sizes[0] / float(len(proj)))
            seconds.append((sizes[1] / float(len(proj))) if len(sizes) > 1
                           else 0.0)
            if worst is None or k > worst[0]:
                worst = (k, ts, arr, proj, cuts)
        tot = sum(counts.values())
        print('--- %s : %d snapshots ---' % (leg, tot))
        for k in sorted(counts):
            print('   %d cluster%s : %4d snapshots (%.1f %%)'
                  % (k, ' ' if k == 1 else 's', counts[k],
                     100.0 * counts[k] / tot))
        # The cluster COUNT alone overstates multimodality badly: at this
        # gap a single blob plus a handful of stragglers in the tail
        # counts as "6 clusters". What decides whether a cloud is really
        # bimodal is how much mass the SECOND cluster carries.
        print('   dominant cluster share : median %.4f  min %.4f'
              % (med(shares), min(shares)))
        print('   SECOND cluster share   : median %.4f  max %.4f'
              % (med(seconds), max(seconds)))
        print('   snapshots where the second cluster holds > 10 %% of the'
              ' particles: %d / %d'
              % (sum(1 for v in seconds if v > 0.10), len(seconds)))
        if worst and worst[0] > 1:
            k, ts, arr, proj, cuts = worst
            print('   widest split, t=%.1f s, %d clusters:'
                  % ((ts - t0) if t0 is not None else ts, k))
            bounds = [0] + [int(c) + 1 for c in cuts] + [len(proj)]
            for a, b in zip(bounds[:-1], bounds[1:]):
                print('     %5d particles, projection %.3f .. %.3f'
                      % (b - a, proj[a], proj[b - 1]))
        else:
            print('   no snapshot in this leg splits at the %.3f m gap'
                  % args.gap)
    return 0


# ------------------------------------------------------- 6. regions
def cmd_regions(args):
    """Wall-adjacent vs the control region, SAME simulator instance."""
    tag = args.run or RUN
    hdr('C2-NAV.30 -- wall-adjacent vs control, one simulator instance')
    print('Both legs come from run %s, so the comparison controls for'
          % tag)
    print('the simulator instance rather than straddling two of them.')
    print()
    print('%-22s%14s%14s' % ('quantity', CONTROL_LEG, WALL_LEG))
    print('-' * 78)
    A, B = samples(tag, CONTROL_LEG), samples(tag, WALL_LEG)
    if not A or not B:
        print('missing samples: %s=%d %s=%d'
              % (CONTROL_LEG, len(A), WALL_LEG, len(B)))
        return 1

    def row(label, f, fmt='%+.4f'):
        va, vb = f(A), f(B)
        print('%-22s%14s%14s'
              % (label, (fmt % va) if va is not None else '-',
                 (fmt % vb) if vb is not None else '-'))

    row('n samples', lambda S: len(S), '%d')
    row('AMCL err (med)',
        lambda S: med([d['amcl_err'] for d in S
                       if d['amcl_err'] is not None]), '%.4f')
    row('AMCL dy (med)',
        lambda S: med([d['amcl_dy'] for d in S
                       if d['amcl_dy'] is not None]))
    row('cloud-mean dy (med)', lambda S: med([d['cloud_dy'] for d in S]))
    row('cloud sd y (med)', lambda S: med([d['pc_sy'] for d in S]), '%.4f')
    row('major axis sd (med)', lambda S: med([d['pc_a1'] for d in S]),
        '%.4f')
    row('particle count (med)', lambda S: med([d['pc_n'] for d in S]), '%d')
    row('distinct/total (med)',
        lambda S: med([d['pc_nuniq'] / d['pc_n'] for d in S if d['pc_n']]),
        '%.4f')
    row('GT in y-range (%)',
        lambda S: 100.0 * sum(1 for d in S if d['gt_inside_y']) / len(S),
        '%.1f')
    row('GT in central 90% (%)',
        lambda S: 100.0 * sum(1 for d in S if d['gt_inside_y90']) / len(S),
        '%.1f')
    row('|cloud mean-AMCL| med',
        lambda S: med([math.hypot(d['pc_mx'] - d['amcl_x'],
                                  d['pc_my'] - d['amcl_y'])
                       for d in S if d['amcl_x'] is not None]), '%.4f')
    row('scan_min (med)',
        lambda S: med([d['scan_min'] for d in S
                       if d['scan_min'] is not None]), '%.4f')
    return 0


# ------------------------------------------------------ 7. temporal
def cmd_temporal(args):
    """When does the bias appear, relative to entering the region?"""
    tag = args.run or RUN
    leg = args.leg or WALL_LEG
    S = samples(tag, leg)
    hdr('C2-NAV.30 -- temporal alignment, %s / %s' % (tag, leg))
    if not S:
        print('no samples')
        return 1
    print('One row per fresh cloud message -- the filter\'s own cadence,')
    print('not a resampled 10 Hz line. Gaps in t are gaps in the filter.')
    print()
    print('%6s %6s %8s %8s %8s %8s %8s %8s %7s'
          % ('t', 'dt', 'gt_y', 'am_dy', 'pc_dy', 'pc_sy', 'pc_n',
             'v_act', 'polygon'))
    print('-' * 78)
    prev = None
    every = max(1, len(S) // args.rows)
    for i, d in enumerate(S):
        if i % every and i != len(S) - 1:
            continue
        dt = (d['t'] - prev) if prev is not None else float('nan')
        prev = d['t']
        print('%6.1f %6.2f %8.3f %8s %8.3f %8.4f %8d %8s %7s'
              % (d['t'], dt, d['gt_y_map'],
                 '%+.3f' % d['amcl_dy'] if d['amcl_dy'] is not None else '-',
                 d['cloud_dy'], d['pc_sy'], int(d['pc_n']),
                 '%+.3f' % d['v_act'] if d['v_act'] is not None else '-',
                 (d['cm_polygon'] or '-')[:7]))
    print()
    # Does the cloud bias track motion or sit still?
    dys = [d['cloud_dy'] for d in S]
    print('cloud-mean dy: first %.4f  median %.4f  last %.4f  '
          'p05 %.4f p95 %.4f'
          % (dys[0], med(dys), dys[-1], pct(dys, 5), pct(dys, 95)))
    moving = [d['cloud_dy'] for d in S
              if d['v_act'] is not None and abs(d['v_act']) > 0.02]
    still = [d['cloud_dy'] for d in S
             if d['v_act'] is not None and abs(d['v_act']) <= 0.02]
    print('cloud-mean dy while MOVING  n=%d median %s'
          % (len(moving), ('%.4f' % med(moving)) if moving else '-'))
    print('cloud-mean dy while STOPPED n=%d median %s'
          % (len(still), ('%.4f' % med(still)) if still else '-'))
    sd = [d['pc_sy'] for d in S]
    print('cloud sd y:    first %.4f  median %.4f  last %.4f'
          % (sd[0], med(sd), sd[-1]))
    return 0


# ------------------------------------------------------- 8. verdict
def cmd_verdict(args):
    """The A/B/C/D classification, stated against its own evidence."""
    tag = args.run or RUN
    hdr('C2-NAV.30 -- causal classification')
    S = samples(tag, WALL_LEG)
    C = samples(tag, CONTROL_LEG)
    if not S:
        print('no wall-adjacent samples: INDETERMINATE (D) by absence of '
              'data, which is not a finding about AMCL')
        return 1
    inside = sum(1 for d in S if d['gt_inside_y'])
    inside90 = sum(1 for d in S if d['gt_inside_y90'])
    amcl = [d for d in S if d['amcl_err'] is not None]
    ess = [d['pc_ess'] / d['pc_n'] for d in S
           if d['pc_ess'] is not None and d['pc_n']]
    flat = sum(1 for e in ess if abs(e - 1.0) < 1e-6)
    print('EVIDENCE')
    print('  wall-adjacent fresh cloud samples          %d' % len(S))
    print('  GT inside the particle y-range             %d / %d (%.1f %%)'
          % (inside, len(S), 100.0 * inside / len(S)))
    print('  GT inside the central 90 %% in y            %d / %d (%.1f %%)'
          % (inside90, len(S), 100.0 * inside90 / len(S)))
    print('  median cloud-mean dy                       %+.4f m'
          % med([d['cloud_dy'] for d in S]))
    if amcl:
        print('  median AMCL dy                             %+.4f m'
              % med([d['amcl_dy'] for d in amcl]))
        print('  median |cloud mean - AMCL pose|            %.4f m'
              % med([math.hypot(d['pc_mx'] - d['amcl_x'],
                                d['pc_my'] - d['amcl_y']) for d in amcl]))
    print('  median cloud sd in y                       %.4f m'
          % med([d['pc_sy'] for d in S]))
    print('  median distinct positions / particles      %.4f'
          % med([d['pc_nuniq'] / d['pc_n'] for d in S if d['pc_n']]))
    if ess:
        print('  weights FLAT (ESS == n) on                 %d / %d samples'
              % (flat, len(ess)))
    if C:
        print('  control (%s) median cloud dy       %+.4f m'
              % (CONTROL_LEG, med([d['cloud_dy'] for d in C])))
        print('  control median cloud sd in y               %.4f m'
              % med([d['pc_sy'] for d in C]))
    print()
    print('READING (the classification follows the numbers above; it is')
    print('not asserted independently of them):')
    print('  A DEPLETION/COLLAPSE  needs the cloud NARROW and centred near')
    print('    the biased pose with little or no support at GT.')
    print('  B WEIGHTING/MULTIMODALITY  needs the cloud still spanning GT')
    print('    while the reported output sits south of it.')
    print('  C HEALTHY CLOUD  needs the cloud broadly consistent with GT.')
    print('  D INDETERMINATE.')
    print()
    if ess and flat == len(ess):
        print('LIMITATION, and it is not optional. The published weights')
        print('are FLAT on every sample (ESS == n exactly), which is what')
        print('`resample_interval: 1` produces: the set is published after')
        print('resampling has already levelled them. So this run')
        print('distinguishes cloud GEOMETRY and SUPPORT only. It does NOT')
        print('observe the importance weights that drove the resampling,')
        print('and therefore cannot by itself diagnose the weighting or')
        print('resampling mechanism.')
    return 0


# ---------------------------------------------------------- 9. dump
def cmd_dump(args):
    """Freeze what the analyses read into the committed bundle.

    Raw particle sets are far too large to commit, so per-snapshot
    DERIVED statistics are frozen instead and `support`'s summary is
    recomputed from them. Commands that genuinely need raw particles
    (`modes`) say so when scratch is gone rather than printing something
    that looks like a measurement.
    """
    tag = args.run or RUN
    out = {'run': tag, 'legs': {tag: {}}, 'traces': {}, 'cloud_meta': {},
           'support': {}}
    legs = legs_of(tag)
    for name, leg in legs.items():
        out['legs'][tag][name] = leg
        rows = read_trace(tag, name)
        if rows:
            schema = list(rows[0].keys())
            keep = [r for r in rows if r.get('pc_n') or r.get('amcl_x')]
            # The bundle keeps only rows carrying a fresh cloud or a
            # fresh AMCL sample -- the whole 10 Hz trace would be far
            # larger than a committed artifact should be. That decimation
            # would make `avail`'s coverage columns WRONG when they are
            # computed from the bundle (maxgap would read 0.0 because
            # every retained row carries a sample). So the quantities
            # that depend on the discarded rows are carried explicitly
            # rather than recomputed from what survived.
            gaps, cur = [], 0
            for r in rows:
                if r.get('pc_n'):
                    gaps.append(cur)
                    cur = 0
                else:
                    cur += 1
            gaps.append(cur)
            out['traces']['%s/%s' % (tag, name)] = {
                'schema': schema,
                'n_rows_total': len(rows),
                'duration_s': fl(rows[-1]['t_rel']),
                'max_cloud_gap_rows': max(gaps) if gaps else 0,
                'rows': [[r[k] for k in schema] for r in keep]}
        m = cloud_meta(tag, name)
        if m:
            out['cloud_meta']['%s/%s' % (tag, name)] = m
        recs = []
        for ts, arr in read_clouds(tag, name):
            x, y, w = (arr[:, 0].astype(float), arr[:, 1].astype(float),
                       arr[:, 3].astype(float))
            recs.append({'ts': round(float(ts), 3), 'n': int(len(arr)),
                         'ylo': round(float(y.min()), 5),
                         'yhi': round(float(y.max()), 5),
                         'xlo': round(float(x.min()), 5),
                         'xhi': round(float(x.max()), 5),
                         'my': round(float(y.mean()), 5),
                         'mx': round(float(x.mean()), 5),
                         'wsum': float(w.sum())})
        if recs:
            out['support']['%s/%s' % (tag, name)] = recs
    with open(BUNDLE, 'w') as f:
        json.dump(out, f)
    print('wrote %s  (%d legs, %d traces, %d support series, %.1f KB)'
          % (BUNDLE, len(out['legs'][tag]), len(out['traces']),
             len(out['support']), os.path.getsize(BUNDLE) / 1024.0))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest='cmd')
    for name, fn, helptext in (
            ('selftest', cmd_selftest, 'the Part A gate, offline'),
            ('avail', cmd_avail, 'cloud coverage and data quality'),
            ('table', cmd_table, 'GT vs AMCL vs cloud'),
            ('support', cmd_support, 'the critical geometric test'),
            ('modes', cmd_modes, 'multimodality detection'),
            ('regions', cmd_regions, 'wall-adjacent vs control'),
            ('temporal', cmd_temporal, 'when the bias appears'),
            ('verdict', cmd_verdict, 'the classification'),
            ('dump', cmd_dump, 'freeze the committed bundle')):
        p = sub.add_parser(name, help=helptext)
        p.set_defaults(fn=fn)
        p.add_argument('--run', default=None)
        p.add_argument('--leg', default=None)
        p.add_argument('--rows', type=int, default=30)
        p.add_argument('--gap', type=float, default=0.05)
    args = ap.parse_args(argv)
    if not getattr(args, 'fn', None):
        ap.print_help()
        return 2
    return args.fn(args)


if __name__ == '__main__':
    sys.exit(main())
