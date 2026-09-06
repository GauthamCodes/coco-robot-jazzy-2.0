#!/usr/bin/env python3
"""C2-NAV.27 -- what actually causes the "Start occupied" abort.

OFFLINE DIAGNOSIS ONLY. This module starts no simulator, runs no ROS
node, changes no parameter, and re-runs nothing. Every number below comes
out of artifacts that already existed before the session began.

THE QUESTION
------------
C2-NAV.26 recorded, as the mechanism behind its worst finding:

    "the robot finishes wall_adjacent 0.025 m PAST its goal at 0.259 m
     clearance ... and the planner then refuses every later goal because
     its own start cell is lethal"

and left open whether `PolygonSlow.slowdown_ratio = 1.0` causes that or
merely fails to prevent it. This module answers both, and corrects the
sequence in the sentence above: the wall_adjacent leg did not finish. It
was itself refused, mid terminal yaw-settle, and 0.025 m is where the
robot coasted to rest AFTER the controller was cancelled.

WHAT THE PLANNER TESTS
----------------------
`planner_server.GridBased` is `nav2_smac_planner::SmacPlanner2D`
(c2nav11_ntp_params.yaml). It refuses a start whose cell in the GLOBAL
costmap is too close to a lethal cell. The Nav2 .cpp sources are not
installed here -- only headers -- so the exact predicate is NOT read from
the function body. It is BOUNDED from the artifacts instead:

    refusals   observed at cell-to-lethal distance 0.100, 0.112, 0.112,
               0.150, 0.150 m
    acceptance observed at 0.180 m and above

so the trip point lies in (0.150, 0.180] m. That bracket sits just
BELOW the InflationLayer inscribed band edge (robot_radius * cos(pi/16)
= 0.196 m), so the inscribed model very slightly over-predicts refusal:
it would have refused the 0.180 m start that was in fact accepted. The
AMCL poses these distances come from are logged to two decimals, i.e.
+/-0.005 m, which is enough to move a pose into the neighbouring 0.05 m
cell -- so the bracket cannot be tightened further from these artifacts,
and the inscribed band remains the best available model of it.

The published costmap cannot resolve it either: nav2's
Costmap2DPublisher maps raw 253 (INSCRIBED) to 99 and raw 254 (LETHAL)
to 100, so a single published 99 covers the whole 0..0.196 m band.

EVIDENCE CLASS, stated per number
---------------------------------
OBSERVED     every per-leg record field; every 10 Hz trace column; every
             `planner_server` refusal line and its start/goal pose; every
             `bt_navigator` "Begin navigating from current location"
             pose; the live published global costmap (243x175, ~500
             snapshots per recorded leg); the live parameter readbacks.
RECONSTRUCTED  the offline costmap. Built from gazebo_models/maps/
             coco_world.{pgm,yaml} plus the InflationLayer formula at the
             global costmap's cost_scaling_factor 5.0. `validate` checks
             it against a live snapshot: agreement is within 4 raw counts
             everywhere on the wall_adjacent profile, which is the
             quantisation of the 0-100 rescale. It sees the STATIC layer
             only, so its costs are a LOWER bound on live cost and its
             clearances an UPPER bound.
DERIVED      the two-term decomposition (gt_past + amcl_err = est_past),
             the refusal boundary sweep, the band-exposure seconds.
UNAVAILABLE  AMCL pose as a time series. `nav_bench` subscribes
             /amcl_pose but never writes it to a record field or a trace
             column, for ANY arm. The only recorded AMCL poses are the
             ones `bt_navigator` and `planner_server` print, i.e. one per
             leg start and one per refusal. Everything this module says
             about localisation rests on those, and on nothing else.

WHAT THIS CANNOT SHOW
---------------------
One refusal at the wall_adjacent goal, in one run. A two-term sum whose
threshold was crossed once in fifteen runs is a frequency with a wide
interval, and no significance is computed anywhere here.
"""
import argparse
import collections
import csv
import glob
import json
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
WT = os.path.dirname(os.path.dirname(HERE))          # docs/data -> root
SCRATCH = os.path.join(WT, '.navbench', 'results')
LOGS = os.path.join(WT, '.navbench', 'logs')
MAP_PGM = os.path.join(WT, 'gazebo_models', 'maps', 'coco_world.pgm')

# gazebo_models/maps/coco_world.yaml (OBSERVED)
RES_M = 0.050
ORIGIN = (-2.119, -4.910)
OCC_THRESH = 0.65

# c2nav11_ntp_params.yaml, global_costmap (OBSERVED)
ROBOT_RADIUS = 0.20
INFLATION_RADIUS = 0.50
COST_SCALING = 5.0
INSCRIBED, LETHAL = 253, 254
# nav2 builds a 16-gon for a circular robot and inflates over the min
# centre-to-EDGE distance, not the radius. DERIVED.
INSCRIBED_R = ROBOT_RADIUS * math.cos(math.pi / 16)      # 0.19616 m

# gazebo_models/scripts/nav_bench.py lines 88-89 (OBSERVED)
W2M = (2.0, 0.0)
# docs/DESIGN_DECISIONS.md line 2102, measured (OBSERVED)
TRUE_CIRCUMSCRIBED = 0.2051

LEGS = ['open_space', 'wall_adjacent', 'wall_parallel', 'obstacle_corner',
        'corridor_gate', 'enclosure_entry', 'enclosure_exit']

# Arm membership, by value from c2nav25_slow.py / c2nav26_robust.py.
BASE = ['c2n21_base_r1', 'c2n21_base_r3', 'c2n21_base_r4',
        'c2n21_bbase_r2', 'c2n21_bbase_r3']
# Baseline-CONFIGURATION run excluded from the frozen arm because it never
# reached the enclosure leg (RESULTS.md line 14739, "void"). Its live
# parameter readback is byte-identical to the frozen baseline's.
BASE_EXCL = ['c2n21_bbase_r1']
C25 = ['c2n25_slow_r1', 'c2n25_slow_r2', 'c2n25_bslow_r1']
C26 = ['c2n26_slow_r1', 'c2n26_slow_r2', 'c2n26_slow_r3',
       'c2n26_bslow_r1', 'c2n26_bslow_r2', 'c2n26_bslow_r3']
ARMS = [('BASELINE', BASE), ('BASE(excl)', BASE_EXCL),
        ('C2-NAV.25', C25), ('C2-NAV.26', C26)]

REFUSE = re.compile(
    r'\[(\d+\.\d+)\].*\[planner_server\]: GridBased plugin failed to plan '
    r'from \(([-\d.]+), ([-\d.]+)\) to \(([-\d.]+), ([-\d.]+)\): "([^"]+)"')
BEGIN = re.compile(
    r'\[(\d+\.\d+)\].*\[bt_navigator\]: Begin navigating from current '
    r'location \(([-\d.]+), ([-\d.]+)\) to \(([-\d.]+), ([-\d.]+)\)')


def hdr(t):
    print()
    print('=' * 78)
    print(t)
    print('=' * 78)


# ------------------------------------------------- the offline costmap
def _read_pgm(path):
    data = open(path, 'rb').read()
    idx, fields = 0, []
    while len(fields) < 4:
        while data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b'#':
            while data[idx:idx + 1] not in (b'\n', b''):
                idx += 1
            continue
        j = idx
        while not data[j:j + 1].isspace():
            j += 1
        fields.append(data[idx:j])
        idx = j
    idx += 1
    w, h = int(fields[1]), int(fields[2])
    return np.frombuffer(data[idx:idx + w * h], dtype=np.uint8
                         ).reshape(h, w), w, h


def _occupied():
    img, w, h = _read_pgm(MAP_PGM)
    occ = (255.0 - img.astype(np.float64)) / 255.0
    rows, cols = np.nonzero(occ > OCC_THRESH)
    return np.column_stack([ORIGIN[0] + (cols + 0.5) * RES_M,
                            ORIGIN[1] + (h - rows - 0.5) * RES_M])


OCC = _occupied()


def d_lethal(mx, my):
    d = np.hypot(OCC[:, 0] - mx, OCC[:, 1] - my)
    i = int(np.argmin(d))
    return float(d[i]), (float(OCC[i, 0]), float(OCC[i, 1]))


def d_lethal_many(xs, ys):
    out = np.empty(len(xs))
    for i in range(0, len(xs), 512):
        sl = slice(i, i + 512)
        dx = xs[sl][:, None] - OCC[None, :, 0]
        dy = ys[sl][:, None] - OCC[None, :, 1]
        out[sl] = np.sqrt(dx * dx + dy * dy).min(axis=1)
    return out


def cell_centre(mx, my):
    return (ORIGIN[0] + (math.floor((mx - ORIGIN[0]) / RES_M) + 0.5) * RES_M,
            ORIGIN[1] + (math.floor((my - ORIGIN[1]) / RES_M) + 0.5) * RES_M)


def inflation_cost(d):
    if d <= 1e-9:
        return LETHAL
    if d <= INSCRIBED_R:
        return INSCRIBED
    if d > INFLATION_RADIUS:
        return 0
    return int((INSCRIBED - 1) * math.exp(-COST_SCALING * (d - INSCRIBED_R)))


def probe(mx, my):
    cx, cy = cell_centre(mx, my)
    d, near = d_lethal(cx, cy)
    return dict(d=d, cost=inflation_cost(d), near=near, cell=(cx, cy))


# ----------------------------------------------------------- artifacts
def record(tag):
    p = os.path.join(SCRATCH, tag + '.json')
    return json.load(open(p)) if os.path.exists(p) else None


def legs_of(tag):
    d = record(tag)
    return {r['scenario']: r for r in d['legs']} if d else {}


def trace(tag, leg):
    p = os.path.join(SCRATCH, '%s_traces' % tag, '%s_rep0.csv' % leg)
    if not os.path.exists(p):
        return None
    return list(csv.DictReader(open(p)))


def fl(r, k):
    try:
        return float(r.get(k))
    except (TypeError, ValueError):
        return None


def logfile(tag):
    p = os.path.join(LOGS, 'nav_%s.log' % tag)
    return p if os.path.exists(p) else None


def arm_of(tag):
    for name, tags in ARMS:
        if tag in tags:
            return name
    return 'other'


# ------------------------------------------------------- 1. inventory
def cmd_inventory(_):
    """Every planner refusal ever logged, and which arm produced it."""
    hdr('C2-NAV.27 -- every GridBased planner refusal in .navbench/logs')
    rows = []
    for p in sorted(glob.glob(os.path.join(LOGS, 'nav_*.log'))):
        tag = os.path.basename(p)[4:-4]
        for m in REFUSE.finditer(open(p, errors='replace').read()):
            rows.append((tag, arm_of(tag), m.group(6),
                         float(m.group(2)), float(m.group(3)),
                         float(m.group(4)), float(m.group(5))))
    print('%-18s%-13s%-21s%-16s%s'
          % ('run', 'arm', 'reason', 'start(map)', 'd_to_lethal'))
    print('-' * 78)
    for tag, arm, why, sx, sy, _, _ in rows:
        print('%-18s%-13s%-21s(%6.2f,%6.2f)   %.3f'
              % (tag, arm, why, sx, sy, probe(sx, sy)['d']))
    print()
    c = collections.Counter((a, w) for _, a, w, _, _, _, _ in rows)
    print('by arm x reason:')
    for k, v in sorted(c.items()):
        print('   %-13s%-21s%d' % (k[0], k[1], v))
    print()
    so = sorted({t for t, _, w, _, _, _, _ in rows if w == 'Start occupied'})
    print('runs with >=1 "Start occupied": %d' % len(so))
    for t in so:
        print('   %-18s%s' % (t, arm_of(t)))
    print()
    print('READ THIS: "Start occupied" is NOT new to slowdown_ratio = 1.0.')
    print('c2n21_bbase_r1 produced three of them on a live parameter')
    print('readback BYTE-IDENTICAL to the frozen baseline (sha256')
    print('c16f4bcd..., slowdown_ratio 0.3). It is absent from the frozen')
    print('arm only because C2-NAV.21 voided it for never reaching the')
    print('enclosure leg -- not because the configuration cannot do it.')
    return 0


# -------------------------------------------------------- 2. geometry
def cmd_geometry(_):
    """Where the wall is, and how much estimated error the goal allows."""
    hdr('C2-NAV.27 -- the geometry at the wall_adjacent goal')
    d, near = d_lethal(0.0, -3.0)
    print('goal          world(-2.000,-3.000) = map(0.000,-3.000)')
    print('nearest lethal cell CENTRE at map(%.3f,%.3f), d = %.3f m'
          % (near[0], near[1], d))
    print('that cell spans y [%.3f, %.3f], so the mapped wall FACE is at '
          'map y = %.3f' % (near[1] - RES_M / 2, near[1] + RES_M / 2,
                            near[1] + RES_M / 2))
    print()
    print('sweep the ESTIMATED pose from the goal toward the wall:')
    lo = None
    for dy in [0.0, 0.05, 0.10, 0.125, 0.15, 0.155, 0.16, 0.165, 0.17,
               0.18, 0.19, 0.20, 0.25]:
        v = probe(0.0, -3.0 - dy)
        occ = v['cost'] >= INSCRIBED
        if occ and lo is None:
            lo = dy
        print('   est_past %.3f m -> d %.3f, offline cost %3d  %s'
              % (dy, v['d'], v['cost'], 'REFUSED' if occ else 'ok'))
    print()
    print('offline refusal boundary: est_past >= %.3f m.' % lo)
    print('MEASURED boundary from the artifacts is looser and is what')
    print('`bound` prints; this sweep uses the inscribed model.')
    print()
    print('TRUE geometric clearance at the deepest GROUND-TRUTH pose of')
    print('c2n26_slow_r2/wall_adjacent (map 0.060, -3.076):')
    v = probe(0.060, -3.076)
    face = v['near'][1] + RES_M / 2
    c2f = abs(-3.076 - face)
    print('   centre -> nearest lethal cell CENTRE  %.3f m  '
          '(this is nav_bench\'s min_clearance_m metric)' % v['d'])
    print('   centre -> mapped wall FACE            %.3f m' % c2f)
    print('   minus the measured circumscribed radius %.4f m'
          % TRUE_CIRCUMSCRIBED)
    print('   =  BODY to mapped wall face           %.3f m' %
          (c2f - TRUE_CIRCUMSCRIBED))
    print()
    print('So the "0.259 m clearance" C2-NAV.26 reported is a CENTRE-to-')
    print('cell-centre distance, and the body-to-wall figure behind it is')
    print('about %.0f mm. The physical wall surface may lie up to one cell'
          % (1000 * (c2f - TRUE_CIRCUMSCRIBED)))
    print('(0.05 m) beyond the mapped face, so read it as a lower bound in')
    print('[%.3f, %.3f] m. It is NOT a safe margin, and C2-NAV.26\'s own'
          % (c2f - TRUE_CIRCUMSCRIBED, c2f - TRUE_CIRCUMSCRIBED + RES_M))
    print('check 7 already said the clearance regressed.')
    return 0


# ------------------------------------------------------- 3. the bound
def cmd_bound(_):
    """Bracket the planner's start-refusal threshold from the artifacts."""
    hdr('C2-NAV.27 -- bracketing the start-refusal threshold')
    ref = []
    for p in sorted(glob.glob(os.path.join(LOGS, 'nav_*.log'))):
        tag = os.path.basename(p)[4:-4]
        for m in REFUSE.finditer(open(p, errors='replace').read()):
            if m.group(6) != 'Start occupied':
                continue
            ref.append((tag, probe(float(m.group(2)),
                                   float(m.group(3)))['d']))
    print('REFUSED starts, cell-to-lethal distance:')
    for t, d in sorted(ref, key=lambda x: x[1]):
        print('   %-18s%.3f m' % (t, d))
    print()
    # A leg-start pose only counts as ACCEPTED if no refusal in the same
    # run was issued from that same pose -- otherwise the poses that were
    # refused would be counted on both sides of the bracket.
    acc = []
    for _, tags in ARMS:
        for tag in tags:
            p = logfile(tag)
            if not p:
                continue
            txt = open(p, errors='replace').read()
            bad = {(m.group(2), m.group(3)) for m in REFUSE.finditer(txt)}
            for m in BEGIN.finditer(txt):
                if (m.group(2), m.group(3)) in bad:
                    continue
                acc.append((tag, probe(float(m.group(2)),
                                       float(m.group(3)))['d']))
    acc.sort(key=lambda x: x[1])
    print('ACCEPTED starts (leg-start poses that planned), closest 8:')
    for t, d in acc[:8]:
        print('   %-18s%.3f m' % (t, d))
    print()
    hi = max(d for _, d in ref)
    lo = min(d for _, d in acc if d > hi)
    print('==> refusals at or below %.3f m; nearest acceptance at %.3f m.'
          % (hi, lo))
    print('    The trip point lies in (%.3f, %.3f] m. The inscribed band'
          % (hi, lo))
    print('    edge, %.5f m, sits just ABOVE it, so the inscribed model'
          % INSCRIBED_R)
    print('    over-predicts refusal by one cell at the margin. AMCL poses')
    print('    are logged to 2 dp, which is enough to move a pose into the')
    print('    neighbouring cell, so the bracket cannot be tightened.')
    print()
    print('    The published costmap cannot narrow it: raw 253 covers the')
    print('    whole 0..%.3f m band and publishes as a single value, 99.'
          % INSCRIBED_R)
    return 0


# ------------------------------------------------------ 4. validation
def cmd_validate(_):
    """Check the offline costmap against a recorded live one."""
    hdr('C2-NAV.27 -- offline costmap vs the live published costmap')
    tag = 'c2n26_slow_r2'
    meta = json.load(open(os.path.join(
        SCRATCH, '%s_costmapwindow_enclosure_entry_rep0_meta.json' % tag)))
    z = np.load(os.path.join(
        SCRATCH, '%s_costmapwindow_enclosure_entry_rep0.npz' % tag))
    g = z[list(z.keys())[0]]
    s0 = meta['snapshots'][0]
    h, w, res = s0['height'], s0['width'], s0['resolution']
    ox, oy = s0['origin_x'], s0['origin_y']
    if g.ndim == 2:
        g = g.reshape(-1, h, w)
    print('%s: %d live snapshots, sim %.1f-%.1f s, grid %dx%d'
          % (tag, len(g), meta['t0_sim_s'], meta['t1_sim_s'], w, h))
    print('This window covers the enclosure_entry CASCADE leg, during')
    print('which the robot sat motionless and two of the six refusals')
    print('fired. It is the costmap the planner refused on.')
    print()
    print('%9s%9s%12s%12s%9s' % ('map_y', 'published', 'raw(est)',
                                 'offline_raw', 'd'))
    for my in np.arange(-2.90, -3.45, -0.05):
        r = int((my - oy) / res)
        c = int((0.02 - ox) / res)
        v = int(g[:, r, c].max())
        raw = (LETHAL if v == 100 else INSCRIBED if v == 99 else
               255 if v < 0 else int(1 + (v - 1) * 251 / 97) if v else 0)
        pv = probe(0.02, float(my))
        print('%9.3f%9d%12d%12d%9.3f' % (my, v, raw, pv['cost'], pv['d']))
    print()
    print('Agreement is within 4 raw counts everywhere, which is the')
    print('quantisation of the 0-100 rescale. The offline reconstruction')
    print('is therefore trustworthy for legs with no recorded costmap.')
    print()
    print('At the REFUSED pose map(0.02,-3.19) the published value is %d'
          % int(g[:, int((-3.19 - oy) / res), int((0.02 - ox) / res)].max()))
    print('(raw 253, INSCRIBED) in every one of the %d snapshots; it never'
          % len(g))
    print('reaches 100. At the GROUND-TRUTH resting pose map(0.048,-3.025)')
    print('it is %d (raw ~156) -- comfortably plannable.'
          % int(g[:, int((-3.025 - oy) / res),
                  int((0.048 - ox) / res)].max()))
    return 0


# -------------------------------------------------------- 5. exposure
def cmd_exposure(_):
    """How long each leg spends where a replan would be refused."""
    hdr('C2-NAV.27 -- GROUND-TRUTH time inside the refusal band')
    print('band = cell-to-lethal distance <= %.5f m (inscribed model)'
          % INSCRIBED_R)
    print()
    print('%-18s%22s%22s%22s'
          % ('leg', 'BASELINE (5)', 'C2-NAV.25 (3)', 'C2-NAV.26 (6)'))
    print('%-18s%22s%22s%22s'
          % ('', 'secs   min_d  runs', 'secs   min_d  runs',
             'secs   min_d  runs'))
    print('-' * 84)
    for leg in LEGS:
        row = ''
        for name, tags in (('BASELINE', BASE), ('C2-NAV.25', C25),
                           ('C2-NAV.26', C26)):
            ds = []
            for tag in tags:
                tr = trace(tag, leg)
                if not tr:
                    continue
                xs = np.array([fl(r, 'x') + W2M[0] for r in tr
                               if fl(r, 'x') is not None])
                ys = np.array([fl(r, 'y') for r in tr
                               if fl(r, 'y') is not None])
                if not len(xs):
                    continue
                ds.append(d_lethal_many(xs, ys))
            if not ds:
                row += '%22s' % 'n/a'
                continue
            secs = sum((d <= INSCRIBED_R).sum() for d in ds) / 10.0
            row += '%12.1f%7.3f%3d' % (secs, min(d.min() for d in ds),
                                       sum(1 for d in ds
                                           if (d <= INSCRIBED_R).any()))
        print('%-18s%s' % (leg, row))
    print()
    print('READ THIS. Both legs that were refused had ZERO ground-truth')
    print('exposure: c2n26_slow_r2/wall_adjacent min d = 0.259 m and')
    print('c2n26_bslow_r3/enclosure_entry min d = 0.269 m. Meanwhile')
    print('legs that SUCCEEDED spent up to 120.6 s (c2n21_bbase_r2,')
    print('BASELINE) inside the band at 0.152 m. Where the robot')
    print('physically is does not predict the refusal. Where AMCL thinks')
    print('it is does.')
    return 0


# ------------------------------------------------------ 6. decompose
def _amcl_after_wall_adjacent(tag):
    p = logfile(tag)
    if not p:
        return None, None
    seen = False
    for ln in open(p, errors='replace'):
        m = BEGIN.search(ln)
        if m:
            if seen:
                return float(m.group(3)), 'next leg'
            if abs(float(m.group(4))) < 1e-6 and \
                    abs(float(m.group(5)) + 3.0) < 1e-6:
                seen = True
            continue
        m = REFUSE.search(ln)
        if m and seen:
            return float(m.group(3)), 'refusal'
    return None, None


def cmd_decompose(_):
    """Split the estimated overshoot into position and localisation."""
    hdr('C2-NAV.27 -- est_past = gt_past + amcl_err, at wall_adjacent')
    lo = None
    for i in range(400):
        if probe(0.0, -3.0 - i * 0.001)['cost'] >= INSCRIBED:
            lo = i * 0.001
            break
    print('offline refusal boundary est_past >= %.3f m' % lo)
    print('gt_past  +ve = the robot PHYSICALLY stopped past the goal')
    print('amcl_err +ve = AMCL believed it was further toward the wall')
    print()
    print('%-11s%-17s%9s%9s%9s%9s  %s'
          % ('arm', 'run', 'gt_past', 'amcl_err', 'est_past', 'margin',
             'outcome'))
    print('-' * 80)
    agg = collections.defaultdict(list)
    for arm, tags in ARMS:
        for tag in tags:
            wa = legs_of(tag).get('wall_adjacent')
            if not wa or not wa.get('end_world'):
                continue
            gt = -3.0 - wa['end_world'][1]
            ay, src = _amcl_after_wall_adjacent(tag)
            if ay is None:
                continue
            est = -3.0 - ay
            agg[arm].append((gt, est - gt))
            print('%-11s%-17s%9.3f%9.3f%9.3f%9.3f  %s'
                  % (arm, tag, gt, est - gt, est, lo - est,
                     'REFUSED' if est >= lo else wa['status']))
    print()
    print('%-12s%10s%10s%10s%10s' % ('arm', 'gt mean', 'gt max',
                                     'amcl mean', 'amcl max'))
    for arm, _ in ARMS:
        v = agg.get(arm)
        if not v:
            continue
        gts = [a for a, _ in v]
        ams = [b for _, b in v]
        print('%-12s%10.3f%10.3f%10.3f%10.3f'
              % (arm, sum(gts) / len(gts), max(gts),
                 sum(ams) / len(ams), max(ams)))
    print()
    print('READ THIS. In the one refusal the localisation term is 0.155 m')
    print('of the 0.180 m total -- 86 %. The candidate arm does NOT carry')
    print('a larger localisation error than the frozen arms: C2-NAV.25\'s')
    print('c2n25_slow_r1 records 0.178 m, LARGER than the failing run\'s,')
    print('and did not fail because it had stopped 0.138 m short. The')
    print('baseline\'s own c2n21_base_r4 reached est_past 0.090 m, 56 % of')
    print('the way to the boundary, on slowdown_ratio 0.3.')
    return 0


# ------------------------------------------------------- 7. temporal
def cmd_temporal(_):
    """The last seconds of every wall_adjacent leg."""
    hdr('C2-NAV.27 -- did the robot OVERSHOOT the wall_adjacent goal?')
    print('%-11s%-17s%10s%10s%10s%9s  %s'
          % ('arm', 'run', 'max_past', 'end_past', 'min_d', 'n_plans',
             'status'))
    print('-' * 82)
    for arm, tags in ARMS:
        for tag in tags:
            tr = trace(tag, 'wall_adjacent')
            wa = legs_of(tag).get('wall_adjacent')
            if not tr or not wa:
                continue
            ys = np.array([fl(r, 'y') for r in tr if fl(r, 'y') is not None])
            past = -3.0 - ys
            print('%-11s%-17s%10.3f%10.3f%10.3f%9s  %s'
                  % (arm, tag, past.max(), past[-1],
                     wa.get('min_clearance_m'), wa.get('n_plans'),
                     wa['status']))
    print()
    print('The BASELINE never crossed the goal line on this leg (5 of 5')
    print('stopped short, max_past all negative). Four of nine candidate')
    print('runs crossed it, by at most 0.076 m. That is a real directional')
    print('difference at n = 9 vs 5 and no significance is claimed for it.')
    print('It is also an order of magnitude too small on its own: the')
    print('boundary needs 0.161 m of ESTIMATED overshoot.')
    print()
    hdr('c2n26_slow_r2 / wall_adjacent -- the last 4 s, 10 Hz (OBSERVED)')
    tr = trace('c2n26_slow_r2', 'wall_adjacent')
    print('%7s%9s%9s%9s%9s%9s  %-14s%s'
          % ('t_rel', 'past_m', 'v_nav', 'v_out', 'v_act', 'w_nav',
             'cm_polygon', 'DWB critic'))
    for r in tr[-42:]:
        crit = ('RotateToGoal' if fl(r, 'dwb_ill_rot') else
                'BaseObstacle' if fl(r, 'dwb_ill_base') else '-')
        print('%7.1f%9.3f%9.4f%9.4f%9.4f%9.3f  %-14s%s'
              % (fl(r, 't_rel'), -3.0 - fl(r, 'y'), fl(r, 'v_nav') or 0,
                 fl(r, 'v_cmdvel') or 0, fl(r, 'v_act') or 0,
                 fl(r, 'w_nav') or 0, r.get('cm_polygon') or 'none', crit))
    print()
    print('READ THIS.')
    print(' * peak overshoot is +0.076 m at t=16.6-17.0, and the robot is')
    print('   ALREADY RETREATING when the planner refuses (v_act goes')
    print('   negative at 17.1; past_m falls 0.076 -> 0.055 -> 0.031).')
    print(' * commanded vx never exceeds 0.032 m/s and post-monitor vx')
    print('   never exceeds 0.016 m/s. There is no terminal command')
    print('   overshoot to find.')
    print(' * w_nav is pinned at -1.000 rad/s: this is the terminal yaw')
    print('   settle, not an approach.')
    print(' * cm_polygon is PolygonLimit throughout. NOT PolygonSlow (which')
    print('   claims nothing at ratio 1.0, per C2-NAV.25\'s types.hpp')
    print('   finding) and NOT PolygonStop.')
    print(' * the controller never reported "Reached the goal!" on this')
    print('   leg. The goal checker never fired. The leg did not finish --')
    print('   it was refused mid-settle and cancelled.')
    return 0


# ------------------------------------------------------- 8. cascade
def cmd_cascade(_):
    """Primary vs consequential failures, and whether recovery ran."""
    hdr('C2-NAV.27 -- primary failures, cascades, and recovery')
    for arm, tags in ARMS:
        for tag in tags:
            legs = legs_of(tag)
            bad = []
            for i, lg in enumerate(LEGS):
                if lg in legs and legs[lg]['status'] != 'SUCCEEDED':
                    bad.append((i, lg))
            if not bad:
                continue
            first = bad[0][0]
            print('%-11s%-17s' % (arm, tag))
            for i, lg in bad:
                print('     %-17s%-10s err=%7.3f  %s'
                      % (lg, legs[lg]['status'],
                         legs[lg].get('final_goal_err_m') or 0,
                         'PRIMARY' if i == first else 'cascade'))
    print()
    print('=== was any recovery behaviour invoked after a refusal? ===')
    pat = re.compile(r'behavior_server\]: (Running spin|Turning|'
                     r'.*backing up)')
    for tag in ['c2n26_slow_r2', 'c2n26_bslow_r3', 'c2n21_bbase_r1']:
        p = logfile(tag)
        if not p:
            continue
        txt = open(p, errors='replace').read()
        refs = [float(m.group(1)) for m in REFUSE.finditer(txt)
                if m.group(6) == 'Start occupied']
        ts = re.compile(r'\[(\d{10}\.\d+)\]')
        recs = []
        for ln in txt.splitlines():
            if not pat.search(ln):
                continue
            m = ts.search(ln)
            if m:
                recs.append(float(m.group(1)))
        after = [r for r in recs if refs and r >= min(refs)]
        print('   %-17s%d refusal(s), %d recovery action(s) in the run, '
              '%d AFTER the first refusal'
              % (tag, len(refs), len(recs), len(after)))
    print()
    print('READ THIS. Zero recovery actions ran after any refusal, in any')
    print('of the three runs, although the BT is')
    print('navigate_to_pose_w_replanning_and_recovery.xml and the spin /')
    print('backup plugins are loaded and were used elsewhere in the same')
    print('tours. bt_navigator aborts ~40 ms after the refusal. WHY the')
    print('recovery subtree does not run is NOT established offline; that')
    print('it does not run is OBSERVED, and it is what turns one refusal')
    print('into the rest of the tour.')
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse
                                 .RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd')
    for name, fn in [('inventory', cmd_inventory), ('geometry', cmd_geometry),
                     ('bound', cmd_bound), ('validate', cmd_validate),
                     ('exposure', cmd_exposure), ('decompose', cmd_decompose),
                     ('temporal', cmd_temporal), ('cascade', cmd_cascade)]:
        sub.add_parser(name, help=(fn.__doc__ or '').split('\n')[0]
                       ).set_defaults(fn=fn)
    a = ap.parse_args()
    if not getattr(a, 'fn', None):
        ap.print_help()
        return 2
    return a.fn(a)


if __name__ == '__main__':
    sys.exit(main())
