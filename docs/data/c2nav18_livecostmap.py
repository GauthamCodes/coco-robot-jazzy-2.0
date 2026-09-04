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
"""C2-NAV.18: capture and diff the REAL /global_costmap/costmap at the
route-selection divergence.

C2-NAV.17 rejected the near-tie hypothesis: an offline reconstruction of
the global costmap from canonical world geometry, run from both real
captured GOOD/BAD states, selects the SAME safe NW-pinch route every
time -- never BAD's real 230.6 mm PolygonStop intrusion, anywhere in a
+/-750 mm neighbourhood of either state. That session's own INFERRED
conclusion: since a static reconstruction is insensitive to pose
everywhere tested, the real split most likely traces to a difference in
the two runs' actual LIVE global-costmap CONTENT at the tick (transient
obstacle_layer/voxel_layer marks, incomplete propagation right after a
leg-transition/costmap-clearing event, or similar) -- not to pose.

This module is pure offline analysis of a NEW artifact `nav_bench.py`
(C2-NAV.18) now writes: `<tag>_costmapwindow_enclosure_entry_rep0.npz` +
`_meta.json`, the REAL runtime `/global_costmap/costmap` (not a
reconstruction) across the whole leg, one entry per message actually
published (5.0 Hz nominal per c2nav11_ntp_params.yaml, measured not
assumed -- see `measure_cadence`). No Nav2 parameter, BT file, goal,
waypoint, or RemovePassedGoals setting was touched to capture it.

Reuses BY IMPORT, not restates:
  - c2nav15_planwindow.py (as `pw`): self_test, snapshots, first_bad_plan,
    classify, load_planwindow, bench_json, build_clearance_grid, BOXES,
    NW_CORNER, BOX1_X0/X1/Y0/Y1, POLY_STOP_R.
  - c2nav16_compare.py (as `cc`): stop_csv_path/load_stop_csv (unused
    directly here but kept importable for a future DWB cross-check).
  - c2nav13_heading.py: load_trace, RATE_PERIOD, waypoint_timeline,
    divergence_timing, BOX1, rect.
  - c2nav12_report.py: WAYPOINT, GOAL_SHIFTED, SW_CORNER, DEADLOCK_POSE.
  - c2nav14_heading_pose.py: HEADING_POSE.
  - c2nav8_report.py: WORLD_TO_MAP, nearest_full.

Two NEW data sources per tag, both local scratch, `.navbench/` never
tracked in this repo:
  1. `.navbench/results/{tag}_costmapwindow_enclosure_entry_rep0.npz`
     -- stacked int8 grids (n, height, width) + per-message sim timestamp.
  2. `.navbench/results/{tag}_costmapwindow_enclosure_entry_rep0_meta.json`
     -- per-message frame_id/resolution/width/height/origin, checked for
     drift, not assumed constant (brief section 13).
Every number this script derives is written to the committed
`docs/data/c2nav18_bench.json` so the finding survives even though the
raw artifacts might not.

Usage:
  python3 c2nav18_livecostmap.py selftest
  python3 c2nav18_livecostmap.py meta            # GOOD/BAD costmap metadata + cadence
  python3 c2nav18_livecostmap.py alignment       # T_PRUNE / FIRST_BAD_PLAN / costmap ts alignment
  python3 c2nav18_livecostmap.py diff            # cell diff at the matched tick pair
  python3 c2nav18_livecostmap.py routecost       # the brief's own central 2x2 question
  python3 c2nav18_livecostmap.py onset           # temporal-ordering test (brief section 14)
  python3 c2nav18_livecostmap.py dump <out.json>
  python3 c2nav18_livecostmap.py viz
  python3 c2nav18_livecostmap.py all
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import c2nav15_planwindow as pw                                # noqa: E402
import c2nav16_compare as cc                                    # noqa: E402
from c2nav8_report import WORLD_TO_MAP, nearest_full            # noqa: E402
from c2nav9_corridor import POLY_STOP_R, rect                   # noqa: E402
from c2nav12_report import (                                    # noqa: E402
    WAYPOINT, GOAL_SHIFTED, SW_CORNER, DEADLOCK_POSE,
)
from c2nav13_heading import (                                   # noqa: E402
    BOX1, RATE_PERIOD, load_trace, waypoint_timeline, divergence_timing,
)
from c2nav14_heading_pose import HEADING_POSE                   # noqa: E402

REPO_ROOT = pw.REPO_ROOT
RESULTS_DIR = pw.RESULTS_DIR
LEG = 'enclosure_entry'
NW_CORNER = pw.NW_CORNER
BOX1_X0, BOX1_X1, BOX1_Y0, BOX1_Y1 = rect(BOX1)

# ---------------------------------------------------------------------
# Runs. Filled in once C2-NAV.18's own live captures exist; the rest of
# this module works for ANY (GOOD, BAD) tag pair, exactly like
# c2nav16_compare.py's own GOOD/BAD constants.
# ---------------------------------------------------------------------
GOOD = 'c2n18_tour_r1'
BAD = 'c2n18_tour_bad'    # NOT CAPTURED this session -- see GOOD_TAGS below
# This session ran the brief's own 3-tour cap (section 6) and got 3/3
# SUCCEEDED -- 0 BAD reproductions. All three live costmap captures are
# real and valid; GOOD_TAGS lets replicate_noise_floor()/
# visualize_replicates() use all three even though the primary
# GOOD-vs-BAD functions above have no BAD to compare against yet.
GOOD_TAGS = ['c2n18_tour_r1', 'c2n18_tour_r2', 'c2n18_tour_r3']

# The WAYPOINT RemovePassedGoals tick this whole chain has measured at
# this exact configuration (C2-NAV.16, reproduced independently below
# per-tag rather than assumed -- see alignment()).
EXPECT_TICK_S = 9.009


def hdr(t):
    print()
    print('=' * 78)
    print(t)
    print('=' * 78)


# ---------------------------------------------------------------------
# 0. SELF-TEST -- reproduce known committed facts before trusting
#    anything new (brief section 21).
# ---------------------------------------------------------------------

def self_test():
    hdr('SELF-TEST: reproduce known committed facts before trusting '
        'anything new')
    ok = pw.self_test()

    # C2-NAV.17's own static-reconstruction rejection numbers, quoted
    # verbatim -- this session's premise depends on that verdict holding.
    print(f'  C2-NAV.17 static route-cost gap: GOOD_START +17.12%, '
          f'BAD_START +27.60% (both REJECTED the near-tie hypothesis) '
          f'-- premise of this session, not re-derived here')

    d = math.dist(DEADLOCK_POSE, SW_CORNER)
    print(f'  DEADLOCK_POSE {DEADLOCK_POSE} vs SW_CORNER {SW_CORNER}: '
          f'{d*1000:.1f} mm  want a few hundred mm (same pocket)  '
          f'{"PASS" if d < 0.5 else "FAIL"}')
    ok &= d < 0.5

    print()
    print('SELF-TEST: ALL PASS' if ok else 'SELF-TEST: FAILURE -- DO NOT '
          'TRUST ANYTHING BELOW')
    return ok


# ---------------------------------------------------------------------
# 1. LOAD + VALIDATE the costmap-window capture (brief section 17:
#    invalid-run handling).
# ---------------------------------------------------------------------

def costmap_window_paths(tag, leg=LEG, rep=0):
    base = os.path.join(RESULTS_DIR, f'{tag}_costmapwindow_{leg}_rep{rep}')
    return base + '.npz', base + '_meta.json'


def load_costmap_window(tag, leg=LEG, rep=0):
    """Returns a dict with `valid` set False and a `reason` if this
    capture cannot be trusted -- never silently proceeds on a broken
    capture (brief section 17)."""
    npz_path, meta_path = costmap_window_paths(tag, leg, rep)
    if not (os.path.exists(npz_path) and os.path.exists(meta_path)):
        return dict(valid=False, reason=f'missing capture files for {tag} '
                    f'({npz_path} / {meta_path})')
    with open(meta_path) as f:
        meta = json.load(f)
    npz = np.load(npz_path)
    data = npz['data']
    ts = npz['ts_sim_s']
    snaps = meta['snapshots']
    n = len(snaps)
    if n == 0:
        return dict(valid=False, reason=f'{tag}: zero costmap snapshots '
                    'captured')
    if data.shape[0] != n or len(ts) != n:
        return dict(valid=False, reason=f'{tag}: array/meta length '
                    f'mismatch (data={data.shape[0]} ts={len(ts)} '
                    f'meta_snapshots={n})')
    res0, w0, h0 = snaps[0]['resolution'], snaps[0]['width'], snaps[0]['height']
    ox0, oy0, oyaw0 = (snaps[0]['origin_x'], snaps[0]['origin_y'],
                       snaps[0]['origin_yaw'])
    drift = [i for i, s in enumerate(snaps)
             if s['resolution'] != res0 or s['width'] != w0
             or s['height'] != h0 or abs(s['origin_x'] - ox0) > 1e-6
             or abs(s['origin_y'] - oy0) > 1e-6
             or abs(s['origin_yaw']) > 1e-6]
    if abs(oyaw0) > 1e-6:
        return dict(valid=False, reason=f'{tag}: costmap origin_yaw='
                    f'{oyaw0} rad, not axis-aligned -- world<->pixel '
                    'mapping below assumes yaw=0')
    intervals = [round(ts[i + 1] - ts[i], 4) for i in range(len(ts) - 1)]
    return dict(valid=True, tag=tag, meta=meta, data=data, ts_sim_s=ts,
                snapshots=snaps, resolution=res0, width=w0, height=h0,
                origin_x=ox0, origin_y=oy0, origin_yaw=oyaw0,
                geometry_drift_indices=drift,
                n_shape_mismatches=meta.get('n_shape_mismatches', 0),
                t0_sim_s=meta['t0_sim_s'], t1_sim_s=meta['t1_sim_s'],
                intervals_s=intervals,
                mean_interval_s=(round(sum(intervals) / len(intervals), 4)
                                 if intervals else None),
                max_interval_s=(round(max(intervals), 4)
                                if intervals else None))


def report_meta():
    hdr('COSTMAP-WINDOW METADATA + measured cadence (brief section 5: '
        'do NOT assume the update frequency)')
    result = {}
    for tag in (GOOD, BAD):
        lo = load_costmap_window(tag)
        if not lo['valid']:
            print(f'  {tag}: INVALID -- {lo["reason"]}')
            result[tag] = lo
            continue
        print(f'  {tag}: n={len(lo["snapshots"])} snapshots, '
              f't0={lo["t0_sim_s"]}s t1={lo["t1_sim_s"]}s, '
              f'grid {lo["width"]}x{lo["height"]} @ {lo["resolution"]}m, '
              f'origin=({lo["origin_x"]:.3f},{lo["origin_y"]:.3f})')
        print(f'    measured publish interval: mean={lo["mean_interval_s"]}s '
              f'max={lo["max_interval_s"]}s '
              f'(nominal 1/5.0={1/5.0:.3f}s per params)')
        print(f'    geometry drift across the leg: '
              f'{len(lo["geometry_drift_indices"])} of '
              f'{len(lo["snapshots"])} snapshots')
        print(f'    shape mismatches discarded at capture time: '
              f'{lo["n_shape_mismatches"]}')
        result[tag] = {k: v for k, v in lo.items()
                       if k not in ('data',)}
    return result


# ---------------------------------------------------------------------
# 2. TIMESTAMP ALIGNMENT (brief section 5, 9, 13): T_PRUNE,
#    FIRST_BAD_PLAN, and the nearest costmap snapshot to each, per tag --
#    reusing pw/cc/c2nav13_heading machinery, not reimplementing it.
# ---------------------------------------------------------------------

def nearest_costmap_idx(lo, target_offset_s):
    """Nearest costmap snapshot (by ts_offset_from_t0_s) to a leg-relative
    target time. Returns (index, |dt| seconds)."""
    best_i, best_dt = None, None
    for i, s in enumerate(lo['snapshots']):
        dt = abs(s['ts_offset_from_t0_s'] - target_offset_s)
        if best_dt is None or dt < best_dt:
            best_dt, best_i = dt, i
    return best_i, best_dt


def alignment(tag):
    """T_PRUNE (WAYPOINT removal tick, from this run's OWN GT trace, not
    assumed identical to C2-NAV.16's), FIRST_BAD_PLAN (from this run's
    OWN /plan capture), and which costmap snapshot lands closest to
    each -- with the time gap reported, per brief section 5's own
    'do not call a costmap "at the tick" unless the timestamp alignment
    supports it.'"""
    lo = load_costmap_window(tag)
    wp_tl = waypoint_timeline([tag])
    div = divergence_timing([tag])
    fb = pw.first_bad_plan(tag)
    out = dict(tag=tag, costmap_valid=lo['valid'])
    wp = wp_tl.get(tag, {})
    out['t_prune_s'] = wp.get('removal_tick_s')
    out['t_prune_dist_at_removal_m'] = wp.get('dist_at_removal_m')
    dv = div.get(tag, {})
    out['sw_commit_t_s'] = dv.get('sw_commit_t_s')
    if fb and fb['found']:
        out['first_bad_plan_t_s'] = fb['snapshot']['ts_offset_from_t0_s']
        out['first_bad_plan_reasons'] = fb['reasons']
    else:
        out['first_bad_plan_t_s'] = None
        out['first_bad_plan_reasons'] = []
    if lo['valid']:
        for key, target in (('t_prune', out['t_prune_s']),
                             ('first_bad_plan', out['first_bad_plan_t_s'])):
            if target is None:
                out[f'costmap_nearest_{key}'] = None
                continue
            idx, dt = nearest_costmap_idx(lo, target)
            out[f'costmap_nearest_{key}'] = dict(
                idx=idx, ts_offset_s=lo['snapshots'][idx]['ts_offset_from_t0_s'],
                dt_from_target_s=round(dt, 4))
    return out


def report_alignment():
    hdr('TIMESTAMP ALIGNMENT: T_PRUNE / FIRST_BAD_PLAN / nearest costmap '
        'snapshot -- per tag, from THIS run\'s own traces')
    result = {}
    for tag in (GOOD, BAD):
        a = alignment(tag)
        print(f'  {tag}:')
        print(f'    T_PRUNE (WAYPOINT removal tick)   : {a["t_prune_s"]} s')
        print(f'    FIRST_BAD_PLAN                    : '
              f'{a["first_bad_plan_t_s"]} s  {a["first_bad_plan_reasons"]}')
        print(f'    GT SW-column commit                : {a["sw_commit_t_s"]} s')
        cn = a.get('costmap_nearest_t_prune')
        if cn:
            print(f'    nearest costmap snapshot to T_PRUNE: idx={cn["idx"]} '
                  f'ts_offset={cn["ts_offset_s"]}s  '
                  f'|dt|={cn["dt_from_target_s"]}s')
        cn2 = a.get('costmap_nearest_first_bad_plan')
        if cn2:
            print(f'    nearest costmap snapshot to FIRST_BAD_PLAN: '
                  f'idx={cn2["idx"]} ts_offset={cn2["ts_offset_s"]}s  '
                  f'|dt|={cn2["dt_from_target_s"]}s')
        result[tag] = a
    return result


# ---------------------------------------------------------------------
# 3. WORLD <-> PIXEL, cost sampling, and grid diff.
# ---------------------------------------------------------------------

def world_to_pixel(lo, wx, wy):
    mx = wx + WORLD_TO_MAP[0]
    my = wy + WORLD_TO_MAP[1]
    gx = int(math.floor((mx - lo['origin_x']) / lo['resolution']))
    gy = int(math.floor((my - lo['origin_y']) / lo['resolution']))
    return gx, gy


def sample_cost(lo, idx, wx, wy):
    gx, gy = world_to_pixel(lo, wx, wy)
    if 0 <= gx < lo['width'] and 0 <= gy < lo['height']:
        return int(lo['data'][idx, gy, gx])
    return None


def route_cost(lo, idx, path_pts_world):
    """Samples the REAL live costmap `lo` (at snapshot `idx`) along a
    REAL captured /plan polyline. Brief section 10's own metric list."""
    n_unknown = n_oob = 0
    costs = []
    for (x, y) in path_pts_world:
        c = sample_cost(lo, idx, x, y)
        if c is None:
            n_oob += 1
            continue
        if c < 0:
            n_unknown += 1
            continue
        costs.append(c)
    out = dict(n_points=len(path_pts_world), n_valid=len(costs),
               n_unknown=n_unknown, n_oob=n_oob)
    if not costs:
        out.update(sum=None, min=None, max=None, mean=None,
                    n_free=0, n_inflated=0, n_lethal=0)
        return out
    out['sum'] = int(sum(costs))
    out['min'] = int(min(costs))
    out['max'] = int(max(costs))
    out['mean'] = round(sum(costs) / len(costs), 3)
    out['n_free'] = sum(1 for c in costs if c == 0)
    out['n_lethal'] = sum(1 for c in costs if c >= 99)
    out['n_inflated'] = sum(1 for c in costs if 0 < c < 99)
    return out


def geometry_matches(lo_a, lo_b):
    return (lo_a['resolution'] == lo_b['resolution'] and
            lo_a['width'] == lo_b['width'] and
            lo_a['height'] == lo_b['height'] and
            abs(lo_a['origin_x'] - lo_b['origin_x']) < 1e-6 and
            abs(lo_a['origin_y'] - lo_b['origin_y']) < 1e-6)


def diff_grids(lo_a, idx_a, lo_b, idx_b):
    if not geometry_matches(lo_a, lo_b):
        return dict(valid=False, reason='grid geometry differs between runs '
                    f'({lo_a["width"]}x{lo_a["height"]}@{lo_a["resolution"]} '
                    f'vs {lo_b["width"]}x{lo_b["height"]}@{lo_b["resolution"]})')
    A = lo_a['data'][idx_a].astype(np.int16)
    B = lo_b['data'][idx_b].astype(np.int16)
    diff = B - A
    free_to_inflated = int(np.count_nonzero((A == 0) & (B > 0)))
    inflated_to_free = int(np.count_nonzero((A > 0) & (B == 0)))
    return dict(valid=True, n_cells=int(A.size),
                n_differing_cells=int(np.count_nonzero(diff)),
                max_abs_diff=int(np.max(np.abs(diff))) if diff.size else 0,
                mean_abs_diff=(round(float(np.mean(np.abs(diff))), 6)
                               if diff.size else 0.0),
                free_to_inflated_cells=free_to_inflated,
                inflated_to_free_cells=inflated_to_free)


def region_mask(lo, center_world, radius_m):
    h, w = lo['height'], lo['width']
    ys, xs = np.mgrid[0:h, 0:w]
    map_x = lo['origin_x'] + (xs + 0.5) * lo['resolution']
    map_y = lo['origin_y'] + (ys + 0.5) * lo['resolution']
    wx = map_x - WORLD_TO_MAP[0]
    wy = map_y - WORLD_TO_MAP[1]
    return (wx - center_world[0]) ** 2 + (wy - center_world[1]) ** 2 <= radius_m ** 2


def region_diff(lo_a, idx_a, lo_b, idx_b, center_world, radius_m):
    gd = diff_grids(lo_a, idx_a, lo_b, idx_b)
    if not gd['valid']:
        return gd
    mask = region_mask(lo_a, center_world, radius_m)
    A = lo_a['data'][idx_a].astype(np.int16)[mask]
    B = lo_b['data'][idx_b].astype(np.int16)[mask]
    diff = B - A
    return dict(valid=True, center=list(center_world), radius_m=radius_m,
                n_cells_in_region=int(mask.sum()),
                n_differing_cells=int(np.count_nonzero(diff)),
                max_abs_diff=int(np.max(np.abs(diff))) if diff.size else 0,
                mean_abs_diff=(round(float(np.mean(np.abs(diff))), 6)
                               if diff.size else 0.0),
                free_to_inflated=int(np.count_nonzero((A == 0) & (B > 0))),
                inflated_to_free=int(np.count_nonzero((A > 0) & (B == 0))))


REGIONS = {
    'sw_corner': (SW_CORNER, 0.6),
    'nw_pinch': (NW_CORNER, 0.6),
    'goal_corridor': (GOAL_SHIFTED, 0.6),
    'waypoint': (WAYPOINT, 0.6),
}


def report_diff():
    hdr('CELL DIFF at the matched T_PRUNE-nearest costmap snapshot pair, '
        'whole-grid and per-region')
    lo_good = load_costmap_window(GOOD)
    lo_bad = load_costmap_window(BAD)
    if not (lo_good['valid'] and lo_bad['valid']):
        print(f'  INVALID: good={lo_good.get("reason")} '
              f'bad={lo_bad.get("reason")}')
        return dict(valid=False)
    a_good = alignment(GOOD)
    a_bad = alignment(BAD)
    ig = a_good['costmap_nearest_t_prune']['idx']
    ib = a_bad['costmap_nearest_t_prune']['idx']
    print(f'  GOOD snapshot idx={ig} (dt from T_PRUNE '
          f'{a_good["costmap_nearest_t_prune"]["dt_from_target_s"]}s)')
    print(f'  BAD  snapshot idx={ib} (dt from T_PRUNE '
          f'{a_bad["costmap_nearest_t_prune"]["dt_from_target_s"]}s)')
    whole = diff_grids(lo_good, ig, lo_bad, ib)
    print(f'  WHOLE GRID: {whole}')
    regions = {}
    for name, (center, radius) in REGIONS.items():
        rd = region_diff(lo_good, ig, lo_bad, ib, center, radius)
        print(f'  region {name:<14} centre={center} r={radius}m: '
              f'n_cells={rd.get("n_cells_in_region")} '
              f'n_diff={rd.get("n_differing_cells")} '
              f'max_abs={rd.get("max_abs_diff")} '
              f'free->infl={rd.get("free_to_inflated")} '
              f'infl->free={rd.get("inflated_to_free")}')
        regions[name] = rd
    return dict(valid=True, good_idx=ig, bad_idx=ib, whole_grid=whole,
                regions=regions)


# ---------------------------------------------------------------------
# 4. ROUTE COST (brief section 10 -- the MOST IMPORTANT question).
# ---------------------------------------------------------------------

def route_cost_matrix():
    """GOOD's real plan polyline and BAD's real plan polyline (the two
    ACTUAL captured route classes), each sampled against BOTH runs' REAL
    live costmap at the tick-nearest snapshot. Brief section 10's own
    2x2: does the live map reverse or materially reduce the cost
    ordering C2-NAV.17's static reconstruction found?"""
    hdr('ROUTE COST MATRIX: real GOOD/BAD /plan polylines x real GOOD/BAD '
        'live costmaps, at the tick-nearest snapshot')
    lo_good = load_costmap_window(GOOD)
    lo_bad = load_costmap_window(BAD)
    if not (lo_good['valid'] and lo_bad['valid']):
        print('  INVALID costmap capture(s), cannot price routes')
        return dict(valid=False)
    a_good = alignment(GOOD)
    a_bad = alignment(BAD)
    ig = a_good['costmap_nearest_t_prune']['idx']
    ib = a_bad['costmap_nearest_t_prune']['idx']

    # The real captured /plan polyline immediately AT/AFTER the tick in
    # each run -- the same snapshot pw.first_divergence-style analysis
    # already identifies as where the two routes' content splits.
    _pw_good, good_snap = pw.snapshots(GOOD, quiet=True)
    _pw_bad, bad_snap = pw.snapshots(BAD, quiet=True)

    def tick_plan(analyzed):
        return min(analyzed, key=lambda a: abs(
            a['ts_offset_from_t0_s'] - EXPECT_TICK_S))

    good_plan = tick_plan(good_snap)
    bad_plan = tick_plan(bad_snap)
    good_pw_raw = _pw_good['snapshots'][good_snap.index(good_plan)]
    bad_pw_raw = _pw_bad['snapshots'][bad_snap.index(bad_plan)]
    good_pts = [tuple(p) for p in good_pw_raw['poses_world']]
    bad_pts = [tuple(p) for p in bad_pw_raw['poses_world']]

    print(f'  GOOD route: t_offset={good_plan["ts_offset_from_t0_s"]}s, '
          f'{len(good_pts)} poses, SW_col={good_plan["plan_enters_sw_column"]}')
    print(f'  BAD  route: t_offset={bad_plan["ts_offset_from_t0_s"]}s, '
          f'{len(bad_pts)} poses, SW_col={bad_plan["plan_enters_sw_column"]}')

    matrix = {}
    for map_name, lo, idx in (('GOOD_map', lo_good, ig), ('BAD_map', lo_bad, ib)):
        for route_name, pts in (('safe_route(GOOD)', good_pts),
                                ('sw_route(BAD)', bad_pts)):
            rc = route_cost(lo, idx, pts)
            key = f'{route_name}_under_{map_name}'
            matrix[key] = rc
            print(f'  {key:<32}: sum={rc["sum"]} mean={rc["mean"]} '
                  f'max={rc["max"]} n_inflated={rc["n_inflated"]} '
                  f'n_free={rc["n_free"]} n_lethal={rc["n_lethal"]} '
                  f'n_unknown={rc["n_unknown"]} n_oob={rc["n_oob"]}')

    def ordering(map_name):
        safe = matrix[f'safe_route(GOOD)_under_{map_name}']
        sw = matrix[f'sw_route(BAD)_under_{map_name}']
        if safe['mean'] is None or sw['mean'] is None:
            return None
        return dict(safe_mean=safe['mean'], sw_mean=sw['mean'],
                    sw_cheaper=sw['mean'] < safe['mean'],
                    gap_pct=(round(100.0 * (safe['mean'] - sw['mean']) /
                                   max(safe['mean'], 1e-6), 2)))

    good_map_order = ordering('GOOD_map')
    bad_map_order = ordering('BAD_map')
    print()
    print(f'  Under GOOD\'s own live map: {good_map_order}')
    print(f'  Under BAD\'s own live map : {bad_map_order}')
    reversal = (good_map_order and bad_map_order and
                good_map_order['sw_cheaper'] != bad_map_order['sw_cheaper'])
    print()
    print(f'  ORDERING REVERSES between GOOD-map and BAD-map: {reversal}')
    return dict(valid=True, good_tick_snapshot_idx=ig, bad_tick_snapshot_idx=ib,
                good_plan_t_offset_s=good_plan['ts_offset_from_t0_s'],
                bad_plan_t_offset_s=bad_plan['ts_offset_from_t0_s'],
                matrix=matrix, good_map_order=good_map_order,
                bad_map_order=bad_map_order, ordering_reverses=reversal)


# ---------------------------------------------------------------------
# 5. TEMPORAL-ORDERING TEST (brief section 14).
# ---------------------------------------------------------------------

def onset_test(region_name='sw_corner', significance_cells=None):
    """Walk BAD's own costmap snapshots in leg-relative time; at each,
    diff against GOOD's nearest-in-time snapshot restricted to `region`.
    `significance_cells` is measured from the FIRST matched pair (both
    runs still on the identical tour prefix, long before any route can
    differ) as the sensor/raytrace noise floor, not assumed -- brief
    section 14 forbids treating any diff as meaningful without evidence
    it exceeds ordinary frame-to-frame jitter."""
    hdr(f'TEMPORAL-ORDERING TEST: region={region_name} -- does the '
        'costmap diff precede or follow FIRST_BAD_PLAN?')
    lo_good = load_costmap_window(GOOD)
    lo_bad = load_costmap_window(BAD)
    if not (lo_good['valid'] and lo_bad['valid']):
        print('  INVALID costmap capture(s)')
        return dict(valid=False)
    center, radius = REGIONS[region_name]

    series = []
    for s in lo_bad['snapshots']:
        t = s['ts_offset_from_t0_s']
        gi, gdt = nearest_costmap_idx(lo_good, t)
        bi = lo_bad['snapshots'].index(s)
        rd = region_diff(lo_good, gi, lo_bad, bi, center, radius)
        if rd['valid']:
            series.append(dict(t_offset_s=t, good_dt_s=round(gdt, 3),
                               n_differing_cells=rd['n_differing_cells'],
                               max_abs_diff=rd['max_abs_diff']))

    if not series:
        print('  no matched pairs')
        return dict(valid=False)

    noise_floor = significance_cells
    if noise_floor is None:
        early = [r for r in series if r['t_offset_s'] < 3.0]
        noise_floor = max((r['n_differing_cells'] for r in early), default=0)
    threshold = max(noise_floor * 3, noise_floor + 3)
    print(f'  measured early-window (t<3s) noise floor: {noise_floor} '
          f'differing cells; significance threshold: {threshold}')

    onset = next((r for r in series if r['n_differing_cells'] > threshold),
                 None)
    print(f'  {"t_offset":>9} {"n_diff":>7} {"max_abs":>8} {"good_dt":>8}')
    for r in series:
        flag = '  <-- ONSET' if onset is r else ''
        print(f'  {r["t_offset_s"]:>9.3f} {r["n_differing_cells"]:>7} '
              f'{r["max_abs_diff"]:>8} {r["good_dt_s"]:>8.3f}{flag}')

    a_bad = alignment(BAD)
    fb_t = a_bad['first_bad_plan_t_s']
    t_prune = a_bad['t_prune_s']
    onset_t = onset['t_offset_s'] if onset else None
    onset_str = (str(onset_t) if onset_t is not None else
                'NEVER (region never exceeded the measured noise floor)')
    print()
    print(f'  T_PRUNE               : {t_prune} s')
    print(f'  FIRST_BAD_PLAN (BAD)  : {fb_t} s')
    print(f'  costmap diff onset    : {onset_str} s')
    if onset_t is not None and fb_t is not None:
        if onset_t < fb_t - 1e-6:
            order = ('A: costmap difference precedes the divergent plan '
                     f'by {fb_t - onset_t:.3f}s')
        elif onset_t > fb_t + 1e-6:
            order = ('B: the divergent plan precedes the costmap '
                     f'difference by {onset_t - fb_t:.3f}s')
        else:
            order = 'C: same matched instant, cannot be temporally separated'
    else:
        order = 'UNRESOLVED: onset or FIRST_BAD_PLAN not established'
    print(f'  TEMPORAL ORDER: {order}')
    return dict(valid=True, region=region_name, noise_floor=noise_floor,
                threshold=threshold, series=series, onset=onset,
                t_prune_s=t_prune, first_bad_plan_t_s=fb_t,
                onset_t_s=onset_t, temporal_order=order)


# ---------------------------------------------------------------------
# 5b. GOOD-vs-GOOD REPLICATE NOISE FLOOR. Built because this session's own
#     3 fresh tours (brief section 6's own cap) all SUCCEEDED -- 0/3 BAD
#     reproductions -- so the GOOD-vs-BAD diff/route-cost/onset functions
#     above have nothing to compare against yet. Rather than discard the
#     3 real live-costmap captures already made, this measures what
#     costmap DIFFERENCE looks like between two runs that are NOT
#     divergent in outcome (both SUCCEEDED) -- the honest noise floor any
#     future GOOD-vs-BAD diff must clear to be called diagnostic, not
#     ordinary dynamic-layer variation. Brief section 12 anticipates
#     exactly this category ("obstacle_layer/voxel_layer... observation
#     timing... clearing/raytracing").
# ---------------------------------------------------------------------

def replicate_noise_floor(tags):
    """Pairwise whole-grid and per-region diff between every pair of
    `tags`, each at ITS OWN T_PRUNE-nearest costmap snapshot (T_PRUNE
    itself measured per-tag, not assumed shared -- see alignment())."""
    hdr(f'GOOD-vs-GOOD REPLICATE NOISE FLOOR across {tags}, each at its '
        'own T_PRUNE-nearest snapshot')
    los = {t: load_costmap_window(t) for t in tags}
    aligns = {t: alignment(t) for t in tags}
    for t in tags:
        lo = los[t]
        if not lo['valid']:
            print(f'  {t}: INVALID -- {lo["reason"]}')
            continue
        a = aligns[t]
        print(f'  {t}: T_PRUNE={a["t_prune_s"]}s  '
              f'SW_commit={a["sw_commit_t_s"]}  '
              f'first_bad_plan={a["first_bad_plan_t_s"]}  '
              f'{a["first_bad_plan_reasons"]}')
    pairs = []
    import itertools
    for ta, tb in itertools.combinations(tags, 2):
        loa, lob = los[ta], los[tb]
        if not (loa['valid'] and lob['valid']):
            continue
        ia = aligns[ta]['costmap_nearest_t_prune']['idx']
        ib = aligns[tb]['costmap_nearest_t_prune']['idx']
        whole = diff_grids(loa, ia, lob, ib)
        regions = {name: region_diff(loa, ia, lob, ib, c, r)
                  for name, (c, r) in REGIONS.items()}
        print(f'\n  {ta} (idx={ia}) vs {tb} (idx={ib}):')
        print(f'    whole grid: n_diff={whole.get("n_differing_cells")} '
              f'/ {whole.get("n_cells")}  max_abs={whole.get("max_abs_diff")} '
              f'mean_abs={whole.get("mean_abs_diff")}')
        for name, rd in regions.items():
            print(f'    region {name:<14}: n_diff={rd.get("n_differing_cells")}'
                  f'/{rd.get("n_cells_in_region")}  '
                  f'max_abs={rd.get("max_abs_diff")}  '
                  f'free->infl={rd.get("free_to_inflated")}  '
                  f'infl->free={rd.get("inflated_to_free")}')
        pairs.append(dict(a=ta, b=tb, a_idx=ia, b_idx=ib, whole_grid=whole,
                          regions=regions))
    return dict(tags=tags, alignments={t: aligns[t] for t in tags},
                pairs=pairs)


def visualize_replicates(tags, out_path=None):
    """One panel per tag: its own live costmap at its own T_PRUNE-nearest
    snapshot, with its own tick-nearest /plan overlaid -- the visual
    record of this session's actual finding (all 3 runs' plans/GT tracks
    entered the SW column/PolygonStop region and all 3 still SUCCEEDED)."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    out_path = out_path or os.path.join(
        REPO_ROOT, 'docs', 'images', 'c2nav18_replicates.png')

    fig, axes = plt.subplots(1, len(tags), figsize=(7 * len(tags), 7), dpi=150)
    if len(tags) == 1:
        axes = [axes]
    for ax, tag in zip(axes, tags):
        lo = load_costmap_window(tag)
        if not lo['valid']:
            ax.text(0.5, 0.5, f'{tag}\nINVALID', ha='center', va='center')
            continue
        a = alignment(tag)
        idx = a['costmap_nearest_t_prune']['idx']
        x0 = lo['origin_x'] - WORLD_TO_MAP[0]
        y0 = lo['origin_y'] - WORLD_TO_MAP[1]
        x1 = x0 + lo['width'] * lo['resolution']
        y1 = y0 + lo['height'] * lo['resolution']
        ax.imshow(lo['data'][idx], origin='lower', extent=(x0, x1, y0, y1),
                  cmap='viridis', vmin=-1, vmax=100)
        _pw, snap = pw.snapshots(tag, quiet=True)
        if snap:
            tick_snap = min(snap, key=lambda s: abs(
                s['ts_offset_from_t0_s'] - a['t_prune_s']))
            raw = _pw['snapshots'][snap.index(tick_snap)]
            pts = raw['poses_world']
            if pts:
                ax.plot([p[0] for p in pts], [p[1] for p in pts],
                        color='red', linewidth=2.0, label='tick-nearest /plan')
        rows = load_trace(tag, LEG) or []
        if rows:
            ax.plot([r['x'] for r in rows], [r['y'] for r in rows],
                    color='white', linewidth=1.3, alpha=0.8, label='GT track')
        for (name, pt) in (('SW', SW_CORNER), ('NW', NW_CORNER)):
            ax.plot(*pt, marker='*', color='yellow', markersize=12,
                    markeredgecolor='black')
        ax.set_xlim(-4.3, -2.3)
        ax.set_ylim(-0.6, 3.3)
        ax.set_title(f'{tag}\nT_PRUNE={a["t_prune_s"]:.2f}s  '
                     f'SW_commit={a["sw_commit_t_s"]}\nSUCCEEDED, '
                     'grazed SW column and recovered')
        ax.legend(fontsize=7, loc='lower left')
    fig.suptitle('C2-NAV.18: 3/3 fresh GOOD tours, live costmap @ own '
                 'T_PRUNE-nearest snapshot -- all grazed the SW column, '
                 'none deadlocked (no BAD reproduced this session)',
                 fontsize=11)
    fig.savefig(out_path, bbox_inches='tight')
    print(f'wrote {out_path}')


# ---------------------------------------------------------------------
# 6. DUMP + summary + viz
# ---------------------------------------------------------------------

def summary():
    hdr(f'C2-NAV.18 SUMMARY: GOOD ({GOOD}) vs BAD ({BAD})')
    meta = report_meta()
    align = report_alignment()
    diff = report_diff()
    routecost = route_cost_matrix()
    onset = onset_test('sw_corner')
    noise = replicate_noise_floor(GOOD_TAGS)
    return dict(meta=meta, alignment=align, diff=diff,
                route_cost=routecost, onset=onset,
                replicate_noise_floor=noise)


def dump(out_path):
    record = dict(good_tag=GOOD, bad_tag=BAD, good_tags=GOOD_TAGS, leg=LEG,
                  self_test_pass=self_test())
    record.update(summary())
    with open(out_path, 'w') as f:
        json.dump(record, f, indent=1, default=str)
    print(f'\nwrote {out_path}')
    return record


def visualize(out_path=None):
    """GOOD live costmap + GOOD plan, BAD live costmap + BAD plan, and a
    diff panel, all at the T_PRUNE-nearest snapshot -- brief section 15's
    own required visualization."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    out_path = out_path or os.path.join(
        REPO_ROOT, 'docs', 'images', 'c2nav18_livecostmap.png')

    lo_good = load_costmap_window(GOOD)
    lo_bad = load_costmap_window(BAD)
    if not (lo_good['valid'] and lo_bad['valid']):
        print(f'INVALID capture(s): good={lo_good.get("reason")} '
              f'bad={lo_bad.get("reason")}')
        return

    a_good = alignment(GOOD)
    a_bad = alignment(BAD)
    ig = a_good['costmap_nearest_t_prune']['idx']
    ib = a_bad['costmap_nearest_t_prune']['idx']

    def extent_world(lo):
        x0 = lo['origin_x'] - WORLD_TO_MAP[0]
        y0 = lo['origin_y'] - WORLD_TO_MAP[1]
        x1 = x0 + lo['width'] * lo['resolution']
        y1 = y0 + lo['height'] * lo['resolution']
        return (x0, x1, y0, y1)

    fig, axes = plt.subplots(1, 3, figsize=(20, 7), dpi=150)
    for ax, lo, idx, tag, label in (
            (axes[0], lo_good, ig, GOOD, 'GOOD'),
            (axes[1], lo_bad, ib, BAD, 'BAD')):
        ext = extent_world(lo)
        ax.imshow(lo['data'][idx], origin='lower', extent=ext,
                  cmap='viridis', vmin=-1, vmax=100)
        _pw, snap = pw.snapshots(tag, quiet=True)
        tick_snap = min(snap, key=lambda a: abs(
            a['ts_offset_from_t0_s'] - EXPECT_TICK_S))
        raw = _pw['snapshots'][snap.index(tick_snap)]
        pts = raw['poses_world']
        if pts:
            ax.plot([p[0] for p in pts], [p[1] for p in pts], color='red',
                    linewidth=2.0, label='tick-nearest /plan')
        for (name, pt) in (('SW', SW_CORNER), ('NW', NW_CORNER)):
            ax.plot(*pt, marker='*', color='yellow', markersize=12,
                    markeredgecolor='black')
        ax.set_xlim(-4.3, -2.3)
        ax.set_ylim(-0.6, 3.3)
        ax.set_title(f'{label} ({tag})\nlive costmap @ '
                     f'idx={idx}, ts_offset='
                     f'{lo["snapshots"][idx]["ts_offset_from_t0_s"]:.2f}s')
        ax.legend(fontsize=7, loc='lower left')

    if geometry_matches(lo_good, lo_bad):
        diff = (lo_bad['data'][ib].astype(np.int16) -
                lo_good['data'][ig].astype(np.int16))
        ext = extent_world(lo_good)
        im = axes[2].imshow(diff, origin='lower', extent=ext, cmap='RdBu_r',
                            vmin=-100, vmax=100)
        for (name, pt) in (('SW', SW_CORNER), ('NW', NW_CORNER)):
            axes[2].plot(*pt, marker='*', color='yellow', markersize=12,
                         markeredgecolor='black')
        axes[2].set_xlim(-4.3, -2.3)
        axes[2].set_ylim(-0.6, 3.3)
        axes[2].set_title('BAD - GOOD cost diff')
        fig.colorbar(im, ax=axes[2], fraction=0.046)
    else:
        axes[2].text(0.5, 0.5, 'geometry mismatch,\nno diff panel',
                     ha='center', va='center')

    fig.suptitle('C2-NAV.18: live /global_costmap/costmap, GOOD vs BAD, '
                 'at the WAYPOINT-removal-tick-nearest snapshot', fontsize=11)
    fig.savefig(out_path, bbox_inches='tight')
    print(f'wrote {out_path}')


def all_(argv):
    ok = self_test()
    summary()
    return 0 if ok else 1


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'all'
    if cmd == 'selftest':
        return 0 if self_test() else 1
    if cmd == 'meta':
        report_meta()
        return 0
    if cmd == 'alignment':
        report_alignment()
        return 0
    if cmd == 'diff':
        report_diff()
        return 0
    if cmd == 'routecost':
        route_cost_matrix()
        return 0
    if cmd == 'onset':
        onset_test(sys.argv[2] if len(sys.argv) > 2 else 'sw_corner')
        return 0
    if cmd == 'dump':
        out = sys.argv[2] if len(sys.argv) > 2 else \
            os.path.join(HERE, 'c2nav18_bench.json')
        dump(out)
        return 0
    if cmd == 'replicates':
        replicate_noise_floor(GOOD_TAGS)
        return 0
    if cmd == 'viz':
        visualize()
        return 0
    if cmd == 'vizreplicates':
        visualize_replicates(GOOD_TAGS)
        return 0
    if cmd == 'all':
        return all_(sys.argv[2:])
    print(f'unknown command: {cmd}')
    return 2


if __name__ == '__main__':
    sys.exit(main())
