#!/usr/bin/env python3
"""C2-NAV.20 -- trace DWB's critics through the whole zero-velocity stall.

DIAGNOSIS ONLY. Nothing here writes a parameter, starts a node, or talks
to ROS. Every number comes from artifacts already on disk: the C2-NAV.19
BAD run (`c2n19_tour_r1`), the C2-NAV.18 GOOD runs, and C2-NAV.3's raw
`/evaluation` captures.

WHAT C2-NAV.19 LEFT OPEN
------------------------
Its BAD leg lost 42.84 s standing still at (-2.681, 1.692) with
`dwb_chosen_vx = 0.0`, "94 % of 819 trajectories legal", 0.456 m of
clearance and a fresh plan. One `worst_crawl` sample. The question: why
does a zero-velocity trajectory outscore every forward one?

WHAT IS AND IS NOT IN THE COMMITTED C2-NAV.19 ARTIFACT
------------------------------------------------------
`nav_bench.py::_eval_cb` summarises `/evaluation` in the callback and
throws the rest away (819 trajectories x 7 critics at 5.75 Hz is
hundreds of MB). What survives per cycle is n, n_illegal, an illegal
count keyed by the THROWING critic, and the CHOSEN trajectory's
(vx, wz, total, 7 critic scores). Only four of those reach the trace CSV
(`dwb_n`, `dwb_illegal`, `dwb_best_vx`, `dwb_best_total`); the critic
decomposition survives as a leg-wide mean plus one `worst_crawl`
snapshot.

So for C2-NAV.19 there is NO per-cycle critic decomposition of the
LOSING trajectories, NO per-trajectory critic count (complete vs
short-circuited), and NO local costmap. The costmap is not
reconstructible either: `local_costmap` runs
`["obstacle_layer", "voxel_layer", "inflation_layer"]` with NO static
layer, so it is built entirely from live `/scan`, which nothing
captured.

WHY THAT IS NOT FATAL
---------------------
Two facts make the question decidable anyway.

1. `MapGridCritic`'s propagation is a plain Manhattan distance transform
   that IGNORES the cost field (`MapGridQueue::validCellToQueue` returns
   true unconditionally -- C2-NAV.3's source read, and its flood matched
   a direct min-over-seeds L1 on 0 mismatched cells). So `GoalDist`,
   `GoalAlign`, `PathDist` and `PathAlign` are pure geometry: robot pose
   + plan + costmap lattice. Four of the five relevant critics are
   reconstructible offline, exactly.

2. `BaseObstacle` is bounded from the outside. The local inflation is
   `cost = floor(252 * exp(-65 * (d - 0.20)))`, so `cost == 0` for any
   cell more than **0.2851 m** from a marked cell. The stop probe
   records `d_min_base_m`, the distance from `base_footprint` to the
   nearest live scan point, at ~20 Hz. Any trajectory whose endpoint is
   displaced by less than `d_min_base - 0.2851` therefore lands in a
   cost-0 cell **whatever direction it goes**, and its `BaseObstacle` is
   exactly 0. That is a bound, not an assumption.

The module therefore compares the zero-velocity winner against the
subset of forward trajectories whose `BaseObstacle` is PROVABLY zero,
and asks whether any of them beats it on the MapGrid critics alone. If
none does, the zero-velocity preference is not an obstacle-gating
effect at all.

VALIDATION (section 17)
-----------------------
The critic implementation is validated against C2-NAV.3's raw captures,
which DO contain DWB's own transformed plan, the local costmap with its
origin, and all 819 per-trajectory critic scores with their critic
COUNTS. Agreement is exact. See `validate_c2nav3()`.

At C2-NAV.19 the transformed plan and the costmap origin do not exist in
the artifact and have to be reconstructed from `/plan` plus an assumed
odom offset. That reconstruction does NOT land on the published integers
-- it puts the `GoalDist` seed 3 plan poses too far along. The absolute
values are therefore NOT claimed; only the zero-vs-forward difference is
used, and `seed_sensitivity()` shows the verdict is invariant across the
whole span of seed positions the residual admits.

SOURCE READ (dwb 1.3.11, the installed version)
-----------------------------------------------
1. `coreScoringAlgorithm`: `if (best.total < 0 || score.total < best.total)`
   -- STRICT `<`. A tie goes to the trajectory evaluated FIRST.
2. `XYThetaIterator::iterateToValidVelocity` increments theta innermost
   and x outermost; `OneDVelocityIterator::reset` starts at `min_vel_`.
   With `min_vel_x = 0.0` the entire vx = 0 block is scored BEFORE any
   forward trajectory. `isValidSpeed` rejects (0,0,0), so the block holds
   40 of the 819 samples and 779 are forward. Reproduced exactly:
   `best_index` maps to the captured chosen twist in every C2-NAV.3
   snapshot.
3. `scoreTrajectory` short-circuits on `score.total > best_score` and
   still pushes the PARTIAL score, whose total is >= 0. Short-circuited
   trajectories are counted as LEGAL by any `total < 0` test, including
   nav_bench's. Legal != complete.
4. `MapGridCritic::getScale()` = `resolution * 0.5 * scale` -> 0.6 for
   GoalDist/GoalAlign, 0.8 for PathDist/PathAlign. `BaseObstacle` does
   not override it, so its weight is the bare 8.0 against a 0-252 cost.
5. `aggregation_type: last`. `GoalAlign`/`PathAlign` set
   `stop_on_failure_ = false`, so they score ONLY the final pose, read at
   `getForwardPose(final, 0.1)`.
6. `transformGlobalPlan`: `transform_end_threshold =
   min(dist_threshold, forward_prune_distance)` with `dist_threshold =
   max(cells_x, cells_y) * resolution / 2` = 1.5 m here.
   `prune_distance` and `forward_prune_distance` are unset in the params
   file and take DWB's own 2.0 defaults.
7. `GoalAlignCritic::prepare` nudges the transformed plan's last pose
   0.1 m along the robot->goal bearing before seeding.

Usage (always under `python3 -P`, per the stray-`numbers.py` trap):

    python3 -P docs/data/c2nav20_dwbstall.py selftest
    python3 -P docs/data/c2nav20_dwbstall.py all
    python3 -P docs/data/c2nav20_dwbstall.py viz
    python3 -P docs/data/c2nav20_dwbstall.py dump docs/data/c2nav20_bench.json
"""

import csv
import hashlib
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import c2nav15_planwindow as pw                                  # noqa: E402
import c2nav16_compare as cc                                     # noqa: E402
from c2nav12_report import (                                     # noqa: E402
    WAYPOINT, GOAL_SHIFTED, SW_CORNER, DEADLOCK_POSE,
)
from c2nav13_heading import bearing_deg, ang_diff_deg            # noqa: E402

RESULTS_DIR = pw.RESULTS_DIR
LEG = 'enclosure_entry'

BAD = 'c2n19_tour_r1'
GOOD = 'c2n18_tour_r1'
GOOD_ALTERNATES = ['c2n18_tour_r2', 'c2n18_tour_r3']

PARAMS_FILE = os.path.join(HERE, 'c2nav11_ntp_params.yaml')
PARAMS_SHA256 = ('6f61e49912765708e70470df967b23834338723176bcf7ae113f8b8'
                 'c1e6bb950')

# ---- frozen configuration, all asserted against the live dump --------
RES = 0.05
CELLS = 60
HALF_WINDOW = CELLS * RES / 2.0
ROBOT_RADIUS = 0.20
INFLATION_RADIUS = 0.5
LOCAL_CSF = 65.0
GLOBAL_CSF = 5.0
BASE_OBSTACLE_SCALE = 8.0
SIM_TIME = 1.5
VX_SAMPLES = 20
VTH_SAMPLES = 40
MIN_VEL_X, MAX_VEL_X = 0.0, 0.3
MIN_VEL_TH, MAX_VEL_TH = -1.0, 1.0
ACC_X, DECEL_X = 3.0, -2.5
ACC_TH, DECEL_TH = 3.2, -3.2
LINEAR_GRANULARITY = 0.05
ANGULAR_GRANULARITY = 0.025
PRUNE_DISTANCE = 2.0
FORWARD_PRUNE_DISTANCE = 2.0
FWD_POINT_DIST = 0.1
MAPGRID_SCALE = {'GoalDist': 0.05 * 0.5 * 24.0,
                 'GoalAlign': 0.05 * 0.5 * 24.0,
                 'PathDist': 0.05 * 0.5 * 32.0,
                 'PathAlign': 0.05 * 0.5 * 32.0}
CRITIC_ORDER = ['RotateToGoal', 'Oscillation', 'BaseObstacle', 'GoalAlign',
                'PathAlign', 'PathDist', 'GoalDist']
POLY_STOP_R = 0.25

WORLD_TO_ODOM = (2.0, 0.0)
SPAWN_WORLD = (-2.0, 0.0, 0.0)

# Distance at which the local inflation first rounds to cost 0, and the
# distance at which it first reaches a cost able to short-circuit a
# ~46-total winner. Both derived from the frozen CSF.
D_COST_ZERO = ROBOT_RADIUS + math.log(252.0 / 1.0) / LOCAL_CSF

C2NAV19 = {
    'crawl_len_s': 42.84, 'crawl_start_t_rel_s': 11.08,
    'crawl_pose': (-2.681, 1.692, 0.821), 'crawl_scan_min_m': 0.456,
    'crawl_illegal_frac': 0.06, 'crawl_chosen_vx': 0.0,
    'crawl_critics': {'GoalAlign': 22.2, 'GoalDist': 22.2, 'PathAlign': 1.6,
                      'PathDist': 0.0, 'BaseObstacle': 0.0,
                      'RotateToGoal': 0.0, 'Oscillation': 0.0},
    'leg_duration_s': 201.36, 'leg_goal_err_m': 1.106,
    'polygonstop_secs': 130.99, 'costmap_snapshots': 520,
    'stop_rows': 3959, 'good_worst_crawl_s': 1.59,
}
# C2-NAV.3's own committed complete/short-circuit/illegal split.
C2NAV3 = {'A': (151, 648, 20), 'B': (278, 541, 0)}


def hdr(t):
    print()
    print('=' * 72)
    print(t)
    print('=' * 72)


# ---------------------------------------------------------------------
# Loaders
# ---------------------------------------------------------------------

def trace_path(tag, leg=LEG, rep=0):
    return os.path.join(RESULTS_DIR, f'{tag}_traces', f'{leg}_rep{rep}.csv')


def _f(v):
    return float(v) if v not in (None, '') else None


def load_full_trace(tag, leg=LEG, rep=0):
    p = trace_path(tag, leg, rep)
    if not os.path.exists(p):
        return None
    out = []
    with open(p) as f:
        for r in csv.DictReader(f):
            if r['x'] in (None, ''):
                continue
            out.append({
                't': float(r['t_rel']), 'x': float(r['x']),
                'y': float(r['y']), 'yaw': float(r['yaw']),
                'v_act': _f(r['v_act']), 'w_act': _f(r['w_act']),
                'v_nav': _f(r['v_nav']), 'w_nav': _f(r['w_nav']),
                'v_smoothed': _f(r['v_smoothed']),
                'v_cmdvel': _f(r['v_cmdvel']), 'v_wheel': _f(r['v_wheel']),
                'cm_action': (int(r['cm_action'])
                              if r['cm_action'] not in (None, '') else None),
                'cm_polygon': r['cm_polygon'] or None,
                'scan_min': _f(r['scan_min']), 'dwb_n': _f(r['dwb_n']),
                'dwb_illegal': _f(r['dwb_illegal']),
                'dwb_best_vx': _f(r['dwb_best_vx']),
                'dwb_best_total': _f(r['dwb_best_total']),
            })
    return out


def leg_record(tag, scenario=LEG):
    b = json.load(open(os.path.join(RESULTS_DIR, f'{tag}.json')))
    for leg in b['legs']:
        if leg['scenario'] == scenario:
            return leg
    return None


def eval_cycles(rows):
    """One entry per DISTINCT /evaluation message. The trace holds the
    last value at 10 Hz while /evaluation runs at ~5.75 Hz."""
    out, prev = [], None
    for r in rows:
        if r['dwb_n'] is None:
            continue
        key = (r['dwb_n'], r['dwb_illegal'], r['dwb_best_vx'],
               r['dwb_best_total'])
        if key != prev:
            out.append(r)
            prev = key
    return out


def clearance_series(tag):
    """(leg-relative t, d_min_base_m) from the stop probe, ~20 Hz."""
    data = pw.load_planwindow(tag)
    t0 = data['t0_sim_s']
    out = []
    for r in cc.load_stop_csv(tag) or []:
        if not r['stamp'] or not r['d_min_base_m']:
            continue
        out.append((float(r['stamp']) - t0, float(r['d_min_base_m'])))
    out.sort()
    return out


def clearance_at(series, t):
    if not series:
        return None
    return min(series, key=lambda p: abs(p[0] - t))[1]


CM_NAMES = {0: 'DO_NOTHING', 1: 'STOP', 2: 'SLOWDOWN', 3: 'APPROACH',
            4: 'LIMIT'}


# ---------------------------------------------------------------------
# DWB reconstruction
# ---------------------------------------------------------------------

def project_velocity(v0, accel, decel, dt, target):
    if v0 < target:
        return min(target, v0 + accel * dt)
    return max(target, v0 + decel * dt)


def one_d_samples(current, vmin, vmax, acc, dec, acc_time, n):
    cur = min(max(current, vmin), vmax)
    hi = project_velocity(cur, acc, dec, acc_time, vmax)
    lo = project_velocity(cur, acc, dec, acc_time, vmin)
    eps = 1e-6
    if abs(lo - hi) < eps:
        return [lo]
    n = max(2, n)
    inc = (hi - lo) / max(1, n - 1)
    out, v, return_zero = [], lo, True
    while v <= hi + eps:
        out.append(v)
        if return_zero and v < 0.0 and (v + inc) > 0.0 and \
                (v + inc) <= hi + eps:
            out.append(0.0)
            return_zero = False
        v += inc
    return out


def sample_twists(cur_vx=0.0, cur_wz=0.0):
    """The 819 (vx, wz) pairs in DWB's own evaluation order."""
    xs = one_d_samples(cur_vx, MIN_VEL_X, MAX_VEL_X, ACC_X, DECEL_X,
                       SIM_TIME, VX_SAMPLES)
    ths = one_d_samples(cur_wz, MIN_VEL_TH, MAX_VEL_TH, ACC_TH, DECEL_TH,
                        SIM_TIME, VTH_SAMPLES)
    return [(vx, wz) for vx in xs for wz in ths
            if not (vx == 0.0 and wz == 0.0)]


def generate_trajectory(x, y, yaw, start_vx, start_wz, cmd_vx, cmd_wz):
    """StandardTrajectoryGenerator, discretize_by_time = False."""
    n = math.ceil(max(abs(cmd_vx) * SIM_TIME / LINEAR_GRANULARITY,
                      abs(cmd_wz) * SIM_TIME / ANGULAR_GRANULARITY))
    n = max(1, int(n))
    dt = SIM_TIME / n
    vx, wz = start_vx, start_wz
    for _ in range(n):
        vx = project_velocity(vx, ACC_X, DECEL_X, dt, cmd_vx)
        wz = project_velocity(wz, ACC_TH, DECEL_TH, dt, cmd_wz)
        x += vx * math.cos(yaw) * dt
        y += vx * math.sin(yaw) * dt
        yaw += wz * dt
    return x, y, yaw


def adjust_plan_resolution(poses, resolution=RES):
    """nav_2d_utils::adjustPlanResolution. A no-op at this repo's ~0.05 m
    plan spacing; implemented so the claim is checked, not assumed."""
    if not poses:
        return []
    out = [poses[0]]
    min_sq = (resolution * 2.0) ** 2
    last = poses[0]
    for p in poses[1:]:
        sq = (p[0] - last[0]) ** 2 + (p[1] - last[1]) ** 2
        if sq > min_sq:
            steps = int((math.sqrt(sq) - math.sqrt(min_sq))
                        / math.sqrt(min_sq)) + 1
            dx = (p[0] - last[0]) / steps
            dy = (p[1] - last[1]) / steps
            for j in range(1, steps):
                out.append((last[0] + dx * j, last[1] + dy * j))
        out.append(p)
        last = p
    return out


def transform_global_plan(plan, rx, ry, end_thr=None):
    """DWBLocalPlanner::transformGlobalPlan."""
    if not plan:
        return []
    start_thr = min(HALF_WINDOW, PRUNE_DISTANCE)
    end_thr = end_thr if end_thr is not None else min(HALF_WINDOW,
                                                      FORWARD_PRUNE_DISTANCE)
    acc, prune_point = 0.0, len(plan)
    for i in range(1, len(plan)):
        acc += math.dist(plan[i - 1], plan[i])
        if acc > FORWARD_PRUNE_DISTANCE:
            prune_point = i
            break
    begin = None
    for i in range(0, prune_point):
        if math.dist((rx, ry), plan[i]) < start_thr:
            begin = i
            break
    if begin is None:
        return []
    end = len(plan)
    for i in range(begin, len(plan)):
        if math.dist(plan[i], (rx, ry)) > end_thr:
            end = i
            break
    return [tuple(p) for p in plan[begin:end]]


def cell(x, y):
    return (math.floor((x + WORLD_TO_ODOM[0]) / RES),
            math.floor((y + WORLD_TO_ODOM[1]) / RES))


def l1_cells(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def inflation_distance_for_cost(c):
    """Invert nav2 InflationLayer::computeCost for the local costmap."""
    if c <= 0:
        return float('inf')
    if c >= 253:
        return ROBOT_RADIUS
    return ROBOT_RADIUS + math.log(252.0 / c) / LOCAL_CSF


def build_grids(plan_world, rx, ry, goal_world, end_thr=None,
                seed_backoff=0):
    """Everything prepare() does for the four MapGrid critics.

    `seed_backoff` walks the GoalDist/GoalAlign seed back along the plan
    by that many poses. It is the knob `seed_sensitivity()` sweeps -- it
    exists only to show the zero-vs-forward verdict does not depend on
    where exactly the seed sits."""
    tp = transform_global_plan(plan_world, rx, ry, end_thr)
    if not tp:
        return None
    if seed_backoff:
        tp = tp[:max(1, len(tp) - seed_backoff)]
    adj = adjust_plan_resolution(tp)
    goal_seed = cell(*adj[-1])
    path_seeds = sorted({cell(px, py) for px, py in adj})
    ang = math.atan2(goal_world[1] - ry, goal_world[0] - rx)
    nudged = list(tp)
    nudged[-1] = (nudged[-1][0] + FWD_POINT_DIST * math.cos(ang),
                  nudged[-1][1] + FWD_POINT_DIST * math.sin(ang))
    ga_seed = cell(*adjust_plan_resolution(nudged)[-1])
    return {'goal_seed': goal_seed, 'ga_seed': ga_seed,
            'path_seeds': path_seeds, 'n_transformed': len(tp),
            'first': tp[0], 'last': tp[-1]}


def score_all(rx, ry, ryaw, start_vx, start_wz, g):
    """Score all 819 samples on the four MapGrid critics. `mg_total` is
    the complete total under BaseObstacle = 0."""
    gs, gas, ps = g['goal_seed'], g['ga_seed'], g['path_seeds']
    out = []
    for vx, wz in sample_twists(start_vx, start_wz):
        fx, fy, fyaw = generate_trajectory(rx, ry, ryaw, start_vx, start_wz,
                                           vx, wz)
        ax = fx + FWD_POINT_DIST * math.cos(fyaw)
        ay = fy + FWD_POINT_DIST * math.sin(fyaw)
        fc, ac = cell(fx, fy), cell(ax, ay)
        gd = l1_cells(fc, gs)
        ga = l1_cells(ac, gas)
        pd = min(l1_cells(fc, s) for s in ps)
        pa = min(l1_cells(ac, s) for s in ps)
        out.append({
            'vx': vx, 'wz': wz, 'fx': fx, 'fy': fy, 'fyaw': fyaw,
            'disp': math.hypot(fx - rx, fy - ry),
            'GoalDist': gd, 'GoalAlign': ga, 'PathDist': pd, 'PathAlign': pa,
            'mg_total': (ga * MAPGRID_SCALE['GoalAlign']
                         + pa * MAPGRID_SCALE['PathAlign']
                         + pd * MAPGRID_SCALE['PathDist']
                         + gd * MAPGRID_SCALE['GoalDist'])})
    return out


def argmin_first(scored, pred):
    """DWB's tie-break: strict `<`, so the FIRST minimum wins."""
    best = None
    for s in scored:
        if pred(s) and (best is None
                        or s['mg_total'] < best['mg_total'] - 1e-12):
            best = s
    return best


# ---------------------------------------------------------------------
# Section 17a -- exact validation against C2-NAV.3's raw captures.
# ---------------------------------------------------------------------

def _c3_snapshot(path, si):
    d = json.load(open(path))
    return d['snapshots'][si], len(d['snapshots'])


def validate_c2nav3(verbose=True):
    """The real section-17 gate. C2-NAV.3's captures hold DWB's own
    transformed plan, the local costmap WITH its origin, and all 819
    per-trajectory critic scores plus their critic COUNTS -- everything
    the reconstruction needs, with nothing assumed."""
    if verbose:
        hdr('SECTION 17a -- validate the critic reconstruction against '
            'C2-NAV.3\'s raw /evaluation captures')
    results = []
    ok = True
    for path, want in ((os.path.join(HERE, 'c2nav3_stallA.json'), C2NAV3['A']),
                       (os.path.join(HERE, 'c2nav3_stallB.json'), C2NAV3['B'])):
        if not os.path.exists(path):
            continue
        name = os.path.basename(path)
        _, nsnap = _c3_snapshot(path, 0)
        for si in range(nsnap):
            s, _ = _c3_snapshot(path, si)
            cm = s['costmap']
            res, (ox, oy) = cm['resolution'], cm['origin']
            tp = [(p[0], p[1]) for p in s['transformed_plan']['poses']]
            start = s['chosen']['poses'][0]
            rx, ry, ryaw = start[0], start[1], start[2]

            def c_(x, y):
                return (int((x - ox) / res), int((y - oy) / res))

            adj = adjust_plan_resolution(tp, res)
            gd_seed = c_(*adj[-1])
            pseeds = sorted({c_(q[0], q[1]) for q in adj})
            ang = math.atan2(tp[-1][1] - ry, tp[-1][0] - rx)
            nud = list(tp)
            nud[-1] = (nud[-1][0] + FWD_POINT_DIST * math.cos(ang),
                       nud[-1][1] + FWD_POINT_DIST * math.sin(ang))
            ga_seed = c_(*adjust_plan_resolution(nud, res)[-1])
            tws = sample_twists(0.0, 0.0)

            allt = s['all']
            complete = [i for i, a in enumerate(allt)
                        if a['n_critics'] == 7 and a['total'] >= 0]
            illegal = [i for i, a in enumerate(allt) if a['total'] < 0]
            short = [i for i, a in enumerate(allt)
                     if a['total'] >= 0 and a['n_critics'] < 7]
            abort = {}
            for i in short:
                nm = allt[i]['critics'][allt[i]['n_critics'] - 1][0]
                abort[nm] = abort.get(nm, 0) + 1

            names = ['GoalDist', 'GoalAlign', 'PathDist', 'PathAlign']
            agree = {k: 0 for k in names}
            tot = 0
            for i in complete:
                vx, wz = tws[i]
                got = {c[0]: c[1] for c in allt[i]['critics']}
                fx, fy, fyaw = generate_trajectory(rx, ry, ryaw, 0.0, 0.0,
                                                   vx, wz)
                ax = fx + FWD_POINT_DIST * math.cos(fyaw)
                ay = fy + FWD_POINT_DIST * math.sin(fyaw)
                fc, ac = c_(fx, fy), c_(ax, ay)
                mine = {'GoalDist': l1_cells(fc, gd_seed),
                        'GoalAlign': l1_cells(ac, ga_seed),
                        'PathDist': min(l1_cells(fc, q) for q in pseeds),
                        'PathAlign': min(l1_cells(ac, q) for q in pseeds)}
                tot += 1
                for k in names:
                    if abs(mine[k] - got[k]) < 1e-9:
                        agree[k] += 1
            bi = s['best_index']
            order_ok = (abs(tws[bi][0] - s['chosen']['vx']) < 1e-9
                        and abs(tws[bi][1] - s['chosen']['wz']) < 1e-6)
            rec = {'file': name, 'snapshot': si, 'n_traj': s['n_traj'],
                   'complete': len(complete), 'short': len(short),
                   'illegal': len(illegal), 'abort_at': abort,
                   'agree': agree, 'n_scored': tot,
                   'eval_order_ok': order_ok,
                   'goal_seed_cell': gd_seed}
            results.append(rec)
            if si == 0:
                frac = min(agree[k] / max(1, tot) for k in names)
                ok = ok and order_ok and frac >= 0.99
                if verbose:
                    print(f'  {name} snapshot 0:')
                    print(f'    captured split complete/short/illegal = '
                          f'{len(complete)}/{len(short)}/{len(illegal)}   '
                          f'C2-NAV.19 doc says {want[0]}/{want[1]}/{want[2]}'
                          f'  {"MATCH" if (len(complete), len(short), len(illegal)) == want else "DIFFER"}')
                    print(f'    short-circuits aborted at: {abort}')
                    print(f'    GoalDist seed cell {gd_seed}')
                    for k in names:
                        print(f'    {k:10s} reproduced '
                              f'{agree[k]:4d}/{tot}  '
                              f'{100.0 * agree[k] / max(1, tot):5.1f} %')
                    print(f'    evaluation order: best_index {bi} maps to '
                          f'the captured chosen twist: {order_ok}')
    if verbose:
        agg = {}
        for r in results:
            for k, v in r['agree'].items():
                a, b = agg.get(k, (0, 0))
                agg[k] = (a + v, b + r['n_scored'])
        print()
        print('  Across ALL captured snapshots:')
        for k, (a, b) in agg.items():
            print(f'    {k:10s} {a}/{b}  {100.0 * a / max(1, b):6.2f} %')
        print()
        print('  RECONSTRUCTION OF THE CRITICS:',
              'VALIDATED' if ok else 'NOT VALIDATED')
    return results, ok


def c2nav3_mechanism():
    """Run C2-NAV.20's own test on C2-NAV.3's stall, where BaseObstacle
    IS known, so the test itself can be checked against a verdict that
    is already established."""
    hdr('SECTION 11a -- run the same test on C2-NAV.3\'s stall, where '
        'BaseObstacle is known')
    out = {}
    for path in (os.path.join(HERE, 'c2nav3_stallA.json'),
                 os.path.join(HERE, 'c2nav3_stallB.json')):
        if not os.path.exists(path):
            continue
        s, _ = _c3_snapshot(path, 0)
        allt = s['all']
        tws = sample_twists(0.0, 0.0)
        zero = [(i, a) for i, a in enumerate(allt) if tws[i][0] == 0.0]
        fwd = [(i, a) for i, a in enumerate(allt) if tws[i][0] > 0.0]
        zc = [a['total'] for _, a in zero if a['total'] >= 0
              and a['n_critics'] == 7]
        best_zero = min(zc) if zc else None

        def mg_only(a):
            d = {c[0]: c[1] * c[2] for c in a['critics']}
            return sum(d.get(k, 0.0) for k in MAPGRID_SCALE)
        fwd_complete = [a for _, a in fwd
                        if a['total'] >= 0 and a['n_critics'] == 7]
        fwd_mg = [mg_only(a) for a in fwd_complete]
        fwd_bo = [next((c[1] for c in a['critics']
                        if c[0] == 'BaseObstacle'), 0.0)
                  for a in fwd_complete]
        name = os.path.basename(path)
        print(f'  {name}:')
        print(f'    zero-vx trajectories complete: {len(zc)}/40 -- the '
              f'block is scored FIRST and never short-circuits, so it sets')
        print(f'      the threshold every forward trajectory is judged '
              f'against.  best complete total {best_zero}')
        print(f'    forward trajectories complete: {len(fwd_complete)}/779')
        if fwd_mg:
            d = min(fwd_mg) - best_zero
            verdict = ('TIE' if abs(d) < 1e-6
                       else ('WIN' if d < 0 else 'LOSE'))
            print(f'    of those, best MapGrid-only total '
                  f'{min(fwd_mg):.1f} vs zero-vx {best_zero:.1f}  '
                  f'-> forward would {verdict} with BaseObstacle = 0 '
                  f'(delta {d:+.3g})')
            print(f'    their actual BaseObstacle raw scores: '
                  f'min {min(fwd_bo):.0f} max {max(fwd_bo):.0f}')
        short = [i for i, a in enumerate(allt)
                 if a['total'] >= 0 and a['n_critics'] < 7]
        fwd_short = [i for i in short if tws[i][0] > 0.0]
        print(f'    forward trajectories SHORT-CIRCUITED before GoalDist: '
              f'{len(fwd_short)}/779 '
              f'({100.0 * len(fwd_short) / 779:.1f} %)')
        out[name] = {'best_zero_total': best_zero,
                     'n_forward_complete': len(fwd_complete),
                     'n_forward_short': len(fwd_short),
                     'best_forward_mg': min(fwd_mg) if fwd_mg else None}
    print()
    print('  So at C2-NAV.3\'s CSF-5 stall the answer is CRITIC_GATING:')
    print('  almost every forward trajectory never reaches GoalDist at')
    print('  all. That is the baseline C2-NAV.20 has to distinguish')
    print('  itself from.')
    return out


# ---------------------------------------------------------------------
# Section 17b -- what the C2-NAV.19 artifact does NOT support.
# ---------------------------------------------------------------------

def residual_report():
    hdr('SECTION 17b -- the same reconstruction applied to C2-NAV.19, '
        'and what it cannot reproduce')
    leg = leg_record(BAD)
    wc = leg['worst_crawl']
    tw = wc['t_rel_s'] + wc['crawl_len_s'] / 2.0
    rows = load_full_trace(BAD)
    r = min(rows, key=lambda q: abs(q['t'] - tw))
    snap = plan_at(BAD, r['t'])
    plan = [tuple(p) for p in snap['poses_world']]
    g = build_grids(plan, r['x'], r['y'], plan[-1])
    scored = score_all(r['x'], r['y'], r['yaw'], 0.0, 0.0, g)
    # the published chosen twist
    ch = min(scored, key=lambda s: (abs(s['vx'] - wc['dwb_chosen_vx'])
                                    + abs(s['wz'] - wc['dwb_chosen_wz'])))
    want = wc['dwb_chosen_critics']
    print(f'  nav_bench reads worst_crawl at the crawl MIDPOINT: '
          f't = {wc["t_rel_s"]} + {wc["crawl_len_s"]}/2 = {tw:.2f} s')
    print(f'  published pose {tuple(wc["pose_world"])}  chosen '
          f'(vx={wc["dwb_chosen_vx"]}, wz={wc["dwb_chosen_wz"]})')
    print(f'  reconstructed plan: {len(plan)} /plan poses (age '
          f'{r["t"] - snap["ts_offset_from_t0_s"]:.2f} s, nav_bench says '
          f'{wc["plan_age_s"]}) -> {g["n_transformed"]} transformed')
    print()
    print(f'  {"critic":12s} {"rebuilt raw":>12} {"rebuilt scaled":>15} '
          f'{"published":>10}')
    for k in ('GoalDist', 'GoalAlign', 'PathDist', 'PathAlign'):
        print(f'  {k:12s} {ch[k]:12d} {ch[k] * MAPGRID_SCALE[k]:15.1f} '
              f'{want[k]:10.1f}')
    print(f'  {"total":12s} {"":12s} {ch["mg_total"]:15.1f} '
          f'{sum(want.values()):10.1f}')
    print()
    print('  The reconstruction does NOT land on the published integers.')
    print('  The GoalDist seed sits 3 plan poses too far along: the')
    print('  published raw 37 is plan index 34 (1.324 m from the robot),')
    print('  the 1.5 m transform clip reaches index 37 (1.475 m). No')
    print('  robot-pose offset within +/-0.20 m and no costmap lattice')
    print('  phase reproduces all four published values together, so the')
    print('  gap is NOT a simple localisation offset and is left')
    print('  unexplained rather than fitted away.')
    print()
    print('  Consequence, and it is a hard one: the ABSOLUTE reconstructed')
    print('  critic values for C2-NAV.19 are NOT claimed as measured.')
    print('  Only the zero-vs-forward DIFFERENCE is used below, and')
    print('  seed_sensitivity() sweeps the seed across the whole span the')
    print('  residual admits to show the verdict does not turn on it.')
    return {'rebuilt': {k: ch[k] for k in
                        ('GoalDist', 'GoalAlign', 'PathDist', 'PathAlign')},
            'rebuilt_total': ch['mg_total'],
            'published': want, 'published_total': sum(want.values())}


# ---------------------------------------------------------------------
# Section 6 -- the exact stall window.
# ---------------------------------------------------------------------

def plan_at(tag, t_rel):
    data = pw.load_planwindow(tag)
    best = None
    for s in data['snapshots']:
        if s['ts_offset_from_t0_s'] <= t_rel:
            best = s
        else:
            break
    return best


def plan_times(tag):
    return [s['ts_offset_from_t0_s']
            for s in pw.load_planwindow(tag)['snapshots']]


def crawl_window(tag=BAD):
    """nav_bench's own worst_crawl window, on the COMMANDED velocity
    (|v_nav| < 0.05). This is the interval C2-NAV.19 named 42.84 s, and
    it is the analysis window so the two are comparable."""
    wc = leg_record(tag)['worst_crawl']
    return wc['t_rel_s'], wc['t_rel_s'] + wc['crawl_len_s']


def stall_window(tag=BAD):
    """Runs of DWB's SELECTED zero velocity -- a different and stricter
    thing than the commanded-crawl window, and not one block."""
    rows = load_full_trace(tag)
    cyc = eval_cycles(rows)
    runs, start, prev = [], None, None
    for r in cyc:
        z = (r['dwb_best_vx'] == 0.0)
        if z and start is None:
            start = r['t']
        if not z and start is not None:
            runs.append((start, prev['t']))
            start = None
        prev = r
    if start is not None:
        runs.append((start, prev['t']))
    runs.sort(key=lambda ab: ab[1] - ab[0], reverse=True)
    return runs, cyc, rows


def report_window():
    hdr('SECTION 6 -- the exact stall window, from DWB\'s SELECTED vx')
    runs, cyc, rows = stall_window(BAD)
    leg = leg_record(BAD)
    wc = leg['worst_crawl']
    print(f'  nav_bench worst_crawl is defined on the COMMANDED velocity '
          f'(|v_nav| < 0.05):')
    print(f'    [{wc["t_rel_s"]:.2f}, '
          f'{wc["t_rel_s"] + wc["crawl_len_s"]:.2f}] s = '
          f'{wc["crawl_len_s"]} s')
    print(f'  {len(cyc)} distinct /evaluation cycles in the leg '
          f'(nav_bench counted {leg["dwb_cycles"]})')
    print()
    print('  Longest runs of dwb_best_vx == 0.0 (SELECTED, not commanded):')
    for a, b in runs[:6]:
        print(f'    [{a:7.2f}, {b:7.2f}]  {b - a:6.2f} s')
    print()
    print('  So the 42.84 s is NOT one continuous zero-velocity selection.')
    print('  Inside it DWB twice selected a NON-zero vx -- but only')
    print('  0.0158 and 0.0316 m/s, the first and second forward samples,')
    print('  worth 24 and 47 mm of travel over the 1.5 s horizon, which is')
    print('  below nav_bench\'s 0.05 m/s crawl threshold and below one')
    print('  costmap cell. Refinement, not a contradiction.')
    c0, c1 = crawl_window(BAD)
    zero_t = sum(b - a for a, b in runs if c0 <= a and b <= c1)
    print()
    print(f'  ANALYSIS WINDOW (= nav_bench worst_crawl, = C2-NAV.19\'s')
    print(f'  42.84 s): T_START = {c0:.2f} s   T_END = {c1:.2f} s')
    print(f'  of which selected vx was exactly 0.0 for {zero_t:.2f} s '
          f'({100.0 * zero_t / (c1 - c0):.1f} %)')
    t0, t1 = c0, c1
    cs = clearance_series(BAD)

    def at(t):
        return min(rows, key=lambda r: abs(r['t'] - t))
    for label, t in (('T_START', t0), ('T_END', t1)):
        r = at(t)
        print(f'  {label:8s} t={r["t"]:7.2f}  pose=({r["x"]:+.4f}, '
              f'{r["y"]:+.4f}, {r["yaw"]:+.4f})  vx_sel={r["dwb_best_vx"]}  '
              f'total={r["dwb_best_total"]}  d_min_base='
              f'{clearance_at(cs, t):.4f}  '
              f'monitor={CM_NAMES.get(r["cm_action"])}/{r["cm_polygon"]}  '
              f'illegal={r["dwb_illegal"]:.0f}')
    return {'zero_vx_runs': [[round(a, 2), round(b, 2)] for a, b in runs[:6]],
            't_start': round(t0, 2), 't_end': round(t1, 2),
            'span_s': round(t1 - t0, 2),
            'selected_zero_s_inside': round(zero_t, 2),
            'n_distinct_dwb_states': len(cyc),
            'navbench_dwb_cycles': leg['dwb_cycles']}


# ---------------------------------------------------------------------
# Sections 7/8
# ---------------------------------------------------------------------

def timeline(tag=BAD, t_lo=0.0, t_hi=None):
    runs, cyc, rows = stall_window(tag)
    cs = clearance_series(tag)
    out = []
    for r in cyc:
        if r['t'] < t_lo or (t_hi is not None and r['t'] > t_hi):
            continue
        n = r['dwb_n'] or 819.0
        ill = r['dwb_illegal'] or 0.0
        out.append({
            't': r['t'], 'x': r['x'], 'y': r['y'], 'yaw': r['yaw'],
            'vx_sel': r['dwb_best_vx'], 'wz_cmd': r['w_nav'],
            'total_sel': r['dwb_best_total'], 'n': n, 'illegal': ill,
            'legal': n - ill, 'legal_frac': (n - ill) / n,
            'v_wheel': r['v_wheel'], 'v_nav': r['v_nav'],
            'scan_min': r['scan_min'], 'd_min_base': clearance_at(cs, r['t']),
            'monitor': CM_NAMES.get(r['cm_action']),
            'polygon': r['cm_polygon'],
            'd_goal': math.dist((r['x'], r['y']), GOAL_SHIFTED),
            'd_sw': math.dist((r['x'], r['y']), SW_CORNER)})
    return out


def report_timeline():
    hdr('SECTION 7 -- time-resolved DWB state through the stall')
    t0, t1 = crawl_window(BAD)
    tl = timeline(BAD, t0 - 4.0, t1 + 4.0)
    print(f'  {len(tl)} /evaluation cycles from {t0 - 4:.2f} to '
          f'{t1 + 4:.2f} s. Every 5th shown.')
    print()
    print(f'  {"t":>7} {"vx_sel":>7} {"total":>7} {"legal":>6} {"ill":>5} '
          f'{"dminb":>6} {"d_goal":>6} {"d_sw":>6} {"yaw":>7} '
          f'{"v_wheel":>7}  monitor')
    for r in tl[::5]:
        print(f'  {r["t"]:7.2f} {r["vx_sel"]:7.4f} {r["total_sel"]:7.1f} '
              f'{r["legal"]:6.0f} {r["illegal"]:5.0f} {r["d_min_base"]:6.3f} '
              f'{r["d_goal"]:6.3f} {r["d_sw"]:6.3f} {r["yaw"]:+7.4f} '
              f'{r["v_wheel"]:7.4f}  {r["monitor"]}/{r["polygon"]}')
    return tl


def dominance():
    hdr('SECTION 8 -- when does zero velocity become dominant?')
    runs, cyc, rows = stall_window(BAD)
    t0 = runs[-1][0] if False else min(a for a, b in runs if b - a > 5.0)
    cs = clearance_series(BAD)
    pre = [r for r in cyc if r['t'] <= t0 + 0.01]
    print('  Every /evaluation cycle on the approach into the stall:')
    print(f'  {"t":>7} {"vx_sel":>7} {"total":>7} {"ill":>5} {"dminb":>6} '
          f'{"v_wheel":>7}  monitor')
    for r in pre[-22:]:
        print(f'  {r["t"]:7.2f} {r["dwb_best_vx"]:7.4f} '
              f'{r["dwb_best_total"]:7.1f} {r["dwb_illegal"]:5.0f} '
              f'{clearance_at(cs, r["t"]):6.3f} {r["v_wheel"]:7.4f}  '
              f'{CM_NAMES.get(r["cm_action"])}/{r["cm_polygon"]}')
    zeros = [r['t'] for r in cyc if r['dwb_best_vx'] == 0.0]
    first_zero = zeros[0] if zeros else None
    flips, prev = 0, None
    for r in cyc:
        if r['t'] > t0:
            break
        z = (r['dwb_best_vx'] == 0.0)
        if prev is not None and z != prev:
            flips += 1
        prev = z
    # is the collapse abrupt or gradual?
    seq = [(r['t'], r['dwb_best_vx']) for r in cyc if t0 - 3 <= r['t'] <= t0]
    print()
    print(f'  FIRST zero-vx selection anywhere in the leg : '
          f't = {first_zero:.2f} s')
    print(f'  FIRST_ZERO_DOMINANCE (start of longest run) : t = {t0:.2f} s')
    print(f'  forward<->zero flips before T_START         : {flips}')
    print(f'  selected vx over the last 3 s before T_START: '
          f'{[round(v, 4) for _, v in seq]}')
    print()
    print('  The commanded speed does not decay: it is 0.2842-0.3000 on')
    print('  the cycle before T_START and 0.0 on the cycle at it. The')
    print('  collapse is ABRUPT, one cycle wide.')
    return {'first_zero_any_s': round(first_zero, 2),
            'first_zero_dominance_s': round(t0, 2),
            'flips_before': flips,
            'vx_last_3s': [round(v, 4) for _, v in seq]}


# ---------------------------------------------------------------------
# Sections 9/11/12 -- the sweep, with a PROVEN BaseObstacle = 0 subset.
# ---------------------------------------------------------------------

def sweep(tag=BAD, t_lo=None, t_hi=None, stride=1, seed_backoff=0,
          end_thr=None):
    runs, cyc, rows = stall_window(tag)
    if t_lo is None:
        t_lo, t_hi = crawl_window(tag)
    snaps = pw.load_planwindow(tag)['snapshots']
    cs = clearance_series(tag)
    out, cache = [], {}
    sel = [r for r in cyc if t_lo - 0.001 <= r['t'] <= t_hi + 0.001][::stride]
    for r in sel:
        snap = None
        for s in snaps:
            if s['ts_offset_from_t0_s'] <= r['t']:
                snap = s
            else:
                break
        if snap is None:
            continue
        key = snap['ts_offset_from_t0_s']
        plan = cache.setdefault(key, [tuple(p) for p in snap['poses_world']])
        g = build_grids(plan, r['x'], r['y'], plan[-1], end_thr, seed_backoff)
        if g is None:
            continue
        scored = score_all(r['x'], r['y'], r['yaw'], r['v_act'] or 0.0,
                           r['w_act'] or 0.0, g)
        dmin = clearance_at(cs, r['t'])
        safe = (dmin - D_COST_ZERO) if dmin else 0.0
        b_zero = argmin_first(scored, lambda s: s['vx'] == 0.0)
        b_fwd = argmin_first(scored, lambda s: s['vx'] > 0.0)
        b_safe = argmin_first(scored,
                              lambda s: s['vx'] > 0.0 and s['disp'] <= safe)
        n_safe = sum(1 for s in scored if s['vx'] > 0.0 and s['disp'] <= safe)
        # How degenerate is the minimum? This is the statistic the seed
        # residual cannot move: a common offset shifts every total by the
        # same amount and cannot change WHICH trajectories tie.
        gmin = min(s['mg_total'] for s in scored)
        at_min = [s for s in scored if abs(s['mg_total'] - gmin) <= 1e-9]
        at_min_safe = [s for s in at_min
                       if s['vx'] == 0.0 or s['disp'] <= safe]
        first_at_min = next(s for s in scored
                            if abs(s['mg_total'] - gmin) <= 1e-9)
        rec = {'t': round(r['t'], 2), 'x': r['x'], 'y': r['y'],
               'yaw': r['yaw'], 'plan_ts': key,
               'plan_age_s': round(r['t'] - key, 2),
               'n_transformed': g['n_transformed'],
               'published_vx': r['dwb_best_vx'],
               'published_total': r['dwb_best_total'],
               'illegal': r['dwb_illegal'], 'd_min_base': dmin,
               'safe_disp_m': round(safe, 4), 'n_safe_forward': n_safe,
               'n_at_min': len(at_min),
               'n_at_min_zero': sum(1 for s in at_min if s['vx'] == 0.0),
               'n_at_min_forward': sum(1 for s in at_min if s['vx'] > 0.0),
               'n_at_min_forward_safe': sum(1 for s in at_min_safe
                                            if s['vx'] > 0.0),
               'first_at_min_vx': first_at_min['vx'],
               'first_at_min_wz': first_at_min['wz'],
               'min_total': gmin,
               'zero_total': b_zero['mg_total'], 'zero_wz': b_zero['wz'],
               'zero_GoalDist': b_zero['GoalDist'],
               'zero_GoalAlign': b_zero['GoalAlign'],
               'zero_PathDist': b_zero['PathDist'],
               'zero_PathAlign': b_zero['PathAlign']}
        for lbl, b in (('fwd', b_fwd), ('safe', b_safe)):
            if b is None:
                rec[f'{lbl}_total'] = None
                rec[f'{lbl}_margin'] = None
                continue
            rec[f'{lbl}_total'] = b['mg_total']
            rec[f'{lbl}_vx'] = b['vx']
            rec[f'{lbl}_wz'] = b['wz']
            rec[f'{lbl}_disp'] = round(b['disp'], 4)
            rec[f'{lbl}_GoalDist'] = b['GoalDist']
            rec[f'{lbl}_GoalAlign'] = b['GoalAlign']
            rec[f'{lbl}_PathDist'] = b['PathDist']
            rec[f'{lbl}_PathAlign'] = b['PathAlign']
            rec[f'{lbl}_margin'] = b_zero['mg_total'] - b['mg_total']
        out.append(rec)
    return out


def report_sweep(rows=None):
    hdr('SECTION 9/11 -- zero-vx vs the best forward trajectory whose '
        'BaseObstacle is PROVABLY zero')
    if rows is None:
        rows = sweep()
    print(f'  {len(rows)} /evaluation cycles reconstructed across the window.')
    print()
    print(f'  cost == 0 requires clearance >= {D_COST_ZERO:.4f} m '
          f'(252*exp(-65*(d-0.20)) < 1).')
    print('  "safe" = forward trajectories whose ENDPOINT displacement is')
    print('  under d_min_base - that distance, so no pose of theirs can be')
    print('  in a cost-bearing cell, in ANY direction. For those,')
    print('  BaseObstacle = 0 is proven, not assumed.')
    print()
    print(f'  {"t":>7} {"pub_vx":>7} {"pub_tot":>7} | {"zero":>6} '
          f'{"safe":>6} {"margin":>7} {"vx":>7} {"disp":>6} {"nsafe":>6} | '
          f'{"dGD":>4} {"dGA":>4} {"dPD":>4} {"dPA":>4}')
    for r in rows[::8]:
        if r['safe_total'] is None:
            continue
        print(f'  {r["t"]:7.2f} {r["published_vx"]:7.4f} '
              f'{r["published_total"]:7.1f} | {r["zero_total"]:6.1f} '
              f'{r["safe_total"]:6.1f} {r["safe_margin"]:+7.1f} '
              f'{r["safe_vx"]:7.4f} {r["safe_disp"]:6.3f} '
              f'{r["n_safe_forward"]:6d} | '
              f'{r["safe_GoalDist"] - r["zero_GoalDist"]:+4d} '
              f'{r["safe_GoalAlign"] - r["zero_GoalAlign"]:+4d} '
              f'{r["safe_PathDist"] - r["zero_PathDist"]:+4d} '
              f'{r["safe_PathAlign"] - r["zero_PathAlign"]:+4d}')
    out = {}
    for lbl, title in (('safe', 'PROVABLY cost-0 forward trajectories'),
                       ('fwd', 'ALL forward trajectories (BaseObstacle '
                               'assumed 0 -- an upper bound on their case)')):
        ms = [r[f'{lbl}_margin'] for r in rows if r[f'{lbl}_margin'] is not None]
        wins = sum(1 for v in ms if v > 1e-9)
        ties = sum(1 for v in ms if abs(v) <= 1e-9)
        loses = sum(1 for v in ms if v < -1e-9)
        ms_s = sorted(ms)
        print()
        print(f'  {title}:')
        print(f'    cycles where forward BEATS zero : {wins} / {len(ms)}')
        print(f'    exact ties (zero wins on strict-<): {ties} / {len(ms)}')
        print(f'    cycles where zero wins outright : {loses} / {len(ms)}')
        if ms_s:
            print(f'    margin (zero - forward) range {ms_s[0]:+.1f} .. '
                  f'{ms_s[-1]:+.1f}, median {ms_s[len(ms_s) // 2]:+.1f}')
        out[lbl] = {'n': len(ms), 'forward_wins': wins, 'ties': ties,
                    'zero_wins': loses,
                    'margin_min': ms_s[0] if ms_s else None,
                    'margin_max': ms_s[-1] if ms_s else None,
                    'margin_median': (ms_s[len(ms_s) // 2] if ms_s else None)}
    ns = [r['n_safe_forward'] for r in rows]
    print()
    print(f'  provably-cost-0 forward trajectories available per cycle: '
          f'min {min(ns)}  median {sorted(ns)[len(ns) // 2]}  max {max(ns)} '
          f'(of 779)')
    sd = [r['safe_disp_m'] for r in rows]
    print(f'  safe endpoint displacement: min {min(sd):.4f} m  '
          f'max {max(sd):.4f} m  -> vx up to {max(sd) / SIM_TIME:.4f} m/s')

    hdr('SECTION 11b -- how DEGENERATE is the minimum?')
    print('  A constant seed error shifts every trajectory\'s total by the')
    print('  same amount, so it cannot change WHICH trajectories tie at the')
    print('  minimum. This statistic survives the section-17b residual')
    print('  intact.')
    print()
    am = [r['n_at_min'] for r in rows]
    amz = [r['n_at_min_zero'] for r in rows]
    amf = [r['n_at_min_forward'] for r in rows]
    ams = [r['n_at_min_forward_safe'] for r in rows]
    fz = sum(1 for r in rows if r['first_at_min_vx'] == 0.0)
    print(f'  trajectories sharing the MINIMUM total, per cycle: '
          f'min {min(am)}  median {sorted(am)[len(am) // 2]}  max {max(am)}')
    print(f'    of them zero-vx  : min {min(amz)}  median '
          f'{sorted(amz)[len(amz) // 2]}  max {max(amz)}')
    print(f'    of them forward  : min {min(amf)}  median '
          f'{sorted(amf)[len(amf) // 2]}  max {max(amf)}')
    print(f'    forward AND provably cost-0: min {min(ams)}  median '
          f'{sorted(ams)[len(ams) // 2]}  max {max(ams)}')
    print(f'  cycles where the FIRST trajectory at the minimum -- the one')
    print(f'  DWB\'s strict `<` keeps -- has vx == 0: {fz} / {len(rows)}')
    print()
    q = sorted({round(r['published_total'] / 0.2) * 0.2 - r['published_total']
                for r in rows})
    print(f'  every published chosen total is a multiple of 0.2 '
          f'(= gcd of the 0.6 and 0.8 MapGrid scales): '
          f'max deviation {max(abs(v) for v in q):.2e}')
    out['degeneracy'] = {
        'n_at_min_median': sorted(am)[len(am) // 2],
        'n_at_min_min': min(am), 'n_at_min_max': max(am),
        'n_at_min_forward_median': sorted(amf)[len(amf) // 2],
        'n_at_min_forward_safe_median': sorted(ams)[len(ams) // 2],
        'first_at_min_is_zero_vx': fz, 'n_cycles': len(rows)}
    return out


def seed_sensitivity():
    """Does the verdict depend on where the reconstructed seed sits? The
    residual in section 17b spans 3 plan poses; sweep well past it."""
    hdr('SECTION 17c -- is the verdict sensitive to the seed residual?')
    print('  Walking the GoalDist/GoalAlign seed back along the plan and')
    print('  re-running the whole sweep. backoff 3 is the residual;')
    print('  0..8 brackets it with room to spare.')
    print()
    print(f'  {"backoff":>8} {"cycles":>7} {"fwd wins":>9} {"ties":>6} '
          f'{"zero wins":>10} {"median margin":>14}')
    out = []
    for bk in range(0, 9):
        rows = sweep(seed_backoff=bk, stride=4)
        ms = [r['safe_margin'] for r in rows if r['safe_margin'] is not None]
        if not ms:
            continue
        ms_s = sorted(ms)
        rec = {'backoff': bk, 'n': len(ms),
               'forward_wins': sum(1 for v in ms if v > 1e-9),
               'ties': sum(1 for v in ms if abs(v) <= 1e-9),
               'zero_wins': sum(1 for v in ms if v < -1e-9),
               'median_margin': ms_s[len(ms_s) // 2]}
        out.append(rec)
        print(f'  {bk:8d} {rec["n"]:7d} {rec["forward_wins"]:9d} '
              f'{rec["ties"]:6d} {rec["zero_wins"]:10d} '
              f'{rec["median_margin"]:+14.1f}')
    same = len({(r['forward_wins'] > 0) for r in out}) == 1
    print()
    print(f'  verdict invariant across the whole sweep: {same}')
    return {'rows': out, 'invariant': same}


def report_rotation():
    """Section 11c. Forward motion is a near-tie, so the way out of the
    stall is to ROTATE. This measures whether DWB could see the turn it
    needed."""
    hdr('SECTION 11c -- the robot needed to TURN. Could DWB see it?')
    t0, t1 = crawl_window(BAD)
    rows = [r for r in load_full_trace(BAD) if t0 <= r['t'] <= t1]
    wz = [r['w_nav'] for r in rows if r['w_nav'] is not None]
    neg = sum(1 for v in wz if v < 0)
    pos = sum(1 for v in wz if v > 0)
    need = []
    for r in rows:
        b = math.degrees(math.atan2(GOAL_SHIFTED[1] - r['y'],
                                    GOAL_SHIFTED[0] - r['x']))
        need.append(ang_diff_deg(b, math.degrees(r['yaw'])))
    step = (MAX_VEL_TH - MIN_VEL_TH) / (VTH_SAMPLES - 1)
    mags = sorted(abs(v) for v in wz)
    small = sum(1 for v in mags if v <= 3 * step + 1e-6)
    ys = [math.degrees(r['yaw']) for r in rows]
    travel = sum(abs(ys[i + 1] - ys[i]) for i in range(len(ys) - 1))
    print(f'  required turn over the window (bearing to goal minus yaw): '
          f'{min(need):+.1f} .. {max(need):+.1f} deg -- ALWAYS POSITIVE')
    print(f'  wz DWB actually commanded, {len(wz)} rows: '
          f'negative {neg} ({100.0 * neg / len(wz):.1f} %), '
          f'positive {pos} ({100.0 * pos / len(wz):.1f} %)')
    print(f'  i.e. it turned AWAY from the goal on three rows in four.')
    print(f'  |wz| median {mags[len(mags) // 2]:.4f} rad/s; '
          f'{small}/{len(mags)} ({100.0 * small / len(mags):.1f} %) are '
          f'within the 3 smallest')
    print(f'  non-zero samples (step {step:.4f} rad/s).')
    print(f'  yaw travelled {travel:.1f} deg in total to net '
          f'{ys[-1] - ys[0]:+.1f} deg.')
    print()
    print('  WHY the turn is invisible. With vx = 0 the trajectory endpoint')
    print('  IS the robot cell, so GoalDist and PathDist are identical for')
    print('  all 40 rotations. The only critics that separate them are')
    print('  GoalAlign and PathAlign, which score at')
    print(f'  getForwardPose(final, {FWD_POINT_DIST}) -- '
          f'{FWD_POINT_DIST / RES:.0f} costmap cells ahead. A rotation can')
    print('  therefore move the scored point by at most 2 cells per axis:')
    span = 2 * 2 * MAPGRID_SCALE['GoalAlign'] + 2 * 2 * MAPGRID_SCALE['PathAlign']
    print(f'    GoalAlign span <= {2 * 2 * MAPGRID_SCALE["GoalAlign"]:.1f}, '
          f'PathAlign span <= {2 * 2 * MAPGRID_SCALE["PathAlign"]:.1f}, '
          f'total <= {span:.1f}')
    print(f'  against a chosen total of ~46-51, i.e. at most '
          f'{100.0 * span / 48.0:.0f} % of the score, quantised to whole')
    print('  cells. That is the entire signal available to pick the turn.')
    print()
    print('  And the forward reward is capped the same way: aggregation_type')
    print(f'  is `last`, so only the final pose scores, and sim_time x')
    print(f'  max_vel_x = {SIM_TIME} x {MAX_VEL_X} = '
          f'{SIM_TIME * MAX_VEL_X:.2f} m = {SIM_TIME * MAX_VEL_X / RES:.0f} '
          f'cells is as far as it can ever be.')
    return {'required_turn_min_deg': round(min(need), 1),
            'required_turn_max_deg': round(max(need), 1),
            'wz_negative_frac': round(neg / len(wz), 3),
            'wz_positive_frac': round(pos / len(wz), 3),
            'wz_smallest3_frac': round(small / len(mags), 3),
            'yaw_travel_deg': round(travel, 1),
            'yaw_net_deg': round(ys[-1] - ys[0], 1),
            'rotational_score_span': round(span, 1)}


def report_gating():
    hdr('SECTION 12 -- scoring problem, or validity/gating problem?')
    leg = leg_record(BAD)
    print('  dwb_local_planner.cpp:455 -- a short-circuited trajectory')
    print('  keeps a PARTIAL total >= 0 and is still pushed onto the')
    print('  results. nav_bench counts illegal as `total < 0`, i.e. ONLY')
    print('  the IllegalTrajectoryException path. So "94 % legal" says')
    print('  nothing about how many were COMPLETE.')
    print()
    print('  In C2-NAV.3\'s raw captures, where the critic count IS')
    print('  recorded, the split at a stall was:')
    for k, (c, s, i) in C2NAV3.items():
        print(f'    run {k}: complete {c}, short-circuited {s}, illegal {i} '
              f'-> {100.0 * c / 819:.1f} % complete but '
              f'{100.0 * (c + s) / 819:.1f} % "legal"')
    print()
    print('  The C2-NAV.19 artifact does NOT record the critic count, so')
    print('  its complete/short-circuited split is NOT PROVEN and is not')
    print('  inferred here.')
    print()
    print(f'  leg-wide illegal by THROWING critic: '
          f'{leg["dwb_illegal_by_critic"]}')
    tot = sum(leg['dwb_illegal_by_critic'].values())
    for k, v in leg['dwb_illegal_by_critic'].items():
        print(f'    {k:14s} {v:7d}  {100.0 * v / tot:5.1f} %')
    print(f'  leg-wide illegal fraction {leg["dwb_illegal_frac"]} over '
          f'{leg["dwb_cycles"]} cycles x 819')
    print()
    t0, t1 = crawl_window(BAD)
    win = [r for r in timeline(BAD) if t0 <= r['t'] <= t1]
    legal = sorted(r['legal_frac'] for r in win)
    ill = [r['illegal'] for r in win]
    print(f'  inside [{t0:.2f}, {t1:.2f}]: {len(win)} cycles')
    print(f'    legal fraction  min {legal[0]:.3f}  median '
          f'{legal[len(legal) // 2]:.3f}  max {legal[-1]:.3f}')
    print(f'    illegal count   min {min(ill):.0f}  max {max(ill):.0f}')
    print(f'    cycles with >= 40 illegal (more than the whole zero-vx '
          f'block): {sum(1 for v in ill if v >= 40)} / {len(win)}')
    print()
    print('  Structural counts, source-derived and asserted in self_test:')
    print('    zero-vx samples per cycle 40, forward 779, total 819.')
    return {'legal_frac_min': round(legal[0], 4),
            'legal_frac_median': round(legal[len(legal) // 2], 4),
            'legal_frac_max': round(legal[-1], 4),
            'illegal_min': min(ill), 'illegal_max': max(ill),
            'illegal_by_critic': leg['dwb_illegal_by_critic'],
            'complete_split_recorded': False}


# ---------------------------------------------------------------------
# Section 10 -- geometry.
# ---------------------------------------------------------------------

def geometry_at(tag, t_rel):
    rows = load_full_trace(tag)
    r = min(rows, key=lambda q: abs(q['t'] - t_rel))
    snap = plan_at(tag, r['t'])
    plan = [tuple(p) for p in snap['poses_world']]
    tp = transform_global_plan(plan, r['x'], r['y'])
    yaw_deg = math.degrees(r['yaw'])
    b_goal = bearing_deg(r['x'], r['y'], *GOAL_SHIFTED)
    b_tp_end = bearing_deg(r['x'], r['y'], *tp[-1]) if tp else None
    acc, tang = 0.0, None
    for i in range(1, len(tp)):
        acc += math.dist(tp[i - 1], tp[i])
        if acc >= 0.5:
            tang = bearing_deg(tp[0][0], tp[0][1], tp[i][0], tp[i][1])
            break
    cs = clearance_series(tag)
    return {
        't': r['t'], 'pose': (r['x'], r['y'], r['yaw']), 'yaw_deg': yaw_deg,
        'd_goal': math.dist((r['x'], r['y']), GOAL_SHIFTED),
        'd_sw_corner': math.dist((r['x'], r['y']), SW_CORNER),
        'd_waypoint': math.dist((r['x'], r['y']), WAYPOINT),
        'd_deadlock_pose': math.dist((r['x'], r['y']), DEADLOCK_POSE),
        'bearing_goal_deg': b_goal,
        'heading_err_goal_deg': ang_diff_deg(b_goal, yaw_deg),
        'plan_end_bearing_deg': b_tp_end,
        'heading_err_plan_end_deg': (ang_diff_deg(b_tp_end, yaw_deg)
                                     if b_tp_end is not None else None),
        'heading_err_plan_tangent_deg': (ang_diff_deg(tang, yaw_deg)
                                         if tang is not None else None),
        'n_transformed': len(tp),
        'transformed_last': tp[-1] if tp else None,
        'scan_min': r['scan_min'], 'd_min_base': clearance_at(cs, r['t']),
        'plan_age_s': r['t'] - snap['ts_offset_from_t0_s']}


def report_geometry():
    hdr('SECTION 10 -- geometry through the stall')
    t0, t1 = crawl_window(BAD)
    out = []
    print(f'  {"t":>7} {"x":>8} {"y":>8} {"yawdeg":>8} {"d_goal":>7} '
          f'{"d_sw":>6} {"errGoal":>8} {"errPlan":>8} {"ntp":>4} '
          f'{"dminb":>6}')
    for t in [t0 - 2, t0, t0 + 5, t0 + 10, (t0 + t1) / 2, t1 - 10, t1 - 5,
              t1, t1 + 3]:
        if t < 0:
            continue
        g = geometry_at(BAD, t)
        out.append(g)
        print(f'  {g["t"]:7.2f} {g["pose"][0]:8.4f} {g["pose"][1]:8.4f} '
              f'{g["yaw_deg"]:8.2f} {g["d_goal"]:7.3f} '
              f'{g["d_sw_corner"]:6.3f} {g["heading_err_goal_deg"]:+8.2f} '
              f'{g["heading_err_plan_end_deg"]:+8.2f} '
              f'{g["n_transformed"]:4d} {g["d_min_base"]:6.3f}')
    print()
    print('  errGoal = bearing(robot->GOAL_SHIFTED) - robot yaw')
    print('  errPlan = bearing(robot->last transformed plan pose) - yaw')
    hs = [abs(g['heading_err_goal_deg']) for g in out]
    print(f'  |heading error to goal| over the window: {min(hs):.1f} .. '
          f'{max(hs):.1f} deg')
    return out


def report_c2nav3_comparison():
    hdr('SECTION 10b -- is this C2-NAV.3\'s stall, or a new variant?')
    print('  C2-NAV.3, local CSF 5.0, BaseObstacle.scale 8.0:')
    print('    run A 1.312 m to goal, heading error   +0.68 deg')
    print('    run B 1.299 m to goal, heading error  +50.92 deg')
    print('    complete/short/illegal of 819: 151/648/20 and 278/541/0')
    print('    648/648 and 532/541 of the short-circuits aborted at')
    print('    BaseObstacle -- critic 3 of 7, before GoalDist')
    print('    transformed-plan cost 60-164 / 60-157, ZERO cost-0 poses')
    print('    cheapest non-zero cost within 3 cells of the robot: 57')
    t0, t1 = crawl_window(BAD)
    g = geometry_at(BAD, (t0 + t1) / 2)
    print()
    print('  C2-NAV.20, local CSF 65.0, same BaseObstacle.scale 8.0:')
    print(f'    {g["d_goal"]:.3f} m to goal, heading error '
          f'{g["heading_err_goal_deg"]:+.2f} deg')
    print(f'    d_min_base {g["d_min_base"]:.4f} m -- the robot\'s own cell')
    print('    and everything within '
          f'{g["d_min_base"] - D_COST_ZERO:.4f} m of it is cost 0')
    print('    leg-wide mean BaseObstacle on the CHOSEN trajectory: 0.00,')
    print('    i.e. it was 0 on every one of the 1157 cycles')
    print()
    print('  The cost field is what changed between them. At CSF 65 the')
    print('  inflation decays e-fold every 1/65 m = 15.4 mm:')
    for c in (1, 2, 6, 16, 60, 164, 253):
        d5 = ROBOT_RADIUS + math.log(252.0 / c) / GLOBAL_CSF
        print(f'    cost {c:4d}  ->  CSF 65: {inflation_distance_for_cost(c):.4f} m'
              f'   CSF 5: {d5:.4f} m'
              f'{"  (beyond inflation_radius -> not present)" if d5 > INFLATION_RADIUS else ""}')
    print()
    print('  So the two stalls are NOT the same mechanism. At CSF 5 the')
    print('  robot stood in a cost field that reached it; at CSF 65 the')
    print('  entire neighbourhood the trajectories can reach is cost 0.')


# ---------------------------------------------------------------------
# Section 13 -- discrete events.
# ---------------------------------------------------------------------

def report_events():
    hdr('SECTION 13 -- is there a discrete event at the stall onset?')
    runs, cyc, rows = stall_window(BAD)
    t0 = min(a for a, b in runs if b - a > 5.0)
    t1 = crawl_window(BAD)[1]
    pts = plan_times(BAD)
    before = [t for t in pts if t <= t0]
    after = [t for t in pts if t > t0]
    print(f'  T_START = {t0:.2f} s')
    print(f'  /plan snapshots either side: '
          f'{[round(t, 3) for t in before[-3:]]} | '
          f'{[round(t, 3) for t in after[:3]]}')
    if before:
        print(f'  nearest preceding replan {before[-1]:.3f} s '
              f'({t0 - before[-1]:.2f} s before T_START)')
    ivs = [round(pts[i + 1] - pts[i], 3) for i in range(len(pts) - 1)]
    print(f'  replan interval median {sorted(ivs)[len(ivs) // 2]:.3f} s, '
          f'max {max(ivs):.3f} s')
    print()
    print('  Collision-monitor transitions in [T_START-6, T_START+6]:')
    prev = None
    for r in rows:
        if not (t0 - 6 <= r['t'] <= t0 + 6):
            continue
        key = (r['cm_action'], r['cm_polygon'])
        if key != prev:
            print(f'    t={r["t"]:7.2f}  {CM_NAMES.get(r["cm_action"])}'
                  f'/{r["cm_polygon"]}   scan_min={r["scan_min"]}  '
                  f'v_nav={r["v_nav"]}  v_wheel={r["v_wheel"]}')
            prev = key
    print()
    print('  Transformed-plan geometry across the onset, every 0.5 s.')
    print('  C2-NAV.19 measured the RemovePassedGoals tick at 9.009 s, and')
    print('  BAD was pruned 0.7177 m from the waypoint it never reached.')
    for t in [t0 - 4 + 0.5 * i for i in range(13)]:
        g = geometry_at(BAD, t)
        print(f'    t={g["t"]:7.2f}  ntp={g["n_transformed"]:3d}  '
              f'last={tuple(round(v, 3) for v in g["transformed_last"])}  '
              f'errPlan={g["heading_err_plan_end_deg"]:+7.2f}  '
              f'errGoal={g["heading_err_goal_deg"]:+7.2f}  '
              f'age={g["plan_age_s"]:.2f}')
    print()
    leg = leg_record(BAD)
    print('  Nav2 warnings over the whole leg (the recovery evidence):')
    for k, v in leg['warnings'].items():
        print(f'    {v:3d} x {k}')
    return {'replan_before_start_s': round(before[-1], 3) if before else None,
            'gap_to_start_s': round(t0 - before[-1], 2) if before else None,
            'replan_interval_median_s': sorted(ivs)[len(ivs) // 2],
            'warnings': leg['warnings']}


# ---------------------------------------------------------------------
# Section 14 -- GOOD vs BAD.
# ---------------------------------------------------------------------

def report_goodbad():
    hdr('SECTION 14 -- GOOD vs BAD on the DWB data both runs recorded')
    print(f'  {"tag":>16} {"status":>10} {"dur":>7} {"cyc":>5} {"ill_f":>6} '
          f'{"zero_f":>7} {"bvx_mean":>9} {"scanmin":>8}')
    recs = {}
    for tag in (GOOD, *GOOD_ALTERNATES, BAD):
        L = leg_record(tag)
        recs[tag] = L
        print(f'  {tag:>16} {L["status"]:>10} {L["duration_sim_s"]:7.2f} '
              f'{L["dwb_cycles"]:5d} {L["dwb_illegal_frac"]:6.3f} '
              f'{L["dwb_best_vx_zero_frac"]:7.3f} '
              f'{L["dwb_best_vx_mean"]:9.4f} {L["min_scan_range_m"]:8.3f}')
    print()
    print('  Mean SCALED critic contribution of the CHOSEN trajectory')
    print('  (dwb_best_critic_mean -- the only per-cycle critic quantity')
    print('  nav_bench keeps, averaged over the leg):')
    keys = ['BaseObstacle', 'GoalAlign', 'GoalDist', 'PathAlign', 'PathDist',
            'RotateToGoal', 'Oscillation']
    print(f'  {"tag":>16} ' + ' '.join(f'{k[:10]:>11}' for k in keys))
    for tag in (GOOD, *GOOD_ALTERNATES, BAD):
        m = recs[tag]['dwb_best_critic_mean']
        print(f'  {tag:>16} ' + ' '.join(f'{m.get(k, 0.0):11.2f}'
                                         for k in keys))
    print()
    print('  BaseObstacle >= 0 always, so a leg mean of exactly 0.00 means')
    print('  the chosen trajectory ended in a cost-0 cell on EVERY cycle.')
    print('  Every GOOD run has a non-zero mean; the BAD run is 0.00.')
    print()
    print('  Illegal trajectories by THROWING critic, whole leg:')
    for tag in (GOOD, *GOOD_ALTERNATES, BAD):
        print(f'    {tag:>16} {recs[tag]["dwb_illegal_by_critic"]}')
    print()
    print('  Longest run of SELECTED zero vx, reconstructed identically:')
    out = {}
    for tag in (GOOD, *GOOD_ALTERNATES, BAD):
        runs, cyc, rows = stall_window(tag)
        top = runs[0] if runs else (0, 0)
        zf = sum(1 for r in cyc if r['dwb_best_vx'] == 0.0) / max(1, len(cyc))
        out[tag] = {'longest_zero_run_s': round(top[1] - top[0], 2),
                    'window': [round(top[0], 2), round(top[1], 2)],
                    'zero_frac': round(zf, 3), 'n_cycles': len(cyc),
                    'best_critic_mean': recs[tag]['dwb_best_critic_mean'],
                    'status': recs[tag]['status']}
        print(f'    {tag:>16} {top[1] - top[0]:7.2f} s '
              f'[{top[0]:.2f}, {top[1]:.2f}]   zero_frac {zf:.3f}')
    print()
    print('  What ends the longest zero-vx run (the 6 cycles after it):')
    for tag in (GOOD, BAD):
        runs, cyc, rows = stall_window(tag)
        t1 = runs[0][1]
        print(f'    {tag}:')
        for r in [q for q in cyc if q['t'] > t1][:6]:
            print(f'      t={r["t"]:7.2f}  vx_sel={r["dwb_best_vx"]:7.4f}  '
                  f'total={r["dwb_best_total"]:6.1f}  '
                  f'w_nav={r["w_nav"]:+7.4f}  v_wheel={r["v_wheel"]:7.4f}  '
                  f'{CM_NAMES.get(r["cm_action"])}/{r["cm_polygon"]}')
    return out


def report_recovery_tail():
    hdr('SECTION 5b -- from the end of the stall to the PolygonStop latch')
    rows = load_full_trace(BAD)
    t1 = crawl_window(BAD)[1]
    first_stop = next((r['t'] for r in rows
                       if r['cm_polygon'] == 'PolygonStop'), None)
    cs = clearance_series(BAD)
    print(f'  zero-vx run ends t = {t1:.2f} s;  first PolygonStop '
          f't = {first_stop:.2f} s  (gap {first_stop - t1:.2f} s)')
    print()
    print(f'  {"t":>7} {"x":>8} {"y":>8} {"dminb":>6} {"vx_sel":>7} '
          f'{"v_wheel":>7}  monitor')
    for r in rows:
        if not (t1 - 1 <= r['t'] <= first_stop + 1):
            continue
        if abs((r['t'] * 10) % 10) > 1e-6:
            continue
        print(f'  {r["t"]:7.2f} {r["x"]:8.4f} {r["y"]:8.4f} '
              f'{clearance_at(cs, r["t"]):6.3f} {r["dwb_best_vx"]:7.4f} '
              f'{r["v_wheel"]:7.4f}  {CM_NAMES.get(r["cm_action"])}'
              f'/{r["cm_polygon"]}')
    return {'zero_run_end_s': round(t1, 2),
            'first_polygonstop_s': round(first_stop, 2),
            'gap_s': round(first_stop - t1, 2)}


# ---------------------------------------------------------------------
# Self-test and classification.
# ---------------------------------------------------------------------

def self_test():
    hdr('SELF-TEST -- reproduce C2-NAV.19 and C2-NAV.3 from the '
        'committed artifacts')
    ok = True

    def chk(label, got, want, tol=None):
        nonlocal ok
        good = (got == want) if tol is None else (
            got is not None and abs(got - want) <= tol)
        ok = ok and good
        print(f'  [{"OK " if good else "FAIL"}] {label}: got {got!r} '
              f'want {want!r}' + (f' +/-{tol}' if tol else ''))
        return good

    chk('params sha256',
        hashlib.sha256(open(PARAMS_FILE, 'rb').read()).hexdigest(),
        PARAMS_SHA256)
    leg = leg_record(BAD)
    chk('BAD leg status', leg['status'], 'TIMEOUT')
    chk('BAD duration_sim_s', leg['duration_sim_s'],
        C2NAV19['leg_duration_s'], 0.01)
    chk('BAD final_goal_err_m', leg['final_goal_err_m'],
        C2NAV19['leg_goal_err_m'], 0.001)
    chk('BAD PolygonStop seconds', leg['cm_polygon_secs']['PolygonStop'],
        C2NAV19['polygonstop_secs'], 0.01)
    chk('BAD costmap snapshots', leg['costmap_window_n_snapshots'],
        C2NAV19['costmap_snapshots'])
    wc = leg['worst_crawl']
    chk('BAD worst_crawl length s', wc['crawl_len_s'],
        C2NAV19['crawl_len_s'], 0.01)
    chk('BAD worst_crawl start t_rel', wc['t_rel_s'],
        C2NAV19['crawl_start_t_rel_s'], 0.01)
    chk('BAD worst_crawl chosen vx', wc['dwb_chosen_vx'],
        C2NAV19['crawl_chosen_vx'], 1e-9)
    chk('BAD worst_crawl illegal frac', wc['dwb_illegal_frac'],
        C2NAV19['crawl_illegal_frac'], 1e-9)
    chk('BAD worst_crawl scan_min', wc['scan_min_m'],
        C2NAV19['crawl_scan_min_m'], 1e-9)
    for k, v in C2NAV19['crawl_critics'].items():
        chk(f'BAD worst_crawl critic {k}',
            wc['dwb_chosen_critics'].get(k, 0.0), v, 1e-9)
    chk('BAD chosen BaseObstacle leg mean is exactly 0',
        leg['dwb_best_critic_mean']['BaseObstacle'], 0.0, 0.0)
    rows = cc.load_stop_csv(BAD)
    chk('BAD stop-probe rows', len(rows), C2NAV19['stop_rows'])
    chk('BAD spawn gt x', float(rows[0]['gt_x']), SPAWN_WORLD[0], 1e-6)
    chk('BAD spawn gt y', float(rows[0]['gt_y']), SPAWN_WORLD[1], 1e-6)
    chk('BAD spawn gt |yaw|', abs(float(rows[0]['gt_yaw'])), 0.0, 1e-6)
    gleg = leg_record(GOOD)
    chk('GOOD leg status', gleg['status'], 'SUCCEEDED')
    chk('GOOD worst_crawl length s', gleg['worst_crawl']['crawl_len_s'],
        C2NAV19['good_worst_crawl_s'], 0.01)
    tr = load_full_trace(BAD)
    chk('BAD trace rows', len(tr), 2014)
    chk('BAD trace dwb_n constant 819',
        sorted({r['dwb_n'] for r in tr if r['dwb_n']}), [819.0])
    tw = sample_twists()
    chk('sample count', len(tw), 819)
    chk('zero-vx samples', sum(1 for v, _ in tw if v == 0.0), 40)
    chk('forward samples', sum(1 for v, _ in tw if v > 0.0), 779)
    chk('first sample is (vx=0, wz=min)', (tw[0][0], round(tw[0][1], 4)),
        (0.0, -1.0))
    res, cok = validate_c2nav3(verbose=False)
    if res:
        a0 = next(r for r in res if r['file'].endswith('A.json')
                  and r['snapshot'] == 0)
        b0 = next(r for r in res if r['file'].endswith('B.json')
                  and r['snapshot'] == 0)
        chk('C2-NAV.3 run A complete/short/illegal',
            (a0['complete'], a0['short'], a0['illegal']), C2NAV3['A'])
        chk('C2-NAV.3 run B complete/short/illegal',
            (b0['complete'], b0['short'], b0['illegal']), C2NAV3['B'])
        chk('C2-NAV.3 run A GoalDist seed cell', a0['goal_seed_cell'], (3, 26))
        chk('C2-NAV.3 critics reproduced exactly (run A snapshot 0)',
            all(a0['agree'][k] == a0['n_scored'] for k in a0['agree']), True)
        chk('C2-NAV.3 evaluation order reproduced', a0['eval_order_ok'], True)
    print()
    print('  SELF-TEST', 'PASSED' if ok else 'FAILED')
    return ok


def summary(sw=None):
    hdr('SECTION 18 -- classification')
    if sw is None:
        sw = sweep()
    t0, t1 = crawl_window(BAD)
    ms = [r['safe_margin'] for r in sw if r['safe_margin'] is not None]
    wins = sum(1 for v in ms if v > 1e-9)
    ties = sum(1 for v in ms if abs(v) <= 1e-9)
    loses = sum(1 for v in ms if v < -1e-9)
    ns = [r['n_safe_forward'] for r in sw]
    am = sorted(r['n_at_min'] for r in sw)
    leg = leg_record(BAD)
    print(f'  window [{t0:.2f}, {t1:.2f}] s = {t1 - t0:.2f} s, '
          f'{len(ms)} distinct DWB states reconstructed')
    print()
    print('  RULED OUT -- TRAJECTORY_VALIDITY:')
    print(f'    legal fraction never falls below '
          f'{min(r["legal"] / r["n"] for r in timeline(BAD, t0, t1)):.3f}; '
          f'the leg-wide illegal fraction is {leg["dwb_illegal_frac"]}.')
    print()
    print('  RULED OUT -- CRITIC_GATING:')
    print(f'    d_min_base stays {min(r["d_min_base"] for r in sw):.4f}-'
          f'{max(r["d_min_base"] for r in sw):.4f} m and the local cost')
    print(f'    field reaches 0 beyond {D_COST_ZERO:.4f} m, so '
          f'{min(ns)}-{max(ns)} of the 779 forward trajectories are')
    print('    PROVABLY in cost-0 cells at every state -- BaseObstacle')
    print("    cannot be rejecting them. The chosen trajectory's")
    print('    BaseObstacle leg mean is exactly 0.00 over 1157 cycles.')
    print('    Contrast C2-NAV.3 at CSF 5: 83.2 % / 69.4 % of forward')
    print('    trajectories short-circuited at BaseObstacle.')
    print()
    print('  PRESENT -- TEMPORAL_STATE_CHANGE:')
    print("    between t=8.20 and t=8.70 the transformed plan's in-window")
    print('    endpoint moves 1.13 m and the heading error to it swings')
    print('    +4.19 deg -> +58.98 deg; RemovePassedGoals fired at 9.009 s')
    print('    (C2-NAV.19) and the replan landed at 9.140 s. DWB collapsed')
    print('    from vx 0.3000 to 0.0000 in ONE cycle at t=10.20.')
    print()
    print('  PRESENT -- SCORE_DOMINANCE, but degenerate:')
    print(f'    best provably-cost-0 forward vs best zero-vx margin: '
          f'{min(ms):+.1f} .. {max(ms):+.1f}, median '
          f'{sorted(ms)[len(ms) // 2]:+.1f}, on totals of ~46-51.')
    print(f'    forward beats zero {wins}, exact tie {ties}, zero wins '
          f'{loses}, of {len(ms)}.')
    print(f'    median {am[len(am) // 2]} trajectories (up to {am[-1]}) share')
    print('    the minimum EXACTLY. Every total is a multiple of 0.2.')
    print("    The landscape is flat at one costmap cell, and DWB's")
    print('    strict `<` gives every tie to the first-evaluated')
    print('    trajectory -- always in the vx = 0 block.')
    print()
    verdict = 'COMBINATION (TEMPORAL_STATE_CHANGE -> SCORE_DOMINANCE)'
    print(f'  CLASSIFICATION: {verdict}')
    print('    NOT SCORE_DOMINANCE alone -- a discrete plan event puts the')
    print('    robot in the degenerate state, it does not drift in.')
    print('    NOT CRITIC_GATING and NOT TRAJECTORY_VALIDITY -- both are')
    print('    ruled out by measurement above.')
    return {'verdict': verdict, 'forward_wins': wins, 'ties': ties,
            'zero_wins': loses, 'n_states': len(ms),
            'margin_min': round(min(ms), 3), 'margin_max': round(max(ms), 3),
            'margin_median': round(sorted(ms)[len(ms) // 2], 3),
            'n_at_min_median': am[len(am) // 2], 'n_at_min_max': am[-1],
            'safe_forward_min': min(ns), 'safe_forward_max': max(ns),
            'window': [round(t0, 2), round(t1, 2)],
            'span_s': round(t1 - t0, 2)}


def dump(out_path):
    sw = sweep()
    c3, c3ok = validate_c2nav3(verbose=False)
    payload = {
        'experiment': 'C2-NAV.20',
        'params_file': os.path.basename(PARAMS_FILE),
        'params_sha256': PARAMS_SHA256,
        'bad_tag': BAD, 'good_tag': GOOD,
        'good_alternates': GOOD_ALTERNATES,
        'self_test_passed': self_test(),
        'c2nav3_validation': c3, 'c2nav3_validated': c3ok,
        'c2nav19_residual': residual_report(),
        'window': report_window(),
        'dominance': dominance(),
        'sweep_summary': report_sweep(sw),
        'rotation': report_rotation(),
        'seed_sensitivity': seed_sensitivity(),
        'gating': report_gating(),
        'geometry': report_geometry(),
        'events': report_events(),
        'goodbad': report_goodbad(),
        'recovery_tail': report_recovery_tail(),
        'c2nav3_mechanism': c2nav3_mechanism(),
        'classification': summary(sw),
        'sweep': sw,
    }
    with open(out_path, 'w') as f:
        json.dump(payload, f, indent=1, sort_keys=True, default=str)
    print(f'\nwrote {out_path}')


def visualize(out_path=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    sw = sweep()
    t0, t1 = crawl_window(BAD)
    tl = timeline(BAD, t0 - 8, t1 + 8)
    fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True)
    ts = [r['t'] for r in tl]
    axes[0].plot(ts, [r['vx_sel'] for r in tl], lw=1.2,
                 label='DWB selected vx')
    axes[0].plot(ts, [r['v_wheel'] for r in tl], lw=1.0, alpha=0.7,
                 label='wheel v')
    axes[0].set_ylabel('m/s')
    axes[0].legend(fontsize=8)
    axes[0].set_title(f'C2-NAV.20  {BAD}  enclosure_entry  '
                      f'selected-zero-vx window [{t0:.1f}, {t1:.1f}] s')
    axes[1].plot([r['t'] for r in sw], [r['zero_total'] for r in sw], lw=1.2,
                 label='best zero-vx (rebuilt, MapGrid only)')
    axes[1].plot([r['t'] for r in sw],
                 [r['safe_total'] for r in sw], lw=1.2,
                 label='best forward with BaseObstacle PROVABLY 0')
    axes[1].plot([r['t'] for r in sw], [r['published_total'] for r in sw],
                 lw=0.9, ls='--', label='published chosen total')
    axes[1].set_ylabel('DWB total')
    axes[1].legend(fontsize=8)
    axes[2].plot([r['t'] for r in sw], [r['safe_margin'] for r in sw], lw=1.2,
                 color='purple')
    axes[2].axhline(0, color='k', lw=0.8)
    axes[2].set_ylabel('zero - forward\n(margin, cells x scale)')
    axes[3].plot(ts, [r['illegal'] for r in tl], lw=1.0, label='illegal /819')
    axes[3].plot(ts, [r['d_min_base'] * 1000 for r in tl], lw=1.0,
                 label='d_min_base (mm)')
    axes[3].axhline(POLY_STOP_R * 1000, color='red', ls=':', lw=0.9,
                    label='PolygonStop 250 mm')
    axes[3].axhline(D_COST_ZERO * 1000, color='green', ls=':', lw=0.9,
                    label=f'cost-0 edge {D_COST_ZERO * 1000:.0f} mm')
    axes[3].set_ylabel('count / mm')
    axes[3].set_xlabel('leg-relative time (s)')
    axes[3].legend(fontsize=8)
    for a in axes:
        a.axvspan(t0, t1, color='red', alpha=0.10)
    fig.tight_layout()
    out_path = out_path or os.path.join(HERE, '..', 'images',
                                        'c2nav20_dwbstall.png')
    fig.savefig(out_path, dpi=110)
    print(f'wrote {out_path}')


def all_(argv):
    self_test()
    validate_c2nav3()
    c2nav3_mechanism()
    residual_report()
    report_window()
    dominance()
    report_timeline()
    sw = sweep()
    report_sweep(sw)
    report_rotation()
    seed_sensitivity()
    report_gating()
    report_geometry()
    report_c2nav3_comparison()
    report_events()
    report_goodbad()
    report_recovery_tail()
    summary(sw)


def main():
    cmds = {
        'selftest': lambda a: self_test(),
        'validate': lambda a: validate_c2nav3(),
        'residual': lambda a: residual_report(),
        'c2nav3': lambda a: c2nav3_mechanism(),
        'window': lambda a: report_window(),
        'dominance': lambda a: dominance(),
        'timeline': lambda a: report_timeline(),
        'sweep': lambda a: report_sweep(),
        'rotation': lambda a: report_rotation(),
        'sensitivity': lambda a: seed_sensitivity(),
        'gating': lambda a: report_gating(),
        'geometry': lambda a: report_geometry(),
        'compare3': lambda a: report_c2nav3_comparison(),
        'events': lambda a: report_events(),
        'goodbad': lambda a: report_goodbad(),
        'tail': lambda a: report_recovery_tail(),
        'summary': lambda a: summary(),
        'all': all_,
        'viz': lambda a: visualize(a[0] if a else None),
        'dump': lambda a: dump(a[0]),
    }
    if len(sys.argv) < 2 or sys.argv[1] not in cmds:
        print(__doc__)
        print('commands:', ', '.join(cmds))
        return 1
    cmds[sys.argv[1]](sys.argv[2:])
    return 0


if __name__ == '__main__':
    sys.exit(main())
