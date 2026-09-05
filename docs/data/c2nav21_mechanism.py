#!/usr/bin/env python3
"""C2-NAV.21 -- offline decomposition of every DWB mechanism that could
break the zero-velocity score degeneracy C2-NAV.20 measured.

DIAGNOSIS ONLY. Nothing here writes a parameter, starts a node or talks
to ROS. It is C2-NAV.20's validated reconstruction with every scoring
constant turned into an argument, so a candidate parameter change can be
scored against the 145 recorded BAD states BEFORE a simulator is
launched.

WHAT C2-NAV.20 LEFT OPEN
------------------------
It measured the degeneracy (margin -0.6 .. +1.4 on totals of 46-51,
median exactly 0.0, median 3 trajectories tied at the minimum, every
total a multiple of 0.2) and named ONE candidate:
`{Goal,Path}Align.forward_point_distance` 0.1 -> 0.325. It did not test
the alternatives it also named -- `aggregation_type`, the trajectory
horizon, the velocity lattice, the PathDist/GoalDist scale relationship
-- and it could not, because its scorer hard-codes `last` aggregation and
a single forward-point distance.

WHAT THIS MODULE ADDS
---------------------
1. A FULL-POSE trajectory generator (C2-NAV.20 keeps only the endpoint,
   which is all `last` aggregation needs). `sum` and `product` score
   every pose, and the pose COUNT is itself velocity-dependent
   (`getTimeSteps`), so the endpoint is not sufficient to predict them.
2. `aggregation_type` in {last, sum, product}, per critic, exactly as
   `MapGridCritic::scoreTrajectory` implements it -- including the
   `start_index` shortcut that applies only when
   `aggregation_type == last AND !stop_on_failure_`, which is true for
   GoalAlign/PathAlign and false for GoalDist/PathDist.
3. Separate `forward_point_distance` for GoalAlign and PathAlign, applied
   in BOTH places the source applies it: the seed nudge in
   `GoalAlignCritic::prepare`, and the scored point in `scorePose`.
4. The local costmap's finite extent, so a seed nudged past the 3 m
   rolling window is CLIPPED by `getLastPoseOnCostmap` rather than
   silently used -- the failure mode a bigger forward_point_distance
   could actually introduce.
5. `sim_time`, `vx_samples`, `vtheta_samples`, the granularities and all
   four MapGrid scales as arguments.

SOURCE READ (dwb 1.3.11, the installed version; adds to C2-NAV.20's list)
------------------------------------------------------------------------
8.  `MapGridCritic::scoreTrajectory`: `score = 0` (or 1 for Product);
    `start_index = poses.size()-1` ONLY when `Last && !stop_on_failure_`.
    So GoalDist/PathDist (stop_on_failure_ = true) walk EVERY pose even
    under `last` -- they just overwrite `score` each time -- while
    GoalAlign/PathAlign (which set `stop_on_failure_ = false` in their
    own `onInit`) read the final pose alone. Under `sum`/`product` ALL
    four walk every pose.
9.  `StandardTrajectoryGenerator::getTimeSteps`:
    `num_steps = ceil(max(|v|*sim_time/linear_granularity,
                          |w|*sim_time/angular_granularity))`, minimum 1,
    and `generateTrajectory` pushes the start pose, then one pose per
    step, then (include_last_point, default true) the final pose AGAIN.
    So `poses.size() == num_steps + 2` and it ranges from 3 to 62 across
    the lattice. Under `sum` that count multiplies the score directly.
10. `BaseObstacleCritic::scoreTrajectory` has its own `sum_scores` flag
    (default false -> last pose only) and is NOT a MapGridCritic, so
    `aggregation_type` does not reach it.
11. `GoalDistCritic::getLastPoseOnCostmap` returns the LAST plan pose
    that is inside the costmap and not NO_INFORMATION, after
    `adjustPlanResolution` has interpolated it to <= 2*resolution
    spacing. A nudged final pose outside the window therefore clips to
    the window edge instead of being used or throwing.
12. `MapGridCritic::scorePose` throws "Trajectory Goes Off Grid" OUTSIDE
    the `stop_on_failure_` guard, so an alignment point that leaves the
    3 m window makes the trajectory illegal even for GoalAlign. This
    module measures the worst-case alignment radius so the claim that it
    cannot happen is checked, not assumed.
13. `XYThetaIterator::startNewIteration` passes `sim_time_` as the
    velocity iterator's `acc_time`. With acc_x 3.0 / acc_theta 3.2
    against spans of 0.3 and 2.0 the window saturates for any sim_time
    above ~0.1 s, so the 819-sample lattice is INVARIANT to sim_time in
    this configuration -- only the horizon moves. Asserted in selftest.

VALIDATION
----------
`selftest` refuses to report anything until, at the BASELINE config:
  * the full-pose generator's final pose equals C2-NAV.20's endpoint
    generator exactly, over the whole lattice at every one of the 145
    states;
  * this module's scorer reproduces C2-NAV.20's `score_all` exactly (all
    four critics, all 819 trajectories, all 145 states);
  * C2-NAV.3's raw captures still reproduce through C2-NAV.20's own
    `validate_c2nav3()` after this module has run (no global corruption);
  * C2-NAV.3's committed complete/short-circuit/illegal splits
    (151/648/20 and 278/541/0) still come out of C2-NAV.20's checker;
  * the sample lattice is 819 = 20 vx x 41 theta - 1, invariant to
    sim_time in {0.5, 1.0, 1.5, 2.5};
  * C2-NAV.20's published degeneracy numbers (16 forward wins / 9 exact
    ties / 12 zero wins over the strided sweep, median margin 0.0,
    median 3 tied at the minimum, all totals multiples of 0.2) come back
    out of THIS module's baseline evaluation.

Usage (always under `python3 -P`, per the stray-`numbers.py` trap):

    python3 -P docs/data/c2nav21_mechanism.py selftest
    python3 -P docs/data/c2nav21_mechanism.py export      # committed states
    python3 -P docs/data/c2nav21_mechanism.py baseline
    python3 -P docs/data/c2nav21_mechanism.py matrix      # the ranking
    python3 -P docs/data/c2nav21_mechanism.py bounds      # off-grid check
    python3 -P docs/data/c2nav21_mechanism.py all
    python3 -P docs/data/c2nav21_mechanism.py dump docs/data/c2nav21_bench.json
"""

import json
import math
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import c2nav20_dwbstall as c20                                   # noqa: E402

STATES_FILE = os.path.join(HERE, 'c2nav21_states.json')

# The local costmap is 3 x 3 m at 0.05 m, rolling_window: true, so it
# spans +/- 30 cells about the robot's own cell. `plugins` carries no
# static layer and neither obstacle_layer nor voxel_layer sets
# track_unknown_space, so unknown cells default to FREE_SPACE and the
# NO_INFORMATION arm of getLastPoseOnCostmap cannot fire here; the only
# way off the costmap is the window edge.
HALF_CELLS = 30


def hdr(t):
    c20.hdr(t)


# ---------------------------------------------------------------------
# Configuration -- every scoring constant, as data
# ---------------------------------------------------------------------

class Config(dict):
    """A DWB scoring configuration. Every field maps to one parameter in
    `c2nav11_ntp_params.yaml` (or to a dwb default the file leaves
    unset), so a Config is exactly what a candidate params file would
    have to say."""

    def __init__(self, name, note='', **kw):
        base = dict(
            name=name, note=note,
            sim_time=1.5,
            vx_samples=20, vtheta_samples=40,
            linear_granularity=0.05, angular_granularity=0.025,
            fpd_goal=0.1, fpd_path=0.1,
            agg_goal_dist='last', agg_goal_align='last',
            agg_path_dist='last', agg_path_align='last',
            scale_goal_dist=24.0, scale_goal_align=24.0,
            scale_path_dist=32.0, scale_path_align=32.0,
            # BaseObstacle is not a MapGridCritic and does not override
            # getScale(), so its weight is the bare parameter against a
            # 0-252 cost -- not resolution * 0.5 * scale.
            scale_base_obstacle=8.0,
        )
        base.update(kw)
        dict.__init__(self, base)

    def __getattr__(self, k):
        try:
            return self[k]
        except KeyError:
            raise AttributeError(k)

    def scale(self, critic):
        """MapGridCritic::getScale() = resolution * 0.5 * scale_."""
        return c20.RES * 0.5 * self['scale_' + critic]

    def traj_key(self):
        """Trajectory geometry depends on these and nothing else, so the
        lattice is generated once per key and shared across candidates
        that differ only in scoring."""
        return (self['sim_time'], self['vx_samples'], self['vtheta_samples'],
                self['linear_granularity'], self['angular_granularity'])


BASELINE = Config(
    'A0-baseline',
    'c2nav11_ntp_params.yaml as frozen by C2-NAV.20, unchanged')


# ---------------------------------------------------------------------
# Lattice and trajectories, with every pose
# ---------------------------------------------------------------------

def sample_twists(cfg, cur_vx, cur_wz):
    """XYThetaIterator's order: theta innermost, x outermost, starting at
    min_vel_. (0,0,0) is rejected by isValidSpeed."""
    xs = c20.one_d_samples(cur_vx, c20.MIN_VEL_X, c20.MAX_VEL_X,
                           c20.ACC_X, c20.DECEL_X,
                           cfg['sim_time'], cfg['vx_samples'])
    ths = c20.one_d_samples(cur_wz, c20.MIN_VEL_TH, c20.MAX_VEL_TH,
                            c20.ACC_TH, c20.DECEL_TH,
                            cfg['sim_time'], cfg['vtheta_samples'])
    return [(vx, wz) for vx in xs for wz in ths
            if not (vx == 0.0 and wz == 0.0)]


def trajectory_poses(cfg, x, y, yaw, start_vx, start_wz, cmd_vx, cmd_wz):
    """StandardTrajectoryGenerator::generateTrajectory, every pose.

    discretize_by_time is false and include_last_point defaults to true,
    so the returned list is [start] + one pose per step + [final again].
    """
    st = cfg['sim_time']
    n = math.ceil(max(abs(cmd_vx) * st / cfg['linear_granularity'],
                      abs(cmd_wz) * st / cfg['angular_granularity']))
    n = max(1, int(n))
    dt = st / n
    vx, wz = start_vx, start_wz
    poses = [(x, y, yaw)]
    for _ in range(n):
        vx = c20.project_velocity(vx, c20.ACC_X, c20.DECEL_X, dt, cmd_vx)
        wz = c20.project_velocity(wz, c20.ACC_TH, c20.DECEL_TH, dt, cmd_wz)
        x += vx * math.cos(yaw) * dt
        y += vx * math.sin(yaw) * dt
        yaw += wz * dt
        poses.append((x, y, yaw))
    poses.append((x, y, yaw))          # include_last_point
    return poses


def build_lattice(cfg, st):
    """All trajectories at one recorded state. Cached by traj_key."""
    twists = sample_twists(cfg, st['v_act'], st['w_act'])
    out = []
    for vx, wz in twists:
        poses = trajectory_poses(cfg, st['x'], st['y'], st['yaw'],
                                 st['v_act'], st['w_act'], vx, wz)
        fx, fy, _ = poses[-1]
        out.append({'vx': vx, 'wz': wz, 'poses': poses,
                    'disp': math.hypot(fx - st['x'], fy - st['y'])})
    return out


# ---------------------------------------------------------------------
# Critic preparation, with the costmap's finite extent
# ---------------------------------------------------------------------

def map_bounds(rx, ry):
    """The rolling local costmap's cell extent about the robot."""
    cx, cy = c20.cell(rx, ry)
    return (cx - HALF_CELLS, cx + HALF_CELLS - 1,
            cy - HALF_CELLS, cy + HALF_CELLS - 1)


def on_map(c, b):
    return b[0] <= c[0] <= b[1] and b[2] <= c[1] <= b[3]


def last_pose_on_costmap(poses, b):
    """GoalDistCritic::getLastPoseOnCostmap. Walks the ALREADY adjusted
    plan, keeps the last pose inside the window, and stops at the first
    pose that leaves it after having been inside. Returns (cell,
    clipped) where `clipped` says the nudged endpoint was not usable."""
    last, started = None, False
    for p in poses:
        c = c20.cell(p[0], p[1])
        if on_map(c, b):
            last, started = c, True
        elif started:
            return last, True
    if last is None:
        return None, True
    return last, (c20.cell(poses[-1][0], poses[-1][1]) != last)


def prepare(cfg, plan_world, rx, ry, goal_world, end_thr=None,
            seed_backoff=0):
    """Everything the four MapGrid critics' prepare() does, with the seed
    nudge parameterised by GoalAlign.forward_point_distance and the
    costmap's extent enforced."""
    tp = c20.transform_global_plan(plan_world, rx, ry, end_thr)
    if not tp:
        return None
    if seed_backoff:
        tp = tp[:max(1, len(tp) - seed_backoff)]
    b = map_bounds(rx, ry)
    adj = c20.adjust_plan_resolution(tp)

    goal_seed, gd_clipped = last_pose_on_costmap(adj, b)

    ang = math.atan2(goal_world[1] - ry, goal_world[0] - rx)
    nudged = list(tp)
    nudged[-1] = (nudged[-1][0] + cfg['fpd_goal'] * math.cos(ang),
                  nudged[-1][1] + cfg['fpd_goal'] * math.sin(ang))
    ga_seed, ga_clipped = last_pose_on_costmap(
        c20.adjust_plan_resolution(nudged), b)

    path_seeds = sorted({c20.cell(px, py) for px, py in adj
                         if on_map(c20.cell(px, py), b)})
    if goal_seed is None or ga_seed is None or not path_seeds:
        return None

    # PathAlignCritic::prepare zeroes its own scale once the robot is
    # within forward_point_distance of the goal.
    pa_zero_scale = (math.dist((rx, ry), goal_world) <= cfg['fpd_path'])

    return {'goal_seed': goal_seed, 'ga_seed': ga_seed,
            'path_seeds': path_seeds, 'bounds': b,
            'gd_clipped': gd_clipped, 'ga_clipped': ga_clipped,
            'pa_zero_scale': pa_zero_scale,
            'n_transformed': len(tp), 'first': tp[0], 'last': tp[-1]}


class Field:
    """A memoised min-over-seeds L1 field. C2-NAV.3 measured DWB's own
    flood against exactly this on 0 mismatched cells, because
    `MapGridQueue::validCellToQueue` returns true unconditionally."""

    def __init__(self, seeds):
        self.seeds = list(seeds)
        self.cache = {}

    def __call__(self, c):
        v = self.cache.get(c)
        if v is None:
            v = min(abs(c[0] - s[0]) + abs(c[1] - s[1]) for s in self.seeds)
            self.cache[c] = v
        return v


# ---------------------------------------------------------------------
# Scoring, with aggregation
# ---------------------------------------------------------------------

def aggregate(values, how):
    """MapGridCritic::scoreTrajectory's switch, over the poses it walks."""
    if how == 'last':
        return values[-1]
    if how == 'sum':
        return float(sum(values))
    if how == 'product':
        score = 1.0
        for v in values:
            if score > 0:
                score *= v
        return score
    raise ValueError(how)


def score_all(cfg, st, lat, g):
    """Score every trajectory on the four MapGrid critics.

    `total` is the complete DWB total under BaseObstacle = 0, which is
    what the C2-NAV.20 clearance bound proves for the trajectories this
    module compares. RotateToGoal and Oscillation contribute 0 to the
    chosen trajectory on every one of the BAD leg's 1157 cycles
    (measured, C2-NAV.20 selftest), so they are omitted.
    """
    gdf = Field([g['goal_seed']])
    gaf = Field([g['ga_seed']])
    pf = Field(g['path_seeds'])
    fg, fp = cfg['fpd_goal'], cfg['fpd_path']
    s_gd, s_ga = cfg.scale('goal_dist'), cfg.scale('goal_align')
    s_pd = cfg.scale('path_dist')
    s_pa = 0.0 if g['pa_zero_scale'] else cfg.scale('path_align')
    a_gd, a_ga = cfg['agg_goal_dist'], cfg['agg_goal_align']
    a_pd, a_pa = cfg['agg_path_dist'], cfg['agg_path_align']

    # start_index applies only to Last AND !stop_on_failure_, which is
    # GoalAlign/PathAlign. GoalDist/PathDist walk every pose regardless.
    ga_last_only = (a_ga == 'last')
    pa_last_only = (a_pa == 'last')

    out = []
    for tr in lat:
        poses = tr['poses']
        body = poses[-1:] if a_gd == 'last' and a_pd == 'last' else poses
        # GoalDist/PathDist read the pose itself. Under `last` the value
        # is the final pose's, so only that pose need be evaluated -- the
        # walk over the others exists in the source solely to throw, and
        # BaseObstacle = 0 is proven for the trajectories compared here.
        gd_vals = [gdf(c20.cell(p[0], p[1])) for p in body]
        pd_vals = [pf(c20.cell(p[0], p[1])) for p in body]

        al = poses[-1:] if ga_last_only else poses
        ga_vals = [gaf(c20.cell(p[0] + fg * math.cos(p[2]),
                                p[1] + fg * math.sin(p[2]))) for p in al]
        pl = poses[-1:] if pa_last_only else poses
        pa_vals = [pf(c20.cell(p[0] + fp * math.cos(p[2]),
                               p[1] + fp * math.sin(p[2]))) for p in pl]

        gd = aggregate(gd_vals, a_gd)
        pd = aggregate(pd_vals, a_pd)
        ga = aggregate(ga_vals, a_ga)
        pa = aggregate(pa_vals, a_pa)
        out.append({
            'vx': tr['vx'], 'wz': tr['wz'], 'disp': tr['disp'],
            'n_poses': len(poses),
            'GoalDist': gd, 'GoalAlign': ga, 'PathDist': pd, 'PathAlign': pa,
            'total': gd * s_gd + ga * s_ga + pd * s_pd + pa * s_pa})
    return out


def argmin_first(scored, pred=None):
    """DWB's tie-break: strict `<`, so the FIRST minimum in evaluation
    order wins."""
    best = None
    for s in scored:
        if pred is not None and not pred(s):
            continue
        if best is None or s['total'] < best['total'] - 1e-12:
            best = s
    return best


# ---------------------------------------------------------------------
# The recorded states
# ---------------------------------------------------------------------

def build_states():
    """The 145 distinct DWB states inside C2-NAV.19's BAD crawl window,
    read from the scratch traces the way C2-NAV.20's sweep() does."""
    runs, cyc, rows = c20.stall_window(c20.BAD)
    t_lo, t_hi = c20.crawl_window(c20.BAD)
    snaps = c20.pw.load_planwindow(c20.BAD)['snapshots']
    cs = c20.clearance_series(c20.BAD)
    out = []
    for r in [x for x in cyc if t_lo - 0.001 <= x['t'] <= t_hi + 0.001]:
        snap = None
        for s in snaps:
            if s['ts_offset_from_t0_s'] <= r['t']:
                snap = s
            else:
                break
        if snap is None:
            continue
        dmin = c20.clearance_at(cs, r['t'])
        out.append({
            't': round(r['t'], 2), 'x': r['x'], 'y': r['y'], 'yaw': r['yaw'],
            'v_act': r['v_act'] or 0.0, 'w_act': r['w_act'] or 0.0,
            'plan_ts': snap['ts_offset_from_t0_s'],
            'plan': [list(p) for p in snap['poses_world']],
            'd_min_base': dmin,
            'published_vx': r['dwb_best_vx'],
            'published_total': r['dwb_best_total'],
            'published_illegal': r['dwb_illegal'],
            # DWB's own output on /cmd_vel_nav at this instant. The trace
            # carries no dwb_best_wz, so this is how the CHOSEN rotation
            # is recovered: /cmd_vel_nav is the controller's raw command,
            # upstream of the velocity smoother and the arbiter.
            'v_nav': r['v_nav'], 'w_nav': r['w_nav'],
        })
    return out


def export_states(path=STATES_FILE):
    st = build_states()
    plans, index = [], {}
    for s in st:
        k = s['plan_ts']
        if k not in index:
            index[k] = len(plans)
            plans.append({'ts': k, 'poses': s['plan']})
        s = dict(s)
    rows = []
    for s in st:
        r = dict(s)
        r['plan_idx'] = index[s['plan_ts']]
        del r['plan']
        rows.append(r)
    full = c20.load_full_trace(c20.BAD) or []
    # The trace samples at 10 Hz with the last value held, but DWB only
    # ran when it published /evaluation -- 1157 times over the leg, i.e.
    # 5.75 Hz, because 819 trajectories x 7 critics does not fit in the
    # 10 Hz controller period. `debrief()` therefore ran ONCE per
    # /evaluation message, not once per trace row, and the Oscillation
    # state machine has to be replayed at that cadence.
    tick = {round(r['t'], 3) for r in c20.eval_cycles(full)}
    osc = []
    for r in full:
        osc.append([round(r['t'], 3), round(r['x'], 6), round(r['y'], 6),
                    round(r['yaw'], 6),
                    None if r['v_nav'] is None else round(r['v_nav'], 6),
                    None if r['w_nav'] is None else round(r['w_nav'], 6),
                    1 if round(r['t'], 3) in tick else 0])
    payload = {
        'experiment': 'C2-NAV.21',
        'source_run': c20.BAD,
        'cmd_trace_cols': ['t_rel_s', 'x', 'y', 'yaw', 'v_nav', 'w_nav',
                           'is_controller_tick'],
        'cmd_trace': osc,
        'source_leg': c20.LEG,
        'params_file': os.path.basename(c20.PARAMS_FILE),
        'params_sha256': c20.PARAMS_SHA256,
        'window_s': list(c20.crawl_window(c20.BAD)),
        'note': ('Every distinct /evaluation state inside C2-NAV.19 BAD '
                 'run c2n19_tour_r1 leg enclosure_entry, with the global '
                 'plan snapshot live at that instant and the stop probe '
                 'clearance. Committed so every C2-NAV.21 table '
                 'regenerates without the untracked .navbench scratch.'),
        'plans': plans, 'states': rows,
    }
    with open(path, 'w') as f:
        json.dump(payload, f, separators=(',', ':'), sort_keys=True)
    print(f'wrote {path}: {len(rows)} states, {len(plans)} distinct plans')
    return payload


_STATES = None


def states():
    """Prefer the committed file; fall back to the scratch traces."""
    global _STATES
    if _STATES is not None:
        return _STATES
    if os.path.exists(STATES_FILE):
        d = json.load(open(STATES_FILE))
        out = []
        for r in d['states']:
            r = dict(r)
            r['plan'] = [tuple(p) for p in d['plans'][r['plan_idx']]['poses']]
            out.append(r)
        _STATES = out
    else:
        st = build_states()
        for s in st:
            s['plan'] = [tuple(p) for p in s['plan']]
        _STATES = st
    return _STATES


# ---------------------------------------------------------------------
# Evaluating a configuration
# ---------------------------------------------------------------------

_LAT_CACHE = {}


def lattice_for(cfg, st):
    key = (cfg.traj_key(), st['t'])
    lat = _LAT_CACHE.get(key)
    if lat is None:
        lat = build_lattice(cfg, st)
        _LAT_CACHE[key] = lat
    return lat


def required_turn_deg(st):
    """Bearing from the robot to its own plan's far end, minus its yaw --
    the turn the robot has to make. C2-NAV.20 measured it positive at
    every state in the window."""
    plan = st['plan']
    tgt = plan[-1]
    return c20.ang_diff_deg(
        c20.bearing_deg(st['x'], st['y'], tgt[0], tgt[1]),
        math.degrees(st['yaw']))


def evaluate(cfg, sts=None, seed_backoff=0):
    """One record per recorded state."""
    sts = sts if sts is not None else states()
    out = []
    for st in sts:
        g = prepare(cfg, st['plan'], st['x'], st['y'], st['plan'][-1],
                    seed_backoff=seed_backoff)
        if g is None:
            continue
        lat = lattice_for(cfg, st)
        sc = score_all(cfg, st, lat, g)
        dmin = st['d_min_base']
        safe = (dmin - c20.D_COST_ZERO) if dmin else 0.0

        b_zero = argmin_first(sc, lambda s: s['vx'] == 0.0)
        b_fwd = argmin_first(sc, lambda s: s['vx'] > 0.0)
        b_safe = argmin_first(sc, lambda s: s['vx'] > 0.0
                              and s['disp'] <= safe)
        sel_all = argmin_first(sc)
        sel_safe = argmin_first(sc, lambda s: s['vx'] == 0.0
                                or s['disp'] <= safe)

        gmin = min(s['total'] for s in sc)
        at_min = [s for s in sc if abs(s['total'] - gmin) <= 1e-9]
        zero_blk = [s for s in sc if s['vx'] == 0.0]
        fwd_blk = [s for s in sc if s['vx'] > 0.0]
        zt = [s['total'] for s in zero_blk]

        need = required_turn_deg(st)
        out.append({
            't': st['t'], 'x': st['x'], 'y': st['y'], 'yaw': st['yaw'],
            'd_min_base': dmin, 'safe_disp_m': round(safe, 4),
            'n_safe_forward': sum(1 for s in fwd_blk if s['disp'] <= safe),
            'n_traj': len(sc), 'n_zero': len(zero_blk),
            'required_turn_deg': round(need, 2),
            'gd_clipped': g['gd_clipped'], 'ga_clipped': g['ga_clipped'],
            'zero_total': b_zero['total'], 'zero_wz': b_zero['wz'],
            'zero_GoalDist': b_zero['GoalDist'],
            'zero_GoalAlign': b_zero['GoalAlign'],
            'zero_PathDist': b_zero['PathDist'],
            'zero_PathAlign': b_zero['PathAlign'],
            'fwd_total': b_fwd['total'] if b_fwd else None,
            'safe_total': b_safe['total'] if b_safe else None,
            'safe_vx': b_safe['vx'] if b_safe else None,
            'safe_wz': b_safe['wz'] if b_safe else None,
            'safe_disp': round(b_safe['disp'], 4) if b_safe else None,
            'safe_GoalDist': b_safe['GoalDist'] if b_safe else None,
            'safe_GoalAlign': b_safe['GoalAlign'] if b_safe else None,
            'safe_PathDist': b_safe['PathDist'] if b_safe else None,
            'safe_PathAlign': b_safe['PathAlign'] if b_safe else None,
            'margin': (b_zero['total'] - b_safe['total']) if b_safe else None,
            'margin_any': (b_zero['total'] - b_fwd['total'])
            if b_fwd else None,
            'n_at_min': len(at_min),
            'n_at_min_zero': sum(1 for s in at_min if s['vx'] == 0.0),
            'sel_vx': sel_all['vx'], 'sel_wz': sel_all['wz'],
            'sel_total': sel_all['total'],
            'sel_safe_vx': sel_safe['vx'], 'sel_safe_wz': sel_safe['wz'],
            'rot_span': max(zt) - min(zt),
            'fwd_reach_cells': max(s['disp'] for s in fwd_blk) / c20.RES,
        })
    return out


def _q(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return (None, None, None)
    return (min(vals), statistics.median(vals), max(vals))


def summarize(cfg, rows):
    """The metric block section 11 of the brief asks for, per candidate."""
    n = len(rows)
    marg = [r['margin'] for r in rows if r['margin'] is not None]
    fwd_win = sum(1 for m in marg if m > 1e-9)
    tie = sum(1 for m in marg if abs(m) <= 1e-9)
    zero_win = sum(1 for m in marg if m < -1e-9)
    sel_fwd = sum(1 for r in rows if r['sel_vx'] > 0.0)
    sel_safe_fwd = sum(1 for r in rows if r['sel_safe_vx'] > 0.0)
    # Does the selected rotation go the way the robot needs to turn?
    right_wz = sum(1 for r in rows
                   if r['sel_wz'] * r['required_turn_deg'] > 0)
    right_wz_safe = sum(1 for r in rows
                        if r['sel_safe_wz'] * r['required_turn_deg'] > 0)
    neg_wz = sum(1 for r in rows if r['sel_wz'] < 0)
    quant = _totals_quantum(rows)
    return {
        'name': cfg['name'], 'note': cfg['note'], 'n_states': n,
        'margin_min': _q(marg)[0], 'margin_median': _q(marg)[1],
        'margin_max': _q(marg)[2],
        'forward_wins': fwd_win, 'exact_ties': tie, 'zero_wins': zero_win,
        'tie_frac': round(tie / n, 4) if n else None,
        'n_at_min_median': statistics.median([r['n_at_min'] for r in rows]),
        'n_at_min_max': max(r['n_at_min'] for r in rows),
        'rot_span_min': round(_q([r['rot_span'] for r in rows])[0], 4),
        'rot_span_median': round(_q([r['rot_span'] for r in rows])[1], 4),
        'rot_span_max': round(_q([r['rot_span'] for r in rows])[2], 4),
        'selected_forward_frac': round(sel_fwd / n, 4) if n else None,
        'selected_forward_frac_safe': round(sel_safe_fwd / n, 4)
        if n else None,
        'selected_vx_median': statistics.median([r['sel_vx'] for r in rows]),
        'selected_wz_median': statistics.median([r['sel_wz'] for r in rows]),
        'correct_turn_frac': round(right_wz / n, 4) if n else None,
        'correct_turn_frac_safe': round(right_wz_safe / n, 4) if n else None,
        'negative_wz_frac': round(neg_wz / n, 4) if n else None,
        'total_quantum': quant,
        'goal_seed_clipped': sum(1 for r in rows if r['gd_clipped']),
        'ga_seed_clipped': sum(1 for r in rows if r['ga_clipped']),
        'n_safe_forward_median': statistics.median(
            [r['n_safe_forward'] for r in rows]),
    }


def _totals_quantum(rows):
    """The greatest common divisor of the score increments, to 1e-9. It
    is 0.2 at baseline (gcd of the 0.6 and 0.8 scales) and that quantum
    is what makes exact ties common."""
    vals = sorted({round(r['zero_total'], 9) for r in rows}
                  | {round(r['safe_total'], 9) for r in rows
                     if r['safe_total'] is not None})
    if len(vals) < 2:
        return None
    diffs = [round(b - a, 9) for a, b in zip(vals, vals[1:]) if b - a > 1e-9]
    if not diffs:
        return None
    g = diffs[0]
    for d in diffs[1:]:
        while d > 1e-9:
            g, d = d, g - math.floor(g / d) * d
    return round(g, 6)


# ---------------------------------------------------------------------
# The Oscillation critic's state machine, replayed from the commands
# ---------------------------------------------------------------------

# dwb defaults; `c2nav11_ntp_params.yaml` sets no Oscillation.* key, and
# the historical searchParam fallbacks for x_only_threshold are COMMENTED
# OUT in 1.3.11, so `FollowPath.min_speed_xy: 0.0` never reaches it.
OSC_RESET_DIST = 0.05
OSC_RESET_ANGLE = 0.2
OSC_RESET_TIME = -1.0
OSC_X_ONLY_THRESHOLD = 0.05


class Trend:
    """OscillationCritic::CommandTrend, verbatim."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.sign = 0
        self.positive_only = False
        self.negative_only = False

    def update(self, v):
        flag = False
        if v < 0.0:
            if self.sign > 0:
                self.negative_only = True
                flag = True
            self.sign = -1
        elif v > 0.0:
            if self.sign < 0:
                self.positive_only = True
                flag = True
            self.sign = 1
        return flag

    def is_oscillating(self, v):
        return ((self.positive_only and v < 0.0)
                or (self.negative_only and v > 0.0))

    def flipped(self):
        return self.positive_only or self.negative_only


def cmd_trace():
    """The leg's 10 Hz (pose, command) sequence, from the committed state
    file where it exists."""
    if os.path.exists(STATES_FILE):
        d = json.load(open(STATES_FILE))
        if 'cmd_trace' in d:
            return [{'t': r[0], 'x': r[1], 'y': r[2], 'yaw': r[3],
                     'v_nav': r[4], 'w_nav': r[5],
                     'tick': (r[6] if len(r) > 6 else 1)}
                    for r in d['cmd_trace']]
    return c20.load_full_trace(c20.BAD) or []


def replay_oscillation():
    """Replay OscillationCritic exactly as `debrief` runs it, over the
    recorded commands, and ask at every cycle which rotation direction was
    BANNED when DWB scored the next one.

    Decidable offline because the critic's entire state is a function of
    the command sequence and the pose sequence, both of which the trace
    holds at 10 Hz. Nothing is fitted: the four parameters are dwb's own
    defaults and the params file sets none of them.
    """
    rows = [r for r in cmd_trace()
            if r['w_nav'] is not None and r.get('tick', 1)]
    xt, tt = Trend(), Trend()
    prev_pose = None
    out = []
    for r in rows:
        pose = (r['x'], r['y'], r['yaw'])
        # The state left by every previous debrief() is what the critic
        # scores THIS cycle against.
        ban = 0
        if tt.positive_only:
            ban = -1                    # negative rotations illegal
        elif tt.negative_only:
            ban = 1                     # positive rotations illegal
        out.append({'t': r['t'], 'ban': ban, 'w_nav': r['w_nav'],
                    'v_nav': r['v_nav'], 'yaw': r['yaw'],
                    'violation': (ban == 1 and r['w_nav'] > 0.0)
                    or (ban == -1 and r['w_nav'] < 0.0)})
        flag = xt.update(r['v_nav'] if r['v_nav'] is not None else 0.0)
        if (OSC_X_ONLY_THRESHOLD < 0.0
                or abs(r['v_nav'] or 0.0) <= OSC_X_ONLY_THRESHOLD):
            flag = tt.update(r['w_nav']) or flag
        if flag:
            prev_pose = pose
        if xt.flipped() or tt.flipped():
            avail = False
            if prev_pose is not None:
                if (OSC_RESET_DIST >= 0.0
                        and math.dist(pose[:2], prev_pose[:2])
                        > OSC_RESET_DIST):
                    avail = True
                if (OSC_RESET_ANGLE >= 0.0
                        and abs(pose[2] - prev_pose[2]) > OSC_RESET_ANGLE):
                    avail = True
            if avail:
                xt.reset()
                tt.reset()
    return out


def oscillation_report():
    hdr('C2-NAV.21 -- the Oscillation critic, replayed over the BAD leg')
    print('  OscillationCritic latches a DIRECTIONAL BAN the first time the')
    print('  commanded rotation changes sign while |vx| <= '
          f'{OSC_X_ONLY_THRESHOLD} (which is')
    print('  every cycle of a zero-velocity stall), and holds it until the')
    print(f'  robot has moved {OSC_RESET_DIST} m or turned {OSC_RESET_ANGLE}'
          f' rad ({math.degrees(OSC_RESET_ANGLE):.1f} deg) from')
    print('  the pose where it latched. scoreTrajectory then throws for')
    print('  EVERY trajectory of the banned sign: half the lattice.')
    print()
    rep = replay_oscillation()
    t0, t1 = c20.crawl_window(c20.BAD)
    win = [r for r in rep if t0 - 0.001 <= r['t'] <= t1 + 0.001]
    viol = [r for r in rep if r['violation']]
    banned = [r for r in rep if r['ban'] != 0]
    banned_w = [r for r in win if r['ban'] != 0]
    lat = c20.sample_twists(0.0, 0.0)
    n_pos = sum(1 for _, wz in lat if wz > 0)
    n_neg = sum(1 for _, wz in lat if wz < 0)
    implied = sum(n_pos if r['ban'] == 1 else n_neg for r in banned)
    leg = c20.leg_record(c20.BAD)
    reported = (leg.get('dwb_illegal_by_critic') or {}).get('Oscillation')
    cycles = leg.get('dwb_cycles')
    scale = (cycles / len(rep)) if rep else 0.0
    out = {
        'trace_rows': len(rep),
        'rows_with_a_ban': len(banned),
        'ban_frac': round(len(banned) / len(rep), 4) if rep else None,
        'rows_in_crawl_window': len(win),
        'rows_with_a_ban_in_window': len(banned_w),
        'ban_frac_in_window': round(len(banned_w) / len(win), 4)
        if win else None,
        'sign_violations': len(viol),
        'lattice_positive_wz': n_pos, 'lattice_negative_wz': n_neg,
        'implied_oscillation_illegals_at_10hz': implied,
        'eval_cycles': cycles, 'trace_to_eval_scale': round(scale, 4),
        'implied_oscillation_illegals_at_eval_rate': round(implied * scale),
        'reported_oscillation_illegals': reported,
    }
    for k, v in out.items():
        print(f'    {k:<46} {v}')
    print()
    print('  VIOLATIONS are cycles where the replayed ban says a sign was')
    print('  illegal and DWB commanded it anyway. A correct replay has 0.')
    print()
    rows = evaluate(BASELINE)
    need = {r['t']: r['required_turn_deg'] for r in rows}
    wrong = agree = 0
    for r in win:
        nd = need.get(round(r['t'], 2))
        if nd is None:
            continue
        if r['ban'] == 1 and nd > 0:
            wrong += 1
        elif r['ban'] != 0:
            agree += 1
    print(f'    crawl-window cycles whose ban forbids the needed turn: '
          f'{wrong}')
    print(f'    crawl-window cycles banned the other way:              '
          f'{agree}')
    out['window_cycles_banning_the_needed_turn'] = wrong
    out['window_cycles_banning_the_other_way'] = agree
    return out


# ---------------------------------------------------------------------
# Full-critic emulation, validated where ground truth exists
# ---------------------------------------------------------------------

LETHAL, INSCRIBED, NO_INFO = 254, 253, 255


def c2nav3_snapshots(path):
    d = json.load(open(path))
    return d['snapshots']


def emulate_c2nav3(snap, cfg=None, oscillation_ban=0):
    """Every critic DWB ran, on a snapshot where the costmap, the
    transformed plan and all 819 per-trajectory scores were captured.

    This is the only place a SELECTION can be checked rather than a
    score: C2-NAV.20 validated the four MapGrid critic values, but its
    `argmin_first` was never tested against a recorded `best_index`
    under the full critic set, with BaseObstacle's illegals and the
    short-circuit in place.

    `oscillation_ban`: +1 bans wz > 0, -1 bans wz < 0, 0 no ban. The
    critic's own state is not in the capture, so it is a parameter, and
    the captured illegal attribution says which value is right.
    """
    cfg = cfg or BASELINE
    cm = snap['costmap']
    res, (ox, oy) = cm['resolution'], cm['origin']
    sx, sy, data = cm['size_x'], cm['size_y'], cm['data']
    tp = [(p[0], p[1]) for p in snap['transformed_plan']['poses']]
    rx, ry, ryaw = snap['chosen']['poses'][0][:3]

    def c_(x, y):
        return (int((x - ox) / res), int((y - oy) / res))

    def cost(c):
        if not (0 <= c[0] < sx and 0 <= c[1] < sy):
            return None                      # worldToMap fails -> throws
        return data[c[1] * sx + c[0]]

    adj = c20.adjust_plan_resolution(tp, res)
    gd_seed = c_(*adj[-1])
    pseeds = sorted({c_(q[0], q[1]) for q in adj})
    ang = math.atan2(tp[-1][1] - ry, tp[-1][0] - rx)
    nud = list(tp)
    nud[-1] = (nud[-1][0] + cfg['fpd_goal'] * math.cos(ang),
               nud[-1][1] + cfg['fpd_goal'] * math.sin(ang))
    ga_seed = c_(*c20.adjust_plan_resolution(nud, res)[-1])

    gdf, gaf, pf = Field([gd_seed]), Field([ga_seed]), Field(pseeds)
    fg, fp = cfg['fpd_goal'], cfg['fpd_path']
    twists = sample_twists(cfg, 0.0, 0.0)

    out = []
    for idx, (vx, wz) in enumerate(twists):
        poses = trajectory_poses(cfg, rx, ry, ryaw, 0.0, 0.0, vx, wz)
        rec = {'i': idx, 'vx': vx, 'wz': wz, 'n_poses': len(poses),
               'illegal': None, 'total': None}
        if oscillation_ban > 0 and wz > 0.0:
            rec['illegal'] = 'Oscillation'
            out.append(rec)
            continue
        if oscillation_ban < 0 and wz < 0.0:
            rec['illegal'] = 'Oscillation'
            out.append(rec)
            continue
        bo, bad = 0.0, None
        for p in poses:
            c = cost(c_(p[0], p[1]))
            if c is None or c in (LETHAL, INSCRIBED, NO_INFO):
                bad = 'BaseObstacle'
                break
            bo = c
        if bad:
            rec['illegal'] = bad
            out.append(rec)
            continue
        gd = gdf(c_(poses[-1][0], poses[-1][1]))
        pd = pf(c_(poses[-1][0], poses[-1][1]))
        fx, fy, fyaw = poses[-1]
        ga = gaf(c_(fx + fg * math.cos(fyaw), fy + fg * math.sin(fyaw)))
        pa = pf(c_(fx + fp * math.cos(fyaw), fy + fp * math.sin(fyaw)))
        rec.update({'GoalDist': gd, 'GoalAlign': ga, 'PathDist': pd,
                    'PathAlign': pa, 'BaseObstacle': bo,
                    'total': (bo * cfg['scale_base_obstacle']
                              + ga * cfg.scale('goal_align')
                              + pa * cfg.scale('path_align')
                              + pd * cfg.scale('path_dist')
                              + gd * cfg.scale('goal_dist'))})
        out.append(rec)
    return out


def validate_selection():
    """Does argmin_first over THIS module's reconstructed totals pick the
    trajectory DWB actually picked? Run where the answer is recorded."""
    hdr('C2-NAV.21 -- validate the SELECTION, not just the scores, '
        'against C2-NAV.3\'s captured best_index')
    print('  C2-NAV.20 validated the four MapGrid critic VALUES. It never')
    print('  checked that a reconstruction picks the same trajectory DWB')
    print('  picked, because the C2-NAV.19 artifact records no per-')
    print('  trajectory data to check against. C2-NAV.3\'s does: the local')
    print('  costmap with its origin, the transformed plan, and all 819')
    print('  scores with their critic counts.')
    print()
    out = []
    for name in ('c2nav3_stallA.json', 'c2nav3_stallB.json'):
        path = os.path.join(HERE, name)
        if not os.path.exists(path):
            continue
        for si, snap in enumerate(c2nav3_snapshots(path)):
            cap = snap['all']
            ban = 0
            capt_ill = {}
            for a in cap:
                if a['total'] < 0 and a['critics']:
                    k = a['critics'][0][0]
                    capt_ill[k] = capt_ill.get(k, 0) + 1
            if capt_ill.get('Oscillation'):
                pos = sum(1 for a, (vx, wz) in
                          zip(cap, sample_twists(BASELINE, 0.0, 0.0))
                          if a['total'] < 0 and a['critics']
                          and a['critics'][0][0] == 'Oscillation' and wz > 0)
                ban = 1 if pos else -1
            em = emulate_c2nav3(snap, oscillation_ban=ban)
            npose_ok = sum(1 for a, b in zip(cap, em)
                           if a['n_poses'] == b['n_poses'])
            ill_ok = sum(1 for a, b in zip(cap, em)
                         if (a['total'] < 0) == (b['illegal'] is not None))
            legal = [b for b in em if b['illegal'] is None]
            best = argmin_first(legal)
            ci = snap['best_index']
            got = (round(best['vx'], 6), round(best['wz'], 6))
            want = (round(snap['chosen']['vx'], 6),
                    round(snap['chosen']['wz'], 6))
            rec = {'file': name, 'snapshot': si, 'n_traj': len(cap),
                   'captured_illegal_by_critic': capt_ill,
                   'oscillation_ban': ban,
                   'n_poses_reproduced': f'{npose_ok}/{len(cap)}',
                   'illegal_flag_reproduced': f'{ill_ok}/{len(cap)}',
                   'selected': got, 'captured': want,
                   'selection_match': got == want,
                   'captured_best_index': ci,
                   'model_best_index': best['i']}
            out.append(rec)
            print(f'  {name} snap {si}: n_poses {npose_ok}/{len(cap)}  '
                  f'illegal-flag {ill_ok}/{len(cap)}  '
                  f'ban {ban}  captured illegals {capt_ill}')
            print(f'      model picked (vx={got[0]}, wz={got[1]}) '
                  f'index {best["i"]};  DWB picked '
                  f'(vx={want[0]}, wz={want[1]}) index {ci}  '
                  f'-> {"MATCH" if got == want else "DIFFER"}')
    n = len(out)
    ok = sum(1 for r in out if r['selection_match'])
    print()
    print(f'  SELECTION reproduced on {ok}/{n} captured snapshots.')
    return out


# ---------------------------------------------------------------------
# What this reconstruction can and cannot predict at C2-NAV.19
# ---------------------------------------------------------------------

def rotation_residual():
    """Measure how far DWB's ACTUAL command sat from this module's own
    optimum at the same state, and withdraw every statistic that depends
    on which individual trajectory wins.

    C2-NAV.20 already recorded that the C2-NAV.19 reconstruction puts the
    GoalDist seed 3 plan poses too far along and that no pose offset or
    lattice phase closes it, so it used ONLY the zero-vs-forward
    difference, which a common shift cannot move. This section shows why
    that restraint was right, and extends it: the choice of ROTATION is
    not invariant to the residual either, and this module gets it wrong.
    """
    hdr('C2-NAV.21 -- what the C2-NAV.19 reconstruction does NOT support')
    sts = states()
    rows = evaluate(BASELINE)
    by_t = {r['t']: r for r in rows}
    gaps, ranks = [], []
    live_pos = live_neg = model_pos = model_neg = agree = 0
    for st in sts:
        r = by_t.get(st['t'])
        if r is None or st.get('w_nav') is None:
            continue
        g = prepare(BASELINE, st['plan'], st['x'], st['y'], st['plan'][-1])
        sc = score_all(BASELINE, st, lattice_for(BASELINE, st), g)
        best = argmin_first(sc)
        live = min(sc, key=lambda q: (abs(q['vx'] - (st['v_nav'] or 0.0))
                                      + abs(q['wz'] - st['w_nav'])))
        gaps.append(live['total'] - best['total'])
        ranks.append(sum(1 for q in sc if q['total'] < live['total'] - 1e-9))
        live_pos += st['w_nav'] > 1e-9
        live_neg += st['w_nav'] < -1e-9
        model_pos += r['sel_wz'] > 1e-9
        model_neg += r['sel_wz'] < -1e-9
        agree += (st['w_nav'] * r['sel_wz'] > 0)
    n = len(gaps)
    out = {
        'n_states': n,
        'live_wz_positive': live_pos, 'live_wz_negative': live_neg,
        'model_wz_positive': model_pos, 'model_wz_negative': model_neg,
        'rotation_sign_agreement': agree,
        'rotation_sign_agreement_frac': round(agree / n, 4) if n else None,
        'score_gap_live_vs_model_optimum_min': round(min(gaps), 2),
        'score_gap_live_vs_model_optimum_median':
            round(statistics.median(gaps), 2),
        'score_gap_live_vs_model_optimum_max': round(max(gaps), 2),
        'trajectories_better_than_live_median':
            statistics.median(ranks),
        'one_cell_in_score_points': [c20.RES * 0.5 * 24.0,
                                     c20.RES * 0.5 * 32.0],
    }
    for k, v in out.items():
        print(f'    {k:<44} {v}')
    print()
    print('  The live command sits a median '
          f'{out["score_gap_live_vs_model_optimum_median"]} score points -- '
          'several costmap')
    print('  cells -- above this module\'s optimum, with a median of')
    print(f'  {out["trajectories_better_than_live_median"]:.0f} of 819 '
          'trajectories scoring strictly better. That is far')
    print('  outside quantisation noise, so it is NOT the degeneracy: the')
    print('  reconstruction\'s INPUTS are wrong for C2-NAV.19 in a way that')
    print('  changes which individual trajectory wins.')
    print()
    print('  WITHDRAWN, therefore, for C2-NAV.19: `sel_vx`, `sel_wz`,')
    print('  `correct_turn_frac`, `negative_wz_frac` -- every statistic')
    print('  about WHICH trajectory is chosen. They are computed and kept')
    print('  in the record, and they must not be read as predictions.')
    print()
    print('  STILL SUPPORTED: the margin between the best zero-vx and the')
    print('  best provably-cost-0 forward trajectory, the exact-tie count,')
    print('  the number of trajectories sharing the minimum, and the score')
    print('  span across the rotation block. A constant error in the seed')
    print('  shifts every total by the same amount and cannot change which')
    print('  of them tie; C2-NAV.20 measured that invariance over a seed')
    print('  backoff of 0-8 plan poses and this module reproduces it.')
    print()
    print('  `selected_forward_frac` is kept because it is not an')
    print('  independent claim: it equals forward_wins/n exactly (the zero')
    print('  block is evaluated first, so a forward trajectory is the')
    print('  global argmin exactly when it strictly beats every zero one).')
    return out


# ---------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------

def _chk(results, label, got, want, tol=None):
    if tol is None:
        ok = got == want
    else:
        ok = got is not None and abs(got - want) <= tol
    results.append((ok, label, got, want))
    print(f'  [{"OK " if ok else "FAIL"}] {label}: got {got!r} want {want!r}'
          + (f' +/-{tol}' if tol else ''))
    return ok


def self_test():
    hdr('C2-NAV.21 SELF-TEST -- this module must reproduce C2-NAV.20 and '
        'C2-NAV.3 exactly before any candidate is scored')
    res = []
    sts = states()
    _chk(res, 'recorded states', len(sts), 145)

    # 1. The lattice, and its invariance to sim_time.
    st0 = sts[0]
    for t in (0.5, 1.0, 1.5, 2.5):
        cfg = Config('probe', sim_time=t)
        _chk(res, f'lattice size at sim_time={t}',
             len(sample_twists(cfg, st0['v_act'], st0['w_act'])), 819)
    _chk(res, 'zero-vx block',
         sum(1 for vx, _ in sample_twists(BASELINE, st0['v_act'],
                                          st0['w_act']) if vx == 0.0), 40)
    _chk(res, 'first sample is (vx=0, wz=min)',
         sample_twists(BASELINE, st0['v_act'], st0['w_act'])[0], (0.0, -1.0))

    # 2. The full-pose generator's endpoint == C2-NAV.20's endpoint, over
    #    the whole lattice at every state.
    bad_end = 0
    bad_count = 0
    for st in sts:
        c20_saved = (c20.SIM_TIME, c20.FWD_POINT_DIST)
        for vx, wz in c20.sample_twists(st['v_act'], st['w_act']):
            a = c20.generate_trajectory(st['x'], st['y'], st['yaw'],
                                        st['v_act'], st['w_act'], vx, wz)
            p = trajectory_poses(BASELINE, st['x'], st['y'], st['yaw'],
                                 st['v_act'], st['w_act'], vx, wz)
            if max(abs(a[0] - p[-1][0]), abs(a[1] - p[-1][1]),
                   abs(a[2] - p[-1][2])) > 1e-12:
                bad_end += 1
            n = math.ceil(max(abs(vx) * 1.5 / 0.05, abs(wz) * 1.5 / 0.025))
            if len(p) != max(1, int(n)) + 2:
                bad_count += 1
        assert (c20.SIM_TIME, c20.FWD_POINT_DIST) == c20_saved
    _chk(res, 'endpoint mismatches vs C2-NAV.20 generator', bad_end, 0)
    _chk(res, 'pose-count mismatches vs getTimeSteps', bad_count, 0)

    # 3. Baseline scoring == C2-NAV.20's score_all, exactly.
    mism = 0
    for st in sts:
        g20 = c20.build_grids(st['plan'], st['x'], st['y'], st['plan'][-1])
        g21 = prepare(BASELINE, st['plan'], st['x'], st['y'], st['plan'][-1])
        if g20 is None or g21 is None:
            mism += 1
            continue
        s20 = c20.score_all(st['x'], st['y'], st['yaw'], st['v_act'],
                            st['w_act'], g20)
        s21 = score_all(BASELINE, st, lattice_for(BASELINE, st), g21)
        if len(s20) != len(s21):
            mism += 1
            continue
        for a, b in zip(s20, s21):
            if (a['GoalDist'] != b['GoalDist']
                    or a['GoalAlign'] != b['GoalAlign']
                    or a['PathDist'] != b['PathDist']
                    or a['PathAlign'] != b['PathAlign']
                    or abs(a['mg_total'] - b['total']) > 1e-9):
                mism += 1
    _chk(res, 'critic mismatches vs C2-NAV.20 score_all', mism, 0)

    # 4. C2-NAV.20's published degeneracy numbers, out of THIS module.
    rows = evaluate(BASELINE)
    s = summarize(BASELINE, rows)
    _chk(res, 'baseline median margin', s['margin_median'], 0.0, 1e-9)
    _chk(res, 'baseline margin min', s['margin_min'], -0.6, 0.05)
    _chk(res, 'baseline margin max', s['margin_max'], 1.4, 0.05)
    _chk(res, 'baseline median trajectories tied at minimum',
         s['n_at_min_median'], 3)
    _chk(res, 'baseline max trajectories tied at minimum',
         s['n_at_min_max'], 16)
    _chk(res, 'baseline score quantum', s['total_quantum'], 0.2, 1e-6)
    _chk(res, 'baseline forward/tie/zero',
         (s['forward_wins'], s['exact_ties'], s['zero_wins']), (67, 32, 46))
    _chk(res, 'baseline median provably-cost-0 forward trajectories',
         s['n_safe_forward_median'], 297)
    # The C2-NAV.20 strided sweep (stride 4, 37 states) published
    # 16/9/12; reproduce that stride exactly.
    sub = rows[::4]
    m = [r['margin'] for r in sub if r['margin'] is not None]
    _chk(res, 'C2-NAV.20 stride-4 forward/tie/zero',
         (sum(1 for v in m if v > 1e-9), sum(1 for v in m if abs(v) <= 1e-9),
          sum(1 for v in m if v < -1e-9)), (16, 9, 12))

    # 5. The rotation cap. C2-NAV.20 derived <= 5.6 by hand from
    #    "2 cells per axis"; the true worst case is a seed on the
    #    DIAGONAL, where the alignment point's L1 to it swings by
    #    2*sqrt(2)*r_cells ~ 5.66 cells rather than 4. Measured max is
    #    6.0, so the hand bound was mildly optimistic. What it got right
    #    is the structural half, and that is what is asserted here:
    #    across the whole zero-vx block GoalDist and PathDist are
    #    IDENTICAL (the endpoint is the robot's own cell), so the
    #    alignment critics are the entire signal for choosing a rotation.
    gd_span = pd_span = 0
    for st in sts:
        g = prepare(BASELINE, st['plan'], st['x'], st['y'], st['plan'][-1])
        z = [q for q in score_all(BASELINE, st, lattice_for(BASELINE, st), g)
             if q['vx'] == 0.0]
        gd = [q['GoalDist'] for q in z]
        pd = [q['PathDist'] for q in z]
        gd_span = max(gd_span, max(gd) - min(gd))
        pd_span = max(pd_span, max(pd) - min(pd))
    _chk(res, 'GoalDist span across the zero-vx block, worst state',
         gd_span, 0)
    _chk(res, 'PathDist span across the zero-vx block, worst state',
         pd_span, 0)
    _chk(res, 'baseline rotation-block span, max', s['rot_span_max'], 6.0,
         1e-9)
    _chk(res, 'baseline rotation-block span, median',
         s['rot_span_median'], 4.6, 1e-9)

    # 6. C2-NAV.3 still reproduces through C2-NAV.20 after all of the
    #    above -- proof this module corrupted no shared global.
    _, c3ok = c20.validate_c2nav3(verbose=False)
    _chk(res, 'C2-NAV.3 critics still reproduce', c3ok, True)
    # C2-NAV.20's own self-test carries C2-NAV.3's committed
    # complete/short-circuit/illegal splits (151/648/20 and 278/541/0),
    # its GoalDist seed cell and its evaluation-order check. Running it
    # LAST proves this module left no shared global altered.
    _chk(res, 'C2-NAV.20 own self-test still passes', c20.self_test(), True)

    ok = all(r[0] for r in res)
    print()
    print('  SELF-TEST ' + ('PASSED' if ok else 'FAILED'))
    return ok


# ---------------------------------------------------------------------
# The candidate matrix
# ---------------------------------------------------------------------

def candidates():
    """Every mechanism C2-NAV.20 named, and nothing else. Each entry
    carries the measured symptom it should move and the falsifier that
    rejects it."""
    return [
        (BASELINE, {
            'mechanism': 'none -- the frozen C2-NAV.20 configuration',
            'symptom': 'margin median 0.0, 3 tied at the minimum',
            'falsifier': 'n/a'}),

        (Config('A1-fpd-0.325', 'dwb own default, which the params file '
                'overrode', fpd_goal=0.325, fpd_path=0.325), {
            'mechanism': 'alignment lookahead: the point GoalAlign and '
                         'PathAlign score sits 6.5 cells ahead of the '
                         'trajectory endpoint instead of 2, so a rotation '
                         'moves it 3.25x further',
            'symptom': 'rotation-block span, correct-turn fraction',
            'falsifier': 'rotation span and tie frequency barely move'}),

        (Config('A2-fpd-0.20', 'intermediate, only if A1 overshoots',
                fpd_goal=0.20, fpd_path=0.20), {
            'mechanism': 'same, at half the step',
            'symptom': 'rotation-block span',
            'falsifier': 'no monotone trend between 0.1, 0.2 and 0.325'}),

        (Config('B1-agg-sum', 'aggregation_type: sum on all four MapGrid '
                'critics', agg_goal_dist='sum', agg_goal_align='sum',
                agg_path_dist='sum', agg_path_align='sum'), {
            'mechanism': 'endpoint-only scoring gives forward motion at '
                         'most 9 cells of GoalDist; summing over the '
                         'whole trajectory should reward sustained '
                         'progress',
            'symptom': 'score separation between forward and zero',
            'falsifier': 'the pose COUNT is velocity-dependent '
                         '(num_steps + 2, 3..62), so a sum is dominated '
                         'by how many poses a trajectory has, not by '
                         'where they are'}),

        (Config('B2-agg-sum-align', 'aggregation_type: sum on the two '
                'alignment critics only', agg_goal_align='sum',
                agg_path_align='sum'), {
            'mechanism': 'as B1 but confined to the critics that '
                         'discriminate rotations',
            'symptom': 'rotation-block span',
            'falsifier': 'same pose-count confound, on the block where '
                         'pose count varies most (|wz| spans 0..1)'}),

        (Config('B3-agg-product', 'aggregation_type: product',
                agg_goal_dist='product', agg_goal_align='product',
                agg_path_dist='product', agg_path_align='product'), {
            'mechanism': 'multiplicative aggregation',
            'symptom': 'score separation',
            'falsifier': 'product of ~40 over up to 62 poses spans 60 '
                         'orders of magnitude and collapses to 0 the '
                         'moment any pose sits on a seed'}),

        (Config('C1-vx-40', 'vx_samples 20 -> 40', vx_samples=40), {
            'mechanism': 'the velocity lattice is too coarse to place a '
                         'trajectory endpoint where it scores better',
            'symptom': 'exact-tie count, margin',
            'falsifier': 'the tie is between endpoints one CELL apart, '
                         'and 20 vx samples already put 288-334 forward '
                         'endpoints in distinct cells'}),

        (Config('C2-vth-80', 'vtheta_samples 40 -> 80', vtheta_samples=80), {
            'mechanism': 'the rotation lattice is too coarse',
            'symptom': 'rotation-block span, correct-turn fraction',
            'falsifier': 'a finer lattice inside an unchanged 2-cell '
                         'alignment radius cannot widen the span'}),

        (Config('D1-sim-2.5', 'sim_time 1.5 -> 2.5', sim_time=2.5), {
            'mechanism': 'the horizon caps forward reach at '
                         '0.3*1.5 = 9 cells, which is small against a '
                         'seed 27 cells away',
            'symptom': 'GoalDist separation, margin',
            'falsifier': 'reach scales but so does the trajectory\'s '
                         'exposure; and the local costmap is only '
                         '+/-30 cells'}),

        (Config('D2-sim-1.0', 'sim_time 1.5 -> 1.0', sim_time=1.0), {
            'mechanism': 'shorter horizon, opposite direction, to show '
                         'the sign of the effect',
            'symptom': 'GoalDist separation',
            'falsifier': 'monotonicity check only'}),

        (Config('E1-pathdist-24', 'PathDist.scale 32 -> 24, matching '
                'GoalDist', scale_path_dist=24.0), {
            'mechanism': 'a 1-cell GoalDist gain costs a 1-cell PathDist '
                         'loss, and PathDist is weighted 0.8 against '
                         'GoalDist 0.6 -- so the measured typical trade '
                         '(dGD -1, dPD +1) is a NET PENALTY of +0.2 for '
                         'moving forward',
            'symptom': 'margin sign, forward-win count',
            'falsifier': 'the measured per-critic deltas do not show '
                         'PathDist cancelling GoalDist'}),

        (Config('E2-goaldist-32', 'GoalDist.scale 24 -> 32, matching '
                'PathDist', scale_goal_dist=32.0), {
            'mechanism': 'same imbalance, corrected from the other side',
            'symptom': 'margin sign, forward-win count',
            'falsifier': 'as E1'}),
    ]


def matrix(sel=None):
    hdr('C2-NAV.21 SECTION 4/5/6 -- offline mechanism matrix over the 145 '
        'recorded BAD states')
    print('  Every row is the SAME 145 states from C2-NAV.19 BAD run')
    print(f'  {c20.BAD} leg {c20.LEG}, rescored under one changed '
          'parameter.')
    print('  BaseObstacle = 0 is PROVEN for the "safe" comparison by the')
    print(f'  C2-NAV.20 clearance bound (cost 0 beyond '
          f'{c20.D_COST_ZERO:.4f} m).')
    print()
    print('  margin = best zero-vx total - best provably-cost-0 forward '
          'total.')
    print('  A POSITIVE margin means forward motion genuinely scores '
          'better.')
    print('  rot_span = score range across the 40 zero-vx rotations: the')
    print('  entire signal DWB has for choosing which way to turn.')
    print('  fwd_win = fraction of states where a forward trajectory '
          'strictly')
    print('  beats every zero-vx one; identical to forward_wins/n.')
    print('  The rotation-CHOICE statistics are withdrawn -- see')
    print('  `python3 -P c2nav21_mechanism.py residual`.')
    print()
    out = []
    hd = (f'  {"candidate":<16} {"margin min/med/max":>22} '
          f'{"f/t/z":>12} {"tied":>6} {"rot_span":>18} {"fwd_win":>8} '
          f'{"quantum":>8}')
    print(hd)
    print('  ' + '-' * (len(hd) - 2))
    for cfg, meta in candidates():
        if sel and cfg['name'] not in sel:
            continue
        rows = evaluate(cfg)
        s = summarize(cfg, rows)
        s['meta'] = meta
        out.append(s)
        print(f'  {s["name"]:<16} '
              f'{s["margin_min"]:>7.2f}/{s["margin_median"]:>6.2f}/'
              f'{s["margin_max"]:>7.2f} '
              f'{s["forward_wins"]:>3}/{s["exact_ties"]:>3}/'
              f'{s["zero_wins"]:>3} '
              f'{s["n_at_min_median"]:>3.0f}/{s["n_at_min_max"]:<3} '
              f'{s["rot_span_min"]:>5.1f}/{s["rot_span_median"]:>5.1f}/'
              f'{s["rot_span_max"]:>5.1f} '
              f'{s["selected_forward_frac"]:>8.3f} '
              f'{str(s["total_quantum"]):>8}')
    return out


def detail(name):
    """The per-critic decomposition for one candidate, against baseline."""
    cand = {c['name']: (c, m) for c, m in candidates()}
    cfg, meta = cand[name]
    hdr(f'C2-NAV.21 detail -- {name}')
    for k, v in meta.items():
        print(f'  {k:<11}: {v}')
    print()
    b = {r['t']: r for r in evaluate(BASELINE)}
    rows = evaluate(cfg)
    s = summarize(cfg, rows)
    print(f'  {"t":>7} {"need":>7} | {"base margin":>11} {"cand margin":>11} '
          f'| {"base rot":>9} {"cand rot":>9} | {"sel vx":>7} {"sel wz":>7}')
    for r in rows[::8]:
        o = b.get(r['t'])
        if o is None:
            continue
        print(f'  {r["t"]:>7.2f} {r["required_turn_deg"]:>7.1f} | '
              f'{(o["margin"] if o["margin"] is not None else float("nan")):>11.2f} '
              f'{(r["margin"] if r["margin"] is not None else float("nan")):>11.2f} | '
              f'{o["rot_span"]:>9.2f} {r["rot_span"]:>9.2f} | '
              f'{r["sel_vx"]:>7.4f} {r["sel_wz"]:>7.4f}')
    print()
    print('  per-critic median deltas (best safe forward minus best zero):')
    for lbl in ('GoalDist', 'GoalAlign', 'PathDist', 'PathAlign'):
        d = [r[f'safe_{lbl}'] - r[f'zero_{lbl}'] for r in rows
             if r[f'safe_{lbl}'] is not None]
        d0 = [b[r['t']][f'safe_{lbl}'] - b[r['t']][f'zero_{lbl}']
              for r in rows
              if r['t'] in b and b[r['t']][f'safe_{lbl}'] is not None]
        print(f'    {lbl:<10} baseline {statistics.median(d0):>8.2f}   '
              f'candidate {statistics.median(d):>8.2f}')
    print()
    for k, v in s.items():
        if k != 'meta':
            print(f'    {k:<32} {v}')
    return s


def bounds_report():
    """Section 12 of the source read: can a bigger forward_point_distance
    push a scored point off the 3 m local costmap and make previously
    legal trajectories illegal?"""
    hdr('C2-NAV.21 -- does a larger forward_point_distance leave the '
        'local costmap?')
    print('  The local costmap is 3 x 3 m, rolling, so every scored point')
    print(f'  must stay within {HALF_CELLS} cells '
          f'({HALF_CELLS * c20.RES:.2f} m) of the robot on each axis, or')
    print('  MapGridCritic::scorePose throws "Trajectory Goes Off Grid" --')
    print('  outside the stop_on_failure_ guard, so even GoalAlign throws.')
    print()
    out = []
    for cfg in (BASELINE,
                Config('A1-fpd-0.325', '', fpd_goal=0.325, fpd_path=0.325),
                Config('D1-sim-2.5', '', sim_time=2.5),
                Config('D1+A1', '', sim_time=2.5, fpd_goal=0.325,
                       fpd_path=0.325)):
        worst_pose = worst_align = 0.0
        off = 0
        gd_clip = ga_clip = 0
        for st in states():
            b = map_bounds(st['x'], st['y'])
            g = prepare(cfg, st['plan'], st['x'], st['y'], st['plan'][-1])
            if g is None:
                continue
            gd_clip += 1 if g['gd_clipped'] else 0
            ga_clip += 1 if g['ga_clipped'] else 0
            for tr in lattice_for(cfg, st):
                for p in tr['poses']:
                    worst_pose = max(worst_pose,
                                     math.hypot(p[0] - st['x'],
                                                p[1] - st['y']))
                    for fpd in (cfg['fpd_goal'], cfg['fpd_path']):
                        ax = p[0] + fpd * math.cos(p[2])
                        ay = p[1] + fpd * math.sin(p[2])
                        worst_align = max(worst_align,
                                          math.hypot(ax - st['x'],
                                                     ay - st['y']))
                        if not on_map(c20.cell(ax, ay), b):
                            off += 1
        rec = {'name': cfg['name'],
               'worst_pose_radius_m': round(worst_pose, 4),
               'worst_alignment_radius_m': round(worst_align, 4),
               'window_half_extent_m': HALF_CELLS * c20.RES,
               'scored_points_off_costmap': off,
               'goal_seed_clipped_states': gd_clip,
               'ga_seed_clipped_states': ga_clip,
               'n_states': len(states())}
        out.append(rec)
        print(f'  {rec["name"]:<14} worst pose {rec["worst_pose_radius_m"]:.3f} m'
              f'   worst alignment point '
              f'{rec["worst_alignment_radius_m"]:.3f} m'
              f'   off-costmap {off}'
              f'   seeds clipped GoalDist {gd_clip} / GoalAlign {ga_clip}'
              f' of {rec["n_states"]}')
    return out


def baseline_report():
    hdr('C2-NAV.21 baseline -- C2-NAV.20 reproduced through this module')
    rows = evaluate(BASELINE)
    s = summarize(BASELINE, rows)
    for k, v in s.items():
        if k != 'meta':
            print(f'  {k:<32} {v}')
    return s


def dump(out_path):
    payload = {
        'experiment': 'C2-NAV.21',
        'stage': 'offline mechanism decomposition',
        'params_file': os.path.basename(c20.PARAMS_FILE),
        'params_sha256': c20.PARAMS_SHA256,
        'source_run': c20.BAD, 'source_leg': c20.LEG,
        'self_test_passed': self_test(),
        'selection_validation': validate_selection(),
        'baseline': baseline_report(),
        'residual': rotation_residual(),
        'oscillation': oscillation_report(),
        'matrix': matrix(),
        'bounds': bounds_report(),
        'candidates': [{'name': c['name'], 'config': dict(c), 'meta': m}
                       for c, m in candidates()],
    }
    with open(out_path, 'w') as f:
        json.dump(payload, f, indent=1, sort_keys=True, default=str)
    print(f'\nwrote {out_path}')


def main():
    a = sys.argv[1:] or ['all']
    cmd = a[0]
    if cmd == 'selftest':
        sys.exit(0 if self_test() else 1)
    elif cmd == 'export':
        export_states(a[1] if len(a) > 1 else STATES_FILE)
    elif cmd == 'baseline':
        baseline_report()
    elif cmd == 'matrix':
        matrix(a[1:] or None)
    elif cmd == 'detail':
        detail(a[1])
    elif cmd == 'bounds':
        bounds_report()
    elif cmd == 'selection':
        validate_selection()
    elif cmd == 'oscillation':
        oscillation_report()
    elif cmd == 'residual':
        rotation_residual()
    elif cmd == 'dump':
        dump(a[1])
    elif cmd == 'all':
        self_test()
        validate_selection()
        baseline_report()
        rotation_residual()
        oscillation_report()
        matrix()
        bounds_report()
    else:
        print(__doc__)
        sys.exit(2)


if __name__ == '__main__':
    main()
