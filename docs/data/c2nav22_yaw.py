#!/usr/bin/env python3
"""C2-NAV.22 -- reconstruct the terminal yaw settle from recorded traces.

C2-NAV.21 measured that the enclosure leg's terminal rotation is 47-84 %
of the leg and that a single final heading can cost 200-1124 degrees of
yaw travel. It did not measure WHY. This module answers that from the
committed traces alone; it starts no simulator, writes no parameter and
imports no ROS.

WHAT THE TARGET YAW IS, AND WHY IT IS A CONSTANT
------------------------------------------------
Established from source, not assumed:

1. `nav_bench.py` sends every goal pose -- `NavigateToPose` (:706) and
   every `NavigateThroughPoses` pose (:793) -- with
   `orientation.w = 1.0`, i.e. yaw 0. There is no per-leg yaw.
2. `SmacPlanner2D::createPlan` with `use_final_approach_orientation:
   false` (the frozen baseline) ends with
   `plan.poses.back().pose.orientation = goal.pose.orientation`
   (smac_planner_2d.cpp:344-345). Every other branch -- the
   same-cell corner case (:267) and the pose loop (:256-259) -- also
   leaves w=1.0.
3. `ControllerServer::setPlannerPath` takes `end_pose_ =
   path.poses.back()` (controller_server.cpp:635) and
   `isGoalReached()` scores the robot against THAT (:816-818).
4. `DWBLocalPlanner::prepareGlobalPlan` takes `goal_pose.pose =
   global_plan_.poses.back()` (dwb_local_planner.cpp:296) and hands it
   to every critic's `prepare()` (:322), which is where
   `RotateToGoalCritic::goal_yaw_` comes from.

So the goal checker and DWB settle on the SAME yaw, it is 0 rad, and it
is constant for the whole leg. `--goal-yaw` exists only to test that
conclusion's sensitivity, not to fit it.

Because the target is 0 and the trace's yaw is already wrapped to
(-pi, pi], the signed yaw error is exactly `-yaw` and `|yaw_err| =
|yaw|`. A robot at yaw +/-pi is at the MAXIMUM possible heading error
and sits on the cusp where the two turn directions cost the same.

WHAT THE THREE ARRIVAL CONDITIONS ARE
-------------------------------------
The brief asks these be kept apart, and they are three different
thresholds owned by three different pieces of code:

A. POSITIONAL ARRIVAL, outer:  dist <= 0.25 m, `SimpleGoalChecker.
   xy_goal_tolerance`. This is nav_bench's own transit/terminal split.
A'. POSITIONAL ARRIVAL, inner: dist <= 0.05 m, `FollowPath.
   xy_goal_tolerance`. `RotateToGoalCritic` reads THIS one
   (rotate_to_goal.cpp:60-62, `searchAndGetParam(dwb_plugin_name_ +
   ".xy_goal_tolerance", 0.25)`) and latches `in_window_` on it.
B. HEADING ARRIVAL:            |yaw_err| <= 0.25 rad, `SimpleGoalChecker.
   yaw_goal_tolerance`.
C. SETTLEMENT:                 the last sample with |w_act| above the
   noise floor.

THE FRAME CAVEAT, WHICH BOUNDS EVERY ABSOLUTE HEADING NUMBER HERE
-----------------------------------------------------------------
The trace's `yaw`, `x` and `y` are GROUND TRUTH, from Gazebo's
`/model/coco/odometry`. The goal checker and every DWB critic are fed
`costmap_ros_->getRobotPose()` -- the AMCL `map -> base_link` estimate.
They are not the same pose, and the difference is NOT recorded in any
committed artifact.

It is not small, and it is measured here rather than assumed
(`ordinary`): across the 18 ORDINARY legs of three baseline tours --
legs with no terminal pathology at all, settling in 0.8 to 7.8 s -- the
controller stops commanding while the ground-truth heading is still
**0.194 to 0.492 rad** from the goal yaw, and all 18 report SUCCEEDED.
Only 3 of 21 SUCCEEDED legs in those tours end inside the tolerance in
ground truth. A leg cannot pass a 0.25 rad yaw check at a true error of
0.49 rad, so the checker is scoring a heading this trace does not
contain.

Consequences, applied throughout:

* Absolute heading error, `t_arrive_heading` and anything derived from
  them are GROUND-TRUTH quantities and are labelled as such. This module
  never claims the controller's own heading error.
* Yaw TRAVEL, sign reversals and turn direction are frame-independent --
  a rotation is a rotation -- and carry the weight of the argument.
* Whether the controller believed it was inside `FollowPath.
  xy_goal_tolerance` is read from `dwb_ill_rot`, the critic's OWN
  per-cycle rejection count, not from ground-truth distance.
  `RotateToGoalCritic` rejects every translating trajectory once
  `rotating_` latches, so on this 819-trajectory lattice the count is
  bimodal -- exactly 779 (1811 samples) or exactly 0 (5489), with no
  third value across the 7300 terminal samples of the five runs that
  record the column -- and 0 means `in_window_` was false.

USAGE
-----
    python3 -P docs/data/c2nav22_yaw.py selftest
    python3 -P docs/data/c2nav22_yaw.py phases
    python3 -P docs/data/c2nav22_yaw.py table
    python3 -P docs/data/c2nav22_yaw.py attribution
    python3 -P docs/data/c2nav22_yaw.py ordinary
    python3 -P docs/data/c2nav22_yaw.py latchstate
    python3 -P docs/data/c2nav22_yaw.py leg c2n21_base_r4
    python3 -P docs/data/c2nav22_yaw.py chain
    python3 -P docs/data/c2nav22_yaw.py latch
    python3 -P docs/data/c2nav22_yaw.py counterfactuals
    python3 -P docs/data/c2nav22_yaw.py dump docs/data/c2nav22_yaw.json
"""

import csv
import json
import math
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WT = os.path.abspath(os.path.join(HERE, '..', '..'))
SCRATCH = os.path.join(WT, '.navbench', 'results')
# Original CSV header per tag, filled by load_trace.
# Column PRESENCE is a schema fact; column VALUES are
# measurements. Never infer the first from the second.
_SCHEMA = {}
BUNDLE = os.path.join(HERE, 'c2nav22_yaw.json')
LEG = 'enclosure_entry'

# --- configuration constants, all from the frozen C2-NAV.20 baseline ---
GOAL_WORLD = (-3.575, 2.95)      # c2n21_matrix.sh --goal override
GOAL_YAW = 0.0                   # established in the docstring above
OUTER_XY_TOL = 0.25              # SimpleGoalChecker.xy_goal_tolerance
INNER_XY_TOL = 0.05              # FollowPath.xy_goal_tolerance
YAW_TOL = 0.25                   # SimpleGoalChecker.yaw_goal_tolerance
OSC_RESET_ANGLE = 0.2            # dwb OscillationCritic default
OSC_RESET_DIST = 0.05            # dwb OscillationCritic default
X_ONLY_THRESHOLD = 0.05          # dwb OscillationCritic default
# Ground-truth angular noise floor. The Gazebo twist is not exactly zero
# on a stationary robot; every rate statistic below uses this deadband
# and `sensitivity` reports what changes if it is moved.
W_DEADBAND = 0.02
FULL_WZ_BAN = 400                # one wz sign of the 20 x 41 - 1 lattice
ROT_BLOCK = 779                  # every translating trajectory of 819
# Above this the correct turn direction is unambiguous even allowing the
# whole 0.492 rad ground-truth-vs-localisation offset measured on the
# ordinary legs.
SIGN_CUT = 0.6

# The runs C2-NAV.21 published a terminal split for. `frac` and `travel`
# are its committed numbers and are what `selftest` must reproduce.
KNOWN = {
    'c2n18_tour_r1':  dict(leg=64.93,  transit=23.72, terminal=41.21,
                           frac=0.635, travel=3.487,  status='SUCCEEDED'),
    'c2n18_tour_r2':  dict(leg=70.05,  transit=24.01, terminal=46.04,
                           frac=0.657, travel=4.998,  status='SUCCEEDED'),
    'c2n18_tour_r3':  dict(leg=103.76, transit=42.51, terminal=61.25,
                           frac=0.590, travel=4.400,  status='SUCCEEDED'),
    'c2n21_base_r1':  dict(leg=177.12, transit=93.39, terminal=83.73,
                           frac=0.473, travel=11.242, status='SUCCEEDED'),
    'c2n21_base_r3':  dict(leg=71.54,  transit=23.45, terminal=48.09,
                           frac=0.672, travel=4.236,  status='SUCCEEDED'),
    'c2n21_base_r4':  dict(leg=194.17, transit=42.27, terminal=151.90,
                           frac=0.782, travel=19.610, status='SUCCEEDED'),
    'c2n21_fpd_r3':   dict(leg=201.26, transit=81.56, terminal=119.70,
                           frac=0.595, travel=3.104,  status='TIMEOUT'),
    'c2n21_bbase_r2': dict(leg=202.35, transit=32.85, terminal=169.50,
                           frac=0.838, travel=7.077,  status='TIMEOUT'),
    'c2n21_bbase_r3': dict(leg=192.81, transit=35.84, terminal=156.97,
                           frac=0.814, travel=6.197,  status='SUCCEEDED'),
    # The only baseline tour that never reached the xy tolerance at all.
    'c2n19_tour_r1':  dict(leg=201.36, transit=None, terminal=0.0,
                           frac=None, travel=None,   status='TIMEOUT'),
}
ORDER = ['c2n18_tour_r1', 'c2n18_tour_r2', 'c2n18_tour_r3',
         'c2n21_base_r1', 'c2n21_base_r3', 'c2n21_base_r4',
         'c2n21_fpd_r3', 'c2n21_bbase_r2', 'c2n21_bbase_r3',
         'c2n19_tour_r1']


# ---------------------------------------------------------------- utils
def hdr(t):
    print()
    print('=' * 78)
    print(t)
    print('=' * 78)


def wrap(a):
    """Normalise to (-pi, pi]. The same arithmetic as angles::
    normalize_angle, which is what every nav2 comparison here uses."""
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def shortest(frm, to):
    """angles::shortest_angular_distance(frm, to)."""
    return wrap(to - frm)


def fl(v):
    if v is None or v == '':
        return None
    try:
        return float(v)
    except ValueError:
        return None


def med(xs):
    return statistics.median(xs) if xs else None


def sgn(v, dead=0.0):
    if v is None:
        return 0
    if v > dead:
        return 1
    if v < -dead:
        return -1
    return 0


# --------------------------------------------------------------- loader
def _trace_path(tag):
    return os.path.join(SCRATCH, f'{tag}_traces', f'{LEG}_rep0.csv')


def load_trace(tag):
    """Rows of the enclosure leg's 10 Hz trace, from scratch or bundle.

    Returns a list of dicts with float-or-None values. The C2-NAV.21
    degeneracy columns are absent from older runs; they come back as
    None rather than 0, because a blank is a missing measurement and a
    zero is a claim.
    """
    cols = ('t_rel', 'x', 'y', 'yaw', 'v_act', 'w_act', 'v_nav', 'w_nav',
            'v_smoothed', 'v_cmdvel', 'v_wheel', 'scan_min',
            'dwb_best_vx', 'dwb_best_wz', 'dwb_margin', 'dwb_n_at_min',
            'dwb_ill_osc', 'dwb_ill_base', 'dwb_ill_rot')
    strc = ('cm_action', 'cm_polygon')
    p = _trace_path(tag)
    if os.path.exists(p):
        out = []
        with open(p) as f:
            rd = csv.DictReader(f)
            _SCHEMA[tag] = set(rd.fieldnames or ())
            for r in rd:
                d = {c: fl(r.get(c)) for c in cols}
                for c in strc:
                    v = r.get(c)
                    d[c] = v if v else None
                out.append(d)
        return out
    b = _bundle()
    if tag in b.get('traces', {}):
        _SCHEMA[tag] = set(b['traces'][tag].get('schema')
                           or b['traces'][tag]['columns'])
        keys = b['traces'][tag]['columns']
        return [dict(zip(keys, row)) for row in b['traces'][tag]['rows']]
    raise SystemExit(f'no trace for {tag}: neither {p} nor a bundle entry')


_BUNDLE_CACHE = {}


def _bundle():
    if not _BUNDLE_CACHE:
        if os.path.exists(BUNDLE):
            with open(BUNDLE) as f:
                _BUNDLE_CACHE.update(json.load(f))
        else:
            _BUNDLE_CACHE['traces'] = {}
    return _BUNDLE_CACHE


def load_record(tag):
    """The enclosure leg's nav_bench record, if the scratch run survives."""
    p = os.path.join(SCRATCH, f'{tag}.json')
    if os.path.exists(p):
        with open(p) as f:
            d = json.load(f)
        for r in d.get('legs', []):
            if r.get('scenario') == LEG:
                return r
    return _bundle().get('records', {}).get(tag)


# ------------------------------------------------------------- analysis
def analyse(tag, goal_yaw=GOAL_YAW, dead=W_DEADBAND):
    """Every C2-NAV.22 statistic for one enclosure leg."""
    rows = [r for r in load_trace(tag) if r.get('yaw') is not None
            and r.get('x') is not None]
    if not rows:
        raise SystemExit(f'{tag}: trace has no pose samples')
    gx, gy = GOAL_WORLD
    t = [r['t_rel'] for r in rows]
    yaw = [r['yaw'] for r in rows]
    dist = [math.hypot(r['x'] - gx, r['y'] - gy) for r in rows]
    # Signed heading error, wrapped exactly as SimpleGoalChecker does.
    err = [shortest(y, goal_yaw) for y in yaw]

    # --- A / A' positional arrival -----------------------------------
    t_out = next((t[i] for i in range(len(t)) if dist[i] <= OUTER_XY_TOL),
                 None)
    t_in = next((t[i] for i in range(len(t)) if dist[i] <= INNER_XY_TOL),
                None)
    t_end = t[-1]

    res = dict(tag=tag, n_samples=len(rows), t_end=t_end,
               t_arrive_outer=t_out, t_arrive_inner=t_in,
               dist_min=min(dist), dist_final=dist[-1],
               yaw_final=yaw[-1], err_final=err[-1],
               goal_yaw=goal_yaw)
    if t_out is None:
        res['note'] = 'never reached the outer xy tolerance'
        return res

    i0 = t.index(t_out)
    term = list(range(i0, len(rows)))

    # --- B heading arrival, and how often it was LOST again ----------
    t_head = None
    n_reexit = 0
    inside = False
    for i in term:
        if abs(err[i]) <= YAW_TOL:
            if t_head is None:
                t_head = t[i]
            inside = True
        else:
            if inside:
                n_reexit += 1
            inside = False
    res['t_arrive_heading'] = t_head
    res['n_heading_window_exits'] = n_reexit
    res['err_at_arrival'] = err[i0]
    res['err_abs_max_terminal'] = max(abs(err[i]) for i in term)

    # --- C settlement -------------------------------------------------
    t_settle = None
    for i in reversed(term):
        w = rows[i].get('w_act')
        if w is not None and abs(w) > dead:
            t_settle = t[i]
            break
    res['t_settle'] = t_settle

    # --- yaw travel ---------------------------------------------------
    # Absolute travel is summed on the WRAPPED per-sample delta, which is
    # what nav_bench does; signed travel is the same deltas kept signed,
    # so `abs - |signed|` is exactly the wasted rotation.
    d_abs = d_sig = 0.0
    for i in term[:-1]:
        d = shortest(yaw[i], yaw[i + 1])
        d_abs += abs(d)
        d_sig += d
    res['yaw_travel_abs'] = d_abs
    res['yaw_travel_signed'] = d_sig
    res['yaw_travel_wasted'] = d_abs - abs(d_sig)
    # The direct cost of the heading the robot actually had on arrival.
    res['yaw_needed'] = abs(err[i0])
    res['travel_ratio'] = (d_abs / abs(err[i0])) if abs(err[i0]) > 1e-6 \
        else None

    # --- sign structure ----------------------------------------------
    def flips(key, deadband):
        vals = [rows[i].get(key) for i in term]
        n, last = 0, 0
        for v in vals:
            s = sgn(v, deadband)
            if s == 0:
                continue
            if last and s != last:
                n += 1
            last = s
        return n

    res['n_sign_flips_w_act'] = flips('w_act', dead)
    res['n_sign_flips_w_nav'] = flips('w_nav', dead)
    res['n_sign_flips_dwb_best_wz'] = (
        flips('dwb_best_wz', 0.0)
        if any(rows[i].get('dwb_best_wz') is not None for i in term)
        else None)

    # Zero crossings of the heading error: each one is an overshoot of
    # the goal heading itself, not merely a change of turn direction.
    n_cross = 0
    last = sgn(err[i0])
    for i in term:
        s = sgn(err[i])
        if s and last and s != last:
            n_cross += 1
        if s:
            last = s
    res['n_err_zero_crossings'] = n_cross

    # Monotonic convergence? Count samples where |err| grows.
    grow = sum(1 for i in term[:-1] if abs(err[i + 1]) > abs(err[i]) + 1e-9)
    res['frac_samples_err_growing'] = grow / max(1, len(term) - 1)

    # --- the +/-pi cusp ------------------------------------------------
    # |err| > pi - OSC_RESET_ANGLE is the band where the two turn
    # directions are within one Oscillation reset of each other.
    cusp = [i for i in term if abs(err[i]) > math.pi - OSC_RESET_ANGLE]
    res['n_samples_near_pi'] = len(cusp)
    res['frac_terminal_near_pi'] = len(cusp) / len(term)
    res['t_first_near_pi'] = t[cusp[0]] if cusp else None
    # Actual wraps of the recorded yaw across +/-pi.
    res['n_pi_wraps'] = sum(
        1 for i in term[:-1]
        if abs(yaw[i]) > 2.0 and abs(yaw[i + 1]) > 2.0
        and yaw[i] * yaw[i + 1] < 0)

    # --- command chain -------------------------------------------------
    res['chain'] = chain_stats(rows, term, dead)

    # --- DWB rejections in the terminal window -------------------------
    for key, out in (('dwb_ill_osc', 'osc'), ('dwb_ill_rot', 'rot'),
                     ('dwb_ill_base', 'base')):
        vals = [rows[i].get(key) for i in term
                if rows[i].get(key) is not None]
        res[f'ill_{out}_samples'] = len(vals)
        res[f'ill_{out}_at_full_ban'] = sum(1 for v in vals
                                            if v >= FULL_WZ_BAN)
    return res


def chain_stats(rows, term, dead):
    """Commanded vs achieved angular rate, split by collision-monitor
    action. The monitor's `slowdown` multiplies the whole twist by
    `slowdown_ratio`, so if it is throttling the rotation the ratio
    |w_act| / |w_nav| separates by `cm_action`."""
    buckets = {}
    for i in term:
        r = rows[i]
        wn, wa = r.get('w_nav'), r.get('w_act')
        if wn is None or wa is None or abs(wn) < dead:
            continue
        key = r.get('cm_polygon') or (r.get('cm_action') or 'none')
        b = buckets.setdefault(key, {'ratio': [], 'wn': [], 'wa': []})
        b['ratio'].append(abs(wa) / abs(wn))
        b['wn'].append(abs(wn))
        b['wa'].append(abs(wa))
    out = {}
    for k, b in buckets.items():
        out[k] = dict(n=len(b['ratio']),
                      ratio_med=med(b['ratio']),
                      w_nav_med=med(b['wn']),
                      w_act_med=med(b['wa']))
    return out


# ------------------------------------------------------- latch emulator
def latch(tag, dead=W_DEADBAND):
    """Replay dwb's OscillationCritic state machine over the recorded
    commands, exactly as `debrief()` runs it.

    This is a REPLAY, not a measurement: the critic is driven by the
    command DWB selected each control cycle and by the pose it held when
    it prepared, both of which the trace records at 10 Hz against the
    controller's 10 Hz. Where the trace also carries the critic's own
    illegal count (`dwb_ill_osc`, C2-NAV.21 runs only) the replay is
    scored against it rather than trusted.
    """
    rows = [r for r in load_trace(tag) if r.get('yaw') is not None
            and r.get('x') is not None]
    gx, gy = GOAL_WORLD
    t_out = None
    for r in rows:
        if math.hypot(r['x'] - gx, r['y'] - gy) <= OUTER_XY_TOL:
            t_out = r['t_rel']
            break
    if t_out is None:
        return None

    sign = 0                      # theta_trend_.sign_
    pos_only = neg_only = False
    prev_pose = None
    n_ban = n_reset = 0
    banned_samples = 0
    total = 0
    agree = disagree = 0
    ban_runs = []
    run_start = None
    for r in rows:
        if r['t_rel'] < t_out:
            continue
        # `setOscillationFlags` only updates the theta trend while the
        # commanded |vx| is at or below x_only_threshold (0.05).
        vx = r.get('v_nav')
        wz = r.get('w_nav')
        if wz is None or vx is None:
            continue
        total += 1
        pose = (r['x'], r['y'], r['yaw'])
        flag_set = False
        if abs(vx) <= X_ONLY_THRESHOLD:
            s = sgn(wz)
            if s < 0 and sign > 0:
                neg_only, flag_set = True, True
            elif s > 0 and sign < 0:
                pos_only, flag_set = True, True
            if s:
                sign = s
        if flag_set:
            prev_pose = pose
            n_ban += 1
            if run_start is None:
                run_start = r['t_rel']
        if (pos_only or neg_only) and prev_pose is not None:
            # resetAvailable(): translation OR rotation past the limits.
            dx, dy = pose[0] - prev_pose[0], pose[1] - prev_pose[1]
            # NOTE: dwb compares the RAW theta difference, unwrapped
            # (oscillation.cpp:190). Reproduced here deliberately.
            dth = pose[2] - prev_pose[2]
            if (dx * dx + dy * dy) > OSC_RESET_DIST ** 2 or \
                    abs(dth) > OSC_RESET_ANGLE:
                pos_only = neg_only = False
                sign = 0
                n_reset += 1
                if run_start is not None:
                    ban_runs.append(r['t_rel'] - run_start)
                    run_start = None
        if pos_only or neg_only:
            banned_samples += 1
        # Score the replay against the critic's own count when present.
        obs = r.get('dwb_ill_osc')
        if obs is not None:
            pred = pos_only or neg_only
            if pred == (obs >= FULL_WZ_BAN):
                agree += 1
            else:
                disagree += 1
    return dict(tag=tag, samples=total, n_ban_events=n_ban,
                n_resets=n_reset, banned_samples=banned_samples,
                banned_frac=banned_samples / total if total else None,
                ban_run_med=med(ban_runs), ban_run_max=max(ban_runs)
                if ban_runs else None,
                replay_agree=agree, replay_disagree=disagree,
                replay_accuracy=(agree / (agree + disagree))
                if (agree + disagree) else None)


# --------------------------------------------------------------- report
def _row(a):
    def f(k, n=2):
        v = a.get(k)
        return '-' if v is None else f'{v:.{n}f}'
    return (f'{a["tag"]:<16} {f("t_arrive_outer"):>8} '
            f'{f("t_arrive_inner"):>8} {f("t_arrive_heading"):>8} '
            f'{f("t_settle"):>8} {f("t_end"):>8} '
            f'{f("yaw_travel_abs",3):>9} {f("yaw_travel_signed",3):>9} '
            f'{f("yaw_needed",3):>7} '
            f'{a.get("n_sign_flips_w_act",0):>6} '
            f'{a.get("n_err_zero_crossings",0):>6} '
            f'{a.get("n_heading_window_exits",0):>6}')


def cmd_table(tags=None):
    hdr('C2-NAV.22 -- terminal yaw, every enclosure leg with a trace')
    print(f'goal {GOAL_WORLD} yaw {GOAL_YAW}; outer xy {OUTER_XY_TOL} m, '
          f'inner xy {INNER_XY_TOL} m, yaw {YAW_TOL} rad')
    print()
    print(f'{"tag":<16} {"t_xy.25":>8} {"t_xy.05":>8} {"t_head":>8} '
          f'{"t_settle":>8} {"t_end":>8} {"|yaw|":>9} {"net yaw":>9} '
          f'{"needed":>7} {"wflip":>6} {"cross":>6} {"exits":>6}')
    print('-' * 110)
    out = []
    for tag in (tags or ORDER):
        try:
            a = analyse(tag)
        except SystemExit as e:
            print(f'{tag:<16} -- {e}')
            continue
        out.append(a)
        if a.get('note'):
            print(f'{a["tag"]:<16} {a["note"]}')
            continue
        print(_row(a))
    print()
    print('t_xy.25  outer positional arrival (SimpleGoalChecker)')
    print('t_xy.05  inner positional arrival (FollowPath, RotateToGoal '
          'in_window_ latch)')
    print('t_head   first sample inside yaw_goal_tolerance after t_xy.25')
    print('t_settle last sample with |w_act| > %.2f rad/s' % W_DEADBAND)
    print('|yaw|    absolute yaw travelled after t_xy.25 (rad)')
    print('needed   |heading error| at t_xy.25 -- the whole job (rad)')
    print('wflip    sign changes of the ACHIEVED angular rate')
    print('cross    times the heading error crossed zero (overshoots)')
    print('exits    times the robot left the yaw tolerance after '
          'entering it')
    return out


def cmd_leg(tag):
    a = analyse(tag)
    hdr(f'C2-NAV.22 -- {tag}')
    k = KNOWN.get(tag, {})
    if k:
        print(f'C2-NAV.21 published: leg {k["leg"]} s, transit '
              f'{k["transit"]} s, terminal {k["terminal"]} s, '
              f'travel {k["travel"]} rad, {k["status"]}')
    for key in ('n_samples', 't_end', 't_arrive_outer', 't_arrive_inner',
                't_arrive_heading', 't_settle', 'dist_min', 'dist_final',
                'yaw_final', 'err_at_arrival', 'err_final',
                'err_abs_max_terminal', 'yaw_needed', 'yaw_travel_abs',
                'yaw_travel_signed', 'yaw_travel_wasted', 'travel_ratio',
                'n_sign_flips_w_act', 'n_sign_flips_w_nav',
                'n_sign_flips_dwb_best_wz', 'n_err_zero_crossings',
                'n_heading_window_exits', 'frac_samples_err_growing',
                'n_samples_near_pi', 'frac_terminal_near_pi',
                't_first_near_pi', 'n_pi_wraps', 'ill_osc_at_full_ban',
                'ill_rot_samples', 'ill_base_samples'):
        v = a.get(key)
        if isinstance(v, float):
            v = round(v, 4)
        print(f'  {key:<28} {v}')
    print('  command chain by collision-monitor polygon:')
    for k2, v in sorted(a['chain'].items()):
        print(f'    {k2:<16} n={v["n"]:<6} |w_act|/|w_nav| med='
              f'{v["ratio_med"]:.3f}  w_nav med={v["w_nav_med"]:.4f}'
              f'  w_act med={v["w_act_med"]:.4f}')
    return a


def cmd_chain():
    hdr('C2-NAV.22 -- commanded vs achieved angular rate, terminal window')
    print('The collision monitor\'s SLOWDOWN multiplies the whole twist by '
          'slowdown_ratio 0.3.')
    print('If it were throttling the rotation, |w_act|/|w_nav| would '
          'separate by polygon.')
    print()
    print(f'{"tag":<16} {"polygon":<16} {"n":>6} {"ratio":>8} '
          f'{"w_nav":>8} {"w_act":>8}')
    print('-' * 70)
    for tag in ORDER:
        try:
            a = analyse(tag)
        except SystemExit:
            continue
        if a.get('note'):
            continue
        for k, v in sorted(a['chain'].items()):
            print(f'{tag:<16} {k:<16} {v["n"]:>6} {v["ratio_med"]:>8.3f} '
                  f'{v["w_nav_med"]:>8.4f} {v["w_act_med"]:>8.4f}')


def cmd_latch():
    hdr('C2-NAV.22 -- OscillationCritic latch, replayed over the '
        'terminal window')
    print('dwb defaults: reset_dist 0.05 m, reset_angle 0.2 rad, '
          'x_only_threshold 0.05.')
    print('`accuracy` scores the replay against the critic\'s OWN '
          'per-cycle illegal count')
    print('(dwb_ill_osc >= 400 = a full one-sign ban); it is blank for '
          'runs recorded before C2-NAV.21.')
    print()
    print(f'{"tag":<16} {"samples":>8} {"bans":>6} {"resets":>7} '
          f'{"banned%":>8} {"run_med":>8} {"run_max":>8} {"accuracy":>9}')
    print('-' * 78)
    for tag in ORDER:
        try:
            r = latch(tag)
        except SystemExit:
            continue
        if not r:
            continue
        acc = ('-' if r['replay_accuracy'] is None
               else f'{r["replay_accuracy"]:.3f}')
        print(f'{r["tag"]:<16} {r["samples"]:>8} {r["n_ban_events"]:>6} '
              f'{r["n_resets"]:>7} {r["banned_frac"]:>8.3f} '
              f'{(r["ban_run_med"] or 0):>8.2f} '
              f'{(r["ban_run_max"] or 0):>8.2f} {acc:>9}')


# ------------------------------------------- phases, latch, attribution
def _prep(tag):
    """Rows with a pose, plus the two positional-arrival indices."""
    rows = [r for r in load_trace(tag) if r.get('yaw') is not None
            and r.get('x') is not None]
    gx, gy = GOAL_WORLD
    d = [math.hypot(r['x'] - gx, r['y'] - gy) for r in rows]
    i_out = next((i for i in range(len(rows)) if d[i] <= OUTER_XY_TOL),
                 None)
    i_in = next((i for i in range(len(rows)) if d[i] <= INNER_XY_TOL),
                None)
    return rows, d, i_out, i_in


def _has_latch(rows, tag=None):
    """Does this run RECORD RotateToGoal's per-cycle rejection count?

    Answered from the trace's SCHEMA, never from its values.
    `dwb_ill_rot` is blank on any cycle the critic rejected nothing, so
    a value-based probe cannot tell a run predating the C2-NAV.21
    columns from one where the critic never fired at all -- and it
    reported `c2n21_fpd_r3`, whose column is present and blank on every
    row, as the former. It is the latter, and that is evidence, not a
    missing measurement: a leg that arrived and never once entered
    rotate-in-place.

    This repo's own rule, turned on its own instrument: a check whose
    success condition is "we saw nothing" must first prove it can see
    something.
    """
    if tag is not None and tag in _SCHEMA:
        return 'dwb_ill_rot' in _SCHEMA[tag]
    return any(r.get('dwb_ill_rot') is not None for r in rows)


def _latched(r):
    """Did RotateToGoalCritic reject the translating block this cycle?

    None when the run predates the C2-NAV.21 per-cycle columns.
    """
    v = r.get('dwb_ill_rot')
    return None if v is None else v > 0


def _travel(rows, a, b):
    return sum(abs(shortest(rows[i]['yaw'], rows[i + 1]['yaw']))
               for i in range(a, min(b, len(rows) - 1)))


def _pathlen(rows, a, b):
    return sum(math.dist((rows[i]['x'], rows[i]['y']),
                         (rows[i + 1]['x'], rows[i + 1]['y']))
               for i in range(a, min(b, len(rows) - 1)))


def cmd_phases():
    hdr('C2-NAV.22 -- the leg has THREE phases, not two')
    print('C2-NAV.21 split the leg at the goal checker\'s 0.25 m and '
          'called everything')
    print('after it "terminal rotation". RotateToGoalCritic latches on '
          'FollowPath.')
    print('xy_goal_tolerance = 0.05 m instead, so between the two the '
          'robot is neither')
    print('transiting nor rotating in place: it is CREEPING the last '
          '200 mm.')
    print()
    print(f'{"tag":<16} {"transit":>8} {"CREEP":>8} {"rotate":>8} '
          f'{"leg":>8} {"creep%":>7} {"rot%":>6} {"creep_m":>8} '
          f'{"creep_v":>8} {"yaw_creep":>10} {"yaw_rot":>8} {"min_d":>7}')
    print('-' * 116)
    for tag in ORDER:
        try:
            rows, d, i_out, i_in = _prep(tag)
        except SystemExit:
            continue
        if i_out is None:
            print(f'{tag:<16} never reached the 0.25 m outer tolerance')
            continue
        end = rows[-1]['t_rel']
        t_out = rows[i_out]['t_rel']
        if i_in is None:
            i_in, t_in = len(rows) - 1, end
        else:
            t_in = rows[i_in]['t_rel']
        creep, rot = t_in - t_out, end - t_in
        cm = _pathlen(rows, i_out, i_in)
        print(f'{tag:<16} {t_out:>8.2f} {creep:>8.2f} {rot:>8.2f} '
              f'{end:>8.2f} {creep/end:>7.3f} {rot/end:>6.3f} '
              f'{cm:>8.3f} {(cm/creep if creep else 0):>8.4f} '
              f'{_travel(rows,i_out,i_in):>10.3f} '
              f'{_travel(rows,i_in,len(rows)-1):>8.3f} {min(d):>7.3f}')
    print()
    print('transit    0 -> first sample within 0.25 m of the goal')
    print('CREEP      0.25 m -> first sample within 0.05 m')
    print('rotate     0.05 m -> end of leg')
    print('creep_v    mean speed over the creep, m/s')
    print('yaw_*      yaw travelled in that phase, rad')
    print()
    print('And the same split read from the CONTROLLER\'s own state '
          'instead of from')
    print('ground-truth distance -- `dwb_ill_rot` > 0 is a cycle in '
          'which RotateToGoal')
    print('rejected the translating block, i.e. it believed it was '
          'inside 0.05 m:')
    print()
    print(f'{"tag":<16} {"terminal cycles":>16} {"latched":>9} '
          f'{"latched%":>9} {"yaw latched":>12} {"yaw unlatched":>14}')
    print('-' * 82)
    for tag in ORDER:
        try:
            rows, d, i_out, i_in = _prep(tag)
        except SystemExit:
            continue
        if i_out is None or not _has_latch(rows, tag):
            continue
        term = rows[i_out:]
        lat = [_latched(r) for r in term]
        yl = yu = 0.0
        for i in range(len(term) - 1):
            dd = abs(shortest(term[i]['yaw'], term[i + 1]['yaw']))
            if lat[i]:
                yl += dd
            else:
                yu += dd
        n = len(term)
        print(f'{tag:<16} {n:>16} {sum(1 for v in lat if v):>9} '
              f'{sum(1 for v in lat if v)/n:>9.3f} {yl:>12.3f} '
              f'{yu:>14.3f}')


def cmd_latchstate():
    hdr('C2-NAV.22 -- how long RotateToGoal stays OFF, and what it costs')
    print('`in_window_` is a latch: `in_window_ = in_window_ || dxy_sq '
          '<= tol^2`. The')
    print('ONLY thing that clears it is `reset()`, and the only caller '
          'is')
    print('`DWBLocalPlanner::setPlan`, which the controller runs '
          'whenever the BT hands')
    print('it a path it does not already have '
          '(dwb_local_planner.cpp:238-246). After a')
    print('reset it re-latches in the same cycle IF the controller\'s '
          'own distance to')
    print('the plan\'s last pose is within 0.05 m. An episode lasting '
          'many plan')
    print('periods therefore says the controller did NOT think it was '
          'within 0.05 m.')
    print()
    print(f'{"tag":<16} {"rot_s":>7} {"off%":>6} {"n_off":>6} '
          f'{"off_med":>8} {"off_max":>8} {"on_med":>7} {"plan_per":>9} '
          f'{"yaw_off":>8} {"yaw_on":>7}')
    print('-' * 92)
    for tag in ORDER:
        try:
            rows, d, i_out, i_in = _prep(tag)
        except SystemExit:
            continue
        if i_in is None or not _has_latch(rows, tag):
            continue
        term = rows[i_in:]
        if len(term) < 5:
            continue
        lat = [_latched(r) for r in term]
        off, on = [], []
        yo = yn = 0.0
        start, cur = term[0]['t_rel'], lat[0]
        for i in range(1, len(term)):
            dd = abs(shortest(term[i - 1]['yaw'], term[i]['yaw']))
            if lat[i - 1]:
                yn += dd
            else:
                yo += dd
            if lat[i] != cur:
                (on if cur else off).append(term[i]['t_rel'] - start)
                start, cur = term[i]['t_rel'], lat[i]
        (on if cur else off).append(term[-1]['t_rel'] - start)
        per = _plan_period(tag)
        n = len(lat)
        print(f'{tag:<16} {term[-1]["t_rel"]-term[0]["t_rel"]:>7.1f} '
              f'{sum(1 for v in lat if not v)/n:>6.3f} {len(off):>6} '
              f'{(med(off) or 0):>8.2f} {(max(off) if off else 0):>8.2f} '
              f'{(med(on) or 0):>7.2f} {(per or 0):>9.3f} '
              f'{yo:>8.3f} {yn:>7.3f}')
    print()
    print('off%      fraction of rotate-phase cycles with RotateToGoal '
          'inactive')
    print('off_*     duration of one uninterrupted OFF episode, s')
    print('plan_per  median /plan republication period on this leg '
          '(RateController 0.333 Hz)')
    print('yaw_*     yaw travelled with the critic off / on, rad')


def _plan_period(tag):
    p = os.path.join(SCRATCH,
                     f'{tag}_planwindow_{LEG}_rep0.json')
    if not os.path.exists(p):
        p2 = _bundle().get('plan_periods', {}).get(tag)
        return p2
    with open(p) as f:
        ts = [s['ts_offset_from_t0_s'] for s in json.load(f)['snapshots']]
    d = [ts[i + 1] - ts[i] for i in range(len(ts) - 1)]
    return med(d)


def cmd_attribution():
    hdr('C2-NAV.22 -- when DWB turns AWAY from the goal heading, what '
        'state is it in?')
    print('Rotate-in-place cycles only. Restricted to |yaw| > '
          f'{SIGN_CUT} rad so the correct')
    print('turn direction survives the whole 0.492 rad '
          'ground-truth-vs-localisation offset')
    print('measured on the ordinary legs. One row per distinct DWB '
          'selection, so the')
    print('10 Hz carry-forward cannot inflate a count.')
    print()
    print(f'{"tag":<16} {"n":>6} {"toward":>7} {"away":>6} {"away%":>7} '
          f'{"osc ban":>8} {"unlatched":>10} {"NEITHER":>8}')
    print('-' * 76)
    tot = dict(n=0, tw=0, aw=0, osc=0, unl=0, nei=0)
    for tag in ORDER:
        try:
            rows, d, i_out, i_in = _prep(tag)
        except SystemExit:
            continue
        if i_in is None or not _has_latch(rows, tag):
            continue
        n = tw = aw = osc = unl = nei = 0
        seen = set()
        for r in rows[i_in:]:
            w = r.get('dwb_best_wz')
            if w is None or abs(w) < 1e-9 or abs(r['yaw']) <= SIGN_CUT:
                continue
            key = (round(w, 4), round(r['yaw'], 4))
            if key in seen:
                continue
            seen.add(key)
            n += 1
            if (w > 0) == (r['yaw'] < 0):
                tw += 1
                continue
            aw += 1
            if (r.get('dwb_ill_osc') or 0) >= FULL_WZ_BAN:
                osc += 1
            elif not _latched(r):
                unl += 1
            else:
                nei += 1
        if not n:
            continue
        for k, v in (('n', n), ('tw', tw), ('aw', aw), ('osc', osc),
                     ('unl', unl), ('nei', nei)):
            tot[k] += v
        print(f'{tag:<16} {n:>6} {tw:>7} {aw:>6} {aw/n:>7.3f} '
              f'{osc:>8} {unl:>10} {nei:>8}')
    print('-' * 76)
    n = tot['n'] or 1
    print(f'{"ALL":<16} {tot["n"]:>6} {tot["tw"]:>7} {tot["aw"]:>6} '
          f'{tot["aw"]/n:>7.3f} {tot["osc"]:>8} {tot["unl"]:>10} '
          f'{tot["nei"]:>8}')
    print()
    print('osc ban    that cycle carried a full one-sign Oscillation '
          f'ban (>= {FULL_WZ_BAN} illegals)')
    print('unlatched  RotateToGoal rejected nothing that cycle: its '
          'in_window_ latch was')
    print('           not set, so it scored every trajectory 0.0 and '
          'the heading was')
    print('           chosen by the path-alignment critics instead')
    print('NEITHER    RotateToGoal latched, no Oscillation ban, and DWB '
          'still turned the')
    print('           wrong way. This column is the residual the '
          'diagnosis has to explain.')
    return tot


def cmd_ordinary():
    """The control that establishes the frame caveat.

    Every leg of the benchmark is sent with `orientation.w = 1.0`, so a
    leg that SUCCEEDED should end within `yaw_goal_tolerance` of 0 if
    the checker were scoring the heading this trace records. The
    ORDINARY legs -- ones with no terminal pathology, settling in
    0.8-7.8 s -- are the clean test, because nothing else is going on.
    """
    hdr('C2-NAV.22 -- the control: where every leg of a tour actually '
        'stops')
    b = _bundle().get('ordinary', {})
    if not b:
        print('no ordinary-leg record in the bundle')
        return
    print(f'{"tag":<16} {"leg":<18} {"status":<10} {"secs":>7} '
          f'{"t_term":>7} {"yaw_end":>9} {"|yaw|<=0.25":>12}')
    print('-' * 84)
    worst = 0.0
    n_ok = n_bad = 0
    for tag in sorted(b):
        for r in b[tag]:
            y = r['yaw_end']
            inside = y is not None and abs(y) <= YAW_TOL
            if r['status'] == 'SUCCEEDED' and y is not None:
                if inside:
                    n_ok += 1
                else:
                    n_bad += 1
                    worst = max(worst, abs(y))
            print(f'{tag:<16} {r["leg"]:<18} {r["status"]:<10} '
                  f'{r["secs"]:>7.2f} {(r["t_terminal_s"] or 0):>7.1f} '
                  f'{(y if y is not None else float("nan")):>9.4f} '
                  f'{"yes" if inside else "NO":>12}')
        print()
    print(f'SUCCEEDED legs ending INSIDE the 0.25 rad yaw tolerance in '
          f'ground truth: {n_ok}')
    print(f'SUCCEEDED legs ending OUTSIDE it:                          '
          f'         {n_bad}   worst {worst:.4f} rad')
    print()
    print('A leg cannot pass a 0.25 rad yaw check at a true error of '
          f'{worst:.2f} rad. The')
    print('checker is scoring `costmap_ros_->getRobotPose()` -- the '
          'AMCL map -> base_link')
    print('estimate -- and this trace records Gazebo ground truth. '
          'They differ, the')
    print('difference is not recorded anywhere, and every absolute '
          'heading number in')
    print('this module is bounded by it.')


# --------------------------------------------------------------- gates
def cmd_selftest():
    """Refuse to report anything until the reconstruction reproduces
    C2-NAV.21's committed terminal split from the raw traces."""
    hdr('C2-NAV.22 self-test -- reproduce C2-NAV.21 from the traces')
    fails = []
    print(f'{"tag":<16} {"quantity":<22} {"committed":>10} '
          f'{"recomputed":>11} {"delta":>9}')
    print('-' * 74)
    for tag in ORDER:
        k = KNOWN[tag]
        try:
            a = analyse(tag)
        except SystemExit as e:
            fails.append(f'{tag}: {e}')
            continue
        if k['transit'] is None:
            ok = a.get('t_arrive_outer') is None
            print(f'{tag:<16} {"never reached xy tol":<22} '
                  f'{"yes":>10} {"yes" if ok else "NO":>11} {"":>9}')
            if not ok:
                fails.append(f'{tag}: expected no xy arrival, got '
                             f'{a.get("t_arrive_outer")}')
            continue
        # transit time: nav_bench's t_reach - t0, on the same 0.25 m tol.
        got = a['t_arrive_outer']
        d = got - k['transit']
        print(f'{tag:<16} {"transit s":<22} {k["transit"]:>10.2f} '
              f'{got:>11.2f} {d:>+9.2f}')
        if abs(d) > 0.35:
            fails.append(f'{tag}: transit {got} vs {k["transit"]}')
        # terminal share of the leg.
        frac = (a['t_end'] - got) / a['t_end']
        d = frac - k['frac']
        print(f'{tag:<16} {"terminal frac":<22} {k["frac"]:>10.3f} '
              f'{frac:>11.3f} {d:>+9.3f}')
        if abs(d) > 0.02:
            fails.append(f'{tag}: frac {frac:.3f} vs {k["frac"]}')
        # yaw travel. nav_bench integrates the RAW ground-truth series;
        # the trace is resampled to 10 Hz, so this reconstruction is a
        # LOWER BOUND and is asserted as one, not as equality.
        got = a['yaw_travel_abs']
        rel = got / k['travel']
        print(f'{tag:<16} {"yaw travel rad (>=85%)":<22} '
              f'{k["travel"]:>10.3f} {got:>11.3f} {rel:>8.2f}x')
        if not (0.85 <= rel <= 1.02):
            fails.append(f'{tag}: yaw travel {got:.3f} vs {k["travel"]} '
                         f'({rel:.2f}x)')
    print()
    # The instrument must be able to SEE a latch before any run is
    # allowed to report one -- the repo's own rule about checks whose
    # success condition is "we saw nothing".
    seen = 0
    for tag in ORDER:
        try:
            r = latch(tag)
        except SystemExit:
            continue
        if r and r['n_ban_events'] > 0:
            seen += 1
    print(f'latch replay reports a ban on {seen} of {len(ORDER)} legs '
          f'(must be > 0 before any zero is trusted)')
    if seen == 0:
        fails.append('latch replay never fired: cannot trust a zero')

    # And the wrap arithmetic must match angles::shortest_angular_distance.
    checks = [(3.0, -3.0, 0.2832), (-3.0, 3.0, -0.2832), (0.0, 0.0, 0.0),
              (math.pi, -math.pi, 0.0), (0.1, -0.1, -0.2)]
    for frm, to, want in checks:
        got = shortest(frm, to)
        if abs(got - want) > 1e-3:
            fails.append(f'shortest({frm},{to}) = {got}, want {want}')
    print(f'wrap arithmetic: {len(checks)} checks')

    print()
    if fails:
        print(f'SELFTEST FAILED -- {len(fails)}')
        for f in fails:
            print('  ' + f)
        return 1
    print('SELFTEST PASSED')
    return 0


def cmd_counterfactuals():
    hdr('C2-NAV.22 -- can each candidate account for the numbers?')
    rows = []
    for tag in ORDER:
        try:
            a = analyse(tag)
        except SystemExit:
            continue
        if not a.get('note'):
            rows.append(a)
    if not rows:
        print('no legs available')
        return

    print('A. FollowPath.xy_goal_tolerance 0.05 vs goal checker 0.25')
    print('   RotateToGoalCritic reads FollowPath.xy_goal_tolerance and')
    print('   latches in_window_ there. If the mismatch created the long')
    print('   terminal phase, the robot would have to reach 0.05 m EARLY')
    print('   and rotate from then on.')
    print(f'   {"tag":<16} {"t_xy.25":>8} {"t_xy.05":>8} '
          f'{"gap s":>8} {"min dist":>9} {"t_end":>8}')
    for a in rows:
        ti = a['t_arrive_inner']
        gap = (ti - a['t_arrive_outer']) if ti is not None else None
        print(f'   {a["tag"]:<16} {a["t_arrive_outer"]:>8.2f} '
              f'{("-" if ti is None else f"{ti:.2f}"):>8} '
              f'{("-" if gap is None else f"{gap:.2f}"):>8} '
              f'{a["dist_min"]:>9.3f} {a["t_end"]:>8.2f}')
    print()

    print('B. Oscillation: repeated sign changes')
    print(f'   {"tag":<16} {"wflips":>7} {"crossings":>10} '
          f'{"exits":>6} {"banned%":>8} {"full bans":>10}')
    for a in rows:
        r = latch(a['tag']) or {}
        bf = r.get('banned_frac')
        print(f'   {a["tag"]:<16} {a["n_sign_flips_w_act"]:>7} '
              f'{a["n_err_zero_crossings"]:>10} '
              f'{a["n_heading_window_exits"]:>6} '
              f'{("-" if bf is None else f"{bf:.3f}"):>8} '
              f'{a.get("ill_osc_at_full_ban", 0):>10}')
    print()

    print('C. Angle wrapping / the +/-pi cusp')
    print(f'   {"tag":<16} {"err@arrival":>12} {"|err|max":>9} '
          f'{"near-pi%":>9} {"pi wraps":>9} {"needed":>7} {"travel":>8}')
    for a in rows:
        print(f'   {a["tag"]:<16} {a["err_at_arrival"]:>12.3f} '
              f'{a["err_abs_max_terminal"]:>9.3f} '
              f'{a["frac_terminal_near_pi"]:>9.3f} '
              f'{a["n_pi_wraps"]:>9} {a["yaw_needed"]:>7.3f} '
              f'{a["yaw_travel_abs"]:>8.3f}')
    print()

    print('D. Target yaw drift -- the final resting heading of each leg.')
    print('   The target is 0 rad for every leg (see the module')
    print('   docstring). A leg that STOPS outside +/-0.25 rad of it')
    print('   cannot have been ended by the yaw check against 0.')
    print(f'   {"tag":<16} {"status":<10} {"yaw_final":>10} '
          f'{"|err|final":>11} {"<= 0.25?":>9}')
    for a in rows:
        st = KNOWN[a['tag']]['status']
        e = abs(a['err_final'])
        print(f'   {a["tag"]:<16} {st:<10} {a["yaw_final"]:>10.4f} '
              f'{e:>11.4f} {"yes" if e <= YAW_TOL else "NO":>9}')
    print()

    print('E. PolygonSlow: commanded vs achieved angular rate')
    print('   (full breakdown: `chain`)')
    for a in rows:
        parts = [f'{k}={v["ratio_med"]:.2f}(n={v["n"]})'
                 for k, v in sorted(a['chain'].items())]
        print(f'   {a["tag"]:<16} ' + '  '.join(parts))


def cmd_sensitivity():
    hdr('C2-NAV.22 -- sensitivity of the sign-flip count to the deadband')
    print('The achieved angular rate is noisy on a nearly-stationary '
          'robot, so a')
    print('sign-flip count is only meaningful with its deadband '
          'attached.')
    print()
    deads = [0.005, 0.01, 0.02, 0.05, 0.10, 0.15]
    print(f'{"tag":<16} ' + ' '.join(f'{d:>7}' for d in deads))
    print('-' * 66)
    for tag in ORDER:
        try:
            vals = []
            for d in deads:
                a = analyse(tag, dead=d)
                vals.append('-' if a.get('note')
                            else str(a['n_sign_flips_w_act']))
        except SystemExit:
            continue
        print(f'{tag:<16} ' + ' '.join(f'{v:>7}' for v in vals))


def cmd_dump(path):
    """Freeze every trace and record this module reads into one file, so
    the tables regenerate from docs/data/ alone."""
    cols = ('t_rel', 'x', 'y', 'yaw', 'v_act', 'w_act', 'v_nav', 'w_nav',
            'cm_action', 'cm_polygon', 'dwb_best_vx', 'dwb_best_wz',
            'dwb_margin', 'dwb_n_at_min', 'dwb_ill_osc', 'dwb_ill_base',
            'dwb_ill_rot')
    out = {'columns': list(cols), 'traces': {}, 'records': {},
           'plan_periods': {}, 'ordinary': {},
           'goal_world': list(GOAL_WORLD), 'goal_yaw': GOAL_YAW}
    # The control: the LAST recorded heading of every leg of three
    # baseline tours, ordinary legs included. Small, and it is what the
    # frame caveat rests on.
    for tag in ('c2n18_tour_r1', 'c2n21_base_r3', 'c2n21_base_r4'):
        p = os.path.join(SCRATCH, f'{tag}.json')
        if not os.path.exists(p):
            continue
        with open(p) as f:
            legs = json.load(f)['legs']
        rows = []
        for leg in legs:
            name = leg['scenario']
            tp = os.path.join(SCRATCH, f'{tag}_traces', f'{name}_rep0.csv')
            yaw = None
            if os.path.exists(tp):
                with open(tp) as f:
                    for r in csv.DictReader(f):
                        if r.get('yaw'):
                            yaw = float(r['yaw'])
            rows.append({'leg': name, 'status': leg.get('status'),
                         'secs': leg.get('duration_sim_s'),
                         't_terminal_s': leg.get('t_terminal_s'),
                         'n_progress_failures':
                             leg.get('n_progress_failures'),
                         'yaw_end': yaw})
        out['ordinary'][tag] = rows
    for tag in ORDER:
        p = _trace_path(tag)
        if not os.path.exists(p):
            continue
        rows = []
        with open(p) as f:
            for r in csv.DictReader(f):
                # NOT `x or None`: 0.0 is falsey, and a zero velocity or
                # a zero illegal count is a measurement, not a blank.
                out_row = []
                for c in cols:
                    if c in ('cm_action', 'cm_polygon'):
                        v = r.get(c)
                        out_row.append(v if v else None)
                    else:
                        out_row.append(fl(r.get(c)))
                rows.append(out_row)
        with open(p) as f:
            schema = list(csv.DictReader(f).fieldnames or ())
        out['traces'][tag] = {'columns': list(cols), 'rows': rows,
                              'schema': schema}
        rec = load_record(tag)
        if rec:
            out['records'][tag] = {
                k: rec.get(k) for k in
                ('status', 'duration_sim_s', 't_transit_s', 't_terminal_s',
                 'terminal_frac_of_leg', 'terminal_yaw_travel_rad',
                 'final_goal_err_m', 'n_plans', 'n_progress_failures',
                 'cm_action_frac', 'cm_polygon_secs',
                 'dwb_illegal_by_critic_terminal',
                 'dwb_illegal_by_critic_transit', 'goal_world')}
        per = _plan_period(tag)
        if per is not None:
            out['plan_periods'][tag] = per
    with open(path, 'w') as f:
        json.dump(out, f, separators=(',', ':'), sort_keys=True)
    print(f'wrote {path}: {len(out["traces"])} traces, '
          f'{len(out["records"])} records, '
          f'{os.path.getsize(path) / 1e6:.2f} MB')


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    cmd, rest = argv[1], argv[2:]
    if cmd == 'selftest':
        return cmd_selftest()
    if cmd == 'table':
        cmd_table(rest or None)
        return 0
    if cmd == 'phases':
        cmd_phases()
        return 0
    if cmd == 'latchstate':
        cmd_latchstate()
        return 0
    if cmd == 'attribution':
        cmd_attribution()
        return 0
    if cmd == 'ordinary':
        cmd_ordinary()
        return 0
    if cmd == 'leg':
        for t in rest:
            cmd_leg(t)
        return 0
    if cmd == 'chain':
        cmd_chain()
        return 0
    if cmd == 'latch':
        cmd_latch()
        return 0
    if cmd == 'counterfactuals':
        cmd_counterfactuals()
        return 0
    if cmd == 'sensitivity':
        cmd_sensitivity()
        return 0
    if cmd == 'dump':
        cmd_dump(rest[0] if rest else BUNDLE)
        return 0
    if cmd == 'all':
        rc = cmd_selftest()
        cmd_phases()
        cmd_table()
        cmd_attribution()
        cmd_latchstate()
        cmd_ordinary()
        cmd_latch()
        cmd_chain()
        cmd_counterfactuals()
        cmd_sensitivity()
        return rc
    print(f'unknown command {cmd!r}')
    return 2


if __name__ == '__main__':
    sys.exit(main(sys.argv))
