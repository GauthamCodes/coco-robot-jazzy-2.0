#!/usr/bin/env python3
"""C2-NAV.24 -- where the terminal creep velocity actually comes from.

OFFLINE. No simulator, no ROS node, no parameter written. Every number
below is reconstructed from traces already committed by C2-NAV.18 /
C2-NAV.21, using the command-chain columns `nav_bench.py` has recorded
since C2-NAV.21 and which no earlier analysis read.

THE QUESTION
------------
C2-NAV.22 measured that every leg spends 9.2-151.8 s creeping from
0.25 m to 0.05 m at 7-24 mm/s, and C2-NAV.23 established that widening
`FollowPath.xy_goal_tolerance` cannot fix it (it converts the creep into
terminal position error, because RotateToGoal bans translation the
moment it latches). Neither settled WHERE the small number is born:

  HYPOTHESIS A  DWB itself selects a tiny forward velocity.
  HYPOTHESIS B  DWB selects a normal one and the collision monitor's
                PolygonSlow scales it down before it reaches the wheels.

The two imply different fixes -- a controller change versus a monitor
change -- so they must be separated before anything else is tried live.

WHY THE TRACES CAN ANSWER IT
----------------------------
`nav_bench.py` records FIVE points on the same command chain at 10 Hz,
which is also the controller frequency:

  dwb_best_vx   `/evaluation`, DWB's own scoring dump. This is
                `m.twists[m.best_index].traj.velocity.x` -- the
                trajectory DWB SELECTED, read out of the controller
                before anything downstream can touch it.
  v_nav         `/cmd_vel_nav`      controller_server's published command
  v_smoothed    `/cmd_vel_smoothed` velocity_smoother output
  v_cmdvel      `/cmd_vel`          collision_monitor OUTPUT
  v_wheel       `/diff_drive_controller/cmd_vel`  what the arbiter passes
  v_act         `/model/coco/odometry`  Gazebo ground truth

The collision monitor sits between `v_smoothed` and `v_cmdvel`
(`cmd_vel_in_topic: cmd_vel_smoothed`, `cmd_vel_out_topic: cmd_vel`), so
PolygonSlow's contribution is EXACTLY the v_cmdvel / v_smoothed ratio and
nothing else on the chain can be confused for it. `cm_polygon` names the
polygon that was applied on the same cycle, so the ratio can be split by
whether PolygonSlow was actually active -- which is what turns this from
an assumption into a measurement.

EVIDENCE CLASS, stated per number
---------------------------------
OBSERVED     every column above, plus cm_action / cm_polygon and the
             dwb_* degeneracy fields, all written live by nav_bench.
ZOH-ALIGNED  the trace is resampled at 10 Hz by `last_at` (zero-order
             hold). Stages are therefore compared at a common timestamp,
             not as the same command instance; skew is bounded by one
             10 Hz controller cycle. Medians over a window of tens to
             hundreds of cycles are robust to it, single samples are not,
             and this module never quotes a single-sample ratio.
DERIVED      distance to goal, the creep-window bounds, and the
             PolygonSlow counterfactual, each marked where it appears.
UNAVAILABLE  the AMCL distance to goal. `nav_bench` subscribes to
             `/amcl_pose` and keeps it in memory, but never writes it to
             a trace or a record, so requirement 8 of the C2-NAV.24 brief
             CANNOT be answered from the frozen artifacts. It is reported
             as unavailable rather than substituted with ground truth.

WHAT THE LATTICE QUANTISATION GIVES US FOR FREE
-----------------------------------------------
`vx_samples: 20` over `[min_vel_x 0.0, max_vel_x 0.3]` makes the forward
lattice exactly {0, 0.3/19, 2*0.3/19, ...} = k * 15.789 mm/s. Every
`dwb_best_vx` in these traces lands on that grid, so DWB's choice can be
reported as an INDEX -- "the 2nd of 19 non-zero forward samples" -- which
is a far stronger statement than a velocity in isolation. It says
directly how far down its own menu DWB reached.
"""

import argparse
import csv
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WT = os.path.dirname(HERE)                      # docs/ -> worktree root
WT = os.path.dirname(WT)
SCRATCH = os.path.join(WT, '.navbench', 'results')
BUNDLE = os.path.join(HERE, 'c2nav24_chain.json')

# --- frozen baseline constants, every one from c2nav11_ntp_params.yaml --
# SimpleGoalChecker.xy_goal_tolerance, and the split nav_bench's own
# GOAL_XY_TOLERANCE uses.
OUTER_XY_TOL = 0.25
INNER_XY_TOL = 0.05      # FollowPath.xy_goal_tolerance
MAX_VEL_X = 0.3          # controller_server FollowPath.max_vel_x
VX_SAMPLES = 20          # ... vx_samples
VX_STEP = MAX_VEL_X / (VX_SAMPLES - 1)          # 15.789 mm/s
SLOWDOWN_RATIO = 0.3     # collision_monitor PolygonSlow.slowdown_ratio
LATTICE = 819            # 20 vx x 41 vtheta - 1, as C2-NAV.22 fixed it
ROT_BLOCK = 779          # every translating trajectory of the 819
FULL_WZ_BAN = 400        # one wz sign of the lattice

# Below this a velocity is indistinguishable from a commanded stop; used
# only to keep ratio denominators meaningful, never to reclassify a
# sample. `sensitivity` reports what moving it does.
V_FLOOR = 0.005

LEGS = ['open_space', 'wall_adjacent', 'wall_parallel', 'obstacle_corner',
        'corridor_gate', 'enclosure_entry', 'enclosure_exit']

# The nine runs that carry the C2-NAV.21 degeneracy columns, plus the
# three C2-NAV.18 tours that carry the chain columns but not the critic
# split. Order is the one C2-NAV.22 published.
RUNS = ['c2n18_tour_r1', 'c2n18_tour_r2', 'c2n18_tour_r3',
        'c2n21_base_r1', 'c2n21_base_r3', 'c2n21_base_r4',
        'c2n21_fpd_r3', 'c2n21_bbase_r2', 'c2n21_bbase_r3',
        'c2n19_tour_r1']

CHAIN = ['dwb_best_vx', 'v_nav', 'v_smoothed', 'v_cmdvel', 'v_wheel',
         'v_act']
STAGE = [('v_nav', 'dwb_best_vx', 'controller publish'),
         ('v_smoothed', 'v_nav', 'velocity_smoother'),
         ('v_cmdvel', 'v_smoothed', 'COLLISION MONITOR'),
         ('v_wheel', 'v_cmdvel', 'arbiter -> wheels'),
         ('v_act', 'v_wheel', 'plant (gt)')]

COLS = ('t_rel', 'x', 'y', 'yaw', 'v_act', 'w_act', 'v_nav', 'w_nav',
        'v_smoothed', 'v_cmdvel', 'v_wheel', 'scan_min',
        'dwb_n', 'dwb_illegal', 'dwb_best_vx', 'dwb_best_total',
        'dwb_best_wz', 'dwb_complete', 'dwb_zero_best', 'dwb_fwd_best',
        'dwb_margin', 'dwb_rot_span', 'dwb_n_at_min',
        'dwb_ill_osc', 'dwb_ill_base', 'dwb_ill_rot')
STRCOLS = ('cm_action', 'cm_polygon')


# ---------------------------------------------------------------- utils
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


def med(xs):
    if not xs:
        return None
    s = sorted(xs)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def pct(xs, q):
    if not xs:
        return None
    s = sorted(xs)
    return s[max(0, min(len(s) - 1, int(round(q * (len(s) - 1)))))]


def f4(v, w=8):
    return f'{v:>{w}.4f}' if v is not None else f'{"-":>{w}}'


def f3(v, w=7):
    return f'{v:>{w}.3f}' if v is not None else f'{"-":>{w}}'


# --------------------------------------------------------------- loader
_BUNDLE_CACHE = {}
_SCHEMA = {}


def _bundle():
    if not _BUNDLE_CACHE:
        if os.path.exists(BUNDLE):
            with open(BUNDLE) as f:
                _BUNDLE_CACHE.update(json.load(f))
        else:
            _BUNDLE_CACHE['traces'] = {}
            _BUNDLE_CACHE['records'] = {}
    return _BUNDLE_CACHE


def load_trace(tag, leg):
    """One leg's 10 Hz trace, from the scratch tree or the bundle.

    Missing degeneracy columns come back as None rather than 0: a blank
    is a missing measurement and a zero is a claim.
    """
    p = os.path.join(SCRATCH, f'{tag}_traces', f'{leg}_rep0.csv')
    if os.path.exists(p):
        out = []
        with open(p) as f:
            rd = csv.DictReader(f)
            _SCHEMA[(tag, leg)] = set(rd.fieldnames or ())
            for r in rd:
                d = {c: fl(r.get(c)) for c in COLS}
                for c in STRCOLS:
                    v = r.get(c)
                    d[c] = v if v else None
                out.append(d)
        return out
    b = _bundle().get('traces', {}).get(tag, {}).get(leg)
    if b:
        _SCHEMA[(tag, leg)] = set(b['columns'])
        return [dict(zip(b['columns'], row)) for row in b['rows']]
    return None


def load_record(tag):
    """The run's nav_bench record: per-leg goal, status and live split."""
    p = os.path.join(SCRATCH, f'{tag}.json')
    if os.path.exists(p):
        with open(p) as f:
            return json.load(f)
    return _bundle().get('records', {}).get(tag)


def legs_of(tag):
    """{leg: record} for one run, or {} if the run is not available."""
    d = load_record(tag)
    if not d:
        return {}
    return {r['scenario']: r for r in d.get('legs', [])}


# ------------------------------------------------------------- windowing
def windows(tag, leg):
    """Split one leg into transit / creep / inner.

    DERIVED. `dist` is ground-truth Euclidean distance to the leg's own
    `goal_world`. The creep window is [first sample at or inside
    OUTER_XY_TOL, first sample at or inside INNER_XY_TOL) -- exactly the
    0.25 -> 0.05 m band the C2-NAV.24 question names, and its outer edge
    is the same instant nav_bench splits transit from terminal on.
    """
    rec = legs_of(tag).get(leg)
    rows = load_trace(tag, leg)
    if rec is None or not rows:
        return None
    gx, gy = rec['goal_world']
    rows = [r for r in rows if r.get('x') is not None]
    if not rows:
        return None
    dist = [math.hypot(r['x'] - gx, r['y'] - gy) for r in rows]
    i0 = next((i for i, d in enumerate(dist) if d <= OUTER_XY_TOL), None)
    i1 = (next((i for i in range(i0, len(dist)) if dist[i] <= INNER_XY_TOL),
               len(dist)) if i0 is not None else None)
    return dict(tag=tag, leg=leg, rec=rec, rows=rows, dist=dist,
                i_outer=i0, i_inner=i1,
                transit=list(range(0, i0)) if i0 is not None
                else list(range(len(rows))),
                creep=list(range(i0, i1)) if i0 is not None else [],
                terminal=list(range(i0, len(rows))) if i0 is not None
                else [])


def all_windows(runs=None, legs=None):
    for tag in (runs or RUNS):
        for leg in (legs or LEGS):
            w = windows(tag, leg)
            if w:
                yield w


# ------------------------------------------------------------- measures
def col(w, idx, c):
    return [abs(w['rows'][i][c]) for i in idx
            if w['rows'][i].get(c) is not None]


def stage_ratio(w, idx, num, den, floor=V_FLOOR, polygon=None):
    """Median of |num| / |den| over cycles where the DENOMINATOR is above
    the floor. OBSERVED numerator and denominator, ZOH-ALIGNED pairing.

    Restricting on the denominator only is deliberate: excluding cycles
    where the NUMERATOR is small would discard exactly the cycles where a
    downstream stage did the reducing, which is the effect being
    measured.
    """
    out = []
    for i in idx:
        r = w['rows'][i]
        a, b = r.get(num), r.get(den)
        if a is None or b is None or abs(b) < floor:
            continue
        if polygon is not None and (r.get('cm_polygon') or 'none') != polygon:
            continue
        out.append(abs(a) / abs(b))
    return med(out), len(out)


def lattice_index(v):
    """Which of the 20 forward samples DWB picked. DERIVED from
    vx_samples/max_vel_x; returns None if v is not on the grid."""
    if v is None:
        return None
    k = v / VX_STEP
    return int(round(k)) if abs(k - round(k)) < 0.02 else None


# --------------------------------------------------------------- report
def cmd_avail(args):
    hdr('C2-NAV.24 -- data availability, per run and leg')
    print('creep = samples in the 0.25 -> 0.05 m band. `chain` counts '
          'cycles carrying')
    print('all five chain stages; `dwb` counts cycles carrying the '
          'C2-NAV.21 critic split.')
    print()
    print(f'{"tag":<16}{"leg":<17}{"status":<10}{"rows":>6}{"creep":>7}'
          f'{"secs":>7}{"chain":>7}{"dwb":>6}  note')
    print('-' * 88)
    n_ok = 0
    for w in all_windows():
        idx = w['creep']
        rows = w['rows']
        chain = sum(1 for i in idx
                    if all(rows[i].get(c) is not None for c in CHAIN))
        dwbn = sum(1 for i in idx if rows[i].get('dwb_ill_rot') is not None)
        secs = (rows[idx[-1]]['t_rel'] - rows[idx[0]]['t_rel']) if idx else 0.0
        note = ''
        if w['i_outer'] is None:
            note = 'never reached 0.25 m'
        elif not idx:
            note = 'entered already inside 0.05 m'
        elif w['i_inner'] == len(rows):
            note = 'never reached 0.05 m'
        if idx:
            n_ok += 1
        print(f'{w["tag"]:<16}{w["leg"]:<17}{w["rec"]["status"]:<10}'
              f'{len(rows):>6}{len(idx):>7}{secs:>7.1f}{chain:>7}{dwbn:>6}'
              f'  {note}')
    print()
    print(f'legs with a non-empty creep window: {n_ok}')


def cmd_chain(args):
    """The five-stage reconstruction. This is the answer."""
    hdr('C2-NAV.24 -- the command chain across the 0.25 -> 0.05 m creep')
    print('All medians of |v| over the creep window, m/s. OBSERVED at '
          'each stage;')
    print('stages are ZOH-aligned at a common 10 Hz timestamp.')
    print()
    print(f'{"tag":<16}{"leg":<17}{"n":>5}{"secs":>6}'
          f'{"dwb_vx":>9}{"v_nav":>9}{"v_smth":>9}{"v_cmd":>9}'
          f'{"v_whl":>9}{"v_act":>9}{"slow%":>7}')
    print('-' * 106)
    for w in all_windows(args.runs, args.legs):
        idx = w['creep']
        if not idx:
            continue
        rows = w['rows']
        secs = rows[idx[-1]]['t_rel'] - rows[idx[0]]['t_rel']
        slow = 100.0 * sum(1 for i in idx
                           if rows[i].get('cm_polygon') == 'PolygonSlow') \
            / len(idx)
        print(f'{w["tag"]:<16}{w["leg"]:<17}{len(idx):>5}{secs:>6.1f}'
              + ''.join(f4(med(col(w, idx, c)), 9) for c in CHAIN)
              + f'{slow:>7.1f}')


def cmd_stages(args):
    """The decisive split: which STAGE multiplies the command down."""
    hdr('C2-NAV.24 -- per-stage gain across the creep window')
    print('Median |downstream| / |upstream| per chain stage, over cycles '
          'whose UPSTREAM')
    print(f'value exceeds {V_FLOOR} m/s. A stage that does not attenuate '
          'reads 1.00.')
    print('The collision monitor row is the whole of Hypothesis B: '
          'PolygonSlow multiplies')
    print(f'the twist by slowdown_ratio = {SLOWDOWN_RATIO}, so if B were '
          'true this row reads ~0.30')
    print('and every other row reads ~1.00.')
    print()
    print(f'{"stage":<22}{"n":>7}{"median":>9}{"p10":>8}{"p90":>8}')
    print('-' * 56)
    pool = {s: [] for s in STAGE}
    for w in all_windows(args.runs, args.legs):
        for i in w['creep']:
            r = w['rows'][i]
            for s in STAGE:
                num, den, _ = s
                a, b = r.get(num), r.get(den)
                if a is None or b is None or abs(b) < V_FLOOR:
                    continue
                pool[s].append(abs(a) / abs(b))
    for s in STAGE:
        num, den, label = s
        v = pool[s]
        print(f'{label:<22}{len(v):>7}{f3(med(v), 9)}'
              f'{f3(pct(v, 0.10), 8)}{f3(pct(v, 0.90), 8)}')
    print()
    print('Same rows, split by the polygon the monitor applied on that '
          'cycle. This is')
    print('what makes PolygonSlow a MEASUREMENT and not an assumption: '
          'the monitor stage')
    print('must read ~1.00 when no polygon is active and ~0.30 when '
          'PolygonSlow is.')
    print()
    print(f'{"stage":<22}{"polygon":<16}{"n":>7}{"median":>9}')
    print('-' * 56)
    bypoly = {}
    for w in all_windows(args.runs, args.legs):
        for i in w['creep']:
            r = w['rows'][i]
            p = r.get('cm_polygon') or 'none'
            for s in STAGE:
                num, den, _ = s
                a, b = r.get(num), r.get(den)
                if a is None or b is None or abs(b) < V_FLOOR:
                    continue
                bypoly.setdefault((s, p), []).append(abs(a) / abs(b))
    for s in STAGE:
        for p in sorted({k[1] for k in bypoly if k[0] is s}):
            v = bypoly[(s, p)]
            print(f'{s[2]:<22}{p:<16}{len(v):>7}{f3(med(v), 9)}')
        print()


def cmd_dwb(args):
    """What DWB itself chose, and what it had to choose from."""
    hdr('C2-NAV.24 -- DWB\'s own selection across the creep window')
    print(f'The forward lattice is {VX_SAMPLES} samples over '
          f'[0, {MAX_VEL_X}] m/s, i.e. a step of')
    print(f'{VX_STEP * 1000:.3f} mm/s. `idx` is the median lattice index '
          'DWB SELECTED among the')
    print('cycles where it chose to translate at all -- how far down its '
          'own menu it')
    print('reached. `zero%` is the share of cycles where it selected '
          'vx = 0 exactly.')
    print()
    print('`fwd_avail%` is the share of cycles that had at least one '
          'COMPLETE, LEGAL')
    print('forward trajectory to choose (dwb_fwd_best present). '
          '`transit_vx` is the same')
    print('median over the leg BEFORE 0.25 m, as the within-leg control.')
    print()
    print(f'{"tag":<16}{"leg":<17}{"n":>5}{"zero%":>7}{"dwb_vx+":>9}'
          f'{"idx":>5}{"fwd_av%":>8}{"transit":>9}{"ratio":>7}')
    print('-' * 90)
    for w in all_windows(args.runs, args.legs):
        idx = w['creep']
        if not idx:
            continue
        rows = w['rows']
        dv = [rows[i]['dwb_best_vx'] for i in idx
              if rows[i].get('dwb_best_vx') is not None]
        if not dv:
            continue
        zero = 100.0 * sum(1 for v in dv if abs(v) < 1e-9) / len(dv)
        pos = [abs(v) for v in dv if abs(v) >= 1e-9]
        mp = med(pos)
        li = lattice_index(mp)
        fa = [rows[i].get('dwb_fwd_best') for i in idx]
        fav = (100.0 * sum(1 for v in fa if v is not None) / len(fa)
               if any(v is not None for v in fa) else None)
        tv = [abs(rows[i]['dwb_best_vx']) for i in w['transit']
              if rows[i].get('dwb_best_vx') is not None]
        mt = med(tv)
        ratio = (mp / mt) if (mp and mt) else None
        print(f'{w["tag"]:<16}{w["leg"]:<17}{len(dv):>5}{zero:>7.1f}'
              f'{f4(mp, 9)}{(li if li is not None else -1):>5}'
              f'{f3(fav, 8) if fav is not None else f"{chr(45):>8}"}'
              f'{f4(mt, 9)}{f3(ratio, 7)}')


def mean(xs):
    return (sum(xs) / len(xs)) if xs else None


def cmd_attrib(args):
    """The A / B decomposition, and the counterfactual.

    MEANS, not medians, throughout. Throughput over a window is a mean:
    a cycle on which DWB selected vx = 0 contributes zero distance, and
    that is exactly what should be averaged in. A median hides it -- on
    the legs where DWB chose vx = 0 on more than half the cycles the
    median choice is literally 0.0000, which is a true statement about
    DWB but a useless factor.
    """
    hdr('C2-NAV.24 -- attribution: how much of the creep speed is whose')
    print('Two independent multiplications take the transit command down '
          'to the creep')
    print('command, and both are measured on the same cycles:')
    print()
    print('  dwb_f   mean |dwb_best_vx| over the creep window '
          '/ mean over that leg\'s transit')
    print('  mon_f   mean |v_cmdvel| / mean |v_smoothed| over the creep '
          'window')
    print()
    print('mon_f is a mean-of-means, NOT the 0.300 per-cycle ratio: it '
          'carries PolygonSlow\'s')
    print('DUTY CYCLE as well as its depth, so a leg the polygon touches '
          'half the time')
    print('reads ~0.65 and one it never touches reads 1.00. That is the '
          'honest')
    print('window-level number and it is what the counterfactual needs.')
    print()
    print('DERIVED: `no_slow_s` is what this creep would have taken with '
          'the monitor stage')
    print('removed and nothing else changed -- observed seconds scaled by '
          'mon_f. It assumes')
    print('the creep distance and DWB\'s own choice are unchanged, which '
          'C2-NAV.22 already')
    print('established is sound: the DWB generator uses sim_time, not '
          'sim_period, so the')
    print('lattice is full-range every cycle and does NOT shrink because '
          'the previous')
    print('command was slowed. There is no feedback path from the '
          'monitor back into DWB.')
    print()
    print(f'{"tag":<16}{"leg":<17}{"secs":>7}{"dwb_f":>8}{"mon_f":>8}'
          f'{"product":>9}{"no_slow_s":>10}{"saved_s":>9}')
    print('-' * 92)
    tot_obs = tot_cf = 0.0
    dwbs, mons = [], []
    for w in all_windows(args.runs, args.legs):
        idx = w['creep']
        if not idx:
            continue
        rows = w['rows']
        secs = rows[idx[-1]]['t_rel'] - rows[idx[0]]['t_rel']
        dv_c = mean(col(w, idx, 'dwb_best_vx'))
        dv_t = mean(col(w, w['transit'], 'dwb_best_vx'))
        dwb_f = (dv_c / dv_t) if (dv_c is not None and dv_t) else None
        sm = mean(col(w, idx, 'v_smoothed'))
        cv = mean(col(w, idx, 'v_cmdvel'))
        mon_f = (cv / sm) if (cv is not None and sm) else None
        prod = (dwb_f * mon_f) if (dwb_f is not None
                                   and mon_f is not None) else None
        cf = (secs * mon_f) if mon_f is not None else None
        saved = (secs - cf) if cf is not None else None
        if cf is not None:
            tot_obs += secs
            tot_cf += cf
        if dwb_f is not None:
            dwbs.append(dwb_f)
        if mon_f is not None:
            mons.append(mon_f)
        print(f'{w["tag"]:<16}{w["leg"]:<17}{secs:>7.1f}{f3(dwb_f, 8)}'
              f'{f3(mon_f, 8)}{f3(prod, 9)}{f3(cf, 10)}{f3(saved, 9)}')
    print('-' * 92)
    print(f'{"median factor across legs":<40}{f3(med(dwbs), 8)}'
          f'{f3(med(mons), 8)}')
    if tot_obs:
        print()
        print(f'Creep seconds observed across these legs:      '
              f'{tot_obs:>8.1f}')
        print(f'Creep seconds with the monitor stage removed:  '
              f'{tot_cf:>8.1f}   (DERIVED)')
        print(f'Attributable to the collision monitor:         '
              f'{tot_obs - tot_cf:>8.1f} s = '
              f'{100.0 * (tot_obs - tot_cf) / tot_obs:.1f} %')
        print()
        print('The complement is NOT "attributable to DWB" -- removing '
              'the monitor still')
        print(f'leaves {tot_cf:.0f} s of creep, and dwb_f says why: DWB '
              'is choosing a command')
        print('several times smaller than its own transit command before '
              'the monitor')
        print('touches anything.')


def cmd_why(args):
    """When DWB selected vx = 0, could it have chosen forward?

    Requirement 10 of the C2-NAV.24 brief. This is the question that
    decides what a controller-level fix would have to change, and the
    committed traces answer it directly because `_eval_cb` recorded, per
    cycle and over COMPLETE trajectories only:

      dwb_zero_best  best (lowest) total among vx == 0 trajectories
      dwb_fwd_best   best total among translating ones
      dwb_margin     zero_best - fwd_best

    DWB MINIMISES total, so margin > 0 means a forward trajectory scored
    STRICTLY BETTER than every stationary one. The classes are therefore:

      BANNED     no complete legal forward trajectory existed at all
                 (dwb_fwd_best absent). Nothing DWB could have picked.
      OUTSCORED  forward existed and scored WORSE (margin <= 0). DWB
                 preferred to rotate; this is a scoring outcome, not a
                 ban, and it is the only class a critic re-weighting
                 could move.
      ANOMALY    forward existed and scored better, yet vx = 0 was
                 selected. Expected to be rare; printed rather than
                 hidden, because a large count would mean the margin
                 fields do not mean what this module claims.

    `rot_ban%` is the share of ALL creep cycles on which
    RotateToGoalCritic alone rejected at least ROT_BLOCK trajectories --
    the signature C2-NAV.22 fixed for "the latch is engaged and every
    translating trajectory is illegal".
    """
    hdr('C2-NAV.24 -- when DWB chose vx = 0, what were its options?')
    print(f'Only runs carrying the C2-NAV.21 critic columns can answer '
          f'this. ROT_BLOCK = {ROT_BLOCK}')
    print(f'of the {LATTICE}-trajectory lattice is "every translating '
          f'trajectory rejected".')
    print()
    print(f'{"tag":<16}{"leg":<17}{"n0":>6}{"banned%":>9}{"outsc%":>8}'
          f'{"anom%":>7}{"rot_ban%":>9}{"osc_ban%":>9}')
    print('-' * 90)
    tot = {'banned': 0, 'outscored': 0, 'anomaly': 0, 'n': 0}
    for w in all_windows(args.runs, args.legs):
        idx = w['creep']
        rows = w['rows']
        z = [i for i in idx if rows[i].get('dwb_best_vx') is not None
             and abs(rows[i]['dwb_best_vx']) < 1e-9]
        # Only classify where the degeneracy columns are present.
        z = [i for i in z if rows[i].get('dwb_complete') is not None]
        if not z:
            continue
        banned = sum(1 for i in z if rows[i].get('dwb_fwd_best') is None)
        outsc = sum(1 for i in z if rows[i].get('dwb_fwd_best') is not None
                    and (rows[i].get('dwb_margin') or 0.0) <= 0.0)
        anom = sum(1 for i in z if rows[i].get('dwb_fwd_best') is not None
                   and (rows[i].get('dwb_margin') or 0.0) > 0.0)
        rb = [i for i in idx if (rows[i].get('dwb_ill_rot') or 0) >= ROT_BLOCK]
        ob = [i for i in idx if (rows[i].get('dwb_ill_osc') or 0)
              >= FULL_WZ_BAN]
        tot['banned'] += banned
        tot['outscored'] += outsc
        tot['anomaly'] += anom
        tot['n'] += len(z)
        n = len(z)
        print(f'{w["tag"]:<16}{w["leg"]:<17}{n:>6}'
              f'{100.0 * banned / n:>9.1f}{100.0 * outsc / n:>8.1f}'
              f'{100.0 * anom / n:>7.1f}'
              f'{100.0 * len(rb) / len(idx):>9.1f}'
              f'{100.0 * len(ob) / len(idx):>9.1f}')
    print('-' * 90)
    n = tot['n']
    if n:
        print(f'{"ALL vx=0 creep cycles":<39}{n:>6}'
              f'{100.0 * tot["banned"] / n:>9.1f}'
              f'{100.0 * tot["outscored"] / n:>8.1f}'
              f'{100.0 * tot["anomaly"] / n:>7.1f}')


def cmd_compare(args):
    """One normal terminal approach against one pathological one."""
    hdr('C2-NAV.24 -- representative terminal windows, normal against '
        'pathological')
    print('The three runs the C2-NAV.24 brief names, each shown on the '
          'leg that has the')
    print('pathology (enclosure_entry) and on an ordinary leg of the '
          'SAME tour as the')
    print('within-run control. Every velocity is a MEAN over the creep '
          'window, because')
    print('these windows are 62-85 % vx = 0 cycles and a median of that '
          'is just 0.0000 --')
    print('true, but it describes the zeros rather than the throughput.')
    print()
    print('dwb_vx is DWB\'s OWN selection before anything downstream; '
          'v_cmd is after the')
    print('collision monitor. mon_f = mean(v_cmd)/mean(v_smth) carries '
          'PolygonSlow\'s duty')
    print('cycle as well as its depth. banned% is the share of the vx = '
          '0 cycles that had')
    print('NO legal complete forward trajectory to pick.')
    print()
    print(f'{"tag":<16}{"leg":<17}{"secs":>7}{"zero%":>7}{"band%":>7}'
          f'{"dwb_vx":>9}{"v_smth":>9}{"v_cmd":>9}{"v_act":>9}'
          f'{"mon_f":>7}{"slow%":>7}')
    print('-' * 112)
    want = [('c2n21_base_r3', 'enclosure_entry'),
            ('c2n21_base_r3', 'obstacle_corner'),
            ('c2n21_bbase_r2', 'enclosure_entry'),
            ('c2n21_bbase_r2', 'obstacle_corner'),
            ('c2n21_base_r4', 'enclosure_entry'),
            ('c2n21_base_r4', 'corridor_gate')]
    if args.runs or args.legs:
        want = [(t, l) for t, l in want
                if (not args.runs or t in args.runs)
                and (not args.legs or l in args.legs)]
    for tag, leg in want:
        w = windows(tag, leg)
        if not w or not w['creep']:
            print(f'{tag:<16}{leg:<17}  no creep window')
            continue
        idx, rows = w['creep'], w['rows']
        secs = rows[idx[-1]]['t_rel'] - rows[idx[0]]['t_rel']
        dv = [rows[i]['dwb_best_vx'] for i in idx
              if rows[i].get('dwb_best_vx') is not None]
        zero = 100.0 * sum(1 for v in dv if abs(v) < 1e-9) / len(dv) \
            if dv else None
        z = [i for i in idx if rows[i].get('dwb_best_vx') is not None
             and abs(rows[i]['dwb_best_vx']) < 1e-9
             and rows[i].get('dwb_complete') is not None]
        band = (100.0 * sum(1 for i in z
                            if rows[i].get('dwb_fwd_best') is None) / len(z)
                if z else None)
        sm = mean(col(w, idx, 'v_smoothed'))
        cv = mean(col(w, idx, 'v_cmdvel'))
        mon_f = (cv / sm) if (cv is not None and sm) else None
        slow = 100.0 * sum(1 for i in idx
                           if rows[i].get('cm_polygon') == 'PolygonSlow') \
            / len(idx)
        print(f'{tag:<16}{leg:<17}{secs:>7.1f}'
              f'{(zero if zero is not None else -1):>7.1f}'
              f'{f3(band, 7) if band is not None else f"{chr(45):>7}"}'
              f'{f4(mean(col(w, idx, "dwb_best_vx")), 9)}'
              f'{f4(sm, 9)}{f4(cv, 9)}'
              f'{f4(mean(col(w, idx, "v_act")), 9)}'
              f'{f3(mon_f, 7)}{slow:>7.1f}')


def cmd_verdict(args):
    """A / B / both, decided by the brief's own decision rule.

    The two effects MULTIPLY, so their relative contribution is a share
    of the log reduction, not of the linear one. A stage that halves the
    command and a stage that halves it again contribute equally, and
    only logs say so.

    reduction_total = mean(v_cmdvel in creep) / mean(dwb_best_vx in
    transit), decomposed as

        dwb_f = mean(dwb_vx creep) / mean(dwb_vx transit)     Hypothesis A
        mon_f = mean(v_cmdvel creep) / mean(v_smoothed creep) Hypothesis B

    with the residual being the publish + smoother stages, which are
    measured pass-throughs and should therefore be ~0 %.
    """
    hdr('C2-NAV.24 -- verdict: A, B, both, or indeterminate')
    print('Share of the LOG reduction from each leg\'s own transit '
          'command down to what')
    print('the collision monitor emitted during its creep. The stages '
          'multiply, so a')
    print('log share is the only additive accounting of them.')
    print()
    print(f'{"tag":<16}{"leg":<17}{"secs":>7}{"total_x":>9}'
          f'{"dwb_x":>8}{"mon_x":>8}{"dwb%":>7}{"mon%":>7}{"resid%":>8}')
    print('-' * 96)
    agg = []
    for w in all_windows(args.runs, args.legs):
        idx = w['creep']
        if not idx:
            continue
        rows = w['rows']
        secs = rows[idx[-1]]['t_rel'] - rows[idx[0]]['t_rel']
        dv_t = mean(col(w, w['transit'], 'dwb_best_vx'))
        dv_c = mean(col(w, idx, 'dwb_best_vx'))
        sm = mean(col(w, idx, 'v_smoothed'))
        cv = mean(col(w, idx, 'v_cmdvel'))
        if not (dv_t and dv_c and sm and cv):
            continue
        tot = cv / dv_t
        dwb_f, mon_f = dv_c / dv_t, cv / sm
        if not (0 < tot < 1 and 0 < dwb_f and 0 < mon_f):
            continue
        lt = math.log(tot)
        ld = math.log(dwb_f) if dwb_f < 1 else 0.0
        lm = math.log(mon_f) if mon_f < 1 else 0.0
        if lt >= 0:
            continue
        dpc, mpc = 100.0 * ld / lt, 100.0 * lm / lt
        agg.append((secs, ld, lm, lt, w))
        print(f'{w["tag"]:<16}{w["leg"]:<17}{secs:>7.1f}{1 / tot:>9.1f}'
              f'{1 / dwb_f:>8.1f}{(1 / mon_f):>8.2f}{dpc:>7.1f}{mpc:>7.1f}'
              f'{100.0 - dpc - mpc:>8.1f}')
    print('-' * 96)
    if not agg:
        print('no leg had both factors measurable')
        return
    # Time-weighted, because a 152 s creep and a 2 s creep are not
    # equally important facts about this robot.
    W = sum(a[0] for a in agg)
    d = sum(a[0] * a[1] for a in agg) / W
    m = sum(a[0] * a[2] for a in agg) / W
    t = sum(a[0] * a[3] for a in agg) / W
    print(f'time-weighted over {len(agg)} legs / {W:.0f} creep seconds:')
    print(f'  DWB\'s own selection   {100.0 * d / t:>5.1f} % of the log '
          f'reduction')
    print(f'  collision monitor     {100.0 * m / t:>5.1f} %')
    print(f'  residual (publish + smoother, measured pass-throughs) '
          f'{100.0 * (t - d - m) / t:>5.1f} %')
    print()
    pathological = [a for a in agg if a[0] >= 20.0]
    if pathological:
        W2 = sum(a[0] for a in pathological)
        d2 = sum(a[0] * a[1] for a in pathological) / W2
        m2 = sum(a[0] * a[2] for a in pathological) / W2
        t2 = sum(a[0] * a[3] for a in pathological) / W2
        print(f'restricted to the {len(pathological)} PATHOLOGICAL legs '
              f'(creep >= 20 s, {W2:.0f} s):')
        print(f'  DWB\'s own selection   {100.0 * d2 / t2:>5.1f} %')
        print(f'  collision monitor     {100.0 * m2 / t2:>5.1f} %')


def cmd_sensitivity(args):
    """Does the answer move if the ratio floor moves?"""
    hdr('C2-NAV.24 -- sensitivity of the monitor factor to V_FLOOR')
    print('The monitor stage ratio is the load-bearing number. If it '
          'depended on where')
    print('the denominator floor is put, it would be an artefact of the '
          'instrument.')
    print()
    print(f'{"floor m/s":>10}{"n":>8}{"monitor":>9}{"smoother":>10}'
          f'{"publish":>9}{"wheels":>9}')
    print('-' * 56)
    for floor in (0.001, 0.002, 0.005, 0.01, 0.02, 0.05):
        acc = {s: [] for s in STAGE}
        for w in all_windows(args.runs, args.legs):
            for i in w['creep']:
                r = w['rows'][i]
                for s in STAGE:
                    num, den, _ = s
                    a, b = r.get(num), r.get(den)
                    if a is None or b is None or abs(b) < floor:
                        continue
                    acc[s].append(abs(a) / abs(b))
        mon = acc[STAGE[2]]
        print(f'{floor:>10.3f}{len(mon):>8}{f3(med(mon), 9)}'
              f'{f3(med(acc[STAGE[1]]), 10)}{f3(med(acc[STAGE[0]]), 9)}'
              f'{f3(med(acc[STAGE[3]]), 9)}')


def cmd_selftest(args):
    """Gates. The module refuses to be believed until these pass.

    G1 reproduces nav_bench's OWN live transit/terminal split from the
       trace, using the same 0.25 m rule. It is the check that the
       offline windowing is the same windowing the live run used, and it
       is the sharpest available test of the distance arithmetic and the
       goal frame: t_transit_s is a first-crossing INSTANT, so any error
       in either would move it.
    G2 reproduces the live `final_goal_err_m` from the trace's last
       sample -- the second half of the frame check, and unlike a median
       it is weighting-invariant (see G5).
    G3 is the instrument check for the monitor stage: with NO polygon
       active the monitor must be a pass-through (ratio ~1.0). If this
       failed, a 0.3 measured under PolygonSlow would mean nothing.
    G4 asserts every dwb_best_vx lies on the declared forward lattice.
    G5 is NOT a pass/fail gate but a measured caveat, reported so it
       cannot be mistaken for agreement. nav_bench's live
       `terminal_v_med` is SAMPLE-COUNT weighted over the raw ground
       truth series, and `_gt_cb` timestamps each sample with the NODE
       CLOCK AT CALLBACK TIME rather than the message stamp -- so the
       raw series bunches whenever the executor is free, which on a
       near-stationary terminal window means the stopped tail is
       over-represented. This module is TIME-uniform at 10 Hz. The two
       therefore disagree on exactly the legs that stop early, and the
       disagreement is printed rather than tuned away. Time-uniform is
       the correct weighting for every question C2-NAV.24 asks ("how
       long did the creep take", "what did the chain do per cycle"), so
       it is the one used throughout; no velocity median in this module
       is comparable to a live `*_v_med`.
    """
    hdr('C2-NAV.24 -- selftest')
    fails = []

    # G1 / G2 -- against nav_bench's own live numbers.
    n1 = n2 = 0
    worst1 = worst2 = 0.0
    for w in all_windows():
        rec, rows = w['rec'], w['rows']
        live_t = rec.get('t_transit_s')
        if live_t is not None and w['i_outer'] is not None:
            mine = rows[w['i_outer']]['t_rel'] - rows[0]['t_rel']
            d = abs(mine - live_t)
            worst1 = max(worst1, d)
            n1 += 1
            if d > 0.35:
                fails.append(f'G1 {w["tag"]}/{w["leg"]}: transit '
                             f'{mine:.2f} vs live {live_t:.2f}')
        live_e = rec.get('final_goal_err_m')
        if live_e is not None:
            gx, gy = rec['goal_world']
            mine = math.hypot(rows[-1]['x'] - gx, rows[-1]['y'] - gy)
            d = abs(mine - live_e)
            worst2 = max(worst2, d)
            n2 += 1
            # The trace's last sample is the ZOH of the last raw sample
            # at or before t1, so this can disagree by up to one 10 Hz
            # cycle of travel. At the 30-80 mm/s these legs end at that
            # is 3-8 mm, which is what the limit is: a tighter one would
            # be testing the resampler, not the frame.
            if d > 0.010:
                fails.append(f'G2 {w["tag"]}/{w["leg"]}: final err '
                             f'{mine:.4f} vs live {live_e:.4f}')
    print(f'G1 transit split vs nav_bench live: {n1} legs, '
          f'worst {worst1:.2f} s   (limit 0.35 s = 3.5 cycles)')
    print(f'G2 final_goal_err_m vs live:        {n2} legs, '
          f'worst {worst2:.4f} m (limit 0.010 = one ZOH cycle of travel)')

    # G3 -- the monitor is a pass-through when idle.
    idle, slow = [], []
    for w in all_windows():
        for i in w['terminal']:
            r = w['rows'][i]
            a, b = r.get('v_cmdvel'), r.get('v_smoothed')
            if a is None or b is None or abs(b) < V_FLOOR:
                continue
            p = r.get('cm_polygon') or 'none'
            (slow if p == 'PolygonSlow' else
             idle if p == 'none' else []).append(abs(a) / abs(b))
    mi, ms = med(idle), med(slow)
    print(f'G3 monitor gain, no polygon active:  n={len(idle):<6} '
          f'median {mi:.3f}   (expect ~1.00)')
    print(f'   monitor gain, PolygonSlow active: n={len(slow):<6} '
          f'median {ms:.3f}   (declared slowdown_ratio '
          f'{SLOWDOWN_RATIO})')
    if mi is None or abs(mi - 1.0) > 0.05:
        fails.append(f'G3 idle monitor gain {mi} is not a pass-through')
    if ms is None or abs(ms - SLOWDOWN_RATIO) > 0.05:
        fails.append(f'G3 PolygonSlow gain {ms} != {SLOWDOWN_RATIO}')

    # G4 -- lattice quantisation.
    off = tot = 0
    for w in all_windows():
        for i in w['terminal']:
            v = w['rows'][i].get('dwb_best_vx')
            if v is None:
                continue
            tot += 1
            if lattice_index(abs(v)) is None:
                off += 1
    print(f'G4 dwb_best_vx on the {VX_SAMPLES}-sample forward lattice: '
          f'{tot - off}/{tot} on grid')
    if tot and off / tot > 0.01:
        fails.append(f'G4 {off}/{tot} dwb_best_vx off the declared lattice')

    # G5 -- reported, never gated. See the docstring.
    diffs = []
    for w in all_windows():
        live_v = w['rec'].get('terminal_v_med')
        mine = med(col(w, w['terminal'], 'v_act'))
        if live_v is not None and mine is not None:
            diffs.append((abs(mine - live_v), w['tag'], w['leg'],
                          mine, live_v))
    diffs.sort(reverse=True)
    n_gap = sum(1 for d in diffs if d[0] > 0.010)
    print('G5 CAVEAT, not a gate: time-uniform terminal_v_med vs live '
          'sample-counted --')
    print(f'   {n_gap}/{len(diffs)} legs differ by > 0.010 m/s; worst '
          f'{diffs[0][0]:.4f} on {diffs[0][1]}/{diffs[0][2]} '
          f'({diffs[0][3]:.4f} vs {diffs[0][4]:.4f}).')
    print('   Different weightings of different series, not a '
          'disagreement about the robot.')
    print('   Every velocity median in this module is the time-uniform '
          'one.')

    print()
    if fails:
        print('FAILED:')
        for f in fails:
            print('  ' + f)
        return 1
    print('all gates pass')
    return 0


def cmd_dump(args):
    """Freeze every trace this module reads into a JSON bundle, so the
    analysis reproduces from the repository alone once the scratch tree
    is gone."""
    # Only the columns this module reads. The full 28-column trace is
    # 5.3 MB bundled; these 17 are 2.4 MB and reproduce every number
    # here. The scratch CSVs remain the wider record if a later
    # experiment needs the rest.
    keep = ('t_rel', 'x', 'y', 'v_act', 'w_act', 'v_nav', 'v_smoothed',
            'v_cmdvel', 'v_wheel', 'dwb_best_vx', 'dwb_best_wz',
            'dwb_complete', 'dwb_zero_best', 'dwb_fwd_best', 'dwb_margin',
            'dwb_ill_rot', 'dwb_ill_osc')
    out = {'traces': {}, 'records': {}}
    for tag in RUNS:
        rec = load_record(tag)
        if rec:
            out['records'][tag] = rec
        for leg in LEGS:
            rows = load_trace(tag, leg)
            if not rows:
                continue
            cols = list(keep) + list(STRCOLS)
            out['traces'].setdefault(tag, {})[leg] = {
                'columns': cols,
                'rows': [[(round(r[c], 5) if isinstance(r.get(c), float)
                           else r.get(c)) for c in cols] for r in rows]}
    with open(args.path, 'w') as f:
        json.dump(out, f, separators=(',', ':'))
    n = sum(len(v) for v in out['traces'].values())
    print(f'wrote {args.path}: {len(out["records"])} records, {n} traces')
    return 0


CMDS = {'avail': cmd_avail, 'chain': cmd_chain, 'stages': cmd_stages,
        'why': cmd_why, 'verdict': cmd_verdict,
        'dwb': cmd_dwb, 'attrib': cmd_attrib, 'compare': cmd_compare,
        'sensitivity': cmd_sensitivity, 'selftest': cmd_selftest}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse
                                 .RawDescriptionHelpFormatter)
    ap.add_argument('cmd', choices=list(CMDS) + ['dump', 'all'])
    ap.add_argument('--runs', nargs='*', default=None)
    ap.add_argument('--legs', nargs='*', default=None)
    ap.add_argument('--path', default=BUNDLE)
    # C2-NAV.23's reproducibility test, kept: point this at a path that
    # does not exist and every command must fall back to the committed
    # bundle and produce byte-identical output.
    ap.add_argument('--scratch', default=None,
                    help='override the .navbench scratch tree')
    a = ap.parse_args(argv)
    if a.scratch is not None:
        global SCRATCH
        SCRATCH = a.scratch
    if a.cmd == 'dump':
        return cmd_dump(a)
    if a.cmd == 'all':
        rc = cmd_selftest(a)
        for c in ('avail', 'chain', 'stages', 'dwb', 'why', 'attrib',
                  'compare', 'verdict', 'sensitivity'):
            CMDS[c](a)
        return rc
    r = CMDS[a.cmd](a)
    return r if isinstance(r, int) else 0


if __name__ == '__main__':
    sys.exit(main())
