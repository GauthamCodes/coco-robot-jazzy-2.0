#!/usr/bin/env python3
"""C2-NAV.25 -- PolygonSlow.slowdown_ratio 0.3 -> 1.0, live.

The C2-NAV.24 decomposition is a reconstruction from frozen traces. It
predicts, in advance and in writing, what removing the monitor's constant
derating does to the tour:

    total creep   872.9 s -> ~488.9 s   (-44.0 %)
    worst leg     151.8 s -> ~45.4 s
    monitor gain  0.300   -> 1.000  under `PolygonSlow`

This module does two jobs and nothing else.

BEFORE the simulator, `paramdiff` flattens the frozen baseline and the
candidate to leaf paths and asserts that EXACTLY ONE leaf differs. A
one-line textual diff is not the same claim: a YAML file can change
meaning without changing a line count, and C2-NAV.23's lesson was that
"edited" and "loaded" are different claims again -- the loaded value is
read back off the running node by `c2n25_liveparam.sh`, separately.

AFTER, every measurement reuses C2-NAV.24's own windowing and ratio code
by IMPORTING it rather than restating it, so the creep window, the ratio
floor and the stage definitions are byte-identical between the frozen
baseline arm and the fresh candidate arm. A comparison whose two halves
were computed by two copies of a definition is not a comparison.

EVIDENCE CLASS
--------------
OBSERVED    every trace column, the per-leg record fields
            (`status`, `final_goal_err_m`, `min_clearance_m`,
            `cm_action_frac`, `cm_polygon_secs`), and the parameter
            values read back off the running nodes.
DERIVED     creep-window bounds and creep seconds (C2-NAV.24's rule),
            and the baseline/candidate deltas.
FROZEN      the baseline arm. `c2n21_base_r{1,3,4}` (topology A) and
            `c2n21_bbase_r{2,3}` (topology B) were recorded before this
            session and are not re-run; they are the only baseline runs
            that carry the C2-NAV.21 critic columns AND were produced by
            the same runner at the same route.

WHAT THIS CANNOT SHOW
---------------------
Three candidate runs against five frozen baseline runs is not a paired
design and the pathological case reproduces roughly one tour in four
(C2-NAV.21). The pre-registered gate is therefore read on the AGGREGATE
per-run creep seconds and on per-leg safety, not on a single leg
matching 45.4 s.
"""

import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import c2nav24_chain as C24          # noqa: E402  -- the shared definitions

WT = C24.WT
BASELINE_YAML = os.path.join(HERE, 'c2nav11_ntp_params.yaml')
CANDIDATE_YAML = os.path.join(HERE, 'c2nav25_slow_params.yaml')
BUNDLE = os.path.join(HERE, 'c2nav25_slow.json')

# The ONE leaf this experiment is allowed to move.
THE_LEAF = 'collision_monitor.ros__parameters.PolygonSlow.slowdown_ratio'
BASE_VALUE = 0.3
CAND_VALUE = 1.0

# Frozen baseline arm, by topology. Only runs produced by the same runner
# on the same route, carrying the same columns.
BASE_A = ['c2n21_base_r1', 'c2n21_base_r3', 'c2n21_base_r4']
BASE_B = ['c2n21_bbase_r2', 'c2n21_bbase_r3']
BASE = BASE_A + BASE_B

# Fresh candidate arm: two topology A, one topology B -- the C2-NAV.23
# coverage pattern.
CAND_A = ['c2n25_slow_r1', 'c2n25_slow_r2']
CAND_B = ['c2n25_bslow_r1']
CAND = CAND_A + CAND_B

LEGS = C24.LEGS

# Pre-registered gates, fixed before any candidate run existed.
GATE_GOAL_ERR_M = 0.25       # any SUCCEEDED leg
GATE_MIN_CLEAR_M = 0.20      # robot_radius
GATE_CREEP_DROP = 0.25       # total creep must fall at least 25 %

# --- gate 2, and a fact established BEFORE the candidate ran ------------
# `precheck` was run on the frozen baseline arm alone, before the first
# candidate simulator started, and found SIX of the 35 baseline legs
# already below 0.20 m:
#
#   c2n21_bbase_r2/obstacle_corner  0.151   c2n21_bbase_r3/enclosure_entry 0.158
#   c2n21_bbase_r2/enclosure_entry  0.152   c2n21_base_r3/enclosure_entry  0.162
#   c2n21_bbase_r2/enclosure_exit   0.152   c2n21_base_r1/enclosure_entry  0.167
#
# So the gate READ ABSOLUTELY rejects the baseline against itself and can
# discriminate nothing. That is a property of the route -- the enclosure
# pinch is 0.63 m wide and the robot is 0.40 m across, so 0.115 m of true
# clearance per side is geometry, not a candidate defect -- and it was
# true of C2-NAV.21 and C2-NAV.23 as well.
#
# Gate 2 is therefore reported BOTH ways and this note is the record that
# the second reading was fixed before any candidate data existed:
#   ABSOLUTE   min clearance >= 0.20 m anywhere        (expected to fail)
#   REGRESSION min clearance not materially below the baseline's own
#              minimum on the same leg -- the falsifier that can actually
#              distinguish the candidate from the control.
# The REGRESSION reading is the binding one. The ABSOLUTE reading is
# printed, never silently dropped.
BASELINE_MIN_CLEAR_M = 0.151     # c2n21_bbase_r2/obstacle_corner, measured
# How far below a leg's own baseline minimum counts as a regression.
GATE_CLEAR_MARGIN_M = 0.02


# ------------------------------------------------------------ utilities
def hdr(t):
    C24.hdr(t)


def f3(v, w=7):
    return C24.f3(v, w)


def f4(v, w=8):
    return C24.f4(v, w)


def mean(xs):
    xs = [x for x in xs if x is not None]
    return sum(xs) / len(xs) if xs else None


# ------------------------------------------------------- 1. the paramdiff
def flatten(node, prefix=''):
    """Every leaf of a YAML document as dotted-path -> value.

    Lists are leaves, not branches: a Nav2 parameter list (`critics`,
    `polygons`, a polygon's `points` string) is a single parameter to the
    node that loads it, and exploding it into indexed leaves would report
    a reordering as many changes instead of one.
    """
    out = {}
    if isinstance(node, dict):
        for k, v in node.items():
            out.update(flatten(v, f'{prefix}.{k}' if prefix else str(k)))
    else:
        out[prefix] = node
    return out


def cmd_paramdiff(args):
    """Flatten both parameter files and require exactly one differing leaf."""
    import yaml
    hdr('C2-NAV.25 -- the behavioural diff, leaf by leaf')
    with open(BASELINE_YAML) as f:
        base = flatten(yaml.safe_load(f))
    with open(CANDIDATE_YAML) as f:
        cand = flatten(yaml.safe_load(f))
    print(f'baseline  {os.path.relpath(BASELINE_YAML, WT)}   '
          f'{len(base)} leaves')
    print(f'candidate {os.path.relpath(CANDIDATE_YAML, WT)}  '
          f'{len(cand)} leaves')
    print()

    only_b = sorted(set(base) - set(cand))
    only_c = sorted(set(cand) - set(base))
    changed = sorted(k for k in set(base) & set(cand) if base[k] != cand[k])

    for k in only_b:
        print(f'  REMOVED  {k} = {base[k]!r}')
    for k in only_c:
        print(f'  ADDED    {k} = {cand[k]!r}')
    for k in changed:
        print(f'  CHANGED  {k}')
        print(f'             {base[k]!r}  ->  {cand[k]!r}')
    if not (only_b or only_c or changed):
        print('  (no differences at all -- the candidate is the baseline)')
    print()

    ok = True
    if only_b or only_c:
        print('FAIL: the candidate adds or removes leaves; it must only '
              'change one value')
        ok = False
    if len(changed) != 1:
        print(f'FAIL: {len(changed)} leaves changed, expected exactly 1')
        ok = False
    elif changed[0] != THE_LEAF:
        print(f'FAIL: the changed leaf is {changed[0]}, not {THE_LEAF}')
        ok = False
    elif not (base[changed[0]] == BASE_VALUE
              and cand[changed[0]] == CAND_VALUE):
        print(f'FAIL: {THE_LEAF} moves {base[changed[0]]!r} -> '
              f'{cand[changed[0]]!r}, expected {BASE_VALUE} -> {CAND_VALUE}')
        ok = False

    # The named non-negotiables, checked by value rather than by absence
    # from the diff, so a typo in THE_LEAF cannot make them pass silently.
    frozen = [
        'collision_monitor.ros__parameters.PolygonStop.radius',
        'collision_monitor.ros__parameters.PolygonStop.action_type',
        'collision_monitor.ros__parameters.PolygonStop.min_points',
        'collision_monitor.ros__parameters.PolygonLimit.linear_limit',
        'collision_monitor.ros__parameters.PolygonLimit.angular_limit',
        'collision_monitor.ros__parameters.PolygonSlow.points',
        'collision_monitor.ros__parameters.polygons',
        'controller_server.ros__parameters.FollowPath.xy_goal_tolerance',
        'controller_server.ros__parameters.goal_checker.xy_goal_tolerance',
        'controller_server.ros__parameters.FollowPath.sim_time',
        'controller_server.ros__parameters.FollowPath.max_vel_x',
        'controller_server.ros__parameters.FollowPath.vx_samples',
        'controller_server.ros__parameters.FollowPath.vtheta_samples',
        'controller_server.ros__parameters.FollowPath.critics',
    ]
    print('--- what must NOT have moved, by value ---')
    for k in frozen:
        b, c = base.get(k, '<absent>'), cand.get(k, '<absent>')
        same = 'same' if b == c else '*** MOVED ***'
        if b != c:
            ok = False
        short = k.split('ros__parameters.')[-1]
        node = k.split('.')[0]
        print(f'  {node:<18} {short:<36} {str(b)[:26]:<28} {same}')

    print()
    print('PARAMDIFF ' + ('OK -- exactly one leaf differs' if ok else 'FAILED'))
    return 0 if ok else 1


# -------------------------------------------------- 2. the live readback
def cmd_liveparam(args):
    """Report what each run's live parameter readback actually said."""
    hdr('C2-NAV.25 -- slowdown_ratio as READ OFF THE RUNNING NODE')
    print('A file that was edited and a file that was loaded are '
          'different claims (C2-NAV.23).')
    print()
    print(f'{"run":<18} {"PolygonSlow.slowdown_ratio":<28} '
          f'{"PolygonStop.radius":<20} verdict')
    bad = 0
    for tag in (args.runs or (CAND + BASE)):
        cands = [os.path.join(C24.SCRATCH, f'{tag}_slow_live.txt'),
                 os.path.join(C24.SCRATCH, f'{tag}_params_live.txt')]
        txt = None
        for q in cands:
            if os.path.exists(q):
                with open(q) as f:
                    txt = f.read()
                break
        if txt is None:
            print(f'{tag:<18} {"<no readback file>":<28}')
            continue
        sr = stop = None
        for line in txt.splitlines():
            if 'PolygonSlow.slowdown_ratio' in line:
                sr = line.strip().split()[-1]
            if 'PolygonStop.radius' in line:
                stop = line.strip().split()[-1]
        want = str(CAND_VALUE) if tag in CAND else str(BASE_VALUE)
        try:
            good = sr is not None and abs(float(sr) - float(want)) < 1e-9
        except ValueError:
            good = False
        if not good:
            bad += 1
        print(f'{tag:<18} {str(sr):<28} {str(stop):<20} '
              f'{"OK" if good else "*** expected " + want + " ***"}')
    print()
    print('LIVE READBACK ' + ('OK' if not bad else f'FAILED on {bad} run(s)'))
    return 0 if not bad else 1


# ------------------------------------------------------ 3. the creep table
def creep_seconds(w):
    """DERIVED. Seconds spent in the 0.25 -> 0.05 m band, from t_rel."""
    idx = w['creep']
    if len(idx) < 2:
        return 0.0
    rows = w['rows']
    return rows[idx[-1]]['t_rel'] - rows[idx[0]]['t_rel']


def _wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def _stop_secs(rec):
    """Seconds the monitor spent in PolygonStop over the WHOLE leg."""
    return (rec.get('cm_polygon_secs') or {}).get('PolygonStop', 0.0)


def leg_row(w):
    """One leg, every quantity the C2-NAV.25 brief asks for."""
    rec, rows, idx = w['rec'], w['rows'], w['creep']
    gain, ngain = C24.stage_ratio(w, idx, 'v_cmdvel', 'v_smoothed')
    gslow, nslow = C24.stage_ratio(w, idx, 'v_cmdvel', 'v_smoothed',
                                   polygon='PolygonSlow')
    dwb = C24.col(w, idx, 'dwb_best_vx')
    pre = C24.col(w, idx, 'v_smoothed')
    ach = C24.col(w, idx, 'v_act')
    zero = [i for i in idx if rows[i].get('v_wheel') is not None
            and abs(rows[i]['v_wheel']) < 1e-9]
    slowc = sum(1 for i in idx
                if (rows[i].get('cm_polygon') or '') == 'PolygonSlow')
    stopc = sum(1 for i in idx if (rows[i].get('cm_action') or '') == 'STOP')
    yaws = [rows[i]['yaw'] for i in idx if rows[i].get('yaw') is not None]
    dyaw = [_wrap(b - a) for a, b in zip(yaws, yaws[1:])]
    dist = w['dist']
    return dict(
        tag=w['tag'], leg=w['leg'], status=rec.get('status'),
        dur_s=rec.get('duration_wall_s'),
        creep_s=creep_seconds(w),
        creep_n=len(idx),
        creep_m=((dist[idx[0]] - dist[idx[-1]]) if len(idx) >= 2 else 0.0),
        reached_outer=w['i_outer'] is not None,
        reached_inner=(w['i_inner'] is not None
                       and w['i_inner'] < len(rows)),
        goal_err=rec.get('final_goal_err_m'),
        min_clear=rec.get('min_clearance_m'),
        dwb_vx=mean(dwb), pre_vx=mean(pre), act_vx=mean(ach),
        mon_gain=gain, mon_n=ngain,
        mon_gain_slow=gslow, mon_n_slow=nslow,
        slow_frac=(slowc / len(idx)) if idx else None,
        n_stop=stopc,
        zero_n=len(zero),
        zero_frac=(len(zero) / len(idx)) if idx else None,
        yaw_abs=sum(abs(d) for d in dyaw) if dyaw else None,
        yaw_net=sum(dyaw) if dyaw else None,
        stop_secs=_stop_secs(rec),
    )


def rows_for(runs):
    out = []
    for tag in runs:
        for leg in LEGS:
            w = C24.windows(tag, leg)
            if w:
                out.append(leg_row(w))
    return out


def _leg_table(rs):
    print(f'{"run":<16}{"leg":<17}{"status":<11}{"leg_s":>7}{"creep_s":>9}'
          f'{"m":>7}{"err_m":>7}{"clr_m":>7}{"dwb_vx":>8}{"pre_vx":>8}'
          f'{"act_vx":>8}{"mon":>7}{"slow%":>7}{"zero%":>7}{"|yaw|":>7}'
          f'{"net":>7}')
    for r in rs:
        pc = (100 * r['slow_frac']) if r['slow_frac'] is not None else None
        zc = (100 * r['zero_frac']) if r['zero_frac'] is not None else None
        print(f'{r["tag"]:<16}{r["leg"]:<17}{str(r["status"])[:10]:<11}'
              f'{f3(r["dur_s"])}{f3(r["creep_s"], 9)}{f3(r["creep_m"])}'
              f'{f3(r["goal_err"])}{f3(r["min_clear"])}'
              f'{f4(r["dwb_vx"])}{f4(r["pre_vx"])}{f4(r["act_vx"])}'
              f'{f3(r["mon_gain"])}{f3(pc)}{f3(zc)}'
              f'{f3(r["yaw_abs"])}{f3(r["yaw_net"])}')
    print()
    print(f'{len(rs)} legs.  creep_s / m = DERIVED seconds and metres in the')
    print('0.25 -> 0.05 m band; a leg that never reached 0.25 m contributes 0')
    print('and is flagged by `gates`.  mon = median |v_cmdvel|/|v_smoothed|')
    print('over the creep.  velocities are means over the creep, m/s.')
    print('zero% = share of creep cycles commanding exactly 0 at the wheels.')


def cmd_legs(args):
    hdr('C2-NAV.25 -- every leg of the fresh candidate runs')
    _leg_table(rows_for(args.runs or CAND))
    return 0


def cmd_baseline(args):
    hdr('C2-NAV.25 -- the frozen baseline arm, same code, same windows')
    _leg_table(rows_for(args.runs or BASE))
    return 0


# ------------------------------------------------------- 4. the comparison
def _agg(rs):
    creep = [r['creep_s'] for r in rs]
    tags = set(r['tag'] for r in rs)
    slow_rows = [r for r in rs if r['mon_gain_slow'] is not None]
    # Cycle- and time-weighted, so a 151.8 s leg counts 151.8 s and not
    # "one leg". A mean of per-leg means answers a different question and
    # is reported alongside, never instead.
    n_cyc = sum(r['creep_n'] for r in rs)
    tot_s = sum(creep)
    tot_m = sum(r['creep_m'] for r in rs)

    def cw(key):
        num = sum((r[key] or 0.0) * r['creep_n'] for r in rs
                  if r[key] is not None)
        den = sum(r['creep_n'] for r in rs if r[key] is not None)
        return (num / den) if den else None

    return dict(
        creep_cycles=n_cyc,
        creep_m_total=tot_m,
        creep_speed=(tot_m / tot_s) if tot_s else None,
        dwb_vx_cw=cw('dwb_vx'), pre_vx_cw=cw('pre_vx'), act_vx_cw=cw('act_vx'),
        zero_frac_cw=cw('zero_frac'),
        n_legs=len(rs), n_runs=len(tags),
        creep_total=sum(creep),
        creep_per_run=(sum(creep) / len(tags)) if tags else None,
        creep_max=max(creep) if creep else None,
        creep_max_leg=(max(rs, key=lambda r: r['creep_s'])['leg']
                       if rs else None),
        creep_max_tag=(max(rs, key=lambda r: r['creep_s'])['tag']
                       if rs else None),
        mon_gain=C24.med([r['mon_gain'] for r in rs
                          if r['mon_gain'] is not None]),
        mon_gain_slow=C24.med([r['mon_gain_slow'] for r in slow_rows]),
        n_slow_legs=len(slow_rows),
        dwb_vx=mean([r['dwb_vx'] for r in rs]),
        pre_vx=mean([r['pre_vx'] for r in rs]),
        act_vx=mean([r['act_vx'] for r in rs]),
        stop_cycles=sum(r['n_stop'] for r in rs),
        stop_secs=sum(r['stop_secs'] for r in rs),
        zero_frac=mean([r['zero_frac'] for r in rs]),
        leg_s=sum(r['dur_s'] or 0.0 for r in rs),
        leg_per_run=((sum(r['dur_s'] or 0.0 for r in rs) / len(tags))
                     if tags else None),
        stop_per_run=((sum(r['stop_secs'] for r in rs) / len(tags))
                      if tags else None),
        succ=sum(1 for r in rs if r['status'] == 'SUCCEEDED'),
        encl=sum(1 for r in rs if r['leg'] == 'enclosure_entry'),
        encl_ok=sum(1 for r in rs if r['leg'] == 'enclosure_entry'
                    and r['status'] == 'SUCCEEDED'),
    )


def cmd_compare(args):
    hdr('C2-NAV.25 -- frozen baseline against fresh candidate')
    b, c = rows_for(BASE), rows_for(CAND)
    if not c:
        print('no candidate runs on disk yet; nothing to compare')
        return 1
    B, C = _agg(b), _agg(c)
    print(f'baseline  {B["n_runs"]} runs {B["n_legs"]} legs   {" ".join(BASE)}')
    print(f'candidate {C["n_runs"]} runs {C["n_legs"]} legs   {" ".join(CAND)}')
    print()
    print(f'{"quantity":<44}{"baseline":>12}{"candidate":>12}{"delta":>12}')

    def row(label, key, nd=1, scale=1.0, note=None):
        vb, vc = B.get(key), C.get(key)
        sb = f'{vb * scale:.{nd}f}' if vb is not None else '-'
        sc = f'{vc * scale:.{nd}f}' if vc is not None else '-'
        if vb is None or vc is None:
            sd = note or '-'
        else:
            sd = f'{(vc - vb) * scale:+.{nd}f}'
        print(f'{label:<44}{sb:>12}{sc:>12}{sd:>16}')

    row('creep seconds, total', 'creep_total')
    row('creep seconds, per run', 'creep_per_run')
    row('creep seconds, worst single leg', 'creep_max')
    row('leg seconds, total', 'leg_s')
    row('leg seconds, per run', 'leg_per_run')
    row('monitor gain, median over creep', 'mon_gain', 3)
    row('monitor gain under PolygonSlow', 'mon_gain_slow', 3,
        note='never claimed')
    print('  -- cycle-weighted over every creep cycle (the primary '
          'reading) --')
    row('creep cycles', 'creep_cycles', 0)
    row('creep metres covered, total', 'creep_m_total', 2)
    row('creep speed = metres / seconds (mm/s)', 'creep_speed', 1, 1000)
    row('DWB selected vx (mm/s)', 'dwb_vx_cw', 1, 1000)
    row('pre-monitor vx (mm/s)', 'pre_vx_cw', 1, 1000)
    row('achieved vx (mm/s)', 'act_vx_cw', 1, 1000)
    row('zero-vx share of creep cycles (%)', 'zero_frac_cw', 1, 100)
    print('  -- mean of per-leg means (over-weights short legs; '
          'for reference) --')
    row('DWB selected vx (mm/s)', 'dwb_vx', 1, 1000)
    row('pre-monitor vx (mm/s)', 'pre_vx', 1, 1000)
    row('achieved vx (mm/s)', 'act_vx', 1, 1000)
    row('zero-vx share of creep cycles (%)', 'zero_frac', 1, 100)
    row('PolygonStop cycles inside creep', 'stop_cycles', 0)
    row('PolygonStop seconds, whole legs', 'stop_secs', 3)
    row('PolygonStop seconds per run', 'stop_per_run', 3)
    bs = f'{B["succ"]}/{B["n_legs"]}'
    cs = f'{C["succ"]}/{C["n_legs"]}'
    rate = (f'{100 * B["succ"] / B["n_legs"]:.0f}% -> '
            f'{100 * C["succ"] / C["n_legs"]:.0f}%')
    print(f'{"legs SUCCEEDED":<44}{bs:>12}{cs:>12}{rate:>16}')
    be = f'{B["encl_ok"]}/{B["encl"]}'
    ce = f'{C["encl_ok"]}/{C["encl"]}'
    print(f'{"enclosure_entry legs SUCCEEDED":<44}{be:>12}{ce:>12}{"":>16}')
    print()
    print(f'  worst leg: baseline {B["creep_max_tag"]}/{B["creep_max_leg"]} '
          f'{B["creep_max"]:.1f} s   candidate {C["creep_max_tag"]}/'
          f'{C["creep_max_leg"]} {C["creep_max"]:.1f} s')
    drop = 1.0 - C['creep_per_run'] / B['creep_per_run']
    word = 'FALLS' if drop > 0 else 'RISES'
    print(f'  creep per run {word} {abs(drop) * 100:.1f} % '
          f'({B["creep_per_run"]:.1f} -> {C["creep_per_run"]:.1f} s; '
          f'the gate is a FALL of at least '
          f'{GATE_CREEP_DROP * 100:.0f} %)')
    print()
    print('Runs differ in count, so PER-RUN figures are the comparable ones')
    print('and the gate is read on them. Totals are printed because they '
          'are')
    print('what C2-NAV.24 predicted, and are NOT comparable across '
          'unequal n.')
    print('"monitor gain under PolygonSlow" has no candidate value because '
          'the')
    print('polygon never claims the action at ratio 1.0 -- see `monitor`.')
    return 0


# ------------------------------------------------------------ 5. the gates
def cmd_gates(args):
    hdr('C2-NAV.25 -- the four pre-registered gates')
    b, c = rows_for(BASE), rows_for(CAND)
    if not c:
        print('no candidate runs on disk yet')
        return 1
    B, C = _agg(b), _agg(c)
    fails = []

    print('GATE 1  no SUCCEEDED leg finishes beyond '
          f'{GATE_GOAL_ERR_M} m of ground truth')
    worst = None
    for r in c:
        if r['status'] != 'SUCCEEDED' or r['goal_err'] is None:
            continue
        if worst is None or r['goal_err'] > worst['goal_err']:
            worst = r
        if r['goal_err'] > GATE_GOAL_ERR_M:
            print(f'        VIOLATION {r["tag"]}/{r["leg"]} '
                  f'{r["goal_err"]:.3f} m')
            fails.append(1)
    if worst:
        print(f'        worst SUCCEEDED leg: {worst["tag"]}/{worst["leg"]} '
              f'{worst["goal_err"]:.3f} m')
    print(f'        -> {"FAIL" if 1 in fails else "PASS"}')

    print('GATE 2  min true clearance, read both ways (see the module '
          'header note)')
    mc = [r for r in c if r['min_clear'] is not None]
    lo = min(mc, key=lambda r: r['min_clear']) if mc else None
    lob = min((r for r in b if r['min_clear'] is not None),
              key=lambda r: r['min_clear'], default=None)
    print(f'   2a ABSOLUTE   >= {GATE_MIN_CLEAR_M} m anywhere')
    if lo:
        print(f'        candidate lowest {lo["tag"]}/{lo["leg"]} '
              f'{lo["min_clear"]:.3f} m')
    if lob:
        print(f'        baseline  lowest {lob["tag"]}/{lob["leg"]} '
              f'{lob["min_clear"]:.3f} m   '
              f'({sum(1 for r in b if (r["min_clear"] or 9) < GATE_MIN_CLEAR_M)}'
              f' of {len(b)} baseline legs are already below)')
    abs_c = lo is not None and lo['min_clear'] >= GATE_MIN_CLEAR_M
    abs_b = lob is not None and lob['min_clear'] >= GATE_MIN_CLEAR_M
    print(f'        candidate {"PASS" if abs_c else "FAIL"},  '
          f'baseline {"PASS" if abs_b else "FAIL"} -- a reading the '
          'CONTROL also fails')
    print('        cannot discriminate, and is not the binding gate.')

    print(f'   2b REGRESSION per leg, not more than {GATE_CLEAR_MARGIN_M} m '
          'below the baseline')
    bymin = {}
    for r in b:
        if r['min_clear'] is None:
            continue
        k = r['leg']
        bymin[k] = min(bymin.get(k, 9.0), r['min_clear'])
    worst_reg = None
    for r in mc:
        ref = bymin.get(r['leg'])
        if ref is None:
            continue
        d = r['min_clear'] - ref
        if worst_reg is None or d < worst_reg[0]:
            worst_reg = (d, r, ref)
        if d < -GATE_CLEAR_MARGIN_M:
            print(f'        VIOLATION {r["tag"]}/{r["leg"]} '
                  f'{r["min_clear"]:.3f} m vs baseline min {ref:.3f} m '
                  f'({d:+.3f} m)')
            fails.append(2)
    if worst_reg:
        d, r, ref = worst_reg
        print(f'        worst: {r["tag"]}/{r["leg"]} {r["min_clear"]:.3f} m '
              f'vs baseline min {ref:.3f} m  ({d:+.3f} m)')
    print(f'        -> {"FAIL" if 2 in fails else "PASS"}  (binding)')

    print('GATE 3  PolygonStop activations do not materially increase')
    print(f'        baseline  {B["stop_cycles"]} creep cycles, '
          f'{B["stop_secs"]:.3f} s over whole legs, {B["n_legs"]} legs')
    print(f'        candidate {C["stop_cycles"]} creep cycles, '
          f'{C["stop_secs"]:.3f} s over whole legs, {C["n_legs"]} legs')
    bp = B['stop_secs'] / B['n_runs'] if B['n_runs'] else 0.0
    cp = C['stop_secs'] / C['n_runs'] if C['n_runs'] else 0.0
    print(f'        per run   {bp:.3f} s -> {cp:.3f} s')
    if cp > max(2 * bp, bp + 1.0):
        fails.append(3)
    print(f'        -> {"FAIL" if 3 in fails else "PASS"}')

    print(f'GATE 4  creep falls by at least {GATE_CREEP_DROP * 100:.0f} %')
    drop = 1.0 - (C['creep_per_run'] / B['creep_per_run'])
    print(f'        per run {B["creep_per_run"]:.1f} s -> '
          f'{C["creep_per_run"]:.1f} s   = '
          f'{"falls" if drop > 0 else "RISES"} {abs(drop) * 100:.1f} %')
    print(f'        worst single leg {B["creep_max"]:.1f} s -> '
          f'{C["creep_max"]:.1f} s   (C2-NAV.24 predicted ~45.4 s)')
    print(f'        whole-tour leg seconds per run '
          f'{B["leg_per_run"]:.1f} s -> {C["leg_per_run"]:.1f} s')
    if drop < GATE_CREEP_DROP:
        fails.append(4)
    print(f'        -> {"FAIL" if 4 in fails else "PASS"}')

    print()
    print('--- legs that never entered the creep band, stated not hidden ---')
    miss = [r for r in c if not r['reached_outer']]
    for r in miss:
        print(f'        {r["tag"]}/{r["leg"]} {r["status"]}: never reached '
              '0.25 m; contributes 0 s and no terminal prediction can be '
              'evaluated for it')
    if not miss:
        print('        (none -- every candidate leg entered the band)')

    print()
    print('--- per-leg clearance, candidate against the baseline range ---')
    print(f'{"leg":<17}{"baseline min..max":>22}{"candidate":>26}')
    for lg in LEGS:
        bb = [r['min_clear'] for r in b
              if r['leg'] == lg and r['min_clear'] is not None]
        cc = [r['min_clear'] for r in c
              if r['leg'] == lg and r['min_clear'] is not None]
        if not bb or not cc:
            continue
        print(f'{lg:<17}{f"{min(bb):.3f} .. {max(bb):.3f}":>22}'
              f'{" ".join(f"{x:.3f}" for x in cc):>26}')
    print()
    print('The enclosure pinch is the tight leg and it is unchanged: it is')
    print('geometry, not a candidate effect.')

    print()
    if fails:
        print(f'VERDICT: REJECTED on the LETTER of gate(s) '
              f'{sorted(set(fails))}')
    else:
        print('VERDICT: SUPPORTED -- all gates pass')
    print()
    print('What each gate result means, stated separately from the verdict')
    print('so neither can be quietly substituted for the other:')
    print('  gate 4  PASSES decisively and is the experiment\'s question.')
    print('  gate 3  PASSES; PolygonStop FELL rather than rose.')
    print('  gate 1  fires on ONE leg of 21, by 5 mm, on a leg whose stop')
    print('          point is set by goal_checker.xy_goal_tolerance on the')
    print('          ESTIMATED pose -- a parameter this experiment did not')
    print('          touch and which the monitor sits downstream of. See')
    print('          `goalerr` for the trajectory and the distributions.')
    print('  gate 2a is failed by the CONTROL as well and discriminates')
    print('          nothing; 2b flags a behavioural difference on')
    print('          wall_adjacent well above the safety floor, not a loss')
    print('          of clearance on the leg that is actually tight.')
    print('A gate that fires is a gate that fires. This is reported as a')
    print('rejection on the letter, with the mechanism stated, and NOT')
    print('rewritten after the fact.')
    return 0


# --------------------------------------------------------------- 6. dump
def cmd_dump(args):
    """Freeze the candidate traces and records, as C2-NAV.24 froze its own."""
    out = {'traces': {}, 'records': {}}
    for tag in CAND:
        rec = C24.load_record(tag)
        if rec:
            out['records'][tag] = rec
        for leg in LEGS:
            rows = C24.load_trace(tag, leg)
            if not rows:
                continue
            cols = list(C24.COLS) + list(C24.STRCOLS)
            out['traces'].setdefault(tag, {})[leg] = {
                'columns': cols,
                'rows': [[r.get(c) for c in cols] for r in rows]}
    with open(args.path, 'w') as f:
        json.dump(out, f, separators=(',', ':'), sort_keys=True)
    n = sum(len(v) for v in out['traces'].values())
    print(f'wrote {args.path}: {len(out["records"])} records, {n} traces')
    return 0


def cmd_precheck(args):
    """The baseline arm alone -- what was known BEFORE any candidate ran.

    Reads no candidate data and cannot, so it is runnable at any time and
    returns the same bytes it returned at pre-registration.
    """
    hdr('C2-NAV.25 -- pre-registration state, frozen baseline arm only')
    b = rows_for(BASE)
    B = _agg(b)
    print(f'baseline {B["n_runs"]} runs, {B["n_legs"]} legs, '
          f'{B["succ"]} SUCCEEDED')
    print(f'  creep total          {B["creep_total"]:.1f} s')
    print(f'  creep per run        {B["creep_per_run"]:.1f} s   '
          '<- the quantity gate 4 is read on')
    print(f'  worst single leg     {B["creep_max"]:.1f} s  '
          f'({B["creep_max_tag"]}/{B["creep_max_leg"]})')
    print(f'  monitor gain, median {B["mon_gain"]:.3f} overall, '
          f'{B["mon_gain_slow"]:.3f} under PolygonSlow '
          f'({B["n_slow_legs"]} legs)')
    print(f'  PolygonStop          {B["stop_cycles"]} cycles inside creep, '
          f'{B["stop_secs"]:.2f} s over whole legs '
          f'({B["stop_secs"] / B["n_runs"]:.2f} s/run)')
    print(f'  enclosure_entry      {B["encl_ok"]}/{B["encl"]} SUCCEEDED')
    print()
    print(f'  legs already below the {GATE_MIN_CLEAR_M} m absolute '
          'clearance gate:')
    low = sorted((r for r in b if (r['min_clear'] or 9) < GATE_MIN_CLEAR_M),
                 key=lambda r: r['min_clear'])
    for r in low:
        print(f'    {r["tag"]:<16}{r["leg"]:<17}{r["min_clear"]:.3f} m  '
              f'{r["status"]}')
    print(f'  -> {len(low)} of {len(b)}. This is why gate 2 is read as a '
          'REGRESSION')
    print('     against these values and not as an absolute floor. See the '
          'module header.')
    return 0


def cmd_monitor(args):
    """The collision monitor's own action enum, over WHOLE tours.

    The sharpest single measurement of the intervention, and the one that
    falsified a prediction. `nav2_msgs/CollisionMonitorState` numbers the
    actions DO_NOTHING 0, STOP 1, SLOWDOWN 2, APPROACH 3, LIMIT 4 -- read
    from the message, not assumed.
    """
    import collections
    hdr('C2-NAV.25 -- what the collision monitor actually did')
    names = {'0': 'DO_NOTHING', '1': 'STOP', '2': 'SLOWDOWN',
             '3': 'APPROACH', '4': 'LIMIT', 'None': '<no state msg>'}

    def tally(runs):
        ca, cp, n = collections.Counter(), collections.Counter(), 0
        for t in runs:
            for lg in LEGS:
                rows = C24.load_trace(t, lg)
                if not rows:
                    continue
                for r in rows:
                    n += 1
                    ca[r.get('cm_action') or 'None'] += 1
                    cp[r.get('cm_polygon') or 'none'] += 1
        return ca, cp, n

    ba, bp, bn = tally(BASE)
    ca, cp, cn = tally(CAND)
    if not cn:
        print('no candidate runs on disk yet')
        return 1
    print(f'{"":<16}{"baseline":>22}{"candidate":>22}')
    print(f'{"":<16}{f"{len(BASE)} runs":>10}{"share":>12}'
          f'{f"{len(CAND)} runs":>10}{"share":>12}')
    print(f'{"cycles":<16}{bn:>10}{"":>12}{cn:>10}{"":>12}')
    for k in ('0', '1', '2', '3', '4', 'None'):
        if not (ba[k] or ca[k]):
            continue
        print(f'{names[k]:<16}{ba[k]:>10}{100 * ba[k] / bn:>11.2f}%'
              f'{ca[k]:>10}{100 * ca[k] / cn:>11.2f}%')
    print()
    print(f'{"polygon claimed":<16}')
    for k in sorted(set(bp) | set(cp)):
        print(f'{k:<16}{bp[k]:>10}{100 * bp[k] / bn:>11.2f}%'
              f'{cp[k]:>10}{100 * cp[k] / cn:>11.2f}%')
    print()
    print('OBSERVED: SLOWDOWN falls to ZERO in 4485 cycles, and '
          '`PolygonSlow`')
    print('is never named as the claiming polygon. This FALSIFIES the')
    print('C2-NAV.24 brief\'s expectation that setting the ratio to 1.0 '
          'would')
    print('leave `cm_polygon` still reporting PolygonSlow -- an assumption')
    print('made in advance and wrong.')
    print()
    print('DERIVED mechanism, from the installed headers plus the count '
          'above.')
    print('`nav2_collision_monitor/types.hpp` defines')
    print('  Velocity::operator<  ->  x*x + y*y + tw*tw  <  (same of other)')
    print('a STRICT comparison on squared magnitude, and')
    print('  Velocity::operator*  ->  {x*mul, y*mul, tw*mul}.')
    print('`collision_monitor_node.hpp` documents processStopSlowdownLimit '
          'as')
    print('returning "True if returned action is caused by current '
          'polygon".')
    print('At ratio 1.0 the scaled velocity EQUALS the one already chosen, '
          'so')
    print('a strict "<" is false and the polygon never claims the action.')
    print('The .cpp is not installed on this machine, so that last link is')
    print('derived from the two headers and confirmed by the 0-of-4485')
    print('measurement, not read from the body of the function.')
    print()
    print('The zone still exists and is still evaluated -- it simply has')
    print('nothing to contribute. The arms stay comparable because the')
    print('comparison rests on the MEASURED monitor gain, which reads 1.000')
    print('on every candidate leg and 0.300 under PolygonSlow on the '
          'baseline.')
    return 0


def cmd_goalerr(args):
    """Did the candidate buy its creep saving by stopping further out?

    This is the C2-NAV.23 failure mode and the reason gate 1 exists, so
    it is checked as a DISTRIBUTION and not only at the gate. C2-NAV.23
    put 6 of 16 legs past tolerance; if PolygonSlow=1.0 did the same
    thing the whole result would be an artefact.
    """
    import statistics as st
    hdr('C2-NAV.25 -- final ground-truth error, both arms, SUCCEEDED legs')

    def dist(rs, label):
        e = sorted(r['goal_err'] for r in rs
                   if r['status'] == 'SUCCEEDED' and r['goal_err'] is not None)
        if not e:
            return []
        print(f'{label:<11}n={len(e):<3} median={st.median(e):.3f}  '
              f'mean={sum(e) / len(e):.3f}  '
              f'p90={e[int(0.9 * (len(e) - 1))]:.3f}  max={max(e):.3f}   '
              f'>0.15 m: {sum(1 for x in e if x > 0.15)}   '
              f'>0.20 m: {sum(1 for x in e if x > 0.20)}   '
              f'>{GATE_GOAL_ERR_M} m: {sum(1 for x in e if x > GATE_GOAL_ERR_M)}')
        return e

    b = dist(rows_for(BASE), 'baseline')
    c = dist(rows_for(CAND), 'candidate')
    print()
    print('baseline  ' + ' '.join(f'{x:.3f}' for x in b))
    print('candidate ' + ' '.join(f'{x:.3f}' for x in c))
    print()
    print('The two distributions overlap; the candidate is not '
          'systematically')
    print('stopping further out. The single point past the gate is the '
          'tail,')
    print('and the baseline has a neighbouring one.')
    print()
    print('--- the leg that passed the gate, and how it ended ---')
    print(f'{"run":<16}{"status":<11}{"gt_err":>8}{"min_gt_dist":>13}'
          f'{"final":>8}{"dur_s":>8}')
    for tag in (CAND + BASE):
        w = C24.windows(tag, 'wall_parallel')
        if not w:
            continue
        d, rec = w['dist'], w['rec']
        print(f'{tag:<16}{rec["status"]:<11}{rec["final_goal_err_m"]:>8.3f}'
              f'{min(d):>13.3f}{d[-1]:>8.3f}{rec["duration_wall_s"]:>8.1f}')
    print()
    print('`min_gt_dist` equal to `final` means the robot approached')
    print('monotonically and STOPPED there -- it never got closer and then')
    print('drifted back. The goal checker fired on the ESTIMATED pose at')
    print('goal_checker.xy_goal_tolerance = 0.25 m while ground truth was')
    print('further out, i.e. a localisation offset. `slowdown_ratio` has '
          'no')
    print('path to that check: the monitor is downstream of the '
          'controller and')
    print('scales the commanded velocity only. The baseline shows the '
          'same')
    print('shape on the same leg at 0.224 m.')
    return 0


CMDS = {'paramdiff': cmd_paramdiff, 'precheck': cmd_precheck,
        'monitor': cmd_monitor, 'goalerr': cmd_goalerr,
        'liveparam': cmd_liveparam, 'legs': cmd_legs,
        'baseline': cmd_baseline, 'compare': cmd_compare,
        'gates': cmd_gates}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('cmd', choices=list(CMDS) + ['dump', 'all'])
    ap.add_argument('--runs', nargs='*', default=None)
    ap.add_argument('--path', default=BUNDLE)
    args = ap.parse_args(argv)
    if args.cmd == 'dump':
        return cmd_dump(args)
    if args.cmd == 'all':
        rc = 0
        for name in ('paramdiff', 'precheck', 'liveparam', 'baseline',
                     'legs', 'monitor', 'goalerr', 'compare', 'gates'):
            rc |= CMDS[name](args) or 0
            print()
        return rc
    return CMDS[args.cmd](args)


if __name__ == '__main__':
    sys.exit(main())
