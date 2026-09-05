#!/usr/bin/env python3
"""C2-NAV.23 -- does FollowPath.xy_goal_tolerance 0.05 -> 0.25 remove the
terminal creep/relatch pathology without losing ground-truth position?

THE EXPERIMENT
--------------
Exactly ONE behavioural parameter differs from the frozen C2-NAV.20/.21
baseline `c2nav11_ntp_params.yaml` (sha256 6f61e499...1e6bb950):

    controller_server/FollowPath/xy_goal_tolerance : 0.05 -> 0.25

That is asserted here two ways, and the module refuses to report anything
until both pass:

  * PARSED, not textual. The two YAML files are flattened to leaves and
    compared; exactly one leaf may differ. A textual diff would pass a
    change that YAML merges away, and would fail on a comment.
  * LIVE, not on disk. `.navbench/results/<tag>_fpxy_live.txt` carries
    `FollowPath.xy_goal_tolerance` read back off the running
    `controller_server` for every candidate run. A file that was edited
    and a file that was LOADED are different claims, and the existing
    runner proves only the first: `c2n6_verify.sh` reads
    `goal_checker.xy_goal_tolerance` and not FollowPath's.

WHY THIS PARAMETER
------------------
C2-NAV.22 measured that across 2300 rotate-in-place cycles DWB turns away
from the goal heading on 885 (38.5 %), and that every one is either a
full one-sign Oscillation ban (191) or a cycle with RotateToGoalCritic
inactive (694) -- NEITHER = 0. The critic goes inactive because
`DWBLocalPlanner::setPlan` resets every critic each time the BT
republishes the path (measured period 2.95-3.04 s) and
`RotateToGoalCritic` re-latches only within `FollowPath.xy_goal_tolerance`
= 0.05 m, a window smaller than this stack's own position error. Raising
it to the goal checker's own 0.25 m is the single line that addresses the
dominant mechanism.

Verified single-variable at the source, not assumed: across nav2 1.3.11
the only consumer of `FollowPath.xy_goal_tolerance` is
`RotateToGoalCritic` (rotate_to_goal.cpp:61-64); `SimpleGoalChecker`
reads its own namespaced copy (simple_goal_checker.cpp:75-86).

THE FALSIFIER, FIXED BEFORE THE RUN
-----------------------------------
With `in_window_` latched at 0.25 m the robot may stop translating as
soon as it is inside 0.25 m, so a leg could report SUCCESS while sitting
short of its goal. The candidate FAILS if any fresh leg finishes with a
ground-truth `final_goal_err_m` > 0.25 m. `falsifier` reports that over
every leg of every fresh tour, against the same statistic on the frozen
baseline, so a pre-existing condition cannot be scored as a regression.

FRAME CAVEAT, INHERITED AND UNCHANGED
-------------------------------------
The traces record Gazebo ground truth; the goal checker and every critic
are fed the AMCL estimate. C2-NAV.22 measured 18 of 18 ordinary legs
stopping 0.194-0.492 rad from a heading target they pass. Absolute
heading arrival therefore cannot be dated from these artifacts and is not
claimed here. Yaw travel, sign reversals and turn direction are
frame-independent and carry the argument; the latch state is read from
the critic's own per-cycle rejection count.

USAGE
-----
    python3 -P docs/data/c2nav23_fpxy.py gate
    python3 -P docs/data/c2nav23_fpxy.py falsifier
    python3 -P docs/data/c2nav23_fpxy.py phases
    python3 -P docs/data/c2nav23_fpxy.py latch
    python3 -P docs/data/c2nav23_fpxy.py terminal
    python3 -P docs/data/c2nav23_fpxy.py attribution
    python3 -P docs/data/c2nav23_fpxy.py verdict
    python3 -P docs/data/c2nav23_fpxy.py all
    python3 -P docs/data/c2nav23_fpxy.py dump docs/data/c2nav23_fpxy.json
"""

import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
# -P is used deliberately (the repo's own sys.path[0] trap), so the
# sibling module has to be put on the path explicitly rather than
# inherited from the script's directory.
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import c2nav22_yaw as y                                    # noqa: E402

WT = y.WT
# Pointing this at a path that does not exist is how the bundle's
# self-sufficiency is checked: `falsifier`, `paired` and `arrival` -- the
# three tables that carry the verdict -- must produce identical numbers
# with the scratch runs gone. C2-NAV.22 verified its own bundle the same
# way.
SCRATCH = os.environ.get('C2NAV23_SCRATCH', y.SCRATCH)
BASELINE_PARAMS = os.path.join(HERE, 'c2nav11_ntp_params.yaml')
CAND_PARAMS = os.path.join(HERE, 'c2nav23_fpxy_params.yaml')
BASELINE_SHA = ('6f61e49912765708e70470df967b23834338723176bc'
                'f7ae113f8b8c1e6bb950')
THE_LEAF = '/controller_server/ros__parameters/FollowPath/xy_goal_tolerance'
FROM_VAL, TO_VAL = 0.05, 0.25
BUNDLE = os.path.join(HERE, 'c2nav23_fpxy.json')

# The fresh C2-NAV.23 runs, in the order they were run. Topology A twice
# and topology B once: the pathological terminal case reproduces roughly
# one tour in four on A, and was present on 2 of 3 baseline B tours.
CAND = [
    ('c2n23_fpxy_r1', 'A'),
    ('c2n23_fpxy_r2', 'A'),
    ('c2n23_bfpxy_r1', 'B'),
]
# The frozen baseline: exactly C2-NAV.22's ORDER, which is what the
# published tables report. `c2n19_tour_r1` never reached the outer
# tolerance and is carried so that fact stays visible.
BASE = list(y.ORDER)
# The subset carrying the C2-NAV.21 per-cycle critic columns, i.e. the
# legs a latch comparison is even defined on.
BASE_LATCH = ['c2n21_base_r1', 'c2n21_base_r3', 'c2n21_base_r4',
              'c2n21_fpd_r3', 'c2n21_bbase_r2', 'c2n21_bbase_r3']
TOPO = {'c2n21_bbase_r2': 'B', 'c2n21_bbase_r3': 'B'}

FALSIFIER_M = 0.25


def topo(tag):
    if tag in TOPO:
        return TOPO[tag]
    for t, tp in CAND:
        if t == tag:
            return tp
    return 'A'


# ------------------------------------------------------------ provenance
def _flat(d, p=''):
    o = {}
    if isinstance(d, dict):
        for k, v in d.items():
            o.update(_flat(v, p + '/' + str(k)))
    else:
        o[p] = d
    return o


def param_diff():
    """The parsed-leaf diff between the frozen baseline and the candidate."""
    import yaml
    with open(BASELINE_PARAMS) as f:
        a = _flat(yaml.safe_load(f))
    with open(CAND_PARAMS) as f:
        b = _flat(yaml.safe_load(f))
    ks = set(a) | set(b)
    return [(k, a.get(k, '<absent>'), b.get(k, '<absent>')) for k in sorted(ks)
            if a.get(k, '<absent>') != b.get(k, '<absent>')]


def _sha(p):
    import hashlib
    h = hashlib.sha256()
    with open(p, 'rb') as f:
        for chunk in iter(lambda: f.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()


def live_fpxy(tag):
    """FollowPath.xy_goal_tolerance as read back off the RUNNING node."""
    p = os.path.join(SCRATCH, f'{tag}_fpxy_live.txt')
    if not os.path.exists(p):
        return None
    with open(p) as f:
        for line in f:
            if 'FollowPath.xy_goal_tolerance' in line:
                for tok in line.replace(':', ' ').split():
                    try:
                        return float(tok)
                    except ValueError:
                        continue
    return None


def have(tag):
    """Was this run actually made? Scratch first, then the frozen bundle."""
    if os.path.exists(os.path.join(SCRATCH, f'{tag}.json')):
        return True
    return bool(_bundle23().get('legs', {}).get(tag))


def cmd_gate():
    y.hdr('C2-NAV.23 gate -- one parameter, and it was actually loaded')
    ok = True

    sha = _sha(BASELINE_PARAMS)
    good = sha == BASELINE_SHA
    ok &= good
    print(f'frozen baseline sha256   {sha[:16]}...  '
          f'{"MATCHES C2-NAV.20/.21/.22" if good else "*** MOVED ***"}')

    d = param_diff()
    print(f'parsed leaves differing baseline -> candidate: {len(d)}')
    for k, a, b in d:
        print(f'   {k}\n       {a!r} -> {b!r}')
    one = (len(d) == 1 and d[0][0] == THE_LEAF
           and abs(float(d[0][1]) - FROM_VAL) < 1e-12
           and abs(float(d[0][2]) - TO_VAL) < 1e-12)
    ok &= one
    print('   -> ' + ('EXACTLY the intended single change' if one
                      else '*** NOT the intended change ***'))

    print()
    print('and read back off the LIVE controller_server, per fresh run:')
    print(f'{"tag":<18} {"topology":>8} {"live FollowPath.xy_goal_tol":>28} '
          f'{"record":>8}')
    print('-' * 68)
    n_run = 0
    for tag, tp in CAND:
        if not have(tag):
            print(f'{tag:<18} {tp:>8} {"-- not run --":>28} {"no":>8}')
            continue
        n_run += 1
        v = live_fpxy(tag)
        good = v is not None and abs(v - TO_VAL) < 1e-9
        ok &= good
        print(f'{tag:<18} {tp:>8} '
              f'{(f"{v}" if v is not None else "MISSING"):>28} '
              f'{"yes":>8}   {"OK" if good else "*** WRONG ***"}')
    if n_run == 0:
        ok = False
        print('no fresh runs present')

    print()
    print('and the frozen C2-NAV.22 reconstruction still passes '
          '(its own gate, unchanged):')
    try:
        y.cmd_selftest()
    except SystemExit as e:
        if e.code:
            ok = False
    print()
    print('GATE PASSED' if ok else 'GATE FAILED')
    return ok


# ------------------------------------------------------------- falsifier
def legs(tag):
    """The tour's per-leg nav_bench records, from scratch or the bundle."""
    p = os.path.join(SCRATCH, f'{tag}.json')
    if not os.path.exists(p):
        return _bundle23().get('legs', {}).get(tag, [])
    with open(p) as f:
        return json.load(f).get('legs', [])


def cmd_falsifier():
    y.hdr('C2-NAV.23 CRITICAL FALSIFIER -- ground-truth position at the '
          'end of every leg')
    print('Fixed before the run: the candidate FAILS if any fresh leg '
          'finishes with a')
    print(f'ground-truth final_goal_err_m > {FALSIFIER_M} m, even if Nav2 '
          'reports SUCCESS.')
    print('`final_goal_err_m` is nav_bench\'s own field, computed from '
          '/model/coco/odometry')
    print('(Gazebo ground truth), not from AMCL.')
    print()
    out = {'candidate': [], 'baseline': []}
    for label, tags in (('CANDIDATE (fresh)', [t for t, _ in CAND]),
                        ('BASELINE (frozen, for the same statistic)',
                         ['c2n21_base_r1', 'c2n21_base_r2', 'c2n21_base_r3',
                          'c2n21_base_r4', 'c2n21_bbase_r1',
                          'c2n21_bbase_r2', 'c2n21_bbase_r3'])):
        print(f'--- {label} ---')
        print(f'{"tag":<18} {"leg":<18} {"status":<10} {"err_m":>7} '
              f'{"> tol":>6}')
        print('-' * 64)
        for tag in tags:
            ls = legs(tag)
            if not ls:
                continue
            for r in ls:
                e = r.get('final_goal_err_m')
                bad = e is not None and e > FALSIFIER_M
                rec = dict(tag=tag, leg=r.get('scenario'),
                           status=r.get('status'), err=e, over=bool(bad))
                out['candidate' if label.startswith('CANDIDATE')
                    else 'baseline'].append(rec)
                print(f'{tag:<18} {r.get("scenario",""):<18} '
                      f'{r.get("status",""):<10} '
                      f'{(e if e is not None else float("nan")):>7.3f} '
                      f'{("YES" if bad else ""):>6}')
            print()
    print('=' * 78)
    for key, label in (('candidate', 'CANDIDATE'), ('baseline', 'BASELINE')):
        rs = out[key]
        if not rs:
            continue
        succ = [r for r in rs if r['status'] == 'SUCCEEDED']
        over_all = [r for r in rs if r['over']]
        over_succ = [r for r in succ if r['over']]
        print(f'{label}: {len(rs)} legs, {len(succ)} SUCCEEDED')
        print(f'   legs over {FALSIFIER_M} m, ALL statuses      : '
              f'{len(over_all)}'
              + (('  -> ' + ', '.join(f'{r["tag"]}/{r["leg"]}'
                                      f'({r["status"]},{r["err"]:.3f})'
                                      for r in over_all)) if over_all else ''))
        print(f'   legs over {FALSIFIER_M} m, SUCCEEDED only    : '
              f'{len(over_succ)}'
              + (('  -> ' + ', '.join(f'{r["tag"]}/{r["leg"]}'
                                      f'({r["err"]:.3f})'
                                      for r in over_succ))
                 if over_succ else ''))
        es = [r['err'] for r in succ if r['err'] is not None]
        if es:
            print(f'   SUCCEEDED final error  max {max(es):.3f} m   '
                  f'median {y.med(es):.3f} m')
        print()
    print('Read the SUCCEEDED-only line as the silent-failure test: a leg '
          'that reports')
    print('TIMEOUT has not claimed to have arrived, and both arms have '
          'those.')
    return out


# ------------------------------------------------------- terminal metrics
def _record(tag):
    for r in legs(tag):
        if r.get('scenario') == y.LEG:
            return r
    return y._bundle().get('records', {}).get(tag)


def metrics(tag):
    """Every C2-NAV.23 per-leg number, candidate or baseline alike."""
    a = y.analyse(tag)
    rows, d, i_out, i_in = y._prep(tag)
    rec = _record(tag) or {}
    m = dict(tag=tag, topo=topo(tag),
             status=rec.get('status'),
             leg_s=rec.get('duration_sim_s'),
             final_err_m=rec.get('final_goal_err_m'),
             t_out=a.get('t_arrive_outer'), t_in=a.get('t_arrive_inner'),
             dist_min=a.get('dist_min'), dist_final=a.get('dist_final'),
             t_end=a.get('t_end'), t_settle=a.get('t_settle'),
             yaw_abs=a.get('yaw_travel_abs'),
             yaw_signed=a.get('yaw_travel_signed'),
             yaw_wasted=a.get('yaw_travel_wasted'),
             yaw_needed=a.get('yaw_needed'),
             travel_ratio=a.get('travel_ratio'),
             flips_w_act=a.get('n_sign_flips_w_act'),
             flips_w_nav=a.get('n_sign_flips_w_nav'),
             flips_dwb=a.get('n_sign_flips_dwb_best_wz'),
             zero_cross=a.get('n_err_zero_crossings'),
             n_reexit=a.get('n_heading_window_exits'),
             cm_polygon_secs=rec.get('cm_polygon_secs'),
             cm_action_frac=rec.get('cm_action_frac'),
             worst_crawl=rec.get('worst_crawl'),
             n_progress_failures=rec.get('n_progress_failures'),
             plan_period=y._plan_period(tag))
    if i_out is None:
        m['note'] = 'never reached the 0.25 m outer tolerance'
        return m
    end = rows[-1]['t_rel']
    t_out = rows[i_out]['t_rel']
    j_in = len(rows) - 1 if i_in is None else i_in
    t_in = end if i_in is None else rows[i_in]['t_rel']
    m['transit_s'] = t_out
    m['creep_s'] = t_in - t_out
    m['rotate_s'] = end - t_in
    m['terminal_s'] = end - t_out
    m['terminal_frac'] = (end - t_out) / end if end else None
    m['creep_m'] = y._pathlen(rows, i_out, j_in)
    m['creep_v'] = (m['creep_m'] / m['creep_s']) if m['creep_s'] else None
    m['yaw_creep'] = y._travel(rows, i_out, j_in)
    m['yaw_rot'] = y._travel(rows, j_in, len(rows) - 1)
    m['path_in_ball_m'] = y._pathlen(rows, i_out, len(rows) - 1)

    # latch, read from the critic's own per-cycle rejection count
    if y._has_latch(rows, tag):
        term = rows[i_out:]
        lat = [y._latched(r) for r in term]
        n = len(lat)
        m['term_cycles'] = n
        m['latched'] = sum(1 for v in lat if v)
        m['latched_frac'] = m['latched'] / n if n else None
        yl = yu = 0.0
        for i in range(n - 1):
            dd = abs(y.shortest(term[i]['yaw'], term[i + 1]['yaw']))
            if lat[i]:
                yl += dd
            else:
                yu += dd
        m['yaw_latched'] = yl
        m['yaw_unlatched'] = yu
        m['yaw_unlatched_frac'] = (yu / (yl + yu)) if (yl + yu) > 1e-9 else None
        # OFF episodes over the same window
        off, on = [], []
        start, cur = term[0]['t_rel'], lat[0]
        for i in range(1, n):
            if lat[i] != cur:
                (on if cur else off).append(term[i]['t_rel'] - start)
                start, cur = term[i]['t_rel'], lat[i]
        (on if cur else off).append(term[-1]['t_rel'] - start)
        m['n_off'] = len(off)
        m['off_med'] = y.med(off)
        m['off_max'] = max(off) if off else None
        m['on_med'] = y.med(on)
        m['off_frac'] = sum(1 for v in lat if not v) / n if n else None
    else:
        m['term_cycles'] = None

    # DWB turn-direction attribution over the terminal window
    n = tw = aw = osc = unl = nei = 0
    if y._has_latch(rows, tag):
        seen = set()
        for r in rows[i_out:]:
            w = r.get('dwb_best_wz')
            if w is None or abs(w) < 1e-9 or abs(r['yaw']) <= y.SIGN_CUT:
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
            if (r.get('dwb_ill_osc') or 0) >= y.FULL_WZ_BAN:
                osc += 1
            elif not y._latched(r):
                unl += 1
            else:
                nei += 1
    m['att'] = dict(n=n, toward=tw, away=aw, osc=osc, unlatched=unl,
                    neither=nei, away_frac=(aw / n) if n else None)

    # PolygonStop, from the trace itself as well as the record
    stop = sum(1 for r in rows[i_out:] if (r.get('cm_polygon') or '')
               == 'PolygonStop')
    m['polygonstop_terminal_samples'] = stop
    m['polygonstop_terminal_s'] = stop * 0.1
    return m


def _f(v, w=8, p=3, nan='-'):
    if v is None:
        return f'{nan:>{w}}'
    return f'{v:>{w}.{p}f}'


def _rows(tags):
    out = []
    for t in tags:
        try:
            out.append(metrics(t))
        except SystemExit:
            continue
    return out


def cmd_terminal():
    y.hdr('C2-NAV.23 -- the terminal window, candidate against the frozen '
          'baseline')
    print('terminal = first sample within 0.25 m of the goal -> end of leg. '
          'The frozen')
    print('baseline rows are C2-NAV.22\'s own, recomputed here by the same '
          'code path.')
    print()
    for label, tags in (('BASELINE (frozen)', BASE),
                        ('CANDIDATE (fresh, FollowPath.xy_goal_tol = 0.25)',
                         [t for t, _ in CAND if have(t)])):
        print(f'--- {label} ---')
        print(f'{"tag":<16} {"tp":>2} {"status":<10} {"leg_s":>7} '
              f'{"t@.25":>7} {"t@.05":>7} {"term_s":>7} {"term%":>6} '
              f'{"yaw_abs":>8} {"yaw_net":>8} {"ratio":>6} {"err_m":>6}')
        print('-' * 106)
        for m in _rows(tags):
            if m.get('note'):
                print(f'{m["tag"]:<16} {m["topo"]:>2} '
                      f'{str(m["status"]):<10} {_f(m["leg_s"],7,1)} '
                      f'   {m["note"]}')
                continue
            print(f'{m["tag"]:<16} {m["topo"]:>2} {str(m["status"]):<10} '
                  f'{_f(m["leg_s"],7,1)} {_f(m["t_out"],7,1)} '
                  f'{_f(m["t_in"],7,1)} {_f(m["terminal_s"],7,1)} '
                  f'{_f(m["terminal_frac"],6,3)} {_f(m["yaw_abs"],8)} '
                  f'{_f(m["yaw_signed"],8)} {_f(m["travel_ratio"],6,2)} '
                  f'{_f(m["final_err_m"],6)}')
        print()
    print('t@.25 / t@.05  first sample within 0.25 m / 0.05 m of the goal, s')
    print('term_s         terminal window duration (t@.25 -> end of leg), s')
    print('yaw_abs        absolute yaw travelled in the terminal window, rad')
    print('yaw_net        signed yaw travelled over the same window, rad')
    print('ratio          yaw_abs / |heading error on arrival|; 1.0 is the '
          'direct turn')
    print('err_m          ground-truth final_goal_err_m for the leg')


def cmd_phases():
    y.hdr('C2-NAV.23 -- the creep, which is the phase the parameter '
          'was meant to remove')
    print('C2-NAV.22 measured a third phase between transit and '
          'rotate-in-place: the')
    print('robot creeping 0.25 m -> 0.05 m at 7.4-23.9 mm/s because '
          'RotateToGoal could')
    print('not latch until it was inside 0.05 m. At a 0.25 m latch window '
          'there is no')
    print('reason for the controller to close that distance at all, so '
          'this table is')
    print('read two ways and both are stated: a SHORTER creep means the '
          'pathology is')
    print('gone, and a creep that never completes (t@.05 never reached) '
          'means the')
    print('robot stopped where the goal checker was already satisfied -- '
          'which is')
    print('correct behaviour only if `falsifier` passes.')
    print()
    for label, tags in (('BASELINE (frozen)', BASE),
                        ('CANDIDATE (fresh)',
                         [t for t, _ in CAND if have(t)])):
        print(f'--- {label} ---')
        print(f'{"tag":<16} {"tp":>2} {"transit":>8} {"CREEP":>8} '
              f'{"rotate":>8} {"leg":>8} {"creep_m":>8} {"creep_v":>8} '
              f'{"yaw_creep":>10} {"yaw_rot":>8} {"min_d":>7} '
              f'{"in_ball_m":>10}')
        print('-' * 116)
        for m in _rows(tags):
            if m.get('note'):
                print(f'{m["tag"]:<16} {m["topo"]:>2}   {m["note"]}')
                continue
            reached = '' if m['t_in'] is not None else '  (never reached .05)'
            print(f'{m["tag"]:<16} {m["topo"]:>2} {_f(m["transit_s"],8,2)} '
                  f'{_f(m["creep_s"],8,2)} {_f(m["rotate_s"],8,2)} '
                  f'{_f(m["t_end"],8,2)} {_f(m["creep_m"],8,3)} '
                  f'{_f(m["creep_v"],8,4)} {_f(m["yaw_creep"],10,3)} '
                  f'{_f(m["yaw_rot"],8,3)} {_f(m["dist_min"],7,3)} '
                  f'{_f(m["path_in_ball_m"],10,3)}{reached}')
        print()
    print('in_ball_m  path length driven INSIDE the 0.25 m goal ball, m. '
          'C2-NAV.22\'s')
    print('           worst baseline leg drove 1.116 m of it.')


def cmd_latch():
    y.hdr('C2-NAV.23 -- RotateToGoal, the mechanism under test')
    print('`dwb_ill_rot` > 0 is a cycle in which RotateToGoalCritic '
          'rejected the')
    print('translating block, i.e. its in_window_ latch was SET. Measured '
          'over the')
    print('terminal window (inside 0.25 m), which is the same window on '
          'both arms.')
    print('This is the PRIMARY READ: C2-NAV.22 predicted the unlatched '
          'fraction goes')
    print('to ~0, and fixed in advance that if it does not move the '
          'candidate is')
    print('dropped rather than retuned.')
    print()
    for label, tags in (('BASELINE (frozen)', BASE_LATCH),
                        ('CANDIDATE (fresh)',
                         [t for t, _ in CAND if have(t)])):
        print(f'--- {label} ---')
        print(f'{"tag":<16} {"tp":>2} {"cycles":>7} {"latched":>8} '
              f'{"latched%":>9} {"off%":>6} {"n_off":>6} {"off_med":>8} '
              f'{"off_max":>8} {"yaw_on":>7} {"yaw_off":>8} '
              f'{"yaw_off%":>9} {"plan_per":>9}')
        print('-' * 118)
        for m in _rows(tags):
            if m.get('term_cycles') is None:
                continue
            print(f'{m["tag"]:<16} {m["topo"]:>2} {m["term_cycles"]:>7} '
                  f'{m["latched"]:>8} {_f(m["latched_frac"],9,3)} '
                  f'{_f(m["off_frac"],6,3)} {m["n_off"]:>6} '
                  f'{_f(m["off_med"],8,2)} {_f(m["off_max"],8,2)} '
                  f'{_f(m["yaw_latched"],7,2)} {_f(m["yaw_unlatched"],8,2)} '
                  f'{_f(m["yaw_unlatched_frac"],9,3)} '
                  f'{_f(m["plan_period"],9,3)}')
        print()
    print('off%       fraction of terminal cycles with RotateToGoal '
          'INACTIVE')
    print('yaw_off%   fraction of terminal yaw travelled with it inactive')
    print('plan_per   median /plan republication period, s '
          '(RateController 0.333 Hz).')
    print('           Each republication calls setPlan, which resets the '
          'latch.')


def cmd_attribution():
    y.hdr('C2-NAV.23 -- when DWB turns AWAY from the goal heading, what '
          'state is it in?')
    print('Same arithmetic as C2-NAV.22, over the terminal window. One row '
          'per distinct')
    print(f'DWB selection, restricted to |yaw| > {y.SIGN_CUT} rad so the '
          'correct turn')
    print('direction survives the 0.492 rad ground-truth-vs-localisation '
          'offset.')
    print()
    for label, tags in (('BASELINE (frozen)', BASE_LATCH),
                        ('CANDIDATE (fresh)',
                         [t for t, _ in CAND if have(t)])):
        print(f'--- {label} ---')
        print(f'{"tag":<16} {"tp":>2} {"n":>6} {"toward":>7} {"away":>6} '
              f'{"away%":>7} {"osc ban":>8} {"unlatched":>10} '
              f'{"NEITHER":>8}')
        print('-' * 82)
        tot = dict(n=0, toward=0, away=0, osc=0, unlatched=0, neither=0)
        for m in _rows(tags):
            # A leg that never reached the 0.25 m ball has no terminal
            # window, so it has no attribution -- which is itself the
            # candidate's result and is reported by `falsifier` and
            # `paired`, not silently dropped here.
            att = m.get('att')
            if not att or not att['n']:
                continue
            for k in tot:
                tot[k] += att[k]
            print(f'{m["tag"]:<16} {m["topo"]:>2} {att["n"]:>6} '
                  f'{att["toward"]:>7} {att["away"]:>6} '
                  f'{_f(att["away_frac"],7,3)} {att["osc"]:>8} '
                  f'{att["unlatched"]:>10} {att["neither"]:>8}')
        n = tot['n'] or 1
        print('-' * 82)
        print(f'{"ALL":<16} {"":>2} {tot["n"]:>6} {tot["toward"]:>7} '
              f'{tot["away"]:>6} {tot["away"]/n:>7.3f} {tot["osc"]:>8} '
              f'{tot["unlatched"]:>10} {tot["neither"]:>8}')
        print()
    print('NEITHER is the residual: latched, no Oscillation ban, and DWB '
          'still turned')
    print('the wrong way. C2-NAV.22 measured it as 0 over 2300 baseline '
          'cycles.')


def _leg_trace(tag, leg):
    """Rows of one leg's 10 Hz trace, plus its column SCHEMA.

    Schema, not values: `dwb_ill_rot` is blank on every cycle the critic
    rejected nothing, so a value-based probe cannot tell a run predating
    the C2-NAV.21 columns from a leg where RotateToGoal never fired at
    all. C2-NAV.22 was bitten by exactly that and the fix is inherited.
    """
    import csv
    p = os.path.join(SCRATCH, f'{tag}_traces', f'{leg}_rep0.csv')
    if not os.path.exists(p):
        return None, set()
    with open(p) as f:
        rd = csv.DictReader(f)
        rows = list(rd)
        return rows, set(rd.fieldnames or ())


_B23 = {}


def _bundle23():
    if not _B23:
        if os.path.exists(BUNDLE):
            with open(BUNDLE) as f:
                _B23.update(json.load(f))
        else:
            _B23['arrival'] = {}
    return _B23


def arrival(tag):
    """Per-leg arrival geometry for a whole tour, ordinary legs included.

    This is where the candidate's evidence actually lives. All three
    fresh runs lost `enclosure_entry` before it reached the 0.25 m ball,
    so the enclosure leg carries no terminal-phase measurement on the
    candidate arm at all -- but every ordinary leg does, and the
    mechanism under test is not specific to the enclosure.

    `d_latch` is the ground-truth distance to the goal at the FIRST cycle
    RotateToGoalCritic rejected the translating block, i.e. the first
    cycle its in_window_ latch was set. That is the parameter, read off
    the robot rather than off the file: it should sit near 0.05 m on the
    baseline and near 0.25 m on the candidate.
    """
    out = []
    for r in legs(tag):
        leg = r.get('scenario')
        rows, sch = _leg_trace(tag, leg)
        if not rows or 'dwb_ill_rot' not in sch:
            continue
        g = r.get('goal_world')
        if not g:
            continue
        gx, gy = g
        pts = [(y.fl(q.get('x')), y.fl(q.get('y')),
                y.fl(q.get('dwb_ill_rot')), y.fl(q.get('yaw')))
               for q in rows]
        pts = [q for q in pts if q[0] is not None and q[3] is not None]
        if not pts:
            continue
        d = [math.hypot(q[0] - gx, q[1] - gy) for q in pts]
        lat = [(q[2] or 0) > 0 for q in pts]
        i_first = next((i for i in range(len(lat)) if lat[i]), None)
        yaw_after = None
        if i_first is not None:
            yaw_after = sum(
                abs(y.shortest(pts[i][3], pts[i + 1][3]))
                for i in range(i_first, len(pts) - 1))
        out.append(dict(
            tag=tag, leg=leg, status=r.get('status'),
            secs=r.get('duration_sim_s'),
            final_err_m=r.get('final_goal_err_m'),
            d_final=d[-1], d_min=min(d),
            d_latch=(d[i_first] if i_first is not None else None),
            latched_frac=sum(lat) / len(lat),
            cycles=len(lat), yaw_after_latch=yaw_after))
    if not out:
        # The 10 Hz traces live in the scratch tree, which is not part of
        # the repository. With them gone, fall back to the frozen bundle
        # so every table here regenerates from docs/data/ alone.
        return _bundle23().get('arrival', {}).get(tag, [])
    return out


def cmd_arrival():
    y.hdr('C2-NAV.23 -- where RotateToGoal latches, and where the robot '
          'stops')
    print('d_latch is the ground-truth distance to the goal at the first '
          'cycle the')
    print('critic rejected the translating block. It IS the parameter, '
          'measured off')
    print('the robot: ~0.05 m expected on the baseline, ~0.25 m on the '
          'candidate.')
    print('Once latched, every translating trajectory is illegal, so the '
          'robot cannot')
    print('close the remaining distance by driving -- it can only rotate.')
    print()
    for label, tags in (
            ('BASELINE (frozen)',
             ['c2n21_base_r1', 'c2n21_base_r3', 'c2n21_base_r4',
              'c2n21_bbase_r2', 'c2n21_bbase_r3']),
            ('CANDIDATE (fresh)', [t for t, _ in CAND if have(t)])):
        print(f'--- {label} ---')
        print(f'{"tag":<18}{"leg":<17}{"status":<10}{"secs":>7}'
              f'{"d_latch":>9}{"d_final":>9}{"err_m":>7}{"lat%":>7}'
              f'{"yaw_after":>10}')
        print('-' * 94)
        dl, df = [], []
        for t in tags:
            for a in arrival(t):
                if a['d_latch'] is not None:
                    dl.append(a['d_latch'])
                if a['status'] == 'SUCCEEDED' and a['final_err_m'] is not None:
                    df.append(a['final_err_m'])
                print(f'{a["tag"]:<18}{a["leg"]:<17}{str(a["status"]):<10}'
                      f'{_f(a["secs"],7,2)}'
                      f'{(f"{a["d_latch"]:.3f}" if a["d_latch"] is not None else "never"):>9}'
                      f'{_f(a["d_final"],9,3)}{_f(a["final_err_m"],7,3)}'
                      f'{_f(a["latched_frac"],7,3)}'
                      f'{_f(a["yaw_after_latch"],10,2)}')
        print()
        if dl:
            print(f'   d_latch over {len(dl)} legs: min {min(dl):.3f}  '
                  f'median {y.med(dl):.3f}  max {max(dl):.3f} m')
        if df:
            over = [v for v in df if v > FALSIFIER_M]
            print(f'   SUCCEEDED final error over {len(df)} legs: '
                  f'median {y.med(df):.3f}  max {max(df):.3f} m   '
                  f'over {FALSIFIER_M} m: {len(over)}')
        print()


BASE_TOURS = ['c2n21_base_r1', 'c2n21_base_r3', 'c2n21_base_r4',
              'c2n21_bbase_r2', 'c2n21_bbase_r3']


def cmd_paired():
    """Same scenario, same goal, both arms.

    `open_space` is the decisive row and the reason this table exists.
    It is the FIRST leg of the tour, so the controller cannot be
    carrying a latch from a previous leg into it: whatever
    RotateToGoalCritic does there, it does because of this parameter and
    nothing else. Every later leg begins with whatever state the
    previous one left behind, which is a real effect but a confounded
    measurement, and is reported as such.
    """
    y.hdr('C2-NAV.23 -- the same leg, both arms')
    print('d_latch  ground-truth distance to the goal when RotateToGoal '
          'first banned')
    print('         translation. err_m  ground-truth distance where the '
          'leg ended.')
    print('If the mechanism is what C2-NAV.22 says, these two track each '
          'other AND')
    print('track the parameter: the robot stops where the critic latches.')
    print()
    order = ['open_space', 'wall_adjacent', 'wall_parallel',
             'obstacle_corner', 'corridor_gate', 'enclosure_entry',
             'enclosure_exit']
    ab = {}
    for arm, tags in (('base', BASE_TOURS),
                      ('cand', [t for t, _ in CAND if have(t)])):
        for t in tags:
            for a in arrival(t):
                ab.setdefault((a['leg'], arm), []).append(a)
    print(f'{"leg":<17}{"arm":<6}{"n":>3}{"d_latch med":>12}'
          f'{"d_latch range":>22}{"err med":>9}{"err max":>9}'
          f'{"secs med":>10}{"over .25":>9}')
    print('-' * 98)
    for leg in order:
        for arm, name in (('base', 'BASE'), ('cand', 'CAND')):
            rs = ab.get((leg, arm), [])
            if not rs:
                continue
            dl = [r['d_latch'] for r in rs if r['d_latch'] is not None]
            er = [r['final_err_m'] for r in rs
                  if r['status'] == 'SUCCEEDED' and r['final_err_m'] is not None]
            sc = [r['secs'] for r in rs if r['secs'] is not None]
            rng = (f'{min(dl):.3f} - {max(dl):.3f}' if dl else 'never')
            print(f'{leg:<17}{name:<6}{len(rs):>3}'
                  f'{(f"{y.med(dl):.3f}" if dl else "-"):>12}{rng:>22}'
                  f'{(f"{y.med(er):.3f}" if er else "-"):>9}'
                  f'{(f"{max(er):.3f}" if er else "-"):>9}'
                  f'{(f"{y.med(sc):.2f}" if sc else "-"):>10}'
                  f'{sum(1 for v in er if v > FALSIFIER_M):>9}')
        print()
    print('open_space is the unconfounded row: first leg of the tour, no')
    print('previous-leg latch to inherit. Later legs begin with whatever '
          'state the')
    print('previous leg left, which is why their d_latch runs to metres '
          'on the')
    print('candidate arm -- a real consequence, but not a clean reading '
          'of the')
    print('parameter, and it is not claimed as one.')


def cmd_safety():
    y.hdr('C2-NAV.23 -- the regression check outside the terminal-yaw '
          'objective')
    print('PolygonStop is the collision monitor gating the wheels to zero. '
          'A candidate')
    print('that traded terminal yaw for a stop-zone deadlock is rejected. '
          'worst_crawl')
    print('is nav_bench\'s longest run of near-zero forward velocity on '
          'the leg.')
    print()
    for label, tags in (('BASELINE (frozen)', BASE),
                        ('CANDIDATE (fresh)',
                         [t for t, _ in CAND if have(t)])):
        print(f'--- {label} ---')
        print(f'{"tag":<16} {"tp":>2} {"PolyStop_s":>11} '
              f'{"PolySlow_s":>11} {"stop_term_s":>12} {"crawl_s":>8} '
              f'{"crawl_d_m":>10} {"progress_fail":>14}')
        print('-' * 92)
        for m in _rows(tags):
            ps = (m.get('cm_polygon_secs') or {})
            wc = m.get('worst_crawl') or {}
            print(f'{m["tag"]:<16} {m["topo"]:>2} '
                  f'{_f(ps.get("PolygonStop"),11,2,"0.00")} '
                  f'{_f(ps.get("PolygonSlow"),11,2,"0.00")} '
                  f'{_f(m.get("polygonstop_terminal_s"),12,2)} '
                  f'{_f(wc.get("crawl_len_s"),8,2)} '
                  f'{_f(wc.get("dist_to_goal_m"),10,3)} '
                  f'{str(m.get("n_progress_failures")):>14}')
        print()
    print('stop_term_s  PolygonStop time inside the terminal window, from '
          'the 10 Hz trace')
    print('crawl_d_m    distance to the goal at the worst crawl')


def _agg(tags, key, sub=None):
    vs = []
    for m in _rows(tags):
        v = m.get(key)
        if sub is not None and isinstance(v, dict):
            v = v.get(sub)
        if v is not None:
            vs.append(v)
    return vs


def cmd_verdict():
    y.hdr('C2-NAV.23 verdict')
    cand = [t for t, _ in CAND if have(t)]
    if not cand:
        print('no fresh candidate runs present')
        return
    base = BASE_LATCH
    rows = [('terminal window s', 'terminal_s', None, 1),
            ('terminal fraction of leg', 'terminal_frac', None, 3),
            ('absolute yaw travel rad', 'yaw_abs', None, 2),
            ('yaw travel / needed', 'travel_ratio', None, 2),
            ('creep 0.25->0.05 s', 'creep_s', None, 1),
            ('path driven in goal ball m', 'path_in_ball_m', None, 3),
            ('RotateToGoal OFF fraction', 'off_frac', None, 3),
            ('yaw travelled unlatched frac', 'yaw_unlatched_frac', None, 3),
            ('DWB away-from-goal fraction', 'att', 'away_frac', 3),
            ('w_act sign flips', 'flips_w_act', None, 1),
            ('heading-error zero crossings', 'zero_cross', None, 1),
            ('final ground-truth err m', 'final_err_m', None, 3)]
    print(f'{"quantity":<32} {"baseline (n)":>16} {"candidate (n)":>16} '
          f'{"direction":>12}')
    print('-' * 80)
    for label, key, sub, p in rows:
        b = _agg(base, key, sub)
        c = _agg(cand, key, sub)
        if not b or not c:
            continue
        mb, mc = y.med(b), y.med(c)
        arrow = 'lower' if mc < mb else ('higher' if mc > mb else 'same')
        print(f'{label:<32} {mb:>11.{p}f} ({len(b):>2}) '
              f'{mc:>11.{p}f} ({len(c):>2}) {arrow:>12}')
    print()
    print('Medians. n is the number of legs the quantity is defined on. '
          'The baseline')
    print('column is the frozen C2-NAV.21 set carrying the per-cycle '
          'critic columns.')
    print()
    print('worst case, which is what the pathology is about:')
    for label, key, sub, p in (('terminal window s', 'terminal_s', None, 1),
                               ('absolute yaw travel rad', 'yaw_abs',
                                None, 2),
                               ('final ground-truth err m', 'final_err_m',
                                None, 3)):
        b = _agg(base, key, sub)
        c = _agg(cand, key, sub)
        if b and c:
            print(f'  {label:<30} baseline max {max(b):>8.{p}f}   '
                  f'candidate max {max(c):>8.{p}f}')


def cmd_dump(path):
    out = dict(
        experiment='C2-NAV.23',
        variable=THE_LEAF, frm=FROM_VAL, to=TO_VAL,
        baseline_params_sha256=_sha(BASELINE_PARAMS),
        candidate_params_sha256=_sha(CAND_PARAMS),
        parsed_leaf_diff=[dict(leaf=k, frm=a, to=b) for k, a, b
                          in param_diff()],
        live_readback={t: live_fpxy(t) for t, _ in CAND if have(t)},
        candidate={}, baseline={}, legs={}, arrival={})
    for t, _ in CAND:
        if have(t):
            out['candidate'][t] = metrics(t)
            out['legs'][t] = legs(t)
            out['arrival'][t] = arrival(t)
    for t in (BASE + BASE_TOURS + ['c2n21_base_r2', 'c2n21_bbase_r1']):
        if t not in out['legs']:
            ls = legs(t)
            if ls:
                out['legs'][t] = ls
        try:
            out['baseline'][t] = metrics(t)
        except SystemExit:
            continue
        if t not in out['arrival']:
            a = arrival(t)
            if a:
                out['arrival'][t] = a
    with open(path, 'w') as f:
        json.dump(out, f, indent=1, sort_keys=True, default=str)
    print(f'wrote {path}  ({os.path.getsize(path)} bytes)')


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 0
    cmd, rest = argv[1], argv[2:]
    if cmd == 'gate':
        return 0 if cmd_gate() else 1
    if cmd == 'falsifier':
        cmd_falsifier()
    elif cmd == 'terminal':
        cmd_terminal()
    elif cmd == 'phases':
        cmd_phases()
    elif cmd == 'latch':
        cmd_latch()
    elif cmd == 'attribution':
        cmd_attribution()
    elif cmd == 'arrival':
        cmd_arrival()
    elif cmd == 'paired':
        cmd_paired()
    elif cmd == 'safety':
        cmd_safety()
    elif cmd == 'verdict':
        cmd_verdict()
    elif cmd == 'dump':
        cmd_dump(rest[0] if rest else BUNDLE)
    elif cmd == 'all':
        if not cmd_gate():
            print()
            print('gate failed -- refusing to report results')
            return 1
        cmd_falsifier()
        cmd_paired()
        cmd_arrival()
        cmd_terminal()
        cmd_phases()
        cmd_latch()
        cmd_attribution()
        cmd_safety()
        cmd_verdict()
    else:
        print(__doc__)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
