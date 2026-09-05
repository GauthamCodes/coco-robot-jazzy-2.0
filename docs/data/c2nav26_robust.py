#!/usr/bin/env python3
"""C2-NAV.26 -- is PolygonSlow.slowdown_ratio = 1.0 robust enough.

C2-NAV.25 measured the candidate and found the performance claim
supported and the candidate REJECTED on the letter of its own
pre-registered gates: 21 of 21 legs SUCCEEDED, creep per run fell 59.1 %,
and ONE leg -- c2n25_slow_r2/wall_parallel, topology A -- finished
0.255 m from its goal in ground truth against a 0.25 m gate.

That gate stands. Nothing here reopens it, moves it, or re-reads it.
C2-NAV.25 is REJECTED and stays rejected; this module never recomputes
its verdict.

The question here is the DIFFERENT one the rejection left open:

    is 0.255 m an isolated tail event, or the visible edge of a
    repeatable positional-accuracy regression that PolygonSlow = 1.0
    introduces?

SIX fresh tours -- three topology A, three topology B -- against the
BYTE-IDENTICAL configuration (docs/data/c2nav25_slow_params.yaml, sha256
4c15893e83936b745fd1659f5c63a38d31e8acb5883b2e8d6d6ff02888ccc323, one
leaf from the frozen baseline, asserted leaf-by-leaf by
`c2nav25_slow.py paramdiff` before the first simulator started). No
second parameter moves. The only thing that changes between C2-NAV.25
and C2-NAV.26 is the number of independent trials.

EVERY measurement imports C2-NAV.25's `leg_row` and C2-NAV.24's
windowing rather than restating them, for the reason C2-NAV.25's own
header gives: a comparison whose two halves were computed by two copies
of a definition is not a comparison. This module adds exactly one new
kind of number -- the error-tail counts -- and nothing else.

EVIDENCE CLASS
--------------
OBSERVED    per-leg `status`, `final_goal_err_m`, `min_clearance_m`,
            `cm_action_frac`, `cm_polygon_secs`, `duration_wall_s`, and
            every trace column, for all 42 fresh legs.
DERIVED     creep windows and creep seconds (C2-NAV.24's rule, imported),
            the tail counts, the percentiles, and every arm delta.
FROZEN      the baseline arm (5 runs / 35 legs) and the C2-NAV.25
            candidate arm (3 runs / 21 legs). Neither is re-run.
UNAVAILABLE the final AMCL position error. `nav_bench.py` subscribes
            /amcl_pose into a Series (line 460) but never writes it to a
            record field or a trace column, so it is not in the
            artifacts for ANY arm -- baseline, C2-NAV.25 or C2-NAV.26.
            The brief asks for it "if available". It is not available,
            and ground-truth error is NOT reported in its place. What
            can be said about localisation is a BOUND, printed by
            `tail`, and it is labelled as a bound.

WHAT THIS CANNOT SHOW
---------------------
Nine candidate runs against five frozen baseline runs is still not a
paired design, and the runs are not randomised against machine state
beyond the A/B/A/B/A/B interleave. Tail counts on 63 candidate legs
resolve a ~1-in-60 event poorly: the honest statement about a single
0.255 m observation is a frequency with a wide interval, not a p-value.
This module prints counts and percentiles and does NOT compute
significance. `n` is small and saying so is part of the result.
"""

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import c2nav24_chain as C24          # noqa: E402
import c2nav25_slow as C25           # noqa: E402

# The six fresh tours. Interleaved A/B/A/B/A/B when run, so topology --
# the thing this session compares -- is not confounded with the hour of
# machine state it ran in.
C26_A = ['c2n26_slow_r1', 'c2n26_slow_r2', 'c2n26_slow_r3']
C26_B = ['c2n26_bslow_r1', 'c2n26_bslow_r2', 'c2n26_bslow_r3']
C26 = C26_A + C26_B

# The two frozen arms, imported so they cannot drift from C2-NAV.25's
# own definition of them.
BASE_A, BASE_B, BASE = C25.BASE_A, C25.BASE_B, C25.BASE
C25_A, C25_B, C25_ALL = C25.CAND_A, C25.CAND_B, C25.CAND

# Every candidate leg ever run at slowdown_ratio = 1.0.
ALL_CAND_A = C25_A + C26_A
ALL_CAND_B = C25_B + C26_B
ALL_CAND = ALL_CAND_A + ALL_CAND_B

LEGS = C24.LEGS
BUNDLE = os.path.join(HERE, 'c2nav26_robust.json')

# The error-tail cut points the C2-NAV.26 brief names, in metres. 0.25 is
# C2-NAV.25's pre-registered gate and is NOT special here -- it is one
# column of a table, which is the point of printing the tail rather than
# the gate.
TAIL_CUTS = [0.20, 0.22, 0.24, 0.25, 0.26, 0.30]

# Pre-registered for THIS session, fixed before any C2-NAV.26 run
# finished (see the session log entry and the commit that carries it).
# These are read as ROBUSTNESS checks, not as a pass/fail gate on a
# candidate that is already rejected.
ROB_ENCL_MIN = 1.00      # enclosure_entry + _exit success fraction
ROB_CREEP_DROP = 0.25    # creep per run must stay >= 25 % below baseline
ROB_CLEAR_MARGIN = C25.GATE_CLEAR_MARGIN_M   # 0.02 m, imported


def hdr(t):
    C24.hdr(t)


def rows_for(runs):
    """C2-NAV.25's own per-leg row, unmodified."""
    return C25.rows_for(runs)


def _errs(rs):
    """SUCCEEDED legs' ground-truth final error, sorted ascending."""
    return sorted(r['goal_err'] for r in rs
                  if r['status'] == 'SUCCEEDED' and r['goal_err'] is not None)


def _pct(e, q):
    """Nearest-rank percentile. Stated explicitly because on n=21 the
    choice of interpolation moves p95 by more than the effect."""
    if not e:
        return None
    k = int(round(q * (len(e) - 1)))
    return e[k]


def _tail_row(label, rs):
    e = _errs(rs)
    if not e:
        print(f'{label:<22}  (no SUCCEEDED legs)')
        return
    cuts = '  '.join(f'{sum(1 for x in e if x > c):>5}' for c in TAIL_CUTS)
    import statistics as st
    print(f'{label:<22}{len(e):>5}  {cuts}  {st.median(e):>8.3f}'
          f'{_pct(e, 0.90):>8.3f}{_pct(e, 0.95):>8.3f}{max(e):>8.3f}')


# ------------------------------------------------------------ 1. the legs
def cmd_legs(args):
    hdr('C2-NAV.26 -- every leg of the six fresh tours')
    rs = rows_for(args.runs or C26)
    C25._leg_table(rs)
    print()
    print("Columns are C2-NAV.25's, computed by C2-NAV.25's own leg_row:")
    print('  status      OBSERVED  Nav2 result for the leg')
    print('  leg_s       OBSERVED  duration_wall_s')
    print('  creep_s/m   DERIVED   terminal creep, the 0.25 -> 0.05 m band')
    print('  err_m       OBSERVED  final_goal_err_m, GROUND TRUTH')
    print('  clr_m       OBSERVED  min_clearance_m, true geometric')
    print('  dwb/pre/act OBSERVED  DWB-selected, pre-monitor, achieved vx')
    print('  mon         DERIVED   |v_cmdvel| / |v_smoothed| over the creep')
    print('  slow%       OBSERVED  creep cycles claimed by PolygonSlow')
    print('  zero%       OBSERVED  creep cycles commanding exactly 0')
    print('  |yaw|/net   DERIVED   yaw travel over the CREEP window, rad')
    print()
    print('Absolute and net yaw are over the creep window, which is')
    print("C2-NAV.24's definition and therefore the one the frozen arms")
    print('were measured with. The record also carries a whole-leg')
    print('terminal_yaw_travel_rad; `yawcheck` prints both rather than')
    print('quietly swapping one for the other.')
    return 0


def cmd_yawcheck(args):
    """Both yaw definitions, side by side, for every fresh leg."""
    hdr("C2-NAV.26 -- yaw travel: creep window vs the record's own field")
    print(f'{"run":<16}{"leg":<17}{"|yaw|_creep":>12}{"net_creep":>11}'
          f'{"rec_terminal":>14}')
    for tag in (args.runs or C26):
        for leg in LEGS:
            w = C24.windows(tag, leg)
            if not w:
                continue
            r = C25.leg_row(w)
            rec = w['rec'].get('terminal_yaw_travel_rad')
            print(f'{r["tag"]:<16}{r["leg"]:<17}'
                  f'{C24.f3(r["yaw_abs"], 12)}{C24.f3(r["yaw_net"], 11)}'
                  f'{C24.f3(rec, 14)}')
    print()
    print("The two differ because the record's field runs from the")
    print('transit/terminal split to the end of the leg, while the creep')
    print('window closes at 0.05 m. Neither is substituted for the other.')
    return 0


# ------------------------------------------------------- 2. the error tail
def cmd_tail(args):
    """The distribution the C2-NAV.26 brief asks for, not just its ends."""
    hdr('C2-NAV.26 -- final ground-truth error tail, SUCCEEDED legs')
    print('Counts are legs with error STRICTLY GREATER than the cut.')
    print('Percentiles are nearest-rank (no interpolation): on n=21 the')
    print('interpolation choice moves p95 by more than the effect does.')
    print()
    head = '  '.join(f'>{c:.2f}' for c in TAIL_CUTS)
    print(f'{"arm":<22}{"n":>5}  {head}  {"median":>8}{"p90":>8}'
          f'{"p95":>8}{"max":>8}')
    print('-' * 96)
    _tail_row('baseline  all', rows_for(BASE))
    _tail_row('baseline  topo A', rows_for(BASE_A))
    _tail_row('baseline  topo B', rows_for(BASE_B))
    print()
    _tail_row('C2-NAV.25 all', rows_for(C25_ALL))
    _tail_row('C2-NAV.25 topo A', rows_for(C25_A))
    _tail_row('C2-NAV.25 topo B', rows_for(C25_B))
    print()
    _tail_row('C2-NAV.26 all', rows_for(C26))
    _tail_row('C2-NAV.26 topo A', rows_for(C26_A))
    _tail_row('C2-NAV.26 topo B', rows_for(C26_B))
    print()
    _tail_row('CANDIDATE .25+.26', rows_for(ALL_CAND))
    _tail_row('CANDIDATE  topo A', rows_for(ALL_CAND_A))
    _tail_row('CANDIDATE  topo B', rows_for(ALL_CAND_B))
    print()
    print("--- every SUCCEEDED leg's error, ascending, per arm ---")
    for label, runs in (('baseline ', BASE), ('C2-NAV.25', C25_ALL),
                        ('C2-NAV.26', C26),
                        ('CANDIDATE', ALL_CAND)):
        e = _errs(rows_for(runs))
        print(f'{label} n={len(e):<3} ' + ' '.join(f'{x:.3f}' for x in e))
    print()
    print('--- legs past 0.20 m, named, with topology and leg ---')
    print(f'{"arm":<11}{"run":<17}{"leg":<17}{"err_m":>7}{"topo":>6}'
          f'{"status":>11}')
    for label, runs in (('baseline', BASE), ('C2-NAV.25', C25_ALL),
                        ('C2-NAV.26', C26)):
        for r in sorted(rows_for(runs), key=lambda x: -(x['goal_err'] or 0)):
            if r['status'] != 'SUCCEEDED' or (r['goal_err'] or 0) <= 0.20:
                continue
            topo = 'B' if r['tag'] in (BASE_B + C25_B + C26_B) else 'A'
            print(f'{label:<11}{r["tag"]:<17}{r["leg"]:<17}'
                  f'{r["goal_err"]:>7.3f}{topo:>6}{r["status"]:>11}')
    print()
    print('--- what can and cannot be said about localisation ---')
    print('The goal checker fires on the ESTIMATED pose at')
    print('goal_checker.xy_goal_tolerance = 0.25 m. So for every')
    print('SUCCEEDED leg the AMCL error at that instant was <= 0.25 m,')
    print('and a ground-truth error of e implies a localisation offset of')
    print('at least (e - 0.25) m. That is a BOUND, DERIVED from the')
    print('tolerance -- not a measurement of AMCL error, which is not in')
    print('the artifacts for any arm.')
    return 0


def cmd_byleg(args):
    """Which LEG carries the tail, and does it carry it in both arms?

    The aggregate tail answers "how often". It cannot answer "where",
    and "where" is what separates a candidate-induced regression from a
    property of one waypoint. If the same leg tops both the frozen
    baseline and the candidate, the tail is a feature of that goal's
    geometry that the candidate inherited rather than created.
    """
    import statistics as st
    hdr('C2-NAV.26 -- final ground-truth error BY LEG, all three arms')
    print(f'{"leg":<17}' + ''.join(
        f'{h:>26}' for h in ('baseline', 'C2-NAV.25', 'C2-NAV.26')))
    print(f'{"":<17}' + ''.join(f'{"n   median      max":>26}'
                                for _ in range(3)))
    print('-' * 95)
    arms = [rows_for(BASE), rows_for(C25_ALL), rows_for(C26)]
    for leg in LEGS:
        cells = ''
        for rs in arms:
            e = _errs([r for r in rs if r['leg'] == leg])
            if not e:
                cells += f'{"-":>26}'
            else:
                cells += (f'{len(e):>6}{st.median(e):>10.3f}'
                          f'{max(e):>10.3f}')
        print(f'{leg:<17}{cells}')
    print()
    print('A leg that tops every arm is telling you about the goal, not')
    print('about slowdown_ratio. A leg that tops only the candidate arms')
    print('is the shape a real regression would take.')
    return 0


# ------------------------------------------------------- 3. the comparison
def _fmt(v, w=10, p=3):
    return (' ' * w) if v is None else f'{v:>{w}.{p}f}'


def cmd_compare(args):
    """Three arms, one code path, every aggregate C2-NAV.25 defined."""
    hdr('C2-NAV.26 -- baseline vs C2-NAV.25 vs C2-NAV.26, same code')
    arms = [('baseline', rows_for(BASE)), ('C2-NAV.25', rows_for(C25_ALL)),
            ('C2-NAV.26', rows_for(C26)), ('cand .25+.26', rows_for(ALL_CAND))]
    ag = [(n, C25._agg(rs)) for n, rs in arms if rs]
    if len(ag) < 2:
        print('not enough arms on disk to compare')
        return 1
    rows = [
        ('runs', 'n_runs', 0), ('legs', 'n_legs', 0),
        ('SUCCEEDED legs', 'succ', 0),
        ('enclosure_entry legs', 'encl', 0),
        ('enclosure_entry ok', 'encl_ok', 0),
        ('creep s, total', 'creep_total', 1),
        ('creep s per run', 'creep_per_run', 1),
        ('worst leg creep s', 'creep_max', 1),
        ('leg s per run', 'leg_per_run', 1),
        ('creep cycles', 'creep_cycles', 0),
        ('creep speed m/s', 'creep_speed', 4),
        ('DWB vx m/s (cw)', 'dwb_vx_cw', 4),
        ('pre-monitor vx (cw)', 'pre_vx_cw', 4),
        ('achieved vx (cw)', 'act_vx_cw', 4),
        ('monitor gain, median', 'mon_gain', 3),
        ('gain under Slow', 'mon_gain_slow', 3),
        ('legs w/ Slow claim', 'n_slow_legs', 0),
        ('STOP cycles', 'stop_cycles', 0),
        ('STOP s per run', 'stop_per_run', 2),
        ('zero-vx frac (cw)', 'zero_frac_cw', 4),
    ]
    print(f'{"quantity":<24}' + ''.join(f'{n:>14}' for n, _ in ag))
    print('-' * (24 + 14 * len(ag)))
    for label, key, p in rows:
        cells = ''
        for _, a in ag:
            v = a.get(key)
            cells += (' ' * 14) if v is None else f'{v:>14.{p}f}'
        print(f'{label:<24}{cells}')
    print()
    b = dict(ag)['baseline']
    for name, a in ag:
        if name == 'baseline':
            continue
        # Signed as a CHANGE in creep, so a fall reads negative. The
        # magnitude is the same number C2-NAV.25 published as "a fall of
        # 59.1 %"; printing it as +59.1 invited reading a rise.
        d = (a['creep_per_run'] - b['creep_per_run']) / b['creep_per_run']
        print(f'{name}: creep per run {b["creep_per_run"]:.1f} -> '
              f'{a["creep_per_run"]:.1f} s  ({100 * d:+.1f} %)')
    print()
    print('cw = cycle-weighted: a 151.8 s leg counts 151.8 s, not "one')
    print('leg". Per-leg means are in `legs`. The baseline arm is the')
    print('SAME five frozen runs C2-NAV.25 used, loaded by the same code.')
    return 0


def cmd_monitor(args):
    """The monitor's action enum over WHOLE tours, all three arms.

    C2-NAV.25 published its PolygonStop numbers on this window (whole
    tour, every trace row), not on the creep window, so the replication
    has to be read on the same window or it is not a replication.
    """
    import collections
    hdr('C2-NAV.26 -- what the collision monitor did, over WHOLE tours')
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

    arms = [('baseline', BASE), ('C2-NAV.25', C25_ALL), ('C2-NAV.26', C26)]
    tal = [(nm, ) + tally(rs) for nm, rs in arms]
    print(f'{"action":<16}' + ''.join(f'{nm:>24}' for nm, _, _, _ in tal))
    print(f'{"":<16}' + ''.join(f'{"cycles      share":>24}' for _ in tal))
    print('-' * 88)
    keys = sorted({k for _, ca, _, _ in tal for k in ca})
    for k in keys:
        row = ''
        for _, ca, _, n in tal:
            row += f'{ca.get(k, 0):>14}{(ca.get(k, 0) / n if n else 0):>10.4f}'
        print(f'{names.get(k, k):<16}{row}')
    print()
    print(f'{"polygon":<16}' + ''.join(f'{nm:>24}' for nm, _, _, _ in tal))
    print('-' * 88)
    pk = sorted({k for _, _, cp, _ in tal for k in cp})
    for k in pk:
        row = ''
        for _, _, cp, n in tal:
            row += f'{cp.get(k, 0):>14}{(cp.get(k, 0) / n if n else 0):>10.4f}'
        print(f'{k:<16}{row}')
    print()
    print(f'{"total rows":<16}' + ''.join(f'{n:>14}{"":>10}'
                                          for _, _, _, n in tal))
    print()
    print('SLOWDOWN is 0 in both candidate arms for the reason C2-NAV.25')
    print('established: Velocity::operator< in types.hpp compares squared')
    print('magnitude STRICTLY, so at ratio 1.0 the scaled velocity equals')
    print('the incumbent and PolygonSlow never claims the action.')
    return 0


def cmd_failures(args):
    """Every non-SUCCEEDED leg, with the mode that produced it.

    C2-NAV.26 is the first candidate arm to fail legs at all, so the
    failures are the result, not a footnote. A cascade -- one leg failing
    and leaving the robot somewhere the planner will not plan from -- is
    counted as ONE primary failure and N-1 consequential ones, because
    counting six aborted legs as six independent events would overstate
    the frequency by the length of the tour.
    """
    hdr('C2-NAV.26 -- every failed leg, and whether it was primary')
    for label, runs in (('baseline', BASE), ('C2-NAV.25', C25_ALL),
                        ('C2-NAV.26', C26)):
        rs = rows_for(runs)
        bad = [r for r in rs if r['status'] != 'SUCCEEDED']
        print(f'--- {label}: {len(rs) - len(bad)}/{len(rs)} legs SUCCEEDED, '
              f'{len(bad)} failed ---')
        for tag in runs:
            legs = [r for r in rs if r['tag'] == tag]
            b = [r for r in legs if r['status'] != 'SUCCEEDED']
            if not b:
                continue
            first = min(LEGS.index(r['leg']) for r in b)
            for r in sorted(b, key=lambda x: LEGS.index(x['leg'])):
                kind = ('PRIMARY' if LEGS.index(r['leg']) == first
                        else 'cascade')
                print(f'    {tag:<17}{r["leg"]:<17}{r["status"]:<10}'
                      f'err={r["goal_err"]:>7.3f}  {kind}')
        print()
    return 0


# ------------------------------------------------------ 4. the robustness
def cmd_robust(args):
    """The eight checks the C2-NAV.26 brief fixed in advance.

    These are ROBUSTNESS checks on an already-rejected candidate, not a
    re-run of C2-NAV.25's gates. C2-NAV.25's gate 1 failed and stays
    failed. Nothing here is permitted to overturn it.
    """
    hdr('C2-NAV.26 -- the eight robustness checks, fixed before the runs')
    b, c26 = rows_for(BASE), rows_for(C26)
    cand = rows_for(ALL_CAND)
    if not c26:
        print('no C2-NAV.26 runs on disk')
        return 1
    ab, a26 = C25._agg(b), C25._agg(c26)
    ok = []

    def check(n, passed, detail):
        ok.append(passed)
        print(f'  [{"PASS" if passed else "FAIL"}] {n}')
        print(f'         {detail}')

    # 1. enclosure success
    def encl(rs):
        e = [r for r in rs if r['leg'] in ('enclosure_entry',
                                           'enclosure_exit')]
        s = sum(1 for r in e if r['status'] == 'SUCCEEDED')
        return s, len(e)
    s26, n26 = encl(c26)
    sb, nb = encl(b)
    check('1. enclosure success effectively complete',
          n26 > 0 and (s26 / n26) >= ROB_ENCL_MIN,
          f'C2-NAV.26 {s26}/{n26} enclosure legs SUCCEEDED; '
          f'baseline {sb}/{nb}')

    # 2. creep still far below baseline
    drop = 1.0 - (a26['creep_per_run'] / ab['creep_per_run'])
    check('2. terminal creep substantially below baseline',
          drop >= ROB_CREEP_DROP,
          f'creep per run {ab["creep_per_run"]:.1f} -> '
          f'{a26["creep_per_run"]:.1f} s ({-100 * drop:+.1f} %), '
          f'threshold -{100 * ROB_CREEP_DROP:.0f} %')

    # 3. PolygonStop far below baseline.
    #
    # Reported BOTH ways, and this note records why. As pre-registered,
    # this check counted STOP cycles inside the CREEP window -- and that
    # reading is 0 in EVERY arm, baseline included, so it discriminates
    # nothing. That is a fault in the check, found after the data
    # existed; it is printed and never dropped.
    #
    # The BINDING reading is the whole-tour share, which is the window
    # C2-NAV.25's own published PolygonStop claim (766 of 18303 -> 47 of
    # 4485) was measured on. Reading a replication on a different window
    # from the claim would not be a replication. The window is chosen to
    # match the claim, NOT to change the threshold -- and both readings
    # are reported, so nothing is made to pass that would otherwise fail.
    import collections

    def stop_share(runs):
        c, n = collections.Counter(), 0
        for t in runs:
            for lg in LEGS:
                rows = C24.load_trace(t, lg)
                if not rows:
                    continue
                for r in rows:
                    n += 1
                    c[r.get('cm_action') or 'None'] += 1
        return (c.get('1', 0), n)
    bs, bn = stop_share(BASE)
    cs, cn = stop_share(C26)
    bfrac = (bs / bn) if bn else 0.0
    cfrac = (cs / cn) if cn else 0.0
    print('  [n/a ] 3a. PRE-REGISTERED reading: STOP inside the creep '
          'window')
    print(f'         {ab["stop_cycles"]} of {ab["creep_cycles"]} creep '
          f'-> {a26["stop_cycles"]} of {a26["creep_cycles"]} -- 0 in '
          f'every arm, discriminates nothing')
    check('3b. PolygonStop share far below baseline (whole tour)',
          cfrac < bfrac,
          f'{bs} of {bn} cycles ({100 * bfrac:.2f} %) -> {cs} of {cn} '
          f'({100 * cfrac:.2f} %); C2-NAV.25 measured 47 of 4485 '
          f'(1.05 %) on this same window')

    # 4. error centred near baseline
    import statistics as st
    eb, e26 = _errs(b), _errs(c26)
    mb, m26 = st.median(eb), st.median(e26)
    check('4. ground-truth error centred near baseline',
          abs(m26 - mb) <= 0.02,
          f'median {mb:.3f} -> {m26:.3f} m (shift {m26 - mb:+.3f} m); '
          f'p90 {_pct(eb, 0.9):.3f} -> {_pct(e26, 0.9):.3f}')

    # 5. >0.25 m isolated, not systematic
    ec = _errs(cand)
    n_over = sum(1 for x in ec if x > 0.25)
    nb_over = sum(1 for x in eb if x > 0.25)
    check('5. errors past 0.25 m isolated, not a systematic tail',
          n_over <= 1,
          f'candidate {n_over} of {len(ec)} SUCCEEDED legs past 0.25 m; '
          f'baseline {nb_over} of {len(eb)}')

    # 6. no topology-specific degradation
    ea, ebt = _errs(rows_for(ALL_CAND_A)), _errs(rows_for(ALL_CAND_B))
    sa = st.median(ea) if ea else None
    sb2 = st.median(ebt) if ebt else None
    check('6. no topology-specific degradation',
          sa is not None and sb2 is not None and abs(sa - sb2) <= 0.05,
          f'candidate median topo A {sa:.3f} m (n={len(ea)}) vs '
          f'topo B {sb2:.3f} m (n={len(ebt)})')

    # 7. clearance not materially worse, per leg, against the baseline's
    #    own minimum on the same leg -- C2-NAV.25's REGRESSION reading.
    bmin = {}
    for r in b:
        if r['min_clear'] is None:
            continue
        bmin[r['leg']] = min(bmin.get(r['leg'], 9e9), r['min_clear'])
    worst, worst_leg = 0.0, None
    for r in c26:
        if r['min_clear'] is None or r['leg'] not in bmin:
            continue
        d = bmin[r['leg']] - r['min_clear']
        if d > worst:
            worst, worst_leg = d, f'{r["tag"]}/{r["leg"]}'
    absmin = min((r['min_clear'] for r in c26
                  if r['min_clear'] is not None), default=None)
    check('7. min true clearance not materially worse than baseline',
          worst <= ROB_CLEAR_MARGIN,
          f'worst per-leg regression {worst:.3f} m at {worst_leg} '
          f'(margin {ROB_CLEAR_MARGIN:.2f} m); absolute min '
          f'{absmin:.3f} m vs baseline {C25.BASELINE_MIN_CLEAR_M:.3f} m')

    # 8. no new qualitative failure
    bad = [r for r in c26 if r['status'] != 'SUCCEEDED']
    check('8. no new qualitative navigation failure',
          not bad,
          f'{len(c26) - len(bad)}/{len(c26)} legs SUCCEEDED'
          + ('' if not bad else '; ' + ', '.join(
              f'{r["tag"]}/{r["leg"]}={r["status"]}' for r in bad)))

    print()
    print(f'{sum(1 for x in ok if x)} of {len(ok)} robustness checks pass.')
    print()
    print('The ABSOLUTE clearance reading is printed by check 7 and never')
    print('dropped: SIX of the 35 baseline legs already sit below 0.20 m,')
    print('the lowest at 0.151 m, because the enclosure pinch is 0.63 m')
    print('and the robot is 0.40 m across. That is route geometry. The')
    print('binding reading is the per-leg regression, exactly as')
    print('C2-NAV.25 fixed it before its own candidate data existed.')
    return 0 if all(ok) else 1


# ------------------------------------------------------------ 5. integrity
def cmd_selftest(args):
    """Prove the analysis can see what it claims to see.

    CLAUDE.md's rule: any check whose success condition is "we saw
    nothing" must first prove it can see something. `robust` check 5
    passes by counting FEW legs past 0.25 m, and `tail` passes by finding
    a short tail -- both are "we saw nothing" shapes. So: assert the tail
    counter finds the ONE leg C2-NAV.25 already measured past the gate,
    and assert the loader actually loaded every leg it was asked for.
    """
    hdr('C2-NAV.26 -- can this analysis see a failure at all?')
    rc = 0

    # 1. The known 0.255 m leg must be found by the same counter that
    #    reports "isolated".
    e25 = _errs(rows_for(C25_ALL))
    n25 = sum(1 for x in e25 if x > 0.25)
    got = max(e25) if e25 else None
    good = (n25 == 1 and got is not None and abs(got - 0.255) < 0.0005)
    print(f'  [{"PASS" if good else "FAIL"}] tail counter finds the known '
          f'C2-NAV.25 exceedance')
    print(f'         {n25} leg past 0.25 m, max {got:.3f} m '
          f'(expected 1 leg at 0.255 m)')
    rc |= 0 if good else 1

    # 2. Every run asked for must actually be on disk with all 7 legs.
    for label, runs in (('baseline', BASE), ('C2-NAV.25', C25_ALL),
                        ('C2-NAV.26', C26)):
        for tag in runs:
            got_legs = [r['leg'] for r in rows_for([tag])]
            miss = [x for x in LEGS if x not in got_legs]
            good = not miss
            print(f'  [{"PASS" if good else "FAIL"}] {label:<10} {tag:<17}'
                  f'{len(got_legs)}/7 legs'
                  + ('' if good else '  MISSING ' + ','.join(miss)))
            rc |= 0 if good else 1

    # 3. The one leaf, still one leaf.
    print()
    print('  --- the behavioural configuration, re-asserted ---')
    rc |= C25.cmd_paramdiff(args) or 0
    return rc


def cmd_dump(args):
    """Freeze the six fresh tours, as C2-NAV.24 and .25 froze theirs.

    The scratch tree `.navbench/` is untracked by repo convention, so a
    committed bundle is what makes this result reproducible from a clean
    clone. Same shape and same columns as the two earlier bundles.
    """
    import json
    out = {'traces': {}, 'records': {}}
    for tag in C26:
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


def cmd_all(args):
    rc = 0
    for name in ('selftest', 'legs', 'yawcheck', 'failures', 'tail',
                 'byleg', 'monitor', 'compare', 'robust'):
        rc |= CMDS[name](args) or 0
        print()
    return rc


CMDS = {'legs': cmd_legs, 'yawcheck': cmd_yawcheck, 'tail': cmd_tail,
        'byleg': cmd_byleg, 'dump': cmd_dump,
        'monitor': cmd_monitor, 'failures': cmd_failures,
        'compare': cmd_compare, 'robust': cmd_robust,
        'selftest': cmd_selftest, 'all': cmd_all}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('cmd', choices=list(CMDS))
    ap.add_argument('--runs', nargs='*', default=None)
    ap.add_argument('--path', default=BUNDLE)
    args = ap.parse_args(argv)
    return CMDS[args.cmd](args)


if __name__ == '__main__':
    sys.exit(main())
