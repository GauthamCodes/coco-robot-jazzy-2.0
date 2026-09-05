#!/usr/bin/env python3
"""C2-NAV.21 -- read the live candidate runs and build the metric table.

Every number section 11 of the C2-NAV.21 brief asks for, per run, from
the run artifacts alone. Nothing here is derived from terminal history.

The degeneracy columns (`margin`, `n_at_min`, `rot_span`, `complete`, the
per-cycle illegal split) exist only in runs recorded with the C2-NAV.21
`nav_bench.py`; older runs are reported with those columns blank rather
than filled with a default, because a blank is a missing measurement and
a zero is a claim.

Safety is read from the C2-NAV.6 stop probe's `d_min_base_m` -- the true
geometric distance from `base_footprint` to the nearest live scan point
at ~20 Hz -- and NOT from nav_bench's `min_clearance_m`, which is
quantised by the static map's cell centres and is not trustworthy inside
the enclosure.

    python3 -P docs/data/c2nav21_live.py table c2n21_base_r1 c2n21_fpd_r1
    python3 -P docs/data/c2nav21_live.py leg   c2n21_fpd_r1
    python3 -P docs/data/c2nav21_live.py tour  c2n21_fpd_r1
    python3 -P docs/data/c2nav21_live.py selftest
    python3 -P docs/data/c2nav21_live.py dump  out.json  tag [tag ...]
"""

import csv
import json
import math
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

WT = os.path.abspath(os.path.join(HERE, '..', '..'))
RESULTS = os.path.join(WT, '.navbench', 'results')
LEG = 'enclosure_entry'

# Physical references, both measured elsewhere in this repo.
CIRCUMSCRIBED_R = 0.2051
POLY_STOP_R = 0.25

# C2-NAV.19's BAD run and C2-NAV.18's three GOOD ones, for the self-test
# and for the baseline column of every table.
BAD = 'c2n19_tour_r1'
GOOD = ['c2n18_tour_r1', 'c2n18_tour_r2', 'c2n18_tour_r3']
# Committed observations these tools must reproduce (C2-NAV.19/.20).
KNOWN = {
    BAD: {'status': 'TIMEOUT', 'duration_sim_s': 201.36,
          'final_goal_err_m': 1.106, 'worst_crawl_s': 42.84,
          'dwb_best_vx_zero_frac': 0.607},
    'c2n18_tour_r1': {'status': 'SUCCEEDED', 'worst_crawl_s': 1.59},
}


def hdr(t):
    print()
    print('=' * 78)
    print(t)
    print('=' * 78)


def _f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def bench(tag):
    p = os.path.join(RESULTS, f'{tag}.json')
    return json.load(open(p)) if os.path.exists(p) else None


def leg_record(tag, scenario=LEG):
    b = bench(tag)
    if not b:
        return None
    for leg in b.get('legs', []):
        if leg['scenario'] == scenario:
            return leg
    return None


def prior_legs_ok(tag, scenario=LEG):
    """Did the robot ARRIVE at this leg?

    A tour runs its legs back to back on one simulator, so a leg whose
    predecessors failed did not start from where the experiment says it
    starts. Measured on c2n21_base_r2: `corridor_gate` timed out 1.243 m
    in with PolygonStop latched, and every leg after it -- including
    `enclosure_entry` -- reports TIMEOUT having moved 0.000 m. Scoring
    that as an enclosure failure would be scoring a corridor wedge.

    Returns (ok, first_failing_leg).
    """
    b = bench(tag)
    if not b:
        return (False, None)
    for leg in b.get('legs', []):
        if leg['scenario'] == scenario:
            return (True, None)
        if leg.get('status') != 'SUCCEEDED':
            return (False, leg['scenario'])
    return (False, None)


def trace(tag, scenario=LEG, rep=0):
    p = os.path.join(RESULTS, f'{tag}_traces', f'{scenario}_rep{rep}.csv')
    if not os.path.exists(p):
        return []
    with open(p) as f:
        return list(csv.DictReader(f))


def stop_rows(tag):
    p = os.path.join(RESULTS, f'{tag}_stop.csv')
    if not os.path.exists(p):
        return []
    with open(p) as f:
        return list(csv.DictReader(f))


def has_degeneracy(tag):
    """Was this run recorded with the C2-NAV.21 instrument?"""
    rows = trace(tag)
    return bool(rows) and 'dwb_margin' in rows[0]


# ---------------------------------------------------------------------
# Per-cycle series
# ---------------------------------------------------------------------

def eval_cycles(rows):
    """One entry per DISTINCT /evaluation message: the trace holds the
    last value at 10 Hz while DWB publishes at ~5.75 Hz."""
    out, prev = [], None
    for r in rows:
        if not r.get('dwb_n'):
            continue
        key = (r['dwb_n'], r['dwb_illegal'], r['dwb_best_vx'],
               r['dwb_best_total'], r.get('dwb_best_wz', ''))
        if key != prev:
            out.append(r)
            prev = key
    return out


def zero_runs(rows):
    """Runs of consecutive rows whose SELECTED vx is exactly 0."""
    runs, start, prev_t = [], None, None
    for r in rows:
        v = _f(r.get('dwb_best_vx'))
        t = _f(r.get('t_rel'))
        if v is not None and abs(v) < 1e-9:
            if start is None:
                start = t
            prev_t = t
        else:
            if start is not None:
                runs.append((start, prev_t, prev_t - start))
            start = None
    if start is not None:
        runs.append((start, prev_t, prev_t - start))
    return runs


def q(vals, nd=3):
    vals = [v for v in vals if v is not None]
    if not vals:
        return None
    vals.sort()
    return {'n': len(vals), 'min': round(vals[0], nd),
            'median': round(vals[len(vals) // 2], nd),
            'max': round(vals[-1], nd),
            'mean': round(sum(vals) / len(vals), nd)}


# ---------------------------------------------------------------------
# One leg, every metric section 11 asks for
# ---------------------------------------------------------------------

def leg_metrics(tag, scenario=LEG):
    lr = leg_record(tag, scenario)
    if lr is None:
        return None
    rows = trace(tag, scenario)
    cyc = eval_cycles(rows)
    zr = zero_runs(cyc)
    vx = [_f(r.get('dwb_best_vx')) for r in cyc]
    vx = [v for v in vx if v is not None]
    wz = [_f(r.get('dwb_best_wz')) for r in cyc]
    wz = [v for v in wz if v is not None]
    marg = [_f(r.get('dwb_margin')) for r in cyc]
    marg = [v for v in marg if v is not None]
    span = [_f(r.get('dwb_rot_span')) for r in cyc]
    nmin = [_f(r.get('dwb_n_at_min')) for r in cyc]
    comp = [_f(r.get('dwb_complete')) for r in cyc]
    osc = [_f(r.get('dwb_ill_osc')) or 0.0 for r in cyc]
    base = [_f(r.get('dwb_ill_base')) or 0.0 for r in cyc]
    ill = [_f(r.get('dwb_illegal')) for r in cyc]
    n = [_f(r.get('dwb_n')) for r in cyc]

    # True geometric clearance, over the leg's own wall-clock span.
    sp = stop_rows(tag)
    d = [_f(r.get('d_min_base_m')) for r in sp]
    d = [v for v in d if v is not None and math.isfinite(v)]
    # The stop probe's own column names, checked against its header
    # rather than guessed: monitor_polygon / monitor_action / n_in_stop.
    poly = sum(1 for r in sp
               if (r.get('monitor_polygon') or '')
               .lower().startswith('polygonstop'))
    stop_action = sum(1 for r in sp
                      if (r.get('monitor_action') or '').upper() == 'STOP')
    in_stop = sum(1 for r in sp if (_f(r.get('n_in_stop')) or 0) > 0)

    # The degeneracy statistics have to be read where the stall is, not
    # over the whole leg: the terminal rotation makes RotateToGoal reject
    # every forward trajectory by design, which would be scored as a
    # zero-vx "win" that has nothing to do with the enclosure. nav_bench
    # already splits transit from terminal; `worst_crawl` names the
    # window C2-NAV.20 analysed.
    wc = lr.get('worst_crawl') or {}
    w0 = wc.get('t_rel_s')
    w1 = (w0 + wc['crawl_len_s']) if (w0 is not None
                                      and wc.get('crawl_len_s')) else None
    t_transit = lr.get('t_transit_s')

    def _sub(lo, hi):
        return [r for r in cyc
                if lo is not None and hi is not None
                and lo - 1e-9 <= (_f(r.get('t_rel')) or -1) <= hi + 1e-9]

    ok, failed_at = prior_legs_ok(tag, scenario)
    m = {
        'tag': tag, 'scenario': scenario,
        # A leg only measures what it claims to measure if the robot got
        # there. This is the gate, not a footnote.
        'reached_this_leg': ok,
        'first_failing_prior_leg': failed_at,
        'status': lr.get('status'),
        'duration_sim_s': lr.get('duration_sim_s'),
        'final_goal_err_m': lr.get('final_goal_err_m'),
        'worst_crawl_s': (lr.get('worst_crawl') or {}).get('crawl_len_s'),
        'worst_crawl_start_s': (lr.get('worst_crawl') or {}).get('t_rel_s'),
        'dwb_cycles': lr.get('dwb_cycles'),
        'dwb_illegal_frac': lr.get('dwb_illegal_frac'),
        'dwb_illegal_by_critic': lr.get('dwb_illegal_by_critic'),
        'dwb_best_critic_mean': lr.get('dwb_best_critic_mean'),
        'zero_vx_frac': (round(sum(1 for v in vx if abs(v) < 1e-9) / len(vx),
                               4) if vx else None),
        'longest_zero_run_s': round(max((r[2] for r in zr), default=0.0), 2),
        'n_zero_runs': len(zr),
        'first_zero_onset_s': (zr[0][0] if zr else None),
        'selected_vx': q(vx, 4), 'selected_wz': q(wz, 4),
        'negative_wz_frac': (round(sum(1 for v in wz if v < -1e-9) / len(wz),
                                   4) if wz else None),
        'positive_wz_frac': (round(sum(1 for v in wz if v > 1e-9) / len(wz),
                                   4) if wz else None),
        'illegal_per_cycle': q(ill, 1), 'traj_per_cycle': q(n, 1),
        'polygonstop_rows': poly, 'stop_rows': len(sp),
        'monitor_stop_rows': stop_action, 'rows_with_point_in_stop': in_stop,
        'd_min_base_m': q(d, 4),
        'd_min_base_below_polystop': sum(1 for v in d if v < POLY_STOP_R),
        'd_min_base_below_circumscribed': sum(1 for v in d
                                              if v < CIRCUMSCRIBED_R),
        'degeneracy_recorded': has_degeneracy(tag),
    }
    if m['degeneracy_recorded']:
        for label, sub in (('crawl', _sub(w0, w1)),
                           ('transit', _sub(0.0, t_transit))):
            sm = [_f(r.get('dwb_margin')) for r in sub]
            sm = [v for v in sm if v is not None]
            svx = [_f(r.get('dwb_best_vx')) for r in sub]
            svx = [v for v in svx if v is not None]
            swz = [_f(r.get('dwb_best_wz')) for r in sub]
            swz = [v for v in swz if v is not None]
            so = [_f(r.get('dwb_ill_osc')) or 0.0 for r in sub]
            m[f'{label}_cycles'] = len(sub)
            m[f'{label}_margin'] = q(sm)
            m[f'{label}_forward_wins'] = sum(1 for v in sm if v > 1e-9)
            m[f'{label}_exact_ties'] = sum(1 for v in sm if abs(v) <= 1e-9)
            m[f'{label}_zero_wins'] = sum(1 for v in sm if v < -1e-9)
            m[f'{label}_n_at_min'] = q([_f(r.get('dwb_n_at_min'))
                                        for r in sub], 1)
            m[f'{label}_rot_span'] = q([_f(r.get('dwb_rot_span'))
                                        for r in sub])
            m[f'{label}_complete'] = q([_f(r.get('dwb_complete'))
                                        for r in sub], 1)
            m[f'{label}_zero_vx_frac'] = (
                round(sum(1 for v in svx if abs(v) < 1e-9) / len(svx), 4)
                if svx else None)
            m[f'{label}_negative_wz_frac'] = (
                round(sum(1 for v in swz if v < -1e-9) / len(swz), 4)
                if swz else None)
            m[f'{label}_osc_ban_cycles'] = sum(1 for v in so if v > 300)
            m[f'{label}_osc_ban_frac'] = (round(
                sum(1 for v in so if v > 300) / len(sub), 4) if sub else None)
        m['illegal_by_critic_transit'] = lr.get(
            'dwb_illegal_by_critic_transit')
        m['illegal_by_critic_terminal'] = lr.get(
            'dwb_illegal_by_critic_terminal')
        m.update({
            'margin': q(marg), 'rot_span': q(span), 'n_at_min': q(nmin, 1),
            'complete': q(comp, 1),
            'margin_cycles': len(marg),
            'forward_wins': sum(1 for v in marg if v > 1e-9),
            'exact_ties': sum(1 for v in marg if abs(v) <= 1e-9),
            'zero_wins': sum(1 for v in marg if v < -1e-9),
            'tie_frac': (round(sum(1 for v in marg if abs(v) <= 1e-9)
                               / len(marg), 4) if marg else None),
            'osc_illegal_cycles': sum(1 for v in osc if v > 0),
            'osc_illegal_max': max(osc) if osc else 0,
            'osc_illegal_cycles_over_300': sum(1 for v in osc if v > 300),
            'base_illegal_median': (statistics.median(base)
                                    if base else None),
        })
    return m


def tour_metrics(tag):
    b = bench(tag)
    if not b:
        return None
    legs = []
    for leg in b.get('legs', []):
        legs.append({'scenario': leg['scenario'], 'status': leg.get('status'),
                     'duration_sim_s': leg.get('duration_sim_s'),
                     'final_goal_err_m': leg.get('final_goal_err_m'),
                     'worst_crawl_s': (leg.get('worst_crawl') or {})
                     .get('crawl_len_s'),
                     'final_yaw_err_deg': leg.get('final_yaw_err_deg')})
    sp = stop_rows(tag)
    d = [_f(r.get('d_min_base_m')) for r in sp]
    d = [v for v in d if v is not None and math.isfinite(v)]
    return {'tag': tag, 'n_legs': len(legs),
            'succeeded': sum(1 for x in legs if x['status'] == 'SUCCEEDED'),
            'legs': legs,
            'tour_d_min_base_m': q(d, 4),
            'tour_below_polystop': sum(1 for v in d if v < POLY_STOP_R),
            'tour_below_circumscribed': sum(1 for v in d
                                            if v < CIRCUMSCRIBED_R)}


# ---------------------------------------------------------------------
# Reports
# ---------------------------------------------------------------------

def report_leg(tag):
    m = leg_metrics(tag)
    hdr(f'C2-NAV.21 live -- {tag} leg {LEG}')
    if m is None:
        print('  no such run')
        return None
    for k, v in m.items():
        print(f'  {k:<34} {v}')
    return m


def report_tour(tag):
    t = tour_metrics(tag)
    hdr(f'C2-NAV.21 live -- {tag} seven-leg tour')
    if t is None:
        print('  no such run')
        return None
    print(f'  {"leg":<20} {"status":<12} {"dur_s":>8} {"err_m":>8} '
          f'{"crawl_s":>8} {"yaw_deg":>8}')
    for x in t['legs']:
        print(f'  {x["scenario"]:<20} {str(x["status"]):<12} '
              f'{(x["duration_sim_s"] or 0):>8.2f} '
              f'{(x["final_goal_err_m"] or 0):>8.3f} '
              f'{(x["worst_crawl_s"] or 0):>8.2f} '
              f'{(x["final_yaw_err_deg"] or 0):>8.2f}')
    print(f'  succeeded {t["succeeded"]}/{t["n_legs"]}   '
          f'true clearance {t["tour_d_min_base_m"]}   '
          f'rows under PolygonStop {t["tour_below_polystop"]}   '
          f'under circumscribed {t["tour_below_circumscribed"]}')
    return t


def report_table(tags):
    hdr('C2-NAV.21 live candidate table -- ' + LEG)
    print('  zfrac = fraction of DWB cycles whose SELECTED vx is 0')
    print('  zrun  = longest continuous run of selected vx = 0')
    print('  margin= best zero-vx total minus best forward total, over')
    print('          COMPLETE trajectories only (a short-circuited total is')
    print('          a partial sum and cannot be compared to a complete one),')
    print('          READ INSIDE THE WORST CRAWL WINDOW. Leg-wide would be')
    print('          dominated by the terminal rotation, where RotateToGoal')
    print('          rejects every forward trajectory by design.')
    print('  span  = score range across the zero-vx rotation block')
    print('  negwz = fraction of selected wz that are negative (leg-wide)')
    print('  osc   = cycles where Oscillation banned >300 trajectories --')
    print('          a full directional ban is exactly 400 of 819.')
    print('  d_min = true geometric base-to-scan clearance (stop probe),')
    print(f'          NOT nav_bench min_clearance_m. PolygonStop '
          f'{POLY_STOP_R} m,')
    print(f'          circumscribed radius {CIRCUMSCRIBED_R} m.')
    print()
    print(f'  {"run":<18} {"status":<10} {"zfrac":>6} {"zrun":>7} '
          f'{"margin med":>11} {"f/t/z":>13} {"tied":>6} {"span med":>9} '
          f'{"negwz":>6} {"osc":>6} {"d_min":>7} {"err_m":>7}')
    out = []
    for tag in tags:
        m = leg_metrics(tag)
        if m is None:
            print(f'  {tag:<18} (no artifact)')
            continue
        out.append(m)
        dg = m.get('degeneracy_recorded')
        mm = (f'{m["crawl_margin"]["median"]:>11.2f}'
              if dg and m.get('crawl_margin') else f'{"--":>11}')
        ftz = (f'{m["crawl_forward_wins"]:>4}/{m["crawl_exact_ties"]:>3}/'
               f'{m["crawl_zero_wins"]:>3}' if dg else f'{"--":>13}')
        tied = (f'{m["crawl_n_at_min"]["median"]:>6.1f}'
                if dg and m.get('crawl_n_at_min') else f'{"--":>6}')
        sp = (f'{m["crawl_rot_span"]["median"]:>9.2f}'
              if dg and m.get('crawl_rot_span') else f'{"--":>9}')
        nw = (f'{m["negative_wz_frac"]:>6.3f}'
              if m.get('negative_wz_frac') is not None else f'{"--":>6}')
        dmin = (f'{m["d_min_base_m"]["min"]:>7.3f}'
                if m.get('d_min_base_m') else f'{"--":>7}')
        osc = (f'{m["crawl_osc_ban_frac"]:>6.3f}'
               if dg and m.get('crawl_osc_ban_frac') is not None
               else f'{"--":>6}')
        if not m['reached_this_leg']:
            print(f'  {tag:<18} {"VOID":<10} did not reach this leg -- '
                  f'wedged at {m["first_failing_prior_leg"]}; every later '
                  f'leg moved 0.000 m')
            continue
        print(f'  {tag:<18} {str(m["status"]):<10} '
              f'{(m["zero_vx_frac"] or 0):>6.3f} '
              f'{m["longest_zero_run_s"]:>7.2f} {mm} {ftz} {tied} {sp} '
              f'{nw} {osc} {dmin} {(m["final_goal_err_m"] or 0):>7.3f}')
    return out


def report_arms(specs):
    """Group runs into candidate arms and report only the VALID ones.

    `specs` are `arm=tag,tag,...`. A run whose earlier legs failed did
    not start the enclosure from where the experiment says it starts, so
    it is excluded from every aggregate and counted separately -- an
    excluded run is a smaller sample, not a failure of the candidate.
    """
    hdr('C2-NAV.21 -- per-arm summary over runs that REACHED the leg')
    print('  Outcome is the enclosure_entry status. The degeneracy')
    print('  statistics are per-DWB-cycle inside the crawl window, so a')
    print('  single valid run already carries hundreds of samples of')
    print('  them, while the OUTCOME is one sample per run and the')
    print('  failure is intermittent. Read the two differently.')
    print()
    out = []
    for spec in specs:
        arm, _, taglist = spec.partition('=')
        tags = [t for t in taglist.split(',') if t]
        valid, void = [], []
        for t in tags:
            m = leg_metrics(t)
            if m is None:
                continue
            (valid if m['reached_this_leg'] else void).append(m)
        rec = {'arm': arm, 'n_runs': len(tags), 'n_valid': len(valid),
               'n_void': len(void),
               'void_tags': [(m['tag'], m['first_failing_prior_leg'])
                             for m in void],
               'outcomes': [(m['tag'], m['status']) for m in valid],
               'succeeded': sum(1 for m in valid
                                if m['status'] == 'SUCCEEDED')}
        for key, src in (('margin_median', 'crawl_margin'),
                         ('rot_span_median', 'crawl_rot_span'),
                         ('n_at_min_median', 'crawl_n_at_min')):
            vals = [m[src]['median'] for m in valid
                    if m.get(src) and m[src].get('median') is not None]
            rec[key] = q(vals) if vals else None
        for key in ('crawl_zero_vx_frac', 'longest_zero_run_s',
                    'crawl_osc_ban_frac', 'duration_sim_s',
                    'final_goal_err_m'):
            vals = [m[key] for m in valid if m.get(key) is not None]
            rec[key] = q(vals) if vals else None
        dmins = [m['d_min_base_m']['min'] for m in valid
                 if m.get('d_min_base_m')]
        rec['true_clearance_min'] = min(dmins) if dmins else None
        rec['any_below_circumscribed'] = sum(
            m['d_min_base_below_circumscribed'] for m in valid)
        out.append(rec)
        print(f'  {arm}: {rec["n_valid"]} valid of {rec["n_runs"]}'
              + (f'  (VOID: {rec["void_tags"]})' if void else ''))
        print(f'    outcomes            {rec["outcomes"]}')
        for k in ('margin_median', 'rot_span_median', 'n_at_min_median',
                  'crawl_zero_vx_frac', 'longest_zero_run_s',
                  'crawl_osc_ban_frac', 'duration_sim_s',
                  'final_goal_err_m'):
            print(f'    {k:<20} {rec[k]}')
        print(f'    true clearance min  {rec["true_clearance_min"]}   '
              f'rows below circumscribed radius '
              f'{rec["any_below_circumscribed"]}')
        print()
    return out


def self_test():
    """Refuse to report until the reader reproduces C2-NAV.19's and
    C2-NAV.18's committed observations from their own artifacts."""
    hdr('C2-NAV.21 live-reader self-test')
    ok = True
    for tag, want in KNOWN.items():
        m = leg_metrics(tag)
        if m is None:
            print(f'  [SKIP] {tag}: artifact not on this machine')
            continue
        for k, v in want.items():
            if k == 'dwb_best_vx_zero_frac':
                got = m['zero_vx_frac']
                good = got is not None and abs(got - v) < 0.02
            elif k == 'worst_crawl_s':
                got = m['worst_crawl_s']
                good = got is not None and abs(got - v) < 0.01
            elif isinstance(v, float):
                got = m.get(k)
                good = got is not None and abs(got - v) < 0.01
            else:
                got = m.get(k)
                good = got == v
            ok = ok and good
            print(f'  [{"OK " if good else "FAIL"}] {tag} {k}: '
                  f'got {got!r} want {v!r}')
        # The degeneracy columns must be ABSENT for pre-C2-NAV.21 runs,
        # not zero: a zero would be a claim the run never made.
        good = (m['reached_this_leg'] is True)
        ok = ok and good
        print(f'  [{"OK " if good else "FAIL"}] {tag} reached the leg: '
              f'got {m["reached_this_leg"]!r} want True')
        good = (m['degeneracy_recorded'] is False)
        ok = ok and good
        print(f'  [{"OK " if good else "FAIL"}] {tag} degeneracy columns '
              f'correctly absent: got {m["degeneracy_recorded"]!r} '
              f'want False')
    print()
    print('  SELF-TEST ' + ('PASSED' if ok else 'FAILED'))
    return ok


def dump(path, tags):
    payload = {'experiment': 'C2-NAV.21', 'stage': 'live',
               'self_test_passed': self_test(),
               'polygonstop_radius_m': POLY_STOP_R,
               'circumscribed_radius_m': CIRCUMSCRIBED_R,
               'legs': [leg_metrics(t) for t in tags],
               'tours': [tour_metrics(t) for t in tags]}
    payload['valid_legs'] = [m['tag'] for m in payload['legs']
                             if m and m['reached_this_leg']]
    payload['void_legs'] = [(m['tag'], m['first_failing_prior_leg'])
                            for m in payload['legs']
                            if m and not m['reached_this_leg']]
    with open(path, 'w') as f:
        json.dump(payload, f, indent=1, sort_keys=True, default=str)
    print(f'\nwrote {path}')


def main():
    a = sys.argv[1:] or ['selftest']
    cmd = a[0]
    if cmd == 'selftest':
        sys.exit(0 if self_test() else 1)
    elif cmd == 'leg':
        for t in a[1:]:
            report_leg(t)
    elif cmd == 'tour':
        for t in a[1:]:
            report_tour(t)
    elif cmd == 'table':
        report_table(a[1:])
    elif cmd == 'arms':
        report_arms(a[1:])
    elif cmd == 'dump':
        dump(a[1], a[2:])
    else:
        print(__doc__)
        sys.exit(2)


if __name__ == '__main__':
    main()
