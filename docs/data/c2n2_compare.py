#!/usr/bin/env python3
"""C2-NAV.2 vs the committed C2-NAV.0 baseline, enclosure_entry only."""
import json
import statistics as st
import sys

# Defaults assume cwd is docs/data, the way c2nav0_analysis.py is run.
BASE = sys.argv[1] if len(sys.argv) > 1 else 'c2nav0_baselineA.json'
EXP = sys.argv[2] if len(sys.argv) > 2 else 'c2n2_navA_baseobs.json'
PROBE = sys.argv[3] if len(sys.argv) > 3 else 'c2n2_eval_probe.json'


def legs(path, scen='enclosure_entry'):
    d = json.load(open(path))
    return [l for l in d['legs'] if l.get('scenario') == scen]


def med(xs):
    xs = [x for x in xs if x is not None]
    return round(st.median(xs), 3) if xs else None


def get(ls, k):
    return [l.get(k) for l in ls]


def bo(ls):
    """Mean BaseObstacle contribution to the chosen trajectory."""
    return [l.get('dwb_best_critic_mean', {}).get('BaseObstacle') for l in ls]


def share(ls):
    """BaseObstacle as a share of the chosen trajectory's total score."""
    out = []
    for l in ls:
        c = l.get('dwb_best_critic_mean') or {}
        tot = sum(v for v in c.values())
        out.append(round(100.0 * c.get('BaseObstacle', 0.0) / tot, 1)
                   if tot else None)
    return out


def main():
    b, e = legs(BASE), legs(EXP)
    print(f'baseline legs {len(b)}   experiment legs {len(e)}\n')

    print('PER-REPEAT')
    hdr = ('run', 'status', 'start', 'end', 'len_m', 'goal_err',
           'vx0_frac', 'BaseObs', 'BO%', 'scan_min', 'prog')
    print(('{:>10} {:>8} {:>16} {:>16} {:>7} {:>8} {:>8} {:>8} {:>6} '
           '{:>8} {:>5}').format(*hdr))
    for tag, ls in (('C2-NAV.0', b), ('C2-NAV.2', e)):
        for l, s in zip(ls, share(ls)):
            sw = l.get('start_world') or [None, None]
            ew = l.get('end_world') or [None, None]
            c = l.get('dwb_best_critic_mean') or {}
            print(('{:>10} {:>8} {:>16} {:>16} {:>7} {:>8} {:>8} {:>8} '
                   '{:>6} {:>8} {:>5}').format(
                f"{tag} r{l.get('rep')}",
                str(l.get('status')),
                f'({sw[0]},{sw[1]})' if sw[0] is not None else '-',
                f'({ew[0]},{ew[1]})' if ew[0] is not None else '-',
                str(l.get('path_len_m')),
                str(l.get('final_goal_err_m')),
                str(l.get('dwb_best_vx_zero_frac')),
                str(round(c.get('BaseObstacle', 0.0), 2)),
                str(s),
                str(l.get('min_scan_range_m')),
                str(l.get('n_progress_failures'))))
        print()

    print('MEDIANS')
    rows = [
        ('success', f"{sum(1 for l in b if l.get('status')=='SUCCEEDED')}/3",
         f"{sum(1 for l in e if l.get('status')=='SUCCEEDED')}/3"),
        ('duration_sim_s', med(get(b, 'duration_sim_s')),
         med(get(e, 'duration_sim_s'))),
        ('path_len_m', med(get(b, 'path_len_m')), med(get(e, 'path_len_m'))),
        ('final_goal_err_m', med(get(b, 'final_goal_err_m')),
         med(get(e, 'final_goal_err_m'))),
        ('dwb_best_vx_zero_frac', med(get(b, 'dwb_best_vx_zero_frac')),
         med(get(e, 'dwb_best_vx_zero_frac'))),
        ('dwb_best_vx_mean', med(get(b, 'dwb_best_vx_mean')),
         med(get(e, 'dwb_best_vx_mean'))),
        ('BaseObstacle on chosen', med(bo(b)), med(bo(e))),
        ('BaseObstacle % of chosen', med(share(b)), med(share(e))),
        ('min_clearance_m', med(get(b, 'min_clearance_m')),
         med(get(e, 'min_clearance_m'))),
        ('min_scan_range_m', med(get(b, 'min_scan_range_m')),
         med(get(e, 'min_scan_range_m'))),
        ('dwb_illegal_frac', med(get(b, 'dwb_illegal_frac')),
         med(get(e, 'dwb_illegal_frac'))),
        ('n_progress_failures', med(get(b, 'n_progress_failures')),
         med(get(e, 'n_progress_failures'))),
        ('dwb_hz', med(get(b, 'dwb_hz')), med(get(e, 'dwb_hz'))),
        ('cm gated frac', med(get(b, 'cm_gated_frac')),
         med(get(e, 'cm_gated_frac'))),
    ]
    print('{:>28} {:>14} {:>14}'.format('metric', 'C2-NAV.0', 'C2-NAV.2'))
    for name, x, y in rows:
        print('{:>28} {:>14} {:>14}'.format(name, str(x), str(y)))

    print('\nSTALL SNAPSHOTS')
    for tag, ls in (('C2-NAV.0', b), ('C2-NAV.2', e)):
        for l in ls:
            w = l.get('worst_crawl') or {}
            if not w:
                continue
            cr = w.get('dwb_chosen_critics') or {}
            tot = sum(cr.values()) or 1.0
            print(f"  {tag} r{l.get('rep')}: {w.get('crawl_len_s')}s at "
                  f"{w.get('pose_world')} dist={w.get('dist_to_goal_m')} "
                  f"vx={w.get('dwb_chosen_vx')} "
                  f"illegal={w.get('dwb_n_illegal')}/{w.get('dwb_n_traj')} "
                  f"BaseObs={cr.get('BaseObstacle')} "
                  f"({100.0*cr.get('BaseObstacle', 0.0)/tot:.1f}% of chosen) "
                  f"cm={w.get('collision_monitor')} "
                  f"scan={w.get('scan_min_m')}")

    # The decisive evidence: at the stall, why does forward motion lose?
    try:
        d = json.load(open(PROBE))
    except OSError:
        print(f'\n(no probe file {PROBE})')
        return 0
    print(f'\nEVALUATION PROBE — {len(d)} control cycles at the stall')
    print('  chosen vx == 0 in '
          f"{sum(1 for c in d if abs(c['chosen']['vx']) < 1e-9)}/{len(d)}")
    print('  legal forward (vx>=0.15) trajectories per cycle: '
          f"{min(c['n_fwd_legal'] for c in d)}-"
          f"{max(c['n_fwd_legal'] for c in d)} of 819")
    print('  BaseObstacle on the chosen trajectory: '
          f"{sorted(set(round(c['chosen']['critics'].get('BaseObstacle', 0), 3) for c in d))}")
    gaps = [c['gap_total'] for c in d if c.get('gap_total') is not None]
    print(f'  median score gap (best forward - chosen): {st.median(gaps):.2f}')
    agg = {}
    for c in d:
        for k, v in (c.get('gap_fwd_minus_chosen') or {}).items():
            agg[k] = agg.get(k, 0.0) + v
    print('  that gap, summed by critic over all cycles:')
    for k, v in sorted(agg.items(), key=lambda kv: -kv[1]):
        print(f'      {k:14s} {v:8.2f}')
    print('\n  objective vs commanded vx, cycle 0 (best legal total per vx):')
    print('      {:>8} {:>8} {:>9} {:>9} {:>10} {:>9} {:>10}'.format(
        'vx', 'total', 'BaseObs', 'GoalDist', 'GoalAlign', 'PathDist',
        'PathAlign'))
    for vx, rec in d[0]['by_vx'].items():
        cr = rec['critics']
        print('      {:8.4f} {:8.2f} {:9.2f} {:9.2f} {:10.2f} {:9.2f} '
              '{:10.2f}'.format(
                  float(vx), rec['total'], cr.get('BaseObstacle', 0),
                  cr.get('GoalDist', 0), cr.get('GoalAlign', 0),
                  cr.get('PathDist', 0), cr.get('PathAlign', 0)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
