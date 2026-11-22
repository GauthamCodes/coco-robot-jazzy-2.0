#!/usr/bin/env python3
"""Aggregate nav_bench results into the C2-NAV.0 baseline table.

Per scenario across repeats: success rate, and the median of each
measure. Medians rather than means because n=3 and one 75 s timeout
would drag every mean it touched.
"""
import json
import statistics
import sys

ORDER = ['open_space', 'wall_adjacent', 'wall_parallel', 'obstacle_corner',
         'corridor_gate', 'enclosure_entry', 'enclosure_exit']


def med(vals):
    vals = [v for v in vals if v is not None]
    return round(statistics.median(vals), 3) if vals else None


def main(paths):
    for path in paths:
        d = json.load(open(path))
        legs = d['legs']
        print(f"\n######## {d['tag']}  ({len(legs)} legs)")
        by = {}
        for l in legs:
            by.setdefault(l.get('scenario'), []).append(l)
        hdr = (f'{"scenario":<17}{"ok":>6}{"t_s":>7}{"transit":>8}'
               f'{"term":>7}{"len":>7}{"clear":>7}{"v_tr":>7}{"stop":>5}'
               f'{"osc":>5}{"gate%":>7}{"prog":>5}{"ill%":>7}')
        print(hdr)
        print('-' * len(hdr))
        for s in ORDER:
            ls = by.get(s)
            if not ls:
                continue
            ok = sum(1 for l in ls if l.get('status') == 'SUCCEEDED')
            print(f'{s:<17}{f"{ok}/{len(ls)}":>6}'
                  f'{med([l.get("duration_sim_s") for l in ls]):>7}'
                  f'{str(med([l.get("t_transit_s") for l in ls])):>8}'
                  f'{str(med([l.get("t_terminal_s") for l in ls])):>7}'
                  f'{med([l.get("path_len_m") for l in ls]):>7}'
                  f'{med([l.get("min_clearance_m") for l in ls]):>7}'
                  f'{str(med([l.get("transit_speed_mean") for l in ls])):>7}'
                  f'{med([l.get("n_stops") for l in ls]):>5}'
                  f'{med([l.get("n_osc_cmd") for l in ls]):>5}'
                  f'{med([l.get("cm_gated_frac") for l in ls]):>7}'
                  f'{med([l.get("n_progress_failures") for l in ls]):>5}'
                  f'{med([l.get("dwb_illegal_frac") for l in ls]):>7}')
        # Whole-tag rollups that the per-scenario table hides.
        tr = [l.get('t_transit_s') for l in legs if l.get('t_transit_s')]
        te = [l.get('t_terminal_s') for l in legs if l.get('t_terminal_s')]
        print(f'\n  legs succeeded          '
              f'{sum(1 for l in legs if l.get("status") == "SUCCEEDED")}'
              f'/{len(legs)}')
        if tr and te:
            print(f'  median transit          {statistics.median(tr):.2f} s')
            print(f'  median terminal         {statistics.median(te):.2f} s')
            print(f'  terminal share of leg   '
                  f'{statistics.median(te) / (statistics.median(te) + statistics.median(tr)):.1%}')
        print(f'  median transit speed    '
              f'{med([l.get("transit_speed_mean") for l in legs])} m/s '
              f'(max_vel_x is 0.30)')
        print(f'  median DWB rate         '
              f'{med([l.get("dwb_hz") for l in legs])} Hz '
              f'(controller_frequency 10.0)')
        print(f'  total progress failures '
              f'{sum(l.get("n_progress_failures", 0) for l in legs)}')
        print(f'  total loop-rate misses  '
              f'{sum(l.get("n_loop_rate_misses", 0) for l in legs)}')
        print(f'  total stale-cmd drops   '
              f'{sum(l.get("n_stale_cmd_drops", 0) for l in legs)}')
        # Which critic rejected trajectories, summed over every leg.
        tot = {}
        for l in legs:
            for k, v in (l.get('dwb_illegal_by_critic') or {}).items():
                tot[k] = tot.get(k, 0) + v
        n = sum(tot.values()) or 1
        print('  rejections by critic:   ' + ', '.join(
            f'{k} {v} ({100.0 * v / n:.1f}%)'
            for k, v in sorted(tot.items(), key=lambda kv: -kv[1])))
        # Mean scaled contribution to the CHOSEN trajectory.
        crit = {}
        for l in legs:
            for k, v in (l.get('dwb_best_critic_mean') or {}).items():
                crit.setdefault(k, []).append(v)
        s = {k: statistics.mean(v) for k, v in crit.items()}
        tt = sum(s.values()) or 1
        print('  chosen-trajectory score share: ' + ', '.join(
            f'{k} {v:.1f} ({100.0 * v / tt:.1f}%)'
            for k, v in sorted(s.items(), key=lambda kv: -kv[1])))


if __name__ == '__main__':
    main(sys.argv[1:])
