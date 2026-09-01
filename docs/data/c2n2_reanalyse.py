#!/usr/bin/env python3
"""Re-analyse a c2n2 evaluation capture, honouring short-circuiting.

`short_circuit_trajectory_evaluation: true` makes DWB stop scoring a
trajectory the moment its RUNNING total exceeds the best complete score so
far. The critic order is RotateToGoal, Oscillation, BaseObstacle,
GoalAlign, PathAlign, PathDist, GoalDist, so an aborted trajectory carries
only the critics evaluated before the abort and its `total` is a PARTIAL
sum. Treating the missing critics as 0.0 and differencing produces large
negative GoalDist/GoalAlign terms that are an artefact of the abort, not a
measurement. This separates the two cases:

  complete  -- all 7 critics scored; the score gap is a real decomposition
  aborted   -- fewer than 7; the only valid statement is that the critics
               scored BEFORE the abort already exceed the winner's total,
               and which critic carried that sum.
"""
import json
import statistics as st
import sys

NCRIT = 7


def is_complete(critics):
    return len(critics) >= NCRIT


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else 'c2n2_eval_probe.json'
    d = json.load(open(path))
    print(f'{path}: {len(d)} cycles, stall at {d[0]["pose"]}, '
          f'{d[0]["dist_to_goal"]} m to goal')
    if d[0].get('yaw') is not None:
        print(f'  robot yaw {d[0]["yaw"]}  bearing to goal '
              f'{d[0]["bearing_to_goal"]}')
    print()

    chosen_zero = sum(1 for c in d if abs(c['chosen']['vx']) < 1e-9)
    print(f'chosen vx == 0 in {chosen_zero}/{len(d)} cycles')
    print('BaseObstacle on the CHOSEN trajectory: '
          f'{sorted(set(round(c["chosen"]["critics"].get("BaseObstacle", 0), 3) for c in d))}')
    print('chosen total: '
          f'{sorted(set(round(c["chosen"]["total"], 1) for c in d))}')
    print()

    # Split the per-vx table into completely scored and aborted.
    print('Per cycle, over the forward (vx >= 0.15) samples in by_vx:')
    print('  {:>4} {:>9} {:>9} {:>12} {:>12} {:>10}'.format(
        'cyc', 'n_fwd', 'complete', 'best_complete', 'chosen_total', 'verdict'))
    rows = []
    for i, c in enumerate(d):
        fwd = {vx: r for vx, r in c['by_vx'].items() if float(vx) >= 0.15}
        comp = {vx: r for vx, r in fwd.items() if is_complete(r['critics'])}
        ch = c['chosen']['total']
        best_comp = min((r['total'] for r in comp.values()), default=None)
        if best_comp is None:
            verdict = 'all aborted'
        elif best_comp > ch:
            verdict = 'loses'
        else:
            verdict = 'WINS?'
        rows.append((len(fwd), len(comp), best_comp, ch))
        print('  {:>4} {:>9} {:>9} {:>12} {:>12} {:>10}'.format(
            i, len(fwd), len(comp),
            'n/a' if best_comp is None else f'{best_comp:.2f}',
            f'{ch:.2f}', verdict))
    print()

    # Case A: completely scored forward trajectories that still lose.
    gaps, contrib = [], {}
    for c in d:
        ch = c['chosen']
        fwd = [r for vx, r in c['by_vx'].items()
               if float(vx) >= 0.15 and is_complete(r['critics'])]
        if not fwd or not is_complete(ch['critics']):
            continue
        best = min(fwd, key=lambda r: r['total'])
        gaps.append(best['total'] - ch['total'])
        for k in set(best['critics']) | set(ch['critics']):
            contrib[k] = contrib.get(k, 0.0) + (
                best['critics'].get(k, 0.0) - ch['critics'].get(k, 0.0))
    if gaps:
        print(f'CASE A -- {len(gaps)} cycles have a COMPLETELY scored forward')
        print('trajectory. It still loses to standing still. The gap, and')
        print('what each critic contributes to it:')
        print(f'  median gap {st.median(gaps):.2f}   '
              f'range {min(gaps):.2f}..{max(gaps):.2f}')
        for k, v in sorted(contrib.items(), key=lambda kv: -kv[1]):
            print(f'    {k:14s} {v:9.2f}')
    else:
        print('CASE A -- no cycle has a completely scored forward trajectory.')
    print()

    # Case B: forward trajectories aborted, and on which critic.
    ab_n, ab_bo = 0, 0
    bo_vals, ch_tot = [], []
    for c in d:
        ch = c['chosen']['total']
        for vx, r in c['by_vx'].items():
            if float(vx) < 0.15 or is_complete(r['critics']):
                continue
            ab_n += 1
            b = r['critics'].get('BaseObstacle', 0.0)
            if b > ch:
                ab_bo += 1
                bo_vals.append(b)
                ch_tot.append(ch)
    print(f'CASE B -- {ab_n} forward vx samples were ABORTED mid-scoring.')
    if ab_n:
        print(f'  of those, {ab_bo} were already over the winner on '
              'BaseObstacle ALONE')
        if bo_vals:
            print(f'  BaseObstacle on those: {sorted(set(round(v,1) for v in bo_vals))}'
                  f'  vs a winning total of {sorted(set(round(v,1) for v in ch_tot))}')
            print('  i.e. at scale 2.0 the implied cell cost is '
                  f'{sorted(set(round(v/2.0,1) for v in bo_vals))} '
                  '(BaseObstacle score / scale)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
