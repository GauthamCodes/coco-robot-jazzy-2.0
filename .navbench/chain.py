#!/usr/bin/env python3
"""Is the collision monitor's output what reaches the wheels?

Reads every nav_bench trace for a tag and compares, row by row:

    v_smoothed (monitor INPUT) -> v_cmdvel (monitor OUTPUT) -> v_wheel

Two questions, kept apart because they have different answers:
  1. Does the monitor ever actually reduce a command (input != output)?
     If it never does, "the monitor gates" is a claim about state, not
     about velocity.
  2. When it does, does the reduced value reach the wheels? A wheel
     command ABOVE the monitor's output is the signature of another
     publisher on the path -- the /cmd_vel_nav ownership loop.
"""
import csv
import glob
import sys
from collections import Counter

TOL = 1e-4


def num(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def main(pattern):
    files = sorted(glob.glob(pattern))
    if not files:
        print(f'no traces matching {pattern}')
        return 1
    tot = Counter()
    worst_gap = 0.0
    worst_row = None
    per_scen = {}
    for path in files:
        name = path.split('/')[-1].replace('.csv', '')
        s = Counter()
        with open(path) as f:
            for row in csv.DictReader(f):
                vin = num(row['v_smoothed'])
                vout = num(row['v_cmdvel'])
                vwh = num(row['v_wheel'])
                if vin is None or vout is None or vwh is None:
                    continue
                s['rows'] += 1
                if abs(vin - vout) > TOL:
                    s['monitor_reduced'] += 1
                    if abs(vout) < abs(vin) - TOL:
                        s['reduced_correctly'] += 1
                if abs(vout - vwh) > TOL:
                    s['out_ne_wheel'] += 1
                # The defect signature: wheels FASTER than the gate.
                if abs(vwh) > abs(vout) + TOL:
                    s['wheel_exceeds_gate'] += 1
                    gap = abs(vwh) - abs(vout)
                    if gap > worst_gap:
                        worst_gap = gap
                        worst_row = (name, row['t_rel'], vin, vout, vwh,
                                     row['cm_action'])
        per_scen[name] = s
        tot.update(s)

    print(f'{len(files)} traces, {tot["rows"]} rows\n')
    print(f'{"":<34}{"rows":>7}{"gated":>7}{"out!=wheel":>11}'
          f'{"wheel>gate":>11}')
    for name, s in sorted(per_scen.items()):
        if s['rows']:
            print(f'{name:<34}{s["rows"]:>7}{s["monitor_reduced"]:>7}'
                  f'{s["out_ne_wheel"]:>11}{s["wheel_exceeds_gate"]:>11}')
    print(f'\n{"TOTAL":<34}{tot["rows"]:>7}{tot["monitor_reduced"]:>7}'
          f'{tot["out_ne_wheel"]:>11}{tot["wheel_exceeds_gate"]:>11}')
    print(f'\nmonitor reduced the command on '
          f'{tot["monitor_reduced"]}/{tot["rows"]} rows '
          f'({100.0 * tot["monitor_reduced"] / max(tot["rows"], 1):.2f} %)')
    print(f'wheels exceeded the monitor output on '
          f'{tot["wheel_exceeds_gate"]}/{tot["rows"]} rows '
          f'({100.0 * tot["wheel_exceeds_gate"] / max(tot["rows"], 1):.2f} %)')
    if worst_row:
        print(f'worst overshoot: {worst_row[0]} t={worst_row[1]}s  '
              f'in={worst_row[2]} gate={worst_row[3]} wheel={worst_row[4]} '
              f'cm_action={worst_row[5]}  gap={worst_gap:.4f} m/s')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1]))
