#!/usr/bin/env python3
"""C2-NAV.27 -- the last seconds of wall_adjacent, every run, every arm.

OFFLINE ONLY.

The brief's central question is which of two things happened:

    (a) the robot OVERSHOT a wall-adjacent goal under command, or
    (b) the robot stopped somewhere legal and only the NEXT planner
        rejected that place.

Neither turns out to be the whole story for c2n26_slow_r2, and this
prints the evidence that decides it: the ground-truth distance past the
goal over the last seconds, the commanded and achieved vx, which
collision-monitor polygon was active, and which DWB critic was
rejecting trajectories.
"""
import csv
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..'))
RES = os.path.join(ROOT, 'results')

BASE = ['c2n21_base_r1', 'c2n21_base_r3', 'c2n21_base_r4',
        'c2n21_bbase_r2', 'c2n21_bbase_r3']
C25 = ['c2n25_slow_r1', 'c2n25_slow_r2', 'c2n25_bslow_r1']
C26 = ['c2n26_slow_r1', 'c2n26_slow_r2', 'c2n26_slow_r3',
       'c2n26_bslow_r1', 'c2n26_bslow_r2', 'c2n26_bslow_r3']
ARMS = [('BASELINE', BASE), ('C2-NAV.25', C25), ('C2-NAV.26', C26)]
GOAL = (-2.0, -3.0)


def trace(tag, leg='wall_adjacent'):
    p = os.path.join(RES, '%s_traces' % tag, '%s_rep0.csv' % leg)
    if not os.path.exists(p):
        return None
    out = []
    with open(p) as f:
        for r in csv.DictReader(f):
            out.append(r)
    return out


def f(r, k):
    v = r.get(k)
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def rec(tag):
    p = os.path.join(RES, tag + '.json')
    with open(p) as fh:
        d = json.load(fh)
    return {x['scenario']: x for x in d['legs']}


def main():
    print('=== wall_adjacent: how far PAST the goal did the robot ever '
          'get, in GROUND TRUTH? (+ve = past, toward the wall) ===')
    print('%-11s%-17s%11s%11s%11s%11s%11s'
          % ('arm', 'run', 'max_past', 'final_past', 'min_d_lethal',
             'term_yaw', 'status'))
    print('-' * 84)
    for arm, tags in ARMS:
        for tag in tags:
            tr = trace(tag)
            if not tr:
                continue
            ys = np.array([f(r, 'y') for r in tr
                           if f(r, 'y') is not None])
            past = -3.0 - ys           # +ve when y < -3.0
            R = rec(tag)['wall_adjacent']
            print('%-11s%-17s%11.3f%11.3f%11.3f%11.2f  %s'
                  % (arm, tag, past.max(), past[-1],
                     R.get('min_clearance_m'),
                     R.get('terminal_yaw_travel_rad') or 0.0,
                     R.get('status')))
    print()

    print('=== c2n26_slow_r2 / wall_adjacent: the last 5 s, 10 Hz '
          '(OBSERVED trace) ===')
    tr = trace('c2n26_slow_r2')
    print('%7s%9s%9s%9s%9s%9s%9s%9s  %-14s%s'
          % ('t_rel', 'past_m', 'v_nav', 'v_out', 'v_act', 'w_nav',
             'w_act', 'scan', 'cm_polygon', 'worst DWB critic'))
    for r in tr[-52:]:
        t = f(r, 't_rel')
        y = f(r, 'y')
        crit = 'RotateToGoal' if f(r, 'dwb_ill_rot') else (
            'BaseObstacle' if f(r, 'dwb_ill_base') else '-')
        print('%7.1f%9.3f%9.4f%9.4f%9.4f%9.3f%9.3f%9.3f  %-14s%s'
              % (t, -3.0 - y, f(r, 'v_nav') or 0, f(r, 'v_cmdvel') or 0,
                 f(r, 'v_act') or 0, f(r, 'w_nav') or 0,
                 f(r, 'w_act') or 0, f(r, 'scan_min') or 0,
                 r.get('cm_polygon') or 'none', crit))
    print()

    print('=== did the goal checker ever fire on this leg? ===')
    print('(searching the run log between the leg start and the abort)')
    lg = os.path.join(ROOT, 'logs', 'nav_c2n26_slow_r2.log')
    on = False
    for ln in open(lg, errors='replace'):
        if 'Begin navigating from current location (-0.01, -2.07)' in ln:
            on = True
        if on and ('Reached the goal' in ln or 'Start occupied' in ln
                   or 'Goal succeeded' in ln):
            print('   ' + ln.strip()[:150])
            if 'Start occupied' in ln:
                break
    print()

    print('=== terminal-phase shape of wall_adjacent, by arm ===')
    print('%-11s%-17s%10s%10s%10s%10s%10s'
          % ('arm', 'run', 't_term_s', 'term_yaw', 'term_v_med',
             'n_plans', 'dur_s'))
    for arm, tags in ARMS:
        for tag in tags:
            try:
                R = rec(tag)['wall_adjacent']
            except (KeyError, FileNotFoundError):
                continue
            print('%-11s%-17s%10s%10s%10s%10s%10s'
                  % (arm, tag, R.get('t_terminal_s'),
                     R.get('terminal_yaw_travel_rad'),
                     R.get('terminal_v_med'), R.get('n_plans'),
                     R.get('duration_sim_s')))
    return 0


if __name__ == '__main__':
    sys.exit(main())
