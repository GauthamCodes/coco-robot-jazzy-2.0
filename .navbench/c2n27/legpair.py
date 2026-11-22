#!/usr/bin/env python3
"""C2-NAV.27 -- pair every leg's AMCL pose against ground truth, all arms.

OFFLINE ONLY.

`bt_navigator` prints "Begin navigating from current location (x, y)" at
the start of every leg. That x,y is the AMCL estimate in the MAP frame --
the only recorded AMCL pose in these artifacts (nav_bench subscribes
/amcl_pose but never writes it to a record field or a trace column, as
C2-NAV.26's header says). The leg record's `end_world` is GROUND TRUTH.

Leg N's "Begin navigating" fires ~1.1-1.3 s after leg N-1 stopped, and
the trace shows the robot motionless across that gap, so pairing leg N's
AMCL start against leg N-1's ground-truth END measures the localisation
error AT THE MOMENT THE PLANNER WAS ASKED. That pairing is RECONSTRUCTED,
not observed; the two numbers it pairs are each OBSERVED.

Then run each of those AMCL poses through the SmacPlanner2D start test
(geom.verdict) and ask: would the planner have refused?
"""
import os
import re
import sys
import json

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import geom  # noqa: E402

ROOT = os.path.abspath(os.path.join(HERE, '..'))
LOGS = os.path.join(ROOT, 'logs')
RES = os.path.join(ROOT, 'results')

BEGIN = re.compile(
    r'\[(\d+\.\d+)\].*\[bt_navigator\]: Begin navigating from current '
    r'location \(([-\d.]+), ([-\d.]+)\)')
BEGIN_TP = re.compile(
    r'\[(\d+\.\d+)\].*\[bt_navigator\]: Begin navigating from current '
    r'location through \d+ poses')
REACHED = re.compile(
    r'\[(\d+\.\d+)\].*\[controller_server\]: Reached the goal!')
REFUSE = re.compile(
    r'\[(\d+\.\d+)\].*\[planner_server\]: GridBased plugin failed to plan '
    r'from \(([-\d.]+), ([-\d.]+)\) to \(([-\d.]+), ([-\d.]+)\): "([^"]+)"')

LEGS = ['open_space', 'wall_adjacent', 'wall_parallel', 'obstacle_corner',
        'corridor_gate', 'enclosure_entry', 'enclosure_exit']

BASE_A = ['c2n21_base_r1', 'c2n21_base_r3', 'c2n21_base_r4']
BASE_B = ['c2n21_bbase_r2', 'c2n21_bbase_r3']
C25 = ['c2n25_slow_r1', 'c2n25_slow_r2', 'c2n25_bslow_r1']
C26 = ['c2n26_slow_r1', 'c2n26_slow_r2', 'c2n26_slow_r3',
       'c2n26_bslow_r1', 'c2n26_bslow_r2', 'c2n26_bslow_r3']
ARMS = [('BASELINE', BASE_A + BASE_B), ('C2-NAV.25', C25), ('C2-NAV.26', C26)]


def record(tag):
    p = os.path.join(RES, tag + '.json')
    if not os.path.exists(p):
        return None
    with open(p) as f:
        return json.load(f)


def log_events(tag):
    p = os.path.join(LOGS, 'nav_%s.log' % tag)
    if not os.path.exists(p):
        return None
    ev = []
    for ln in open(p, errors='replace'):
        m = BEGIN.search(ln)
        if m:
            ev.append(('begin', float(m.group(1)),
                       float(m.group(2)), float(m.group(3))))
            continue
        m = BEGIN_TP.search(ln)
        if m:
            ev.append(('begin_tp', float(m.group(1)), None, None))
            continue
        m = REACHED.search(ln)
        if m:
            ev.append(('reached', float(m.group(1)), None, None))
            continue
        m = REFUSE.search(ln)
        if m:
            ev.append(('refuse', float(m.group(1)), float(m.group(2)),
                       float(m.group(3)), m.group(6)))
    return ev


def main():
    print('=== leg-by-leg: ground-truth END of leg N-1 vs AMCL START of '
          'leg N, and the planner verdict at that AMCL pose ===')
    print('map = world + (2.0, 0.0). "refuse?" replays the SmacPlanner2D')
    print('start test offline on the STATIC map (a LOWER bound on live '
          'cost).')
    print()
    rows = []
    for arm, tags in ARMS:
        print('---------- %s ----------' % arm)
        print('%-17s%-17s%9s%9s%9s%9s%9s  %s'
              % ('run', 'prev leg', 'gt_endy', 'amcl_y', 'err_xy',
                 'd_cell', 'cost', 'refuse?'))
        for tag in tags:
            rec = record(tag)
            ev = log_events(tag)
            if not rec or ev is None:
                print('   %-16s (artifacts missing)' % tag)
                continue
            legs = {r['scenario']: r for r in rec['legs']}
            begins = [e for e in ev if e[0] in ('begin', 'begin_tp')]
            # begins[i] starts LEGS[i]; pair begins[i] with LEGS[i-1] end
            for i in range(1, min(len(begins), len(LEGS))):
                b = begins[i]
                prev = LEGS[i - 1]
                if b[0] == 'begin_tp' or prev not in legs:
                    continue
                end = legs[prev].get('end_world')
                if not end:
                    continue
                ax, ay = b[2], b[3]                 # AMCL, map frame
                gx, gy = end[0] + 2.0, end[1]       # ground truth, map frame
                err = ((ax - gx) ** 2 + (ay - gy) ** 2) ** 0.5
                v = geom.verdict(ax, ay)
                rows.append(dict(arm=arm, tag=tag, prev=prev, err=err,
                                 d_cell=v['d_cell'], cost=v['cost'],
                                 refuse=v['occupied'],
                                 status=legs[prev]['status'],
                                 gt_end=end, amcl=(ax, ay)))
                print('%-17s%-17s%9.3f%9.3f%9.3f%9.3f%9d  %s'
                      % (tag, prev, end[1], ay, err, v['d_cell'],
                         v['cost'], 'REFUSE' if v['occupied'] else '.'))
        print()

    print('=== localisation error at leg handover, by arm (OBSERVED '
          'pairing of two OBSERVED quantities) ===')
    for arm, _ in ARMS:
        e = sorted(r['err'] for r in rows if r['arm'] == arm)
        if not e:
            continue
        print('  %-12s n=%2d  median %.3f  p90 %.3f  max %.3f'
              % (arm, len(e), e[len(e) // 2], e[int(0.9 * (len(e) - 1))],
                 e[-1]))
    print()

    print('=== wall_adjacent specifically: where did each run finish, '
          'and how far was the estimate? ===')
    print('%-17s%-11s%10s%10s%10s%10s%9s  %s'
          % ('run', 'arm', 'gt_end_y', 'past goal', 'amcl_y', 'amcl err',
             'd_cell', 'verdict'))
    for r in rows:
        if r['prev'] != 'wall_adjacent':
            continue
        past = -3.0 - r['gt_end'][1]     # +ve = past the goal, toward wall
        print('%-17s%-11s%10.3f%10.3f%10.3f%10.3f%9.3f  %s'
              % (r['tag'], r['arm'], r['gt_end'][1], past, r['amcl'][1],
                 r['err'], r['d_cell'],
                 'REFUSED' if r['refuse'] else 'plannable'))
    print()
    print('NOTE: a run whose wall_adjacent leg ABORTED has no following')
    print('"Begin navigating" for the next leg in the normal position, so')
    print('read the refusal rows from inv_startocc.py for those.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
