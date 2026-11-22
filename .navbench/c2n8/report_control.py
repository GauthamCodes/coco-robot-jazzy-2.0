#!/usr/bin/env python3
"""C2-NAV.8 positive control for c2nav8_report.py's two new readings.

The central C2-NAV.8 claims are a COUNT THAT SHOULD BE ZERO (STOP frames
on the exit) and a DISTANCE THAT SHOULD BE LARGE (true clearance). The
repo's trap list is explicit about this shape: any check whose success
condition is "we saw nothing" must first prove it can see something.

So the report's seven-leg segmentation and its clearance computation are
pointed at C2-NAV.6's and C2-NAV.7's COMMITTED CSVs, where the answers
are already published, and asserted against them:

  C2-NAV.6 base r1 (min_points 4, goal -3.45), docs/data:
    enclosure_exit  1470 STOP frames, 6 returns inside the circle on
                    every one of them, true clearance 0.2437 m to
                    box_obstacle_1 -- the only real incursion in the
                    series, and the one nav_bench reported as 0.339 m.
  C2-NAV.7 cand r1..r3 (same params, goal -3.575):
    enclosure_exit  0 STOP frames, 0 inside the circle
    5325 frames across the three runs -- which is the sum of the two
    LABELLED legs (1630 + 1851 + 1844), not the CSV row count. The CSVs
    hold 5384 rows; the other 59 are recorded before the first goal is
    accepted, when /plan has no tail and a row cannot be attributed to
    any leg. Both are asserted here, because the difference is exactly
    the kind of off-by-a-segment that would make a per-leg STOP count
    look clean by dropping frames.

If the segmentation is wrong, the C2-NAV.6 assertions fail and the zero
is never trusted. Run from THIS directory.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.normpath(os.path.join(HERE, '..', '..', 'docs', 'data'))
sys.path.insert(0, DATA)
import c2nav8_report as R                                   # noqa: E402
from c2nav7_geom import nearest                             # noqa: E402

FAILED = []


def check(label, got, want, tol=None):
    ok = (abs(got - want) <= tol) if tol is not None else (got == want)
    print(f'{"ok  " if ok else "FAIL"}  {label}: got {got}, want {want}')
    if not ok:
        FAILED.append(label)


def rows_of(tag):
    return R._csv_rows({'tag': tag,
                        'stop_csv': os.path.join(DATA, f'{tag}_stop.csv')})


def seg(rows):
    out = {}
    for r in rows:
        out.setdefault(R._row_leg(r), []).append(r)
    return out


def main():
    print('=== C2-NAV.6 baseline: the report must SEE the stop ===')
    rows = rows_of('c2nav6_base_r1')
    check('c2nav6_base_r1 rows read', len(rows) > 0, True)
    s = seg(rows)
    ex = s.get('enclosure_exit', [])
    st = [r for r in ex if r['monitor_action'] == 'STOP']
    check('enclosure_exit segmented at all', len(ex) > 0, True)
    check('enclosure_exit STOP frames', len(st), 1470)
    ins = {int(r['n_in_stop']) for r in st}
    check('returns inside the circle on every STOP frame', ins, {6})
    pts = [(float(r['gt_x']), float(r['gt_y'])) for r in ex if r['gt_x']]
    d, who, _ = min(nearest(x, y)[0] for x, y in pts)[0], '', ''
    (d, who, _), _pt = min((nearest(x, y)[0], (x, y)) for x, y in pts)
    check('true minimum clearance on the exit (m)', round(d, 4), 0.2437,
          tol=0.0002)
    check('nearest box on the exit', who, 'box_obstacle_1')
    # C2-NAV.8 adds the rest of the world to the clearance list. On the
    # enclosure legs that must change NOTHING -- if it does, one of the
    # added shapes is wrong and every C2-NAV.7 comparison moves with it.
    (d2, who2, _), _p2 = min((R.nearest_full(x, y)[0], (x, y)) for x, y in pts)
    check('nearest_full agrees with C2-NAV.7 on the exit (m)', round(d2, 4),
          0.2437, tol=0.0002)
    check('nearest_full agrees on which obstacle', who2, 'box_obstacle_1')
    en = s.get('enclosure_entry', [])
    check('enclosure_entry also segmented', len(en) > 0, True)

    print()
    print('=== C2-NAV.7 candidate: the report must reproduce the ZERO ===')
    tot_rows = tot_stop = tot_labelled = 0
    for tag in ('c2nav7_cand_r1', 'c2nav7_cand_r2', 'c2nav7_cand_r3'):
        rows = rows_of(tag)
        s = seg(rows)
        ex = s.get('enclosure_exit', [])
        st = [r for r in ex if r['monitor_action'] == 'STOP']
        ins = max((int(r['n_in_stop']) for r in ex if r['n_in_stop'] != ''),
                  default=None)
        tot_rows += len(rows)
        tot_labelled += sum(len(v) for k, v in s.items() if k is not None)
        tot_stop += sum(1 for r in rows if r['monitor_action'] == 'STOP')
        print(f'  {tag}: {len(rows)} rows, exit {len(ex)} frames, '
              f'{len(st)} STOP, max in circle {ins}')
        check(f'{tag} exit STOP frames', len(st), 0)
        check(f'{tag} exit max returns inside the circle', ins, 0)
    check('C2-NAV.7 labelled frames (the committed 5325)', tot_labelled, 5325)
    check('C2-NAV.7 total CSV rows', tot_rows, 5384)
    check('unlabelled pre-goal rows', tot_rows - tot_labelled, 59)
    check('C2-NAV.7 total STOP frames', tot_stop, 0)

    print()
    print('=== segmentation covers the legs, not just two ===')
    rows = rows_of('c2nav7_cand_r1')
    names = {R._row_leg(r) for r in rows}
    print(f'  labels seen in a two-leg run: {sorted(str(n) for n in names)}')
    check('no row lands on a wrong leg',
          names <= {None, 'enclosure_entry', 'enclosure_exit'}, True)

    print(f'\n{len(FAILED)} failed')
    return 1 if FAILED else 0


if __name__ == '__main__':
    sys.exit(main())
