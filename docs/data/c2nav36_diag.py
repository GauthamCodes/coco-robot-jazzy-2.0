#!/usr/bin/env python3
"""C2-NAV.36 -- offline analysis for the coco_nav_diag AmclNode diagnostic
capture (paired with the c2nav36_gt_sidecar.py ground-truth CSV).

Investigation-support tooling.  Nothing here starts a simulator, changes a
parameter, or touches `main`.  It reads two already-produced files (a JSONL
diagnostic capture and a GT CSV) and answers, for the first time from the
ACTUAL consumed odometry input rather than the historical `/amcl_pose`-vs-GT
PROXY (see docs/agents/HANDOFF.md C2-NAV.35 SS5-SS6):

    (odometry input AMCL actually consumed) - (ground truth), per update

Run every mode from the WORKTREE ROOT (`-P` keeps this script's own
directory off sys.path -- the numbers.py/trace.py shadowing trap in
CLAUDE.md):

    python3 -P docs/data/c2nav36_diag.py selftest         # 0 external inputs
    python3 -P docs/data/c2nav36_diag.py schema <diag.jsonl>
    python3 -P docs/data/c2nav36_diag.py join <diag.jsonl> <gt.csv> [--out csv]

DATA PROVENANCE
----------------
Neither <diag.jsonl> nor <gt.csv> exists in this repository as of this
session.  They are produced by a FUTURE, separately authorized live run:
`amcl_diag` (coco_nav_diag) with `diag_enabled:=true` alongside
`c2nav36_gt_sidecar.py` capturing `/model/coco/odometry` in parallel.  This
file's `join` mode is implemented and unit-tested (`selftest`) against
synthetic fixtures, but has not been run against real data -- there is
none yet.  See docs/agents/HANDOFF.md for exactly what remains unknown.

WHAT `join` COMPUTES
---------------------
For each laserReceived() cycle whose motion_delta event has
anchor_valid=true and motion_update_invoked=true (i.e. the model actually
consumed a delta this cycle -- see coco_nav_diag/diag_recorder.hpp), the
recorded odom-frame delta is resolved into a body-frame along-track/
lateral/yaw triple using the ANCHOR's own odom yaw, and differenced
against the equivalent triple computed from ground truth, using GT samples
LINEARLY INTERPOLATED between the two real samples bracketing the anchor
time and the current scan time -- never a nearest-receipt-time
substitution (CODEX_REVIEW.md SS35.5 item 4).  An update whose anchor or
current time falls outside the GT capture's time range, or whose anchor
event is missing from the capture, is UNOBSERVABLE and is reported as
such, never silently folded into a "clean" verdict -- the same trap
CLAUDE.md and C2-NAV.33 already paid for once ("any check whose success
condition is 'we saw nothing' must first prove it can see something").
"""

import argparse
import csv
import json
import math
import os
import sys
from collections import Counter

HERE = os.path.dirname(os.path.abspath(__file__))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def parse_stamp(sec, nanosec):
    return sec + nanosec * 1e-9


# ---------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------

def load_diag_jsonl(path):
    """Returns a list of dict records, one per JSONL line.  Tolerates a
    leading diag_session_start line (not an odom_tf_lookup/motion_delta
    event)."""
    records = []
    with open(path) as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError as e:
                sys.exit(f'{path}:{lineno}: malformed JSONL: {e}')
    return records


def load_gt_csv(path):
    """Returns GT samples sorted by acquisition-stamp time (seconds,
    float).  Uses stamp_sec/stamp_nanosec (header.stamp / acquisition
    time), never recv_wall_* (receipt time) -- see c2nav36_gt_sidecar.py's
    module docstring for why that distinction matters here."""
    rows = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            rows.append({
                't': parse_stamp(int(row['stamp_sec']), int(row['stamp_nanosec'])),
                'x': float(row['x']), 'y': float(row['y']), 'yaw': float(row['yaw']),
            })
    rows.sort(key=lambda r: r['t'])
    return rows


# ---------------------------------------------------------------------
# GT interpolation -- bracket only, never extrapolate
# ---------------------------------------------------------------------

def interpolate_gt(gt_rows, t):
    """Linearly interpolate x/y/yaw at time t between the two real GT
    samples bracketing it.  Returns None (never a nearest-sample
    substitution) if gt_rows is empty or t falls outside
    [gt_rows[0]['t'], gt_rows[-1]['t']].  On a hit, also returns the
    bracket width ('gap') so a caller can flag a suspiciously wide
    bracket rather than trusting an arbitrarily stale interpolation."""
    if not gt_rows or t < gt_rows[0]['t'] or t > gt_rows[-1]['t']:
        return None
    lo, hi = 0, len(gt_rows) - 1
    while lo < hi - 1:
        mid = (lo + hi) // 2
        if gt_rows[mid]['t'] <= t:
            lo = mid
        else:
            hi = mid
    a, b = gt_rows[lo], gt_rows[hi]
    gap = b['t'] - a['t']
    frac = 0.0 if gap <= 0 else (t - a['t']) / gap
    x = a['x'] + frac * (b['x'] - a['x'])
    y = a['y'] + frac * (b['y'] - a['y'])
    yaw = wrap(a['yaw'] + frac * wrap(b['yaw'] - a['yaw']))
    return {'x': x, 'y': y, 'yaw': yaw, 'gap': gap, 'frac': frac}


def body_frame_delta(dx, dy, dyaw, anchor_yaw):
    """Rotates a world/odom-frame (dx, dy) into the along-track/lateral
    frame defined by anchor_yaw, matching docs/data/c2nav34_odom.py's
    body_delta convention exactly (needed so C2-NAV.36 results are
    directly comparable to the historical C2-NAV.28/.34 residuals)."""
    c, s = math.cos(anchor_yaw), math.sin(anchor_yaw)
    return (c * dx + s * dy, -s * dx + c * dy, wrap(dyaw))


# ---------------------------------------------------------------------
# Join
# ---------------------------------------------------------------------

def index_by_update(records):
    by_update = {}
    for r in records:
        t = r.get('type')
        if t in ('odom_tf_lookup', 'motion_delta'):
            by_update.setdefault(r['update_index'], {})[t] = r
    return by_update


def _event_time(bundle):
    """The one recorded scan_stamp for an update_index's bundle -- present
    on whichever of odom_tf_lookup/motion_delta exists for it (they always
    agree when both are present, same laserReceived() cycle)."""
    ev = bundle.get('motion_delta') or bundle.get('odom_tf_lookup')
    if ev is None:
        return None
    return parse_stamp(ev['scan_stamp_sec'], ev['scan_stamp_nanosec'])


def join_records(diag_records, gt_rows):
    """Returns (correlated, unobservable, candidate_count).

    correlated: list of dicts, one per consumed motion update that could
    be matched against bracketing GT at both the anchor and current scan
    time -- along/lateral/yaw for both the consumed odometry input and GT,
    plus the signed residual (odom - GT) in each axis.

    unobservable: list of (update_index, reason) for every candidate that
    could NOT be resolved -- missing anchor event, GT not bracketing
    either endpoint, etc.  Never silently dropped.

    candidate_count: how many motion_delta events had
    anchor_valid=True and motion_update_invoked=True, i.e. how many
    cycles actually asked the motion model to consume a delta. The
    denominator for the blindness guard.
    """
    by_update = index_by_update(diag_records)
    correlated = []
    unobservable = []
    candidate_count = 0

    for idx in sorted(by_update):
        bundle = by_update[idx]
        md = bundle.get('motion_delta')
        if md is None:
            continue  # TF lookup failed this cycle, or no motion_delta event at all
        if not md.get('anchor_valid') or not md.get('motion_update_invoked'):
            continue  # anchor-init cycle, or below-threshold cycle: no consumed delta
        candidate_count += 1

        cur_t = parse_stamp(md['scan_stamp_sec'], md['scan_stamp_nanosec'])
        anchor_idx = md['anchor_update_index']
        anchor_bundle = by_update.get(anchor_idx)
        if anchor_bundle is None:
            unobservable.append((idx, f'anchor update_index {anchor_idx} not in capture'))
            continue
        anchor_t = _event_time(anchor_bundle)
        if anchor_t is None:
            unobservable.append((idx, f'anchor update_index {anchor_idx} has no timestamped event'))
            continue

        gt_cur = interpolate_gt(gt_rows, cur_t)
        gt_anchor = interpolate_gt(gt_rows, anchor_t)
        if gt_cur is None or gt_anchor is None:
            unobservable.append((idx, 'GT capture does not bracket this update in time'))
            continue

        od_along, od_lat, od_dyaw = body_frame_delta(
            md['delta_x'], md['delta_y'], md['delta_yaw'], md['anchor_yaw'])
        gt_along, gt_lat, gt_dyaw = body_frame_delta(
            gt_cur['x'] - gt_anchor['x'], gt_cur['y'] - gt_anchor['y'],
            wrap(gt_cur['yaw'] - gt_anchor['yaw']), gt_anchor['yaw'])

        correlated.append({
            'update_index': idx,
            'anchor_update_index': anchor_idx,
            'od_along': od_along, 'od_lat': od_lat, 'od_dyaw': od_dyaw,
            'gt_along': gt_along, 'gt_lat': gt_lat, 'gt_dyaw': gt_dyaw,
            'residual_along': od_along - gt_along,
            'residual_lat': od_lat - gt_lat,
            'residual_yaw': wrap(od_dyaw - gt_dyaw),
            'gt_anchor_gap': gt_anchor['gap'], 'gt_cur_gap': gt_cur['gap'],
        })

    return correlated, unobservable, candidate_count


def summarize(correlated, unobservable, candidate_count):
    """The blindness guard.  Returns a dict with an explicit 'status':
    'UNOBSERVABLE' when nothing could be correlated (candidate_count == 0
    or correlated is empty) -- reported as its own outcome, never
    silently presented as a zero-bias finding.  'OK' otherwise, still
    carrying the unobservable count/reasons for the report."""
    n = len(correlated)
    reasons = Counter(reason for _, reason in unobservable)
    if candidate_count == 0:
        return {
            'status': 'UNOBSERVABLE', 'reason': 'no candidate motion updates in capture',
            'n_correlated': 0, 'n_candidates': 0, 'n_unobservable': 0, 'reasons': reasons,
        }
    if n == 0:
        return {
            'status': 'UNOBSERVABLE',
            'reason': 'every candidate update was unobservable',
            'n_correlated': 0, 'n_candidates': candidate_count,
            'n_unobservable': len(unobservable), 'reasons': reasons,
        }
    mean_along = sum(r['residual_along'] for r in correlated) / n
    mean_lat = sum(r['residual_lat'] for r in correlated) / n
    mean_yaw = sum(r['residual_yaw'] for r in correlated) / n
    return {
        'status': 'OK',
        'n_correlated': n, 'n_candidates': candidate_count,
        'n_unobservable': len(unobservable), 'reasons': reasons,
        'mean_residual_along': mean_along,
        'mean_residual_lat': mean_lat,
        'mean_residual_yaw': mean_yaw,
    }


# ---------------------------------------------------------------------
# Modes
# ---------------------------------------------------------------------

EXPECTED_FIELDS = {
    'odom_tf_lookup': {
        'schema_version', 'type', 'update_index', 'scan_stamp_sec',
        'scan_stamp_nanosec', 'base_frame_id', 'odom_frame_id',
        'lookup_success', 'odom_x', 'odom_y', 'odom_yaw', 'odom_qx',
        'odom_qy', 'odom_qz', 'odom_qw', 'odom_pose_stamp_sec',
        'odom_pose_stamp_nanosec', 'error_message', 'consecutive_failures',
        'node_now_sec', 'node_now_nanosec',
    },
    'motion_delta': {
        'schema_version', 'type', 'update_index', 'scan_stamp_sec',
        'scan_stamp_nanosec', 'is_anchor_init', 'anchor_valid',
        'anchor_update_index', 'anchor_x', 'anchor_y', 'anchor_yaw',
        'pose_x', 'pose_y', 'pose_yaw', 'delta_x', 'delta_y', 'delta_yaw',
        'motion_model_formula_applicable', 'delta_rot1', 'delta_trans',
        'delta_rot2', 'motion_model_type', 'should_update_filter',
        'motion_update_invoked', 'node_now_sec', 'node_now_nanosec',
    },
}


def mode_schema():
    if len(sys.argv) != 3:
        sys.exit(f'usage: {sys.argv[0]} schema <diag.jsonl>')
    records = load_diag_jsonl(sys.argv[2])
    counts = Counter(r.get('type') for r in records)
    print(f'{len(records)} records: {dict(counts)}')
    mismatches = 0
    for r in records:
        t = r.get('type')
        if t not in EXPECTED_FIELDS:
            continue
        got = set(r.keys())
        want = EXPECTED_FIELDS[t]
        if got != want:
            mismatches += 1
            print(f'  schema mismatch in a {t} record: missing={want - got} extra={got - want}')
    if mismatches == 0:
        print('  all odom_tf_lookup / motion_delta records match the expected schema')
        return 0
    print(f'  {mismatches} record(s) did not match')
    return 1


def mode_join():
    ap = argparse.ArgumentParser(prog=f'{sys.argv[0]} join')
    ap.add_argument('diag_jsonl')
    ap.add_argument('gt_csv')
    ap.add_argument('--out', help='optional: write correlated rows as CSV here')
    args = ap.parse_args(sys.argv[2:])

    records = load_diag_jsonl(args.diag_jsonl)
    gt_rows = load_gt_csv(args.gt_csv)
    correlated, unobservable, candidate_count = join_records(records, gt_rows)
    summary = summarize(correlated, unobservable, candidate_count)

    print(f'status: {summary["status"]}')
    print(f'candidates (consumed motion updates): {summary["n_candidates"]}')
    print(f'correlated: {summary["n_correlated"]}')
    print(f'unobservable: {summary["n_unobservable"]}')
    if summary.get('reasons'):
        for reason, count in summary['reasons'].most_common():
            print(f'    {count}x  {reason}')
    if summary['status'] == 'OK':
        print(f'mean residual along-track (odom input - GT): {summary["mean_residual_along"]:+.6f} m')
        print(f'mean residual lateral     (odom input - GT): {summary["mean_residual_lat"]:+.6f} m')
        print(f'mean residual yaw         (odom input - GT): {summary["mean_residual_yaw"]:+.6f} rad')

    if args.out and correlated:
        with open(args.out, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(correlated[0].keys()))
            w.writeheader()
            w.writerows(correlated)
        print(f'wrote {len(correlated)} correlated rows to {args.out}')

    return 0 if summary['status'] == 'OK' else 2


def mode_selftest():
    """Synthetic fixtures only -- no capture, no simulator, no repo data."""
    n = fail = 0

    def chk(name, cond):
        nonlocal n, fail
        n += 1
        if not cond:
            fail += 1
            print(f'  FAILED: {name}')

    # --- interpolate_gt ---
    gt = [
        {'t': 10.0, 'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        {'t': 11.0, 'x': 1.0, 'y': 0.0, 'yaw': 0.0},
        {'t': 12.0, 'x': 1.0, 'y': 2.0, 'yaw': math.pi / 2},
    ]
    exact = interpolate_gt(gt, 11.0)
    exact_ok = (
        exact is not None
        and abs(exact['x'] - 1.0) < 1e-12
        and abs(exact['y'] - 0.0) < 1e-12
    )
    chk('exact-sample interpolation returns the sample', exact_ok)
    mid = interpolate_gt(gt, 10.5)
    chk('midpoint interpolation is linear', mid is not None and abs(mid['x'] - 0.5) < 1e-12)
    chk('before first sample is None (no extrapolation)', interpolate_gt(gt, 9.999) is None)
    chk('after last sample is None (no extrapolation)', interpolate_gt(gt, 12.001) is None)
    chk('empty GT is None', interpolate_gt([], 10.0) is None)

    # --- body_frame_delta matches c2nav34_odom.py's body_delta convention:
    # a pure-x world delta rotated by a 90-degree anchor yaw becomes a pure
    # lateral (not along-track) body delta.
    along, lat, dyaw = body_frame_delta(1.0, 0.0, 0.0, math.pi / 2)
    chk('rotation into anchor frame: along ~ 0', abs(along) < 1e-9)
    chk('rotation into anchor frame: lateral ~ -1', abs(lat - (-1.0)) < 1e-9)

    # --- join_records: a clean two-update chain with a KNOWN injected
    # along-track discrepancy between the consumed odometry input and GT,
    # exactly the "synthetic injection recovered exactly" pattern
    # docs/data/c2nav34_odom.py's own selftest uses.
    def synth_capture(injected_along_error):
        # update_index 1: anchor-init at t=0, pose=(0,0,0)
        # update_index 2: consumes delta (anchor=idx1 -> pose=(1+err,0,0)),
        #   i.e. the recorded odom delta is 1.0 + injected_along_error
        #   while GT genuinely moves exactly 1.0 m along +x.
        return [
            {'type': 'diag_session_start', 'node_name': 'amcl', 'max_events': 100, 'pid': 1},
            {
                'type': 'odom_tf_lookup', 'update_index': 1, 'scan_stamp_sec': 0,
                'scan_stamp_nanosec': 0, 'lookup_success': True,
                'odom_x': 0.0, 'odom_y': 0.0, 'odom_yaw': 0.0,
            },
            {
                'type': 'motion_delta', 'update_index': 1, 'scan_stamp_sec': 0,
                'scan_stamp_nanosec': 0, 'is_anchor_init': True, 'anchor_valid': False,
                'anchor_update_index': 0, 'pose_x': 0.0, 'pose_y': 0.0, 'pose_yaw': 0.0,
                'should_update_filter': False, 'motion_update_invoked': False,
            },
            {
                'type': 'odom_tf_lookup', 'update_index': 2, 'scan_stamp_sec': 1,
                'scan_stamp_nanosec': 0, 'lookup_success': True,
                'odom_x': 1.0 + injected_along_error, 'odom_y': 0.0, 'odom_yaw': 0.0,
            },
            {
                'type': 'motion_delta', 'update_index': 2, 'scan_stamp_sec': 1,
                'scan_stamp_nanosec': 0, 'is_anchor_init': False, 'anchor_valid': True,
                'anchor_update_index': 1, 'anchor_x': 0.0, 'anchor_y': 0.0, 'anchor_yaw': 0.0,
                'pose_x': 1.0 + injected_along_error, 'pose_y': 0.0, 'pose_yaw': 0.0,
                'delta_x': 1.0 + injected_along_error, 'delta_y': 0.0, 'delta_yaw': 0.0,
                'should_update_filter': True, 'motion_update_invoked': True,
            },
            # update_index 3: TF lookup FAILS this cycle -- no motion_delta
            # event at all. Must be excluded from the candidate set, never
            # silently treated as a zero-delta consumed update.
            {
                'type': 'odom_tf_lookup', 'update_index': 3, 'scan_stamp_sec': 2,
                'scan_stamp_nanosec': 0, 'lookup_success': False,
                'error_message': 'extrapolation', 'consecutive_failures': 1,
            },
        ]

    gt_clean = [
        {'t': 0.0, 'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        {'t': 1.0, 'x': 1.0, 'y': 0.0, 'yaw': 0.0},
    ]
    records = synth_capture(injected_along_error=0.05)
    correlated, unobservable, candidates = join_records(records, gt_clean)
    chk('exactly one candidate (the TF-failure cycle is excluded)', candidates == 1)
    chk('exactly one correlated row', len(correlated) == 1)
    if correlated:
        chk(
            f'injected +0.05 m along-track residual recovered exactly '
            f'(got {correlated[0]["residual_along"]:+.9f})',
            abs(correlated[0]['residual_along'] - 0.05) < 1e-9)
        chk('lateral residual ~ 0', abs(correlated[0]['residual_lat']) < 1e-9)
        chk('yaw residual ~ 0', abs(correlated[0]['residual_yaw']) < 1e-9)
    summary = summarize(correlated, unobservable, candidates)
    chk('summary status OK on a clean overlapping capture', summary['status'] == 'OK')

    # --- blindness guard: GT capture entirely outside the diag capture's
    # time range must report UNOBSERVABLE, never a false "zero bias" OK.
    gt_far_away = [
        {'t': 1000.0, 'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        {'t': 1001.0, 'x': 0.0, 'y': 0.0, 'yaw': 0.0},
    ]
    correlated2, unobservable2, candidates2 = join_records(records, gt_far_away)
    chk('candidates still counted even when GT is blind', candidates2 == 1)
    chk('nothing correlated when GT never brackets any update', len(correlated2) == 0)
    summary2 = summarize(correlated2, unobservable2, candidates2)
    chk('blindness guard reports UNOBSERVABLE, not a clean finding', summary2['status'] == 'UNOBSERVABLE')

    # --- an empty capture (no candidates at all) is also UNOBSERVABLE,
    # not vacuously OK.
    summary3 = summarize(*join_records([], gt_clean))
    chk('empty capture is UNOBSERVABLE, not vacuously OK', summary3['status'] == 'UNOBSERVABLE')

    # --- a chained anchor (three updates) resolves anchor_update_index
    # transitively: update 3's anchor is update 2, whose own anchor was
    # update 1.
    chained = [
        {
            'type': 'motion_delta', 'update_index': 1, 'scan_stamp_sec': 0,
            'scan_stamp_nanosec': 0, 'is_anchor_init': True, 'anchor_valid': False,
            'anchor_update_index': 0, 'pose_x': 0.0, 'pose_y': 0.0, 'pose_yaw': 0.0,
            'should_update_filter': False, 'motion_update_invoked': False,
        },
        {
            'type': 'motion_delta', 'update_index': 2, 'scan_stamp_sec': 1,
            'scan_stamp_nanosec': 0, 'is_anchor_init': False, 'anchor_valid': True,
            'anchor_update_index': 1, 'anchor_x': 0.0, 'anchor_y': 0.0, 'anchor_yaw': 0.0,
            'pose_x': 1.0, 'pose_y': 0.0, 'pose_yaw': 0.0,
            'delta_x': 1.0, 'delta_y': 0.0, 'delta_yaw': 0.0,
            'should_update_filter': True, 'motion_update_invoked': True,
        },
        {
            'type': 'motion_delta', 'update_index': 3, 'scan_stamp_sec': 2,
            'scan_stamp_nanosec': 0, 'is_anchor_init': False, 'anchor_valid': True,
            'anchor_update_index': 2, 'anchor_x': 1.0, 'anchor_y': 0.0, 'anchor_yaw': 0.0,
            'pose_x': 2.0, 'pose_y': 0.0, 'pose_yaw': 0.0,
            'delta_x': 1.0, 'delta_y': 0.0, 'delta_yaw': 0.0,
            'should_update_filter': True, 'motion_update_invoked': True,
        },
    ]
    gt_chain = [
        {'t': 0.0, 'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        {'t': 1.0, 'x': 1.0, 'y': 0.0, 'yaw': 0.0},
        {'t': 2.0, 'x': 2.0, 'y': 0.0, 'yaw': 0.0},
    ]
    correlated4, unobservable4, candidates4 = join_records(chained, gt_chain)
    chk('chained anchors: both consuming updates resolve', len(correlated4) == 2)
    chk(
        'chained anchors: update 3 looked up anchor time via update 2, not update 1',
        correlated4[1]['anchor_update_index'] == 2)

    print(f'\n{n - fail} passed, {fail} FAILED')
    return 1 if fail else 0


MODES = {'selftest': mode_selftest, 'schema': mode_schema, 'join': mode_join}

if __name__ == '__main__':
    if len(sys.argv) < 2 or sys.argv[1] not in MODES:
        sys.exit(f'usage: {sys.argv[0]} {{{"|".join(MODES)}}} [args...]')
    sys.exit(MODES[sys.argv[1]]() or 0)
