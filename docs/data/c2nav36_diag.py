#!/usr/bin/env python3
"""C2-NAV.36 / C2-NAV.39 -- offline analysis for the coco_nav_diag AmclNode
diagnostic capture, paired with the c2nav36_gt_sidecar.py ground-truth CSV.

Investigation-support tooling.  Nothing here starts a simulator or changes a
parameter.  It reads a JSONL diagnostic capture and a GT CSV and computes,
per consumed motion update,

    (odometry input AMCL actually consumed) - (ground truth)

Run every mode from the WORKTREE ROOT (`-P` keeps this script's own
directory off sys.path -- the numbers.py/trace.py shadowing trap in
CLAUDE.md):

    python3 -P docs/data/c2nav36_diag.py selftest
    python3 -P docs/data/c2nav36_diag.py schema <diag.jsonl>
    python3 -P docs/data/c2nav36_diag.py join <diag.jsonl> <gt.csv> \\
        [--out rows.csv] [--summary-json summary.json] [--max-gt-gap 0.1]

WHAT `join` COMPUTES
---------------------
For each laserReceived() cycle whose motion_delta event has
anchor_valid=true and motion_update_invoked=true, the recorded odom-frame
delta is resolved into a body-frame along-track/lateral/yaw triple using the
ANCHOR's own odom yaw, and differenced against the same triple computed from
ground truth LINEARLY INTERPOLATED between the two real GT samples
bracketing the anchor time and the current scan time -- never a
nearest-receipt-time substitution (CODEX_REVIEW.md SS35.5 item 4).

VALIDATION BEFORE ANY NUMBER (C2-NAV.39)
-----------------------------------------
Capture-level checks.  Any failure makes the status INVALID and no residual
is reported:
  - schema 2: the session header is first and appears once, the footer is
    last and appears once, the footer's counts agree with the file, and it
    reports zero dropped events and zero write failures
  - no duplicate (type, update_index); per event type, in FILE order,
    update_index and scan stamp strictly increase and the node clock never
    goes backwards
  - one base frame and one odom frame across the whole capture
  - GT acquisition stamps strictly increase in FILE order (checked before
    anything sorts them), one (frame_id, child_frame_id) pair throughout,
    and the GT body frame is AMCL's base frame
  - GT <gt.csv>.meta.json, when present: zero dropped rows, a row count
    that matches the CSV, and a clean shutdown
Row-level checks.  A failure makes that one update UNOBSERVABLE with the
reason, so it can neither pass silently nor vanish:
  - the paired odom_tf_lookup exists, succeeded, carries the same scan
    stamp, returned a pose stamped AT the scan time, and holds exactly the
    pose the motion model consumed
  - the anchor precedes the update in index and stamp, and the anchor pose
    recorded with the delta is the pose recorded at the anchor update
  - delta == pose - anchor, and (rot1, trans, rot2) is the differential-model
    split of that delta and recomposes to it (the split itself is checked
    against the INSTALLED nav2_amcl model by coco_nav_diag's
    test_motion_decomposition_oracle)
  - ground truth brackets both endpoints within --max-gt-gap seconds

Status and exit code: OK 0, UNOBSERVABLE 2, INVALID 3, LEGACY_UNVERIFIED 4.
LEGACY_UNVERIFIED is a schema-1 capture (C2-NAV.37, no footer) or a GT CSV
without a meta file: every check that CAN run did run, but drop accounting
cannot be verified from the files, so it is never reported as a plain OK.
"""

import argparse
import csv
import importlib.util
import json
import math
import os
import sys
from collections import Counter

HERE = os.path.dirname(os.path.abspath(__file__))

NS = 1_000_000_000
MAX_GT_GAP_S = 0.1          # ~5x the measured 48-50 Hz GT period (C2-NAV.37)
POSE_TOL = 2e-9             # two quanta of the recorder's %.9f serialization
DECOMP_TOL = 1e-6
TRANS_CUTOFF = 0.01         # DifferentialMotionModel's rot1 cutoff, metres

SESSION_START = 'diag_session_start'
SESSION_END = 'diag_session_end'
EVENT_TYPES = ('odom_tf_lookup', 'motion_delta')
EXIT_CODES = {'OK': 0, 'UNOBSERVABLE': 2, 'INVALID': 3, 'LEGACY_UNVERIFIED': 4}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def angle_diff(a, b):
    """nav2_amcl::angleutils::angle_diff, statement for statement."""
    a = wrap(a)
    b = wrap(b)
    d1 = a - b
    d2 = 2 * math.pi - abs(d1)
    if d1 > 0:
        d2 *= -1.0
    return d1 if abs(d1) < abs(d2) else d2


def decompose(dx, dy, dyaw, anchor_yaw, below_cutoff=None):
    """(rot1, trans, rot2) exactly as computeDifferentialDecomposition() splits a delta."""
    trans = math.sqrt(dx * dx + dy * dy)
    below = trans < TRANS_CUTOFF if below_cutoff is None else below_cutoff
    rot1 = 0.0 if below else angle_diff(math.atan2(dy, dx), anchor_yaw)
    return rot1, trans, angle_diff(dyaw, rot1)


def parse_stamp(sec, nanosec):
    return sec + nanosec * 1e-9


def stamp_ns(sec, nanosec):
    return int(sec) * NS + int(nanosec)


# ---------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------

def load_diag_jsonl(path):
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
    """GT samples in FILE order -- ordering is validated before anything sorts
    them.  Keyed on stamp_sec/stamp_nanosec (acquisition time), never on
    recv_wall_* (receipt time)."""
    rows = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            sec, nsec = int(row['stamp_sec']), int(row['stamp_nanosec'])
            rows.append({
                't': parse_stamp(sec, nsec), 'ns': stamp_ns(sec, nsec),
                'x': float(row['x']), 'y': float(row['y']), 'yaw': float(row['yaw']),
                'frame_id': row.get('frame_id', ''),
                'child_frame_id': row.get('child_frame_id', ''),
            })
    return rows


def load_gt_meta(gt_path):
    path = gt_path + '.meta.json'
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return json.load(f)


# ---------------------------------------------------------------------
# GT interpolation -- bracket only, never extrapolate
# ---------------------------------------------------------------------

def interpolate_gt(gt_rows, t):
    """Linearly interpolate x/y/yaw at time t between the two real GT
    samples bracketing it (gt_rows sorted by time).  Returns None if t falls
    outside the capture; on a hit also returns the bracket width ('gap')."""
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
    """Rotates a world/odom-frame (dx, dy) into the along-track/lateral frame
    defined by anchor_yaw, matching docs/data/c2nav34_odom.py's body_delta."""
    c, s = math.cos(anchor_yaw), math.sin(anchor_yaw)
    return (c * dx + s * dy, -s * dx + c * dy, wrap(dyaw))


# ---------------------------------------------------------------------
# Capture-level validation
# ---------------------------------------------------------------------

def _backwards(values):
    return sum(1 for a, b in zip(values, values[1:]) if b < a)


def _not_increasing(values):
    return sum(1 for a, b in zip(values, values[1:]) if b <= a)


def validate_capture(records):
    """Integrity of a diag JSONL.  Returns (schema, failures, notes)."""
    failures, notes = [], []
    versions = sorted({r['schema_version'] for r in records if 'schema_version' in r})
    if len(versions) > 1:
        failures.append(f'mixed schema versions {versions}')
    schema = versions[-1] if versions else None
    types = [r.get('type') for r in records]
    events = [r for r in records if r.get('type') in EVENT_TYPES]
    starts = [i for i, t in enumerate(types) if t == SESSION_START]
    ends = [i for i, t in enumerate(types) if t == SESSION_END]

    if schema is not None and schema >= 2:
        if starts != [0]:
            failures.append('session header missing, repeated, or not first '
                            '(concatenated or truncated capture)')
        if not ends:
            failures.append('session footer missing: capture truncated or the '
                            'recorder was killed before stop()')
        elif ends != [len(records) - 1]:
            failures.append('session footer repeated or not the last record')
        else:
            foot = records[ends[0]]
            if foot.get('dropped'):
                failures.append(f"recorder dropped {foot['dropped']} events")
            if foot.get('write_failures'):
                failures.append(f"recorder reported {foot['write_failures']} write failures")
            if foot.get('recorded') != len(events) or foot.get('written') != len(events):
                failures.append(f"footer says recorded={foot.get('recorded')} "
                                f"written={foot.get('written')} but the file holds "
                                f'{len(events)} events')
            if events and foot.get('last_update_index') != events[-1]['update_index']:
                failures.append(f"footer last_update_index {foot.get('last_update_index')} "
                                f"is not the last event's {events[-1]['update_index']}")
    else:
        if len(starts) > 1:
            failures.append('multiple session headers (concatenated capture)')
        notes.append('schema-1 capture: no session footer, recorder drop '
                     'accounting cannot be verified from the file')

    identities = Counter((r['type'], r['update_index']) for r in events)
    dups = sorted(k for k, c in identities.items() if c > 1)
    if dups:
        failures.append(f'{len(dups)} duplicate record identities (type, update_index), '
                        f'first {dups[0]}')
    for kind in EVENT_TYPES:
        seq = [r for r in events if r['type'] == kind]
        n = _not_increasing([r['update_index'] for r in seq])
        if n:
            failures.append(f'{kind}: update_index not strictly increasing in file order ({n}x)')
        scans = [stamp_ns(r['scan_stamp_sec'], r['scan_stamp_nanosec']) for r in seq]
        n = _backwards(scans)
        if n:
            failures.append(f'{kind}: scan stamp went backwards {n}x (clock discontinuity)')
        n = sum(1 for a, b in zip(scans, scans[1:]) if a == b)
        if n:
            failures.append(f'{kind}: {n} consecutive updates share a scan stamp '
                            '(duplicate scan identity)')
        nows = [stamp_ns(r['node_now_sec'], r['node_now_nanosec'])
                for r in seq if 'node_now_sec' in r]
        n = _backwards(nows)
        if n:
            failures.append(f'{kind}: node clock went backwards {n}x (clock discontinuity)')

    tf = [r for r in events if r['type'] == 'odom_tf_lookup']
    for field in ('base_frame_id', 'odom_frame_id'):
        seen = sorted({str(r.get(field)) for r in tf})
        if len(seen) > 1:
            failures.append(f'{field} changed within the capture: {seen}')
    return schema, failures, notes


def validate_gt(rows, meta, base_frame):
    """Integrity of a GT capture, rows in FILE order.  Returns (failures, notes)."""
    failures, notes = [], []
    stamps = [r['ns'] for r in rows]
    n = _backwards(stamps)
    if n:
        failures.append(f'GT acquisition stamp went backwards {n}x in file order '
                        '(clock discontinuity)')
    n = len(stamps) - len(set(stamps))
    if n:
        failures.append(f'{n} duplicate GT acquisition stamps')
    pairs = sorted({(r['frame_id'], r['child_frame_id']) for r in rows})
    if len(pairs) > 1:
        failures.append(f'GT frame change within the capture: {pairs}')
    elif pairs and base_frame is not None and pairs[0][1] != base_frame:
        failures.append(f"GT body frame '{pairs[0][1]}' is not AMCL's base frame "
                        f"'{base_frame}' (model-to-base correspondence)")
    if meta is None:
        notes.append('GT capture has no .meta.json: row drops cannot be verified')
    else:
        if meta.get('rows_dropped'):
            failures.append(f"GT sidecar dropped {meta['rows_dropped']} rows")
        if meta.get('rows_written') != len(rows):
            failures.append(f"GT meta says {meta.get('rows_written')} rows, "
                            f'the CSV holds {len(rows)}')
        if not meta.get('clean_shutdown'):
            failures.append('GT sidecar did not shut down cleanly')
    return failures, notes


# ---------------------------------------------------------------------
# Row-level checks and the join
# ---------------------------------------------------------------------

def _close(a, b, tol=POSE_TOL):
    return abs(a - b) <= tol


def _yaw_close(a, b, tol=POSE_TOL):
    return abs(wrap(a - b)) <= tol


def index_by_update(records):
    by_update = {}
    for r in records:
        t = r.get('type')
        if t in EVENT_TYPES:
            by_update.setdefault(r['update_index'], {})[t] = r
    return by_update


def _event_of(bundle):
    return bundle.get('motion_delta') or bundle.get('odom_tf_lookup')


def check_pairing(bundle, md):
    tf = bundle.get('odom_tf_lookup')
    if tf is None:
        return 'no paired odom_tf_lookup for this update'
    if not tf.get('lookup_success'):
        return 'paired odom_tf_lookup failed'
    scan = stamp_ns(md['scan_stamp_sec'], md['scan_stamp_nanosec'])
    if stamp_ns(tf['scan_stamp_sec'], tf['scan_stamp_nanosec']) != scan:
        return 'paired odom_tf_lookup has a different scan stamp'
    if stamp_ns(tf['odom_pose_stamp_sec'], tf['odom_pose_stamp_nanosec']) != scan:
        return 'TF returned a pose stamped away from the scan time'
    if not (_close(tf['odom_x'], md['pose_x']) and _close(tf['odom_y'], md['pose_y'])
            and _yaw_close(tf['odom_yaw'], md['pose_yaw'])):
        return 'paired TF pose differs from the pose AMCL consumed'
    return None


def check_anchor(by_update, md, idx, cur_ns):
    anchor_idx = md['anchor_update_index']
    if anchor_idx >= idx:
        return 'anchor update_index does not precede this update'
    bundle = by_update.get(anchor_idx)
    if bundle is None:
        return f'anchor update_index {anchor_idx} not in capture'
    ev = _event_of(bundle)
    if ev is None:
        return f'anchor update_index {anchor_idx} has no timestamped event'
    if stamp_ns(ev['scan_stamp_sec'], ev['scan_stamp_nanosec']) > cur_ns:
        return 'anchor scan stamp is after this update'
    if 'pose_x' in ev:
        ax, ay, ayaw = ev['pose_x'], ev['pose_y'], ev['pose_yaw']
    else:
        ax, ay, ayaw = ev['odom_x'], ev['odom_y'], ev['odom_yaw']
    if not (_close(ax, md['anchor_x']) and _close(ay, md['anchor_y'])
            and _yaw_close(ayaw, md['anchor_yaw'])):
        return 'anchor pose differs from the pose recorded at the anchor update'
    return None


def check_delta(md):
    tol = 3 * POSE_TOL
    ok = (_close(md['delta_x'], md['pose_x'] - md['anchor_x'], tol)
          and _close(md['delta_y'], md['pose_y'] - md['anchor_y'], tol)
          and _yaw_close(md['delta_yaw'], angle_diff(md['pose_yaw'], md['anchor_yaw']), tol))
    return None if ok else 'recorded delta is not pose - anchor'


def check_decomposition(md):
    if not md.get('motion_model_formula_applicable'):
        return None
    dx, dy, dyaw, ayaw = md['delta_x'], md['delta_y'], md['delta_yaw'], md['anchor_yaw']
    rot1, trans, rot2 = md['delta_rot1'], md['delta_trans'], md['delta_rot2']
    # Nine-decimal serialization can move a delta across the 1 cm cutoff;
    # right at the cutoff either branch is accepted.
    near_cutoff = abs(math.hypot(dx, dy) - TRANS_CUTOFF) < 1e-8
    branches = (True, False) if near_cutoff else (None,)

    def matches(branch):
        r1, tr, r2 = decompose(dx, dy, dyaw, ayaw, branch)
        return (abs(tr - trans) <= DECOMP_TOL and abs(angle_diff(r1, rot1)) <= DECOMP_TOL
                and abs(angle_diff(r2, rot2)) <= DECOMP_TOL)

    if not any(matches(b) for b in branches):
        return 'recorded (rot1, trans, rot2) is not the differential-model split of the delta'
    if trans >= TRANS_CUTOFF:
        if (abs(trans * math.cos(ayaw + rot1) - dx) > DECOMP_TOL
                or abs(trans * math.sin(ayaw + rot1) - dy) > DECOMP_TOL
                or abs(angle_diff(rot1 + rot2, dyaw)) > DECOMP_TOL):
            return 'decomposition does not recompose to the recorded delta'
    return None


def join_records(diag_records, gt_rows, max_gt_gap=MAX_GT_GAP_S):
    """gt_rows sorted by acquisition time.  Returns (correlated, unobservable,
    candidate_count); candidate_count is every consumed motion update and is
    the denominator of the blindness guard."""
    by_update = index_by_update(diag_records)
    correlated = []
    unobservable = []
    candidate_count = 0

    for idx in sorted(by_update):
        bundle = by_update[idx]
        md = bundle.get('motion_delta')
        if md is None:
            continue  # TF lookup failed this cycle: no consumed delta
        if not md.get('anchor_valid') or not md.get('motion_update_invoked'):
            continue  # anchor-init or below-threshold cycle
        candidate_count += 1

        cur_ns = stamp_ns(md['scan_stamp_sec'], md['scan_stamp_nanosec'])
        reason = (check_pairing(bundle, md) or check_anchor(by_update, md, idx, cur_ns)
                  or check_delta(md) or check_decomposition(md))
        if reason:
            unobservable.append((idx, reason))
            continue

        anchor_idx = md['anchor_update_index']
        anchor_ev = _event_of(by_update[anchor_idx])
        anchor_t = parse_stamp(anchor_ev['scan_stamp_sec'], anchor_ev['scan_stamp_nanosec'])
        cur_t = parse_stamp(md['scan_stamp_sec'], md['scan_stamp_nanosec'])
        gt_cur = interpolate_gt(gt_rows, cur_t)
        gt_anchor = interpolate_gt(gt_rows, anchor_t)
        if gt_cur is None or gt_anchor is None:
            unobservable.append((idx, 'GT capture does not bracket this update in time'))
            continue
        if max(gt_cur['gap'], gt_anchor['gap']) > max_gt_gap:
            unobservable.append((idx, f'GT bracket wider than {max_gt_gap:g} s'))
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
    """The blindness guard: UNOBSERVABLE when nothing could be correlated,
    never a silent zero-bias finding."""
    n = len(correlated)
    reasons = Counter(reason for _, reason in unobservable)
    if candidate_count == 0:
        return {
            'status': 'UNOBSERVABLE', 'reason': 'no candidate motion updates in capture',
            'n_correlated': 0, 'n_candidates': 0, 'n_unobservable': 0, 'reasons': reasons,
        }
    if n == 0:
        return {
            'status': 'UNOBSERVABLE', 'reason': 'every candidate update was unobservable',
            'n_correlated': 0, 'n_candidates': candidate_count,
            'n_unobservable': len(unobservable), 'reasons': reasons,
        }
    return {
        'status': 'OK',
        'n_correlated': n, 'n_candidates': candidate_count,
        'n_unobservable': len(unobservable), 'reasons': reasons,
        'mean_residual_along': sum(r['residual_along'] for r in correlated) / n,
        'mean_residual_lat': sum(r['residual_lat'] for r in correlated) / n,
        'mean_residual_yaw': sum(r['residual_yaw'] for r in correlated) / n,
    }


def analyze(records, gt_rows_file_order, gt_meta, max_gt_gap=MAX_GT_GAP_S):
    """Validate, then join.  Returns (summary, correlated rows)."""
    schema, failures, notes = validate_capture(records)
    base_frames = {r.get('base_frame_id') for r in records if r.get('type') == 'odom_tf_lookup'}
    base_frame = next(iter(base_frames)) if len(base_frames) == 1 else None
    gt_failures, gt_notes = validate_gt(gt_rows_file_order, gt_meta, base_frame)
    failures += gt_failures
    notes += gt_notes

    gt_sorted = sorted(gt_rows_file_order, key=lambda r: r['ns'])
    correlated, unobservable, candidates = join_records(records, gt_sorted, max_gt_gap)
    summary = summarize(correlated, unobservable, candidates)
    summary.update(schema_version=schema, integrity_failures=failures,
                   integrity_notes=notes, max_gt_gap_s=max_gt_gap)
    if failures:
        summary['status'] = 'INVALID'
        for key in ('mean_residual_along', 'mean_residual_lat', 'mean_residual_yaw'):
            summary.pop(key, None)
        correlated = []
    elif summary['status'] == 'OK' and notes:
        summary['status'] = 'LEGACY_UNVERIFIED'
    return summary, correlated


# ---------------------------------------------------------------------
# Modes
# ---------------------------------------------------------------------

EXPECTED_FIELDS = {
    SESSION_START: {'schema_version', 'type', 'node_name', 'max_events', 'pid'},
    SESSION_END: {'schema_version', 'type', 'recorded', 'dropped', 'written',
                  'write_failures', 'last_update_index'},
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
        want = EXPECTED_FIELDS.get(r.get('type'))
        if want is None:
            continue
        got = set(r.keys())
        if got != want:
            mismatches += 1
            print(f"  schema mismatch in a {r.get('type')} record: "
                  f'missing={want - got} extra={got - want}')
    if mismatches == 0:
        print('  every record matches the expected schema')
        return 0
    print(f'  {mismatches} record(s) did not match')
    return 1


def mode_join():
    ap = argparse.ArgumentParser(prog=f'{sys.argv[0]} join')
    ap.add_argument('diag_jsonl')
    ap.add_argument('gt_csv')
    ap.add_argument('--out', help='write correlated rows as CSV here')
    ap.add_argument('--summary-json', help='write the summary as JSON here')
    ap.add_argument('--max-gt-gap', type=float, default=MAX_GT_GAP_S,
                    help='widest GT bracket accepted, seconds (default %(default)s)')
    args = ap.parse_args(sys.argv[2:])

    records = load_diag_jsonl(args.diag_jsonl)
    gt_rows = load_gt_csv(args.gt_csv)
    summary, correlated = analyze(records, gt_rows, load_gt_meta(args.gt_csv),
                                  args.max_gt_gap)

    print(f'status: {summary["status"]}')
    print(f'schema_version: {summary["schema_version"]}')
    for failure in summary['integrity_failures']:
        print(f'  INTEGRITY FAILURE: {failure}')
    for note in summary['integrity_notes']:
        print(f'  note: {note}')
    print(f'candidates (consumed motion updates): {summary["n_candidates"]}')
    print(f'correlated: {summary["n_correlated"]}')
    print(f'unobservable: {summary["n_unobservable"]}')
    for reason, count in summary['reasons'].most_common():
        print(f'    {count}x  {reason}')
    if 'mean_residual_along' in summary:
        print(f'mean residual along-track (odom input - GT): {summary["mean_residual_along"]:+.6f} m')
        print(f'mean residual lateral     (odom input - GT): {summary["mean_residual_lat"]:+.6f} m')
        print(f'mean residual yaw         (odom input - GT): {summary["mean_residual_yaw"]:+.6f} rad')

    if args.out and correlated:
        with open(args.out, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(correlated[0].keys()))
            w.writeheader()
            w.writerows(correlated)
        print(f'wrote {len(correlated)} correlated rows to {args.out}')
    if args.summary_json:
        with open(args.summary_json, 'w') as f:
            json.dump(dict(summary, reasons=dict(summary['reasons'])), f, indent=1)
        print(f'wrote summary to {args.summary_json}')

    return EXIT_CODES[summary['status']]


# ---------------------------------------------------------------------
# Selftest fixtures -- synthetic only, no capture, no simulator
# ---------------------------------------------------------------------

def _tf_event(idx, t_ns, pose, ok=True, schema=2):
    sec, nsec = divmod(int(t_ns), NS)
    x, y, yaw = pose if ok else (0.0, 0.0, 0.0)
    return {
        'schema_version': schema, 'type': 'odom_tf_lookup', 'update_index': idx,
        'scan_stamp_sec': sec, 'scan_stamp_nanosec': nsec,
        'base_frame_id': 'base_footprint', 'odom_frame_id': 'odom',
        'lookup_success': ok, 'odom_x': x, 'odom_y': y, 'odom_yaw': yaw,
        'odom_qx': 0.0, 'odom_qy': 0.0, 'odom_qz': math.sin(yaw / 2), 'odom_qw': math.cos(yaw / 2),
        'odom_pose_stamp_sec': sec if ok else 0, 'odom_pose_stamp_nanosec': nsec if ok else 0,
        'error_message': '' if ok else 'extrapolation', 'consecutive_failures': 0 if ok else 1,
        'node_now_sec': sec, 'node_now_nanosec': nsec,
    }


def _md_event(idx, t_ns, pose, anchor=None, anchor_idx=0, schema=2):
    sec, nsec = divmod(int(t_ns), NS)
    x, y, yaw = pose
    consumed = anchor is not None
    rec = {
        'schema_version': schema, 'type': 'motion_delta', 'update_index': idx,
        'scan_stamp_sec': sec, 'scan_stamp_nanosec': nsec,
        'is_anchor_init': not consumed, 'anchor_valid': consumed,
        'anchor_update_index': anchor_idx if consumed else 0,
        'anchor_x': 0.0, 'anchor_y': 0.0, 'anchor_yaw': 0.0,
        'pose_x': x, 'pose_y': y, 'pose_yaw': yaw,
        'delta_x': 0.0, 'delta_y': 0.0, 'delta_yaw': 0.0,
        'motion_model_formula_applicable': consumed,
        'delta_rot1': 0.0, 'delta_trans': 0.0, 'delta_rot2': 0.0,
        'motion_model_type': 'nav2_amcl::DifferentialMotionModel',
        'should_update_filter': consumed, 'motion_update_invoked': consumed,
        'node_now_sec': sec, 'node_now_nanosec': nsec,
    }
    if consumed:
        ax, ay, ayaw = anchor
        dx, dy, dyaw = x - ax, y - ay, angle_diff(yaw, ayaw)
        rot1, trans, rot2 = decompose(dx, dy, dyaw, ayaw)
        rec.update(anchor_x=ax, anchor_y=ay, anchor_yaw=ayaw, delta_x=dx, delta_y=dy,
                   delta_yaw=dyaw, delta_rot1=rot1, delta_trans=trans, delta_rot2=rot2)
    return rec


def _session(events, dropped=0, write_failures=0):
    head = {'schema_version': 2, 'type': SESSION_START, 'node_name': '/amcl',
            'max_events': 1000, 'pid': 1}
    foot = {'schema_version': 2, 'type': SESSION_END, 'recorded': len(events),
            'dropped': dropped, 'written': len(events), 'write_failures': write_failures,
            'last_update_index': events[-1]['update_index'] if events else 0}
    return [head] + list(events) + [foot]


def _clean_events(injected_along_error=0.05, schema=2):
    # update 1: anchor-init at t=1 s; update 2: consumes a +x delta of
    # 1.0 + error at t=2 s while GT moves exactly 1.0 m; update 3: TF fails.
    p1 = (0.0, 0.0, 0.0)
    p2 = (1.0 + injected_along_error, 0.0, 0.0)
    return [
        _tf_event(1, 1 * NS, p1, schema=schema), _md_event(1, 1 * NS, p1, schema=schema),
        _tf_event(2, 2 * NS, p2, schema=schema),
        _md_event(2, 2 * NS, p2, anchor=p1, anchor_idx=1, schema=schema),
        _tf_event(3, 3 * NS, p1, ok=False, schema=schema),
    ]


def _gt_rows(t_start=0.5, t_end=3.5, step=0.05, hole=None,
             x_of=lambda t: min(max(t - 1.0, 0.0), 1.0)):
    rows = []
    for i in range(int(round((t_end - t_start) / step)) + 1):
        t_ns = int(round((t_start + i * step) * NS))
        t = parse_stamp(*divmod(t_ns, NS))
        if hole and hole[0] < t < hole[1]:
            continue
        rows.append({'t': t, 'ns': t_ns, 'x': x_of(t), 'y': 0.0, 'yaw': 0.0,
                     'frame_id': 'world', 'child_frame_id': 'base_footprint'})
    return rows


def _gt_meta(rows, **overrides):
    meta = {'schema_version': 1, 'rows_written': len(rows), 'rows_dropped': 0,
            'clean_shutdown': True}
    meta.update(overrides)
    return meta


def _load_sidecar():
    spec = importlib.util.spec_from_file_location(
        'c2nav36_gt_sidecar', os.path.join(HERE, 'c2nav36_gt_sidecar.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def mode_selftest():
    n = fail = 0

    def chk(name, cond):
        nonlocal n, fail
        n += 1
        if not cond:
            fail += 1
            print(f'  FAILED: {name}')

    # --- interpolation, rotation, the model's own angle arithmetic
    gt3 = [{'t': 10.0, 'x': 0.0, 'y': 0.0, 'yaw': 0.0},
           {'t': 11.0, 'x': 1.0, 'y': 0.0, 'yaw': 0.0},
           {'t': 12.0, 'x': 1.0, 'y': 2.0, 'yaw': math.pi / 2}]
    exact = interpolate_gt(gt3, 11.0)
    chk('exact-sample interpolation returns the sample',
        exact is not None and abs(exact['x'] - 1.0) < 1e-12 and abs(exact['y']) < 1e-12)
    mid = interpolate_gt(gt3, 10.5)
    chk('midpoint interpolation is linear', mid is not None and abs(mid['x'] - 0.5) < 1e-12)
    chk('before first sample is None (no extrapolation)', interpolate_gt(gt3, 9.999) is None)
    chk('after last sample is None (no extrapolation)', interpolate_gt(gt3, 12.001) is None)
    chk('empty GT is None', interpolate_gt([], 10.0) is None)
    along, lat, _ = body_frame_delta(1.0, 0.0, 0.0, math.pi / 2)
    chk('rotation into anchor frame: +x at 90 deg is lateral -1',
        abs(along) < 1e-9 and abs(lat + 1.0) < 1e-9)
    chk('angle_diff takes the short way across pi',
        abs(angle_diff(3.0, -3.0) - (6.0 - 2 * math.pi)) < 1e-12)
    chk('below the 1 cm cutoff rot1 is exactly zero', decompose(0.004, 0.003, 0.2, 1.0)[0] == 0.0)
    r1, tr, _ = decompose(-0.2, 0.0, 0.0, 0.0)
    chk('reverse motion is rot1 = pi with positive trans',
        abs(abs(r1) - math.pi) < 1e-12 and abs(tr - 0.2) < 1e-12)

    def run(records, rows, meta, max_gap=MAX_GT_GAP_S):
        return analyze(records, rows, meta, max_gap)[0]

    gt = _gt_rows()
    clean = _session(_clean_events())

    # --- a clean capture, and the injected residual comes back exactly
    s = run(clean, gt, _gt_meta(gt))
    chk(f'clean schema-2 capture is OK (got {s["status"]} {s["integrity_failures"]})',
        s['status'] == 'OK')
    chk('exactly one candidate (the TF-failure cycle is excluded)', s['n_candidates'] == 1)
    chk('exactly one correlated row', s['n_correlated'] == 1)
    chk('injected +0.05 m along-track residual recovered exactly',
        abs(s.get('mean_residual_along', 1.0) - 0.05) < 1e-9)
    chk('lateral and yaw residuals ~ 0',
        abs(s.get('mean_residual_lat', 1.0)) < 1e-9 and abs(s.get('mean_residual_yaw', 1.0)) < 1e-9)

    # --- legacy captures are analyzable but never a plain OK
    s = run(_clean_events(schema=1), gt, None)
    chk(f'schema-1 capture is LEGACY_UNVERIFIED (got {s["status"]})',
        s['status'] == 'LEGACY_UNVERIFIED')
    chk('legacy capture still recovers the injected residual',
        abs(s.get('mean_residual_along', 1.0) - 0.05) < 1e-9)
    s = run(clean, gt, None)
    chk('schema-2 capture with a GT CSV lacking meta is LEGACY_UNVERIFIED',
        s['status'] == 'LEGACY_UNVERIFIED')

    # --- capture-level failures: INVALID, and no residual is reported
    def invalid(name, records, rows=None, meta='default', expect=''):
        rows = gt if rows is None else rows
        meta = _gt_meta(rows) if meta == 'default' else meta
        s = run(records, rows, meta)
        chk(f'INVALID: {name} (got {s["status"]} {s["integrity_failures"]})',
            s['status'] == 'INVALID' and 'mean_residual_along' not in s
            and any(expect in f for f in s['integrity_failures']))

    ev = _clean_events()
    invalid('footer missing', _session(ev)[:-1], expect='footer missing')
    invalid('recorder dropped events', _session(ev, dropped=3), expect='dropped 3')
    invalid('recorder write failures', _session(ev, write_failures=1), expect='write failures')
    tampered = _session(ev)
    tampered[-1] = dict(tampered[-1], recorded=4, written=4)
    invalid('footer counts disagree with the file', tampered, expect='footer says')
    invalid('concatenated capture', _session(ev) + _session(ev), expect='header')
    invalid('records after the footer', _session(ev) + [ev[4]], expect='not the last')
    invalid('duplicate record identity', _session(ev[:2] + ev[:2] + ev[2:]),
            expect='duplicate record identities')
    invalid('update_index out of order', _session([ev[2], ev[3], ev[0], ev[1], ev[4]]),
            expect='not strictly increasing')
    back = list(ev)
    back[4] = dict(back[4], scan_stamp_sec=0, scan_stamp_nanosec=500000000)
    invalid('scan stamp goes backwards', _session(back), expect='scan stamp went backwards')
    clock = list(ev)
    clock[4] = dict(clock[4], node_now_sec=0)
    invalid('node clock goes backwards', _session(clock), expect='node clock went backwards')
    same = list(ev)
    same[4] = dict(same[4], scan_stamp_sec=2, scan_stamp_nanosec=0)
    invalid('two updates share one scan stamp', _session(same), expect='share a scan stamp')
    frames = list(ev)
    frames[4] = dict(frames[4], base_frame_id='coco/base_link')
    invalid('base frame changes mid-capture', _session(frames), expect='base_frame_id changed')

    g_back = list(gt)
    g_back[10], g_back[11] = g_back[11], g_back[10]
    invalid('GT stamp goes backwards in file order', clean, g_back,
            expect='GT acquisition stamp went backwards')
    invalid('duplicate GT stamp', clean, gt[:20] + [dict(gt[19])] + gt[20:],
            expect='duplicate GT')
    invalid('GT frame changes', clean,
            [dict(r, frame_id='odom') if i > 30 else r for i, r in enumerate(gt)],
            expect='GT frame change')
    invalid('GT body frame is not the AMCL base frame', clean,
            [dict(r, child_frame_id='coco/chassis') for r in gt], expect='correspondence')
    invalid('GT sidecar dropped rows', clean, gt, _gt_meta(gt, rows_dropped=5), expect='dropped 5')
    invalid('GT meta row count mismatch', clean, gt, _gt_meta(gt, rows_written=len(gt) + 1),
            expect='the CSV holds')
    invalid('GT sidecar unclean shutdown', clean, gt, _gt_meta(gt, clean_shutdown=False),
            expect='cleanly')

    # --- row-level failures: the update is unobservable, with its reason
    def rowcase(name, events, expect, rows=None):
        rows = gt if rows is None else rows
        s = run(_session(events), rows, _gt_meta(rows))
        reasons = list(s['reasons'])
        chk(f'row rejected: {name} (got {s["status"]} {reasons} {s["integrity_failures"]})',
            s['status'] == 'UNOBSERVABLE' and any(expect in r for r in reasons))

    e = _clean_events()
    e[2] = dict(e[2], odom_x=e[2]['odom_x'] + 0.01)
    rowcase('paired TF pose differs', e, 'differs from the pose AMCL consumed')
    e = _clean_events()
    e[2] = dict(e[2], odom_pose_stamp_nanosec=100)
    rowcase('TF stamp differs from scan stamp', e, 'stamped away from the scan')
    e = _clean_events()
    del e[2]
    rowcase('no paired TF lookup', e, 'no paired odom_tf_lookup')
    e = _clean_events()
    e[3] = _md_event(2, 2 * NS, (1.05, 0.0, 0.0), anchor=(0.2, 0.0, 0.0), anchor_idx=1)
    rowcase('anchor pose is not the anchor-update pose', e, 'anchor pose differs')
    e = _clean_events()
    e[3] = dict(e[3], anchor_update_index=2)
    rowcase('anchor does not precede the update', e, 'does not precede')
    e = _clean_events()
    e[3] = dict(e[3], delta_x=e[3]['delta_x'] + 0.001)
    rowcase('delta is not pose - anchor', e, 'not pose - anchor')
    e = _clean_events()
    e[3] = dict(e[3], delta_rot2=e[3]['delta_rot2'] + 0.01)
    rowcase('decomposition is not the differential split', e, 'differential-model split')
    rowcase('GT bracket wider than the limit', _clean_events(), 'GT bracket wider',
            rows=_gt_rows(hole=(1.8, 2.3)))

    q1, q2 = (0.0, 0.0, 0.3), (0.3 * math.cos(0.3), 0.3 * math.sin(0.3), 0.5)
    turn = [_tf_event(1, NS, q1), _md_event(1, NS, q1),
            _tf_event(2, 2 * NS, q2), _md_event(2, 2 * NS, q2, anchor=q1, anchor_idx=1)]
    s = run(_session(turn), gt, _gt_meta(gt))
    chk(f'a turning update passes every row-level check (got {s["status"]} {dict(s["reasons"])})',
        s['status'] == 'OK')

    # --- blindness guard
    far = _gt_rows(t_start=1000.0, t_end=1001.0)
    s = run(clean, far, _gt_meta(far))
    chk('GT that never brackets an update is UNOBSERVABLE, not a clean finding',
        s['status'] == 'UNOBSERVABLE' and s['n_candidates'] == 1)
    s = run([], gt, _gt_meta(gt))
    chk('empty capture is UNOBSERVABLE, not vacuously OK', s['status'] == 'UNOBSERVABLE')

    # --- chained anchors resolve through each update's own anchor
    p1, p2, p3 = (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)
    chain = [_tf_event(1, NS, p1), _md_event(1, NS, p1),
             _tf_event(2, 2 * NS, p2), _md_event(2, 2 * NS, p2, anchor=p1, anchor_idx=1),
             _tf_event(3, 3 * NS, p3), _md_event(3, 3 * NS, p3, anchor=p2, anchor_idx=2)]
    rows = _gt_rows(t_end=4.0, x_of=lambda t: max(t - 1.0, 0.0))
    s, corr = analyze(_session(chain), rows, _gt_meta(rows))
    chk(f'chained anchors: both consuming updates resolve (got {s["status"]})',
        s['status'] == 'OK' and len(corr) == 2)
    chk('chained anchors: update 3 anchors on update 2',
        len(corr) == 2 and corr[1]['anchor_update_index'] == 2)

    # --- the GT sidecar's own accounting
    sidecar = _load_sidecar()
    stats = sidecar.CaptureStats('/model/coco/odometry', max_rows=3)
    for sec, nsec in ((1, 0), (1, 20000000), (1, 20000000), (1, 10000000)):
        if stats.accept():
            stats.observe(sec, nsec, 'world', 'base_footprint')
    meta = stats.to_dict(clean_shutdown=True)
    chk(f'sidecar meta counts rows, drops and duplicate stamps (got {meta})',
        meta['rows_written'] == 3 and meta['rows_dropped'] == 1
        and meta['duplicate_stamps'] == 1 and meta['stamp_regressions'] == 0
        and meta['frame_pairs'] == [['world', 'base_footprint']]
        and meta['clean_shutdown'] is True)
    row = sidecar.format_row(1, 2, 3, 4, 'world', 'base_footprint',
                             0.1, 0.2, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0)
    chk('sidecar row format matches its CSV header',
        len(row) == len(sidecar.CSV_HEADER) and row[6] == '0.100000000')

    print(f'\n{n - fail} passed, {fail} FAILED')
    return 1 if fail else 0


MODES = {'selftest': mode_selftest, 'schema': mode_schema, 'join': mode_join}

if __name__ == '__main__':
    if len(sys.argv) < 2 or sys.argv[1] not in MODES:
        sys.exit(f'usage: {sys.argv[0]} {{{"|".join(MODES)}}} [args...]')
    sys.exit(MODES[sys.argv[1]]() or 0)
