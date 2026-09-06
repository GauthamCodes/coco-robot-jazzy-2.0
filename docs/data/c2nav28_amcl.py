#!/usr/bin/env python3
"""C2-NAV.28 -- when does AMCL diverge, relative to the terminal yaw?

TWO PARTS, and this module is the instrument for both.

PART A is an INSTRUMENTATION change to `nav_bench.py::write_trace`: three
columns, `amcl_x` / `amcl_y` / `amcl_yaw`, appended to the 10 Hz per-leg
trace. Nothing else in that file moved. `selftest` is the gate on that
claim and runs entirely offline, against artifacts frozen before this
session.

PART B is FOUR fresh tours under the EXISTING configuration -- no
parameter was changed by this session, in either direction -- read by the
remaining commands.

THE QUESTION
------------
C2-NAV.27 established that "Start occupied" is a statement about the
ESTIMATED pose: at the clearest refusal, ground truth sat 0.259 m from
the nearest lethal cell and AMCL placed the robot 0.155 m further into
the wall, and 86 % of the estimated overshoot was the localisation term.
It could not say WHEN that 0.155 m appeared, because the only AMCL
evidence in the artifacts was a handful of two-decimal poses quoted
inside `bt_navigator` and `planner_server` log lines.

So the open question is temporal ordering:

    A  AMCL has already diverged before the terminal yaw behaviour
    B  the terminal yaw behaviour contributes to the divergence
    C  the two are independent
    D  something more specific couples them

WHAT THE NEW COLUMNS ARE, AND WHAT A BLANK MEANS
------------------------------------------------
`amcl_*` is the map-frame `/amcl_pose` estimate. It is the one column
group in the trace that is NOT zero-order held: a row carries the LAST
sample whose timestamp fell in that row's half-open 0.1 s bucket, and
three blanks when the bucket was empty. AMCL publishes on its own filter
schedule and stops entirely when the robot stops, so holding the value
forward would draw a flat line that looks like a filter still running.
`write_trace`'s docstring carries the full rule; `selftest` check 2
asserts it.

MEASURED IN THE FRESH RUNS: /amcl_pose arrives at 1-2 Hz, not 10 Hz,
with gaps up to ~2 s, and it stops entirely while the robot is stopped.
`avail` prints the per-leg cadence. So the trace column is SAMPLED at
10 Hz and POPULATED at the filter's own rate, and the honest phrasing of
what Part A delivers is "AMCL is recorded wherever AMCL published",
not "AMCL is recorded at 10 Hz".

Every headline number here is therefore measured ONLY on rows carrying a
fresh sample. Forward-filling and differencing against a moving ground
truth would book the robot's own odometry travel as localisation error,
because the effective estimate keeps moving between publications even
though `amcl_x` does not. `--fill` opts into the filled series for
context; `errors()` carries the full reasoning.

FRAMES
------
`x`, `y` are WORLD (ground truth, `/model/coco/odometry`). `amcl_x`,
`amcl_y` are MAP. map = world + (2.0, 0.0), the same constant
`nav_bench.py` applies to every goal it sends. Error is defined
est - truth in the MAP frame:

    amcl_error_x = amcl_x - (x + 2.0)
    amcl_error_y = amcl_y - (y + 0.0)
    amcl_error_norm = hypot(amcl_error_x, amcl_error_y)

so +y error means AMCL believes the robot is further NORTH than it is,
and at the wall_adjacent goal -- where the wall is to the SOUTH -- a
NEGATIVE y error is the one that pushes the estimate into the wall.

EVIDENCE CLASS, stated per number
---------------------------------
OBSERVED     every trace column including the new ones; every per-leg
             record field; every `planner_server` refusal line and
             `bt_navigator` "Begin navigating" line in the run logs.
DERIVED      the errors above, the divergence threshold (from the
             observed distribution, see `divergence`), the phase split
             (the record's own `t_transit_s`), and every correlation.
ALIGNED      run-log lines are stamped in SYSTEM time, the trace in SIM
             time. They are bridged per leg by anchoring on that leg's
             "Begin navigating" line and rescaling by the leg's own
             measured `duration_sim_s / duration_wall_s`, then CHECKED
             by matching the two-decimal pose the log line quotes against
             the trace's own `amcl_x`/`amcl_y`. Both numbers are printed;
             where they disagree the disagreement is printed too.
UNAVAILABLE  RotateToGoal's internal latch state. It is not published.
             What IS observed is `dwb_ill_rot` -- the count of candidate
             trajectories RotateToGoal rejected on that control cycle --
             which is non-zero exactly while the latch bans translation.
             That is a proxy and is labelled one everywhere.
             Also unavailable: /plan message times per row. Plan resets
             are inferred from `dwb_ill_rot` dropping to zero for one or
             more cycles mid-terminal, which is the signature C2-NAV.22
             established for `setPlan()` clearing the latch, and is
             labelled an inference.

WHAT THIS CANNOT SHOW
---------------------
Four runs. This is a TIME-SERIES question, not a rate question, and four
tours are enough to see the shape of a divergence and not nearly enough
to estimate how often one happens. No claim here is a frequency.

Correlation is not causation and the `ordering` verdict says only which
signal moved first. Nothing in this session changed a parameter, so
nothing here can attribute anything to one.
"""
import argparse
import bisect
import collections
import csv
import json
import math
import os
import re
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
WT = os.path.dirname(os.path.dirname(HERE))          # docs/data -> root
SCRATCH = os.path.join(WT, '.navbench', 'results')
LOGS = os.path.join(WT, '.navbench', 'logs')
NAV_BENCH = os.path.join(WT, 'gazebo_models', 'scripts', 'nav_bench.py')

WORLD_TO_MAP_X = 2.0
WORLD_TO_MAP_Y = 0.0

# The three appended columns, in order. Part A's whole schema change.
AMCL_COLS = ['amcl_x', 'amcl_y', 'amcl_yaw']

# The 28-column trace schema every C2-NAV.21 ... C2-NAV.27 artifact
# carries, read out of the FROZEN c2nav22_yaw.json bundle rather than
# retyped here -- see `_frozen_schema`.
FROZEN_BUNDLE = os.path.join(HERE, 'c2nav22_yaw.json')

# The commit this session STARTED from -- C2-NAV.27. Check 4 compares
# against this, not HEAD: once the instrumentation is committed a
# HEAD-relative diff is empty and the check passes for the wrong reason.
BASE = '88ecb05'

# C2-NAV.28's own four fresh runs.
RUNS_A = ['c2n28_a_r1', 'c2n28_a_r2']
RUNS_B = ['c2n28_b_r1']
RUNS_FOCUS = ['c2n28_focus_r1']
RUNS = RUNS_A + RUNS_B + RUNS_FOCUS

REFUSE = re.compile(
    r'\[(\d+\.\d+)\].*\[planner_server\]: GridBased plugin failed to plan '
    r'from \(([-\d.]+), ([-\d.]+)\) to \(([-\d.]+), ([-\d.]+)\): "([^"]+)"')
# BOTH forms, and the second one is load-bearing. A NavigateThroughPoses
# leg logs "Begin navigating from current location through N poses to
# (gx, gy)" and carries NO start pose, so a regex written only for the
# NavigateToPose form silently misses it. That is not cosmetic: the leg
# boundaries are found by ORDER, so one missed line shifts every later
# leg by one and books a refusal against the wrong scenario. It did
# exactly that here before this was fixed -- an enclosure refusal landed
# on corridor_gate at t~42.7 s of a 21.5 s leg, which is how it was
# caught.
BEGIN = re.compile(
    r'\[(\d+\.\d+)\].*\[bt_navigator\]: Begin navigating from current '
    r'location (?:through \d+ poses to|\(([-\d.]+), ([-\d.]+)\) to) '
    r'\(([-\d.]+), ([-\d.]+)\)')

# Forward-fill cap. A held AMCL sample older than this is treated as no
# sample at all rather than as an estimate. 1.0 s is ten trace rows and
# well over the filter's own worst observed gap while moving; `avail`
# prints the gap distribution so the cap can be checked against it.
FILL_CAP_S = 1.0


def hdr(t):
    print()
    print('=' * 78)
    print(t)
    print('=' * 78)


def fl(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def ang_norm(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def _git(*args):
    """git, always -C the worktree this file lives in."""
    return subprocess.run(('git', '-C', WT) + args,
                          capture_output=True, text=True)


# --------------------------------------------------------------- I/O
def _frozen_schema():
    """The 28-column pre-C2-NAV.28 schema, from the frozen bundle."""
    with open(FROZEN_BUNDLE) as f:
        b = json.load(f)
    got = {tuple(v['schema']) for v in b['traces'].values()}
    return list(max(got, key=len))


BUNDLE = os.path.join(HERE, 'c2nav28_amcl.json')
_bundle = None


def bundle():
    """The frozen C2-NAV.28 evidence, or None.

    `.navbench/` is scratch and is not committed, so every number in
    this session would become unreproducible the moment that directory
    is cleared. `dump` freezes what the analyses actually read -- the
    per-leg record fields, the FRESH AMCL samples with the ground truth
    alongside them, and the planner refusal lines -- into a committed
    JSON bundle, and every reader below falls back to it. Live scratch
    always wins when present, so a re-run is never masked by the bundle.
    """
    global _bundle
    if _bundle is None:
        if os.path.exists(BUNDLE):
            with open(BUNDLE) as f:
                _bundle = json.load(f)
        else:
            _bundle = {}
    return _bundle


def legs_of(tag):
    p = os.path.join(SCRATCH, tag + '.json')
    if os.path.exists(p):
        with open(p) as f:
            return {leg['scenario']: leg for leg in json.load(f)['legs']}
    b = bundle().get('legs', {}).get(tag)
    return dict(b) if b else {}


def trace_path(tag, leg, rep=0):
    return os.path.join(SCRATCH, '%s_traces' % tag,
                        '%s_rep%d.csv' % (leg, rep))


def read_trace(tag, leg, rep=0):
    p = trace_path(tag, leg, rep)
    if os.path.exists(p):
        with open(p) as f:
            return list(csv.DictReader(f))
    # Frozen fallback. The bundle stores only the rows that carried a
    # FRESH sample plus the signals the analyses classify on, so a
    # bundle-backed run reproduces every headline number (all of which
    # are fresh-only by construction) and CANNOT reproduce `--fill`,
    # which needs the rows in between. `avail` says so.
    rows = bundle().get('traces', {}).get(tag, {}).get(leg)
    return [dict(r) for r in rows] if rows else None


def has_amcl(rows):
    return bool(rows) and all(c in rows[0] for c in AMCL_COLS)


def fill_amcl(rows, cap_s=FILL_CAP_S):
    """Forward-fill the AMCL columns, EXPLICITLY and with a staleness cap.

    Returns (filled, n_fresh, n_filled, n_none). `filled[i]` is
    (x, y, yaw, age_s) or None. Nothing is interpolated: a filled row
    carries the previous sample verbatim plus how old it is.
    """
    out = []
    last = None
    last_t = None
    n_fresh = n_fill = n_none = 0
    for r in rows:
        t = fl(r.get('t_rel'))
        ax, ay, aw = (fl(r.get('amcl_x')), fl(r.get('amcl_y')),
                      fl(r.get('amcl_yaw')))
        if ax is not None and ay is not None and aw is not None:
            last, last_t = (ax, ay, aw), t
            out.append((ax, ay, aw, 0.0))
            n_fresh += 1
        elif last is not None and t is not None and last_t is not None \
                and (t - last_t) <= cap_s:
            out.append((last[0], last[1], last[2], round(t - last_t, 2)))
            n_fill += 1
        else:
            out.append(None)
            n_none += 1
    return out, n_fresh, n_fill, n_none


def errors(rows, filled, fresh_only=True):
    """(t, gt_xy_map, amcl_xy, ex, ey, norm, eyaw, age) per usable row.

    `fresh_only` defaults TRUE, and that default is load-bearing rather
    than conservative. `/amcl_pose` is a discrete filter output -- the
    fresh runs publish it at 1-2 Hz with gaps up to ~2 s, and it stops
    entirely when the robot stops. The estimate the PLANNER sees does not
    stop: map->odom is held while odometry keeps moving under it, so the
    effective estimate continues to travel between publications. Holding
    `amcl_x` flat across a gap and differencing it against a moving
    ground truth therefore books the robot's own odometry travel as
    localisation error -- an artefact that grows with speed and would
    have manufactured exactly the "divergence during motion" signal this
    session is trying to test for.

    So every headline number here is measured ONLY on rows carrying a
    fresh sample (age 0.0). `--fill` opts into the forward-filled series
    for context, and `age` is returned either way so a filled row can
    always be told from a fresh one.
    """
    out = []
    for r, a in zip(rows, filled):
        if a is None:
            continue
        if fresh_only and a[3] != 0.0:
            continue
        t, x, y, yaw = (fl(r.get('t_rel')), fl(r.get('x')), fl(r.get('y')),
                        fl(r.get('yaw')))
        if None in (t, x, y):
            continue
        gx, gy = x + WORLD_TO_MAP_X, y + WORLD_TO_MAP_Y
        ex, ey = a[0] - gx, a[1] - gy
        eyaw = ang_norm(a[2] - yaw) if yaw is not None else None
        out.append((t, (gx, gy), (a[0], a[1]), ex, ey,
                    math.hypot(ex, ey), eyaw, a[3]))
    return out


def rtf_of(leg):
    ds, dw = leg.get('duration_sim_s'), leg.get('duration_wall_s')
    if not ds or not dw:
        return None
    return ds / dw


# ------------------------------------------------------ 1. selftest
def _fake_trace(amcl_samples, t0=100.0, t1=100.5, gt_hz=20.0):
    """Run the REAL write_trace against a synthetic node. Returns rows."""
    import importlib.util
    spec = importlib.util.spec_from_file_location('nb_under_test', NAV_BENCH)
    mod = importlib.util.module_from_spec(spec)
    # nav_bench imports rclpy at module scope; if that is unavailable the
    # selftest says so rather than silently skipping the check.
    spec.loader.exec_module(mod)

    class S:
        def __init__(self, pairs):
            self.t = [p[0] for p in pairs]
            self.v = [p[1] for p in pairs]

        def window(self, a, b):
            i = bisect.bisect_left(self.t, a)
            j = bisect.bisect_right(self.t, b)
            return self.t[i:j], self.v[i:j]

    class FakeLock:
        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

    n = int((t1 - t0) * gt_hz) + 1
    # NOT named `gt`: a class body does not close over an enclosing
    # function's locals, so `gt = S(gt)` below would resolve the RHS in
    # the module globals and raise NameError.
    gt_rows = [(t0 + i / gt_hz, (0.1 * i, 0.2 * i, 0.3, 0.4, 0.5))
               for i in range(n)]

    class FakeNode:
        _lock = FakeLock()
        gt = S(gt_rows)
        amcl = S(amcl_samples)
        nav = S([])
        smooth = S([])
        out = S([])
        wheel = S([])
        cmstate = S([])
        scanmin = S([])
        evals = S([])

    fd, path = tempfile.mkstemp(suffix='.csv')
    os.close(fd)
    try:
        mod.write_trace(FakeNode(), path, t0, t1)
        with open(path) as f:
            return list(csv.DictReader(f))
    finally:
        os.unlink(path)


def _check(ok, label, detail=''):
    print('  %-4s %s%s' % ('OK' if ok else 'FAIL', label,
                           ('  -- ' + detail) if detail else ''))
    return 0 if ok else 1


def cmd_selftest(_args):
    """The Part A gate. Offline. No simulator, no ROS graph."""
    hdr('C2-NAV.28 selftest -- the instrumentation, before any run')
    bad = 0

    # ---- check 1: schema is the frozen schema plus exactly three cols
    print('1. trace schema')
    frozen = _frozen_schema()
    try:
        rows = _fake_trace([(100.05, (1.0, 2.0, 0.5))])
    except Exception as e:                       # noqa: BLE001
        print('  FAIL could not exercise write_trace: %r' % (e,))
        print('  (nav_bench.py imports rclpy at module scope; this check')
        print('   needs a sourced ROS 2 environment.)')
        return 1
    got = list(rows[0].keys())
    bad += _check(got[:len(frozen)] == frozen,
                  'the %d frozen columns are unchanged, in order'
                  % len(frozen),
                  'first difference at index %s' % next(
                      (i for i, (a, b) in enumerate(zip(got, frozen))
                       if a != b), 'none'))
    bad += _check(got[len(frozen):] == AMCL_COLS,
                  'exactly %s appended, and nothing else' % AMCL_COLS,
                  'got %r' % (got[len(frozen):],))
    bad += _check(len(got) == len(frozen) + 3,
                  'column count %d -> %d' % (len(frozen), len(got)))

    # ---- check 2: the missing-sample rule, stated and asserted
    print()
    print('2. AMCL sampling: bucket (t-0.1, t], no forward fill')
    # samples at t0+0.05 and t0+0.25 only. Rows are t0, t0+0.1 ... t0+0.5.
    rows = _fake_trace([(100.05, (1.0, 2.0, 0.5)),
                        (100.25, (3.0, 4.0, 0.6))])
    seq = [(r['t_rel'], r['amcl_x'], r['amcl_y'], r['amcl_yaw'])
           for r in rows]
    want = [('0.0', '', '', ''),
            ('0.1', '1.0', '2.0', '0.5'),
            ('0.2', '', '', ''),
            ('0.3', '3.0', '4.0', '0.6'),
            ('0.4', '', '', ''),
            ('0.5', '', '', '')]
    bad += _check(seq == want, 'each sample lands in exactly one row',
                  'got %r' % (seq,))
    bad += _check(all(s[1] == '' for s in seq if s[0] in ('0.2', '0.4')),
                  'a row with no sample is BLANK, not the previous value')
    # two samples in one bucket: the later one wins, the earlier is not
    # invented into a neighbouring row.
    rows = _fake_trace([(100.12, (1.0, 1.0, 0.1)),
                        (100.18, (2.0, 2.0, 0.2))])
    seq = [(r['t_rel'], r['amcl_x']) for r in rows]
    bad += _check(seq[2] == ('0.2', '2.0') and seq[1] == ('0.1', ''),
                  'a crowded bucket keeps the LATEST sample',
                  'got %r' % (seq[:4],))
    # no samples at all: three blanks on every row, no crash
    rows = _fake_trace([])
    bad += _check(all(r['amcl_x'] == '' and r['amcl_y'] == ''
                      and r['amcl_yaw'] == '' for r in rows),
                  'a leg with no /amcl_pose at all writes blanks')
    # The property that actually matters, asserted directly rather than
    # assumed: the buckets TILE. Row times accumulate as `t += 0.1` --
    # they always have, and C2-NAV.28 did not touch that -- so 100.2 is
    # really 100.19999999999999 and a sample landing within a float
    # epsilon of a boundary may fall on either side of it. What must
    # hold regardless is that a sample in its own bucket appears exactly
    # once, in the row at or just after its own timestamp.
    #
    # Spacing is 0.11 s -- wider than a bucket -- so no sample is
    # decimated by the crowded-bucket rule checked just above.
    samples = [(100.0 + 0.11 * k, (float(k), float(k), 0.01 * k))
               for k in range(1, 5)]
    rows = _fake_trace(samples, t0=100.0, t1=100.5)
    seen = [(fl(r['t_rel']), fl(r['amcl_x']))
            for r in rows if r['amcl_x']]
    want = sorted(s[1][0] for s in samples if 100.0 < s[0] <= 100.5)
    bad += _check(sorted(v for _, v in seen) == want,
                  'every sample in the window appears exactly once',
                  'want %r got %r' % (want, sorted(v for _, v in seen)))
    offs = []
    for t_row, xv in seen:
        src = next(s[0] for s in samples if s[1][0] == xv)
        offs.append((100.0 + t_row) - src)
    bad += _check(bool(offs) and all(-0.02 <= o < 0.12 for o in offs),
                  'each sample lands in the row at or just after its own '
                  'timestamp', 'offsets %r' % [round(o, 3) for o in offs])

    # ---- check 3: the existing analyses still reproduce
    print()
    print('3. frozen analyses reproduce (they read committed bundles, so')
    print('   this proves the instrumentation did not disturb them)')
    r = subprocess.run([sys.executable,
                        os.path.join(HERE, 'c2nav22_yaw.py'), 'selftest'],
                       capture_output=True, text=True)
    bad += _check(r.returncode == 0 and 'SELFTEST PASSED' in r.stdout,
                  'C2-NAV.22 selftest passes',
                  'rc=%d tail=%r' % (r.returncode, r.stdout[-120:]))
    r = subprocess.run([sys.executable,
                        os.path.join(HERE, 'c2nav24_chain.py'), 'stages'],
                       capture_output=True, text=True)
    mon = [ln for ln in r.stdout.splitlines()
           if ln.startswith('COLLISION MONITOR')]
    ok = bool(mon) and mon[0].split()[2:5] == ['3854', '0.301', '0.297']
    bad += _check(ok, 'C2-NAV.24 monitor stage still n=3854 median=0.301',
                  'got %r' % (mon[0] if mon else None,))

    # ---- check 4: nothing behavioural moved
    print()
    print('4. no behavioural configuration was modified')
    allow = {'gazebo_models/scripts/nav_bench.py',
             'docs/data/c2nav28_amcl.py',
             'docs/data/c2nav28_amcl.json',
             'docs/data/c2nav28_matrix.sh',
             'docs/SESSION_LOG.md'}
    changed = {ln for ln in _git('diff', '--name-only', BASE).stdout.split()
               if ln}
    print('  (vs %s, the C2-NAV.27 commit this session started from)'
          % BASE)
    bad += _check(bool(changed) and changed <= allow,
                  'every file this session touched is instrumentation, '
                  'analysis or notes',
                  'changed=%r unexpected=%r'
                  % (sorted(changed), sorted(changed - allow)))
    # and inside nav_bench.py, only write_trace moved.
    old = _git('show', BASE + ':gazebo_models/scripts/nav_bench.py').stdout
    with open(NAV_BENCH) as f:
        new = f.read()
    marker = 'def write_trace(node, path, t0, t1):'
    bad += _check(old.split(marker)[0] == new.split(marker)[0],
                  'everything BEFORE write_trace is byte-identical')
    end = 'def main(argv=None):'
    bad += _check(old.split(end, 1)[1] == new.split(end, 1)[1],
                  'everything AFTER write_trace is byte-identical')
    for name in ('c2nav11_ntp_params.yaml', 'c2nav25_slow_params.yaml'):
        p = os.path.join(HERE, name)
        d = _git('diff', '--name-only', BASE, '--',
                 os.path.relpath(p, WT)).stdout.strip()
        bad += _check(os.path.exists(p) and not d,
                      '%s unchanged vs %s' % (name, BASE))

    print()
    print('SELFTEST %s' % ('PASSED' if bad == 0 else 'FAILED (%d)' % bad))
    return 1 if bad else 0


# --------------------------------------------------------- 2. avail
def cmd_avail(args):
    """Which runs carry AMCL columns, and at what real cadence."""
    hdr('C2-NAV.28 -- AMCL coverage per leg')
    print('rows      = 10 Hz trace rows in the leg')
    print('fresh     = rows carrying a NEW /amcl_pose sample')
    print("hz        = fresh / leg seconds: the filter's OWN cadence")
    print('maxgap    = longest run of rows with no new sample, seconds')
    print('filled    = rows forward-filled within the %.1f s cap'
          % FILL_CAP_S)
    print('none      = rows with no usable estimate at all')
    print()
    print('%-16s%-18s%7s%7s%7s%8s%8s%6s'
          % ('run', 'leg', 'rows', 'fresh', 'hz', 'maxgap', 'filled',
             'none'))
    print('-' * 78)
    for tag in (args.runs or RUNS):
        legs = legs_of(tag)
        if not legs:
            print('%-16s(no record)' % tag)
            continue
        for name, leg in legs.items():
            rows = read_trace(tag, name)
            if not rows:
                print('%-16s%-18s(no trace)' % (tag, name))
                continue
            if not has_amcl(rows):
                print('%-16s%-18s(no AMCL columns -- pre-C2-NAV.28 trace)'
                      % (tag, name))
                continue
            _, nf, nfill, nnone = fill_amcl(rows)
            ts = [fl(r['t_rel']) for r in rows]
            fresh_t = [fl(r['t_rel']) for r in rows if r.get('amcl_x')]
            gap = max((b - a for a, b in zip(fresh_t, fresh_t[1:])),
                      default=None)
            dur = (ts[-1] - ts[0]) if len(ts) > 1 else 0.0
            print('%-16s%-18s%7d%7d%7.2f%8s%8d%6d'
                  % (tag, name, len(rows), nf,
                     (nf / dur) if dur else 0.0,
                     ('%.1f' % gap) if gap else '-', nfill, nnone))
    return 0


# --------------------------------------------------------- 3. error
def _stat(vals):
    if not vals:
        return None, None, None
    s = sorted(vals)
    return (s[len(s) // 2], s[int(0.95 * (len(s) - 1))], s[-1])


def cmd_error(args):
    """AMCL error before / during terminal settling, per leg."""
    hdr('C2-NAV.28 -- AMCL error, transit vs terminal')
    print('Error is est - truth in the MAP frame, so it is the quantity')
    print("the planner and controller actually see. Split at the record's")
    print('own t_transit_s (first entry into xy tolerance), which is the')
    print('same boundary C2-NAV.22 used.')
    print()
    print('%-16s%-18s%9s%9s%9s%9s%9s'
          % ('run', 'leg', 'tr_med', 'tr_max', 'te_med', 'te_max', 'd_max'))
    print('-' * 78)
    out = []
    for tag in (args.runs or RUNS):
        for name, leg in legs_of(tag).items():
            rows = read_trace(tag, name)
            if not rows or not has_amcl(rows):
                continue
            filled, _, _, _ = fill_amcl(rows)
            e = errors(rows, filled, not args.fill)
            tt = leg.get('t_transit_s')
            tr = [x[5] for x in e if tt is None or x[0] <= tt]
            te = [x[5] for x in e if tt is not None and x[0] > tt]
            trm, _, trx = _stat(tr)
            tem, _, tex = _stat(te)
            print('%-16s%-18s%9s%9s%9s%9s%9s'
                  % (tag, name,
                     '%.3f' % trm if trm is not None else '-',
                     '%.3f' % trx if trx is not None else '-',
                     '%.3f' % tem if tem is not None else '-',
                     '%.3f' % tex if tex is not None else '-',
                     '%.3f' % ((tex or 0) - (trx or 0))
                     if (tex is not None and trx is not None) else '-'))
            out.append((tag, name, trm, trx, tem, tex))
    print()
    grew = [o for o in out if o[3] is not None and o[5] is not None
            and o[5] > o[3] + 0.01]
    print('legs whose terminal MAX exceeds their transit MAX by > 0.01 m: '
          '%d of %d' % (len(grew), len(out)))
    return 0


# ---------------------------------------------------- 4. divergence
def _collect(runs, fresh_only=True):
    pool, per = [], []
    for tag in runs:
        for name, leg in legs_of(tag).items():
            rows = read_trace(tag, name)
            if not rows or not has_amcl(rows):
                continue
            filled, _, _, _ = fill_amcl(rows)
            e = errors(rows, filled, fresh_only)
            if not e:
                continue
            pool.extend(x[5] for x in e)
            per.append((tag, name, leg, e))
    return pool, per


def _threshold(pool):
    s = sorted(pool)
    return round(s[int(0.95 * (len(s) - 1))], 3)


def cmd_divergence(args):
    """Define "material" from the observed distribution, then time it."""
    hdr('C2-NAV.28 -- what counts as material divergence, and when')
    pool, per = _collect(args.runs or RUNS, not args.fill)
    if not pool:
        print('no fresh runs with AMCL columns yet -- run Part B first.')
        return 1
    s = sorted(pool)

    def q(p):
        return s[min(len(s) - 1, int(p * (len(s) - 1)))]

    print('pooled |amcl error| over every usable row of every fresh leg')
    print('  n        %d' % len(s))
    print('  median   %.3f m' % q(0.50))
    print('  p75      %.3f m' % q(0.75))
    print('  p90      %.3f m' % q(0.90))
    print('  p95      %.3f m' % q(0.95))
    print('  p99      %.3f m' % q(0.99))
    print('  max      %.3f m' % s[-1])
    thr = _threshold(pool)
    print()
    print('MATERIAL DIVERGENCE := |amcl error| > %.3f m, the pooled p95.'
          % thr)
    print("It is a percentile of THIS session's own distribution, fixed")
    print('before any leg is inspected individually, so it is not a')
    print('threshold chosen to make a particular leg cross it. It is not')
    print('a safety limit and carries no physical meaning beyond "in the')
    print('worst 5 % of what this configuration does".')
    print()
    print('%-16s%-18s%9s%9s%9s  %s'
          % ('run', 'leg', 't_cross', 't_trans', 'dur', 'shape'))
    print('-' * 78)
    for tag, name, leg, e in per:
        cross = next((x[0] for x in e if x[5] > thr), None)
        tt = leg.get('t_transit_s')
        print('%-16s%-18s%9s%9s%9s  %s'
              % (tag, name,
                 '%.1f' % cross if cross is not None else '-',
                 '%.1f' % tt if tt is not None else '-',
                 '%.1f' % (leg.get('duration_sim_s') or 0.0),
                 _shape(e, thr)))
    return 0


def _shape(e, thr):
    """Monotonic / step / oscillatory / stable, from the error series."""
    v = [x[5] for x in e]
    if len(v) < 10:
        return 'too short'
    if max(v) <= thr:
        return 'stable (never material)'
    d = [b - a for a, b in zip(v, v[1:])]
    biggest = max(abs(x) for x in d)
    span = max(v) - min(v)
    up = sum(1 for x in d if x > 0)
    frac_up = up / len(d)
    signflips = sum(1 for a, b in zip(d, d[1:]) if a * b < 0)
    if span > 0 and biggest > 0.5 * span:
        return 'step (one jump = %.0f%% of the whole range)' % (
            100 * biggest / span)
    if frac_up > 0.75:
        return 'monotonic rise'
    if frac_up < 0.25:
        return 'monotonic fall'
    if signflips > 0.4 * len(d):
        return 'oscillatory'
    return 'mixed'


# ------------------------------------------------------ 5. timeline
def cmd_timeline(args):
    """The compact per-row timeline the brief asks for."""
    tag, leg_name = args.tag, args.leg
    leg = legs_of(tag).get(leg_name)
    rows = read_trace(tag, leg_name)
    if not leg or not rows:
        print('no such leg: %s / %s' % (tag, leg_name))
        return 1
    if not has_amcl(rows):
        print('%s / %s has no AMCL columns' % (tag, leg_name))
        return 1
    filled, nf, _, nnone = fill_amcl(rows)
    hdr('C2-NAV.28 timeline -- %s / %s  (%s)'
        % (tag, leg_name, leg.get('status')))
    print('t_transit_s %s   duration %.1f s   fresh AMCL rows %d / %d'
          % (leg.get('t_transit_s'), leg.get('duration_sim_s') or 0.0,
             nf, len(rows)))
    print('A = fresh AMCL sample on this row; . = forward-filled;')
    print('blank amcl columns = no usable estimate (%d rows)' % nnone)
    print("rot = dwb_ill_rot, RotateToGoal's rejection count -- the")
    print('      PROXY for its latch banning translation (see header)')
    print()
    refus = _refusals_for(tag, leg_name, leg)
    rt = {round(r['t_rel_est'], 1): r for r in refus}
    print('%6s %s %-15s %-15s%7s%7s%7s%7s%7s%6s %-13s%s'
          % ('t', ' ', 'gt(map)', 'amcl(map)', 'err', 'gt_yaw',
             'a_yaw', 'v_nav', 'v_act', 'rot', 'polygon', ' planner'))
    print('-' * 118)
    step = max(1, int(args.every * 10))
    for i, (r, a) in enumerate(zip(rows, filled)):
        t = fl(r['t_rel'])
        keep = (i % step == 0) or (round(t, 1) in rt) or bool(r.get('amcl_x'))
        if args.since is not None and t < args.since:
            keep = False
        if args.until is not None and t > args.until:
            keep = False
        if not keep:
            continue
        x, y, yaw = fl(r['x']), fl(r['y']), fl(r['yaw'])
        gm = ('(%6.3f,%6.3f)' % (x + WORLD_TO_MAP_X, y + WORLD_TO_MAP_Y)
              if x is not None else '')
        if a is None or x is None:
            am, err, ayaw, mark = '', '', '', ' '
        else:
            am = '(%6.3f,%6.3f)' % (a[0], a[1])
            err = '%.3f' % math.hypot(a[0] - (x + WORLD_TO_MAP_X),
                                      a[1] - (y + WORLD_TO_MAP_Y))
            ayaw = '%.3f' % a[2]
            mark = 'A' if r.get('amcl_x') else '.'
        ev = rt.get(round(t, 1))
        print('%6.1f %s %-15s %-15s%7s%7s%7s%7s%7s%6s %-13s%s'
              % (t, mark, gm, am, err,
                 '%.3f' % yaw if yaw is not None else '',
                 ayaw,
                 r.get('v_nav', ''), r.get('v_act', ''),
                 r.get('dwb_ill_rot', ''), r.get('cm_polygon', ''),
                 (' <-- ' + ev['reason']) if ev else ''))
    if refus:
        print()
        print('planner lines aligned into this leg (see ALIGNED in header):')
        for r in refus:
            print('  t~%.1f s  %-16s start(map) %s  '
                  'trace amcl at that row %s   |match %s|'
                  % (r['t_rel_est'], r['reason'], r['start'],
                     r['trace_amcl'], r['match']))
    return 0


def _refusals_for(tag, leg_name, leg):
    """Planner refusal lines placed into one leg's trace time.

    System-stamped log lines -> sim-time trace offsets, anchored on the
    leg's own "Begin navigating" line and rescaled by that leg's measured
    duration_sim_s / duration_wall_s. The pose the log line quotes is
    then matched against the trace's own AMCL columns, and BOTH the
    aligned time and the match quality are reported.
    """
    p = os.path.join(LOGS, 'nav_%s.log' % tag)
    if not os.path.exists(p):
        return []
    txt = open(p, errors='replace').read()
    begins = [float(m.group(1)) for m in BEGIN.finditer(txt)]
    order = list(legs_of(tag).keys())
    if leg_name not in order:
        return []
    idx = order.index(leg_name)
    if idx >= len(begins):
        return []
    t_begin = begins[idx]
    t_end = begins[idx + 1] if idx + 1 < len(begins) else float('inf')
    rtf = rtf_of(leg) or 1.0
    rows = read_trace(tag, leg_name) or []
    filled, _, _, _ = fill_amcl(rows)
    out = []
    for m in REFUSE.finditer(txt):
        ts = float(m.group(1))
        if not (t_begin <= ts < t_end):
            continue
        t_rel = (ts - t_begin) * rtf
        sx, sy = float(m.group(2)), float(m.group(3))
        i = (min(range(len(rows)),
                 key=lambda k: abs((fl(rows[k]['t_rel']) or 0) - t_rel))
             if rows else None)
        a = filled[i] if i is not None else None
        d = (math.hypot(a[0] - sx, a[1] - sy) if a else None)
        out.append({'t_rel_est': t_rel, 'reason': m.group(6),
                    'start': '(%.2f,%.2f)' % (sx, sy),
                    'trace_amcl': ('(%.2f,%.2f)' % (a[0], a[1])
                                   if a else 'none'),
                    'match': ('%.3f m' % d) if d is not None else 'n/a',
                    'sx': sx, 'sy': sy, 'row': i})
    return out


def _gt_at(rows, t_rel):
    """Ground truth in the MAP frame at the trace row nearest t_rel."""
    best = None
    for r in rows:
        t = fl(r.get('t_rel'))
        x, y = fl(r.get('x')), fl(r.get('y'))
        if t is None or x is None or y is None:
            continue
        d = abs(t - t_rel)
        if best is None or d < best[0]:
            best = (d, x + WORLD_TO_MAP_X, y + WORLD_TO_MAP_Y)
    return (best[1], best[2]) if best else None


# ----------------------------------------------------- 6. correlate
def cmd_correlate(args):
    """Does divergence GROWTH coincide with the named signals?

    The interval is BETWEEN CONSECUTIVE FRESH AMCL SAMPLES, not a fixed
    1 s window. That is forced by the measured cadence: /amcl_pose
    arrives at 1-2 Hz, so a fixed window with a fresh sample at both
    ends is rare (12 of them across all four runs) and a design built on
    one would be reporting noise. Consecutive fresh pairs use every
    sample the filter published and nothing else.

    Growth is reported per second, since the intervals differ in length.
    """
    hdr('C2-NAV.28 -- what the divergence grows alongside')
    print('One row per pair of CONSECUTIVE fresh /amcl_pose samples.')
    print('d|e|/s is the change in |amcl error| over that interval,')
    print('divided by its length. The interval is classified by the')
    print('trace rows it spans. This is ASSOCIATION: nothing here')
    print('establishes direction, and the classes overlap by design.')
    print()
    buckets = collections.defaultdict(list)
    for tag in (args.runs or RUNS):
        for name, leg in legs_of(tag).items():
            rows = read_trace(tag, name)
            if not rows or not has_amcl(rows):
                continue
            filled, _, _, _ = fill_amcl(rows)
            fresh = [(k, r) for k, r in enumerate(rows) if r.get('amcl_x')]
            e = errors(rows, filled)
            emap = {round(x[0], 1): x for x in e}
            for (k0, r0), (k1, r1) in zip(fresh, fresh[1:]):
                t0 = round(fl(r0['t_rel']) or 0.0, 1)
                t1 = round(fl(r1['t_rel']) or 0.0, 1)
                if t0 not in emap or t1 not in emap or t1 <= t0:
                    continue
                rate = (emap[t1][5] - emap[t0][5]) / (t1 - t0)
                win = rows[k0:k1 + 1]
                n = len(win) or 1
                wz = max(abs(fl(r.get('w_act')) or 0) for r in win)
                rot = sum(1 for r in win if fl(r.get('dwb_ill_rot'))) / n
                zerovx = sum(
                    1 for r in win
                    if r.get('dwb_best_vx') not in ('', None)
                    and (fl(r.get('dwb_best_vx')) or 0) == 0) / n
                vact = max(abs(fl(r.get('v_act')) or 0) for r in win)
                poly = collections.Counter(
                    r.get('cm_polygon') or 'none'
                    for r in win).most_common(1)[0][0]
                tt = leg.get('t_transit_s')
                buckets['phase: terminal' if (tt is not None and t0 > tt)
                        else 'phase: transit'].append(rate)
                buckets['|wz| > 0.5 rad/s' if wz > 0.5
                        else '|wz| <= 0.5 rad/s'].append(rate)
                buckets['|v_act| > 0.05 m/s' if vact > 0.05
                        else '|v_act| <= 0.05 m/s'].append(rate)
                buckets['RotateToGoal rejecting' if rot >= 0.5
                        else 'RotateToGoal quiet'].append(rate)
                buckets['DWB best vx == 0' if zerovx >= 0.5
                        else 'DWB best vx != 0'].append(rate)
                buckets['polygon ' + poly].append(rate)
    if not buckets:
        print('no fresh runs with AMCL columns yet -- run Part B first.')
        return 1
    print('%-28s%6s%10s%10s%10s%10s%8s'
          % ('interval class', 'n', 'mean/s', 'med/s', 'p05/s', 'p95/s',
             'frac>0'))
    print('-' * 78)
    for k in sorted(buckets):
        v = sorted(buckets[k])
        print('%-28s%6d%10.4f%10.4f%10.4f%10.4f%8.2f'
              % (k, len(v), sum(v) / len(v), v[len(v) // 2],
                 v[int(0.05 * (len(v) - 1))],
                 v[int(0.95 * (len(v) - 1))],
                 sum(1 for x in v if x > 0) / len(v)))
    print()
    print('frac>0 is the share of intervals in which the error GREW. A')
    print('class that systematically drives divergence would sit well')
    print('above 0.50 there, not merely carry a wider tail.')
    return 0


# ------------------------------------------------------ 7. startocc
def cmd_startocc(args):
    """Every planner refusal in the fresh runs, against AMCL and truth."""
    hdr('C2-NAV.28 -- planner refusals in the fresh runs')
    n = 0
    for tag in (args.runs or RUNS):
        p = os.path.join(LOGS, 'nav_%s.log' % tag)
        if not os.path.exists(p):
            print('%-16s(no run log)' % tag)
            continue
        for name, leg in legs_of(tag).items():
            rows = read_trace(tag, name) or []
            filled, _, _, _ = fill_amcl(rows)
            e = errors(rows, filled, False)
            for r in _refusals_for(tag, name, leg):
                n += 1
                near = (min(e, key=lambda x: abs(x[0] - r['t_rel_est']))
                        if e else None)
                print()
                print('%s / %s  t~%.1f s  %s'
                      % (tag, name, r['t_rel_est'], r['reason']))
                print('   planner start (map)      %s' % r['start'])
                print('   trace AMCL at that row   %s   (match %s)'
                      % (r['trace_amcl'], r['match']))
                # Ground truth at the refusal row, from the trace's own
                # 10 Hz gt columns -- available even when the filter
                # published nothing during this leg.
                gt = _gt_at(rows, r['t_rel_est'])
                if gt:
                    print('   ground truth (map)       (%.3f,%.3f)' % gt)
                    print('   AMCL error, from the LOG pose (2 dp)  '
                          '%.3f m  (dx %+.3f, dy %+.3f)'
                          % (math.hypot(r['sx'] - gt[0], r['sy'] - gt[1]),
                             r['sx'] - gt[0], r['sy'] - gt[1]))
                else:
                    print('   ground truth             UNAVAILABLE')
                if near and abs(near[0] - r['t_rel_est']) < 2.0:
                    print('   AMCL error, from the TRACE column     '
                          '%.3f m  (dx %+.3f, dy %+.3f)  '
                          'sample age %.1f s'
                          % (near[5], near[3], near[4], near[7]))
                else:
                    print('   AMCL error, from the TRACE column     '
                          'UNAVAILABLE -- no /amcl_pose within 2 s of '
                          'the refusal')
    if n == 0:
        print()
        print('NO planner refusal of any kind occurred in the fresh runs.')
        print('This is stated as an observation, not converted into a')
        print('claim about how often refusals happen: four tours cannot')
        print('bound a rate that C2-NAV.26 saw in roughly one leg in four.')
    return 0


# ------------------------------------------------------ 8. ordering
def cmd_ordering(args):
    """The stop-condition answer: before, during, or after."""
    hdr('C2-NAV.28 -- temporal ordering: divergence vs terminal yaw')
    pool, per = _collect(args.runs or RUNS, not args.fill)
    if not pool:
        print('no fresh runs with AMCL columns yet -- run Part B first.')
        return 1
    thr = _threshold(pool)
    print('material divergence threshold %.3f m (pooled p95, see '
          '`divergence`)' % thr)
    print()
    print('%-16s%-18s%9s%9s%9s  %s'
          % ('run', 'leg', 'e@0', 'e@trans', 'e@end', 'ordering'))
    print('-' * 78)
    counts = collections.Counter()
    for tag, name, leg, e in per:
        tt = leg.get('t_transit_s')
        e0 = e[0][5]
        eend = e[-1][5]
        etr = None
        if tt is not None:
            cand = [x for x in e if x[0] <= tt]
            etr = cand[-1][5] if cand else None
        cross = next((x[0] for x in e if x[5] > thr), None)
        if cross is None:
            verdict = 'never material'
        elif tt is None:
            verdict = 'no terminal phase (never reached xy tol)'
        elif cross < tt:
            verdict = 'BEFORE terminal (%.1f s < %.1f s)' % (cross, tt)
        else:
            verdict = 'DURING/AFTER terminal (%.1f s >= %.1f s)' % (cross, tt)
        counts[verdict.split(' (')[0]] += 1
        print('%-16s%-18s%9.3f%9s%9.3f  %s'
              % (tag, name, e0,
                 '%.3f' % etr if etr is not None else '-', eend, verdict))
    print()
    for k, v in counts.most_common():
        print('  %-42s %d legs' % (k, v))
    return 0


# ---------------------------------------------------------- 9. dump
# Columns the bundle keeps per frozen row. Everything the analyses read:
# the trace time, ground truth, the AMCL sample itself, and the signals
# `correlate` and `timeline` classify on. Rows WITHOUT a fresh AMCL
# sample are not kept -- see read_trace.
DUMP_COLS = ['t_rel', 'x', 'y', 'yaw', 'v_act', 'w_act', 'v_nav', 'w_nav',
             'cm_polygon', 'dwb_best_vx', 'dwb_ill_rot',
             'amcl_x', 'amcl_y', 'amcl_yaw']

# Record fields the bundle keeps per leg.
DUMP_LEG = ['scenario', 'status', 'duration_sim_s', 'duration_wall_s',
            't_transit_s', 't_terminal_s', 'terminal_frac_of_leg',
            'terminal_yaw_travel_rad', 'final_goal_err_m', 'n_plans',
            'n_progress_failures', 'goal_world', 'cm_polygon_secs']


def cmd_dump(args):
    """Freeze the fresh runs into the committed bundle."""
    hdr('C2-NAV.28 -- freezing the fresh runs into %s'
        % os.path.basename(BUNDLE))
    out = {'runs': list(args.runs or RUNS), 'legs': {}, 'traces': {},
           'refusals': {}, 'gt_at_refusal': {}}
    n_rows = 0
    empty = []
    for tag in (args.runs or RUNS):
        legs = legs_of(tag)
        if not legs:
            print('%-16s(no record -- skipped)' % tag)
            continue
        out['legs'][tag] = {n: {k: leg.get(k) for k in DUMP_LEG}
                            for n, leg in legs.items()}
        out['traces'][tag] = {}
        out['refusals'][tag] = {}
        for name, leg in legs.items():
            rows = read_trace(tag, name)
            if not rows:
                continue
            keep = [{k: r.get(k, '') for k in DUMP_COLS}
                    for r in rows if r.get('amcl_x')]
            out['traces'][tag][name] = keep
            n_rows += len(keep)
            if not keep:
                empty.append('%s/%s' % (tag, name))
            ref = _refusals_for(tag, name, leg)
            if ref:
                out['refusals'][tag][name] = ref
                # the refusal rows need ground truth, which the fresh
                # rows may not straddle -- freeze it explicitly.
                out['gt_at_refusal'].setdefault(tag, {})[name] = [
                    {'t_rel_est': r['t_rel_est'],
                     'gt_map': _gt_at(rows, r['t_rel_est'])} for r in ref]
    with open(BUNDLE, 'w') as f:
        json.dump(out, f, separators=(',', ':'), sort_keys=True)
    print('wrote %s: %d runs, %d frozen fresh rows, %.1f KB'
          % (BUNDLE, len(out['traces']), n_rows,
             os.path.getsize(BUNDLE) / 1e3))
    print()
    if empty:
        print()
        print('%d leg(s) froze ZERO rows because /amcl_pose never'
              % len(empty))
        print('fired during them, so a bundle-backed table lists that')
        print('many fewer legs than a scratch-backed one. They are:')
        for k in empty:
            print('   %s' % k)
        print('Both counts are right for their input; no number changes.')
    print()
    print('The bundle keeps FRESH AMCL rows only, which is every row any')
    print('headline number is computed from. `--fill` needs the rows in')
    print('between and therefore works only against live .navbench')
    print('scratch; it is context, never a headline.')
    return 0


CMDS = {'selftest': cmd_selftest, 'avail': cmd_avail, 'error': cmd_error,
        'divergence': cmd_divergence, 'timeline': cmd_timeline,
        'correlate': cmd_correlate, 'startocc': cmd_startocc,
        'ordering': cmd_ordering, 'dump': cmd_dump}


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('cmd', choices=list(CMDS) + ['all'])
    ap.add_argument('--runs', nargs='*', default=None)
    ap.add_argument('--tag', default=None)
    ap.add_argument('--leg', default=None)
    ap.add_argument('--every', type=float, default=1.0,
                    help='timeline: seconds between forced rows')
    ap.add_argument('--since', type=float, default=None)
    ap.add_argument('--until', type=float, default=None)
    ap.add_argument('--fill', action='store_true',
                    help='measure on the forward-filled series '
                         'instead of fresh samples only -- see '
                         'errors() for why that is not the '
                         'default')
    a = ap.parse_args(argv)
    if a.cmd == 'all':
        rc = 0
        for name in ('avail', 'error', 'divergence', 'correlate',
                     'startocc', 'ordering'):
            rc |= CMDS[name](a) or 0
        return rc
    return CMDS[a.cmd](a) or 0


if __name__ == '__main__':
    sys.exit(main())
