#!/usr/bin/env python3
"""C2-NAV.34 -- is the UNRECORDED odom -> base_footprint motion consumed by
AMCL a quantitatively plausible source of the wall-adjacent bias?

Investigation only.  Nothing here starts a simulator, changes a parameter or
touches `main`.  Every number is produced from artefacts already in the repo
or already on this machine.

Run every mode from the WORKTREE ROOT:

    python3 -P docs/data/c2nav34_odom.py selftest   # 0 external inputs
    python3 -P docs/data/c2nav34_odom.py geom       # wheel geometry, Part 3
    python3 -P docs/data/c2nav34_odom.py incr       # the headline residual
    python3 -P docs/data/c2nav34_odom.py heading    # place dependence, Part 6
    python3 -P docs/data/c2nav34_odom.py elim       # two eliminations
    python3 -P docs/data/c2nav34_odom.py cmd        # command-tracking bound
    python3 -P docs/data/c2nav34_odom.py verdict

`-P` matters: it keeps the script's own directory off sys.path, which is the
`numbers.py`/`trace.py` shadowing trap in CLAUDE.md.

DATA PROVENANCE
---------------
`docs/data/c2nav28_amcl.json`  -- COMMITTED.  Per-AMCL-update rows carrying
   ground truth (`x`,`y`,`yaw`, world frame, from the gz OdometryPublisher)
   and the AMCL estimate (`amcl_x`,`amcl_y`,`amcl_yaw`, map frame).
`docs/data/c2nav34_odom.json` -- COMMITTED, built by `build` from the 10 Hz
   trace CSVs under `.navbench/results/c2n28_*_traces/`, which are NOT in the
   repository.  Rebuilding it needs those CSVs; every other mode does not.

WHAT IS AND IS NOT MEASURED HERE
--------------------------------
The odom -> base_footprint transform is recorded NOWHERE (C2-NAV.33 F10, and
re-verified: `nav_bench.py` installs no TransformListener and subscribes to no
wheel-odometry topic).  So no mode below measures the odometry.  What they
measure is AMCL's own increment against ground truth, which is

    (AMCL delta) - (GT delta)  =  (odometry error)  +  (laser correction)

and the two terms are NOT separated by anything in this file.  That is the
whole reason a new measurement is proposed rather than a conclusion drawn.
"""

import csv
import glob
import json
import math
import os
import statistics as st
import sys

# map = world + (2.0, 0.0), unrotated -- nav_bench.py:88-89, and identically
# docs/data/c2nav28_amcl.py:127-128.
WORLD_TO_MAP_X = 2.0
WORLD_TO_MAP_Y = 0.0

# gazebo_models/urdf/coco_controllers.yaml, diff_drive_controller block.
WHEEL_SEPARATION = 0.274
WHEEL_RADIUS = 0.0585
WHEEL_SEPARATION_MULTIPLIER = 1.10

# gazebo_models/urdf/coco_robo2.xacro.  chassis_joint carries rpy=(pi/2,0,0)
# and xyz=(0.12,-0.08,0); the four wheel joints hang off chassis_link at
# (-0.03|-0.21, 0.045, 0.057|-0.217).  Rx(pi/2) maps (x,y,z)->(x,-z,y).
CHASSIS_XYZ = (0.12, -0.08, 0.0)
WHEEL_JOINTS_CHASSIS = {
    'base_Revolute-1': (-0.03, 0.045, 0.057),    # wheel1  front-right
    'base_Revolute-2': (-0.21, 0.045, 0.057),    # wheel2  rear-right
    'base_Revolute-3': (-0.03, 0.045, -0.217),   # wheel3  front-left
    'base_Revolute-4': (-0.21, 0.045, -0.217),   # wheel4  rear-left
}

HERE = os.path.dirname(os.path.abspath(__file__))
BUNDLE28 = os.path.join(HERE, 'c2nav28_amcl.json')
BUNDLE34 = os.path.join(HERE, 'c2nav34_odom.json')
TRACE_GLOB = '.navbench/results/c2n28_*_traces/*.csv'


# ---------------------------------------------------------------- geometry --

def wrap(a):
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def rx90(p):
    """Rotate by +pi/2 about x: (x,y,z) -> (x, -z, y)."""
    return (p[0], -p[2], p[1])


def wheel_positions_base_link():
    """Wheel joint origins expressed in base_link, from the xacro."""
    out = {}
    for name, p in WHEEL_JOINTS_CHASSIS.items():
        r = rx90(p)
        out[name] = (r[0] + CHASSIS_XYZ[0],
                     r[1] + CHASSIS_XYZ[1],
                     r[2] + CHASSIS_XYZ[2])
    return out


def body_delta(p0, p1):
    """SE(2) body-frame delta: p0^-1 . p1 -> (dx_along, dy_lateral, dyaw).

    A left-multiplied constant frame offset (world -> map) cancels exactly,
    so GT deltas and AMCL deltas are directly comparable without knowing
    WORLD_TO_MAP.  This is the same argument C2-NAV.32 used.
    """
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    c, s = math.cos(p0[2]), math.sin(p0[2])
    return (c * dx + s * dy, -s * dx + c * dy, wrap(p1[2] - p0[2]))


def mean_ci(xs):
    """(mean, lo, hi) with a 1.96 normal quantile."""
    n = len(xs)
    if n < 2:
        return (float('nan'),) * 3
    m = st.fmean(xs)
    h = 1.96 * st.stdev(xs) / math.sqrt(n)
    return m, m - h, m + h


# -------------------------------------------------------------------- data --

def load28():
    with open(BUNDLE28) as f:
        return json.load(f)


def load34():
    if not os.path.exists(BUNDLE34):
        sys.exit(f'MISSING: {BUNDLE34}\n'
                 'This is a distinct outcome, not a finding: rebuild it with\n'
                 f'  python3 -P {__file__} build\n'
                 'which needs the uncommitted trace CSVs under .navbench/.')
    with open(BUNDLE34) as f:
        return json.load(f)


def legs28(d):
    """Yield (run, leg, [(gt, amcl), ...]) with both poses present."""
    for run, legs in sorted(d['traces'].items()):
        for leg, rows in sorted(legs.items()):
            seq = []
            for r in rows:
                try:
                    g = (float(r['x']), float(r['y']), float(r['yaw']))
                    a = (float(r['amcl_x']), float(r['amcl_y']),
                         float(r['amcl_yaw']))
                except (KeyError, TypeError, ValueError):
                    continue
                seq.append((g, a))
            if len(seq) >= 4:
                yield run, leg, seq


def residuals(seq):
    """[(r_along, r_lateral, r_yaw)] per consecutive AMCL update."""
    out = []
    for i in range(1, len(seq)):
        dg = body_delta(seq[i - 1][0], seq[i][0])
        da = body_delta(seq[i - 1][1], seq[i][1])
        out.append((da[0] - dg[0], da[1] - dg[1], wrap(da[2] - dg[2])))
    return out


def gt_along(seq):
    return sum(body_delta(seq[i - 1][0], seq[i][0])[0]
               for i in range(1, len(seq)))


def world_err(pose_gt, pose_amcl):
    return (pose_amcl[0] - (pose_gt[0] + WORLD_TO_MAP_X),
            pose_amcl[1] - (pose_gt[1] + WORLD_TO_MAP_Y),
            wrap(pose_amcl[2] - pose_gt[2]))


# ------------------------------------------------------------------- modes --

def mode_geom():
    print('C2-NAV.34 geom -- effective vs physical wheel geometry')
    print()
    pos = wheel_positions_base_link()
    print('Wheel joint origins in base_link, derived from coco_robo2.xacro:')
    for n in sorted(pos):
        p = pos[n]
        print(f'  {n:18s} ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f})')
    ys = sorted(p[1] for p in pos.values())
    track = max(ys) - min(ys)
    print()
    print(f'  physical lateral track      = {track:.6f} m')
    print(f'  wheel_separation parameter  = {WHEEL_SEPARATION:.6f} m')
    print(f'  difference                  = {abs(track - WHEEL_SEPARATION):.2e} m')
    assert abs(track - WHEEL_SEPARATION) < 1e-9, 'track != parameter'
    print('  => the NOMINAL parameter IS the true physical track. (measured)')
    print()
    b_eff = WHEEL_SEPARATION * WHEEL_SEPARATION_MULTIPLIER
    print(f'  wheel_separation_multiplier = {WHEEL_SEPARATION_MULTIPLIER}')
    print(f'  effective separation b_eff  = {b_eff:.6f} m')
    print(f'  b_eff / b_true              = {b_eff / track:.4f}  '
          f'(+{100 * (b_eff / track - 1):.1f} %)')
    print()
    print('THE CANCELLATION IDENTITY  (derived)')
    print('  command path : u_r - u_l = w_cmd * b_eff')
    print('  odometry     : w_odom    = (u_r - u_l) / b_eff  ==  w_cmd')
    print('  => b_eff CANCELS.  The multiplier cannot by itself bias the')
    print('     reported yaw rate; odometry reports back the commanded yaw')
    print('     rate whatever the multiplier is.  What it does change is the')
    print('     PHYSICAL yaw the wheels are asked to produce:')
    print('       w_true = eta * w_cmd * b_eff / b_true = eta * 1.10 * w_cmd')
    print('     so   w_odom / w_true = 1 / (1.10 * eta),  unbiased iff')
    print(f'     eta = 1/1.10 = {1 / WHEEL_SEPARATION_MULTIPLIER:.4f}.')
    print()
    print('  eta (skid-steer yaw efficiency) is condition dependent and is')
    print('  NOT measured anywhere in this repository. (UNKNOWN)')
    print()
    print('LINEAR CHANNEL  (derived)')
    print('  v_odom = (u_r + u_l)/2 = v_cmd exactly -- no multiplier, no')
    print('  separation.  With position_feedback=true (the diff_drive')
    print('  default, not overridden) odometry integrates MEASURED wheel')
    print('  positions, so longitudinal wheel slip is absorbed in full as')
    print('  over-reported forward distance.')


def mode_incr():
    d = load28()
    print('C2-NAV.34 incr -- signed body-frame residual per AMCL update')
    print()
    print('  r = (AMCL body delta) - (GT body delta)')
    print('    = (odometry error) + (laser correction)      [NOT separated]')
    print()
    print(f"{'run':16s} {'leg':16s} {'n':>3s} {'sum_along':>10s} "
          f"{'GT_along':>9s} {'excess%':>8s} {'sum_lat':>8s} {'sum_yaw':>8s}")
    byleg = {}
    for run, leg, seq in legs28(d):
        R = residuals(seq)
        sa = sum(r[0] for r in R)
        sl = sum(r[1] for r in R)
        sy = sum(r[2] for r in R)
        ga = gt_along(seq)
        pct = 100.0 * sa / ga if abs(ga) > 1e-6 else float('nan')
        print(f'{run:16s} {leg:16s} {len(R):3d} {sa:10.4f} {ga:9.4f} '
              f'{pct:8.2f} {sl:8.4f} {sy:8.4f}')
        byleg.setdefault(leg, []).append(pct)
    print()
    print('  along-track excess, per leg (one value per run):')
    allv = []
    for leg, v in sorted(byleg.items()):
        allv.extend(v)
        print(f'    {leg:18s} n={len(v)} mean={st.fmean(v):7.2f} % '
              f'values={[round(z, 1) for z in v]}')
    print(f'    {"ALL":18s} n={len(allv)} mean={st.fmean(allv):7.2f} %')
    pos = sum(1 for z in allv if z > 0)
    print(f'    positive in {pos} of {len(allv)} legs')
    print()
    # pooled per-update statistics -- signed, so noise cannot inflate them
    print('  pooled per-update residual, signed means with 95 % CI:')
    pool = {}
    for _run, leg, seq in legs28(d):
        pool.setdefault(leg, []).extend(residuals(seq))
    print(f"    {'leg':18s} {'n':>4s} {'mean along':>11s} {'95% CI':>22s}"
          f" {'mean yaw':>10s}")
    everything = []
    for leg, R in sorted(pool.items()):
        everything.extend(R)
        a = [r[0] for r in R]
        m, lo, hi = mean_ci(a)
        my, _, _ = mean_ci([r[2] for r in R])
        print(f'    {leg:18s} {len(R):4d} {m:11.5f} [{lo:9.5f},{hi:9.5f}]'
              f' {my:10.5f}')
    a = [r[0] for r in everything]
    m, lo, hi = mean_ci(a)
    my, ylo, yhi = mean_ci([r[2] for r in everything])
    print(f'    {"ALL":18s} {len(everything):4d} {m:11.5f} '
          f'[{lo:9.5f},{hi:9.5f}] {my:10.5f}')
    print()
    print(f'  ALL along-track: mean {m:+.5f} m/update, CI [{lo:+.5f},{hi:+.5f}]'
          f' -- {"EXCLUDES" if lo > 0 or hi < 0 else "straddles"} zero')
    print(f'  ALL yaw        : mean {my:+.5f} rad/update, CI '
          f'[{ylo:+.5f},{yhi:+.5f}] -- '
          f'{"EXCLUDES" if ylo > 0 or yhi < 0 else "straddles"} zero')


def mode_heading():
    b = load34()
    print('C2-NAV.34 heading -- can ONE body-frame error make DIFFERENT')
    print('world-frame displacements at different places?  (Part 6)')
    print()
    print('  If the residual is along-track and positive, each leg\'s world')
    print('  error should point along that leg\'s direction of travel.')
    print()
    print(f"{'run/leg':28s} {'course':>8s} {'obs_dX':>8s} {'obs_dY':>8s} "
          f"{'|obs|':>7s} {'proj_fwd':>9s} {'proj_lat':>9s}")
    fwd_pos = 0
    tot = 0
    rows = []
    for e in b['legs']:
        course = math.atan2(e['net_dy'], e['net_dx'])
        ux, uy = math.cos(course), math.sin(course)
        pf = e['obs_dx'] * ux + e['obs_dy'] * uy
        pl = -e['obs_dx'] * uy + e['obs_dy'] * ux
        mag = math.hypot(e['obs_dx'], e['obs_dy'])
        if e['path'] < 0.5:
            continue
        tot += 1
        fwd_pos += pf > 0
        rows.append((e['leg'], pf, pl))
        print(f"{e['run'] + '/' + e['leg']:28s} "
              f"{math.degrees(course):8.1f} {e['obs_dx']:8.4f} "
              f"{e['obs_dy']:8.4f} {mag:7.4f} {pf:9.4f} {pl:9.4f}")
    print()
    print(f'  world error projects FORWARD along the course in '
          f'{fwd_pos} of {tot} legs')
    pf = [r[1] for r in rows]
    pl = [r[2] for r in rows]
    m, lo, hi = mean_ci(pf)
    ml, llo, lhi = mean_ci(pl)
    print(f'  mean forward projection {m:+.4f} m  CI [{lo:+.4f},{hi:+.4f}]')
    print(f'  mean lateral projection {ml:+.4f} m  CI [{llo:+.4f},{lhi:+.4f}]')
    print()
    print('  the two southbound legs, which are where the bias was named:')
    for e in b['legs']:
        if e['leg'] in ('open_space', 'wall_adjacent'):
            print(f"    {e['run'] + '/' + e['leg']:28s} course "
                  f"{math.degrees(math.atan2(e['net_dy'], e['net_dx'])):7.1f} deg"
                  f"  ey {e['ey_start']:+.4f} -> {e['ey_end']:+.4f}")


def mode_elim():
    b = load34()
    print('C2-NAV.34 elim -- two mechanisms that do NOT explain the bias')
    print()
    print('(1) HEADING-ERROR INTEGRAL.  If the position error were the')
    print('    consequence of the heading error, integrating')
    print('        e_dot = e_psi * (-vy_world, +vx_world)')
    print('    against GROUND-TRUTH velocity and the OBSERVED heading error')
    print('    would reproduce the observed position error.')
    print()
    print(f"{'run/leg':28s} {'obs_dY':>8s} {'pred_dY':>8s} {'ratio':>7s}")
    ok = tot = 0
    for e in b['legs']:
        if e['path'] < 0.5:
            continue
        tot += 1
        r = e['head_pred_dy'] / e['obs_dy'] if abs(e['obs_dy']) > 1e-6 else float('nan')
        if not math.isnan(r) and 0.5 <= r <= 2.0:
            ok += 1
        print(f"{e['run'] + '/' + e['leg']:28s} {e['obs_dy']:8.4f} "
              f"{e['head_pred_dy']:8.4f} {r:7.2f}")
    print(f'\n  within a factor of 2 and correct sign in {ok} of {tot} legs')
    print('  At open_space the prediction is near ZERO while the observation')
    print('  is about -0.10 m.  The heading error does not generate it.')
    print()
    print('(2) LATERAL-SKID BLINDNESS.  A differential-drive integrator has')
    print('    no lateral state, so real sideways motion is invisible to it')
    print('    and must accumulate as error:')
    print('        (odom - true) = -integral( v_lat * n_hat ) dt')
    print()
    print(f"{'run/leg':28s} {'lat_abs':>8s} {'lat/path':>9s} "
          f"{'blindY':>8s} {'obs_dY':>8s} {'ratio':>7s}")
    same = tot = 0
    for e in b['legs']:
        if e['path'] < 0.5:
            continue
        tot += 1
        r = e['blind_dy'] / e['obs_dy'] if abs(e['obs_dy']) > 1e-6 else float('nan')
        if not math.isnan(r) and r > 0:
            same += 1
        print(f"{e['run'] + '/' + e['leg']:28s} {e['lat_abs']:8.4f} "
              f"{e['lat_abs'] / e['path']:9.4f} {e['blind_dy']:8.4f} "
              f"{e['obs_dy']:8.4f} {r:7.2f}")
    print(f'\n  sign AGREES with the observation in {same} of {tot} legs')
    tot_lat = sum(e['lat_abs'] for e in b['legs'])
    tot_path = sum(e['path'] for e in b['legs'])
    print(f'  the robot really does slide: total |lateral| {tot_lat:.3f} m '
          f'over {tot_path:.3f} m of path = {100 * tot_lat / tot_path:.1f} %')
    print('  but the term is small and points the WRONG WAY, so it is')
    print('  eliminated as the principal mechanism.')


def mode_cmd():
    b = load34()
    c = b['cmd_tracking']
    print('C2-NAV.34 cmd -- how much of the along-track excess is already')
    print('visible in the COMMAND channel?')
    print()
    print('  v_odom == v_cmd exactly (geom), so if the robot achieves less')
    print('  than the commanded linear velocity, odometry over-reports')
    print('  forward distance by the reciprocal.')
    print()
    print(f"  slope v_act / v_wheel  = {c['slope']:.4f}   n = {c['n']}")
    print(f"  (steady windows: v_wheel held >= {c['hold_s']} s, |v_wheel| >= "
          f"{c['vmin']} m/s, robot moving, collision monitor not gating)")
    over = 100.0 * (1.0 / c['slope'] - 1.0)
    print(f'  => odometry over-reports forward distance by {over:.2f} %')
    print()
    print('  THIS IS A LOWER BOUND, and the gap is the point:')
    print('   * it captures only the velocity-tracking shortfall of the')
    print('     controller, computed from the COMMAND;')
    print('   * with position_feedback=true the odometry integrates MEASURED')
    print('     wheel rotation, so any wheel that spins faster than the')
    print('     ground moves adds over-report on top, and that component is')
    print('     invisible to every artefact in this repository.')
    print()
    d = load28()
    pool = []
    for _run, _leg, seq in legs28(d):
        ga = gt_along(seq)
        if abs(ga) > 1e-6:
            pool.append(100.0 * sum(r[0] for r in residuals(seq)) / ga)
    print(f"  AMCL's own along-track excess, same runs: "
          f"{st.fmean(pool):.2f} % (n = {len(pool)} legs)")
    print(f'  command-channel bound {over:.2f} %  <  AMCL excess '
          f'{st.fmean(pool):.2f} %')
    print('  The laser correction is restoring (C2-NAV.33, corr -0.71), so')
    print("  the raw odometry excess is at least AMCL's.  The unexplained")
    print(f'  span is roughly {st.fmean(pool) - over:.1f} percentage points,')
    print('  and wheel slip is the candidate that lives exactly there.')


def mode_verdict():
    d = load28()
    pool = []
    for _run, _leg, seq in legs28(d):
        ga = gt_along(seq)
        if abs(ga) > 1e-6:
            pool.append(100.0 * sum(r[0] for r in residuals(seq)) / ga)
    everything = []
    for _run, _leg, seq in legs28(d):
        everything.extend(residuals(seq))
    m, lo, hi = mean_ci([r[0] for r in everything])
    print('C2-NAV.34 verdict')
    print()
    print(f'  along-track residual   {st.fmean(pool):+.2f} % of GT travel, '
          f'positive in {sum(1 for z in pool if z > 0)} of {len(pool)} legs')
    print(f'  per-update signed mean {m:+.5f} m, 95 % CI [{lo:+.5f},{hi:+.5f}]')
    print()
    if lo > 0:
        print('  (A) AMCL RECEIVES MOTION THAT OVER-REPORTS FORWARD TRAVEL,')
        print('      and the two southbound legs are exactly the legs that')
        print('      carry the southward bias.  This is CONSISTENT WITH but')
        print('      DOES NOT PROVE an odometry bias: the residual measured')
        print('      here is (odometry error + laser correction) and nothing')
        print('      in this repository separates the two.')
    else:
        print('  (D) INSUFFICIENT -- the along-track residual CI straddles zero.')
    print()
    print('  The discriminating measurement is NOT in this file and cannot')
    print('  be: odom -> base_footprint is recorded nowhere.  See HANDOFF.md')
    print('  section 9 for the instrumentation and 11 for the falsifier.')


# ------------------------------------------------------------------- build --

def _f(r, k):
    v = r.get(k)
    if v is None or v == '':
        return None
    try:
        return float(v)
    except ValueError:
        return None


def mode_build():
    """Derive the committed bundle from the uncommitted 10 Hz trace CSVs."""
    files = sorted(glob.glob(TRACE_GLOB))
    if not files:
        sys.exit(f'no traces matched {TRACE_GLOB} -- nothing to build')
    out = {'source_glob': TRACE_GLOB, 'n_files': len(files), 'legs': []}
    for f in files:
        with open(f) as fh:
            rows = list(csv.DictReader(fh))
        run = os.path.basename(os.path.dirname(f)).replace('_traces', '')
        leg = os.path.basename(f).replace('_rep0.csv', '')
        t, x, y, th = [], [], [], []
        ex, ey, ep = [], [], []
        amcl = []
        held = None
        for r in rows:
            a, b_, c = _f(r, 'x'), _f(r, 'y'), _f(r, 'yaw')
            if a is None or b_ is None or c is None:
                continue
            ax, ay, ap = _f(r, 'amcl_x'), _f(r, 'amcl_y'), _f(r, 'amcl_yaw')
            if ax is not None:
                held = (ax - (a + WORLD_TO_MAP_X), ay - (b_ + WORLD_TO_MAP_Y),
                        wrap(ap - c))
                amcl.append(held)
            t.append(_f(r, 't_rel')); x.append(a); y.append(b_); th.append(c)
            ex.append(held[0] if held else None)
            ey.append(held[1] if held else None)
            ep.append(held[2] if held else None)
        if len(t) < 10 or len(amcl) < 3:
            continue
        lat_s = lat_a = path = 0.0
        bx = by = 0.0
        px = py = None
        for i in range(1, len(t) - 1):
            dt = t[i + 1] - t[i - 1]
            step = t[i] - t[i - 1]
            if dt <= 0 or step <= 0:
                continue
            vx = (x[i + 1] - x[i - 1]) / dt
            vy = (y[i + 1] - y[i - 1]) / dt
            s, c = math.sin(th[i]), math.cos(th[i])
            vlat = -vx * s + vy * c
            vlon = vx * c + vy * s
            lat_s += vlat * step
            lat_a += abs(vlat) * step
            path += math.hypot(vlon, vlat) * step
            bx += -vlat * (-s) * step
            by += -vlat * c * step
            if ep[i] is not None and ep[i - 1] is not None:
                if px is None:
                    px, py = ex[i - 1], ey[i - 1]
                e = 0.5 * (ep[i] + ep[i - 1])
                px += -e * vy * step
                py += e * vx * step
        out['legs'].append({
            'run': run, 'leg': leg,
            'path': round(path, 5),
            'net_dx': round(x[-1] - x[0], 5),
            'net_dy': round(y[-1] - y[0], 5),
            'lat_signed': round(lat_s, 5),
            'lat_abs': round(lat_a, 5),
            'blind_dx': round(bx, 5),
            'blind_dy': round(by, 5),
            'ex_start': round(amcl[0][0], 5), 'ex_end': round(amcl[-1][0], 5),
            'ey_start': round(amcl[0][1], 5), 'ey_end': round(amcl[-1][1], 5),
            'ep_start': round(amcl[0][2], 5), 'ep_end': round(amcl[-1][2], 5),
            'obs_dx': round(amcl[-1][0] - amcl[0][0], 5),
            'obs_dy': round(amcl[-1][1] - amcl[0][1], 5),
            'head_pred_dy': round(py, 5) if py is not None else 0.0,
            'head_pred_dx': round(px, 5) if px is not None else 0.0,
        })

    # steady-window linear command tracking
    hold_s, vmin = 0.5, 0.10
    num = den = 0.0
    n = 0
    for f in files:
        with open(f) as fh:
            rows = list(csv.DictReader(fh))
        hold = 0
        for i in range(1, len(rows)):
            vw, pw = _f(rows[i], 'v_wheel'), _f(rows[i - 1], 'v_wheel')
            if vw is None:
                hold = 0
                continue
            hold = hold + 1 if pw == vw else 1
            va = _f(rows[i], 'v_act')
            if va is None or hold < int(hold_s * 10) or abs(vw) < vmin:
                continue
            if abs(va) < 0.02 or (rows[i].get('cm_polygon') or '') != '':
                continue
            num += vw * va
            den += vw * vw
            n += 1
    out['cmd_tracking'] = {'slope': round(num / den, 6) if den else None,
                           'n': n, 'hold_s': hold_s, 'vmin': vmin}
    with open(BUNDLE34, 'w') as fh:
        json.dump(out, fh, indent=1, sort_keys=True)
    print(f'wrote {BUNDLE34}: {len(out["legs"])} legs from {len(files)} files')
    print(f'cmd_tracking: {out["cmd_tracking"]}')


# ---------------------------------------------------------------- selftest --

def mode_selftest():
    """Arithmetic and null controls.  No external input at all."""
    n = fail = 0

    def chk(name, cond):
        nonlocal n, fail
        n += 1
        if not cond:
            fail += 1
            print(f'  FAILED: {name}')

    # --- SE(2) algebra ---
    p0 = (1.0, 2.0, 0.3)
    p1 = (1.7, 2.4, 0.9)
    d = body_delta(p0, p1)
    chk('body_delta of identical poses is zero',
        all(abs(v) < 1e-15 for v in body_delta(p0, p0)))
    # a constant left-multiplied SE(2) offset must cancel exactly
    for (ox, oy, oth) in ((2.0, 0.0, 0.0), (-3.1, 4.2, 0.0)):
        def xf(p):
            c, s = math.cos(oth), math.sin(oth)
            return (ox + c * p[0] - s * p[1], oy + s * p[0] + c * p[1],
                    wrap(p[2] + oth))
        d2 = body_delta(xf(p0), xf(p1))
        chk(f'translation offset ({ox},{oy}) cancels in body_delta',
            all(abs(a - b) < 1e-12 for a, b in zip(d, d2)))
    # a rotated frame must ALSO cancel (this is what makes GT-vs-AMCL legal)
    oth = 0.7
    def xr(p):
        c, s = math.cos(oth), math.sin(oth)
        return (c * p[0] - s * p[1], s * p[0] + c * p[1], wrap(p[2] + oth))
    d3 = body_delta(xr(p0), xr(p1))
    chk('rotation offset cancels in body_delta',
        all(abs(a - b) < 1e-12 for a, b in zip(d, d3)))

    chk('wrap(+pi) stays +pi', abs(wrap(math.pi) - math.pi) < 1e-12)
    chk('wrap(3pi) == pi', abs(abs(wrap(3 * math.pi)) - math.pi) < 1e-12)
    chk('wrap(-3.5pi) in range', -math.pi < wrap(-3.5 * math.pi) <= math.pi)

    # --- geometry ---
    pos = wheel_positions_base_link()
    chk('four wheels', len(pos) == 4)
    xs = sorted({round(p[0], 6) for p in pos.values()})
    ys = sorted({round(p[1], 6) for p in pos.values()})
    zs = sorted({round(p[2], 6) for p in pos.values()})
    chk('wheel x at +-0.09', xs == [-0.09, 0.09])
    chk('wheel y at +-0.137', ys == [-0.137, 0.137])
    chk('wheel z at 0.045', zs == [0.045])
    chk('physical track == wheel_separation parameter',
        abs((max(ys) - min(ys)) - WHEEL_SEPARATION) < 1e-9)
    b_eff = WHEEL_SEPARATION * WHEEL_SEPARATION_MULTIPLIER
    chk('b_eff is 0.3014', abs(b_eff - 0.3014) < 1e-9)
    chk('unbiased-eta is 1/1.10',
        abs(1 / WHEEL_SEPARATION_MULTIPLIER - 0.909090909) < 1e-8)

    # --- NULL CONTROL: AMCL replaced by ground truth must give exactly 0 ---
    if os.path.exists(BUNDLE28):
        d28 = load28()
        worst_a = worst_y = 0.0
        legs = 0
        for _run, _leg, seq in legs28(d28):
            legs += 1
            null = [(g, g) for g, _a in seq]
            for r in residuals(null):
                worst_a = max(worst_a, abs(r[0]))
                worst_y = max(worst_y, abs(r[2]))
        chk('null control ran on some legs', legs > 0)
        chk(f'null control along-track == 0 (worst {worst_a:.3e})',
            worst_a < 1e-12)
        chk(f'null control yaw == 0 (worst {worst_y:.3e})', worst_y < 1e-12)
        print(f'  null control: {legs} legs, worst |along| {worst_a:.3e} m, '
              f'worst |yaw| {worst_y:.3e} rad')

        # a SYNTHETIC +8 % along-track over-report must be recovered
        scale = 1.08
        rec = []
        for _run, _leg, seq in legs28(d28):
            syn = [seq[0][0]]
            for i in range(1, len(seq)):
                dg = body_delta(seq[i - 1][0], seq[i][0])
                p = syn[-1]
                c, s = math.cos(p[2]), math.sin(p[2])
                dx, dy = dg[0] * scale, dg[1]
                syn.append((p[0] + c * dx - s * dy, p[1] + s * dx + c * dy,
                            wrap(p[2] + dg[2])))
            pair = list(zip([g for g, _ in seq], syn))
            ga = gt_along(pair)
            if abs(ga) > 1e-6:
                rec.append(sum(r[0] for r in residuals(pair)) / ga)
        if rec:
            got = st.fmean(rec)
            chk(f'synthetic +8 % recovered as {100 * got:.3f} %',
                abs(got - 0.08) < 1e-6)
            print(f'  synthetic injection: asked +8.000000 %, '
                  f'recovered {100 * got:+.6f} %')
    else:
        print(f'  (skipped data-backed checks: {BUNDLE28} absent)')

    print(f'\n{n - fail} passed, {fail} FAILED')
    return 1 if fail else 0


MODES = {
    'geom': mode_geom, 'incr': mode_incr, 'heading': mode_heading,
    'elim': mode_elim, 'cmd': mode_cmd, 'verdict': mode_verdict,
    'build': mode_build, 'selftest': mode_selftest,
}

if __name__ == '__main__':
    if len(sys.argv) != 2 or sys.argv[1] not in MODES:
        sys.exit(f'usage: {sys.argv[0]} {{{"|".join(MODES)}}}')
    sys.exit(MODES[sys.argv[1]]() or 0)
