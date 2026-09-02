#!/usr/bin/env python3
# Copyright 2026 Gautham Anil
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""C2-NAV.4 pre-drive falsifier: what a different cost_scaling_factor
would have done to a costmap that was actually measured.

C2-NAV.3 left one question: is there any command from the stall pose whose
BaseObstacle cost is small enough that the MapGrid critics can pay for it?
Today there is not -- the cheapest cell on the transformed plan is 60, and
60 x 8.0 = 480 against a winning total near 36.

The single variable under test is
`local_costmap.inflation_layer.cost_scaling_factor`. This predicts its
effect EXACTLY, without a simulator, because the inflation layer is a
lookup table over integer cell offsets:

  nav2_costmap_2d::InflationLayer::computeCost      inflation_layer.hpp
      distance == 0                     -> 254  LETHAL_OBSTACLE
      distance * res <= inscribed       -> 253  INSCRIBED_INFLATED_OBSTACLE
      else  (unsigned char) 252 * exp(-CSF * (distance * res - inscribed))

  distance is `cached_distances_[dx][dy]` = hypot(dx, dy) in CELLS, over
  integer offsets with hypot <= cell_inflation_radius = ceil(radius/res),
  and the BFS assigns each cell the offset of its NEAREST source.

So the achievable cost values are a finite set, one per reachable
(dx, dy) ring. Reading a cost back therefore identifies the ring exactly
-- no continuous inversion, no rounding assumption -- and the cost the
same cell would carry under a different CSF is that same ring evaluated
at the new CSF. The identification is checked against the captured
costmap before anything is predicted from it: if a single observed cost
is not in the generated set, the remap is not applied.

`inflation_radius` is HELD FIXED. Only CSF moves. A cell at cost 0 under
the baseline is a cell beyond the inflation radius (verified below: at
CSF 5.0 no ring inside 0.5 m rounds to 0), so it stays 0.

Having remapped the grid, it rescores DWB's whole decision through the
C2-NAV.3 rebuild -- every (vx, wz) the sampler actually evaluated,
regenerated at the captured pose and scored to COMPLETION, so the argmin
is DWB's own argmin and no partial short-circuit total is mistaken for a
decomposition.

Usage:
  python3 c2nav4_costfield.py <capture>_stall.json [snapshot_index]
                              [--csf 5,10,20,...] [--json OUT]
"""
import json
import math
import statistics
import sys

from c2nav3_mapgrid import (Costmap, MapGrid, seeds_goal_dist,
                            seeds_path_dist)
from c2nav3_probe import (FPD, MAX_VEL_X, MIN_VEL_X, VX_SAMPLES,
                          generate, score, short_circuited_total)

LETHAL = 254
INSCRIBED = 253
NO_INFORMATION = 255

# The local_costmap the capture was taken UNDER. 5.0 is the C2-NAV.0
# baseline; a capture taken with a candidate parameter file must be read
# with that candidate's value (--base-csf), or the ring identification
# below fails loudly rather than silently mis-scaling every prediction.
BASE_CSF = 5.0
INFLATION_RADIUS = 0.5
RES = 0.05
ROBOT_RADIUS = 0.20

FOOTPRINT_PADDING = 0.01     # nav2_costmap_2d Costmap2DROS default


def _sign(x):
    """nav2_costmap_2d::sign -- note sign(0) is +1, not 0."""
    return -1.0 if x < 0 else 1.0


def make_footprint_from_radius(radius, n=16):
    """nav2_costmap_2d::makeFootprintFromRadius: a regular 16-gon of
    CIRCUMradius `radius`."""
    return [(math.cos(2 * math.pi * i / n) * radius,
             math.sin(2 * math.pi * i / n) * radius) for i in range(n)]


def pad_footprint(fp, padding):
    """nav2_costmap_2d::padFootprint -- per-AXIS, by sign, not radial."""
    return [(x + _sign(x) * padding, y + _sign(y) * padding)
            for (x, y) in fp]


def _dist_to_segment(px, py, ax, ay, bx, by):
    ab2 = (bx - ax) ** 2 + (by - ay) ** 2
    if ab2 == 0.0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / ab2
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (ax + t * (bx - ax)), py - (ay + t * (by - ay)))


def inscribed_of(fp):
    """nav2_costmap_2d::calculateMinAndMaxDistances -- the minimum distance
    from the origin to any footprint SEGMENT, i.e. the apothem."""
    n = len(fp)
    return min(_dist_to_segment(0.0, 0.0, fp[i][0], fp[i][1],
                                fp[(i + 1) % n][0], fp[(i + 1) % n][1])
               for i in range(n))


# The inflation layer uses LayeredCostmap's INSCRIBED radius, and that is
# not robot_radius. Costmap2DROS turns robot_radius into a regular 16-gon
# of circumradius 0.20, then pads it by `footprint_padding` (default
# 0.01, per axis and by sign), and the inscribed radius is the padded
# polygon's apothem. All three readings are offered and the one that
# reproduces the captured costmap is the one used; guessing between them
# shifts every predicted cost by a few per cent, silently.
INSCRIBED_CANDIDATES = [
    ('padded 16-gon apothem, padding 0.01',
     inscribed_of(pad_footprint(make_footprint_from_radius(ROBOT_RADIUS),
                                FOOTPRINT_PADDING))),
    ('unpadded 16-gon apothem R*cos(pi/16)',
     ROBOT_RADIUS * math.cos(math.pi / 16)),
    ('robot_radius R', ROBOT_RADIUS),
]


def compute_cost(distance_cells, csf, inscribed, res=RES):
    """InflationLayer::computeCost, verbatim, distance in CELLS."""
    if distance_cells == 0:
        return LETHAL
    if distance_cells * res <= inscribed:
        return INSCRIBED
    factor = math.exp(-1.0 * csf * (distance_cells * res - inscribed))
    return int((INSCRIBED - 1) * factor)      # C truncation to uchar


def rings(inflation_radius=INFLATION_RADIUS, res=RES):
    """Every distance the BFS can assign: hypot(dx, dy) over integer cell
    offsets inside cell_inflation_radius = cellDistance(inflation_radius).

    cellDistance is ceil(radius / res), and enqueue() rejects
    distance > cell_inflation_radius_, so the bound is inclusive.
    """
    cir = math.ceil(inflation_radius / res)
    out = set()
    for dx in range(cir + 1):
        for dy in range(cir + 1):
            d = math.hypot(dx, dy)
            if d <= cir:
                out.add(round(d, 12))
    return sorted(out)


def cost_table(csf, inscribed):
    """{ring distance -> cost} at this cost_scaling_factor."""
    return {d: compute_cost(d, csf, inscribed) for d in rings()}


def identify_inscribed(costmap_values):
    """Pick the inscribed radius whose generated cost set contains every
    inflated cost actually present. Returns (chosen, present, report)."""
    present = sorted({c for c in costmap_values
                      if c not in (0, INSCRIBED, LETHAL, NO_INFORMATION)})
    report = []
    chosen = None
    for label, r in INSCRIBED_CANDIDATES:
        generated = set(cost_table(BASE_CSF, r).values())
        missing = [c for c in present if c not in generated]
        report.append((label, r, len(present), len(missing), missing[:8]))
        if not missing and chosen is None:
            chosen = (label, r)
    return chosen, present, report


def remap(csf_new, inscribed):
    """{baseline cost -> (min new cost, max new cost)} over every ring that
    produces that baseline cost. A one-to-one baseline cost gives an exact
    answer; a collision is reported as a range rather than hidden."""
    base = cost_table(BASE_CSF, inscribed)
    new = cost_table(csf_new, inscribed)
    buckets = {}
    for d, c in base.items():
        buckets.setdefault(c, []).append(new[d])
    out = {c: (min(v), max(v)) for c, v in buckets.items()}
    out[0] = (0, 0)                 # beyond the inflation radius: unchanged
    out[INSCRIBED] = (INSCRIBED, INSCRIBED)
    out[LETHAL] = (LETHAL, LETHAL)
    out[NO_INFORMATION] = (NO_INFORMATION, NO_INFORMATION)
    return out


def ambiguity(table):
    """How many baseline costs map to more than one new cost, and the
    worst spread. A remap is only exact where this is zero."""
    amb = [(c, lo, hi) for c, (lo, hi) in table.items() if lo != hi]
    return len(amb), max((hi - lo for _c, lo, hi in amb), default=0)


def remapped_meta(meta, table, pick='max'):
    """A costmap dict with every cost remapped. `pick` chooses which end of
    an ambiguous bucket to use; 'max' is the conservative one -- it never
    claims the field is cheaper than it might be."""
    i = 1 if pick == 'max' else 0
    out = dict(meta)
    out['data'] = [table[c][i] for c in meta['data']]
    return out


def describe(vals):
    if not vals:
        return {}
    s = sorted(vals)
    return {
        'n': len(s), 'min': s[0], 'max': s[-1],
        'median': statistics.median(s),
        'p25': s[max(0, int(0.25 * (len(s) - 1)))],
        'p75': s[max(0, int(0.75 * (len(s) - 1)))],
        'n_zero': sum(1 for v in s if v == 0),
        'n_le_3': sum(1 for v in s if v <= 3),
    }


def plan_costs(cm, plan):
    out = []
    for (x, y) in plan:
        c = cm.world_to_map(x, y)
        out.append(cm.cost(*c) if c else None)
    return [c for c in out if c is not None]


def fit_start_velocity(s, rx, ry, rth):
    """The C2-NAV.3 probe's own fit, so the regenerated trajectories are
    the ones DWB scored. Returns (start_vel, worst pose error)."""
    def worst_err(sv):
        w = 0.0
        for pr in s['probes'].values():
            vx, wz = pr['evaluated']
            regen = generate((rx, ry, rth), sv, (vx, wz))
            dwbp = pr['poses']
            n = min(len(regen), len(dwbp))
            for i in range(n):
                w = max(w, math.dist(regen[i][:2], dwbp[i][:2]))
        return w

    best_sv, best_err = (0.0, 0.0), worst_err((0.0, 0.0))
    lo_x, hi_x, lo_t, hi_t = -0.31, 0.31, -1.05, 1.05
    for _ in range(6):
        for a in [lo_x + i * (hi_x - lo_x) / 20.0 for i in range(21)]:
            for b in [lo_t + i * (hi_t - lo_t) / 20.0 for i in range(21)]:
                e = worst_err((a, b))
                if e < best_err:
                    best_err, best_sv = e, (a, b)
        sx = (hi_x - lo_x) / 20.0
        st = (hi_t - lo_t) / 20.0
        lo_x, hi_x = best_sv[0] - sx, best_sv[0] + sx
        lo_t, hi_t = best_sv[1] - st, best_sv[1] + st
    return best_sv, best_err


def build_grids(cm, plan, rx, ry):
    pdg = MapGrid(cm)
    pdg.seeds = seeds_path_dist(cm, plan)[0]
    pdg.propagate()
    gdg = MapGrid(cm)
    gdg.seeds = seeds_goal_dist(cm, plan)[0]
    gdg.propagate()
    ang = math.atan2(plan[-1][1] - ry, plan[-1][0] - rx)
    ga_plan = list(plan)
    ga_plan[-1] = (plan[-1][0] + FPD * math.cos(ang),
                   plan[-1][1] + FPD * math.sin(ang))
    gag = MapGrid(cm)
    gag.seeds = seeds_goal_dist(cm, ga_plan)[0]
    gag.propagate()
    return pdg, gdg, gag


def rescore_all(cm, plan, start, start_vel, samples):
    """Every (vx, wz) DWB evaluated, regenerated and scored to COMPLETION.

    DWB's short-circuit is an evaluation-order optimisation: it changes
    which totals are REPORTED, never which trajectory has the lowest
    complete total. So the argmin here is DWB's argmin, and every number
    in it is a full decomposition.
    """
    rx, ry, _rth = start
    pdg, gdg, gag = build_grids(cm, plan, rx, ry)
    rows = []
    for (vx, wz) in samples:
        poses = generate(start, start_vel, (vx, wz))
        st, total, raw, why = score(cm, pdg, gdg, gag, poses)
        rows.append({
            'vx': vx, 'wz': wz,
            'status': st, 'total': total, 'raw': dict(raw), 'why': why,
            'end_m': math.dist(poses[-1][:2], (rx, ry)),
        })
    legal = [r for r in rows if r['status'] == 'OK']
    win = min(legal, key=lambda r: r['total']) if legal else None
    return rows, win


def sweep_table(cm, plan, start, start_vel, wz_fixed, best_total):
    rx, ry, _rth = start
    pdg, gdg, gag = build_grids(cm, plan, rx, ry)
    vxs = [MIN_VEL_X + i * (MAX_VEL_X - MIN_VEL_X) / (VX_SAMPLES - 1)
           for i in range(VX_SAMPLES)]
    out = []
    for vx in vxs:
        poses = generate(start, start_vel, (vx, wz_fixed))
        st, total, raw, why = score(cm, pdg, gdg, gag, poses)
        n, _partial, aborted = short_circuited_total(raw, best_total)
        out.append({
            'vx': vx, 'end_m': math.dist(poses[-1][:2], (rx, ry)),
            'status': st, 'total': total, 'raw': dict(raw), 'why': why,
            'n_critics_dwb_would_score': n, 'short_circuited': aborted,
            'complete': st == 'OK' and not aborted,
        })
    return out


def main():
    args = list(sys.argv[1:])
    path = args[0] if args and not args[0].startswith('--') else \
        '.navbench/results/c2n3_stall.json'
    which = 0
    csfs = [2.5, 5.0, 10.0, 15.0, 20.0, 30.0, 40.0, 50.0]
    out_json = None
    i = 1 if (args and not args[0].startswith('--')) else 0
    while i < len(args):
        if args[i] == '--csf':
            csfs = [float(v) for v in args[i + 1].split(',')]
            i += 2
        elif args[i] == '--json':
            out_json = args[i + 1]
            i += 2
        elif args[i] == '--base-csf':
            # The cost_scaling_factor the CAPTURE was taken under. Getting
            # this wrong does not produce a subtly wrong answer: the ring
            # identification stops reproducing the grid and the run aborts.
            global BASE_CSF
            BASE_CSF = float(args[i + 1])
            i += 2
        else:
            which = int(args[i])
            i += 1

    d = json.load(open(path))
    s = d['snapshots'][which]
    meta = s['costmap']
    cm0 = Costmap(meta)
    plan = [(p[0], p[1]) for p in s['transformed_plan']['poses']]

    starts = {tuple(round(v, 6) for v in pr['poses'][0])
              for pr in s['probes'].values()}
    if len(starts) != 1:
        print(f'start poses disagree: {starts}')
        return 1
    rx, ry, rth = next(iter(starts))

    print(f'=== {path} snapshot {which} ===')
    print(f"robot ({rx:.4f}, {ry:.4f}) yaw {rth:.4f} in "
          f"{meta['frame_id']}, cell {cm0.world_to_map(rx, ry)}, cost "
          f"{cm0.cost(*cm0.world_to_map(rx, ry))}")
    print(f"distance to goal {s['dist_to_goal_world']:.4f} m, heading error "
          f"{s['heading_error_to_goal_deg']:+.2f} deg")
    print(f"DWB chose vx {s['chosen']['vx']:.4f} wz {s['chosen']['wz']:.4f} "
          f"total {s['chosen']['total']:.2f} "
          f"({s['chosen']['n_critics']} critics)")
    dwb_best = float(s['chosen']['total'])

    # -- 1. identify the inflation table --------------------------------
    print()
    print('=== identifying the inflation table against the captured grid ===')
    chosen, present, report = identify_inscribed(meta['data'])
    print(f'  distinct inflated costs present: {len(present)}')
    for label, r, _npres, nmiss, sample in report:
        verdict = 'REPRODUCES ALL' if nmiss == 0 else \
            f'{nmiss} not generated, e.g. {sample}'
        print(f'  inscribed {r:.6f} m  ({label:<28}) {verdict}')
    if chosen is None:
        print('  NO CANDIDATE REPRODUCES THE GRID -- not remapping')
        return 1
    label, inscribed = chosen
    print(f'  using inscribed = {inscribed:.6f} m ({label})')

    base_tab = cost_table(BASE_CSF, inscribed)
    zero_inside = [dd for dd, c in base_tab.items()
                   if c == 0 and dd * RES <= INFLATION_RADIUS]
    print(f'  rings inside inflation_radius that round to cost 0 at '
          f'CSF {BASE_CSF}: {len(zero_inside)}')
    if zero_inside:
        print('  *** WARNING: at this base CSF a cost-0 cell is NOT '
              'necessarily beyond the inflation radius, so remapping cost '
              '0 is')
        print('  *** AMBIGUOUS and the predictions below understate the '
              'cost at lower CSF. Remap FROM the 5.0 baseline, where this '
              'count is 0.')
    else:
        print('  (so cost 0 == beyond the inflation radius, and CSF '
              'cannot move it: the remap of cost 0 is exact)')

    # -- 2. the baseline picture ----------------------------------------
    pc0 = plan_costs(cm0, plan)
    print()
    print(f'=== transformed plan: {len(plan)} poses ===')
    print(f'  baseline CSF {BASE_CSF}: {describe(pc0)}')

    # ray straight ahead: the endpoint cells a forward command lands in
    ray = []
    for k in range(0, 21):
        dd = k * 0.05
        x, y = rx + dd * math.cos(rth), ry + dd * math.sin(rth)
        c = cm0.world_to_map(x, y)
        ray.append((dd, cm0.cost(*c) if c else None))

    # -- 3. generator validation ----------------------------------------
    start_vel, gen_err = fit_start_velocity(s, rx, ry, rth)
    print()
    print('=== generator validation (C2-NAV.3 fit) ===')
    print(f'  fitted start velocity vx {start_vel[0]:+.4f} '
          f'wz {start_vel[1]:+.4f}, worst pose error {gen_err:.6f} m')
    if gen_err > 5e-3:
        print('  GENERATOR DOES NOT MATCH -- predictions below are not valid')
        return 1

    samples = [(a['vx'], a['wz']) for a in s['all']]
    print(f'  {len(samples)} evaluated (vx, wz) samples replayed per CSF')

    # -- 4. per-CSF prediction ------------------------------------------
    results = {}
    print()
    hdr = ('  {:>6} {:>6} {:>6} {:>7} {:>6} {:>6} {:>5} | {:>7} {:>7} '
           '{:>8} {:>6} {:>8}')
    print("=== per cost_scaling_factor: the plan, and DWB's own decision "
          'replayed ===')
    print('  plan cost over the transformed plan; then the argmin over all '
          f'{len(samples)} evaluated samples,')
    print('  every one scored to COMPLETION (no short-circuit partials).')
    print(hdr.format('CSF', 'min', 'p25', 'median', 'p75', 'max', '<=3',
                     'win_vx', 'win_wz', 'win_tot', 'BaseOb', 'zero_tot'))
    for csf in csfs:
        tab = remap(csf, inscribed)
        n_amb, worst_amb = ambiguity(tab)
        meta_n = remapped_meta(meta, tab, pick='max')
        cm = Costmap(meta_n)
        pc = plan_costs(cm, plan)
        rows, win = rescore_all(cm, plan, (rx, ry, rth), start_vel, samples)
        # The zero-velocity reference is the best trajectory that does not
        # translate, over every wz the sampler produced -- not the single
        # (0, 0) sample, which the sampler need not contain.
        zero = min((r for r in rows
                    if abs(r['vx']) < 1e-9 and r['status'] == 'OK'),
                   key=lambda r: r['total'], default=None)
        zero_tot = zero['total'] if zero else None
        sw = sweep_table(cm, plan, (rx, ry, rth), start_vel, 0.0,
                         win['total'] if win else dwb_best)
        results[csf] = {
            'plan': describe(pc), 'plan_costs': pc,
            'remap_ambiguous_costs': n_amb, 'remap_worst_spread': worst_amb,
            'winner': None if win is None else {
                'vx': win['vx'], 'wz': win['wz'], 'total': win['total'],
                'raw': win['raw'], 'end_m': win['end_m']},
            'zero_total': zero_tot,
            'n_legal': sum(1 for r in rows if r['status'] == 'OK'),
            'n_illegal': sum(1 for r in rows if r['status'] != 'OK'),
            'sweep_wz0': sw,
            'ray': [(dd, (None if c is None else tab[c][1]))
                    for dd, c in ray],
        }
        st = describe(pc)
        print(hdr.format(
            f'{csf:g}', st['min'], st['p25'], st['median'], st['p75'],
            st['max'], st['n_le_3'],
            f"{win['vx']:.4f}" if win else 'none',
            f"{win['wz']:.4f}" if win else 'none',
            f"{win['total']:.2f}" if win else 'n/a',
            f"{win['raw'].get('BaseObstacle', float('nan')):.0f}"
            if win else 'n/a',
            f'{zero_tot:.2f}' if zero_tot is not None else 'n/a'))

    print()
    print('  remap ambiguity (baseline costs mapping to >1 new cost): ' +
          ', '.join(f"{c:g}:{results[c]['remap_ambiguous_costs']}"
                    f"/{results[c]['remap_worst_spread']}" for c in csfs))

    # -- 5. the controlled wz = 0 sweep, per CSF ------------------------
    for csf in csfs:
        sw = results[csf]['sweep_wz0']
        print()
        print(f'=== controlled sweep, wz held at 0.0000, CSF {csf:g} ===')
        print('  {:>7} {:>7} {:>9} {:>7} {:>7} {:>7} {:>7} {:>7} {:>6} '
              '{:>20}'.format('vx', 'end_m', 'total', 'BaseOb', 'GoalAl',
                              'PathAl', 'PathDs', 'GoalDs', 'ncrit',
                              'verdict'))
        winner = results[csf]['winner']
        wt = winner['total'] if winner else None
        for r in sw:
            raw = r['raw']
            if r['status'] != 'OK':
                verdict = 'ILLEGAL ' + (r['why'] or '').split(':')[0]
            elif r['short_circuited']:
                verdict = f"short-circuit @{r['n_critics_dwb_would_score']}"
            elif wt is not None and abs(r['total'] - wt) < 1e-9:
                verdict = 'WINS overall'
            else:
                verdict = 'complete, loses'
            print('  {:>7.4f} {:>7.3f} {:>9} {:>7} {:>7} {:>7} {:>7} {:>7} '
                  '{:>6} {:>20}'.format(
                      r['vx'], r['end_m'],
                      f"{r['total']:.2f}" if r['total'] is not None else 'n/a',
                      f"{raw.get('BaseObstacle', float('nan')):.0f}",
                      f"{raw.get('GoalAlign', float('nan')):.0f}",
                      f"{raw.get('PathAlign', float('nan')):.0f}",
                      f"{raw.get('PathDist', float('nan')):.0f}",
                      f"{raw.get('GoalDist', float('nan')):.0f}",
                      r['n_critics_dwb_would_score'], verdict))

    # -- 6. the straight-ahead ray --------------------------------------
    print()
    print('=== cost straight ahead of the robot, per CSF ===')
    print('  {:>7}'.format('ahead') + ''.join(f'{c:>8g}' for c in csfs))
    for k, (dd, _c) in enumerate(ray):
        vals = [results[c]['ray'][k][1] for c in csfs]
        print('  {:>7.2f}'.format(dd) +
              ''.join('     n/a' if v is None else f'{v:>8}' for v in vals))

    if out_json:
        with open(out_json, 'w') as f:
            json.dump({
                'source': path, 'snapshot': which,
                'inscribed_radius': inscribed, 'inscribed_label': label,
                'base_csf': BASE_CSF, 'inflation_radius': INFLATION_RADIUS,
                'robot': {'x': rx, 'y': ry, 'yaw': rth},
                'dist_to_goal': s['dist_to_goal_world'],
                'heading_error_deg': s['heading_error_to_goal_deg'],
                'dwb_chosen': s['chosen']['vx'],
                'dwb_chosen_total': dwb_best,
                'generator_error_m': gen_err,
                'start_velocity': list(start_vel),
                'baseline_plan': describe(pc0),
                'results': {str(k): v for k, v in results.items()},
            }, f)
        print(f'\nwrote {out_json}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
