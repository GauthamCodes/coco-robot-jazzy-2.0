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
"""C2-NAV.19: the GOOD/BAD live-costmap comparison C2-NAV.18 could not
complete, plus the recovery-vs-deadlock decomposition it named as the
real open question.

C2-NAV.18 built and validated the /global_costmap/costmap capture
pipeline on three fresh tours, but all three SUCCEEDED, so the central
GOOD-vs-BAD diff was INCONCLUSIVE. It left two things: (a) capture one
real BAD and run the already-built diff, and (b) compare POST-SW-column
behaviour between a recovering GOOD and a frozen BAD using C2-NAV.16's
own dwb_command_window machinery, not new instrumentation.

This session's first fresh tour (`c2n19_tour_r1`) reproduced the
deadlock, so both are done here.

REUSES BY IMPORT, restates nothing:
  - c2nav18_livecostmap.py (as `lc`): the entire costmap pipeline --
    load_costmap_window, alignment, diff_grids/region_diff, route_cost,
    route_cost_matrix, onset_test, replicate_noise_floor, visualize.
    Only the GOOD/BAD tag constants are rebound (that module is
    explicitly written to work for ANY tag pair); its own file is left
    byte-unchanged so C2-NAV.18's committed defaults still document
    C2-NAV.18.
  - c2nav16_compare.py (as `cc`): load_stop_csv, dwb_command_window,
    stall_duration.
  - c2nav15_planwindow.py (as `pw`): load_planwindow, snapshots.
  - c2nav13_heading.py: load_trace.
  - c2nav12_report.py: WAYPOINT, GOAL_SHIFTED, DEADLOCK_POSE, SW_CORNER.

NEW here, because no prior C2-NAV module computes it:
  - `leg_rows`      : stop-probe rows clipped to the enclosure_entry leg
                      in leg-relative seconds (C2-NAV.16's convention,
                      generalised from a +/-1.5 s window to the whole leg).
  - `latch_profile` : what the collision monitor and DWB do AFTER the
                      first PolygonStop activation -- the deadlock itself.
  - `approach_profile`: the closest the run's own lidar ever came to the
                      base, against PolygonStop's 0.25 m radius. This is
                      the measured discriminator.
  - `phase_windows` : the three-phase decomposition of the BAD leg.
  - `rpg_compare`   : RemovePassedGoals conditions in both runs
                      (brief section 13). Nothing is changed; only read.

Usage:
  python3 -P c2nav19_goodbad.py selftest
  python3 -P c2nav19_goodbad.py costmap     # the C2-NAV.18 pipeline, this pair
  python3 -P c2nav19_goodbad.py recovery    # sections 11/15
  python3 -P c2nav19_goodbad.py rpg         # section 13
  python3 -P c2nav19_goodbad.py phases      # the three-phase decomposition
  python3 -P c2nav19_goodbad.py viz
  python3 -P c2nav19_goodbad.py dump <out.json>
  python3 -P c2nav19_goodbad.py all
"""
import json
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import c2nav18_livecostmap as lc                     # noqa: E402
import c2nav16_compare as cc                         # noqa: E402
import c2nav15_planwindow as pw                      # noqa: E402
import c2nav13_heading as h13                        # noqa: E402
from c2nav12_report import (                         # noqa: E402
    DEADLOCK_POSE, GOAL_SHIFTED, SW_CORNER, WAYPOINT,
)

# ---------------------------------------------------------------------
# The pair. GOOD is chosen in section 8 of the brief's own terms; see
# `good_reference_selection()` for the measured justification.
# ---------------------------------------------------------------------
GOOD = 'c2n18_tour_r1'
BAD = 'c2n19_tour_r1'
GOOD_ALTERNATES = ['c2n18_tour_r2', 'c2n18_tour_r3']
LEG = lc.LEG

# Rebind the C2-NAV.18 pipeline onto this session's real pair. That
# module's own file is deliberately NOT edited.
lc.GOOD = GOOD
lc.BAD = BAD

# PolygonStop geometry, from c2nav11_ntp_params.yaml (not re-derived).
POLY_STOP_R = 0.25
POLY_STOP_MIN_POINTS = 4

hdr = lc.hdr


# ---------------------------------------------------------------------
# 0. SELF-TEST -- reproduce known committed facts before trusting new ones.
# ---------------------------------------------------------------------

def self_test():
    ok = lc.self_test()
    hdr('C2-NAV.19 ADDITIONAL SELF-TEST: this session\'s own BAD run')
    tr = h13.load_trace(BAD, LEG)
    last = tr[-1]
    d_dead = 1000.0 * math.hypot(last['x'] - DEADLOCK_POSE[0],
                                 last['y'] - DEADLOCK_POSE[1])
    print(f'  BAD frozen pose ({last["x"]:.4f}, {last["y"]:.4f}) vs '
          f'DEADLOCK_POSE {DEADLOCK_POSE}: {d_dead:.1f} mm  '
          f'want < 250 mm (same pocket)  '
          f'{"PASS" if d_dead < 250 else "FAIL"}')
    ok = ok and d_dead < 250

    # C2-NAV.13's own west-column / south-of-box discriminator, which
    # exists specifically to exclude the box's EAST-face pocket.
    _nm, cx, cy, w, h = h13.BOX1
    x0, y0 = cx - w / 2.0, cy - h / 2.0
    west = last['x'] < x0 + 0.15
    south = last['y'] < y0
    print(f'  BAD frozen pose in WEST column (x < {x0 + 0.15:.3f}): {west}  '
          f'and SOUTH of box (y < {y0:.3f}): {south}  '
          f'{"PASS" if (west and south) else "FAIL"}')
    ok = ok and west and south

    # The BAD run must actually have tripped PolygonStop; a TIMEOUT alone
    # is not the mechanism (brief section 5).
    ap = approach_profile(BAD)
    tripped = ap['min_d_min_base_m'] < POLY_STOP_R
    print(f'  BAD min lidar-to-base {ap["min_d_min_base_m"]:.4f} m < '
          f'PolygonStop radius {POLY_STOP_R} m: {tripped}  '
          f'{"PASS" if tripped else "FAIL"}')
    ok = ok and tripped

    apg = approach_profile(GOOD)
    clear = apg['min_d_min_base_m'] > POLY_STOP_R
    print(f'  GOOD min lidar-to-base {apg["min_d_min_base_m"]:.4f} m > '
          f'{POLY_STOP_R} m (never tripped): {clear}  '
          f'{"PASS" if clear else "FAIL"}')
    ok = ok and clear

    print(f'\nC2-NAV.19 SELF-TEST: {"ALL PASS" if ok else "FAILURES ABOVE"}')
    return ok


# ---------------------------------------------------------------------
# 1. GOOD-reference selection (brief section 8).
# ---------------------------------------------------------------------

def _leg_record(tag, scenario=LEG):
    d = json.load(open(os.path.join(lc.RESULTS_DIR, f'{tag}.json')))
    for lg in d['legs']:
        if lg['scenario'] == scenario:
            return lg
    return None


def _pose_at(tag, t):
    tr = h13.load_trace(tag, LEG)
    return min(tr, key=lambda r: abs(r['t'] - t))


def good_reference_selection():
    """Why c2n18_tour_r1 is the GOOD reference: the brief's own criteria
    (corridor_gate exit pose, T_PRUNE, pose at pruning, waypoint
    distance), measured for every available GOOD candidate."""
    hdr('GOOD-REFERENCE SELECTION (brief section 8), all candidates measured')
    a_bad = lc.alignment(BAD)
    bad_cg = _leg_record(BAD, 'corridor_gate')['end_world']
    bad_pose = _pose_at(BAD, a_bad['t_prune_s'])
    print(f'  BAD {BAD}: corridor_gate exit {bad_cg}, '
          f'T_PRUNE {a_bad["t_prune_s"]:.3f}s, '
          f'pose_at_prune ({bad_pose["x"]:.4f}, {bad_pose["y"]:.4f})')
    rows = {}
    for tag in [GOOD] + GOOD_ALTERNATES:
        a = lc.alignment(tag)
        cg = _leg_record(tag, 'corridor_gate')['end_world']
        p = _pose_at(tag, a['t_prune_s'])
        d_cg = 1000.0 * math.hypot(cg[0] - bad_cg[0], cg[1] - bad_cg[1])
        d_pose = 1000.0 * math.hypot(p['x'] - bad_pose['x'],
                                     p['y'] - bad_pose['y'])
        rows[tag] = dict(
            corridor_gate_exit=cg,
            corridor_gate_delta_mm=round(d_cg, 1),
            t_prune_s=round(a['t_prune_s'], 4),
            t_prune_matches_bad=abs(a['t_prune_s'] - a_bad['t_prune_s']) < 1e-6,
            pose_at_prune=(round(p['x'], 4), round(p['y'], 4)),
            pose_delta_mm=round(d_pose, 1),
            wp_dist_at_prune_m=round(a['t_prune_dist_at_removal_m'], 4),
        )
        print(f'  {tag}: corridor_gate delta {d_cg:7.1f} mm | '
              f'T_PRUNE {a["t_prune_s"]:7.3f}s '
              f'(match={rows[tag]["t_prune_matches_bad"]}) | '
              f'pose delta {d_pose:7.1f} mm | '
              f'wp_dist_at_prune {a["t_prune_dist_at_removal_m"]:.4f} m')
    print(f'\n  SELECTED GOOD = {GOOD}: exact T_PRUNE match and the '
          f'smallest corridor_gate exit delta.')
    return dict(bad=dict(corridor_gate_exit=bad_cg,
                         t_prune_s=round(a_bad['t_prune_s'], 4),
                         pose_at_prune=(round(bad_pose['x'], 4),
                                        round(bad_pose['y'], 4))),
                candidates=rows, selected=GOOD)


# ---------------------------------------------------------------------
# 2. Stop-probe rows for the whole leg (generalises C2-NAV.16's window).
# ---------------------------------------------------------------------

def leg_rows(tag):
    """Every stop-probe row inside the enclosure_entry leg, in
    leg-relative seconds. Same t0 convention C2-NAV.16's
    dwb_command_window uses (stamp - planwindow t0_sim_s); that function
    takes a +/-1.5 s window around one tick, this takes the whole leg."""
    pwd = pw.load_planwindow(tag)
    rows = cc.load_stop_csv(tag)
    if pwd is None or rows is None:
        return []
    t0 = pwd['t0_sim_s']
    t1 = pwd.get('t1_sim_s')
    out = []
    for r in rows:
        if not r.get('stamp'):
            continue
        st = float(r['stamp'])
        if st < t0 or (t1 is not None and st > t1):
            continue

        def f(k):
            v = r.get(k)
            try:
                return float(v)
            except (TypeError, ValueError):
                return None

        def i(k):
            v = r.get(k)
            try:
                return int(float(v))
            except (TypeError, ValueError):
                return 0

        out.append(dict(
            t=round(st - t0, 3), gt_x=f('gt_x'), gt_y=f('gt_y'),
            n_in_stop=i('n_in_stop'), n_in_slow=i('n_in_slow'),
            n_in_limit=i('n_in_limit'), d_min=f('d_min_base_m'),
            r_min=f('r_min_lidar_m'),
            action=(r.get('monitor_action') or '').strip(),
            polygon=(r.get('monitor_polygon') or '').strip(),
            v_nav=f('v_nav'), w_nav=f('w_nav'),
            v_out=f('v_out'), v_wheel=f('v_wheel')))
    out.sort(key=lambda r: r['t'])
    return out


# ---------------------------------------------------------------------
# 3. The measured discriminator: closest lidar approach vs PolygonStop.
# ---------------------------------------------------------------------

def approach_profile(tag):
    """PolygonStop is driven by /scan points inside a 0.25 m polygon --
    NOT by the costmap. This is the run's own closest approach in that
    same quantity, so it is directly comparable to the trigger."""
    rows = leg_rows(tag)
    d = [(r['t'], r['d_min']) for r in rows if r['d_min'] is not None]
    if not d:
        return dict(tag=tag, valid=False)
    tmin, dmin = min(d, key=lambda x: x[1])
    return dict(
        tag=tag, valid=True, n_rows=len(d),
        min_d_min_base_m=round(dmin, 4), at_t_s=tmin,
        margin_to_polystop_mm=round(1000.0 * (dmin - POLY_STOP_R), 1),
        tripped_polystop=dmin < POLY_STOP_R,
        n_rows_below_270mm=len([1 for _t, v in d if v < 0.270]),
        n_rows_below_260mm=len([1 for _t, v in d if v < 0.260]),
        max_n_in_stop=max(r['n_in_stop'] for r in rows),
    )


# ---------------------------------------------------------------------
# 4. Recovery vs deadlock (brief section 11), and the DWB / collision
#    monitor cross-check (brief section 15).
# ---------------------------------------------------------------------

def latch_profile(tag):
    """After the FIRST PolygonStop activation: does anything ever move
    again, does the monitor ever release, and what is DWB asking for?"""
    rows = leg_rows(tag)
    stop = [r for r in rows if r['action'] == 'STOP']
    if not stop:
        return dict(tag=tag, latched=False, n_rows=len(rows),
                    n_STOP_rows=0)
    t_first = stop[0]['t']
    after = [r for r in rows if r['t'] >= t_first]
    vn = [r['v_nav'] for r in after if r['v_nav'] is not None]
    vw = [r['v_wheel'] for r in after if r['v_wheel'] is not None]
    wn = [r['w_nav'] for r in after if r['w_nav'] is not None]
    dm = [r['d_min'] for r in after if r['d_min'] is not None]
    escapes = [r['t'] for r in after if r['action'] != 'STOP']
    moving = [r for r in after
              if r['v_wheel'] is not None and abs(r['v_wheel']) > 1e-3]
    return dict(
        tag=tag, latched=True, n_rows=len(rows), n_STOP_rows=len(stop),
        frac_STOP=round(len(stop) / len(rows), 4),
        T_FIRST_STOP_s=t_first,
        first_stop_pose=(round(stop[0]['gt_x'], 4), round(stop[0]['gt_y'], 4)),
        first_stop_n_in_stop=stop[0]['n_in_stop'],
        first_stop_v_nav=stop[0]['v_nav'],
        first_stop_v_wheel=stop[0]['v_wheel'],
        n_rows_after_latch=len(after),
        # DWB (and the behaviour server) keep asking:
        v_nav_after_latch=dict(
            min=round(min(vn), 4), max=round(max(vn), 4),
            n_negative=len([v for v in vn if v < -1e-6]),
            n_positive=len([v for v in vn if v > 1e-6]),
            n_zero=len([v for v in vn if abs(v) <= 1e-6])),
        w_nav_after_latch=dict(min=round(min(wn), 4), max=round(max(wn), 4)),
        # ...and the wheels never move again:
        v_wheel_after_latch=dict(min=round(min(vw), 4), max=round(max(vw), 4),
                                 n_rows_moving=len(moving)),
        n_rows_monitor_released=len(escapes),
        first_release_t_s=escapes[0] if escapes else None,
        d_min_after_latch=(round(min(dm), 4), round(max(dm), 4)),
        d_min_band_mm=round(1000.0 * (max(dm) - min(dm)), 2),
        n_in_stop_after_latch=(min(r['n_in_stop'] for r in after),
                               max(r['n_in_stop'] for r in after)),
    )


def report_recovery():
    hdr('RECOVERY vs DEADLOCK (brief section 11) and DWB / collision-'
        'monitor cross-check (section 15)')
    out = {}
    for tag, label in ((GOOD, 'GOOD'), (BAD, 'BAD ')):
        ap = approach_profile(tag)
        lp = latch_profile(tag)
        out[tag] = dict(approach=ap, latch=lp)
        print(f'  {label} {tag}:')
        print(f'    closest lidar-to-base : {ap["min_d_min_base_m"]} m '
              f'at t={ap["at_t_s"]}s  '
              f'(margin vs PolygonStop {POLY_STOP_R} m: '
              f'{ap["margin_to_polystop_mm"]:+.1f} mm)')
        print(f'    rows < 270 mm / < 260 mm: '
              f'{ap["n_rows_below_270mm"]} / {ap["n_rows_below_260mm"]}   '
              f'max points in stop polygon: {ap["max_n_in_stop"]} '
              f'(threshold {POLY_STOP_MIN_POINTS})')
        if not lp['latched']:
            print('    PolygonStop NEVER activated -- no latch, '
                  'no recovery needed')
        else:
            print(f'    T_FIRST_STOP          : {lp["T_FIRST_STOP_s"]}s at '
                  f'{lp["first_stop_pose"]}')
            print(f'    after the latch       : '
                  f'{lp["n_rows_after_latch"]} rows, monitor released on '
                  f'{lp["n_rows_monitor_released"]} of them; wheels moved on '
                  f'{lp["v_wheel_after_latch"]["n_rows_moving"]}')
            print(f'    DWB kept commanding   : v_nav in '
                  f'[{lp["v_nav_after_latch"]["min"]}, '
                  f'{lp["v_nav_after_latch"]["max"]}] '
                  f'({lp["v_nav_after_latch"]["n_negative"]} reverse rows), '
                  f'w_nav in [{lp["w_nav_after_latch"]["min"]}, '
                  f'{lp["w_nav_after_latch"]["max"]}]')
            print(f'    lidar distance froze  : '
                  f'{lp["d_min_after_latch"]} m -- a '
                  f'{lp["d_min_band_mm"]} mm band for the rest of the leg')
    return out


# ---------------------------------------------------------------------
# 5. The three-phase decomposition of the BAD leg.
# ---------------------------------------------------------------------

def window_stats(tag, lo, hi, label):
    rows = [r for r in leg_rows(tag) if lo <= r['t'] <= hi]
    if not rows:
        return dict(label=label, tag=tag, n_rows=0)
    acts = {}
    for r in rows:
        acts[r['action']] = acts.get(r['action'], 0) + 1
    vn = [r['v_nav'] for r in rows if r['v_nav'] is not None]
    vw = [r['v_wheel'] for r in rows if r['v_wheel'] is not None]
    dm = [r['d_min'] for r in rows if r['d_min'] is not None]
    moving = [r for r in rows
              if r['v_wheel'] is not None and abs(r['v_wheel']) > 1e-3]
    return dict(
        label=label, tag=tag, window_s=(lo, hi), n_rows=len(rows),
        monitor_actions=acts,
        max_n_in_stop=max(r['n_in_stop'] for r in rows),
        d_min_range_m=(round(min(dm), 4), round(max(dm), 4)) if dm else None,
        v_nav_max=round(max(vn), 4), v_nav_min=round(min(vn), 4),
        frac_v_nav_zero=round(
            len([v for v in vn if abs(v) <= 1e-6]) / len(vn), 4),
        v_wheel_max=round(max(vw), 4),
        frac_rows_wheel_moving=round(len(moving) / len(rows), 4),
    )


def path_len(tag, t_lo, t_hi):
    tr = [r for r in h13.load_trace(tag, LEG) if t_lo <= r['t'] <= t_hi]
    if len(tr) < 2:
        return 0.0
    return round(sum(math.hypot(tr[i + 1]['x'] - tr[i]['x'],
                                tr[i + 1]['y'] - tr[i]['y'])
                     for i in range(len(tr) - 1)), 3)


def phase_windows():
    """The BAD leg is three mechanisms in sequence, not one. Boundaries
    are taken from the run's own measured events, not chosen by eye:
    T_PRUNE, T_FIRST_STOP, and the end of the worst_crawl the bench
    itself recorded."""
    hdr('BAD LEG PHASE DECOMPOSITION -- three mechanisms, in sequence')
    a_bad = lc.alignment(BAD)
    lp = latch_profile(BAD)
    crawl = _leg_record(BAD)['worst_crawl']
    t_prune = a_bad['t_prune_s']
    t_crawl0 = crawl['t_rel_s']
    t_crawl1 = round(t_crawl0 + crawl['crawl_len_s'], 2)
    t_stop = lp['T_FIRST_STOP_s']
    t_end = h13.load_trace(BAD, LEG)[-1]['t']
    wins = [
        window_stats(BAD, 0.0, t_prune, f'BAD approach (0 -> T_PRUNE {t_prune:.3f}s)'),
        window_stats(BAD, t_crawl0, t_crawl1,
                     f'BAD worst_crawl ({t_crawl0} -> {t_crawl1}s)'),
        window_stats(BAD, t_crawl1, t_stop,
                     f'BAD creep west ({t_crawl1} -> {t_stop}s)'),
        window_stats(BAD, t_stop, t_end, f'BAD latched ({t_stop} -> {t_end}s)'),
        window_stats(GOOD, t_crawl0, t_crawl1,
                     f'GOOD same window ({t_crawl0} -> {t_crawl1}s)'),
    ]
    for w in wins:
        print(f'  {w["label"]}')
        print(f'    monitor={w["monitor_actions"]} max_in_stop={w["max_n_in_stop"]}')
        print(f'    d_min={w["d_min_range_m"]} m  '
              f'v_nav in [{w["v_nav_min"]}, {w["v_nav_max"]}] '
              f'({100*w["frac_v_nav_zero"]:.1f}% exactly zero)  '
              f'wheels moving {100*w["frac_rows_wheel_moving"]:.1f}% of rows')
    lens = dict(
        bad_0_to_prune_m=path_len(BAD, 0.0, t_prune),
        bad_prune_to_stop_m=path_len(BAD, t_prune, t_stop),
        bad_after_latch_m=path_len(BAD, t_stop, 1e9),
        good_0_to_prune_m=path_len(GOOD, 0.0, t_prune),
        good_prune_to_end_m=path_len(GOOD, t_prune, 1e9),
    )
    print(f'\n  path length: {json.dumps(lens)}')
    print(f'  BAD worst_crawl (from the bench itself): {crawl["crawl_len_s"]}s '
          f'at {tuple(round(v, 3) for v in crawl["pose_world"][:2])}, '
          f'dwb_chosen_vx={crawl["dwb_chosen_vx"]}, '
          f'dwb_illegal_frac={crawl["dwb_illegal_frac"]}, '
          f'scan_min={crawl["scan_min_m"]} m, had_plan={crawl["had_plan"]}, '
          f'monitor={crawl["collision_monitor"]}')
    good_crawl = _leg_record(GOOD)['worst_crawl']
    print(f'  GOOD worst_crawl                       : '
          f'{good_crawl["crawl_len_s"]}s at '
          f'{tuple(round(v, 3) for v in good_crawl["pose_world"][:2])}, '
          f'dwb_chosen_vx={good_crawl["dwb_chosen_vx"]}')
    return dict(windows=wins, path_len_m=lens, bad_worst_crawl=crawl,
                good_worst_crawl=good_crawl)


# ---------------------------------------------------------------------
# 6. RemovePassedGoals comparison (brief section 13). Read only.
# ---------------------------------------------------------------------

def rpg_compare():
    hdr('REMOVE-PASSED-GOALS COMPARISON (brief section 13) -- read only, '
        'nothing changed')
    out = {}
    for tag, label in ((GOOD, 'GOOD'), (BAD, 'BAD ')):
        a = lc.alignment(tag)
        p = _pose_at(tag, a['t_prune_s'])
        out[tag] = dict(
            t_prune_s=round(a['t_prune_s'], 4),
            dist_to_waypoint_at_removal_m=round(
                a['t_prune_dist_at_removal_m'], 4),
            waypoint_reached=(a['t_prune_dist_at_removal_m'] <= 0.25),
            pose_at_prune=(round(p['x'], 4), round(p['y'], 4)),
            yaw_deg_at_prune=round(math.degrees(p['yaw']), 2),
            first_bad_plan_t_s=a['first_bad_plan_t_s'],
            sw_commit_t_s=a['sw_commit_t_s'],
            costmap_nearest_t_prune=a['costmap_nearest_t_prune'],
        )
        print(f'  {label} {tag}: T_PRUNE {out[tag]["t_prune_s"]}s, '
              f'waypoint distance at removal '
              f'{out[tag]["dist_to_waypoint_at_removal_m"]} m '
              f'(reached={out[tag]["waypoint_reached"]}), '
              f'pose {out[tag]["pose_at_prune"]} @ '
              f'{out[tag]["yaw_deg_at_prune"]} deg')
    g, b = out[GOOD], out[BAD]
    dd = round(b['dist_to_waypoint_at_removal_m']
               - g['dist_to_waypoint_at_removal_m'], 4)
    dp = round(1000.0 * math.hypot(
        b['pose_at_prune'][0] - g['pose_at_prune'][0],
        b['pose_at_prune'][1] - g['pose_at_prune'][1]), 1)
    dy = round(b['yaw_deg_at_prune'] - g['yaw_deg_at_prune'], 2)
    print(f'\n  SAME tick in both ({g["t_prune_s"]}s == {b["t_prune_s"]}s): '
          f'{g["t_prune_s"] == b["t_prune_s"]}')
    print(f'  BAD is pruned {dd} m FURTHER from the waypoint than GOOD; '
          f'pose delta {dp} mm, yaw delta {dy} deg')
    out['delta'] = dict(dist_delta_m=dd, pose_delta_mm=dp, yaw_delta_deg=dy,
                        same_tick=(g['t_prune_s'] == b['t_prune_s']))
    return out


# ---------------------------------------------------------------------
# 7. Roll-ups.
# ---------------------------------------------------------------------

def report_costmap():
    """The whole C2-NAV.18 pipeline, on this session's real pair."""
    return dict(meta=lc.report_meta(),
                alignment=lc.report_alignment(),
                diff=lc.report_diff(),
                route_cost=lc.route_cost_matrix(),
                onset=lc.onset_test('sw_corner'),
                replicate_noise_floor=lc.replicate_noise_floor(lc.GOOD_TAGS))


def summary():
    hdr(f'C2-NAV.19 SUMMARY: GOOD ({GOOD}) vs BAD ({BAD})')
    return dict(good_reference=good_reference_selection(),
                costmap=report_costmap(),
                rpg=rpg_compare(),
                recovery=report_recovery(),
                phases=phase_windows())


def dump(out_path):
    record = dict(good_tag=GOOD, bad_tag=BAD, leg=LEG,
                  good_alternates=GOOD_ALTERNATES,
                  polygon_stop_radius_m=POLY_STOP_R,
                  polygon_stop_min_points=POLY_STOP_MIN_POINTS,
                  self_test_pass=self_test())
    record.update(summary())
    with open(out_path, 'w') as f:
        json.dump(record, f, indent=1, default=str)
    print(f'\nwrote {out_path}')
    return record


def visualize(out_path=None):
    """Four panels: GOOD live costmap + its plan, BAD live costmap + its
    plan, the signed cell difference, and the measured discriminator --
    lidar-to-base distance against PolygonStop's 0.25 m radius."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.patches import Circle, Rectangle

    out_path = out_path or os.path.join(
        lc.REPO_ROOT, 'docs', 'images', 'c2nav19_goodbad.png')

    lo_g = lc.load_costmap_window(GOOD)
    lo_b = lc.load_costmap_window(BAD)
    a_g, a_b = lc.alignment(GOOD), lc.alignment(BAD)
    ig = a_g['costmap_nearest_t_prune']['idx']
    ib = a_b['costmap_nearest_t_prune']['idx']

    def extent_world(lo):
        x0 = lo['origin_x'] - lc.WORLD_TO_MAP[0]
        y0 = lo['origin_y'] - lc.WORLD_TO_MAP[1]
        return (x0, x0 + lo['width'] * lo['resolution'],
                y0, y0 + lo['height'] * lo['resolution'])

    def tick_plan(tag):
        _raw, snap = pw.snapshots(tag, quiet=True)
        best = min(snap, key=lambda a: abs(a['ts_offset_from_t0_s']
                                           - lc.EXPECT_TICK_S))
        return [tuple(p) for p in _raw['snapshots'][snap.index(best)]
                ['poses_world']]

    fig, axes = plt.subplots(1, 4, figsize=(26, 6.4), dpi=150)
    _nm, cx, cy, bw, bh = h13.BOX1

    for ax, lo, idx, tag, label in ((axes[0], lo_g, ig, GOOD, 'GOOD'),
                                    (axes[1], lo_b, ib, BAD, 'BAD')):
        ax.imshow(lo['data'][idx], origin='lower', extent=extent_world(lo),
                  cmap='viridis', vmin=0, vmax=100, interpolation='nearest')
        pts = tick_plan(tag)
        ax.plot([p[0] for p in pts], [p[1] for p in pts], 'r-', lw=2.0,
                label='/plan at the tick')
        tr = h13.load_trace(tag, LEG)
        ax.plot([r['x'] for r in tr], [r['y'] for r in tr], 'w-', lw=1.2,
                alpha=0.85, label='GT track')
        ax.add_patch(Rectangle((cx - bw / 2, cy - bh / 2), bw, bh, fill=False,
                               ec='orange', lw=1.6))
        ax.add_patch(Circle(SW_CORNER, POLY_STOP_R, fill=False, ec='red',
                            lw=1.4, ls='--'))
        ax.plot(*WAYPOINT, 'c^', ms=9, label='WAYPOINT')
        ax.plot(*GOAL_SHIFTED, 'y*', ms=14, label='goal')
        ap = approach_profile(tag)
        align = a_g if label == 'GOOD' else a_b
        dt = align['costmap_nearest_t_prune']['dt_from_target_s']
        ax.set_title(f'{label}  {tag}\n'
                     f'live costmap idx={idx}, |dt| from T_PRUNE {dt}s\n'
                     f'closest lidar-to-base {ap["min_d_min_base_m"]} m '
                     f'({ap["margin_to_polystop_mm"]:+.1f} mm vs PolygonStop)',
                     fontsize=10)
        ax.set_xlim(-4.2, -2.0)
        ax.set_ylim(0.2, 3.4)
        ax.legend(loc='lower left', fontsize=7)

    # Panel 3: signed difference at the matched tick pair.
    d = lo_b['data'][ib].astype(np.int16) - lo_g['data'][ig].astype(np.int16)
    im = axes[2].imshow(d, origin='lower', extent=extent_world(lo_g),
                        cmap='coolwarm', vmin=-100, vmax=100,
                        interpolation='nearest')
    axes[2].add_patch(Rectangle((cx - bw / 2, cy - bh / 2), bw, bh, fill=False,
                                ec='k', lw=1.6))
    axes[2].add_patch(Circle(SW_CORNER, 0.6, fill=False, ec='k', lw=1.2,
                             ls='--'))
    wd = lc.diff_grids(lo_g, ig, lo_b, ib)
    rd = lc.region_diff(lo_g, ig, lo_b, ib, SW_CORNER, 0.6)
    axes[2].set_title(
        f'BAD - GOOD live costmap at the tick\n'
        f'whole grid {wd["n_differing_cells"]}/{wd["n_cells"]} '
        f'({100*wd["n_differing_cells"]/wd["n_cells"]:.1f}%), '
        f'SW corner {rd["n_differing_cells"]}/{rd["n_cells_in_region"]} '
        f'({100*rd["n_differing_cells"]/rd["n_cells_in_region"]:.1f}%)\n'
        f'C2-NAV.18 GOOD-vs-GOOD noise floor: 7.1-8.5% / up to 40.2%',
        fontsize=10)
    axes[2].set_xlim(-4.2, -2.0)
    axes[2].set_ylim(0.2, 3.4)
    fig.colorbar(im, ax=axes[2], fraction=0.046)

    # Panel 4: the measured discriminator.
    for tag, colour, label in ((GOOD, 'tab:green', 'GOOD'),
                               (BAD, 'tab:red', 'BAD')):
        rows = leg_rows(tag)
        axes[3].plot([r['t'] for r in rows],
                     [r['d_min'] for r in rows], color=colour, lw=1.1,
                     label=f'{label} {tag}')
    axes[3].axhline(POLY_STOP_R, color='k', ls='--', lw=1.4,
                    label=f'PolygonStop radius {POLY_STOP_R} m')
    lp = latch_profile(BAD)
    axes[3].axvline(lp['T_FIRST_STOP_s'], color='tab:red', ls=':', lw=1.2,
                    label=f'T_FIRST_STOP {lp["T_FIRST_STOP_s"]}s')
    axes[3].axvline(a_b['t_prune_s'], color='tab:blue', ls=':', lw=1.2,
                    label=f'T_PRUNE {a_b["t_prune_s"]:.3f}s')
    axes[3].set_xlabel('leg-relative time (s)')
    axes[3].set_ylabel('min lidar distance to base (m)')
    axes[3].set_ylim(0.20, 0.85)
    axes[3].set_title('THE MEASURED DISCRIMINATOR\n'
                      f'GOOD closest {approach_profile(GOOD)["min_d_min_base_m"]} m '
                      f'(+{approach_profile(GOOD)["margin_to_polystop_mm"]:.1f} mm), '
                      f'BAD {approach_profile(BAD)["min_d_min_base_m"]} m '
                      f'({approach_profile(BAD)["margin_to_polystop_mm"]:+.1f} mm)',
                      fontsize=10)
    axes[3].legend(loc='upper right', fontsize=7)
    axes[3].grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_path, bbox_inches='tight')
    print(f'wrote {out_path}')
    return out_path


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'all'
    if cmd == 'selftest':
        return 0 if self_test() else 1
    if cmd == 'goodref':
        good_reference_selection()
        return 0
    if cmd == 'costmap':
        report_costmap()
        return 0
    if cmd == 'recovery':
        report_recovery()
        return 0
    if cmd == 'rpg':
        rpg_compare()
        return 0
    if cmd == 'phases':
        phase_windows()
        return 0
    if cmd == 'viz':
        visualize(sys.argv[2] if len(sys.argv) > 2 else None)
        return 0
    if cmd == 'dump':
        dump(sys.argv[2] if len(sys.argv) > 2
             else os.path.join(_HERE, 'c2nav19_bench.json'))
        return 0
    if cmd == 'all':
        self_test()
        summary()
        return 0
    print(__doc__)
    return 2


if __name__ == '__main__':
    sys.exit(main())
