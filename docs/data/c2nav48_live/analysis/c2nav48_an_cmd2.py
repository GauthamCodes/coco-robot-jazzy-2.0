#!/usr/bin/env python3
"""C2-NAV.48 command-path safety, compact: the four metrics the sprint must
report -- bypass, stop authority, gated-zero residual, stale drops."""
import json
import os
import sys

ROWS = [
    ('bypass (wheel==raw controller)', 'bypass_source.wheel_matches_raw_controller'),
    ('bypass rows, nav-owned', 'nav_active_rows.bypass_source.wheel_matches_raw_controller'),
    ('PolygonStop rows', 'stop_breach.stop_rows'),
    ('  ...with wheels DRIVEN', 'stop_breach.stop_rows_wheels_driven'),
    ('gated_zero_moving (nav-owned)', 'nav_active_rows.monitor_authority.gated_zero_moving'),
    ('  worst wheel while gated 0 (m/s)', 'nav_active_rows.monitor_authority.worst_wheel_while_gated_zero_ms'),
    ('monitor exceeded (nav-owned)', 'nav_active_rows.monitor_authority.exceeded'),
    ('stale drops (wheel_eq_raw_only)', 'smoother.wheel_eq_raw_only'),
]


def flat(d, p=''):
    o = {}
    for k, v in (d or {}).items():
        if isinstance(v, dict):
            o.update(flat(v, p + k + '.'))
        else:
            o[p + k] = v
    return o


def load(path):
    p = os.path.join(path, 'cmdpath', 'summary.json')
    if not os.path.exists(p):
        return None
    with open(p) as f:
        return flat(json.load(f))


H = os.path.expanduser('~/coco_nav_runs/c2nav46_m6')
N = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1
                       else '~/coco_nav_runs/c2nav48_repro')

cols = []
for r in ('r2_blue', 'r3_blue'):
    cols.append((f'46:{r.replace("_blue", "")}', load(os.path.join(H, r))))
if os.path.isdir(N):
    for d in sorted(os.listdir(N)):
        s = load(os.path.join(N, d))
        if s:
            cols.append((f'48:{d.split("_")[0]}{d.split("_")[1][:1]}', s))

w = 13
print(' ' * 36 + ''.join(f'{c[0]:>{w}}' for c in cols))
for label, key in ROWS:
    vals = []
    for _, s in cols:
        v = s.get(key) if s else None
        vals.append('-' if v is None else (f'{v:g}' if isinstance(v, (int, float)) else str(v)))
    print(f'{label:<36}' + ''.join(f'{v:>{w}}' for v in vals))
