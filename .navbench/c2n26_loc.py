"""Localisation error at each leg START, from the nav log + ground truth.

bt_navigator prints "Begin navigating from current location (x, y)" in
the MAP frame -- that is the AMCL estimate. nav_bench records
`start_world`, which is Gazebo ground truth in the WORLD frame. The two
frames differ by a constant offset, verified against every leg goal in
this session: map = world + (MAP_DX, MAP_DY).

So |estimate - truth| at leg start is a real localisation error. It is
NOT the "final AMCL error" the brief asked for -- that is at leg END and
is not recorded anywhere -- and it is not reported as if it were.
"""
import json, os, re, math, sys

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, 'results')
L = os.path.join(HERE, 'logs')
MAP_DX, MAP_DY = 2.0, 0.0
BEGIN = re.compile(r'Begin navigating from current location '
                   r'\(([-0-9.]+), ([-0-9.]+)\) to '
                   r'\(([-0-9.]+), ([-0-9.]+)\)')

RUNS = sys.argv[1:] or [
    'c2n26_slow_r1', 'c2n26_bslow_r1', 'c2n26_slow_r2',
    'c2n26_bslow_r2', 'c2n26_slow_r3', 'c2n26_bslow_r3',
    'c2n25_slow_r1', 'c2n25_slow_r2', 'c2n25_bslow_r1',
    'c2n21_base_r1', 'c2n21_base_r3', 'c2n21_base_r4',
    'c2n21_bbase_r2', 'c2n21_bbase_r3']

print(f'{"run":<17}{"leg":<17}{"status":<10}{"loc_err_start":>14}'
      f'{"gt_start":>20}{"amcl_start":>20}')
worst = []
for tag in RUNS:
    p = os.path.join(R, tag + '.json')
    lg = os.path.join(L, f'nav_{tag}.log')
    if not (os.path.exists(p) and os.path.exists(lg)):
        continue
    d = json.load(open(p))
    begins = BEGIN.findall(open(lg, errors='replace').read())
    # Match each leg to the FIRST begin whose goal equals that leg's goal.
    used = set()
    for L_ in d['legs']:
        gx, gy = L_['goal_world']
        mgx, mgy = gx + MAP_DX, gy + MAP_DY
        hit = None
        for i, (ax, ay, tx, ty) in enumerate(begins):
            if i in used:
                continue
            if abs(float(tx) - mgx) < 0.02 and abs(float(ty) - mgy) < 0.02:
                hit = i
                break
        if hit is None:
            continue
        used.add(hit)
        ax, ay = float(begins[hit][0]), float(begins[hit][1])
        sx, sy = L_['start_world']
        e = math.hypot(ax - (sx + MAP_DX), ay - (sy + MAP_DY))
        print(f'{tag:<17}{L_["scenario"]:<17}{L_["status"][:9]:<10}'
              f'{e:>14.3f}{f"({sx:.2f},{sy:.2f})":>20}'
              f'{f"({ax:.2f},{ay:.2f})":>20}')
        worst.append((e, tag, L_['scenario'], L_['status']))
print()
worst.sort(reverse=True)
print('--- worst 12 localisation errors at leg start, all arms ---')
for e, t, s, st in worst[:12]:
    print(f'  {e:.3f} m  {t}/{s}  {st}')
