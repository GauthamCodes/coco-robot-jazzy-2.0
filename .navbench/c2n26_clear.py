"""min_clearance_m and final error by leg and arm -- ALL legs, any status."""
import json
import os
import statistics as st

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, 'results')
ARMS = {
    'baseline': ['c2n21_base_r1', 'c2n21_base_r3', 'c2n21_base_r4',
                 'c2n21_bbase_r2', 'c2n21_bbase_r3'],
    'C2-NAV.25': ['c2n25_slow_r1', 'c2n25_slow_r2', 'c2n25_bslow_r1'],
    'C2-NAV.26': ['c2n26_slow_r1', 'c2n26_slow_r2', 'c2n26_slow_r3',
                  'c2n26_bslow_r1', 'c2n26_bslow_r2', 'c2n26_bslow_r3'],
}
LEGS = ['open_space', 'wall_adjacent', 'wall_parallel', 'obstacle_corner',
        'corridor_gate', 'enclosure_entry', 'enclosure_exit']


def load(tags):
    out = []
    for t in tags:
        p = os.path.join(R, t + '.json')
        if not os.path.exists(p):
            continue
        for L in json.load(open(p))['legs']:
            out.append((t, L))
    return out


print('min_clearance_m by leg -- ALL legs regardless of status')
print(f'{"leg":<17}' + ''.join(f'{a:>28}' for a in ARMS))
print(f'{"":<17}' + ''.join(f'{"n     min  median":>28}' for _ in ARMS))
print('-' * 101)
for leg in LEGS:
    row = ''
    for a, tags in ARMS.items():
        v = [L['min_clearance_m'] for t, L in load(tags)
             if L['scenario'] == leg and L.get('min_clearance_m') is not None]
        row += (f'{len(v):>8}{min(v):>10.3f}{st.median(v):>10.3f}'
                if v else f'{"-":>28}')
    print(f'{leg:<17}{row}')

print()
print('wall_adjacent in detail -- the leg whose end pose broke slow_r2')
print(f'{"arm":<11}{"run":<17}{"status":<10}{"clear_m":>9}{"err_m":>8}'
      f'{"end_world":>20}')
for a, tags in ARMS.items():
    for t, L in load(tags):
        if L['scenario'] != 'wall_adjacent':
            continue
        ew = L['end_world']
        print(f'{a:<11}{t:<17}{L["status"][:9]:<10}'
              f'{L["min_clearance_m"]:>9.3f}{L["final_goal_err_m"]:>8.3f}'
              f'{f"({ew[0]:.3f},{ew[1]:.3f})":>20}')
