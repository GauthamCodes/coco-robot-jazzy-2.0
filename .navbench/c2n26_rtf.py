import json, os
R = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results')
order = ['c2n26_slow_r1', 'c2n26_bslow_r1', 'c2n26_slow_r2',
         'c2n26_bslow_r2', 'c2n26_slow_r3', 'c2n26_bslow_r3']
print(f'{"#":<3}{"run":<17}{"rtf(med)":>10}{"hz_cmd":>9}{"loop_miss":>11}'
      f'{"stale_drops":>13}{"ok/7":>6}')
for i, t in enumerate(order, 1):
    p = os.path.join(R, t + '.json')
    if not os.path.exists(p):
        continue
    d = json.load(open(p))
    L = d['legs']
    rtf = sorted(x.get('rtf') or 0 for x in L)[len(L)//2]
    hz = sorted(x.get('hz_cmd_vel') or 0 for x in L)[len(L)//2]
    miss = sum(x.get('n_loop_rate_misses') or 0 for x in L)
    stale = sum(x.get('n_stale_cmd_drops') or 0 for x in L)
    ok = sum(1 for x in L if x['status'] == 'SUCCEEDED')
    print(f'{i:<3}{t:<17}{rtf:>10.3f}{hz:>9.2f}{miss:>11}{stale:>13}'
          f'{ok:>6}')
