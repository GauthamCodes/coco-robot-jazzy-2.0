import json, os
R = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results')
d = json.load(open(os.path.join(R, 'c2n25_slow_r1.json')))
print('TOP:', sorted(d.keys()))
leg = d['legs'][0]
for k in sorted(leg.keys()):
    v = leg[k]
    s = str(v)
    print(f'  {k:<28} {s[:70]}')
