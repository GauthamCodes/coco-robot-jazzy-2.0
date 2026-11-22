#!/usr/bin/env python3
"""Where did enclosure_entry START in each committed experiment?

C2-NAV.7 ran `--only enclosure_entry,enclosure_exit`, so its entry began
at the SPAWN. C2-NAV.5's and C2-NAV.8's entry is leg 6 of 7 and begins
wherever corridor_gate left the robot. If the approach differs, a two-leg
result does not transfer to the tour -- which is the whole reason
C2-NAV.8 is a whole-tour validation and not three more two-leg runs.
"""
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.normpath(os.path.join(HERE, '..', '..', 'docs', 'data'))


def show(path, want_tag_prefix):
    doc = json.load(open(os.path.join(DATA, path)))
    for run in doc['runs']:
        if not run['tag'].startswith(want_tag_prefix):
            continue
        for lg in run['legs']:
            if lg['scenario'] != 'enclosure_entry':
                continue
            s = lg.get('start_world')
            g = lg.get('goal_world')
            print(f'  {run["tag"]:<20} start ({s[0]:+.3f}, {s[1]:+.3f})  '
                  f'goal ({g[0]:+.3f}, {g[1]:+.3f})  '
                  f'{lg["status"]:<10} {lg.get("duration_sim_s")} s  '
                  f'err {lg.get("final_goal_err_m")} m')


print('C2-NAV.7 (two-leg runs, entry starts at the spawn):')
show('c2nav7_bench.json', 'c2n7')
print()
print('C2-NAV.5 CSF-65 tour (entry is leg 6, starts where corridor_gate '
      'ended):')
show('c2nav5_bench.json', 'c2n5_tour_csf65')
print()
print('C2-NAV.5 two-leg-style enclosure runs (entry alone, from the spawn):')
show('c2nav5_bench.json', 'c2n5_enc_csf65')
