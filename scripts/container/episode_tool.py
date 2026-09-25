#!/usr/bin/env python3
"""The episode specification from the command line (coco_sim.episode).

usage:
  episode_tool.py generate --seed S [--level fixed] [--colour C]  -> manifest JSON
  episode_tool.py task-view MANIFEST                              -> the robot's view
  episode_tool.py record MANIFEST MISSION_RESULT COMMIT           -> EpisodeResult JSON

`record` turns one mission run (extract_mission_result.py's result.json)
into a validated EpisodeResult -- outcome, failure reason, timings named
by their clock -- and then asks check_reproducible(): does the seed still
generate the recorded episode? Nothing here edits the generator.
"""
import argparse
import json
import sys

from coco_sim.episode import (check_reproducible, episode_from_json,
                              generate_episode, record_result)


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    g = sub.add_parser('generate')
    g.add_argument('--seed', type=int, required=True)
    g.add_argument('--level', default='fixed')
    g.add_argument('--colour', default=None)
    t = sub.add_parser('task-view')
    t.add_argument('manifest')
    r = sub.add_parser('record')
    r.add_argument('manifest')
    r.add_argument('mission_result')
    r.add_argument('commit')
    a = ap.parse_args()

    if a.cmd == 'generate':
        print(generate_episode(seed=a.seed, level=a.level,
                               requested_colour=a.colour).to_json())
    elif a.cmd == 'task-view':
        print(json.dumps(episode_from_json(open(a.manifest).read()).task_view(),
                         sort_keys=True))
    else:
        spec = episode_from_json(open(a.manifest).read())
        m = json.load(open(a.mission_result))
        outcome_line = m.get('outcome') or ''
        if outcome_line.startswith('COMPLETE'):
            outcome, reason = 'complete', ''
        elif outcome_line.startswith('ABORT'):
            outcome, reason = 'aborted', m.get('reason') or 'ABORT'
        else:
            outcome, reason = 'void', 'the mission never reached a terminal state'
        # Wall only: /mission/state's `elapsed` is per-state, not a mission
        # sim duration, and no other sim-time duration is recorded.
        timings = {}
        if m.get('wall_duration_s') is not None:
            timings['mission_wall_s'] = m['wall_duration_s']
        measurements = {k: m.get(k) for k in (
            'localization_recoveries', 'home_arrival_error_m',
            'pre_ramp_arrival_error_m', 'lift_mm', 'magnet_detached')}
        res = record_result(spec, outcome, failure_reason=reason,
                            timings=timings, software_commit=a.commit,
                            measurements=measurements)
        out = json.loads(res.to_json())
        out['reproducible_from_seed'] = check_reproducible(res)
        print(json.dumps(out, indent=2, sort_keys=True))


if __name__ == '__main__':
    sys.exit(main())
