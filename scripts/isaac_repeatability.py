#!/usr/bin/env python3
# Copyright 2026 Gautham Anil
# SPDX-License-Identifier: Apache-2.0
"""Ten sequential fresh sessions; retain every failure, never retry a trial."""
import argparse
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    parser.add_argument('--raw-evidence', required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    rows = []
    repo = Path(__file__).resolve().parents[1]
    for index in range(1, 11):
        out = args.output / f'trial{index:02d}'
        command = [
            sys.executable,
            str(repo / 'scripts/isaac_smoke.py'),
            str(out),
            '--nav',
            '--domain',
            str(90 + index if index <= 5 else 78),
            '--raw-evidence',
            str(args.raw_evidence),
        ]
        if index > 2:
            command += ['--rviz']
        if index > 8:
            command += ['--gui']
        with (args.output / f'trial{index:02d}_runner.log').open('w') as stream:
            code = subprocess.call(command, stdout=stream, stderr=subprocess.STDOUT)
        result = json.loads((out / 'result.json').read_text())
        nav = (
            json.loads((out / 'navigation.json').read_text())
            if (out / 'navigation.json').exists()
            else {}
        )
        row = {
            'run': index,
            'command': command,
            'exit': code,
            **{
                key: result.get(key)
                for key in (
                    'passed',
                    'failure_class',
                    'clean_shutdown',
                    'auxiliary_clean_shutdown',
                    'zero_motion_verified',
                    'elapsed_wall_s',
                )
            },
            **{
                key: nav.get(key)
                for key in (
                    'startup_ready',
                    'action_server_ready',
                    'goal_accepted',
                    'status',
                    'final_error_map_m',
                    'goal_elapsed_wall_s',
                    'goal_elapsed_sim_s',
                )
            },
            'planner_succeeded': any(
                key.startswith('compute_path_to_pose:') and value == 4
                for key, value in nav.get('action_statuses', {}).items()
            ),
            'controller_succeeded': any(
                key.startswith('follow_path:') and value == 4
                for key, value in nav.get('action_statuses', {}).items()
            ),
        }
        rows.append(row)
        (args.output / 'summary.json').write_text(json.dumps(rows, indent=2))
        print(json.dumps(row), flush=True)
        if result.get('leftover_processes'):
            raise SystemExit(
                'Stopped batch: owned process cleanup failed; inspect session evidence'
            )
    raise SystemExit(0 if all(row['passed'] for row in rows) else 1)


if __name__ == '__main__':
    main()
