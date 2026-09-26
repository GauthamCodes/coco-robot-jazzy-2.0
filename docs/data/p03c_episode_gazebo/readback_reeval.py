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

"""
Re-judge each run's RECORDED gz poses with the current read-back check.

    python3 readback_reeval.py RUN_DIR [RUN_DIR ...]

The poses are the ones gz printed during the run (``readback_spawn.json``
``observed``); nothing is re-measured. This exists because the checker
the matrix ran with had no slack on the region's near edge and failed
fixed_blue for a 10 um settle; see README.md.
"""

import json
import math
import os
import sys

from coco_sim.backends import check_instantiation, ObservedPose
from coco_sim.episode import episode_from_json


def main():
    """Print one line per run: as recorded, and as re-judged."""
    for run in sys.argv[1:]:
        with open(os.path.join(run, 'manifest.json')) as f:
            spec = episode_from_json(f.read())
        with open(os.path.join(run, 'readback_spawn.json')) as f:
            recorded = json.load(f)
        observed = {name: ObservedPose(**pose)
                    for name, pose in recorded['observed'].items()}
        tol = recorded['tolerances']
        now = check_instantiation(spec, observed, xy_tol=tol['xy_m'],
                                  z_tol=tol['z_m'],
                                  tilt_tol=tol['tilt_rad'])
        worst_xy = max(math.hypot(m['dx'], m['dy'])
                       for m in now['models'].values())
        worst_dz = max(abs(m['dz']) for m in now['models'].values())
        worst_tilt = max(m['tilt'] for m in now['models'].values())
        print(f'{os.path.basename(run.rstrip("/")):14s} recorded '
              f'ok={recorded["ok"]!s:5s} now ok={now["ok"]!s:5s} '
              f'layout_valid={now["layout_valid"]!s:5s} '
              f'worst xy {worst_xy * 1e6:7.1f} um  |dz| '
              f'{worst_dz * 1e6:6.1f} um  tilt {worst_tilt * 1e6:5.1f} urad'
              + (f'  [{now["layout_error"]}]' if now['layout_error']
                 else ''))


if __name__ == '__main__':
    main()
