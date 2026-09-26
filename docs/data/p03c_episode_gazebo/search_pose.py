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
Where the robot was, and which way it faced, while it searched.

    python3 search_pose.py RUN_DIR [RUN_DIR ...]

From each run's recorder trace (gz ground truth x, y, yaw per row, with
the mission state) and its manifest: the pose at the first row of each
state from VERIFY_CLIMB to APPROACH_TARGET, and the requested target's
range and bearing from it. The camera is at base-x 0.125 on the robot's
axis with a 1.25 rad horizontal field of view (coco_config), so a bearing
beyond about +-35.8 deg is out of frame.
"""

import csv
import json
import math
import os
import sys

STATES = ('VERIFY_CLIMB', 'SEARCH_TARGET', 'RECOVERY', 'APPROACH_TARGET')


def main():
    """Print one block per run directory."""
    for run in sys.argv[1:]:
        manifest = json.load(open(os.path.join(run, 'manifest.json')))
        target = next(t for t in manifest['targets']
                      if t['colour'] == manifest['requested_colour'])
        rows = list(csv.DictReader(open(os.path.join(run, 'cmdpath',
                                                     'trace.csv'))))
        print(f'{os.path.basename(run)}: {target["colour"]} at '
              f'({target["x"]:.4f}, {target["y"]:+.4f}) '
              f'region {target["region_id"]}')
        seen = set()
        window = []
        for row in rows:
            state = row['mission_state']
            if state in ('SEARCH_TARGET', 'RECOVERY'):
                window.append(row)
            if state not in STATES or state in seen or not row['x']:
                continue
            seen.add(state)
            x, y, yaw = (float(row[k]) for k in ('x', 'y', 'yaw'))
            cam_x = x + 0.125 * math.cos(yaw)
            cam_y = y + 0.125 * math.sin(yaw)
            bearing = math.atan2(target['y'] - cam_y,
                                 target['x'] - cam_x) - yaw
            print(f'  first {state:15s} x={x:.3f} y={y:+.3f} '
                  f'yaw={math.degrees(yaw):+6.1f} deg  range '
                  f'{math.hypot(target["x"] - cam_x, target["y"] - cam_y):.3f}'
                  f' m  bearing {math.degrees(bearing):+6.1f} deg')
        if window:
            ys = [float(r['y']) for r in window if r['y']]
            yaws = [math.degrees(float(r['yaw'])) for r in window if r['yaw']]
            xs = [float(r['x']) for r in window if r['x']]
            print(f'  while searching: x {min(xs):.3f}..{max(xs):.3f} '
                  f'y {min(ys):+.3f}..{max(ys):+.3f} '
                  f'yaw {min(yaws):+.1f}..{max(yaws):+.1f} deg '
                  f'({len(window)} rows)')


if __name__ == '__main__':
    main()
