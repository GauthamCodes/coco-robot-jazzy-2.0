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
"""C2-NAV.43 Part G, offline: how each arm represents the ramp in capture.json.

Restricted to the ramp's central band, |y| <= BAND_Y, so box_obstacle_2's
footprint (y -1.65..-1.15, touching the ramp's south-west corner) cannot be
counted as ramp. For each ramp-facing pose and each capture arm:

  ramp cells      marked cells inside the band's ramp footprint
  first mark x    the smallest world x among them -- where, travelling up the
                  ramp axis, the representation starts; with the surface
                  height there
  coverage        of the band cells that (a) lie inside the 3 x 3 m costmap
                  window, (b) lie inside the camera's current horizontal view
                  within the 2.5 m obstacle range, and (c) carry a surface at
                  or above the depth source's min_obstacle_height (0.05 m),
                  the fraction the arm marked. This is the ramp area the depth
                  source COULD represent; the LiDAR arm is scored over the
                  same cells.

  python3 -P docs/data/c2nav43_ramp.py ~/coco_nav_runs/c2nav43/capture_r03/capture.json
"""

import importlib.util
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location('c2nav43_perception', os.path.join(HERE, 'c2nav43_perception.py'))
P = importlib.util.module_from_spec(spec)
spec.loader.exec_module(P)

BAND_Y = 1.0
MIN_H = 0.05
RES = 0.05


# ramp_edge is not scored here: from (2.0, -2.0) facing north the robot sees
# the wedge's south SIDE FACE, which occludes the band interior for every arm
# (measured: 0 band cells in all three). Its side-face marks are in capture.json.
def analyse(capture, poses=('ramp_entrance', 'ramp_surface')):
    out = {}
    for name in poses:
        e = capture['poses'].get(name)
        if not e or 'gt' not in e:
            continue
        gx, gy, gyaw = e['gt']
        # candidate cells: a 0.05 m lattice over the band, centred like the costmap
        xs = np.arange(P.RAMP['x0'] + RES / 2, P.RAMP['x1'], RES)
        ys = np.arange(-BAND_Y + RES / 2, BAND_Y, RES)
        cx, cy = [a.ravel() for a in np.meshgrid(xs, ys)]
        in_window = (np.abs(cx - gx) <= 1.5) & (np.abs(cy - gy) <= 1.5)
        in_cam = P.sensor_fov(cx, cy, (gx, gy, gyaw))['camera']
        high = P.ramp_height(cx, cy) >= MIN_H
        target = in_window & in_cam & high
        entry = {'robot': [gx, gy, round(gyaw, 4)], 'representable_cells': int(target.sum())}
        for arm in P.CAPTURE_ARMS:
            cells = np.asarray(e[arm].get('cells_xy', []), dtype=float).reshape(-1, 2)
            band = ((cells[:, 0] >= P.RAMP['x0']) & (cells[:, 0] <= P.RAMP['x1'])
                    & (np.abs(cells[:, 1]) <= BAND_Y))
            ramp_cells = cells[band]
            marked = {(round(x / RES), round(y / RES)) for x, y in
                      zip(ramp_cells[:, 0] - RES / 2, ramp_cells[:, 1] - RES / 2)}
            hit = np.array([(round((x - RES / 2) / RES), round((y - RES / 2) / RES)) in marked
                            for x, y in zip(cx[target], cy[target])], dtype=bool)
            first = float(ramp_cells[:, 0].min()) if len(ramp_cells) else None
            entry[arm] = {
                'ramp_cells': int(len(ramp_cells)),
                'first_mark_x': round(first, 3) if first is not None else None,
                'surface_height_at_first_mark_m': (round(float(P.ramp_height(first, 0.0)), 3)
                                                   if first is not None else None),
                'coverage_of_representable': (round(float(hit.mean()), 3) if len(hit) else None),
            }
        out[name] = entry
    return out


def main(argv):
    capture = json.load(open(os.path.expanduser(argv[0])))
    res = analyse(capture)
    print(f'ramp central band |y| <= {BAND_Y} m; representable = in window, in camera view, '
          f'surface >= {MIN_H} m')
    print('| pose | robot | representable cells | arm | ramp cells | first mark x (surface m) | coverage |')
    print('|---|---|---|---|---|---|---|')
    for name, e in res.items():
        for arm in P.CAPTURE_ARMS:
            a = e[arm]
            print(f"| `{name}` | {tuple(e['robot'])} | {e['representable_cells']} | {arm} | "
                  f"{a['ramp_cells']} | {a['first_mark_x']} ({a['surface_height_at_first_mark_m']}) | "
                  f"{a['coverage_of_representable']} |")
    if len(argv) > 1:
        with open(argv[1], 'w') as f:
            json.dump(res, f, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
