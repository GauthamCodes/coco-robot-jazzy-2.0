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
Stage C generator evidence: determinism, validity and variation, offline.

    python3 docs/data/p03c_episode_gazebo/episode_evidence.py [N]

Needs coco_sim and coco_config importable (an overlay, or both source
package directories on sys.path). Prints; the committed .txt is its output.
"""

from collections import Counter
import hashlib
import json
import sys

from coco_config.robot import (FIXED_REGION_MAP, lane_for_colour,
                               region_by_id, TARGET_ROW_X)
from coco_sim.backends import GazeboBackend, IsaacBackend
from coco_sim.episode import (approach_corridor_blocked,
                              compat_mission_inputs, generate_episode,
                              InvalidEpisode, LEVELS, rebind, region_area,
                              REGION_LATERAL_LIMIT, validate_episode)


def sha(text):
    """Return the first 16 hex of sha256(text)."""
    return hashlib.sha256(text.encode()).hexdigest()[:16]


def main():
    """Print every block of the evidence."""
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 10000

    print('== same seed, generated twice (manifest sha256[:16]) ==')
    for level in LEVELS:
        a = generate_episode(seed=1827, level=level).to_json()
        b = generate_episode(seed=1827, level=level).to_json()
        print(f'{level:9s} {sha(a)} {sha(b)} identical={a == b}')

    print(f'\n== {n} seeds per level ==')
    for level in LEVELS:
        valid = in_region = blocked = 0
        maps, requested_moved = Counter(), 0
        dx, dy = [], []
        for seed in range(n):
            try:
                spec = generate_episode(seed=seed, level=level)
                validate_episode(spec)
                valid += 1
            except InvalidEpisode:
                continue
            maps[tuple(sorted(spec.region_map().items()))] += 1
            req = spec.target(spec.requested_colour)
            if req.region_id != FIXED_REGION_MAP[req.colour]:
                requested_moved += 1
            ok = True
            for t in spec.targets:
                region = region_by_id(t.region_id)
                (xl, xh), (yl, yh) = region_area(region, t.diameter)
                ok &= xl <= t.x <= xh and yl <= t.y <= yh
                dx.append(t.x - region.row_x)
                dy.append(t.y - region.lane_y)
                for o in spec.targets:
                    if o is not t and approach_corridor_blocked(t, o):
                        blocked += 1
            in_region += ok
        print(f'{level:9s} valid {valid}/{n}  inside own region {in_region}/{n}'
              f'  corridor-blocked pairs {blocked}'
              f'  distinct colour->region maps {len(maps)}'
              f'  requested colour off its frozen lane {requested_moved}/{n}')
        print(f'{"":9s} dx from row [{min(dx):+.4f}, {max(dx):+.4f}] m'
              f'  dy from lane [{min(dy):+.4f}, {max(dy):+.4f}] m')

    print('\n== the POSITION area, derived ==')
    print(f'REGION_LATERAL_LIMIT = {REGION_LATERAL_LIMIT} m')
    for colour in ('red', 'yellow'):
        spec = generate_episode(seed=0)
        t = spec.target(colour)
        (xl, xh), (yl, yh) = region_area(region_by_id(t.region_id),
                                         t.diameter)
        print(f'{colour:6s} d={t.diameter * 1000:.0f} mm  x [{xl:.4f}, '
              f'{xh:.4f}]  y [{yl:+.4f}, {yh:+.4f}]  (row {TARGET_ROW_X},'
              f' lane {lane_for_colour(colour):+.2f})')

    print('\n== one episode, both sides of the boundary ==')
    spec = generate_episode(seed=2, level='positions', requested_colour='red')
    print('task_view():          ', json.dumps(spec.task_view()))
    print('compat_mission_inputs:', json.dumps(compat_mission_inputs(spec)))
    print('manifest targets:     ', json.dumps(
        [{k: t[k] for k in ('colour', 'region_id', 'x', 'y')}
         for t in spec.manifest()['targets']]))

    print('\n== one manifest, two backends ==')
    gz = GazeboBackend().translate(spec)
    isaac = IsaacBackend().translate(rebind(spec, backend='isaac'))
    for s, p in zip(gz.targets, isaac.targets):
        print(f'{s.name:14s} gz ({s.x:.4f}, {s.y:+.4f}, {s.z:.4f})  '
              f'isaac {p.path} {tuple(round(v, 4) for v in p.position)}')
    print('isaac missing world pieces:', isaac.missing)


if __name__ == '__main__':
    main()
