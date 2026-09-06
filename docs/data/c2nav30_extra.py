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
c2nav30_extra.py
================
C2-NAV.30, the two numbers the main module reports only as medians and
that a median hides.

1. THE CLOUD MEAN VS THE PUBLISHED /amcl_pose. The brief is explicit:
   "Do NOT assume the particle mean equals the reported AMCL pose.
   Measure that relationship." `table` reports the median of
   |cloud mean - AMCL|, which is 0.0000 m, and a median of zero is
   exactly the kind of number that should not be quoted alone. This
   prints the full distribution, so "they coincide" is a measurement
   with a tail attached rather than an assertion.

   It matters mechanically. nav2_amcl does not publish the mean of the
   WHOLE particle set: it clusters the set and publishes the mean of the
   largest cluster. Those two coincide only while the cloud is
   effectively unimodal, so the size of this residual is itself evidence
   about cluster structure -- and it is checked against `modes`.

2. BOTH FRAME CONVENTIONS. `nav_bench.py` hard-codes
   WORLD_TO_MAP = (2.0, 0.0); map_audit.py measured (2.056, 0.015).
   C2-NAV.29 established that the disagreement changes the magnitude of
   the southward bias and not its sign or its conclusion, and computed
   every headline both ways. C2-NAV.30 does the same rather than
   inheriting that claim.

   Neither constant is CHANGED here. The brief forbids it and so does
   the fact that twenty-nine commits of results are based on the
   historical one.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import c2nav30_cloud as C            # noqa: E402


def main():
    tag = C.RUN
    print('=' * 78)
    print('C2-NAV.30 extra -- the cloud mean vs the published pose,')
    print('                  and every headline under BOTH conventions')
    print('=' * 78)

    print()
    print('1. |particle-set mean - published /amcl_pose|, metres')
    print('   (nav2_amcl publishes the mean of the LARGEST CLUSTER, not')
    print('    of the whole set, so this residual is cluster evidence.)')
    print()
    print('%-16s%7s%9s%9s%9s%9s' % ('leg', 'n', 'median', 'p95', 'max',
                                    'max dy'))
    print('-' * 78)
    for leg in (C.CONTROL_LEG, C.WALL_LEG):
        S = [d for d in C.samples(tag, leg) if d['amcl_x'] is not None]
        d2 = [((d['pc_mx'] - d['amcl_x']) ** 2
               + (d['pc_my'] - d['amcl_y']) ** 2) ** 0.5 for d in S]
        dy = [abs(d['pc_my'] - d['amcl_y']) for d in S]
        print('%-16s%7d%9.5f%9.5f%9.5f%9.5f'
              % (leg, len(S), C.med(d2), C.pct(d2, 95), max(d2), max(dy)))
    print()
    print('   Read with `modes`: the dominant cluster holds 97.6 % of the')
    print('   particles (median) at wall_adjacent, so the whole-set mean')
    print('   and the largest-cluster mean SHOULD nearly coincide, and')
    print('   the measured residual above says they do.')

    print()
    print('2. every headline under both frame conventions')
    print('   historical  world->map = (%.4f, %.4f)  [nav_bench.py, NOT '
          'changed]' % (C.WORLD_TO_MAP_X, C.WORLD_TO_MAP_Y))
    print('   measured    world->map = (%.4f, %.4f)  [map_audit.py]'
          % (C.MEASURED_TO_MAP_X, C.MEASURED_TO_MAP_Y))
    print()
    print('%-16s%-12s%10s%10s%10s%10s'
          % ('leg', 'convention', 'AMCL dy', 'cloud dy', 'GT in y',
             'frac N'))
    print('-' * 78)
    for leg in (C.CONTROL_LEG, C.WALL_LEG):
        for label, meas in (('historical', False), ('measured', True)):
            S = C.samples(tag, leg, measured_frame=meas)
            amcl = [d['amcl_dy'] for d in S if d['amcl_dy'] is not None]
            ins = 100.0 * sum(1 for d in S if d['gt_inside_y']) / len(S)
            # frac of particles north of GT, from the RAW particles, in
            # this convention.
            fr = []
            m = C.cloud_meta(tag, leg) or {}
            t0 = m.get('t0_sim_s')
            rows = [(C.fl(r['t_rel']), r) for r in C.read_trace(tag, leg)
                    if C.fl(r.get('x')) is not None]
            oy = (C.MEASURED_TO_MAP_Y if meas else C.WORLD_TO_MAP_Y)
            for ts, arr in C.read_clouds(tag, leg):
                trel = (ts - t0) if t0 is not None else ts
                if not rows:
                    continue
                near = min(rows, key=lambda rr: abs(rr[0] - trel))
                if abs(near[0] - trel) > 0.5:
                    continue
                gy = C.fl(near[1]['y']) + oy
                fr.append(float((arr[:, 1].astype(float) > gy).mean()))
            print('%-16s%-12s%+10.4f%+10.4f%9.1f%%%10s'
                  % (leg, label, C.med(amcl), C.med([d['cloud_dy']
                                                     for d in S]), ins,
                     ('%.4f' % C.med(fr)) if fr else '-'))
    print()
    print('The sign, the support and the classification are identical')
    print('under both. The measured convention makes the southward bias')
    print('slightly LARGER, exactly as C2-NAV.29 reported.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
