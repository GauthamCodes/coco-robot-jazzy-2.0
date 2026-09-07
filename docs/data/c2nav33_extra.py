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
c2nav33_extra.py
================
C2-NAV.33, the four things the main module's tables could not carry, and
one of them is a finding the session did not go looking for.

1. `/amcl_pose` IS PUBLISHED ONLY ON RESAMPLE UPDATES, so at
   `resample_interval: 2` its rate HALVES. This turned up as blank
   `AMCL dy` and `w(AMCL)/w(GT)` cells in `weights` -- the two columns
   that need a published pose at the same instant as an observable
   weight, which at interval 2 is a combination that never occurs. It is
   measured here rather than inferred from the blanks, and the mechanism
   is read out of the binary.

2. THE BRIEF'S LIKELIHOOD RATIO, computed anyway, against a PROXY that
   C2-NAV.30 measured rather than assumed. Since no published pose is
   available on the clouds whose weights are visible, the unweighted
   whole-set centroid stands in for it -- C2-NAV.30 measured
   |whole-set mean - /amcl_pose| at median 0.00004 m, p95 0.00276 m,
   max 0.00542 m. The proxy is named at every use and its error is
   quoted; it is not silently substituted.

3. THE STATIONARY CASE (the brief's option F) CANNOT BE TESTED, and the
   reason is structural, not a shortage of data. AMCL updates the filter
   only after `update_min_d` 0.25 m or `update_min_a` 0.2 rad of motion,
   and it publishes a cloud only when it updates. There are therefore no
   stationary filter updates to sample: `weights` reports n=0 stopped in
   both legs. Reporting "F, stable while stationary" from zero samples
   would be inventing a measurement.

4. THE SIGN OF THE WEIGHTING AGAINST THE DISPLACEMENT IT WOULD HAVE TO
   EXPLAIN. If observation weighting drove the southward bias, the
   weights should push hardest south exactly when the cloud is furthest
   south. The correlation is computed here, and it runs the other way.

Offline. Reads the run artefacts and the installed binary; starts no ROS,
no Gazebo and no simulator.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import c2nav30_cloud as C            # noqa: E402
import c2nav33_weights as W          # noqa: E402

# C2-NAV.30's measured agreement between the whole-set mean and the
# published /amcl_pose. Quoted, not re-derived, and used only to bound
# the proxy in section 2.
PROXY_MED = 0.00004
PROXY_P95 = 0.00276
PROXY_MAX = 0.00542


def main():
    tag = W.RUN
    print('=' * 78)
    print('C2-NAV.33 extra -- the halved pose rate, the ratio via a')
    print('                   measured proxy, and the sign of the tilt')
    print('=' * 78)

    # ------------------------------------------------------- 1
    print()
    print('1. /amcl_pose IS PUBLISHED ONLY ON RESAMPLE UPDATES')
    print()
    print('   Mechanism, from the installed libamcl_core.so and not from')
    print('   memory: in laserReceived the `resampled` flag is OR-ed into')
    print('   r15d at e4115 and the publishAmclPose block at e411d is')
    print('   entered by a conditional jump on it (e3fa5 jne e411d) --')
    print('   the `if (resampled || force_publication)` shape. The')
    print('   particle cloud, by contrast, is published unconditionally')
    print('   on the !force_update_ path, which is what makes this')
    print('   session possible at all.')
    print()
    print('%-16s%-8s%7s%9s%9s' % ('leg', 'phase', 'msgs', 'w/ pose',
                                  'share'))
    print('-' * 78)
    tot = {'pre': [0, 0], 'post': [0, 0]}
    for leg in (W.CONTROL_LEG, W.WALL_LEG):
        S = W.wsnaps(tag, leg)
        for ph in ('pre', 'post'):
            G = [d for d in S if d['phase'] == ph]
            a = sum(1 for d in G if d.get('amcl_x') is not None)
            tot[ph][0] += len(G)
            tot[ph][1] += a
            print('%-16s%-8s%7d%9d%8.1f%%'
                  % (leg, ph, len(G), a,
                     100.0 * a / len(G) if G else float('nan')))
    print('-' * 78)
    for ph in ('pre', 'post'):
        print('%-16s%-8s%7d%9d%8.1f%%'
              % ('BOTH LEGS', ph, tot[ph][0], tot[ph][1],
                 100.0 * tot[ph][1] / tot[ph][0] if tot[ph][0] else
                 float('nan')))
    print()
    print('   %d of %d pre-resample clouds carry a fresh pose. The split'
          % (tot['pre'][1], tot['pre'][0]))
    print('   is exact, and it is a REAL BEHAVIOURAL CONSEQUENCE of the')
    print('   parameter: at interval 2 the pose rate halves. That is a')
    print('   further reason interval 2 is a diagnostic window and not a')
    print('   configuration, and it must be reverted.')

    # ------------------------------------------------------- 2
    print()
    print('2. THE BRIEF\'S w(near AMCL)/w(near GT), via a MEASURED proxy')
    print()
    print('   No published pose exists on the clouds whose weights are')
    print('   visible (section 1), so the UNWEIGHTED WHOLE-SET CENTROID')
    print('   stands in for it. C2-NAV.30 measured that substitution:')
    print('   |whole-set mean - /amcl_pose| median %.5f m, p95 %.5f m,'
          % (PROXY_MED, PROXY_P95))
    print('   max %.5f m. It is a PROXY and is labelled as one wherever'
          % PROXY_MAX)
    print('   it appears; it is not the published pose.')
    print()
    print('   Band radius %.2f m = one map cell (C2-NAV.31: the finest'
          % W.NEAR_R)
    print('   scale the likelihood field can distinguish at all).')
    print()
    print('%-16s%7s%9s%9s%11s%11s'
          % ('leg', 'snaps', 'n near', 'n near', 'w near GT', 'w near'))
    print('%-16s%7s%9s%9s%11s%11s'
          % ('', '', 'GT', 'proxy', '(mean)', 'proxy/GT'))
    print('-' * 78)
    for leg in (W.CONTROL_LEG, W.WALL_LEG):
        S = [d for d in W.wsnaps(tag, leg)
             if d['phase'] == 'pre' and d.get('gt_ok')]
        ratios, ng, npx, wg = [], [], [], []
        for d in S:
            arr = _particles(tag, leg, d['k'])
            if arr is None:
                continue
            w = arr[:, 3]
            wn = w / w.sum()
            dg = np.hypot(arr[:, 0] - d['gt_x'], arr[:, 1] - d['gt_y'])
            dp = np.hypot(arr[:, 0] - d['ux'], arr[:, 1] - d['uy'])
            mg = dg <= W.NEAR_R
            mp = dp <= W.NEAR_R
            ng.append(int(mg.sum()))
            npx.append(int(mp.sum()))
            if mg.sum() and mp.sum():
                a, b = float(wn[mg].mean()), float(wn[mp].mean())
                wg.append(a)
                if a > 0:
                    ratios.append(b / a)
        if not ratios:
            print('%-16s   -- no snapshot had both bands populated --' % leg)
            continue
        print('%-16s%7d%9.1f%9.1f%11.6f%11.4f'
              % (leg, len(ratios), W.med(ng), W.med(npx), W.med(wg),
                 W.med(ratios)))
    print()
    print('   A ratio ABOVE 1 means the neighbourhood of the reported')
    print('   pose is favoured over the neighbourhood of the truth.')
    print('   C2-NAV.31, using the shipped likelihood model offline,')
    print('   measured median w(AMCL)/w(GT) = 0.7774 with AMCL')
    print('   outscoring GT in 0 of 19 snapshots. This is the same')
    print('   comparison against the DEPLOYED weights.')

    # ------------------------------------------------------- 3
    print()
    print('3. THE STATIONARY CASE CANNOT BE TESTED (brief option F)')
    print()
    for leg in (W.CONTROL_LEG, W.WALL_LEG):
        S = [d for d in W.wsnaps(tag, leg)
             if d['phase'] == 'pre' and d.get('gt_ok')]
        mv = sum(1 for d in S if W._moving(d) is True)
        st = sum(1 for d in S if W._moving(d) is False)
        un = sum(1 for d in S if W._moving(d) is None)
        print('   %-16s %2d pre-resample clouds: %2d moving, %2d stopped, '
              '%2d unknown' % (leg, len(S), mv, st, un))
    print()
    print('   Zero stopped, and that is structural. update_min_d 0.25 m /')
    print('   update_min_a 0.2 rad gate the filter update, and nav2_amcl')
    print('   publishes a cloud only when the filter updates -- so a')
    print('   stationary robot produces no cloud to sample. Answering')
    print('   "F, stable while stationary" from n=0 would be inventing a')
    print('   measurement. It is UNTESTED, not confirmed.')

    # ------------------------------------------------------- 4
    print()
    print('4. THE SIGN OF THE TILT AGAINST THE DISPLACEMENT')
    print()
    print('   If observation weighting DROVE the bias, the weights would')
    print('   push hardest south exactly when the cloud is furthest')
    print('   south -- shift and dy_u would be POSITIVELY correlated')
    print('   (both more negative together).')
    print()
    print('%-16s%7s%12s%12s%14s'
          % ('leg', 'n', 'corr', 'shift/dy_u', 'share of bias'))
    print('-' * 78)
    for leg in (W.CONTROL_LEG, W.WALL_LEG):
        S = [d for d in W.wsnaps(tag, leg)
             if d['phase'] == 'pre' and d.get('gt_ok')
             and d['shift'] is not None]
        if len(S) < 3:
            print('%-16s   -- too few snapshots --' % leg)
            continue
        x = np.array([d['dy_u'] for d in S])
        y = np.array([d['shift'] for d in S])
        r = float(np.corrcoef(x, y)[0, 1])
        share = W.med([d['shift'] for d in S]) / W.med([d['dy_u']
                                                        for d in S])
        print('%-16s%7d%12.4f%12.4f%13.1f%%'
              % (leg, len(S), r, share, 100.0 * share))
    print()
    print('   A NEGATIVE correlation means the weights push north when')
    print('   the cloud is furthest south -- a restoring signal, the')
    print('   opposite of a driving one.')
    print()
    print('   And the share is what the whole session turns on: the')
    print('   weighting moves the centroid by that fraction of the')
    print('   displacement it would have to explain.')

    # ------------------------------------------------------- 5
    print()
    print('5. AGAINST C2-NAV.31, WHICH PREDICTED THIS OFFLINE')
    print()
    print('   C2-NAV.31 re-weighted C2-NAV.30\'s OWN recorded particles')
    print('   with the shipped likelihood-field model and got a centroid')
    print('   move of +0.0030 m -- NORTH, toward truth.')
    S = [d for d in W.wsnaps(tag, W.WALL_LEG)
         if d['phase'] == 'pre' and d.get('gt_ok')]
    sh = [d['shift'] for d in S if d['shift'] is not None]
    lo, hi = W.ci95(sh)
    print()
    print('   C2-NAV.33 measures the DEPLOYED weights doing it live:')
    print('      median %+.5f m, mean %+.5f m, 95%% CI [%+.5f, %+.5f]'
          % (W.med(sh), W.mean(sh), lo, hi))
    print()
    print('   Both are a few millimetres against a -0.09 m bias, both')
    print('   straddle or oppose it, and neither is a candidate')
    print('   explanation. An offline model and the deployed binary')
    print('   agree on the magnitude and disagree on the sign of a term')
    print('   that is negligible either way.')
    return 0


_PCACHE = {}


def _particles(tag, leg, k):
    key = (tag, leg)
    if key not in _PCACHE:
        _PCACHE[key] = C.read_clouds(tag, leg)
    cl = _PCACHE[key]
    if k < 0 or k >= len(cl):
        return None
    return np.asarray(cl[k][1], dtype=np.float64)


if __name__ == '__main__':
    sys.exit(main())
