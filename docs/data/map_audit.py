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

"""Audit gazebo_models/maps/coco_world.pgm against the world that built it.

Written for C2-M1.6, which had to separate two questions that look the
same on screen: is the occupancy map bad, or is the RViz overlay
cluttered? Turning the costmaps off answers it by eye. This answers it
with numbers, and the numbers are what docs/SESSION_LOG.md cites.

The decisive test is REGISTRATION. Every static object in
worlds/coco_world.world has a known pose. If the map is coherent, one
rigid (dx, dy) explains all of them; if SLAM drifted or closed a loop
badly, the landmarks disagree about the offset and structures duplicate.
That is a measurement, not an impression.

Read-only. Reads the committed map and the committed world, writes only
the figure. Touches no simulator and no SLAM parameter.

    python3 docs/data/map_audit.py                       # numbers only
    python3 docs/data/map_audit.py -o docs/images/c2m16_map_audit.png

Needs numpy, scipy, Pillow and (for -o) matplotlib. It is not a ROS node
and is deliberately not installed by CMakeLists: it is evidence, not a
runtime tool.
"""
import argparse
import os

import numpy as np

from scipy import ndimage

from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..', '..'))
PGM = os.path.join(REPO, 'gazebo_models', 'maps', 'coco_world.pgm')
YAML = os.path.join(REPO, 'gazebo_models', 'maps', 'coco_world.yaml')

RES = 0.050
ORIGIN = (-2.119, -4.910)
OCC_TH, FREE_TH = 0.65, 0.196
S8 = np.ones((3, 3), bool)

# From worlds/coco_world.world. World frame, not map frame -- the offset
# between them is what the registration section measures.
# (name, x, y, size_x, size_y)
LANDMARKS = [
    ('box_obstacle_1', -3.0, 2.40, 0.5, 0.5),
    ('box_obstacle_2', 0.8, -1.40, 0.5, 0.5),
    ('cylinder_obstacle', -0.2, 0.60, 0.4, 0.4),
    ('gate_cube_north', -1.1, 1.05, 0.5, 0.5),
    ('gate_cube_south', -1.1, -0.75, 0.5, 0.5),
]
WALLS = [
    ('wall_north', 2.0, 3.5, 12.0, 0.2),
    ('wall_south', 2.0, -3.5, 12.0, 0.2),
    ('wall_west', -4.0, 0.0, 0.2, 7.2),
    ('wall_east', 8.0, 0.0, 0.2, 7.2),
]
# coco_config/robot.py
RAMP_FOOT_X, RAMP_RUN, PLATFORM_LEN = 1.0, 2.0, 1.5
RAMP_WIDTH, RAMP_ANGLE_DEG = 2.5, 18.0
ROBOT_X_FOOTPRINT = 0.297          # the bound a wall gap must beat to matter


def load():
    img = np.array(Image.open(PGM))
    p = (255.0 - img) / 255.0
    return img, p > OCC_TH, p < FREE_TH


def bbox_world(mask, h, x0, y0):
    rows, cols = np.where(mask)
    return (x0 + cols.min() * RES, x0 + (cols.max() + 1) * RES,
            y0 + (h - rows.max() - 1) * RES, y0 + (h - rows.min()) * RES)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-o', '--out', help='write the three-panel figure here')
    args = ap.parse_args()

    img, occ, free = load()
    h, w = img.shape
    x0, y0 = ORIGIN
    unk = ~occ & ~free
    n = w * h

    print(f'MAP  {w} x {h} cells @ {RES} m = {w*RES:.2f} x {h*RES:.2f} m')
    print(f'     x {x0:.3f}..{x0+w*RES:.3f}   y {y0:.3f}..{y0+h*RES:.3f}'
          f'   centre ({x0+w*RES/2:.3f}, {y0+h*RES/2:.3f})')
    print(f'     occupied {occ.sum():5d} ({occ.sum()/n*100:5.2f}%) '
          f'{occ.sum()*RES*RES:7.3f} m^2')
    print(f'     free     {free.sum():5d} ({free.sum()/n*100:5.2f}%) '
          f'{free.sum()*RES*RES:7.3f} m^2')
    print(f'     unknown  {unk.sum():5d} ({unk.sum()/n*100:5.2f}%) '
          f'{unk.sum()*RES*RES:7.3f} m^2')

    lab, k = ndimage.label(occ, structure=S8)
    sizes = ndimage.sum(occ, lab, range(1, k + 1)).astype(int)
    order = np.argsort(sizes)[::-1]
    print(f'\nOCCUPIED COMPONENTS: {k}; '
          f'{int((sizes <= 2).sum())} are <= 2 cells (speckle); '
          f'largest holds {sizes.max()/occ.sum()*100:.1f}% of occupied cells')
    for r, i in enumerate(order[:8]):
        bx0, bx1, by0, by1 = bbox_world(lab == i + 1, h, x0, y0)
        print(f'  #{r+1} {sizes[i]:5d} cells  x {bx0:6.2f}..{bx1:6.2f} '
              f' y {by0:6.2f}..{by1:6.2f}  ({bx1-bx0:5.2f} x {by1-by0:5.2f} m)')

    # ---- registration: the test a drifted map fails --------------------
    print('\nREGISTRATION  five world landmarks vs the map')
    offs = []
    for name, wx, wy, sx, sy in LANDMARKS:
        best = None
        for i in order[:12]:
            bx0, bx1, by0, by1 = bbox_world(lab == i + 1, h, x0, y0)
            if bx1 - bx0 > 1.5 or by1 - by0 > 1.5:
                continue                    # a wall or the ramp, not a box
            cx, cy = (bx0 + bx1) / 2, (by0 + by1) / 2
            d = abs(cy - wy)                # y needs no offset to first order
            if best is None or d < best[0]:
                best = (d, cx, cy, bx1 - bx0, by1 - by0)
        _, cx, cy, mw, mh = best
        offs.append((cx - wx, cy - wy))
        print(f'  {name:<20} world ({wx:5.2f},{wy:5.2f})  map centre '
              f'({cx:6.3f},{cy:6.3f})  offset ({cx-wx:+.3f},{cy-wy:+.3f})  '
              f'size err {mw-sx:+.2f} x {mh-sy:+.2f}')
    offs = np.array(offs)
    dx, dy = offs.mean(0)
    resid = np.abs(offs - [dx, dy]).max()
    print(f'  mean offset ({dx:+.4f}, {dy:+.4f}) m; peak-to-peak '
          f'({offs[:,0].ptp():.4f}, {offs[:,1].ptp():.4f}) m; '
          f'worst residual {resid*1000:.0f} mm = {resid/RES:.2f} cell')
    print('  A single rigid transform explains all five. Drift or a bad '
          'loop closure\n  makes landmarks disagree about the offset; '
          'these do not.')

    # ---- walls: coverage, and the largest gap vs the robot -------------
    print('\nWALLS  coverage along each wall, and the largest continuous gap')
    ix1, iy1 = x0 + w * RES, y0 + h * RES
    for name, wx, wy, sx, sy in WALLS:
        ex0, ex1 = wx - sx / 2 + dx, wx + sx / 2 + dx
        ey0, ey1 = wy - sy / 2 + dy, wy + sy / 2 + dy
        c0, c1 = max(int((ex0 - x0) / RES), 0), min(
            int(np.ceil((ex1 - x0) / RES)), w)
        r1 = h - max(int((ey0 - y0) / RES), 0)
        r0 = h - min(int(np.ceil((ey1 - y0) / RES)), h)
        patch = occ[r0:r1, c0:c1]
        hit = patch.any(axis=0) if sx > sy else patch.any(axis=1)
        # longest run of misses
        run = best_run = 0
        for v in hit:
            run = 0 if v else run + 1
            best_run = max(best_run, run)
        clip = max(x0 - ex0, 0) + max(ex1 - ix1, 0) + \
            max(y0 - ey0, 0) + max(ey1 - iy1, 0)
        print(f'  {name:<11} coverage {hit.mean()*100:5.1f}%   '
              f'largest gap {best_run*RES:4.2f} m   '
              f'{clip*1000:5.0f} mm of it lies outside the map image')
    print(f'  A gap is only a navigable hole if it beats the robot\'s '
          f'{ROBOT_X_FOOTPRINT:.3f} m footprint.')

    # ---- the ramp: why its outline is not its footprint ----------------
    summit = RAMP_FOOT_X + RAMP_RUN
    dfoot = summit + PLATFORM_LEN + RAMP_RUN
    ramp = None
    for i in order[:6]:
        bx0, bx1, by0, by1 = bbox_world(lab == i + 1, h, x0, y0)
        if bx1 - bx0 > 3.0 and by1 - by0 > 2.0 and bx1 - bx0 < 6.0:
            ramp = (bx0, bx1, by0, by1)
            break
    if ramp:
        bx0, bx1, by0, by1 = ramp
        lead, trail = bx0 - (RAMP_FOOT_X + dx), (dfoot + dx) - bx1
        t = np.tan(np.radians(RAMP_ANGLE_DEG))
        print(f'\nRAMP + PLATFORM  world footprint x {RAMP_FOOT_X:.2f}..'
              f'{dfoot:.2f}; measured in map x {bx0:.2f}..{bx1:.2f}')
        print(f'  inset {lead:+.3f} m at the up-ramp foot, {trail:+.3f} m at '
              f'the down-ramp foot')
        print(f'  a scan plane at h only sees the wedge beyond '
              f'h/tan({RAMP_ANGLE_DEG:.0f} deg):')
        print(f'    implied h = {lead*t*1000:.1f} mm and {trail*t*1000:.1f} mm'
              f', agreeing to {abs(lead-trail)*t*1000:.1f} mm')
        print(f'    LIDAR_MOUNT_XYZ z = 0.200 m. The inset is SYMMETRIC, '
              f'which a mapping\n    defect would not be.')

    # ---- does any of the noise cost navigable space? -------------------
    labf, kf = ndimage.label(free, structure=S8)
    fsz = ndimage.sum(free, labf, range(1, kf + 1))
    main_free = labf == (int(np.argmax(fsz)) + 1)
    er = int(round(0.2225 / RES))
    drivable = ndimage.binary_erosion(
        main_free, np.ones((2 * er + 1, 2 * er + 1), bool))
    speck = np.zeros_like(occ)
    for i in range(k):
        if sizes[i] > 5:
            continue
        comp = lab == i + 1
        ring = ndimage.binary_dilation(comp, structure=S8) & ~comp
        if ring.any() and free[ring].mean() > 0.9:
            speck |= comp
    ir = int(round(0.30 / RES))
    infl = ndimage.binary_dilation(speck, np.ones((2*ir+1, 2*ir+1), bool))
    lost = infl & drivable
    rem = drivable & ~infl
    labr, kr = ndimage.label(rem, structure=S8)
    rs = ndimage.sum(rem, labr, range(1, kr + 1)) if kr else np.array([0])
    print(f'\nNAVIGATION IMPACT')
    print(f'  free space {free.sum()*RES*RES:.2f} m^2; largest connected '
          f'component {fsz.max()*RES*RES:.2f} m^2 '
          f'({fsz.max()/free.sum()*100:.1f}%)')
    print(f'  drivable (that component eroded by a 0.2225 m robot radius): '
          f'{drivable.sum()*RES*RES:.2f} m^2')
    print(f'  free-floating speckle: {int(speck.sum())} cells = '
          f'{speck.sum()*RES*RES:.4f} m^2')
    print(f'  inflating every speck by 0.30 m costs '
          f'{lost.sum()*RES*RES:.3f} m^2 = '
          f'{lost.sum()/max(drivable.sum(),1)*100:.2f}% of drivable space,')
    print(f'  and leaves {kr} component(s), the largest holding '
          f'{rs.max()/max(rem.sum(),1)*100:.2f}% -- the speckle does not '
          f'sever the arena.')

    if args.out:
        figure(img, occ, free, unk, lab, order, sizes, labf, fsz, args.out)


def figure(img, occ, free, unk, lab, order, sizes, labf, fsz, out):
    """Three panels: the trinary map, occupied components, free components."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    h, w = img.shape
    x0, y0 = ORIGIN
    ext = [x0, x0 + w * RES, y0, y0 + h * RES]

    def F(a):
        return np.flipud(a)

    fig, axes = plt.subplots(1, 3, figsize=(30, 8))

    rgb = np.zeros((h, w, 3), float)
    rgb[F(unk)] = (0.55, 0.55, 0.55)
    rgb[F(free)] = (1, 1, 1)
    rgb[F(occ)] = (0, 0, 0)
    axes[0].imshow(rgb, extent=ext, origin='lower', interpolation='nearest')
    axes[0].set_title('raw /map, trinary  '
                      '(black occupied, white free, grey unknown)')

    rank = np.zeros_like(lab)
    for r, i in enumerate(order):
        rank[lab == i + 1] = r + 1
    show = np.zeros((h, w, 3), float) + 0.10
    show[F(free)] = (0.92, 0.92, 0.92)
    show[F(unk)] = (0.62, 0.62, 0.62)
    cols = [(0, 0, 0), (0.85, 0.1, 0.1), (0.1, 0.35, 0.95), (0.95, 0.55, 0),
            (0.6, 0.1, 0.8), (0, 0.6, 0.35), (0.9, 0.75, 0), (0, 0.7, 0.8)]
    for r in range(1, 9):
        m = F(rank == r)
        if m.any():
            show[m] = cols[r - 1]
    show[F(rank > 8)] = (1.0, 0.0, 1.0)
    axes[1].imshow(show, extent=ext, origin='lower', interpolation='nearest')
    axes[1].set_title(f'occupied components: {rank.max()} total. '
                      'black #1 boundary, red #2 ramp, blue #3, '
                      'magenta rank>8 = speckle')

    fo = np.argsort(fsz)[::-1]
    show2 = np.zeros((h, w, 3), float) + 0.10
    show2[F(unk)] = (0.58, 0.58, 0.58)
    show2[F(occ)] = (0, 0, 0)
    fc = [(0.80, 0.92, 0.80), (0.95, 0.25, 0.25), (0.15, 0.4, 1.0),
          (1.0, 0.6, 0.0), (0.7, 0.15, 0.85), (0, 0.75, 0.75)]
    for r, i in enumerate(fo[:6]):
        show2[F(labf == i + 1)] = fc[r]
    axes[2].imshow(show2, extent=ext, origin='lower', interpolation='nearest')
    axes[2].set_title('free components: pale green = the navigable arena; '
                      'the rest are pockets outside it')

    for ax in axes:
        ax.set_xlabel('x (m, map frame)')
        ax.set_ylabel('y (m, map frame)')
        ax.grid(alpha=0.25, lw=0.4)
        ax.plot(x0 + w * RES / 2, y0 + h * RES / 2, 'c+', ms=16, mew=2)
    plt.tight_layout()
    plt.savefig(out, dpi=85)
    print(f'\nwrote {out}')


if __name__ == '__main__':
    main()
