#!/usr/bin/env python3
"""Offline: distance-to-nearest-obstacle for candidate benchmark poses.

Reads the shipped occupancy map and reports, for each candidate pose,
the clearance to the nearest occupied cell.  Used to design the C2-NAV.0
scenarios against measured geometry rather than against the world file's
nominal numbers (the map is what Nav2 actually plans on).
"""
import sys

import numpy as np

MAP_PGM = '/home/gautham/ros2_ws/src/coco-robot-ros2/gazebo_models/maps/coco_world.pgm'
RES = 0.05
ORIGIN = (-2.119, -4.910)      # map-frame origin of cell (0,0)
W2M = (2.0, 0.0)               # map = world + this

ROBOT_RADIUS = 0.20
INFLATION = 0.50


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    # P5 header: magic, (comments), width height, maxval
    toks, i = [], 2
    while len(toks) < 3:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while data[i:i + 1] not in (b'\n', b''):
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        toks.append(int(data[i:j]))
        i = j
    i += 1
    w, h, _ = toks
    img = np.frombuffer(data[i:i + w * h], dtype=np.uint8).reshape(h, w)
    return img


def occupancy(img):
    """Trinary decode, matching map_server: value = (255-px)/255."""
    p = (255.0 - img.astype(np.float64)) / 255.0
    occ = p > 0.65
    return occ[::-1, :]          # PGM row 0 is the TOP = max y


def edt(occ):
    """Exact Euclidean distance (m) to nearest occupied cell, brute force
    over occupied cells only -- the map is small enough."""
    ys, xs = np.nonzero(occ)
    h, w = occ.shape
    gy, gx = np.mgrid[0:h, 0:w]
    best = np.full((h, w), np.inf)
    for cy, cx in zip(ys, xs):
        d = (gy - cy) ** 2 + (gx - cx) ** 2
        np.minimum(best, d, out=best)
    return np.sqrt(best) * RES


def main():
    img = read_pgm(MAP_PGM)
    occ = occupancy(img)
    print(f'map {occ.shape[1]}x{occ.shape[0]} cells, '
          f'{occ.sum()} occupied, res {RES}')
    dist = edt(occ)

    def clearance(wx, wy):
        mx, my = wx + W2M[0], wy + W2M[1]
        cx = int((mx - ORIGIN[0]) / RES)
        cy = int((my - ORIGIN[1]) / RES)
        if not (0 <= cx < occ.shape[1] and 0 <= cy < occ.shape[0]):
            return None
        return dist[cy, cx]

    cands = [
        ('spawn/home',            -2.00,  0.00),
        ('open_space goal',       -2.00, -2.20),
        ('wall_adjacent goal',    -2.00, -3.00),
        ('wall_parallel goal',     0.50, -2.95),
        ('obstacle_corner goal',   0.30, -0.30),
        ('corridor/gate goal',    -2.60, -0.10),
        ('gate centre',           -1.10,  0.15),
        ('enclosure goal',        -3.45,  2.95),
        ('pocket mouth',          -3.55,  1.80),
        ('nw pinch centre',       -3.57,  2.40),
    ]
    print(f'\n{"pose":<24}{"world":>16}{"clear(m)":>10}  verdict')
    for name, wx, wy in cands:
        c = clearance(wx, wy)
        if c is None:
            v = 'OFF MAP'
        elif c < ROBOT_RADIUS:
            v = f'INSCRIBED (<{ROBOT_RADIUS}) - unreachable'
        elif c < INFLATION:
            v = f'in inflation ({ROBOT_RADIUS}..{INFLATION})'
        else:
            v = 'free of inflation'
        print(f'{name:<24}{f"({wx},{wy})":>16}{c if c is None else round(c,3):>10}  {v}')

    # Width of the NW pinch: scan x across at the box's y band.
    print('\nNW pinch profile (world y=2.40, x from -3.9 to -2.6):')
    for i in range(27):
        wx = -3.90 + i * 0.05
        c = clearance(wx, 2.40)
        flag = ('LETHAL' if c is not None and c < 0.001 else
                'inscribed' if c is not None and c < ROBOT_RADIUS else
                'free')
        print(f'  x={wx:+.2f}  clear={c:.3f}  {flag}')

    print('\nZone A gate profile (world x=-1.10, y from -1.2 to 1.5):')
    for i in range(28):
        wy = -1.20 + i * 0.10
        c = clearance(-1.10, wy)
        flag = ('LETHAL' if c < 0.001 else
                'inscribed' if c < ROBOT_RADIUS else
                'inflated' if c < INFLATION else 'free')
        print(f'  y={wy:+.2f}  clear={c:.3f}  {flag}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
