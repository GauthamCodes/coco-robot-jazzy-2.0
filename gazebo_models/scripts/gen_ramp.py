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
gen_ramp.py
===========
Parametric ramp generator: emit a clean right-triangular-prism wedge as a
binary STL, in **metres** (scale 1.0), for Gazebo.

This exists because the shipped `rampcoco.stl` was an unclimbable CAD shell
— a ~66 deg near-vertical face with an overhang, not a driveable grade — so
the RL robot could never mount it (see docs/DESIGN_DECISIONS.md). A wedge is
the honest primitive: a single flat driving surface at exactly the requested
angle.

Geometry (local frame, before the launch file positions it):

    z
    ^            C = (run, rise)        rise = run * tan(angle)
    |           /|
    |          / |                      hypotenuse A->C is the driving
    |         /  |                        surface, slope == angle
    |        /   |
    |       / .  |  <- vertical back face (x = run)
    +------A-----B----> x
       A=(0,0)  B=(run,0)                foot edge A is at z=0: zero step

Extruded `width` in y, centred on y=0. Exactly 8 facets (2 triangular ends +
3 rectangular faces x 2 triangles), watertight, outward normals.

Stdlib only (struct + math) on purpose: this runs at build time and in a
unit test, neither of which should need numpy in the interpreter.

Usage:
  python3 gen_ramp.py --angle-deg 18 --run 2.5 --width 2.0 --out ramp_wedge.stl
"""

import argparse
import math
import struct


def _tri(v0, v1, v2, ref):
    """Return (normal, v0, v1, v2) with winding fixed so the facet's normal
    points along `ref` (the outward direction for this face)."""
    ux, uy, uz = (v1[i] - v0[i] for i in range(3))
    wx, wy, wz = (v2[i] - v0[i] for i in range(3))
    nx = uy * wz - uz * wy
    ny = uz * wx - ux * wz
    nz = ux * wy - uy * wx
    if nx * ref[0] + ny * ref[1] + nz * ref[2] < 0:
        v1, v2 = v2, v1
        nx, ny, nz = -nx, -ny, -nz
    mag = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
    return ((nx / mag, ny / mag, nz / mag), v0, v1, v2)


def wedge_triangles(angle_deg, run, width):
    """The 8 triangles (each: normal, v0, v1, v2) of the wedge prism."""
    rise = run * math.tan(math.radians(angle_deg))
    y0, y1 = -width / 2.0, width / 2.0
    # foot (A), back-bottom (B), summit (C) at each end plane
    a0, b0, c0 = (0.0, y0, 0.0), (run, y0, 0.0), (run, y0, rise)
    a1, b1, c1 = (0.0, y1, 0.0), (run, y1, 0.0), (run, y1, rise)
    s = math.sin(math.radians(angle_deg))
    c = math.cos(math.radians(angle_deg))
    return [
        _tri(a0, b0, c0, (0.0, -1.0, 0.0)),   # left triangular end (-y)
        _tri(a1, b1, c1, (0.0, 1.0, 0.0)),    # right triangular end (+y)
        _tri(a0, b0, b1, (0.0, 0.0, -1.0)),   # bottom (z=0)
        _tri(a0, b1, a1, (0.0, 0.0, -1.0)),
        _tri(b0, c0, c1, (1.0, 0.0, 0.0)),    # vertical back face (x=run)
        _tri(b0, c1, b1, (1.0, 0.0, 0.0)),
        _tri(a0, c0, c1, (-s, 0.0, c)),       # driving surface (hypotenuse)
        _tri(a0, c1, a1, (-s, 0.0, c)),
    ]


def write_stl(path, triangles):
    """Write triangles to a binary STL file."""
    with open(path, 'wb') as f:
        f.write(b'coco parametric ramp wedge'.ljust(80, b'\0'))
        f.write(struct.pack('<I', len(triangles)))
        for normal, v0, v1, v2 in triangles:
            f.write(struct.pack('<3f', *normal))
            for v in (v0, v1, v2):
                f.write(struct.pack('<3f', *v))
            f.write(struct.pack('<H', 0))


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--angle-deg', type=float, default=18.0,
                    help='grade of the driving surface (deg). Keep below ~30: '
                         'on the ramp the robot pitches nose-up by ~angle, and '
                         'the RL tip-over terminator fires at 0.6 rad (34 deg).')
    ap.add_argument('--run', type=float, default=2.5,
                    help='horizontal length of the ramp (m)')
    ap.add_argument('--width', type=float, default=2.0,
                    help='width across the ramp (m)')
    ap.add_argument('--out', required=True, help='output STL path')
    args = ap.parse_args()

    tris = wedge_triangles(args.angle_deg, args.run, args.width)
    write_stl(args.out, tris)
    rise = args.run * math.tan(math.radians(args.angle_deg))
    print(f'wrote {args.out}: {len(tris)} facets, '
          f'{args.angle_deg} deg, run {args.run} m, rise {rise:.3f} m, '
          f'width {args.width} m')


if __name__ == '__main__':
    main()
