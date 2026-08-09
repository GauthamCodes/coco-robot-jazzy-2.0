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
Where to drop parity probes, and why there rather than on a grid.

The Yard's cross-engine parity is measured by DROPPING SPHERES AND
COMPARING WHERE THEY SETTLE, in both engines, from the same start
heights. Not by sampling the surface, and the distinction is the whole
point of this module.

A height-sample test passes on a world one engine cannot see into.
Measured in Phase 2: MuJoCo replaces a mesh with its convex hull for
collision, silently, and a V-trough whose floor was at z = -0.400 had a
probe settle at z = +0.0496 — 450 mm out — while both engines agreed on
the file, its SHA-256, and every analytically sampled height. Only
something that touches the surface can tell the difference.

Three further ways a naive probe set passes when it should not, each
countered here:

1. **Sampling on the lattice.** Grid NODES agree between a heightfield
   and a mesh even when the triangulation diagonal differs, and a wrong
   diagonal moves the surface between nodes by the full cell relief. So
   every rough-terrain probe sits at a deliberately irrational fraction
   of a cell (``_OFFSET``), never on a node and never at a cell centre.

2. **Symmetry.** A field that is mirrored or transposed relative to the
   other engine's still matches everywhere if the probe set is
   symmetric. ``FIDUCIAL`` is a handful of points at a deliberately
   asymmetric (x, y) whose expected heights are all different from one
   another, so a mirrored world cannot reproduce them.

3. **Uniform coverage.** 200 probes spread evenly over 40 m² of mostly
   flat apron measures the flat apron. These cluster where the geometry
   is hard and concave — troughs, depressions, the void, the undercut —
   and along route centrelines at the WHEEL offsets (+/-0.137 m), which
   is where the robot actually touches.
"""

import math

import numpy as np

# Irrational-ish fractions of a cell. Any probe on a rough surface lands
# here rather than on a node (0.0) or a centre (0.5), because those are
# exactly the two positions at which a wrong triangulation diagonal still
# gives the right answer.
_OFFSET = (0.2831853, 0.6180339)

# The wheel is what the robot stands on, so it is the probe that decides
# whether a surface is drivable. The 5 mm probe exists to reach INTO
# concavities the wheel cannot enter: at the curb undercut a wheel-radius
# sphere settles on the lip in both engines whether or not the pocket is
# modelled, which would report parity it has not tested.
WHEEL_PROBE = 0.0585
SMALL_PROBE = 0.005


def _stations(x0, x1, n):
    """n stations, offset off both ends so none lands on a feature edge."""
    return [x0 + (x1 - x0) * (i + 0.5) / n for i in range(n)]


def probe_points(s, radius=WHEEL_PROBE):
    """[(label, x, y, z0)] — where to drop probes for this Yard sample.

    Deliberately NOT a function of the engine: both get the identical
    list, so a disagreement is the engines and never the sampling.

    ``z0`` is an explicit start height, or ``None`` meaning "just clear of
    the analytic surface". It is not decoration. The first run of this
    test reported clean parity on the curb undercut while never entering
    it: the probes started at ``height(x, y)``, which over the lip is the
    lip's TOP, so they settled on the lip in both engines and would have
    done so whether or not the pocket beneath was modelled at all. A
    concave feature can only be probed from inside it. Same correction for
    the bridge void, where starting at the apron tested that the apron is
    there rather than that the hole is.
    """
    p = s.params
    pts = []

    # ── route centrelines, at the wheel offsets ──────────────────────────
    # +/-0.137 m is half the track. Sampling the centreline alone would
    # miss camber entirely — the centreline of a cambered ramp is at the
    # same height whatever the camber.
    for key, route in s.routes.items():
        for i, x in enumerate(_stations(route.x_foot + 0.10,
                                        route.x_top - 0.10, 9)):
            for sgn in (-1.0, 1.0):
                y = route.y_centre + sgn * 0.137
                pts.append((f'route_{key}_s{i}_{"L" if sgn > 0 else "R"}',
                            x, y, None))

    # ── washboard: crests, troughs and both flanks ───────────────────────
    wb = p['washboard']
    wx0, wx1 = p['deck']['sections']['washboard']['x']
    lam = wb['wavelength']
    n_lam = int((wx1 - wx0) / lam)
    for c in range(n_lam):
        base = wx0 + c * lam
        for frac, tag in ((0.25, 'crest'), (0.75, 'trough'),
                          (0.10, 'rise'), (0.60, 'fall')):
            x = base + frac * lam
            if x > wx1 - 0.01:
                continue
            for y, ytag in ((0.0, 'c'), (1.4, 'y14')):
                pts.append((f'wb_{tag}{c}_{ytag}', x, y, None))

    # ── rubble: the DEPRESSIONS specifically ─────────────────────────────
    route_c = s.routes['c']
    g = s.rubble
    ny, nx = g.shape
    # lowest cells, away from the tapered ends where relief is suppressed
    inner = g[2:ny - 2, int(nx * 0.25):int(nx * 0.8)]
    flat = inner.ravel()
    order = np.argsort(flat)
    picks = list(order[:14]) + list(order[-8:])
    h, w = inner.shape
    for n, idx in enumerate(picks):
        i, j = divmod(int(idx), w)
        gi, gj = i + 2, j + int(nx * 0.25)
        x = route_c.x_foot + (gj + _OFFSET[0]) / (nx - 1) * route_c.run
        y = (route_c.y_centre - route_c.width / 2.0
             + (gi + _OFFSET[1]) / (ny - 1) * route_c.width)
        tag = 'low' if n < 14 else 'high'
        pts.append((f'rubble_{tag}{n}', x, y, None))

    # ── the bridge, and the void beside it ───────────────────────────────
    b = p['bridge']
    bx0, bx1 = p['deck']['sections']['bridge']['x']
    bw = b['width']['value']
    void_z = p['deck']['z'] + 0.05 + radius
    for i, x in enumerate(_stations(bx0, bx1, 5)):
        pts.append((f'bridge_on{i}', x, 0.0, None))
        pts.append((f'bridge_edge{i}', x, bw / 2.0 - 0.05, None))
        # the void: a 0.650 m drop. If either engine models it as a low
        # step or fills it, the settle height says so immediately.
        # started ABOVE deck height so the probe must fall THROUGH the
        # void. Started at the apron it would only confirm the apron.
        pts.append((f'bridge_void_L{i}', x, bw / 2.0 + 0.45, void_z))
        pts.append((f'bridge_void_R{i}', x, -bw / 2.0 - 0.45, void_z))

    # ── the under-deck cavity: a genuine overhang ────────────────────────
    # The deck slabs float 0.55 m above the apron, so the space beneath
    # them is a real concave void with a roof. That is the shape a convex
    # hull fills and a height sample cannot see, and unlike the curb
    # overhang this one is never driven into, so probing it costs the
    # physics nothing.
    #
    # A probe started on the apron UNDER the deck must stay on the apron.
    # If either engine treats the slab as solid to the ground, or hulls
    # the deck down to it, the probe is ejected and the two disagree by
    # roughly the deck height.
    for name in ('staging', 'bay'):
        x0, x1 = p['deck']['sections'][name]['x']
        for i, x in enumerate(_stations(x0 + 0.05, x1 - 0.05, 3)):
            for j, y in enumerate((-1.9, 0.4, 2.4)):
                pts.append((f'underdeck_{name}{i}_{j}', x, y,
                            radius + 0.002))

    # ── flat references: staging, bay, descent, apron ────────────────────
    for i, x in enumerate(_stations(*p['deck']['sections']['staging']['x'],
                                    2)):
        pts.append((f'staging{i}', x, 0.3, None))
    for i, x in enumerate(_stations(*p['deck']['sections']['bay']['x'], 4)):
        pts.append((f'bay{i}', x, -0.4 + 0.3 * i, None))
    d = p['descent']
    run = p['deck']['z'] / math.tan(math.radians(d['grade_deg']))
    dx0 = p['deck']['x'][1]
    for i, x in enumerate(_stations(dx0 + 0.1, dx0 + run - 0.1, 5)):
        pts.append((f'descent{i}', x, 0.137 * (1 if i % 2 else -1), None))
    for i, x in enumerate(_stations(-4.6, -3.4, 3)):
        pts.append((f'apron{i}', x, -2.6, None))

    # ── asymmetric fiducial ──────────────────────────────────────────────
    # Four points chosen so their expected heights are mutually distinct
    # and none is at a mirror image of another. A world that is flipped in
    # y, transposed, or offset cannot reproduce all four.
    pts += [('fiducial_A', -1.731, 2.417, None),
            ('fiducial_B', -0.913, -2.114, None),
            ('fiducial_C', 1.117, 1.319, None),
            ('fiducial_D', 3.271, -1.877, None)]

    return _despace(pts, radius)


def _despace(pts, radius):
    """Drop probes that would touch a probe already kept.

    Two spheres closer than 2r collide with EACH OTHER and settle high,
    which reads as a terrain disagreement in whichever engine resolves
    the pile differently. Dropping them is honest; silently keeping them
    would manufacture a parity failure out of the probe set.
    """
    keep, sep = [], 2.2 * radius
    for label, x, y, z0 in pts:
        if all((x - kx) ** 2 + (y - ky) ** 2 >= sep ** 2
               for _, kx, ky, _z in keep):
            keep.append((label, x, y, z0))
    return keep
