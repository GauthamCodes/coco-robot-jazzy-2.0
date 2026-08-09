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
The Yard — one generator, two simulators.

Reads ``coco_sim/worlds/yard_params.yaml`` and emits the MuJoCo model
(training) and the Gazebo SDF world (evaluation) from the SAME feature
list. Not two generators that agree; one generator, called twice.

Why it is built this way
------------------------
**One analytic ``height(x, y)`` is the sole source of truth.** No image
and no mesh is the master. Both engines are then derived from it, and
``test_yard_parity.py`` checks the two derivations against each other by
DROPPING PROBES AND COMPARING WHERE THEY SETTLE — not by sampling
heights, for a reason paid for in Phase 2:

  MuJoCo replaces a mesh with its CONVEX HULL for collision, silently. A
  V-trough with its floor at z = -0.400 had a probe settle at z = +0.0496,
  on the hull lid — 450 mm of error, while both engines agreed on the
  file, its checksum, and every analytically sampled height. A geometric
  parity test passes on a world one engine cannot see into. Every concave
  feature here (washboard troughs, rubble depressions, the bridge void,
  the curb undercut) is exactly that failure mode, so parity is measured
  with physics.

**Primitives wherever the surface is planar.** Ramps, curb, bridge, deck
and descent are boxes in BOTH engines, so their parity is exact by
construction rather than by tolerance. It also sidesteps a heightfield
being single-valued: a heightfield cannot represent the bridge's void as
a real hole, only as a low floor, and a low floor is not a negative
obstacle.

**Heightfields only where the surface is genuinely rough** — the rubble
on Route C and the washboard on the deck. MuJoCo gets an ``<hfield>``;
Gazebo gets an STL emitted on **MuJoCo's own triangulation diagonal**
(see ``HFIELD_DIAGONAL``), because a mismatched diagonal moves the
surface by up to the local cell relief while every grid NODE still
agrees — invisible to any parity test that samples on the lattice.

What this module does NOT own
-----------------------------
The target bay's four cylinders and their ``DetachableJoint`` magnets.
Those are generated as SDF strings inside ``full_world_robo.launch.py``
and stay there: the magnet pipeline is what M6 measured 20/20 grasps
through. The Yard generator owns TERRAIN. It emits the deck the bay sits
on, flat and at ``deck.z``, and stops there.

``coco_world.world`` is not touched either. The Yard is a sibling world;
v1 keeps its 10/10 traverse and its 19/20 fetch matrix.
"""

import math
import os
import struct
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np

import yaml

# ── MuJoCo's heightfield triangulation, measured not assumed ─────────────
# Each grid cell is split into two triangles, and WHICH diagonal is used
# decides the surface everywhere except the grid nodes themselves. Get it
# wrong in the Gazebo STL and both engines still agree at every node while
# the surface between them differs by the local cell relief.
#
# Probed on mujoco 3.11.0: a 2x2 field with corner (i=1, j=1) raised to 1
# and the other three at 0 rays to 0.5 at the cell centre. That is only
# possible if the diagonal joins (i, j) to (i+1, j+1) — the split along
# (i, j+1)-(i+1, j) would put the centre on the zero edge and ray to 0.0.
# Two off-diagonal probes confirm it: (u, v) = (0.75, 0.25) and
# (0.25, 0.75) both ray to 0.25, matching the barycentric prediction for
# this diagonal and not the other.
#
# test_yard_parity.py RE-DERIVES this from MuJoCo at test time rather than
# trusting the constant, so a MuJoCo upgrade that changes the convention
# fails a test instead of quietly warping the evaluation world.
HFIELD_DIAGONAL = 'ij_to_i1j1'

# hfield_data is row-major, data[i * ncol + j], with j indexing x and i
# indexing y, first row at -y. Also measured in the same probe.
DEFAULT_PARAMS = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'worlds', 'yard_params.yaml')


def load_params(path=None):
    """Load yard_params.yaml. THE only source of Yard geometry."""
    with open(path or DEFAULT_PARAMS) as handle:
        return yaml.safe_load(handle)


# ── per-episode sample ───────────────────────────────────────────────────
@dataclass(frozen=True)
class RouteSample:
    key: str
    grade_deg: float
    camber_deg: float
    friction: float
    width: float
    y_centre: float
    x_top: float          # world x where the ramp meets the deck (or curb)
    z_top: float          # world z at x_top
    run: float            # derived: (z_top) / tan(grade)

    @property
    def x_foot(self):
        return self.x_top - self.run


@dataclass(frozen=True)
class YardSample:
    """Everything an episode needs, resolved. Reproducible from a seed."""
    params: dict
    seed: int
    routes: dict                      # key -> RouteSample
    rubble: np.ndarray = field(repr=False, default=None)
    payload_mass: float = 0.0
    payload_cog: float = 0.0
    torque_scale: float = 1.0
    initial_yaw: float = 0.0
    yaw_gain: float = 1.0


def _tan(deg):
    return math.tan(math.radians(deg))


def sample_yard(params=None, seed=0, randomise=True):
    """Resolve the Yard for one episode. Deterministic given ``seed``.

    Every randomised quantity in M7_DESIGN 2.5 is drawn HERE, from one
    generator seeded once, so an episode is reproducible from the seed
    alone — which is the whole property a debuggable training run needs.
    """
    params = params or load_params()
    rng = np.random.default_rng(seed)
    rand = params['randomisation']
    deck_z = params['deck']['z']
    curb_h = params['curb']['height']['value']

    routes = {}
    for key in ('a', 'b', 'c'):
        spec = params['routes'][key]
        cap = spec.get('grade_jitter_cap_deg')
        lo, hi = rand['grade_jitter_deg']
        if cap is not None:
            lo, hi = max(lo, -cap), min(hi, cap)
        jitter = float(rng.uniform(lo, hi)) if randomise else 0.0
        grade = spec['grade_deg'] + jitter
        camber = (float(rng.uniform(*spec['camber_deg']))
                  if randomise else 0.0)
        mu = (float(rng.uniform(*spec['friction']))
              if randomise else params['friction']['default'])
        # Route C stops short of the deck by the curb: its ramp climbs to
        # deck_z - curb_h and the last curb_h is a vertical step.
        if key == 'c':
            # A CLEAN vertical step. The ramp runs to the deck's leading
            # edge at deck_z - curb_h and the last curb_h is a face.
            #
            # This carried a 60 mm overhanging lip until the parity probes
            # were run, on the theory that an undercut gave the settle test
            # a concave feature. It gave it a hole instead -- the probes
            # fell 650 mm to the apron, because the ramp stopped at the
            # lip's outer edge and nothing floored the pocket. Flooring it
            # exposed the real objection: a robot cannot drive UNDER a lip
            # and then UP through it, so an overhang makes the curb
            # unclimbable no matter the approach speed, and Section I's
            # whole question would have been answered by a modelling
            # accident. Concave coverage moved to the cavity beneath the
            # deck slabs, which is a genuine overhang the robot never
            # drives into. See probes.under_deck.
            x_top, z_top = 0.0, deck_z - curb_h
        else:
            x_top, z_top = 0.0, deck_z
        routes[key] = RouteSample(
            key=key, grade_deg=grade, camber_deg=camber, friction=mu,
            width=spec['width'], y_centre=spec['y_centre'],
            x_top=x_top, z_top=z_top, run=z_top / _tan(grade))

    rub = params['routes']['c']['rubble']
    # The grid SHAPE is pinned to the nominal route, not the jittered one.
    # mjModel.hfield_data is allocated at compile time, so a per-episode
    # grade jitter that changed nrow/ncol could not be written into an
    # already-compiled model at all -- and recompiling per episode is
    # exactly the cost this design avoids. The jitter moves the field's
    # EXTENT (hfield_size and the geom pos) instead, both runtime fields.
    nominal_run = (deck_z - curb_h) / _tan(params['routes']['c']['grade_deg'])
    rubble = _rubble_field(routes['c'], rub, rng, nominal_run)

    if randomise:
        payload = float(rng.uniform(*rand['payload_mass']))
        cog = float(rng.uniform(*rand['payload_cog_offset']))
        torque = float(rng.uniform(*rand['torque_scale']))
        yaw0 = float(rng.uniform(*rand['initial_yaw']))
        gain = float(rng.uniform(*rand['yaw_gain']))
    else:
        payload = cog = yaw0 = 0.0
        torque = gain = 1.0

    return YardSample(params=params, seed=int(seed), routes=routes,
                      rubble=rubble, payload_mass=payload, payload_cog=cog,
                      torque_scale=torque, initial_yaw=yaw0, yaw_gain=gain)


# ── the rubble field ─────────────────────────────────────────────────────
def _rubble_field(route, rub, rng, nominal_run=None):
    """Correlated Gaussian relief for Route C, tapered at both ends.

    Tapered deliberately. Untapered, the noise at the top of the ramp
    lands directly under the curb, so a curb specified at 28 mm would
    actually present anything from 4 mm to 52 mm depending on the episode
    seed — and Section I's whole question is at what height and speed a
    curb is mountable. A feature whose size is a random variable cannot be
    the answer to that. Tapered at the foot for the same reason in
    reverse: the entry should be a ramp, not a random step.
    """
    cell = rub['cell']
    grid_run = nominal_run if nominal_run is not None else route.run
    nx = max(2, int(round(grid_run / cell)) + 1)
    ny = max(2, int(round(route.width / cell)) + 1)
    lc_cells = max(1.0, rub['correlation_length'] / cell)

    white = rng.normal(size=(ny, nx))
    half = int(math.ceil(3 * lc_cells))
    axis = np.arange(-half, half + 1)
    ker = np.exp(-0.5 * (axis / lc_cells) ** 2)
    ker /= math.sqrt(float((ker ** 2).sum()))
    smooth = np.apply_along_axis(
        lambda r: np.convolve(r, ker, mode='same'), 1, white)
    smooth = np.apply_along_axis(
        lambda c: np.convolve(c, ker, mode='same'), 0, smooth)
    sd = float(smooth.std()) or 1.0
    smooth = smooth / sd * rub['rms']['value']

    xs = np.linspace(0.0, route.run, nx)
    taper = np.ones(nx)
    foot, crest = 0.15, 0.30
    taper *= np.clip(xs / foot, 0.0, 1.0)
    taper *= np.clip((route.run - xs) / crest, 0.0, 1.0)
    # raised cosine, so the taper itself does not introduce a slope break
    taper = 0.5 - 0.5 * np.cos(math.pi * taper)
    return smooth * taper[None, :]


# ── the analytic surface: the sole source of truth ───────────────────────
def height(x, y, s):
    """Top surface of the Yard at (x, y). The design, in one function.

    Every feature is solid, so the surface is the MAX over the features
    covering (x, y), floored by the apron at z = 0. That is not a
    modelling convenience — it is what the boxes physically do. A cambered
    ramp box rests ON the ground plane, so where its plane falls below
    zero the apron simply wins, which is how a real ramp feathers into the
    floor instead of digging a trench beside itself.

    The bridge void is the one place the answer is 0 while the deck is
    overhead, and that is the point: a 2-D lidar cannot see a hole.
    """
    p = s.params
    z = 0.0
    deck_z = p['deck']['z']

    for key, route in s.routes.items():
        if not _in_route(x, y, route):
            continue
        plane = ((x - route.x_foot) * _tan(route.grade_deg)
                 + (y - route.y_centre) * _tan(route.camber_deg))
        if key == 'c':
            plane += _rubble_at(x, y, route, s)
        z = max(z, min(plane, route.z_top + 0.5))

    z = max(z, _deck_height(x, y, s, deck_z))

    # descent
    d = p['descent']
    dx0 = p['deck']['x'][1]
    run = deck_z / _tan(d['grade_deg'])
    if dx0 <= x <= dx0 + run and abs(y - d['y_centre']) <= d['width'] / 2:
        z = max(z, deck_z - (x - dx0) * _tan(d['grade_deg']))
    return z


def _in_route(x, y, route):
    return (route.x_foot <= x <= route.x_top
            and abs(y - route.y_centre) <= route.width / 2.0)


def _rubble_at(x, y, route, s):
    """Bilinear read of the rubble grid, on MuJoCo's triangulation.

    Bilinear is NOT what either engine does — both triangulate. This is
    the design surface; the parity test measures the engines.
    """
    g = s.rubble
    ny, nx = g.shape
    u = (x - route.x_foot) / route.run * (nx - 1)
    v = ((y - route.y_centre + route.width / 2.0)
         / route.width * (ny - 1))
    j, i = int(np.clip(u, 0, nx - 2)), int(np.clip(v, 0, ny - 2))
    fu, fv = u - j, v - i
    h00, h01 = g[i, j], g[i, j + 1]
    h10, h11 = g[i + 1, j], g[i + 1, j + 1]
    # MuJoCo's diagonal joins (i, j) to (i+1, j+1): pick the triangle by
    # which side of that diagonal (fu, fv) falls on.
    if fu >= fv:
        return h00 * (1 - fu) + h01 * (fu - fv) + h11 * fv
    return h00 * (1 - fv) + h10 * (fv - fu) + h11 * fu


def _deck_height(x, y, s, deck_z):
    """Deck top at (x, y), or 0 where the deck is absent."""
    p = s.params
    dy0, dy1 = p['deck']['y']
    sec = p['deck']['sections']
    if not (dy0 <= y <= dy1):
        return 0.0

    for name, rng_ in sec.items():
        x0, x1 = rng_['x']
        if not (x0 <= x <= x1):
            continue
        if name == 'washboard':
            wb = p['washboard']
            return deck_z + wb['amplitude']['value'] * math.sin(
                2 * math.pi * (x - x0) / wb['wavelength'])
        if name == 'bridge':
            b = p['bridge']
            if abs(y - b['y_centre']) <= b['width']['value'] / 2.0:
                return deck_z
            return 0.0        # the void. A hole, not a low step.
        return deck_z
    return 0.0


# ── geometry helpers, shared by both emitters ────────────────────────────
def _ramp_frame(route):
    """Rotation whose +z is the ramp's surface normal, +x points uphill."""
    g, c = math.radians(route.grade_deg), math.radians(route.camber_deg)
    u = np.array([math.cos(g), 0.0, math.sin(g)])
    v0 = np.array([0.0, 1.0, 0.0])
    # rotate v0 about u by the camber angle (Rodrigues)
    v = (v0 * math.cos(c) + np.cross(u, v0) * math.sin(c)
         + u * float(np.dot(u, v0)) * (1 - math.cos(c)))
    n = np.cross(u, v)
    return np.column_stack([u, v / np.linalg.norm(v), n / np.linalg.norm(n)])


def rot_to_quat(rot):
    """(w, x, y, z) from a rotation matrix — MJCF's quat order."""
    tr = rot[0, 0] + rot[1, 1] + rot[2, 2]
    if tr > 0:
        sq = math.sqrt(tr + 1.0) * 2
        w = 0.25 * sq
        xx = (rot[2, 1] - rot[1, 2]) / sq
        yy = (rot[0, 2] - rot[2, 0]) / sq
        zz = (rot[1, 0] - rot[0, 1]) / sq
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        sq = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2
        w = (rot[2, 1] - rot[1, 2]) / sq
        xx, yy, zz = 0.25 * sq, (rot[0, 1] + rot[1, 0]) / sq, \
            (rot[0, 2] + rot[2, 0]) / sq
    elif rot[1, 1] > rot[2, 2]:
        sq = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2
        w = (rot[0, 2] - rot[2, 0]) / sq
        xx, yy, zz = (rot[0, 1] + rot[1, 0]) / sq, 0.25 * sq, \
            (rot[1, 2] + rot[2, 1]) / sq
    else:
        sq = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2
        w = (rot[1, 0] - rot[0, 1]) / sq
        xx, yy, zz = (rot[0, 2] + rot[2, 0]) / sq, \
            (rot[1, 2] + rot[2, 1]) / sq, 0.25 * sq
    return (w, xx, yy, zz)


def rot_to_rpy(rot):
    """(roll, pitch, yaw) for SDF's <pose>, i.e. R = Rz(y)Ry(p)Rx(r)."""
    pitch = math.asin(max(-1.0, min(1.0, -rot[2, 0])))
    if abs(rot[2, 0]) < 0.999999:
        roll = math.atan2(rot[2, 1], rot[2, 2])
        yaw = math.atan2(rot[1, 0], rot[0, 0])
    else:
        roll, yaw = math.atan2(-rot[1, 2], rot[1, 1]), 0.0
    return (roll, pitch, yaw)


RAMP_THICKNESS = 0.30
DECK_THICKNESS = 0.10


@dataclass
class Box:
    name: str
    pos: Tuple[float, float, float]
    quat: Tuple[float, float, float, float]
    rpy: Tuple[float, float, float]
    half: Tuple[float, float, float]
    mu: float


@dataclass
class Field:
    name: str
    data: np.ndarray                  # metres, absolute elevation
    x0: float
    x1: float
    y0: float
    y1: float
    mu: float


def features(s):
    """Every solid in the Yard, ONCE. Both emitters read this list.

    This is the join point that makes cross-engine parity structural: the
    MJCF and the SDF are two renderings of this list, not two descriptions
    of the same intent.
    """
    p = s.params
    deck_z = p['deck']['z']
    dy0, dy1 = p['deck']['y']
    boxes, fields = [], []

    for key, route in s.routes.items():
        if key == 'c':
            g = np.zeros_like(s.rubble)
            ny, nx = g.shape
            xs = np.linspace(route.x_foot, route.x_top, nx)
            ys = np.linspace(route.y_centre - route.width / 2.0,
                             route.y_centre + route.width / 2.0, ny)
            plane = ((xs - route.x_foot)[None, :] * _tan(route.grade_deg)
                     + (ys - route.y_centre)[:, None]
                     * _tan(route.camber_deg))
            fields.append(Field(
                name='rubble', data=plane + s.rubble,
                x0=route.x_foot, x1=route.x_top,
                y0=ys[0], y1=ys[-1], mu=route.friction))
            continue
        rot = _ramp_frame(route)
        length = route.run / math.cos(math.radians(route.grade_deg))
        width = route.width / math.cos(math.radians(route.camber_deg))
        mid_x = (route.x_foot + route.x_top) / 2.0
        top = np.array([mid_x, route.y_centre,
                        (mid_x - route.x_foot) * _tan(route.grade_deg)])
        centre = top - rot[:, 2] * (RAMP_THICKNESS / 2.0)
        boxes.append(Box(
            name=f'route_{key}', pos=tuple(centre), quat=rot_to_quat(rot),
            rpy=rot_to_rpy(rot),
            half=(length / 2.0, width / 2.0, RAMP_THICKNESS / 2.0),
            mu=route.friction))

    mu_deck = p['friction']['default']
    sec = p['deck']['sections']

    def slab(name, x0, x1, y0, y1, top_z=deck_z, thick=DECK_THICKNESS):
        boxes.append(Box(
            name=name,
            pos=((x0 + x1) / 2.0, (y0 + y1) / 2.0, top_z - thick / 2.0),
            quat=(1.0, 0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0),
            half=((x1 - x0) / 2.0, (y1 - y0) / 2.0, thick / 2.0),
            mu=mu_deck))

    slab('deck_staging', *sec['staging']['x'], dy0, dy1)
    slab('deck_transition', *sec['transition']['x'], dy0, dy1)
    b = p['bridge']
    bw = b['width']['value']
    slab('bridge', *sec['bridge']['x'],
         b['y_centre'] - bw / 2.0, b['y_centre'] + bw / 2.0)
    slab('deck_approach', sec['approach']['x'][0], sec['bay']['x'][1],
         dy0, dy1)

    wb = p['washboard']
    wx0, wx1 = sec['washboard']['x']
    amp = wb['amplitude']['value']
    cell = wb['cell']
    nx = int(round((wx1 - wx0) / cell)) + 1
    ny = max(2, int(round((dy1 - dy0) / cell / 8)) + 1)
    xs = np.linspace(wx0, wx1, nx)
    prof = deck_z + amp * np.sin(2 * math.pi * (xs - wx0) / wb['wavelength'])
    fields.append(Field(name='washboard', data=np.tile(prof, (ny, 1)),
                        x0=wx0, x1=wx1, y0=dy0, y1=dy1, mu=mu_deck))

    d = p['descent']
    run = deck_z / _tan(d['grade_deg'])
    droute = RouteSample(
        key='descent', grade_deg=-d['grade_deg'], camber_deg=0.0,
        friction=mu_deck, width=d['width'], y_centre=d['y_centre'],
        x_top=p['deck']['x'][1] + run, z_top=0.0, run=run)
    rot = _ramp_frame(droute)
    length = run / math.cos(math.radians(d['grade_deg']))
    mid_x = p['deck']['x'][1] + run / 2.0
    top = np.array([mid_x, d['y_centre'], deck_z / 2.0])
    centre = top - rot[:, 2] * (RAMP_THICKNESS / 2.0)
    boxes.append(Box(
        name='descent', pos=tuple(centre), quat=rot_to_quat(rot),
        rpy=rot_to_rpy(rot),
        half=(length / 2.0, d['width'] / 2.0, RAMP_THICKNESS / 2.0),
        mu=mu_deck))
    return boxes, fields


# ── MuJoCo ───────────────────────────────────────────────────────────────
def _field_geom_z(f):
    """(z0, elevation, base) for an hfield placed at absolute heights."""
    lo, hi = float(f.data.min()), float(f.data.max())
    elev = max(hi - lo, 1e-4)
    return lo, elev, 0.60


def yard_mjcf_fragment(s, mu_wheel):
    """Assets, worldbody geoms and contact pairs for the Yard."""
    boxes, fields = features(s)
    assets, geoms, pairs = [], [], []

    for b in boxes:
        geoms.append(
            f'    <geom name="{b.name}" type="box" '
            f'pos="{b.pos[0]:.6f} {b.pos[1]:.6f} {b.pos[2]:.6f}" '
            f'quat="{b.quat[0]:.9f} {b.quat[1]:.9f} {b.quat[2]:.9f} '
            f'{b.quat[3]:.9f}" '
            f'size="{b.half[0]:.6f} {b.half[1]:.6f} {b.half[2]:.6f}" '
            f'rgba="0.55 0.55 0.55 1"/>')
    for f in fields:
        nrow, ncol = f.data.shape
        z0, elev, base = _field_geom_z(f)
        assets.append(
            f'    <hfield name="{f.name}" nrow="{nrow}" ncol="{ncol}" '
            f'size="{(f.x1 - f.x0) / 2.0:.6f} {(f.y1 - f.y0) / 2.0:.6f} '
            f'{elev:.6f} {base:.6f}"/>')
        geoms.append(
            f'    <geom name="{f.name}" type="hfield" hfield="{f.name}" '
            f'pos="{(f.x0 + f.x1) / 2.0:.6f} {(f.y0 + f.y1) / 2.0:.6f} '
            f'{z0:.6f}" rgba="0.55 0.55 0.55 1"/>')

    # Explicit pairs, for the same reason the flat model has them: MuJoCo
    # combines geom friction as the elementwise MAX, so with the wheels
    # pinned at the calibrated 0.4 every terrain below 0.4 would be a
    # silent no-op and the bottom of 2.5's 0.35-1.10 range unreachable.
    for name, mu in ([(b.name, b.mu) for b in boxes]
                     + [(f.name, f.mu) for f in fields]):
        for wheel, _, _, _ in _wheel_names():
            pairs.append(
                f'    <pair geom1="{name}" geom2="{wheel}_geom" '
                f'friction="{mu:.6f} {mu:.6f} 0.005 0.0001 0.0001" '
                f'solref="{_solref()} 1" solimp="{_solimp()} 0.99 0.001"/>')
    return ('\n'.join(assets), '\n'.join(geoms), '\n'.join(pairs), fields)


def _wheel_names():
    from coco_sim.mjcf import wheel_positions
    return wheel_positions()


def _solref():
    from coco_sim.mjcf import CONTACT_SOLREF
    return CONTACT_SOLREF


def _solimp():
    from coco_sim.mjcf import CONTACT_SOLIMP_D0
    return CONTACT_SOLIMP_D0


def build_yard_mjcf(s):
    """Full MJCF: the calibrated base from coco_sim.mjcf, plus the Yard."""
    from coco_sim.mjcf import WHEEL_FRICTION, build_mjcf
    base = build_mjcf()
    assets, geoms, pairs, fields = yard_mjcf_fragment(s, WHEEL_FRICTION)
    xml = base.replace(
        '  <worldbody>',
        (f'  <asset>\n{assets}\n  </asset>\n\n  <worldbody>\n{geoms}')
        if assets else f'  <worldbody>\n{geoms}')
    xml = xml.replace('  </contact>', f'{pairs}\n  </contact>')
    return xml, fields


def apply_hfields(model, fields):
    """Push elevation data into a compiled model. ~43 us, no recompile.

    This is what makes per-episode terrain affordable: recompiling the
    model for a new rubble seed would dominate the step budget, and
    MuJoCo lets the field be overwritten in place instead.
    """
    import mujoco
    for f in fields:
        hid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_HFIELD, f.name)
        lo, elev, _ = _field_geom_z(f)
        adr, nrow, ncol = (model.hfield_adr[hid], model.hfield_nrow[hid],
                           model.hfield_ncol[hid])
        if f.data.shape != (nrow, ncol):
            raise ValueError(
                f'hfield {f.name}: compiled grid is {nrow}x{ncol} but the '
                f'sample supplied {f.data.shape}. The grid shape is fixed '
                f'at compile time; vary the extent, not the resolution.')
        # extent and placement are runtime fields, so grade jitter moves
        # the field without a recompile
        model.hfield_size[hid] = [(f.x1 - f.x0) / 2.0, (f.y1 - f.y0) / 2.0,
                                  elev, 0.60]
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f.name)
        if gid >= 0:
            model.geom_pos[gid] = [(f.x0 + f.x1) / 2.0,
                                   (f.y0 + f.y1) / 2.0, lo]
        norm = (f.data - lo) / elev
        model.hfield_data[adr:adr + nrow * ncol] = norm.astype(
            np.float32).ravel()


# ── Gazebo ───────────────────────────────────────────────────────────────
def _stl_triangles(f):
    """Closed solid for one heightfield, on MUJOCO'S diagonal.

    The top surface uses the (i, j)-(i+1, j+1) split that HFIELD_DIAGONAL
    records, so the Gazebo mesh and the MuJoCo hfield are the same surface
    between nodes as well as at them. A skirt and a flat bottom close it,
    because an open shell is not a collision solid.
    """
    ny, nx = f.data.shape
    xs = np.linspace(f.x0, f.x1, nx)
    ys = np.linspace(f.y0, f.y1, ny)
    base = float(f.data.min()) - 0.30
    tris = []

    def add(v0, v1, v2, ref):
        tris.append(_facet(v0, v1, v2, ref))

    for i in range(ny - 1):
        for j in range(nx - 1):
            p00 = (xs[j], ys[i], f.data[i, j])
            p01 = (xs[j + 1], ys[i], f.data[i, j + 1])
            p10 = (xs[j], ys[i + 1], f.data[i + 1, j])
            p11 = (xs[j + 1], ys[i + 1], f.data[i + 1, j + 1])
            add(p00, p01, p11, (0, 0, 1))
            add(p00, p11, p10, (0, 0, 1))
    for j in range(nx - 1):
        for i, ref in ((0, (0, -1, 0)), (ny - 1, (0, 1, 0))):
            a = (xs[j], ys[i], f.data[i, j])
            b = (xs[j + 1], ys[i], f.data[i, j + 1])
            add(a, b, (xs[j + 1], ys[i], base), ref)
            add(a, (xs[j + 1], ys[i], base), (xs[j], ys[i], base), ref)
    for i in range(ny - 1):
        for j, ref in ((0, (-1, 0, 0)), (nx - 1, (1, 0, 0))):
            a = (xs[j], ys[i], f.data[i, j])
            b = (xs[j], ys[i + 1], f.data[i + 1, j])
            add(a, b, (xs[j], ys[i + 1], base), ref)
            add(a, (xs[j], ys[i + 1], base), (xs[j], ys[i], base), ref)
    c = [(f.x0, f.y0, base), (f.x1, f.y0, base),
         (f.x1, f.y1, base), (f.x0, f.y1, base)]
    add(c[0], c[1], c[2], (0, 0, -1))
    add(c[0], c[2], c[3], (0, 0, -1))
    return tris


def _facet(v0, v1, v2, ref):
    ux, uy, uz = (v1[i] - v0[i] for i in range(3))
    wx, wy, wz = (v2[i] - v0[i] for i in range(3))
    nx_, ny_, nz_ = (uy * wz - uz * wy, uz * wx - ux * wz, ux * wy - uy * wx)
    if nx_ * ref[0] + ny_ * ref[1] + nz_ * ref[2] < 0:
        v1, v2 = v2, v1
        nx_, ny_, nz_ = -nx_, -ny_, -nz_
    mag = math.sqrt(nx_ ** 2 + ny_ ** 2 + nz_ ** 2) or 1.0
    return ((nx_ / mag, ny_ / mag, nz_ / mag), v0, v1, v2)


def write_stl(path, triangles):
    """Binary STL, stdlib only.

    gazebo_models/scripts/gen_ramp.py has an identical writer and this is
    NOT imported from there on purpose: coco_sim must not depend on
    gazebo_models (CLAUDE.md rule 6 — colcon refuses to order the
    workspace if that edge appears, and it has broken twice). Twelve lines
    of struct.pack is the cheaper of the two costs.
    """
    with open(path, 'wb') as handle:
        handle.write(b'coco yard heightfield'.ljust(80, b'\0'))
        handle.write(struct.pack('<I', len(triangles)))
        for normal, v0, v1, v2 in triangles:
            handle.write(struct.pack('<3f', *normal))
            for v in (v0, v1, v2):
                handle.write(struct.pack('<3f', *v))
            handle.write(struct.pack('<H', 0))


GREY = '0.55 0.55 0.55 1'

# Everything in this world is GREY, ambient and diffuse only.
# coco_perception classifies targets by HUE, and its 16/16 result was
# measured on frames whose only saturated pixels were the four targets. A
# textured or coloured terrain would put competing hue in exactly those
# frames. The world is not the place to be decorative.
_SDF_HEADER = """<?xml version="1.0"?>
<!-- GENERATED by coco_sim.yard from coco_sim/worlds/yard_params.yaml.
     Do not edit by hand: regenerate. coco_world.world is NOT this file
     and is not touched by it - v1 keeps its 10/10 and its 19/20. -->
<sdf version="1.8">
  <world name="coco_yard">

    <physics name="default_physics" type="dart">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <gravity>0 0 -9.8</gravity>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <light name="fill" type="directional">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 5 0 0 0</pose>
      <diffuse>0.4 0.4 0.4 1</diffuse>
      <direction>0.5 -0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.4 0.4 0.4 1</ambient><diffuse>0.4 0.4 0.4 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

_SDF_FOOTER = """
  </world>
</sdf>
"""

_BOX_MODEL = """
    <model name="{name}">
      <static>true</static>
      <link name="link">
        <pose>{px:.6f} {py:.6f} {pz:.6f} {r:.9f} {p:.9f} {yw:.9f}</pose>
        <collision name="collision">
          <geometry><box><size>{sx:.6f} {sy:.6f} {sz:.6f}</size></box></geometry>
          <surface><friction><ode><mu>{mu:.6f}</mu><mu2>{mu:.6f}</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.6f} {sy:.6f} {sz:.6f}</size></box></geometry>
          <material><ambient>{grey}</ambient><diffuse>{grey}</diffuse></material>
        </visual>
      </link>
    </model>
"""

_MESH_MODEL = """
    <model name="{name}">
      <static>true</static>
      <link name="link">
        <pose>0 0 0 0 0 0</pose>
        <collision name="collision">
          <geometry><mesh><uri>{uri}/{name}.stl</uri></mesh></geometry>
          <surface><friction><ode><mu>{mu:.6f}</mu><mu2>{mu:.6f}</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><mesh><uri>{uri}/{name}.stl</uri></mesh></geometry>
          <material><ambient>{grey}</ambient><diffuse>{grey}</diffuse></material>
        </visual>
      </link>
    </model>
"""


def build_yard_sdf(s, mesh_uri='model://coco_yard/meshes'):
    """The Gazebo world, from the same feature list as the MJCF.

    Note what is NOT here: the platform, the four targets and their
    DetachableJoint magnets. Those are emitted as SDF strings inside
    full_world_robo.launch.py and they stay there, because that pipeline
    is what M6 measured 20/20 grasps through and it binds its child once
    on first spawn. The Yard generator owns terrain; the bay keeps its
    existing path.
    """
    boxes, fields = features(s)
    parts = [_BOX_MODEL.format(
        name=b.name, px=b.pos[0], py=b.pos[1], pz=b.pos[2],
        r=b.rpy[0], p=b.rpy[1], yw=b.rpy[2],
        sx=2 * b.half[0], sy=2 * b.half[1], sz=2 * b.half[2],
        mu=b.mu, grey=GREY) for b in boxes]
    parts += [_MESH_MODEL.format(name=f.name, uri=mesh_uri, mu=f.mu,
                                 grey=GREY) for f in fields]
    return _SDF_HEADER + ''.join(parts) + _SDF_FOOTER


def write_yard(outdir, s=None, mesh_uri='model://coco_yard/meshes'):
    """Write the SDF world and every heightfield STL to ``outdir``.

    Both artefacts come from the same ``features(s)`` call, so the world
    and the meshes cannot describe different terrain.
    """
    s = s or sample_yard(randomise=False)
    meshes = os.path.join(outdir, 'meshes')
    os.makedirs(meshes, exist_ok=True)
    _, fields = features(s)
    for f in fields:
        write_stl(os.path.join(meshes, f'{f.name}.stl'), _stl_triangles(f))
    world = os.path.join(outdir, 'coco_yard.world')
    with open(world, 'w') as handle:
        handle.write(build_yard_sdf(s, mesh_uri=mesh_uri))
    return world


def main():
    """Regenerate the Gazebo Yard assets."""
    import argparse
    ap = argparse.ArgumentParser(description='Generate the Yard world.')
    ap.add_argument('-o', '--outdir', required=True)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--randomise', action='store_true')
    args = ap.parse_args()
    s = sample_yard(seed=args.seed, randomise=args.randomise)
    print('wrote', write_yard(args.outdir, s))


if __name__ == '__main__':
    main()
