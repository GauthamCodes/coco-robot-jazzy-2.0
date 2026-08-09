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
Cross-engine parity, and the assumptions it rests on.

The full test drops probes in BOTH simulators and compares where they
settle; that needs Gazebo and is run as a session measurement (numbers in
docs/RESULTS.md). What lives here is everything checkable without a
simulator launch, plus the one fact the Gazebo half silently depends on:
MuJoCo's heightfield triangulation diagonal, RE-DERIVED from MuJoCo
rather than trusted, so a version bump fails a test instead of quietly
warping the evaluation world.
"""
import mujoco

import numpy as np

from coco_sim.probes import SMALL_PROBE, WHEEL_PROBE, probe_points
from coco_sim.yard import (HFIELD_DIAGONAL, apply_hfields, build_yard_mjcf,
                           height, sample_yard)

import pytest


@pytest.fixture(scope='module')
def yard():
    return sample_yard(seed=0, randomise=False)


@pytest.fixture(scope='module')
def compiled(yard):
    xml, fields = build_yard_mjcf(yard)
    model = mujoco.MjModel.from_xml_string(xml)
    apply_hfields(model, fields)
    data = mujoco.MjData(model)
    # park the robot far off the Yard so it never intercepts a probe ray
    data.qpos[0:3] = [60.0, 60.0, 0.06]
    mujoco.mj_forward(model, data)
    return model, data


def _ray(model, data, x, y):
    gid = np.zeros(1, dtype=np.int32)
    dist = mujoco.mj_ray(model, data, np.array([x, y, 4.0]),
                         np.array([0.0, 0.0, -1.0]), None, 1, -1, gid)
    return (4.0 - dist) if dist >= 0 else None


def test_mujoco_hfield_diagonal_is_still_what_the_stl_writer_assumes():
    """Re-derive the triangulation convention from MuJoCo itself.

    A 2x2 field with only corner (i=1, j=1) raised rays to 0.5 at the cell
    centre if the split joins (i, j) to (i+1, j+1), and to 0.0 if it joins
    (i, j+1) to (i+1, j). Get this wrong in the Gazebo STL and both
    engines still agree at every grid NODE while the surface between them
    differs by the full cell relief — which no sampling test would catch.
    """
    xml = """<mujoco>
     <asset><hfield name="h" nrow="2" ncol="2" size="1 1 1 0.5"/></asset>
     <worldbody><geom name="g" type="hfield" hfield="h"/></worldbody>
    </mujoco>"""
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    model.hfield_data[:] = np.array([0, 0, 0, 1], dtype=np.float32)
    mujoco.mj_forward(model, data)
    centre = _ray(model, data, 0.0, 0.0)
    assert centre == pytest.approx(0.5, abs=1e-3), (
        f'MuJoCo {mujoco.__version__} no longer splits heightfield cells on '
        f'the {HFIELD_DIAGONAL} diagonal. coco_sim.yard._stl_triangles '
        f'emits the Gazebo mesh on that assumption, so the two worlds are '
        f'now different surfaces between grid nodes.')
    # the off-diagonal corners confirm it is that diagonal and not merely
    # a symmetric interpolation
    assert _ray(model, data, 0.5, -0.5) == pytest.approx(0.25, abs=1e-3)
    assert _ray(model, data, -0.5, 0.5) == pytest.approx(0.25, abs=1e-3)


def test_the_analytic_surface_is_what_mujoco_actually_built(yard, compiled):
    """height() is the design; this checks the engine agrees with it.

    Sampled at fractional cell positions, never on a node — on-node
    samples agree even when the triangulation is wrong.
    """
    model, data = compiled
    worst = 0.0
    for label, x, y, _z in probe_points(yard, SMALL_PROBE):
        if 'void' in label or 'underdeck' in label:
            continue          # these are deliberately not on the surface
        got = _ray(model, data, x, y)
        if got is None:
            continue
        worst = max(worst, abs(got - height(x, y, yard)))
    # the residual is heightfield discretisation on the washboard's
    # sinusoid, not a placement error
    assert worst < 1.0e-3, f'analytic vs MuJoCo disagree by {worst * 1e3:.2f} mm'


def test_probes_never_land_on_a_grid_node(yard):
    """The one place a wrong triangulation still gives the right answer."""
    route_c = yard.routes['c']
    cell = yard.params['routes']['c']['rubble']['cell']
    for label, x, y, _z in probe_points(yard, SMALL_PROBE):
        if not label.startswith('rubble'):
            continue
        frac = ((x - route_c.x_foot) / cell) % 1.0
        assert 0.05 < frac < 0.95, f'{label} sits on a lattice node'


def test_concave_features_are_probed_from_inside(yard):
    """A probe started above a cavity settles on its roof and reports
    nothing about the cavity. The first run of this test did exactly that
    at the curb and reported a clean pass."""
    pts = {p[0]: p for p in probe_points(yard, SMALL_PROBE)}
    deck_z = yard.params['deck']['z']
    under = [p for k, p in pts.items() if k.startswith('underdeck')]
    assert under, 'no under-deck cavity probes'
    for _label, _x, _y, z0 in under:
        assert z0 is not None and z0 < deck_z / 2

    void = [p for k, p in pts.items() if k.startswith('bridge_void')]
    assert void
    for _label, _x, _y, z0 in void:
        assert z0 is not None and z0 > deck_z


def test_probe_sets_are_spaced_so_probes_cannot_collide(yard):
    """Two spheres closer than 2r settle on each other, which reads as a
    terrain disagreement in whichever engine resolves the pile first."""
    for radius in (WHEEL_PROBE, SMALL_PROBE):
        pts = probe_points(yard, radius)
        assert len(pts) > 80
        for i, (_l1, x1, y1, _z1) in enumerate(pts):
            for _l2, x2, y2, _z2 in pts[i + 1:]:
                assert (x1 - x2) ** 2 + (y1 - y2) ** 2 >= (2 * radius) ** 2
