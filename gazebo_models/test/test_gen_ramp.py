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

"""The parametric ramp wedge (gen_ramp.py) — pure geometry, no simulator.

The wedge replaced an unclimbable CAD mesh, so the property that matters is
that the driving surface is at *exactly* the requested grade and starts flush
with the ground (zero step). Stdlib only, matching the generator: numpy is not
guaranteed in this interpreter.
"""
import math
import os
import struct
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from gen_ramp import wedge_triangles, write_stl  # noqa: E402


def _verts(tris):
    """Every vertex across all facets; each facet is (normal, v0, v1, v2)."""
    return [v for _, *vs in tris for v in vs]


def test_exactly_eight_facets():
    """A triangular prism is 8 triangles: 2 ends + 3 faces x 2."""
    assert len(wedge_triangles(18.0, 2.5, 2.0)) == 8


@pytest.mark.parametrize('angle', [12.0, 18.0, 24.0, 30.0])
def test_driving_surface_matches_requested_angle(angle):
    tris = wedge_triangles(angle, 2.5, 2.0)
    verts = _verts(tris)
    run = max(v[0] for v in verts)
    rise = max(v[2] for v in verts)
    measured = math.degrees(math.atan2(rise, run))
    assert measured == pytest.approx(angle, abs=0.1)


def test_foot_is_flush_with_the_ground():
    """The foot edge must be at z=0 — a step is what made the old mesh
    unclimbable."""
    verts = _verts(wedge_triangles(18.0, 2.5, 2.0))
    assert min(v[2] for v in verts) == pytest.approx(0.0, abs=1e-6)


def test_summit_height_is_run_times_tan():
    run, angle = 2.5, 18.0
    verts = _verts(wedge_triangles(angle, run, 2.0))
    rise = max(v[2] for v in verts)
    assert rise == pytest.approx(run * math.tan(math.radians(angle)), abs=1e-6)


def test_bounds_are_run_by_width_by_rise():
    run, width, angle = 2.5, 2.0, 18.0
    verts = _verts(wedge_triangles(angle, run, width))
    xs = [v[0] for v in verts]
    ys = [v[1] for v in verts]
    assert min(xs) == pytest.approx(0.0) and max(xs) == pytest.approx(run)
    assert min(ys) == pytest.approx(-width / 2)
    assert max(ys) == pytest.approx(width / 2)


def test_normals_are_unit_length():
    for normal, *_ in wedge_triangles(18.0, 2.5, 2.0):
        assert math.sqrt(sum(c * c for c in normal)) == pytest.approx(1.0, abs=1e-5)


def test_written_stl_round_trips(tmp_path):
    """A written binary STL parses back to the same 8 facets."""
    out = tmp_path / 'w.stl'
    write_stl(str(out), wedge_triangles(18.0, 2.5, 2.0))
    with open(out, 'rb') as f:
        f.read(80)
        n = struct.unpack('<I', f.read(4))[0]
        assert n == 8
        for _ in range(n):
            f.read(12 * 4 + 2)   # normal + 3 verts + attr
        assert f.read() == b''    # nothing trailing


def test_committed_curriculum_meshes_exist_and_are_correct():
    """The three curriculum wedges the launch file selects by angle are
    present in meshes/ and carry the grade their filename claims."""
    meshes = os.path.join(os.path.dirname(__file__), '..', 'meshes')
    for angle in (12, 18, 24):
        path = os.path.join(meshes, f'ramp_wedge_{angle}.stl')
        assert os.path.exists(path), f'missing committed wedge {path}'
        with open(path, 'rb') as f:
            f.read(80)
            n = struct.unpack('<I', f.read(4))[0]
            verts = []
            for _ in range(n):
                f.read(12)
                verts += [struct.unpack('<3f', f.read(12)) for _ in range(3)]
                f.read(2)
        assert n == 8
        run = max(v[0] for v in verts)
        rise = max(v[2] for v in verts)
        assert math.degrees(math.atan2(rise, run)) == pytest.approx(angle, abs=0.1)
