"""Geometry-kernel validation gates.

The mesh is the same design state as the spec, realized in 3D — so every
closed part's mesh volume must agree with the spec's derived property, and
the arc shape must honor the spec's own LE-tube length. Mesh tests need the
l1 extra (trimesh); the closed-form arc checks always run.
"""
import math

import pytest

from kytoon.geometry import HAS_TRIMESH, ArcWing, build
from kytoon.spec import load_all

SPECS = "specs"
needs_trimesh = pytest.mark.skipif(not HAS_TRIMESH,
                                   reason="l1 extra (trimesh) not installed")


@pytest.fixture(scope="module")
def specs():
    return {s.mk: s for s in load_all(SPECS)}


# --- closed-form arc checks (no trimesh needed) -------------------------------

def test_mk1_arc_pinned_by_le_tube_length(specs):
    """44 m developed LE on a 38 m span → h/b ≈ 0.25, flatter than the V3."""
    g = ArcWing.from_spec(specs["I"])
    assert g.arc_length == pytest.approx(44.0, rel=0.01)
    assert 0.20 < g.height_ratio < 0.30
    assert g.height_ratio < ArcWing.height_ratio     # flatter than default


def test_mk3_arc_falls_back_to_v3_shape(specs):
    """No LE tube on the spine → nothing pins the arc; V3 default holds."""
    g = ArcWing.from_spec(specs["III"])
    assert g.height_ratio == pytest.approx(ArcWing.height_ratio)


# --- mesh volume/area gates vs spec-derived properties -------------------------

@pytest.fixture(scope="module")
def scenes(specs):
    if not HAS_TRIMESH:
        pytest.skip("trimesh not installed")
    return {mk: build(s) for mk, s in specs.items()}


@needs_trimesh
def test_torus_mesh_volume_matches_spec(specs, scenes):
    mesh = scenes["IV"].geometry["torus"]
    assert mesh.is_watertight
    assert mesh.volume == pytest.approx(specs["IV"].torus.volume, rel=0.03)


@needs_trimesh
def test_lobe_mesh_volume_matches_spec(specs, scenes):
    mesh = scenes["II"].geometry["lobe"]
    assert mesh.is_watertight
    assert mesh.volume == pytest.approx(specs["II"].lobe.volume, rel=0.02)


@needs_trimesh
def test_le_tube_mesh_volume_matches_spec(specs, scenes):
    mesh = scenes["I"].geometry["le_tube"]
    assert mesh.volume == pytest.approx(specs["I"].le_tube.volume, rel=0.05)


@needs_trimesh
def test_spar_mesh_volume_matches_spec(specs, scenes):
    mesh = scenes["III"].geometry["spar"]
    assert mesh.volume == pytest.approx(specs["III"].spar.volume, rel=0.03)


@needs_trimesh
def test_fatwing_lofted_body_volume_matches_spec(specs, scenes):
    """The lofted manta body carries the spec's closed-form He volume
    (K_A·t·c section area integrated over the linear taper)."""
    fw = specs["V"].fat_wing
    body = scenes["V"].geometry["body"]
    assert body.is_watertight
    assert abs(body.volume) == pytest.approx(fw.volume, rel=0.05)


@needs_trimesh
def test_fatwing_has_three_tether_fixtures(scenes):
    names = set(scenes["V"].geometry.keys())
    assert {"fixture_port", "fixture_main", "fixture_stbd"} <= names


@needs_trimesh
def test_side_delta_mesh_area_matches_spec(specs, scenes):
    """Mk II's two side deltas together carry the spec's wing area."""
    total = (scenes["II"].geometry["wing_stbd"].area
             + scenes["II"].geometry["wing_port"].area)
    assert total == pytest.approx(specs["II"].canopy.area, rel=0.02)


@needs_trimesh
def test_canopy_mesh_area_band(specs, scenes):
    """Flat (mesh) area exceeds projected (spec) area by the arc factor —
    inside a plausible band, never below 1."""
    for mk in ("I", "III"):
        area = scenes[mk].geometry["canopy"].area
        ratio = area / specs[mk].canopy.area
        assert 1.0 < ratio < 1.35, (mk, ratio)


@needs_trimesh
def test_all_specs_build_and_export(specs, tmp_path):
    from kytoon.geometry import export
    for s in specs.values():
        paths = export(s, tmp_path)
        assert all(p.stat().st_size > 5_000 for p in paths)


@needs_trimesh
def test_blimp_alternate_hull_volume():
    """The retired blimp stays buildable (specs/alternates/, off-fleet)."""
    from kytoon.spec import load_spec
    s = load_spec("specs/alternates/mk5a_blimp.yaml")
    mesh = build(s).geometry["hull"]
    assert mesh.is_watertight
    assert mesh.volume == pytest.approx(s.hull.volume, rel=0.02)
