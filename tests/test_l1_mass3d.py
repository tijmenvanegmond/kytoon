"""Stage 1 gates: 3D mass properties for the lateral model.

The contract has two halves. First, exact reduction: the tensor and the
6×6 added-mass matrix must reproduce every planar term
`l1_trim.mass_props` already uses, *at the same configuration* — if they
don't, the lateral model is wrong and everything built on it inherits
the error. Second, the two lateral facts that decide the modes: roll is
added-mass dominated, and sideslip added mass is negligible.

Both `mass_props_3d` and `l1_trim.mass_props` now default to the SPEC's
panel fold (Γ = 10° since 2026-07-25), so tests that mean the flat
reference loft say `dihedral_deg=0.0` out loud.
"""
import math

import numpy as np
import pytest

from kytoon.solvers.l1_mass3d import (
    HAS_L1, attach_point_3d, mass_props_3d,
)
from kytoon.solvers.l1_trim import attach_point, mass_props
from kytoon.spec import load_all

needs_l1 = pytest.mark.skipif(not HAS_L1,
                              reason="l1 extra (aerosandbox) not installed")


@pytest.fixture(scope="module")
def specs():
    return {s.mk: s for s in load_all("specs")}


@pytest.fixture(scope="module")
def p3(specs):
    return mass_props_3d(specs["V"])


@needs_l1
def test_refuses_non_fatwing(specs):
    with pytest.raises(ValueError, match="lofted fat wing"):
        mass_props_3d(specs["I"])


# --- exact reduction to the planar model ---------------------------------

@needs_l1
def test_planar_terms_reproduce_l1_trim(specs, p3):
    """Every quantity the longitudinal model uses must come back
    unchanged — at the spec's fold, which is the configuration both
    models actually fly."""
    p2 = mass_props(specs["V"])
    assert p3.m_total == pytest.approx(p2.m_total, rel=1e-12)
    assert p3.buoyancy_n == pytest.approx(p2.buoyancy_n, rel=1e-9)
    assert p3.r_cg[0] == pytest.approx(p2.r_cg[0], abs=1e-9)
    assert p3.r_cg[2] == pytest.approx(p2.r_cg[1], abs=1e-9)
    assert p3.r_cb[0] == pytest.approx(p2.r_cb[0], abs=1e-9)
    assert p3.r_cb[2] == pytest.approx(p2.r_cb[1], abs=1e-9)
    assert p3.i_pitch == pytest.approx(p2.i_yy, rel=1e-9)
    # 3e-4: same trapezoid rule and the same cos²Γ normal projection, but
    # 401 strips here vs l1_trim's 201, and the integrand has kinks at the
    # root from chord_at's |y| and from the fold
    assert p3.added[2, 2] == pytest.approx(p2.m_added_z, rel=3e-4)
    assert p3.added[0, 0] == pytest.approx(p2.m_added_x, rel=1e-9)


@needs_l1
def test_pitch_added_inertia_exceeds_planar_by_the_surge_offset(specs, p3):
    """The one term where 3D is not a pure restatement: surge added mass
    acts at the CG's vertical offset, so it contributes m_x·Δz² to pitch
    added inertia. `l1_trim` treats m_added_x and i_added as independent
    diagonal entries and omits it — worth ~75 kg·m² (0.4 %). Anchoring
    the difference rather than loosening a tolerance keeps the reduction
    honest.
    """
    # pinned to the FLAT loft on both sides: the identity assumes the
    # strips lie at z = 0, which folding breaks by construction
    p2 = mass_props(specs["V"], dihedral_deg=0.0)
    pf = mass_props_3d(specs["V"], dihedral_deg=0.0)
    dz = pf.r_cg[2]
    assert pf.added[4, 4] - p2.m_added_x * dz ** 2 == pytest.approx(
        p2.i_added, rel=1e-4)


@needs_l1
def test_attach_point_reduces_to_planar(specs):
    s = specs["V"]
    for pos in s.bridle.positions:
        a3 = attach_point_3d(s, pos)
        a2 = attach_point(s, pos)
        assert a3[0] == pytest.approx(a2[0], abs=1e-12)
        assert a3[2] == pytest.approx(a2[1], abs=1e-12)
    # ... and carries the span offset the planar model drops
    assert attach_point_3d(s, s.bridle.positions[2])[1] == pytest.approx(
        (s.bridle.positions[2] - 0.5) * s.fat_wing.span)


@needs_l1
def test_lateral_symmetry(p3):
    """The airframe is symmetric about y = 0: only I_xz survives."""
    # 1e-4 not 1e-12: trimesh's vertex merge leaves the tip caps very
    # slightly asymmetric, worth ~0.1 kg·m² out of 12 400
    assert abs(p3.inertia[0, 1]) < 1e-4 * p3.i_roll
    assert abs(p3.inertia[1, 2]) < 1e-4 * p3.i_yaw
    assert p3.inertia[0, 2] == pytest.approx(p3.inertia[2, 0], rel=1e-9)
    assert abs(p3.inertia[0, 2]) > 0.0        # sweep gives a real product


# --- the two facts that decide the lateral modes -------------------------

@needs_l1
def test_roll_is_added_mass_dominated(p3):
    """πρc²/4 picks up a y² lever in roll, so the air the wing swings is
    an order of magnitude heavier than the wing. A lateral model that
    drops this rolls ~12× too fast."""
    assert p3.added_roll > 8.0 * p3.i_roll
    # pitch has no such lever — its arm is chordwise, metres not tens
    assert p3.added[4, 4] < 10.0 * p3.i_pitch


@needs_l1
def test_sideslip_added_mass_is_negligible(specs, p3):
    """Edge-on a wing displaces almost nothing. Strip theory has no
    spanwise term at all, so this is structurally zero on the flat loft.
    The spec's Γ = 10° tilts the panel normals and buys a few percent —
    still small enough that sway inertia is essentially bare airframe
    mass, which is what the lateral modes care about."""
    flat = mass_props_3d(specs["V"], dihedral_deg=0.0)
    assert flat.added_sway < 1e-6 * flat.added_heave
    assert p3.added_sway < 0.05 * p3.added_heave


# --- dihedral, carried from the start ------------------------------------

@needs_l1
def test_fold_preserves_volume(specs):
    """The fold is a rigid rotation, so it must not create or destroy
    helium — only move where it sits."""
    # 1 %, not exact: each panel rotates rigidly, but the two of them
    # meeting at the root form a real crease whose wedge volume differs
    # from the flat loft. That is geometry, not error.
    flat = mass_props_3d(specs["V"], dihedral_deg=0.0)
    for g in (10.0, 20.0, 30.0):
        assert mass_props_3d(specs["V"], dihedral_deg=g).volume == \
            pytest.approx(flat.volume, rel=1e-2)


@needs_l1
def test_dihedral_moves_cb_vertically_not_chordwise(specs):
    """Refines the reason Γ was worth carrying: folding the panels moves
    the CB, but it moves it UP, not aft. The single-confluence pitch
    instability rests on the CB being ~2 m aft of the pull point in x,
    and Γ barely touches that — so dihedral is a roll-stability lever,
    not a fix for the longitudinal finding.
    """
    flat = mass_props_3d(specs["V"], dihedral_deg=0.0)
    folded = mass_props_3d(specs["V"], dihedral_deg=30.0)
    assert abs(folded.r_cb[0] - flat.r_cb[0]) < 0.05      # chordwise: no
    assert folded.r_cb[2] - flat.r_cb[2] > 2.0            # vertical: yes


@needs_l1
def test_dihedral_raises_roll_pendulum_stiffness(specs):
    """The payoff: CB rising above the main pull point is buoyant roll
    stiffness B·Δz, and Γ = 30° roughly doubles it."""
    s = specs["V"]
    att = attach_point_3d(s, s.bridle.positions[1])       # centre, unfolded
    flat = mass_props_3d(s, dihedral_deg=0.0)
    folded = mass_props_3d(s, dihedral_deg=30.0)
    k_flat = flat.buoyancy_n * abs(flat.r_cb[2] - att[2])
    k_fold = folded.buoyancy_n * abs(folded.r_cb[2] - att[2])
    assert k_fold > 2.0 * k_flat


@needs_l1
def test_dihedral_creates_sideslip_added_mass(specs):
    """Tilted panel normals gain a y component, so sway added mass grows
    from nothing — a consistency check on the strip orientation."""
    q = mass_props_3d(specs["V"], dihedral_deg=30.0)
    assert 0.05 < q.added_sway / q.added_heave < 0.5
