"""Stage 4 gates: validate the lateral model before any GDScript.

Two independent checks, as planned:

  * **Symmetry decoupling.** At β = 0 with no roll, the linearisation
    must split cleanly into longitudinal (surge, heave, pitch) and
    lateral (sway, roll, yaw) blocks. This is exact, not approximate —
    any cross-participation means a sign or frame error somewhere in
    Stages 1–3, which is precisely the failure mode this stack is prone
    to.
  * **The roll mode as a probe of roll added inertia**, the least
    trusted number in Stage 1. Its frequency is √(k_roll/I_total), and
    ignoring added inertia would put it 3.6× too high — so the mode
    genuinely discriminates rather than merely being consistent.
"""
import math

import numpy as np
import pytest

from kytoon.solvers.l1_dyn3d import HAS_L1, mass_matrix, modes, solve
from kytoon.solvers.l1_mass3d import mass_props_3d
from kytoon.spec import load_all

needs_l1 = pytest.mark.skipif(not HAS_L1,
                              reason="l1 extra (aerosandbox) not installed")

LONGITUDINAL = {"surge", "heave", "pitch"}
LATERAL = {"sway", "roll", "yaw"}
LON_IDX = [0, 2, 4]
LAT_IDX = [1, 3, 5]


@pytest.fixture(scope="module")
def specs():
    return {s.mk: s for s in load_all("specs")}


@pytest.fixture(scope="module")
def rep(specs):
    return solve(specs["V"], wind=12.0)


@needs_l1
def test_refuses_non_fatwing(specs):
    with pytest.raises(ValueError, match="lofted fat wing"):
        solve(specs["I"])


# --- structural validation -------------------------------------------------

@needs_l1
def test_longitudinal_and_lateral_decouple(rep):
    """Exact consequence of the airframe's symmetry about y = 0. A mode
    may live in one block or the other, never both."""
    # 1e-4 is set by the finite-difference step in the Jacobians, not by
    # physics: real cross-coupling would be O(0.1), so this still catches
    # a sign or frame error by four orders of magnitude.
    for m in rep.modes:
        lon = float(m.participation[LON_IDX].sum())
        lat = float(m.participation[LAT_IDX].sum())
        assert min(lon, lat) < 1e-4, (
            f"mode {m.eig:.3f} mixes blocks: longitudinal {lon:.3e}, "
            f"lateral {lat:.3e} — a sign or frame error in Stages 1-3")


@needs_l1
def test_mass_matrix_is_symmetric_positive_definite(specs):
    """Generalised mass must be a valid inertia: symmetric, and every
    eigenvalue positive. Added-mass coupling terms are the easy way to
    break this."""
    props = mass_props_3d(specs["V"])
    m = mass_matrix(props, np.eye(3))
    assert np.allclose(m, m.T, rtol=1e-9, atol=1e-6)
    assert np.linalg.eigvalsh(m).min() > 0.0


@needs_l1
def test_both_families_of_mode_are_present(rep):
    """The lateral half is the whole point: the planar model has no
    representation for sway, roll or yaw."""
    kinds = {m.kind for m in rep.modes}
    assert kinds & LONGITUDINAL
    assert LATERAL <= kinds or len(kinds & LATERAL) >= 2


# --- the added-inertia probe ----------------------------------------------

@needs_l1
def test_roll_mode_matches_the_closed_form(specs, rep):
    """√(k_roll / (I_roll + A_roll)) computed two ways: by hand from the
    stiffness and Stage 1's inertia, and from the eigen-decomposition of
    the full 12-state system.

    Tight on the flat loft, where roll is a clean single-DOF mode. The
    spec's Γ = 10° couples roll into sway and yaw, so the 1-DOF closed
    form is a worse idealisation there — 8 %, which is still nowhere
    near the 3.6× error that dropping added inertia would cause.
    """
    flat = solve(specs["V"], dihedral_deg=0.0, fin_area_m2=0.0)
    roll_flat = flat.of_kind("roll")
    assert roll_flat is not None
    assert abs(roll_flat.eig.imag) == pytest.approx(
        flat.roll_freq_closed_form, rel=0.05)

    roll = rep.of_kind("roll")
    assert roll is not None
    assert abs(roll.eig.imag) == pytest.approx(
        rep.roll_freq_closed_form, rel=0.12)


@needs_l1
def test_roll_mode_discriminates_added_inertia(rep):
    """Stage 1 claims roll added inertia is ~12× structural. If that were
    wrong the roll mode would sit somewhere else entirely — this is a
    3×-plus discriminator, so the check has teeth."""
    assert rep.roll_freq_no_added / rep.roll_freq_closed_form > 3.0


@needs_l1
def test_roll_inertia_is_added_mass_dominated(rep):
    assert rep.i_roll_total > 8.0 * rep.i_roll_struct


# --- stability ------------------------------------------------------------

@needs_l1
def test_longitudinal_dynamics_stay_stable(rep):
    """Everything the planar programme established must survive going
    3D: the longitudinal block is still fully damped."""
    assert rep.max_real_longitudinal < 0.0


@needs_l1
def test_bare_reference_diverges_laterally(specs):
    """The FINDING that drove the design change, kept as a regression on
    the bare configuration: flat loft, no fin — what Mk V was before
    2026-07-25 — diverges laterally at ~+1.5 /s. Something the
    longitudinal programme could not have seen at all.

    It survives dropping control-line stiffness 100×, moving the control
    attachments chordwise, and changing pod standoff; a fin alone at
    Γ = 0 only gets it to +0.38 /s.
    """
    bare = solve(specs["V"], dihedral_deg=0.0, fin_area_m2=0.0)
    assert bare.max_real_lateral > 1.0
    assert any("LATERAL DIVERGENCE" in f for f in bare.flags)


@needs_l1
def test_spec_configuration_is_laterally_stable(rep):
    """...and the adopted Γ = 10° + 12 m² fin cures it. Thin margin, so
    this is the gate that notices if anything erodes it."""
    assert rep.max_real_lateral < 0.0
    assert not any("LATERAL DIVERGENCE" in f for f in rep.flags)


@needs_l1
def test_dihedral_fixes_the_sway_damping(specs, rep):
    """The dominant driver, isolated. On the bare wing CY_β > 0 means a
    side force in the *same* direction as the slip, so the air feeds the
    sway: C[1,1] > 0, worth ~0.8 /s on its own. Γ = 10° is chosen
    precisely because it drives CY_β to ≈ 0 — the design point sits
    where the least-trusted derivative stops mattering."""
    bare = solve(specs["V"], dihedral_deg=0.0, fin_area_m2=0.0)
    assert bare.sway_damping > 0.0
    assert bare.sway_damping / bare.spec.total_mass > 0.5
    assert rep.sway_damping < bare.sway_damping


@needs_l1
def test_the_stable_island_is_narrow(specs):
    """Why the spec says Γ = 10° AND 12 m², not one or the other. The two
    lateral problems pull opposite ways — sway damping wants dihedral,
    yaw wants none plus a fin — so the stable region is an island, and
    each neighbouring configuration must still diverge. If one of them
    goes stable the island has moved and the spec deserves a re-read.
    """
    for gamma, fin in ((0.0, 12.0), (20.0, 12.0), (10.0, 0.0)):
        off = solve(specs["V"], wind=12.0, dihedral_deg=gamma,
                    fin_area_m2=fin)
        assert off.max_real_lateral > 0.0, (
            f"Γ={gamma}, fin={fin} m² should NOT be stable — if it is, "
            "the island moved and the spec should be revisited")
    on = solve(specs["V"], wind=12.0, dihedral_deg=10.0, fin_area_m2=12.0)
    assert on.max_real_lateral < 0.0
    assert on.max_real_longitudinal < 0.0


@needs_l1
def test_divergence_is_slower_than_the_roll_mode(rep):
    """Sanity on timescales: whatever is diverging must be slower than
    the rig's own roll response, or the linearisation is meaningless."""
    roll = rep.of_kind("roll")
    assert roll is not None
    assert rep.max_real_lateral < abs(roll.eig.imag)


@needs_l1
def test_pitch_mode_is_in_the_planar_family(rep):
    """The longitudinal answer must not have moved: `l1_trim` puts the
    pitch mode near 3 rad/s, well damped."""
    pitch = rep.of_kind("pitch")
    assert pitch is not None
    assert 1.0 < abs(pitch.eig.imag) < 8.0
    assert pitch.eig.real < 0.0
