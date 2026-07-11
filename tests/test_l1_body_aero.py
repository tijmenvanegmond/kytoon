"""L1 body-interference aero gates (Mk II lobe+wing, Mk V hull+wing).

Runs only with the l1 extra (aerosandbox). AeroBuildup is semi-empirical —
these gates check bounds and orderings, not benchmark agreement:
the rigid-smooth model must sit BELOW the conservative hand-picked drag,
resultants must stay in a band (cl dominates cr), and the Mk V coverage
claim must hold at BOTH bounds.
"""
import pytest

from kytoon.solvers.l1_body_aero import (
    HAS_AEROSANDBOX,
    solve,
    v_max_tether_with,
)
from kytoon.spec import load_all

SPECS = "specs"
needs_asb = pytest.mark.skipif(not HAS_AEROSANDBOX,
                               reason="l1 extra (aerosandbox) not installed")


@pytest.fixture(scope="module")
def specs():
    return {s.mk: s for s in load_all(SPECS)}


@pytest.fixture(scope="module")
def mk5(specs):
    if not HAS_AEROSANDBOX:
        pytest.skip("aerosandbox not installed")
    return solve(specs["V"])


@pytest.fixture(scope="module")
def mk2(specs):
    if not HAS_AEROSANDBOX:
        pytest.skip("aerosandbox not installed")
    return solve(specs["II"])


# --- Mk V ---------------------------------------------------------------------

@needs_asb
def test_mk5_operating_cl_reached_pre_stall(mk5):
    assert mk5.op_alpha is not None
    assert mk5.op_alpha < 12


@needs_asb
def test_mk5_rigid_model_sits_below_handpicked_drag(mk5, specs):
    """Ordering check: rigid smooth hull (lower bound) must undercut the
    conservative spec cd_op. If it doesn't, one of the two is wrong."""
    assert mk5.cd_op_l1 < specs["V"].canopy.cd_op


@needs_asb
def test_mk5_resultant_band(mk5):
    # cl dominates cr, so even a 3x cd disagreement stays in a tight band
    assert 0.80 < mk5.cr_ratio < 1.10


@needs_asb
def test_mk5_coverage_claim_holds_at_both_bounds(mk5, specs):
    """The single-kytoon claim (v_max > 20) must survive BOTH the
    conservative spec cr and the optimistic L1 cr."""
    assert v_max_tether_with(specs["V"], mk5.cr_op_spec) > 20
    assert v_max_tether_with(specs["V"], mk5.cr_op_l1) > 20


@needs_asb
def test_hull_drag_in_plausible_bluff_band(mk5, specs):
    """CD at alpha 0, re-referenced to hull frontal area, must land in the
    airship band (Hoerner-ish 0.03–0.3) — catches unit/reference blunders."""
    import math
    cd0 = min(p.cd for p in mk5.clean.pts)
    frontal = math.pi / 4 * specs["V"].hull.diameter ** 2
    cd_frontal = cd0 * specs["V"].canopy.area / frontal
    assert 0.03 < cd_frontal < 0.30


# --- Mk II --------------------------------------------------------------------

@needs_asb
def test_mk2_resultant_band(mk2):
    assert 0.80 < mk2.cr_ratio < 1.10


@needs_asb
def test_mk2_wake_blanketing_flagged(mk2):
    assert any("blanketing" in f for f in mk2.flags)


# --- non-hybrids refuse -------------------------------------------------------

@needs_asb
def test_non_hybrids_rejected(specs):
    for mk in ("I", "III", "IV"):
        with pytest.raises(ValueError):
            solve(specs[mk])
