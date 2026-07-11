"""L1 body-interference aero gates (Mk II lobe+wing).

Runs only with the l1 extra (aerosandbox). AeroBuildup is semi-empirical —
these gates check bounds and orderings, not benchmark agreement. (Mk V left
this suite at the 2026-07-11 fat-wing pivot; its gates live in
test_l1_aero.py now.)
"""
import pytest

from kytoon.solvers.l1_body_aero import HAS_AEROSANDBOX, solve
from kytoon.spec import load_all

SPECS = "specs"
needs_asb = pytest.mark.skipif(not HAS_AEROSANDBOX,
                               reason="l1 extra (aerosandbox) not installed")


@pytest.fixture(scope="module")
def specs():
    return {s.mk: s for s in load_all(SPECS)}


@pytest.fixture(scope="module")
def mk2(specs):
    if not HAS_AEROSANDBOX:
        pytest.skip("aerosandbox not installed")
    return solve(specs["II"])


@needs_asb
def test_mk2_operating_cl_reached_pre_stall(mk2):
    assert mk2.op_alpha is not None
    assert mk2.op_alpha < 12


@needs_asb
def test_mk2_rigid_model_sits_below_handpicked_drag(mk2, specs):
    """Ordering check: the rigid revolution-body model (drag lower bound)
    must undercut the conservative spec cd_op."""
    assert mk2.cd_op_l1 < specs["II"].canopy.cd_op


@needs_asb
def test_mk2_resultant_band(mk2):
    # cl dominates cr, so even a 3x cd disagreement stays in a tight band
    assert 0.80 < mk2.cr_ratio < 1.10


@needs_asb
def test_mk2_wake_blanketing_flagged(mk2):
    assert any("blanketing" in f for f in mk2.flags)


@needs_asb
def test_non_hybrids_rejected(specs):
    for mk in ("I", "III", "IV", "V"):
        with pytest.raises(ValueError):
            solve(specs[mk])


# --- blimp alternate (specs/alternates/, not in the fleet) --------------------

@pytest.fixture(scope="module")
def mk5a():
    if not HAS_AEROSANDBOX:
        pytest.skip("aerosandbox not installed")
    from kytoon.spec import load_spec
    return solve(load_spec("specs/alternates/mk5a_blimp.yaml"))


@needs_asb
def test_blimp_alternate_still_solves(mk5a):
    assert mk5a.op_alpha is not None and mk5a.op_alpha < 12
    assert mk5a.cd_op_l1 < mk5a.spec.canopy.cd_op   # rigid = lower bound
    assert 0.80 < mk5a.cr_ratio < 1.10


@needs_asb
def test_blimp_hull_drag_in_plausible_band(mk5a):
    """CD at alpha 0 re-referenced to hull frontal: airship band 0.03–0.3."""
    import math
    cd0 = min(p.cd for p in mk5a.clean.pts)
    frontal = math.pi / 4 * mk5a.spec.hull.diameter ** 2
    assert 0.03 < cd0 * mk5a.spec.canopy.area / frontal < 0.30
