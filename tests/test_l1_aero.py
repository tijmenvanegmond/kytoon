"""L1 aero validation gates.

Runs only when the l1 extra (VSM) is installed — the default install stays
L0-only. Slowest suite in the repo (~1–2 min): two full VSM alpha sweeps.

Gate philosophy (same contract as test_l0.py): the bands below encode the
measured deviation of the parametric-arc + Breukels + VSM pipeline from the
vendored TU Delft V3 wind-tunnel benchmark. Numbers drifting outside the
bands mean the pipeline or a spec broke — do not widen a band without a
source or a documented model change.
"""
import math

import numpy as np
import pytest

from kytoon import aero
from kytoon.solvers.l1_aero import HAS_VSM, ArcWing, solve, sweep
from kytoon.spec import load_all

SPECS = "specs"
needs_vsm = pytest.mark.skipif(not HAS_VSM, reason="l1 extra (VSM) not installed")


@pytest.fixture(scope="module")
def specs():
    return {s.mk: s for s in load_all(SPECS)}


# --- geometry closed forms (no VSM needed) ----------------------------------

def test_arc_geometry_closed_form_v3():
    """Arc from V3 bulk numbers reproduces its published shape."""
    g = ArcWing(span=8.32, area=19.753)
    # h/b = tan(θ0/2)/2 → θ0 for the V3's 3.13/8.32
    assert math.degrees(g.arc_half_angle) == pytest.approx(73.9, abs=0.2)
    assert g.arc_radius == pytest.approx(4.33, abs=0.02)
    # arc height back out of the geometry
    h = g.arc_radius * (1 - math.cos(g.arc_half_angle))
    assert h / g.span == pytest.approx(ArcWing.height_ratio, rel=0.01)


def test_arc_sections_integrate_to_projected_area():
    g = ArcWing(span=8.32, area=19.753, n_sections=40)
    pts = g.section_points()
    area = sum(
        0.5 * ((te1 - le1)[0] + (te2 - le2)[0]) * abs(le2[1] - le1[1])
        for (le1, te1), (le2, te2) in zip(pts, pts[1:])
    )
    assert area == pytest.approx(19.753, rel=0.01)


def test_mk1_arc_tube_ratio_from_spec(specs):
    g = ArcWing.from_spec(specs["I"])
    # Ø0.9 m LE tube on a 400/38 m mean chord
    assert g.tube_t == pytest.approx(0.9 / (400 / 38), rel=1e-6)


# --- VSM pipeline vs vendored benchmark --------------------------------------

@pytest.fixture(scope="module")
def v3_polar():
    return sweep(ArcWing(span=8.32, area=19.753), n_panels=30)


@needs_vsm
def test_v3_clmax_within_benchmark_band(v3_polar):
    """CL_max within 15% of the wind tunnel (1.07); CFD w/ struts gives 1.35,
    so the pipeline sitting a little high is expected, not alarming."""
    wt = aero.wind_tunnel().cl_max
    assert v3_polar.cl_max == pytest.approx(wt, rel=0.15)


@needs_vsm
def test_v3_ldmax_conservative_band(v3_polar):
    """Breukels sections are draggier than the rigid tunnel model: accept
    (L/D)max up to 25% below the tunnel's 8.7, never above +10%."""
    wt = aero.wind_tunnel().ld_max
    assert 0.75 * wt < v3_polar.ld_max < 1.10 * wt


@needs_vsm
def test_v3_operating_cl_pre_stall(v3_polar):
    a = v3_polar.alpha_for_cl(0.8)
    assert a is not None and 5 < a < 12


# --- Mk-specific solves -------------------------------------------------------

@pytest.fixture(scope="module")
def mk1_report(specs):
    return solve(specs["I"], n_panels=30)


@needs_vsm
def test_mk1_reaches_operating_cl(mk1_report):
    assert mk1_report.op_alpha is not None
    assert mk1_report.op_alpha < 15


@needs_vsm
def test_mk1_resultant_consistent_with_spec(mk1_report):
    """Mk I's own L1 polar must support the spec's cl_op/cd_op within 20%
    on resultant force — the L0 tow numbers stand on this."""
    assert 0.8 < mk1_report.cr_ratio < 1.2


@needs_vsm
def test_mk2_wing_only_is_flagged(specs):
    rep = solve(specs["II"], n_panels=20, alphas=np.array([5.0]))
    assert any("lobe" in f for f in rep.flags)


# --- Mk V fat wing (2026-07-11 pivot: solves here via VSM, not body-aero) -----

@pytest.fixture(scope="module")
def mk5_report(specs):
    return solve(specs["V"], n_panels=30)


@needs_vsm
def test_mk5_fatwing_reaches_operating_cl(mk5_report):
    assert mk5_report.op_alpha is not None
    assert mk5_report.op_alpha < 15


@needs_vsm
def test_mk5_fatwing_resultant_band(mk5_report):
    assert 0.8 < mk5_report.cr_ratio < 1.2


@needs_vsm
def test_mk5_breukels_extrapolation_flagged(mk5_report):
    """t/c 0.28 is beyond the LEI regression fit range — must be flagged."""
    assert any("extrapolated" in f for f in mk5_report.flags)


@needs_vsm
def test_mk5_coverage_claim_holds_at_both_bounds(mk5_report, specs):
    """Single-kytoon claim (v_max > 20) at both the spec cr and the VSM cr."""
    from kytoon.solvers.l1_body_aero import v_max_tether_with
    assert v_max_tether_with(specs["V"], mk5_report.cr_op_spec) > 20
    assert v_max_tether_with(specs["V"], mk5_report.cr_op_l1) > 20


@needs_vsm
def test_mk3_twin_skin_is_flagged(specs):
    rep = solve(specs["III"], n_panels=20, alphas=np.array([5.0]))
    assert any("twin-skin" in f for f in rep.flags)


@needs_vsm
def test_mk4_has_no_wing(specs):
    with pytest.raises(ValueError):
        solve(specs["IV"])
