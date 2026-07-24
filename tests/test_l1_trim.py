"""L1 3-line trim validation gates (Mk V fat wing).

Runs only when the l1 extra (aerosandbox + trimesh) is installed. The
contract encodes the 2026-07-24 findings: a single-confluence Mk V is
passively unstable, the 3-line taut-taut rig pins pitch and is steerable
across the envelope with a small winchlet. If a spec change breaks one of
these gates, that is a design conversation, not a reason to loosen it.
"""
import math

import numpy as np
import pytest

from kytoon.solvers.l1_trim import (
    HAS_L1,
    _derivs,
    aero_table,
    attach_point,
    eigenvalues,
    mass_props,
    solve,
    trim_point,
    DYNEEMA_STRAIN_MBL,
    FAIRLEAD_HEIGHT,
    TENSION_FRAC,
    _rotm,
)
from kytoon.spec import load_all

needs_l1 = pytest.mark.skipif(not HAS_L1,
                              reason="l1 extra (aerosandbox) not installed")


@pytest.fixture(scope="module")
def specs():
    return {s.mk: s for s in load_all("specs")}


@pytest.fixture(scope="module")
def rep(specs):
    return solve(specs["V"])


@needs_l1
def test_refuses_non_fatwing(specs):
    with pytest.raises(ValueError, match="3-line fat-wing"):
        solve(specs["I"])


# --- physics anchor ----------------------------------------------------------

@needs_l1
def test_closed_form_trim_is_a_dynamic_equilibrium(specs, rep):
    """The parallel-line closed form must agree with the full elastic
    two-line dynamics: residual acceleration ~0 at the reconstructed pose."""
    s = specs["V"]
    tp = rep.op
    table = aero_table(s)
    props = mass_props(s)
    anchor = np.array([0.0, FAIRLEAD_HEIGHT])
    a = math.radians(tp.alpha_deg)
    u_hat = np.array([math.cos(math.radians(tp.elevation_deg)),
                      math.sin(math.radians(tp.elevation_deg))])
    R = _rotm(a)
    x0 = anchor + u_hat * s.tether.length \
        - R @ attach_point(s, s.bridle.positions[1])
    k_main = (s.tether.mbl_kn * 1e3 / DYNEEMA_STRAIN_MBL) / s.tether.length
    k_ctl = (2 * s.bridle.control_mbl_kn * 1e3 / DYNEEMA_STRAIN_MBL) \
        / s.tether.length
    dm = float(np.linalg.norm(
        anchor - (x0 + R @ attach_point(s, s.bridle.positions[1]))))
    dc = float(np.linalg.norm(
        anchor - (x0 + R @ attach_point(s, s.bridle.positions[2]))))
    st = np.array([x0[0], x0[1], a, 0.0, 0.0, 0.0])
    d = _derivs(s, props, table, st,
                dm - tp.t_main_n / k_main, dc - tp.t_ctl_n / k_ctl,
                tp.wind, k_main, k_ctl)
    assert np.linalg.norm(d[3:5]) < 0.2          # m/s² — parallel-line error
    assert abs(d[5]) < 0.01                      # rad/s²


# --- design gates -------------------------------------------------------------

@needs_l1
def test_trim_feasible_at_cl_op(rep):
    assert rep.op.feasible
    assert 9.0 < rep.op.alpha_deg < 15.0         # cl_op within the linear range


@needs_l1
def test_steerable_band_covers_depower_to_power(rep):
    """Winchlet must command everything from near-zero lift to cl_op."""
    for u, band in rep.bands.items():
        assert band is not None, f"no feasible alpha at {u} m/s"
        lo, hi = band
        assert lo <= 1.0 and hi >= 12.0, f"band {band} too narrow at {u} m/s"


@needs_l1
def test_mission_schedule_covers_envelope(specs, rep):
    """A feasible depower schedule must exist from near-calm to L0 v_max."""
    s = specs["V"]
    winds = [tp.wind for tp in rep.mission]
    assert min(winds) <= 4 and max(winds) >= 23
    assert len(winds) == 10                       # no gaps in the sweep
    for tp in rep.mission:
        assert tp.feasible
        assert tp.t_total_n <= TENSION_FRAC * s.tether.wll_n * 1.001


@needs_l1
def test_winchlet_budget_is_small(rep):
    """The steering hardware claim: a few kN, a few metres of travel."""
    assert max(tp.t_ctl_n for tp in rep.mission) < 8e3
    assert rep.winchlet_travel_m < 6.0


@needs_l1
def test_taut_taut_rig_is_stable(rep):
    """Fast modes damped; at most a slow drift left to the winch loop."""
    assert rep.max_fast_re < 0.0
    assert abs(rep.max_slow_re) < 0.1


@needs_l1
def test_single_confluence_instability_stays_flagged(rep):
    assert any("passively UNSTABLE" in f for f in rep.flags)


@needs_l1
def test_near_vertical_elevation_flagged(rep):
    """Tow-geometry caveat must surface until the §6 elevation conversation
    is resolved."""
    assert rep.op.elevation_deg > 75
    assert any("elevation" in f for f in rep.flags)
