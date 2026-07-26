"""Stage 2 gates: lateral aero and the winchlet steering answer.

The headline these encode: differential winchlet trim CAN roll the
Manta, and the response is damping-limited — the air sets the rate, not
the 140 t·m² of roll inertia. Also gated: the tailless swept planform is
weathercock-UNSTABLE on its own, which is a real finding about what the
3-line rig has to do, not a bug.
"""
import math

import numpy as np
import pytest

from kytoon.solvers.l1_lat_aero import (
    HAS_L1, beta_sweep, lift_curve_slope, roll_damping, solve,
    steer_response, yaw_damping,
)
from kytoon.solvers.l1_mass3d import mass_props_3d
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
    with pytest.raises(ValueError, match="lofted fat wing"):
        solve(specs["I"])


# --- physics anchor: roll damping by hand --------------------------------

@needs_l1
def test_roll_damping_matches_closed_form(specs):
    """Strip integral vs the closed form for a linearly tapered wing:

        Cl_p = −2·a0·∫c y² dy / (S b²),
        ∫c y² dy = c₀b³/4 · [1/3 − (1−λ)/4]

    Hand-computable, so a silent change in the strip loop shows up here.
    """
    s = specs["V"]
    fw = s.fat_wing
    a0 = lift_curve_slope(s)
    integral = fw.chord * fw.span ** 3 / 4 * (1 / 3 - (1 - fw.taper) / 4)
    expect = -2.0 * a0 * integral / (s.canopy.area * fw.span ** 2)
    assert roll_damping(s) == pytest.approx(expect, rel=1e-3)


@needs_l1
def test_roll_damping_is_physical_and_bounded(rep):
    """Negative (it opposes roll) and between the 3D-slope and 2π
    estimates, which bracket the real value."""
    assert -1.5 < rep.cl_p < -0.1
    assert rep.cl_p_thin < rep.cl_p                # 2π gives more damping
    assert rep.cl_p_thin / rep.cl_p == pytest.approx(
        2 * math.pi / lift_curve_slope(rep.spec), rel=1e-6)


@needs_l1
def test_yaw_damping_is_negative(specs):
    assert yaw_damping(specs["V"]) < 0.0


# --- static derivatives ---------------------------------------------------

@needs_l1
def test_tailless_wing_is_weathercock_unstable(specs):
    """FINDING, not a bug: the BARE swept planform — flat, no fin — has
    Cn_β < 0. It is weak (|Cn_β| ~ 0.005 vs ~0.1 for a finned aircraft),
    but negative, and it is the reason the spec now carries a fin."""
    bare = solve(specs["V"], dihedral_deg=0.0, fin_area_m2=0.0)
    assert bare.cn_beta < 0.0
    assert abs(bare.cn_beta) < 0.05


@needs_l1
def test_spec_configuration_is_weathercock_stable(rep):
    """...and with the spec's Γ = 10° + 12 m² fin, it is cured: the fin
    has to beat both the bare planform AND the extra yaw divergence the
    dihedral brings."""
    assert rep.cn_beta > 0.0


@needs_l1
def test_sweep_alone_gives_stable_roll_in_sideslip(rep):
    assert rep.cl_beta < 0.0


@needs_l1
def test_dihedral_strengthens_the_dihedral_effect(specs):
    """Classic check on the folded planform: Γ must make Cl_β more
    negative. If it doesn't, the panels are folded the wrong way."""
    flat = solve(specs["V"], dihedral_deg=0.0)
    fold = solve(specs["V"], dihedral_deg=20.0)
    assert fold.cl_beta < flat.cl_beta


# --- the winchlet answer --------------------------------------------------

@needs_l1
def test_steering_is_damping_limited(rep):
    """The roll rate is set by the air, not the mass: τ well under a
    second against 140 t·m² of roll + added inertia. Steering is a
    sustained-pull problem, not an impulse one."""
    for r in rep.steer:
        assert r.damping_limited
        assert r.tau_s < 1.0
        assert r.inertia_kgm2 > 1e5


@needs_l1
def test_winchlet_has_useful_roll_authority(specs, rep):
    """The question that started all of this. At 12 m/s, 4 kN of
    differential — an eighth of the control pair's 33 kN WLL — must bank
    the wing 30° in under 10 s."""
    r = next(x for x in rep.steer if x.wind == 12.0 and x.delta_t_n == 4e3)
    assert r.time_to_30deg_s < 10.0
    assert r.p_steady_deg_s > 3.0
    ctl_wll = 2 * specs["V"].bridle.control_mbl_kn * 1e3 \
        / specs["V"].tether.safety_factor
    assert r.delta_t_n < 0.2 * ctl_wll


@needs_l1
def test_roll_rate_scales_linearly_with_differential(rep):
    a = next(x for x in rep.steer if x.wind == 12.0 and x.delta_t_n == 2e3)
    b = next(x for x in rep.steer if x.wind == 12.0 and x.delta_t_n == 8e3)
    assert b.p_steady_deg_s == pytest.approx(4 * a.p_steady_deg_s, rel=1e-6)


@needs_l1
def test_roll_rate_falls_with_wind(rep):
    """Damping ∝ V while the couple is fixed, so the same pull rolls it
    more slowly as the wind builds — 8 vs 12 m/s must be exactly 3:2."""
    slow = next(x for x in rep.steer if x.wind == 8.0 and x.delta_t_n == 2e3)
    fast = next(x for x in rep.steer if x.wind == 12.0 and x.delta_t_n == 2e3)
    assert slow.p_steady_deg_s / fast.p_steady_deg_s == pytest.approx(
        12.0 / 8.0, rel=1e-6)


@needs_l1
def test_flags_disclose_the_model_limits(rep):
    joined = " ".join(rep.flags)
    assert "not benchmark-anchored" in joined.lower() \
        or "NOT benchmark-anchored" in joined
    assert "RIGID" in joined                   # no fabric warp modelled
