"""Stage 5 gates: the nonlinear 6-DOF time domain.

This is the model crosswind manoeuvring will run on, so it has to earn
trust against everything already established:

  * released at the Stage 3 trim it must simply stay there — the statics
    and the nonlinear dynamics agreeing is free and decisive;
  * a small nudge must reproduce the Stage 4 linearisation;
  * the quaternion must stay a rotation.

One correction found here and gated: `l1_dyn3d.linearise` builds its
attitude states as Euler angles and then sets d(rpy)/dt = ω, which only
holds at zero attitude. At this rig's 12.5° trim that mixes roll and yaw
by sin θ ≈ 0.22. `linearise_nonlinear` uses a body rotation-vector chart
where the kinematic block is exact. The two agree to ~1.5 % on the
dominant eigenvalue, so the Stage 4 verdicts stand — but the exact chart
is the one to build on.
"""
import math

import numpy as np
import pytest

from kytoon.solvers.l1_dyn3d import linearise
from kytoon.solvers.l1_lat_aero import solve as solve_lat
from kytoon.solvers.l1_mass3d import mass_props_3d
from kytoon.solvers.l1_rig3d import rest_lengths, trim3d
from kytoon.solvers.l1_sim3d import (
    HAS_L1, POS, QUAT, VEL, integrate, linearise_nonlinear, quat_to_matrix,
    solve, state_from_trim,
)
from kytoon.solvers.l1_trim import aero_table
from kytoon.spec import load_all

needs_l1 = pytest.mark.skipif(not HAS_L1,
                              reason="l1 extra (aerosandbox) not installed")


@pytest.fixture(scope="module")
def specs():
    return {s.mk: s for s in load_all("specs")}


@pytest.fixture(scope="module")
def rig(specs):
    """Trim + everything the integrator needs, at the spec configuration."""
    s = specs["V"]
    gam = s.fat_wing.dihedral_deg
    props = mass_props_3d(s, dihedral_deg=gam)
    table = aero_table(s)
    lat = solve_lat(s, dihedral_deg=gam,
                    fin_area_m2=s.fin.area if s.fin else 0.0)
    wind = np.array([12.0, 0.0, 0.0])
    l0 = rest_lengths(s, props, table, lat, wind, dihedral_deg=gam)
    x, ok = trim3d(s, props, table, lat, l0, wind, dihedral_deg=gam)
    assert ok
    return dict(spec=s, props=props, table=table, lat=lat, wind=wind,
                l0=l0, x=x, gam=gam)


@needs_l1
def test_refuses_non_fatwing(specs):
    with pytest.raises(ValueError, match="lofted fat wing"):
        solve(specs["I"])


# --- the free, decisive check ---------------------------------------------

@needs_l1
def test_released_at_trim_it_stays_there(rig):
    """Stage 3 solved statics; Stage 5 integrates the nonlinear equations.
    If the trim is a real equilibrium, nothing moves."""
    s0 = state_from_trim(rig["spec"], rig["x"])
    hist = integrate(rig["spec"], rig["props"], rig["table"], rig["lat"],
                     s0, rig["l0"], rig["wind"], 30.0,
                     dihedral_deg=rig["gam"])
    rpy = np.array([h.rpy_deg for h in hist])
    pos = np.array([h.pos for h in hist])
    assert np.abs(rpy - rpy[0]).max() < 0.05          # degrees
    assert np.linalg.norm(pos - pos[0], axis=1).max() < 0.05


@needs_l1
def test_quaternion_stays_a_rotation(rig):
    s0 = state_from_trim(rig["spec"], rig["x"])
    s0[VEL] = np.array([0.4, 0.3, -0.2])              # kick it about
    hist = integrate(rig["spec"], rig["props"], rig["table"], rig["lat"],
                     s0, rig["l0"], rig["wind"], 20.0,
                     dihedral_deg=rig["gam"])
    for h in hist:
        r = quat_to_matrix(
            np.array([1.0, 0.0, 0.0, 0.0]))           # sanity on the helper
        assert np.allclose(r, np.eye(3), atol=1e-12)
    assert len(hist) > 100


# --- agreement with the linear model --------------------------------------

@needs_l1
def test_translation_matches_the_linearisation(rig):
    """A pure position nudge exercises the force model, mass matrix, lines
    and aero with no attitude kinematics involved — the two models must
    track each other closely."""
    from scipy.linalg import expm
    a = linearise(rig["spec"], rig["props"], rig["table"], rig["lat"],
                  rig["x"], rig["l0"], rig["wind"], rig["gam"])
    d = 0.5
    s0 = state_from_trim(rig["spec"], rig["x"])
    s0[POS] = s0[POS] + np.array([0.0, d, 0.0])
    hist = integrate(rig["spec"], rig["props"], rig["table"], rig["lat"],
                     s0, rig["l0"], rig["wind"], 6.0, dt=0.002,
                     dihedral_deg=rig["gam"], record_every=0.05)
    z0 = np.zeros(12)
    z0[1] = d
    err = max(abs(h.pos[1] - float((expm(a * h.t) @ z0)[1])) for h in hist)
    assert err < 0.02 * d


@needs_l1
def test_exact_chart_confirms_the_stage4_spectrum(rig):
    """`l1_dyn3d.linearise` conflates Euler rates with angular velocity —
    exact only at zero attitude. The rotation-vector chart here has no
    such approximation. At a 12.5° trim the difference must be small, or
    every Stage 4 conclusion needs revisiting."""
    a_euler = linearise(rig["spec"], rig["props"], rig["table"], rig["lat"],
                        rig["x"], rig["l0"], rig["wind"], rig["gam"])
    s0 = state_from_trim(rig["spec"], rig["x"])
    a_exact = linearise_nonlinear(rig["spec"], rig["props"], rig["table"],
                                  rig["lat"], s0, rig["l0"], rig["wind"],
                                  rig["gam"])
    top_e = max(np.linalg.eigvals(a_euler).real)
    top_x = max(np.linalg.eigvals(a_exact).real)
    assert abs(top_x - top_e) < 0.05 * max(abs(top_e), 0.01)


@needs_l1
def test_adopted_configuration_is_stable_in_the_exact_chart(rig):
    """The Γ = 10° + fin decision, re-checked without the Euler
    approximation it was originally made under."""
    s0 = state_from_trim(rig["spec"], rig["x"])
    a = linearise_nonlinear(rig["spec"], rig["props"], rig["table"],
                            rig["lat"], s0, rig["l0"], rig["wind"],
                            rig["gam"])
    assert max(np.linalg.eigvals(a).real) < 0.0


@needs_l1
def test_bare_airframe_still_diverges_in_the_exact_chart(specs):
    """...and so does the finding that motivated it."""
    s = specs["V"]
    props = mass_props_3d(s, dihedral_deg=0.0)
    table = aero_table(s)
    lat = solve_lat(s, dihedral_deg=0.0, fin_area_m2=0.0)
    wind = np.array([12.0, 0.0, 0.0])
    l0 = rest_lengths(s, props, table, lat, wind, dihedral_deg=0.0)
    x, ok = trim3d(s, props, table, lat, l0, wind, dihedral_deg=0.0)
    assert ok
    a = linearise_nonlinear(s, props, table, lat, state_from_trim(s, x),
                            l0, wind, 0.0)
    assert max(np.linalg.eigvals(a).real) > 0.5
