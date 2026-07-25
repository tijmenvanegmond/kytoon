"""Stage 3 gates: 3D force closure on the three-line rig.

The headline: the bridle supplies the yaw stiffness the wing does not
have. Stage 2 measured Cn_β < 0 — a tailless swept planform diverges in
yaw on its own — so heading has to be held by the geometry of two
control lines running from a common pod to attachments at ±0.38 span.
These gates say it is, and by how much.

Also gated here is the axis bookkeeping, because it is where this model
is easiest to get silently wrong: AeroBuildup reports moments in
flight-dynamics body axes (x forward, z down) while the loft is in
geometry axes (x aft, z up), and its β is the negative of the one used
here.
"""
import math

import numpy as np
import pytest

from kytoon.solvers.l1_rig3d import (
    HAS_L1, MAIN, PORT, STBD, net_wrench, rest_lengths, rotation, solve,
    trim3d, wind_angles,
)
from kytoon.solvers.l1_lat_aero import solve as solve_lat
from kytoon.solvers.l1_mass3d import mass_props_3d
from kytoon.solvers.l1_trim import aero_table, mass_props, trim_point
from kytoon.spec import load_all

needs_l1 = pytest.mark.skipif(not HAS_L1,
                              reason="l1 extra (aerosandbox) not installed")


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


# --- axis bookkeeping -----------------------------------------------------

@needs_l1
def test_wind_angles_match_aerosandbox():
    """Decisive convention check against AeroSandbox's own freestream
    direction: α agrees, β is negated. If this drifts, every lateral
    coefficient is being applied with the wrong sign."""
    import aerosandbox as asb
    for a_deg, b_deg in [(0, 0), (10, 0), (0, 5), (0, -5), (10, 5),
                         (-4, -3)]:
        op = asb.OperatingPoint(velocity=12.0, alpha=a_deg, beta=b_deg)
        v = np.ravel(op.compute_freestream_direction_geometry_axes()) * 12.0
        alpha, beta = wind_angles(v)
        assert math.degrees(alpha) == pytest.approx(a_deg, abs=1e-6)
        assert math.degrees(beta) == pytest.approx(-b_deg, abs=1e-6)


@needs_l1
def test_rotations_are_right_handed():
    """All three axes right-handed, so restoring stiffness is uniformly
    −∂M/∂angle. In this x-aft/z-up frame that means +φ raises starboard
    and +θ lifts the leading edge."""
    for k, axis in enumerate((np.array([1.0, 0, 0]), np.array([0, 1.0, 0]),
                              np.array([0, 0, 1.0]))):
        rpy = np.zeros(3)
        rpy[k] = 1e-4
        # right-handed: R v ≈ v + (axis × v) θ
        v = np.array([0.3, 0.5, 0.7])
        got = rotation(rpy) @ v
        want = v + np.cross(axis, v) * 1e-4
        assert np.allclose(got, want, atol=1e-9)
    # +θ must drop the trailing edge (+x), matching the planar model
    assert (rotation(np.array([0.0, 0.2, 0.0])) @ np.array([1.0, 0, 0]))[2] < 0


# --- planar reduction -----------------------------------------------------

@needs_l1
def test_symmetric_rig_stays_in_the_plane(rep):
    """Symmetric lines, no crosswind: the 3D solve must find a planar
    equilibrium — no roll, no yaw, no sideslip, no lateral offset."""
    assert abs(rep.beta_deg) < 1e-3
    assert abs(math.degrees(rep.pose[3])) < 1e-3      # roll
    assert abs(math.degrees(rep.pose[5])) < 1e-3      # yaw
    assert abs(rep.pose[1]) < 1e-3                    # y position
    assert rep.tensions[PORT] == pytest.approx(rep.tensions[STBD], rel=1e-6)


@needs_l1
def test_tensions_agree_with_the_planar_solver(specs, rep):
    """Cross-model check: at the same α, the 3D closure's main and
    summed-control tensions must land near `l1_trim`'s closed form."""
    s = specs["V"]
    tp = trim_point(s, mass_props(s), aero_table(s), 12.0, rep.alpha_deg)
    assert rep.tensions[MAIN] == pytest.approx(tp.t_main_n, rel=0.15)
    ctl_total = rep.tensions[PORT] + rep.tensions[STBD]
    assert ctl_total == pytest.approx(tp.t_ctl_n, rel=0.25)


# --- the Stage 2 question, answered ---------------------------------------

@needs_l1
def test_wing_alone_diverges_in_yaw(rep):
    """Regression of the Stage 2 finding, in force terms."""
    assert rep.k_yaw_aero < 0.0


@needs_l1
def test_bridle_supplies_the_missing_yaw_stiffness(rep):
    """The headline. Two lines from a common pod to attachments at
    ±0.38 span form a V that resists yaw, and it must beat the wing's
    own divergence by a wide margin."""
    assert rep.yaw_stable
    assert rep.k_yaw > 0.0
    assert rep.bridle_yaw_margin > 5.0


@needs_l1
def test_rig_is_roll_stiff(rep):
    """Short, stiff control lines at a 12 m lever dominate roll."""
    assert rep.k_roll > 0.0
    assert rep.k_roll > 100 * abs(rep.k_yaw_aero)


# --- steering -------------------------------------------------------------

@needs_l1
def test_differential_trim_steers(rep):
    """All three differential cases must converge and produce a real
    lateral excursion — this is the winchlet answer in static form."""
    for s in rep.steer:
        assert s["converged"]
    biggest = rep.steer[-1]
    assert abs(biggest["roll_deg"]) > 1.0
    assert abs(biggest["y_offset_m"]) > 2.0


@needs_l1
def test_steering_is_monotonic_and_roughly_linear(rep):
    rolls = [abs(s["roll_deg"]) for s in rep.steer]
    offs = [abs(s["y_offset_m"]) for s in rep.steer]
    assert rolls == sorted(rolls)
    assert offs == sorted(offs)
    # 6× the input should give roughly 6× the response at these angles
    ratio = rolls[-1] / rolls[0]
    assert 4.0 < ratio < 8.0


@needs_l1
def test_shortening_port_banks_and_flies_to_port(rep):
    """Sign coherence: pulling the port line down drops the port wing
    (+φ raises starboard in this frame) and the kite tracks to −y."""
    for s in rep.steer:
        assert s["roll_deg"] > 0.0
        assert s["y_offset_m"] < 0.0
        assert s["beta_deg"] < 0.0
