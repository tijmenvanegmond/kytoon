"""L1 3-line trim validation gates (Mk V fat wing).

Runs only when the l1 extra (aerosandbox + trimesh) is installed. The
contract encodes the 2026-07-24 findings: a single-confluence Mk V is
passively unstable; the 3-line taut-taut rig pins pitch; with the winchlet
pod riding the main tether (bridle.pod_standoff_m) the rig is passively
stable with locked winches and survives a +50 % gust with stall margin.
If a spec change breaks one of these gates, that is a design conversation,
not a reason to loosen it.
"""
import math

import numpy as np
import pytest

from kytoon.solvers.l1_trim import (
    G,
    HAS_L1,
    TENSION_FRAC,
    _derivs,
    aero_table,
    attach_point,
    ctl_line_length,
    eigenvalues,
    hang_moment,
    hang_trim,
    level_chord_fraction,
    line_stiffnesses,
    mass_props,
    pendant_for_level,
    reconstruct_rig,
    solve,
    trim_point,
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


# --- physics anchors ---------------------------------------------------------

@needs_l1
def test_closed_form_trim_is_a_dynamic_equilibrium(specs, rep):
    """The parallel-line closed form must agree with the full elastic
    dynamics: residual acceleration ~0 at the reconstructed pose."""
    s = specs["V"]
    tp = rep.op
    table = aero_table(s)
    props = mass_props(s)
    st, l0m, l0c, k_main, k_ctl = reconstruct_rig(s, tp)
    d = _derivs(s, props, table, st, l0m, l0c, tp.wind, k_main, k_ctl)
    assert np.linalg.norm(d[3:5]) < 0.5          # m/s² — small-angle approx
    assert abs(d[5]) < 0.02                      # rad/s²


@needs_l1
def test_pod_shortens_and_stiffens_the_control_path(specs):
    """k = EA/L: the 50 m pod must stiffen the pair ≈ L_tether/L_ctl over
    the ship-based rig."""
    s = specs["V"]
    assert s.bridle.pod_standoff_m == 50
    _, k_pod = line_stiffnesses(s)
    ship = s.model_copy(deep=True)
    ship.bridle.pod_standoff_m = None
    _, k_ship = line_stiffnesses(ship)
    ratio = k_pod / k_ship
    assert ratio == pytest.approx(s.tether.length / ctl_line_length(s),
                                  rel=1e-6)
    assert ratio > 7


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
    """The pod winchlet claim: a few kN and sub-metre travel."""
    assert max(tp.t_ctl_n for tp in rep.mission) < 8e3
    assert rep.winchlet_travel_m < 1.0


@needs_l1
def test_pod_rig_is_passively_stable(rep):
    """With the 50 m pod, every mode is damped (drift at most neutral) —
    the winchlet is a trim actuator, not a stabilizer."""
    assert rep.max_fast_re < -0.05
    assert rep.max_slow_re < 0.02


@needs_l1
def test_ship_rig_needs_active_control(specs):
    """Regression of the finding that motivated the pod: ship-based
    winches leave an undamped slow drift mode."""
    ship = specs["V"].model_copy(deep=True)
    ship.bridle.pod_standoff_m = None
    rep = solve(ship)
    assert rep.max_slow_re > 0.02 or rep.max_fast_re > -0.05


@needs_l1
def test_locked_winch_gust_keeps_stall_margin(specs, rep):
    """+50 % 1-cos gust at 12 m/s with the winches LOCKED: alpha must stay
    ≥5° below the ~22° stall region and return to trim."""
    s = specs["V"]
    table = aero_table(s)
    props = mass_props(s)
    tp = next(t for t in rep.mission if t.wind == 12)
    st, l0m, l0c, k_main, k_ctl = reconstruct_rig(s, tp)
    dt = 0.008
    a_peak, t_peak, alpha = -1e9, 0.0, tp.alpha_deg
    for i in range(int(30.0 / dt)):
        t = i * dt
        wind = 12.0 + (3 * (1 - math.cos(2 * math.pi * (t - 8) / 6))
                       if 8 <= t <= 14 else 0.0)
        k1 = _derivs(s, props, table, st, l0m, l0c, wind, k_main, k_ctl)
        k2 = _derivs(s, props, table, st + dt / 2 * k1, l0m, l0c, wind,
                     k_main, k_ctl)
        k3 = _derivs(s, props, table, st + dt / 2 * k2, l0m, l0c, wind,
                     k_main, k_ctl)
        k4 = _derivs(s, props, table, st + dt * k3, l0m, l0c, wind,
                     k_main, k_ctl)
        st = st + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        wa = np.array([wind - st[3], -st[4]])
        alpha = math.degrees(st[2] + math.atan2(wa[1], wa[0]))
        if t > 7:
            a_peak = max(a_peak, alpha)
    assert a_peak < 17.0, f"gust alpha peak {a_peak:.1f}° eats stall margin"
    assert abs(alpha - tp.alpha_deg) < 2.5, \
        f"did not return to trim: {alpha:.1f}° vs {tp.alpha_deg:.1f}°"


# --- capture-hover hang statics ------------------------------------------

@needs_l1
def test_zero_q_hang_anchor(specs, rep):
    """Physics anchor: at zero q the kite hangs where the gravity+buoyancy
    resultant passes through the main attach — hand formula
    tan(theta) = -sum(dx*Fz)/sum(dz*Fz) over the body-frame arms."""
    s = specs["V"]
    props = mass_props(s)
    p = attach_point(s, s.bridle.positions[1])
    num, den = 0.0, 0.0
    for pb, fz in ((props.r_skin, -props.m_skin * G),
                   (props.r_cb, props.buoyancy_n)):
        num += (pb[0] - p[0]) * fz
        den += (pb[1] - p[1]) * fz
    expect = math.degrees(math.atan2(-num, den))
    roots = hang_trim(s, props, aero_table(s), 0.0)
    stable = [t for t, ok in roots if ok]
    assert len(stable) == 1
    assert stable[0] == pytest.approx(expect, abs=0.5)
    assert rep.hang_theta_deg == pytest.approx(stable[0], abs=0.1)
    # Re-baselined 2026-07-25: adopting Γ = 10° for LATERAL stability
    # lifts the CB 0.96 m, which lengthens the buoyancy arm about the
    # attach and pulls the hang from −41° to −31°. A free improvement to
    # the capture-hover levelling problem, from a change made for an
    # unrelated reason — but still nose-down, so the pendant work stands.
    assert -35 < stable[0] < -26


@needs_l1
def test_hang_matches_sim_free_hang_before_the_pod_docks(specs):
    """Cross-model regression against the Godot recovery run.

    Re-pointed 2026-07-25. It used to compare against the run's final
    attitude (−57°), but at the adopted Γ = 10° that is no longer a free
    hang: the raised outboard attachments keep the control pair in
    reach, so once the pod docks its drum auto-tends at 3 kN and pulls
    the kite to θ ≈ +1.5° — level, which is what the capture pendant was
    for. The comparable moment is now PRE-dock, where the control lines
    are slack (T_ctl = 0) and the kite genuinely hangs: the sim reads
    −29.8° there against this static −30.6°.
    """
    s = specs["V"]
    roots = hang_trim(s, mass_props(s), aero_table(s), 0.0)
    stable = [t for t, ok in roots if ok]
    assert len(stable) == 1
    assert abs(stable[0] - (-29.8)) < 3.0


@needs_l1
def test_te_pendant_levels_the_hover(specs):
    """Option B: an aft capture pendant pinned by net buoyancy — both
    lines must stay loaded at a level hang."""
    s = specs["V"]
    props = mass_props(s)
    t_pend, t_main = pendant_for_level(s, props, aero_table(s), 0.95)
    assert 0.4e3 < t_pend < 1.2e3
    assert t_main > 1.5e3


@needs_l1
def test_ctl_drum_lock_can_pin_level_hover(specs):
    """Option C: the swept outboard stations sit ~2.6 m aft of the main
    attach, so a LOCKED (not tension-tended) ctl drum can hold a level
    hang — feasible but with thin main-line margin."""
    s = specs["V"]
    props = mass_props(s)
    table = aero_table(s)
    m0 = hang_moment(s, props, table, 0.0, wind=0.0)
    p_main = attach_point(s, s.bridle.positions[1])
    p_ctl = attach_point(s, s.bridle.positions[2])
    # ctl pulls straight down at the aft station: M = t * (x_ctl - x_main)
    t_ctl = -m0 / (p_ctl[0] - p_main[0])
    b_net = props.buoyancy_n - props.m_total * G
    t_main = b_net - t_ctl
    assert 1.5e3 < t_ctl < 3.5e3
    assert 0.0 < t_main < 1.0e3           # thin — why option B ranks first


@needs_l1
def test_level_attach_station_conflicts_with_flight(specs):
    """Option A falsified: the chord station that levels the hang cannot
    fly the mission (documents the flying-trim/hang-trim conflict)."""
    s = specs["V"]
    f_level = level_chord_fraction(s, mass_props(s))
    assert f_level == pytest.approx(0.47, abs=0.02)
    leveled = s.model_copy(deep=True)
    leveled.bridle.chord_fraction = f_level
    rep = solve(leveled)
    assert not rep.op.feasible or len(rep.mission) < 10


@needs_l1
def test_single_confluence_instability_stays_flagged(rep):
    assert any("passively UNSTABLE" in f for f in rep.flags)
    assert any("capture hover hangs" in f for f in rep.flags)


@needs_l1
def test_near_vertical_elevation_flagged(rep):
    """Tow-geometry caveat must surface until the §6 elevation conversation
    is resolved."""
    assert rep.op.elevation_deg > 75
    assert any("elevation" in f for f in rep.flags)
