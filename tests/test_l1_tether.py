"""L1 tether validation gates.

Runs only when the l1 extra (moorpy) is installed. Same contract philosophy
as the other suites: anchors are hand-computable force balances; gates
compare the drag/sag-corrected line against the L0 straight-line picture
and must only move with a documented model change.
"""
import math

import pytest

from kytoon.solvers.l0 import G, RHO_AIR, solve_wind_envelope
from kytoon.solvers.l1_tether import (
    HAS_MOORPY,
    kite_force,
    solve,
    v_max_tether,
)
from kytoon.spec import load_all

SPECS = "specs"
needs_moorpy = pytest.mark.skipif(not HAS_MOORPY,
                                  reason="l1 extra (moorpy) not installed")


@pytest.fixture(scope="module")
def specs():
    return {s.mk: s for s in load_all(SPECS)}


# --- physics anchors ---------------------------------------------------------

@needs_moorpy
def test_zero_wind_buoyant_vertical_balance(specs):
    """Mk II in calm air: ship-end tension = net buoyant pull − line weight."""
    s = specs["II"]
    rep = solve(s, 0.0)
    f_up = kite_force(s, 0.0)[2]
    w_line = s.tether.linear_density * G * s.tether.length
    assert rep.tension_ship_n == pytest.approx(f_up - w_line, rel=0.02)
    assert rep.elevation_chord_deg > 85          # hangs vertical
    assert rep.altitude == pytest.approx(s.tether.length, rel=0.02)


@needs_moorpy
def test_horizontal_momentum_balance_mk1(specs):
    """Ship-end horizontal force = kite drag + integrated line drag."""
    s = specs["I"]
    rep = solve(s, 12.0)
    f_ship_x = rep.tension_ship_n * math.cos(math.radians(rep.elevation_ship_deg))
    assert f_ship_x == pytest.approx(rep.kite_force_n[0] + rep.line_drag_n,
                                     rel=0.03)


@needs_moorpy
def test_line_drag_magnitude_hand_check(specs):
    """400 m of 14 mm line broadside-ish in 12 m/s: O(0.5 kN), not 5, not 0.05."""
    rep = solve(specs["I"], 12.0)
    assert 200 < rep.line_drag_n < 800


# --- gates vs the L0 straight-line picture ------------------------------------

@needs_moorpy
def test_mk1_tether_vmax_near_l0_limit(specs):
    """Drag adds tension, buoyancy deficit sheds a little: the corrected
    tether ceiling stays within 10% of L0's straight-line WLL figure."""
    s = specs["I"]
    cr = math.hypot(s.canopy.cl_op, s.canopy.cd_op)
    v_l0 = math.sqrt(2 * s.tether.wll_n / (RHO_AIR * s.canopy.area * cr))
    assert v_max_tether(s) == pytest.approx(v_l0, rel=0.10)


@needs_moorpy
def test_mk4_tether_vmax_matches_l0_bluff_model(specs):
    """Torus has no lift to shed — line drag can only lower the ceiling."""
    s = specs["IV"]
    v_l1 = v_max_tether(s)
    v_l0 = solve_wind_envelope(s).v_max_ms
    assert v_l1 <= v_l0
    assert v_l1 == pytest.approx(v_l0, rel=0.05)


@needs_moorpy
def test_elevation_is_an_output_and_disagrees_with_spec(specs):
    """Force balance puts Mk I's tether near atan(L/D) ≈ 78°, not the spec's
    40°. The flag documents the finding; L0's eta_v is the design question."""
    rep = solve(specs["I"], 12.0)
    assert rep.elevation_chord_deg > 60
    assert any("elevation" in f for f in rep.flags)


@needs_moorpy
def test_wll_exceedance_is_flagged(specs):
    rep = solve(specs["I"], 25.0)
    assert rep.wll_fraction > 1.0
    assert any("WLL" in f for f in rep.flags)


@needs_moorpy
def test_sag_small_under_tension_mk1(specs):
    """At 27 kN on a 0.43 kN line, sag must be metres, not tens of metres."""
    rep = solve(specs["I"], 12.0)
    assert rep.sag_max_m < 5.0
