"""L0 validation gates.

Anchors: hand-computable physics + published reference values.
"""
import math

import pytest

from kytoon.solvers.l0 import (
    NET_LIFT_PER_M3,
    solve,
    solve_buoyancy,
    solve_structure,
    solve_wind_envelope,
)
from kytoon.spec import InflatableTube, KytoonSpec, load_all, load_spec

SPECS = "specs"


@pytest.fixture(scope="module")
def specs():
    return {s.mk: s for s in load_all(SPECS)}


def test_helium_net_lift_constant():
    # standard figure: He lifts ≈ 1.05 kg per m³ at sea level
    assert NET_LIFT_PER_M3 == pytest.approx(1.046, abs=0.01)


def test_torus_volume_closed_form(specs):
    t = specs["IV"].torus
    R, r = t.ring_diameter / 2, t.tube_diameter / 2
    assert t.volume == pytest.approx(2 * math.pi**2 * R * r**2)
    # 13.5 m ring / 4.5 m tube → ≈ 674 m³
    assert t.volume == pytest.approx(674, rel=0.01)


def test_wrinkle_moment_reference():
    """Comer & Levy wrinkle onset: M_w = p·π·r³/2.
    Tube Ø0.6 m @ 0.4 bar → 40 kPa·π·0.027/2 = 1.696 kN·m."""
    tube = InflatableTube(diameter=0.6, length=10, pressure_bar=0.4)
    res = solve_structure(
        KytoonSpec(
            name="t", mk="t", archetype="spine", spar=tube,
            canopy={"area": 100, "span": 10},
        ),
        tow_force_n=0,
    )
    spar = next(s for s in res if s.label == "keel spar")
    assert spar.wrinkle_moment_nm == pytest.approx(1696, rel=0.01)


def test_hoop_running_load():
    """N = p·r: 25 kPa on r=0.45 m → 11.25 kN/m."""
    tube = InflatableTube(diameter=0.9, length=44, pressure_bar=0.25)
    assert tube.pressure_pa * tube.radius == pytest.approx(11_250)


def test_mk2_and_mk4_are_calm_air_capable(specs):
    for mk in ("II", "IV"):
        assert solve_buoyancy(specs[mk]).calm_air_capable, mk


def test_mk1_and_mk3_need_wind(specs):
    for mk in ("I", "III"):
        env = solve_wind_envelope(specs[mk])
        assert env.v_min_ms > 0, mk
        assert env.v_min_ms < 8, f"{mk} v_min implausibly high"


def test_envelopes_ordered(specs):
    for s in specs.values():
        env = solve_wind_envelope(s)
        assert env.v_max_ms > env.v_min_ms


def test_mk1_tow_force_magnitude(specs):
    """Sanity vs SkySails-class data: 400 m² @ 12 m/s should pull
    tens of kN (they quote ~ 2000 kW-equivalent tow on 400 m²)."""
    env = solve_wind_envelope(specs["I"])
    assert 20 < env.tow_force_at_12ms_kn < 60


def test_envelope_dent_limiter(specs):
    """Pressurized envelopes lose shape once q exceeds superpressure:
    v_dent = sqrt(2·500/ρ) ≈ 28.6 m/s at the assumed 500 Pa. Binds Mk II
    (side-wing v2); must NOT bind the 2000 Pa torus (57 m/s > its 44)."""
    env2 = solve_wind_envelope(specs["II"])
    assert env2.v_max_limiter == "envelope dent"
    assert env2.v_max_ms == pytest.approx(28.6, abs=0.1)
    assert solve_wind_envelope(specs["IV"]).v_max_limiter != "envelope dent"


def test_mk5_single_kytoon_coverage(specs):
    """Mk V's competing claim: one winged blimp covers the whole 0–20+ m/s
    requirement alone (challenges the two-kytoon carriage logic)."""
    env = solve_wind_envelope(specs["V"])
    assert env.v_min_ms == 0.0
    assert env.v_max_ms > 20.0
    assert solve_buoyancy(specs["V"]).calm_air_capable


def test_fleet_covers_zero_to_20ms(specs):
    envs = [solve_wind_envelope(s) for s in specs.values()]
    assert min(e.v_min_ms for e in envs) == 0.0
    assert max(e.v_max_ms for e in envs) > 20.0


def test_full_solve_runs_for_all(specs):
    for s in specs.values():
        rep = solve(s)
        assert rep.structure, s.mk


# --- benchmark calibration gates (TU Delft V3, awegroup open data) ----------
from kytoon import aero  # noqa: E402


def test_windtunnel_clmax_matches_publication():
    """Poland 2025/26 report CL_max ≈ 1.07 for the V3 rigid model."""
    assert aero.wind_tunnel().cl_max == pytest.approx(1.07, abs=0.03)


def test_windtunnel_ld_max_reasonable():
    # clean-wing (L/D)_max ≈ 8.7 at ~9° alpha
    assert aero.wind_tunnel().ld_max == pytest.approx(8.7, abs=0.5)
    assert 7 < aero.wind_tunnel().alpha_cl_max  # stall well past operating


def test_operating_cl_reached_pre_stall():
    # CL=0.8 traction point should sit at a modest, pre-stall alpha
    a, cl, cd = aero.calibrated_operating_point(400.0, 0.8)
    assert 5 < a < 12
    assert cl == pytest.approx(0.8, abs=0.02)


def test_mk1_handpicked_coeffs_are_benchmark_consistent(specs):
    """Our Mk I cl_op/cd_op should agree with the benchmark within tolerance,
    else the spec is making unsupported aero claims."""
    _, cl_bench, cd_bench = aero.calibrated_operating_point(
        specs["I"].wing_area, specs["I"].canopy.cl_op
    )
    # resultant-force coefficient is what drives tow load
    import math
    cr_spec = math.hypot(specs["I"].canopy.cl_op, specs["I"].canopy.cd_op)
    cr_bench = math.hypot(cl_bench, cd_bench)
    assert cr_spec == pytest.approx(cr_bench, rel=0.15)
