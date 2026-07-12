"""L0 analytic solver layer.

Closed-form physics only — the cheap 80%. Every result carries its assumptions.

Buoyancy   : Archimedes on He volumes (ISA sea level).
Spar/tubes : pressurized-beam theory — hoop stress, wrinkle-onset moment
             M_w = p·π·r³/2, collapse ≈ 2·M_w  (Comer & Levy / Veldman).
Wind env   : v_min from static-lift deficit; v_max from min(tether WLL,
             spar wrinkle margin, canopy fabric limit).
Tether     : straight-line approximation with weight-induced sag angle,
             drag loading noted but not integrated (that's L1/MoorPy).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from kytoon.spec import Archetype, InflatableTube, KytoonSpec

RHO_AIR = 1.225        # kg/m³, ISA sea level
RHO_HE = 0.1786        # kg/m³ (incl. typical purity penalty ~ +2%, ignored at L0)
G = 9.81
NET_LIFT_PER_M3 = RHO_AIR - RHO_HE   # ≈ 1.046 kg/m³


# ---------------------------------------------------------------------------
@dataclass
class BuoyancyResult:
    he_volume: float            # m³
    gross_static_lift_kg: float # He displacement lift
    structure_mass_kg: float
    payload_mass_kg: float
    net_static_lift_kg: float   # gross - all flying mass (excl. tether)
    tether_mass_kg: float
    net_incl_tether_kg: float   # can it hold its own tether up in calm air?

    @property
    def calm_air_capable(self) -> bool:
        return self.net_incl_tether_kg > 0


def solve_buoyancy(spec: KytoonSpec) -> BuoyancyResult:
    gross = spec.helium_volume * NET_LIFT_PER_M3
    net = gross - spec.total_mass
    return BuoyancyResult(
        he_volume=spec.helium_volume,
        gross_static_lift_kg=gross,
        structure_mass_kg=spec.structure_mass,
        payload_mass_kg=spec.payload_mass,
        net_static_lift_kg=net,
        tether_mass_kg=spec.tether.mass,
        net_incl_tether_kg=net - spec.tether.mass,
    )


# ---------------------------------------------------------------------------
@dataclass
class TubeStressResult:
    label: str
    hoop_stress_n_per_m: float      # fabric running load [N/m]
    hoop_utilization: float          # vs fabric strength (want < ~0.2 with SF 5)
    wrinkle_moment_nm: float         # bending moment at wrinkle onset
    collapse_moment_nm: float
    applied_moment_nm: float
    bending_utilization: float       # applied / wrinkle (want < 1.0)

    @property
    def ok(self) -> bool:
        # SF 4 on fabric (inflatable-structure convention), wrinkle onset as bending limit
        return self.hoop_utilization <= 0.25 and self.bending_utilization < 1.0


def _tube_stress(
    tube: InflatableTube, label: str, applied_moment: float
) -> TubeStressResult:
    # hoop running load in fabric: N = p * r  [N/m]
    hoop = tube.pressure_pa * tube.radius
    m_wrinkle = tube.pressure_pa * math.pi * tube.radius**3 / 2
    return TubeStressResult(
        label=label,
        hoop_stress_n_per_m=hoop,
        hoop_utilization=hoop / tube.fabric_strength_n_per_m,
        wrinkle_moment_nm=m_wrinkle,
        collapse_moment_nm=2 * m_wrinkle,
        applied_moment_nm=applied_moment,
        bending_utilization=applied_moment / m_wrinkle if m_wrinkle else math.inf,
    )


def solve_structure(spec: KytoonSpec, tow_force_n: float) -> list[TubeStressResult]:
    """Estimate governing bending moments from the tow force.

    Model: aerodynamic load enters distributed along the span; the bridle
    reacts it at `spec.bridle.positions`. The worst unsupported segment acts
    as a beam with uniformly distributed load w = F_tow / span carrying
    M ≈ w·L_seg²/8 (simply-supported approximation).
    """
    results: list[TubeStressResult] = []

    def worst_moment(span: float) -> float:
        pts = sorted({0.0, *spec.bridle.positions, 1.0})
        seg = max(b - a for a, b in zip(pts, pts[1:])) * span
        w = tow_force_n / span if span else 0.0
        return w * seg**2 / 8

    if spec.le_tube is not None and spec.canopy is not None:
        # The tensioned canopy membrane carries most aero load straight to
        # the bridles (tensairity effect); the LE tube sees ~35% as bending.
        TUBE_LOAD_SHARE = 0.35
        m = TUBE_LOAD_SHARE * worst_moment(spec.canopy.span)
        results.append(_tube_stress(spec.le_tube, "LE tube", m))

    if spec.spar is not None:
        m = worst_moment(spec.spar.length)
        # dock point load adds F=m_dock·g at midspan of worst segment: M += F·L/4
        if spec.dock_capacity:
            pts = sorted({0.0, *spec.bridle.positions, 1.0})
            seg = max(b - a for a, b in zip(pts, pts[1:])) * spec.spar.length
            m += spec.dock_capacity * G * seg / 4
        results.append(_tube_stress(spec.spar, "keel spar", m))

    if spec.struts is not None and spec.canopy is not None:
        # struts are chordwise profile stiffeners: simply supported between
        # LE tube and TE bridle, carrying the membrane-shared strip load
        share = 0.35 * tow_force_n / max(spec.n_struts, 1)
        m = share * spec.struts.length / 8
        results.append(_tube_stress(spec.struts, f"strut ×{spec.n_struts}", m))

    if spec.lobe is not None:
        # near-zero-superpressure envelope; assume 500 Pa gust superpressure
        hoop = 500 * (spec.lobe.diameter / 2) / 2   # sphere: N = p·r/2
        results.append(
            TubeStressResult(
                label="lobe envelope (hoop only)",
                hoop_stress_n_per_m=hoop,
                hoop_utilization=hoop / 60_000,
                wrinkle_moment_nm=math.nan,
                collapse_moment_nm=math.nan,
                applied_moment_nm=0.0,
                bending_utilization=0.0,
            )
        )

    if spec.fat_wing is not None:
        # lofted fat wing: one pressurized body carries ALL aero bending.
        # Wrinkle: equivalent beam of diameter t_max (conservative for a
        # wing box). Hoop: the skin balloons between internal cell webs,
        # bulge radius ≈ cell pitch / 2 — so more cells = tighter skin.
        fw = spec.fat_wing
        beam = _tube_stress(fw.equivalent_tube,
                            f"fat-wing box (equiv Ø{fw.t_max:.1f} m)",
                            worst_moment(fw.span))
        hoop = fw.pressure_bar * 1e5 * fw.cell_pitch / 2
        results.append(TubeStressResult(
            label=beam.label,
            hoop_stress_n_per_m=hoop,
            hoop_utilization=hoop / fw.fabric_strength_n_per_m,
            wrinkle_moment_nm=beam.wrinkle_moment_nm,
            collapse_moment_nm=beam.collapse_moment_nm,
            applied_moment_nm=beam.applied_moment_nm,
            bending_utilization=beam.bending_utilization,
        ))

    if spec.hull is not None:
        # near-zero-superpressure hull; 500 Pa gust superpressure, cylinder
        # hoop N = p·r at max diameter (worse than the spheroid's ends)
        hoop = 500 * (spec.hull.diameter / 2)
        results.append(
            TubeStressResult(
                label="hull envelope (hoop only)",
                hoop_stress_n_per_m=hoop,
                hoop_utilization=hoop / spec.hull.fabric_strength_n_per_m,
                wrinkle_moment_nm=math.nan,
                collapse_moment_nm=math.nan,
                applied_moment_nm=0.0,
                bending_utilization=0.0,
            )
        )

    if spec.torus is not None:
        # torus: hoop check only at L0 (bending of a ring under 3-pt bridle
        # is an L1 problem). Treat minor tube as hoop vessel.
        hoop = spec.torus.pressure_bar * 1e5 * (spec.torus.tube_diameter / 2)
        results.append(
            TubeStressResult(
                label="torus tube (hoop only)",
                hoop_stress_n_per_m=hoop,
                hoop_utilization=hoop / spec.torus.fabric_strength_n_per_m,
                wrinkle_moment_nm=math.nan,
                collapse_moment_nm=math.nan,
                applied_moment_nm=0.0,
                bending_utilization=0.0,
            )
        )
    return results


# ---------------------------------------------------------------------------
@dataclass
class WindEnvelope:
    v_min_ms: float                 # 0 if buoyant enough to fly own mass+tether
    v_max_ms: float
    v_max_limiter: str              # what sets the ceiling
    tow_force_at_12ms_kn: float
    vertical_capacity_at_10ms_kg: float  # spare vertical lift at 10 m/s
    v_mission_ms: float = 0.0       # ceiling with tether elevation ≥ 45°
                                    # (≤ v_max; binds drag-only aerostats,
                                    # whose fixed buoyancy loses to v² drag)


def solve_wind_envelope(spec: KytoonSpec) -> WindEnvelope:
    buoy = solve_buoyancy(spec)
    deficit_kg = max(0.0, -(buoy.net_static_lift_kg - spec.tether.mass))
    eta_v = math.cos(math.radians(90 - spec.tether.elevation_deg))  # vertical fraction
    S, cl, cr = spec.wing_area, 0.0, 0.0
    if spec.canopy:
        cl, cr = spec.canopy.cl_op, spec.canopy.cr_op

    if deficit_kg <= 0 or S == 0:
        v_min = 0.0
    else:
        v_min = math.sqrt(2 * deficit_kg * G / (RHO_AIR * S * cl * eta_v))

    # v_max candidates ------------------------------------------------------
    limits: dict[str, float] = {}
    if S:
        q_of = lambda F: F / (S * cr)                       # noqa: E731
        v_of = lambda q: math.sqrt(2 * q / RHO_AIR)         # noqa: E731
        # tether working-load limit
        limits["tether WLL"] = v_of(q_of(spec.tether.wll_n))
        # wrinkle margin: find tow force where worst tube hits utilization 1.0
        lo, hi = 1.0, 5e6
        for _ in range(60):
            mid = (lo + hi) / 2
            worst = max(
                (r.bending_utilization for r in solve_structure(spec, mid)
                 if not math.isnan(r.bending_utilization)),
                default=0.0,
            )
            if worst < 1.0:
                lo = mid
            else:
                hi = mid
        limits["spar/LE wrinkle"] = v_of(q_of(lo))
        # canopy fabric limit — running load ~ q·c (chord); crude but bounding
        chord = S / spec.canopy.span
        q_fab = 0.25 * 60_000 / chord  # SF 4 on 60 kN/m ripstop laminate
        limits["canopy fabric"] = v_of(q_fab)
    else:
        # pure aerostat (torus): drag-limited by tether
        cd_blimp = 0.5
        frontal = (
            spec.torus.ring_diameter * spec.torus.tube_diameter
            if spec.torus else 1.0
        )
        limits["tether WLL (drag)"] = math.sqrt(
            2 * spec.tether.wll_n / (RHO_AIR * cd_blimp * frontal)
        )

    # pressurized-envelope dent: past q ≈ superpressure, the stagnation
    # point pushes the envelope in and it loses shape. The lobe uses the
    # same 500 Pa gust superpressure as its hoop check; torus its spec
    # pressure. High-pressure tubes (LEI, spar, fat wing) don't dent at
    # flyable q; wings-only archetypes have no such surface.
    if spec.lobe is not None or spec.hull is not None:
        limits["envelope dent"] = math.sqrt(2 * 500 / RHO_AIR)
    elif spec.torus is not None:
        limits["envelope dent"] = math.sqrt(
            2 * spec.torus.pressure_bar * 1e5 / RHO_AIR)

    limiter = min(limits, key=limits.get)  # type: ignore[arg-type]
    v_max = limits[limiter]

    # mission ceiling: straight-line force balance, elevation = atan(V/H).
    # Holding station (ISR pod, capture line overhead) needs ≥ 45°. Winged
    # kytoons gain vertical force with q and never lose the angle inside
    # their envelope; drag-only aerostats blow down as v² (L1 MoorPy
    # cross-check: Mk IV sits at 44° @ 15 m/s with sag included).
    net_n = (buoy.net_static_lift_kg - spec.tether.mass) * G

    def _elev_ok(v: float) -> bool:
        q = 0.5 * RHO_AIR * v**2
        if S:
            vert = q * S * cl + net_n
            horiz = q * S * spec.canopy.cd_op
        else:
            vert = net_n
            horiz = q * 0.5 * (spec.torus.ring_diameter *
                               spec.torus.tube_diameter if spec.torus else 1.0)
        return vert >= horiz  # tan(45°) = 1

    if _elev_ok(v_max):
        v_mission = v_max
    else:
        lo_v, hi_v = 0.0, v_max
        for _ in range(50):
            mid = 0.5 * (lo_v + hi_v)
            if _elev_ok(mid):
                lo_v = mid
            else:
                hi_v = mid
        v_mission = 0.5 * (lo_v + hi_v)

    q12 = 0.5 * RHO_AIR * 12**2
    tow12 = q12 * S * cr / 1e3 if S else 0.0
    q10 = 0.5 * RHO_AIR * 10**2
    vert10 = (q10 * S * cl * eta_v) / G if S else 0.0
    spare10 = vert10 + buoy.net_static_lift_kg - spec.tether.mass

    return WindEnvelope(
        v_min_ms=v_min,
        v_max_ms=v_max,
        v_max_limiter=limiter,
        tow_force_at_12ms_kn=tow12,
        vertical_capacity_at_10ms_kg=spare10,
        v_mission_ms=v_mission,
    )


# ---------------------------------------------------------------------------
@dataclass
class L0Report:
    spec: KytoonSpec
    buoyancy: BuoyancyResult
    envelope: WindEnvelope
    structure: list[TubeStressResult] = field(default_factory=list)


def solve(spec: KytoonSpec) -> L0Report:
    env = solve_wind_envelope(spec)
    tow_n = env.tow_force_at_12ms_kn * 1e3
    return L0Report(
        spec=spec,
        buoyancy=solve_buoyancy(spec),
        envelope=env,
        structure=solve_structure(spec, tow_n),
    )
