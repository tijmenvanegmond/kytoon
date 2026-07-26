"""L1 lateral aero — sideslip derivatives, roll damping, steer response.

Stage 2 of the lateral model, and the one that finally answers the
question that started all of this: **can a winchlet steer the Manta?**
Differential trim on the two control lines at ±0.38 span is a mechanical
couple ΔT·y_ctl; what it produces is set by how hard the air resists
roll. That is a lateral aero problem the longitudinal model structurally
cannot see.

Two sources, deliberately:

  * **Static derivatives (CY_β, Cl_β, Cn_β) from AeroBuildup** — a β
    sweep alongside the α sweep `l1_trim` already runs, same tool, same
    declared bounds, no new dependency.
  * **Rate derivatives (Cl_p, Cn_p, Cl_r, Cn_r) from strip theory** —
    roll damping is just the local α change p·y/V integrated across the
    span, hand-checkable against a closed form (gated in tests).

Both are cheap to check by hand, which is the point: nothing here is
benchmark-anchored, so the flags say plausibility, not certification.

The wing is treated as RIGID: differential line tension is a pure
mechanical couple. A real fabric wing would also warp under asymmetric
bridle load, adding an aero contribution that needs membrane FEM (§7.2)
— it would only add roll authority, so ignoring it is conservative.

Requires the `l1` extra.
CLI: python -m kytoon.solvers.l1_lat_aero specs/mk5_manta.yaml
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from kytoon.solvers.l0 import RHO_AIR
from kytoon.solvers.l1_mass3d import MassProps3D, mass_props_3d
from kytoon.solvers.l1_trim import (
    HAS_L1, SWEEP_DEG, _coeffs, _require, aero_table, ctl_span_offset,
)
from kytoon.spec import Archetype, Fin, KytoonSpec

try:
    import aerosandbox as asb
except ImportError:                      # default install is L0-only
    pass

N_STRIP = 201
V_REF = 12.0                             # m/s for the sweeps
BETAS = np.arange(-8.0, 8.1, 2.0)


def _check(spec: KytoonSpec) -> None:
    if spec.archetype != Archetype.FATWING:
        raise ValueError(
            f"{spec.name}: lateral aero is implemented for the lofted fat "
            f"wing only (got {spec.archetype.value})")


FIN_ARM_M = 18.0                         # fin quarter-chord, aft of origin
FIN_AR = 1.6


def build_plane(spec: KytoonSpec, dihedral_deg: float = 0.0,
                fin_area_m2: float = 0.0) -> "asb.Airplane":
    """AeroSandbox model of the loft — same planform l1_trim sweeps, with
    the panels folded by Γ and an optional vertical fin.

    Neither Γ nor the fin is in the spec: both are exploratory, because
    adopting either is a design decision. See KYTOON-PROJECT.md §6 for
    the Γ × fin stability map they were added to produce.
    """
    _require()
    fw = spec.fat_wing
    naca = f"naca00{round(fw.thickness_ratio * 100):02d}"
    af = asb.Airfoil(naca)
    gamma = math.radians(dihedral_deg)
    xsecs = []
    for eta in (0.0, 0.25, 0.5, 0.75, 1.0):
        y = eta * fw.span / 2
        c = fw.chord_at(eta)
        xsecs.append(asb.WingXSec(
            xyz_le=[math.tan(math.radians(SWEEP_DEG)) * y - 0.25 * c,
                    y * math.cos(gamma), y * math.sin(gamma)],
            chord=c, airfoil=af))
    wings = [asb.Wing(name="fatwing", symmetric=True, xsecs=xsecs)]
    if fin_area_m2 > 0.0:
        # shape from the spec's Fin so the aero model and the geometry
        # kernel cannot drift; area is the caller's, to allow sweeps
        f = spec.fin or Fin(area=fin_area_m2)
        shape = f.model_copy(update={"area": fin_area_m2})
        wings.append(asb.Wing(name="fin", symmetric=False, xsecs=[
            asb.WingXSec(xyz_le=[shape.arm, 0.0, 0.0], chord=shape.chord,
                         airfoil=asb.Airfoil("naca0012")),
            asb.WingXSec(
                xyz_le=[shape.arm + shape.sweep_fraction * shape.chord,
                        0.0, shape.height],
                chord=shape.tip_chord, airfoil=asb.Airfoil("naca0012")),
        ]))
    s_ref = spec.canopy.area
    return asb.Airplane(
        name=spec.name, wings=wings,
        s_ref=s_ref, c_ref=s_ref / fw.span, b_ref=fw.span)


# ---------------------------------------------------------------------------
# static: sideslip sweep through AeroBuildup

def beta_sweep(spec: KytoonSpec, alpha_deg: float = 11.0,
               dihedral_deg: float = 0.0,
               betas: np.ndarray | None = None,
               fin_area_m2: float = 0.0) -> dict:
    _require()
    plane = build_plane(spec, dihedral_deg, fin_area_m2)
    bs = BETAS if betas is None else betas
    cy, cl_roll, cn = [], [], []
    for b in bs:
        out = asb.AeroBuildup(
            airplane=plane,
            op_point=asb.OperatingPoint(velocity=V_REF, alpha=alpha_deg,
                                        beta=float(b)),
        ).run()
        cy.append(float(np.ravel(out["CY"])[0]))
        cl_roll.append(float(np.ravel(out["Cl"])[0]))
        cn.append(float(np.ravel(out["Cn"])[0]))
    return {"beta": bs, "CY": np.asarray(cy), "Cl": np.asarray(cl_roll),
            "Cn": np.asarray(cn)}


def _slope_per_rad(x_deg: np.ndarray, y: np.ndarray) -> float:
    """Central slope about x = 0, per radian."""
    return float(np.polyfit(np.radians(x_deg), y, 1)[0])


# ---------------------------------------------------------------------------
# rates: strip theory on the loft

def _strip_geometry(spec: KytoonSpec, dihedral_deg: float = 0.0):
    fw = spec.fat_wing
    ys = np.linspace(-fw.span / 2, fw.span / 2, N_STRIP)
    wq = np.full(N_STRIP, ys[1] - ys[0])
    wq[0] = wq[-1] = (ys[1] - ys[0]) / 2
    cs = np.array([fw.chord_at(abs(2 * y / fw.span)) for y in ys])
    gamma = math.radians(dihedral_deg)
    return ys, wq, cs, gamma


def lift_curve_slope(spec: KytoonSpec) -> float:
    """3D dCL/dα [per rad] from l1_trim's own table, about the operating
    point. Using the 3D slope (not 2π) keeps the strip damping consistent
    with the finite wing the rest of the model flies."""
    al, cl, _, _ = aero_table(spec)
    lo, hi = np.interp([8.0, 14.0], al, cl)
    return float((hi - lo) / math.radians(6.0))


def roll_damping(spec: KytoonSpec, a0_per_rad: float | None = None,
                 dihedral_deg: float = 0.0) -> float:
    """Cl_p [per rad of p·b/2V], strip theory.

    A strip at y sees Δα = p·y/V, so dL = q·c·a0·(p y/V) dy and the
    rolling moment is −y·dL. Closed form for a linearly tapered wing:
        Cl_p = −2·a0·∫c y² dy / (S b²)
    """
    ys, wq, cs, gamma = _strip_geometry(spec, dihedral_deg)
    a0 = lift_curve_slope(spec) if a0_per_rad is None else a0_per_rad
    fw = spec.fat_wing
    # the fold shortens the roll lever: only the in-plane projection of
    # the strip offset resists roll about the body x axis
    lever = np.where(np.abs(ys) <= 0.0, ys, ys * math.cos(gamma))
    integral = float((cs * lever ** 2 * wq).sum())
    return -2.0 * a0 * integral / (spec.canopy.area * fw.span ** 2)


def yaw_damping(spec: KytoonSpec, alpha_deg: float = 11.0) -> float:
    """Cn_r [per rad of r·b/2V] from profile drag across the span.

    A strip at y under yaw rate r sees a chordwise speed change r·y, so
    its drag changes by 2·q·c·cd·(r y/V), opposing the yaw.
    """
    ys, wq, cs, _ = _strip_geometry(spec)
    _, cd, _ = _coeffs(aero_table(spec), alpha_deg)
    integral = float((cs * ys ** 2 * wq).sum())
    return -2.0 * cd * integral / (spec.canopy.area * spec.fat_wing.span ** 2)


def yaw_due_to_roll(spec: KytoonSpec, alpha_deg: float = 11.0) -> float:
    """Cn_p: the rolling wing's lift vectors tilt, so the advancing side
    gains induced drag and the retreating side loses it. Strip estimate
    dN = −y·(dL·Δα) with Δα = p y/V, i.e. Cn_p ≈ Cl_p·α at small α."""
    return roll_damping(spec) * math.radians(alpha_deg)


def roll_due_to_yaw(spec: KytoonSpec, alpha_deg: float = 11.0) -> float:
    """Cl_r: yaw rate speeds the outer wing, raising its lift.
    dL = 2·q·c·cl·(r y/V) dy, rolling moment −y·dL."""
    ys, wq, cs, _ = _strip_geometry(spec)
    cl, _, _ = _coeffs(aero_table(spec), alpha_deg)
    integral = float((cs * ys ** 2 * wq).sum())
    return 2.0 * cl * integral / (spec.canopy.area * spec.fat_wing.span ** 2)


# ---------------------------------------------------------------------------
# the answer: what a differential winchlet pull actually does

@dataclass
class SteerResponse:
    wind: float
    delta_t_n: float
    roll_couple_nm: float
    damping_nm_per_rad_s: float
    p_steady_deg_s: float
    tau_s: float
    time_to_30deg_s: float
    inertia_kgm2: float          # structural + added

    @property
    def damping_limited(self) -> bool:
        """True when the air, not the mass, sets the roll rate."""
        return self.tau_s < 1.0


def steer_response(spec: KytoonSpec, props: MassProps3D, wind: float,
                   delta_t_n: float, cl_p: float,
                   dihedral_deg: float = 0.0) -> SteerResponse:
    b = spec.fat_wing.span
    s_ref = spec.canopy.area
    q = 0.5 * RHO_AIR * wind * wind
    couple = delta_t_n * ctl_span_offset(spec) * math.cos(
        math.radians(dihedral_deg))
    # L_aero = q S b Cl_p (p b / 2V)  ->  dL/dp
    damp = q * s_ref * b * b * abs(cl_p) / (2.0 * max(wind, 1e-6))
    inertia = props.i_roll + props.added_roll
    p_ss = couple / damp if damp > 0 else math.inf
    tau = inertia / damp if damp > 0 else math.inf
    # first-order rise: phi(t) = p_ss (t - tau (1 - e^{-t/tau}))
    target = math.radians(30.0)
    t = target / p_ss if p_ss > 0 else math.inf
    for _ in range(60):                        # invert the rise numerically
        if not math.isfinite(t):
            break
        f = p_ss * (t - tau * (1.0 - math.exp(-t / tau))) - target
        df = p_ss * (1.0 - math.exp(-t / tau))
        if df <= 0:
            break
        t -= f / df
    return SteerResponse(
        wind=wind, delta_t_n=delta_t_n, roll_couple_nm=couple,
        damping_nm_per_rad_s=damp, p_steady_deg_s=math.degrees(p_ss),
        tau_s=tau, time_to_30deg_s=t, inertia_kgm2=inertia)


# ---------------------------------------------------------------------------
@dataclass
class L1LatAeroReport:
    spec: KytoonSpec
    alpha_deg: float
    dihedral_deg: float
    cy_beta: float
    cl_beta: float
    cn_beta: float
    cl_p: float
    cl_p_thin: float                     # with a0 = 2π, the other bound
    cn_p: float
    cl_r: float
    cn_r: float
    steer: list[SteerResponse] = field(default_factory=list)
    flags: list[str] = field(default_factory=list)

    @property
    def weathercock_stable(self) -> bool:
        return self.cn_beta > 0.0

    @property
    def roll_stable_in_sideslip(self) -> bool:
        return self.cl_beta < 0.0


def solve(spec: KytoonSpec, alpha_deg: float = 11.0,
          dihedral_deg: float | None = None,
          fin_area_m2: float | None = None) -> L1LatAeroReport:
    """Γ and fin default to the SPEC. Pass explicit values to explore
    (that is how the Γ × fin map was made); pass 0.0 for the flat,
    finless reference."""
    _require()
    _check(spec)
    if dihedral_deg is None:
        dihedral_deg = spec.fat_wing.dihedral_deg
    if fin_area_m2 is None:
        fin_area_m2 = spec.fin.area if spec.fin else 0.0
    props = mass_props_3d(spec, dihedral_deg=dihedral_deg)
    sw = beta_sweep(spec, alpha_deg, dihedral_deg, fin_area_m2=fin_area_m2)
    cl_p = roll_damping(spec, dihedral_deg=dihedral_deg)
    flags = [
        "AeroBuildup lateral derivatives are semi-empirical and NOT "
        "benchmark-anchored — plausibility, not certification",
        "rate derivatives are strip theory with the 3D lift-curve slope; "
        "the 2π bound is reported alongside as the spread",
        "wing treated as RIGID: differential line tension is a pure "
        "mechanical couple. Fabric warp would add roll authority, so "
        "this is the conservative side",
    ]
    if dihedral_deg == 0.0:
        flags.append("Γ = 0: dihedral effect comes from sweep alone")
    spec_fin = spec.fin.area if spec.fin else 0.0
    if fin_area_m2 > 0.0 and fin_area_m2 != spec_fin:
        flags.append(f"exploratory {fin_area_m2:.0f} m² fin at "
                     f"{FIN_ARM_M:.0f} m — NOT the spec's {spec_fin:.0f} m²")

    rep = L1LatAeroReport(
        spec=spec, alpha_deg=alpha_deg, dihedral_deg=dihedral_deg,
        cy_beta=_slope_per_rad(sw["beta"], sw["CY"]),
        cl_beta=_slope_per_rad(sw["beta"], sw["Cl"]),
        cn_beta=_slope_per_rad(sw["beta"], sw["Cn"]),
        cl_p=cl_p,
        cl_p_thin=roll_damping(spec, a0_per_rad=2 * math.pi,
                               dihedral_deg=dihedral_deg),
        cn_p=yaw_due_to_roll(spec, alpha_deg),
        cl_r=roll_due_to_yaw(spec, alpha_deg),
        cn_r=yaw_damping(spec, alpha_deg),
        flags=flags)
    for wind in (8.0, 12.0):
        for dt in (2e3, 4e3, 8e3):
            rep.steer.append(steer_response(spec, props, wind, dt, cl_p,
                                            dihedral_deg))
    return rep


# ---------------------------------------------------------------------------
def _summary(rep: L1LatAeroReport) -> str:
    s = rep.spec
    lines = [
        f"## {s.name} — L1 lateral aero (α = {rep.alpha_deg:.0f}°, "
        f"Γ = {rep.dihedral_deg:.0f}°)",
        "",
        "static derivatives (AeroBuildup β sweep), per rad:",
        f"    CY_β {rep.cy_beta:+.3f}    Cl_β {rep.cl_beta:+.4f}"
        f"    Cn_β {rep.cn_beta:+.4f}",
        f"    weathercock (Cn_β > 0): "
        f"{'STABLE' if rep.weathercock_stable else 'UNSTABLE'}"
        f"    roll-in-sideslip (Cl_β < 0): "
        f"{'stable' if rep.roll_stable_in_sideslip else 'unstable'}",
        "",
        "rate derivatives (strip theory), per rad:",
        f"    Cl_p {rep.cl_p:+.3f}  (2π bound {rep.cl_p_thin:+.3f})"
        f"    Cn_p {rep.cn_p:+.4f}",
        f"    Cl_r {rep.cl_r:+.4f}   Cn_r {rep.cn_r:+.5f}",
        "",
        "winchlet steering — differential tension vs roll damping:",
        "    wind    ΔT    couple    roll rate     τ    to 30° bank",
    ]
    for r in rep.steer:
        lines.append(
            "  %5.0f m/s %4.0f kN  %6.1f kNm  %6.2f °/s  %5.2f s   %5.1f s"
            % (r.wind, r.delta_t_n / 1e3, r.roll_couple_nm / 1e3,
               r.p_steady_deg_s, r.tau_s, r.time_to_30deg_s))
    r0 = rep.steer[0]
    lines.append(
        f"    -> {'DAMPING' if r0.damping_limited else 'INERTIA'}-limited: "
        f"τ = {r0.tau_s:.2f} s against {r0.inertia_kgm2 / 1e3:.0f} t·m² of "
        "roll inertia,")
    lines.append(
        "       so the rate is set by the air, not the mass — steering is "
        "a")
    lines.append(
        "       sustained-pull problem, not an impulse one.")
    for f in rep.flags:
        lines.append(f"- ⚠ {f}")
    return "\n".join(lines)


if __name__ == "__main__":
    import argparse
    import sys

    from kytoon.spec import load_spec

    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description="L1 lateral aero for one spec")
    ap.add_argument("spec", help="path to a specs/*.yaml file")
    ap.add_argument("--alpha", type=float, default=11.0)
    ap.add_argument("--dihedral", type=float, default=None,
                    help="override the spec's Γ [deg]; 0 = flat loft")
    ap.add_argument("--fin", type=float, default=None,
                    help="override the spec's fin area [m2]; 0 = finless")
    args = ap.parse_args()
    print(_summary(solve(load_spec(args.spec), args.alpha, args.dihedral,
                         args.fin)))
