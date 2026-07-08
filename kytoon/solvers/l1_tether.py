"""L1 tether solver — quasi-static line with weight + aero drag (MoorPy).

Replaces the L0 straight-line tether (KYTOON-PROJECT.md §3.5) with NREL's
MoorPy quasi-static mooring solver, run "upside down": the kite tether is a
mooring line in a uniform current of *air* (System(rho=1.225), wind as the
current vector), anchored at the ship winch with the kytoon's aerodynamic +
buoyant resultant applied as an external force on the free end.

What this corrects over L0:
  - tether drag integrates along the line (raises ship-end tension → v_max)
  - sag: line shape, kite altitude loss vs the straight-line assumption
  - tether elevation becomes an OUTPUT of the force balance. L0 treats
    spec.tether.elevation_deg as an input; the L1 solve reports the actual
    chord and end angles, and flags disagreement with the spec value.

Deliberately still quasi-static: no dynamics, no gusts (L2 territory).

Requires the `l1` extra (moorpy is on PyPI, pure Python).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from kytoon.solvers.l0 import G, RHO_AIR, solve_buoyancy
from kytoon.spec import Archetype, KytoonSpec

try:
    import moorpy
    HAS_MOORPY = True
except ImportError:  # default install is L0-only by design
    HAS_MOORPY = False

# HMPE (Dyneema-class) braid: axial stiffness EA ≈ 50 × MBL is a standard
# rough figure; quasi-static shape is insensitive to it (stretch ≪ 1%).
EA_PER_MBL = 50.0
CD_LINE = 1.1        # transverse cylinder drag (same figure as bridle model)
CDAX_LINE = 0.008    # tangential skin friction
WINCH_ABOVE_WATER = 5.0   # m — deck winch height; line below this = in the sea
CD_TORUS = 0.5       # bluff-body ring drag, same as L0
_DEPTH = 1000.0      # keep MoorPy's seabed far away; we fly, not moor


def kite_force(spec: KytoonSpec, v_wind: float) -> np.ndarray:
    """Net external force the kytoon applies to the tether top [N].

    x downwind, z up. Aero at the spec operating point (cl_op/cd_op on wing
    area; bluff-body drag for the torus) + net static lift (buoyancy − mass).
    """
    q = 0.5 * RHO_AIR * v_wind**2
    buoy = solve_buoyancy(spec).net_static_lift_kg * G
    if spec.canopy is not None:
        s = spec.canopy.area
        return np.array([q * s * spec.canopy.cd_op, 0.0,
                         q * s * spec.canopy.cl_op + buoy])
    frontal = (spec.torus.ring_diameter * spec.torus.tube_diameter
               if spec.torus else 0.0)
    return np.array([q * CD_TORUS * frontal, 0.0, buoy])


@dataclass
class L1TetherReport:
    spec: KytoonSpec
    v_wind: float
    kite_force_n: np.ndarray        # applied end force [x, y, z]
    converged: bool
    altitude: float                 # kite height above winch [m]
    altitude_straight: float        # L·sin(spec elevation) — the L0 picture
    elevation_chord_deg: float      # winch→kite line-of-sight angle
    elevation_ship_deg: float       # line tangent at the winch
    elevation_kite_deg: float       # line tangent at the kite
    tension_ship_n: float
    tension_kite_n: float
    wll_fraction: float             # ship-end tension / tether WLL
    sag_max_m: float                # max perpendicular deviation from chord
    line_drag_n: float              # integrated aero drag on the line
    flags: list[str] = field(default_factory=list)


def solve(spec: KytoonSpec, v_wind: float, n_segs: int = 40) -> L1TetherReport:
    """Quasi-static tether equilibrium at one wind speed."""
    if not HAS_MOORPY:
        raise ImportError(
            "moorpy not installed — L1 tether needs the l1 extra: "
            'pip install "kytoon-sim[l1]"'
        )
    t = spec.tether
    f_kite = kite_force(spec, v_wind)
    flags: list[str] = []

    ms = moorpy.System(depth=_DEPTH, rho=RHO_AIR, g=G,
                       current=[v_wind, 0.0, 0.0])
    d = t.diameter_mm / 1000
    w = (t.linear_density - math.pi / 4 * d**2 * RHO_AIR) * G
    mbl = t.mbl_kn * 1e3
    ms.lineTypes["tether"] = dict(
        name="tether", d_vol=d, m=t.linear_density, w=w,
        EA=EA_PER_MBL * mbl, MBL=mbl, Cd=CD_LINE, CdAx=CDAX_LINE,
    )

    ms.addPoint(1, [0.0, 0.0, 0.0])                       # winch (fixed)
    # start straight along the applied-force direction (robust guess)
    f_norm = np.linalg.norm(f_kite)
    r0 = (t.length * f_kite / f_norm) if f_norm else [0, 0, t.length]
    ms.addPoint(0, list(r0), fExt=list(f_kite))           # kite (free)
    ms.addLine(t.length, "tether", nSegs=n_segs, pointA=1, pointB=2)

    ms.initialize()
    converged = bool(ms.solveEquilibrium())

    kite = ms.pointList[1]
    line = ms.lineList[0]
    alt = float(kite.r[2])
    if not converged:
        flags.append("MoorPy equilibrium did not converge — treat as no fly")
    if alt <= 0:
        flags.append("kite below winch height — no positive-altitude "
                     "equilibrium at this wind")

    # line shape → sag + water check
    xs, ys, zs, _ = line.getLineCoords(0.0)
    coords = np.column_stack([xs, ys, zs])
    if float(np.min(zs)) < -WINCH_ABOVE_WATER:
        flags.append("tether bight drops below the waterline")
    chord = kite.r - np.zeros(3)
    c_norm = np.linalg.norm(chord)
    if c_norm > 0:
        proj = coords @ (chord / c_norm)
        perp = coords - np.outer(proj, chord / c_norm)
        sag = float(np.max(np.linalg.norm(perp, axis=1)))
    else:
        sag = float("nan")

    f_ship = np.asarray(line.fA, dtype=float)
    f_kite_end = np.asarray(line.fB, dtype=float)
    tension_ship = float(np.linalg.norm(f_ship))
    wll = mbl / t.safety_factor
    if tension_ship > wll:
        flags.append(f"ship-end tension {tension_ship/1e3:.1f} kN exceeds "
                     f"WLL {wll/1e3:.1f} kN")

    elev_chord = math.degrees(math.atan2(kite.r[2], max(kite.r[0], 1e-9)))
    if abs(elev_chord - t.elevation_deg) > 10:
        flags.append(
            f"force-balance elevation {elev_chord:.0f}° vs spec "
            f"elevation_deg {t.elevation_deg}° — L0 uses the spec value as "
            "an input; it is actually an output (design conversation)"
        )

    return L1TetherReport(
        spec=spec,
        v_wind=v_wind,
        kite_force_n=f_kite,
        converged=converged,
        altitude=alt,
        altitude_straight=t.length * math.sin(math.radians(t.elevation_deg)),
        elevation_chord_deg=elev_chord,
        elevation_ship_deg=math.degrees(math.atan2(f_ship[2], f_ship[0])),
        elevation_kite_deg=math.degrees(
            math.atan2(-f_kite_end[2], -f_kite_end[0])),
        tension_ship_n=tension_ship,
        tension_kite_n=float(np.linalg.norm(f_kite_end)),
        wll_fraction=tension_ship / wll,
        sag_max_m=sag,
        line_drag_n=float(np.linalg.norm(line.fCurrent)),
        flags=flags,
    )


def v_max_tether(spec: KytoonSpec, v_lo: float = 1.0, v_hi: float = 80.0,
                 tol: float = 0.05) -> float:
    """Wind speed where ship-end tension (incl. line drag/sag) hits the WLL.

    The L1 counterpart of L0's straight-line 'tether WLL' limit — always at
    or below it, because the line's own drag adds tension at the winch.
    """
    wll_frac = lambda v: solve(spec, v, n_segs=20).wll_fraction  # noqa: E731
    if wll_frac(v_hi) < 1.0:
        return v_hi
    while v_hi - v_lo > tol:
        mid = 0.5 * (v_lo + v_hi)
        if wll_frac(mid) < 1.0:
            v_lo = mid
        else:
            v_hi = mid
    return 0.5 * (v_lo + v_hi)


# ---------------------------------------------------------------------------
def _summary(rep: L1TetherReport) -> str:
    lines = [
        f"## {rep.spec.name} — L1 tether @ {rep.v_wind:.1f} m/s (MoorPy)",
        "",
        f"- kite altitude {rep.altitude:.0f} m "
        f"(straight-line spec picture: {rep.altitude_straight:.0f} m)",
        f"- elevation: chord {rep.elevation_chord_deg:.1f}°, "
        f"ship end {rep.elevation_ship_deg:.1f}°, "
        f"kite end {rep.elevation_kite_deg:.1f}° "
        f"(spec input: {rep.spec.tether.elevation_deg}°)",
        f"- tension ship {rep.tension_ship_n/1e3:.1f} kN "
        f"({rep.wll_fraction*100:.0f}% WLL), "
        f"kite {rep.tension_kite_n/1e3:.1f} kN",
        f"- sag {rep.sag_max_m:.1f} m, line drag {rep.line_drag_n:.0f} N",
    ]
    for f in rep.flags:
        lines.append(f"- ⚠ {f}")
    return "\n".join(lines)


if __name__ == "__main__":
    import argparse
    import sys

    from kytoon.spec import load_spec

    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description="L1 tether solve for one spec")
    ap.add_argument("spec", help="path to a specs/*.yaml file")
    ap.add_argument("-v", "--wind", type=float, default=12.0)
    ap.add_argument("--vmax", action="store_true",
                    help="also bisect the drag-corrected tether v_max")
    args = ap.parse_args()
    s = load_spec(args.spec)
    print(_summary(solve(s, args.wind)))
    if args.vmax:
        print(f"- drag-corrected tether v_max: {v_max_tether(s):.1f} m/s")
