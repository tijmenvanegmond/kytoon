"""L1 body-interference aero — hybrid archetypes (Mk II lobe+wing,
Mk V hull+wing) through AeroSandbox's AeroBuildup.

The VSM path (l1_aero.py) handles pure membrane wings; it cannot model a
large buoyant body next to the wing. This module covers that gap with
AeroSandbox's semi-empirical component buildup: wing lifting-line + body of
revolution + basic wing-body interference. It is NOT benchmark-anchored the
way the V3 path is — its job is to bound the hand-picked spec coefficients,
not to certify them.

Honesty notes (also emitted as report flags):
  - Mk V hull is modeled rigid and smooth → body drag is a LOWER bound
    (soft envelope wrinkles/separation not captured).
  - Mk II's oblate lobe is approximated as an equivalent body of revolution
    (frontal area matched) and its wake blanketing of the keel wing is NOT
    captured → CL is an UPPER bound.
  - Bridle parasitic drag added with the same scaling law as l1_aero.

Requires the `l1` extra (aerosandbox rides in with VSM's neuralfoil dep).

CLI: python -m kytoon.solvers.l1_body_aero specs/mk5_manta.yaml
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from kytoon.aero import Polar, PolarPoint, SystemPolar, bridle_cd
from kytoon.spec import Archetype, KytoonSpec

try:
    import aerosandbox as asb
    HAS_AEROSANDBOX = True
except ImportError:  # default install is L0-only by design
    HAS_AEROSANDBOX = False

WING_AIRFOIL = "naca2410"       # thin cambered section for fabric deltas
N_BODY_SECTIONS = 21


def _require() -> None:
    if not HAS_AEROSANDBOX:
        raise ImportError(
            "aerosandbox not installed — body aero needs the l1 extra: "
            'pip install "kytoon-sim[l1]"'
        )


def _revolution_body(name: str, length: float, radius: float,
                     center: tuple[float, float, float] = (0, 0, 0)):
    """Spheroid of revolution about x (blimp hull / equivalent lobe)."""
    xs = np.linspace(0, 1, N_BODY_SECTIONS)
    return asb.Fuselage(
        name=name,
        xsecs=[
            asb.FuselageXSec(
                xyz_c=[center[0] + length * (x - 0.5), center[1], center[2]],
                radius=radius * float(
                    np.sqrt(max(1 - (2 * x - 1) ** 2, 0.0))),
            )
            for x in xs
        ],
    )


def build_airplane(spec: KytoonSpec) -> "asb.Airplane":
    """AeroSandbox model of a hybrid (body + wing) archetype."""
    _require()
    af = asb.Airfoil(WING_AIRFOIL)

    if spec.archetype == Archetype.BLIMP:
        # geometry mirrors kytoon.geometry's blimp branch
        hr = spec.hull.diameter / 2
        y_root = 0.9 * hr
        half_span = spec.canopy.span / 2 - y_root
        root = spec.canopy.area / half_span
        wing = asb.Wing(
            name="side deltas", symmetric=True,
            xsecs=[
                asb.WingXSec(xyz_le=[-root / 2, y_root, 0],
                             chord=root, airfoil=af),
                asb.WingXSec(
                    xyz_le=[-root / 2 + 0.75 * root, spec.canopy.span / 2, 0.8],
                    chord=0.02 * root, airfoil=af),
            ],
        )
        body = _revolution_body("hull", spec.hull.length, hr)

    elif spec.archetype == Archetype.HELIKITE:
        # v2: two side deltas rooted on the lobe equator (mirrors
        # kytoon.geometry), lobe as an equivalent-frontal revolution body
        y_root = 0.9 * spec.lobe.diameter / 2
        half_span = spec.canopy.span / 2 - y_root
        root = spec.canopy.area / half_span
        wing = asb.Wing(
            name="side deltas", symmetric=True,
            xsecs=[
                asb.WingXSec(xyz_le=[-root / 2, y_root, 0],
                             chord=root, airfoil=af),
                asb.WingXSec(
                    xyz_le=[-root / 2 + 0.75 * root, spec.canopy.span / 2, 0.6],
                    chord=0.02 * root, airfoil=af),
            ],
        )
        r_eq = math.sqrt(spec.lobe.diameter / 2 * spec.lobe.height / 2)
        body = _revolution_body("lobe (equiv. revolution)",
                                spec.lobe.diameter, r_eq)
    else:
        raise ValueError(
            f"{spec.name}: {spec.archetype.value} is not a body+wing hybrid "
            "— use l1_aero (VSM) or L0"
        )

    return asb.Airplane(
        name=spec.name, wings=[wing], fuselages=[body],
        s_ref=spec.canopy.area,
        c_ref=spec.canopy.area / spec.canopy.span,
        b_ref=spec.canopy.span,
    )


def sweep(plane: "asb.Airplane", ref_area: float,
          alphas: np.ndarray | None = None, umag: float = 10.0) -> Polar:
    """Alpha sweep → Polar (coefficients on the spec's wing area)."""
    _require()
    if alphas is None:
        alphas = np.arange(-4.0, 20.5, 2.0)
    pts = []
    for a in alphas:
        aero = asb.AeroBuildup(
            airplane=plane,
            op_point=asb.OperatingPoint(velocity=umag, alpha=float(a)),
        ).run()
        pts.append(PolarPoint(alpha=float(a),
                              cl=float(np.ravel(aero["CL"])[0]),
                              cd=float(np.ravel(aero["CD"])[0])))
    return Polar(pts, source=f"AeroBuildup S_ref={ref_area}")


# ---------------------------------------------------------------------------
@dataclass
class L1BodyAeroReport:
    spec: KytoonSpec
    clean: Polar                    # wing+body, no bridle
    system: SystemPolar             # + bridle parasitic drag
    op_alpha: float | None
    cl_op_l1: float
    cd_op_l1: float
    flags: list[str] = field(default_factory=list)

    @property
    def cr_op_l1(self) -> float:
        return math.hypot(self.cl_op_l1, self.cd_op_l1)

    @property
    def cr_op_spec(self) -> float:
        return math.hypot(self.spec.canopy.cl_op, self.spec.canopy.cd_op)

    @property
    def cr_ratio(self) -> float:
        return self.cr_op_l1 / self.cr_op_spec if self.cr_op_spec else math.inf


def solve(spec: KytoonSpec, alphas: np.ndarray | None = None,
          umag: float = 10.0) -> L1BodyAeroReport:
    flags: list[str] = []
    if spec.archetype == Archetype.BLIMP:
        flags.append("rigid smooth hull assumed — body drag is a lower "
                     "bound for a soft envelope")
    if spec.archetype == Archetype.HELIKITE:
        flags.append("lobe interference on the side wings only partially "
                     "captured (equivalent revolution body); wake "
                     "blanketing unmodeled — CL is an upper bound")
    flags.append("AeroBuildup is semi-empirical, not benchmark-anchored — "
                 "bounds the spec coefficients, does not certify them")

    plane = build_airplane(spec)
    clean = sweep(plane, spec.canopy.area, alphas, umag)

    scale = (spec.canopy.area / 19.753) ** 0.5
    n_lines = max(1, round(82 * scale))
    mean_len = (96.0 / 82) * scale
    system = SystemPolar(wing=clean,
                         cd_bridle=bridle_cd(n_lines, mean_len,
                                             spec.canopy.area))

    target_cl = spec.canopy.cl_op
    op_alpha = clean.alpha_for_cl(target_cl)
    if op_alpha is None:
        flags.append(f"spec cl_op={target_cl} NOT reached in the swept "
                     f"range (CL_max={clean.cl_max:.2f})")
        op_alpha_eff = clean.alpha_cl_max
    else:
        op_alpha_eff = op_alpha

    return L1BodyAeroReport(
        spec=spec,
        clean=clean,
        system=system,
        op_alpha=op_alpha,
        cl_op_l1=clean.cl(op_alpha_eff),
        cd_op_l1=system.cd(op_alpha_eff),
        flags=flags,
    )


def v_max_tether_with(spec: KytoonSpec, cr: float) -> float:
    """Straight-line tether-WLL ceiling for a given resultant coefficient
    (the L0 formula, re-evaluated with L1 coefficients)."""
    from kytoon.solvers.l0 import RHO_AIR
    return math.sqrt(
        2 * spec.tether.wll_n / (RHO_AIR * spec.canopy.area * cr))


# ---------------------------------------------------------------------------
def _summary(rep: L1BodyAeroReport) -> str:
    s = rep.spec
    lines = [
        f"## {s.name} — L1 body-interference aero (AeroBuildup)",
        "",
        f"- CL_max(swept) {rep.clean.cl_max:.2f}, "
        f"system (L/D)max {rep.system.ld_max:.1f}",
        f"- operating point cl_op={s.canopy.cl_op}: "
        + (f"α ≈ {rep.op_alpha:.1f}°" if rep.op_alpha is not None
           else "NOT reached"),
        f"- L1 cl/cd at op: {rep.cl_op_l1:.2f} / {rep.cd_op_l1:.3f} "
        f"(spec: {s.canopy.cl_op} / {s.canopy.cd_op}) — "
        f"resultant ratio L1/spec = {rep.cr_ratio:.2f}",
        f"- tether-WLL v_max with L1 cr: "
        f"{v_max_tether_with(s, rep.cr_op_l1):.1f} m/s "
        f"(spec cr gives {v_max_tether_with(s, rep.cr_op_spec):.1f})",
    ]
    for f in rep.flags:
        lines.append(f"- ⚠ {f}")
    return "\n".join(lines)


if __name__ == "__main__":
    import argparse
    import sys

    from kytoon.spec import load_spec

    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description="L1 body aero for one spec")
    ap.add_argument("spec", help="path to a specs/*.yaml file")
    args = ap.parse_args()
    print(_summary(solve(load_spec(args.spec))))
