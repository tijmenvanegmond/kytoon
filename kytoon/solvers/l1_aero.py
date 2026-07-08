"""L1 aerodynamic solver — Vortex Step Method over parametric LEI geometry.

First fidelity step above L0: instead of borrowing the TU Delft V3's polar
(kytoon/aero.py), build each Mk's *own* wing from its spec and solve it with
the awegroup Vortex Step Method (VSM) — the open solver TU Delft validates
against the very dataset vendored in data/tudelft_v3/.

Geometry model (deliberately minimal — the YAML spec is the parameter set):
  C-shaped circular arc in the front view, matched to the spec's projected
  span and area. Arc depth and chord taper default to the V3's measured
  shape (height/span 0.376, tip/center chord 0.55). Sections are flat
  (no twist/billow — that is membrane FEM territory, L1 structure task).

Section aerodynamics: Breukels' 2-parameter LEI airfoil regression
  (t = LE-tube diameter / chord, kappa = camber), as shipped with VSM.

Validation anchor (tests/test_l1_aero.py): this pipeline, fed only the V3's
published bulk numbers, reproduces the vendored wind-tunnel polar with
  CL_max  +10%   (1.18 vs 1.07; published CFD with struts gives 1.35)
  (L/D)max −19%  (7.0 vs 8.7; Breukels sections are draggier than the
                  smooth rigid tunnel model — treat L1 L/D as conservative)
Both deviations are gated in the tests; numbers outside those bands mean
the pipeline (or a spec) broke, not that the model got better.

Requires the `l1` extra (VSM is git-installed, not on PyPI):
  pip install "kytoon-sim[l1]"
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from kytoon.aero import Polar, PolarPoint, SystemPolar, bridle_cd
from kytoon.spec import Archetype, KytoonSpec

try:
    from VSM.core.AirfoilAerodynamics import AirfoilAerodynamics
    from VSM.core.BodyAerodynamics import BodyAerodynamics
    from VSM.core.Solver import Solver as VSMSolver
    from VSM.core.WingGeometry import Wing
    HAS_VSM = True
except ImportError:  # default install is L0-only by design
    HAS_VSM = False

# V3-derived shape defaults (data/tudelft_v3/properties.yaml):
# projected height 3.13 m / projected span 8.32 m; chord fullness 0.85
# (flat area / (flat span · max chord)) → parabolic taper ratio 0.55.
ARC_HEIGHT_RATIO = 0.376
CHORD_TAPER = 0.55
KAPPA_CAMBER = 0.08          # typical LEI camber for Breukels regression
TWIN_SKIN_TUBE_T = 0.06      # slim effective LE for twin-skin sections (approx.)
SECTION_ALPHA_RANGE = [-15.0, 30.0, 0.5]   # deg, Breukels polar span


@dataclass
class ArcWing:
    """Parametric C-arc LEI wing, fully determined by spec bulk numbers."""
    span: float            # m, projected (tip-to-tip width)
    area: float            # m², projected
    height_ratio: float = ARC_HEIGHT_RATIO
    taper: float = CHORD_TAPER
    tube_t: float = 0.10   # LE tube diameter / chord for Breukels section
    kappa: float = KAPPA_CAMBER
    n_sections: int = 20

    @classmethod
    def from_spec(cls, spec: KytoonSpec) -> "ArcWing":
        if spec.canopy is None:
            raise ValueError(f"{spec.name}: no canopy — no wing to solve at L1")
        mean_chord = spec.canopy.area / spec.canopy.span
        if spec.le_tube is not None:
            tube_t = spec.le_tube.diameter / mean_chord
        else:
            tube_t = TWIN_SKIN_TUBE_T
        return cls(span=spec.canopy.span, area=spec.canopy.area, tube_t=tube_t)

    # --- closed-form arc geometry (hand-checkable, in the L0 spirit) -------
    @property
    def arc_half_angle(self) -> float:
        # height/span = tan(θ0/2)/2 for a circular arc
        return 2 * math.atan(2 * self.height_ratio)

    @property
    def arc_radius(self) -> float:
        return self.span / (2 * math.sin(self.arc_half_angle))

    def _chord_shape(self, phi: float) -> float:
        return 1 - (1 - self.taper) * (phi / self.arc_half_angle) ** 2

    def section_points(self) -> list[tuple[np.ndarray, np.ndarray]]:
        """(LE, TE) per section: x downstream, y spanwise, z up."""
        th0, r = self.arc_half_angle, self.arc_radius
        phis = np.linspace(-th0, th0, self.n_sections)
        # projected area = ∫ c dy; y = R·sinφ carries the cosφ Jacobian
        c0 = self.area / np.trapezoid([self._chord_shape(p) for p in phis],
                                      r * np.sin(phis))
        out = []
        for phi in phis:
            le = np.array([0.0, r * math.sin(phi), r * math.cos(phi)])
            te = le + np.array([c0 * self._chord_shape(phi), 0.0, 0.0])
            out.append((le, te))
        return out


def build_body(geom: ArcWing, n_panels: int = 40) -> "BodyAerodynamics":
    if not HAS_VSM:
        raise ImportError(
            "VSM not installed — L1 aero needs the l1 extra: "
            'pip install "kytoon-sim[l1]"'
        )
    polar = AirfoilAerodynamics.from_yaml_entry(
        "breukels_regression",
        {"t": geom.tube_t, "kappa": geom.kappa},
        alpha_range=SECTION_ALPHA_RANGE,
    ).to_polar_array()
    wing = Wing(n_panels=n_panels, spanwise_panel_distribution="uniform")
    for le, te in geom.section_points():
        wing.add_section(le, te, polar)
    return BodyAerodynamics.instantiate(n_panels=n_panels, wing_instance=wing)


def sweep(geom: ArcWing, alphas: np.ndarray | None = None, umag: float = 10.0,
          n_panels: int = 40) -> Polar:
    """Alpha sweep → clean-wing Polar (same type kytoon.aero uses)."""
    if alphas is None:
        alphas = np.arange(-5.0, 25.1, 2.5)
    body = build_body(geom, n_panels)
    solver = VSMSolver()
    pts = []
    for a in alphas:
        body.va_initialize(Umag=umag, angle_of_attack=float(a), side_slip=0.0)
        res = solver.solve(body)
        pts.append(PolarPoint(alpha=float(a), cl=res["cl"], cd=res["cd"]))
    return Polar(pts, source=f"VSM arc-wing b={geom.span} S={geom.area}")


# ---------------------------------------------------------------------------
@dataclass
class L1AeroReport:
    spec: KytoonSpec
    geometry: ArcWing
    clean: Polar                    # wing-only, VSM
    system: SystemPolar             # + bridle parasitic drag
    op_alpha: float | None          # alpha reaching spec cl_op (front side)
    cl_op_l1: float
    cd_op_l1: float                 # system drag at op point
    flags: list[str] = field(default_factory=list)

    @property
    def cr_op_l1(self) -> float:
        return math.hypot(self.cl_op_l1, self.cd_op_l1)

    @property
    def cr_op_spec(self) -> float:
        return math.hypot(self.spec.canopy.cl_op, self.spec.canopy.cd_op)

    @property
    def cr_ratio(self) -> float:
        """L1 / spec resultant-force coefficient — drives tow force directly."""
        return self.cr_op_l1 / self.cr_op_spec if self.cr_op_spec else math.inf


def solve(spec: KytoonSpec, n_panels: int = 40,
          alphas: np.ndarray | None = None, umag: float = 10.0) -> L1AeroReport:
    """Mk-specific polar from the spec's own geometry (L0-interface shape)."""
    flags: list[str] = []
    if spec.archetype == Archetype.TORUS:
        raise ValueError(f"{spec.name}: pure aerostat, no wing — L1 aero n/a")
    if spec.archetype == Archetype.HELIKITE:
        flags.append(
            "lobe interference NOT modeled — wing-only polar; do not feed "
            "into the Mk II envelope (task queue §7.5)"
        )
    if spec.canopy is not None and spec.canopy.twin_skin:
        flags.append(
            "twin-skin section approximated by slim Breukels LEI profile "
            f"(t={TWIN_SKIN_TUBE_T}) — expect conservative L/D"
        )

    geom = ArcWing.from_spec(spec)
    clean = sweep(geom, alphas, umag, n_panels)

    # same bridle scaling law as the L0 calibration path (aero.py)
    scale = (spec.canopy.area / 19.753) ** 0.5
    n_lines = max(1, round(82 * scale))
    mean_len = (96.0 / 82) * scale
    system = SystemPolar(wing=clean,
                         cd_bridle=bridle_cd(n_lines, mean_len, spec.canopy.area))

    target_cl = spec.canopy.cl_op
    op_alpha = clean.alpha_for_cl(target_cl)
    if op_alpha is None:
        flags.append(
            f"spec cl_op={target_cl} NOT reached pre-stall at L1 "
            f"(CL_max={clean.cl_max:.2f} @ {clean.alpha_cl_max:.1f}°) — "
            "spec is making an unsupported aero claim"
        )
        op_alpha_eff = clean.alpha_cl_max
    else:
        op_alpha_eff = op_alpha

    return L1AeroReport(
        spec=spec,
        geometry=geom,
        clean=clean,
        system=system,
        op_alpha=op_alpha,
        cl_op_l1=clean.cl(op_alpha_eff),
        cd_op_l1=system.cd(op_alpha_eff),
        flags=flags,
    )


# ---------------------------------------------------------------------------
def _summary(rep: L1AeroReport) -> str:
    g = rep.geometry
    lines = [
        f"## {rep.spec.name} — L1 aero (VSM)",
        "",
        f"- arc wing: span {g.span} m, area {g.area} m², "
        f"tube t={g.tube_t:.3f}, {g.n_sections} sections",
        f"- CL_max {rep.clean.cl_max:.2f} @ {rep.clean.alpha_cl_max:.1f}°, "
        f"clean (L/D)max {rep.clean.ld_max:.1f}, "
        f"system (L/D)max {rep.system.ld_max:.1f}",
        f"- operating point cl_op={rep.spec.canopy.cl_op}: "
        + (f"α ≈ {rep.op_alpha:.1f}°" if rep.op_alpha is not None
           else "NOT reached"),
        f"- L1 cl/cd at op: {rep.cl_op_l1:.2f} / {rep.cd_op_l1:.3f} "
        f"(spec: {rep.spec.canopy.cl_op} / {rep.spec.canopy.cd_op}) — "
        f"resultant ratio L1/spec = {rep.cr_ratio:.2f}",
    ]
    for f in rep.flags:
        lines.append(f"- ⚠ {f}")
    return "\n".join(lines)


if __name__ == "__main__":
    import argparse
    import sys

    from kytoon.spec import load_spec

    sys.stdout.reconfigure(encoding="utf-8")  # summary uses α/⚠; cp1252 consoles fail

    ap = argparse.ArgumentParser(description="L1 VSM polar for one spec")
    ap.add_argument("spec", help="path to a specs/*.yaml file")
    ap.add_argument("--panels", type=int, default=40)
    args = ap.parse_args()
    print(_summary(solve(load_spec(args.spec), n_panels=args.panels)))
