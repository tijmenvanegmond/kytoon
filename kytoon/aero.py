"""Aerodynamic calibration from the TU Delft V3 LEI kite benchmark.

Data source (open, CC-BY): awegroup/TUDELFT_V3_KITE
  - Poland et al. 2025/2026, WES 11, 911: rigid 1:6.5 scale wind-tunnel model
    of the 25 m² V3 kite, α ∈ [-11.6, 24.5]°, Re 5e5.
  - Viré et al. 2022, Energies 15, 1450: RANS CFD with struts, Re 1e6.

Reference geometry (properties.yaml):
  projected area 19.753 m², projected AR 3.498, 8 struts, bridle 96 m total.

WHY THIS MATTERS FOR THE KYTOONS
  The V3 is a high-performance AWE traction kite; its *clean-wing* (L/D)_max
  ≈ 8.7 is far better than our static lifters will see, because (a) we fly
  quasi-static, not crosswind, and (b) bridle parasitic drag is large. The
  wind-tunnel model excludes bridle lines, so we add a bridle-drag term to
  get a defensible *system* polar. That system polar is what calibrates the
  cl_op / cd_op we had hand-picked in the Mk specs.
"""
from __future__ import annotations

import bisect
import csv
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

DATA_DIR = Path(__file__).resolve().parent.parent / "data" / "tudelft_v3"

# V3 reference values used to non-dimensionalise the published coefficients
V3_PROJECTED_AREA = 19.753      # m²
V3_PROJECTED_AR = 3.498
V3_BRIDLE_LENGTH = 96.0         # m, total line length
V3_BRIDLE_DIAM = 0.0025         # m, representative Dyneema line ~2.5 mm
CD_CYLINDER = 1.0               # bluff-body drag coeff for bridle lines


@dataclass
class PolarPoint:
    alpha: float
    cl: float
    cd: float
    cl_ci: float = 0.0
    cd_ci: float = 0.0


class Polar:
    """Monotone-alpha lookup with linear interpolation."""

    def __init__(self, points: list[PolarPoint], source: str):
        self.pts = sorted(points, key=lambda p: p.alpha)
        self._a = [p.alpha for p in self.pts]
        self.source = source

    def _interp(self, alpha: float, attr: str) -> float:
        a = self._a
        if alpha <= a[0]:
            return getattr(self.pts[0], attr)
        if alpha >= a[-1]:
            return getattr(self.pts[-1], attr)
        i = bisect.bisect_left(a, alpha)
        lo, hi = self.pts[i - 1], self.pts[i]
        t = (alpha - lo.alpha) / (hi.alpha - lo.alpha)
        return getattr(lo, attr) + t * (getattr(hi, attr) - getattr(lo, attr))

    def cl(self, alpha: float) -> float:
        return self._interp(alpha, "cl")

    def cd(self, alpha: float) -> float:
        return self._interp(alpha, "cd")

    @property
    def cl_max(self) -> float:
        return max(p.cl for p in self.pts)

    @property
    def alpha_cl_max(self) -> float:
        return max(self.pts, key=lambda p: p.cl).alpha

    @property
    def ld_max(self) -> float:
        return max(p.cl / p.cd for p in self.pts if p.cd > 0)

    def alpha_for_cl(self, target_cl: float) -> float | None:
        """Lowest alpha on the front side reaching target_cl."""
        for lo, hi in zip(self.pts, self.pts[1:]):
            if lo.cl <= target_cl <= hi.cl and hi.cl > lo.cl:
                t = (target_cl - lo.cl) / (hi.cl - lo.cl)
                return lo.alpha + t * (hi.alpha - lo.alpha)
        return None


def _load_csv(path: Path) -> Polar:
    pts = []
    with open(path) as f:
        for row in csv.DictReader(f):
            if abs(float(row["beta"])) > 1e-6:
                continue
            pts.append(
                PolarPoint(
                    alpha=float(row["alpha"]),
                    cl=float(row["CL"]),
                    cd=float(row["CD"]),
                    cl_ci=float(row.get("CL_ci", 0) or 0),
                    cd_ci=float(row.get("CD_ci", 0) or 0),
                )
            )
    return Polar(pts, source=path.stem)


@lru_cache(maxsize=None)
def wind_tunnel() -> Polar:
    return _load_csv(DATA_DIR / "WindTunnel_Re5e5_alpha_sweep_beta_0_Poland2025.csv")


@lru_cache(maxsize=None)
def cfd_re1e6() -> Polar:
    return _load_csv(
        DATA_DIR / "CFD_RANS_Re1e6_alpha_sweep_beta_0_Vire2022_CorrectedByPoland2025.csv"
    )


def bridle_cd(n_bridle_lines: int, mean_line_length: float, wing_area: float) -> float:
    """Parasitic CD contribution of the bridle line array, referenced to wing area.

    CD_bridle = CD_cyl · (frontal line area) / A_wing.
    Frontal area ≈ n · L_mean · d (lines seen broadside, worst case ×0.5 avg).
    """
    frontal = 0.5 * n_bridle_lines * mean_line_length * V3_BRIDLE_DIAM
    return CD_CYLINDER * frontal / wing_area


@dataclass
class SystemPolar:
    """Wing polar + bridle parasitic drag → what a real tethered kite sees."""
    wing: Polar
    cd_bridle: float

    def cl(self, alpha: float) -> float:
        return self.wing.cl(alpha)

    def cd(self, alpha: float) -> float:
        return self.wing.cd(alpha) + self.cd_bridle

    def ld(self, alpha: float) -> float:
        c = self.cd(alpha)
        return self.wing.cl(alpha) / c if c else 0.0

    @property
    def ld_max(self) -> float:
        return max(
            self.wing.cl(p.alpha) / self.cd(p.alpha)
            for p in self.wing.pts
            if self.cd(p.alpha) > 0
        )


def system_polar_for(wing_area: float, n_bridle_lines: int = 82,
                     bridle_total_length: float = 96.0,
                     base: str = "wind_tunnel") -> SystemPolar:
    """Build a system polar. Bridle count/length scale from V3 by area^0.5."""
    scale = (wing_area / V3_PROJECTED_AREA) ** 0.5
    n = max(1, round(n_bridle_lines * scale))
    mean_len = (bridle_total_length / n_bridle_lines) * scale
    wing = wind_tunnel() if base == "wind_tunnel" else cfd_re1e6()
    return SystemPolar(wing=wing, cd_bridle=bridle_cd(n, mean_len, wing_area))


def calibrated_operating_point(wing_area: float, target_cl: float = 0.8
                               ) -> tuple[float, float, float]:
    """Return (alpha, cl, cd_system) at a traction operating CL, from the benchmark."""
    sp = system_polar_for(wing_area)
    alpha = sp.wing.alpha_for_cl(target_cl) or sp.wing.alpha_cl_max
    return alpha, sp.cl(alpha), sp.cd(alpha)
