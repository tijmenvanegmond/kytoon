"""L1 trim & pitch stability — 3-line fat-wing kytoons (Mk V «Manta»).

Answers the question L0 asserts and cannot check: does Mk V trim at a
useful angle of attack on its main + 2-control-line rig, is that trim
stable, and what does the control winchlet need (tension, travel)?

Model (longitudinal plane, all closed-form except the eigencheck):

  * Aero: AeroBuildup alpha sweep (CL, CD, **Cm**) on the loft's own
    planform — symmetric NACA-4 sections at t/c, taper, 15° sweep,
    matching `kytoon.geometry._lofted_fatwing`. Semi-empirical: drag is
    a LOWER bound (elevation therefore an upper bound), and t/c 0.28 is
    outside the method's comfort zone — bounds, not certification.
  * Mass properties from the actual loft mesh: buoyancy at the volume
    centroid, skin mass at the shell centroid, pod+rigging at the main
    attach; pitch inertia from the shell; strip-theory added mass.
  * Lines: main (spec.tether) at the center bridle station, the two
    control lines (spec.bridle.control_mbl_kn each) lumped at the
    outboard stations' chordwise offset. All attach on the lower surface
    at spec.bridle.chord_fraction of local chord. Both run to one
    fairlead `FAIRLEAD_HEIGHT` above water, `tether.length` away — at
    that distance the lines are near-parallel, which makes the taut-taut
    trim closed-form:

      force closure   T_vec = -(F_aero + W + B)   → total tension, elevation
      moment closure  M_main + T_ctl · arm = 0    → control tension

    Feasible iff both tensions positive and inside their WLLs.
  * Stability: eigenvalues of the linearized 6-state (x, z, θ, u, w, q)
    taut-taut dynamics at trim. A single slow drift mode (|Re| < 0.1/s)
    is acceptable — that is the winchlet's slow loop; everything faster
    must be damped.

A single-confluence bridle version of Mk V is passively UNSTABLE at any
useful alpha (buoyancy centroid sits ~2 m aft of any stable pull point) —
that finding is what makes the 3-line rig load-bearing rather than
optional. See KYTOON-PROJECT.md §6 (2026-07-24).

Requires the `l1` extra (aerosandbox + trimesh).
CLI: python -m kytoon.solvers.l1_trim specs/mk5_manta.yaml
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from kytoon.solvers.l0 import NET_LIFT_PER_M3, RHO_AIR
from kytoon.spec import Archetype, KytoonSpec

try:
    import aerosandbox as asb
    from kytoon.geometry import HAS_TRIMESH, _lofted_fatwing
    HAS_L1 = HAS_TRIMESH
except ImportError:  # default install is L0-only by design
    HAS_L1 = False

G = 9.81
SWEEP_DEG = 15.0          # quarter-chord sweep — must match kytoon.geometry
CD_LINES = 0.02           # bridle/control-line parasitic drag (l1 ballpark)
CM_Q = -1.5               # pitch-rate damping estimate (per rad, q·c/2V)
DYNEEMA_STRAIN_MBL = 0.035  # line strain at MBL → EA = MBL / strain
FAIRLEAD_HEIGHT = 5.0     # winch fairlead above waterline [m]
ALPHAS = np.arange(-8.0, 25.0, 1.0)
ALPHA_MAX_CMD = 14.0      # commanded-alpha ceiling (gust/stall margin)
TENSION_FRAC = 0.75       # schedule depowers to hold T_tot ≤ this × WLL

_TABLE_CACHE: dict[tuple, tuple] = {}


def _require() -> None:
    if not HAS_L1:
        raise ImportError(
            "aerosandbox/trimesh not installed — l1_trim needs the l1 "
            'extra: pip install "kytoon-sim[l1]"'
        )


def _check(spec: KytoonSpec) -> None:
    if spec.archetype != Archetype.FATWING:
        raise ValueError(
            f"{spec.name}: l1_trim models the 3-line fat-wing rig only "
            f"(got {spec.archetype.value})"
        )
    if len(spec.bridle.positions) != 3:
        raise ValueError(
            f"{spec.name}: 3-line trim needs exactly 3 bridle positions "
            f"(control / main / control), got {spec.bridle.positions}"
        )


# ---------------------------------------------------------------------------
# aero table

def aero_table(spec: KytoonSpec) -> tuple[np.ndarray, ...]:
    """(alphas, CL, CD, Cm) via AeroBuildup on the loft's planform."""
    _require()
    fw = spec.fat_wing
    key = (fw.span, fw.chord, fw.taper, fw.thickness_ratio)
    if key in _TABLE_CACHE:
        return _TABLE_CACHE[key]

    naca = f"naca00{round(fw.thickness_ratio * 100):02d}"
    af = asb.Airfoil(naca)
    xsecs = []
    for eta in (0.0, 0.25, 0.5, 0.75, 1.0):
        y = eta * fw.span / 2
        c = fw.chord_at(eta)
        xsecs.append(asb.WingXSec(
            xyz_le=[math.tan(math.radians(SWEEP_DEG)) * y - 0.25 * c, y, 0.0],
            chord=c, airfoil=af))
    s_ref = spec.canopy.area
    plane = asb.Airplane(
        name=spec.name,
        wings=[asb.Wing(name="fatwing", symmetric=True, xsecs=xsecs)],
        s_ref=s_ref, c_ref=s_ref / fw.span, b_ref=fw.span)

    rows = []
    for a in ALPHAS:
        out = asb.AeroBuildup(
            airplane=plane,
            op_point=asb.OperatingPoint(velocity=12.0, alpha=float(a)),
        ).run()
        rows.append([float(np.ravel(out[k])[0]) for k in ("CL", "CD", "Cm")])
    cl, cd, cm = np.asarray(rows).T
    _TABLE_CACHE[key] = (ALPHAS.copy(), cl, cd + CD_LINES, cm)
    return _TABLE_CACHE[key]


def _coeffs(table, a_deg: float) -> tuple[float, float, float]:
    al, cl, cd, cm = table
    a = float(np.clip(a_deg, al[0], al[-1]))
    return (float(np.interp(a, al, cl)), float(np.interp(a, al, cd)),
            float(np.interp(a, al, cm)))


# ---------------------------------------------------------------------------
# mass properties & rig geometry

@dataclass
class MassProps:
    m_total: float            # airframe mass (skin + pod), tether excluded
    r_cg: np.ndarray          # (x, z) body frame
    r_cb: np.ndarray
    r_skin: np.ndarray
    m_skin: float
    m_pod: float
    buoyancy_n: float
    i_yy: float
    m_added_x: float
    m_added_z: float
    i_added: float


def _naca_halfz(xb: float, tr: float, c: float) -> float:
    return 5 * tr * c * (0.2969 * math.sqrt(xb) - 0.1260 * xb
                         - 0.3516 * xb**2 + 0.2843 * xb**3 - 0.1036 * xb**4)


def attach_point(spec: KytoonSpec, span_pos: float) -> np.ndarray:
    """Body-frame (x, z) of a lower-surface attach at a spanwise station."""
    fw = spec.fat_wing
    f = spec.bridle.chord_fraction
    y_frac = abs(2 * span_pos - 1.0)
    c = fw.chord_at(y_frac)
    y = (span_pos - 0.5) * fw.span
    x_le = math.tan(math.radians(SWEEP_DEG)) * abs(y) - 0.25 * c
    return np.array([x_le + f * c,
                     -_naca_halfz(f, fw.thickness_ratio, c)])


def mass_props(spec: KytoonSpec) -> MassProps:
    _require()
    fw = spec.fat_wing
    mesh = _lofted_fatwing(fw)
    w_f = mesh.area_faces
    r_cb = mesh.center_mass[[0, 2]]
    r_skin = ((mesh.triangles_center * w_f[:, None]).sum(0) / w_f.sum())[[0, 2]]
    m_skin = fw.mass + spec.canopy.mass
    m_pod = spec.payload_mass + spec.rigging_mass
    p_main = attach_point(spec, spec.bridle.positions[1])
    r_cg = (m_skin * r_skin + m_pod * p_main) / (m_skin + m_pod)

    d2 = ((mesh.triangles_center[:, [0, 2]] - r_cg) ** 2).sum(1)
    i_yy = (m_skin / w_f.sum()) * (w_f * d2).sum() \
        + m_pod * ((p_main - r_cg) ** 2).sum()

    ys = np.linspace(-fw.span / 2, fw.span / 2, 201)
    cs = np.array([fw.chord_at(abs(2 * y / fw.span)) for y in ys])
    m_az = float(np.trapezoid(math.pi * RHO_AIR * cs**2 / 4, ys))
    xqc = np.tan(np.radians(SWEEP_DEG)) * np.abs(ys)
    i_az = float(np.trapezoid(
        math.pi * RHO_AIR * cs**2 / 4
        * ((xqc - r_cg[0])**2 + cs**2 / 32), ys))

    return MassProps(
        m_total=m_skin + m_pod, r_cg=r_cg, r_cb=r_cb, r_skin=r_skin,
        m_skin=m_skin, m_pod=m_pod,
        buoyancy_n=NET_LIFT_PER_M3 * mesh.volume * G,
        i_yy=i_yy, m_added_x=0.2 * RHO_AIR * mesh.volume,
        m_added_z=m_az, i_added=i_az)


def _rotm(th: float) -> np.ndarray:
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, s], [-s, c]])


def _cross2(r: np.ndarray, f: np.ndarray) -> float:
    """Pitch-up-positive moment: (r × F)_y with x downwind, z up."""
    return r[1] * f[0] - r[0] * f[1]


# ---------------------------------------------------------------------------
# closed-form taut-taut trim

@dataclass
class TrimPoint:
    wind: float
    alpha_deg: float
    cl: float
    t_total_n: float
    t_main_n: float
    t_ctl_n: float            # lumped pair
    elevation_deg: float
    dl0_ctl_m: float          # control rest length relative to main
    feasible: bool


def trim_point(spec: KytoonSpec, props: MassProps, table,
               wind: float, alpha_deg: float) -> TrimPoint:
    cl, cd, cm = _coeffs(table, alpha_deg)
    S = spec.canopy.area
    c_ref = S / spec.canopy.span
    q = 0.5 * RHO_AIR * wind**2
    f_aero = q * S * np.array([cd, cl])
    f_wb = np.array([0.0, -props.m_total * G + props.buoyancy_n])
    t_vec = -(f_aero + f_wb)
    t_tot = float(np.linalg.norm(t_vec))
    u_hat = t_vec / t_tot                     # kite → anchor
    elev = math.degrees(math.atan2(-u_hat[1], -u_hat[0]))

    R = _rotm(math.radians(alpha_deg))
    p_main = R @ attach_point(spec, spec.bridle.positions[1])
    p_ctl = R @ attach_point(spec, spec.bridle.positions[2])
    m = q * S * c_ref * cm
    m += _cross2(-p_main, f_aero)             # aero acts at body origin
    m += _cross2(R @ props.r_skin - p_main,
                 np.array([0.0, -props.m_skin * G]))
    # pod sits at the main attach → no arm
    m += _cross2(R @ props.r_cb - p_main,
                 np.array([0.0, props.buoyancy_n]))
    arm = _cross2(p_ctl - p_main, u_hat)
    t_ctl = -m / arm
    t_main = t_tot - t_ctl

    k_main = (spec.tether.mbl_kn * 1e3 / DYNEEMA_STRAIN_MBL) \
        / spec.tether.length
    k_ctl = (2 * spec.bridle.control_mbl_kn * 1e3 / DYNEEMA_STRAIN_MBL) \
        / spec.tether.length
    delta_geom = float((p_ctl - p_main) @ (-u_hat))
    dl0 = delta_geom - (t_ctl / k_ctl - t_main / k_main)

    ctl_wll = 2 * spec.bridle.control_mbl_kn * 1e3 / spec.tether.safety_factor
    feasible = (t_ctl > 300.0 and t_main > 300.0 and t_ctl < ctl_wll
                and t_tot < spec.tether.wll_n)
    return TrimPoint(wind, alpha_deg, cl, t_tot, t_main, t_ctl,
                     elev, dl0, feasible)


def alpha_band(spec: KytoonSpec, props: MassProps, table,
               wind: float) -> tuple[float, float] | None:
    """Feasible commanded-alpha range at a wind speed."""
    ok = [a for a in np.arange(-2.0, 18.1, 0.25)
          if trim_point(spec, props, table, wind, float(a)).feasible]
    return (min(ok), max(ok)) if ok else None


def schedule(spec: KytoonSpec, props: MassProps, table,
             winds: tuple[float, ...] = (4, 6, 8, 10, 12, 14, 16, 18, 20, 23),
             ) -> list[TrimPoint]:
    """Max-lift trim per wind, depowered to hold TENSION_FRAC × WLL."""
    out = []
    for u in winds:
        for a in np.arange(ALPHA_MAX_CMD, -2.1, -0.25):
            tp = trim_point(spec, props, table, u, float(a))
            if tp.feasible and tp.t_total_n <= TENSION_FRAC * spec.tether.wll_n:
                out.append(tp)
                break
    return out


# ---------------------------------------------------------------------------
# eigen-stability of the taut-taut rig at a trim point

def _derivs(spec, props, table, s, l0m, l0c, wind, k_main, k_ctl,
            c_line=2000.0):
    S = spec.canopy.area
    c_ref = S / spec.canopy.span
    anchor = np.array([0.0, FAIRLEAD_HEIGHT])
    x, z, th, u, w, om = s
    R = _rotm(th)
    wa = np.array([wind - u, -w])
    va = max(float(np.linalg.norm(wa)), 1e-6)
    alpha = math.degrees(th + math.atan2(wa[1], wa[0]))
    cl, cd, cm = _coeffs(table, alpha)
    q = 0.5 * RHO_AIR * va**2
    dhat = wa / va
    lhat = np.array([-dhat[1], dhat[0]])
    F = q * S * (cd * dhat + cl * lhat)
    M = q * S * c_ref * (cm + CM_Q * om * c_ref / (2 * va))
    p_main_b = attach_point(spec, spec.bridle.positions[1])
    for pb, fv in ((props.r_skin, np.array([0.0, -props.m_skin * G])),
                   (p_main_b, np.array([0.0, -props.m_pod * G])),
                   (props.r_cb, np.array([0.0, props.buoyancy_n]))):
        rw = R @ pb
        F = F + fv
        M += _cross2(rw, fv)
    for pb, l0, k in ((p_main_b, l0m, k_main),
                      (attach_point(spec, spec.bridle.positions[2]),
                       l0c, k_ctl)):
        rw = R @ pb
        r_att = np.array([x, z]) + rw
        v_att = np.array([u, w]) + om * np.array([rw[1], -rw[0]])
        d = anchor - r_att
        dist = float(np.linalg.norm(d))
        uv = d / dist
        stretch = dist - l0
        if stretch > 0:
            T = max(k * stretch - c_line * float(v_att @ uv), 0.0)
            F = F + T * uv
            M += _cross2(rw, T * uv)
    rcg = R @ props.r_cg
    m_cg = M - _cross2(rcg, F)
    return np.array([u, w, om,
                     F[0] / (props.m_total + props.m_added_x),
                     F[1] / (props.m_total + props.m_added_z),
                     m_cg / (props.i_yy + props.i_added)])


def eigenvalues(spec: KytoonSpec, props: MassProps, table,
                tp: TrimPoint) -> np.ndarray:
    """Eigenvalues of the linearized taut-taut dynamics at a trim point."""
    anchor = np.array([0.0, FAIRLEAD_HEIGHT])
    a = math.radians(tp.alpha_deg)
    u_hat = np.array([math.cos(math.radians(tp.elevation_deg)),
                      math.sin(math.radians(tp.elevation_deg))])
    r_main_w = anchor + u_hat * spec.tether.length
    R = _rotm(a)
    x0 = r_main_w - R @ attach_point(spec, spec.bridle.positions[1])
    k_main = (spec.tether.mbl_kn * 1e3 / DYNEEMA_STRAIN_MBL) \
        / spec.tether.length
    k_ctl = (2 * spec.bridle.control_mbl_kn * 1e3 / DYNEEMA_STRAIN_MBL) \
        / spec.tether.length
    dm = float(np.linalg.norm(
        anchor - (x0 + R @ attach_point(spec, spec.bridle.positions[1]))))
    dc = float(np.linalg.norm(
        anchor - (x0 + R @ attach_point(spec, spec.bridle.positions[2]))))
    l0m = dm - tp.t_main_n / k_main
    l0c = dc - tp.t_ctl_n / k_ctl
    s0 = np.array([x0[0], x0[1], a, 0.0, 0.0, 0.0])
    J = np.zeros((6, 6))
    for j in range(6):
        dp = np.zeros(6)
        dp[j] = 1e-4
        fp = _derivs(spec, props, table, s0 + dp, l0m, l0c, tp.wind,
                     k_main, k_ctl)
        fm = _derivs(spec, props, table, s0 - dp, l0m, l0c, tp.wind,
                     k_main, k_ctl)
        J[:, j] = (fp - fm) / 2e-4
    return np.linalg.eigvals(J)


# ---------------------------------------------------------------------------
@dataclass
class L1TrimReport:
    spec: KytoonSpec
    props: MassProps
    op: TrimPoint                      # at cl_op, 12 m/s
    bands: dict[float, tuple[float, float] | None]
    mission: list[TrimPoint]
    eigs: np.ndarray                   # at the 12 m/s mission trim
    flags: list[str] = field(default_factory=list)

    @property
    def winchlet_travel_m(self) -> float:
        dl = [tp.dl0_ctl_m for tp in self.mission]
        return max(dl) - min(dl) if dl else math.nan

    @property
    def max_fast_re(self) -> float:
        """Most-unstable eigenvalue excluding the slow drift mode(s)."""
        fast = [e.real for e in self.eigs
                if abs(e.real) >= 0.1 or abs(e.imag) > 1e-6]
        return max(fast) if fast else 0.0

    @property
    def max_slow_re(self) -> float:
        slow = [e.real for e in self.eigs
                if abs(e.real) < 0.1 and abs(e.imag) <= 1e-6]
        return max(slow) if slow else 0.0


def solve(spec: KytoonSpec) -> L1TrimReport:
    _require()
    _check(spec)
    table = aero_table(spec)
    props = mass_props(spec)

    al, cl_t, _, _ = table
    alpha_op = float(np.interp(spec.canopy.cl_op, cl_t, al))
    op = trim_point(spec, props, table, 12.0, alpha_op)
    bands = {u: alpha_band(spec, props, table, u) for u in (6.0, 12.0, 20.0)}
    mission = schedule(spec, props, table)
    tp12 = next((tp for tp in mission if tp.wind == 12), op)
    eigs = eigenvalues(spec, props, table, tp12)

    flags = [
        "AeroBuildup drag is a LOWER bound → elevation is an upper bound, "
        "horizontal tow a lower bound",
        f"Cm from semi-empirical buildup at t/c "
        f"{spec.fat_wing.thickness_ratio} — bounds, not certification",
        "longitudinal plane only; lateral/roll modes not analyzed",
        "single-confluence bridle is passively UNSTABLE at useful alpha "
        "— the 3-line rig is load-bearing, not optional",
    ]
    if op.elevation_deg > 75:
        flags.append(
            f"trim elevation {op.elevation_deg:.0f}° — near-vertical line; "
            "horizontal tow is a small fraction of line tension "
            "(feeds the §6 'elevation is an output' conversation)")
    return L1TrimReport(spec, props, op, bands, mission, eigs, flags)


# ---------------------------------------------------------------------------
def _summary(rep: L1TrimReport) -> str:
    s = rep.spec
    lines = [
        f"## {s.name} — L1 3-line trim & pitch stability",
        "",
        f"- cl_op {s.canopy.cl_op} → α ≈ {rep.op.alpha_deg:.1f}°; at 12 m/s: "
        f"T {rep.op.t_total_n/1e3:.1f} kN (main {rep.op.t_main_n/1e3:.1f} / "
        f"ctl {rep.op.t_ctl_n/1e3:.1f}), elevation "
        f"{rep.op.elevation_deg:.0f}°"
        + ("" if rep.op.feasible else "  [INFEASIBLE]"),
    ]
    for u, band in rep.bands.items():
        txt = (f"[{band[0]:.1f}, {band[1]:.1f}]°" if band else "none")
        lines.append(f"- steerable α at {u:.0f} m/s: {txt}")
    lines.append("- mission schedule "
                 f"(α ≤ {ALPHA_MAX_CMD:.0f}°, T ≤ {TENSION_FRAC:.0%} WLL):")
    for tp in rep.mission:
        lines.append(
            f"    {tp.wind:4.0f} m/s  α {tp.alpha_deg:5.2f}°  CL {tp.cl:.2f}"
            f"  T {tp.t_total_n/1e3:5.1f} kN  elev {tp.elevation_deg:.0f}°"
            f"  ctl {tp.t_ctl_n/1e3:.1f} kN  ΔL0 {tp.dl0_ctl_m:+.2f} m")
    lines.append(f"- winchlet: travel {rep.winchlet_travel_m:.1f} m across "
                 "the schedule, tension "
                 f"≤ {max(tp.t_ctl_n for tp in rep.mission)/1e3:.1f} kN")
    lines.append(f"- eigenvalues @12 m/s trim: max Re (fast modes) "
                 f"{rep.max_fast_re:+.2f} /s, slow drift "
                 f"{rep.max_slow_re:+.3f} /s")
    for f in rep.flags:
        lines.append(f"- ⚠ {f}")
    return "\n".join(lines)


if __name__ == "__main__":
    import argparse
    import sys

    from kytoon.spec import load_spec

    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description="L1 3-line trim for one spec")
    ap.add_argument("spec", help="path to a specs/*.yaml file")
    args = ap.parse_args()
    print(_summary(solve(load_spec(args.spec))))
