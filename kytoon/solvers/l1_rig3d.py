"""L1 3D rig — force closure on the main line plus two control lines.

Stage 3 of the lateral model. The longitudinal solver computes
`ctl_span_offset()` and then throws it away, lumping the control pair
into one in-plane force. Here the two lines separate: each runs from the
winchlet pod to its own attachment at ±0.38 span, and equilibrium is a
6-DOF problem (3 force + 3 moment closures) rather than 3.

Two things fall out that nothing before could see:

  * **Yaw stiffness.** Stage 2 found the tailless swept planform is
    weathercock-UNSTABLE (Cn_β ≈ −0.005/rad). Nothing aerodynamic
    restores heading, so it has to be the bridle: two lines from a
    common pod to two separated attachments form a V that resists yaw.
    This module measures whether that is enough.
  * **What differential trim actually produces.** Not a roll moment in
    isolation — a new equilibrium, bank and heading and sideslip
    together, with the tether reacting all of it.

Frames: body x downstream, y starboard, z up — the same axes AeroSandbox
uses for the geometry it is handed, so α/β and the lateral coefficients
transfer without a convention change (gated). World is x downwind, z up.
Attitude is a 3-2-1 (yaw, pitch, roll) sequence; statics only, so Euler
is safe here — the time-domain model must use quaternions.

Requires the `l1` extra.
CLI: python -m kytoon.solvers.l1_rig3d specs/mk5_manta.yaml
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import fsolve

from kytoon.solvers.l0 import RHO_AIR
from kytoon.solvers.l1_lat_aero import solve as solve_lat
from kytoon.solvers.l1_mass3d import (
    MassProps3D, attach_point_3d, mass_props_3d,
)
from kytoon.solvers.l1_trim import (
    DYNEEMA_STRAIN_MBL, FAIRLEAD_HEIGHT, G, HAS_L1, _coeffs, _require,
    aero_table,
)
from kytoon.spec import Archetype, KytoonSpec

PORT, MAIN, STBD = 0, 1, 2               # spec.bridle.positions ordering


def _check(spec: KytoonSpec) -> None:
    if spec.archetype != Archetype.FATWING:
        raise ValueError(
            f"{spec.name}: the 3-line rig model is for the lofted fat "
            f"wing only (got {spec.archetype.value})")
    if len(spec.bridle.positions) != 3:
        raise ValueError(f"{spec.name}: need exactly 3 bridle positions")


def rotation(rpy: np.ndarray) -> np.ndarray:
    """Body→world for a 3-2-1 (yaw ψ, pitch θ, roll φ) sequence.

    All three are right-handed about the body axes, so a restoring
    stiffness is uniformly −∂M/∂angle. Consequences of x-aft/z-up:
    +θ lifts the leading edge (matching the planar model) and +φ raises
    the STARBOARD wing (the opposite of the aviation convention, which
    is defined in an x-forward frame).
    """
    phi, th, psi = rpy
    rx = np.array([[1, 0, 0],
                   [0, math.cos(phi), -math.sin(phi)],
                   [0, math.sin(phi), math.cos(phi)]])
    ry = np.array([[math.cos(th), 0, math.sin(th)],
                   [0, 1, 0],
                   [-math.sin(th), 0, math.cos(th)]])
    rz = np.array([[math.cos(psi), -math.sin(psi), 0],
                   [math.sin(psi), math.cos(psi), 0],
                   [0, 0, 1]])
    return rz @ ry @ rx


def wind_angles(v_body: np.ndarray) -> tuple[float, float]:
    """(α, β) in radians from the relative WIND (air velocity) in body
    axes — the same vector the planar model uses.

    α matches AeroSandbox exactly (verified against its own freestream
    direction, gated). β is the negative of AeroSandbox's: at β_asb =
    +5° its freestream carries a −y component, so β here comes out −5°.
    `aero_wrench` carries the flips; nothing else should.
    """
    speed = float(np.linalg.norm(v_body))
    if speed < 1e-9:
        return 0.0, 0.0
    return (math.atan2(v_body[2], v_body[0]),
            math.asin(float(np.clip(v_body[1] / speed, -1.0, 1.0))))


# ---------------------------------------------------------------------------
@dataclass
class RigWrench:
    force: np.ndarray                    # world
    moment: np.ndarray                   # world, about the CG
    tensions: np.ndarray                 # [port, main, stbd]
    pod: np.ndarray                      # world
    alpha_deg: float
    beta_deg: float


def aero_wrench(spec: KytoonSpec, props: MassProps3D, table, lat,
                rot: np.ndarray, wind_world: np.ndarray
                ) -> tuple[np.ndarray, np.ndarray, float, float]:
    """Aero force (world) and moment about the CG (world), statics.

    Longitudinal coefficients come from `l1_trim`'s table; lateral ones
    are the Stage 2 derivatives, linear in β. Force acts at the body
    origin, as in the planar model.
    """
    v_body = rot.T @ wind_world
    alpha, beta = wind_angles(v_body)
    speed = float(np.linalg.norm(v_body))
    q = 0.5 * RHO_AIR * speed * speed
    s_ref = spec.canopy.area
    c_ref = s_ref / spec.canopy.span
    b_ref = spec.fat_wing.span

    cl, cd, cm = _coeffs(table, math.degrees(alpha))
    d_hat = v_body / max(speed, 1e-9)
    z_b = np.array([0.0, 0.0, 1.0])
    l_vec = z_b - float(z_b @ d_hat) * d_hat
    l_hat = l_vec / max(float(np.linalg.norm(l_vec)), 1e-9)
    y_hat = -np.cross(d_hat, l_hat)      # +y at β = 0

    # Axis bookkeeping, verified against AeroSandbox (gated):
    #   β_asb = −β here, and AeroBuildup reports Cl/Cn in flight-dynamics
    #   body axes (x forward, z down) while this frame is x aft, z up, so
    #   both moment coefficients also flip. The two flips CANCEL for the
    #   β-derivatives: Cl_β and Cn_β apply directly to β here. CY is a
    #   force and y does not flip, so it takes the single β flip.
    f_body = q * s_ref * (cd * d_hat + cl * l_hat
                          - lat.cy_beta * beta * y_hat)
    m_body = np.array([q * s_ref * b_ref * lat.cl_beta * beta,
                       q * s_ref * c_ref * cm,
                       q * s_ref * b_ref * lat.cn_beta * beta])
    # aero acts at the body origin; shift to the CG
    m_body = m_body + np.cross(-props.r_cg, f_body)
    return (rot @ f_body, rot @ m_body,
            math.degrees(alpha), math.degrees(beta))


def line_wrench(spec: KytoonSpec, props: MassProps3D, pos: np.ndarray,
                rot: np.ndarray, l0: np.ndarray, dihedral_deg: float = 0.0
                ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Main line to the fairlead plus two control lines to the pod.

    Tension-only springs, k = EA/L. The pod rides the main line at the
    spec's standoff below the kite, so control-line geometry follows the
    main line's direction — the whole point of the pod rig.
    """
    anchor = np.array([0.0, 0.0, FAIRLEAD_HEIGHT])
    ea_main = spec.tether.mbl_kn * 1e3 / DYNEEMA_STRAIN_MBL
    ea_ctl = spec.bridle.control_mbl_kn * 1e3 / DYNEEMA_STRAIN_MBL

    p_main = pos + rot @ attach_point_3d(spec, spec.bridle.positions[MAIN],
                                         dihedral_deg)
    to_anchor = anchor - p_main
    dist = float(np.linalg.norm(to_anchor))
    u_main = to_anchor / max(dist, 1e-9)
    k_main = ea_main / max(l0[MAIN], 1.0)
    t_main = max(k_main * (dist - l0[MAIN]), 0.0)

    standoff = min(spec.bridle.pod_standoff_m or 0.0, 0.9 * dist)
    pod = p_main + standoff * u_main

    force = t_main * u_main
    moment = np.cross(p_main - (pos + rot @ props.r_cg), force)
    tensions = np.zeros(3)
    tensions[MAIN] = t_main

    for side in (PORT, STBD):
        att = pos + rot @ attach_point_3d(spec, spec.bridle.positions[side],
                                          dihedral_deg)
        d = pod - att
        length = float(np.linalg.norm(d))
        k_ctl = ea_ctl / max(l0[side], 1.0)
        t = max(k_ctl * (length - l0[side]), 0.0)
        f = t * d / max(length, 1e-9)
        force = force + f
        moment = moment + np.cross(att - (pos + rot @ props.r_cg), f)
        tensions[side] = t
    return force, moment, tensions, pod


def net_wrench(spec: KytoonSpec, props: MassProps3D, table, lat,
               pos: np.ndarray, rpy: np.ndarray, l0: np.ndarray,
               wind_world: np.ndarray, dihedral_deg: float = 0.0
               ) -> RigWrench:
    rot = rotation(rpy)
    f_a, m_a, alpha, beta = aero_wrench(spec, props, table, lat, rot,
                                        wind_world)
    f_l, m_l, tensions, pod = line_wrench(spec, props, pos, rot, l0,
                                          dihedral_deg)
    # weight at the CG (no moment), buoyancy at the CB
    f_w = np.array([0.0, 0.0, -props.m_total * G])
    f_b = np.array([0.0, 0.0, props.buoyancy_n])
    m_b = np.cross(rot @ (props.r_cb - props.r_cg), f_b)
    return RigWrench(force=f_a + f_l + f_w + f_b,
                     moment=m_a + m_l + m_b,
                     tensions=tensions, pod=pod,
                     alpha_deg=alpha, beta_deg=beta)


# ---------------------------------------------------------------------------
def trim3d(spec: KytoonSpec, props: MassProps3D, table, lat,
           l0: np.ndarray, wind_world: np.ndarray,
           guess: np.ndarray | None = None,
           dihedral_deg: float = 0.0) -> tuple[np.ndarray, bool]:
    """Solve the 6 closure equations for (position, attitude)."""
    if guess is None:
        elev = math.radians(80.0)
        r = spec.tether.length
        guess = np.array([r * math.cos(elev), 0.0,
                          FAIRLEAD_HEIGHT + r * math.sin(elev),
                          0.0, math.radians(12.0), 0.0])

    # Position is O(100 m) and attitude O(0.1 rad), line springs are tens
    # of kN/m against aero forces of a few kN — without scaling both the
    # unknowns and the residuals, the solver stalls on the asymmetric
    # cases and silently returns its guess.
    scale = np.array([50.0, 50.0, 50.0, 0.1, 0.1, 0.1])

    def residual(xs):
        x = xs * scale
        w = net_wrench(spec, props, table, lat, x[:3], x[3:], l0,
                       wind_world, dihedral_deg)
        return np.concatenate([w.force / 1e3, w.moment / 1e4])

    sol, _, ok, _ = fsolve(residual, guess / scale, full_output=True,
                           xtol=1e-10, maxfev=4000)
    return sol * scale, ok == 1


def stiffness(spec: KytoonSpec, props: MassProps3D, table, lat,
              x: np.ndarray, l0: np.ndarray, wind_world: np.ndarray,
              dihedral_deg: float = 0.0) -> np.ndarray:
    """6×6 ∂(force, moment)/∂(position, attitude) about a pose."""
    j = np.zeros((6, 6))
    for k in range(6):
        step = 1e-4 if k < 3 else 1e-5
        d = np.zeros(6)
        d[k] = step
        wp = net_wrench(spec, props, table, lat, (x + d)[:3], (x + d)[3:],
                        l0, wind_world, dihedral_deg)
        wm = net_wrench(spec, props, table, lat, (x - d)[:3], (x - d)[3:],
                        l0, wind_world, dihedral_deg)
        j[:, k] = np.concatenate([wp.force - wm.force,
                                  wp.moment - wm.moment]) / (2 * step)
    return j


def rest_lengths(spec: KytoonSpec, props: MassProps3D, table, lat,
                 wind_world: np.ndarray, alpha_target_deg: float = 14.0,
                 dihedral_deg: float = 0.0) -> np.ndarray:
    """Symmetric rest lengths that put the rig near a target α.

    Built geometrically from a nominal pose rather than solved: the trim
    solve then finds the true equilibrium nearby.
    """
    anchor = np.array([0.0, 0.0, FAIRLEAD_HEIGHT])
    elev = math.radians(80.0)
    pos = np.array([spec.tether.length * math.cos(elev), 0.0,
                    FAIRLEAD_HEIGHT + spec.tether.length * math.sin(elev)])
    rot = rotation(np.array([0.0, math.radians(alpha_target_deg), 0.0]))
    p_main = pos + rot @ attach_point_3d(spec, spec.bridle.positions[MAIN],
                                         dihedral_deg)
    dist = float(np.linalg.norm(anchor - p_main))
    u = (anchor - p_main) / dist
    pod = p_main + min(spec.bridle.pod_standoff_m or 0.0, 0.9 * dist) * u
    l0 = np.zeros(3)
    l0[MAIN] = dist * (1 - 0.02)          # ~2 % strain -> working tension
    for side in (PORT, STBD):
        att = pos + rot @ attach_point_3d(
            spec, spec.bridle.positions[side], dihedral_deg)
        l0[side] = float(np.linalg.norm(pod - att)) * (1 - 0.004)
    return l0


# ---------------------------------------------------------------------------
@dataclass
class L1Rig3DReport:
    spec: KytoonSpec
    wind: float
    pose: np.ndarray
    tensions: np.ndarray
    alpha_deg: float
    beta_deg: float
    k_yaw: float                         # N·m per rad, restoring positive
    k_roll: float
    k_yaw_aero: float                    # the wing alone, for contrast
    steer: list[dict] = field(default_factory=list)
    flags: list[str] = field(default_factory=list)

    @property
    def yaw_stable(self) -> bool:
        return self.k_yaw > 0.0

    @property
    def k_yaw_bridle(self) -> float:
        """The bridle's own contribution: k_yaw is the net of bridle and
        wing, and the wing's part is negative."""
        return self.k_yaw - self.k_yaw_aero

    @property
    def bridle_yaw_margin(self) -> float:
        """How many times the wing's own yaw divergence the bridle
        overcomes. Below 1 the rig is directionally unstable."""
        return (self.k_yaw_bridle / abs(self.k_yaw_aero)
                if self.k_yaw_aero != 0 else math.inf)


def solve(spec: KytoonSpec, wind: float = 12.0,
          dihedral_deg: float | None = None,
          fin_area_m2: float | None = None) -> L1Rig3DReport:
    """Γ and fin default to the spec; pass 0.0 for the bare reference —
    flat loft, no fin, which is the configuration the yaw and lateral
    findings were made on."""
    _require()
    _check(spec)
    if dihedral_deg is None:
        dihedral_deg = spec.fat_wing.dihedral_deg
    if fin_area_m2 is None:
        fin_area_m2 = spec.fin.area if spec.fin else 0.0
    props = mass_props_3d(spec, dihedral_deg=dihedral_deg)
    table = aero_table(spec)
    lat = solve_lat(spec, dihedral_deg=dihedral_deg,
                    fin_area_m2=fin_area_m2)
    wind_world = np.array([wind, 0.0, 0.0])

    l0 = rest_lengths(spec, props, table, lat, wind_world,
                      dihedral_deg=dihedral_deg)
    x, ok = trim3d(spec, props, table, lat, l0, wind_world,
                   dihedral_deg=dihedral_deg)
    w = net_wrench(spec, props, table, lat, x[:3], x[3:], l0, wind_world,
                   dihedral_deg)
    k = stiffness(spec, props, table, lat, x, l0, wind_world, dihedral_deg)

    # Yaw stiffness of the wing alone. Yawing by +ψ gives β = −ψ, so
    # K = −∂M_z/∂ψ = +q·S·b·Cn_β — negative for this planform, i.e.
    # the wing on its own diverges in yaw.
    q = 0.5 * RHO_AIR * wind * wind
    k_yaw_aero = q * spec.canopy.area * spec.fat_wing.span * lat.cn_beta

    flags = list(lat.flags)
    flags.append("statics only: Euler angles are safe here, the "
                 "time-domain model must use quaternions")
    if not ok:
        flags.append("trim solve did NOT converge — numbers below are the "
                     "solver's last iterate")

    rep = L1Rig3DReport(
        spec=spec, wind=wind, pose=x, tensions=w.tensions,
        alpha_deg=w.alpha_deg, beta_deg=w.beta_deg,
        k_yaw=-k[5, 5], k_roll=-k[3, 3], k_yaw_aero=k_yaw_aero, flags=flags)

    # differential trim: shorten one control line, lengthen the other
    for delta in (0.05, 0.15, 0.30):
        l0d = l0.copy()
        l0d[PORT] -= delta
        l0d[STBD] += delta
        xd, okd = trim3d(spec, props, table, lat, l0d, wind_world,
                         guess=x, dihedral_deg=dihedral_deg)
        wd = net_wrench(spec, props, table, lat, xd[:3], xd[3:], l0d,
                        wind_world, dihedral_deg)
        rep.steer.append({
            "delta_m": delta,
            "converged": okd,
            "roll_deg": math.degrees(xd[3]),
            "yaw_deg": math.degrees(xd[5]),
            "beta_deg": wd.beta_deg,
            "y_offset_m": xd[1] - x[1],
            "dT_kn": (wd.tensions[PORT] - wd.tensions[STBD]) / 1e3,
        })
    return rep


def _summary(rep: L1Rig3DReport) -> str:
    x = rep.pose
    lines = [
        f"## {rep.spec.name} — L1 3D rig closure ({rep.wind:.0f} m/s)",
        "",
        f"- trim: α {rep.alpha_deg:.1f}°  β {rep.beta_deg:+.2f}°  "
        f"roll {math.degrees(x[3]):+.2f}°  pitch {math.degrees(x[4]):.1f}°  "
        f"yaw {math.degrees(x[5]):+.2f}°",
        f"- position ({x[0]:.0f}, {x[1]:+.2f}, {x[2]:.0f}) m",
        f"- tensions: main {rep.tensions[MAIN]/1e3:.1f} kN, "
        f"port {rep.tensions[PORT]/1e3:.2f}, stbd {rep.tensions[STBD]/1e3:.2f}",
        "",
        "yaw stiffness — the question Stage 2 left open:",
        f"    wing alone      {rep.k_yaw_aero/1e3:+9.1f} kN·m/rad  "
        f"({'destabilising' if rep.k_yaw_aero < 0 else 'stabilising'})",
        f"    bridle alone    {rep.k_yaw_bridle/1e3:+9.1f} kN·m/rad",
        f"    net rig         {rep.k_yaw/1e3:+9.1f} kN·m/rad  "
        f"({'STABLE' if rep.yaw_stable else 'UNSTABLE'})",
        f"    bridle overcomes the wing by {rep.bridle_yaw_margin:.1f}×",
        f"    roll stiffness  {rep.k_roll/1e3:+9.1f} kN·m/rad",
        "",
        "differential trim (port shortened, starboard paid out):",
        "    ΔL     roll     yaw      β     side    ΔT",
    ]
    for s in rep.steer:
        lines.append(
            "  %5.2f m %+6.2f° %+6.2f° %+6.2f° %+6.1f m %+5.2f kN%s"
            % (s["delta_m"], s["roll_deg"], s["yaw_deg"], s["beta_deg"],
               s["y_offset_m"], s["dT_kn"],
               "" if s["converged"] else "  (no converge)"))
    for f in rep.flags:
        lines.append(f"- ⚠ {f}")
    return "\n".join(lines)


if __name__ == "__main__":
    import argparse
    import sys

    from kytoon.spec import load_spec

    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description="L1 3D rig closure for a spec")
    ap.add_argument("spec", help="path to a specs/*.yaml file")
    ap.add_argument("--wind", type=float, default=12.0)
    ap.add_argument("--dihedral", type=float, default=None,
                    help="override the spec's Γ [deg]; 0 = flat loft")
    ap.add_argument("--fin", type=float, default=None,
                    help="override the spec's fin area [m2]; 0 = finless")
    args = ap.parse_args()
    print(_summary(solve(load_spec(args.spec), args.wind, args.dihedral,
                         args.fin)))
