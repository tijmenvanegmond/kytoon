"""L1 3D dynamics — linearised 12-state modes of the three-line rig.

Stage 4: validate the lateral model *before* porting anything to
GDScript. Two independent checks, both cheap:

  * **Planar reduction of the dynamics.** At β = 0 with no roll, the 3D
    linearisation must return `l1_trim`'s longitudinal modes — the same
    pitch and pendulum eigenvalues. Statics already reduce (Stage 3);
    this closes the loop on inertia and damping.
  * **The roll mode as a probe of added inertia.** Roll added inertia is
    the least-trusted number in Stage 1 (11.9× structural), and it is
    directly observable: the rig's roll-on-the-bridle frequency is
    √(k_roll / (I_roll + A_roll)). Including added inertia moves the
    period from ~0.24 s to ~0.86 s — a 3.6× discriminator, not a
    subtle correction.

Everything new here is the lateral half: sway, roll and yaw modes that
the longitudinal model has no representation for.

States are world-frame perturbations about a 3D trim:
[δx, δy, δz, δφ, δθ, δψ, and their rates]. Linear only — Coriolis and
gyroscopic terms vanish to first order about a trim at rest, which is
also why Euler angles are safe here. The time-domain port must use
quaternions.

Requires the `l1` extra.
CLI: python -m kytoon.solvers.l1_dyn3d specs/mk5_manta.yaml
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from kytoon.solvers.l0 import RHO_AIR
from kytoon.solvers.l1_lat_aero import solve as solve_lat
from kytoon.solvers.l1_mass3d import (
    MassProps3D, attach_point_3d, mass_props_3d,
)
from kytoon.solvers.l1_rig3d import (
    MAIN, PORT, STBD, RigWrench, _check, aero_wrench, line_wrench,
    rest_lengths, rotation, trim3d, stiffness, wind_angles,
)
from kytoon.solvers.l1_trim import (
    CM_Q, FAIRLEAD_HEIGHT, G, HAS_L1, _require, aero_table,
)
from kytoon.spec import KytoonSpec

C_LINE = 2000.0                          # N per m/s, as in the planar model


# ---------------------------------------------------------------------------
def mass_matrix(props: MassProps3D, rot: np.ndarray) -> np.ndarray:
    """6×6 generalised mass in WORLD axes: structural + added.

    The body-frame block is [[m·I + A_tt, A_tr], [A_trᵀ, I + A_rr]];
    rotating it keeps it consistent with the world-frame perturbation
    states.
    """
    m = np.zeros((6, 6))
    m[:3, :3] = props.m_total * np.eye(3)
    m[3:, 3:] = props.inertia
    m = m + props.added
    t = np.zeros((6, 6))
    t[:3, :3] = rot
    t[3:, 3:] = rot
    return t @ m @ t.T


def rate_wrench(spec: KytoonSpec, lat, rot: np.ndarray, speed: float,
                omega_world: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Aero damping from body rates (p, q, r), world-frame output.

    Pitch uses `l1_trim`'s Cm_q so the planar reduction is exact; roll
    and yaw use the Stage 2 strip-theory derivatives.
    """
    if speed < 1e-6:
        return np.zeros(3), np.zeros(3)
    s_ref = spec.canopy.area
    b_ref = spec.fat_wing.span
    c_ref = s_ref / spec.canopy.span
    q_dyn = 0.5 * RHO_AIR * speed * speed
    p, q, r = rot.T @ omega_world
    p_hat = p * b_ref / (2 * speed)
    q_hat = q * c_ref / (2 * speed)
    r_hat = r * b_ref / (2 * speed)
    m_body = np.array([
        q_dyn * s_ref * b_ref * (lat.cl_p * p_hat + lat.cl_r * r_hat),
        q_dyn * s_ref * c_ref * (CM_Q * q_hat),
        q_dyn * s_ref * b_ref * (lat.cn_p * p_hat + lat.cn_r * r_hat),
    ])
    return np.zeros(3), rot @ m_body


def line_damping_wrench(spec: KytoonSpec, props: MassProps3D,
                        pos: np.ndarray, rot: np.ndarray,
                        vel: np.ndarray, omega: np.ndarray,
                        pod: np.ndarray, dihedral_deg: float = 0.0
                        ) -> tuple[np.ndarray, np.ndarray]:
    """Structural damping along each taut line, world frame."""
    anchor = np.array([0.0, 0.0, FAIRLEAD_HEIGHT])
    cg = pos + rot @ props.r_cg
    force = np.zeros(3)
    moment = np.zeros(3)
    targets = [(spec.bridle.positions[MAIN], anchor),
               (spec.bridle.positions[PORT], pod),
               (spec.bridle.positions[STBD], pod)]
    for span_pos, far in targets:
        att = pos + rot @ attach_point_3d(spec, span_pos, dihedral_deg)
        d = far - att
        n = float(np.linalg.norm(d))
        if n < 1e-9:
            continue
        u = d / n
        v_att = vel + np.cross(omega, att - cg)
        # u points from the attach toward the far end, so v·u > 0 means
        # the line is SHORTENING and tension must drop — the damping
        # force is −c·(v·u)·u. (The planar model writes the same thing as
        # `T = k·stretch − c_line·(v·uv)`; getting this backwards makes
        # the damper a driver and every mode goes unstable.)
        f = -C_LINE * float(v_att @ u) * u
        force = force + f
        moment = moment + np.cross(att - cg, f)
    return force, moment


def net_wrench_dyn(spec: KytoonSpec, props: MassProps3D, table, lat,
                   pos: np.ndarray, rpy: np.ndarray, vel: np.ndarray,
                   omega: np.ndarray, l0: np.ndarray,
                   wind_world: np.ndarray,
                   dihedral_deg: float = 0.0) -> RigWrench:
    """Static wrench plus the velocity-dependent terms."""
    rot = rotation(rpy)
    v_rel = wind_world - vel                    # air relative to the kite
    f_a, m_a, alpha, beta = aero_wrench(spec, props, table, lat, rot, v_rel)
    f_l, m_l, tensions, pod = line_wrench(spec, props, pos, rot, l0,
                                          dihedral_deg)
    speed = float(np.linalg.norm(v_rel))
    f_r, m_r = rate_wrench(spec, lat, rot, speed, omega)
    f_d, m_d = line_damping_wrench(spec, props, pos, rot, vel, omega, pod,
                                   dihedral_deg)
    f_w = np.array([0.0, 0.0, -props.m_total * G])
    f_b = np.array([0.0, 0.0, props.buoyancy_n])
    m_b = np.cross(rot @ (props.r_cb - props.r_cg), f_b)
    return RigWrench(force=f_a + f_l + f_w + f_b + f_r + f_d,
                     moment=m_a + m_l + m_b + m_r + m_d,
                     tensions=tensions, pod=pod,
                     alpha_deg=alpha, beta_deg=beta)


def damping_matrix(spec: KytoonSpec, props: MassProps3D, table, lat,
                   x: np.ndarray, l0: np.ndarray, wind_world: np.ndarray,
                   dihedral_deg: float = 0.0) -> np.ndarray:
    """6×6 ∂(force, moment)/∂(velocity, angular velocity) at a trim."""
    c = np.zeros((6, 6))
    step = 1e-4
    for k in range(6):
        d = np.zeros(6)
        d[k] = step
        wp = net_wrench_dyn(spec, props, table, lat, x[:3], x[3:], d[:3],
                            d[3:], l0, wind_world, dihedral_deg)
        wm = net_wrench_dyn(spec, props, table, lat, x[:3], x[3:], -d[:3],
                            -d[3:], l0, wind_world, dihedral_deg)
        c[:, k] = np.concatenate([wp.force - wm.force,
                                  wp.moment - wm.moment]) / (2 * step)
    return c


# ---------------------------------------------------------------------------
@dataclass
class Mode:
    eig: complex
    kind: str                            # dominant state
    participation: np.ndarray

    @property
    def period_s(self) -> float:
        return (2 * math.pi / abs(self.eig.imag)
                if abs(self.eig.imag) > 1e-9 else math.inf)

    @property
    def damped(self) -> bool:
        return self.eig.real < 0.0


LABELS = ["surge", "sway", "heave", "roll", "pitch", "yaw"]


def linearise(spec: KytoonSpec, props: MassProps3D, table, lat,
              x: np.ndarray, l0: np.ndarray, wind_world: np.ndarray,
              dihedral_deg: float = 0.0) -> np.ndarray:
    """12×12 state matrix about a 3D trim."""
    k = stiffness(spec, props, table, lat, x, l0, wind_world, dihedral_deg)
    c = damping_matrix(spec, props, table, lat, x, l0, wind_world,
                       dihedral_deg)
    m = mass_matrix(props, rotation(x[3:]))
    minv = np.linalg.inv(m)
    a = np.zeros((12, 12))
    a[:6, 6:] = np.eye(6)
    a[6:, :6] = minv @ k
    a[6:, 6:] = minv @ c
    return a


def modes(a: np.ndarray, angle_scale: float = 16.0) -> list[Mode]:
    """Eigenmodes, classified by scaled participation.

    Translations are metres and rotations radians, so comparing them raw
    labels every mode a translation. Scaling angles by a reference arm
    (half-span by default) compares *tip motion* against body motion,
    which is what "this is a roll mode" actually means.
    """
    scale = np.array([1.0, 1.0, 1.0, angle_scale, angle_scale, angle_scale])
    vals, vecs = np.linalg.eig(a)
    out = []
    for i, ev in enumerate(vals):
        part = np.abs(vecs[:6, i]) * scale
        total = part.sum()
        part = part / total if total > 0 else part
        out.append(Mode(eig=complex(ev), kind=LABELS[int(np.argmax(part))],
                        participation=part))
    return sorted(out, key=lambda m: -m.eig.real)


# ---------------------------------------------------------------------------
@dataclass
class L1Dyn3DReport:
    spec: KytoonSpec
    wind: float
    modes: list[Mode]
    k_roll: float
    i_roll_total: float
    i_roll_struct: float
    roll_freq_closed_form: float         # rad/s

    @property
    def roll_freq_no_added(self) -> float:
        """What the roll mode would be if added inertia were ignored —
        the discriminator that makes this a real test of Stage 1."""
        return math.sqrt(max(self.k_roll, 0.0) / self.i_roll_struct)
    flags: list[str] = field(default_factory=list)

    def of_kind(self, kind: str) -> Mode | None:
        cand = [m for m in self.modes if m.kind == kind]
        return max(cand, key=lambda m: abs(m.eig.imag)) if cand else None

    def lateral(self) -> list[Mode]:
        return [m for m in self.modes
                if m.participation[[1, 3, 5]].sum() > 0.5]

    def longitudinal(self) -> list[Mode]:
        return [m for m in self.modes
                if m.participation[[0, 2, 4]].sum() > 0.5]

    @property
    def all_damped(self) -> bool:
        return all(m.eig.real < 1e-6 for m in self.modes)

    @property
    def max_real(self) -> float:
        return max(m.eig.real for m in self.modes)

    @property
    def max_real_lateral(self) -> float:
        ms = self.lateral()
        return max(m.eig.real for m in ms) if ms else -math.inf

    @property
    def max_real_longitudinal(self) -> float:
        ms = self.longitudinal()
        return max(m.eig.real for m in ms) if ms else -math.inf

    @property
    def sway_damping(self) -> float:
        """C[1,1]: positive means the air feeds the sway, not damps it."""
        return self._c11

    _c11: float = 0.0


def solve(spec: KytoonSpec, wind: float = 12.0,
          dihedral_deg: float = 0.0) -> L1Dyn3DReport:
    _require()
    _check(spec)
    props = mass_props_3d(spec, dihedral_deg=dihedral_deg)
    table = aero_table(spec)
    lat = solve_lat(spec, dihedral_deg=dihedral_deg)
    wind_world = np.array([wind, 0.0, 0.0])
    l0 = rest_lengths(spec, props, table, lat, wind_world,
                      dihedral_deg=dihedral_deg)
    x, ok = trim3d(spec, props, table, lat, l0, wind_world,
                   dihedral_deg=dihedral_deg)
    a = linearise(spec, props, table, lat, x, l0, wind_world, dihedral_deg)
    ms = modes(a, angle_scale=spec.fat_wing.span / 2)

    k = stiffness(spec, props, table, lat, x, l0, wind_world, dihedral_deg)
    c = damping_matrix(spec, props, table, lat, x, l0, wind_world,
                       dihedral_deg)
    k_roll = -k[3, 3]
    i_roll = props.i_roll + props.added_roll

    flags = list(lat.flags)
    if not ok:
        flags.append("trim did not converge — modes are about the last "
                     "iterate")
    flags.append("linear about a trim at rest: no Coriolis/gyroscopic "
                 "terms, so Euler angles are safe HERE only")

    rep = L1Dyn3DReport(
        spec=spec, wind=wind, modes=ms, k_roll=k_roll,
        i_roll_total=i_roll, i_roll_struct=props.i_roll,
        roll_freq_closed_form=math.sqrt(max(k_roll, 0.0) / i_roll),
        flags=flags)
    rep._c11 = float(c[1, 1])
    if rep.max_real_lateral > 1e-6:
        rep.flags.insert(0, (
            "LATERAL DIVERGENCE at %+.2f /s (doubling in %.1f s). Two "
            "drivers: CY_β > 0 feeds the sway (C[1,1] = %+.0f N per m/s, "
            "i.e. %.2f /s on its own), and control-line geometry couples "
            "roll into sway and yaw. Longitudinal is unaffected (%+.2f /s)."
            % (rep.max_real_lateral,
               math.log(2) / rep.max_real_lateral,
               rep._c11, max(rep._c11, 0.0) / props.m_total,
               rep.max_real_longitudinal)))
    return rep


def _summary(rep: L1Dyn3DReport) -> str:
    lines = [
        f"## {rep.spec.name} — L1 3D modes ({rep.wind:.0f} m/s)",
        "",
        "    eigenvalue            period   dominant   damped",
    ]
    for m in rep.modes:
        if m.eig.imag < -1e-9:
            continue                      # one of each conjugate pair
        per = (f"{m.period_s:6.2f} s" if math.isfinite(m.period_s)
               else "     —  ")
        lines.append("  %+8.3f %+8.3fj   %s   %-7s   %s"
                     % (m.eig.real, m.eig.imag, per, m.kind,
                        "yes" if m.damped else "NO"))
    roll = rep.of_kind("roll")
    lines += [
        "",
        "roll mode as a probe of added inertia:",
        f"    k_roll {rep.k_roll/1e3:.0f} kN·m/rad, "
        f"I_roll+A_roll {rep.i_roll_total/1e3:.0f} t·m²",
        f"    closed form √(k/I) = {rep.roll_freq_closed_form:.2f} rad/s "
        f"({2 * math.pi / rep.roll_freq_closed_form:.2f} s)",
    ]
    if roll is not None:
        lines.append(f"    eigen              = {abs(roll.eig.imag):.2f} "
                     f"rad/s ({roll.period_s:.2f} s)")
    lines.append(
        f"    ignoring added inertia would give "
        f"{rep.roll_freq_no_added:.2f} rad/s "
        f"({2 * math.pi / rep.roll_freq_no_added:.2f} s) — "
        f"{rep.roll_freq_no_added / rep.roll_freq_closed_form:.1f}× off, "
        "so this mode really does discriminate")
    for f in rep.flags:
        lines.append(f"- ⚠ {f}")
    return "\n".join(lines)


if __name__ == "__main__":
    import argparse
    import sys

    from kytoon.spec import load_spec

    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description="L1 3D modes for one spec")
    ap.add_argument("spec", help="path to a specs/*.yaml file")
    ap.add_argument("--wind", type=float, default=12.0)
    ap.add_argument("--dihedral", type=float, default=0.0)
    args = ap.parse_args()
    print(_summary(solve(load_spec(args.spec), args.wind, args.dihedral)))
