"""L1 3D time domain — 6-DOF rigid body on the three-line rig.

Stage 5. Stage 4 linearised about a trim and read off the modes; this
integrates the nonlinear equations, which is what crosswind manoeuvring
needs (a figure-eight is not a small perturbation about anything).

Two things make this more than "Stage 4 with a time loop":

  * **Quaternion attitude.** The recovery endgame already reaches θ ≈
    −57° and keeps rotating; Euler angles hit gimbal lock at −90° and
    would spend an evening looking like broken physics.
  * **Kirchhoff form, not Newton–Euler.** Added inertia is ~12×
    structural in roll and added mass ~10× the airframe in heave, so the
    velocity-dependent coupling those terms produce is a leading effect,
    not a correction. The equations are the standard rigid-body +
    added-mass form used for underwater vehicles (Fossen), which is the
    right analogy: a buoyant body whose surrounding fluid outweighs it.

        (M_rb + M_a)·ν̇ + C(ν)·ν = τ

    with ν = [u v w p q r] in body axes and C built from the SAME 6×6 M,
    so the added mass appears in the Coriolis terms too. Dropping C is
    what makes a big light wing spin up wrongly under combined roll+yaw.

Forces come from Stage 3/4 unchanged — same aero table, same lateral
derivatives, same three elastic lines and pod — so this reduces to their
answers and is gated on doing so: released at the Stage 3 trim it must
stay there, and a small kick must ring at the Stage 4 eigenvalues.

Requires the `l1` extra.
CLI: python -m kytoon.solvers.l1_sim3d specs/mk5_manta.yaml
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from kytoon.solvers.l1_dyn3d import (
    line_damping_wrench, mass_matrix, rate_wrench,
)
from kytoon.solvers.l1_lat_aero import solve as solve_lat
from kytoon.solvers.l1_mass3d import MassProps3D, mass_props_3d
from kytoon.solvers.l1_rig3d import (
    _check, aero_wrench, line_wrench, rest_lengths, rotation, trim3d,
)
from kytoon.solvers.l1_trim import G, HAS_L1, _require, aero_table
from kytoon.spec import KytoonSpec

# state layout
POS = slice(0, 3)
QUAT = slice(3, 7)                       # (w, x, y, z)
VEL = slice(7, 10)                       # body axes
OMG = slice(10, 13)                      # body axes
N_STATE = 13


def skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    """Body→world rotation from a (w, x, y, z) quaternion."""
    w, x, y, z = q / max(float(np.linalg.norm(q)), 1e-12)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(r: np.ndarray) -> np.ndarray:
    tr = float(np.trace(r))
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return np.array([0.25 * s, (r[2, 1] - r[1, 2]) / s,
                         (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s])
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k]) * 2
    q = np.zeros(4)
    q[0] = (r[k, j] - r[j, k]) / s
    q[1 + i] = 0.25 * s
    q[1 + j] = (r[j, i] + r[i, j]) / s
    q[1 + k] = (r[k, i] + r[i, k]) / s
    return q


def quat_rates(q: np.ndarray, omega_body: np.ndarray) -> np.ndarray:
    """q̇ = ½ q ⊗ (0, ω), plus a gentle pull back onto the unit sphere so
    integration drift cannot silently rescale the attitude."""
    w, x, y, z = q
    p, qq, r = omega_body
    dq = 0.5 * np.array([
        -x * p - y * qq - z * r,
        w * p + y * r - z * qq,
        w * qq - x * r + z * p,
        w * r + x * qq - y * p,
    ])
    return dq + 1.0 * (1.0 - float(q @ q)) * q


def coriolis(m6: np.ndarray, nu: np.ndarray) -> np.ndarray:
    """C(ν) for a symmetric 6×6 mass matrix (rigid body + added mass).

    Standard result: the cross terms are built from the *momenta*
    M·ν, so added mass enters here exactly as structural mass does.
    """
    lin = m6[:3, :3] @ nu[:3] + m6[:3, 3:] @ nu[3:]      # linear momentum
    ang = m6[3:, :3] @ nu[:3] + m6[3:, 3:] @ nu[3:]      # angular momentum
    c = np.zeros((6, 6))
    c[:3, 3:] = -skew(lin)
    c[3:, :3] = -skew(lin)
    c[3:, 3:] = -skew(ang)
    return c


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def linearise_nonlinear(spec: KytoonSpec, props: MassProps3D, table, lat,
                        state: np.ndarray, l0: np.ndarray,
                        wind_world: np.ndarray, dihedral_deg: float,
                        eps: float = 1e-6) -> np.ndarray:
    """12×12 state matrix by differencing the NONLINEAR model.

    Coordinates are [δpos_world, δrotvec_body, δv_body, δω_body]. The
    attitude chart is a body-frame rotation vector applied as q ⊗ δq, so
    d(rotvec)/dt = ω_body *exactly* to first order.

    That is the point of doing it this way. `l1_dyn3d.linearise` builds
    its attitude block as Euler angles and then sets d(rpy)/dt = ω, which
    is only true at zero attitude — at this rig's 12.5° trim it mixes
    roll and yaw by sin θ ≈ 0.22 and corrupts precisely the slow lateral
    modes. Translation was never affected (the two agree to 0.2 % on a
    pure position nudge), so this replaces the attitude half only.
    """
    rot0 = quat_to_matrix(state[QUAT])
    m6 = mass_matrix(props, np.eye(3))

    def perturbed(k: int, h: float) -> np.ndarray:
        s = state.copy()
        if k < 3:
            s[POS] = s[POS] + h * np.eye(3)[k]
        elif k < 6:
            dv = h * np.eye(3)[k - 3]
            dq = np.concatenate([[1.0], 0.5 * dv])
            s[QUAT] = quat_mul(s[QUAT], dq)
            s[QUAT] /= np.linalg.norm(s[QUAT])
        elif k < 9:
            s[VEL] = s[VEL] + h * np.eye(3)[k - 6]
        else:
            s[OMG] = s[OMG] + h * np.eye(3)[k - 9]
        return s

    def reduced(s: np.ndarray) -> np.ndarray:
        d, *_ = derivatives(spec, props, table, lat, s, l0, wind_world,
                            dihedral_deg, m6)
        rot = quat_to_matrix(s[QUAT])
        # d(rotvec)/dt in the body chart is just omega_body
        return np.concatenate([d[POS], s[OMG], d[VEL], d[OMG]])

    a = np.zeros((12, 12))
    for k in range(12):
        a[:, k] = (reduced(perturbed(k, eps))
                   - reduced(perturbed(k, -eps))) / (2 * eps)
    return a


# ---------------------------------------------------------------------------
@dataclass
class SimSample:
    t: float
    pos: np.ndarray
    rpy_deg: np.ndarray
    alpha_deg: float
    beta_deg: float
    tensions: np.ndarray
    speed: float


def _rpy_from_matrix(r: np.ndarray) -> np.ndarray:
    """(φ, θ, ψ) for the 3-2-1 sequence `l1_rig3d.rotation` builds."""
    theta = math.asin(float(np.clip(r[0, 2], -1.0, 1.0)))
    phi = math.atan2(-r[1, 2], r[2, 2])
    psi = math.atan2(-r[0, 1], r[0, 0])
    return np.degrees([phi, theta, psi])


def wrench_world(spec: KytoonSpec, props: MassProps3D, table, lat,
                 pos: np.ndarray, rot: np.ndarray, vel_world: np.ndarray,
                 omega_world: np.ndarray, l0: np.ndarray,
                 wind_world: np.ndarray, dihedral_deg: float
                 ) -> tuple[np.ndarray, np.ndarray, np.ndarray, float, float]:
    """Total external wrench about the CG, world axes — the Stage 3/4
    force model verbatim."""
    v_rel = wind_world - vel_world
    f_a, m_a, alpha, beta = aero_wrench(spec, props, table, lat, rot, v_rel)
    f_l, m_l, tensions, pod = line_wrench(spec, props, pos, rot, l0,
                                          dihedral_deg)
    f_r, m_r = rate_wrench(spec, lat, rot, float(np.linalg.norm(v_rel)),
                           omega_world)
    f_d, m_d = line_damping_wrench(spec, props, pos, rot, vel_world,
                                   omega_world, pod, dihedral_deg)
    f_w = np.array([0.0, 0.0, -props.m_total * G])
    f_b = np.array([0.0, 0.0, props.buoyancy_n])
    m_b = np.cross(rot @ (props.r_cb - props.r_cg), f_b)
    return (f_a + f_l + f_w + f_b + f_r + f_d,
            m_a + m_l + m_b + m_r + m_d, tensions, alpha, beta)


def derivatives(spec: KytoonSpec, props: MassProps3D, table, lat,
                state: np.ndarray, l0: np.ndarray, wind_world: np.ndarray,
                dihedral_deg: float, m6_body: np.ndarray
                ) -> tuple[np.ndarray, np.ndarray, float, float]:
    q = state[QUAT]
    rot = quat_to_matrix(q)
    nu = np.concatenate([state[VEL], state[OMG]])
    vel_world = rot @ state[VEL]
    omega_world = rot @ state[OMG]

    f_w, m_w, tensions, alpha, beta = wrench_world(
        spec, props, table, lat, state[POS], rot, vel_world, omega_world,
        l0, wind_world, dihedral_deg)
    tau = np.concatenate([rot.T @ f_w, rot.T @ m_w])

    nu_dot = np.linalg.solve(m6_body, tau - coriolis(m6_body, nu) @ nu)

    d = np.zeros(N_STATE)
    d[POS] = vel_world
    d[QUAT] = quat_rates(q, state[OMG])
    d[VEL] = nu_dot[:3]
    d[OMG] = nu_dot[3:]
    return d, tensions, alpha, beta


# ---------------------------------------------------------------------------
def state_from_trim(spec: KytoonSpec, x_trim: np.ndarray) -> np.ndarray:
    """13-state at rest from Stage 3's (position, attitude) solution."""
    s = np.zeros(N_STATE)
    s[POS] = x_trim[:3]
    s[QUAT] = matrix_to_quat(rotation(x_trim[3:]))
    return s


def integrate(spec: KytoonSpec, props: MassProps3D, table, lat,
              state: np.ndarray, l0: np.ndarray, wind_world: np.ndarray,
              t_end: float, dt: float = 0.005,
              dihedral_deg: float = 0.0, record_every: float = 0.1,
              control=None) -> list[SimSample]:
    """RK4. `control(t, state) -> l0` may retrim the drums each step."""
    m6 = mass_matrix(props, np.eye(3))        # body axes: rot = I
    out: list[SimSample] = []
    n = int(round(t_end / dt))
    rec = 0.0
    for i in range(n):
        t = i * dt
        if control is not None:
            l0 = control(t, state)

        def f(st):
            return derivatives(spec, props, table, lat, st, l0, wind_world,
                               dihedral_deg, m6)

        k1, tensions, alpha, beta = f(state)
        k2, *_ = f(state + dt / 2 * k1)
        k3, *_ = f(state + dt / 2 * k2)
        k4, *_ = f(state + dt * k3)
        if t >= rec:
            rot = quat_to_matrix(state[QUAT])
            out.append(SimSample(
                t=t, pos=state[POS].copy(), rpy_deg=_rpy_from_matrix(rot),
                alpha_deg=alpha, beta_deg=beta, tensions=tensions.copy(),
                speed=float(np.linalg.norm(state[VEL]))))
            rec += record_every
        state = state + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        state[QUAT] /= max(float(np.linalg.norm(state[QUAT])), 1e-12)
    return out


# ---------------------------------------------------------------------------
@dataclass
class L1Sim3DReport:
    spec: KytoonSpec
    wind: float
    hold: list[SimSample]                # released at trim
    kick: list[SimSample]                # small roll kick
    flags: list[str] = field(default_factory=list)

    @property
    def hold_drift_deg(self) -> float:
        """Largest attitude excursion when released at the trim."""
        a = np.array([s.rpy_deg for s in self.hold])
        return float(np.abs(a - a[0]).max())

    @property
    def hold_drift_m(self) -> float:
        p = np.array([s.pos for s in self.hold])
        return float(np.linalg.norm(p - p[0], axis=1).max())

    def roll_ring_hz(self) -> float:
        """Frequency of the roll ring-down after the kick, by zero
        crossings — compared against Stage 4's eigenvalue."""
        t = np.array([s.t for s in self.kick])
        phi = np.array([s.rpy_deg[0] for s in self.kick])
        phi = phi - phi[-1]
        sign = np.sign(phi)
        crossings = np.where(np.diff(sign) != 0)[0]
        if len(crossings) < 2:
            return float("nan")
        periods = np.diff(t[crossings]) * 2
        return float(1.0 / np.mean(periods))


def solve(spec: KytoonSpec, wind: float = 12.0, dihedral_deg: float | None
          = None, t_hold: float = 30.0, t_kick: float = 12.0
          ) -> L1Sim3DReport:
    _require()
    _check(spec)
    if dihedral_deg is None:
        dihedral_deg = spec.fat_wing.dihedral_deg
    props = mass_props_3d(spec, dihedral_deg=dihedral_deg)
    table = aero_table(spec)
    lat = solve_lat(spec, dihedral_deg=dihedral_deg,
                    fin_area_m2=spec.fin.area if spec.fin else 0.0)
    wind_world = np.array([wind, 0.0, 0.0])
    l0 = rest_lengths(spec, props, table, lat, wind_world,
                      dihedral_deg=dihedral_deg)
    x, ok = trim3d(spec, props, table, lat, l0, wind_world,
                   dihedral_deg=dihedral_deg)

    s0 = state_from_trim(spec, x)
    hold = integrate(spec, props, table, lat, s0.copy(), l0, wind_world,
                     t_hold, dihedral_deg=dihedral_deg)

    kicked = s0.copy()
    kicked[OMG] = np.array([0.05, 0.0, 0.0])      # small roll rate
    kick = integrate(spec, props, table, lat, kicked, l0, wind_world,
                     t_kick, dihedral_deg=dihedral_deg, record_every=0.02)

    flags = list(lat.flags)
    flags.append("Kirchhoff form with the full 6×6: added mass appears in "
                 "the Coriolis terms, not just the inertia")
    if not ok:
        flags.append("trim did not converge — the hold case starts off "
                     "equilibrium")
    return L1Sim3DReport(spec=spec, wind=wind, hold=hold, kick=kick,
                         flags=flags)


def _summary(rep: L1Sim3DReport) -> str:
    h = rep.hold[-1]
    lines = [
        f"## {rep.spec.name} — L1 3D time domain ({rep.wind:.0f} m/s)",
        "",
        f"released at the Stage 3 trim, {rep.hold[-1].t:.0f} s:",
        f"    attitude drift {rep.hold_drift_deg:.2f}°   "
        f"position drift {rep.hold_drift_m:.2f} m",
        f"    end: roll {h.rpy_deg[0]:+.2f}  pitch {h.rpy_deg[1]:+.2f}  "
        f"yaw {h.rpy_deg[2]:+.2f}   α {h.alpha_deg:.2f}  β {h.beta_deg:+.2f}",
        f"    tensions main {h.tensions[1]/1e3:.1f} kN, "
        f"port {h.tensions[0]/1e3:.2f}, stbd {h.tensions[2]/1e3:.2f}",
        "",
        f"roll kick (0.05 rad/s): ring-down at "
        f"{rep.roll_ring_hz():.2f} Hz "
        f"({2 * math.pi * rep.roll_ring_hz():.2f} rad/s)",
    ]
    for f in rep.flags:
        lines.append(f"- ⚠ {f}")
    return "\n".join(lines)


if __name__ == "__main__":
    import argparse
    import sys

    from kytoon.spec import load_spec

    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description="L1 3D time domain for a spec")
    ap.add_argument("spec")
    ap.add_argument("--wind", type=float, default=12.0)
    args = ap.parse_args()
    print(_summary(solve(load_spec(args.spec), args.wind)))
