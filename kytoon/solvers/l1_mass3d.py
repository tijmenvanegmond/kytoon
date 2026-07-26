"""L1 mass properties in 3D — inertia tensor + 6×6 added-mass matrix.

Stage 1 of the lateral model. `l1_trim.mass_props` gives the planar terms
(m, r_cg, I_yy, and three added-mass scalars); the lateral question — can
differential winchlet trim on two lines at ±0.38 span roll the wing? —
needs the full tensors. This module computes them from the same loft mesh
and the same strip integral, and **reduces exactly** to the planar values
at Γ = 0 (gated in tests/test_l1_mass3d.py).

Two things dominate the lateral modes and are the reason this comes
first:

  * **Roll added inertia is huge.** The strip normal added mass πρc²/4
    picks up a y² lever in roll, so the air the wing must swing is an
    order of magnitude heavier than the wing. Pitch has no such lever
    (its arm is the chordwise offset, metres not tens of metres).
  * **Sideslip added mass is nearly nothing.** Edge-on, a wing displaces
    only its own thickness of air. Sway is ~t²/c² of heave.

Frames: body axes, x downstream, y starboard span, z up; origin at the
loft's own origin (centre quarter-chord), same as `l1_trim`. Inertia and
added mass are about/through the CG. Added-mass ordering is
[surge, sway, heave, roll, pitch, yaw].

Dihedral Γ (`dihedral_deg`, `fold_eta`) is carried from the start even
though everything currently runs at Γ = 0: folding the panels is a rigid
rotation, so it preserves volume but *moves the volume centroid* — and
the CB sitting aft of the pull point is precisely the term the
single-confluence instability finding rests on. Γ is a study parameter,
not spec design state.

Requires the `l1` extra.
CLI: python -m kytoon.solvers.l1_mass3d specs/mk5_manta.yaml
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from kytoon.solvers.l0 import NET_LIFT_PER_M3, RHO_AIR
from kytoon.solvers.l1_trim import (
    fold,
    G, SWEEP_DEG, HAS_L1, _naca_halfz, _require,
)
from kytoon.spec import Archetype, KytoonSpec

try:
    from kytoon.geometry import _lofted_fatwing
except ImportError:                      # default install is L0-only
    pass

N_STRIP = 401                            # spanwise strips for added mass
SURGE_VOLUME_FRACTION = 0.2              # l1_trim's surge convention


@dataclass
class MassProps3D:
    """Rigid-body mass properties of the airframe, body frame, about CG."""
    m_total: float
    m_skin: float
    m_pod: float
    buoyancy_n: float
    volume: float
    r_cg: np.ndarray                     # (3,)
    r_cb: np.ndarray                     # (3,) volume centroid
    r_skin: np.ndarray                   # (3,) shell (area) centroid
    inertia: np.ndarray                  # (3,3) about CG
    added: np.ndarray                    # (6,6) about CG
    dihedral_deg: float
    fold_eta: float
    surge_strip_kg: float                # strip-theory surge, for comparison

    @property
    def net_lift_kg(self) -> float:
        return self.buoyancy_n / G - self.m_total

    @property
    def i_roll(self) -> float:
        return float(self.inertia[0, 0])

    @property
    def i_pitch(self) -> float:
        return float(self.inertia[1, 1])

    @property
    def i_yaw(self) -> float:
        return float(self.inertia[2, 2])

    @property
    def added_roll(self) -> float:
        return float(self.added[3, 3])

    @property
    def added_sway(self) -> float:
        return float(self.added[1, 1])

    @property
    def added_heave(self) -> float:
        return float(self.added[2, 2])


def _check(spec: KytoonSpec) -> None:
    if spec.archetype != Archetype.FATWING:
        raise ValueError(
            f"{spec.name}: 3D mass properties are implemented for the "
            f"lofted fat wing only (got {spec.archetype.value})")


def attach_point_3d(spec: KytoonSpec, span_pos: float,
                    dihedral_deg: float | None = None,
                    fold_eta: float | None = None) -> np.ndarray:
    """Body-frame (x, y, z) of a lower-surface bridle attachment.

    The planar `l1_trim.attach_point` is this with the y dropped — both
    default to the SPEC's fold, so they cannot silently disagree (they
    did once: the Godot exporter shipped unfolded steering-line
    attachments while the mass properties were folded).
    """
    dihedral_deg, fold_eta = spec_fold(spec, dihedral_deg, fold_eta)
    fw = spec.fat_wing
    f = spec.bridle.chord_fraction
    y_frac = abs(2 * span_pos - 1.0)
    c = fw.chord_at(y_frac)
    y = (span_pos - 0.5) * fw.span
    x_le = math.tan(math.radians(SWEEP_DEG)) * abs(y) - 0.25 * c
    z = -_naca_halfz(f, fw.thickness_ratio, c)
    y_f, z_f = fold(y, z, math.radians(dihedral_deg),
                    fold_eta * fw.span / 2)
    return np.array([x_le + f * c, y_f, z_f])


def spec_fold(spec: KytoonSpec, dihedral_deg: float | None,
              fold_eta: float | None) -> tuple[float, float]:
    """Resolve the panel fold: explicit argument wins, otherwise the
    spec's. Callers that want the flat reference loft must pass 0.0
    explicitly — the spec is the design, not the baseline."""
    fw = spec.fat_wing
    return ((fw.dihedral_deg if dihedral_deg is None else dihedral_deg),
            (fw.fold_eta if fold_eta is None else fold_eta))


def mass_props_3d(spec: KytoonSpec, dihedral_deg: float | None = None,
                  fold_eta: float | None = None,
                  payload_kg: float | None = None) -> MassProps3D:
    """Γ defaults to the SPEC (so this agrees with l1_trim.mass_props);
    pass 0.0 explicitly for the flat reference loft."""
    _require()
    _check(spec)
    dihedral_deg, fold_eta = spec_fold(spec, dihedral_deg, fold_eta)
    fw = spec.fat_wing
    gamma = math.radians(dihedral_deg)
    y_break = fold_eta * fw.span / 2

    # --- structural: shell by area weight + pod point mass ---------------
    mesh = _lofted_fatwing(fw, dihedral_deg=dihedral_deg, fold_eta=fold_eta)
    w = mesh.area_faces
    r_skin = (mesh.triangles_center * w[:, None]).sum(0) / w.sum()
    m_skin = fw.mass + spec.canopy.mass
    payload = spec.payload_mass if payload_kg is None else payload_kg
    m_pod = payload + spec.rigging_mass
    m_total = m_skin + m_pod
    p_main = attach_point_3d(spec, spec.bridle.positions[1],
                             dihedral_deg, fold_eta)
    r_cg = (m_skin * r_skin + m_pod * p_main) / m_total

    pts = np.vstack([mesh.triangles_center, p_main])
    mas = np.concatenate([m_skin * w / w.sum(), [m_pod]])
    d = pts - r_cg
    inertia = np.zeros((3, 3))
    for i in range(3):
        for j in range(3):
            if i == j:
                o = [k for k in range(3) if k != i]
                inertia[i, j] = (mas * (d[:, o[0]] ** 2
                                        + d[:, o[1]] ** 2)).sum()
            else:
                inertia[i, j] = -(mas * d[:, i] * d[:, j]).sum()

    # --- added mass: strip theory on the folded planform -----------------
    # Per strip: normal (πρc²/4), chordwise (πρ(t/2)², small — this is the
    # "edge-on displaces almost nothing" term), and the section's own
    # rotational added inertia πρc⁴/128 about the spanwise axis.
    ys = np.linspace(-fw.span / 2, fw.span / 2, N_STRIP)
    ds = ys[1] - ys[0]                   # rotation preserves arc length
    wq = np.full(N_STRIP, ds)            # trapezoid weights, so this
    wq[0] = wq[-1] = ds / 2              # matches l1_trim's np.trapezoid
    cs = np.array([fw.chord_at(abs(2 * y / fw.span)) for y in ys])
    dm_n = math.pi * RHO_AIR * cs ** 2 / 4 * wq
    t = fw.thickness_ratio * cs
    dm_c_raw = math.pi * RHO_AIR * (t / 2) ** 2 * wq
    di_own = math.pi * RHO_AIR * cs ** 4 / 128 * wq

    # l1_trim models surge as a fraction of displaced volume rather than
    # by strips; keep that convention so the planar reduction is exact,
    # and report the strip value for comparison.
    surge_strip = float(dm_c_raw.sum())
    surge_volume = SURGE_VOLUME_FRACTION * RHO_AIR * mesh.volume
    dm_c = dm_c_raw * (surge_volume / surge_strip if surge_strip > 0 else 0.0)

    added = np.zeros((6, 6))
    for k, y in enumerate(ys):
        sgn = math.copysign(1.0, y) if y != 0 else 1.0
        y_f, z_f = fold(y, 0.0, gamma, y_break)
        inboard = abs(y) <= y_break
        g = 0.0 if inboard else gamma
        n_hat = np.array([0.0, -sgn * math.sin(g), math.cos(g)])
        s_hat = np.array([0.0, sgn * math.cos(g), math.sin(g)])
        c_hat = np.cross(s_hat, n_hat)          # ≈ (1, 0, 0)
        r = np.array([math.tan(math.radians(SWEEP_DEG)) * abs(y),
                      y_f, z_f]) - r_cg
        for dm, axis in ((dm_n[k], n_hat), (dm_c[k], c_hat)):
            rxa = np.cross(r, axis)
            added[:3, :3] += dm * np.outer(axis, axis)
            added[:3, 3:] += dm * np.outer(axis, rxa)
            added[3:, :3] += dm * np.outer(rxa, axis)
            added[3:, 3:] += dm * np.outer(rxa, rxa)
        added[3:, 3:] += di_own[k] * np.outer(s_hat, s_hat)

    return MassProps3D(
        m_total=m_total, m_skin=m_skin, m_pod=m_pod,
        buoyancy_n=NET_LIFT_PER_M3 * mesh.volume * G,
        volume=float(mesh.volume),
        r_cg=r_cg, r_cb=np.asarray(mesh.center_mass, dtype=float),
        r_skin=r_skin, inertia=inertia, added=added,
        dihedral_deg=dihedral_deg, fold_eta=fold_eta,
        surge_strip_kg=surge_strip)


# ---------------------------------------------------------------------------
def _summary(spec: KytoonSpec) -> str:
    p = mass_props_3d(spec)
    lines = [
        f"## {spec.name} — 3D mass properties (Γ = 0)",
        "",
        f"- mass {p.m_total:.0f} kg (skin {p.m_skin:.0f} + pod "
        f"{p.m_pod:.0f}), net lift {p.net_lift_kg:+.0f} kg",
        f"- CG ({p.r_cg[0]:+.2f}, {p.r_cg[1]:+.2f}, {p.r_cg[2]:+.2f})   "
        f"CB ({p.r_cb[0]:+.2f}, {p.r_cb[1]:+.2f}, {p.r_cb[2]:+.2f})",
        "- inertia about CG [kg·m²]:",
    ]
    for row in p.inertia:
        lines.append("    %10.0f %10.0f %10.0f" % tuple(row))
    lines += [
        f"    roll {p.i_roll:.0f}   pitch {p.i_pitch:.0f}   "
        f"yaw {p.i_yaw:.0f}",
        "- added mass [kg, kg·m²]:",
        f"    surge {p.added[0, 0]:8.0f}   sway {p.added_sway:8.0f}   "
        f"heave {p.added_heave:8.0f}",
        f"    roll  {p.added_roll:8.0f}   pitch {p.added[4, 4]:8.0f}   "
        f"yaw   {p.added[5, 5]:8.0f}",
        f"- roll: added is {p.added_roll / p.i_roll:.1f}× the structural "
        f"inertia — the air dominates the lateral modes",
        f"- sway: added is {p.added_sway / p.added_heave:.3f}× heave "
        f"(edge-on the wing displaces almost nothing)",
        f"- heave-pitch coupling A₃₅ = {p.added[2, 4]:.0f} kg·m "
        f"(the planar model omits it; Stage 4 decides)",
        f"- surge: {p.added[0, 0]:.0f} kg by l1_trim's volume rule vs "
        f"{p.surge_strip_kg:.0f} kg by strips",
        "",
        "Dihedral sweep — Γ folds the panels, moving the CB that the "
        "single-confluence instability finding rests on:",
        "",
        "    Γ     CB x     CB z    roll I   added roll   sway add   "
        "ctl attach z",
    ]
    for g in (0.0, 10.0, 20.0, 30.0):
        q = mass_props_3d(spec, dihedral_deg=g)
        a = attach_point_3d(spec, spec.bridle.positions[2], dihedral_deg=g)
        lines.append(
            "  %4.0f°  %+7.3f  %+7.3f  %8.0f  %10.0f  %9.0f  %9.2f"
            % (g, q.r_cb[0], q.r_cb[2], q.i_roll, q.added_roll,
               q.added_sway, a[2]))
    return "\n".join(lines)


if __name__ == "__main__":
    import argparse
    import sys

    from kytoon.spec import load_spec

    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description="3D mass properties for a spec")
    ap.add_argument("spec", help="path to a specs/*.yaml file")
    args = ap.parse_args()
    print(_summary(load_spec(args.spec)))
