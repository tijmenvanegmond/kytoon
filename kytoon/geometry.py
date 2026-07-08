"""Parametric 3D geometry kernel (task 6): spec → meshes.

The YAML specs are already the parameter set; this module realizes them as
3D geometry. Two kinds of parts, by physical nature:
  - pressurized volumes (LE tube, struts, spar, lobe, torus) → closed
    meshes whose volume must agree with the spec's derived properties
    (that agreement is gated in tests/test_geometry.py — the mesh is the
    same design state, not a second one);
  - soft surfaces (canopy, keel wing) → zero-thickness open meshes, area
    checked against the spec.

`ArcWing` — the C-arc shape shared with the L1 aero solver — lives here.
When a spec carries an LE tube, its developed length pins the arc shape
(arc/span = θ0/sin θ0); otherwise the V3 benchmark's shape is the default.
Finding this encoded: Mk I's 44 m tube on a 38 m span → height/span ≈ 0.25,
flatter than the V3's 0.376.

Exports GLB (scene with named parts, browser-viewable) and STL (single
concatenated mesh) per Mk. Requires the `l1` extra (trimesh).

CLI: python -m kytoon.geometry specs/ -o models
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from kytoon.spec import KytoonSpec

try:
    import trimesh
    HAS_TRIMESH = True
except ImportError:  # default install is L0-only by design
    HAS_TRIMESH = False

# V3-derived shape defaults (data/tudelft_v3/properties.yaml):
# projected height 3.13 m / span 8.32 m; chord fullness 0.85 → taper 0.55.
ARC_HEIGHT_RATIO = 0.376
CHORD_TAPER = 0.55
TWIN_SKIN_TUBE_T = 0.06         # slim effective LE for twin-skin sections
SOFT_SURFACE_SECTIONS = 40      # mesh resolution along the arc
TUBE_SECTIONS = 24              # circumferential resolution of tubes


def _theta_from_arc_ratio(ratio: float) -> float:
    """Solve θ/sinθ = arc_length/span for the arc half-angle (bisection)."""
    lo, hi = 1e-6, 2.6
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if mid / math.sin(mid) < ratio:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


@dataclass
class ArcWing:
    """Parametric C-arc LEI wing, fully determined by spec bulk numbers."""
    span: float            # m, projected (tip-to-tip width)
    area: float            # m², projected
    height_ratio: float = ARC_HEIGHT_RATIO
    taper: float = CHORD_TAPER
    tube_t: float = 0.10   # LE tube diameter / chord (Breukels section param)
    kappa: float = 0.08    # section camber (Breukels)
    n_sections: int = 20

    @classmethod
    def from_spec(cls, spec: KytoonSpec) -> "ArcWing":
        if spec.canopy is None:
            raise ValueError(f"{spec.name}: no canopy — no wing geometry")
        mean_chord = spec.canopy.area / spec.canopy.span
        height_ratio = ARC_HEIGHT_RATIO
        tube_t = TWIN_SKIN_TUBE_T   # no LE tube → twin-skin (l1_aero flags it)
        if spec.le_tube is not None:
            tube_t = spec.le_tube.diameter / mean_chord
            # the spec's developed tube length pins the arc shape
            ratio = spec.le_tube.length / spec.canopy.span
            if ratio > 1.001:
                th = _theta_from_arc_ratio(ratio)
                height_ratio = math.tan(th / 2) / 2
        return cls(span=spec.canopy.span, area=spec.canopy.area,
                   height_ratio=height_ratio, tube_t=tube_t)

    # --- closed-form arc geometry (hand-checkable, in the L0 spirit) -------
    @property
    def arc_half_angle(self) -> float:
        # height/span = tan(θ0/2)/2 for a circular arc
        return 2 * math.atan(2 * self.height_ratio)

    @property
    def arc_radius(self) -> float:
        return self.span / (2 * math.sin(self.arc_half_angle))

    @property
    def arc_length(self) -> float:
        return 2 * self.arc_half_angle * self.arc_radius

    def _chord_shape(self, phi: float) -> float:
        return 1 - (1 - self.taper) * (phi / self.arc_half_angle) ** 2

    def section_points(self, n: int | None = None
                       ) -> list[tuple[np.ndarray, np.ndarray]]:
        """(LE, TE) per section: x downstream, y spanwise, z up."""
        th0, r = self.arc_half_angle, self.arc_radius
        phis = np.linspace(-th0, th0, n or self.n_sections)
        # projected area = ∫ c dy; y = R·sinφ already carries the cosφ Jacobian
        c0 = self.area / np.trapezoid([self._chord_shape(p) for p in phis],
                                      r * np.sin(phis))
        out = []
        for phi in phis:
            le = np.array([0.0, r * math.sin(phi), r * math.cos(phi)])
            te = le + np.array([c0 * self._chord_shape(phi), 0.0, 0.0])
            out.append((le, te))
        return out


# --- mesh builders -----------------------------------------------------------
def _require_trimesh() -> None:
    if not HAS_TRIMESH:
        raise ImportError(
            "trimesh not installed — geometry kernel needs the l1 extra: "
            'pip install "kytoon-sim[l1]"'
        )


def _tube_between(a: np.ndarray, b: np.ndarray, diameter: float):
    """Closed cylinder from a to b."""
    seg = np.asarray([a, b], dtype=float)
    return trimesh.creation.cylinder(radius=diameter / 2, segment=seg,
                                     sections=TUBE_SECTIONS)


def _tube_along_points(points: np.ndarray, diameter: float):
    """Tube as consecutive closed cylinders (joint overlap ≈ cancels the
    chord-vs-arc shortfall at this resolution; volume gated at 5%)."""
    parts = [_tube_between(points[i], points[i + 1], diameter)
             for i in range(len(points) - 1)]
    return trimesh.util.concatenate(parts)


def _loft_surface(rows: list[np.ndarray]):
    """Open quad-strip surface between successive rows of equal length."""
    rows = np.asarray(rows)
    n_rows, n_cols = rows.shape[0], rows.shape[1]
    verts = rows.reshape(-1, 3)
    faces = []
    for i in range(n_rows - 1):
        for j in range(n_cols - 1):
            a = i * n_cols + j
            b, c, d = a + 1, a + n_cols, a + n_cols + 1
            faces += [[a, b, c], [b, d, c]]
    return trimesh.Trimesh(vertices=verts, faces=faces, process=False)


def _canopy_mesh(geom: ArcWing, chordwise: int = 8):
    pts = geom.section_points(SOFT_SURFACE_SECTIONS)
    fractions = np.linspace(0, 1, chordwise)
    rows = [np.array([le + f * (te - le) for (le, te) in pts])
            for f in fractions]
    return _loft_surface(rows)


def build(spec: KytoonSpec) -> "trimesh.Scene":
    """Named-part scene for one spec. Parts carry the spec's dimensions."""
    _require_trimesh()
    scene = trimesh.Scene()

    # helikite/blimp canopies are not C-arcs — handled in their own branches
    if spec.canopy is not None and spec.lobe is None and spec.hull is None:
        geom = ArcWing.from_spec(spec)
        scene.add_geometry(_canopy_mesh(geom), geom_name="canopy")
        le_pts = np.array([le for le, _ in
                           geom.section_points(SOFT_SURFACE_SECTIONS)])
        if spec.le_tube is not None:
            scene.add_geometry(
                _tube_along_points(le_pts, spec.le_tube.diameter),
                geom_name="le_tube")
        if spec.struts is not None and spec.n_struts:
            th0 = geom.arc_half_angle
            pts = geom.section_points(SOFT_SURFACE_SECTIONS)
            phis = np.linspace(-th0, th0, SOFT_SURFACE_SECTIONS)
            targets = np.linspace(-th0 * 0.92, th0 * 0.92, spec.n_struts)
            for k, t in enumerate(targets):
                i = int(np.argmin(np.abs(phis - t)))
                le, te = pts[i]
                direction = (te - le) / np.linalg.norm(te - le)
                end = le + direction * spec.struts.length
                scene.add_geometry(
                    _tube_between(le, end, spec.struts.diameter),
                    geom_name=f"strut_{k}")
        if spec.spar is not None:
            # chordwise keel spar under the arc's center section
            le, te = geom.section_points(3)[1]
            lo = le + np.array([0.0, 0.0, -0.6 * spec.spar.diameter])
            direction = (te - le) / np.linalg.norm(te - le)
            scene.add_geometry(
                _tube_between(lo, lo + direction * spec.spar.length,
                              spec.spar.diameter),
                geom_name="spar")

    if spec.lobe is not None:
        # layout (matches the iteration sheet's side view): delta keel wing
        # plane at z=0, lobe seated over its mid-chord, pod slung below.
        c_root = (2 * spec.canopy.area / spec.canopy.span
                  if spec.canopy is not None else spec.lobe.diameter)

        lobe = trimesh.creation.icosphere(subdivisions=4)
        lobe.apply_scale([spec.lobe.diameter / 2, spec.lobe.diameter / 2,
                          spec.lobe.height / 2])
        lobe.apply_translation([0.45 * c_root, 0.0,
                                spec.lobe.height / 2 + 0.5])
        scene.add_geometry(lobe, geom_name="lobe")

        if spec.canopy is not None:
            # swept delta, area = ½·c_root·span; shallow center fold (the
            # "keel") so it reads as a surface with dihedral, not a sticker.
            # projected area is preserved: the fold point only moves in z.
            b2 = spec.canopy.span / 2
            verts = [
                [0.0, 0.0, 0.0],            # apex, forward
                [c_root, -b2, 1.2],         # tip, slight upsweep
                [c_root, b2, 1.2],
                [0.85 * c_root, 0.0, -1.5],  # keel fold, below centerline
            ]
            wing = trimesh.Trimesh(
                vertices=verts, faces=[[0, 1, 3], [0, 3, 2], [1, 2, 3]],
                process=False)
            scene.add_geometry(wing, geom_name="keel_wing")

        # gimbaled EO/IR pod (spec payload), hung under the keel
        pod = trimesh.creation.capsule(radius=0.45, height=1.1)
        pod.apply_transform(
            trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))
        pod.apply_translation([0.55 * c_root, 0.0, -3.2])
        scene.add_geometry(pod, geom_name="pod")

    if spec.hull is not None:
        # prolate hull, long axis downwind (x); two side deltas at max beam
        hl, hr = spec.hull.length / 2, spec.hull.diameter / 2
        hull = trimesh.creation.icosphere(subdivisions=4)
        hull.apply_scale([hl, hr, hr])
        scene.add_geometry(hull, geom_name="hull")

        if spec.canopy is not None:
            # per-wing: root sits on the hull at 0.9·r; size the root chord
            # from the actual root→tip extent so 2·(½·c·h) = spec area
            y_root = 0.9 * hr
            half_span = spec.canopy.span / 2 - y_root
            root = spec.canopy.area / half_span   # 2 wings: ½·c·h each
            x0 = -root / 2 + 1.0                  # root slightly aft of center
            for side, sgn in (("stbd", 1.0), ("port", -1.0)):
                ys = sgn * y_root
                verts = [
                    [x0, ys, 0.0],
                    [x0 + root, ys, 0.0],
                    [x0 + 0.75 * root, sgn * spec.canopy.span / 2, 0.8],
                ]
                wing = trimesh.Trimesh(vertices=verts, faces=[[0, 1, 2]],
                                       process=False)
                scene.add_geometry(wing, geom_name=f"wing_{side}")

        pod = trimesh.creation.capsule(radius=0.45, height=1.1)
        pod.apply_transform(
            trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))
        pod.apply_translation([0.0, 0.0, -hr - 0.8])
        scene.add_geometry(pod, geom_name="pod")

    if spec.torus is not None:
        ring = trimesh.creation.torus(
            major_radius=spec.torus.ring_diameter / 2,
            minor_radius=spec.torus.tube_diameter / 2,
            major_sections=96, minor_sections=48)
        scene.add_geometry(ring, geom_name="torus")

    return scene


def export(spec: KytoonSpec, out_dir: str | Path,
           fmts: tuple[str, ...] = ("glb", "stl")) -> list[Path]:
    """Write the spec's geometry; STL gets one concatenated mesh."""
    _require_trimesh()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    scene = build(spec)
    slug = f"mk{spec.mk.lower()}" if spec.mk.isalpha() else spec.mk
    written = []
    for fmt in fmts:
        p = out / f"{slug}.{fmt}"
        if fmt == "stl":
            trimesh.util.concatenate(
                list(scene.geometry.values())).export(p)
        else:
            scene.export(p)
        written.append(p)
    return written


if __name__ == "__main__":
    import argparse
    import sys

    from kytoon.spec import load_all

    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description="Export 3D geometry per spec")
    ap.add_argument("specs", help="directory of spec YAMLs")
    ap.add_argument("-o", "--out", default="models")
    args = ap.parse_args()
    for s in load_all(args.specs):
        for p in export(s, args.out):
            print(f"{s.name}: {p}")
