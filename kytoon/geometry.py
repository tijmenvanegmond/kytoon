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
        if spec.fat_wing is not None:
            # fat wing: near-flat planform; section thickness from the loft
            height_ratio = 0.05
            tube_t = spec.fat_wing.thickness_ratio
            taper = spec.fat_wing.taper
            return cls(span=spec.canopy.span, area=spec.canopy.area,
                       height_ratio=height_ratio, taper=taper, tube_t=tube_t)
        elif spec.le_tube is not None:
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


def _airfoil_section(chord: float, thickness: float,
                     n_half: int = 18) -> np.ndarray:
    """Closed NACA-4-style symmetric section outline in the x–z plane,
    (n, 2) array ordered around the perimeter. x ∈ [0, chord]."""
    xb = 0.5 * (1 - np.cos(np.linspace(0, math.pi, n_half)))  # cosine spacing
    tr = thickness / chord
    z = 5 * tr * chord * (0.2969 * np.sqrt(xb) - 0.1260 * xb
                          - 0.3516 * xb**2 + 0.2843 * xb**3
                          - 0.1036 * xb**4)   # closed trailing edge
    upper = np.column_stack([xb * chord, z])
    lower = np.column_stack([xb * chord, -z])[::-1]
    return np.vstack([upper, lower[1:-1]])    # closed ring, no dup endpoints


def _lofted_fatwing(fw, sweep_deg: float = 15.0, n_span: int = 33):
    """Watertight manta body: airfoil sections lofted along the tapering
    span, quarter-chord line swept aft. Fat center, thin tips."""
    ys = np.linspace(-fw.span / 2, fw.span / 2, n_span)
    rings = []
    for y in ys:
        c = fw.chord_at(abs(2 * y / fw.span))
        sec = _airfoil_section(c, fw.thickness_ratio * c)
        x_off = math.tan(math.radians(sweep_deg)) * abs(y) - 0.25 * c
        rings.append(np.column_stack([
            sec[:, 0] + x_off, np.full(len(sec), y), sec[:, 1]]))
    rings = np.asarray(rings)                 # (n_span, n_ring, 3)
    n_ring = rings.shape[1]
    verts = rings.reshape(-1, 3)
    faces = []
    for i in range(n_span - 1):
        for j in range(n_ring):
            a = i * n_ring + j
            b = i * n_ring + (j + 1) % n_ring
            c2, d = a + n_ring, b + n_ring
            faces += [[a, b, c2], [b, d, c2]]
    # cap the tips with fans to their centroids
    for i, flip in ((0, False), (n_span - 1, True)):
        centroid = len(verts) + (0 if i == 0 else 1)
        for j in range(n_ring):
            a = i * n_ring + j
            b = i * n_ring + (j + 1) % n_ring
            faces.append([b, a, centroid] if not flip else [a, b, centroid])
    verts = np.vstack([verts, rings[0].mean(axis=0), rings[-1].mean(axis=0)])
    mesh = trimesh.Trimesh(vertices=verts, faces=faces, process=True)
    if mesh.volume < 0:
        mesh.invert()
    return mesh


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

    # helikite/fatwing/blimp canopies are not C-arcs — own branches below
    if (spec.canopy is not None and spec.lobe is None
            and spec.fat_wing is None and spec.hull is None):
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
        # v2 layout: two side deltas rooted on the lobe equator (a single
        # under-lobe fabric sheet cannot hold shape), pod slung below.
        a, cz = spec.lobe.diameter / 2, spec.lobe.height / 2
        lobe = trimesh.creation.icosphere(subdivisions=4)
        lobe.apply_scale([a, a, cz])
        scene.add_geometry(lobe, geom_name="lobe")

        if spec.canopy is not None:
            y_root = 0.9 * a
            half_span = spec.canopy.span / 2 - y_root
            root = spec.canopy.area / half_span   # 2 wings: ½·c·h each
            x0 = -root / 2
            for side, sgn in (("stbd", 1.0), ("port", -1.0)):
                tip = [x0 + 0.75 * root, sgn * spec.canopy.span / 2, 0.6]
                wing = trimesh.Trimesh(
                    vertices=[
                        [x0, sgn * y_root, 0.0],
                        [x0 + root, sgn * y_root, 0.0],
                        tip,
                    ],
                    faces=[[0, 1, 2]], process=False)
                scene.add_geometry(wing, geom_name=f"wing_{side}")
                # stay: lobe upper surface → wing tip (bracing the panel)
                lp = np.array([0.0, sgn * 0.55 * a, 0.75 * cz])
                scene.add_geometry(
                    _tube_between(lp, np.array(tip), 0.06),
                    geom_name=f"stay_{side}")

        # gimbaled EO/IR pod (spec payload), hung under the lobe
        pod = trimesh.creation.capsule(radius=0.45, height=1.1)
        pod.apply_transform(
            trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))
        pod.apply_translation([0.0, 0.0, -cz - 1.0])
        scene.add_geometry(pod, geom_name="pod")

    if spec.fat_wing is not None:
        fw = spec.fat_wing
        sweep_deg = 15.0
        scene.add_geometry(_lofted_fatwing(fw, sweep_deg),
                           geom_name="body")

        # 3-point tether interface on the underside:
        # control / MAIN / control at the bridle stations
        names = ["fixture_port", "fixture_main", "fixture_stbd"]
        for name, p in zip(names, spec.bridle.positions):
            y = (p - 0.5) * fw.span
            c = fw.chord_at(abs(2 * y / fw.span))
            x = math.tan(math.radians(sweep_deg)) * abs(y) + 0.05 * c
            r_fix = 0.35 if name == "fixture_main" else 0.22
            fix = trimesh.creation.icosphere(subdivisions=2, radius=r_fix)
            fix.apply_translation([x, y, -0.48 * fw.thickness_ratio * c])
            scene.add_geometry(fix, geom_name=name)

        # gimbaled EO/IR pod under mid-span, aft of the main fixture
        pod = trimesh.creation.capsule(radius=0.45, height=1.1)
        pod.apply_transform(
            trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))
        pod.apply_translation([0.3 * fw.chord, 0.0,
                               -0.5 * fw.t_max - 0.6])
        scene.add_geometry(pod, geom_name="pod")

    if spec.hull is not None:
        # blimp alternate: prolate hull, long axis downwind; side deltas
        hl, hr = spec.hull.length / 2, spec.hull.diameter / 2
        hull = trimesh.creation.icosphere(subdivisions=4)
        hull.apply_scale([hl, hr, hr])
        scene.add_geometry(hull, geom_name="hull")

        if spec.canopy is not None:
            y_root = 0.9 * hr
            half_span = spec.canopy.span / 2 - y_root
            root = spec.canopy.area / half_span   # 2 wings: ½·c·h each
            x0 = -root / 2 + 1.0
            for side, sgn in (("stbd", 1.0), ("port", -1.0)):
                wing = trimesh.Trimesh(
                    vertices=[
                        [x0, sgn * y_root, 0.0],
                        [x0 + root, sgn * y_root, 0.0],
                        [x0 + 0.75 * root, sgn * spec.canopy.span / 2, 0.8],
                    ],
                    faces=[[0, 1, 2]], process=False)
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
