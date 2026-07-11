# kytoon-sim

L0 analytic design layer for the kiteship kytoon iterations (Mk I–IV).
YAML spec → pydantic model → closed-form solvers → markdown comparison report.

```
uv venv && uv pip install -e ".[dev]"     # or: pip install -e ".[dev]"
pytest                                     # validation gates
python -m kytoon.report specs/ -o reports/l0.md
```

## Layout
- `specs/*.yaml` — one KytoonSpec per Mk (geometry, pressures, tether, bridle, masses)
- `kytoon/spec.py` — pydantic schema; volumes/masses as derived properties
- `kytoon/solvers/l0.py` — buoyancy (Archimedes), tube stress (hoop N=p·r,
  wrinkle onset M_w=p·π·r³/2 per Comer & Levy), wind envelope (v_min from
  static-lift deficit; v_max = min(tether WLL, wrinkle margin via bisection,
  canopy fabric limit))
- `kytoon/report.py` — comparison table + per-member margins + flags
- `kytoon/viz.py` — figures: fleet envelopes, structure margins, L1 polars
  vs benchmark, tether profiles → `reports/figures/`
- `kytoon/geometry.py` — 3D kernel: spec → trimesh scene → `models/*.glb|stl`
  (volumes gated against spec-derived properties)
- `tests/` — physics anchors (He 1.05 kg/m³, torus volume closed form,
  wrinkle-moment reference case) + design gates (fleet covers 0→20+ m/s)

## Model honesty notes
- LE tube / struts see 35% of aero load as bending (tensioned membrane
  carries the rest to the bridles). This factor is the biggest L0 uncertainty
  → validate at L1 with mem4py.
- Tether is straight-line: drag/sag deferred to MoorPy (L1).
- No gust cases, no FSI: L2 (OpenFOAM + preCICE) for the final candidate only.

## Fidelity ladder (next tiers)
- L1 aero — **built**: `kytoon/solvers/l1_aero.py` solves each Mk's own
  parametric C-arc LEI wing with awegroup's Vortex Step Method (Breukels
  section polars). Validated against the vendored V3 wind tunnel data:
  CL_max +10%, (L/D)max −19% (conservative), gated in `tests/test_l1_aero.py`
  (skipped unless the `l1` extra is installed: `pip install -e ".[l1]"`).
  Run one spec: `python -m kytoon.solvers.l1_aero specs/mk1_sled.yaml`.
- L1 tether — **built**: `kytoon/solvers/l1_tether.py` runs the tether as an
  inverted mooring line in air (MoorPy): line drag + sag, true elevation
  angles, drag-corrected tether v_max. `tests/test_l1_tether.py`.
- L1 body aero — **built**: `kytoon/solvers/l1_body_aero.py` bounds the
  Mk II/V wing+body coefficients with AeroSandbox AeroBuildup.
- L1 structure — pending/blocked: mem4py (membrane FEM, calibrates
  TUBE_LOAD_SHARE) needs a C++ toolchain and gmsh meshes; see
  KYTOON-PROJECT.md §7.2.
- L2: OpenFOAM ↔ CalculiX/FEniCSx via preCICE; gust + capture-state loads.
- Reference data: TU Delft V3 benchmark is VENDORED in `data/tudelft_v3/`
  (CC-BY, awegroup) and wired into `kytoon/aero.py`. Wind-tunnel + CFD polars
  calibrate the clean-wing coefficients; see reports/l0.md provenance section
  and data/tudelft_v3/SOURCE.md for citations.
