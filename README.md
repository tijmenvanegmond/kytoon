# kytoon-sim — Manta Focused

L0 analytic design layer for Manta Type A-Z variants.
YAML spec → pydantic model → closed-form solvers → markdown comparison report.

## Quick Start

```bash
# Generate Manta comparison report
python -m kytoon.report_manta specs/manta/ -o reports/manta.md

# Run L0 analysis on a specific Manta type
python -m kytoon.solvers.l0 specs/manta/manta_typeB.yaml

# Export Manta specs for Godot
PYTHONPATH=. python godot/tools/export_manta_specs.py
```

## Project Structure

### Manta Types (A-Z)

The project is now focused on **Manta variants** with different mission profiles:

| Type | Name | Focus | Payload | Tether | Wind Range |
|------|------|-------|---------|--------|------------|
| A | Scout | Lightweight, agile | 20 kg | 300 m | 0-22.7 m/s |
| B | Standard | Balanced (original Mk V) | 60 kg | 400 m | 0-23.1 m/s |
| C | Heavy Lift | High payload | 200 kg | 500 m | 0-25.0 m/s |
| D | Long Range | Extended endurance | 80 kg | 800 m | 0-21.8 m/s |
| E | High Altitude | Stratospheric | 40 kg | 1000 m | 0-21.7 m/s |

- `specs/manta/*.yaml` — Manta Type A-Z specification files
- `reports/manta.md` — Generated comparison report
- `kytoon/report_manta.py` — Manta-focused report generator

### Legacy Designs

Previous Kytoon designs (Mk I-IV) are retained in `specs/` for reference:
- `specs/mk1_sled.yaml` — Mk I Sled
- `specs/mk2_helikite.yaml` — Mk II Helikite
- `specs/mk3_spine.yaml` — Mk III Spine
- `specs/mk4_torus.yaml` — Mk IV Torus
- `specs/mk5_manta.yaml` — Original Mk V (now Type B)
- `specs/alternates/` — Alternate designs

Run the legacy report with:
```bash
python -m kytoon.report specs/ -o reports/l0.md
```

## Godot Integration

The Godot project (`godot/`) supports switching between Manta types at runtime:

- **TAB** / **SHIFT+TAB** — Cycle through types
- **M** — Open type selection menu

Configuration files:
- `godot/data/manta_types.json` — UI metadata (colors, descriptions)
- `godot/data/manta_specs.json` — Technical specifications (auto-generated)
- `godot/common/manta_type_loader.gd` — Type switching logic

See `godot/README.md` for full Godot documentation.

## Layout
- `specs/*.yaml` — KytoonSpec per design (geometry, pressures, tether, bridle, masses)
- `specs/manta/*.yaml` — Manta Type A-Z variants
- `kytoon/spec.py` — pydantic schema; volumes/masses as derived properties
- `kytoon/solvers/l0.py` — buoyancy (Archimedes), tube stress (hoop N=p·r,
  wrinkle onset M_w=p·π·r³/2 per Comer & Levy), wind envelope (v_min from
  static-lift deficit; v_max = min(tether WLL, wrinkle margin via bisection,
  canopy fabric limit))
- `kytoon/report.py` — comparison table + per-member margins + flags (all designs)
- `kytoon/report_manta.py` — Manta-focused comparison report
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
  Run one spec: `python -m kytoon.solvers.l1_aero specs/manta/manta_typeB.yaml`.
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
