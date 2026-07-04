# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

L0 (closed-form/analytic) design layer for four kytoon archetypes — a
ship-launched, tethered lighter-than-air/kite hybrid vehicle. Iterations
Mk I–IV trade off buoyancy vs. aerodynamic lift: Mk I «Sled» (LEI traction
kite), Mk II «Helikite» (buoyant lobe + keel wing), Mk III «Spine» (semi-rigid
spar + twin-skin canopy), Mk IV «Torus» (annular aerostat, no wing).

Pipeline: `specs/*.yaml` → pydantic `KytoonSpec` → closed-form solvers →
markdown comparison report.

## Commands

```
python -m venv .venv && .venv/bin/pip install setuptools wheel pydantic pyyaml numpy pytest
.venv/bin/pip install --no-build-isolation --no-deps -e .   # editable install, see gotcha below
.venv/bin/pytest                                             # all validation gates
.venv/bin/pytest tests/test_l0.py::test_mk1_tow_force_magnitude   # single test
.venv/bin/python -m kytoon.report specs/ -o reports/l0.md    # regenerate comparison report
```

Gotcha: plain `pip install -e ".[dev]"` (as written in README.md) fails with
`Multiple top-level packages discovered in a flat-layout` because `data/`,
`specs/`, `reports/` sit next to `kytoon/` at repo root. `pyproject.toml` now
pins `[tool.setuptools] packages = ["kytoon", "kytoon.solvers"]` to fix this
— don't remove that section.

Gotcha: `reports/l0.md` currently has a hand-written "Aerodynamic calibration
provenance" section appended after the auto-generated table/flags content.
`kytoon/report.py`'s `full_report()` does **not** produce that section —
re-running `python -m kytoon.report specs/ -o reports/l0.md` will silently
drop it. Preserve/reappend that section manually if regenerating, or move it
into `full_report()` if it should be permanent.

## Architecture

Three-stage pipeline, one file per stage:

- `kytoon/spec.py` — pydantic schema. `Archetype` enum picks which component
  fields are required (`_check_archetype` validator). Component models
  (`InflatableTube`, `TorusEnvelope`, `Canopy`, `Lobe`, `Tether`) each derive
  their own `volume`/`mass`/`skin_area` as properties from geometry + areal
  density — there's no separate mass-properties module. `KytoonSpec`
  aggregates these into `helium_volume`, `structure_mass`, `total_mass`.
- `kytoon/solvers/l0.py` — three independent solve functions chained by
  `solve()`: `solve_buoyancy` (Archimedes on He volume vs. total mass),
  `solve_structure` (pressurized-beam theory: hoop load `N = p·r`, wrinkle
  moment `M_w = p·π·r³/2`; applied bending moment comes from treating the
  tow force as a distributed load across the worst unsupported span between
  bridle points), `solve_wind_envelope` (`v_min` from static-lift deficit,
  `v_max` from whichever of tether WLL / wrinkle margin / canopy fabric limit
  binds first — wrinkle margin is found by bisection over tow force).
  `TUBE_LOAD_SHARE = 0.35` encodes the "tensioned membrane carries most of
  the aero load to the bridles, tube only sees 35% as bending" assumption —
  this is called out in README.md as the biggest L0 uncertainty.
- `kytoon/report.py` — turns a list of `L0Report` into the comparison table +
  per-member structure margins + flags seen in `reports/l0.md`.
- `kytoon/aero.py` — independent calibration path, not called by the solvers
  at runtime. Loads vendored TU Delft V3 wind-tunnel/CFD polars from
  `data/tudelft_v3/*.csv`, builds a bridle-drag-corrected `SystemPolar`, and
  is used only by `tests/test_l0.py` to check that the hand-picked
  `cl_op`/`cd_op` values in the Mk I/III specs are consistent with published
  benchmark data (within 15% on resultant-force coefficient).

`tests/test_l0.py` mixes two kinds of checks — keep both when adding specs:
physics anchors against hand-computed or published reference values (e.g.
`NET_LIFT_PER_M3 ≈ 1.046`, torus volume closed form, wrinkle-moment formula),
and fleet-level design gates (e.g. "the four Mk specs together must cover
0→20+ m/s", "Mk II/IV must be calm-air capable", "Mk I/III must need <8 m/s
to fly").

## Fidelity ladder

This repo is L0 only. README.md documents the next tiers if that work
starts here: L1 = AeroSandbox (aero) + mem4py (membrane FEM) + MoorPy
(tether sag/drag) + trimesh/gmsh (geometry), installable via the `l1` extra.
L2 = OpenFOAM ↔ CalculiX/FEniCSx via preCICE for gust/FSI, final candidate
only. Don't add L1/L2 dependencies to the default install.
