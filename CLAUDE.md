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

**Read `KYTOON-PROJECT.md` before modifying solvers, specs, or tests.** It is
the canonical context: the wider kiteship system this feeds, the fleet logic
behind the four Mks, the documented physics approximations ("known lies") that
must be preserved or consciously replaced, findings already derived (don't
re-derive them), and the prioritized task queue. This file covers only
commands and code architecture.

## Commands

Windows (PowerShell):

```
python -m venv .venv
.venv\Scripts\pip install setuptools wheel pydantic pyyaml numpy pytest
.venv\Scripts\pip install --no-build-isolation --no-deps -e .   # editable install, see gotcha below
.venv\Scripts\pytest                                             # all validation gates
.venv\Scripts\pytest tests/test_l0.py::test_mk1_tow_force_magnitude   # single test
.venv\Scripts\python -m kytoon.report specs/ -o reports/l0.md    # regenerate comparison report
.venv\Scripts\python -m kytoon.viz specs/ -o reports/figures     # regenerate figures
.venv\Scripts\python -m kytoon.geometry specs/ -o models         # export 3D models (l1 extra)
```

macOS/Linux (bash): swap `.venv\Scripts\` for `.venv/bin/`.

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
- `kytoon/solvers/l1_aero.py` — L1 tier (optional): builds a parametric
  C-arc LEI wing from a spec's bulk numbers (`ArcWing`), solves it with the
  awegroup Vortex Step Method using Breukels 2-param section polars, and
  returns an `L1AeroReport` (clean + bridle-corrected system polar, operating
  point, spec-consistency ratio, flags). Guarded import: works only with the
  `l1` extra installed; everything else must keep running without it.
  CLI: `python -m kytoon.solvers.l1_aero specs/mk1_sled.yaml`.
- `kytoon/solvers/l1_body_aero.py` — L1 tier (optional): the wing+body
  hybrids (Mk II lobe, Mk V hull) through AeroSandbox `AeroBuildup`.
  Semi-empirical — it *bounds* the hand-picked spec coefficients (rigid
  smooth body = drag lower bound, no wake blanketing = CL upper bound),
  it does not certify them. Refuses non-hybrid archetypes.
  CLI: `python -m kytoon.solvers.l1_body_aero specs/mk5_manta.yaml`.
- `kytoon/solvers/l1_tether.py` — L1 tier (optional): tether as an inverted
  mooring line in air via MoorPy (`System(rho=1.225)`, wind as current).
  Returns `L1TetherReport` (drag/sag line shape, true elevation angles,
  tensions, flags); `v_max_tether()` bisects the drag-corrected WLL ceiling.
  Same guarded-import rule as l1_aero.
  CLI: `python -m kytoon.solvers.l1_tether specs/mk1_sled.yaml -v 12 --vmax`.
- `kytoon/report.py` — turns a list of `L0Report` into the comparison table +
  per-member structure margins + flags seen in `reports/l0.md`.
- `kytoon/geometry.py` — 3D kernel: realizes each spec as a trimesh scene
  (closed meshes for pressurized volumes, open surfaces for soft goods) and
  exports `models/*.glb|stl`. Owns `ArcWing`, the C-arc shape shared with
  `l1_aero` — when a spec has an LE tube, its developed length pins the arc
  (don't reintroduce a second shape definition). Needs the `l1` extra.
- `kytoon/viz.py` — static matplotlib figures into `reports/figures/`:
  fleet envelopes + structure margins (L0-only), Mk polars vs the V3
  benchmark and tether profiles (need the `l1` extra; CLI skips them
  gracefully). Entity colors are fixed per Mk — don't re-derive them from
  series order.
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
to fly"). The suite is a contract: if a spec change breaks a design gate,
that's a design conversation, not a reason to loosen the test.

## Conventions

- `specs/*.yaml` are THE design state. To change a design, edit the spec,
  not constants in code.
- SI units everywhere unless the field name says otherwise
  (`_bar`, `_mm`, `_kn`, `_deg`).
- L0 means every solver result is explainable by hand on paper. Anything
  needing iteration or a mesh belongs in L1, not here (the one existing
  exception: the wrinkle-margin bisection in `solve_wind_envelope`).
- `data/tudelft_v3/` is vendored CC-BY benchmark data — treat as read-only;
  re-download from awegroup if stale.
- Style: dataclasses for results, pydantic for specs, no classes where a
  function does.

## Fidelity ladder

L0 is the default; L1 aero exists behind the `l1` extra
(`pip install -e ".[l1]"` — installs awegroup VSM from git, not PyPI).
L1 tether (MoorPy) is also behind the extra. `tests/test_l1_*.py`'s
dependent tests skip automatically when the extra is absent. Still pending
at L1: mem4py (membrane FEM, to calibrate `TUBE_LOAD_SHARE`) — blocked, see
KYTOON-PROJECT.md §7.2. L2 = OpenFOAM ↔
CalculiX/FEniCSx via preCICE for gust/FSI, final candidate only. Don't add
L1/L2 dependencies to the default install.
