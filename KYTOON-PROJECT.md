# PROJECT.md — kytoon-sim

**Audience: agents (and humans) picking up this work without access to the
originating conversation.** This file is the canonical context. Read it fully
before modifying solvers, specs, or tests.

Last compiled: 2026-07-04.

---

## 1. What this project is

`kytoon-sim` is the **L0 (analytic) design layer** for a family of four
helium-assisted kite ("kytoon") designs, Mk I–IV. The kytoons are one
subsystem of a larger concept — the **kiteship**: a 40 m uncrewed trimaran
(Sea Hunter class) that uses a large tethered kytoon to launch drones
(tether-riding trolley), recover them (Skyhook-style capture line), and do
ISR (EO/IR pod at 150–400 m). A 7-DOF manipulator arm on a deck rail captures
the kytoon itself by a passive grapple fixture on the leading-edge underside
after it is winched down to ~20 m, where it hovers quasi-statically on He
buoyancy.

The ship, arm, and operational design live **outside this repo** as two SVG
drawing sheets (`kiteship-ga.svg`, `kytoon-iterations.svg`) and a narrative
log (`kiteship-project-log.md`). This repo owns the *numbers*.

### The four archetypes (specs/*.yaml)

| Mk | Name | Archetype | Role | Key mechanism |
|----|------|-----------|------|---------------|
| I | Sled | `lei` | launch boost + traction | C-shape LEI, He in LE tube + 8 struts |
| II | Helikite | `helikite` | zero-wind ISR loiter | oblate He lobe + delta keel wing |
| III | Spine | `spine` | boost + drone perch/recharge | inflatable keel spar carrying grapple rail + 300 kg dock |
| IV | Torus | `torus` | calm-air recovery node | He ring, capture line through central duct |

**Fleet logic (load-bearing design decision):** no single Mk covers the
operational wind range. The ship carries two kytoon types — a buoyant one
(II or IV, v_min = 0) plus a traction one (I or III). The test
`test_fleet_covers_zero_to_20ms` encodes this as a hard requirement.
**Common interfaces across all Mks** (do not fork per-Mk): tether
termination + load cell, grapple fixture geometry, IMU/GNSS + He telemetry,
capture-line hardpoint.

---

## 2. Repo map and conventions

```
specs/*.yaml            one KytoonSpec per Mk — THE design state. Edit these,
                        not constants in code, to change a design.
kytoon/spec.py          pydantic v2 schema. Volumes/masses are derived
                        properties, never stored fields.
kytoon/solvers/l0.py    closed-form physics. SI units everywhere unless the
                        field name says otherwise (_bar, _mm, _kn, _deg).
kytoon/aero.py          TU Delft V3 benchmark loader + system-polar model.
kytoon/report.py        CLI: python -m kytoon.report specs/ -o reports/l0.md
data/tudelft_v3/        vendored CC-BY benchmark (see SOURCE.md for citations).
                        Treat as read-only; re-download from awegroup if stale.
tests/test_l0.py        14 tests = the validation contract (see §4).
```

Python ≥3.11, deps: pydantic v2, pyyaml, numpy (numpy currently unused by L0
but reserved). Optional extras: `[l1]` = aerosandbox, trimesh, gmsh. Run:
`pip install -e ".[dev]" && pytest && python -m kytoon.report specs/`.

Style: dataclasses for results, pydantic for specs, no classes where a
function does. Every solver result should be explainable by hand on paper —
that is the definition of L0. If you need iteration or a mesh, it belongs in
L1, not here.

---

## 3. Physics model — assumptions and KNOWN LIES

An agent modifying `l0.py` must preserve these documented approximations or
consciously replace them (and update this file + tests):

1. **Buoyancy**: He net lift 1.046 kg/m³ (ISA sea level, purity ignored).
2. **Tube stress**: hoop running load N = p·r; wrinkle onset
   M_w = p·π·r³/2 (Comer & Levy); collapse ≈ 2·M_w. `ok` gate: hoop
   utilization ≤ 0.25 (SF 4, inflatable-structure convention) AND bending
   utilization < 1.0 (wrinkle onset, not collapse).
3. **⚠ TUBE_LOAD_SHARE = 0.35** in `solve_structure`: LE tube and struts see
   only 35% of aero load as bending — the tensioned canopy membrane carries
   the rest directly to the bridles (tensairity-like effect). **This single
   number is the largest L0 uncertainty and is currently uncalibrated.**
   The keel spar (Mk III) intentionally carries 100% (it IS the load path)
   plus the dock point load. First task of L1 is to validate/replace this
   factor with mem4py membrane FEM.
4. **Wind envelope**: v_min from static-lift deficit at cl_op with vertical
   fraction cos(90° − tether elevation); v_max = min(tether WLL, wrinkle
   margin found by bisection on tow force, canopy fabric limit). Quasi-static
   only — **no gust cases, no crosswind maneuvers, no dynamic loads.**
5. **Tether**: straight line; mass counted, drag and sag NOT integrated
   (deferred to MoorPy at L1). Torus v_max uses bluff-body drag
   (Cd 0.5 × ring frontal area) against tether WLL.
6. **Torus structure**: hoop check only. Ring bending under the 3-point
   bridle is an L1 problem — flagged, not solved.
7. **Mk II lobe**: 500 Pa assumed gust superpressure for the hoop check;
   its wing coefficients (cl_op 0.6, cd_op 0.25 for lobe-wake degradation)
   are hand-picked and NOT covered by the benchmark (§5 caveat).

---

## 4. The test suite is a contract

`tests/test_l0.py` — 14 tests, all passing at last compile. Categories:

- **Physics anchors** (must never change without a source): He net-lift
  constant; torus volume closed form; wrinkle-moment reference case
  (Ø0.6 m @ 0.4 bar → 1.696 kN·m); hoop N = p·r.
- **Design gates** (encode requirements, change only with a design decision):
  Mk II + IV calm-air capable; Mk I + III v_min ∈ (0, 8) m/s; fleet covers
  0 → 20+ m/s; envelopes ordered.
- **Benchmark calibration gates**: wind-tunnel CL_max ≈ 1.07 reproduced;
  (L/D)_max ≈ 8.7; CL = 0.8 reached pre-stall; Mk I's resultant-force
  coefficient within 15% of the benchmark operating point. The last one
  fails if someone edits spec aero coefficients into unsupported territory —
  that is intentional.

If a spec change breaks a design gate, the answer is a design conversation,
not loosening the test.

---

## 5. Aerodynamic provenance (why the numbers are citable)

Clean-wing coefficients for Mk I/III are anchored to the **TU Delft V3 LEI
benchmark** (awegroup/TUDELFT_V3_KITE, CC-BY, vendored in
`data/tudelft_v3/`):

- Wind tunnel: Poland et al. 2026, *Wind Energy Science* 11, 911 —
  1:6.5 rigid model of the 25 m² V3, Re 5e5, α −11.6…24.5°.
  CL_max ≈ 1.07 @ 18°, clean (L/D)_max ≈ 8.7 @ 9°.
- CFD RANS with struts: Viré et al. 2022, *Energies* 15, 1450, Re 1e6.
  CL_max ≈ 1.35 @ 19° (used as Mk III twin-skin upper bound).

`aero.py` adds bridle parasitic drag (bluff-body lines scaled from the V3's
82-line/96 m bridle by area^0.5) to form a *system* polar. Outcome of the
calibration: the originally hand-picked cl_op 0.8 / cd_op 0.15 were
**validated within 15%** (resultant coefficient), not corrected.

Caveats an agent must not silently drop: (a) Mk II and Mk IV aero is NOT
benchmark-covered; (b) the V3 is a crosswind AWE kite — its 5.8 kN nominal
pull is at ~3× true wind apparent speed; our static-lift tow numbers are
legitimately lower per m² and not comparable to AWE traction figures.

---

## 6. Findings so far (do not re-derive)

- **Mk I is not self-neutral.** 37 m³ He ≈ 39 kg lift vs 185 kg mass.
  Self-neutrality needs ~180 m³ (≈Ø2 m LE tube). Current design accepts
  v_min ≈ 3.8 m/s instead. The iteration drawing sheet still says
  "≈ 0 static" — known drawing/code disagreement, see task list.
- **Mk III spar was resized by the solver**: original Ø0.6 m @ 0.4 bar
  wrinkled at 316% under dock load alone; now Ø0.8 m @ 0.6 bar with the
  5-fixture rail doubling as bridle nodes → 48%.
- **Mk I is wrinkle-limited (13.7 m/s), not tether-limited.** Knobs: bridle
  density, tube pressure.
- **Drawing-level finding**: the arm's 15 m reach envelope overlaps the
  tether traveller zone — arm motion planning must treat the tether as a
  dynamic keep-out volume. Unresolved, lives with the ship design.

---

## 7. Task queue (priority order)

1. **L1 aero**: the awegroup org ships an open Vortex Step Method (VSM)
   solver validated against the vendored data. Integrate it (or AeroSandbox
   VLM as fallback) to compute Mk-specific polars instead of borrowing the
   V3's. Entry point suggestion: `kytoon/solvers/l1_aero.py`, keep the
   L0 interface (`solve(spec) -> report`) shape.
2. **L1 structure**: mem4py membrane FEM to calibrate/replace
   TUBE_LOAD_SHARE (§3.3). Success criterion: a computed load-share value
   with a validation case, replacing the 0.35 constant.
3. **L1 tether**: MoorPy quasi-static line (a kite tether is an inverted
   mooring line) — adds drag/sag, corrects v_max and tether-angle numbers.
4. **Sync the drawings**: `kytoon-iterations.svg` spec table predates the
   solver — update its numbers from `reports/l0.md` (esp. Mk I static lift
   and all wind envelopes). Drawings live outside the repo.
5. **Mk II aero**: no benchmark exists for lobe+wing; either find aerostat
   hybrid data or schedule an L1 VLM study with the lobe modeled as a body.
6. **Geometry kernel** (enables L1/L2): parametric mesh generation per Mk
   (trimesh/gmsh, STL export). The YAML specs are already the parameter set.
7. **L2 (later)**: OpenFOAM ↔ CalculiX/FEniCSx via preCICE for gust and
   capture-state (arm-attached) load cases, final candidate only.

---

## 8. Wider-context pointers

- Companion artifacts (not in repo): `kiteship-ga.svg` (ship GA),
  `kytoon-iterations.svg` (Mk sheet), `kiteship-project-log.md` (narrative),
  `kytoon-l0-report.md` (generated — regenerate, don't hand-edit).
- Domain group worth knowing: TU Delft AWE group (Schmehl), github.com/awegroup
  — VSM, awebox (trajectory optimization on CasADi), and the V3 dataset.
- Design idiom of this project: **YAML spec → derived properties → solver →
  report, gated by tests.** Keep new capability inside that loop. When a
  solver contradicts a drawing or a claim, the solver output + a test wins,
  and the drawing gets a sync task.
