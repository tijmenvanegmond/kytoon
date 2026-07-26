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

The ship, arm, and operational design live as two SVG
drawing sheets under docs (`kiteship-ga.svg`, `kytoon-iterations.svg`) and a narrative
log (`kiteship-project-log.md`). This repo owns the *numbers*.

### The four archetypes (specs/*.yaml)

| Mk | Name | Archetype | Role | Key mechanism |
|----|------|-----------|------|---------------|
| I | Sled | `lei` | launch boost + traction | C-shape LEI, He in LE tube + 8 struts |
| II | Helikite | `helikite` | zero-wind ISR loiter | oblate He lobe + 2 braced side deltas (v2) |
| III | Spine | `spine` | boost + drone perch/recharge | inflatable keel spar carrying grapple rail + 300 kg dock |
| IV | Torus | `torus` | calm-air recovery node | He ring, capture line through central duct |
| V | Manta | `fatwing` | single-kytoon challenger | lofted multi-cell He fat wing, 3-pt tether (main + 2 control) |

(Retired alternates live in `specs/alternates/` — currently the original
winged-blimp Manta (`blimp` archetype, Mk V-A). They stay solvable and
tested but are NOT part of the fleet gates.)

**Fleet logic (load-bearing design decision, now CHALLENGED):** the
original logic — no single Mk covers the operational wind range, so the
ship carries a buoyant type (II/IV) plus a traction type (I/III) — is
encoded in `test_fleet_covers_zero_to_20ms`. **Mk V (fat-wing v2,
2026-07-11) solves 0–23.1 m/s alone at L0 with 18 kN tow @ 12 m/s**
(`test_mk5_single_kytoon_coverage`) — unlike its blimp predecessor it
challenges BOTH slots: calm-air ISR (buoyant, +232 kg incl. tether) and
a meaningful share of the traction role (18 kN vs Mk I/III's ~28). Caveats:
its cl_op 0.7 / cd_op 0.10 are hand-picked; the Breukels section model is
extrapolated at t/c 0.28 (flagged at L1); control-tether authority is
asserted, not analyzed (L2/dynamics).
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
kytoon/solvers/l1_aero.py  L1 aero: parametric C-arc LEI wing from the spec's
                        bulk numbers → VSM (awegroup) polar. Needs the `l1`
                        extra; everything else runs without it.
kytoon/solvers/l1_tether.py  L1 tether: MoorPy quasi-static line in air
                        (drag + sag + true elevation). Also `l1` extra.
kytoon/solvers/l1_body_aero.py  L1 hybrid aero: wing+body hybrids (Mk II;
                        Mk V-A blimp alternate) through AeroSandbox
                        AeroBuildup — bounds, not benchmarks.
kytoon/solvers/l1_trim.py  L1 trim/stability: Mk V 3-line rig — closed-form
                        taut-taut trim map, steering envelope, winchlet
                        budget, eigen stability. Also `l1` extra.
kytoon/solvers/l1_mass3d.py   L1 lateral Stage 1: inertia tensor + 6×6
                        added-mass matrix, dihedral-parameterised.
                        Reduces exactly to l1_trim's planar terms.
kytoon/solvers/l1_lat_aero.py L1 lateral Stage 2: CY/Cl/Cn_β from an
                        AeroBuildup β sweep, rate derivatives by strip
                        theory, and the winchlet steer response.
kytoon/solvers/l1_rig3d.py    L1 lateral Stage 3: 3D force closure with
                        the two control lines separated — yaw stiffness
                        of the bridle, differential-trim steering.
kytoon/solvers/l1_dyn3d.py    L1 lateral Stage 4: 12-state linearisation
                        and modal analysis. Validates the stack (exact
                        longitudinal/lateral decoupling, roll mode as a
                        probe of added inertia) and finds the lateral
                        divergence. Statics-linear; no time domain yet.
kytoon/aero.py          TU Delft V3 benchmark loader + system-polar model.
kytoon/report.py        CLI: python -m kytoon.report specs/ -o reports/l0.md
kytoon/viz.py           CLI: python -m kytoon.viz specs/ -o reports/figures
                        (L0 figures always; polar/tether figures need `l1`)
kytoon/geometry.py      3D kernel: spec → trimesh scene → models/*.glb|stl.
                        Owns ArcWing (shared with l1_aero). Needs `l1`.
data/tudelft_v3/        vendored CC-BY benchmark (see SOURCE.md for citations).
                        Treat as read-only; re-download from awegroup if stale.
tests/test_l0.py        14 tests = the L0 validation contract (see §4).
tests/test_l1_aero.py   11 tests = the L1 aero contract; VSM-dependent ones
                        skip unless the `l1` extra is installed.
tests/test_l1_tether.py 8 tests = the L1 tether contract (skip w/o moorpy).
tests/test_geometry.py  8 tests = mesh volumes must match spec-derived
                        properties; arc shape must honor le_tube.length.
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
   margin found by bisection on tow force, canopy fabric limit, envelope
   dent). The dent limit (added 2026-07-11): a pressurized envelope loses
   shape once q exceeds its superpressure — lobe/hull use the assumed
   500 Pa (→ 28.6 m/s), torus its spec 2000 Pa. Quasi-static only —
   **no gust cases, no crosswind maneuvers, no dynamic loads.**
5. **Tether**: straight line; mass counted, drag and sag NOT integrated
   at L0. Torus v_max uses bluff-body drag (Cd 0.5 × ring frontal area)
   against tether WLL. The L1 correction exists (`l1_tether.py`, MoorPy);
   L0 keeps the straight-line story and the spec keeps `elevation_deg` as
   an input — see §6 for why that input is suspect.
6. **Torus structure**: hoop check only. Ring bending under the 3-point
   bridle is an L1 problem — flagged, not solved.
7. **Mk II lobe**: 500 Pa assumed gust superpressure for the hoop check;
   its wing coefficients (cl_op 0.6, cd_op 0.25 for lobe-wake degradation)
   are hand-picked and NOT covered by the benchmark (§5 caveat).
8. **L1 aero geometry** (`l1_aero.py`): wing is a circular C-arc with the
   V3's shape defaults (height/span 0.376, parabolic taper 0.55), flat
   sections — no twist, no billow (that's membrane FEM, task §7.2). Section
   aero is Breukels' 2-param LEI regression; twin-skin (Mk III) is
   approximated by a slim tube (t = 0.06) and flagged conservative. Measured
   pipeline error vs the vendored wind tunnel: CL_max +10%, (L/D)max −19%
   — both gated in tests.

---

## 4. The test suite is a contract

127 tests across test_l0, test_l1_aero, test_l1_tether, test_viz,
test_geometry, test_l1_body_aero, test_l1_trim, and the lateral stack
test_l1_mass3d / test_l1_lat_aero / test_l1_rig3d / test_l1_dyn3d — all
passing at last compile. Categories:

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
- **Trim/stability gates** (test_l1_trim.py): Mk V's 3-line rig must trim
  feasibly at cl_op, steer α from ≤1° to ≥12° at 6/12/20 m/s, hold a
  depower schedule 4→23 m/s under 75 % WLL with < 8 kN control tension
  and < 1 m winchlet travel; with the 50 m pod the rig must be passively
  stable (all modes damped, drift at most neutral) and a locked-winch
  +50 % gust must keep ≥5° stall margin and return to trim; the ship-rig
  regression (undamped drift without the pod) is also gated, as are the
  closed-form-vs-dynamics anchor and the single-confluence-instability
  and near-vertical-tow flags. Capture-hover hang statics (2026-07-24):
  the zero-q hang anchor (hand formula), the wind-on hang matching the
  Godot sim's recovery endgame (cross-model regression, ±4°), and one
  gate per rigging option — TE pendant levels the hover (0.4–1.2 kN,
  main stays loaded), locked ctl drum can pin level (thin main margin),
  and the hang-leveling attach station cannot fly the mission.
- **Lateral dynamics gates** (test_l1_dyn3d): longitudinal and lateral
  must decouple to finite-difference noise; the generalised mass must
  stay symmetric positive-definite; the roll mode must match √(k/I)
  within 5 % *and* sit >3× from the no-added-inertia value; the
  longitudinal block must stay damped; and the lateral divergence is
  gated **as a finding** — if a fin lands in the spec or CY_β is
  revised, that test fails and demands a re-read rather than passing in
  silence.
- **Lateral gates** (test_l1_mass3d/lat_aero/rig3d): every planar term
  must come back unchanged at Γ = 0 (the reduction is the main guard on
  the whole lateral stack); roll added inertia > 8× structural; roll
  damping must match the closed form for a tapered wing; the wing must
  stay weathercock-unstable and the bridle must beat it by > 5×;
  differential trim must steer, monotonically and with coherent signs;
  and α/β extraction is checked against AeroSandbox's own freestream
  direction, since that convention is where this model is easiest to
  get silently wrong.
- **L1 pipeline gates** (test_l1_aero.py): the parametric-V3-through-VSM
  polar stays inside its measured error bands (CL_max ±15% of tunnel,
  (L/D)max in [−25%, +10%]); Mk I reaches cl_op pre-stall on its *own*
  geometry; Mk I resultant coefficient within 20% of spec; Mk II lobe and
  Mk III twin-skin approximations are flagged; Mk IV refuses (no wing).

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
  v_min ≈ 3.8 m/s instead. (The iteration sheet's old "≈ 0 static" claim
  was fixed in REV B, 2026-07-08.)
- **Mk III spar was resized by the solver**: original Ø0.6 m @ 0.4 bar
  wrinkled at 316% under dock load alone; now Ø0.8 m @ 0.6 bar with the
  5-fixture rail doubling as bridle nodes → 48%.
- **Mk I is wrinkle-limited (13.7 m/s), not tether-limited.** Knobs: bridle
  density, tube pressure.
- **Drawing-level finding**: the arm's 15 m reach envelope overlaps the
  tether traveller zone — arm motion planning must treat the tether as a
  dynamic keep-out volume. Unresolved, lives with the ship design.
- **Mk V «Manta» covers the requirement alone (2026-07-08)**: 648 m³
  prolate hull + 130 m² side deltas → +469 kg net static, 0–26.1 m/s
  tether-limited at L0.
- **Mk II design iteration: tether was the binding limit (2026-07-11)**:
  the 12 mm/110 kN line capped the envelope at 19.2 m/s while the canopy
  fabric ceiling sits at ~48. Upgraded to Mk III's 16 mm/200 kN class
  (winch/termination commonality): envelope now 0–25.9 m/s (Mk V parity)
  for 20 kg of net lift (534 → 478 incl. tether). Spec + iteration sheet
  (REV C) updated. Next binding limit is still the tether; going past
  ~29 m/s (18 mm) costs commonality and lift for little ISR value.
- **Mk V pivot: winged blimp → lofted fat wing (2026-07-11)**: a manta —
  NACA-form sections lofted along a linearly tapering span (center chord
  13.3 m → 35% at the tips, t/c 0.28, quarter-chord swept 15°), one
  pressurized body with 5 chordwise cells; 3-point tether (main + 2
  control lines at the bridle stations). A tube-bundle representation was
  tried first and rejected — tubes can't produce the fat-center/thin-tip
  shape; the loft's volume stays closed-form (K_A·t·c section area over
  the taper, gated against the mesh). Structure: cell-web pitch sets the
  skin-bulge hoop (11%), section depth the equivalent-beam wrinkle (10%
  at 12 m/s; the L0 loop caught the original 0.2 bar hoop bust). Card:
  530 m³, **+232 kg incl. tether, 0–23.1 m/s (tether WLL, no dent
  limit), 18 kN tow @ 12 m/s** — the first airframe to credibly straddle
  both fleet slots. Breukels extrapolation at t/c 0.28 flagged at L1;
  control-tether authority asserted, not analyzed. The blimp survives as
  Mk V-A in `specs/alternates/`.
- **Mk IV's headline ceiling is a tension number, not a mission number
  (2026-07-11)**: with no wing, fixed buoyancy fights v²-growing drag —
  L1 tether blow-down: 83° elevation / 398 m altitude at 5 m/s degrades to
  44°/280 m at 15 and 20°/136 m at 25. Its *useful* envelope (elevation
  ≥ 45°, capture line workable) is ≈ 0–14 m/s, not the 0–44 the L0 table
  suggests. Mk II v2 holds ≥ 64°/360 m through 25 m/s because wing lift
  grows with q. Candidate improvement: add a "mission ceiling"
  (elevation-floor) metric to the envelope solve so the comparison table
  stops flattering drag-only aerostats.
- **Mk II v2: side deltas replace the under-lobe keel sheet (2026-07-11,
  REV D)**: a single 250 m² unsupported fabric surface under the lobe
  cannot hold shape — no load path to its trailing edge. Replaced with two
  44 m² deltas rooted on the lobe equator with tip stays (same config
  family as Mk V). Smaller wing in cleaner side flow: cl_op 0.55 /
  cd_op 0.30 on 88 m². Net static rose to +558 kg (less canopy mass);
  ceiling is now the **lobe dent limit 28.6 m/s** — an airframe limit,
  not the line — making Mk II the highest-ceiling buoyant kytoon. Tow
  fell to 4.9 kN @ 12 m/s (irrelevant to the ISR role). AeroBuildup
  bounds the new coefficients at resultant ratio 0.88.
- **Body-interference study bounds the Mk II/V coefficients (2026-07-11,
  task 5)**: AeroSandbox AeroBuildup (rigid smooth bodies — a lower drag
  bound). Mk V: cl_op 0.65 at α≈7°, L1 system cd 0.059 vs the hand-picked
  0.18 (spec is very conservative); resultant ratio 0.97. **The
  single-kytoon claim holds at BOTH bounds** (v_max 26.1 spec / 26.6 L1,
  gated in tests). Mk II: cl_op 0.6 at α≈9°, resultant ratio 0.93 —
  consistent, though wake blanketing is not modeled (CL upper bound). The
  big cd disagreements mostly move L/D and hence tether elevation, not the
  envelope, because cl dominates the resultant. Two-kytoon carriage is now
  formally optional for ISR/recovery per L1 bounds; the traction role
  (launch boost, 28 kN-class tow) still needs Mk I/III. Certifying (not
  bounding) the hybrids needs real hybrid-aerostat data or L2 CFD.
- **Mk I's spec implies a flatter arc than the V3 (2026-07-08)**: the
  44 m developed LE-tube length on a 38 m span pins the C-arc at
  height/span ≈ 0.25 vs the V3's 0.376. `ArcWing.from_spec` now derives
  the arc from the tube length when present; Mk I's L1 operating point
  barely moved (cr_ratio 0.99 → 1.00, op α 8.9° → 9.3°) — the spec
  coefficients still hold on the self-consistent shape.
- **Tether elevation is an output, and the spec inputs are wrong
  (2026-07-08)**: quasi-static force balance puts Mk I's tether at ≈78°
  elevation at the operating point (atan of system L/D ≈ 5.3, sag ≈ 1 m at
  27 kN), not the spec's 40°; Mk IV rides at ≈57° at 12 m/s, not 75°.
  L0's `eta_v = sin(elevation_deg)` therefore *underestimates* Mk I/III
  vertical lift (v_min would drop ≈3.8 → ≈3.1 m/s with eta_v from force
  balance) and *overestimates* Mk IV's. OPEN DESIGN CONVERSATION: either
  make elevation an L0 output (change §3.4's model + specs + gates) or
  re-justify `elevation_deg` as an operational constraint (winch/traveller
  geometry), not a physics input. Until decided, L0 numbers stand.
- **Mk V pitch trim: a single-confluence bridle CANNOT fly it, the 3-line
  rig can (2026-07-24)**: static moment balance on the actual loft (CB
  3.24 m aft of center c/4 vs aero center ≈1.2 m aft; symmetric untwisted
  sections → Cm₀ = 0) shows every stable pull point trims nose-down
  (CL < 0) and every positive-CL trim is unstable — the He centroid sits
  ~2 m behind any stable pivot and nothing aerodynamic counters it. The
  asserted 3-line architecture rescues it: with main + control lines taut
  to a common fairlead, pitch is geometrically pinned. Closed-form
  taut-taut trim (`l1_trim.py`): α commandable ≈ 0–18° at every wind
  4–23 m/s, control-pair tension only 3–4 kN (2×8 mm lines, 8× margin),
  winchlet travel ≈ 3.4 m across the whole envelope; linearized 6-state
  eigenvalues at 12 m/s: pitch −1.7±3.1j, pendulum −0.27±0.45j, one slow
  drift +0.07 /s (winch-loop territory). A +50 % 1-cos gust at 12 m/s
  peaks at 38 kN (57 % WLL) even with winches locked, but α excursions
  reach ~24° → commanded-α ceiling 14° in the schedule. Control-tether
  authority is now ANALYZED (was: asserted). New spec fields:
  `bridle.chord_fraction` (0.35) and `bridle.control_mbl_kn`. Caveats:
  AeroBuildup Cm at t/c 0.28 (bounds, not certification), longitudinal
  plane only, tether drag/sag not coupled.
- **Winchlet pod on the tether makes Mk V passively stable (2026-07-24)**:
  moving the control winchlet from the ship to a pod riding the main line
  50 m below the kite (spec: `bridle.pod_standoff_m: 50`) leaves the pitch
  moment arm unchanged (that is attach geometry, 2.6 m) but multiplies the
  control-path stiffness by L_tether/L_ctl ≈ 7.7× (55 vs 7 kN/m pair):
  elastic pitch pinning goes from 1.3× to 10× the passive divergence.
  Consequences, all gated: every eigenmode damped (the ship rig's +0.07/s
  drift mode vanishes — winchlet becomes a trim actuator, not a
  stabilizer); locked-winch +50 % gust at 12 m/s peaks at α ≈ 15° (8°
  stall margin vs 0° for ship winches) and returns to trim unaided;
  winchlet travel over the whole 4–23 m/s schedule drops 3.4 → 0.75 m at
  ≤ 4.2 kN. Also deletes 2×350 m of control-line drag/weight (~28 kg)
  for a pod of comparable mass hanging on the line. Open: pod swing mode
  (modeled as riding the line rigidly), pod power/data.
- **Capture-hover hang is closed-form, and the rigging shortlist is
  solved (2026-07-24)**: at zero q a hanging buoyant body settles where
  the gravity+buoyancy resultant passes through the loaded attach —
  algebra, not dynamics (`l1_trim.hang_trim`). Mk V's current rig hangs
  **−41° nose-down** (−54° with the 5 m/s residual wind — matching the
  Godot sim's recovery endgame within 4°, which was still converging at
  cutoff; the sim's −57° "mystery" is just this hang). Spanwise stations
  can't pin pitch when aero dies. Options, each one evaluation:
  (a) moving the attach to the hang-leveling station f = 0.47 is
  FALSIFIED — flight trim infeasible there (gated); (b) **aft capture
  pendant** at ~0.95c: 0.78 kN holds a level hang with 2.0 kN left on
  the main — cheapest robust fix, and net buoyancy is what keeps both
  lines loaded; (c) **lock the ctl drum through docking** instead of
  tension-tending: the 15° sweep puts the outboard stations 2.6 m aft
  of the main attach, so the pair can pin a level hang at T_ctl ≈
  2.4 kN — zero new hardware but T_main margin only ~0.4 kN. Ranking:
  b > c > a. Still open for the sim pass (statics can't answer it):
  the pendulum excursion envelope in ship frame on ~20 m of line — the
  spec the capture arm's motion planner actually needs.
- **A winchlet CAN steer the Manta — the lateral model, Stages 1–3
  (2026-07-25)**: the founding question was always lateral (differential
  trim on two lines at ±0.38 span is a roll moment, which the
  longitudinal model structurally cannot represent), so it needed 3D.
  Three new solvers, all reducing exactly to the planar model and gated
  against it: `l1_mass3d` (inertia tensor + 6×6 added mass),
  `l1_lat_aero` (β sweep through AeroBuildup for CY/Cl/Cn; rate
  derivatives by strip theory), `l1_rig3d` (3D force closure, two
  control lines separated). Findings:
  * **Roll is added-mass dominated 11.9×** (129 400 vs 10 880 kg·m²) —
    πρc²/4 picks up a y² lever in roll that pitch has no analogue for.
    A lateral model that skips the 6×6 rolls an order of magnitude too
    fast. Sideslip added mass is *structurally* zero in strip theory
    (no spanwise term), so sway inertia is bare airframe mass.
  * **Steering works and is DAMPING-limited, not inertia-limited.**
    Cl_p = −0.39 (−0.79 on the 2π bound). τ ≈ 0.3–0.5 s against
    140 t·m² of roll + added inertia, so the air sets the rate: at
    12 m/s, 2 kN of differential gives 3.3 °/s and 4 kN banks 30° in
    5 s, using an eighth of the control pair's WLL. Operationally this
    is a sustained-pull problem, not an impulse one. Roll rate falls
    as 1/V (damping ∝ V, couple fixed).
  * **The wing is weathercock-UNSTABLE (Cn_β = −0.005/rad)** — tailless
    and finless — **and the bridle supplies what it lacks**: two lines
    from a common pod to separated attachments give +108 kN·m/rad
    against the wing's −4.3, a 25× margin, net +103 stable. This is why
    the 3-line rig is load-bearing in yaw as well as pitch.
  * **Dihedral is a real trade, not a free win.** Γ multiplies steering
    authority (0.30 m differential: 7 m lateral at Γ=0 → 70 m at Γ=10)
    and roll stiffness (2.5× at Γ=30, from CB rising 2.8 m above the
    pull point), but it deepens the yaw divergence 13× and eats the
    bridle margin: 25× at Γ=0 → 3.0× at Γ=10 → 1.7× at Γ=20. Γ ≳ 25°
    would need a fin or a wider control-line base. Γ moves the CB
    *vertically*, essentially not chordwise, so it does NOT relieve the
    longitudinal instability finding below.
  Caveats: AeroBuildup lateral derivatives are semi-empirical and not
  benchmark-anchored; rate derivatives are strip theory reported with
  their 2π bound; the wing is treated as rigid, so fabric warp under
  asymmetric bridle load (which would only add authority) is omitted;
  statics only.
- **Mk V's LATERAL dynamics diverge — Stage 4 (2026-07-25)**: the 12-state
  linearisation (`l1_dyn3d`) validates cleanly and then reports a
  divergent lateral mode at **+1.49 /s (0.5 s doubling)** at 12 m/s,
  while the longitudinal block stays damped (−0.12 /s). Everything the
  planar programme established survives; the new information is
  entirely in the half it could not represent.
  The model earns the right to say so first: longitudinal and lateral
  decouple **exactly** (cross-participation ≤ 2e-6, i.e. to
  finite-difference noise), and the roll mode lands at 7.12 rad/s
  against the 7.33 closed form — 3 %, and 3.6× away from the value
  without added inertia, so Stage 1's contested term is confirmed
  dynamically.
  Two separable drivers:
  * **CY_β = +0.126/rad → negative sway damping.** A side force in the
    *same* direction as the slip, so the air feeds the motion:
    C[1,1] = +211 N per m/s, which alone is +0.79 /s — and that is
    exactly the floor the sensitivity sweep bottoms out at.
  * **Control-line geometry couples roll into sway and yaw.** Static
    condensation: the tether's sway restoring is −437 N/m, the roll path
    returns +433, leaving −4 N/m. The roll↔yaw loop gain is 0.90.
  Sign convention independently validated: adding a 20 m² aft fin drives
  Cn_β −0.005 → +0.097 and CY_β +0.126 → −0.023, exactly as a
  weathercock stabiliser must, so these are real predictions and not a
  frame error. **A fin is indicated** (20 m² cuts divergence to
  +0.38 /s) but does not cure it alone. The instability survives
  dropping control-line stiffness 100×, moving the control attachments
  chordwise, and changing pod standoff (longer standoff is *worse*:
  +3.0 /s at 200 m — the 50 m pod is the right choice laterally too).
  **Do not treat this as settled.** CY_β is a single semi-empirical
  derivative carrying most of the result, real tethered wings of this
  class do fly, and a 1.3 s doubling time would make one unflyable — so
  the magnitude is the prime suspect. Resolving it needs an independent
  CY_β (VSM with sideslip, or hybrid-aerostat data) before either the
  problem or the fin is designed around. Gated as a finding in
  `test_l1_dyn3d` so a change surfaces as a failure, not silence.
- **Mk V recovery procedure, from the Godot sim layer (2026-07-24)**:
  full winch-in 400 → 20 m at 5 m/s (godot/mkv_sim.gd
  `--recovery-test`, a validated GDScript port of l1_trim's dynamics
  with k = EA/L line law) only works as: tension-governed main reel
  (speed steps pogo the elastic line — ζ ≈ 0.16, observed 60–150 kN
  snap loads), α-hold on the winchlet rather than attitude-hold
  (2 m/s descent in 5 m/s wind adds ~22° inflow — θ must go nose-down
  in descent), and no depower (buoyant kite descends under winch pull;
  slack control lines = no attitude authority). Clean run: 190 s,
  T ≤ 10 kN, buoyant hover at ~23 m. OPEN: hover attitude after the pod
  docks is softly constrained (tended drum, near-zero q) — capture-state
  attitude needs pod-as-node dynamics or L2. Details in godot/README.md.
- **Capture hover levels on a constant-tension tend (2026-07-25, Godot
  sim)**: the docked winchlet drum holding 3 kN on the control pair puts
  the hover at θ ≈ +2° (vs the −41° untended static hang), tension ≤ 20 kN
  through the descent. So the tension-tend rigging option works and the
  aft capture pendant is not the only route to a level hover — both stay
  on the table. Two model errors had hidden this: the tend was a spring
  around a shifting rest length whose damping cancelled it (dead-slack
  control lines through the whole hover, kite ditched), and the aero table
  simply CLAMPED outside its validated −8…24° band, so a stalled wing kept
  gliding on CD ≈ 0.05 at α = −60° and every slack-line upset became an
  unrecoverable dive. The sim now blends to a flat plate outside the band
  (CD → 1.9) and flags it on the HUD; the Python solvers are unchanged and
  the parity gate still matches to 0.02° in α. Related: free-flying, the
  airframe's static margin is **−1.83 m (−20 % MAC)** — CG 2.89 m vs
  neutral point 1.06 m — so losing line tension always ends in a pitch
  departure. Another reason the 3-line rig is load-bearing.
- **Mk V tow is nearly vertical (2026-07-24)**: the same trim map puts the
  line at 83–87° elevation across the envelope (buoyancy + high bound-L/D)
  — at 12 m/s, 24 kN line tension is only ~3–5 kN of *horizontal* pull.
  The "18 kN tow" headline is line tension, not traction; whether that
  suffices for the launch-boost role is part of the open §6 elevation
  conversation (real drag is higher than the bound → real elevation lower,
  real horizontal tow higher).
- **L1 confirms the hand-picked aero (2026-07-08)**: solving Mk I's and
  Mk III's *own* parametric geometry with VSM reproduces the spec operating
  points — resultant-force ratio L1/spec 0.99 (Mk I, cl_op 0.8 at α≈8.9°)
  and 1.00 (Mk III, cl_op 0.9 at α≈11.2°). Mk I L1 cd_op 0.121 vs spec 0.15
  (spec slightly conservative); Mk III L1 cd_op 0.149 vs spec 0.12 (spec
  slightly optimistic, flagged — twin-skin section model is conservative).

---

## 7. Task queue (priority order)

1. ~~**L1 aero**~~ — v1 DONE 2026-07-08: `kytoon/solvers/l1_aero.py` builds a
   parametric C-arc wing per spec and solves it with awegroup VSM (Breukels
   sections), `solve(spec) -> L1AeroReport`. Validated against the vendored
   tunnel data (§3.8 error bands). Result: Mk I and Mk III spec coefficients
   hold on their own geometry (resultant ratio 0.99 / 1.00). Remaining for
   v2: geometric twist/billow (couples to §7.2), feed L1 polars back into
   the wind-envelope solve, Mk II lobe (→ task 5).
2. **L1 structure**: mem4py membrane FEM to calibrate/replace
   TUBE_LOAD_SHARE (§3.3). Success criterion: a computed load-share value
   with a validation case, replacing the 0.35 constant.
   **BLOCKED 2026-07-08 on this dev machine**: mem4py is Cython+Eigen,
   git-only and dormant (23 commits, no releases); no MSVC C++ toolchain
   installed here, and it needs gmsh surface meshes. Task 6's kernel now
   emits watertight STL (a start), but mem4py wants gmsh .msh with physical
   groups — still to do. Unblock paths: (a) install VS Build Tools or use
   WSL for a Linux-side build, (b) extend the geometry kernel to gmsh.
3. ~~**L1 tether**~~ — v1 DONE 2026-07-08: `kytoon/solvers/l1_tether.py`
   runs the tether as an inverted mooring line in air (MoorPy, System
   rho=1.225, wind as current). Adds line drag + sag, computes true
   elevation/tension/altitude, and `v_max_tether()` bisects the
   drag-corrected WLL ceiling. Gated by 8 tests. Key outputs: Mk I tether
   ceiling 16.1 m/s (L0 straight-line: 15.8 — buoyancy deficit unloads the
   line slightly), Mk IV 44.1 vs 44.4. Remaining for v2: feed corrected
   elevation back into the L0 envelope (blocked on the §6 elevation
   design conversation).
4. ~~**Sync the drawings**~~ — DONE 2026-07-08 (`docs/kytoon-iterations.svg`
   REV B): spec table, He volumes, static lifts, wind envelopes, tow forces,
   Mk I strut count/span/AR, Mk III spar callout (Ø0.8 m @ 0.6 bar) and
   5-fixture rail all synced to `reports/l0.md`. Re-sync whenever the solver
   numbers move — the sheet now cites REV B provenance in its footer.
5. ~~**Mk II + Mk V body-interference aero**~~ — v1 DONE 2026-07-11:
   `kytoon/solvers/l1_body_aero.py` (AeroSandbox AeroBuildup, wing + body
   of revolution, bridle drag added). Outcome in §6: Mk V's single-kytoon
   claim holds at both bounds; Mk II consistent on resultant. Remaining to
   *certify* rather than bound: hybrid-aerostat benchmark data (none found
   vendorable yet) or the L2 CFD tier; lobe wake blanketing unmodeled.
6. ~~**Geometry kernel**~~ — v1 DONE 2026-07-08: `kytoon/geometry.py`
   realizes each spec as a trimesh scene (closed meshes for pressurized
   volumes, open surfaces for soft goods) → `models/*.glb|stl`. Mesh
   volumes are gated against the spec-derived properties; the arc shape is
   pinned by the spec's own LE-tube length (see §6 finding). Remaining for
   v2: gmsh FEM-grade meshing (what mem4py actually needs), bridle-line
   geometry, billow/twist once membrane results exist.
7. **L2 (later)**: OpenFOAM ↔ CalculiX/FEniCSx via preCICE for gust and
   capture-state (arm-attached) load cases, final candidate only.
8. ~~**Mk V trim & steerability**~~ — v1 DONE 2026-07-24, pod rig
   2026-07-24: `kytoon/solvers/l1_trim.py` (closed-form taut-taut trim
   map, eigen stability, winchlet budget, `bridle.pod_standoff_m`
   support; findings in §6; locked-winch gust now gated in
   test_l1_trim). Remaining for v2: pick the capture rigging (pendant
   vs drum-lock, §6 hang finding) and sim the winner's transient into
   the hover + the pendulum excursion envelope in ship frame (the
   capture arm's chase spec); pod swing-mode dynamics (pod as its own
   node); lateral/roll + yaw (steering, figure-eights vs the
   near-vertical-tow problem); couple l1_tether drag/sag into the trim
   map; cross-check the Godot sim's lumped-mass segmented tether
   (godot/mkv_sim.gd: sag/weight/drag/honest slack) against l1_tether's
   MoorPy statics; Godot replay of new trajectories (viewer exists:
   `godot/mkv_replay.*`).

---

## 8. Wider-context pointers

- Companion artifacts (in `docs/`): `kiteship-ga.svg` (ship GA),
  `kytoon-iterations.svg` (Mk sheet, REV B), `kiteship-project-log.md`
  (narrative log — historical snapshot, don't retro-edit). The generated
  solver output lives at `reports/l0.md` (regenerate, don't hand-edit).
- Domain group worth knowing: TU Delft AWE group (Schmehl), github.com/awegroup
  — VSM, awebox (trajectory optimization on CasADi), and the V3 dataset.
- Design idiom of this project: **YAML spec → derived properties → solver →
  report, gated by tests.** Keep new capability inside that loop. When a
  solver contradicts a drawing or a claim, the solver output + a test wins,
  and the drawing gets a sync task.
