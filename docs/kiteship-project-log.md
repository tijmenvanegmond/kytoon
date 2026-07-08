# Kiteship — Project Log

*Compiled from a design conversation, 4 July 2026.*
*Concept: an uncrewed kite-drone hybrid system for ship-based drone launch, recovery, and ISR.*

---

## 1. Concept

Combine three proven ideas into one system: SkySails-style towing kites, tethered-aerostat ISR, and Skyhook-style rope recovery (ScanEagle). The kite is a **kytoon** — leading-edge-inflatable (LEI), helium-assisted for static buoyancy — that does three jobs at once:

- **Launch boost** — a drone rides a powered trolley up the tether and releases into clean air above the ship's turbulent boundary layer, saving its most battery-expensive climb phase.
- **Recovery** — the kytoon holds a capture line aloft (Skyhook principle); a fixed-wing drone snags it and gets winched down. The kytoon's compliance absorbs the impulse better than a rigid mast.
- **ISR** — a gimbaled EO/IR pod at altitude gives a much longer horizon than a ship's mast, silently and without fuel burn.

Kite lift scales with area, and area is soft goods — going from 50 m² to 400+ m² is a bigger winch, not a new discipline (SkySails flew 400 m² traction kites off cargo ships). This reframes the concept as a **kiteship class**: a vessel designed *around* the kite system rather than a kite bolted onto a ship, collapsing hangar + catapult + arrestor gear into soft goods, a winch, and a manipulator arm.

---

## 2. Ship concept: uncrewed 40 m trimaran

Settled form factor: **40 m LOA, 16 m beam, side pontoons (amas)** — the same class as DARPA's Sea Hunter, so hull-plus-autonomy is proven territory.

Why a trimaran specifically:
- **Stability** — 15–20 m effective beam makes a long-reach arm swinging a kytoon a rounding error in the stability budget, not a design driver.
- **Roll damping** — a quieter base for capture control.
- **Pontoon volume** — helium reserve, fuel, and trim ballast (to counteract the kite's lateral pull, the way SkySails' cargo ships used sheer displacement).
- **Tether traveller** — a mainsheet-traveller-style attachment spanning the aft cross-beam lets the pull point move windward (propulsion), centerline (hover/ISR), or leeward-aft (recovery) — geometry control a monohull doesn't have.

**Capture concept:** a 7-DOF manipulator arm on a rail, reaching a kytoon that's been winched down to ~20 m and is holding station under helium buoyancy (quasi-static — a deflated ram-air kite would be an uncontrollable falling object; a kytoon under tension is docile). The kytoon carries a passive grapple fixture on the leading-edge underside (ISS-grapple-fixture division of labor: kytoon = passive, arm = active end-effector).

**General arrangement** (see `kiteship-ga.svg`): winch pit and kite stowage bay aft in the center hull, arm on a 12 m rail amidships (15 m reach sweeps both the kytoon-capture zone and the forward drone garage), four drone cells + service bay under the foredeck, tether traveller on the aft cross-beam.

*Design note surfaced by the drawing:* the arm's reach envelope necessarily overlaps the tether traveller — arm motion planning must treat the tether as a dynamic keep-out zone. Not resolved, flagged for the control-system design phase.

---

## 3. Kytoon iterations — Mk I through Mk IV

Four archetypes spanning the buoyancy-vs-aerodynamics trade space (see `kytoon-iterations.svg`):

| | Archetype | Role | Static lift | Wind envelope |
|---|---|---|---|---|
| **Mk I «Sled»** | Baseline LEI traction wing | Launch boost + traction | ≈ self-neutral only | needs wind |
| **Mk II «Helikite»** | Oblate He lobe + delta keel wing | Persistent ISR loiterer | +450 kg, flies at 0 wind | 0–14 m/s |
| **Mk III «Spine»** | Semi-rigid inflatable keel spar, twin-skin canopy | Boost + drone perch/recharge dock | slightly negative | widest range, 6–25 m/s |
| **Mk IV «Torus»** | Annular He ring, capture line through central duct | Calm-air, omnidirectional recovery node | +500 kg | 0–12 m/s |

Key design decisions:
- **Mk III's grapple rail** (3–5 fixtures along the spar) lets the arm capture from any bearing, and doubles as bridle attachment nodes.
- **Mk IV's duct** self-centers a drone onto the capture line and needs no heading control at all — immune to wind-shift upsets at anchor.
- No single kytoon covers the full 0–25 m/s envelope; the operational answer is to carry **two kytoon types** (a buoyant one + Mk I or III), which drove the "common interfaces" requirement: tether termination/load cell, grapple fixture geometry, IMU/GNSS + He telemetry, and capture-line hardpoint are standardized across all four Mks.

---

## 4. Simulation fidelity ladder

Membrane kites are a coupled fluid–structure problem (shape depends on pressure field, which depends on shape) — full FSI is overkill for early iteration. Adopted a three-tier ladder:

- **L0 — analytic (minutes):** Archimedes buoyancy, pressurized-beam stress (hoop N = p·r, wrinkle onset M_w = p·π·r³/2 per Comer & Levy), quasi-static wind envelope. Answers "does this survive" cheaply — built and running (§5).
- **L1 — potential flow + linear FEM (hours):** AeroSandbox (VLM), mem4py (membrane FEM, built for kites), MoorPy (tether as an upside-down mooring line). Not yet built.
- **L2 — coupled FSI (days):** OpenFOAM + FEniCSx/CalculiX via preCICE, only for the final candidate's gust/capture-load cases.

Python was chosen deliberately over the team's usual TypeScript — the entire simulation ecosystem (AeroSandbox, mem4py, MoorPy, preCICE, TU Delft's own tools) lives there.

---

## 5. L0 simulation — `kytoon-sim`

A working repo (`kytoon-sim.zip`): YAML spec per Mk → pydantic schema → closed-form solvers → markdown comparison report, with a pytest suite as the validation gate.

```
kytoon-sim/
├── specs/*.yaml          Mk I–IV design specs (geometry, pressures, tether, bridle, masses)
├── kytoon/
│   ├── spec.py           pydantic KytoonSpec — volumes/masses as derived properties
│   ├── aero.py           TU Delft V3 polar loader + system-polar (bridle drag) model
│   ├── solvers/l0.py     buoyancy, tube stress, wind envelope solvers
│   └── report.py         comparison table + per-member margins + flags (CLI)
├── data/tudelft_v3/      vendored open benchmark data + SOURCE.md citations
├── tests/test_l0.py      14 tests: physics anchors + benchmark calibration gates
└── reports/l0.md         generated comparison report
```

**Findings from the first L0 pass** (iteration in action — several sheet claims turned out wrong on first contact with the numbers):
- Mk I is *not* self-neutral as originally specced — 37 m³ of He nets only 39 kg against a 185 kg kite. Needs ~180 m³ (a ~2 m LE tube) to actually hover unpowered.
- Original Mk III spar (Ø0.6 m, 0.4 bar) wrinkled at 316% utilization under the drone-dock load alone, before any tow force — resized to Ø0.8 m / 0.6 bar with the fixture rail doing double duty as 5 bridle nodes → 48% utilization.
- Mk I's actual ceiling is wrinkle-limited at 13.7 m/s, not tether-limited — a design knob (bridle density, tube pressure), not a hard ceiling.
- Fleet-coverage check passes: Mk II/IV floor at 0 m/s, Mk III reaches 18.5 m/s, confirming the two-kytoon carriage logic numerically.

**Current comparison table** (full report: `kytoon-l0-report.md`):

| Parameter | Mk I «Sled» | Mk II «Helikite» | Mk III «Spine» | Mk IV «Torus» |
|---|---|---|---|---|
| Wing area [m²] | 400 | 250 | 350 | 0 |
| He volume [m³] | 37 | 718 | 5 | 675 |
| Flying mass [kg] | 185 | 218 | 258 | 245 |
| Net static lift [kg] | −146 | 534 | −252 | 461 |
| Calm-air capable | ✘ | ✔ | ✘ | ✔ |
| v_min / v_max [m/s] | 3.8 / 13.7 | 0.0 / 19.2 | 4.9 / 18.5 | 0.0 / 44.4 |
| v_max limiter | spar/LE wrinkle | tether WLL | tether WLL | tether WLL (drag) |
| Tow @ 12 m/s [kN] | 28.7 | 14.3 | 28.0 | 0.0 |

---

## 6. Aerodynamic calibration against TU Delft's open LEI benchmark

TU Delft's Airborne Wind Energy group (awegroup) publishes the **V3 LEI kite dataset** openly (CC-BY, github.com/awegroup/TUDELFT_V3_KITE) — wind-tunnel and CFD polars for a 25 m² kite with projected AR 3.498, close to Mk I's ~3.6.

Vendored into the repo (`data/tudelft_v3/`) and wired into `kytoon/aero.py`:
- **Wind tunnel** (Poland et al. 2026, *Wind Energy Science* 11, 911): 1:6.5 rigid model, Re 5e5, α from −11.6° to 24.5°. CL_max ≈ 1.07 @ 18°, clean-wing (L/D)_max ≈ 8.7 @ 9°.
- **CFD RANS with struts** (Viré et al. 2022, *Energies* 15, 1450): Re 1e6. CL_max ≈ 1.35 @ 19°.

A bridle parasitic-drag term (bluff-body line drag, scaled from the V3's 82-line/96 m system) converts the clean-wing polar into a system polar.

**Result: the benchmark validated the hand-picked coefficients rather than overturning them.** The original cl_op = 0.8 / cd_op = 0.15 for Mk I sit within 15% (resultant-force coefficient) of the calibrated operating point — CL 0.8 is reached at a comfortable pre-stall α ≈ 8°. The tow-force and wind-envelope numbers now carry a citable source. Test suite grew to 14 passing tests, including one that reproduces the published CL_max directly.

Caveats on record: Mk II's lobe-degraded wing and Mk IV (no wing at all) aren't covered by this benchmark; and the V3 is a *crosswind*-optimized AWE kite, so its 5.8 kN nominal pull is at crosswind apparent speed (~3× true wind) — not directly comparable to our static-lifter tow numbers.

**Natural next step:** the same awegroup repo ships their Vortex Step Method solver (open source) — the actual L1 aero tier, already validated against the data just loaded.

---

## 7. Open threads / next steps

- [ ] Build the L1 layer: AeroSandbox/VSM for Mk-specific polars, mem4py for membrane deformation, MoorPy for tether sag/drag.
- [ ] Resolve the arm-reach / tether keep-out-zone overlap in the GA.
- [ ] Update the iteration sheet's spec table text to match the solver's calibrated numbers (currently drawings and code agree on structure but the sheet predates the aero calibration).
- [ ] Decide the two-kytoon carriage pairing formally (Mk II or IV + Mk I or III) against a real ops-envelope requirement.

---

## Artifact index

| File | Description |
|---|---|
| `kiteship-ga.svg` | General arrangement — plan view (main deck) + profile (recovery configuration), 40 m trimaran |
| `kytoon-iterations.svg` | Mk I–IV design sketches with comparison spec table |
| `kytoon-l0-report.md` | Generated L0 solver output: buoyancy, structure margins, wind envelopes, calibration provenance |
| `kytoon-sim.zip` | Full Python repo: specs, solvers, tests, vendored TU Delft benchmark data |
