# Manta — the Godot layer

Godot 4.7 project for the kytoon work. The **Mk V 6-DOF sim is the main
scene**: open the project and press F5. The longitudinal sim it grew out
of stays alongside it (`sim/mkv_sim.tscn`) and keeps its own parity gate.

```
project.godot        name "Manta", main scene = sim/mkv_sim3d.tscn, input map
data/                mkv_sim_params.json — flight model, generated, do not edit
sim/                 the sim: mkv_sim.gd (+ .tscn), sim_hud, sim_camera, trace_plot
common/              kytoon_world.gd — shared sky/sea/ship/kite/line helpers
viz/                 capture scenes: fleet.tscn (turntable), mkv_replay.tscn
tools/               export_sim_params.py, export_mkv_replay.py (run from repo root)
sandbox/             the original claude.ai experiment that started this branch
renders/, frames/    capture output (gitignored, carry a .gdignore)
```

## The 3D sim (`sim/mkv_sim3d.tscn`)

A **6-DOF port of `kytoon.solvers.l1_sim3d`** — the model that can
represent steering, because roll, yaw and sideslip exist in it. The
longitudinal sim below stays as-is; this is a separate scene so its
parity gate is never at risk.

```
godot --path godot sim/mkv_sim3d.tscn                       # fly it
godot --path godot sim/mkv_sim3d.tscn --headless -- --selftest3d=out.csv
godot --path godot sim/mkv_sim3d.tscn -- --shots=DIR        # needs a GPU
```

Keys: **A/D** differential drums (steer), **W/S** common drum (trim),
**↑/↓** wind speed, **←/→** wind direction, **C** camera (rig / kite /
ship / wide), drag and wheel to orbit and zoom, R reset, Space pause.

The camera is `sim/sim_camera.gd` — the same `SimCamera` the longitudinal
sim uses. It used to be an ad-hoc orbit driven off sim time, which is
unflyable the moment you are actually steering. The capture mode
(`--shots`) advances the azimuth itself to keep its slow sweep.

State is a flat 13-array mirroring the Python layout — position (world),
quaternion (w,x,y,z), then velocity and angular velocity in **body**
axes. Two things keep it short and honest:

- **M⁻¹ is exported, not M.** The generalised mass is constant in body
  axes, so the equation of motion is one 6×6 matvec — GDScript needs no
  linear solver and cannot disagree with Python about how one behaves.
- **Kirchhoff form.** Added mass (~12× structural in roll) appears in the
  Coriolis terms too, built from the same 6×6. Dropping that makes a big
  light wing spin up wrongly under combined roll+yaw.

**Parity, re-verified 2026-07-27** over a 40 s manoeuvre with real 3D
content (a rate-limited differential drum input producing 8.7° of roll,
a 19.4° yaw peak and 8.5° of sideslip — the yaw and sideslip figures are
down slightly from the 2026-07-25 run, which predates the geometry
attachment fix in `245e695`):

```
python godot/tools/export_sim3d_params.py    # flight model
python godot/tools/export_sim3d_replay.py    # reference trajectory
godot --path godot sim/mkv_sim3d.tscn --headless -- --selftest3d=out.csv
#   roll 0.0017°  pitch 0.0004°  yaw 0.011°  alpha 0.0006°  beta 0.0044°
#   position 16 mm    tensions 1.6 N of 20.1 kN
```

### Steering has a hard limit, and the sim shows it

Past **~0.10 m of differential** the paying-out control line goes SLACK.
That removes the constraint pinning roll and pitch, and the rig departs
(roll past 120°). It is a magnitude limit, not a rate one — an
instantaneous step and a 0.02 m/s ramp depart identically — and it comes
from line mechanics: the pair carries only 1.74 kN at trim, i.e. 0.063 m
of available stretch. Pre-tensioning does not help, because control
tension is *determined* by moment closure at trim.

So the honest steering authority is **~6° of bank**, not the ~20° Stage 3
statics implied. The sim does not prevent you exceeding it; the HUD
raises **CTL LINE SLACK** and you watch it go. Past that point the aero
is flat-plate extrapolation at β ≈ −46° and is not a prediction.

### Frames

Body/sim axes are the loft's own — x downstream, y starboard, z up.
Godot is X right, Y up, Z toward the viewer. `to_godot()` maps
(x, y, z) → (x, z, −y), which is right-handed, and that is exactly what
`KytoonWorld.kite()`'s −90° X wrapper already applies to the glb — so the
outer node carries `T · R · T⁻¹` and the wrapper supplies the remaining
`T`. **Physics never touches this**; it is drawing only.

One trap worth naming: Godot's `Basis.x/.y/.z` are **columns**, so
numpy's `r[i][j]` is `b[j][i]`. Getting that backwards transposes the
matrix, which leaves *yaw* correct (it reads elements symmetric under the
mistake) while roll and pitch come out wrong by tens of degrees. That is
how it first showed up here — and why the parity gate ran before any
visuals were wired.

## The sim

`sim/mkv_sim.gd` is a **port of `kytoon.solvers.l1_trim`** — the same
6-state longitudinal model (x, z, θ, u, w, ω), RK4 at 240 Hz — plus a
lumped-mass segmented tether. The Python solver is the reference. Do not
add aero or buoyancy physics here that `l1_trim` doesn't have; change the
solver first, re-export, re-run the parity gate.

Beyond `l1_trim`, using the same laws:

- **Segmented main line** (~12 m segments, 2–36 by deployed length):
  EA/L springs, line weight, cylinder drag — it sags, goes honestly
  slack, and the pod rides the real line shape with its control reaction
  applied to the line nodes.
- **Variable line length**: stiffness follows k = EA/L as the winch reels.
- **Pod docking**: when the line gets shorter than the pod standoff the
  pod pins near the fairlead and its drum auto-tends at a constant 3 kN.
- **Post-stall aero outside the validated table** (see below).
- **Payload as an option**, on the kite or on the pod (see below).

### Controls

| key | |
|---|---|
| ↑ / ↓ | wind, 2–26 m/s |
| `-` / `=` | payload mass |
| P | payload on kite ↔ on pod |
| W / S | winchlet trim (control-line length) |
| T | hold current α (see below — α, not θ) |
| I / O | main winch in/out, 2 m/s — hold Shift for ×5 |
| G | +6 m/s 1-cos gust, 6 s |
| `[` / `]` | time rate, 0.25× … 8× (recovery takes 190 s at 1×) |
| 1 / 2 / 3 | scenario: cruise 12 m/s @200 m · loiter 8 @400 · capture hover 5 @25 |
| C | camera: rig / kite / ship / wide |
| drag, wheel | orbit, zoom |
| Space, R, H, Esc | pause, reset, help, quit |

### Payload is a sim option, not a spec edit

`specs/mk5_manta.yaml` stays at its 60 kg EO/IR payload. The sim can fly
other masses (`-` / `=`, 0–1500 kg) and recomputes CG, pitch inertia and
added inertia exactly the way `l1_trim.mass_props` does — the exporter
ships the payload-independent terms (`i_skin_own`, `ia_a/b/d`) and
asserts the rearrangement against the solver.

Buoyancy is fixed at **551 kg** of net He lift, so payload trades
directly against calm-air capability (solver numbers):

| payload | flying mass | net static | min wind to fly |
|--------:|------------:|-----------:|----------------:|
| 60 kg | 266 kg | +285 kg | 1.0 m/s |
| 300 kg | 506 kg | +45 kg | 4.5 m/s |
| 500 kg | 706 kg | −155 kg | 5.5 m/s |
| 700 kg | 906 kg | −355 kg | 6.5 m/s |

Past ~490 kg the airframe stops floating: it becomes a kite that must be
flown and comes down when the wind drops. The HUD's *net lift* row turns
red there. Carrying 500 kg buoyantly would need roughly +470 m³ of helium
(~1000 m³ vs today's 530) — a spec conversation, not a sim setting.

**Where the payload hangs matters** (`P`). On the kite it rides the main
bridle attach and is part of the wing's mass, CG and inertia. On the pod
it hangs on the tether 50 m below as a point mass on the line — the wing
gets lighter and more buoyant, and the line above the pod carries the
weight. 500 kg at 12 m/s, locked winches, +6 m/s gust:

| carried on | θ | α | T_main | gust α | gust T |
|---|--:|--:|--:|--:|--:|
| kite | 15.2° | 15.1° | 18.3 kN | 17.4° | 36.7 kN |
| pod | 13.5° | 13.4° | 20.7 kN | 15.4° | 41.6 kN |

Podding it buys ~2° of stall margin and a flatter wing for ~13 % more
line tension (both well inside the 66.7 kN WLL). It does **not** help
the static problem — the system still has to lift the same mass.

## Aero outside the validated table

`l1_trim`'s polar covers α ∈ [−8°, 24°]. Inside that band the sim
interpolates the solver's own data and the parity gate covers it.
Outside, the sim blends over 12° to a flat plate
(CL = sin 2α, CD = 0.06 … 1.9 sin²α); Cm holds at the edge value.

This is not cosmetic. The table used to simply **clamp**, so a stalled
wing kept gliding on CD ≈ 0.05 at α = −60° — every slack-line upset
became a clean, unrecoverable dive. A real fat wing there is a bluff body
that decelerates and tumbles until buoyancy and the line take over. The
HUD raises **AERO EXTRAPOLATED** whenever you are out of the validated
band: those coefficients are a plausibility model, never a design claim.

Worth knowing *why* upsets happen at all: free-flying, the airframe's
neutral point is 1.06 m aft of the centre c/4 but the CG sits at 2.89 m —
a static margin of **−1.83 m (−20 % MAC)**, divergent at +2.3 kN·m/deg.
Lose line tension and it *will* nose over. That is the same fact that
makes the 3-line rig load-bearing rather than optional.

## Headless modes

```
godot --path godot sim/mkv_sim.tscn --headless -- --selftest=out.csv
godot --path godot sim/mkv_sim.tscn --headless -- --recovery-test=out.csv
godot --path godot sim/mkv_sim.tscn --headless -- --fly-test=out.csv \
      --wind=12 --payload=500 --payload-on-pod
godot --path godot sim/mkv_sim.tscn -- --demo-out=DIR        # needs a GPU
```

`--fly-test` is "just hang there": locked winches, 90 s, gust at t=40,
and it honours `--wind` / `--payload` / `--payload-on-pod`, so it answers
configuration questions the parity gate deliberately refuses to. Those
flags also work on the interactive run.

**`--selftest` is the parity gate.** It runs the locked-winch gust on the
straight-line model from the exact `l1_trim` reconstruction and ignores
every configuration flag — no scenario preset, no non-spec payload, never
podded. Those leak in and the gate silently stops testing what it claims;
that regression has already happened once. It must match
`renders/mkv_replay.csv`:

```
python godot/tools/export_mkv_replay.py     # Python reference trajectory
# then diff the two CSVs — last verified 2026-07-25:
#   alpha 0.020 deg, theta 0.007 deg, T_main 72 N, T_ctl 20 N
```

Regenerate the flight model after any spec or solver change:

```
python godot/tools/export_sim_params.py
```

## Capture scenes

```
godot --path godot viz/fleet.tscn --audio-driver Dummy -- --frames=180 --out=DIR
godot --path godot viz/mkv_replay.tscn --audio-driver Dummy -- --csv=FILE --out=DIR
godot --path godot sandbox/main.tscn --audio-driver Dummy -- --frames=90 --out=DIR
```

`viz/mkv_replay.tscn` is a pure viewer for a Python-computed trajectory —
no physics in Godot, so the picture cannot flatter the model. Assemble
frames with the venv's imageio (bundled ffmpeg):

```python
import glob, imageio.v2 as iio
imgs = [iio.imread(f) for f in sorted(glob.glob('DIR/frame_*.png'))]
w = iio.get_writer('out.mp4', fps=30, codec='libx264', quality=8, pixelformat='yuv420p')
for im in imgs: w.append_data(im)
w.close()
```

## Recovery procedure (what the sim taught us, 2026-07-24)

Winching Mk V from 400 m to the 20 m capture hover at 5 m/s wind fails
three naive ways before it works:

1. **Tension-governed reel**, not constant speed — a 2 m/s speed step on
   400 m of elastic line pogo-bounces the 3-t effective mass (ζ ≈ 0.16)
   into slack/snap cycles (observed 63–150 kN spikes, tumbling).
2. **α-hold on the winchlet, not θ-hold** — descending at 2 m/s in 5 m/s
   wind adds ~22° of inflow; holding attitude lets α run away, so the
   kite climbs, overflies the ship and noses over. Hold α (interactive:
   `T` captures whatever α you are flying) and winching in just descends.
3. **Stay powered** — a buoyant kite needs no depower to descend (the
   winch trivially beats +2.8 kN net buoyancy) and slack control lines
   mean no attitude authority at all.

Result: ~190 s descent, tension ≤ 20 kN, hover at ~27 m with **θ ≈ +2°**
— level. Getting there needed one more fix: the docked drum's "auto-tend"
was modelled as a spring around a shifting rest length, and the line
damping cancelled it, leaving the control pair dead slack exactly when
attitude authority was needed (the kite hung −50° and ditched at 216 s).
A constant-tension drum is what "tend" means; implemented that way it
holds 3.00 kN and levels the hover.

That resolves an open question the other way round from the earlier note:
the tension-tend option **does** work — the aft capture pendant is no
longer the only route to a level hover. Static hang (`l1_trim.hang_trim`,
−41° at zero q) still describes the *untended* rig. What the sim still
owes: the pendulum excursion envelope in ship frame — the capture arm's
chase spec.

## Conventions / gotchas

- The glb exporter authors x-downstream / y-spanwise / **z-up**; glTF is
  Y-up. `KytoonWorld.kite()` applies −90° X plus a yaw: 0 for the
  longitudinal scenes, 90 for the fleet lineup. A model that looks rolled
  over is missing this.
- Model origins sit at the bridle confluence — attach tethers to the node
  origin (small per-Mk keel offsets in `FLEET`).
- Kytoon colors come from `kytoon/viz.py` `MK_COLOR` (mirrored in
  `KytoonWorld.MK_COLOR`) — fixed per Mk, never re-derived from order.
- Canopies are open surfaces: materials need `cull_mode = CULL_DISABLED`
  or they vanish from one side.
- `--headless` disables rendering entirely in Godot 4, so it cannot
  capture frames — that needs a real driver (a GPU here, or the
  Xvfb + Mesa llvmpipe trick in `sandbox/render.sh` on a display-less
  Linux box). Headless is still right for `--selftest`/`--recovery-test`.
- Keep `.gdignore` in `renders/` and `frames/`: without it Godot imports
  every capture PNG as a texture and parses stray CSVs as *translation
  files*. The exporters write it; don't delete it.
- GDScript gotchas hit while building this: no `%g` in format strings,
  `_set` collides with `Object._set`, and `max()`/`min()` return Variant
  so `var x := max(a, b)` fails to infer — use `maxf`/`maxi`.

## Follow-ups

- Cross-check the segmented tether's sag/tension against `l1_tether`'s
  MoorPy statics — same physics, two independent implementations.
- `SimCamera`'s RIG framing focuses on the kite/pod midpoint at a fixed
  70 m, which clips the airframe at the top of frame at the default pod
  standoff. Orbit and zoom get you out of it, but the opening shot is
  wrong. Retuning touches both sims, so it wants its own before/after.

Done 2026-07-27: `viz/mkv_replay.gd` and `viz/fleet_capture.gd` moved
onto `common/kytoon_world.gd` (they predated it and each rebuilt their
own sky/sea/ship/lines) — 570 → 374 lines, verified bit-identical over
rendered frames from both scenes. The fleet keeps its own ship: it is a
small boat at lineup scale, not the 40 m trimaran the sim scenes use.
