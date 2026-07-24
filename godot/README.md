# godot/ — scene capture + sim layer

Godot 4.7 project for rendering and now *simulating* the kytoon designs.
Scenes:

- `main.tscn` + `capture.gd` — the original claude.ai-sandbox experiment:
  procedural primitives, orbiting camera, PNG-per-frame capture. Proved the
  pipeline works with no GPU/display (`render.sh` runs it under Xvfb +
  Mesa llvmpipe on Linux). `orbit.gif` / `orbit.mp4` / `frame_000.png` are
  its outputs.
- `fleet.tscn` + `fleet_capture.gd` — the kytoon fleet turntable: loads
  `../models/*.glb` (from `python -m kytoon.geometry specs/ -o models`),
  rigs all five Mks over the sea with ship + tether, orbits the camera,
  saves a PNG per frame. Outputs in `renders/`.
- `mkv_replay.tscn` + `mkv_replay.gd` — replays a Mk V trajectory CSV
  computed by the Python solver (`export_mkv_replay.py`, pod-rig
  locked-winch gust case). Pure viewer, no physics in Godot.
- `mkv_sim.tscn` + `mkv_sim.gd` — **the sim layer**: a GDScript port of
  `kytoon.solvers.l1_trim._derivs` (6-state longitudinal model, RK4 at
  240 Hz) running live. Parameters come from `mkv_sim_params.json`
  (regenerate with `export_sim_params.py` after any spec/solver change).
  Interactive keys: Up/Down wind, W/S winchlet trim, **I/O main winch**
  (2 m/s, Shift ×5 — full recovery to deck), G gust, R reset, Space
  pause. Extra physics beyond l1_trim, same laws: the main line is a
  **lumped-mass segmented tether** (≈35 m segments, 2–12 adapting to
  deployed length, EA/L springs, line weight, cylinder drag — so it
  sags, goes honestly slack, and the pod rides the actual line shape
  with its control reaction applied to the line nodes); the pod docks
  near the fairlead when the line gets shorter than its standoff (ctl
  drum auto-tends ~3 kN — pitch pinning goes soft, an honest
  consequence). Headless modes: `-- --selftest=out.csv` (locked-winch
  gust, run on the STRAIGHT-line model — the parity gate vs l1_trim;
  verified 2026-07-24 to 0.02° in α / 0.2 % in tension vs
  `renders/mkv_replay.csv`) and `-- --recovery-test=out.csv` (full
  400→20 m winch-in on the segmented line). The Python solver stays the
  reference for the flight model — do not add aero or buoyancy physics
  here that l1_trim doesn't have; the segmented tether is l1_tether's
  territory and should eventually be cross-checked against its MoorPy
  sag/tension numbers.

## Recovery procedure (what the sim taught us, 2026-07-24)

Winching Mk V from 400 m to the 20 m capture hover at 5 m/s wind fails
three naive ways before it works; the working procedure is:

1. **Tension-governed reel**, not constant speed — a 2 m/s speed step on
   400 m of elastic line pogo-bounces the 3-t effective mass (ζ ≈ 0.16)
   into slack/snap cycles (observed 63–150 kN spikes, tumbling).
2. **α-hold on the winchlet, not θ-hold** — descending at 2 m/s in 5 m/s
   wind adds ~22° of inflow; holding attitude runs the wing past stall.
   The winchlet must trim nose-down on the way down (α ≈ 6°) and re-trim
   for the hover.
3. **Stay powered** — a buoyant kite needs no depower to descend (the
   winch trivially beats +2.8 kN net buoyancy) and slack control lines
   mean no attitude authority at all.

Result: 190 s descent, tension never above ~10 kN, ending in a buoyant
hover at ~23 m. The hover attitude itself is NOT a sim problem — it is
closed-form statics (`l1_trim.hang_trim`): the current rig hangs −41°
nose-down at zero q (−54° with the 5 m/s residual, matching this sim's
endgame within 4°). Level-hover options are solved and gated in
test_l1_trim (aft pendant ≈ 0.8 kN, or lock the ctl drum through
docking instead of the tension-tend implemented here). What the sim
still owes once the rigging is chosen: the winner's transient into the
hover and the pendulum excursion envelope in ship frame — the capture
arm's actual chase spec.

## Run (Windows, GPU, window flashes briefly)

```
godot --path godot fleet.tscn --audio-driver Dummy -- --frames=180 --out=C:/some/dir/frames
```

Assemble frames (venv has imageio + bundled ffmpeg):

```python
import glob, imageio.v2 as iio
imgs = [iio.imread(f) for f in sorted(glob.glob(r'frames/frame_*.png'))]
w = iio.get_writer('fleet_orbit.mp4', fps=30, codec='libx264', quality=8, pixelformat='yuv420p')
for im in imgs: w.append_data(im)
w.close()
```

## Conventions / gotchas

- The glb exporter authors x-downstream / y-spanwise / **z-up**; glTF is
  Y-up. Every loaded model therefore gets `rotation_degrees = (-90, 90, 0)`:
  up → +Y, span along X, nose pointing −Z (wind from +Z). A model that looks
  rolled over means this rotation is missing.
- Model origins sit at the bridle confluence — attach tethers to the node
  origin (small per-Mk keel offsets in `FLEET`).
- Kytoon colors come from `kytoon/viz.py` `MK_COLOR` — fixed per Mk, don't
  re-derive.
- Canopies are open surfaces: materials need `cull_mode = CULL_DISABLED`
  or they vanish from one side.
- `--headless` disables rendering entirely in Godot 4 — viewport capture
  needs a real driver (GPU here, or the Xvfb/llvmpipe trick from
  `render.sh` on a display-less Linux box).
