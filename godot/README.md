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
  Interactive keys: Up/Down wind, W/S winchlet, G gust, R reset, Space
  pause. Also `-- --demo-out=DIR` (scripted capture) and
  `-- --selftest=out.csv` (headless; must match the Python trajectory —
  verified 2026-07-24 to 0.012° in α / 0.12 % in tension vs
  `renders/mkv_replay.csv`). The Python solver stays the reference:
  this file is a port, not a fork — do not add physics here that
  l1_trim doesn't have.

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
