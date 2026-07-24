# godot/ — scene capture experiments

Godot 4.7 project for rendering the kytoon designs. Two scenes:

- `main.tscn` + `capture.gd` — the original claude.ai-sandbox experiment:
  procedural primitives, orbiting camera, PNG-per-frame capture. Proved the
  pipeline works with no GPU/display (`render.sh` runs it under Xvfb +
  Mesa llvmpipe on Linux). `orbit.gif` / `orbit.mp4` / `frame_000.png` are
  its outputs.
- `fleet.tscn` + `fleet_capture.gd` — the kytoon fleet turntable: loads
  `../models/*.glb` (from `python -m kytoon.geometry specs/ -o models`),
  rigs all five Mks over the sea with ship + tether, orbits the camera,
  saves a PNG per frame. Outputs in `renders/`.

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
