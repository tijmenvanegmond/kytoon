#!/usr/bin/env bash
# Offline render via Xvfb + Mesa llvmpipe. No GPU, no display server required.
set -euo pipefail
FRAMES="${1:-90}"
OUT="${2:-$PWD/frames}"
mkdir -p "$OUT"
LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe \
xvfb-run -a -s "-screen 0 1280x720x24" \
godot --path "$(dirname "$0")/.." sandbox/main.tscn \
  --rendering-driver opengl3 --audio-driver Dummy \
  -- --frames="$FRAMES" --out="$OUT"
