"""Export a Mk V gust-response trajectory for the Godot replay scene.

Pod-rig configuration (bridle.pod_standoff_m): winchlet pod rides the main
tether below the kite, short steering lines from there. The demo case is
the passive-stability headline: 12 m/s, +6 m/s 1-cos gust at t=15..21,
winches LOCKED — the rig recovers on its own.

Writes CSV (run from the repo root, needs the l1 extra):
t, U, alpha, theta_deg, x, z, xm, zm, xc, zc, xp, zp, T_main, T_ctl
(world coords: x downwind [m], z up [m]; xm/zm main attach, xc/zc control
attach, xp/zp pod — the replay never re-derives geometry).
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.stdout.reconfigure(encoding="utf-8")

from kytoon.solvers.l1_trim import (
    FAIRLEAD_HEIGHT, _derivs, _rotm, aero_table, attach_point,
    ctl_span_offset, line_stiffnesses, mass_props, reconstruct_rig,
    trim_point,
)
from kytoon.spec import load_spec

OUT = "godot/renders/mkv_replay.csv"     # run from the repo root

# renders/ sits inside the Godot project: without .gdignore the editor
# imports every PNG as a texture and parses stray CSVs as translations.
_out_dir = Path(OUT).parent
_out_dir.mkdir(parents=True, exist_ok=True)
(_out_dir / ".gdignore").touch()

spec = load_spec("specs/mk5_manta.yaml")
assert spec.bridle.pod_standoff_m, "demo expects the pod rig"
table = aero_table(spec)
props = mass_props(spec)
tp = trim_point(spec, props, table, 12.0, 14.0)   # mission trim @ 12 m/s
assert tp.feasible

s, L0m, L0c, k_main, k_ctl = reconstruct_rig(spec, tp)
anchor = np.array([0.0, FAIRLEAD_HEIGHT])
p_main_b = attach_point(spec, spec.bridle.positions[1])
p_ctl_b = attach_point(spec, spec.bridle.positions[2])
y_ctl = ctl_span_offset(spec)


def wind(t):
    if 15.0 <= t <= 21.0:
        return 12.0 + 3.0 * (1 - math.cos(2 * math.pi * (t - 15.0) / 6.0))
    return 12.0


def geometry(st):
    """(r_main, r_ctl, pod, T_main, T_ctl) at a state — mirrors _derivs."""
    R = _rotm(st[2])
    rw_m = R @ p_main_b
    r_m = st[:2] + rw_m
    v_m = st[3:5] + st[5] * np.array([rw_m[1], -rw_m[0]])
    d = anchor - r_m
    dist = float(np.linalg.norm(d))
    uv = d / dist
    Tm = max(k_main * (dist - L0m) - 2000.0 * float(v_m @ uv), 0.0) \
        if dist > L0m else 0.0
    pod = r_m + spec.bridle.pod_standoff_m * uv
    rw_c = R @ p_ctl_b
    r_c = st[:2] + rw_c
    v_c = st[3:5] + st[5] * np.array([rw_c[1], -rw_c[0]])
    v = pod - r_c
    v_len = float(np.linalg.norm(v))
    dist3 = math.hypot(v_len, y_ctl)
    Tc = 0.0
    if dist3 > L0c:
        uvc = v / v_len
        Tc = max(k_ctl * (dist3 - L0c)
                 - 2000.0 * float((v_c - v_m) @ uvc), 0.0)
    return r_m, r_c, pod, Tm, Tc


dt, T_END, REC = 0.004, 45.0, 0.1
rows = []
for i in range(int(T_END / dt)):
    t = i * dt
    U = wind(t)
    k1 = _derivs(spec, props, table, s, L0m, L0c, U, k_main, k_ctl)
    k2 = _derivs(spec, props, table, s + dt / 2 * k1, L0m, L0c, U,
                 k_main, k_ctl)
    k3 = _derivs(spec, props, table, s + dt / 2 * k2, L0m, L0c, U,
                 k_main, k_ctl)
    k4 = _derivs(spec, props, table, s + dt * k3, L0m, L0c, U,
                 k_main, k_ctl)
    s = s + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    if i % round(REC / dt) == 0:
        r_m, r_c, pod, Tm, Tc = geometry(s)
        wa = np.array([U - s[3], -s[4]])
        alpha = math.degrees(s[2] + math.atan2(wa[1], wa[0]))
        rows.append([t, U, alpha, math.degrees(s[2]), s[0], s[1],
                     r_m[0], r_m[1], r_c[0], r_c[1], pod[0], pod[1],
                     Tm, Tc])

rows = np.array(rows)
header = "t,U,alpha,theta_deg,x,z,xm,zm,xc,zc,xp,zp,T_main,T_ctl"
np.savetxt(OUT, rows, delimiter=",", header=header, comments="",
           fmt="%.4f")
print(f"wrote {len(rows)} rows -> {OUT}")
print(f"alpha range [{rows[:, 2].min():.1f}, {rows[:, 2].max():.1f}] deg, "
      f"T_main peak {rows[:, 12].max()/1e3:.1f} kN  (winches locked)")
