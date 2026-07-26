"""Export a 6-DOF reference trajectory for the Godot 3D parity gate.

The case has to contain real 3D content or the gate proves nothing: hold
at trim, then step the two control drums differentially so the kite rolls,
yaws and slips — the coupling the longitudinal model cannot represent.

Writes CSV: t, roll, pitch, yaw, alpha, beta, x, y, z, T_port, T_main,
T_stbd. Run from the repo root, needs the l1 extra.
"""
import sys

import numpy as np

sys.stdout.reconfigure(encoding="utf-8")

from kytoon.solvers.l1_lat_aero import solve as solve_lat
from kytoon.solvers.l1_mass3d import mass_props_3d
from kytoon.solvers.l1_rig3d import MAIN, PORT, STBD, rest_lengths, trim3d
from kytoon.solvers.l1_sim3d import integrate, state_from_trim
from kytoon.solvers.l1_trim import aero_table
from kytoon.spec import load_spec

OUT = "godot/renders/mkv_sim3d_replay.csv"
WIND, T_END, DIFF, T_STEP = 12.0, 40.0, 0.10, 5.0

spec = load_spec("specs/mk5_manta.yaml")
gam = spec.fat_wing.dihedral_deg
props = mass_props_3d(spec, dihedral_deg=gam)
table = aero_table(spec)
lat = solve_lat(spec, dihedral_deg=gam,
                fin_area_m2=spec.fin.area if spec.fin else 0.0)
wind = np.array([WIND, 0.0, 0.0])
l0 = rest_lengths(spec, props, table, lat, wind, dihedral_deg=gam)
x, ok = trim3d(spec, props, table, lat, l0, wind, dihedral_deg=gam)
assert ok


RAMP = 0.05          # m/s, the winchlet is rate-limited


def control(t, _state):
    """Differential drum input at T_STEP: port in, starboard out.

    DIFF is held at 0.10 m deliberately. Beyond ~0.10-0.15 m the paying-out
    control line goes SLACK, which removes the constraint that pins roll and
    pitch, and the kite departs (roll past 120 deg). That limit is set by
    line mechanics — the pair only carries ~1.74 kN at trim, i.e. 0.063 m of
    available stretch — not by aero or by winch force, and it is not
    relieved by pre-tensioning. See KYTOON-PROJECT.md section 6.
    """
    out = l0.copy()
    if t >= T_STEP:
        d = min(DIFF, RAMP * (t - T_STEP))
        out[PORT] -= d
        out[STBD] += d
    return out


hist = integrate(spec, props, table, lat, state_from_trim(spec, x), l0,
                 wind, T_END, dt=0.004, dihedral_deg=gam,
                 record_every=0.1, control=control)

rows = np.array([[h.t, h.rpy_deg[0], h.rpy_deg[1], h.rpy_deg[2],
                  h.alpha_deg, h.beta_deg, h.pos[0], h.pos[1], h.pos[2],
                  h.tensions[PORT], h.tensions[MAIN], h.tensions[STBD]]
                 for h in hist])
np.savetxt(OUT, rows, delimiter=",", comments="", fmt="%.5f",
           header="t,roll,pitch,yaw,alpha,beta,x,y,z,T_port,T_main,T_stbd")

end = hist[-1]
print(f"wrote {len(rows)} rows -> {OUT}")
print(f"  differential {DIFF:.2f} m at t={T_STEP:.0f}s")
print(f"  end: roll {end.rpy_deg[0]:+.2f}°  yaw {end.rpy_deg[2]:+.2f}°  "
      f"beta {end.beta_deg:+.2f}°  y {end.pos[1]:+.1f} m")
print(f"  tensions port {end.tensions[PORT]/1e3:.2f} / "
      f"main {end.tensions[MAIN]/1e3:.1f} / "
      f"stbd {end.tensions[STBD]/1e3:.2f} kN")
