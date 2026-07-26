"""Export the Mk V 6-DOF model as JSON for the Godot 3D sim layer.

Companion to `export_sim_params.py` (which serves the longitudinal sim).
Everything `godot/sim/mkv_sim3d.gd` needs to integrate the same equations
as `kytoon.solvers.l1_sim3d`: aero table, lateral derivatives, mass and
added-mass terms, the three-line rig, and a converged 3D trim to start
from.

Two deliberate choices that keep GDScript simple and honest:

  * **M⁻¹ is exported, not M.** The generalised mass is constant in body
    axes, so the equation of motion is one 6×6 matvec — GDScript never
    needs a linear solver, and cannot disagree with Python about how one
    behaves.
  * **Straight lines, as in `l1_sim3d`.** The longitudinal sim's
    segmented tether is a Godot-only extra with no 3D counterpart yet;
    the port must match the reference model first.

Run from the repo root, needs the l1 extra:
    .venv/Scripts/python godot/tools/export_sim3d_params.py
"""
import json
import sys

import numpy as np

sys.stdout.reconfigure(encoding="utf-8")

from kytoon.solvers.l1_dyn3d import C_LINE, mass_matrix
from kytoon.solvers.l1_lat_aero import solve as solve_lat
from kytoon.solvers.l1_mass3d import attach_point_3d, mass_props_3d
from kytoon.solvers.l1_rig3d import MAIN, PORT, STBD, rest_lengths, trim3d
from kytoon.solvers.l1_sim3d import matrix_to_quat, state_from_trim
from kytoon.solvers.l1_trim import (
    CM_Q, DYNEEMA_STRAIN_MBL, FAIRLEAD_HEIGHT, aero_table,
)
from kytoon.spec import load_spec

OUT = "godot/data/mkv_sim3d_params.json"
WIND = 12.0

spec = load_spec("specs/mk5_manta.yaml")
gam = spec.fat_wing.dihedral_deg
fin_area = spec.fin.area if spec.fin else 0.0

props = mass_props_3d(spec, dihedral_deg=gam)
table = aero_table(spec)
lat = solve_lat(spec, dihedral_deg=gam, fin_area_m2=fin_area)
wind_world = np.array([WIND, 0.0, 0.0])
l0 = rest_lengths(spec, props, table, lat, wind_world, dihedral_deg=gam)
x, ok = trim3d(spec, props, table, lat, l0, wind_world, dihedral_deg=gam)
assert ok, "3D trim did not converge — refusing to export"
state = state_from_trim(spec, x)

m6 = mass_matrix(props, np.eye(3))            # body axes, constant
m6_inv = np.linalg.inv(m6)
al, cl, cd, cm = table

params = {
    "name": spec.name,
    "wind_ref": WIND,
    # --- reference geometry / aero ---------------------------------------
    "S": spec.canopy.area,
    "b_ref": spec.fat_wing.span,
    "c_ref": spec.canopy.area / spec.canopy.span,
    "rho": 1.225,
    "g": 9.81,
    "alphas": list(al),
    "cl": list(cl),
    "cd": list(cd),
    "cm": list(cm),
    "cm_q": CM_Q,
    # lateral derivatives (Stage 2). Sign conventions are carried by
    # l1_rig3d.aero_wrench — CY takes the single beta flip, Cl/Cn take
    # two that cancel. Do not "fix" these signs in GDScript.
    "cy_beta": lat.cy_beta,
    "cl_beta": lat.cl_beta,
    "cn_beta": lat.cn_beta,
    "cl_p": lat.cl_p,
    "cl_r": lat.cl_r,
    "cn_p": lat.cn_p,
    "cn_r": lat.cn_r,
    # --- mass -------------------------------------------------------------
    "m_total": props.m_total,
    "m_skin": props.m_skin,
    "m_pod": props.m_pod,
    "buoyancy_n": props.buoyancy_n,
    "r_cg": list(props.r_cg),
    "r_cb": list(props.r_cb),
    "r_skin": list(props.r_skin),
    "m6_inv": [list(row) for row in m6_inv],
    "m6": [list(row) for row in m6],          # for the Coriolis momenta
    # --- rig ---------------------------------------------------------------
    "p_port": list(attach_point_3d(spec, spec.bridle.positions[PORT])),
    "p_main": list(attach_point_3d(spec, spec.bridle.positions[MAIN])),
    "p_stbd": list(attach_point_3d(spec, spec.bridle.positions[STBD])),
    "ea_main": spec.tether.mbl_kn * 1e3 / DYNEEMA_STRAIN_MBL,
    "ea_ctl": spec.bridle.control_mbl_kn * 1e3 / DYNEEMA_STRAIN_MBL,
    "c_line": C_LINE,
    "pod_standoff": spec.bridle.pod_standoff_m,
    "fairlead": [0.0, 0.0, FAIRLEAD_HEIGHT],
    "tether_length": spec.tether.length,
    "wll_n": spec.tether.wll_n,
    "ctl_cap_n": 2 * spec.bridle.control_mbl_kn * 1e3
        / spec.tether.safety_factor,
    "dihedral_deg": gam,
    "fin_area": fin_area,
    # --- converged trim to start from --------------------------------------
    "init": {
        "pos": list(state[0:3]),
        "quat": list(state[3:7]),             # (w, x, y, z)
        "l0": [l0[PORT], l0[MAIN], l0[STBD]],
        "rpy_deg": [float(np.degrees(v)) for v in x[3:]],
    },
}

with open(OUT, "w", encoding="utf-8") as f:
    json.dump(params, f, indent=1)

q = state[3:7]
print(f"wrote {OUT}")
print(f"  trim: pos ({x[0]:.1f}, {x[1]:.2f}, {x[2]:.1f}) m  "
      f"rpy ({np.degrees(x[3]):.2f}, {np.degrees(x[4]):.2f}, "
      f"{np.degrees(x[5]):.2f})°")
print(f"  quat (w,x,y,z) = ({q[0]:.5f}, {q[1]:.5f}, {q[2]:.5f}, {q[3]:.5f})")
print(f"  Γ = {gam:.0f}°, fin {fin_area:.0f} m², "
      f"M⁻¹ condition {np.linalg.cond(m6):.1f}")
