"""Export the Mk V longitudinal model as JSON for the GDScript sim layer.

Everything godot/mkv_sim.gd needs to integrate the same 6-state model as
kytoon.solvers.l1_trim: aero tables, mass properties, rig geometry, line
stiffnesses, and the 12 m/s mission-trim initial state. The GDScript sim
must stay a PORT of l1_trim._derivs — regenerate this file and rerun the
sim's --selftest after any solver/spec change (gated by comparing against
godot/renders/mkv_replay.csv).

Run from the repo root, needs the l1 extra:
    .venv/Scripts/python godot/export_sim_params.py
"""
import json
import sys

sys.stdout.reconfigure(encoding="utf-8")

from kytoon.solvers.l1_trim import (
    CM_Q, FAIRLEAD_HEIGHT, aero_table, attach_point, ctl_span_offset,
    line_stiffnesses, mass_props, reconstruct_rig, trim_point,
)
from kytoon.spec import load_spec

OUT = "godot/mkv_sim_params.json"

spec = load_spec("specs/mk5_manta.yaml")
table = aero_table(spec)
props = mass_props(spec)
tp = trim_point(spec, props, table, 12.0, 14.0)
assert tp.feasible
state, l0m, l0c, k_main, k_ctl = reconstruct_rig(spec, tp)
al, cl, cd, cm = table

params = {
    "name": spec.name,
    "S": spec.canopy.area,
    "c_ref": spec.canopy.area / spec.canopy.span,
    "rho": 1.225,
    "g": 9.81,
    "cm_q": CM_Q,
    "alphas": list(al),
    "cl": list(cl),
    "cd": list(cd),                      # already includes line parasitic
    "cm": list(cm),
    "m_total": props.m_total,
    "m_skin": props.m_skin,
    "m_pod": props.m_pod,
    "buoyancy_n": props.buoyancy_n,
    "r_cg": list(props.r_cg),
    "r_cb": list(props.r_cb),
    "r_skin": list(props.r_skin),
    "i_yy": props.i_yy,
    "i_added": props.i_added,
    "m_added_x": props.m_added_x,
    "m_added_z": props.m_added_z,
    "p_main": list(attach_point(spec, spec.bridle.positions[1])),
    "p_ctl": list(attach_point(spec, spec.bridle.positions[2])),
    "y_ctl": ctl_span_offset(spec),
    "k_main": k_main,
    "k_ctl": k_ctl,
    "c_line": 2000.0,
    "pod_standoff": spec.bridle.pod_standoff_m,
    "fairlead": [0.0, FAIRLEAD_HEIGHT],
    "tether_length": spec.tether.length,
    "wll_n": spec.tether.wll_n,
    "ctl_cap_n": 2 * spec.bridle.control_mbl_kn * 1e3
        / spec.tether.safety_factor,
    "init": {
        "state": list(state),
        "l0_main": l0m,
        "l0_ctl": l0c,
        "wind": 12.0,
        "alpha_trim_deg": tp.alpha_deg,
    },
}

with open(OUT, "w", encoding="utf-8") as f:
    json.dump(params, f, indent=1)
print(f"wrote {OUT}  (trim alpha {tp.alpha_deg:.1f} deg @ 12 m/s, "
      f"k_ctl {k_ctl/1e3:.1f} kN/m)")
