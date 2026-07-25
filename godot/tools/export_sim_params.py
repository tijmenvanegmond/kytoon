"""Export the Mk V longitudinal model as JSON for the GDScript sim layer.

Everything godot/sim/mkv_sim.gd needs to integrate the same 6-state model
as kytoon.solvers.l1_trim: aero tables, mass properties, rig geometry,
line stiffnesses, and the 12 m/s mission-trim initial state. The GDScript
sim must stay a PORT of l1_trim._derivs — regenerate this file and rerun
the sim's --selftest after any solver/spec change (gated by comparing
against godot/renders/mkv_replay.csv).

Run from the repo root, needs the l1 extra:
    .venv/Scripts/python godot/tools/export_sim_params.py
"""
import json
import math
import sys

import numpy as np

sys.stdout.reconfigure(encoding="utf-8")

from kytoon.geometry import _lofted_fatwing
from kytoon.solvers.l0 import RHO_AIR
from kytoon.solvers.l1_trim import (
    CM_Q, FAIRLEAD_HEIGHT, SWEEP_DEG, aero_table, attach_point,
    ctl_span_offset, line_stiffnesses, mass_props, reconstruct_rig,
    trim_point,
)
from kytoon.spec import load_spec

OUT = "godot/data/mkv_sim_params.json"

spec = load_spec("specs/mk5_manta.yaml")
table = aero_table(spec)
props = mass_props(spec)
tp = trim_point(spec, props, table, 12.0, 14.0)
assert tp.feasible
state, l0m, l0c, k_main, k_ctl = reconstruct_rig(spec, tp)
al, cl, cd, cm = table

# --- payload-independent mass terms ---------------------------------------
# The sim lets you fly other payloads (the spec's 60 kg is one option), so
# it must recompute CG and inertia itself. Export the pieces that do NOT
# depend on payload, in the form the sim needs:
#   i_yy(r_cg)    = i_skin_own + m_skin·|r_skin−r_cg|² + m_pod·|p_main−r_cg|²
#   i_added(x_cg) = ia_a − 2·ia_b·x_cg + m_added_z·x_cg² + ia_d
# both exact rearrangements of l1_trim.mass_props (asserted below).
fw = spec.fat_wing
mesh = _lofted_fatwing(fw)
w_f = mesh.area_faces
_d2 = ((mesh.triangles_center[:, [0, 2]] - props.r_skin) ** 2).sum(1)
i_skin_own = float((props.m_skin / w_f.sum()) * (w_f * _d2).sum())

ys = np.linspace(-fw.span / 2, fw.span / 2, 201)
cs = np.array([fw.chord_at(abs(2 * y / fw.span)) for y in ys])
m_strip = math.pi * RHO_AIR * cs**2 / 4
xqc = np.tan(np.radians(SWEEP_DEG)) * np.abs(ys)
ia_a = float(np.trapezoid(m_strip * xqc**2, ys))
ia_b = float(np.trapezoid(m_strip * xqc, ys))
ia_d = float(np.trapezoid(m_strip * cs**2 / 32, ys))

_p_main = attach_point(spec, spec.bridle.positions[1])
assert abs(i_skin_own
           + props.m_skin * float(((props.r_skin - props.r_cg) ** 2).sum())
           + props.m_pod * float(((_p_main - props.r_cg) ** 2).sum())
           - props.i_yy) < 1e-6 * props.i_yy
assert abs(ia_a - 2 * ia_b * props.r_cg[0]
           + props.m_added_z * props.r_cg[0] ** 2 + ia_d
           - props.i_added) < 1e-6 * props.i_added

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
    # payload as a sim option — see the block above
    "payload_ref_kg": spec.payload_mass,
    "rigging_kg": spec.rigging_mass,
    "i_skin_own": i_skin_own,
    "ia_a": ia_a,
    "ia_b": ia_b,
    "ia_d": ia_d,
    "p_main": list(attach_point(spec, spec.bridle.positions[1])),
    "p_ctl": list(attach_point(spec, spec.bridle.positions[2])),
    "y_ctl": ctl_span_offset(spec),
    "k_main": k_main,
    "k_ctl": k_ctl,
    "c_line": 2000.0,
    "pod_standoff": spec.bridle.pod_standoff_m,
    "fairlead": [0.0, FAIRLEAD_HEIGHT],
    "tether_length": spec.tether.length,
    "tether_diameter_m": spec.tether.diameter_mm / 1000.0,
    "tether_linear_density": spec.tether.linear_density,
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
