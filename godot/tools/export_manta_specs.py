"""Export all Manta specs to JSON for Godot consumption.

Run from repo root:
    python godot/tools/export_manta_specs.py

Generates godot/data/manta_specs.json with all Manta type specifications
in a format that Godot can load directly.
"""
import json
import math
from pathlib import Path

from kytoon.spec import load_spec
from kytoon.solvers.l0 import solve, NET_LIFT_PER_M3


def spec_to_dict(spec, report=None):
    """Convert a KytoonSpec to a dictionary suitable for Godot."""
    d = {
        "name": spec.name,
        "mk": spec.mk,
        "archetype": spec.archetype.value,
        "notes": spec.notes,
        "wing_area": spec.wing_area,
        "helium_volume": spec.helium_volume,
        "total_mass": spec.total_mass,
        "structure_mass": spec.structure_mass,
        "payload_mass": spec.payload_mass,
        "rigging_mass": spec.rigging_mass,
        "dock_capacity": spec.dock_capacity,
    }
    
    # Fat wing properties
    if spec.fat_wing:
        fw = spec.fat_wing
        d["fat_wing"] = {
            "span": fw.span,
            "chord": fw.chord,
            "taper": fw.taper,
            "thickness_ratio": fw.thickness_ratio,
            "n_cells": fw.n_cells,
            "pressure_bar": fw.pressure_bar,
            "fabric_areal_density": fw.fabric_areal_density,
            "fabric_strength_n_per_m": fw.fabric_strength_n_per_m,
            "dihedral_deg": fw.dihedral_deg,
            "volume": fw.volume,
            "mass": fw.mass,
            "planform_area": fw.planform_area,
        }
    
    # Canopy properties
    if spec.canopy:
        c = spec.canopy
        d["canopy"] = {
            "area": c.area,
            "span": c.span,
            "areal_density": c.areal_density,
            "cl_op": c.cl_op,
            "cl_max": c.cl_max,
            "cd_op": c.cd_op,
            "twin_skin": c.twin_skin,
            "mass": c.mass,
            "aspect_ratio": c.aspect_ratio,
        }
    
    # Fin properties
    if spec.fin:
        f = spec.fin
        d["fin"] = {
            "area": f.area,
            "arm": f.arm,
            "areal_density": f.areal_density,
            "mass": f.mass,
        }
    
    # Tether properties
    if spec.tether:
        t = spec.tether
        d["tether"] = {
            "length": t.length,
            "diameter_mm": t.diameter_mm,
            "linear_density": t.linear_density,
            "mbl_kn": t.mbl_kn,
            "safety_factor": t.safety_factor,
            "elevation_deg": t.elevation_deg,
            "mass": t.mass,
        }
    
    # Bridle properties
    if spec.bridle:
        b = spec.bridle
        d["bridle"] = {
            "positions": b.positions,
            "chord_fraction": b.chord_fraction,
            "control_mbl_kn": b.control_mbl_kn,
            "pod_standoff_m": b.pod_standoff_m,
        }
    
    # L0 report data if available
    if report:
        d["l0"] = {
            "buoyancy": {
                "gross_static_lift_kg": report.buoyancy.gross_static_lift_kg,
                "net_static_lift_kg": report.buoyancy.net_static_lift_kg,
                "net_incl_tether_kg": report.buoyancy.net_incl_tether_kg,
                "calm_air_capable": report.buoyancy.calm_air_capable,
                "structure_mass_kg": report.buoyancy.structure_mass_kg,
                "payload_mass_kg": report.buoyancy.payload_mass_kg,
                "tether_mass_kg": report.buoyancy.tether_mass_kg,
            },
            "envelope": {
                "v_min_ms": report.envelope.v_min_ms,
                "v_max_ms": report.envelope.v_max_ms,
                "v_max_limiter": report.envelope.v_max_limiter,
                "tow_force_at_12ms_kn": report.envelope.tow_force_at_12ms_kn,
                "vertical_capacity_at_10ms_kg": report.envelope.vertical_capacity_at_10ms_kg,
            },
            "structure": [
                {
                    "label": tube.label,
                    "hoop_stress_n_per_m": tube.hoop_stress_n_per_m,
                    "hoop_utilization": tube.hoop_utilization,
                    "wrinkle_moment_nm": tube.wrinkle_moment_nm,
                    "collapse_moment_nm": tube.collapse_moment_nm,
                    "applied_moment_nm": tube.applied_moment_nm,
                    "bending_utilization": tube.bending_utilization,
                }
                for tube in report.structure
            ],
        }
    
    return d


def main():
    # Load all Manta specs
    specs_dir = Path("specs/manta")
    manta_specs = {}
    
    for yaml_file in sorted(specs_dir.glob("*.yaml")):
        spec = load_spec(yaml_file)
        report = solve(spec)
        spec_dict = spec_to_dict(spec, report)
        manta_specs[spec.mk] = spec_dict
    
    # Create output
    output = {
        "manta_specs": manta_specs,
        "metadata": {
            "count": len(manta_specs),
            "types": list(manta_specs.keys()),
            "generated_by": "export_manta_specs.py",
        }
    }
    
    # Write JSON
    output_path = Path("godot/data/manta_specs.json")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(output, f, indent=2)
    
    print(f"Exported {len(manta_specs)} Manta specs to {output_path}")
    print(f"Types: {', '.join(manta_specs.keys())}")


if __name__ == "__main__":
    main()
