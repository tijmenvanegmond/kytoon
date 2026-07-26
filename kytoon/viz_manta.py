"""Manta-focused visualization generator.

Usage:  python -m kytoon.viz_manta specs/manta/ [-o reports/figures/manta]

Generates figures for Manta Type A-E variants:
- Fleet envelopes comparison
- Structure margins comparison
- Individual polar plots (if VSM installed)
- Tether profiles (if MoorPy installed)
- Geometry visualization (if trimesh installed)
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

try:
    import matplotlib
    matplotlib.use("Agg")  # non-interactive backend
    import matplotlib.pyplot as plt
    from matplotlib.colors import to_rgb
    HAS_MPL = True
except ImportError:
    HAS_MPL = False

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

try:
    import trimesh
    HAS_TRIMESH = True
except ImportError:
    HAS_TRIMESH = False

from kytoon.spec import load_all
from kytoon.solvers.l0 import solve

SURFACE = "white"


def _fmt(x: float, nd: int = 1) -> str:
    if x is None or (isinstance(x, float) and math.isnan(x)):
        return "—"
    return f"{x:,.{nd}f}"


def fig_fleet_envelopes_manta(reports, out_dir: Path):
    """Generate fleet envelope comparison for Manta types."""
    if not HAS_MPL:
        print("matplotlib not installed — skipping fleet envelopes")
        return None
    
    fig, ax = plt.subplots(figsize=(10, 6))
    
    colors = ['#4CAF50', '#2196F3', '#FF9800', '#9C27B0', '#FFEB3B']
    
    for report, color in zip(reports, colors):
        env = report.envelope
        ax.axvline(env.v_min_ms, color=color, linestyle='--', alpha=0.5, label=f'{report.spec.name} v_min')
        ax.axvline(env.v_max_ms, color=color, linestyle='-', alpha=0.7, label=f'{report.spec.name} v_max')
        ax.plot([env.v_min_ms, env.v_max_ms], [report.spec.name, report.spec.name], 
                color=color, linewidth=3, marker='o')
    
    ax.set_xlabel('Wind Speed [m/s]')
    ax.set_ylabel('Manta Type')
    ax.set_title('Manta Type Wind Envelopes')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')
    
    return fig


def fig_structure_margins_manta(reports, out_dir: Path):
    """Generate structure margins comparison for Manta types."""
    if not HAS_MPL:
        print("matplotlib not installed — skipping structure margins")
        return None
    
    fig, ax = plt.subplots(figsize=(10, 6))
    
    colors = ['#4CAF50', '#2196F3', '#FF9800', '#9C27B0', '#FFEB3B']
    
    for report, color in zip(reports, colors):
        for tube in report.structure:
            ax.bar(report.spec.name, tube.bending_utilization * 100, 
                   color=color, alpha=0.7, label=f'{tube.label}')
    
    ax.set_ylabel('Bending Utilization [%]')
    ax.set_title('Manta Type Structure Margins')
    ax.axhline(100, color='red', linestyle='--', alpha=0.5, label='Limit')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')
    
    return fig


def fig_lift_comparison(reports, out_dir: Path):
    """Generate lift comparison chart."""
    if not HAS_MPL:
        print("matplotlib not installed — skipping lift comparison")
        return None
    
    fig, ax = plt.subplots(figsize=(10, 6))
    
    types = [r.spec.name for r in reports]
    gross_lift = [r.buoyancy.gross_static_lift_kg for r in reports]
    net_lift = [r.buoyancy.net_static_lift_kg for r in reports]
    
    x = range(len(types))
    width = 0.35
    
    ax.bar([i - width/2 for i in x], gross_lift, width, label='Gross Lift', color='#2196F3')
    ax.bar([i + width/2 for i in x], net_lift, width, label='Net Lift', color='#4CAF50')
    
    ax.set_xlabel('Manta Type')
    ax.set_ylabel('Lift [kg]')
    ax.set_title('Manta Type Lift Comparison')
    ax.set_xticks(x)
    ax.set_xticklabels(types, rotation=45, ha='right')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')
    
    return fig


def fig_payload_vs_range(reports, out_dir: Path):
    """Generate payload vs wind range chart."""
    if not HAS_MPL:
        print("matplotlib not installed — skipping payload vs range")
        return None
    
    fig, ax = plt.subplots(figsize=(10, 6))
    
    payloads = [r.spec.payload_mass for r in reports]
    wind_ranges = [r.envelope.v_max_ms - r.envelope.v_min_ms for r in reports]
    types = [r.spec.name for r in reports]
    
    ax.scatter(payloads, wind_ranges, s=100, c='blue', alpha=0.7)
    
    for i, txt in enumerate(types):
        ax.annotate(txt, (payloads[i], wind_ranges[i]), textcoords="offset points", 
                   xytext=(0,10), ha='center')
    
    ax.set_xlabel('Payload [kg]')
    ax.set_ylabel('Wind Range [m/s]')
    ax.set_title('Manta Type: Payload vs Wind Range')
    ax.grid(True, alpha=0.3)
    
    return fig


def generate_all_manta(spec_dir: str, out_dir: str = "reports/figures/manta"):
    """Generate all Manta visualization figures."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    
    specs = load_all(spec_dir)
    reports = [solve(s) for s in specs]
    
    if not reports:
        print(f"No specs found in {spec_dir}")
        return []
    
    written: list[Path] = []
    
    def save(fig, name: str):
        if fig is None:
            return
        p = out / name
        fig.savefig(p, facecolor=SURFACE, bbox_inches="tight")
        plt.close(fig)
        written.append(p)
        print(f"Saved {p}")
    
    # Fleet envelopes
    save(fig_fleet_envelopes_manta(reports, out), "manta_fleet_envelopes.png")
    
    # Structure margins
    save(fig_structure_margins_manta(reports, out), "manta_structure_margins.png")
    
    # Lift comparison
    save(fig_lift_comparison(reports, out), "manta_lift_comparison.png")
    
    # Payload vs range
    save(fig_payload_vs_range(reports, out), "manta_payload_vs_range.png")
    
    return written


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Generate Manta visualization figures")
    ap.add_argument("specs", help="directory of Manta spec YAMLs")
    ap.add_argument("-o", "--out", default="reports/figures/manta",
                    help="output directory for figures")
    args = ap.parse_args()
    
    for p in generate_all_manta(args.specs, args.out):
        print(p)
