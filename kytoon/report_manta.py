"""Manta-focused comparison report generator.

Usage:  python -m kytoon.report_manta specs/manta/ [-o reports/manta.md]
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

from kytoon.solvers.l0 import L0Report, solve
from kytoon.spec import load_all


def _fmt(x: float, nd: int = 1) -> str:
    if x is None or (isinstance(x, float) and math.isnan(x)):
        return "—"
    return f"{x:,.{nd}f}"


def comparison_table(reports: list[L0Report]) -> str:
    rows = [
        ("Type", lambda r: r.spec.mk),
        ("Name", lambda r: r.spec.name),
        ("Wing area [m²]", lambda r: _fmt(r.spec.wing_area, 0)),
        ("He volume [m³]", lambda r: _fmt(r.buoyancy.he_volume, 0)),
        ("Flying mass [kg]", lambda r: _fmt(r.spec.total_mass, 0)),
        ("Gross He lift [kg]", lambda r: _fmt(r.buoyancy.gross_static_lift_kg, 0)),
        ("Net static lift [kg]", lambda r: _fmt(r.buoyancy.net_static_lift_kg, 0)),
        ("Net incl. tether [kg]", lambda r: _fmt(r.buoyancy.net_incl_tether_kg, 0)),
        ("Calm-air capable", lambda r: "✔" if r.buoyancy.calm_air_capable else "✘"),
        ("v_min [m/s]", lambda r: _fmt(r.envelope.v_min_ms)),
        ("v_max [m/s]", lambda r: _fmt(r.envelope.v_max_ms)),
        ("v_max limiter", lambda r: r.envelope.v_max_limiter),
        ("Tow @12 m/s [kN]", lambda r: _fmt(r.envelope.tow_force_at_12ms_kn)),
        ("Spare vert. lift @10 m/s [kg]",
         lambda r: _fmt(r.envelope.vertical_capacity_at_10ms_kg, 0)),
        ("Payload [kg]", lambda r: _fmt(r.spec.payload_mass, 0)),
        ("Tether length [m]", lambda r: _fmt(r.spec.tether.length, 0)),
    ]
    hdr = "| Parameter | " + " | ".join(r.spec.name for r in reports) + " |"
    sep = "|---" * (len(reports) + 1) + "|"
    lines = [hdr, sep]
    for label, fn in rows:
        lines.append(f"| {label} | " + " | ".join(fn(r) for r in reports) + " |")
    return "\n".join(lines)


def structure_detail(report: L0Report) -> str:
    lines = [f"### {report.spec.name} — structure @ tow "
             f"{_fmt(report.envelope.tow_force_at_12ms_kn)} kN (12 m/s)"]
    lines.append("| Member | Hoop [kN/m] | Hoop util | M_applied [kN·m] "
                 "| M_wrinkle [kN·m] | Bending util | OK |")
    lines.append("|---|---|---|---|---|---|---|")
    for tube in report.structure:
        ok = "✔" if tube.bending_utilization < 1.0 else "✘"
        hoop_k = tube.hoop_stress_n_per_m / 1000
        hoop_pct = tube.hoop_utilization * 100
        m_app = tube.applied_moment_nm / 1000
        m_wr = tube.wrinkle_moment_nm / 1000
        bend_pct = tube.bending_utilization * 100
        lines.append(
            f"| {tube.label} | {hoop_k:,.1f} | {hoop_pct:.1f}% | "
            f"{m_app:,.2f} | {m_wr:,.2f} | {bend_pct:.1f}% | {ok} |"
        )
    return "\n".join(lines)


def envelope_detail(report: L0Report) -> str:
    env = report.envelope
    lines = [
        f"### {report.spec.name} — wind envelope",
        "",
        f"- **Operating range**: {_fmt(env.v_min_ms)} – {_fmt(env.v_max_ms)} m/s",
        f"- **v_max limited by**: {env.v_max_limiter}",
        f"- **Tow force @ 12 m/s**: {_fmt(env.tow_force_at_12ms_kn)} kN",
        f"- **Vertical capacity @ 10 m/s**: {_fmt(env.vertical_capacity_at_10ms_kg)} kg",
    ]
    return "\n".join(lines)


def flags(report: L0Report) -> list[str]:
    flags = []
    if not report.buoyancy.calm_air_capable:
        needs = report.envelope.v_min_ms
        flags.append(f"needs ≥ {needs:.1f} m/s to stay aloft")
    if report.envelope.v_max_limiter.startswith("tether"):
        flags.append("tether-limited top speed")
    if report.envelope.v_max_limiter.startswith("spar") or \
       report.envelope.v_max_limiter.startswith("LE wrinkle"):
        flags.append("structure-limited top speed")
    if report.envelope.v_max_limiter.startswith("envelope"):
        flags.append("aerodynamic dent limit")
    return flags


def generate_report(specs_dir: str, output_path: str | None = None) -> str:
    """Generate a markdown report comparing all Manta variants."""
    # Load all Manta specs
    specs_path = Path(specs_dir)
    reports = []
    for yaml_file in sorted(specs_path.glob("*.yaml")):
        from kytoon.spec import load_spec
        spec = load_spec(yaml_file)
        report = solve(spec)
        reports.append(report)

    if not reports:
        return "No Manta specs found in {specs_dir}"

    lines = []
    lines.append("# Manta Type Comparison Report")
    lines.append("")
    lines.append("L0 analytic design layer for Manta Type A-Z variants.")
    lines.append("Closed-form physics: Archimedes buoyancy, pressurized-beam wrinkle margins,")
    lines.append("quasi-static wind envelope. All numbers ISA sea level.")
    lines.append("")

    # Comparison table
    lines.append("## Comparison")
    lines.append("")
    lines.append(comparison_table(reports))
    lines.append("")

    # Structure details
    lines.append("## Structure Margins")
    lines.append("")
    for report in reports:
        lines.append("")
        lines.append(structure_detail(report))
        lines.append("")

    # Envelope details
    lines.append("## Wind Envelopes")
    lines.append("")
    for report in reports:
        lines.append("")
        lines.append(envelope_detail(report))
        lines.append("")

    # Flags
    lines.append("## Design Notes")
    lines.append("")
    for report in reports:
        f = flags(report)
        if f:
            lines.append(f"- **{report.spec.name}**: {', '.join(f)}")
        else:
            lines.append(f"- **{report.spec.name}**: no L0 flags")

    report_text = "\n".join(lines)

    if output_path:
        with open(output_path, "w", encoding="utf-8") as f:
            f.write(report_text)
        print(f"wrote {output_path}")

    return report_text


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate Manta comparison report")
    parser.add_argument("specs_dir", nargs="?", default="specs/manta/",
                        help="Directory containing Manta YAML specs")
    parser.add_argument("-o", "--output", default=None,
                        help="Output markdown file path")
    args = parser.parse_args()

    generate_report(args.specs_dir, args.output)
