"""Comparison report generator for L0 results.

Usage:  python -m kytoon.report specs/ [-o reports/l0.md]
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
        ("Wing area [m²]", lambda r: _fmt(r.spec.wing_area, 0)),
        ("He volume [m³]", lambda r: _fmt(r.buoyancy.he_volume, 0)),
        ("Flying mass [kg]", lambda r: _fmt(r.spec.total_mass, 0)),
        ("Gross He lift [kg]", lambda r: _fmt(r.buoyancy.gross_static_lift_kg, 0)),
        ("Net static lift [kg]", lambda r: _fmt(r.buoyancy.net_static_lift_kg, 0)),
        ("… incl. tether [kg]", lambda r: _fmt(r.buoyancy.net_incl_tether_kg, 0)),
        ("Calm-air capable", lambda r: "✔" if r.buoyancy.calm_air_capable else "✘"),
        ("v_min [m/s]", lambda r: _fmt(r.envelope.v_min_ms)),
        ("v_max [m/s]", lambda r: _fmt(r.envelope.v_max_ms)),
        ("v_max limiter", lambda r: r.envelope.v_max_limiter),
        ("Tow @12 m/s [kN]", lambda r: _fmt(r.envelope.tow_force_at_12ms_kn)),
        ("Spare vert. lift @10 m/s [kg]",
         lambda r: _fmt(r.envelope.vertical_capacity_at_10ms_kg, 0)),
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
    for s in report.structure:
        lines.append(
            f"| {s.label} | {_fmt(s.hoop_stress_n_per_m/1e3, 2)} "
            f"| {_fmt(s.hoop_utilization*100)}% "
            f"| {_fmt(s.applied_moment_nm/1e3, 2)} "
            f"| {_fmt(s.wrinkle_moment_nm/1e3, 2)} "
            f"| {_fmt(s.bending_utilization*100)}% "
            f"| {'✔' if s.ok else '✘'} |"
        )
    return "\n".join(lines)


def full_report(reports: list[L0Report]) -> str:
    mks = [r.spec.mk for r in reports]
    parts = [
        f"# Kytoon L0 Analysis — Mk {mks[0]}–{mks[-1]}",
        "",
        "Closed-form layer: Archimedes buoyancy, pressurized-beam wrinkle "
        "margins, quasi-static wind envelope. All numbers ISA sea level; "
        "tether drag/sag deferred to L1 (MoorPy).",
        "",
        "## Comparison",
        "",
        comparison_table(reports),
        "",
        "## Structure margins",
        "",
    ]
    parts += [structure_detail(r) + "\n" for r in reports]
    parts += ["## Flags", ""]
    for r in reports:
        flags = []
        if not r.buoyancy.calm_air_capable and r.envelope.v_min_ms > 0:
            flags.append(f"needs ≥ {_fmt(r.envelope.v_min_ms)} m/s to stay aloft")
        bad = [s for s in r.structure if not s.ok]
        for s in bad:
            flags.append(f"{s.label}: over margin "
                         f"(bend {_fmt(s.bending_utilization*100)}%, "
                         f"hoop {_fmt(s.hoop_utilization*100)}%)")
        if r.buoyancy.net_incl_tether_kg < 0 and r.envelope.v_min_ms == 0:
            flags.append("buoyant but cannot lift full tether in calm air")
        parts.append(f"- **{r.spec.name}**: " + ("; ".join(flags) if flags else "no L0 flags"))
    return "\n".join(parts)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("specs_dir", type=Path)
    p.add_argument("-o", "--out", type=Path, default=None)
    args = p.parse_args(argv)

    reports = [solve(s) for s in load_all(args.specs_dir)]
    text = full_report(reports)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text, encoding="utf-8")  # report carries «»/✔✘
        print(f"wrote {args.out}")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
