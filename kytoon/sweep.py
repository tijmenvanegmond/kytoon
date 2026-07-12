"""Design-iteration sweep for a single spec (built for Mk V «Manta»).

One-at-a-time (OAT) grid: each swept field is varied independently around
the baseline spec while everything else stays fixed, so each panel/row
isolates one knob's effect — not a full factorial (a 4-factor factorial
would blow the L1/VSM budget for a handful of design candidates).

L0 (closed-form) is always solved. L1 aero (VSM, via kytoon.solvers.l1_aero)
is optional and slow — pass --l1 to also validate each point against the
Breukels-extrapolation flag on the fat-wing section.

Overriding fat_wing.chord/taper/span re-derives canopy.area to keep the
KytoonSpec validator's planform-agreement check satisfied (see spec.py
_check_archetype); overriding n_cells/pressure_bar does not touch canopy.

CLI: python -m kytoon.sweep specs/mk5_manta.yaml -o reports/mk5_sweep --l1
"""
from __future__ import annotations

import argparse
import copy
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import yaml

from kytoon.solvers.l0 import L0Report, solve as solve_l0
from kytoon.spec import KytoonSpec
from kytoon.viz import AXIS, INK, INK2, MK_COLOR, MUTED, SURFACE, _style

try:
    from kytoon.solvers.l1_aero import solve as solve_l1_aero
    HAS_L1 = True
except ImportError:  # default install is L0-only by design
    HAS_L1 = False

CRITICAL = "#d03b3b"

# param dotted-path -> (display label, off-baseline values to try either side,
# metric shown: "envelope" = v_max + net lift, "structure" = hoop/bend util —
# chord/taper move the envelope, n_cells/pressure only move structure margin
# since tether WLL (not wrinkle) binds v_max at baseline)
SWEEP_GRID: dict[str, tuple[str, list[float], str]] = {
    "fat_wing.chord": ("center chord [m]", [11.5, 15.0], "envelope"),
    "fat_wing.taper": ("taper λ", [0.25, 0.45], "envelope"),
    "fat_wing.n_cells": ("cells", [3, 7], "structure"),
    "fat_wing.pressure_bar": ("cell pressure [bar]", [0.07, 0.14], "structure"),
}


@dataclass
class SweepPoint:
    param: str
    value: float
    is_baseline: bool
    spec: KytoonSpec
    l0: L0Report
    l1_cr_ratio: float | None = None
    l1_flags: list[str] = field(default_factory=list)
    l1_error: str | None = None


def _get(d: dict, dotted: str):
    node = d
    for p in dotted.split("."):
        node = node[p]
    return node


def _apply(raw: dict, dotted: str, value) -> dict:
    d = copy.deepcopy(raw)
    node = d
    parts = dotted.split(".")
    for p in parts[:-1]:
        node = node[p]
    node[parts[-1]] = value
    if dotted in ("fat_wing.chord", "fat_wing.taper", "fat_wing.span") and d.get("canopy"):
        fw = d["fat_wing"]
        d["canopy"]["area"] = fw["span"] * fw["chord"] * (1 + fw["taper"]) / 2
    return d


def write_variant_specs(base_path: Path, out_dir: Path) -> list[Path]:
    """Persist baseline + every off-baseline grid point as real spec YAMLs
    (specs/manta/a.yaml, b.yaml, ...) — durable, diffable design artifacts
    per repo convention ("specs/*.yaml are THE design state"), not just
    ephemeral sweep output. Excluded from fleet gates the same way
    specs/alternates/ is: load_all() only globs the top-level specs/ dir."""
    raw = yaml.safe_load(base_path.read_text(encoding="utf-8"))
    out_dir.mkdir(parents=True, exist_ok=True)
    letters = iter("abcdefghijklmnopqrstuvwxyz")
    base_name = raw["name"]
    written: list[Path] = []

    def _dump(d: dict) -> Path:
        p = out_dir / f"{next(letters)}.yaml"
        p.write_text(yaml.safe_dump(d, sort_keys=False, allow_unicode=True), encoding="utf-8")
        written.append(p)
        return p

    base = copy.deepcopy(raw)
    base["notes"] = f"Sweep baseline, copied verbatim from {base_path.as_posix()}."
    _dump(base)

    for param, (label, values, _kind) in SWEEP_GRID.items():
        base_val = _get(raw, param)
        tag = param.rsplit(".", 1)[-1]
        for v in values:
            d = _apply(raw, param, v)
            d["name"] = f"{base_name} [{tag}={v}]"
            d["notes"] = (f"Sweep variant: {label} = {v} (baseline {base_val}); "
                          f"all else unchanged from {base_path.as_posix()}.")
            _dump(d)
    return written


def _solve_point(raw: dict, param: str, value, is_baseline: bool, with_l1: bool) -> SweepPoint:
    d = _apply(raw, param, value) if param else raw
    spec = KytoonSpec.model_validate(d)
    l0 = solve_l0(spec)
    cr_ratio, flags, err = None, [], None
    if with_l1:
        try:
            l1 = solve_l1_aero(spec)
            cr_ratio, flags = l1.cr_ratio, l1.flags
        except Exception as e:  # VSM can fail to converge at extreme geometry
            err = str(e)
    return SweepPoint(param=param or "baseline", value=value, is_baseline=is_baseline,
                       spec=spec, l0=l0, l1_cr_ratio=cr_ratio, l1_flags=flags, l1_error=err)


def run_sweep(base_path: Path, with_l1: bool = False) -> tuple[SweepPoint, dict[str, list[SweepPoint]]]:
    if with_l1 and not HAS_L1:
        raise ImportError(
            "L1 aero requested but the l1 extra (VSM) is not installed — "
            'pip install "kytoon-sim[l1]"'
        )
    raw = yaml.safe_load(base_path.read_text(encoding="utf-8"))
    baseline_spec = KytoonSpec.model_validate(raw)
    baseline = _solve_point(raw, "", None, True, with_l1)

    series: dict[str, list[SweepPoint]] = {}
    for param, (_, values, _kind) in SWEEP_GRID.items():
        base_val = _get(raw, param)
        pts = []
        for v in values:
            pts.append(_solve_point(raw, param, v, False, with_l1))
        # insert baseline in the middle for a continuous line, sorted by value
        base_pt = SweepPoint(param=param, value=base_val, is_baseline=True,
                              spec=baseline_spec, l0=baseline.l0,
                              l1_cr_ratio=baseline.l1_cr_ratio,
                              l1_flags=baseline.l1_flags, l1_error=baseline.l1_error)
        pts.append(base_pt)
        pts.sort(key=lambda p: p.value)
        series[param] = pts
    return baseline, series


# ---------------------------------------------------------------------------
def fig_sweep(series: dict[str, list[SweepPoint]], mk: str = "V"):
    """2x2 small multiples: v_max + net lift (incl tether) vs each param."""
    n = len(series)
    ncols = 2
    nrows = -(-n // ncols)
    fig, axes = plt.subplots(nrows, ncols, figsize=(11, 3.4 * nrows),
                             facecolor=SURFACE, dpi=150)
    axes = axes.flatten()
    color = MK_COLOR.get(mk, "#4a3aa7")

    for ax, (param, pts) in zip(axes, series.items()):
        _style(ax)
        label, _, kind = SWEEP_GRID[param]
        xs = [p.value for p in pts]
        fail = [not all(s.ok for s in p.l0.structure) for p in pts]
        base_x = next(p.value for p in pts if p.is_baseline)

        if kind == "envelope":
            y1 = [p.l0.envelope.v_max_ms for p in pts]
            y2 = [p.l0.buoyancy.net_incl_tether_kg for p in pts]
            y1_label, y2_label = "v_max [m/s]", "net lift [kg]"
        else:  # "structure"
            y1 = [max((s.hoop_utilization for s in p.l0.structure), default=0.0) * 100
                  for p in pts]
            y2 = [max((s.bending_utilization for s in p.l0.structure), default=0.0) * 100
                  for p in pts]
            y1_label, y2_label = "hoop util [%]", "bend util [%]"

        ax.plot(xs, y1, "-o", color=color, linewidth=1.6, markersize=5, zorder=3)
        for x, y, f in zip(xs, y1, fail):
            if f:
                ax.scatter([x], [y], color=CRITICAL, s=70, zorder=4, marker="x")
        if kind == "structure":
            ax.axhline(25, color=CRITICAL, linewidth=0.7, linestyle=(0, (2, 2)), zorder=1)

        ax2 = ax.twinx()
        ax2.plot(xs, y2, "--s", color=INK2, linewidth=1.2, markersize=4,
                 alpha=0.8, zorder=3)
        ax2.tick_params(colors=MUTED, labelsize=8, labelcolor=INK2)
        ax2.spines["top"].set_visible(False)
        for side in ("left", "right"):
            ax2.spines[side].set_color(AXIS)

        ax.axvline(base_x, color=MUTED, linewidth=0.8, linestyle=(0, (3, 3)), zorder=1)

        ax.set_xlabel(label, fontsize=9)
        ax.set_ylabel(y1_label, fontsize=8.5, color=color)
        ax2.set_ylabel(y2_label, fontsize=8.5, color=INK2)

    for ax in axes[n:]:
        ax.axis("off")

    fig.suptitle(f"Mk {mk} sweep vs baseline (dotted) — envelope panels: "
                  "v_max solid / net lift dashed. structure panels: hoop "
                  "solid / bend dashed, red = 25% hoop gate",
                  fontsize=10, x=0.02, ha="left", color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    return fig


# ---------------------------------------------------------------------------
def _fmt(x, nd=1):
    return "—" if x is None else f"{x:,.{nd}f}"


def sweep_report(baseline: SweepPoint, series: dict[str, list[SweepPoint]]) -> str:
    lines = [
        f"# {baseline.spec.name} — design sweep",
        "",
        f"Baseline: chord {baseline.spec.fat_wing.chord} m, "
        f"taper {baseline.spec.fat_wing.taper}, "
        f"{baseline.spec.fat_wing.n_cells} cells, "
        f"{baseline.spec.fat_wing.pressure_bar} bar — "
        f"v_max {_fmt(baseline.l0.envelope.v_max_ms)} m/s "
        f"({baseline.l0.envelope.v_max_limiter}), "
        f"net lift incl. tether {_fmt(baseline.l0.buoyancy.net_incl_tether_kg, 0)} kg"
        + (f", L1 cr_ratio {_fmt(baseline.l1_cr_ratio, 2)}"
           if baseline.l1_cr_ratio is not None else ""),
        "",
    ]
    for param, pts in series.items():
        label = SWEEP_GRID[param][0]
        lines.append(f"## {label} ({param})")
        lines.append("")
        hdr = ["value", "v_min", "v_max", "limiter", "tow@12 [kN]",
               "net lift incl tether [kg]", "hoop util", "bend util", "structure OK"]
        if any(p.l1_cr_ratio is not None or p.l1_error for p in pts):
            hdr += ["L1 cr_ratio", "L1 flags"]
        lines.append("| " + " | ".join(hdr) + " |")
        lines.append("|" + "---|" * len(hdr))
        for p in pts:
            e = p.l0.envelope
            ok = all(s.ok for s in p.l0.structure)
            hoop = max((s.hoop_utilization for s in p.l0.structure), default=float("nan"))
            bend = max((s.bending_utilization for s in p.l0.structure), default=float("nan"))
            row = [
                f"{p.value:g}" + (" (baseline)" if p.is_baseline else ""),
                _fmt(e.v_min_ms), _fmt(e.v_max_ms), e.v_max_limiter,
                _fmt(e.tow_force_at_12ms_kn), _fmt(p.l0.buoyancy.net_incl_tether_kg, 0),
                f"{hoop*100:.0f}%", f"{bend*100:.0f}%",
                "✔" if ok else "✘",
            ]
            if any(p2.l1_cr_ratio is not None or p2.l1_error for p2 in pts):
                if p.l1_error:
                    row += ["—", f"error: {p.l1_error}"]
                else:
                    row += [_fmt(p.l1_cr_ratio, 2), "; ".join(p.l1_flags) or "none"]
            lines.append("| " + " | ".join(row) + " |")
        lines.append("")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("spec", type=Path, help="baseline spec YAML")
    ap.add_argument("-o", "--out", type=Path, default=Path("reports/sweep"))
    ap.add_argument("--l1", action="store_true", help="also run L1 aero (VSM) per point")
    ap.add_argument("--write-specs", type=Path, default=None,
                    help="also persist each grid point as a real spec YAML "
                         "under this directory (e.g. specs/manta/)")
    args = ap.parse_args(argv)

    if args.write_specs:
        paths = write_variant_specs(args.spec, args.write_specs)
        for p in paths:
            print(f"wrote {p}")

    baseline, series = run_sweep(args.spec, with_l1=args.l1)

    args.out.mkdir(parents=True, exist_ok=True)
    stem = f"mk{baseline.spec.mk.lower()}_sweep"
    report_path = args.out / f"{stem}.md"
    report_path.write_text(sweep_report(baseline, series), encoding="utf-8")
    print(f"wrote {report_path}")

    fig = fig_sweep(series, mk=baseline.spec.mk)
    fig_path = args.out / f"{stem}.png"
    fig.savefig(fig_path, facecolor=SURFACE)
    plt.close(fig)
    print(f"wrote {fig_path}")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
