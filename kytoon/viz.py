"""Solver-output figures for the L0/L1 reports.

Static matplotlib figures written to reports/figures/. Design rules follow
the project's dataviz conventions: fixed entity colors per Mk (never
re-derived from series order), one measure per axis, recessive grid/axes,
direct labels on every mark (two palette hues sit below 3:1 contrast on the
light surface — labels are the mandated relief).

L0 figures (fleet envelopes, structure margins) need only the default
install; the polar and tether-profile figures need the `l1` extra and are
skipped gracefully by the CLI when it is absent.

CLI: python -m kytoon.viz specs/ -o reports/figures
"""
from __future__ import annotations

import math
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # file output only; no display needed
import matplotlib.pyplot as plt
import numpy as np

from kytoon import aero
from kytoon.solvers.l0 import L0Report, solve
from kytoon.spec import KytoonSpec

# --- palette (validated: scripts/validate_palette.js, light surface) --------
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"
CRITICAL = "#d03b3b"          # status: gate exceeded — never used as a series
MK_COLOR = {"I": "#2a78d6", "II": "#1baf7a", "III": "#eda100",
            "IV": "#008300", "V": "#4a3aa7"}
SERIES = ["#2a78d6", "#1baf7a", "#eda100", "#008300"]   # fixed slot order
SEQ_BLUE = ["#86b6ef", "#5598e7", "#2a78d6", "#1c5cab", "#104281"]  # ordinal


def _style(ax) -> None:
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(AXIS)
        ax.spines[side].set_linewidth(0.8)
    ax.tick_params(colors=MUTED, labelsize=9, labelcolor=INK2)
    ax.grid(True, color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    ax.xaxis.label.set_color(INK2)
    ax.yaxis.label.set_color(INK2)
    ax.title.set_color(INK)


def _fig(w: float = 8.0, h: float = 4.5):
    fig = plt.figure(figsize=(w, h), facecolor=SURFACE, dpi=150)
    return fig


# ---------------------------------------------------------------------------
def fig_fleet_envelopes(reports: list[L0Report]):
    """Range bars: each Mk's wind envelope, limiter annotated, 20 m/s gate."""
    fig = _fig(8.0, 3.6)
    ax = fig.add_subplot(111)
    _style(ax)

    ys = np.arange(len(reports))[::-1]
    for y, rep in zip(ys, reports):
        mk = rep.spec.mk
        env = rep.envelope
        ax.barh(y, env.v_max_ms - env.v_min_ms, left=env.v_min_ms, height=0.5,
                color=MK_COLOR.get(mk, SERIES[0]), edgecolor=SURFACE,
                linewidth=2, zorder=3)
        ax.text(-0.6, y, rep.spec.name, ha="right", va="center",
                fontsize=10, color=INK)
        note = f"{env.v_min_ms:.1f}–{env.v_max_ms:.1f} m/s · {env.v_max_limiter}"
        if env.v_mission_ms < env.v_max_ms - 0.05:
            # blow-down caps the useful range before the line breaks:
            # hatch the flattered part of the bar and say so
            ax.barh(y, env.v_max_ms - env.v_mission_ms, left=env.v_mission_ms,
                    height=0.5, color=SURFACE, alpha=0.55, edgecolor="none",
                    zorder=4)
            ax.plot([env.v_mission_ms] * 2, [y - 0.25, y + 0.25],
                    color=INK, lw=1.4, zorder=5)
            note += f" · mission ≤ {env.v_mission_ms:.1f} (elev < 45°)"
        ax.text(env.v_max_ms + 0.5, y, note,
                ha="left", va="center", fontsize=8.5, color=INK2)

    ax.axvline(20, color=MUTED, linewidth=0.8, linestyle=(0, (4, 3)), zorder=2)
    ax.text(20.4, -0.62, "fleet must cover 0–20 m/s", fontsize=8.5,
            color=MUTED, ha="left", va="center")
    ax.set_ylim(-0.85, len(reports) - 0.5)
    ax.set_yticks([])
    ax.set_xlim(0, max(r.envelope.v_max_ms for r in reports) * 1.28)
    ax.set_xlabel("wind speed [m/s]")
    ax.set_title("Fleet wind envelopes (L0)", fontsize=11, loc="left")
    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
def fig_structure_margins(reports: list[L0Report]):
    """Utilization bars per member: hoop vs its 0.25 gate, bending vs 1.0."""
    rows = [(rep.spec.mk, m) for rep in reports for m in rep.structure]
    fig = _fig(8.0, 0.62 * len(rows) + 1.6)
    ax = fig.add_subplot(111)
    _style(ax)

    labels, ys = [], []
    for i, (mk, m) in enumerate(rows):
        y = len(rows) - 1 - i
        ys.append(y)
        labels.append(f"Mk {mk} · {m.label}")
        pairs = [(m.hoop_utilization, 0.25, -0.16, "hoop")]
        if not math.isnan(m.bending_utilization) and m.bending_utilization > 0:
            pairs.append((m.bending_utilization, 1.0, 0.16, "bending"))
        for util, gate, dy, kind in pairs:
            color = SERIES[0] if kind == "hoop" else SERIES[1]
            if util > gate:
                color = CRITICAL
            ax.barh(y + dy, util, height=0.28, color=color,
                    edgecolor=SURFACE, linewidth=1.5, zorder=3)
            ax.text(util + 0.015, y + dy,
                    f"{kind} {util*100:.0f}%" + (" ⚠" if util > gate else ""),
                    va="center", fontsize=8, color=INK2)

    ax.axvline(0.25, color=SERIES[0], linewidth=0.8, linestyle=(0, (4, 3)),
               zorder=2, alpha=0.7)
    ax.axvline(1.0, color=SERIES[1], linewidth=0.8, linestyle=(0, (4, 3)),
               zorder=2, alpha=0.7)
    ax.text(0.25, -0.72, "hoop gate (SF 4)", fontsize=8,
            color=MUTED, ha="center", va="center")
    ax.text(1.0, -0.72, "wrinkle onset", fontsize=8,
            color=MUTED, ha="center", va="center")
    ax.set_ylim(-1.0, len(rows) - 0.45)
    ax.set_yticks(ys, labels, fontsize=9, color=INK)
    ax.set_xlim(0, 1.25)
    ax.set_xlabel("utilization at 12 m/s tow [-]")
    ax.set_title("Structure margins (L0, pressurized-beam theory)",
                 fontsize=11, loc="left")
    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
def fig_polar(spec: KytoonSpec, n_panels_wing: int = 40):
    """L1 VSM polar vs the vendored V3 benchmark. Needs the l1 extra."""
    from kytoon.solvers.l1_aero import solve as l1_solve

    rep = l1_solve(spec, n_panels=n_panels_wing)
    wt = aero.wind_tunnel()
    cfd = aero.cfd_re1e6()

    fig = _fig(9.5, 4.2)
    ax1 = fig.add_subplot(121)
    ax2 = fig.add_subplot(122)
    for ax in (ax1, ax2):
        _style(ax)

    a = [p.alpha for p in rep.clean.pts]
    cl = [p.cl for p in rep.clean.pts]
    cd_sys = [rep.system.cd(x) for x in a]

    # CL–alpha
    ax1.plot([p.alpha for p in wt.pts], [p.cl for p in wt.pts], "o",
             ms=4.5, color=SERIES[1], label="V3 wind tunnel (Poland)")
    ax1.plot([p.alpha for p in cfd.pts], [p.cl for p in cfd.pts], "^",
             ms=4.5, color=SERIES[2], label="V3 CFD Re1e6 (Viré)")
    ax1.plot(a, cl, color=SERIES[0], lw=2, label=f"{spec.name} VSM (clean)")
    if rep.op_alpha is not None:
        ax1.plot(rep.op_alpha, rep.cl_op_l1, "*", ms=13, color=INK, zorder=5)
        ax1.annotate(f"op  α={rep.op_alpha:.1f}°",
                     (rep.op_alpha, rep.cl_op_l1), textcoords="offset points",
                     xytext=(8, -12), fontsize=8.5, color=INK)
    ax1.set_xlabel("α [deg]")
    ax1.set_ylabel("CL [-]")
    ax1.legend(fontsize=8, frameon=False, labelcolor=INK2, loc="lower right")

    # drag polar
    ax2.plot([p.cd for p in wt.pts], [p.cl for p in wt.pts], "o",
             ms=4.5, color=SERIES[1])
    ax2.plot([p.cd for p in cfd.pts], [p.cl for p in cfd.pts], "^",
             ms=4.5, color=SERIES[2])
    ax2.plot([p.cd for p in rep.clean.pts], cl, color=SERIES[0], lw=2)
    ax2.plot(cd_sys, cl, color=SERIES[0], lw=2, linestyle=(0, (4, 3)),
             label="+ bridle drag (system)")
    ax2.plot(spec.canopy.cd_op, spec.canopy.cl_op, "s", ms=8, mfc="none",
             mec=INK, mew=1.5, zorder=5)
    ax2.annotate("spec cl/cd_op", (spec.canopy.cd_op, spec.canopy.cl_op),
                 textcoords="offset points", xytext=(8, -4),
                 fontsize=8.5, color=INK)
    ax2.set_xlabel("CD [-]")
    ax2.set_ylabel("CL [-]")
    ax2.legend(fontsize=8, frameon=False, labelcolor=INK2, loc="lower right")

    fig.suptitle(f"{spec.name} — L1 polar vs TU Delft V3 benchmark",
                 fontsize=11, color=INK, x=0.02, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    return fig


# ---------------------------------------------------------------------------
def fig_tether_profiles(spec: KytoonSpec, winds=(4.0, 8.0, 12.0, 16.0)):
    """Quasi-static line shapes over wind speed. Needs the l1 extra."""
    from kytoon.solvers.l1_tether import solve as tether_solve

    fig = _fig(7.0, 5.2)
    ax = fig.add_subplot(111)
    _style(ax)

    ramp = SEQ_BLUE[-len(winds):] if len(winds) <= len(SEQ_BLUE) else SEQ_BLUE
    drew = 0
    for i, (v, color) in enumerate(zip(winds, ramp)):
        rep = tether_solve(spec, v)
        if rep.line_xz is None or rep.altitude <= 0 or not rep.converged:
            continue
        ax.plot(rep.line_xz[:, 0], rep.line_xz[:, 1], color=color, lw=2)
        # curves converge near the force-balance angle at higher winds —
        # stagger the labels down the line so they never stack
        frac = 0.92 - 0.16 * i
        node = rep.line_xz[int(frac * (len(rep.line_xz) - 1))]
        ax.annotate(f"{v:g} m/s", node, textcoords="offset points",
                    xytext=(8, 0), fontsize=8.5, color=INK2)
        drew += 1

    t = spec.tether
    e = math.radians(t.elevation_deg)
    ax.plot([0, t.length * math.cos(e)], [0, t.length * math.sin(e)],
            color=MUTED, lw=1, linestyle=(0, (4, 3)))
    ax.annotate(f"spec straight line @ {t.elevation_deg:g}°",
                (t.length * math.cos(e), t.length * math.sin(e)),
                textcoords="offset points", xytext=(6, 2),
                fontsize=8.5, color=MUTED)

    ax.set_aspect("equal")
    ax.set_xlabel("downwind [m]")
    ax.set_ylabel("height above winch [m]")
    ax.set_title(f"{spec.name} — tether shape vs wind (L1, MoorPy)",
                 fontsize=11, loc="left")
    fig.tight_layout()
    return fig if drew else fig   # figure returned even if sparse; CLI notes it


# ---------------------------------------------------------------------------
def fig_fleet_geometry(specs: list[KytoonSpec]):
    """Shaded 3/4 view of every Mk's 3D geometry. Needs the l1 extra."""
    from kytoon.geometry import build

    fig = _fig(3.2 * len(specs), 3.4)
    light = np.array([0.3, -0.5, 0.8])
    light = light / np.linalg.norm(light)

    for i, spec in enumerate(specs):
        ax = fig.add_subplot(1, len(specs), i + 1, projection="3d")
        ax.set_facecolor(SURFACE)
        base = np.array(matplotlib.colors.to_rgb(
            MK_COLOR.get(spec.mk, SERIES[0])))
        scene = build(spec)

        tris, cols = [], []
        for name, mesh in scene.geometry.items():
            t = mesh.vertices[mesh.faces]
            n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
            norms = np.linalg.norm(n, axis=1, keepdims=True)
            n = n / np.where(norms > 0, norms, 1)
            lam = 0.4 + 0.6 * np.abs(n @ light)   # double-sided lambert
            # soft goods a lighter tint; hardware (pod) in ink
            if name in ("canopy", "keel_wing") or name.startswith("wing"):
                c = 0.55 * base + 0.45
            elif name == "pod":
                c = np.array(matplotlib.colors.to_rgb(INK2))
            else:
                c = base
            tris.append(t)
            cols.append(np.clip(lam[:, None] * c, 0, 1))
        tris = np.concatenate(tris)
        cols = np.concatenate(cols)

        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        ax.add_collection3d(Poly3DCollection(tris, facecolors=cols,
                                             edgecolor="none"))
        lo = tris.reshape(-1, 3).min(axis=0)
        hi = tris.reshape(-1, 3).max(axis=0)
        c0 = (lo + hi) / 2
        r = (hi - lo).max() / 2
        ax.set_xlim(c0[0] - r, c0[0] + r)
        ax.set_ylim(c0[1] - r, c0[1] + r)
        ax.set_zlim(c0[2] - r, c0[2] + r)
        ax.set_box_aspect((1, 1, 1))
        ax.set_proj_type("ortho")
        ax.view_init(elev=16, azim=-55)
        ax.set_axis_off()
        ax.set_title(spec.name, fontsize=9.5, color=INK)

    fig.suptitle("Fleet geometry (spec-derived, models/*.glb)",
                 fontsize=11, color=INK, x=0.02, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    return fig


def _slug(name: str) -> str:
    """Filesystem-safe stem: spec names carry «»/spaces/brackets that make
    poor filenames (and are risky to shell out with) — collapse to snake_case."""
    return re.sub(r"\W+", "_", name, flags=re.UNICODE).strip("_").lower() or "spec"


# ---------------------------------------------------------------------------
def generate_all(spec_dir: str | Path, out_dir: str | Path) -> list[Path]:
    """Write every figure the installed extras allow. Returns paths written.

    Polar/tether figures are attempted per spec (not hardcoded per Mk) so
    this works generically on any spec directory, e.g. specs/manta/. A
    missing l1 extra is detected once per figure kind and then skipped for
    the rest of the run rather than re-raising per spec; an archetype that
    genuinely doesn't support a figure (e.g. torus has no wing — l1_aero
    raises ValueError) is skipped with a message, not swallowed silently.
    """
    from kytoon.spec import load_all

    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    specs = load_all(spec_dir)
    reports = [solve(s) for s in specs]
    written: list[Path] = []

    def save(fig, name: str):
        p = out / name
        fig.savefig(p, facecolor=SURFACE, bbox_inches="tight")
        plt.close(fig)
        written.append(p)

    save(fig_fleet_envelopes(reports), "fleet_envelopes.png")
    save(fig_structure_margins(reports), "structure_margins.png")

    l1_aero_available = True
    l1_tether_available = True
    for spec in specs:
        slug = _slug(spec.name)
        if l1_aero_available:
            try:
                save(fig_polar(spec), f"polar_{slug}.png")
            except ImportError:
                print("l1 extra (VSM) absent — skipping polar figures")
                l1_aero_available = False
            except ValueError as e:
                print(f"{spec.name}: polar skipped — {e}")
        if l1_tether_available:
            try:
                save(fig_tether_profiles(spec), f"tether_{slug}.png")
            except ImportError:
                print("l1 extra (moorpy) absent — skipping tether figures")
                l1_tether_available = False
            except ValueError as e:
                print(f"{spec.name}: tether skipped — {e}")

    try:
        save(fig_fleet_geometry(specs), "fleet_geometry.png")
    except ImportError:
        print("l1 extra (trimesh) absent — skipping geometry figure")
    return written


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="Generate solver-output figures")
    ap.add_argument("specs", help="directory of spec YAMLs")
    ap.add_argument("-o", "--out", default="reports/figures")
    args = ap.parse_args()
    for p in generate_all(args.specs, args.out):
        print(p)
