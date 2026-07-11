"""Figure-generation gates.

L0 figures must build on the default install; L1 figures skip without the
extra. These are smoke gates (figures build, contain marks, and write
non-empty files) — layout quality is reviewed by eye, not asserted.
"""
import pytest

from kytoon import viz
from kytoon.solvers.l0 import solve
from kytoon.spec import load_all

SPECS = "specs"


@pytest.fixture(scope="module")
def reports():
    return [solve(s) for s in load_all(SPECS)]


def _saves(fig, tmp_path, name):
    p = tmp_path / name
    fig.savefig(p)
    assert p.stat().st_size > 10_000  # a real raster, not an empty canvas
    return p


def test_fleet_envelopes_builds(reports, tmp_path):
    fig = viz.fig_fleet_envelopes(reports)
    ax = fig.axes[0]
    assert len(ax.patches) == len(reports)          # one range bar per Mk
    _saves(fig, tmp_path, "env.png")


def test_structure_margins_builds(reports, tmp_path):
    fig = viz.fig_structure_margins(reports)
    n_members = sum(len(r.structure) for r in reports)
    assert len(fig.axes[0].patches) >= n_members    # ≥ one bar per member
    _saves(fig, tmp_path, "margins.png")


def test_spec_names_survive_utf8(reports):
    """Regression: cp1252 default read mangled «» into Â«Â» on Windows."""
    assert all("«" in r.spec.name for r in reports)
    assert not any("Â" in r.spec.name for r in reports)


def test_polar_figure_builds(reports, tmp_path):
    pytest.importorskip("VSM")
    spec = next(r.spec for r in reports if r.spec.mk == "I")
    fig = viz.fig_polar(spec, n_panels_wing=20)
    assert len(fig.axes) == 2                       # CL–α + drag polar
    _saves(fig, tmp_path, "polar.png")


def test_tether_figure_builds(reports, tmp_path):
    pytest.importorskip("moorpy")
    spec = next(r.spec for r in reports if r.spec.mk == "IV")
    fig = viz.fig_tether_profiles(spec, winds=(4.0, 12.0))
    assert len(fig.axes[0].lines) >= 3              # 2 winds + spec chord
    _saves(fig, tmp_path, "tether.png")


def test_fleet_geometry_figure_builds(reports, tmp_path):
    pytest.importorskip("trimesh")
    fig = viz.fig_fleet_geometry([r.spec for r in reports])
    assert len(fig.axes) == len(reports)            # one panel per Mk
    _saves(fig, tmp_path, "geometry.png")
