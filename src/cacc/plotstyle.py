"""Shared publication-quality matplotlib house style for CACC figures.

One place that defines the project's visual identity so every figure in
`scripts/` (reproduction, QoS study, certification snapshots) reads as one
coherent deck. Import and call :func:`use_house_style` right after
``matplotlib.use("Agg")``; use the semantic colour roles in :data:`C` and the
:func:`zone_span` / :func:`vehicle_colors` helpers instead of ad-hoc hex.

The categorical colours are drawn from a CVD-validated palette (blue / orange /
green / red — validated as a set: all pass the lightness, chroma and contrast
checks; the green-orange adjacency sits in the 8-12 CVD floor band, which is why
every comparison figure *also* separates the arms by line style + legend, never
colour alone).
"""

from __future__ import annotations

import matplotlib as mpl
import numpy as np

# ---------------------------------------------------------------- ink & chrome
INK = "#1a1a19"        # primary text
INK_SOFT = "#52514e"   # secondary text / axis labels
MUTED = "#8a8880"      # tick labels
GRID = "#e4e3dc"       # hairline gridlines
AXIS = "#c3c2b7"       # spines / baselines
SURFACE = "#ffffff"    # figure & axes face

# ------------------------------------------------- categorical palette (roles)
BLUE = "#2a78d6"
ORANGE = "#e8722e"
GREEN = "#0f8a3c"
RED = "#e0403f"
VIOLET = "#4a3aa7"
YELLOW = "#e0a000"
AQUA = "#1baf7a"

#: Semantic colour roles — use these, not raw hex, so the whole deck stays
#: consistent (adaptive / "our method" is always green; baselines blue; the
#: failing or worst-case arm warm).
C = {
    "fixed_good": BLUE,     # the reproduced baseline design
    "fixed_worst": ORANGE,  # the over-conservative baseline
    "adaptive": GREEN,      # the novel QoS-adaptive controller (hero)
    "fail": RED,            # the arm that hits the wall / goes unstable
    "fix": GREEN,           # the arm that restores the margin
    "plain": BLUE,          # plain-adaptive reference
    "smooth": GREEN,        # smoothed / staggered variant
    "reference": INK_SOFT,  # neutral reference / annotation lines
}

#: Warm tint for the "degraded channel" interference zone band (a warning
#: state, always shipped with a text label — never colour alone).
ZONE_FILL = "#f6b93b"


def use_house_style() -> None:
    """Install the house rcParams globally for the current process."""
    mpl.rcParams.update({
        # typography
        "font.family": "sans-serif",
        "font.sans-serif": ["Segoe UI", "Calibri", "DejaVu Sans", "Arial"],
        "font.size": 11.0,
        "mathtext.fontset": "dejavusans",
        "axes.titlesize": 11.5,
        "axes.titleweight": "medium",
        "axes.titlepad": 8.0,
        "axes.labelsize": 10.0,
        "axes.labelcolor": INK_SOFT,
        "axes.labelpad": 4.0,
        "figure.titlesize": 13.5,
        "figure.titleweight": "bold",
        # colours & surfaces
        "figure.facecolor": SURFACE,
        "axes.facecolor": SURFACE,
        "savefig.facecolor": SURFACE,
        "text.color": INK,
        "axes.edgecolor": AXIS,
        "axes.labelweight": "regular",
        # spines: keep only the reading axes
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.linewidth": 1.0,
        # grid — recessive hairline behind the data
        "axes.grid": True,
        "axes.axisbelow": True,
        "grid.color": GRID,
        "grid.linewidth": 0.8,
        "grid.alpha": 1.0,
        # ticks
        "xtick.color": MUTED,
        "ytick.color": MUTED,
        "xtick.labelcolor": INK_SOFT,
        "ytick.labelcolor": INK_SOFT,
        "xtick.labelsize": 9.0,
        "ytick.labelsize": 9.0,
        "xtick.direction": "out",
        "ytick.direction": "out",
        "xtick.major.size": 3.5,
        "ytick.major.size": 3.5,
        "xtick.major.width": 0.9,
        "ytick.major.width": 0.9,
        # lines & markers
        "lines.linewidth": 1.7,
        "lines.solid_capstyle": "round",
        "lines.markersize": 5.0,
        "lines.markeredgewidth": 0.0,
        # legend
        "legend.frameon": True,
        "legend.framealpha": 0.92,
        "legend.facecolor": SURFACE,
        "legend.edgecolor": GRID,
        "legend.fontsize": 8.5,
        "legend.borderpad": 0.5,
        "legend.handlelength": 1.8,
        "legend.columnspacing": 1.2,
        # figure output
        "figure.dpi": 120,
        "savefig.dpi": 170,
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.18,
        # colour cycle (validated categorical order) for any un-coloured plot
        "axes.prop_cycle": mpl.cycler(
            color=[BLUE, ORANGE, GREEN, RED, VIOLET, YELLOW, AQUA]),
    })


def vehicle_colors(n: int):
    """Perceptually-uniform, CVD-safe ramp for the *n* per-vehicle traces.

    viridis is kept for the many-line (up to 12) ordinal encoding, where a
    single-hue ramp would lose its steps — this is a deliberate exception to
    the single-hue rule for ordered small-multiple traces.
    """
    return mpl.cm.viridis(np.linspace(0.06, 0.86, n))


def zone_span(ax, t0: float, t1: float, *, label: str | None = None,
              first: bool = True) -> None:
    """Shade an interference / degraded-channel window [t0, t1] on *ax*.

    A soft warm band replaces bare dotted vertical lines — it reads as a
    labelled region, not a pair of unrelated markers. Pass ``first=True`` on
    the top axis of a stack to draw the text tag once.
    """
    ax.axvspan(t0, t1, color=ZONE_FILL, alpha=0.14, lw=0, zorder=0)
    for x in (t0, t1):
        ax.axvline(x, color=ZONE_FILL, lw=1.0, alpha=0.55, zorder=0.5)
    if label and first:
        ax.text((t0 + t1) / 2, 0.965, label, transform=_blend(ax),
                ha="center", va="top", fontsize=8.0, color="#9a6b00",
                fontweight="medium",
                bbox=dict(boxstyle="round,pad=0.25", fc="#fff6e0",
                          ec="#f0d089", lw=0.8, alpha=0.95))


def _blend(ax):
    import matplotlib.transforms as mtransforms
    return mtransforms.blended_transform_factory(ax.transData, ax.transAxes)


def finalize(fig, *, top: float = 0.95) -> None:
    """tight_layout leaving room for a suptitle."""
    fig.tight_layout(rect=(0, 0, 1, top))
