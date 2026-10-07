"""Render LaTeX equations to tightly-cropped transparent PNGs for the deck.

Why this exists: the deck used to typeset maths as ASCII art in a monospace
block -- `H(s) = (k_a s^2 + k_v s + k_p) / (tau s^3 + ...)`. Inline slashes
instead of fraction bars, and `_{w,lb}` style subscripts that only sometimes
survived. It was unreadable, which is fatal for the one part of the deck a
reviewer is guaranteed to stop on.

matplotlib's mathtext is used rather than a real LaTeX install: it needs no
external toolchain, ships in the same conda env as everything else, and covers
the subset this project writes (fractions, sub/superscripts, Greek, tildes,
norms, sums). Rendered PNGs are cached by content hash so a rebuild that does
not change an equation does not re-rasterise it.

Everything is rendered at RENDER_PT and scaled at insert time, so the cache is
independent of the size a slide happens to want.
"""

from __future__ import annotations

import hashlib
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt          # noqa: E402

_CACHE = Path(__file__).resolve().parents[2] / "results" / "eq_cache"

# Rasterise big, insert small: keeps the glyphs crisp on a projector.
RENDER_PT = 40
DPI = 220
INK_HEX = "#0F253F"                       # theme.INK

# Math strings mathtext could not parse, collected for the build report.
BAD: list[tuple[str, str]] = []


def _key(tex: str, color: str) -> str:
    return hashlib.sha1(f"{RENDER_PT}|{DPI}|{color}|{tex}".encode()).hexdigest()[:16]


def render(tex: str, *, color: str = INK_HEX) -> tuple[Path, float, float]:
    """Render one math string; return (png_path, width_in, height_in).

    `tex` is mathtext WITHOUT the surrounding $ -- e.g.
    r"\\tilde H(s) = \\frac{\\tilde k_a s^2 + k_v s + k_p}{\\tau s^3 + s^2 + \\gamma s + k_p}"

    The returned inches are the natural size at RENDER_PT; callers scale to
    whatever height they need and keep the aspect ratio.
    """
    _CACHE.mkdir(parents=True, exist_ok=True)
    out = _CACHE / f"eq_{_key(tex, color)}.png"

    if not out.exists():
        fig = plt.figure(figsize=(0.01, 0.01))
        fig.patch.set_alpha(0.0)
        fig.text(0, 0, f"${tex}$", fontsize=RENDER_PT, color=color)
        try:
            fig.savefig(out, dpi=DPI, transparent=True,
                        bbox_inches="tight", pad_inches=0.02)
        except ValueError as exc:
            # mathtext covers a SUBSET of LaTeX -- \underbrace, \tfrac and
            # friends are not in it. Fall back to a literal render so the
            # build still produces a deck, and report it loudly at the end
            # rather than dying on page 6 of 23.
            plt.close(fig)
            BAD.append((tex, str(exc).strip().splitlines()[-1][:90]))
            fig = plt.figure(figsize=(0.01, 0.01))
            fig.patch.set_alpha(0.0)
            fig.text(0, 0, tex, fontsize=RENDER_PT * 0.55, color="#A82B2B")
            fig.savefig(out, dpi=DPI, transparent=True,
                        bbox_inches="tight", pad_inches=0.02)
        plt.close(fig)

    from PIL import Image
    with Image.open(out) as im:
        w_px, h_px = im.size
    return out, w_px / DPI, h_px / DPI


def measure(tex: str) -> tuple[float, float]:
    """(width_in, height_in) at RENDER_PT, for layout passes."""
    _, w, h = render(tex)
    return w, h
