"""Build the Control Systems presentation.

    python scripts/build_ppt.py [output.pptx]

Needs python-pptx (present in the base conda env). Figures are pulled from
matlab/results/ and results/<study>/<newest timestamp>/ — a missing figure
degrades to a grey placeholder rather than failing the build, so the deck can
be produced before every study has been re-run.
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))

from ppt import content_core, content_evidence, content_theory  # noqa: E402
from ppt.theme import fit_report, new_deck  # noqa: E402


def _figure_resolver(root: Path):
    """Return fig(relpath) -> absolute Path, newest-timestamp aware."""
    missing: list[str] = []

    def fig(rel: str) -> Path:
        p = root / rel
        if p.exists():
            return p
        # results/<study>/<stamp>/<name>.png -- fall back to the newest stamp
        parts = Path(rel).parts
        if len(parts) >= 4 and parts[0] == "results":
            study_dir = root / parts[0] / parts[1]
            if study_dir.is_dir():
                stamps = sorted(d for d in study_dir.iterdir() if d.is_dir())
                for d in reversed(stamps):
                    cand = d / parts[-1]
                    if cand.exists():
                        return cand
        missing.append(rel)
        return p

    fig.missing = missing        # type: ignore[attr-defined]
    return fig


def main() -> int:
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else \
        ROOT / "report" / "CACC_Control_Systems_Presentation.pptx"
    out.parent.mkdir(parents=True, exist_ok=True)

    fig = _figure_resolver(ROOT)
    prs = new_deck()

    content_core.build(prs, fig)
    content_theory.build(prs, fig)
    content_evidence.build(prs, fig)

    prs.save(str(out))

    n = len(prs.slides.__iter__.__self__._sldIdLst)  # noqa: SLF001
    print(f"wrote {out}")
    print(f"slides: {n}")
    if fig.missing:                                  # type: ignore[attr-defined]
        print("missing figures (placeholders used):")
        for m in fig.missing:                        # type: ignore[attr-defined]
            print("   ", m)
    from ppt import eqmath
    if eqmath.BAD:
        print(f"UNPARSEABLE MATH ({len(eqmath.BAD)}) -- rendered literally in red:")
        for tex, why in eqmath.BAD:
            print(f"    {tex[:70]}\n        -> {why}")
    else:
        print("math: every equation typeset")

    over = fit_report()
    if over:
        print(f"OVERFULL SLIDES ({len(over)}) -- autofit hit its font floor:")
        for kind, title, size, need, avail in over:
            print(f"    [{kind:7s}] {size:>5.2f}pt  needs {need:>5.2f}in of "
                  f"{avail:>5.2f}in  |  {title}")
    else:
        print("fit: every slide fits above its font floor")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
