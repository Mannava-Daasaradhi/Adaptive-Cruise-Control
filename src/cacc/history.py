"""Version history of stress results: CI for safety.

Every saved campaign report becomes a *version* (a label such as ``v1.2``
or a commit hash) in a history folder, one small JSON per version holding
the grid. :func:`diff` compares two versions cell by cell:

* **regression** a cell that passed now fails (blocks a release);
* **fixed** a cell that failed now passes;
* **worse** / **better** still the same verdict, but the tightest margin
  moved by more than ``tol`` (default 5 % of the threshold);
* **new** / **gone** a cell in only one of the two.

``cacc history save results/.../report.json --label v2`` stores a run,
``cacc history diff`` compares the last two (exit code 1 on a regression,
so it gates CI), ``cacc history show`` prints every cell's verdict over all
versions.
"""

from __future__ import annotations

import json
import math
import re
import subprocess
import time
from pathlib import Path

DEFAULT_STORE = Path(".cacc-history")
TOL = 0.05


def _git_commit() -> str | None:
    try:
        out = subprocess.run(["git", "rev-parse", "--short", "HEAD"],
                             capture_output=True, text=True, timeout=5)
        return out.stdout.strip() or None
    except (OSError, subprocess.SubprocessError):
        return None


def snapshot(report: dict, label: str) -> dict:
    """The part of a campaign report a version keeps."""
    camp = report.get("campaign")
    if not camp:
        raise ValueError("not a stress-campaign report (no campaign grid)")
    cells = {}
    for row in camp["grid"]:
        for cell in row:
            if cell:
                m = cell["margin"]
                cells[cell["name"]] = {
                    "verdict": cell["verdict"], "binding": cell["binding"],
                    "worst": cell["worst"],
                    "margin": m if math.isfinite(m) else (1e9 if m > 0 else -1e9)}
    return {"label": label, "saved": time.strftime("%Y-%m-%d %H:%M:%S"),
            "commit": _git_commit(),
            "controller": report.get("under_test", {}).get("controller", ""),
            "aeb": bool(camp.get("aeb")), "counts": report.get("counts", {}),
            "seeds": report.get("seeds", []), "cells": cells}


def save(report: dict, label: str, store: str | Path = DEFAULT_STORE) -> Path:
    store = Path(store)
    store.mkdir(parents=True, exist_ok=True)
    safe = re.sub(r"[^A-Za-z0-9._-]+", "_", label) or "version"
    n = len(list(store.glob("*.json"))) + 1
    path = store / f"{n:04d}_{safe}.json"
    path.write_text(json.dumps(snapshot(report, label), indent=1), encoding="utf-8")
    return path


def versions(store: str | Path = DEFAULT_STORE) -> list[dict]:
    """Saved versions, oldest first (each with its ``path``)."""
    out = []
    for p in sorted(Path(store).glob("*.json")):
        v = json.loads(p.read_text(encoding="utf-8"))
        v["path"] = str(p)
        out.append(v)
    return out


def find(store, which: str | None, default: int) -> dict:
    vs = versions(store)
    if not vs:
        raise ValueError(f"no saved versions in {store}")
    if which is None:
        return vs[default] if len(vs) >= abs(default) else vs[0]
    for v in vs:
        if v["label"] == which or Path(v["path"]).stem == which:
            return v
    raise ValueError(f"no version {which!r}; have {', '.join(v['label'] for v in vs)}")


def diff(old: dict, new: dict, tol: float = TOL) -> list[dict]:
    """Cell-by-cell changes, regressions first."""
    rows = []
    for name in sorted(set(old["cells"]) | set(new["cells"])):
        a, b = old["cells"].get(name), new["cells"].get(name)
        if a is None or b is None:
            rows.append({"cell": name, "change": "new" if a is None else "gone",
                         "old": a, "new": b, "delta": None})
            continue
        d = b["margin"] - a["margin"]
        if a["verdict"] == "PASS" and b["verdict"] != "PASS":
            change = "regression"
        elif a["verdict"] != "PASS" and b["verdict"] == "PASS":
            change = "fixed"
        elif abs(d) > tol and abs(d) < 1e8:
            change = "worse" if d < 0 else "better"
        else:
            change = "same"
        rows.append({"cell": name, "change": change, "old": a, "new": b, "delta": d})
    order = ["regression", "worse", "gone", "fixed", "better", "new", "same"]
    return sorted(rows, key=lambda r: (order.index(r["change"]), r["cell"]))


def format_diff(old: dict, new: dict, rows: list[dict]) -> str:
    n = {k: sum(r["change"] == k for r in rows)
         for k in ("regression", "worse", "fixed", "better", "new", "gone", "same")}
    lines = [f"{old['label']} → {new['label']}: {n['regression']} regression(s), "
             f"{n['worse']} worse, {n['fixed']} fixed, {n['better']} better, "
             f"{n['same']} unchanged" + (f", {n['new']} new" if n["new"] else "")
             + (f", {n['gone']} gone" if n["gone"] else "")]
    mark = {"regression": "▼▼", "worse": "▼ ", "fixed": "▲▲", "better": "▲ ",
            "new": "+ ", "gone": "- "}
    for r in rows:
        if r["change"] == "same":
            continue
        o, b = r["old"], r["new"]
        what = (f"{o['verdict']} → {b['verdict']}" if o and b else
                (b or o)["verdict"])
        if r["delta"] is not None and abs(r["delta"]) < 1e8:
            what += f", margin {o['margin']:+.0%} → {b['margin']:+.0%}"
        lines.append(f"  {mark[r['change']]} {r['change']:<10} {r['cell']:<34} {what}")
    return "\n".join(lines)


def table(store: str | Path = DEFAULT_STORE) -> tuple[list[dict], list[str], list[list[str]]]:
    """(versions, cell names, verdict matrix [cell][version])."""
    vs = versions(store)
    names = sorted({c for v in vs for c in v["cells"]})
    grid = [[v["cells"].get(c, {}).get("verdict", "") for v in vs] for c in names]
    return vs, names, grid


def format_table(store: str | Path = DEFAULT_STORE) -> str:
    vs, names, grid = table(store)
    if not vs:
        return f"no saved versions in {store}"
    mark = {"PASS": "✓", "FAIL": "✕", "ERROR": "!", "": "·"}
    w = max(8, *(len(v["label"]) for v in vs))
    lines = ["cell".ljust(34) + "".join(v["label"][:w].rjust(w + 1) for v in vs)]
    for name, row in zip(names, grid, strict=True):
        lines.append(name[:34].ljust(34) + "".join(mark.get(x, "?").rjust(w + 1)
                                                    for x in row))
    lines.append("pass".ljust(34) + "".join(
        f"{v['counts'].get('PASS', 0)}/{len(v['cells'])}".rjust(w + 1) for v in vs))
    return "\n".join(lines)
