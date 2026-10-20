"""Standards checklist: map test runs to ACC / CACC / AEB requirements.

Each :data:`REQUIREMENTS` row names a standard, paraphrases what it asks
for, and says how this tool checks it: one or more campaign cells and a
rule on a run metric. :func:`check` runs every cell once per seed, applies
the rules and returns a checklist with a status per row:

* ``PASS`` / ``FAIL`` the measured value against the rule;
* ``REVIEW`` the rule is an ACC limit that a CACC deliberately goes past
  (shorter time gap with V2V); ISO 20035 governs that case;
* ``N/A`` the requirement does not apply (V2V rows for a radar-only ACC).

This is a **pre-assessment**, not certification: the requirement texts are
paraphrased from public summaries, the thresholds are the commonly cited
ones (ISO 15622: 0.8 s minimum time gap, 3.5 m/s^2 deceleration, 2 m/s^2
acceleration, 2.5 m/s^3 jerk averaged over 1 s), and scenario rows (UNECE
R157, Euro NCAP) are judged on "no contact", not on the full test
procedure. Check the purchased standard text before quoting a result.
"""

from __future__ import annotations

import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from cacc import crashlib


def avg_jerk_1s(res) -> float:
    """Largest |a(t + 1 s) - a(t)| / 1 s of any follower (ISO 15622 style)."""
    k = max(1, int(round(1.0 / float(res.t[1] - res.t[0]))))
    a = res.acc[:, 1:]
    return float(np.abs(a[k:] - a[:-k]).max()) if a.shape[0] > k else 0.0


#: id -> (standard, requirement, applies, cells, metric, op, threshold)
#: applies: "all", "v2v" (CACC-type controllers only) or "acc-limit"
REQUIREMENTS: list[tuple] = [
    ("ISO15622-gap", "ISO 15622 (ACC)",
     "Steady-state time gap of at least 0.8 s", "acc-limit",
     [("cruise", "nominal")], "min_time_gap", ">=", 0.8),
    ("ISO15622-decel", "ISO 15622 (ACC)",
     "Automatic braking in normal following at most 3.5 m/s²", "acc-limit",
     [("traffic-wave", "nominal"), ("stop-and-go", "nominal")],
     "peak_decel", "<=", 3.5),
    ("ISO15622-accel", "ISO 15622 (ACC)",
     "Automatic acceleration at most 2 m/s²", "acc-limit",
     [("traffic-wave", "nominal"), ("stop-and-go", "nominal")],
     "peak_accel", "<=", 2.0),
    ("ISO15622-jerk", "ISO 15622 (ACC)",
     "Jerk (1 s average) at most 2.5 m/s³", "acc-limit",
     [("traffic-wave", "nominal"), ("stop-and-go", "nominal")],
     "avg_jerk_1s", "<=", 2.5),
    ("ISO20035-string", "ISO 20035 (CACC)",
     "String-stable following: disturbances shrink down the platoon", "v2v",
     [("traffic-wave", "nominal")], "peak_amplification_max", "<=", 1.0),
    ("ISO20035-degrade", "ISO 20035 (CACC)",
     "Safe degradation when V2V is lost (no contact, ≥ 1 m)", "v2v",
     [("hard-brake", "v2v-blackout")], "min_gap", ">=", 1.0),
    ("ISO20035-latency", "ISO 20035 (CACC)",
     "Safe under a slow radio link (300 ms latency)", "v2v",
     [("hard-brake", "latency-300ms")], "min_gap", ">=", 1.0),
    ("R155-spoof", "UN R155 / ISO 21434 (cyber, proxy)",
     "A spoofed V2V message does not cause a collision", "v2v",
     [("hard-brake", "spoof")], "min_gap", ">=", 1.0),
    ("R157-lead-brake", "UNECE R157 (ALKS) scenario",
     "Lead vehicle brakes hard to a stop: no collision", "all",
     [("nhtsa-lvd", "nominal")], "min_gap", ">", 0.0),
    ("R157-cut-in", "UNECE R157 (ALKS) scenario",
     "A slower car cuts in: no collision", "all",
     [("nhtsa-lane-change", "nominal")], "min_gap", ">", 0.0),
    ("R157-stationary", "UNECE R157 (ALKS) scenario",
     "Stopped car revealed by a lane change: no collision", "all",
     [("nhtsa-lvs", "nominal")], "min_gap", ">", 0.0),
    *[(f"NCAP-{k[5:].upper()}", "Euro NCAP AEB Car-to-Car",
       crashlib.MANEUVERS[k][0].split(": ", 1)[1] + ": no collision", "all",
       [(k, "nominal")], "min_gap", ">", 0.0)
      for k in crashlib.MANEUVERS if k.startswith("ncap")],
    ("ISO26262-radar", "ISO 26262 (fault reaction, proxy)",
     "Frozen radar track during a hard brake: no collision", "all",
     [("hard-brake", "radar-frozen")], "min_gap", ">", 0.0),
    ("ISO26262-brakes", "ISO 26262 (fault reaction, proxy)",
     "Brake fade to 50 % in an emergency stop: no collision", "all",
     [("emergency-stop", "brake-fade")], "min_gap", ">", 0.0),
    ("ISO26262-drag", "ISO 26262 (fault reaction, proxy)",
     "Dragging brake during a hard brake: no collision", "all",
     [("hard-brake", "stuck-brake")], "min_gap", ">", 0.0),
]


def _metric(name: str):
    if name == "avg_jerk_1s":
        return avg_jerk_1s
    from cacc.criteria import METRICS

    return METRICS[name].fn


def _run_cell(args):
    maneuver, condition, controller, options, block, aeb, seed, metrics = args
    import yaml

    from cacc import campaign as cp

    base = yaml.safe_load(cp.BASE.read_text(encoding="utf-8"))
    if block:
        base.setdefault("controller", {}).update(block)
    tree = cp.compose(base, maneuver, condition, controller)
    if aeb:
        tree["aeb"] = {"enabled": True}
    res = cp.replay({"scenario": tree, "name": maneuver}, controller, seed, options)
    return {m: float(_metric(m)(res)) for m in metrics}


def _uses_v2v(controller: str, options: dict) -> bool:
    from cacc.controllers import ControllerParams, make_controller

    return bool(make_controller(controller, ControllerParams(), options or None).uses_v2v)


def check(controller: str = "cacc", options: dict | None = None,
          controller_block: dict | None = None, aeb: bool = False,
          seeds: int = 1, jobs: int = 1, progress=None) -> dict:
    """Run the checklist; returns ``{controller, aeb, rows, counts}``."""
    v2v = _uses_v2v(controller, options or {})
    need: dict[tuple, set] = {}
    for _id, _std, _txt, applies, cells, metric, _op, _thr in REQUIREMENTS:
        if applies == "v2v" and not v2v:
            continue
        for cell in cells:
            need.setdefault(cell, set()).add(metric)
    tasks = [(m, c, controller, options or {}, controller_block, aeb, s,
              tuple(sorted(mets)))
             for (m, c), mets in need.items() for s in range(1, seeds + 1)]
    out = {}
    if jobs > 1:
        with ProcessPoolExecutor(jobs) as pool:
            for i, (t, r) in enumerate(zip(tasks, pool.map(_run_cell, tasks), strict=True)):
                out[(t[0], t[1], t[6])] = r
                if progress:
                    progress(i + 1, len(tasks))
    else:
        for i, t in enumerate(tasks):
            out[(t[0], t[1], t[6])] = _run_cell(t)
            if progress:
                progress(i + 1, len(tasks))
    rows = []
    for rid, std, text, applies, cells, metric, op, thr in REQUIREMENTS:
        row = {"id": rid, "standard": std, "requirement": text,
               "rule": f"{metric} {op} {thr:g}", "cells": [f"{m} | {c}" for m, c in cells]}
        if applies == "v2v" and not v2v:
            rows.append({**row, "status": "N/A", "value": None,
                         "evidence": "radar-only controller: nothing travels over V2V"})
            continue
        vals = [(out[(m, c, s)][metric], f"{m} | {c}", s)
                for m, c in cells for s in range(1, seeds + 1)]
        low = op in (">=", ">")
        val, where, seed = (min if low else max)(vals, key=lambda x: x[0])
        ok = val >= thr if op == ">=" else val > thr if op == ">" else val <= thr
        status = "PASS" if ok else ("REVIEW" if applies == "acc-limit" and v2v else "FAIL")
        note = (" (CACC runs below the ACC limit by design; ISO 20035 applies)"
                if status == "REVIEW" else "")
        rows.append({**row, "status": status, "value": val,
                     "evidence": f"{where}, seed {seed}: {metric} = {val:.3g}{note}"})
    counts = {k: sum(r["status"] == k for r in rows)
              for k in ("PASS", "FAIL", "REVIEW", "N/A")}
    return {"controller": controller, "aeb": aeb, "seeds": seeds, "rows": rows,
            "counts": counts}


def format_text(result: dict) -> str:
    mark = {"PASS": "✓", "FAIL": "✕", "REVIEW": "?", "N/A": "–"}
    lines = [f"Standards pre-assessment: {result['controller']}"
             + (" + AEB" if result["aeb"] else "")]
    for r in result["rows"]:
        lines.append(f"  {mark[r['status']]} {r['status']:<6} {r['id']:<17} "
                     f"{r['requirement']}")
        lines.append(f"{'':28}{r['evidence']}")
    c = result["counts"]
    lines.append(f"{c['PASS']} pass · {c['FAIL']} fail · {c['REVIEW']} review · "
                 f"{c['N/A']} n/a  (paraphrased requirements: a pre-assessment, "
                 "not a certification)")
    return "\n".join(lines)


def markdown(result: dict) -> str:
    icon = {"PASS": "✅", "FAIL": "❌", "REVIEW": "🟡", "N/A": "➖"}
    out = [f"# Standards pre-assessment: `{result['controller']}`"
           + (" + AEB" if result["aeb"] else ""), "",
           "Requirements are paraphrased; scenario rows are judged on no contact. "
           "This is a pre-assessment, not a certification.", "",
           "| | ID | Standard | Requirement | Rule | Evidence |", "|---|---|---|---|---|---|"]
    for r in result["rows"]:
        out.append(f"| {icon[r['status']]} | {r['id']} | {r['standard']} | "
                   f"{r['requirement']} | `{r['rule']}` | {r['evidence']} |")
    return "\n".join(out) + "\n"


def write(result: dict, folder: str | Path) -> Path:
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "standards.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    md = folder / "standards.md"
    md.write_text(markdown(result), encoding="utf-8")
    return md
