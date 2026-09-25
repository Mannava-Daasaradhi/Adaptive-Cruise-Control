"""Evaluation report writers (D-025): JSON, JUnit XML, Markdown, scenarios.

* ``report.json``  — complete machine-readable evidence (per-seed values,
  worst seed, sweep gain curves, tool version, git commit, timestamps).
* ``junit.xml``    — one testcase per (case, criterion) so any CI system
  (GitHub Actions, GitLab, Jenkins) shows controller regressions as tests.
* ``summary.md``   — human-readable verdict table; append it to
  ``$GITHUB_STEP_SUMMARY`` for an in-PR summary.
* ``cases/*.yaml`` — the fully resolved scenario of every case, so a failure
  is reproducible from (scenario file, seed) alone.
"""

from __future__ import annotations

import json
import math
import re
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from xml.etree import ElementTree as ET

import yaml

import cacc


def provenance() -> dict:
    """Tool version, UTC timestamp and (best effort) git commit."""
    info = {"tool": "cacc", "version": cacc.__version__,
            "created_utc": datetime.now(timezone.utc).isoformat(
                timespec="seconds")}
    try:
        out = subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True,
                             text=True, timeout=5, check=True)
        info["git_commit"] = out.stdout.strip()
        dirty = subprocess.run(["git", "status", "--porcelain"],
                               capture_output=True, text=True, timeout=5)
        info["git_dirty"] = bool(dirty.stdout.strip())
    except (OSError, subprocess.SubprocessError):
        pass
    return info


def _finite(x):
    """JSON-safe copy: non-finite floats become strings ('inf', 'nan')."""
    if isinstance(x, float) and not math.isfinite(x):
        return str(x)
    if isinstance(x, dict):
        return {k: _finite(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_finite(v) for v in x]
    return x


def _fmt(x, unit: str = "") -> str:
    if x is None:
        return "—"
    if isinstance(x, float) and math.isinf(x):
        return "never" if x > 0 else "-inf"
    if isinstance(x, float) and math.isnan(x):
        return "nan"
    s = f"{x:.4g}" if isinstance(x, float) else str(x)
    return f"{s} {unit}" if unit and unit != "-" else s


def _gain(ss: dict) -> str:
    """Peak gain with its standard error when the sweep measured one."""
    se = ss.get("peak_gain_se")
    return f"{ss['peak_gain']:.4f}" + (f" ± {se:.4f}" if se else "")


def slug(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9._=-]+", "_", name).strip("_") or "case"


def write_json(report: dict, path: Path) -> None:
    path.write_text(json.dumps(_finite(report), indent=2, allow_nan=False),
                    encoding="utf-8")


def write_junit(report: dict, path: Path) -> None:
    root = ET.Element("testsuites", name=report["plan"])
    tot = {"tests": 0, "failures": 0, "errors": 0}
    for case in report["cases"]:
        suite = ET.SubElement(root, "testsuite", name=case["name"])
        n = {"tests": 0, "failures": 0, "errors": 0}
        cls = f"{report['plan']}.{case['name']}"
        for err in case["errors"]:
            tc = ET.SubElement(suite, "testcase", classname=cls,
                               name=f"run (seed {err['seed']})")
            ET.SubElement(tc, "error", message="simulation raised").text = err["error"]
            n["tests"] += 1
            n["errors"] += 1
        for chk in case["checks"]:
            tc = ET.SubElement(suite, "testcase", classname=cls,
                               name=chk["label"])
            n["tests"] += 1
            if not chk["passed"]:
                n["failures"] += 1
                msg = (f"worst {_fmt(chk['worst'], chk['unit'])} at seed "
                       f"{chk['worst_seed']}; required {chk['op']} "
                       f"{_fmt(chk['threshold'], chk['unit'])}")
                ET.SubElement(tc, "failure", message=msg).text = msg
        ss = case["string_stability"]
        if ss is not None:
            tc = ET.SubElement(suite, "testcase", classname=cls,
                               name=f"string stability (peak |Gamma| <= "
                                    f"{ss['max_gain']:g})")
            n["tests"] += 1
            if not ss["passed"]:
                n["failures"] += 1
                msg = (f"peak |Gamma| = {_gain(ss)} at "
                       f"{ss['peak_omega_rad_s']:.3f} rad/s"
                       + (" (marginal: within 2 standard errors of the "
                          "limit)" if ss["marginal"] else ""))
                ET.SubElement(tc, "failure", message=msg).text = msg
        for k, v in n.items():
            suite.set(k, str(v))
            tot[k] += v
    for k, v in tot.items():
        root.set(k, str(v))
    root.set("time", str(report["elapsed_s"]))
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def render_markdown(report: dict) -> str:
    icon = {"PASS": "✅", "FAIL": "❌", "ERROR": "⚠️"}
    sut = report["under_test"]
    prov = report.get("provenance", {})
    lines = [f"## {icon[report['verdict']]} `{report['plan']}` — "
             f"**{report['verdict']}**", ""]
    if report["description"]:
        lines += [report["description"], ""]
    c = report["counts"]
    lines += [f"- **Controller under test:** `{sut['controller']}`"
              + (f" with options `{sut['options']}`" if sut["options"] else ""),
              f"- **Cases:** {len(report['cases'])} ({c['PASS']} pass, "
              f"{c['FAIL']} fail, {c['ERROR']} error) · **seeds:** "
              f"{report['seeds']} · **wall time:** {report['elapsed_s']} s",
              f"- **Tool:** cacc {prov.get('version', '?')}"
              + (f" @ `{prov['git_commit'][:10]}`"
                 + (" (dirty)" if prov.get("git_dirty") else "")
                 if prov.get("git_commit") else ""), "",
              "| case | check | required | worst | seed | result |",
              "|---|---|---|---|---|---|"]
    for case in report["cases"]:
        for chk in case["checks"]:
            lines.append(
                f"| {case['name']} | `{chk['metric']}` | {chk['op']} "
                f"{_fmt(chk['threshold'], chk['unit'])} | "
                f"{_fmt(chk['worst'], chk['unit'])} | {chk['worst_seed']} | "
                f"{'pass' if chk['passed'] else '**FAIL**'} |")
        ss = case["string_stability"]
        if ss is not None:
            verdict = "pass" if ss["passed"] else "**FAIL**"
            if ss["marginal"]:
                verdict += " (marginal)"
            lines.append(
                f"| {case['name']} | string stability (sweep) | peak \\|Γ\\| ≤ "
                f"{ss['max_gain']:g} | {_gain(ss)} @ "
                f"{ss['peak_omega_rad_s']:.3f} rad/s | — | {verdict} |")
        for err in case["errors"]:
            first = err["error"].strip().splitlines()[-1]
            lines.append(f"| {case['name']} | run error | — | `{first}` | "
                         f"{err['seed']} | **ERROR** |")
    lines += ["", "Reproduce a failing row: the case's resolved scenario is "
              "in `cases/<case>.yaml`; rerun it with the listed seed "
              "(`cacc run cases/<case>.yaml -c <controller> --seed <seed>`)."]
    if any((c["string_stability"] or {}).get("marginal")
           for c in report["cases"]):
        lines += ["", "*marginal*: the sweep's peak gain is within two "
                  "standard errors of the limit under the stochastic channel; "
                  "the verdict is not resolved by this measurement (use the "
                  "analytic robust certificate for built-in controllers)."]
    return "\n".join(lines) + "\n"


def write_all(report: dict, outdir: Path) -> dict[str, Path]:
    """Write every artifact into ``outdir``; return their paths."""
    outdir.mkdir(parents=True, exist_ok=True)
    cases_dir = outdir / "cases"
    cases_dir.mkdir(exist_ok=True)
    cases, used = [], set()
    for case in report["cases"]:
        name = slug(case["name"])
        stem, i = name, 2
        while name in used:  # distinct case names may share a slug
            name, i = f"{stem}-{i}", i + 1
        used.add(name)
        (cases_dir / f"{name}.yaml").write_text(
            yaml.safe_dump(case["scenario"], sort_keys=False), encoding="utf-8")
        cases.append(dict(case, scenario_file=f"cases/{name}.yaml"))
    report = dict(report, cases=cases, provenance=provenance())
    paths = {"json": outdir / "report.json", "junit": outdir / "junit.xml",
             "markdown": outdir / "summary.md"}
    write_json(report, paths["json"])
    write_junit(report, paths["junit"])
    paths["markdown"].write_text(render_markdown(report), encoding="utf-8")
    return paths
