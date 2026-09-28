"""``cacc`` command-line interface (D-025).

    cacc evaluate PLAN.yaml [-o OUT] [-j JOBS]    test plan -> verdict + reports
    cacc sweep SCENARIO.yaml [-c CONTROLLER]      black-box string stability
    cacc run SCENARIO.yaml [-c CONTROLLER] [--seed N]   one run, all metrics
    cacc calibrate LOG.csv [-o OUT]               digital twin of a real ACC
    cacc audit LOG.csv [-o OUT]                   customer audit report (HTML)
    cacc init DIR                                  scaffold a controller project
    cacc metrics                                   list criterion metrics
    cacc gui                                       CACC Studio desktop app

Exit codes: 0 = pass / stable, 1 = fail / unstable, 2 = error (bad plan,
simulation crash, bad arguments).
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import sys
import traceback
from dataclasses import replace
from datetime import datetime
from pathlib import Path

import yaml

import cacc
from cacc.criteria import METRICS

EXIT = {"PASS": 0, "FAIL": 1, "ERROR": 2}


def _options(pairs: list[str]) -> dict:
    """``['T=1.2', 'name=x']`` -> ``{'T': 1.2, 'name': 'x'}`` (YAML-typed)."""
    out = {}
    for p in pairs:
        key, sep, val = p.partition("=")
        if not sep or not key:
            raise SystemExit(f"--option expects key=value, got {p!r}")
        out[key] = yaml.safe_load(val)
    return out


def _logging(verbose: bool) -> None:
    root = logging.getLogger("cacc")
    if not root.handlers:
        h = logging.StreamHandler()
        h.setFormatter(logging.Formatter("%(levelname)-7s %(name)s: %(message)s"))
        root.addHandler(h)
    root.setLevel(logging.INFO if verbose else logging.WARNING)
    if verbose:  # per-run start/done lines are noise even when verbose
        logging.getLogger("cacc.platoon").setLevel(logging.WARNING)


def cmd_evaluate(args) -> int:
    from cacc.evaluate import PlanError, evaluate, load_plan
    from cacc.reporting import write_all

    try:
        plan = load_plan(args.plan)
    except (PlanError, OSError, yaml.YAMLError) as exc:
        print(f"error: invalid test plan {args.plan}: {exc}", file=sys.stderr)
        return 2
    n_runs = len(plan.cases) * (len(plan.seeds) if plan.criteria else 0)
    print(f"cacc {cacc.__version__} · plan '{plan.name}': {len(plan.cases)} "
          f"case(s), {n_runs} run(s)"
          + (f" + {len(plan.cases)} sweep(s)" if plan.sweep else "")
          + f" · {args.jobs} worker(s)", file=sys.stderr)
    report = evaluate(plan, jobs=args.jobs)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    outdir = Path(args.out) if args.out else (
        Path("results") / "evaluations" / plan.name / stamp)
    paths = write_all(report, outdir)
    print(paths["markdown"].read_text(encoding="utf-8"), end="")
    print(f"\nreports: {outdir}/ ({', '.join(p.name for p in paths.values())}, "
          "cases/)", file=sys.stderr)
    return EXIT[report["verdict"]]


def cmd_sweep(args) -> int:
    from cacc.stringstab import SweepConfig, measure_string_stability

    sc = cacc.load_scenario(args.scenario)
    sweep = SweepConfig(n_followers=args.n_followers)
    res = measure_string_stability(sc.config, args.controller, sweep,
                                   _options(args.option))
    if args.json:
        print(json.dumps(res.to_dict(), indent=2))
    else:
        hdr = "  omega [rad/s] " + "".join(f"  hop {i - 1}->{i}"
                                           for i in res.hops)
        print(hdr)
        for k, w in enumerate(res.omega):
            print(f"  {w:13.4f} " + "".join(f"  {g:8.4f}"
                                            for g in res.gain[:, k]))
        verdict = "STRING-STABLE" if res.string_stable(args.max_gain) else (
            "STRING-UNSTABLE")
        print(f"\npeak |Gamma| = {res.peak_gain:.4f} at {res.peak_omega:.3f} "
              f"rad/s  ->  {verdict} (limit {args.max_gain:g})")
    return 0 if res.string_stable(args.max_gain) else 1


def cmd_run(args) -> int:
    sc = cacc.load_scenario(args.scenario)
    cfg = replace(sc.config, seed=args.seed)
    res = cacc.PlatoonSim(cfg, args.controller, sc.leader,
                          _options(args.option)).run()
    vals = {m: METRICS[m].fn(res) for m in METRICS}
    if args.json:
        print(json.dumps({k: (v if math.isfinite(v) else str(v))
                          for k, v in vals.items()}, indent=2))
    else:
        print(f"{sc.name} · controller {args.controller} · seed {args.seed}")
        for m, v in vals.items():
            print(f"  {m:22s} {v:12.4f} {METRICS[m].unit}")
    return 0


def format_twin(r) -> str:
    """Human-readable block for one calibrated follower."""
    m, d = r.model, r.to_dict()
    se = r.se
    lines = [f"{r.follower}  (behind {r.leader}) · {r.duration_s:.0f} s "
             f"ACC-engaged in {r.segments} segment(s) · mean speed "
             f"{r.v_mean:.1f} m/s",
             "  fitted    " + " · ".join(
                 f"{p} {getattr(m, p):.4g} ± {se[p]:.2g} {u}"
                 for p, u in (("k_s", "1/s²"), ("k_v", "1/s"), ("T", "s"),
                              ("s0", "m"), ("tau", "s"))),
             f"  fit       gap RMSE {r.gap_rmse:.3g} m · speed RMSE "
             f"{r.speed_rmse:.3g} m/s"]
    if r.min_time_gap is None:
        need = "is string-unstable at every time gap up to 10 s"
    else:
        need = (f"needs ≥ {r.min_time_gap:.2f} s (margin {r.margin:+.3f} ± "
                f"{r.margin_se:.2g} s)")
    lines.append(f"  verdict   {r.verdict.upper()} — runs at T = {m.T:.2f} s, "
                 f"{need}; ‖Γ‖∞ = {r.hinf:.4f} at {r.peak_omega:.3f} rad/s")
    emp = d["empirical"]
    if emp.get("peak_gain") is not None:
        lines.append(f"  data      model-free peak speed gain "
                     f"{emp['peak_gain']:.3f} at {emp['peak_omega_rad_s']:.3f} "
                     f"rad/s (coherence {emp['coherence']:.2f})")
    else:
        lines.append(f"  data      model-free estimate unavailable "
                     f"({emp.get('note', '')})")
    w = r.what_if
    lower = w["min_string_stable_time_gap_s"]
    lines.append(f"  what-if   V2V feedforward ka = {w['ka']:g} at "
                 f"{w['delay_s'] * 1000:.0f} ms → ‖Γ‖∞ = {w['hinf']:.4f}"
                 + (f", string-stable down to T = {lower:.2f} s"
                    if lower is not None else ", still unstable"))
    return "\n".join(lines)


def cmd_calibrate(args) -> int:
    from cacc.audit import write_twins
    from cacc.fielddata import read_log
    from cacc.twin import calibrate_log

    log_ = read_log(args.log)
    reports, skipped = calibrate_log(log_, args.min_duration,
                                     v2v_ka=args.v2v_ka,
                                     v2v_delay=args.v2v_delay)
    if args.out:
        out = Path(args.out)
        write_twins(reports, out)
        (out / "calibration.json").write_text(json.dumps(
            {"source": str(args.log), "followers": [r.to_dict() for r in reports],
             "skipped": skipped}, indent=2, default=float), encoding="utf-8")
        print(f"twin scenarios + calibration.json written to {out}/",
              file=sys.stderr)
    if args.json:
        print(json.dumps({"source": str(args.log),
                          "followers": [r.to_dict() for r in reports],
                          "skipped": skipped}, indent=2, default=float))
    else:
        print(f"cacc {cacc.__version__} · {args.log} · {log_.n_vehicles} "
              f"vehicles · {log_.dt:.3g} s sampling\n")
        for r in reports:
            print(format_twin(r) + "\n")
        for msg in skipped:
            print(f"skipped: {msg}")
    if not reports:
        return 2
    return 1 if any(r.verdict == "string-unstable" for r in reports) else 0


def cmd_audit(args) -> int:
    from cacc.audit import audit_log, write_audit

    res = audit_log(args.log, args.min_duration, v2v_ka=args.v2v_ka,
                    v2v_delay=args.v2v_delay)
    out = Path(args.out or f"audit_{Path(args.log).stem}")
    paths = write_audit(res, out)
    for v in res.vehicles:
        print(format_twin(v.twin) + "\n")
    for msg in res.skipped:
        print(f"skipped: {msg}")
    print(f"report: {paths['report']}  (+ calibration.json, twin_*.yaml, "
          f"gate_*.yaml in {out}/)")
    return res.exit_code


def cmd_init(args) -> int:
    from cacc.scaffold import scaffold

    try:
        files = scaffold(args.directory, args.name, args.force)
    except FileExistsError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    for f in files:
        print(f"created {f}")
    print(f"\nnext:  cd {args.directory} && cacc evaluate "
          "plans/release_gate.yaml -j 4")
    return 0


def cmd_metrics(args) -> int:
    for name, m in METRICS.items():
        better = ">=" if m.worse == "low" else "<="
        print(f"  {name:22s} {'[' + m.unit + ']':9s} {better}  {m.doc}")
    return 0


def cmd_gui(args) -> int:
    from cacc.gui import main as gui_main

    return gui_main([])


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="cacc", description="String-stability and V2X-robustness test "
        "engine for longitudinal vehicle controllers.")
    p.add_argument("--version", action="version",
                   version=f"cacc {cacc.__version__}")
    p.add_argument("-v", "--verbose", action="store_true")
    sub = p.add_subparsers(dest="cmd", required=True)

    e = sub.add_parser("evaluate", help="run a test plan, write reports")
    e.add_argument("plan", help="test-plan YAML")
    e.add_argument("-o", "--out", help="output directory (default "
                   "results/evaluations/<plan>/<stamp>)")
    e.add_argument("-j", "--jobs", type=int, default=1,
                   help="parallel worker processes")
    e.set_defaults(fn=cmd_evaluate)

    for name, fn, hlp in (("sweep", cmd_sweep, "black-box string-stability "
                           "frequency sweep"),
                          ("run", cmd_run, "one simulation, print all metrics")):
        s = sub.add_parser(name, help=hlp)
        s.add_argument("scenario", help="scenario YAML")
        s.add_argument("-c", "--controller", default="cthp",
                       help="acc | cacc | cthp | module:Name | file.py:Name")
        s.add_argument("--option", action="append", default=[],
                       metavar="KEY=VALUE", help="plugin constructor option")
        s.add_argument("--json", action="store_true")
        s.set_defaults(fn=fn)
        if name == "sweep":
            s.add_argument("--n-followers", type=int, default=3)
            s.add_argument("--max-gain", type=float, default=1.0)
        else:
            s.add_argument("--seed", type=int, default=1)

    c = sub.add_parser("calibrate", help="fit a digital twin of each ACC "
                       "follower in a drive log (OpenACC or generic CSV)")
    c.add_argument("log", help="OpenACC CSV or generic log CSV")
    c.add_argument("-o", "--out", help="write twin scenarios + calibration.json")
    c.add_argument("--min-duration", type=float, default=30.0,
                   help="shortest ACC-engaged segment to use [s]")
    c.add_argument("--v2v-ka", type=float, default=0.5,
                   help="what-if V2V feedforward gain")
    c.add_argument("--v2v-delay", type=float, default=0.1,
                   help="what-if V2V latency [s]")
    c.add_argument("--json", action="store_true")
    c.set_defaults(fn=cmd_calibrate)

    a = sub.add_parser("audit", help="drive log -> String-Stability Audit "
                       "report (HTML) + twins + release gates")
    a.add_argument("log", help="OpenACC CSV or generic platoon log")
    a.add_argument("-o", "--out", help="output directory "
                   "(default: audit_<log name>)")
    a.add_argument("--min-duration", type=float, default=30.0,
                   help="shortest ACC-engaged segment to use [s]")
    a.add_argument("--v2v-ka", type=float, default=0.5,
                   help="V2V feedforward gain drawn in the gain chart")
    a.add_argument("--v2v-delay", type=float, default=0.1,
                   help="V2V latency drawn in the gain chart [s]")
    a.set_defaults(fn=cmd_audit)

    i = sub.add_parser("init", help="scaffold a controller-evaluation project")
    i.add_argument("directory")
    i.add_argument("--name", help="project name (default: directory name)")
    i.add_argument("--force", action="store_true", help="overwrite files")
    i.set_defaults(fn=cmd_init)

    m = sub.add_parser("metrics", help="list the metrics criteria can use")
    m.set_defaults(fn=cmd_metrics)

    g = sub.add_parser("gui", help="open CACC Studio, the desktop app "
                       "(needs pip install -e \".[gui]\")")
    g.set_defaults(fn=cmd_gui)
    return p


def _utf8_streams() -> None:
    """Reports contain ✓ / ✕ / ‖Γ‖; a Windows pipe or redirect defaults to
    cp1252 and would crash printing them, so write UTF-8 there."""
    for stream in (sys.stdout, sys.stderr):
        enc = (getattr(stream, "encoding", None) or "").lower().replace("-", "")
        if enc != "utf8" and hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")


def main(argv: list[str] | None = None) -> int:
    _utf8_streams()
    args = build_parser().parse_args(argv)
    _logging(args.verbose)
    try:
        return args.fn(args)
    except Exception as exc:  # noqa: BLE001 — a crash is ERROR (2), never FAIL (1)
        if args.verbose:
            traceback.print_exc()
        # expected input problems read as-is; anything else names its type
        msg = (str(exc) if isinstance(exc, (ValueError, OSError))
               else f"{type(exc).__name__}: {exc}")
        print(f"error: {msg}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
