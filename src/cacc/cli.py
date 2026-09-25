"""``cacc`` command-line interface (D-025).

    cacc evaluate PLAN.yaml [-o OUT] [-j JOBS]    test plan -> verdict + reports
    cacc sweep SCENARIO.yaml [-c CONTROLLER]      black-box string stability
    cacc run SCENARIO.yaml [-c CONTROLLER] [--seed N]   one run, all metrics
    cacc metrics                                   list criterion metrics

Exit codes: 0 = pass / stable, 1 = fail / unstable, 2 = error (bad plan,
simulation crash, bad arguments).
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import sys
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


def cmd_metrics(args) -> int:
    for name, m in METRICS.items():
        better = ">=" if m.worse == "low" else "<="
        print(f"  {name:22s} {'[' + m.unit + ']':9s} {better}  {m.doc}")
    return 0


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

    m = sub.add_parser("metrics", help="list the metrics criteria can use")
    m.set_defaults(fn=cmd_metrics)
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    _logging(args.verbose)
    try:
        return args.fn(args)
    except (ValueError, TypeError, FileNotFoundError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
