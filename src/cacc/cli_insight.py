"""CLI commands that dig into a controller: breaking points, root causes,
auto-tuning and the field study of real cars.

    cacc envelope -c CTRL [--maneuver M] [--axis A ...]   breaking points
    cacc envelope -c CTRL --map latency grip [-n 7]        2-axis envelope
    cacc explain REPORT.json "hard-brake | wet"            why a cell failed
    cacc tune DESIGN.yaml | --preset P  -o tuned.yaml      auto-tune a design
    cacc field-study FOLDER | --fetch microsim              real ACC cars
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path


def _jobs(n: int | None) -> int:
    return n or max(1, min(8, (os.cpu_count() or 2) - 1))


def cmd_envelope(args) -> int:
    from cacc import envelope as ev
    from cacc.cli_tools import _opts
    from cacc.tfctrl import resolve

    ref, options, block = resolve(args.controller, _opts(args.option))
    crit = args.criterion or None
    kw = dict(options=options, controller_block=block, seeds=args.seeds,
              jobs=_jobs(args.jobs))
    if crit:
        kw["criteria"] = crit
    if args.map:
        env = ev.envelope(ref, args.maneuver, args.map[0], args.map[1], args.n, **kw)
        xs = [f"{x:.3g}" for x in env["x"]]
        print(f"{env['y_label']} \\ {env['x_label']}")
        print(" " * 9 + "".join(x.rjust(7) for x in xs))
        for y, row in zip(env["y"], env["verdict"], strict=True):
            print(f"{y:9.3g}" + "".join(("✓" if v == "PASS" else "✕").rjust(7)
                                        for v in row))
        print(f"{env['pass_share']:.0%} of the map passes")
        return 0
    rows = ev.profile(ref, args.maneuver, axes=args.axis or None, **kw)
    print(f"breaking points · {args.controller} · {args.maneuver}")
    print(ev.format_profile(rows))
    if args.out:
        Path(args.out).write_text(json.dumps(rows, indent=2, default=float),
                                  encoding="utf-8")
    return 0


def cmd_explain(args) -> int:
    from cacc.explain import explain, format_text

    report = json.loads(Path(args.report).read_text(encoding="utf-8"))
    names = [c["name"] for c in report["cases"]]
    targets = ([args.case] if args.case else
               [c["name"] for c in report["cases"] if c["verdict"] != "PASS"])
    if args.case and args.case not in names:
        raise ValueError(f"no case {args.case!r}; cases: {', '.join(names)}")
    if not targets:
        print("every case passes: nothing to explain")
        return 0
    for name in targets:
        print(format_text(explain(report, name, verify=not args.no_fixes,
                                  jobs=_jobs(args.jobs))))
        print()
    return 0


def cmd_tune(args) -> int:
    from cacc import tfctrl as tc
    from cacc.autotune import tune

    if bool(args.design) == bool(args.preset):
        raise ValueError("give a design YAML or --preset NAME")
    d = tc.Design.load(args.design) if args.design else tc.Design.preset(args.preset)
    res = tune(d, tau=args.tau, act_delay=args.act_delay,
               thetas=tuple(args.theta), samples=args.samples,
               jobs=_jobs(args.jobs))
    print(res.summary())
    for line in res.log:
        print("  " + line)
    for sig in tc.SIGNALS:
        print(f"  K_{sig}: {d.blocks.get(sig, '0')}  →  {res.design.blocks.get(sig, '0')}")
    if args.out:
        res.design.save(args.out)
        print(f"wrote {args.out}")
    return 0 if res.passed else 1


def cmd_field_study(args) -> int:
    from cacc import fieldstudy as fs

    folder = args.folder
    source = args.source
    if args.fetch:
        folder = folder or str(Path("results") / "data" / "microsim_acc")
        print(f"downloading microSIM-ACC runs into {folder} …", file=sys.stderr)
        fs.fetch_microsim(folder)
        source = source or "microSIM-ACC ICE (FAU): 2-car commercial ACC runs"
    if not folder:
        raise ValueError("give a folder of logs, or --fetch microsim")
    rows, problems = fs.study(folder, jobs=_jobs(args.jobs), twin=not args.no_twin)
    if not rows:
        raise ValueError(f"no usable runs under {folder}")
    agg = fs.aggregate(rows)
    out = Path(args.out or Path("results") / "field_study")
    md = fs.write(rows, agg, problems, out, source or str(folder))
    print(md.read_text(encoding="utf-8"))
    print(fs.headline(agg))
    return 0


def register(sub) -> None:
    from cacc.envelope import AXES

    e = sub.add_parser("envelope", help="breaking point of every stress axis, "
                       "or a 2-axis safety envelope")
    e.add_argument("-c", "--controller", default="cthp",
                   help="acc | cacc | cthp | module:Name | design .yaml")
    e.add_argument("--maneuver", default="hard-brake")
    e.add_argument("--axis", action="append", choices=list(AXES),
                   help="only these axes (repeatable)")
    e.add_argument("--map", nargs=2, metavar=("X", "Y"), choices=list(AXES),
                   help="map two axes against each other instead")
    e.add_argument("-n", type=int, default=7, help="map resolution")
    e.add_argument("--criterion", action="append",
                   help="pass criterion (repeatable; default: the stress set)")
    e.add_argument("--option", action="append", default=[], metavar="KEY=VALUE")
    e.add_argument("--seeds", type=int, default=1)
    e.add_argument("-j", "--jobs", type=int)
    e.add_argument("-o", "--out", help="write the breaking points as JSON")
    e.set_defaults(fn=cmd_envelope)

    x = sub.add_parser("explain", help="root cause + verified fixes for the "
                       "failed cells of a stress / plan report")
    x.add_argument("report", help="report.json from cacc stress")
    x.add_argument("case", nargs="?", help='cell name, e.g. "hard-brake | wet" '
                   "(default: every failed cell)")
    x.add_argument("--no-fixes", action="store_true",
                   help="skip re-simulating candidate fixes")
    x.add_argument("-j", "--jobs", type=int)
    x.set_defaults(fn=cmd_explain)

    t = sub.add_parser("tune", help="auto-tune a Controller Lab design")
    t.add_argument("design", nargs="?", help="design YAML")
    t.add_argument("--preset", help="start from cacc | cthp | acc | lag-ff")
    t.add_argument("--tau", type=float, default=0.2)
    t.add_argument("--act-delay", type=float, default=0.0)
    t.add_argument("--theta", type=float, nargs="+", default=[0.05, 0.15],
                   help="V2V delays the design must stay string-stable at")
    t.add_argument("--samples", type=int, default=256)
    t.add_argument("-j", "--jobs", type=int)
    t.add_argument("-o", "--out", help="write the tuned design here")
    t.set_defaults(fn=cmd_tune)

    f = sub.add_parser("field-study", help="how real production ACC cars "
                       "behave: amplification, undershoot, gaps")
    f.add_argument("folder", nargs="?", help="folder of drive logs (CSV)")
    f.add_argument("--fetch", choices=("microsim",),
                   help="download a public dataset first")
    f.add_argument("--source", help="dataset name for the summary")
    f.add_argument("--no-twin", action="store_true", help="skip twin fits")
    f.add_argument("-j", "--jobs", type=int)
    f.add_argument("-o", "--out", help="output folder")
    f.set_defaults(fn=cmd_field_study)
