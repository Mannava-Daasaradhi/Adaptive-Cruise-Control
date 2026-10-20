"""CLI commands around the engine: CI, Controller Lab, adapters.

Registered on the main parser by :func:`register`:

    cacc ci REPORT [--badge --summary --comment]   CI comment, summary, badge
    cacc lab DESIGN.yaml [--matlab F]    analyse a Controller Lab design
    cacc lab --preset cacc -o my.yaml    start a design from a built-in law
    cacc ros2-params SCENARIO -c CTRL    ROS 2 params file for ros2_ws
    cacc carla SCENARIO -c CTRL          run the platoon in a CARLA server
"""

from __future__ import annotations

import sys
from pathlib import Path

import yaml


def _opts(pairs) -> dict:
    from cacc.cli import _options

    return _options(pairs or [])


def cmd_ci(args) -> int:
    from cacc import ci

    for line in ci.run(args.report, args.badge, args.summary, args.comment, args.title):
        print(line, file=sys.stderr)
    import json

    verdict = json.loads(Path(args.report).read_text(encoding="utf-8"))["verdict"]
    return {"PASS": 0, "FAIL": 1}.get(verdict, 2) if args.gate else 0


def cmd_lab(args) -> int:
    from cacc import tfctrl as tc

    if args.preset:
        d = tc.Design.preset(args.preset)
        if args.out:
            d.save(args.out)
            print(f"wrote {args.out} — edit the blocks, then: cacc lab {args.out}")
            return 0
    elif args.design:
        d = tc.Design.load(args.design)
    else:
        raise ValueError("give a design file or --preset " + "|".join(tc.PRESETS))
    _, _, rep = tc.analyse(d, args.tau, args.delay, args.act_delay)
    print(f"{d.name}: V2V payload '{d.ff_signal}', h = {d.h:g} s, r = {d.r:g} m, "
          f"{rep.n_states} controller state(s)")
    for sig in tc.SIGNALS:
        print(f"  {tc.SIGNAL_LABELS[sig]:22s} {d.blocks.get(sig, '0')}")
    poles = ", ".join(f"{p.real:.3g}{p.imag:+.3g}j" if abs(p.imag) > 1e-9
                      else f"{p.real:.3g}" for p in rep.poles)
    print(f"closed-loop poles: {poles}\n"
          f"  -> {'internally stable' if rep.internally_stable else 'UNSTABLE'}")
    print(f"peak |Γ(jω)| = {rep.peak_gain:.4f} at {rep.peak_omega:.3g} rad/s "
          f"(τ = {args.tau:g} s, V2V delay {args.delay:g} s) -> "
          f"{'string-stable' if rep.string_stable else 'STRING-UNSTABLE'}")
    if args.matlab:
        Path(args.matlab).write_text(tc.to_matlab(d, args.tau, args.delay),
                                     encoding="utf-8")
        print(f"MATLAB script: {args.matlab}")
    return 0 if rep.internally_stable and rep.string_stable else 1


def cmd_ros2_params(args) -> int:
    from cacc.adapters import ros2
    from cacc.tfctrl import resolve

    tree = yaml.safe_load(Path(args.scenario).read_text(encoding="utf-8"))
    ref, options, block = resolve(args.controller, _opts(args.option))
    text = ros2.dump(ros2.params_from_scenario(tree, ref, options, block),
                     args.scenario)
    if args.out:
        Path(args.out).write_text(text, encoding="utf-8")
        print(f"wrote {args.out}\nros2 launch cacc_platoon platoon.launch.py "
              f"params:={Path(args.out).resolve()}", file=sys.stderr)
    else:
        print(text)
    return 0


def cmd_carla(args) -> int:
    from dataclasses import replace

    from cacc.adapters.carla_bridge import run_carla
    from cacc.scenario import load_scenario
    from cacc.tfctrl import resolve

    ref, options, block = resolve(args.controller, _opts(args.option))
    sc = load_scenario(args.scenario)
    if block:
        sc.config = replace(sc.config, control=replace(sc.config.control, **block))
    log = run_carla(sc, ref, options, args.host, args.port, args.town or None,
                    args.dt, args.out, spawn_index=args.spawn)
    print(f"{log.speed.shape[0]} frames, min gap {log.gap.min():.2f} m"
          + (f" · log: {args.out} (try: cacc audit {args.out})" if args.out else ""))
    return 0


def register(sub) -> None:
    ci = sub.add_parser("ci", help="CI: PR comment, job summary and badge "
                        "from a report.json")
    ci.add_argument("report")
    ci.add_argument("--badge", help="write an SVG status badge here")
    ci.add_argument("--summary", help="append the markdown to this file "
                    "(e.g. $GITHUB_STEP_SUMMARY)")
    ci.add_argument("--comment", action="store_true",
                    help="post / update the result on the pull request")
    ci.add_argument("--title", default="cacc controller gate")
    ci.add_argument("--gate", action="store_true",
                    help="exit 1 on FAIL (0 by default: the run step gates)")
    ci.set_defaults(fn=cmd_ci)

    lab = sub.add_parser("lab", help="Controller Lab: analyse a transfer-"
                         "function design, export to MATLAB")
    lab.add_argument("design", nargs="?", help="design YAML")
    lab.add_argument("--preset", help="start from a built-in law: "
                     "cacc | cthp | acc | lag-ff")
    lab.add_argument("-o", "--out", help="with --preset: write the design here")
    lab.add_argument("--tau", type=float, default=0.2, help="vehicle lag [s]")
    lab.add_argument("--delay", type=float, default=0.05, help="V2V delay [s]")
    lab.add_argument("--act-delay", type=float, default=0.0,
                     help="actuation dead time [s]")
    lab.add_argument("--matlab", help="write a MATLAB/Simulink script here")
    lab.set_defaults(fn=cmd_lab)

    for name, fn, hlp in (("ros2-params", cmd_ros2_params,
                           "ROS 2 params YAML for ros2_ws"),
                          ("carla", cmd_carla, "run the platoon in a CARLA "
                           "server, log it for cacc audit")):
        s = sub.add_parser(name, help=hlp)
        s.add_argument("scenario", help="scenario YAML")
        s.add_argument("-c", "--controller", default="cthp",
                       help="acc | cacc | cthp | module:Name | design .yaml")
        s.add_argument("--option", action="append", default=[],
                       metavar="KEY=VALUE", help="plugin constructor option")
        s.add_argument("-o", "--out", help="output file")
        s.set_defaults(fn=fn)
        if name == "carla":
            s.add_argument("--host", default="localhost")
            s.add_argument("--port", type=int, default=2000)
            s.add_argument("--town", default="Town04",
                           help="map to load ('' keeps the current one)")
            s.add_argument("--dt", type=float, default=0.05, help="frame [s]")
            s.add_argument("--spawn", type=int, default=0,
                           help="spawn point index for the leader")
