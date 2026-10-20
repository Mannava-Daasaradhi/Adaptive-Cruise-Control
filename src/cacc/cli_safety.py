"""CLI commands for release decisions: standards, energy, version history
and hardware in the loop.

    cacc standards -c CTRL [--aeb] [-o DIR]            ISO / UNECE / NCAP checklist
    cacc energy -c cacc cthp acc [-m M] [-k C]          range + comfort table
    cacc history save REPORT.json --label v2            store a version
    cacc history diff [OLD] [NEW]                       regressions (exit 1)
    cacc history show | list                            all versions
    cacc hil --udp HOST:PORT | --serial COM5 | --can can0 | --spawn-device
    cacc hil-device --udp 0.0.0.0:5005                   run the device side here
"""

from __future__ import annotations

import json
import sys
from pathlib import Path


def _jobs(n):
    from cacc.cli_insight import _jobs as j

    return j(n)


def cmd_standards(args) -> int:
    from cacc import standards as st
    from cacc.cli_tools import _opts
    from cacc.tfctrl import resolve

    ref, options, block = resolve(args.controller, _opts(args.option))
    res = st.check(ref, options, block, aeb=args.aeb, seeds=args.seeds,
                   jobs=_jobs(args.jobs))
    res["controller"] = args.controller
    print(st.format_text(res))
    if args.out:
        print(f"written: {st.write(res, args.out)}", file=sys.stderr)
    return 1 if res["counts"]["FAIL"] else 0


def cmd_energy(args) -> int:
    from cacc import energy

    rows = energy.compare(args.controller, args.maneuver, args.condition, aeb=args.aeb)
    print(f"energy & comfort · {args.maneuver} | {args.condition}"
          + (" · AEB" if args.aeb else ""))
    print(energy.format_table(rows))
    print("kWh, range: platoon average · vs lead: worst follower's extra "
          "consumption over the lead car (negative = slipstream gain) · harsh: "
          "worst share of time beyond the ISO 15622 comfort limits")
    return 0


def cmd_history(args) -> int:
    from cacc import history as hi

    if args.action == "save":
        if not args.what:
            print("error: give the report.json to save", file=sys.stderr)
            return 2
        report = json.loads(Path(args.what[0]).read_text(encoding="utf-8"))
        path = hi.save(report, args.label or Path(args.what[0]).parent.name, args.store)
        print(f"saved {path}")
        return 0
    if args.action == "list":
        for v in hi.versions(args.store):
            c = v["counts"]
            print(f"  {v['label']:<16} {v['saved']}  {v['controller']:<10} "
                  f"{c.get('PASS', 0)} pass / {c.get('FAIL', 0)} fail"
                  + (f"  @{v['commit']}" if v.get("commit") else ""))
        return 0
    if args.action == "show":
        print(hi.format_table(args.store))
        return 0
    old = hi.find(args.store, args.what[0] if len(args.what) > 0 else None, -2)
    new = hi.find(args.store, args.what[1] if len(args.what) > 1 else None, -1)
    rows = hi.diff(old, new, args.tol)
    print(hi.format_diff(old, new, rows))
    return 1 if any(r["change"] == "regression" for r in rows) else 0


def cmd_hil(args) -> int:
    from cacc.adapters import hil

    if args.udp:
        transport, address = "udp", args.udp
    elif args.serial:
        transport, address = "serial", args.serial
    elif args.can:
        transport, address = "can", args.can
    elif args.spawn_device:
        transport, address = "udp", ""
    else:
        print("error: give --udp, --serial, --can or --spawn-device", file=sys.stderr)
        return 2
    r = hil.run(args.maneuver, args.condition, transport, address, args.cars,
                args.step, args.timeout, args.realtime, args.law, args.spawn_device)
    print(hil.format_text(r))
    missed = sum(s["misses"] for s in r["timing"])
    return 1 if missed or not r["timing"] else 0


def cmd_hil_device(args) -> int:
    from cacc.adapters import hil_device

    argv = ([f"--{k}", v] for k, v in (("udp", args.udp), ("serial", args.serial),
                                        ("can", args.can)) if v)
    return hil_device.main([*next(argv, []), "--law", args.law, "--kp", str(args.kp),
                            "--kd", str(args.kd), "--h", str(args.h),
                            "--baud", str(args.baud)])


def register(sub) -> None:
    from cacc.campaign import CONDITIONS, MANEUVERS

    s = sub.add_parser("standards", help="checklist against ISO 15622 / 20035, "
                       "UNECE R157, Euro NCAP AEB, fault reaction (pre-assessment)")
    s.add_argument("-c", "--controller", default="cacc")
    s.add_argument("--aeb", action="store_true", help="with the emergency-braking layer")
    s.add_argument("--option", action="append", default=[], metavar="KEY=VALUE")
    s.add_argument("--seeds", type=int, default=1)
    s.add_argument("-j", "--jobs", type=int)
    s.add_argument("-o", "--out", help="write standards.md + standards.json here")
    s.set_defaults(fn=cmd_standards)

    e = sub.add_parser("energy", help="range lost, regen and comfort per controller")
    e.add_argument("-c", "--controller", nargs="+", default=["cthp", "cacc", "acc"])
    e.add_argument("-m", "--maneuver", default="stop-and-go", choices=list(MANEUVERS))
    e.add_argument("-k", "--condition", default="nominal", choices=list(CONDITIONS))
    e.add_argument("--aeb", action="store_true")
    e.set_defaults(fn=cmd_energy)

    h = sub.add_parser("history", help="versions of stress results: save, diff "
                       "(exit 1 on a regression), show")
    h.add_argument("action", choices=("save", "list", "diff", "show"))
    h.add_argument("what", nargs="*", help="save: REPORT.json · diff: [OLD] [NEW] labels")
    h.add_argument("--label", help="version label (save; default: report folder name)")
    h.add_argument("--store", default=".cacc-history", help="history folder")
    h.add_argument("--tol", type=float, default=0.05,
                   help="margin change that counts as worse/better")
    h.set_defaults(fn=cmd_history)

    x = sub.add_parser("hil", help="hardware in the loop: drive a controller on "
                       "a Raspberry Pi / Arduino / CAN ECU from the simulator")
    g = x.add_mutually_exclusive_group()
    g.add_argument("--udp", metavar="HOST:PORT")
    g.add_argument("--serial", metavar="PORT")
    g.add_argument("--can", metavar="CHANNEL")
    g.add_argument("--spawn-device", action="store_true",
                   help="start a device on this machine (demo without hardware)")
    x.add_argument("-m", "--maneuver", default="hard-brake", choices=list(MANEUVERS))
    x.add_argument("-k", "--condition", default="nominal", choices=list(CONDITIONS))
    x.add_argument("--cars", type=int, nargs="+", help="followers on hardware "
                   "(default all; the rest run the same law in software)")
    x.add_argument("--step", type=float, default=0.02, help="control period [s]")
    x.add_argument("--timeout", type=float, default=0.05, help="reply deadline [s]")
    x.add_argument("--realtime", action="store_true", help="pace to the wall clock")
    x.add_argument("--law", choices=("cacc", "acc"), default="cacc")
    x.set_defaults(fn=cmd_hil)

    d = sub.add_parser("hil-device", help="run the HIL device side on this "
                       "machine (same as python hil_device.py on a Pi)")
    g = d.add_mutually_exclusive_group(required=True)
    g.add_argument("--udp", metavar="HOST:PORT")
    g.add_argument("--serial", metavar="PORT")
    g.add_argument("--can", metavar="CHANNEL")
    d.add_argument("--law", choices=("cacc", "acc"), default="cacc")
    d.add_argument("--kp", type=float, default=0.2)
    d.add_argument("--kd", type=float, default=0.7)
    d.add_argument("--h", type=float, default=0.7)
    d.add_argument("--baud", type=int, default=115200)
    d.set_defaults(fn=cmd_hil_device)
