"""Stress campaigns: one controller against a maneuver x condition grid.

A campaign is a test plan (:mod:`cacc.evaluate`) generated from two
libraries: **maneuvers** (what the lead car does: emergency stop, traffic
wave, stop-and-go...) and **conditions** (what goes wrong around it: V2V
latency, packet loss, spoofing, a wet road, a derated battery, slow
brakes...). Every (maneuver, condition) pair becomes one case, run over the
seeds and judged by the criteria, so the result reads as a grid: rows are
maneuvers, columns are conditions, each cell a verdict with its tightest
margin. Cells are reproducible by (case scenario, seed) like any plan case.

Composition rules that keep a cell physical:

* the lead car is on the same road as the followers, so its braking is
  capped at the surface grip ``mu*g`` (a -7 m/s^2 stop on ice is not a
  stress test, it is a scenario that cannot happen);
* an attack window opens when the maneuver starts and lasts 15 s;
* the controller block comes from the chosen controller family (Ploeg
  gains for acc/cacc, Ma-2025 case-A gains for cthp; plugins keep the base).
"""

from __future__ import annotations

import copy
import sys
from pathlib import Path

import yaml

from cacc.ev import SURFACES, G
from cacc.evaluate import TestPlan, evaluate, plan_from_dict

#: default base scenario (repo checkout, or the bundle of the packaged .exe)
BASE = (Path(getattr(sys, "_MEIPASS", ".")) if getattr(sys, "frozen", False)
        else Path(__file__).resolve().parents[2]) / "scenarios" / "ev_highway.yaml"

#: maneuvers: key -> (label, leader block, t_start, t_final)
MANEUVERS: dict[str, tuple[str, dict, float, float]] = {
    "cruise": ("Steady cruise", {"profile": "constant", "value": 0.0,
                                 "probe_amplitude": 0.05}, 10.0, 40.0),
    "traffic-wave": ("Traffic wave (0.5 m/s² sine)",
                     {"profile": "sine", "t_start": 10.0, "amplitude": 0.5,
                      "freq_hz": 0.05, "duration": 40.0}, 10.0, 70.0),
    "stop-and-go": ("Stop-and-go (±4.8 m/s swings)",
                    {"profile": "bursts", "bursts": [[10.0, 40.0, 1.5, 0.05]]},
                    10.0, 70.0),
    "hard-brake": ("Hard brake (−5 m/s² for 3 s)",
                   {"profile": "brake", "t_start": 10.0, "duration": 3.0,
                    "decel": -5.0}, 10.0, 45.0),
    "emergency-stop": ("Emergency stop (−8 m/s² to 0)",
                       {"profile": "brake", "t_start": 10.0, "duration": 3.1,
                        "decel": -8.0}, 10.0, 45.0),
    "full-throttle": ("Full throttle (+2 m/s² for 4 s)",
                      {"profile": "brake", "t_start": 10.0, "duration": 4.0,
                       "decel": 2.0}, 10.0, 45.0),
}

#: conditions: key -> (label, group, dot-path edits)
CONDITIONS: dict[str, tuple[str, str, dict]] = {
    "nominal": ("Nominal", "baseline", {}),
    "latency-150ms": ("V2V latency 150 ms", "radio", {"network.delay": 0.15}),
    "latency-300ms": ("V2V latency 300 ms", "radio", {"network.delay": 0.30}),
    "loss-30": ("30 % packet loss (10 Hz)", "radio",
                {"network.msg_rate": 10.0, "network.loss_prob": 0.30}),
    "burst-loss": ("Burst loss (5-msg bursts)", "radio",
                   {"network.msg_rate": 10.0, "network.loss_prob": 0.20,
                    "network.loss_burst": 5.0}),
    "noisy-v2v": ("Noisy V2V (ρ = 2)", "radio", {"network.noise_rho": 2.0}),
    "spoof": ("Spoofed +4 m/s² (no gate)", "security",
              {"attack": {"kind": "bias", "value": 4.0, "ramp": 1.0,
                          "target_link": 1}}),
    "spoof-gated": ("Spoofed +4 m/s², trust gate on", "security",
                    {"attack": {"kind": "bias", "value": 4.0, "ramp": 1.0,
                                "target_link": 1},
                     "trust": {"enabled": True, "r0": 3.0, "radar_noise": 0.05}}),
    "wet": ("Wet road (μ = 0.5)", "road", {"ev.surface": "wet"}),
    "snow": ("Snow (μ = 0.3)", "road", {"ev.surface": "snow"}),
    "low-battery": ("Low battery (followers 60 kW)", "powertrain",
                    {"ev.power_kw": 60.0}),
    "cold-battery": ("Cold battery (regen 10 kW)", "powertrain",
                     {"ev.regen_kw": 10.0}),
    "full-load": ("Full load (2.5 t)", "powertrain", {"ev.mass": 2500.0}),
    "slow-brakes": ("Actuator dead time 200 ms", "powertrain",
                    {"vehicle.act_delay": 0.2}),
    "long-platoon": ("10-car platoon", "traffic", {"platoon.n_followers": 10}),
}

#: controller-family gains (the base scenario's block is Ploeg 2014)
FAMILY_GAINS = {
    "cthp": {"kp": 0.009, "kv": 0.63, "ka": 0.5, "h": 0.95, "r": 2.5},
    "acc": {"kp": 0.2, "kd": 0.7, "h": 1.4, "r": 2.5},  # ACC needs a long gap
}

DEFAULT_CRITERIA = ("min_gap >= 1.0", "min_ttc >= 1.5",
                    "energy_kwh_100km <= 30.0")
PRESETS = {
    "quick": (("cruise", "hard-brake", "emergency-stop"),
              ("nominal", "latency-300ms", "loss-30", "spoof", "spoof-gated",
               "wet", "slow-brakes")),
    "full": (tuple(MANEUVERS), tuple(CONDITIONS)),
}


def _set(tree: dict, path: str, value) -> None:
    keys = path.split(".")
    node = tree
    for k in keys[:-1]:
        node = node.setdefault(k, {})
    node[keys[-1]] = copy.deepcopy(value)


def compose(base: dict, maneuver: str, condition: str, controller: str) -> dict:
    """The scenario mapping of one grid cell."""
    if maneuver not in MANEUVERS:
        raise ValueError(f"unknown maneuver {maneuver!r}; use one of "
                         f"{', '.join(MANEUVERS)}")
    if condition not in CONDITIONS:
        raise ValueError(f"unknown condition {condition!r}; use one of "
                         f"{', '.join(CONDITIONS)}")
    tree = copy.deepcopy(base)
    _, leader, t_start, t_final = MANEUVERS[maneuver]
    tree["leader"] = copy.deepcopy(leader)
    _set(tree, "sim.t_final", t_final)
    fam = controller.lower()
    if fam in FAMILY_GAINS:
        tree["controller"] = dict(FAMILY_GAINS[fam])
    for path, value in CONDITIONS[condition][2].items():
        _set(tree, path, value)
    if "attack" in tree:
        tree["attack"].setdefault("t0", t_start)
        tree["attack"].setdefault("t1", t_start + 15.0)
    ev = tree.get("ev") or {}
    mu = ev.get("mu", SURFACES.get(ev.get("surface", "dry"), SURFACES["dry"]))
    if tree["leader"].get("profile") == "brake" and "ev" in tree:
        lim = mu * G
        dec = float(tree["leader"].get("decel", -4.0))
        if dec < -lim:  # same road: cap the stop at grip, keep the speed drop
            tree["leader"]["duration"] *= dec / -lim
            tree["leader"]["decel"] = -lim
            _set(tree, "sim.t_final",
                 max(t_final, t_start + tree["leader"]["duration"] + 30.0))
        tree["leader"]["decel"] = min(tree["leader"]["decel"], lim)
    return tree


def build_plan(controller: str = "cthp", maneuvers=None, conditions=None,
               seeds: int = 2, criteria=DEFAULT_CRITERIA, options=None,
               base: str | Path | dict | None = None,
               name: str = "stress-campaign",
               controller_block: dict | None = None) -> TestPlan:
    """A :class:`TestPlan` with one case per (maneuver, condition) cell.

    ``controller_block`` overrides entries of the base scenario's
    ``controller:`` block (e.g. the spacing policy of a Controller Lab
    design); built-in families still take their own gains.
    """
    maneuvers = list(maneuvers or PRESETS["quick"][0])
    conditions = list(conditions or PRESETS["quick"][1])
    if isinstance(base, dict):
        base_tree, root = copy.deepcopy(base), Path(".")
    else:
        path = Path(base) if base else BASE
        base_tree = yaml.safe_load(path.read_text(encoding="utf-8"))
        root = path.parent
    if controller_block:
        base_tree.setdefault("controller", {}).update(controller_block)
    if controller.lower() == "acc":  # nothing travels over V2V
        conditions = [c for c in conditions
                      if CONDITIONS[c][1] not in ("radio", "security")] or ["nominal"]
    cases = [{"name": f"{m} | {c}",
              "set": compose(base_tree, m, c, controller)}
             for m in maneuvers for c in conditions]
    raw = {"name": name,
           "description": f"{len(maneuvers)} maneuver(s) x {len(conditions)} "
                          "condition(s) stress campaign",
           "scenario": base_tree,
           "under_test": {"controller": controller, "options": options or {}},
           "seeds": seeds, "cases": cases, "criteria": list(criteria)}
    plan = plan_from_dict(raw, root=root)
    plan.campaign = {"maneuvers": maneuvers, "conditions": conditions}
    return plan


def margin(check: dict) -> float:
    """Signed relative margin of one check (> 0 passes, < 0 fails)."""
    worst, thr = check["worst"], check["threshold"]
    if worst is None or worst != worst:
        return float("-inf")
    if worst in (float("inf"), float("-inf")):
        good = (worst > 0) == (check["op"] in (">=", ">"))
        return float("inf") if good else float("-inf")
    scale = max(abs(thr), 1e-9)
    d = worst - thr if check["op"] in (">=", ">") else thr - worst
    return d / scale


def grid(report: dict, maneuvers, conditions) -> list[list[dict]]:
    """``report`` cases arranged as rows (maneuvers) x columns (conditions).

    Each cell: verdict, the binding check (smallest margin) and its margin.
    """
    by_name = {c["name"]: c for c in report["cases"]}
    rows = []
    for m in maneuvers:
        row = []
        for c in conditions:
            case = by_name.get(f"{m} | {c}")
            if case is None:
                row.append(None)
                continue
            checks = sorted(case["checks"], key=margin)
            tight = checks[0] if checks else None
            row.append({"name": case["name"], "verdict": case["verdict"],
                        "binding": tight["label"] if tight else "",
                        "metric": tight["metric"] if tight else "",
                        "worst": tight["worst"] if tight else None,
                        "worst_seed": tight["worst_seed"] if tight else None,
                        "margin": margin(tight) if tight else 0.0})
        rows.append(row)
    return rows


def run_campaign(plan: TestPlan, jobs: int = 1, progress=None) -> dict:
    """Evaluate a campaign plan; the report gains a ``campaign`` grid block."""
    report = evaluate(plan, jobs=jobs, progress=progress)
    camp = plan.campaign
    report["campaign"] = {
        "maneuvers": [{"key": m, "label": MANEUVERS[m][0]} for m in camp["maneuvers"]],
        "conditions": [{"key": c, "label": CONDITIONS[c][0],
                        "group": CONDITIONS[c][1]} for c in camp["conditions"]],
        "grid": grid(report, camp["maneuvers"], camp["conditions"]),
    }
    return report


def replay(case: dict, controller: str, seed: int, options: dict | None = None):
    """Re-simulate one campaign cell for telemetry (a :class:`SimResult`)."""
    from dataclasses import replace

    from cacc.platoon import PlatoonSim
    from cacc.scenario import scenario_from_dict

    sc = scenario_from_dict(case["scenario"], default_name=case["name"])
    return PlatoonSim(replace(sc.config, seed=int(seed)), controller, sc.leader,
                      options or {}).run()


def format_grid(report: dict) -> str:
    """Plain-text grid for the terminal: ✓/✕ with the binding margin."""
    camp = report["campaign"]
    conds = [c["key"] for c in camp["conditions"]]
    w = max(14, *(len(c) for c in conds))
    lines = ["maneuver \\ condition".ljust(18) + "".join(c[:w].rjust(w + 1) for c in conds)]
    for man, row in zip(camp["maneuvers"], camp["grid"], strict=True):
        cells = []
        for cell in row:
            if cell is None:
                cells.append("—".rjust(w + 1))
                continue
            mark = {"PASS": "✓", "FAIL": "✕"}.get(cell["verdict"], "!")
            m = cell["margin"]
            txt = f"{mark} {m:+.0%}" if abs(m) < 100 else f"{mark}"
            cells.append(txt.rjust(w + 1))
        lines.append(man["key"].ljust(18) + "".join(cells))
    return "\n".join(lines)
