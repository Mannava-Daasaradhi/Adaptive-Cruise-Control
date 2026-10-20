"""Breaking points and safety envelopes: where exactly a controller fails.

A stress grid says pass/fail at fixed conditions (300 ms latency, 30 %
loss...). An envelope search answers the engineer's real question: *how
much* can go wrong before the car breaks?

* :func:`breaking_point` drives one **axis** (V2V latency, packet loss,
  road grip, motor power, actuator dead time, spoofed acceleration, cut-in
  speed, lead-car braking) from benign to harsh: a coarse scan, then
  bisection between the last pass and the first failure, to ``tol`` of the
  axis range. "This controller breaks at 340 ms latency."
* :func:`profile` runs it for every axis: the tornado chart of margins.
* :func:`envelope` maps two axes against each other (n x n cells, one test
  plan, run in parallel): the pass/fail region and its boundary.

Every point is an ordinary test-plan case judged by the criteria, its
margin the binding criterion's signed relative margin
(:func:`cacc.campaign.margin`), so a breaking point is reproducible as a
case and a seed like any other result.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass

import numpy as np
import yaml

from cacc import campaign as cp
from cacc.evaluate import evaluate, plan_from_dict


@dataclass(frozen=True)
class Axis:
    key: str
    label: str
    unit: str
    benign: float
    harsh: float
    fmt: str = "{:.3g}"

    def value(self, x: float) -> float:
        return self.benign + (self.harsh - self.benign) * x

    def show(self, v: float) -> str:
        return f"{self.fmt.format(v)} {self.unit}".strip()


AXES: dict[str, Axis] = {a.key: a for a in (
    Axis("latency", "V2V latency", "s", 0.0, 1.0, "{:.2f}"),
    Axis("loss", "Packet loss (10 Hz link)", "%", 0.0, 95.0, "{:.0f}"),
    Axis("grip", "Road grip μ", "", 1.0, 0.1, "{:.2f}"),
    Axis("power", "Follower motor power", "kW", 150.0, 10.0, "{:.0f}"),
    Axis("dead-time", "Actuator dead time", "s", 0.0, 0.8, "{:.2f}"),
    Axis("spoof", "Spoofed acceleration on link 1", "m/s²", 0.0, 10.0, "{:.1f}"),
    Axis("cut-in", "Cut-in car slower by", "m/s", 0.0, 12.0, "{:.1f}"),
    Axis("lead-brake", "Lead-car braking", "m/s²", 1.0, 9.5, "{:.1f}"),
)}


def apply_axis(tree: dict, key: str, v: float) -> dict:
    """Set axis ``key`` to value ``v`` in a scenario mapping (in place)."""
    if key == "latency":
        tree.setdefault("network", {})["delay"] = float(v)
    elif key == "loss":
        net = tree.setdefault("network", {})
        net.update(msg_rate=10.0, loss_prob=min(float(v) / 100.0, 0.99))
    elif key == "grip":
        ev = tree.setdefault("ev", {})
        ev.pop("surface", None)
        ev["mu"] = float(v)
    elif key == "power":
        tree.setdefault("ev", {})["power_kw"] = float(v)
    elif key == "dead-time":
        tree.setdefault("vehicle", {})["act_delay"] = (
            0.0 if v < 0.01 else round(float(v), 3))
    elif key == "spoof":
        if v > 1e-9:
            tree["attack"] = {"kind": "bias", "value": float(v), "ramp": 1.0,
                              "target_link": 1, "t0": 10.0, "t1": 25.0}
    elif key == "cut-in":
        n = int(tree.get("platoon", {}).get("n_followers", 5))
        tree["traffic"] = {"cut_ins": [{"t": 10.0, "ahead_of": min(3, n),
                                        "gap_frac": 0.5, "dv": -float(v),
                                        "decel": -2.0, "duration": 1.5}]}
    elif key == "lead-brake":
        dv = 15.0  # same speed drop at every braking level
        tree["leader"] = {"profile": "brake", "t_start": 10.0,
                          "decel": -float(v), "duration": dv / float(v)}
    else:
        raise ValueError(f"unknown axis {key!r}; use one of {', '.join(AXES)}")
    return tree


def _base(controller: str, maneuver: str, condition: str,
          controller_block: dict | None) -> dict:
    tree = yaml.safe_load(cp.BASE.read_text(encoding="utf-8"))
    if controller_block:
        tree.setdefault("controller", {}).update(controller_block)
    return cp.compose(tree, maneuver, condition, controller)


def _physical(tree: dict) -> dict:
    """Keep the lead car's braking within the road's grip, and give the run
    time to settle after the longest stop."""
    t_final = float(tree.get("sim", {}).get("t_final", 45.0))
    cp.cap_leader(tree, 10.0, t_final)
    ld = tree["leader"]
    if ld.get("profile") == "brake":
        tree["sim"]["t_final"] = max(t_final, 10.0 + float(ld["duration"]) + 30.0)
    return tree


def _run(trees: list[tuple[str, dict]], controller, options, criteria, seeds,
         jobs, progress=None) -> list[dict]:
    """Evaluate named scenario trees as one plan -> case reports in order."""
    raw = {"name": "envelope", "scenario": trees[0][1],
           "under_test": {"controller": controller, "options": options or {}},
           "seeds": seeds, "criteria": list(criteria),
           "cases": [{"name": n, "set": t} for n, t in trees]}
    rep = evaluate(plan_from_dict(raw), jobs=jobs, progress=progress)
    return rep["cases"]


def _margin(case: dict) -> float:
    if case["verdict"] == "ERROR" or not case["checks"]:
        return float("-inf")
    return min(cp.margin(c) for c in case["checks"])


def _binding(case: dict) -> str:
    if not case["checks"]:
        return "error"
    return min(case["checks"], key=cp.margin)["label"]


def breaking_point(controller: str = "cthp", maneuver: str = "hard-brake",
                   axis: str = "latency", criteria=cp.DEFAULT_CRITERIA,
                   options=None, seeds: int = 1, condition: str = "nominal",
                   controller_block: dict | None = None, coarse: int = 7,
                   bisect: int = 4, jobs: int = 1, progress=None) -> dict:
    """Find where ``axis`` first makes the case fail (see module doc)."""
    ax = AXES[axis]
    base = _base(controller, maneuver, condition, controller_block)
    xs = list(np.linspace(0.0, 1.0, coarse))

    def tree(x):
        return _physical(apply_axis(copy.deepcopy(base), axis, ax.value(x)))

    cases = _run([(f"{axis}={ax.value(x):.4g}", tree(x)) for x in xs],
                 controller, options, criteria, seeds, jobs, progress)
    pts = [(ax.value(x), _margin(c), c["verdict"]) for x, c in zip(xs, cases)]
    fail = next((k for k, c in enumerate(cases) if c["verdict"] != "PASS"), None)
    out = {"axis": axis, "label": ax.label, "unit": ax.unit,
           "benign": ax.benign, "harsh": ax.harsh, "maneuver": maneuver,
           "condition": condition, "controller": controller}
    if fail is None:
        out.update(status="holds", value=ax.harsh, binding="",
                   text=f"holds across the whole range (up to {ax.show(ax.harsh)})")
    elif fail == 0:
        out.update(status="fails", value=ax.benign, binding=_binding(cases[0]),
                   text=f"fails even at {ax.show(ax.benign)} "
                        f"({_binding(cases[0])})")
    else:
        lo, hi, hit = xs[fail - 1], xs[fail], cases[fail]
        for _ in range(bisect):
            mid = 0.5 * (lo + hi)
            (c,) = _run([(f"{axis}={ax.value(mid):.4g}", tree(mid))],
                        controller, options, criteria, seeds, 1)
            pts.append((ax.value(mid), _margin(c), c["verdict"]))
            if c["verdict"] == "PASS":
                lo = mid
            else:
                hi, hit = mid, c
        v = ax.value(hi)
        out.update(status="breaks", value=v, last_pass=ax.value(lo),
                   binding=_binding(hit),
                   text=f"breaks at {ax.show(v)} ({_binding(hit)}); "
                        f"last pass {ax.show(ax.value(lo))}")
    out["points"] = sorted(pts)
    out["fraction"] = (0.0 if out["status"] == "fails" else 1.0
                       if out["status"] == "holds"
                       else (out["value"] - ax.benign) / (ax.harsh - ax.benign))
    return out


def profile(controller: str = "cthp", maneuver: str = "hard-brake",
            axes=None, progress=None, **kw) -> list[dict]:
    """:func:`breaking_point` for every axis, most fragile first."""
    keys = list(axes or AXES)
    out = []
    for k, key in enumerate(keys):
        out.append(breaking_point(controller, maneuver, key, **kw))
        if progress:
            progress(k + 1, len(keys))
    return sorted(out, key=lambda r: r["fraction"])


def envelope(controller: str = "cthp", maneuver: str = "hard-brake",
             axis_x: str = "latency", axis_y: str = "grip", n: int = 7,
             criteria=cp.DEFAULT_CRITERIA, options=None, seeds: int = 1,
             condition: str = "nominal", controller_block: dict | None = None,
             jobs: int = 1, progress=None) -> dict:
    """Two axes against each other: margins on an n x n grid."""
    ax, ay = AXES[axis_x], AXES[axis_y]
    if axis_x == axis_y:
        raise ValueError("pick two different axes")
    base = _base(controller, maneuver, condition, controller_block)
    xs = [ax.value(x) for x in np.linspace(0, 1, n)]
    ys = [ay.value(y) for y in np.linspace(0, 1, n)]
    trees = []
    for vy in ys:
        for vx in xs:
            t = _physical(apply_axis(apply_axis(copy.deepcopy(base), axis_x, vx),
                                     axis_y, vy))
            trees.append((f"{axis_x}={vx:.4g},{axis_y}={vy:.4g}", t))
    cases = _run(trees, controller, options, criteria, seeds, jobs, progress)
    margin = np.array([_margin(c) for c in cases]).reshape(n, n)
    verdict = np.array([c["verdict"] for c in cases]).reshape(n, n)
    return {"axis_x": axis_x, "axis_y": axis_y, "x": xs, "y": ys,
            "x_label": f"{ax.label} [{ax.unit}]" if ax.unit else ax.label,
            "y_label": f"{ay.label} [{ay.unit}]" if ay.unit else ay.label,
            "margin": margin, "verdict": verdict, "maneuver": maneuver,
            "pass_share": float((verdict == "PASS").mean())}


def format_profile(rows: list[dict]) -> str:
    w = max(len(r["label"]) for r in rows)
    return "\n".join(f"{r['label'].ljust(w)}  {r['text']}" for r in rows)
