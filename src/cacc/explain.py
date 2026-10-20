"""Why did it fail? A physics root-cause for one failed stress cell.

Given a campaign case (its scenario, checks and worst seed) and the replayed
run, :func:`explain` works out, from the run and the plant model:

* **what** failed, **when**, and **which car** (the binding criterion at its
  worst sample);
* **why**, by checking the mechanisms in turn: braking demanded vs what the
  road and motor allowed (and how long the actuator sat on that limit), how
  late the car reacted to the car in front, whether the disturbance grew
  down the platoon (and, for a known law, the analytic |Γ| peak with the
  cell's V2V delay against the clean link, with the phase the delay costs),
  a spoofed or lost V2V message, a human or a cut-in car that broke the V2V
  chain, an energy limit;
* **what fixes it**: candidate changes (longer headway, the V2V trust gate,
  stronger gap feedback...) are re-simulated on the same seed and reported
  with their verdict, so a suggestion is verified, not guessed.
"""

from __future__ import annotations

import copy
import math

import numpy as np

from cacc.campaign import margin, replay
from cacc.ev import EVParams
from cacc.traffic import front_tracks


def _criteria(report: dict) -> list[str]:
    return [f"{c['metric']} {c['op']} {c['value']:g}" for c in report["criteria"]]


def design_of(controller: str, params, options: dict | None):
    """A :class:`cacc.tfctrl.Design` for a built-in or Lab controller (for
    the analytic |Γ|), or None for an opaque plugin."""
    from cacc import tfctrl as tc

    p = params
    if controller == tc.REF:
        o = options or {}
        return tc.Design(dict(o.get("blocks") or {}), o.get("ff_signal", "u"), p.h, p.r)
    blocks = {"cacc": ({"e": f"{p.kp}/(h*s + 1)", "e_dot": f"{p.kd}/(h*s + 1)",
                        "u_ff": "1/(h*s + 1)"}, "u"),
              "cthp": ({"e": f"{p.kp}", "dv": f"{p.kv}", "u_ff": f"{p.ka}"}, "a"),
              "acc": ({"e": f"{p.kp}", "e_dot": f"{p.kd}"}, "u")}.get(controller)
    if blocks is None:
        return None
    return tc.Design(blocks[0], blocks[1], p.h, p.r)


def _culprit(res, metric: str) -> tuple[int, int, str]:
    """(sample, column, who) where a gap / TTC metric was worst."""
    gaps, closing = front_tracks(res)
    n = res.vel.shape[1] - 1
    if metric == "min_ttc":
        with np.errstate(divide="ignore", invalid="ignore"):
            ttc = np.where(closing > 0.1, np.maximum(gaps, 0) / closing, np.inf)
        k, j = np.unravel_index(np.argmin(ttc), ttc.shape)
    else:
        k, j = np.unravel_index(np.argmin(gaps), gaps.shape)
    who = f"car {j + 1}" if j < n else "the car that cut in"
    return int(k), int(j), who


def _bounds(res, i: int) -> tuple[np.ndarray, np.ndarray]:
    veh = res.config.vehicle
    ev = veh.ev
    v = res.vel[:, i]
    if ev is None:
        return np.full(v.size, veh.u_min), np.full(v.size, veh.u_max)
    b = np.array([ev.accel_bounds(float(x), veh.u_min, veh.u_max) for x in v])
    return b[:, 0], b[:, 1]


def diagnose(res, case: dict, controller: str, options: dict | None) -> list[dict]:
    """The mechanisms that explain the binding failure (most telling first)."""
    cfg, tree = res.config, case["scenario"]
    n = cfg.n_followers
    failed = [c for c in case["checks"] if not c["passed"]] or case["checks"]
    bind = min(failed, key=margin)
    out = []
    metric = bind["metric"]
    if metric in ("min_gap", "min_ttc", "min_time_gap"):
        k, j, who = _culprit(res, "min_ttc" if metric == "min_ttc" else "min_gap")
        gaps, closing = front_tracks(res)
        hit = np.flatnonzero(gaps[:, j] <= 0.0)
        if hit.size:  # report the impact, not the overlap after it
            k = int(hit[0])
            out.append({"title": f"{who} hit the car in front at t = {res.t[k]:.1f} s",
                        "detail": f"impact at {closing[max(k - 1, 0), j] * 3.6:.0f} "
                                  f"km/h closing speed ({bind['label']} broken)"})
        else:
            out.append({"title": f"{bind['label']} broken by {who} at "
                                 f"t = {res.t[k]:.1f} s",
                        "detail": f"gap {gaps[k, j]:.2f} m while closing at "
                                  f"{closing[k, j]:.1f} m/s"})
        if j < n:
            out += _braking(res, gaps[:, j], closing[:, j], j + 1, k, tree)
    out += _condition_findings(res, tree, case)
    ratios = _amplification(res)
    if ratios is not None and ratios.max() > 1.02:
        hop = int(np.argmax(ratios)) + 2
        out.append({"title": "the disturbance grew down the platoon",
                    "detail": f"peak spacing error grew ×{ratios.max():.2f} from car "
                              f"{hop - 1} to car {hop} (string-unstable response)"})
    out += _gamma_finding(res, controller, options)
    if metric in ("energy_kwh_100km", "regen_share"):
        ev = cfg.vehicle.ev or EVParams()
        out.append({"title": f"{bind['label']}: {bind['worst']:.3g}",
                    "detail": f"regen limited to {ev.regen_kw:.0f} kW and motor "
                              f"{ev.power_kw:.0f} kW; hard transients burn energy "
                              "the regen cannot recover"})
    if metric in ("peak_jerk", "rms_accel", "peak_decel", "peak_accel"):
        out.append({"title": f"{bind['label']}: worst {bind['worst']:.3g}",
                    "detail": "comfort limit: the law reacts with high-gain "
                              "transients (try a lag on the feedforward)"})
    return out


def _braking(res, gap, closing, i: int, k: int, tree: dict) -> list[dict]:
    """Did car ``i`` lack braking, use too little, or react late (before
    sample ``k``)?"""
    cfg, dt = res.config, res.config.dt
    w0 = max(0, k - int(6.0 / dt))
    seg = slice(w0, k + 1)
    v_front = res.vel[seg, i] - closing[seg]
    grip = (cfg.vehicle.ev or EVParams()).mu * 9.81
    # a cut-in swaps the car in front: clip the one-sample jump to physics
    a_front = np.clip(np.gradient(v_front, dt), -grip, grip)
    lo, _hi = _bounds(res, i)
    sat = np.abs(res.u[seg, i - 1] - lo[seg]) < 1e-3
    asked = float(-res.u[seg, i - 1].min())
    out = []
    # constant-deceleration stopping: to stop closing in a gap g at closing
    # speed c behind a car braking at b_f takes b_f + c^2 / 2g; evaluated
    # when the closing-in began (what a perfect driver would have needed)
    start = np.flatnonzero(closing[seg] > 0.5)
    need = None
    if start.size:
        s0 = int(start[0])
        g0, c0 = max(float(gap[w0 + s0]), 0.1), float(closing[w0 + s0])
        need = c0 * c0 / (2 * g0) + max(0.0, -float(a_front[s0:].min()))
    if sat.mean() > 0.2:
        out.append({"title": f"car {i} ran out of braking",
                    "detail": f"its command sat on the {-lo[seg].max():.1f} m/s² "
                              f"limit (grip μ = {grip / 9.81:.2f}, motor and "
                              f"brakes) {sat.mean():.0%} of the last 6 s"
                              + (f"; the situation needed ≈{need:.1f} m/s²"
                                 if need else "")})
    elif need is not None and asked + 0.5 < need:
        out.append({"title": f"car {i} braked too gently",
                    "detail": f"when it started closing in, stopping needed "
                              f"≈{need:.1f} m/s² (grip allows {grip:.1f}"
                              + ("; even full braking would only have softened "
                                 "the impact" if need > grip else "")
                              + f"); the controller asked for at most {asked:.1f}. "
                              "The law "
                              "has no emergency layer, and its gap feedback "
                              f"(kp = {cfg.control.kp:.3g}) is weak for a sudden "
                              "short gap"})
    brake_front = np.flatnonzero(a_front < -1.0)
    brake_own = np.flatnonzero(res.u[seg, i - 1] < -1.0)
    if brake_front.size and brake_own.size:
        lag = (brake_own[0] - brake_front[0]) * dt
        if lag > 0.3:
            out.append({"title": f"car {i} reacted {lag:.2f} s after the car in "
                                 "front started braking",
                        "detail": _lag_reason(cfg, tree, i)})
    return out


def _lag_reason(cfg, tree, i: int) -> str:
    bits = []
    if cfg.vehicle.act_delay > 0:
        bits.append(f"actuator dead time {cfg.vehicle.act_delay:.2f} s")
    bits.append(f"powertrain lag τ = {cfg.vehicle.tau:.2f} s")
    if (tree.get("traffic") or {}).get("cut_ins") or (
            (tree.get("traffic") or {}).get("humans")):
        bits.append("no V2V from the car ahead (it is not connected)")
    elif cfg.delay > 0:
        bits.append(f"V2V latency {cfg.delay * 1e3:.0f} ms")
    return "the delays in the loop: " + ", ".join(bits)


def _amplification(res):
    if res.err.shape[1] < 2:
        return None
    peak = np.max(np.abs(res.err), axis=0)
    ratio = peak[1:] / np.maximum(peak[:-1], 1e-9)
    return np.where(peak[:-1] > 0.3, ratio, 0.0)  # ignore growth of noise


def _gamma_finding(res, controller: str, options) -> list[dict]:
    from cacc import tfctrl as tc

    cfg = res.config
    try:
        d = design_of(controller, cfg.control, options)
        if d is None:
            return []
        w = np.logspace(-2, 1.5, 1500)
        tau, act = cfg.vehicle.tau, cfg.vehicle.act_delay
        g = np.abs(tc.gamma(w, d, tau, cfg.delay, act))
        g0 = np.abs(tc.gamma(w, d, tau, 0.0, 0.0))
    except (ValueError, ZeroDivisionError):
        return []
    k = int(np.argmax(g))
    if g[k] <= 1.0 + 1e-6:
        return []
    deg = math.degrees(w[k] * (cfg.delay + act))
    return [{"title": f"analytic |Γ| peak {g[k]:.3f} > 1 at ω = {w[k]:.2f} rad/s",
             "detail": f"with zero delays the peak is {g0.max():.3f}; the "
                       f"{(cfg.delay + act) * 1e3:.0f} ms of V2V + actuator delay "
                       f"costs {deg:.0f}° of phase at that frequency, which "
                       "turns damping into amplification"}]


def _condition_findings(res, tree: dict, case: dict) -> list[dict]:
    out = []
    atk = tree.get("attack")
    if atk:
        gated = bool((tree.get("trust") or {}).get("enabled"))
        inj = (float(np.abs(res.ff_inj).max()) if res.ff_inj is not None
               else float(atk.get("value", 0.0)))
        out.append({"title": f"spoofed V2V on link {atk.get('target_link', 1)}: "
                             f"{atk.get('value')} m/s² {atk.get('kind', 'bias')}",
                    "detail": ("the trust gate was on and capped the injected "
                               f"feedforward at {inj:.2f} m/s²" if gated else
                               "nothing checked the message against the radar, "
                               "so the car obeyed a lie about the car ahead")})
    if res.link_stats:
        worst = min(res.link_stats, key=lambda s: s.get("delivered", 1) / max(
            s.get("sent", 1), 1))
        share = worst.get("delivered", 0) / max(worst.get("sent", 1), 1)
        if share < 0.95:
            out.append({"title": f"only {share:.0%} of V2V messages arrived",
                        "detail": "the feedforward was held stale between packets"})
    tr = tree.get("traffic") or {}
    for h in tr.get("humans") or []:
        out.append({"title": f"car {h} is driven by a person",
                    "detail": f"car {h + 1} gets no V2V and runs as a radar-only "
                              "ACC on a headway tuned for CACC"})
    for c in tr.get("cut_ins") or []:
        out.append({"title": f"a car cut in ahead of car {c['ahead_of']} at "
                             f"t = {c['t']:.0f} s",
                    "detail": f"{abs(c.get('dv', 0)):.0f} m/s slower, braking "
                              f"{-c.get('decel', 0):.0f} m/s²; it halves the gap and "
                              "breaks the V2V chain"})
    return out


def fixes(case: dict, controller: str, options: dict | None) -> list[tuple[str, dict]]:
    """Candidate changes as (description, scenario tree)."""
    tree = case["scenario"]
    ctl = tree.get("controller") or {}
    h = float(ctl.get("h", 0.7))
    out = []
    for dh in (0.3, 0.7):
        t = copy.deepcopy(tree)
        t.setdefault("controller", {})["h"] = round(h + dh, 2)
        out.append((f"headway h {h:g} → {h + dh:g} s", t))
    if tree.get("attack") and not (tree.get("trust") or {}).get("enabled"):
        t = copy.deepcopy(tree)
        t["trust"] = {"enabled": True, "r0": 3.0, "radar_noise": 0.05}
        out.append(("turn on the V2V trust gate", t))
    if controller in ("cthp", "acc", "cacc") and float(ctl.get("kp", 0.2)) < 0.15:
        t = copy.deepcopy(tree)
        t["controller"]["kp"] = 0.15
        out.append((f"stronger gap feedback kp {ctl.get('kp')} → 0.15", t))
    if (tree.get("traffic") or {}):
        t = copy.deepcopy(tree)
        t["controller"]["h"] = max(h, 1.4)
        out.append((f"fall back to an ACC headway (h = {max(h, 1.4):g} s) when the "
                    "car ahead is not connected", t))
    if not (tree.get("aeb") or {}).get("enabled"):
        t = copy.deepcopy(tree)
        t["aeb"] = {"enabled": True}
        out.append(("add the emergency-braking layer (AEB)", t))
    return out


def explain(report: dict, case_name: str, controller: str | None = None,
            options: dict | None = None, seed: int | None = None, res=None,
            verify: bool = True, jobs: int = 1) -> dict:
    """Root cause + verified fixes for one case of a stress report."""
    from cacc.envelope import _margin, _run

    case = next((c for c in report["cases"] if c["name"] == case_name), None)
    if case is None:
        raise ValueError(f"no case {case_name!r} in the report")
    controller = controller or report["under_test"]["controller"]
    options = report["under_test"].get("options", {}) if options is None else options
    if seed is None:
        seeds = [c.get("worst_seed") for c in case["checks"] if not c["passed"]]
        seed = next((s for s in seeds if s is not None), report["seeds"][0])
    if case["verdict"] == "ERROR":
        return {"case": case_name, "seed": seed, "verdict": "ERROR",
                "headline": "the run crashed: " + (case["errors"][0]["error"]
                                                   .strip().splitlines()[-1]
                                                   if case["errors"] else "error"),
                "findings": [], "fixes": []}
    res = res if res is not None else replay(case, controller, seed, options)
    found = diagnose(res, case, controller, options)
    out = {"case": case_name, "seed": seed, "verdict": case["verdict"],
           "headline": found[0]["title"] if found else "no failure in this cell",
           "findings": found, "fixes": []}
    if verify and case["verdict"] == "FAIL":
        cands = fixes(case, controller, options)
        if cands:
            cases = _run([(d, t) for d, t in cands], controller, options,
                         _criteria(report), [seed], jobs)
            for (d, _t), c in zip(cands, cases, strict=True):
                out["fixes"].append({"change": d, "verdict": c["verdict"],
                                     "margin": _margin(c)})
            # root-cause fixes first; the AEB safety net only after them
            out["fixes"].sort(key=lambda f: (f["verdict"] != "PASS",
                                             "AEB" in f["change"], -f["margin"]))
    return out


def format_text(ex: dict) -> str:
    lines = [f"{ex['case']} (seed {ex['seed']}): {ex['verdict']}",
             f"  {ex['headline']}", ""]
    for f in ex["findings"][1:] if ex["findings"] else []:
        lines.append(f"  • {f['title']}\n      {f['detail']}")
    if ex["findings"]:
        lines.insert(2, f"      {ex['findings'][0]['detail']}")
    if ex["fixes"]:
        lines += ["", "  Tried on the same seed:"]
        for f in ex["fixes"]:
            m = f["margin"]
            mtxt = f"{m:+.0%}" if math.isfinite(m) else ""
            lines.append(f"    {'✓' if f['verdict'] == 'PASS' else '✕'} "
                         f"{f['change']}  {mtxt}")
    return "\n".join(lines)
