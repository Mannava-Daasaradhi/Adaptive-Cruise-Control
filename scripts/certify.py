"""Monte-Carlo certification harness (D-018): the product face.

One command runs the certification suite — a matrix of scenarios x seeds —
and emits a **self-contained HTML report** with PASS/FAIL verdicts against
explicit acceptance criteria, plus machine-readable metrics. This is the
"replace the hardware testbed" artifact: seeded, reproducible, theory-
instrumented evidence that a controller configuration is (or is not) safe
to deploy.

Suites (each check states its criterion in the report):

* ``base-case-a``     — the reproduced Ma-2025 design under its own noise.
* ``qos-zone``        — QoS-adaptive platoon through a rho 10->3->10 zone.
* ``deep-zone-gains`` — joint gain+headway adaptation through rho = 2.
* ``delay-predictor`` — theta = 0.15 s with the timestamp predictor.

Run (conda env `cacc`):

    python scripts/certify.py [--seeds 5] [--quick]

Output: results/certification/<stamp>/report.html + metrics.json
"""

from __future__ import annotations

import argparse
import base64
import dataclasses
import io
import json
import logging
from datetime import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from cacc import (AdaptConfig, PlatoonSim, hinf_norm, load_scenario,
                  setup_logging)

log = logging.getLogger("cacc.scripts.certify")

KA, KP, KV, TAU0 = 0.5, 0.009, 0.63, 0.5


# ------------------------------------------------------------------ checks
def check(name: str, criterion: str, value: float, ok: bool) -> dict:
    return {"check": name, "criterion": criterion,
            "value": round(float(value), 4), "pass": bool(ok)}


def l2_ratios(res, lo, hi):
    m = (res.t >= lo) & (res.t <= hi)
    norms = np.sqrt(np.trapezoid(res.err[m] ** 2, res.t[m], axis=0))
    norms = np.where(norms < 1e-9, 1e-9, norms)
    return norms[1:] / norms[:-1]


def min_gap(res):
    gaps = np.diff(-res.pos, axis=1) - res.config.vehicle.length
    return float(gaps.min())


def hinf_worst_at(h, rho, kp=KP, kv=KV):
    return max(hinf_norm("cthp", h, kp=kp, tau=TAU0, kv=kv, ka_eff=e * KA)
               for e in (1 - 1 / rho, 1 + 1 / rho))


# ------------------------------------------------------------------ suites
def suite_base_case_a(sc_qos, seeds, quick):
    """Reproduced case A under its designed channel (rho = 5)."""
    sc = load_scenario("scenarios/ma2025_sine.yaml")
    cfg = dataclasses.replace(sc.config,
                              n_followers=4 if quick else 8,
                              t_final=120.0 if quick else 200.0)
    worst_ratio, gaps = -np.inf, []
    for seed in seeds:
        res = PlatoonSim(dataclasses.replace(cfg, seed=seed), "cthp",
                         sc.leader).run()
        worst_ratio = max(worst_ratio, float(l2_ratios(
            res, 15.0, cfg.t_final - 5).max()))
        gaps.append(min_gap(res))
    hinf = hinf_worst_at(0.95, 5.0)
    return [
        check("theory: ||H~||inf(h=0.95, rho=5) <= 1", "<= 1.0", hinf,
              hinf <= 1.0 + 1e-6),
        check("per-hop L2 within noise envelope (all seeds)", "<= 1.05",
              worst_ratio, worst_ratio <= 1.05),
        check("min bumper gap (all seeds)", ">= 5 m", min(gaps),
              min(gaps) >= 5.0),
    ], res


def suite_qos_zone(sc, seeds, quick):
    """QoS-adaptive platoon through the rho 10->3->10 zone."""
    cfg = sc.config
    if quick:
        cfg = dataclasses.replace(cfg, n_followers=4)
    hinf_max, track_err, gaps = -np.inf, [], []
    for seed in seeds:
        res = PlatoonSim(dataclasses.replace(cfg, seed=seed), "cthp",
                         sc.leader).run()
        for t_probe, rho_true in ((70.0, 10.0), (165.0, 3.0), (235.0, 10.0)):
            k = int(np.searchsorted(res.t, t_probe))
            hinf_max = max(hinf_max, hinf_worst_at(
                float(res.h[k].mean()), rho_true))
        k_zone = int(np.searchsorted(res.t, 165.0))
        rho_med = float(np.nanmedian(res.rho_hat[k_zone - 500:k_zone]))
        track_err.append(abs(rho_med - 3.0) / 3.0)
        gaps.append(min_gap(res))
    return [
        check("||H~||inf of in-force h, all phases/seeds", "<= 1.0 + 1e-4",
              hinf_max, hinf_max <= 1.0 + 1e-4),
        check("zone rho tracking error (median)", "<= 30 %",
              max(track_err), max(track_err) <= 0.30),
        check("min bumper gap (all seeds)", ">= 5 m", min(gaps),
              min(gaps) >= 5.0),
    ], res


def suite_deep_zone_gains(sc, seeds, quick):
    """Joint gain + headway adaptation through rho = 2 (D-017)."""
    cfg = dataclasses.replace(
        sc.config,
        n_followers=4 if quick else sc.config.n_followers,
        rho_schedule=((0.0, 10.0), (80.0, 2.0), (175.0, 10.0)),
        adapt=dataclasses.replace(sc.config.adapt, enabled=True,
                                  adapt_gains=True))
    hinf_max, gaps = -np.inf, []
    for seed in seeds:
        res = PlatoonSim(dataclasses.replace(cfg, seed=seed), "cthp",
                         sc.leader).run()
        k = int(np.searchsorted(res.t, 165.0))
        kp_f = float(res.gains[k, :, 0].mean())
        kv_f = float(res.gains[k, :, 1].mean())
        hinf_max = max(hinf_max, hinf_worst_at(
            float(res.h[k].mean()), 2.0, kp=kp_f, kv=kv_f))
        gaps.append(min_gap(res))
    return [
        check("||H~||inf in force inside rho=2 zone", "<= 1.0 + 1e-4",
              hinf_max, hinf_max <= 1.0 + 1e-4),
        check("min bumper gap (all seeds)", ">= 5 m", min(gaps),
              min(gaps) >= 5.0),
    ], res


def suite_delay_predictor(sc, seeds, quick):
    """theta = 0.15 s, predictor on, worst-frequency excitation."""
    from cacc import make_leader_profile
    leader = make_leader_profile({
        "profile": "bursts", "bursts": [[15.0, 97.0, 0.4, 0.07235]],
        "probe_amplitude": 0.0})
    cfg = dataclasses.replace(
        sc.config, n_followers=4 if quick else 8,
        adapt=dataclasses.replace(sc.config.adapt, enabled=False,
                                  predictor=True),
        delay=0.15, noise_rho=5.0, rho_schedule=None, t_final=150.0)
    worst_ratio, gaps = -np.inf, []
    for seed in seeds:
        res = PlatoonSim(dataclasses.replace(cfg, seed=seed), "cthp",
                         leader).run()
        worst_ratio = max(worst_ratio,
                          float(l2_ratios(res, 15.0, 145.0).max()))
        gaps.append(min_gap(res))
    hinf = max(hinf_norm("cthp", 0.95, kp=KP, tau=TAU0, kv=KV,
                         ka_eff=e * KA, theta=0.15, pred_theta_hat=0.15)
               for e in (0.8, 1.2))
    return [
        check("theory: compensated ||H~||inf at theta=0.15", "<= 1.0 + 1e-3",
              hinf, hinf <= 1.0 + 1e-3),
        check("per-hop L2 (all seeds)", "<= 1.05", worst_ratio,
              worst_ratio <= 1.05),
        check("min bumper gap (all seeds)", ">= 5 m", min(gaps),
              min(gaps) >= 5.0),
    ], res


# ------------------------------------------------------------------ report
def snapshot_png(res) -> str:
    """Small errors-vs-time thumbnail, base64-embedded."""
    fig, ax = plt.subplots(figsize=(6.4, 2.6))
    n = res.err.shape[1]
    cmap = plt.cm.viridis(np.linspace(0, 0.9, n))
    for i in range(n):
        ax.plot(res.t, -res.err[:, i], color=cmap[i], lw=0.7)
    ax.set_xlabel("t [s]", fontsize=8)
    ax.set_ylabel(r"$\delta_i$ [m]", fontsize=8)
    ax.tick_params(labelsize=7)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=100)
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode()


def render_html(results: dict, seeds: list[int], outdir: Path) -> Path:
    rows_css = ("body{font-family:Segoe UI,Arial,sans-serif;background:#14161d;"
                "color:#e8e8ee;margin:2em auto;max-width:70em;padding:0 1em}"
                "h1{font-size:1.5em} h2{font-size:1.15em;margin-top:1.6em}"
                "table{border-collapse:collapse;width:100%;margin:.6em 0}"
                "td,th{border:1px solid #333a4d;padding:.35em .6em;"
                "font-size:.9em;text-align:left}"
                ".pass{color:#4caf50;font-weight:600}"
                ".fail{color:#ef5350;font-weight:600}"
                "img{max-width:100%;border:1px solid #333a4d;margin:.4em 0}"
                ".meta{color:#9aa0b4;font-size:.85em}")
    total = sum(len(v["checks"]) for v in results.values())
    passed = sum(c["pass"] for v in results.values() for c in v["checks"])
    verdict = ("CERTIFIED" if passed == total
               else f"FAILED ({total - passed} check(s))")
    parts = [f"<style>{rows_css}</style>",
             "<h1>CACC Virtual Validation — Certification Report</h1>",
             f"<p class='meta'>generated {datetime.now():%Y-%m-%d %H:%M} · "
             f"seeds {seeds} · cacc package v0.3 · 55-test suite</p>",
             f"<h2>Overall: <span class="
             f"'{'pass' if passed == total else 'fail'}'>{verdict}"
             f"</span> — {passed}/{total} checks</h2>"]
    for name, v in results.items():
        parts.append(f"<h2>{name}</h2><p class='meta'>{v['desc']}</p>")
        parts.append("<table><tr><th>check</th><th>criterion</th>"
                     "<th>value</th><th>verdict</th></tr>")
        for c in v["checks"]:
            cls = "pass" if c["pass"] else "fail"
            parts.append(
                f"<tr><td>{c['check']}</td><td>{c['criterion']}</td>"
                f"<td>{c['value']}</td>"
                f"<td class='{cls}'>{'PASS' if c['pass'] else 'FAIL'}</td></tr>")
        parts.append("</table>")
        parts.append(f"<img src='data:image/png;base64,{v['png']}'/>")
    out = outdir / "report.html"
    out.write_text("\n".join(parts), encoding="utf-8")
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, default=5)
    parser.add_argument("--quick", action="store_true")
    args = parser.parse_args()
    setup_logging()
    seeds = list(range(1, args.seeds + 1))
    sc = load_scenario("scenarios/qos_adaptive.yaml")

    suites = {
        "base-case-a — reproduced Ma-2025 design under its own channel":
            suite_base_case_a,
        "qos-zone — adaptive headway through rho 10→3→10":
            suite_qos_zone,
        "deep-zone-gains — joint gain+h adaptation through rho = 2":
            suite_deep_zone_gains,
        "delay-predictor — theta = 0.15 s with timestamp compensation":
            suite_delay_predictor,
    }
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    outdir = Path("results/certification") / stamp
    outdir.mkdir(parents=True, exist_ok=True)

    results = {}
    for name, fn in suites.items():
        log.info("=== suite: %s", name)
        checks, last_res = fn(sc, seeds, args.quick)
        results[name] = {"desc": fn.__doc__.strip(), "checks": checks,
                         "png": snapshot_png(last_res)}
    report = render_html(results, seeds, outdir)
    (outdir / "metrics.json").write_text(json.dumps(
        {k: {"checks": v["checks"]} for k, v in results.items()}, indent=2),
        encoding="utf-8")
    n_fail = sum((not c["pass"]) for v in results.values()
                 for c in v["checks"])
    log.info("certification %s -> %s", "PASSED" if n_fail == 0 else
             f"FAILED ({n_fail})", report)


if __name__ == "__main__":
    main()
