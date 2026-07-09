"""QoS-adaptive CACC study (D-016): estimation + prediction + adaptation.

Produces the evidence for doc 09:

fig1 — theory: fixed-gain required headway h_req(rho) vs the paper's
       re-tuned bound h_lb(ka, rho); the h-only feasibility boundary rho*;
       h_req(theta) with and without the timestamp predictor.
fig2 — the interference-zone experiment: three platoons {fixed-good,
       fixed-worst, adaptive} through rho 10 -> 3 -> 10 with maneuvers in
       and out of the zone; rho-hat tracking; h(t); spacing errors.
fig3 — delayed-channel experiment: fixed h = 0.95 s at theta = 0.15 s with
       and without the feedforward predictor.
fig4 — deep zone (rho 10 -> 2 -> 10): the fixed-gain wall and the online
       gain re-tuning fix (D-017), with the live in-force ||H~||_inf.
fig5 — adaptation smoothing & staggered recovery (D-019): asymmetric
       rho-hat filtering + front-first serialized h ramps vs the plain
       D-016 adaptive platoon in the same interference zone.
metrics.json — per-phase L2 amplification, headway/throughput accounting,
       estimator tracking errors, predictor gains.

Run (conda env `cacc`):  python scripts/qos_adaptive_study.py [--quick]
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import logging
from datetime import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from cacc import (PlatoonSim, cthp_h_lb, hinf_norm, load_scenario,
                  make_leader_profile, min_stable_headway, setup_logging)
from cacc.plotstyle import C, use_house_style, vehicle_colors, zone_span

log = logging.getLogger("cacc.scripts.qos_adaptive_study")

KA, KP, KV, TAU0 = 0.5, 0.009, 0.63, 0.5
PHASES = {"good-1": (15.0, 78.0), "zone": (100.0, 163.0),
          "good-2": (185.0, 235.0)}
PHASE_RHO = {"good-1": 10.0, "zone": 3.0, "good-2": 10.0}


def hinf_worst(h: float, rho: float, theta: float = 0.0,
               pred: bool = False) -> float:
    """||H~||_inf, worst case over both noise-interval ends."""
    return max(hinf_norm("cthp", h, kp=KP, tau=TAU0, kv=KV,
                         ka_eff=end * KA, theta=theta,
                         pred_theta_hat=(theta if pred else 0.0))
               for end in (1.0 - 1.0 / rho, 1.0 + 1.0 / rho))


def h_req_fixed_gain(rho: float, theta: float = 0.0, pred: bool = False) -> float:
    """Worst-case (both noise-interval ends) fixed-gain required headway."""
    h = 0.0
    for end in (1.0 - 1.0 / rho, 1.0 + 1.0 / rho):
        try:
            h = max(h, min_stable_headway(
                "cthp", theta=theta, kp=KP, tau=TAU0, kv=KV,
                ka_eff=end * KA, pred_theta_hat=(theta if pred else 0.0)))
        except ValueError:
            return np.inf
    return h


def l2_per_phase(res, lo: float, hi: float) -> np.ndarray:
    """Per-hop L2 amplification ratios within a time window."""
    m = (res.t >= lo) & (res.t <= hi)
    norms = np.sqrt(np.trapezoid(res.err[m] ** 2, res.t[m], axis=0))
    norms = np.where(norms < 1e-9, 1e-9, norms)
    return norms[1:] / norms[:-1]


# ------------------------------------------------------------------ figures
def fig1_theory(outdir: Path) -> dict:
    rhos = np.array([1.8, 2.0, 2.2, 2.5, 2.8, 3.0, 3.5, 4.0, 5.0, 7.0, 10.0,
                     15.0, 25.0])
    h_fixed = np.array([h_req_fixed_gain(r) for r in rhos])
    h_retuned = np.array([cthp_h_lb(KA, r, TAU0) for r in rhos])
    feasible = np.isfinite(h_fixed)
    rho_star = float(rhos[feasible].min())

    thetas = np.linspace(0.0, 0.30, 13)
    h_nopred = [h_req_fixed_gain(5.0, th, pred=False) for th in thetas]
    h_pred = [h_req_fixed_gain(5.0, th, pred=True) for th in thetas]

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    ax = axes[0]
    ax.plot(rhos[feasible], h_fixed[feasible], "o-", label="fixed case-A gains")
    ax.plot(rhos, h_retuned, "s--", label=r"re-tuned bound $h_{lb}$ (eq. 17)")
    ax.axvline(rho_star, color="r", ls=":",
               label=rf"h-only feasibility ends, $\rho^*\approx${rho_star:.1f}")
    ax.set_xscale("log")
    ax.set_xlabel(r"channel quality $\rho$")
    ax.set_ylabel(r"required headway $h_{req}$ [s]")
    ax.set_title("adaptation target vs channel quality")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[1]
    ax.plot(thetas, h_nopred, "o-", label="no compensation")
    ax.plot(thetas, h_pred, "s-", label="timestamp predictor")
    ax.axhline(h_req_fixed_gain(5.0), color="k", ls=":", lw=1,
               label=r"$\theta=0$ requirement")
    ax.set_xlabel(r"V2V delay $\theta$ [s]")
    ax.set_ylabel(r"$h_{req}$ [s]  ($\rho=5$)")
    ax.set_title("feedforward delay: predictor restores the budget")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(outdir / "fig1_theory.png", dpi=150)
    plt.close(fig)
    return {
        "rho_star_fixed_gain_feasibility": rho_star,
        "h_req_fixed_vs_retuned": {
            str(r): [round(float(hf), 4) if np.isfinite(hf) else None,
                     round(float(hr), 4)]
            for r, hf, hr in zip(rhos, h_fixed, h_retuned)},
        "h_req_theta_rho5": {
            str(round(float(th), 2)): [round(float(a), 4), round(float(b), 4)]
            for th, a, b in zip(thetas, h_nopred, h_pred)},
    }


def run_variant(sc, label: str, adapt_on: bool, h0: float):
    cfg = sc.config
    ctrl = dataclasses.replace(cfg.control, h=h0)
    adapt = dataclasses.replace(cfg.adapt, enabled=adapt_on)
    cfg = dataclasses.replace(cfg, control=ctrl, adapt=adapt)
    log.info("--- variant %s: h0=%.3f adapt=%s", label, h0, adapt_on)
    return PlatoonSim(cfg, "cthp", sc.leader).run()


def fig2_zone_experiment(sc, outdir: Path, quick: bool) -> dict:
    h_worst = h_req_fixed_gain(3.0) + 0.08
    variants = {
        "fixed-good (h=0.95 s)": run_variant(sc, "fixed-good", False, 0.95),
        f"fixed-worst (h={h_worst:.2f} s)":
            run_variant(sc, "fixed-worst", False, h_worst),
        "adaptive": run_variant(sc, "adaptive", True, 0.95),
    }
    colors = {k: c for k, c in zip(
        variants, (C["fixed_good"], C["fixed_worst"], C["adaptive"]))}

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    ax = axes[0]
    res_a = variants["adaptive"]
    ax.plot(res_a.t, res_a.rho_hat[:, 0], color=C["adaptive"], lw=1.4,
            label=r"estimated $\hat\rho$ (veh 1)")
    sched = list(sc.config.rho_schedule) + [(sc.config.t_final, None)]
    for (t0, r0), (t1, _) in zip(sched[:-1], sched[1:]):
        ax.hlines(r0, t0, t1, color=C["reference"], ls="--", lw=1.3,
                  label="true $\\rho$" if t0 == sched[0][0] else None)
    ax.set_ylabel(r"channel quality $\rho$")
    ax.set_title("Online channel-quality estimation tracks the true schedule")
    ax.legend(loc="center right")

    ax = axes[1]
    ax.plot(res_a.t, res_a.h[:, 0], color=C["adaptive"], lw=1.9,
            label="adaptive $h(t)$")
    ax.axhline(0.95, color=C["fixed_good"], ls="--", lw=1.4, label="fixed-good")
    ax.axhline(h_worst, color=C["fixed_worst"], ls="--", lw=1.4,
               label="fixed-worst")
    ax.set_ylabel("time headway $h$ [s]")
    ax.set_title("Headway opens in the zone, then recovers — capacity only when safe")
    ax.legend(loc="center right")

    ax = axes[2]
    for name, res in variants.items():
        last = res.err[:, -1]
        ax.plot(res.t, -last, color=colors[name], lw=1.3, label=name)
    ax.set_ylabel(rf"last-follower error $\delta_{{{res_a.err.shape[1]}}}$ [m]")
    ax.set_xlabel("time t [s]")
    ax.set_title("Spacing error stays bounded across the degraded window")
    ax.legend(loc="upper left")
    z0, z1 = sc.config.rho_schedule[1][0], sc.config.rho_schedule[2][0]
    for i, a in enumerate(axes):
        zone_span(a, z0, z1, label="interference zone  (ρ = 3)", first=(i == 0))
    fig.suptitle("Interference-zone experiment — ρ:  10 → 3 → 10   (case-A gains)")
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(outdir / "fig2_zone_experiment.png", dpi=150)
    plt.close(fig)

    out: dict = {"h_worst_case_design": round(float(h_worst), 4)}
    for name, res in variants.items():
        entry: dict = {}
        for ph, (lo, hi) in PHASES.items():
            r = l2_per_phase(res, lo, hi)
            # frequency-domain verdict at the h actually in force late in
            # the phase (the decisive metric — the boundary is flat, so
            # time-domain growth is slow; see D-010/D-016)
            k_ph = int(np.searchsorted(res.t, hi - 5.0))
            h_ph = float(res.h[k_ph].mean()) if res.h is not None \
                else float(res.config.control.h)
            entry[ph] = {"l2_ratios": [round(float(x), 4) for x in r],
                         "max_ratio": round(float(r.max()), 4),
                         "h_in_force": round(h_ph, 3),
                         "hinf_worst": round(hinf_worst(h_ph, PHASE_RHO[ph]), 5)}
        gaps = np.diff(-res.pos, axis=1) - sc.config.vehicle.length
        entry["min_gap_m"] = round(float(gaps.min()), 3)
        entry["mean_headway_s"] = round(
            float(res.h.mean()) if res.h is not None
            else float(res.config.control.h), 4)
        out[name] = entry
    # estimator tracking quality (adaptive variant, all links)
    err_track = []
    for t0, t1, rho_true in ((20.0, 78.0, 10.0), (110.0, 173.0, 3.0),
                             (200.0, 238.0, 10.0)):
        m = (res_a.t >= t0) & (res_a.t <= t1)
        est = res_a.rho_hat[m]
        err_track.append({
            "window": [t0, t1], "rho_true": rho_true,
            "median_rho_hat": round(float(np.nanmedian(est)), 3)})
    out["estimator_tracking"] = err_track
    return out


def fig3_delay_experiment(sc, outdir: Path) -> dict:
    """Fixed h = 0.95, theta = 0.15 s, constant rho = 5: predictor on/off.

    The leader excites the uncompensated design's worst frequency
    (omega* ~ 0.45 rad/s, where ||H~||_inf = 1.009 vs 1.000 compensated).
    """
    base = sc.config
    ctrl = dataclasses.replace(base.control, h=0.95)
    leader = make_leader_profile({
        "profile": "bursts", "bursts": [[15.0, 97.0, 0.4, 0.07235]],
        "probe_amplitude": 0.0})
    runs = {}
    for noisy in (False, True):
        for pred in (False, True):
            adapt = dataclasses.replace(base.adapt, enabled=False,
                                        predictor=pred)
            cfg = dataclasses.replace(
                base, control=ctrl, adapt=adapt, delay=0.15,
                noise_rho=(5.0 if noisy else None),
                rho_schedule=None, t_final=150.0)
            name = (("noisy" if noisy else "noiseless") + ", "
                    + ("predictor" if pred else "uncompensated"))
            runs[name] = PlatoonSim(cfg, "cthp", leader).run()

    fig, axes = plt.subplots(2, 2, figsize=(11, 7.5), sharey="row",
                             sharex=True)
    for ax, (name, res) in zip(axes.flat, runs.items()):
        n = res.err.shape[1]
        cmap = vehicle_colors(n)
        for i in range(n):
            ax.plot(res.t, -res.err[:, i], color=cmap[i], lw=1.0)
        r = l2_per_phase(res, 15.0, 145.0)
        rho_th = 5.0 if name.startswith("noisy") else 1e9
        hinf = hinf_worst(0.95, rho_th, theta=0.15,
                          pred=name.endswith("predictor"))
        ax.set_title(f"{name}: max per-hop L2 = {r.max():.3f}, "
                     rf"$\|\tilde H\|_\infty$ = {hinf:.4f}", fontsize=10)
        ax.grid(alpha=0.3)
    for ax in axes[1]:
        ax.set_xlabel("t [s]")
    for ax in axes[:, 0]:
        ax.set_ylabel(r"$\delta_i$ [m]")
    fig.suptitle(r"$\theta$ = 0.15 s, h = 0.95 s, leader at "
                 r"$\omega^*\!=0.45$ rad/s — timestamp predictor restores "
                 "string stability")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(outdir / "fig3_delay_predictor.png", dpi=150)
    plt.close(fig)
    return {name: {
        "max_l2_ratio": round(float(l2_per_phase(r, 15.0, 145.0).max()), 4),
        "hinf_worst": round(hinf_worst(
            0.95, 5.0 if name.startswith("noisy") else 1e9, 0.15,
            pred=name.endswith("predictor")), 5)}
        for name, r in runs.items()}


def fig4_gain_retuning(sc, outdir: Path) -> dict:
    """Deep interference zone (rho 10 -> 2 -> 10): h-only adaptation hits
    the fixed-gain wall (unstable even at h_max); joint gain + headway
    re-tuning (D-017) restores the margin. The middle panel shows the
    ||H~||_inf of the configuration actually in force over time."""
    base = sc.config
    deep = dataclasses.replace(
        base, rho_schedule=((0.0, 10.0), (80.0, 2.0), (175.0, 10.0)))
    runs = {}
    for gains_on in (False, True):
        adapt = dataclasses.replace(deep.adapt, enabled=True,
                                    adapt_gains=gains_on)
        cfg = dataclasses.replace(deep, adapt=adapt)
        name = "joint gains+h" if gains_on else "h-only"
        log.info("--- deep-zone variant: %s", name)
        runs[name] = PlatoonSim(cfg, "cthp", sc.leader).run()

    def hinf_in_force(res, t_probe, rho_true):
        k = int(np.searchsorted(res.t, t_probe))
        h = float(res.h[k].mean())
        kp, kv = ((float(res.gains[k, :, 0].mean()),
                   float(res.gains[k, :, 1].mean()))
                  if res.gains is not None else (KP, KV))
        return max(hinf_norm("cthp", h, kp=kp, tau=TAU0, kv=kv,
                             ka_eff=end * KA)
                   for end in (1 - 1 / rho_true, 1 + 1 / rho_true))

    probes = np.arange(4.0, deep.t_final - 2.0, 4.0)
    rho_of = lambda tt: 2.0 if 80.0 <= tt < 175.0 else 10.0  # noqa: E731

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    colors = {"h-only": C["fail"], "joint gains+h": C["fix"]}
    ax = axes[0]
    for name, res in runs.items():
        ax.plot(res.t, res.h[:, 0], color=colors[name], lw=1.3,
                label=f"{name}: h(t)")
        if res.gains is not None:
            ax.plot(res.t, res.gains[:, 0, 1], color=colors[name], lw=1.1,
                    ls="--", label=f"{name}: kv(t)")
    ax.axhline(0.63, color="0.5", ls=":", lw=1, label="case-A kv")
    ax.set_ylabel("h [s] / kv")
    ax.set_title("headway and re-tuned gain (veh 1)")
    ax.legend(fontsize=8, ncol=2)
    ax.grid(alpha=0.3)

    ax = axes[1]
    curves = {}
    for name, res in runs.items():
        vals = [hinf_in_force(res, tp, rho_of(tp)) for tp in probes]
        curves[name] = vals
        ax.plot(probes, vals, "o-", ms=3, color=colors[name], label=name)
    ax.axhline(1.0, color="k", ls=":", lw=1)
    ax.set_ylabel(r"$\|\tilde H\|_\infty$ in force")
    ax.set_title("live string-stability margin (worst noise end at true rho)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[2]
    for name, res in runs.items():
        ax.plot(res.t, -res.err[:, -1], color=colors[name], lw=0.9, label=name)
    ax.set_ylabel(r"$\delta_{last}$ [m]")
    ax.set_xlabel("t [s]")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    for i, a in enumerate(axes):
        zone_span(a, 80.0, 175.0, label="deep zone  (ρ = 2)", first=(i == 0))
    fig.suptitle("Deep zone — ρ:  10 → 2 → 10:  the fixed-gain wall and the "
                 "gain re-tuning fix (D-017)")
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(outdir / "fig4_gain_retuning.png", dpi=150)
    plt.close(fig)

    zone_probes = [tp for tp in probes if 110.0 <= tp <= 170.0]
    return {name: {
        "zone_hinf_in_force_max": round(float(max(
            hinf_in_force(res, tp, 2.0) for tp in zone_probes)), 5),
        "zone_h_settled": round(float(res.h[int(160 / deep.dt), 0]), 3),
        "zone_kv_settled": (round(float(res.gains[int(160 / deep.dt), 0, 1]), 3)
                            if res.gains is not None else 0.63)}
        for name, res in runs.items()}


def fig5_smoothing_stagger(sc, outdir: Path) -> dict:
    """Adaptation smoothing & staggered recovery (D-019) vs plain D-016.

    Same interference zone as fig2 (rho 10 -> 3 -> 10). The D-019 arm adds
    (i) the asymmetric rho-hat low-pass (drops instant, rises filtered) and
    (ii) front-first serialization of the h-recovery ramps. Reported:
    plateau chatter (detrended RMS) and drift, exit-transition per-hop L2,
    recovery-start serialization, mean headway (the capacity price), and
    the in-force ||H~||_inf late in the zone (safety must be unaffected)."""
    LPF_TAU, STAGGER = 4.0, 2.0
    runs = {}
    for d019 in (False, True):
        adapt = dataclasses.replace(
            sc.config.adapt, enabled=True,
            rho_lpf_tau=(LPF_TAU if d019 else 0.0),
            stagger_s=(STAGGER if d019 else 0.0))
        cfg = dataclasses.replace(
            sc.config, control=dataclasses.replace(sc.config.control, h=0.95),
            adapt=adapt)
        name = "adaptive + smoothing/stagger" if d019 else "adaptive (plain)"
        log.info("--- fig5 variant: %s", name)
        runs[name] = PlatoonSim(cfg, "cthp", sc.leader).run()

    def plateau_stats(res, lo=125.0, hi=170.0):
        m = (res.t >= lo) & (res.t <= hi)
        h1 = res.h[m][::100]  # 1 s samples, all followers: (W, n)
        trend = np.apply_along_axis(
            lambda v: np.convolve(v, np.ones(5) / 5, "valid"), 0, h1)
        resid = h1[2:-2] - trend
        return (float(np.sqrt((resid ** 2).mean(axis=0)).max()),
                float((h1.max(axis=0) - h1.min(axis=0)).max()))

    def arrivals(res, t_from=175.0, tol=0.02):
        """Per-follower time of arrival at the post-zone headway.

        NOTE per-hop L2 ratios are meaningless across a *serialized*
        transition (each hop's window energy is dominated by its own
        commanded ramp at a different time — the ratio measures timing
        offsets, not amplification; D-010 caveat squared), so the wave
        metrics here are the peak tail excursion and the arrival spread.
        """
        k0 = int(np.searchsorted(res.t, t_from))
        h_final = res.h[-1]
        out = []
        for i in range(res.h.shape[1]):
            k = int(np.argmax(np.abs(res.h[k0:, i] - h_final[i]) < tol))
            out.append(round(float(t_from + res.t[k]), 2))
        return out

    out: dict = {"config": {"rho_lpf_tau": LPF_TAU, "stagger_s": STAGGER}}
    for name, res in runs.items():
        chatter, drift = plateau_stats(res)
        k170 = int(np.searchsorted(res.t, 170.0))
        m_exit = (res.t >= 175.0) & (res.t <= 230.0)
        arr = arrivals(res)
        out[name] = {
            "plateau_chatter_rms_s": round(chatter, 4),
            "plateau_drift_s": round(drift, 4),
            "exit_peak_tail_m": round(
                float(np.abs(res.err[m_exit, -1]).max()), 3),
            "exit_arrival_spread_s": round(max(arr) - min(arr), 2),
            "arrivals_s": arr,
            "mean_headway_s": round(float(res.h.mean()), 4),
            "zone_hinf_in_force": round(
                hinf_worst(float(res.h[k170].min()), 3.0), 5),
        }

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for ax, (name, res) in zip(axes[:2], runs.items()):
        n = res.h.shape[1]
        cmap = vehicle_colors(n)
        for i in range(n):
            ax.plot(res.t, res.h[:, i], color=cmap[i], lw=1.1,
                    label=f"veh {i + 1}" if i in (0, n - 1) else None)
        ax.set_ylabel("h [s]")
        st = out[name]
        ax.set_title(f"{name}: plateau chatter {st['plateau_chatter_rms_s']}"
                     f" s RMS, exit peak tail {st['exit_peak_tail_m']} m",
                     fontsize=10)
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3)
    ax = axes[2]
    for name, res, col in zip(runs, runs.values(),
                              (C["plain"], C["smooth"])):
        ax.plot(res.t, -res.err[:, -1], color=col, lw=1.3, label=name)
    ax.set_ylabel(r"$\delta_{last}$ [m]")
    ax.set_xlabel("time t [s]")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    z0, z1 = sc.config.rho_schedule[1][0], sc.config.rho_schedule[2][0]
    for i, a in enumerate(axes):
        zone_span(a, z0, z1, label="interference zone", first=(i == 0))
    fig.suptitle("Adaptation smoothing & front-first staggered recovery "
                 "(D-019) — same zone as fig 2")
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(outdir / "fig5_smoothing_stagger.png", dpi=150)
    plt.close(fig)
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--quick", action="store_true",
                        help="3 followers, shorter horizon")
    args = parser.parse_args()
    setup_logging()
    use_house_style()

    sc = load_scenario("scenarios/qos_adaptive.yaml")
    if args.quick:
        sc = dataclasses.replace(
            sc, config=dataclasses.replace(sc.config, n_followers=3))

    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    outdir = Path("results/qos_adaptive") / stamp
    outdir.mkdir(parents=True, exist_ok=True)

    metrics = {
        "theory": fig1_theory(outdir),
        "zone_experiment": fig2_zone_experiment(sc, outdir, args.quick),
        "delay_experiment": fig3_delay_experiment(sc, outdir),
        "gain_retuning": fig4_gain_retuning(sc, outdir),
        "smoothing_stagger": fig5_smoothing_stagger(sc, outdir),
    }
    (outdir / "metrics.json").write_text(
        json.dumps(metrics, indent=2), encoding="utf-8")
    log.info("study complete -> %s", outdir)


if __name__ == "__main__":
    main()
