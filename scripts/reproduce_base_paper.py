"""Reproduce Ma, Pagilla & Darbha 2025 (IEEE T-ITS 26(1)) and extend it.

Reproduced from the paper's Section IV (tau0 = 0.5 s, d = 5 m, N = 12,
rho = 5, leader = one 0.5 m/s^2 sine cycle at 0.1 rad/s):

    fig1  design space: h_lb(ka) for several rho + optimal points (Thm 3.2)
    fig2  |H~(jw)| for case A (h=0.95 s, stable) / case B (h=0.65 s, unstable)
          across the noise interval                        [paper Figs. 6, 9]
    fig3  time-domain spacing errors, cases A and B, and the noisy
          communicated acceleration on link 1->2           [paper Figs. 7, 8, 10]
    fig4  optimal design case C (ka*, h=0.88 s) + platoon-length comparison
                                                           [paper Figs. 12, 13]

Extension beyond the paper (the project's novelty — V2V delay & packet loss):

    fig5  minimum string-stable headway vs V2V delay theta (noiseless /
          noisy channel ends), plus an empirical packet-loss study
          (10 Hz beacons, Bernoulli loss) written into metrics.json

Usage (from the project root, inside the `cacc` conda env):

    python scripts/reproduce_base_paper.py [--outdir results] [--quick]
"""

from __future__ import annotations

import argparse
import json
import logging
from datetime import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from cacc import (
    ControllerParams,
    PlatoonConfig,
    PlatoonSim,
    VehicleParams,
    cthp_gains_feasible,
    cthp_h_lb,
    cthp_optimal,
    expected_w,
    gamma_magnitude,
    hinf_norm,
    min_stable_headway,
    setup_logging,
)
from cacc.metrics import amplification_ratios, l2_errors
from cacc.plotstyle import C, use_house_style, vehicle_colors

log = logging.getLogger("cacc.scripts.reproduce_base_paper")

# ----------------------------- paper constants (Ma et al. 2025, Sec. IV) ---
TAU0 = 0.5  # parasitic actuation lag [s]
RHO = 5.0  # channel noise bound parameter
D_STANDSTILL = 5.0  # their d [m]  ->  r = 1 m + vehicle length 4 m
N_FOLLOWERS = 12
T_FINAL = 200.0

CASE_A = dict(name="A (stable)", kp=0.009, kv=0.63, ka=0.5, h=0.95)
CASE_B = dict(name="B (unstable)", kp=0.009, kv=0.63, ka=0.5, h=0.65)
# case C uses ka* and h = 0.88 s > h*_lb = 0.8727 s
CASE_C = dict(name="C (optimal)", kp=0.003, kv=0.85, ka=None, h=0.88)


def paper_leader(t: float) -> float:
    """Their eq. (46): a0 = 0.5 sin(0.1 (t-10)) for 10 < t < 10 + 20*pi."""
    return 0.5 * np.sin(0.1 * (t - 10.0)) if 10.0 < t < 10.0 + 20.0 * np.pi else 0.0


def paper_config(case: dict, n: int, t_final: float, **net) -> PlatoonConfig:
    return PlatoonConfig(
        n_followers=n,
        v0=20.0,
        vehicle=VehicleParams(tau=TAU0, length=4.0),
        control=ControllerParams(kp=case["kp"], kv=case["kv"], ka=case["ka"],
                                 h=case["h"], r=D_STANDSTILL - 4.0),
        delay=net.get("delay", 0.0),
        loss_prob=net.get("loss_prob", 0.0),
        msg_rate=net.get("msg_rate"),
        noise_rho=net.get("noise_rho", RHO),
        dt=0.01,
        t_final=t_final,
        seed=net.get("seed", 1),
    )


def run_case(case: dict, n: int, t_final: float, **net):
    sim = PlatoonSim(paper_config(case, n, t_final, **net), controller="cthp",
                     leader_accel=paper_leader)
    return sim, sim.run()


# ------------------------------------------------------------------ figures
def fig1_design_space(out: Path) -> None:
    fig, ax = plt.subplots(figsize=(6.5, 4.6))
    for rho, style in ((2.0, "-"), (5.0, "-"), (10.0, "-"), (None, "--")):
        upper = 1.0 if rho is None else 1.0 / (1.0 + 1.0 / rho)
        kas = np.linspace(0.02, upper * 0.995, 400)
        ax.plot(kas, [cthp_h_lb(k, rho, TAU0) for k in kas], style,
                label=r"$\rho = \infty$ (no noise)" if rho is None
                else rf"$\rho = {rho:g}$")
        if rho is not None:
            ka_s, h_s = cthp_optimal(rho, TAU0)
            ax.plot(ka_s, h_s, "k*", ms=9)
    ax.plot(CASE_A["ka"], CASE_A["h"], "s", color=C["fixed_good"], ms=8,
            mec="white", mew=0.9, zorder=5,
            label=r"paper case A ($k_a{=}0.5$, $h_w{=}0.95$ s)")
    ka_star, _ = cthp_optimal(RHO, TAU0)
    ax.plot(ka_star, CASE_C["h"], "^", color=C["adaptive"], ms=10,
            mec="white", mew=0.9, zorder=5,
            label=rf"paper case C ($k_a^*{{=}}{ka_star:.3f}$, $h_w{{=}}0.88$ s)")
    ax.set_xlabel(r"acceleration feedforward gain  $k_a$")
    ax.set_ylabel(r"min robust string-stable headway  $h_{w,lb}$  [s]")
    ax.set_title(rf"Theorem III.2 design space  ($\tau_0 = {TAU0}$ s); "
                 r"$\star$ = optimum (eq. 18-19)")
    ax.set_ylim(0.4, 2.6)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "fig1_design_space.png", dpi=140)
    plt.close(fig)


def fig2_frequency_response(out: Path) -> dict:
    omega = np.logspace(-2, 1, 4000)
    lo, hi = (1 - 1 / RHO) * CASE_A["ka"], (1 + 1 / RHO) * CASE_A["ka"]
    nom = expected_w(RHO) * CASE_A["ka"]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.4), sharey=True)
    peaks: dict = {}
    for ax, case in zip(axes, (CASE_A, CASE_B)):
        for ka_eff, lbl in ((lo, r"$\tilde{k}_a$ low end"),
                            (nom, r"$\tilde{k}_a$ nominal $k_a E[w]$"),
                            (hi, r"$\tilde{k}_a$ high end")):
            mag = gamma_magnitude(omega, "cthp", case["h"], kp=case["kp"],
                                  kv=case["kv"], ka_eff=ka_eff, tau=TAU0)
            ax.semilogx(omega, mag, label=lbl)
            peaks[f"case_{case['name'][0]}_ka_eff_{ka_eff:.3f}"] = float(mag.max())
        ax.axhline(1.0, color="k", ls=":", lw=1)
        ax.set_xlabel(r"$\omega$  [rad/s]")
        ax.set_title(rf"case {case['name']}:  $h_w = {case['h']}$ s")
        ax.grid(alpha=0.3, which="both")
    axes[0].set_ylabel(r"$|\tilde{H}(j\omega;\tau_0)|$")
    axes[0].legend(fontsize=8)
    fig.suptitle("Spacing-error propagation magnitude across the noise interval "
                 "(paper Figs. 6 and 9)")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(out / "fig2_frequency_response.png", dpi=140)
    plt.close(fig)
    return peaks


def fig3_time_domain(out: Path, res_a, sim_a, res_b) -> None:
    n = res_a.config.n_followers
    colors = vehicle_colors(n)
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    for ax, res, case in ((axes[0, 0], res_a, CASE_A), (axes[0, 1], res_b, CASE_B)):
        delta = -res.err  # paper sign convention: delta_i = -e_i
        for i in range(n):
            ax.plot(res.t, delta[:, i], color=colors[i], lw=0.9,
                    label=f"veh {i + 1}" if i % 3 == 0 else None)
        ratios = amplification_ratios(res)
        verdict = "attenuating" if np.all(ratios <= 1.02) else "AMPLIFYING"
        ax.set_title(rf"case {case['name']}:  $h_w={case['h']}$ s  ({verdict})")
        ax.set_xlabel("t [s]")
        ax.set_ylabel(r"$\delta_i$ [m]")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7)
    # panel 3: paper Fig. 8 — actual vs communicated acceleration on link 1->2
    ax = axes[1, 0]
    t = res_a.t
    mask = (t >= 10.0) & (t <= 90.0)
    a1 = res_a.acc[:, 1]
    w = np.array([sim_a.links[1].noise_at(tk) for tk in t])
    ax.plot(t[mask], a1[mask], "k", lw=1.2, label=r"actual  $a_1(t)$")
    ax.plot(t[mask], (w * a1)[mask], color="tab:orange", lw=0.6, alpha=0.9,
            label=r"communicated  $w_{2,1}(t)\,a_1(t)$")
    ax.set_title("Noisy communicated acceleration, link 1→2 (paper Fig. 8)")
    ax.set_xlabel("t [s]")
    ax.set_ylabel(r"a  [m/s$^2$]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    # panel 4: per-vehicle L2 amplification bars
    ax = axes[1, 1]
    idx = np.arange(1, n)
    ax.bar(idx - 0.2, amplification_ratios(res_a), 0.4, label="case A")
    ax.bar(idx + 0.2, amplification_ratios(res_b), 0.4, label="case B")
    ax.axhline(1.0, color="k", ls=":", lw=1)
    ax.set_title(r"L2 error amplification  $\|e_i\|/\|e_{i-1}\|$")
    ax.set_xlabel("vehicle index i")
    ax.grid(alpha=0.3, axis="y")
    ax.legend()
    fig.suptitle(f"CTHP platoon, N = {n}, 16-bit channel noise "
                 rf"$\rho = {RHO:g}$ (paper Figs. 7 and 10)")
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(out / "fig3_time_domain.png", dpi=140)
    plt.close(fig)


def fig4_optimal_case(out: Path, res_c, res_a) -> None:
    n = res_c.config.n_followers
    colors = vehicle_colors(n)
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.4))
    ax = axes[0]
    for i in range(n):
        ax.plot(res_c.t, -res_c.err[:, i], color=colors[i], lw=0.9)
    ax.set_title(rf"case C:  $k_a^*$, $h_w = {CASE_C['h']}$ s, "
                 rf"$k_p = {CASE_C['kp']}$, $k_v = {CASE_C['kv']}$ (paper Fig. 12)")
    ax.set_xlabel("t [s]")
    ax.set_ylabel(r"$\delta_i$ [m]")
    ax.grid(alpha=0.3)
    ax = axes[1]
    for res, lbl in ((res_a, rf"$h_w = {CASE_A['h']}$ s (case A)"),
                     (res_c, rf"$h_w = {CASE_C['h']}$ s (case C)")):
        ax.plot(res.t, res.pos[:, 0] - res.pos[:, -1], lw=1.4, label=lbl)
    ax.set_title(r"platoon length $x_0 - x_N$ — throughput gain (paper Fig. 13)")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("length [m]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9)
    fig.tight_layout()
    fig.savefig(out / "fig4_optimal_case.png", dpi=140)
    plt.close(fig)


def fig5_delay_extension(out: Path, steps: int) -> dict:
    """Project novelty: how V2V delay erodes the headway margin (fixed
    case-A gains), for the noiseless channel and both noise-interval ends."""
    thetas = np.linspace(0.0, 0.5, steps)
    lo = (1 - 1 / RHO) * CASE_A["ka"]
    nom = expected_w(RHO) * CASE_A["ka"]
    curves: dict[str, list[float]] = {}
    for ka_eff, key in ((CASE_A["ka"], "noiseless"), (lo, "noisy_low_end"),
                        (nom, "noisy_nominal")):
        curves[key] = [
            min_stable_headway("cthp", theta=th, kp=CASE_A["kp"], tau=TAU0,
                               kv=CASE_A["kv"], ka_eff=ka_eff)
            for th in thetas
        ]
    fig, ax = plt.subplots(figsize=(6.8, 4.6))
    labels = {"noiseless": r"noiseless ($\tilde{k}_a = k_a$)",
              "noisy_low_end": r"noise, low end ($\tilde{k}_a = (1-1/\rho)k_a$)",
              "noisy_nominal": r"noise, nominal ($\tilde{k}_a = k_a E[w]$)"}
    for key, hs in curves.items():
        ax.plot(thetas, hs, "o-", ms=3, label=labels[key])
    ax.axhline(CASE_A["h"], color="k", ls=":", lw=1,
               label=rf"designed $h_w = {CASE_A['h']}$ s")
    ax.set_xlabel(r"V2V delay  $\theta$  [s]")
    ax.set_ylabel(r"min string-stable headway  $h_{min}$  [s]")
    ax.set_title("Extension: delay erodes the headway margin "
                 f"(case-A gains, τ₀ = {TAU0} s)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "fig5_delay_extension.png", dpi=140)
    plt.close(fig)
    return {"theta_s": thetas.tolist(),
            **{f"h_min_{k}_s": v for k, v in curves.items()}}


def packet_loss_study(n: int, t_final: float, seeds: range) -> list[dict]:
    """Extension: empirical string stability vs Bernoulli beacon loss
    (10 Hz DSRC-like sampling, case-A design, noisy channel)."""
    rows = []
    for loss in (0.0, 0.1, 0.3, 0.5):
        worst = []
        for seed in seeds:
            _, res = run_case(CASE_A, n, t_final, msg_rate=10.0,
                              loss_prob=loss, delay=0.02, seed=seed)
            worst.append(float(np.max(amplification_ratios(res))))
        rows.append({"loss_prob": loss, "msg_rate_hz": 10.0, "delay_s": 0.02,
                     "max_l2_amplification_mean": float(np.mean(worst)),
                     "max_l2_amplification_max": float(np.max(worst)),
                     "seeds": list(seeds)})
        log.info("loss=%.1f -> max L2 amplification %.3f (mean over %d seeds)",
                 loss, np.mean(worst), len(worst))
    return rows


# --------------------------------------------------------------------- main
def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", default="results")
    parser.add_argument("--quick", action="store_true",
                        help="smaller platoon / fewer sweeps (smoke test)")
    args = parser.parse_args()
    setup_logging()
    use_house_style()

    n = 6 if args.quick else N_FOLLOWERS
    t_final = 80.0 if args.quick else T_FINAL
    sweep_steps = 6 if args.quick else 21
    seeds = range(1, 3) if args.quick else range(1, 6)

    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    out = Path(args.outdir) / "base_paper" / stamp
    out.mkdir(parents=True, exist_ok=True)

    # -------- theory checks against the paper's printed numbers
    ka_star, h_star = cthp_optimal(RHO, TAU0)
    case_c = dict(CASE_C, ka=ka_star)
    checks = {
        "h_lb(ka=0.5, rho=5)": {"ours": cthp_h_lb(0.5, RHO, TAU0), "paper": 0.9375},
        "ka_star(rho=5)": {"ours": ka_star, "paper": 0.3183},
        "h_star_lb(rho=5)": {"ours": h_star, "paper": 0.8727},
        "E[w] (16-bit channel)": {"ours": expected_w(RHO), "paper": None},
        "case_A_gains_feasible": {"ours": cthp_gains_feasible(
            CASE_A["kp"], CASE_A["kv"], CASE_A["ka"], CASE_A["h"], RHO, TAU0),
            "paper": True},
        "case_B_gains_feasible": {"ours": cthp_gains_feasible(
            CASE_B["kp"], CASE_B["kv"], CASE_B["ka"], CASE_B["h"], RHO, TAU0),
            "paper": False},
    }
    for k, v in checks.items():
        log.info("check %-28s ours=%s paper=%s", k, v["ours"], v["paper"])

    # -------- figures
    fig1_design_space(out)
    peaks = fig2_frequency_response(out)
    sim_a, res_a = run_case(CASE_A, n, t_final)
    _, res_b = run_case(CASE_B, n, t_final)
    _, res_c = run_case(case_c, n, t_final)
    fig3_time_domain(out, res_a, sim_a, res_b)
    fig4_optimal_case(out, res_c, res_a)
    delay_ext = fig5_delay_extension(out, sweep_steps)
    loss_ext = packet_loss_study(min(n, 8), min(t_final, 120.0), seeds)

    # -------- metrics
    metrics = {
        "paper": "Ma, Pagilla, Darbha, IEEE T-ITS 26(1):1029-1038, 2025, "
                 "doi:10.1109/TITS.2024.3498701",
        "params": {"tau0_s": TAU0, "rho": RHO, "d_m": D_STANDSTILL,
                   "n_followers": n, "t_final_s": t_final},
        "theory_checks": checks,
        "hinf_peaks": peaks,
        "time_domain": {
            "case_A_l2_amplification": [round(float(x), 4)
                                        for x in amplification_ratios(res_a)],
            "case_B_l2_amplification": [round(float(x), 4)
                                        for x in amplification_ratios(res_b)],
            "case_C_l2_amplification": [round(float(x), 4)
                                        for x in amplification_ratios(res_c)],
            "case_A_l2_errors_m": [round(float(x), 4) for x in l2_errors(res_a)],
            "case_B_l2_errors_m": [round(float(x), 4) for x in l2_errors(res_b)],
        },
        "extension_delay_sweep": delay_ext,
        "extension_packet_loss": loss_ext,
    }
    (out / "metrics.json").write_text(json.dumps(metrics, indent=2),
                                      encoding="utf-8")
    log.info("outputs written to %s", out.resolve())


if __name__ == "__main__":
    main()
