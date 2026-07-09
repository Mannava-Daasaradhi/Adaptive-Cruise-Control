"""Chance-constrained string-stability risk certificate study (D-023).

Turns the binary Ma-2025 guarantee ``||H~||_inf <= 1`` into a calibrated
failure probability from the EXACT 16-bit channel law, for the case-A design.

fig (2x2, headline rho):
  (a) CDF of the multiplicative factor w with the support, E[w] and the
      critical w's of the worst-case / chance-constrained headways; the
      unstable lower tail is shaded — it is thin, which is *why* the dividend
      is small (the paper's worst-case design is near risk-optimal);
  (b) the risk curve exceedance(h) = P(||H~||_inf > 1) with the eps budgets and
      the resulting h_cc(eps), plus h_nom / h_wc markers;
  (c) a headway ladder: h_nom (unsafe) < h_ms (mean-square, too permissive) <
      h_cc(eps) ~ h_cheb(eps) < h_wc — the certified ordering;
  (d) Monte-Carlo validation: the analytic exceedance vs the empirical fraction
      of time the live V2V channel lands in the unstable set (on y = x).
metrics.json — every headway and its exceedance for a rho sweep, the deployed
  case-A risk number, and the MC check.

Run (conda env `cacc`):  python scripts/stochastic_cert_study.py
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

from cacc import (ChannelLaw, amp_at, certify_design, chance_headway,
                  chebyshev_headway, exceedance_prob, meansquare_headway,
                  nominal_headway, setup_logging, worstcase_headway)
from cacc.network import V2VLink
from cacc.plotstyle import C, use_house_style

log = logging.getLogger("cacc.scripts.stochastic_cert_study")

EPS = (1e-2, 1e-3, 1e-4)


def mc_exceedance(rho, h, law, t_final=400.0, dt=0.01, seed=7):
    """Empirical P(||H~||_inf > 1) from the live V2V channel realisation."""
    link = V2VLink(delay=0.0, noise_rho=rho, noise_rate=100.0, seed=seed)
    ts = np.arange(0.0, t_final, dt)
    w = np.array([link.noise_at(float(t)) for t in ts])
    return float((amp_at(w, h, law) > 1.0).mean())


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--rho", type=float, default=3.0, help="headline channel rho")
    args = ap.parse_args()
    setup_logging()
    use_house_style()

    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    outdir = Path("results/stochastic_cert") / stamp
    outdir.mkdir(parents=True, exist_ok=True)

    # ---- rho sweep table ----
    metrics: dict = {"design": "case-A (kp=0.009, kv=0.63, ka=0.5, tau0=0.5)",
                     "eps_budgets": list(EPS), "sweep": {}}
    for rho in (2.0, 3.0, 5.0):
        law = ChannelLaw.build(rho)
        row = {
            "E_w": round(law.mean, 5), "std_w": round(law.std, 5),
            "support": [round(x, 4) for x in law.support],
            "h_nom_mean": round(nominal_headway(law), 4),
            "h_meansquare": round(meansquare_headway(law), 4),
            "h_worstcase": round(worstcase_headway(law), 4),
            "h_chance": {f"{e:.0e}": round(chance_headway(e, law), 4) for e in EPS},
            "h_chebyshev": {f"{e:.0e}": round(chebyshev_headway(e, law), 4) for e in EPS},
        }
        metrics["sweep"][f"rho={rho:g}"] = row
    # deployed design risk number
    metrics["deployed_case_a"] = {
        k: (round(v, 6) if isinstance(v, float) else v)
        for k, v in certify_design(0.95, ChannelLaw.build(5.0)).items()}

    # ---- headline rho: figure ----
    rho = args.rho
    law = ChannelLaw.build(rho)
    h_nom, h_ms, h_wc = (nominal_headway(law), meansquare_headway(law),
                         worstcase_headway(law))
    h_cc = {e: chance_headway(e, law) for e in EPS}
    h_cb = {e: chebyshev_headway(e, law) for e in EPS}

    fig, ax = plt.subplots(2, 2, figsize=(13, 9))
    wlo, whi = law.support

    # (a) CDF of w with critical-w markers + unstable tail
    a0 = ax[0, 0]
    cw = np.cumsum(law.p)
    a0.plot(law.w, cw, color=C["adaptive"], lw=1.8)
    # shade the (generally two-tailed) unstable w-band at h_cc(1e-2)
    wgrid = np.linspace(wlo, whi, 400)
    unst = amp_at(wgrid, h_cc[1e-2], law) > 1.0
    first, i = True, 0
    while i < wgrid.size:
        if unst[i]:
            j = i
            while j + 1 < wgrid.size and unst[j + 1]:
                j += 1
            a0.axvspan(wgrid[i], wgrid[j], color=C["fixed_worst"], alpha=0.15,
                       label="unstable band at $h_{cc}(10^{-2})$" if first else None)
            first, i = False, j + 1
        else:
            i += 1
    a0.axvline(law.mean, color="k", ls="--", lw=1.0)
    a0.text(law.mean, 0.5, r"  $E[w]$", fontsize=8, rotation=90, va="center")
    a0.set_xlim(wlo - 0.02, whi + 0.02)
    a0.set_xlabel("channel factor $w$")
    a0.set_ylabel("CDF  $F(w)$")
    a0.set_title(f"(a) exact 16-bit law of $w$  (ρ={rho:g}) — string stability "
                 "needs $w$ in a middle band")
    a0.legend(loc="center right", fontsize=8)

    # (b) risk curve exceedance(h) — mass below the critical w (unstable tail)
    a1 = ax[0, 1]
    hs = np.linspace(h_nom * 0.98, h_wc * 1.02, 60)
    exc = np.array([max(exceedance_prob(h, law), 1e-9) for h in hs])
    a1.semilogy(hs, exc, color=C["adaptive"], lw=1.8)
    for e in EPS:
        a1.axhline(e, color="0.6", ls=":", lw=0.9)
        a1.plot(h_cc[e], e, "o", color=C["fixed_worst"], ms=6)
        a1.text(h_cc[e], e, f"  $h_{{cc}}$({e:.0e})", fontsize=7, va="bottom")
    a1.axvline(h_wc, color="k", ls="--", lw=1.0)
    a1.text(h_wc, exc.min() * 2, "  $h_{wc}$", fontsize=8)
    a1.set_xlabel("time headway $h$ [s]")
    a1.set_ylabel(r"exceedance  $P(\|\tilde H\|_\infty > 1)$")
    a1.set_title("(b) risk vs headway — the certificate maps a budget ε to "
                 "$h_{cc}$(ε)")

    # (c) headway ladder
    a2 = ax[1, 0]
    rows = [("$h_{nom}$ (E[w], unsafe)", h_nom, C["fixed_worst"]),
            ("$h_{ms}$ (mean-square, too permissive)", h_ms, "0.55")]
    for e in EPS:
        rows.append((f"$h_{{cc}}$({e:.0e})", h_cc[e], C["adaptive"]))
    for e in EPS:
        rows.append((f"$h_{{cheb}}$({e:.0e})", h_cb[e], C["fixed_good"]))
    rows.append(("$h_{wc}$ (worst-case)", h_wc, "k"))
    ylab = [r[0] for r in rows]
    yv = np.arange(len(rows))[::-1]
    a2.barh(yv, [r[1] for r in rows], color=[r[2] for r in rows], alpha=0.85)
    a2.axvline(h_wc, color="k", ls="--", lw=0.8)
    a2.set_yticks(yv)
    a2.set_yticklabels(ylab, fontsize=8)
    a2.set_xlim(h_nom * 0.9, h_wc * 1.05)
    a2.set_xlabel("time headway $h$ [s]")
    div_pct = 100 * (h_wc - h_cc[1e-2]) / h_wc
    a2.set_title(f"(c) certified headway ladder (ρ={rho:g}) — a 1% risk budget "
                 f"trims {div_pct:.0f}% off the worst-case headway")

    # (d) Monte-Carlo validation
    a3 = ax[1, 1]
    checks = [h_nom, h_ms, h_cc[1e-2], h_cc[1e-3], h_wc]
    ana = [exceedance_prob(h, law) for h in checks]
    emp = [mc_exceedance(rho, h, law) for h in checks]
    a3.plot([1e-4, 1], [1e-4, 1], color="0.6", ls="--", lw=1.0)
    a3.loglog(np.maximum(ana, 1e-4), np.maximum(emp, 1e-4), "o",
              color=C["adaptive"], ms=8)
    a3.set_xlabel("analytic exceedance (16-bit law)")
    a3.set_ylabel("empirical (live V2V channel)")
    a3.set_title("(d) validation — analytic risk matches the channel realisation")
    metrics["mc_validation"] = {"h": [round(h, 4) for h in checks],
                                "analytic": [round(a, 4) for a in ana],
                                "empirical": [round(e, 4) for e in emp]}
    metrics["headline"] = {
        "rho": rho, "h_worstcase": round(h_wc, 4),
        "h_chance_1e-2": round(h_cc[1e-2], 4), "h_meansquare": round(h_ms, 4),
        "dividend_s": round(h_wc - h_cc[1e-2], 4),
        "dividend_pct": round(100 * (h_wc - h_cc[1e-2]) / h_wc, 1),
        "meansquare_exceedance": round(exceedance_prob(h_ms, law), 4)}

    fig.suptitle("Chance-constrained string-stability risk certificate (D-023) — "
                 f"exact 16-bit channel law, case-A design (ρ={rho:g})")
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(outdir / "fig1_stochastic_cert.png", dpi=150)
    plt.close(fig)

    (outdir / "metrics.json").write_text(json.dumps(metrics, indent=2),
                                         encoding="utf-8")
    log.info("stochastic-cert study complete -> %s", outdir)
    print(f"wrote {outdir}/fig1_stochastic_cert.png")
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
