"""Predictive QoS-map spacing study (D-021): anticipatory vs reactive.

Drives the platoon through a geo-referenced V2V interference patch (rho 10 ->
3, known from a shared channel-quality map) and contrasts two controllers on
the SAME channel:

* **reactive** (D-016) — opens the headway from the online rho estimate, i.e.
  only after the channel has already degraded;
* **predictive** (D-021) — opens the headway from the map preview, before the
  platoon reaches the patch.

fig — three stacked panels for the last follower over the traversal:
  (1) live string-stability margin ||H~||_inf in force (the decisive metric):
      reactive breaches 1 on entry (transiently string-UNSTABLE), predictive
      holds <= 1 throughout;
  (2) time headway h(t): predictive opens *before* the position-marked entry;
  (3) spacing error delta(t): smaller in-zone peak for predictive.
metrics.json — h and margin at entry, in-zone peak error and per-hop L2, the
  fraction of in-zone time each design is string-unstable, and the mean
  headway (the capacity price of anticipation).

Run (conda env `cacc`):  python scripts/predictive_qos_study.py [--quick]
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

from cacc import (PlatoonSim, QoSMap, hinf_norm, load_scenario, setup_logging)
from cacc.plotstyle import C, use_house_style, zone_span

log = logging.getLogger("cacc.scripts.predictive_qos_study")

KA, KP, KV, TAU0 = 0.5, 0.009, 0.63, 0.5


def hinf_worst(h: float, rho: float) -> float:
    """||H~||_inf, worst case over both ends of the noise interval."""
    return max(hinf_norm("cthp", h, kp=KP, tau=TAU0, kv=KV, ka_eff=end * KA)
               for end in (1.0 - 1.0 / rho, 1.0 + 1.0 / rho))


def run(sc, preview_s: float):
    """Run the scenario with the adapter's preview horizon overridden."""
    adapt = dataclasses.replace(sc.config.adapt, preview_s=preview_s)
    cfg = dataclasses.replace(sc.config, adapt=adapt)
    log.info("--- variant preview=%.0fs", preview_s)
    return PlatoonSim(cfg, "cthp", sc.leader).run()


def margin_series(res, qmap: QoSMap) -> np.ndarray:
    """||H~||_inf in force for the last follower along its true rho(x)."""
    x_last = res.pos[:, -1]
    h_last = res.h[:, -1]
    return np.array([hinf_worst(float(h), qmap.rho_at(float(x)))
                     for h, x in zip(h_last, x_last)])


def l2_in_window(res, lo: float, hi: float) -> np.ndarray:
    m = (res.t >= lo) & (res.t <= hi)
    norms = np.sqrt(np.trapezoid(res.err[m] ** 2, res.t[m], axis=0))
    norms = np.where(norms < 1e-9, 1e-9, norms)
    return norms[1:] / norms[:-1]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--quick", action="store_true", help="4 followers")
    args = ap.parse_args()
    setup_logging()
    use_house_style()

    sc = load_scenario("scenarios/predictive_qos.yaml")
    if args.quick:
        sc = dataclasses.replace(
            sc, config=dataclasses.replace(sc.config, n_followers=4))
    preview = float(sc.config.adapt.preview_s)
    base_rho = float(sc.config.noise_rho)
    zones = sc.config.qos_map
    qmap = QoSMap(zones, rho_base=base_rho)
    x0, x1, zone_rho = zones[0]

    runs = {"reactive (D-016)": run(sc, 0.0),
            f"predictive (D-021, {preview:.0f} s preview)": run(sc, preview)}
    colors = {list(runs)[0]: C["fixed_worst"], list(runs)[1]: C["adaptive"]}

    # per-run derived series + metrics
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    outdir = Path("results/predictive_qos") / stamp
    outdir.mkdir(parents=True, exist_ok=True)
    metrics: dict = {"zone": {"x0": x0, "x1": x1, "rho": zone_rho,
                              "rho_base": base_rho, "preview_s": preview}}

    fig, axes = plt.subplots(3, 1, figsize=(11, 9.5), sharex=True)
    entry_marks = {}
    for name, res in runs.items():
        x_last = res.pos[:, -1]
        zin = (x_last >= x0) & (x_last <= x1)  # last follower inside patch
        k_entry = int(np.argmax(zin))
        t_entry = float(res.t[k_entry])
        entry_marks[name] = t_entry
        marg = margin_series(res, qmap)
        # in-zone stats
        marg_zone = marg[zin]
        frac_unstable = float((marg_zone > 1.001).mean()) if zin.any() else 0.0
        peak_zone = float(np.abs(res.err[zin]).max()) if zin.any() else 0.0
        metrics[name] = {
            "h_at_entry_s": round(float(res.h[k_entry, -1]), 3),
            "hinf_at_entry": round(float(marg[k_entry]), 4),
            "peak_error_in_zone_m": round(peak_zone, 3),
            "frac_in_zone_string_unstable": round(frac_unstable, 4),
            "max_hinf_in_zone": round(float(marg_zone.max()) if zin.any() else 0.0, 4),
            "l2_in_zone_maneuver": [round(float(x), 4)
                                    for x in l2_in_window(res, 95.0, 170.0)],
            "mean_headway_s": round(float(res.h.mean()), 4),
        }

        c = colors[name]
        axes[0].plot(res.t, marg, color=c, lw=1.6, label=name)
        axes[1].plot(res.t, res.h[:, -1], color=c, lw=1.8, label=name)
        axes[2].plot(res.t, -res.err[:, -1], color=c, lw=1.4, label=name)
        for ax in axes:
            ax.axvline(t_entry, color=c, ls=":", lw=1.1, alpha=0.7)

    axes[0].axhline(1.0, color="k", ls="--", lw=1.0)
    axes[0].set_ylabel(r"$\|\tilde H\|_\infty$ in force")
    axes[0].set_title("Live string-stability margin — reactive breaches 1 on "
                      "entry; predictive holds it")
    axes[0].legend(loc="upper left", fontsize=9)
    axes[1].set_ylabel("time headway $h$ [s]")
    axes[1].set_title("Headway pre-positioned before the position-marked zone "
                      "entry (dotted)")
    axes[2].set_ylabel(r"last-follower error $\delta$ [m]")
    axes[2].set_xlabel("time t [s]")
    axes[2].set_title("Spacing error — smaller in-zone peak for the predictive "
                      "design")

    # shade the last follower's time in the patch (use the predictive run)
    res_p = list(runs.values())[1]
    x_last = res_p.pos[:, -1]
    zt = res_p.t[(x_last >= x0) & (x_last <= x1)]
    if zt.size:
        for i, ax in enumerate(axes):
            zone_span(ax, float(zt[0]), float(zt[-1]),
                      label=f"interference patch  (ρ = {zone_rho:.0f})",
                      first=(i == 0))
    fig.suptitle("Predictive QoS-map spacing (D-021) — anticipatory headway "
                 f"keeps the certified margin through the ρ={zone_rho:.0f} patch")
    fig.tight_layout(rect=(0, 0, 1, 0.965))
    fig.savefig(outdir / "fig1_predictive_vs_reactive.png", dpi=150)
    plt.close(fig)

    (outdir / "metrics.json").write_text(
        json.dumps(metrics, indent=2), encoding="utf-8")
    log.info("predictive-QoS study complete -> %s", outdir)
    print(f"wrote {outdir}/fig1_predictive_vs_reactive.png")
    print(json.dumps({k: v for k, v in metrics.items() if k != "zone"},
                     indent=2))


if __name__ == "__main__":
    main()
