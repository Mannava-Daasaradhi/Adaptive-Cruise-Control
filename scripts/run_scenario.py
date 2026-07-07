"""Run a platoon scenario under ACC and CACC and produce report-ready outputs.

Usage (from the project root, inside the `cacc` conda env):

    python scripts/run_scenario.py scenarios/leader_brake.yaml
    python scripts/run_scenario.py scenarios/leader_sine.yaml --controller cacc

Outputs land in results/<scenario>/<timestamp>/:
    figure.png     4-panel comparison (speeds, spacing errors, amplification)
    metrics.json   scalar metrics per controller (see cacc.metrics.summary)
"""

from __future__ import annotations

import argparse
import json
import logging
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from cacc import PlatoonSim, load_scenario, setup_logging
from cacc.metrics import amplification_ratios, peak_abs_errors, summary

log = logging.getLogger("cacc.scripts.run_scenario")


def plot_comparison(results: dict, scenario_name: str, out_png: Path) -> None:
    """4-panel figure: CACC speeds, ACC vs CACC spacing errors, peak-error bars."""
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    show = results.get("cacc") or next(iter(results.values()))
    n = show.config.n_followers
    colors = plt.cm.viridis(np.linspace(0, 0.9, n))

    ax = axes[0, 0]
    ax.plot(show.t, show.vel[:, 0], "k--", lw=1.5, label="leader")
    for i in range(n):
        ax.plot(show.t, show.vel[:, i + 1], color=colors[i], lw=1, label=f"veh {i + 1}")
    ax.set_title(f"Vehicle speeds ({show.controller.upper()})")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("v [m/s]")
    ax.legend(fontsize=7, ncol=2)

    err_lim = 0.0
    for key, pos in (("acc", (0, 1)), ("cacc", (1, 1))):
        ax = axes[pos]
        if key not in results:
            ax.set_axis_off()
            continue
        res = results[key]
        for i in range(n):
            ax.plot(res.t, res.err[:, i], color=colors[i], lw=1, label=f"veh {i + 1}")
        ratios = amplification_ratios(res)
        verdict = "attenuating" if np.all(ratios <= 1.02) else "AMPLIFYING"
        ax.set_title(f"Spacing errors — {key.upper()} ({verdict})")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("e [m]")
        ax.legend(fontsize=7, ncol=2)
        err_lim = max(err_lim, float(np.max(np.abs(res.err))))
    for pos in ((0, 1), (1, 1)):  # same scale so amplification is visually obvious
        if not axes[pos].axison:
            continue
        axes[pos].set_ylim(-1.1 * err_lim, 1.1 * err_lim)

    ax = axes[1, 0]
    width = 0.38
    idx = np.arange(1, n + 1)
    for j, (key, res) in enumerate(sorted(results.items())):
        ax.bar(idx + (j - 0.5) * width, peak_abs_errors(res), width, label=key.upper())
    ax.set_title("Peak |spacing error| along the platoon")
    ax.set_xlabel("vehicle index")
    ax.set_ylabel("max |e| [m]")
    ax.set_xticks(idx)
    ax.legend()

    cfg = show.config
    fig.suptitle(
        f"{scenario_name}:  h = {cfg.control.h} s,  V2V delay = {cfg.delay} s,  "
        f"loss = {cfg.loss_prob}", fontsize=13,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_png, dpi=140)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", help="path to a scenarios/*.yaml file")
    parser.add_argument("--controller", choices=["acc", "cacc", "both"], default="both")
    parser.add_argument("--outdir", default="results", help="output root directory")
    args = parser.parse_args()

    setup_logging()
    sc = load_scenario(args.scenario)
    kinds = ["acc", "cacc"] if args.controller == "both" else [args.controller]

    results = {}
    metrics = {}
    for kind in kinds:
        res = PlatoonSim(sc.config, controller=kind, leader_accel=sc.leader).run()
        results[kind] = res
        metrics[kind] = summary(res)
        log.info("%s metrics: %s", kind.upper(), json.dumps(metrics[kind]))

    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    outdir = Path(args.outdir) / sc.name / stamp
    outdir.mkdir(parents=True, exist_ok=True)
    plot_comparison(results, sc.name, outdir / "figure.png")
    payload = {"scenario": sc.name, "config": asdict(sc.config), "metrics": metrics}
    (outdir / "metrics.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    log.info("outputs written to %s", outdir.resolve())


if __name__ == "__main__":
    main()
