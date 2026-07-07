"""Headline analysis: minimum string-stable time headway vs V2V delay.

Frequency-domain sweep (exact delay, no simulation needed): for each delay
theta, bisect the smallest h with ||Gamma_CACC||_inf <= 1; compare against the
delay-independent ACC bound. The gap between the curves is the road-capacity
benefit CACC buys — and how communication delay erodes it.

Usage:
    python scripts/sweep_delay.py [--theta-max 0.5] [--steps 26]
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

from cacc import min_stable_headway, setup_logging
from cacc.controllers import ControllerParams
from cacc.metrics import throughput_veh_per_hour
from cacc.vehicle import VehicleParams

log = logging.getLogger("cacc.scripts.sweep_delay")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--theta-max", type=float, default=0.5)
    parser.add_argument("--steps", type=int, default=26)
    parser.add_argument("--outdir", default="results")
    args = parser.parse_args()

    setup_logging()
    thetas = np.linspace(0.0, args.theta_max, args.steps)
    h_cacc = np.array([min_stable_headway("cacc", theta=th) for th in thetas])
    h_acc = min_stable_headway("acc")
    log.info("ACC min stable headway: %.3f s (delay-independent)", h_acc)
    log.info("CACC min stable headway: %.3f s @ theta=0  ->  %.3f s @ theta=%.2f s",
             h_cacc[0], h_cacc[-1], thetas[-1])

    v0, veh = 20.0, VehicleParams()
    cap = lambda h: throughput_veh_per_hour(v0, ControllerParams(h=h), veh)  # noqa: E731

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.5))
    ax1.plot(thetas, h_cacc, "o-", label="CACC")
    ax1.axhline(h_acc, color="tab:red", ls="--", label=f"ACC ({h_acc:.2f} s)")
    ax1.set_xlabel("V2V delay  theta [s]")
    ax1.set_ylabel("min string-stable headway  h [s]")
    ax1.set_title("Minimum stable time gap vs communication delay")
    ax1.legend()
    ax1.grid(alpha=0.3)

    ax2.plot(thetas, [cap(h) for h in h_cacc], "o-", label="CACC")
    ax2.axhline(cap(h_acc), color="tab:red", ls="--", label="ACC")
    ax2.set_xlabel("V2V delay  theta [s]")
    ax2.set_ylabel(f"lane capacity @ {v0:.0f} m/s [veh/h]")
    ax2.set_title("Road-capacity consequence")
    ax2.legend()
    ax2.grid(alpha=0.3)
    fig.tight_layout()

    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    outdir = Path(args.outdir) / "delay_sweep" / stamp
    outdir.mkdir(parents=True, exist_ok=True)
    fig.savefig(outdir / "figure.png", dpi=140)
    plt.close(fig)
    payload = {
        "theta_s": thetas.tolist(),
        "h_min_cacc_s": h_cacc.tolist(),
        "h_min_acc_s": h_acc,
        "capacity_cacc_veh_h": [cap(h) for h in h_cacc],
        "capacity_acc_veh_h": cap(h_acc),
    }
    (outdir / "sweep.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    log.info("outputs written to %s", outdir.resolve())


if __name__ == "__main__":
    main()
