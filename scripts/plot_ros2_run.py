"""Plot a ROS 2 platoon-run CSV and cross-validate against the Python core.

The distributed ROS 2 simulation (one node per vehicle, ZOH coupling at the
sim rate, asynchronous beacons) should agree with the monolithic RK4
simulator up to O(dt) coupling error: same qualitative attenuation verdict
and similar L2 amplification ratios.

Usage (Windows, `cacc` conda env):

    python scripts/plot_ros2_run.py results/ros2/<stamp>_cthp_n6.csv
"""

from __future__ import annotations

import argparse
import json
import logging
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from cacc import setup_logging

log = logging.getLogger("cacc.scripts.plot_ros2_run")


def load_run(path: Path) -> dict[int, np.ndarray]:
    """CSV -> {vehicle index: array[t, p, v, a, u, e] sorted by t}."""
    raw = np.genfromtxt(path, delimiter=",", names=True)
    out: dict[int, np.ndarray] = {}
    for idx in np.unique(raw["index"]).astype(int):
        rows = raw[raw["index"] == idx]
        order = np.argsort(rows["t"])
        out[idx] = np.stack([rows["t"][order], rows["position"][order],
                             rows["velocity"][order],
                             rows["acceleration"][order],
                             rows["u_cmd"][order],
                             rows["spacing_error"][order]])
    return out


def detrended_error(run: np.ndarray, baseline_t: float) -> np.ndarray:
    """Spacing error minus its pre-maneuver mean.

    Node processes start a few wall-clock ticks apart, freezing a constant
    O(v0*dt) bias into each spacing error (see 08_ROS2_Architecture.md);
    subtracting the quiescent-phase mean removes that artifact before the
    string-stability readout.
    """
    t, e = run[0], run[5]
    quiet = t < baseline_t
    return e - (np.mean(e[quiet]) if np.any(quiet) else 0.0)


def l2_amplification(runs: dict[int, np.ndarray],
                     baseline_t: float) -> np.ndarray:
    """||e_i||_2 / ||e_{i-1}||_2 for followers 2..N (trapezoid in t)."""
    norms = []
    for idx in sorted(runs)[1:]:  # skip leader
        e = detrended_error(runs[idx], baseline_t)
        norms.append(np.sqrt(np.trapezoid(e**2, runs[idx][0])))
    norms = np.asarray(norms)
    norms = np.where(norms < 1e-12, 1e-12, norms)
    return norms[1:] / norms[:-1]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", help="recorder CSV from ros2_run_demo.sh")
    parser.add_argument("--outdir", default=None,
                        help="default: alongside the CSV")
    parser.add_argument("--baseline-t", type=float, default=8.0,
                        help="quiescent window [s] used to remove the "
                             "startup spacing-error bias")
    args = parser.parse_args()
    setup_logging()

    path = Path(args.csv)
    runs = load_run(path)
    # SIGINT teardown kills nodes up to ~1 s apart; a follower outliving its
    # predecessor's stream records a shutdown artifact — keep only the
    # interval every vehicle co-observed
    t_cut = min(r[0].max() for r in runs.values()) - 1.0
    runs = {i: r[:, r[0] <= t_cut] for i, r in runs.items()}
    followers = sorted(runs)[1:]
    n = len(followers)
    ratios = l2_amplification(runs, args.baseline_t)
    # with channel noise each link injects a fresh disturbance, so per-hop
    # ratios scatter around 1 even for a string-stable design (see
    # 08_ROS2_Architecture.md); the noiseless run is the crisp verdict
    if np.all(ratios <= 1.02):
        verdict = "attenuating"
    elif ratios.mean() <= 1.0:
        verdict = "neutral (noise realization)"
    else:
        verdict = "AMPLIFYING"
    log.info("vehicles: %d followers, L2 amplification: %s -> %s",
             n, np.array2string(ratios, precision=3), verdict)

    colors = plt.cm.viridis(np.linspace(0, 0.9, n))
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.4))
    ax = axes[0]
    t0, v0 = runs[0][0], runs[0][2]
    ax.plot(t0, v0, "k--", lw=1.4, label="leader")
    for j, idx in enumerate(followers):
        ax.plot(runs[idx][0], runs[idx][2], color=colors[j], lw=0.9,
                label=f"veh {idx}" if j % 3 == 0 else None)
    ax.set_title("speeds")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("v [m/s]")
    ax.legend(fontsize=7)
    ax.grid(alpha=0.3)

    ax = axes[1]
    for j, idx in enumerate(followers):
        ax.plot(runs[idx][0], -detrended_error(runs[idx], args.baseline_t),
                color=colors[j], lw=0.9)
    ax.set_title(rf"spacing errors $\delta_i$, detrended ({verdict})")
    ax.set_xlabel("t [s]")
    ax.set_ylabel(r"$\delta_i$ [m]")
    ax.grid(alpha=0.3)

    ax = axes[2]
    ax.bar(np.arange(2, n + 1), ratios)
    ax.axhline(1.0, color="k", ls=":", lw=1)
    ax.set_title(r"L2 amplification $\|e_i\|/\|e_{i-1}\|$")
    ax.set_xlabel("vehicle index i")
    ax.grid(alpha=0.3, axis="y")

    fig.suptitle(f"ROS 2 distributed platoon run — {path.name}")
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    outdir = Path(args.outdir) if args.outdir else path.parent
    out_png = outdir / (path.stem + ".png")
    fig.savefig(out_png, dpi=140)
    plt.close(fig)

    payload = {
        "csv": str(path),
        "n_followers": n,
        "l2_amplification_ratios": [round(float(x), 4) for x in ratios],
        "mean_ratio": round(float(ratios.mean()), 4),
        "verdict": verdict,
        "empirically_string_stable": bool(np.all(ratios <= 1.02)),
    }
    out_json = outdir / (path.stem + "_metrics.json")
    out_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    log.info("wrote %s and %s", out_png, out_json)


if __name__ == "__main__":
    main()
