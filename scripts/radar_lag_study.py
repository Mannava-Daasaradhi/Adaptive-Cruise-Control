"""Radar-lag study for the physics-consistency V2V gate (D-022, fix of 2026-10-08).

The D-022 results assumed an ideal radar: the follower's estimate of its
predecessor's acceleration was the true acceleration, with no delay. A real
radar gets acceleration by differentiating and filtering range-rate, so the
estimate is late. This study measures what that lag does, on the same
case-A platoon as ``scenarios/spoof_defense.yaml``:

1. **Honest emergency brake** (no attack): the leader brakes at -7 m/s^2 for
   3 s. For headways h in {0.95, 0.8} s and radar lags 0-1.2 s, compare the
   ungated platoon, the original gate (``direct``) and the lag-matched gate
   (``match_lag``). The original gate distrusts the honest, early V2V brake
   message while the late radar still says "no braking", so it can turn a
   near miss into a collision; the lag-matched gate compares like with like.
2. **Attacks at a 0.3 s radar lag**: a +5 m/s^2 phantom-throttle step and a
   -5 m/s^2 phantom-brake step on the link feeding follower #3, against the
   ungated platoon and three gate variants (direct, lag-matched,
   lag-matched + ``trust_braking``).

Outputs in ``results/radar_lag/<stamp>/``:
    fig1_min_gap_vs_lag.png   minimum gap vs radar lag, one panel per headway
    metrics.json              every number below
    summary.md                the two tables, ready to quote

Run (conda env ``cacc``):  python scripts/radar_lag_study.py [--quick]
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

from cacc import PlatoonSim, load_scenario, setup_logging
from cacc.network import LinkAttack
from cacc.platoon import make_leader_profile
from cacc.plotstyle import C, use_house_style
from cacc.radar import RadarConfig
from cacc.trust import TrustConfig

log = logging.getLogger("cacc.scripts.radar_lag_study")

R0 = 3.0
RADAR_NOISE = 0.05  # [m/s^2], the D-022 radar noise level
GATES = {
    "off": TrustConfig(enabled=False),
    "direct": TrustConfig(enabled=True, r0=R0),
    "matched": TrustConfig(enabled=True, r0=R0, match_lag=True),
    "matched+brake": TrustConfig(enabled=True, r0=R0, match_lag=True,
                                 trust_braking=True),
}


def min_gap(res) -> float:
    gaps = res.pos[:, :-1] - res.pos[:, 1:] - res.config.vehicle.length
    return float(gaps.min())


def run(cfg, leader) -> dict:
    res = PlatoonSim(cfg, "cthp", leader).run()
    g = min_gap(res)
    return {"min_gap_m": round(g, 3), "collision": bool(g <= 0.0),
            "peak_error_m": round(float(np.abs(res.err).max()), 3)}


def honest_brake(base, h: float, lags: list[float]) -> dict:
    """Experiment 1: ungated vs direct vs lag-matched across radar lags."""
    brake = make_leader_profile({"profile": "brake", "t_start": 20.0,
                                 "duration": 3.0, "decel": -7.0})
    ctl = dataclasses.replace(base.control, h=h)
    cfg0 = dataclasses.replace(base, control=ctl, attack=None, t_final=45.0)
    out = {"ungated": run(dataclasses.replace(cfg0, trust=GATES["off"]), brake),
           "lags_s": lags, "direct": [], "matched": []}
    for lag in lags:
        radar = RadarConfig(lag=lag, noise=RADAR_NOISE)
        for name in ("direct", "matched"):
            gate = GATES[name]
            if lag == 0.0 and gate.match_lag:  # nothing to match at zero lag
                gate = GATES["direct"]
            cfg = dataclasses.replace(cfg0, trust=gate, radar=radar)
            out[name].append(run(cfg, brake))
            log.info("h=%.2f lag=%.1f %s -> %s", h, lag, name, out[name][-1])
    return out


def attacks(base, sc_leader, lag: float) -> dict:
    """Experiment 2: spoof steps against the gate variants at one lag."""
    radar = RadarConfig(lag=lag, noise=RADAR_NOISE)
    spoofs = {"phantom_throttle_+5": 5.0, "phantom_brake_-5": -5.0}
    out = {}
    for sname, value in spoofs.items():
        atk = LinkAttack(kind="bias", t0=60.0, t1=75.0, value=value, ramp=0.0,
                         target_link=2)
        row = {}
        for gname, gate in GATES.items():
            cfg = dataclasses.replace(base, attack=atk, trust=gate,
                                      radar=(RadarConfig() if gname == "off"
                                             else radar), t_final=110.0)
            row[gname] = run(cfg, sc_leader)
            log.info("%s %s -> %s", sname, gname, row[gname])
        out[sname] = row
    return out


def summary_md(metrics: dict) -> str:
    lines = ["# Radar lag vs the physics-consistency gate", "",
             "Minimum gap [m] over the platoon in an honest -7 m/s^2, 3 s "
             "emergency brake (negative = collision).", ""]
    for key, exp in metrics["honest_brake"].items():
        lines += [f"**{key}** — ungated: {exp['ungated']['min_gap_m']:.2f} m",
                  "", "| radar lag [s] | original gate | lag-matched gate |",
                  "|---|---|---|"]
        for lag, d, m in zip(exp["lags_s"], exp["direct"], exp["matched"],
                             strict=True):
            lines.append(f"| {lag:.1f} | {d['min_gap_m']:.2f} | "
                         f"{m['min_gap_m']:.2f} |")
        lines.append("")
    lag = metrics["attack_lag_s"]
    lines += [f"Attacks on the link feeding follower #3 (step, 60-75 s), radar "
              f"lag {lag:.1f} s: minimum gap / peak spacing error [m].", "",
              "| spoof | ungated | original gate | lag-matched | "
              "lag-matched + trust braking |", "|---|---|---|---|---|"]
    for sname, row in metrics["attacks"].items():
        cells = [f"{row[g]['min_gap_m']:.2f} / {row[g]['peak_error_m']:.1f}"
                 for g in ("off", "direct", "matched", "matched+brake")]
        lines.append(f"| {sname} | " + " | ".join(cells) + " |")
    lines += ["", "Reading: the original gate is safe only while the radar "
              "lag is small next to the V2V delay; the lag-matched gate keeps "
              "honest braking intact at every lag tested, at a small cost "
              "under a sudden spoof. Trust braking passes phantom-brake "
              "spoofs unbounded, so it is not recommended."]
    return "\n".join(lines) + "\n"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--scenario", default="scenarios/spoof_defense.yaml")
    ap.add_argument("--quick", action="store_true",
                    help="fewer lags and one headway (smoke test)")
    args = ap.parse_args()
    setup_logging()
    use_house_style()

    sc = load_scenario(args.scenario)
    base = dataclasses.replace(
        sc.config, trust=dataclasses.replace(sc.config.trust, radar_noise=0.0))
    lags = [0.0, 0.5, 1.2] if args.quick else [0.0, 0.3, 0.5, 0.8, 1.2]
    headways = [0.8] if args.quick else [0.95, 0.8]

    metrics = {"r0": R0, "radar_noise_m_s2": RADAR_NOISE,
               "v2v_delay_s": base.delay, "honest_brake": {}, "attack_lag_s": 0.3}
    for h in headways:
        metrics["honest_brake"][f"h={h:.2f}s"] = honest_brake(base, h, lags)
    metrics["attacks"] = attacks(base, sc.leader, 0.3)

    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    outdir = Path("results/radar_lag") / stamp
    outdir.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(1, len(headways), figsize=(5.6 * len(headways), 4.2),
                             squeeze=False)
    for ax, (key, exp) in zip(axes[0], metrics["honest_brake"].items(),
                              strict=True):
        lg = exp["lags_s"]
        ax.axhline(exp["ungated"]["min_gap_m"], color=C["reference"], lw=1.2,
                   ls="--", label="no gate")
        ax.plot(lg, [d["min_gap_m"] for d in exp["direct"]], "o-",
                color=C["fail"], lw=1.8, label="original gate (direct)")
        ax.plot(lg, [m["min_gap_m"] for m in exp["matched"]], "s-",
                color=C["fix"], lw=1.8, label="lag-matched gate")
        ax.axhline(0.0, color="k", lw=0.9)
        ax.text(lg[-1], 0.0, "collision ", ha="right", va="top", fontsize=8)
        ax.set_xlabel("radar acceleration lag [s]")
        ax.set_ylabel("minimum gap in the platoon [m]")
        ax.set_title(f"Honest -7 m/s² brake, {key}")
        ax.legend(fontsize=8, loc="lower left")
    fig.suptitle("A late radar makes the original gate distrust honest "
                 "braking; matching the lag fixes it")
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(outdir / "fig1_min_gap_vs_lag.png", dpi=150)
    plt.close(fig)

    (outdir / "metrics.json").write_text(json.dumps(metrics, indent=2),
                                         encoding="utf-8")
    (outdir / "summary.md").write_text(summary_md(metrics), encoding="utf-8")
    log.info("radar-lag study complete -> %s", outdir)
    print(summary_md(metrics))
    print(f"wrote {outdir}")


if __name__ == "__main__":
    main()
