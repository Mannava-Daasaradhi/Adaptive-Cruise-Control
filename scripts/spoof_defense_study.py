"""Physics-consistency V2V gate study (D-022): spoofed vs gated.

A malicious node injects a false acceleration into one follower's feedforward
link (``scenarios/spoof_defense.yaml``). Two controllers are run on the
IDENTICAL attack:

* **ungated** — the raw CTHP law applies ``ka * u_ff`` directly, so the spoof
  propagates unbounded into the command and collapses the gap (a rear-end);
* **gated** (D-022) — the trust gate fuses the V2V feedforward toward the
  follower's independent radar estimate and caps the injected feedforward at
  ``r0 / 2`` for any spoof magnitude.

fig — three stacked panels for the attacked follower over the attack:
  (1) inter-vehicle gap d(t): ungated crosses 0 (collision), gated holds a safe
      standoff;
  (2) trust weight g(t): collapses during the spoof, recovers after;
  (3) injected feedforward |u_ff_eff - a_radar|: the raw design would inject the
      full lie (dashed), the gate caps it at the provable r0/2 (solid).
metrics.json — min gap, peak error and collision flag for each arm, plus the
  measured max injection vs the r0/2 certificate.

Run (conda env `cacc`):  python scripts/spoof_defense_study.py
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
from cacc.plotstyle import C, use_house_style, zone_span

log = logging.getLogger("cacc.scripts.spoof_defense_study")


def run(sc, gate_on: bool):
    """Run the scenario with the trust gate forced on or off."""
    trust = dataclasses.replace(sc.config.trust, enabled=gate_on)
    cfg = dataclasses.replace(sc.config, trust=trust)
    log.info("--- variant gate=%s", "on" if gate_on else "off")
    return PlatoonSim(cfg, "cthp", sc.leader).run()


def gap_series(res, col: int) -> np.ndarray:
    """Bumper-to-bumper gap of the follower in ``col`` behind its predecessor."""
    return res.pos[:, col - 1] - res.pos[:, col] - res.config.vehicle.length


def raw_injection(sc, t: np.ndarray, ka: float) -> np.ndarray:
    """Feedforward the *ungated* design would inject: ka * spoof(t)."""
    a = sc.config.attack
    out = np.zeros_like(t)
    for k, tk in enumerate(t):
        if a.active(float(tk)):
            ramp = 1.0 if a.ramp <= 0 else min(1.0, (tk - a.t0) / a.ramp)
            out[k] = ka * ramp * a.value  # bias kind
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--scenario", default="scenarios/spoof_defense.yaml")
    args = ap.parse_args()
    setup_logging()
    use_house_style()

    sc = load_scenario(args.scenario)
    atk = sc.config.attack
    col = atk.target_link + 1  # pos column of the attacked follower
    ff = atk.target_link  # its column in err / ff_inj (0-based follower)
    ka = sc.config.control.ka
    r0 = sc.config.trust.r0

    ung = run(sc, gate_on=False)
    gat = run(sc, gate_on=True)
    t = ung.t

    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    outdir = Path("results/spoof_defense") / stamp
    outdir.mkdir(parents=True, exist_ok=True)

    metrics = {"attack": {"kind": atk.kind, "value": atk.value,
                          "window": [atk.t0, atk.t1], "target_link": atk.target_link,
                          "r0": r0, "injection_cap": r0 / 2.0},
               "attacked_follower": col}
    for name, res in (("ungated", ung), ("gated", gat)):
        d = gap_series(res, col)
        m = {
            "min_gap_m": round(float(d.min()), 3),
            "collision": bool(d.min() <= 0.0),
            "peak_error_m": round(float(np.abs(res.err[:, ff]).max()), 3),
        }
        if res.ff_inj is not None:  # gated arm records the gate telemetry
            m["max_injection_m_s2"] = round(float(np.abs(res.ff_inj[:, ff]).max()), 4)
            m["min_trust"] = round(float(res.trust[:, ff].min()), 4)
        metrics[name] = m

    # ---- figure: gap, trust, injection for the attacked follower ----
    fig, axes = plt.subplots(3, 1, figsize=(11, 9.5), sharex=True)
    cu, cg = C["fixed_worst"], C["adaptive"]
    aw0, aw1 = float(atk.t0), float(atk.t1)

    axes[0].plot(t, gap_series(ung, col), color=cu, lw=1.7, label="ungated (raw CTHP)")
    axes[0].plot(t, gap_series(gat, col), color=cg, lw=1.8, label="gated (D-022)")
    axes[0].axhline(0.0, color="k", ls="--", lw=1.0)
    axes[0].text(t[-1], 0.0, " collision", va="bottom", ha="right", fontsize=8)
    axes[0].set_ylabel(f"gap of follower #{col} [m]")
    axes[0].set_title("Inter-vehicle gap — the spoof collapses the ungated gap "
                      "(crosses 0); the gate holds a safe standoff")
    axes[0].legend(loc="lower left", fontsize=9)

    axes[1].plot(t, gat.trust[:, ff], color=cg, lw=1.8)
    axes[1].set_ylabel("trust weight $g$")
    axes[1].set_ylim(-0.03, 1.05)
    axes[1].set_title("Trust weight collapses while the V2V message contradicts "
                      "the radar, then recovers")

    axes[2].plot(t, raw_injection(sc, t, ka), color=cu, lw=1.5, ls="--",
                 label=r"raw injection $k_a\cdot$spoof (ungated)")
    axes[2].plot(t, ka * np.abs(gat.ff_inj[:, ff]), color=cg, lw=1.8,
                 label=r"gated injection $k_a|u_{ff}^{eff}-a_{radar}|$")
    axes[2].axhline(ka * r0 / 2, color="k", ls=":", lw=1.2)
    axes[2].text(t[0], ka * r0 / 2, r" certificate $k_a\,r_0/2$",
                 va="bottom", ha="left", fontsize=8)
    axes[2].set_ylabel(r"injected command [m/s$^2$]")
    axes[2].set_xlabel("time t [s]")
    axes[2].set_title("Injected feedforward — the gate caps the lie at the "
                      "provable bound, the raw design does not")
    axes[2].legend(loc="upper right", fontsize=9)

    for i, ax in enumerate(axes):
        zone_span(ax, aw0, aw1, label="V2V spoof active", first=(i == 0))
    fig.suptitle("Physics-consistency V2V gate (D-022) — a bounded-injection "
                 f"certificate ($k_a r_0/2={ka * r0 / 2:.2f}$ m/s$^2$) defeats "
                 "an unbounded spoof")
    fig.tight_layout(rect=(0, 0, 1, 0.965))
    fig.savefig(outdir / "fig1_spoof_defense.png", dpi=150)
    plt.close(fig)

    (outdir / "metrics.json").write_text(json.dumps(metrics, indent=2),
                                         encoding="utf-8")
    log.info("spoof-defence study complete -> %s", outdir)
    print(f"wrote {outdir}/fig1_spoof_defense.png")
    print(json.dumps({k: v for k, v in metrics.items() if k != "attack"},
                     indent=2))


if __name__ == "__main__":
    main()
