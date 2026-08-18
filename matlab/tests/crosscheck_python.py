"""Cross-validate the Simulink model against the project's Python core.

The Python package ``cacc`` is the reference implementation that already
reproduces Ma et al. (2025) to their printed digits (see
``07_Base_Paper_Reproduction_Results.md``).  This script runs the same three
paper cases through it with an IDEAL channel (w == 1), which is what the
Simulink model's ``noise_mode = 'none'`` does, so the two backends can be
compared on dynamics and integration alone -- no stochastic term in the way.

Run:  python matlab/tests/crosscheck_python.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from cacc.controllers import ControllerParams          # noqa: E402
from cacc.platoon import PlatoonConfig, PlatoonSim      # noqa: E402
from cacc.vehicle import VehicleParams                  # noqa: E402

# Sec. IV-A of the paper
TAU0, D_SPACING, V_SS, N = 0.5, 5.0, 25.0, 12
A0_AMP, A0_W, A0_T0 = 0.5, 0.1, 10.0
A0_DUR = 20.0 * math.pi


def leader(t: float) -> float:
    """a0(t) = 0.5 sin(0.1 (t - 10)) on one full period, else 0."""
    if A0_T0 < t < A0_T0 + A0_DUR:
        return A0_AMP * math.sin(A0_W * (t - A0_T0))
    return 0.0


CASES = {
    "case1": dict(ka=0.5, kv=0.63, kp=0.009, h=0.95),
    "case2": dict(ka=0.5, kv=0.63, kp=0.009, h=0.65),
    "case3": dict(ka=0.318305, kv=0.85, kp=0.003, h=0.88),
}


def run(name: str, g: dict) -> np.ndarray:
    veh = VehicleParams(tau=TAU0, length=4.0)
    ctl = ControllerParams(kp=g["kp"], kv=g["kv"], ka=g["ka"], h=g["h"],
                           r=D_SPACING - veh.length)      # r + L = d = 5 m
    cfg = PlatoonConfig(
        n_followers=N, v0=V_SS, vehicle=veh, control=ctl,
        delay=0.0, loss_prob=0.0, msg_rate=None,
        noise_rho=None,                     # ideal channel: w == 1
        dt=0.001, t_final=200.0, seed=1,
    )
    res = PlatoonSim(cfg, controller="cthp", leader_accel=leader).run()
    delta = -res.err                        # sign map D-005: e_i = -delta_i
    return np.max(np.abs(delta), axis=0)


if __name__ == "__main__":
    print("Python core, ideal channel (w == 1), tau = 0.5 s, N = 12, 200 s")
    print(f"{'case':8s} {'max|d_1|':>10s} {'max|d_N|':>10s}   trend")
    for name, g in CASES.items():
        m = run(name, g)
        trend = "decreasing" if m[-1] < m[0] else "INCREASING"
        print(f"{name:8s} {m[0]:10.4f} {m[-1]:10.4f}   {trend}")
        print(f"         profile = {np.round(m, 4).tolist()}")
