"""Linear ACC model of a drive-log twin and its exact simulation (D-026).

``LinearACC`` is the car-following law the twin fits; ``simulate_follower``
drives it with a measured lead-vehicle speed. Split out of :mod:`cacc.twin`
(which re-exports both) so each module stays small.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy import signal

from cacc.analysis import OMEGA_DEFAULT, gamma_magnitude, min_stable_headway

PARAMS = ("k_s", "k_v", "T", "s0", "tau")
PARAMS_DELAY = PARAMS + ("delay",)
UNITS = {"k_s": "1/s^2", "k_v": "1/s", "T": "s", "s0": "m", "tau": "s",
         "delay": "s"}
STRUCTURES = ("lag", "lag+delay")
# fit vector q = (k_s, k_v, T, s_bar, tau[, delay]), s_bar = s0 + T * v_bar:
# the equilibrium gap at the mean speed is far better identified than s0
_LO = np.array([1e-3, 0.0, 0.2, 0.5, 0.05])
_HI = np.array([3.0, 3.0, 5.0, 300.0, 3.0])
_DELAY_BOUNDS = (0.0, 1.5)  # [s]
#: dead times below this are treated as zero (the Pade model is stiff there)
_DELAY_MIN = 1e-3
#: start of each segment left out of the fit residuals [s]: the initial
#: state is estimated from noisy samples, and fitting that transient would
#: reward structures with spare states rather than the car's real dynamics
_BURN_IN_S = 5.0


@dataclass(frozen=True)
class LinearACC:
    """Linear lag ACC: u = k_s (s - s0 - T v) + k_v dv,
    a' = (u(t - delay) - a)/tau (``delay`` = 0 is the classic lag model)."""

    k_s: float
    k_v: float
    T: float
    s0: float
    tau: float
    delay: float = 0.0

    def gain(self, omega: np.ndarray, ka: float = 0.0,
             theta: float = 0.0) -> np.ndarray:
        """|Gamma(j omega)|; ``ka``/``theta`` add V2V feedforward (what-if)."""
        return gamma_magnitude(omega, "cthp", self.T, kp=self.k_s, kv=self.k_v,
                               ka_eff=ka, tau=self.tau, theta=theta,
                               act_delay=self.delay)

    def hinf(self, ka: float = 0.0, theta: float = 0.0) -> tuple[float, float]:
        """(||Gamma||_inf, frequency of the peak [rad/s])."""
        g = self.gain(OMEGA_DEFAULT, ka, theta)
        k = int(np.argmax(g))
        return float(g[k]), float(OMEGA_DEFAULT[k])

    def min_stable_time_gap(self, ka: float = 0.0,
                            theta: float = 0.0) -> float | None:
        """Smallest time gap T at which these gains are string-stable."""
        try:
            return min_stable_headway("cthp", theta=theta, kp=self.k_s,
                                      kv=self.k_v, ka_eff=ka, tau=self.tau,
                                      lo=0.05, hi=10.0, act_delay=self.delay)
        except ValueError:
            return None

    def scenario(self, v0: float, name: str = "twin",
                 description: str = "") -> dict:
        """A scenario mapping that runs this twin with the built-in CTHP law
        (``cacc sweep twin.yaml -c cthp``, or ``base:`` of a test plan)."""
        vehicle = {"tau": round(self.tau, 4), "length": 4.0,
                   "u_min": -8.0, "u_max": 3.0}
        if self.delay >= 0.01:  # the simulator needs dead time >= its step
            vehicle["act_delay"] = round(self.delay, 3)
        return {
            "name": name,
            "description": description,
            "platoon": {"n_followers": 5, "v0": round(float(v0), 3)},
            "vehicle": vehicle,
            "controller": {"kp": round(self.k_s, 5), "kv": round(self.k_v, 5),
                           "ka": 0.0, "h": round(self.T, 4),
                           "r": round(self.s0, 3)},
            "network": {"delay": 0.0},
            "leader": {"profile": "brake", "t_start": 10.0, "duration": 3.0,
                       "decel": -3.0},
            "sim": {"dt": 0.01, "t_final": 120.0, "seed": 1},
        }


def simulate_follower(model: LinearACC, t: np.ndarray, v_lead: np.ndarray,
                      x0: tuple[float, float, float]) -> tuple[np.ndarray,
                                                                np.ndarray]:
    """Gap and speed of ``model`` following the measured ``v_lead``.

    Exact LTI solution (first-order-hold input) from the initial state
    ``x0 = (gap, speed, acceleration)``. A dead time is represented by a
    second-order Pade approximation (two extra states, started in steady
    state), accurate across the string-stability band.
    """
    k_s, k_v, T, s0, tau = (model.k_s, model.k_v, model.T, model.s0, model.tau)
    th = model.delay
    u = np.column_stack([v_lead, np.ones_like(v_lead)])
    if th < _DELAY_MIN:
        A = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, 1.0],
                      [k_s / tau, -(k_s * T + k_v) / tau, -1.0 / tau]])
        B = np.array([[1.0, 0.0], [0.0, 0.0], [k_v / tau, -k_s * s0 / tau]])
        X0 = np.asarray(x0, float)
    else:
        # e^{-th s} ~ 1 - (12/th) s / (s^2 + (6/th) s + 12/th^2)
        c1, c2, c3 = 12.0 / th**2, 6.0 / th, 12.0 / th
        kg = -(k_s * T + k_v)
        A = np.array([[0.0, -1.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 1.0, 0.0, 0.0],
                      [k_s / tau, kg / tau, -1.0 / tau, 0.0, -c3 / tau],
                      [0.0, 0.0, 0.0, 0.0, 1.0],
                      [k_s, kg, 0.0, -c1, -c2]])
        B = np.array([[1.0, 0.0], [0.0, 0.0], [k_v / tau, -k_s * s0 / tau],
                      [0.0, 0.0], [k_v, -k_s * s0]])
        u0 = (k_s * (x0[0] - s0 - T * x0[1]) + k_v * (v_lead[0] - x0[1]))
        X0 = np.array([x0[0], x0[1], x0[2], u0 / c1, 0.0])
    n = A.shape[0]
    sys = signal.StateSpace(A, B, np.eye(n), np.zeros((n, 2)))
    _, y, _ = signal.lsim(sys, u, t - t[0], X0=X0)
    return y[:, 0], y[:, 1]
