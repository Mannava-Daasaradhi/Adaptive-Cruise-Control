"""Scalar performance metrics computed from a :class:`cacc.platoon.SimResult`.

These feed the report tables: disturbance amplification along the platoon
(empirical string stability), road throughput of the spacing policy, and an
acceleration-based energy/comfort proxy.
"""

from __future__ import annotations

import numpy as np

from cacc.controllers import ControllerParams
from cacc.platoon import SimResult
from cacc.vehicle import VehicleParams


def peak_abs_errors(res: SimResult) -> np.ndarray:
    """Peak |spacing error| per follower [m], shape (n,)."""
    return np.max(np.abs(res.err), axis=0)


def l2_errors(res: SimResult) -> np.ndarray:
    """L2 norm of each follower's spacing error, sqrt(int e^2 dt), shape (n,)."""
    dt = res.t[1] - res.t[0]
    return np.sqrt(np.sum(res.err**2, axis=0) * dt)


def amplification_ratios(res: SimResult, kind: str = "l2") -> np.ndarray:
    """Vehicle-to-vehicle error growth ||e_i|| / ||e_{i-1}||, shape (n-1,).

    Ratios <= 1 down the whole platoon = empirically string-stable response.
    ``kind`` selects the norm: ``'l2'`` or ``'peak'``.
    """
    e = l2_errors(res) if kind == "l2" else peak_abs_errors(res)
    e = np.where(e < 1e-12, 1e-12, e)  # guard: quiescent platoon
    return e[1:] / e[:-1]


def empirically_string_stable(res: SimResult, tol: float = 0.02) -> bool:
    """True when no vehicle-to-vehicle L2 amplification exceeds 1 + tol."""
    return bool(np.all(amplification_ratios(res, "l2") <= 1.0 + tol))


def throughput_veh_per_hour(v: float, control: ControllerParams,
                            vehicle: VehicleParams) -> float:
    """Single-lane capacity [veh/h] of the spacing policy at cruise speed v.

    Vehicles occupy (length + r + h*v) meters of road each at speed v.
    """
    return 3600.0 * v / (vehicle.length + control.r + control.h * v)


def accel_rms(res: SimResult) -> np.ndarray:
    """RMS realized acceleration per follower [m/s^2] (energy/comfort proxy).

    Motivation: fuel penalty of a maneuver grows with acceleration variance;
    smoother platoons burn less (documented proxy, not a fuel model).
    """
    return np.sqrt(np.mean(res.acc[:, 1:] ** 2, axis=0))


def summary(res: SimResult) -> dict:
    """JSON-serializable digest used by scripts and the DEVLOG."""
    cfg = res.config
    return {
        "controller": res.controller,
        "n_followers": cfg.n_followers,
        "headway_s": cfg.control.h,
        "delay_s": cfg.delay,
        "loss_prob": cfg.loss_prob,
        "peak_abs_errors_m": [round(float(x), 4) for x in peak_abs_errors(res)],
        "l2_amplification_ratios": [round(float(x), 4) for x in amplification_ratios(res)],
        "empirically_string_stable": empirically_string_stable(res),
        "throughput_veh_per_hour": round(
            throughput_veh_per_hour(cfg.v0, cfg.control, cfg.vehicle), 1
        ),
        "accel_rms_mps2": [round(float(x), 4) for x in accel_rms(res)],
    }
