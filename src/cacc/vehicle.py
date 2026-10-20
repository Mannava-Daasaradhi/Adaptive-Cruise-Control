"""Longitudinal vehicle model: double integrator with first-order actuator lag.

Model (Ploeg et al. 2014, Sec. II) for one vehicle:

    p' = v                    position [m]
    v' = a                    velocity [m/s]
    a' = (u(t - theta_a) - a) / tau
                              realized acceleration tracks command u with lag
                              tau after an optional dead time theta_a

The commanded acceleration ``u`` is saturated to ``[u_min, u_max]`` before it
reaches the actuator, representing engine/brake limits. The dead time
``act_delay`` (theta_a, default 0) models the pure delay of real drivelines
and brake systems; the platoon simulator implements it with a delay line on
the command history (it must then be >= the integration step).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from cacc.ev import EVParams

#: number of dynamic states per vehicle: (p, v, a)
N_STATES = 3


@dataclass(frozen=True)
class VehicleParams:
    """Physical parameters of one vehicle.

    Attributes
    ----------
    tau:
        Actuator (driveline) time constant [s]. Ploeg et al. 2014 use 0.1 s.
    length:
        Vehicle length [m]; used for gap bookkeeping and throughput metrics.
    u_min, u_max:
        Commanded-acceleration saturation limits [m/s^2] (braking / traction).
    act_delay:
        Actuation dead time theta_a [s] before the lag (0 = classic model).
    """

    tau: float = 0.1
    length: float = 4.0
    u_min: float = -8.0
    u_max: float = 3.0
    act_delay: float = 0.0
    ev: EVParams | None = None

    def __post_init__(self):
        if self.tau <= 0.0:
            raise ValueError("vehicle tau must be > 0")
        if self.act_delay < 0.0:
            raise ValueError("vehicle act_delay must be >= 0")

    def clamp(self, u: float, v: float | None = None) -> float:
        """Saturate a commanded acceleration to the actuator limits; with an
        EV powertrain and the speed ``v`` also to motor power and grip."""
        lo, hi = self.u_min, self.u_max
        if self.ev is not None and v is not None:
            lo, hi = self.ev.accel_bounds(v, lo, hi)
        return float(min(max(u, lo), hi))


def vehicle_deriv(state: np.ndarray, u: float, params: VehicleParams) -> np.ndarray:
    """Time derivative of one vehicle's state ``[p, v, a]`` under command ``u``.

    ``u`` is clamped to the actuator limits internally.
    """
    _, v, a = state
    u = params.clamp(u, v)
    return np.array([v, a, (u - a) / params.tau])
