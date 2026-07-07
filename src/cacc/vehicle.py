"""Longitudinal vehicle model: double integrator with first-order actuator lag.

Model (Ploeg et al. 2014, Sec. II) for one vehicle:

    p' = v                    position [m]
    v' = a                    velocity [m/s]
    a' = (u - a) / tau        realized acceleration tracks command u with lag tau

The commanded acceleration ``u`` is saturated to ``[u_min, u_max]`` before it
reaches the actuator, representing engine/brake limits.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

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
    """

    tau: float = 0.1
    length: float = 4.0
    u_min: float = -8.0
    u_max: float = 3.0

    def clamp(self, u: float) -> float:
        """Saturate a commanded acceleration to the actuator limits."""
        return float(min(max(u, self.u_min), self.u_max))


def vehicle_deriv(state: np.ndarray, u: float, params: VehicleParams) -> np.ndarray:
    """Time derivative of one vehicle's state ``[p, v, a]`` under command ``u``.

    ``u`` is clamped to the actuator limits internally.
    """
    _, v, a = state
    u = params.clamp(u)
    return np.array([v, a, (u - a) / params.tau])
