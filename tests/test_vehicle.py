"""Vehicle model: saturation and first-order actuator-lag response."""

import numpy as np

from cacc.vehicle import VehicleParams, vehicle_deriv


def test_clamp_limits():
    p = VehicleParams(u_min=-8.0, u_max=3.0)
    assert p.clamp(-20.0) == -8.0
    assert p.clamp(20.0) == 3.0
    assert p.clamp(1.5) == 1.5


def test_deriv_formula():
    p = VehicleParams(tau=0.1)
    d = vehicle_deriv(np.array([0.0, 20.0, 0.5]), u=1.0, params=p)
    assert d[0] == 20.0  # p' = v
    assert d[1] == 0.5  # v' = a
    assert np.isclose(d[2], (1.0 - 0.5) / 0.1)  # a' = (u - a)/tau


def test_step_response_converges_to_command():
    """Constant command u: realized a converges to u within ~5 tau."""
    p = VehicleParams(tau=0.1)
    x = np.array([0.0, 20.0, 0.0])
    dt, u = 0.001, 1.0
    for k in range(int(5 * p.tau / dt)):
        k1 = vehicle_deriv(x, u, p)
        k2 = vehicle_deriv(x + dt / 2 * k1, u, p)
        k3 = vehicle_deriv(x + dt / 2 * k2, u, p)
        k4 = vehicle_deriv(x + dt * k3, u, p)
        x = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    assert abs(x[2] - u) < 0.01  # a ~= u after 5 time constants
    assert x[1] > 20.0  # vehicle sped up
