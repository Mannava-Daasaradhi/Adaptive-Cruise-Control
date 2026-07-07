"""ACC / CACC control laws: formulas, equilibrium, and filter convergence."""

import numpy as np
import pytest

from cacc.controllers import ACC, CACC, ControllerParams, make_controller


def test_acc_is_static_pd():
    ctrl = ACC(ControllerParams(kp=0.2, kd=0.7))
    u = ctrl.output(np.empty(0), e=2.0, e_dot=-1.0, u_ff=99.0)  # u_ff ignored
    assert np.isclose(u, 0.2 * 2.0 + 0.7 * (-1.0))
    assert ctrl.n_states == 0 and not ctrl.uses_v2v


def test_cacc_equilibrium_is_stationary():
    """At zero error and zero feedforward, xi = 0 must be an equilibrium."""
    ctrl = CACC(ControllerParams())
    d = ctrl.deriv(np.array([0.0]), e=0.0, e_dot=0.0, u_ff=0.0)
    assert np.allclose(d, 0.0)


def test_cacc_filter_dc_gain():
    """h*xi' + xi = kp*e + u_ff: constant inputs drive xi to kp*e + u_ff."""
    prm = ControllerParams(kp=0.2, kd=0.7, h=0.7)
    ctrl = CACC(prm)
    xi = np.array([0.0])
    dt = 0.001
    for _ in range(int(10 * prm.h / dt)):  # ~10 filter time constants
        xi = xi + dt * ctrl.deriv(xi, e=1.0, e_dot=0.0, u_ff=0.5)
    assert abs(xi[0] - (0.2 * 1.0 + 0.5)) < 1e-3
    assert ctrl.uses_v2v


def test_factory():
    prm = ControllerParams()
    assert isinstance(make_controller("acc", prm), ACC)
    assert isinstance(make_controller("CACC", prm), CACC)
    with pytest.raises(ValueError):
        make_controller("mpc", prm)
