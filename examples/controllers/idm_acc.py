"""Example plugin: an IDM-style radar-only ACC (bring-your-own-controller).

The Intelligent Driver Model (Treiber, Hennecke & Helbing 2000) is the
standard stand-in for a commercial ACC in traffic research. It is nonlinear
and works on the raw gap and speed, so it has no closed-form string-stability
transfer function — exactly the case the black-box sweep exists for.

Use it from a test plan:

    under_test:
      controller: examples/controllers/idm_acc.py:IDMACC
      options: {T: 1.2, a_max: 1.0, b: 1.5}

or directly:

    PlatoonSim(cfg, "examples/controllers/idm_acc.py:IDMACC",
               leader, controller_options={"T": 1.2})

Contract (see ``cacc.plugins.LongitudinalController``): class attributes
``n_states``, ``uses_v2v``, ``ff_signal``; methods ``output`` and ``deriv``.
``raw_inputs = True`` asks the simulator for ``gap, v, a, t``;
``equilibrium_gap(v)`` lets it start the platoon in this law's equilibrium.
"""

from __future__ import annotations

import math

import numpy as np


class IDMACC:
    """IDM acceleration law used as an ACC.

    a = a_max * (1 - (v / v_des)^delta - (s*(v, dv) / s)^2)
    s* = s0 + max(0, v T - v dv / (2 sqrt(a_max b)))      (dv = v_lead - v)
    """

    n_states = 0  # static law: no internal state
    uses_v2v = False  # radar only
    ff_signal = "a"  # unused without V2V; required by the protocol
    raw_inputs = True  # we need the raw gap and ego speed

    def __init__(self, params, v_des: float = 36.0, T: float = 1.2,
                 s0: float = 2.0, a_max: float = 1.0, b: float = 1.5,
                 delta: float = 4.0):
        # ``params`` (the scenario's ControllerParams) is not needed: IDM
        # carries its own spacing policy (s0, T) and gains (a_max, b)
        self.v_des, self.T, self.s0 = v_des, T, s0
        self.a_max, self.b, self.delta = a_max, b, delta

    def equilibrium_gap(self, v: float) -> float:
        """Bumper gap [m] at which IDM holds speed ``v`` (needs v < v_des)."""
        free = 1.0 - (v / self.v_des) ** self.delta
        if free <= 0.0:
            raise ValueError(f"v = {v} m/s is not below v_des = {self.v_des}")
        return (self.s0 + v * self.T) / math.sqrt(free)

    def output(self, xc, e, e_dot, u_ff, dv=0.0, *, gap, v, a, t):
        s = max(gap, 0.1)  # guard: IDM brakes hard, never divides by <= 0
        s_star = self.s0 + max(
            0.0, v * self.T - v * dv / (2.0 * math.sqrt(self.a_max * self.b)))
        return self.a_max * (1.0 - (max(v, 0.0) / self.v_des) ** self.delta
                             - (s_star / s) ** 2)

    def deriv(self, xc, e, e_dot, u_ff, dv=0.0, **raw):
        return np.empty(0)
