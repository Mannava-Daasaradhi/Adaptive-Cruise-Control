"""ACC / CACC / CTHP longitudinal controllers.

Constant-time-headway spacing policy for follower ``i`` behind predecessor
``i-1`` (positions are front-bumper positions):

    d_i     = p_{i-1} - p_i - L          actual gap
    d_des,i = r + h * v_i                desired gap
    e_i     = d_i - d_des,i              spacing error
    e_i'    = v_{i-1} - v_i - h * a_i    (measurable onboard: radar + IMU)
    dv_i    = v_{i-1} - v_i              relative speed (radar)

Note ``e_i = -delta_i`` of Ma et al. 2025 (their eq. (4) with d = r + L).

Controllers:

* ``ACC``  (radar only, static PD):    u_i = kp * e_i + kd * e_i'
* ``CACC`` (Ploeg et al. 2014, Eq. (10); dynamic):

      h * xi_i' + xi_i = kp * e_i + kd * e_i' + u_ff,      u_i = xi_i

  where ``u_ff`` is the predecessor's **commanded** acceleration u_{i-1}
  received over the (delayed) V2V link. The 1/(h s + 1) filter compensates
  the spacing-policy dynamics H(s) = h s + 1, which is exactly what makes
  small headways string-stable.

* ``CTHP`` (Ma, Pagilla, Darbha 2025, Eq. (6); static):

      u_i = kp * e_i + kv * dv_i + ka * u_ff

  where ``u_ff = w(t) * a_{i-1}(t)`` is the predecessor's **realized**
  acceleration received over V2V, possibly corrupted by multiplicative
  channel noise w(t) in [1 - 1/rho, 1 + 1/rho] (applied by the link).
  Signs match the paper because e_i = -delta_i.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ControllerParams:
    """Controller gains and spacing policy.

    ``kp, kd, h = 0.2, 0.7, 0.7`` are the Ploeg et al. 2014 demo values;
    ``kv, ka = 0.63, 0.5`` are the Ma et al. 2025 case-A values (their kp is
    0.009 — set it per scenario, the two controller families scale kp very
    differently).

    Attributes
    ----------
    kp, kd:
        PD gains on spacing error and its derivative (ACC / CACC).
    kv, ka:
        Relative-speed and acceleration-feedforward gains (CTHP only).
    h:
        Time headway of the spacing policy [s].
    r:
        Standstill distance [m] (bumper-to-bumper; front-to-front adds L).
    """

    kp: float = 0.2
    kd: float = 0.7
    kv: float = 0.63
    ka: float = 0.5
    h: float = 0.7
    r: float = 2.5


class ACC:
    """Radar-only PD controller (no V2V). Static: contributes no state."""

    n_states = 0
    uses_v2v = False
    ff_signal = "u"  # irrelevant (no V2V) but keeps the interface uniform

    def __init__(self, params: ControllerParams):
        self.params = params

    def output(self, xc: np.ndarray, e: float, e_dot: float, u_ff: float,
               dv: float = 0.0) -> float:
        """Commanded acceleration ``u_i`` (before actuator saturation)."""
        return self.params.kp * e + self.params.kd * e_dot

    def deriv(self, xc: np.ndarray, e: float, e_dot: float, u_ff: float,
              dv: float = 0.0) -> np.ndarray:
        """No internal dynamics."""
        return np.empty(0)


class CACC:
    """Ploeg 2014 CACC: PD feedback + V2V feedforward through 1/(h s + 1).

    Internal state ``xc[0] = xi_i`` is the commanded acceleration.
    """

    n_states = 1
    uses_v2v = True
    ff_signal = "u"  # predecessor's commanded acceleration

    def __init__(self, params: ControllerParams):
        self.params = params

    def output(self, xc: np.ndarray, e: float, e_dot: float, u_ff: float,
               dv: float = 0.0) -> float:
        """Commanded acceleration ``u_i = xi_i``."""
        return float(xc[0])

    def deriv(self, xc: np.ndarray, e: float, e_dot: float, u_ff: float,
              dv: float = 0.0) -> np.ndarray:
        """h * xi' = -xi + kp*e + kd*e' + u_ff."""
        p = self.params
        return np.array([(-xc[0] + p.kp * e + p.kd * e_dot + u_ff) / p.h])


class CTHP:
    """Ma/Pagilla/Darbha 2025 CTHP law (static, predecessor-follower).

    ``u_i = kp*e + kv*dv + ka*u_ff`` where ``u_ff`` is the (noisy, possibly
    delayed) realized acceleration of the predecessor as delivered by the
    V2V link. The channel noise w(t) lives in :class:`cacc.network.V2VLink`,
    the gain ``ka`` lives here — matching the paper's u_i = ka*w*a_{i-1} ...
    """

    n_states = 0
    uses_v2v = True
    ff_signal = "a"  # predecessor's realized acceleration

    def __init__(self, params: ControllerParams):
        self.params = params

    def output(self, xc: np.ndarray, e: float, e_dot: float, u_ff: float,
               dv: float = 0.0) -> float:
        """Commanded acceleration ``u_i`` (before actuator saturation)."""
        p = self.params
        return p.kp * e + p.kv * dv + p.ka * u_ff

    def deriv(self, xc: np.ndarray, e: float, e_dot: float, u_ff: float,
              dv: float = 0.0) -> np.ndarray:
        """No internal dynamics."""
        return np.empty(0)


def make_controller(kind: str, params: ControllerParams):
    """Factory: ``kind`` is ``'acc'``, ``'cacc'`` or ``'cthp'``."""
    kind = kind.lower()
    if kind == "acc":
        return ACC(params)
    if kind == "cacc":
        return CACC(params)
    if kind == "cthp":
        return CTHP(params)
    raise ValueError(f"unknown controller kind: {kind!r} "
                     "(expected 'acc', 'cacc' or 'cthp')")
