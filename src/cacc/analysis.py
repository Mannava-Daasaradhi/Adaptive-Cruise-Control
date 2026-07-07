"""Frequency-domain string-stability analysis (exact delay — no Pade).

Two controller families are analyzed on the imaginary axis:

**Ploeg 2014 (ACC / CACC).** With vehicle transfer function
G(s) = 1 / (s^2 (tau s + 1)) (command u to position p), PD feedback
C(s) = kp + kd s, and spacing-policy dynamics H(s) = h s + 1, the
spacing-error propagation along a homogeneous platoon is governed by:

    ACC  (no V2V):        Gamma(s) =  G C            / (1 + G C H)
    CACC (delay theta):   Gamma(s) = (G C + e^{-theta s}) / (H (1 + G C))

**Ma/Pagilla/Darbha 2025 (CTHP).** The spacing-error propagation transfer
function (their eq. (14), with our optional delay extension on the V2V
feedforward path) is

    H~(s; tau) = (ka_eff s^2 e^{-theta s} + kv s + kp)
                 / (tau s^3 + s^2 + (kv + h kp) s + kp)

where ka_eff = ka * w is the acceleration gain as scaled by the channel:
w = E[w(t)] for nominal analysis, w = 1 + 1/rho for worst-case robust
analysis, w = 1 for a noiseless channel.

**String stability** (L2, homogeneous one-vehicle look-ahead):

    ||Gamma(j w)||_inf <= 1

Theorem III.2 of Ma et al. 2025 is implemented in closed form:
:func:`cthp_h_lb` (their eq. (17)), :func:`cthp_optimal` (eqs. (18)-(19)),
:func:`cthp_gains_feasible` (eqs. (29)-(37) + internal stability).
"""

from __future__ import annotations

import logging
from typing import Sequence

import numpy as np

log = logging.getLogger(__name__)

#: default frequency grid [rad/s] — dense enough to resolve the ~1 rad/s peak
OMEGA_DEFAULT = np.logspace(-3, 2.5, 8000)


def gamma(
    omega: np.ndarray,
    kind: str,
    h: float,
    kp: float = 0.2,
    kd: float = 0.7,
    tau: float = 0.1,
    theta: float = 0.0,
    kv: float = 0.63,
    ka_eff: float = 0.5,
    pred_theta_hat: float = 0.0,
    pred_base: float = 0.4,
) -> np.ndarray:
    """Complex string-stability transfer function Gamma(j*omega).

    Parameters
    ----------
    omega:
        Angular-frequency grid [rad/s].
    kind:
        ``'acc'``, ``'cacc'`` (Ploeg family, gains kp/kd) or ``'cthp'``
        (Ma family, gains kp/kv/ka_eff).
    h, tau, theta:
        Headway [s], vehicle lag [s], V2V delay [s] (feedforward path only).
    kp, kd:
        Ploeg-family PD gains (kp is shared by the CTHP family).
    kv, ka_eff:
        CTHP relative-speed gain and *effective* acceleration gain
        (ka scaled by the channel-noise factor; see module docstring).
    pred_theta_hat, pred_base:
        Timestamp-based feedforward predictor (D-016): the receiver adds
        ``theta_hat`` times a slope estimated from two ``pred_base/2``
        moving-average blocks of its received signal, i.e. the exact lead
        operator P(s) = 1 + theta_hat * A(s) * (1 - e^{-sT/2}) / (T/2)
        with A(s) = (1 - e^{-sT/2})/(s T/2), applied to the delayed
        feedforward. ``pred_theta_hat = 0`` disables it (cthp only).
    """
    s = 1j * np.asarray(omega, dtype=float)
    kind = kind.lower()
    if kind in ("acc", "cacc"):
        G = 1.0 / (s**2 * (tau * s + 1.0))
        C = kp + kd * s
        H = h * s + 1.0
        if kind == "acc":
            return G * C / (1.0 + G * C * H)
        return (G * C + np.exp(-theta * s)) / (H * (1.0 + G * C))
    if kind == "cthp":
        ff = np.exp(-theta * s)
        if pred_theta_hat > 0.0:
            half = 0.5 * pred_base
            blk = (1.0 - np.exp(-half * s)) / np.where(s == 0, 1.0, half * s)
            lead = 1.0 + pred_theta_hat * blk * (1.0 - np.exp(-half * s)) / half
            ff = ff * lead
        num = ka_eff * s**2 * ff + kv * s + kp
        den = tau * s**3 + s**2 + (kv + h * kp) * s + kp
        return num / den
    raise ValueError(f"unknown kind: {kind!r}")


def gamma_magnitude(omega: np.ndarray, kind: str, h: float, **kw) -> np.ndarray:
    """|Gamma(j*omega)| on the given frequency grid."""
    return np.abs(gamma(omega, kind, h, **kw))


def hinf_norm(kind: str, h: float, omega: np.ndarray = OMEGA_DEFAULT, **kw) -> float:
    """||Gamma||_inf approximated as the max over the frequency grid."""
    return float(np.max(gamma_magnitude(omega, kind, h, **kw)))


def is_string_stable(kind: str, h: float, tol: float = 1e-6, **kw) -> bool:
    """True when ||Gamma||_inf <= 1 (within numerical tolerance)."""
    return hinf_norm(kind, h, **kw) <= 1.0 + tol


def min_stable_headway(
    kind: str,
    theta: float = 0.0,
    kp: float = 0.2,
    kd: float = 0.7,
    tau: float = 0.1,
    lo: float = 0.01,
    hi: float = 10.0,
    tol: float = 1e-3,
    kv: float = 0.63,
    ka_eff: float = 0.5,
    pred_theta_hat: float = 0.0,
    pred_base: float = 0.4,
) -> float:
    """Smallest string-stable time headway h (bisection on ``is_string_stable``).

    Raises ``ValueError`` if even ``hi`` is not string-stable. Monotonicity of
    string stability in h (larger headway = more stable) holds for these
    controller families and is assumed by the bisection.
    """
    kind = kind.lower()
    kw: dict = dict(kp=kp, tau=tau)
    if kind == "cthp":
        kw.update(kv=kv, ka_eff=ka_eff, theta=theta,
                  pred_theta_hat=pred_theta_hat, pred_base=pred_base)
    else:
        kw["kd"] = kd
        if kind == "cacc":
            kw["theta"] = theta
    if not is_string_stable(kind, hi, **kw):
        raise ValueError(f"{kind} not string-stable even at h={hi} s")
    if is_string_stable(kind, lo, **kw):
        return lo
    a, b = lo, hi
    while b - a > tol:
        mid = 0.5 * (a + b)
        if is_string_stable(kind, mid, **kw):
            b = mid
        else:
            a = mid
    log.debug("min_stable_headway(%s, theta=%.3f) = %.4f s", kind, theta, b)
    return b


# --------------------------------------------------------------------------
# Ma, Pagilla & Darbha 2025, Theorem III.2 — closed-form design equations
# --------------------------------------------------------------------------
def expected_w(rho: float, gammas: Sequence[float] | None = None) -> float:
    """E[w(t)] of the n-bit multiplicative channel noise (their eq. (12)).

    ``gammas`` defaults to the paper's 16 channel-bit expectations
    (:data:`cacc.network.MA2025_GAMMAS`).
    """
    from cacc.network import MA2025_GAMMAS

    g = np.asarray(MA2025_GAMMAS if gammas is None else gammas, dtype=float)
    return float((1.0 - 1.0 / rho) + (g @ (2.0 ** -np.arange(g.size))) / rho)


def cthp_h_lb(ka: float, rho: float | None, tau0: float) -> float:
    """Robust minimum time headway h_w,lb(ka) — Ma et al. 2025, eq. (17).

        h_lb = 2 tau0 (1 - (1 - 1/rho) ka) / (1 - (1 + 1/rho)^2 ka^2)

    ``rho=None`` (or inf) is the noiseless channel: h_lb = 2 tau0 / (1 + ka)
    (Remark 3.3, recovering the classic bound). Requires
    ka in (0, 1/(1 + 1/rho)) — else the denominator is not positive.
    """
    if rho is None or np.isinf(rho):
        if not 0.0 < ka < 1.0:
            raise ValueError("noiseless case needs ka in (0, 1)")
        return 2.0 * tau0 / (1.0 + ka)
    inv = 1.0 / rho
    if not 0.0 < ka < 1.0 / (1.0 + inv):
        raise ValueError(f"ka must be in (0, {1.0 / (1.0 + inv):.4f}) for rho={rho}")
    num = 1.0 - (1.0 - inv) * ka
    den = 1.0 - (1.0 + inv) ** 2 * ka**2
    return 2.0 * tau0 * num / den


def cthp_optimal(rho: float, tau0: float) -> tuple[float, float]:
    """Headway-minimizing gain and bound (ka*, h*_w,lb) — eqs. (18)-(19).

        ka*    = ((1 - 1/sqrt(rho)) / (1 + 1/sqrt(rho))) / (1 + 1/rho)
        h*_lb  = tau0 (1 + 1/sqrt(rho))^2 / (1 + 1/rho)
    """
    if rho <= 1.0:
        raise ValueError("rho must be > 1")
    rs = 1.0 / np.sqrt(rho)
    ka_star = ((1.0 - rs) / (1.0 + rs)) / (1.0 + 1.0 / rho)
    h_star = tau0 * (1.0 + rs) ** 2 / (1.0 + 1.0 / rho)
    return float(ka_star), float(h_star)


def cthp_gains_feasible(
    kp: float,
    kv: float,
    ka: float,
    h: float,
    rho: float | None,
    tau0: float,
) -> bool:
    """Robust feasibility of (kp, kv) — Ma et al. 2025, eqs. (16), (29).

    Checks, for gamma := kv + h*kp and the worst-case effective gains over
    the noise interval I = [(1-1/rho) ka, (1+1/rho) ka]:

        (16)  0 < ka < 1/(1 + 1/rho)
        (28a) gamma <= (1 - (1+1/rho)^2 ka^2) / (2 tau0)
        (28b) gamma >= sqrt(2 kp (1 - (1-1/rho) ka) + kv^2)
        plus internal stability of D(s): gamma > tau0 * kp  (Routh-Hurwitz)

    ``rho=None`` (or inf) reduces to the noiseless conditions.
    """
    if kp <= 0 or kv <= 0:
        return False
    inv = 0.0 if (rho is None or np.isinf(rho)) else 1.0 / rho
    if not 0.0 < ka < 1.0 / (1.0 + inv):
        return False
    g = kv + h * kp
    upper = (1.0 - (1.0 + inv) ** 2 * ka**2) / (2.0 * tau0)
    lower = np.sqrt(2.0 * kp * (1.0 - (1.0 - inv) * ka) + kv**2)
    return bool(lower <= g <= upper and g > tau0 * kp)
