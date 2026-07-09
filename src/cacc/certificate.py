"""Chance-constrained string-stability risk certificate (D-023).

The Ma-2025 guarantee ``||H~||_inf <= 1`` is enforced at the channel's *worst
case* — the effective feedforward gain ``ka_eff = (1 - 1/rho) * ka``, i.e. all
16 channel bits low, an event of probability ``prod_j (1 - gamma_j)`` (~1e-6).
Designing for it is safe but conservative: the actual multiplicative factor
``w`` is a random variable whose distribution is known exactly from the 16 bit
expectations ``gamma_j``.

This module turns the binary guarantee into a **calibrated risk**. Because the
per-hop amplification ``A(w) = ||H~(.; ka_eff = w*ka)||_inf`` is monotone
*decreasing* in ``w`` over the channel support (verified — larger feedforward
is more string-stable up to the feasibility ceiling), the unstable set is a
clean lower tail ``{w < w_crit(h)}`` and

    exceedance(h) = P(||H~||_inf > 1) = P(w < w_crit(h)) = F(w_crit(h)).

From it we build three certificates, all parameterised by the exact 16-bit law:

* **chance-constrained** (headline) — ``h_cc(eps)`` = the smallest headway with
  ``exceedance(h) <= eps``; equivalently ``min_stable_headway`` at the
  ``eps``-quantile of ``w``. Turns any design into a failure probability and
  gives a *risk-parameterised* headway that replaces the ad-hoc ``rho_safety``
  divisor of the QoS layer ([D-016]) with a calibrated budget;
* **mean-square** — ``h_ms`` with ``E[A(w)^2] <= 1``; the classic stochastic
  notion. Reported as a companion **with a caveat**: it licenses headways whose
  instantaneous ``||H~||_inf`` exceeds 1 a large fraction of the time, i.e. it
  is *too permissive* to be robustly string-stable (see D-023) — its role here
  is to justify the worst-case / chance-constrained choice, not to be used;
* **distribution-free (Cantelli)** — ``h_cheb(eps)`` using only ``E[w]`` and
  ``Var[w]`` (closed form), a conservative bound robust to uncertainty in the
  ``gamma_j``.

For the Ma-2025 channel the dividend is small (``h_cc(1e-2)`` is ~2 % below
``h_cc(1e-6) ~ h_wc``): the certificate *proves the paper's worst-case design
is near risk-optimal* rather than beating it — an honest, useful result.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass

import numpy as np

from cacc.analysis import OMEGA_DEFAULT, hinf_norm, min_stable_headway
from cacc.network import MA2025_GAMMAS

log = logging.getLogger(__name__)

#: frequency grid for the certificate sweeps — the project-standard grid, so
#: the ||H~||_inf values match the rest of the analysis (margins are ~1e-3)
CERT_OMEGA = OMEGA_DEFAULT

# case-A defaults (Ma 2025); override per call for other operating points
KA0, KP0, KV0, TAU0 = 0.5, 0.009, 0.63, 0.5


@dataclass(frozen=True)
class ChannelLaw:
    """Exact distribution of the multiplicative factor ``w`` for one ``rho``.

    ``w = (1 - 1/rho) + U'/rho`` with ``U' = sum_j z_j 2^-j`` and
    ``z_j ~ Bernoulli(gamma_j)`` independent, so ``w`` has ``2^n`` atoms
    (n = 16). Built by iterated convolution of the ``n`` two-point bit laws.
    """

    rho: float
    w: np.ndarray  # atom values, ascending
    p: np.ndarray  # atom probabilities, aligned with ``w``

    @classmethod
    def build(cls, rho: float, gammas=None) -> "ChannelLaw":
        if rho <= 1.0:
            raise ValueError("rho must be > 1")
        g = np.asarray(MA2025_GAMMAS if gammas is None else gammas, dtype=float)
        if not np.all((0.0 < g) & (g < 1.0)):
            raise ValueError("gammas must lie in (0, 1)")
        vals = np.array([0.0])
        pr = np.array([1.0])
        for j, gj in enumerate(g):
            vals = np.concatenate([vals, vals + 2.0 ** -j])
            pr = np.concatenate([pr * (1.0 - gj), pr * gj])
        w = (1.0 - 1.0 / rho) + vals / rho
        order = np.argsort(w, kind="mergesort")
        return cls(float(rho), w[order], pr[order])

    @property
    def mean(self) -> float:
        return float(self.w @ self.p)

    @property
    def var(self) -> float:
        m = self.mean
        return float(self.p @ (self.w - m) ** 2)

    @property
    def std(self) -> float:
        return float(np.sqrt(self.var))

    @property
    def support(self) -> tuple[float, float]:
        return float(self.w[0]), float(self.w[-1])

    def cdf(self, x: float) -> float:
        """``P(w <= x)``."""
        return float(self.p[self.w <= x].sum())

    def quantile(self, eps: float) -> float:
        """Lower-tail ``eps``-quantile ``inf{x : F(x) >= eps}``."""
        if not 0.0 < eps < 1.0:
            raise ValueError("eps must be in (0, 1)")
        c = np.cumsum(self.p)
        return float(self.w[min(int(np.searchsorted(c, eps)), self.w.size - 1)])


# --------------------------------------------------------------- amplification
def _amp(w, h, ka, kp, kv, tau0, theta, omega):
    """``A(w) = ||H~(.; ka_eff = w*ka)||_inf``."""
    return hinf_norm("cthp", h, omega=omega, kp=kp, tau=tau0, kv=kv,
                     ka_eff=float(w) * ka, theta=theta)


def amp_at(w_query, h, law, ka=KA0, kp=KP0, kv=KV0, tau0=TAU0, theta=0.0,
           ng=300, omega=CERT_OMEGA) -> np.ndarray:
    """``A(w)`` at arbitrary ``w`` by interpolation of an ``ng``-point sweep.

    The channel amplification is *not* monotone in ``w``: at a degraded channel
    the feedforward is too weak at low ``w`` (deep fade) AND over-amplified at
    high ``w``, so string stability holds only in a middle band and the
    unstable set is generally two-tailed. Interpolating a dense sweep handles
    any geometry.
    """
    wg = np.linspace(law.support[0], law.support[1], ng)
    ag = np.array([_amp(wi, h, ka, kp, kv, tau0, theta, omega) for wi in wg])
    return np.interp(np.asarray(w_query, dtype=float), wg, ag)


def _max_amp(h, law, ka, kp, kv, tau0, theta, omega, ng=41):
    """Worst ``A(w)`` over the support (both tails / interior)."""
    wg = np.linspace(law.support[0], law.support[1], ng)
    return max(_amp(wi, h, ka, kp, kv, tau0, theta, omega) for wi in wg)


def exceedance_prob(h, law, ka=KA0, kp=KP0, kv=KV0, tau0=TAU0, theta=0.0,
                    ng=300, omega=CERT_OMEGA) -> float:
    """``P(||H~||_inf > 1)`` for headway ``h``, summed over the channel atoms.

    Handles the two-tailed unstable set by interpolating ``A(w)`` from an
    ``ng``-point sweep to every atom and weighting the unstable ones.
    """
    a = amp_at(law.w, h, law, ka, kp, kv, tau0, theta, ng, omega)
    return float(law.p[a > 1.0].sum())


def mean_square_amp(h, law, ka=KA0, kp=KP0, kv=KV0, tau0=TAU0, theta=0.0,
                    ng=300, omega=CERT_OMEGA) -> float:
    """Expected per-hop energy amplification ``E[A(w)^2]``."""
    a = amp_at(law.w, h, law, ka, kp, kv, tau0, theta, ng, omega)
    return float(law.p @ a ** 2)


# ----------------------------------------------------------- headway solvers
def _bisect_h(ok, lo, hi, tol=1e-3):
    """Smallest ``h`` in ``[lo, hi]`` with ``ok(h)`` True (``ok`` monotone up)."""
    if ok(lo):
        return lo
    if not ok(hi):
        raise ValueError(f"predicate not satisfied even at h={hi}")
    while hi - lo > tol:
        m = 0.5 * (lo + hi)
        if ok(m):
            hi = m
        else:
            lo = m
    return hi


def nominal_headway(law, ka=KA0, kp=KP0, kv=KV0, tau0=TAU0, theta=0.0,
                    hi=6.0) -> float:
    """Naive design at the channel mean ``E[w]`` (unsafe; ~50 % exceedance)."""
    return min_stable_headway("cthp", theta=theta, kp=kp, tau=tau0, kv=kv,
                              ka_eff=law.mean * ka, hi=hi)


def worstcase_headway(law, ka=KA0, kp=KP0, kv=KV0, tau0=TAU0, theta=0.0,
                      lo=0.3, hi=6.0, omega=CERT_OMEGA) -> float:
    """Deterministic worst-case headway: ``A(w) <= 1`` over the whole support.

    Stabilises *both* tails (the true robust design); for ``theta = 0`` this
    matches the paper's closed form :func:`cacc.analysis.cthp_h_lb`.
    """
    return _bisect_h(
        lambda h: _max_amp(h, law, ka, kp, kv, tau0, theta, omega) <= 1.0,
        lo, hi)


def chance_headway(eps, law, ka=KA0, kp=KP0, kv=KV0, tau0=TAU0, theta=0.0,
                   lo=0.3, hi=6.0, ng=300, omega=CERT_OMEGA) -> float:
    """Risk-parameterised headway ``h_cc(eps)`` with ``exceedance(h) <= eps``.

    Bisects on ``h`` against the exact two-tailed exceedance (no monotonicity
    assumption on ``A(w)``).
    """
    if not 0.0 < eps < 1.0:
        raise ValueError("eps must be in (0, 1)")
    return _bisect_h(
        lambda h: exceedance_prob(h, law, ka, kp, kv, tau0, theta, ng, omega) <= eps,
        lo, hi)


def chebyshev_headway(eps, law, ka=KA0, kp=KP0, kv=KV0, tau0=TAU0, theta=0.0,
                      lo=0.3, hi=6.0, omega=CERT_OMEGA) -> float:
    """Distribution-free ``h_cheb(eps)`` via a two-sided Chebyshev interval.

    ``P(|w - E[w]| >= t) <= sigma^2 / t^2``; setting the RHS to ``eps`` gives
    ``t = sigma / sqrt(eps)``. The design is made string-stable for every ``w``
    in ``[E[w] - t, E[w] + t]`` (clamped to the support), which guarantees
    ``exceedance <= eps`` using only the mean and variance — conservative and
    robust to uncertainty in the ``gamma_j``.
    """
    if not 0.0 < eps < 1.0:
        raise ValueError("eps must be in (0, 1)")
    t = law.std / np.sqrt(eps)
    wlo = max(law.mean - t, law.support[0])
    whi = min(law.mean + t, law.support[1])

    def ok(h):
        return max(_amp(wlo, h, ka, kp, kv, tau0, theta, omega),
                   _amp(whi, h, ka, kp, kv, tau0, theta, omega)) <= 1.0

    return _bisect_h(ok, lo, hi)


def meansquare_headway(law, ka=KA0, kp=KP0, kv=KV0, tau0=TAU0, theta=0.0,
                       lo=0.3, hi=6.0, ng=300, omega=CERT_OMEGA) -> float:
    """Mean-square headway ``h_ms`` with ``E[A(w)^2] <= 1`` (bisection).

    Companion certificate — see the module docstring caveat: mean-square is
    *too permissive* for this channel to be robustly string-stable.
    """
    return _bisect_h(
        lambda h: mean_square_amp(h, law, ka, kp, kv, tau0, theta, ng, omega) <= 1.0,
        lo, hi)


def certify_design(h, law, ka=KA0, kp=KP0, kv=KV0, tau0=TAU0, theta=0.0,
                   ng=300, omega=CERT_OMEGA) -> dict:
    """Risk summary of a given headway ``h`` on the channel law."""
    wlo, whi = law.support
    return {
        "h": float(h),
        "rho": law.rho,
        "exceedance": exceedance_prob(h, law, ka, kp, kv, tau0, theta, ng, omega),
        "mean_square_amp": mean_square_amp(h, law, ka, kp, kv, tau0, theta, ng, omega),
        "hinf_worst": hinf_norm("cthp", h, omega=omega, kp=kp, tau=tau0, kv=kv,
                                ka_eff=wlo * ka, theta=theta),
        "hinf_mean": hinf_norm("cthp", h, omega=omega, kp=kp, tau=tau0, kv=kv,
                               ka_eff=law.mean * ka, theta=theta),
    }
