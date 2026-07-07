"""Online V2V channel-quality estimation and headway adaptation (D-016).

The QoS-adaptive extension of the Ma-2025 design closes the loop between
channel sensing and control design:

* :class:`ChannelEstimator` — each follower estimates the multiplicative
  noise level rho of ITS OWN link, using only receiver-side quantities: the
  received (noisy, delayed) feedforward ``y = w * a(t - theta)`` and the
  radar-measured predecessor acceleration looked up ``theta_hat`` in the
  past. Their ratio is a sample of ``w``; since ``|w - 1| <= 1/rho``
  *strictly* (the n-bit channel has bounded support), the windowed maximum
  of ``|w - 1|`` under-estimates ``1/rho`` and therefore OVER-estimates
  rho — i.e. assumes the channel is better than it is. The adapter divides
  by ``rho_safety`` to make the design conservative instead.

* :class:`HeadwayAdapter` — slews the follower's time headway h(t) toward
  ``margin + h_req(rho_hat_safe, theta_hat)`` with a rate limit. The target
  comes from the paper's closed form h_lb(ka, rho) (eq. 17) at theta = 0
  and from a precomputed ``min_stable_headway`` lookup (worst case over the
  noise interval, predictor on/off) when a feedforward delay is present.
  The rate limit keeps h(t) slowly varying, so string stability of the
  frozen designs (guaranteed for every h >= h_lb + margin) carries over
  quasi-statically; each hop's transfer function depends only on its own
  follower's h, so per-vehicle heterogeneous headways are admissible.
"""

from __future__ import annotations

import logging
from collections import deque
from dataclasses import dataclass

import numpy as np

from cacc.analysis import min_stable_headway

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class AdaptConfig:
    """Knobs of the QoS-adaptive outer loop (scenario ``adaptation:`` block)."""

    enabled: bool = False
    est_rate: float = 25.0  # [Hz] estimator/adapter update rate
    window: int = 256  # max w-samples kept
    max_age_s: float = 20.0  # [s] w-samples older than this are dropped
    a_min: float = 0.03  # [m/s^2] min |radar accel| to accept a w-sample
    rho_safety: float = 1.2  # rho_hat is divided by this (conservative)
    rho_min: float = 1.5
    rho_max: float = 50.0
    margin: float = 0.08  # [s] headway margin above the stability bound
    rate: float = 0.05  # [s/s] max |dh/dt| slew rate
    h_max: float = 2.5  # [s] adaptation ceiling
    predictor: bool = False  # timestamp-based feedforward lead (D-016)
    pred_base: float = 0.4  # [s] predictor slope baseline


class ChannelEstimator:
    """Windowed estimate of the link's noise level rho from w-samples."""

    def __init__(self, cfg: AdaptConfig):
        self.cfg = cfg
        self._dev = deque(maxlen=cfg.window)  # (t, |w - 1|) samples
        self._rho = None  # last estimate (None until enough excitation)

    @property
    def n_samples(self) -> int:
        return len(self._dev)

    def add_sample(self, t: float, y_received: float,
                   a_radar_delayed: float) -> None:
        """Feed one receiver-side observation pair at time ``t``.

        ``y_received`` = noisy delayed feedforward; ``a_radar_delayed`` =
        radar accel of the predecessor at the same (timestamp-aligned)
        instant. Pairs with too little excitation are rejected — during
        cruise the channel is simply unobservable (hence the leader's
        probing dither in the scenarios).
        """
        if abs(a_radar_delayed) < self.cfg.a_min:
            return
        w = y_received / a_radar_delayed
        if not 0.2 < w < 1.8:  # physically impossible for rho > 1.25: reject
            return
        self._dev.append((t, abs(w - 1.0)))

    def rho_hat(self, t: float | None = None) -> float | None:
        """Point estimate of rho (None until >= 25 fresh samples).

        Samples older than ``max_age_s`` are expired first (a count-based
        window recovers far too slowly after an interference zone when the
        excitation is weak); the 99th percentile of the remaining |w - 1|
        is inverted — an over-estimate of rho by construction (support is
        strictly bounded), made conservative by the adapter's rho_safety.
        """
        if t is not None:
            while self._dev and t - self._dev[0][0] > self.cfg.max_age_s:
                self._dev.popleft()
        if len(self._dev) < 25:
            return self._rho
        d = float(np.quantile(np.asarray([d for _, d in self._dev]), 0.99))
        if d > 1e-6:
            self._rho = float(np.clip(1.0 / d, self.cfg.rho_min, self.cfg.rho_max))
        return self._rho


class HeadwayAdapter:
    """Rate-limited headway adaptation h(t) -> margin + h_req(rho_hat, theta).

    The target is the *fixed-gain* requirement: a ``min_stable_headway``
    lookup table over (rho, theta), worst case over both ends of the noise
    interval (the LOW end k_a_eff = (1 - 1/rho) ka usually binds, D-009),
    for the predictor setting in use. Note this is deliberately NOT the
    paper's closed form h_lb(ka, rho) (eq. 17): that bound presumes (kp,
    kv) re-tuned inside the feasible region *for each rho*, whereas an
    operating platoon adapts h with its gains fixed — the fixed-gain
    requirement is what the running controller actually needs (and exceeds
    h_lb wherever the fixed gains leave the eq.-29 region; e.g. case-A
    kv = 0.63 is infeasible at rho = 2). ``cthp_h_lb`` remains the
    re-tuned-design reference in the theory figures.
    """

    RHO_GRID = np.array([1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 7.0, 10.0, 15.0, 25.0, 50.0])
    THETA_GRID = np.array([0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30])

    def __init__(self, ka: float, kp: float, kv: float, tau0: float,
                 cfg: AdaptConfig, h0: float, theta: float = 0.0,
                 table: np.ndarray | None = None):
        self.ka, self.kp, self.kv, self.tau0 = ka, kp, kv, tau0
        self.cfg = cfg
        self.h = float(h0)
        self.theta = float(theta)
        self._table = table  # share one lookup across a platoon's adapters
        if self._table is None:
            self._table = self._build_table()

    def _build_table(self) -> np.ndarray:
        c = self.cfg
        pred = c.pred_base if c.predictor else 0.0
        tab = np.empty((self.RHO_GRID.size, self.THETA_GRID.size))
        for i, rho in enumerate(self.RHO_GRID):
            for j, th in enumerate(self.THETA_GRID):
                h_req = 0.0
                for end in (1.0 - 1.0 / rho, 1.0 + 1.0 / rho):  # both ends
                    try:
                        h_req = max(h_req, min_stable_headway(
                            "cthp", theta=th, kp=self.kp, tau=self.tau0,
                            kv=self.kv, ka_eff=end * self.ka,
                            pred_theta_hat=(th if c.predictor else 0.0),
                            pred_base=max(pred, 1e-3)))
                    except ValueError:  # no stable h at all: ceiling applies
                        h_req = np.inf
                tab[i, j] = h_req
        # infeasible cells (h-only adaptation cannot stabilize these rho:
        # the gamma upper bound of eq. 28a is h-independent) are capped to
        # a large finite value so interpolation stays finite; update() then
        # saturates at h_max — the adapter's honest best effort
        tab = np.minimum(tab, 10.0)
        log.info("headway lookup table built: rho x theta = %s", tab.shape)
        return tab

    def h_required(self, rho: float) -> float:
        """Fixed-gain required headway at (rho, self.theta), before margin."""
        rho_c = float(np.clip(rho, self.RHO_GRID[0], self.RHO_GRID[-1]))
        th_c = float(np.clip(self.theta, 0.0, self.THETA_GRID[-1]))
        i = int(np.searchsorted(self.RHO_GRID, rho_c) - 1)
        i = max(0, min(i, self.RHO_GRID.size - 2))
        j = int(np.searchsorted(self.THETA_GRID, th_c) - 1)
        j = max(0, min(j, self.THETA_GRID.size - 2))
        fr = (rho_c - self.RHO_GRID[i]) / (self.RHO_GRID[i + 1] - self.RHO_GRID[i])
        ft = (th_c - self.THETA_GRID[j]) / (self.THETA_GRID[j + 1] - self.THETA_GRID[j])
        t = self._table
        return float((1 - fr) * (1 - ft) * t[i, j] + fr * (1 - ft) * t[i + 1, j]
                     + (1 - fr) * ft * t[i, j + 1] + fr * ft * t[i + 1, j + 1])

    def update(self, dt: float, rho_hat: float | None) -> float:
        """Advance h(t) one estimator period toward the current target."""
        c = self.cfg
        if rho_hat is None:  # channel not yet observable: hold
            return self.h
        rho_safe = max(1.05, rho_hat / c.rho_safety)
        target = min(c.h_max, c.margin + self.h_required(rho_safe))
        step = np.clip(target - self.h, -c.rate * dt, c.rate * dt)
        self.h = float(self.h + step)
        return self.h
