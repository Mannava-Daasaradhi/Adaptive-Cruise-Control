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
    adapt_gains: bool = False  # online (kp, kv) re-tuning (D-017)
    gain_rate: float = 0.05  # [1/s] max relative gain slew rate
    kv_frac: float = 0.9  # kv target as a fraction of the eq.-28a ceiling
    rho_lpf_tau: float = 0.0  # [s] asymmetric rho_hat smoothing (D-019); 0 = off
    stagger_s: float = 0.0  # [s] per-follower h-recovery serialization (D-019)
    preview_s: float = 0.0  # [s] predictive QoS-map lookahead horizon (D-021); 0 = off


class ChannelEstimator:
    """Windowed estimate of the link's noise level rho from w-samples."""

    def __init__(self, cfg: AdaptConfig):
        self.cfg = cfg
        self._dev = deque(maxlen=cfg.window)  # (t, |w - 1|) samples
        self._rho = None  # last raw estimate (None until enough excitation)
        self._filt = None  # asymmetric-LPF state (D-019)
        self._filt_t = None  # time of the last filter advance

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
        if len(self._dev) >= 25:
            d = float(np.quantile(np.asarray([d for _, d in self._dev]), 0.99))
            if d > 1e-6:
                self._rho = float(np.clip(1.0 / d, self.cfg.rho_min,
                                          self.cfg.rho_max))
        return self._smooth(t)

    def _smooth(self, t: float | None) -> float | None:
        """Asymmetric low-pass of the raw estimate (D-019, off by default).

        Estimate DROPS (channel worsening) pass through instantly — the
        safety direction must never wait on a filter. Estimate RISES
        (recovery, including the jump when zone-era samples expire) are
        first-order-filtered with time constant ``rho_lpf_tau``, so the
        filtered value rides the lower envelope of the estimator jitter:
        strictly conservative, and it removes the h(t) chatter that the
        steep part of the h_req(rho) map otherwise amplifies. The state
        advances only on timed reads (``t`` given); passive reads
        (recording) return the last filtered value.
        """
        raw = self._rho
        if self.cfg.rho_lpf_tau <= 0.0 or raw is None:
            return raw
        if self._filt is None:
            self._filt, self._filt_t = raw, t
            return raw
        if t is None:
            return self._filt
        dt = max(0.0, t - (self._filt_t if self._filt_t is not None else t))
        self._filt_t = t
        if raw < self._filt:
            self._filt = raw  # fast toward safety
        else:
            self._filt += dt / (self.cfg.rho_lpf_tau + dt) * (raw - self._filt)
        return self._filt


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
        self.stagger_delay = 0.0  # [s] this follower's recovery slot (D-019)
        self._shrink_wait = 0.0  # time a material shrink demand has stood
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

    def update(self, dt: float, rho_hat: float | None,
               h_required: float | None = None,
               rho_preview: float | None = None) -> float:
        """Advance h(t) one estimator period toward the current target.

        ``h_required`` overrides the internal fixed-gain lookup (used when a
        :class:`GainScheduler` supplies the requirement at re-tuned gains).

        ``rho_preview`` is the predictive QoS-map lookahead (D-021): the worst
        channel the follower will meet within the preview horizon. The target
        opens for the WORSE of the measured and previewed channels, so the gap
        is already open on entry. The map is known exactly (no ``rho_safety``
        divide) and available before the estimator warms up, so a pure preview
        (``rho_hat is None``) still drives the headway.
        """
        c = self.cfg
        req = None
        if rho_hat is not None:  # measured (reactive) requirement
            rho_safe = max(1.05, rho_hat / c.rho_safety)
            req = self.h_required(rho_safe) if h_required is None else h_required
        if rho_preview is not None:  # previewed (anticipatory) requirement
            req_prev = self.h_required(max(1.05, float(rho_preview)))
            req = req_prev if req is None else max(req, req_prev)
        if req is None:  # neither channel observable yet: hold
            return self.h
        target = min(c.h_max, c.margin + req)
        # front-first staggered recovery (D-019): a material headway
        # DECREASE waits out this follower's serialization slot so the
        # platoon's gap-closing ramps do not superpose into one long wave;
        # increases (the safety direction) are never gated. The wait clock
        # measures the CURRENT standing episode only — any lapse of the
        # shrink demand resets it, otherwise plateau jitter would pre-earn
        # the slot long before the real recovery front arrives
        if target < self.h - 0.02:
            self._shrink_wait += dt
            if self._shrink_wait < self.stagger_delay:
                target = self.h
        else:
            self._shrink_wait = 0.0
        step = np.clip(target - self.h, -c.rate * dt, c.rate * dt)
        self.h = float(self.h + step)
        return self.h

    @property
    def rho_safe_of(self):
        """Expose the safety mapping for callers pairing a scheduler."""
        return lambda rho_hat: max(1.05, rho_hat / self.cfg.rho_safety)


class GainScheduler:
    """Online (kp, kv) re-tuning for very noisy channels (D-017).

    The h-only adapter hits a wall: the eq.-28a ceiling
    gamma <= (1 - (1 + 1/rho)^2 ka^2) / (2 tau0) is h-independent (gamma =
    kv + h kp only *grows* with h), so once the fixed kv exceeds it — for
    case-A gains below rho ~ 3 — no headway helps much (h_req explodes,
    infeasible below rho* ~ 1.8). The scheduler re-tunes:

        kv(rho) = kv_frac * (1 - (1 + 1/rho)^2 ka^2) / (2 tau0)
        kp(rho) = kp0 * kv(rho) / kv0           (keeps the kp/kv ratio)

    i.e. kv rides at ``kv_frac`` (default 90 %) of the ceiling — a rule
    that *reproduces the paper's own case-A choice at rho = 10*
    (kv(10) = 0.628 vs their 0.63), which is the evidence this is the
    design recipe the paper applied implicitly. h_req at the re-tuned
    gains is bisected per rho (worst case over both noise-interval ends)
    and tabulated once. Gain changes are rate-limited (relative slew
    ``gain_rate`` per second) exactly like the headway, preserving the
    quasi-static string-stability argument.
    """

    RHO_GRID = HeadwayAdapter.RHO_GRID

    def __init__(self, ka: float, kp0: float, kv0: float, tau0: float,
                 cfg: AdaptConfig, theta: float = 0.0,
                 table: np.ndarray | None = None):
        self.ka, self.kp0, self.kv0, self.tau0 = ka, kp0, kv0, tau0
        self.cfg = cfg
        self.theta = float(theta)
        self.kp, self.kv = float(kp0), float(kv0)  # current (slewed) gains
        self._table = table  # rows: (kv_t, kp_t, h_req) per RHO_GRID entry
        if self._table is None:
            self._table = self._build_table()

    def _ceiling(self, rho: float) -> float:
        return (1.0 - (1.0 + 1.0 / rho) ** 2 * self.ka**2) / (2.0 * self.tau0)

    def _build_table(self) -> np.ndarray:
        c = self.cfg
        tab = np.empty((self.RHO_GRID.size, 3))
        for i, rho in enumerate(self.RHO_GRID):
            kv_t = c.kv_frac * self._ceiling(rho)
            kp_t = self.kp0 * kv_t / self.kv0
            h_req = 0.0
            for end in (1.0 - 1.0 / rho, 1.0 + 1.0 / rho):
                try:
                    h_req = max(h_req, min_stable_headway(
                        "cthp", theta=self.theta, kp=kp_t, tau=self.tau0,
                        kv=kv_t, ka_eff=end * self.ka,
                        pred_theta_hat=(self.theta if c.predictor else 0.0),
                        pred_base=max(c.pred_base, 1e-3)))
                except ValueError:
                    h_req = np.inf
            tab[i] = (kv_t, kp_t, min(h_req, 10.0))
        log.info("gain-scheduler table built over %d rho points",
                 self.RHO_GRID.size)
        return tab

    def targets(self, rho: float) -> tuple[float, float, float]:
        """(kv_target, kp_target, h_required) at ``rho`` (interpolated)."""
        rho_c = float(np.clip(rho, self.RHO_GRID[0], self.RHO_GRID[-1]))
        i = int(np.searchsorted(self.RHO_GRID, rho_c) - 1)
        i = max(0, min(i, self.RHO_GRID.size - 2))
        f = (rho_c - self.RHO_GRID[i]) / (self.RHO_GRID[i + 1] - self.RHO_GRID[i])
        row = (1 - f) * self._table[i] + f * self._table[i + 1]
        return float(row[0]), float(row[1]), float(row[2])

    def update(self, dt: float, rho_safe: float) -> tuple[float, float, float]:
        """Slew the live gains toward the targets; returns (kp, kv, h_req)."""
        kv_t, kp_t, h_req = self.targets(rho_safe)
        lim = self.cfg.gain_rate * dt
        self.kv = float(self.kv + np.clip(kv_t - self.kv,
                                          -lim * self.kv0, lim * self.kv0))
        self.kp = float(self.kp + np.clip(kp_t - self.kp,
                                          -lim * self.kp0, lim * self.kp0))
        return self.kp, self.kv, h_req
