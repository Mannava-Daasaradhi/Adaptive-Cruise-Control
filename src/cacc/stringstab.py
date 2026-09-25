"""Black-box string-stability measurement by multisine sweep (D-025).

The analytic verdict ``||Gamma(jw)||_inf <= 1`` (:mod:`cacc.analysis`) needs
the controller's transfer function. A customer's controller is a black box,
so this module *measures* the string-stability transfer instead, the same way
field studies measure commercial ACC (speed propagation along a platoon):

1. The leader's acceleration is an *odd* multisine: small tones at odd
   harmonics ``w_k = 2 pi k / T0`` of a base period ``T0`` with Schroeder
   phases (low crest factor) and an equal *speed* amplitude per tone (so
   acceleration grows with w; with equal acceleration the slow tones carry
   ~100x the speed of the fast ones and their distortion swamps the tiny
   high-frequency response at the tail of the platoon). Odd-only lines
   matter for nonlinear controllers:
   second-order distortion (sums/differences of two odd lines) lands on the
   unexcited even lines instead of corrupting a measured tone. With
   consecutive harmonics 1, 2, 3, ... every adjacent pair's difference falls
   on harmonic 1 — that biased an IDM measurement by 60 % during development.
2. The platoon runs for ``settle_periods`` periods, then ``measure_periods``
   more are recorded.
3. Each vehicle's speed is fitted by least squares with one cosine/sine pair
   per tone plus a linear trend, which soaks up what is left of the slow
   start-up transient (CTHP case A has a ~70 s pole).
4. The per-hop transfer at each tone is ``V_i(w_k) / V_{i-1}(w_k)``. For a
   linear homogeneous platoon every signal (speed, acceleration, spacing
   error) propagates with the same Gamma, so speed is the right choice: it is
   the cleanest signal in a drive log too.

The leader-to-first-follower hop is excluded by default (``first_hop = 2``):
the leader realizes its profile exactly while followers lag, so that hop is
not a homogeneous one for every controller family (e.g. Ploeg CACC, which
broadcasts the commanded acceleration).

**Honest limits.**

* The peak over the measured tones is a lower bound on the true
  ``||Gamma||_inf``: a resonance narrower than the tone spacing, or below the
  lowest tone (``2 pi / T0``), can be missed. The default grid (0.031 -
  3.0 rad/s) is dense where the string-unstable designs of this repository
  peak and matches the analytic Gamma of all three built-in families to
  < 1e-4 (``tests/test_stringstab.py``).
* The result is the small-signal (linearized) string stability around the
  cruise speed; a nonlinear law is measured at the operating point only.
* Under multiplicative channel noise the sweep measures the *average*
  channel, not the worst case: case A at rho = 3 measures stable (0.997)
  although the robust bound (Ma 2025, eq. 17) needs h >= 1.15 s. Use the
  analytic robust certificate (:mod:`cacc.certificate`) or the Monte-Carlo
  time-domain criteria for worst-case channel claims.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, replace
from typing import Callable

import numpy as np

from cacc.platoon import PlatoonConfig, PlatoonSim

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class SweepConfig:
    """Excitation and measurement settings of a string-stability sweep.

    Attributes
    ----------
    base_period:
        Common period ``T0`` [s] of the multisine; tones sit at ``k * 2pi/T0``.
    harmonics:
        Integer harmonic numbers ``k`` of the tones (odd by default; see the
        module docstring for why).
    amplitude:
        Per-tone leader speed amplitude [m/s] (tone acceleration amplitude
        is ``amplitude * w_k``).
    settle_periods:
        Periods discarded before the estimate.
    measure_periods:
        Periods recorded. ``None`` (default) = automatic: 1 for a
        deterministic channel (the estimate is then exact to ~1e-4), 4 for a
        stochastic one (channel noise, loss, radar noise), whose
        period-to-period spread gives the standard error ``gain_se``.
    n_followers:
        Platoon size for the sweep (>= ``first_hop``).
    first_hop:
        Index ``i`` of the first measured hop ``(i-1) -> i``; 2 skips the
        non-homogeneous leader hop.
    detrend_order:
        Degree of the polynomial trend fitted alongside the tones.
    """

    base_period: float = 200.0
    harmonics: tuple[int, ...] = (1, 3, 5, 7, 9, 11, 13, 15, 19, 23, 29, 37,
                                  47, 59, 75, 95)
    amplitude: float = 0.01
    settle_periods: int = 1
    measure_periods: int | None = None
    n_followers: int = 3
    first_hop: int = 2
    detrend_order: int = 1

    def __post_init__(self):
        if self.base_period <= 0 or self.amplitude <= 0:
            raise ValueError("base_period and amplitude must be > 0")
        if not self.harmonics or min(self.harmonics) < 1:
            raise ValueError("harmonics must be positive integers")
        if len(set(self.harmonics)) != len(self.harmonics):
            raise ValueError("harmonics must be distinct")
        if self.settle_periods < 0 or (self.measure_periods is not None
                                       and self.measure_periods < 1):
            raise ValueError("need settle_periods >= 0 and measure_periods >= 1")
        if not 1 <= self.first_hop <= self.n_followers:
            raise ValueError("need 1 <= first_hop <= n_followers")

    @property
    def omegas(self) -> np.ndarray:
        """Tone frequencies [rad/s], ascending."""
        return 2 * np.pi * np.sort(np.asarray(self.harmonics)) / self.base_period

    def periods_for(self, config: PlatoonConfig) -> int:
        """Measured periods for ``config`` (resolves the automatic default)."""
        if self.measure_periods is not None:
            return self.measure_periods
        return 4 if is_stochastic(config) else 1


def is_stochastic(cfg: PlatoonConfig) -> bool:
    """True when the run contains random disturbances (channel noise, packet
    loss, radar noise), i.e. a single period's estimate carries noise."""
    return (cfg.noise_rho is not None or cfg.rho_schedule is not None
            or cfg.qos_map is not None or cfg.loss_prob > 0.0
            or (cfg.trust.enabled and cfg.trust.radar_noise > 0.0))


def multisine(sweep: SweepConfig) -> Callable[[float], float]:
    """Leader acceleration: Schroeder-phased tones of equal speed amplitude."""
    w = sweep.omegas
    amp = sweep.amplitude * w  # accel amplitude giving equal speed amplitude
    # Schroeder's phases for a non-flat power spectrum p_l (low crest factor):
    # phi_k = -2 pi sum_{l<k} (k - l) p_l
    p = amp**2 / np.sum(amp**2)
    k = np.arange(w.size)
    phase = -2 * np.pi * np.array([np.sum((i - k[:i]) * p[:i]) for i in k])

    def a0(t: float) -> float:
        return float((amp * np.sin(w * t + phase)).sum())

    return a0


def tone_phasors(t: np.ndarray, x: np.ndarray, omegas: np.ndarray,
                 detrend_order: int = 1) -> np.ndarray:
    """Least-squares complex amplitudes of ``x`` at ``omegas``.

    ``x`` has shape (S,) or (S, m); the result has shape (F,) or (F, m) with
    ``x ~ Re(X_k e^{j w_k t}) + poly(t)``.
    """
    t = np.asarray(t, dtype=float)
    tn = (t - t.mean()) / max(np.ptp(t), 1e-12) * 2  # well-conditioned trend
    cols = [np.cos(np.outer(t, omegas)), np.sin(np.outer(t, omegas)),
            np.vander(tn, detrend_order + 1, increasing=True)]
    basis = np.hstack(cols)
    coef, *_ = np.linalg.lstsq(basis, x, rcond=None)
    f = omegas.size
    # x = c cos + s sin  =  Re((c - j s) e^{jwt})
    return coef[:f] - 1j * coef[f:2 * f]


@dataclass
class StringStabilityResult:
    """Measured per-hop string-stability transfer."""

    omega: np.ndarray  # (F,) tone frequencies [rad/s]
    transfer: np.ndarray  # (H, F) complex V_i / V_{i-1} per measured hop
    hops: tuple[int, ...]  # follower index i of each hop (i-1 -> i)
    gain_se: np.ndarray | None = None  # (H, F) standard error (>= 2 periods)
    periods: int = 1

    @property
    def gain(self) -> np.ndarray:
        """|Gamma| per hop and tone, shape (H, F)."""
        return np.abs(self.transfer)

    @property
    def peak_gain(self) -> float:
        """Largest measured |Gamma| over hops and tones (lower bound on the
        true H-infinity norm)."""
        return float(self.gain.max())

    @property
    def peak_omega(self) -> float:
        """Tone frequency [rad/s] at which the peak gain occurs."""
        return float(self.omega[np.argmax(self.gain.max(axis=0))])

    @property
    def peak_gain_se(self) -> float | None:
        """Standard error of the peak gain (None for a single period)."""
        if self.gain_se is None:
            return None
        h, f = np.unravel_index(np.argmax(self.gain), self.gain.shape)
        return float(self.gain_se[h, f])

    def marginal(self, max_gain: float = 1.0, k: float = 2.0) -> bool:
        """True when the verdict is within ``k`` standard errors of the
        limit, i.e. not statistically resolved by this measurement."""
        se = self.peak_gain_se
        return se is not None and abs(self.peak_gain - max_gain) <= k * se

    def string_stable(self, max_gain: float = 1.0) -> bool:
        """True when every measured per-hop gain is <= ``max_gain``."""
        return self.peak_gain <= max_gain

    def to_dict(self) -> dict:
        """JSON-serializable digest (gain curve per hop)."""
        return {
            "omega_rad_s": [round(float(w), 5) for w in self.omega],
            "hops": list(self.hops),
            "gain": [[round(float(g), 5) for g in row] for row in self.gain],
            "peak_gain": round(self.peak_gain, 5),
            "peak_omega_rad_s": round(self.peak_omega, 5),
            "peak_gain_se": (None if self.peak_gain_se is None
                             else round(self.peak_gain_se, 6)),
            "periods": self.periods,
        }


def measure_string_stability(
    config: PlatoonConfig,
    controller: str,
    sweep: SweepConfig = SweepConfig(),
    controller_options: dict | None = None,
) -> StringStabilityResult:
    """Run a multisine sweep on ``config`` and measure the per-hop transfer.

    The configuration's vehicle, controller gains and channel are used as-is;
    the leader maneuver is replaced by the multisine and any V2V attack is
    removed (string stability concerns honest operation).
    """
    n_per = sweep.periods_for(config)
    cfg = replace(config, n_followers=sweep.n_followers, attack=None,
                  t_final=(sweep.settle_periods + n_per) * sweep.base_period)
    res = PlatoonSim(cfg, controller, multisine(sweep), controller_options).run()
    seg = int(round(sweep.base_period / cfg.dt))
    k0 = sweep.settle_periods * seg
    # one phasor set per period (F, n+1); averaging the phasors before the
    # ratio is the standard multisine noise reduction, and the spread of the
    # per-period ratios estimates the standard error
    per = np.stack([
        tone_phasors(res.t[k0 + p * seg:k0 + (p + 1) * seg],
                     res.vel[k0 + p * seg:k0 + (p + 1) * seg],
                     sweep.omegas, sweep.detrend_order) for p in range(n_per)])
    V = per.mean(axis=0)
    hops = tuple(range(sweep.first_hop, sweep.n_followers + 1))
    transfer = np.stack([V[:, i] / V[:, i - 1] for i in hops])
    se = None
    if n_per >= 2:
        g = np.stack([np.abs(per[:, :, i] / per[:, :, i - 1]) for i in hops],
                     axis=1)  # (P, H, F)
        se = g.std(axis=0, ddof=1) / np.sqrt(n_per)
    out = StringStabilityResult(sweep.omegas, transfer, hops, se, n_per)
    log.info("string-stability sweep (%s): peak |Gamma| = %.4f at %.3f rad/s",
             controller, out.peak_gain, out.peak_omega)
    return out
