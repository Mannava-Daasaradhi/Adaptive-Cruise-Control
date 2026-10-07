"""On-board radar estimate of the predecessor's acceleration.

Radar measures range and range-rate, not acceleration. A follower gets the
predecessor's acceleration by differentiating range-rate (plus its own IMU)
and filtering the result, so the estimate is **late and noisy**. Two parts
of this repository consume it:

* the physics-consistency spoof gate (:mod:`cacc.trust`, D-022), which
  compares each V2V message against it;
* the online channel estimator (:mod:`cacc.estimation`, D-016), which
  divides the received V2V value by it to sample the channel noise.

The original studies used an ideal radar (true acceleration, no lag). This
module makes the radar a modelled sensor::

    a_radar(t) = a_prev(t - lag) + noise * n(t)

where ``lag`` is the effective group delay of the differentiation filter
and ``n(t)`` is standard-normal noise held piecewise-constant at
``noise_rate`` Hz, deterministic in ``t`` for a given seed (so the four RK4
stage evaluations of one step see consistent values and reruns are exact).
``lag`` must be 0 or at least one integration step: the delay line is read
from the recorded history of accepted steps.

A radar left at its defaults (lag = 0, noise = 0) is the ideal radar of the
original studies, and every published number is unchanged.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RadarConfig:
    """The follower's radar-based acceleration estimate (scenario ``radar:``).

    Attributes
    ----------
    lag:
        Effective delay of the estimate [s] (differentiation + filtering).
        0 = ideal. Production radars typically need a few hundred
        milliseconds of filtering to get a usable acceleration.
    noise:
        Standard deviation of the estimate's error [m/s^2]. 0 = ideal.
    noise_rate:
        Hold rate [Hz] of the noise process.
    seed:
        Base seed of the noise stream (offset per follower).
    """

    lag: float = 0.0
    noise: float = 0.0
    noise_rate: float = 50.0
    seed: int = 11

    def __post_init__(self):
        if self.lag < 0.0:
            raise ValueError("radar lag must be >= 0")
        if self.noise < 0.0:
            raise ValueError("radar noise must be >= 0")
        if self.noise_rate <= 0.0:
            raise ValueError("radar noise_rate must be > 0")

    @property
    def ideal(self) -> bool:
        """True for the ideal radar of the original studies."""
        return self.lag == 0.0 and self.noise == 0.0


class DelayLine:
    """Recorded scalar history, read back ``delay`` seconds in the past.

    ``record`` is called once per accepted integration step; ``read`` may be
    called at any (non-decreasing) RK4 stage time and interpolates linearly.
    Before any history exists it returns ``initial``.
    """

    def __init__(self, delay: float, initial: float = 0.0):
        if delay < 0.0:
            raise ValueError("delay must be >= 0")
        self.delay = float(delay)
        self.initial = float(initial)
        self._ts = np.empty(1024)
        self._xs = np.empty(1024)
        self._n = 0

    def record(self, t: float, x: float) -> None:
        if self._n == self._ts.size:
            self._ts = np.concatenate([self._ts, np.empty(self._ts.size)])
            self._xs = np.concatenate([self._xs, np.empty(self._xs.size)])
        self._ts[self._n] = t
        self._xs[self._n] = x
        self._n += 1

    def read(self, t: float) -> float:
        q = t - self.delay
        n = self._n
        if n == 0 or q <= self._ts[0]:
            return self.initial if n == 0 else float(self._xs[0])
        if q >= self._ts[n - 1]:
            return float(self._xs[n - 1])
        j = int(np.searchsorted(self._ts[:n], q))
        t0, t1 = self._ts[j - 1], self._ts[j]
        x0, x1 = self._xs[j - 1], self._xs[j]
        return float(x0 + (q - t0) / (t1 - t0) * (x1 - x0))


class RadarSensor:
    """One follower's radar estimate of its predecessor's acceleration."""

    def __init__(self, cfg: RadarConfig, seed_offset: int = 0):
        self.cfg = cfg
        self._line = DelayLine(cfg.lag) if cfg.lag > 0.0 else None
        self._rng = np.random.default_rng([cfg.seed, seed_offset, 0x5AD])
        self._cache = np.empty(0)

    def record(self, t: float, a_prev: float) -> None:
        """Store the predecessor's true acceleration at an accepted step."""
        if self._line is not None:
            self._line.record(t, a_prev)

    def noise_at(self, t: float) -> float:
        """Estimate error at time ``t`` [m/s^2] (deterministic in ``t``)."""
        if self.cfg.noise <= 0.0:
            return 0.0
        k = int(max(t, 0.0) * self.cfg.noise_rate + 1e-9)
        if k >= self._cache.size:
            grow = max(k + 1 - self._cache.size, 512)
            self._cache = np.concatenate(
                [self._cache, self._rng.standard_normal(grow)])
        return self.cfg.noise * float(self._cache[k])

    def estimate(self, t: float, a_prev_now: float) -> float:
        """Radar acceleration estimate at time ``t``.

        ``a_prev_now`` is the predecessor's true acceleration at ``t``; it is
        used directly only by a zero-lag radar.
        """
        base = a_prev_now if self._line is None else self._line.read(t)
        return base + self.noise_at(t)
