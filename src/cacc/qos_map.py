"""Shared spatial V2V channel-quality map and preview lookahead (D-021).

The QoS-adaptive loop of [D-016] reacts to the channel it can *measure now*.
By the time a follower's estimator has seen enough noisy samples to lower its
``rho_hat``, the platoon is already inside the interference patch, and the
rate-limited headway opens *behind* the disturbance — the entry transient is
the price of reacting.

A :class:`QoSMap` is a **geo-referenced** model of channel quality along the
road, ``rho(x)``, pooled from prior traversals / neighbour reports and shared
over V2V (every vehicle carries the same map). It buys two things the reactive
estimator cannot:

* **Ground truth for the simulation** — each link's noise level is driven by
  *where that link is*, so follower ``i`` enters a patch later than the leader
  (by its standoff distance), exactly as on a real road.
* **A preview** — :meth:`min_rho_ahead` returns the worst channel a vehicle
  will meet within ``horizon`` seconds at its current speed, so the adapter can
  open the gap *before* entry (see :class:`cacc.estimation.HeadwayAdapter`).

The map is a static function of position, so it is available from ``t = 0``,
before any estimator has warmed up — the predictive controller never pays the
estimator's observability lag.
"""

from __future__ import annotations

import numpy as np


class QoSMap:
    """Piecewise-constant channel quality ``rho(x)`` over the road.

    Parameters
    ----------
    zones:
        Interference patches ``[(x0, x1, rho), ...]`` (metres, metres, rho>1).
        Outside every patch the channel is ``rho_base``. Overlaps resolve to
        the worst (smallest) rho.
    rho_base:
        Good-channel quality outside all patches (typically the scenario's
        ``noise_rho``).
    """

    def __init__(self, zones, rho_base: float):
        z = [(float(a), float(b), float(r)) for a, b, r in zones]
        if any(b <= a for a, b, _ in z):
            raise ValueError("each QoS zone needs x1 > x0")
        if rho_base <= 1.0 or any(r <= 1.0 for *_, r in z):
            raise ValueError("all rho values must be > 1")
        self.zones = tuple(sorted(z))
        self.rho_base = float(rho_base)

    def rho_at(self, x: float) -> float:
        """True channel quality at longitudinal position ``x``."""
        rho = self.rho_base
        for x0, x1, r in self.zones:
            if x0 <= x < x1:
                rho = min(rho, r)
        return rho

    def min_rho_ahead(self, x: float, v: float, horizon: float) -> float:
        """Worst ``rho`` over the preview window ``[x, x + max(0,v)*horizon]``.

        This is what the vehicle will encounter within ``horizon`` seconds at
        its current speed — the quantity the predictive headway target keys
        on. Includes the current position, so it never under-opens.
        """
        x_end = x + max(0.0, v) * max(0.0, horizon)
        rho = self.rho_at(x)
        for x0, x1, r in self.zones:
            if x1 > x and x0 < x_end:  # patch overlaps the preview window
                rho = min(rho, r)
        return rho

    def time_schedule(self, t_grid: np.ndarray,
                      x_of_t: np.ndarray) -> tuple[tuple[float, float], ...]:
        """Convert a position trajectory into a ``V2VLink`` rho step schedule.

        Given sampled positions ``x_of_t[k]`` at times ``t_grid[k]``, emit
        ``[(t_k, rho_k), ...]`` with one entry per change in ``rho(x)`` — the
        exact zone entry/exit times for a vehicle following that trajectory.
        Always starts at ``t = 0``.
        """
        sched: list[tuple[float, float]] = []
        last = None
        for t, x in zip(t_grid, x_of_t):
            rho = self.rho_at(float(x))
            if last is None or abs(rho - last) > 1e-9:
                sched.append((float(t) if sched else 0.0, rho))
                last = rho
        return tuple(sched)
