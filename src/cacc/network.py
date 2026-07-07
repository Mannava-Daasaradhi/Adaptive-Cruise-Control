"""V2V communication link: latency, sampling/packet loss, channel noise.

One directed link carries the predecessor's acceleration signal to its
follower. Two transport modes:

* **continuous** (``msg_rate=None``): the receiver sees ``x(t - delay)``,
  reconstructed by linear interpolation of the transmitted history. Idealized
  but deterministic — the right mode for clean string-stability studies.
* **sampled** (``msg_rate`` in Hz): the sender broadcasts at a fixed message
  rate (e.g. 10 Hz beacons as in IEEE 802.11p / DSRC); each message is lost
  independently with probability ``loss_prob``; the receiver zero-order-holds
  the latest message that has arrived (arrival = sample time + delay).

**Multiplicative channel noise** (Ma, Pagilla, Darbha 2025, eqs. (4)-(5)):
with ``noise_rho = rho`` the delivered value is scaled by

    w(t) = (1 - 1/rho) + (1/rho) * sum_{j=0}^{n-1} z_j(t) / 2^j,

where the ``z_j`` are independent Bernoulli(gamma_j) bits of an n-bit
channel, so ``w(t)`` lies in ``[1 - 1/rho, 1 + 1/rho)``. The paper's 16
channel expectations are the default :data:`MA2025_GAMMAS`. ``w`` is held
constant over intervals of ``1/noise_rate`` seconds and is a deterministic
function of time for a given seed, so RK4 stage evaluations within a step
are consistent and reruns are reproducible.

``receive`` must be called with non-decreasing ``t`` (satisfied by the
fixed-step integrator in :mod:`cacc.platoon`).
"""

from __future__ import annotations

import logging
from typing import Sequence

import numpy as np

log = logging.getLogger(__name__)

#: channel-bit expectations gamma_{i,j} used in Ma et al. 2025, Sec. IV (n=16)
MA2025_GAMMAS: tuple[float, ...] = (
    0.8055, 0.5767, 0.1829, 0.2399, 0.8865, 0.0287, 0.4899, 0.1679,
    0.9787, 0.7127, 0.5005, 0.4711, 0.0596, 0.6820, 0.0424, 0.0714,
)


class V2VLink:
    """Directed predecessor-to-follower communication channel."""

    def __init__(
        self,
        delay: float = 0.1,
        loss_prob: float = 0.0,
        msg_rate: float | None = None,
        seed: int = 0,
        initial: float = 0.0,
        noise_rho: float | None = None,
        noise_gammas: Sequence[float] | None = None,
        noise_rate: float = 100.0,
        rho_schedule: Sequence[tuple[float, float]] | None = None,
    ):
        if delay < 0:
            raise ValueError("delay must be >= 0")
        if not 0.0 <= loss_prob <= 1.0:
            raise ValueError("loss_prob must be in [0, 1]")
        if loss_prob > 0 and msg_rate is None:
            raise ValueError("packet loss requires sampled mode (set msg_rate)")
        if noise_rho is not None and noise_rho <= 1.0:
            raise ValueError("noise_rho must be > 1 (rho -> inf = no noise)")
        self.delay = float(delay)
        self.loss_prob = float(loss_prob)
        self.msg_rate = msg_rate
        self.initial = float(initial)
        self._rng = np.random.default_rng(seed)
        # --- channel noise state (Ma 2025 multiplicative model) ---
        self.noise_rho = noise_rho
        self.noise_rate = float(noise_rate)
        # step schedule [(t_k, rho_k), ...]: rho(t) = rho_k for t >= t_k
        # (time-varying channel quality, e.g. an interference zone)
        self.rho_schedule = None
        if rho_schedule is not None:
            sched = sorted((float(a), float(b)) for a, b in rho_schedule)
            if any(r <= 1.0 for _, r in sched):
                raise ValueError("scheduled rho values must be > 1")
            self.rho_schedule = tuple(sched)
            if noise_rho is None:
                self.noise_rho = sched[0][1]
        if self.noise_rho is not None:
            gammas = MA2025_GAMMAS if noise_gammas is None else tuple(noise_gammas)
            if not all(0.0 < g < 1.0 for g in gammas):
                raise ValueError("noise gammas must lie in (0, 1)")
            self._gammas = np.asarray(gammas)
            self._bit_weights = 2.0 ** (-np.arange(len(gammas)))
            # separate RNG stream so enabling noise never shifts loss draws
            self._noise_rng = np.random.default_rng([seed, 0xCACC])
            self._uu = np.empty(0)  # cached U' = sum z_j 2^-j per interval
        # continuous-mode history (grown geometrically, filled prefix used)
        self._ts = np.empty(1024)
        self._us = np.empty(1024)
        self._n = 0
        # sampled-mode state
        self._next_sample_t = 0.0
        self._pending: list[tuple[float, float]] = []  # (arrival_time, value)
        self._held = self.initial

    @property
    def passthrough(self) -> bool:
        """True when the link is transparent (zero delay, continuous,
        lossless, noiseless). The simulator then wires the live value
        directly, avoiding interpolation at the current time instant."""
        return self.delay == 0.0 and self.msg_rate is None and self.noise_rho is None

    # ------------------------------------------------------------- noise
    def rho_at(self, t: float) -> float | None:
        """Channel quality rho at time ``t`` (constant unless scheduled)."""
        if self.rho_schedule is None:
            return self.noise_rho
        rho = self.rho_schedule[0][1]
        for tk, rk in self.rho_schedule:
            if t >= tk:
                rho = rk
            else:
                break
        return rho

    def noise_at(self, t: float) -> float:
        """Multiplicative channel-noise factor w(t) (1.0 when disabled).

        Deterministic in ``t`` for a given seed: the bit pattern
        U'(t) = sum z_j 2^-j is piecewise-constant on intervals of
        ``1/noise_rate`` s, drawn lazily per interval; w is then formed with
        the (possibly scheduled) rho at ``t``, so the same seed produces
        comparable noise across different rho settings.
        """
        if self.noise_rho is None:
            return 1.0
        k = int(t * self.noise_rate + 1e-9)
        if k >= self._uu.size:  # extend the cache to cover interval k
            need = k + 1 - self._uu.size
            grow = max(need, 1024)
            bits = self._noise_rng.random((grow, self._gammas.size)) < self._gammas
            self._uu = np.concatenate([self._uu, bits @ self._bit_weights])
        rho = self.rho_at(t)
        return float((1.0 - 1.0 / rho) + self._uu[k] / rho)

    # ---------------------------------------------------------- transport
    def send(self, t: float, value: float) -> None:
        """Record the sender's value at time ``t`` (call once per sim step)."""
        if self.msg_rate is None:
            if self._n == self._ts.size:  # grow storage
                self._ts = np.concatenate([self._ts, np.empty(self._ts.size)])
                self._us = np.concatenate([self._us, np.empty(self._us.size)])
            self._ts[self._n] = t
            self._us[self._n] = value
            self._n += 1
        else:
            period = 1.0 / self.msg_rate
            while t >= self._next_sample_t:
                if self._rng.random() >= self.loss_prob:
                    self._pending.append((self._next_sample_t + self.delay, value))
                self._next_sample_t += period

    def receive(self, t: float) -> float:
        """Value visible to the receiver at time ``t`` (noise applied last)."""
        if self.msg_rate is None:
            q = t - self.delay
            n = self._n
            if n == 0 or q <= self._ts[0]:
                raw = self.initial
            elif q >= self._ts[n - 1]:
                raw = float(self._us[n - 1])
            else:
                j = int(np.searchsorted(self._ts[:n], q))
                t0, t1 = self._ts[j - 1], self._ts[j]
                u0, u1 = self._us[j - 1], self._us[j]
                w = (q - t0) / (t1 - t0)
                raw = float(u0 + w * (u1 - u0))
        else:
            # sampled mode: deliver everything that has arrived by t
            while self._pending and self._pending[0][0] <= t:
                self._held = self._pending.pop(0)[1]
            raw = self._held
        return self.noise_at(t) * raw

    def receive_predicted(
        self, t: float, theta_hat: float, base: float = 0.4, n_avg: int = 8,
    ) -> float:
        """Delay-compensated received value (continuous mode only).

        The receiver extrapolates the (noisy, delayed) feedforward forward
        by its timestamp-estimated delay ``theta_hat`` using a slope taken
        from two window-averaged blocks of its own received samples — all
        quantities a real receiver has. Averaging over ``base/2`` windows
        (``n_avg`` samples each) suppresses the multiplicative-noise
        contribution to the slope; the result is a first-order lead
        e^{-theta s} -> e^{-theta s} (1 + theta_hat s L(s)) with L the
        averaging low-pass (see docs/decisions/D-016).
        """
        y_now = self.receive(t)
        if theta_hat <= 0.0 or self.msg_rate is not None or t < base:
            return y_now
        half = 0.5 * base
        qs1 = np.linspace(t - half, t, n_avg)
        qs0 = np.linspace(t - base, t - half, n_avg)
        m1 = float(np.mean([self.receive(q) for q in qs1]))
        m0 = float(np.mean([self.receive(q) for q in qs0]))
        slope = (m1 - m0) / half
        return y_now + theta_hat * slope
