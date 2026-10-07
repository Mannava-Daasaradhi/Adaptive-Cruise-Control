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

  Two impairments that V2X engineers actually measure can be added (sampled
  mode only; fix of 2026-10-08):

  - ``delay_jitter`` [s]: each message's latency is ``delay + X`` with X
    exponential of mean ``delay_jitter`` (queueing / channel-access
    contention), so ``delay`` is the minimum latency. Messages may arrive
    out of order; the receiver keeps the newest by send time and discards
    stale ones, as real stacks do.
  - ``loss_burst`` [messages]: bursty (Gilbert-Elliott) loss instead of
    independent loss. A two-state Markov chain (good: delivered, bad:
    lost) with mean bad-burst length ``loss_burst`` and the same long-run
    loss rate ``loss_prob``. Bursts, not the average rate, are what open
    long packet inter-reception gaps.

  ``stats()`` reports messages sent / delivered and the worst packet
  inter-reception time (PIR), the quantity V2X latency budgets are written in.

**Multiplicative value noise** (Ma, Pagilla, Darbha 2025, eqs. (4)-(5)). Note
what it is and is not: in a digital radio (802.11p, C-V2X) a corrupted packet
fails its CRC and is dropped, so bit errors do not arrive as scaled values.
Read this term as error in the *transmitted value* (the sender's sensor and
quantisation), the interpretation under which the paper's model is physical;
packet-level impairments are delay, jitter and loss above. With ``noise_rho = rho`` the delivered value is scaled by

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

import bisect
import logging
from dataclasses import dataclass
from typing import Sequence

import numpy as np

log = logging.getLogger(__name__)

#: channel-bit expectations gamma_{i,j} used in Ma et al. 2025, Sec. IV (n=16)
MA2025_GAMMAS: tuple[float, ...] = (
    0.8055, 0.5767, 0.1829, 0.2399, 0.8865, 0.0287, 0.4899, 0.1679,
    0.9787, 0.7127, 0.5005, 0.4711, 0.0596, 0.6820, 0.0424, 0.0714,
)


@dataclass(frozen=True)
class LinkAttack:
    """A malicious injection on one V2V link (D-022 threat model).

    Models a compromised / impersonating node that tampers with the delivered
    feedforward over a time window ``[t0, t1)``. The attack is applied to the
    receiver's value *after* channel noise, i.e. it is what actually reaches
    the follower's controller.

    Attributes
    ----------
    kind:
        ``'bias'`` adds ``value`` (a phantom acceleration); ``'override'``
        drives the value toward the constant ``value`` (a fully forged
        message); ``'scale'`` multiplies by ``value`` (a gain attack).
    t0, t1:
        Active window [s); ``t1`` defaults to +inf.
    value:
        Attack amount (units per ``kind``).
    ramp:
        Linear fade-in [s] from ``t0`` (0 = step). A ramp keeps the spoof from
        being a trivially detectable discontinuity.
    target_link:
        0-based index of the attacked link (link ``i`` feeds follower ``i``).
    """

    kind: str = "bias"
    t0: float = 0.0
    t1: float = float("inf")
    value: float = 0.0
    ramp: float = 0.0
    target_link: int = 0

    def active(self, t: float) -> bool:
        return self.t0 <= t < self.t1

    def apply(self, t: float, raw: float) -> float:
        """Corrupt the delivered value ``raw`` at time ``t``."""
        if not self.active(t):
            return raw
        a = 1.0 if self.ramp <= 0.0 else min(1.0, (t - self.t0) / self.ramp)
        if self.kind == "bias":
            return raw + a * self.value
        if self.kind == "override":
            return (1.0 - a) * raw + a * self.value
        if self.kind == "scale":
            return raw * (1.0 + a * (self.value - 1.0))
        raise ValueError(f"unknown attack kind: {self.kind!r}")


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
        attack: LinkAttack | None = None,
        delay_jitter: float = 0.0,
        loss_burst: float | None = None,
    ):
        if delay < 0:
            raise ValueError("delay must be >= 0")
        if not 0.0 <= loss_prob <= 1.0:
            raise ValueError("loss_prob must be in [0, 1]")
        if loss_prob > 0 and msg_rate is None:
            raise ValueError("packet loss requires sampled mode (set msg_rate)")
        if delay_jitter < 0:
            raise ValueError("delay_jitter must be >= 0")
        if delay_jitter > 0 and msg_rate is None:
            raise ValueError("delay_jitter requires sampled mode (set msg_rate)")
        self._ge = None  # Gilbert-Elliott transition probabilities (p_gb, p_bg)
        if loss_burst is not None:
            if msg_rate is None:
                raise ValueError("loss_burst requires sampled mode (set msg_rate)")
            if loss_burst < 1.0:
                raise ValueError("loss_burst is a mean burst length in "
                                 "messages and must be >= 1")
            if not 0.0 < loss_prob < 1.0:
                raise ValueError("loss_burst needs 0 < loss_prob < 1 (the "
                                 "long-run loss rate)")
            p_bg = 1.0 / float(loss_burst)
            p_gb = loss_prob * p_bg / (1.0 - loss_prob)
            if p_gb > 1.0:
                raise ValueError(f"loss_prob={loss_prob} is unreachable with "
                                 f"mean bursts of {loss_burst} messages")
            self._ge = (p_gb, p_bg)
        self.delay_jitter = float(delay_jitter)
        self.loss_burst = loss_burst
        if noise_rho is not None and noise_rho <= 1.0:
            raise ValueError("noise_rho must be > 1 (rho -> inf = no noise)")
        self.delay = float(delay)
        self.loss_prob = float(loss_prob)
        self.msg_rate = msg_rate
        self.initial = float(initial)
        self.attack = attack  # malicious injection on this link (D-022)
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
        # (arrival_time, send_time, value), kept sorted by arrival time
        self._pending: list[tuple[float, float, float]] = []
        self._held = self.initial
        self._held_sent = -np.inf  # send time of the held message
        self._ge_bad = False  # Gilbert-Elliott channel state
        # separate stream so enabling jitter never shifts loss draws
        self._jit_rng = np.random.default_rng([seed, 0x717])
        self._n_sent = 0
        self._n_delivered = 0
        self._last_arrival: float | None = None
        self._max_pir = 0.0

    @property
    def passthrough(self) -> bool:
        """True when the link is transparent (zero delay, continuous,
        lossless, noiseless, un-attacked). The simulator then wires the live
        value directly, avoiding interpolation at the current time instant.
        An attacked link is never passthrough — the spoof rides ``receive``."""
        return (self.delay == 0.0 and self.msg_rate is None
                and self.noise_rho is None and self.attack is None)

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
                ts = self._next_sample_t
                self._n_sent += 1
                if not self._lost():
                    arrival = ts + self.delay
                    if self.delay_jitter > 0.0:
                        arrival += self._jit_rng.exponential(self.delay_jitter)
                        bisect.insort(self._pending, (arrival, ts, value))
                    else:  # FIFO: arrivals are already in order
                        self._pending.append((arrival, ts, value))
                self._next_sample_t += period

    def _lost(self) -> bool:
        """Draw the fate of one message (independent or Gilbert-Elliott)."""
        if self._ge is None:
            return self._rng.random() < self.loss_prob
        p_gb, p_bg = self._ge
        u = self._rng.random()
        self._ge_bad = (u >= p_bg) if self._ge_bad else (u < p_gb)
        return self._ge_bad

    def stats(self) -> dict:
        """Messages sent / delivered and the worst packet inter-reception
        time [s] seen by the receiver so far (sampled mode)."""
        return {"sent": self._n_sent, "delivered": self._n_delivered,
                "max_pir_s": self._max_pir}

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
            # sampled mode: deliver everything that has arrived by t, keeping
            # the newest message by send time (stale arrivals are discarded)
            while self._pending and self._pending[0][0] <= t:
                arrival, sent, value = self._pending.pop(0)
                if sent > self._held_sent:
                    self._held, self._held_sent = value, sent
                    if self._last_arrival is not None:
                        self._max_pir = max(self._max_pir,
                                            arrival - self._last_arrival)
                    self._last_arrival = arrival
                    self._n_delivered += 1
            raw = self._held
        val = self.noise_at(t) * raw
        if self.attack is not None:  # malicious injection (D-022), post-noise
            val = self.attack.apply(t, val)
        return val

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
