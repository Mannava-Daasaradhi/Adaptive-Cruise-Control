"""Physics-consistency V2V trust gate (D-022) — bounded-injection defence.

The CACC/CTHP feedforward ``u_ff`` is the predecessor's acceleration delivered
over the V2V radio. That radio is an *external* input: a compromised or
impersonating node can inject an arbitrary value into it. A raw follower
applies ``ka * u_ff`` directly, so an unbounded spoof drives an unbounded
control command — a phantom-brake or a phantom-throttle that collapses the
gap. String stability says nothing about this: it bounds the response to
*honest* disturbances, not to a lying input.

Every follower, however, already carries a **second, independent** estimate of
its predecessor's acceleration — its radar (range + range-rate, differentiated
with the follower's own IMU). Radar is a separate physical sensor on the ego
vehicle; an attacker sitting on the V2V bus cannot touch it. The gate uses that
independence as a consistency check.

Let ``a_v2v`` be the received feedforward and ``a_radar`` the radar estimate of
the same quantity. Define the inconsistency ``r = |a_v2v - a_radar|`` and the
trust weight

    g(r) = 1 / (1 + (r / r0)^2)   in (0, 1],

then fuse

    u_ff_eff = g * a_v2v + (1 - g) * a_radar.

When the message is physically consistent (r small) g -> 1 and the clean,
un-differentiated V2V value is used unchanged — so the gate is **inert** under
honest operation (it does not perturb the validated string-stability results,
[D-016]/[D-021]). When the message is inconsistent (r large) g -> 0 and the
feedforward falls back to the locally-measured radar value.

**Bounded-injection guarantee (the certificate).** The deviation of the fused
feedforward from the radar-consistent value is

    |u_ff_eff - a_radar| = g(r) * r = r / (1 + (r/r0)^2)  <=  r0 / 2

for every r >= 0, with equality at r = r0 — a hard cap that is *independent of
the spoof magnitude*. Hence the spurious control injected by the CTHP law is at
most ``ka * r0 / 2`` no matter how large the lie, versus ``ka * |spoof|``
(unbounded) for the raw design. This is the security dependent-claim of the
flagship (part B of A + B + certificate).
"""

from __future__ import annotations

import logging
from dataclasses import dataclass

import numpy as np

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class TrustConfig:
    """Knobs of the physics-consistency gate (scenario ``trust:`` block).

    Attributes
    ----------
    enabled:
        Turn the gate on. When off the feedforward is used raw.
    r0:
        Inconsistency scale [m/s^2]. Sets the hard injection cap ``r0 / 2`` and
        the soft knee of the trust curve. Choose it a few times the largest
        *honest* V2V-vs-radar disagreement (channel noise + radar error +
        genuine predecessor jerk) so the gate is transparent to honest traffic
        yet clamps spoofs well below a dangerous command.
    radar_noise:
        Standard deviation [m/s^2] of the independent radar acceleration
        estimate (differentiation noise / range jitter). 0 = ideal radar.
    noise_rate:
        Hold rate [Hz] of the radar-noise process; kept piecewise-constant in
        time (like the channel noise) so RK4 stage evaluations are consistent
        and reruns are reproducible.
    seed:
        Base seed for the radar-noise stream.
    """

    enabled: bool = False
    r0: float = 3.0
    radar_noise: float = 0.0
    noise_rate: float = 50.0
    seed: int = 7


class TrustGate:
    """Per-follower physics-consistency gate; see the module docstring.

    One gate instance guards one follower's incoming feedforward. It holds the
    follower's radar-noise stream (deterministic in ``t`` for a given seed) and
    performs the stateless trust fusion.
    """

    def __init__(self, cfg: TrustConfig, seed_offset: int = 0):
        if cfg.r0 <= 0.0:
            raise ValueError("trust r0 must be > 0")
        if cfg.radar_noise < 0.0:
            raise ValueError("radar_noise must be >= 0")
        self.cfg = cfg
        self.r0 = float(cfg.r0)
        self._rng = np.random.default_rng([cfg.seed, seed_offset, 0x7A9])
        self._cache = np.empty(0)  # per-interval standard-normal radar noise

    # ------------------------------------------------------------ radar model
    def radar_at(self, t: float, a_true: float) -> float:
        """Radar estimate of the predecessor acceleration at time ``t``.

        The radar is an independent, un-spoofable onboard sensor; here it
        observes the true predecessor acceleration corrupted by a
        deterministic-in-``t`` zero-mean noise (0 when ``radar_noise == 0``).
        """
        if self.cfg.radar_noise <= 0.0:
            return float(a_true)
        k = int(t * self.cfg.noise_rate + 1e-9)
        if k >= self._cache.size:
            grow = max(k + 1 - self._cache.size, 512)
            self._cache = np.concatenate(
                [self._cache, self._rng.standard_normal(grow)])
        return float(a_true) + self.cfg.radar_noise * float(self._cache[k])

    # ------------------------------------------------------------------- gate
    def trust(self, r: float) -> float:
        """Trust weight ``g(r) = 1 / (1 + (r/r0)^2)`` in (0, 1]."""
        x = r / self.r0
        return 1.0 / (1.0 + x * x)

    def fuse(self, a_v2v: float, a_radar: float) -> tuple[float, float, float]:
        """Blend V2V toward radar by the trust weight.

        Returns ``(u_ff_eff, g, r)`` where ``r = |a_v2v - a_radar|`` and
        ``|u_ff_eff - a_radar| <= r0 / 2`` holds unconditionally.
        """
        r = abs(a_v2v - a_radar)
        g = self.trust(r)
        u_ff_eff = g * a_v2v + (1.0 - g) * a_radar
        return u_ff_eff, g, r

    def apply(self, t: float, a_v2v: float,
              a_true: float) -> tuple[float, float, float, float]:
        """Full gate step: form the radar estimate then fuse.

        Returns ``(u_ff_eff, g, r, a_radar)``.
        """
        a_radar = self.radar_at(t, a_true)
        u_ff_eff, g, r = self.fuse(a_v2v, a_radar)
        return u_ff_eff, g, r, a_radar
