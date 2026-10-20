"""Automatic emergency braking (AEB): a safety layer under any controller.

A cruise controller is tuned for comfort and string stability; it is not
allowed to brake harder than it needs to, so it can be outrun by a car that
cuts in close and brakes, or by a stopped car revealed late. AEB watches the
radar alone (no V2V, so a spoofed message cannot switch it off) and takes
over the brake when a collision is imminent. Scenario block::

    aeb:
      enabled: true
      ttc_warn: 2.6     # partial braking below this time-to-collision [s]
      ttc_full: 1.6     # full braking below this TTC [s]
      partial: -4.0     # partial braking level [m/s^2]
      margin: 2.0       # gap to keep at standstill [m]

The layer only ever lowers the command (``u = min(u_controller, u_aeb)``),
so with no threat in sight the controller under test is untouched. Besides
TTC it compares the braking the situation needs with the braking the car
can still deliver (grip, brake faults): partial above 70 %, full above 90 %::

    d_need = max(closing^2 / (2 (gap - margin)),          # car ahead holds speed
                 v^2 / (2 (gap - margin + s_front)))      # both brake to a stop

with ``s_front = v_front^2 / (2 |a_front|)`` the stopping distance of a
braking car ahead. Once triggered it stays on (latched on accepted steps)
until the car is no longer closing in, like a production AEB, so it does
not chatter between braking and releasing.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

#: below this closing speed nothing is imminent [m/s]
MIN_CLOSING = 0.3
#: front braking harder than this counts as "braking to a stop" [m/s^2]
FRONT_BRAKING = 0.5


@dataclass(frozen=True)
class AEBConfig:
    enabled: bool = False
    ttc_warn: float = 2.6
    ttc_full: float = 1.6
    partial: float = -4.0
    margin: float = 2.0

    def __post_init__(self):
        if not 0.0 < self.ttc_full <= self.ttc_warn:
            raise ValueError("aeb: need 0 < ttc_full <= ttc_warn")
        if self.partial >= 0.0:
            raise ValueError("aeb partial braking must be < 0")
        if self.margin < 0.0:
            raise ValueError("aeb margin must be >= 0")


def need(gap: float, closing: float, v: float, v_front: float,
         a_front: float, margin: float) -> float:
    """Deceleration [m/s^2, > 0] that still avoids contact (``inf`` = too late)."""
    room = gap - margin
    if room <= 0.05:
        return math.inf if closing > 0.0 or v > 0.5 else 0.0
    d = closing * closing / (2.0 * room) if closing > 0.0 else 0.0
    if a_front < -FRONT_BRAKING:
        s_front = v_front * v_front / (2.0 * -a_front)
        d = max(d, v * v / (2.0 * (room + s_front)))
    return d


class AEB:
    """Per-follower AEB state for the simulator."""

    def __init__(self, cfg: AEBConfig, n: int):
        self.cfg = cfg
        self.latch = np.zeros(n, dtype=int)  # 0 off, 1 partial, 2 full
        self.level = np.zeros(n, dtype=int)  # level used at the last stage k1

    def level_of(self, i: int, gap: float, closing: float, v: float,
                 v_front: float, a_front: float, lo: float) -> int:
        c = self.cfg
        if closing <= MIN_CLOSING and self.latch[i] == 0:
            return 0
        ttc = gap / closing if closing > MIN_CLOSING else math.inf
        d = need(gap, closing, v, v_front, a_front, c.margin)
        lvl = 0
        if ttc < c.ttc_warn or d > 0.7 * -lo:
            lvl = 1
        if ttc < c.ttc_full or d > 0.9 * -lo:
            lvl = 2
        return max(lvl, int(self.latch[i]))

    def command(self, i: int, u: float, gap: float, closing: float, v: float,
                v_front: float, a_front: float, lo: float) -> float:
        """The controller's command ``u`` after the safety layer."""
        lvl = self.level_of(i, gap, closing, v, v_front, a_front, lo)
        self.level[i] = lvl
        if lvl == 0:
            return u
        if lvl == 2:
            return min(u, lo)
        d = need(gap, closing, v, v_front, a_front, self.cfg.margin)
        return min(u, max(lo, min(self.cfg.partial, -1.3 * d)))

    def observe(self, i: int, closing: float, v: float) -> None:
        """Accepted step: latch the level until the threat is gone."""
        if closing <= 0.0 or v < 0.1:
            self.latch[i] = 0
        else:
            self.latch[i] = max(self.latch[i], self.level[i])


def activations(res) -> list[dict]:
    """AEB interventions of a run: car, start, end and level (1 or 2)."""
    lv = getattr(res, "aeb", None)
    if lv is None:
        return []
    out = []
    for i in range(lv.shape[1]):
        on = lv[:, i] > 0
        k = 0
        while k < on.size:
            if on[k]:
                j = k
                while j < on.size and on[j]:
                    j += 1
                out.append({"car": i + 1, "t0": float(res.t[k]),
                            "t1": float(res.t[j - 1]),
                            "level": int(lv[k:j, i].max())})
                k = j
            else:
                k += 1
    return out
