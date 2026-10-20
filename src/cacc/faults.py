"""Sensor and powertrain faults injected into chosen cars for a time window.

Scenario block (a list; ``cars`` are follower numbers, empty = every one)::

    faults:
      - {kind: radar-dropout, cars: [3], t0: 10.0, duration: 2.0}
      - {kind: radar-bias,    cars: [3], value: 3.0}     # reads 3 m too far
      - {kind: radar-noise,   cars: [3], value: 1.0}     # 1 m std, 20 Hz
      - {kind: v2v-dropout,   t0: 10.0, duration: 3.0}   # no messages at all
      - {kind: brake-fade,    cars: [2], value: 0.5}     # half the brake force
      - {kind: motor-derate,  value: 0.4}                # 40 % of motor force
      - {kind: stuck-brake,   cars: [2], value: 1.5}     # 1.5 m/s^2 drag

What each one does to the car (the truth the safety metrics see is never
faulted, only what the car measures or can do):

* ``radar-dropout``: the radar track freezes at its last value (gap and
  relative speed), the classic stale-target failure;
* ``radar-bias`` / ``radar-noise``: the measured gap is offset by ``value``
  metres, or gets Gaussian noise of ``value`` m std held at 20 Hz; the
  relative speed is unaffected (Doppler);
* ``v2v-dropout``: the car receives no V2V message, feedforward 0;
* ``brake-fade``: only ``value`` (0..1) of the braking is available
  (overheated or failing brakes);
* ``motor-derate``: only ``value`` (0..1) of the traction (thermal derate,
  limp-home mode);
* ``stuck-brake``: a dragging brake pulls ``value`` m/s^2 off whatever the
  powertrain delivers.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

KINDS = ("radar-dropout", "radar-bias", "radar-noise", "v2v-dropout",
         "brake-fade", "motor-derate", "stuck-brake")
NOISE_RATE = 20.0  # [Hz] radar-noise hold rate


@dataclass(frozen=True)
class Fault:
    kind: str
    cars: tuple[int, ...] = ()
    t0: float = 10.0
    duration: float | None = 3.0  # None = until the end of the run
    value: float = 0.0

    def __post_init__(self):
        if self.kind not in KINDS:
            raise ValueError(f"unknown fault kind {self.kind!r}; use one of "
                             f"{', '.join(KINDS)}")
        if self.duration is not None and self.duration <= 0.0:
            raise ValueError("fault duration must be > 0 (or null)")
        if self.kind in ("brake-fade", "motor-derate") and not 0.0 <= self.value <= 1.0:
            raise ValueError(f"{self.kind} value is the share left, 0..1")
        if self.kind in ("radar-noise", "stuck-brake") and self.value < 0.0:
            raise ValueError(f"{self.kind} value must be >= 0")

    @classmethod
    def from_dict(cls, raw: dict) -> "Fault":
        extra = set(raw) - {"kind", "cars", "t0", "duration", "value"}
        if extra:
            raise ValueError(f"unknown fault key(s) {sorted(extra)}")
        d = dict(raw)
        d["cars"] = tuple(int(c) for c in d.get("cars") or ())
        return cls(**d)

    def on(self, t: float) -> bool:
        return t >= self.t0 and (self.duration is None or t < self.t0 + self.duration)


def parse(raw) -> tuple[Fault, ...]:
    if isinstance(raw, dict):
        raw = [raw]
    return tuple(f if isinstance(f, Fault) else Fault.from_dict(f) for f in raw or ())


class Faults:
    """Runtime fault state of one simulation."""

    def __init__(self, faults: tuple[Fault, ...], n: int, t_final: float, seed: int):
        for f in faults:
            bad = [c for c in f.cars if not 1 <= c <= n]
            if bad:
                raise ValueError(f"fault {f.kind}: car(s) {bad} not in 1..{n}")
        self.faults = faults
        self.by_car = [[f for f in faults if not f.cars or i + 1 in f.cars]
                       for i in range(n)]
        self.held = np.full((n, 2), np.nan)  # last good (gap, closing)
        rng = np.random.default_rng(seed + 4242)
        self.noise = rng.standard_normal((n, int(t_final * NOISE_RATE) + 2))

    @classmethod
    def build(cls, config) -> "Faults | None":
        fs = getattr(config, "faults", ())
        return cls(fs, config.n_followers, config.t_final, config.seed) if fs else None

    def _active(self, i: int, t: float, kind: str):
        for f in self.by_car[i]:
            if f.kind == kind and f.on(t):
                return f
        return None

    def sense(self, i: int, t: float, gap: float, closing: float) -> tuple[float, float]:
        """What car ``i``'s radar reports: ``(gap, closing)``."""
        if self._active(i, t, "radar-dropout") is not None and np.isfinite(self.held[i, 0]):
            return float(self.held[i, 0]), float(self.held[i, 1])
        f = self._active(i, t, "radar-bias")
        if f is not None:
            gap += f.value
        f = self._active(i, t, "radar-noise")
        if f is not None:
            k = min(int(t * NOISE_RATE), self.noise.shape[1] - 1)
            gap += f.value * self.noise[i, k]
        return gap, closing

    def record(self, i: int, t: float, gap: float, closing: float) -> None:
        """Accepted step: remember the last good radar track."""
        if self._active(i, t, "radar-dropout") is None:
            self.held[i] = gap, closing

    def v2v_ok(self, i: int, t: float) -> bool:
        return self._active(i, t, "v2v-dropout") is None

    def bounds(self, i: int, t: float, lo: float, hi: float) -> tuple[float, float]:
        f = self._active(i, t, "brake-fade")
        if f is not None:
            lo *= f.value
        f = self._active(i, t, "motor-derate")
        if f is not None and hi > 0.0:
            hi *= f.value
        return lo, max(hi, lo)

    def drag(self, i: int, t: float, v: float) -> float:
        f = self._active(i, t, "stuck-brake")
        return f.value if f is not None and v > 0.05 else 0.0
