"""Acceptance criteria for test plans (D-025): run metrics + threshold rules.

Every metric reduces one simulated run (:class:`cacc.platoon.SimResult`) to
the *worst* value over all followers and all time, in engineering units. A
criterion compares a metric against a threshold; across seeds the plan keeps
the worst value (lowest for ``>=`` rules, highest for ``<=`` rules) and the
seed that produced it, so every failure is reproducible as (case, seed).

Criteria are written in a plan either as a string or as a mapping::

    criteria:
      - "min_gap >= 2.0"
      - {metric: min_ttc, op: ">=", value: 1.5, name: "no imminent collision"}
"""

from __future__ import annotations

import math
import operator
import re
from dataclasses import dataclass
from typing import Callable

import numpy as np

from cacc.metrics import accel_rms, amplification_ratios
from cacc.platoon import SimResult

#: speeds below this are ignored by the time-gap metric [m/s]
STANDSTILL_SPEED = 1.0
#: closing speeds below this are ignored by the TTC metric [m/s]
MIN_CLOSING_SPEED = 0.1


def _gaps(res: SimResult) -> np.ndarray:
    """Bumper-to-bumper gaps, shape (S, n)."""
    return np.diff(-res.pos, axis=1) - res.config.vehicle.length


def min_gap(res: SimResult) -> float:
    return float(_gaps(res).min())


def min_time_gap(res: SimResult) -> float:
    v = res.vel[:, 1:]
    moving = v > STANDSTILL_SPEED
    if not moving.any():
        return math.inf
    return float((_gaps(res)[moving] / v[moving]).min())


def min_ttc(res: SimResult) -> float:
    closing = res.vel[:, 1:] - res.vel[:, :-1]  # follower faster than lead
    mask = closing > MIN_CLOSING_SPEED
    if not mask.any():
        return math.inf
    return float((np.maximum(_gaps(res)[mask], 0.0) / closing[mask]).min())


def peak_spacing_error(res: SimResult) -> float:
    return float(np.abs(res.err).max())


def l2_amplification_max(res: SimResult) -> float:
    if res.err.shape[1] < 2:
        return math.nan
    return float(amplification_ratios(res, "l2").max())


def peak_decel(res: SimResult) -> float:
    return float(max(0.0, -res.acc[:, 1:].min()))


def peak_accel(res: SimResult) -> float:
    return float(max(0.0, res.acc[:, 1:].max()))


def peak_jerk(res: SimResult) -> float:
    dt = float(res.t[1] - res.t[0])
    return float(np.abs(np.diff(res.acc[:, 1:], axis=0)).max() / dt)


def rms_accel(res: SimResult) -> float:
    return float(accel_rms(res).max())


@dataclass(frozen=True)
class Metric:
    """A run metric: function, unit, and which direction is worse."""

    fn: Callable[[SimResult], float]
    unit: str
    worse: str  # 'low' (safety margins) or 'high' (errors, loads)
    doc: str


METRICS: dict[str, Metric] = {
    "min_gap": Metric(min_gap, "m", "low",
                      "smallest bumper-to-bumper gap (collision if <= 0)"),
    "min_time_gap": Metric(min_time_gap, "s", "low",
                           "smallest gap / own speed while moving"),
    "min_ttc": Metric(min_ttc, "s", "low",
                      "smallest time-to-collision while closing in"),
    "peak_spacing_error": Metric(peak_spacing_error, "m", "high",
                                 "largest |spacing error| vs the policy"),
    "l2_amplification_max": Metric(l2_amplification_max, "-", "high",
                                   "largest vehicle-to-vehicle L2 error "
                                   "growth (> 1 = amplifying)"),
    "peak_decel": Metric(peak_decel, "m/s^2", "high",
                         "hardest realized braking of any follower"),
    "peak_accel": Metric(peak_accel, "m/s^2", "high",
                         "hardest realized acceleration of any follower"),
    "peak_jerk": Metric(peak_jerk, "m/s^3", "high",
                        "largest |da/dt| of any follower (comfort)"),
    "rms_accel": Metric(rms_accel, "m/s^2", "high",
                        "largest per-follower RMS acceleration "
                        "(comfort / energy proxy)"),
}

_OPS = {"<=": operator.le, ">=": operator.ge, "<": operator.lt,
        ">": operator.gt}
_RULE = re.compile(r"^\s*([a-z_][a-z0-9_]*)\s*(<=|>=|<|>)\s*([-+0-9.eE]+)\s*$")


@dataclass(frozen=True)
class Criterion:
    """One acceptance rule: ``metric op value``."""

    metric: str
    op: str
    value: float
    name: str = ""

    def __post_init__(self):
        if self.metric not in METRICS:
            raise ValueError(f"unknown metric {self.metric!r}; available: "
                             f"{', '.join(METRICS)}")
        if self.op not in _OPS:
            raise ValueError(f"unknown operator {self.op!r}; use one of "
                             f"{', '.join(_OPS)}")

    @property
    def label(self) -> str:
        unit = METRICS[self.metric].unit
        rule = f"{self.metric} {self.op} {self.value:g}" + (
            "" if unit == "-" else f" {unit}")
        return f"{self.name} ({rule})" if self.name else rule

    def passes(self, x: float) -> bool:
        """NaN never passes; +/-inf compare normally (e.g. no closing-in
        event gives ``min_ttc = inf``, which satisfies any ``>=`` rule)."""
        return not math.isnan(x) and bool(_OPS[self.op](x, self.value))

    def worst(self, values: list[float]) -> tuple[float, int]:
        """Worst value across runs and its index (NaN counts as worst)."""
        arr = np.asarray(values, dtype=float)
        if np.isnan(arr).any():
            i = int(np.flatnonzero(np.isnan(arr))[0])
            return math.nan, i
        i = int(arr.argmin() if self.op in (">=", ">") else arr.argmax())
        return float(arr[i]), i

    @classmethod
    def parse(cls, spec) -> "Criterion":
        """From ``"metric op value"`` or ``{metric, op, value[, name]}``."""
        if isinstance(spec, str):
            m = _RULE.match(spec)
            if not m:
                raise ValueError(f"cannot parse criterion {spec!r}; expected "
                                 "e.g. 'min_gap >= 2.0'")
            return cls(m.group(1), m.group(2), float(m.group(3)))
        if isinstance(spec, dict):
            extra = set(spec) - {"metric", "op", "value", "name"}
            if extra:
                raise ValueError(f"unknown criterion keys {sorted(extra)}")
            return cls(spec["metric"], spec["op"], float(spec["value"]),
                       str(spec.get("name", "")))
        raise ValueError(f"criterion must be a string or mapping: {spec!r}")
