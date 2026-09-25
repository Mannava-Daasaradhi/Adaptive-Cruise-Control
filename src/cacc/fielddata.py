"""Field data: recorded platoon drive logs (D-026).

A :class:`PlatoonLog` is the measured counterpart of a simulated run: time,
each vehicle's speed, each follower's bumper-to-bumper gap, and whether the
follower's automation was engaged. Two on-disk formats are read:

**OpenACC** (JRC open database of commercial-ACC car-following experiments,
CC BY 4.0). Layout as consumed by the published ULTra-AV processing code:
five metadata rows (the second one names each vehicle's make/model), then a
header row starting with ``Time`` and columns ``Speed{i}`` [m/s] (vehicle 1 =
platoon leader), ``IVS{i}`` [m] (inter-vehicle spacing between vehicles i
and i+1, bumper to bumper) and optionally ``Driver{i}`` (``ACC`` / ``Human``);
10 Hz sampling. The parser locates the header by its ``Time`` cell rather
than a fixed row count, so small layout differences between campaigns do not
break it.

**Generic** (for a customer's own logs; also what :func:`write_log`
produces)::

    time,speed_0,speed_1,...,speed_n,gap_1,...,gap_n[,engaged_1,...,engaged_n]

``speed_0`` is the leader, ``gap_i`` is the bumper gap ahead of follower
``i``, ``engaged_i`` is 1 while follower ``i``'s ACC was active.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


@dataclass
class PlatoonLog:
    """A recorded platoon: uniform time base, speeds, gaps, engagement."""

    t: np.ndarray  # (S,) [s], uniform
    speed: np.ndarray  # (S, N) [m/s]; column 0 = platoon leader
    gap: np.ndarray  # (S, N-1) [m]; column i-1 = gap ahead of vehicle i
    names: list[str]  # N vehicle labels
    engaged: np.ndarray | None = None  # (S, N) bool: automation active
    source: str = ""
    meta: dict = field(default_factory=dict)

    def __post_init__(self):
        s, n = self.speed.shape
        if self.t.shape != (s,) or self.gap.shape != (s, n - 1):
            raise ValueError("inconsistent log shapes: t (S,), speed (S, N), "
                             "gap (S, N-1) required")
        if len(self.names) != n:
            raise ValueError("need one name per vehicle")
        if n < 2:
            raise ValueError("a platoon log needs at least two vehicles")

    @property
    def dt(self) -> float:
        return float(np.median(np.diff(self.t)))

    @property
    def n_vehicles(self) -> int:
        return self.speed.shape[1]

    def hop_segments(self, i: int, min_duration: float = 30.0
                     ) -> list[slice]:
        """Contiguous sample ranges usable for follower ``i`` (1-based).

        A sample is usable when the leader speed, the follower speed and the
        gap are finite and, if engagement is recorded, the follower's ACC is
        on. Segments shorter than ``min_duration`` seconds are dropped.
        """
        if not 1 <= i < self.n_vehicles:
            raise ValueError(f"follower index must be in 1..{self.n_vehicles - 1}")
        ok = (np.isfinite(self.speed[:, i - 1]) & np.isfinite(self.speed[:, i])
              & np.isfinite(self.gap[:, i - 1]))
        if self.engaged is not None:
            ok &= self.engaged[:, i]
        segs, start = [], None
        for k, flag in enumerate(np.append(ok, False)):
            if flag and start is None:
                start = k
            elif not flag and start is not None:
                if (k - start) * self.dt >= min_duration:
                    segs.append(slice(start, k))
                start = None
        return segs


# ------------------------------------------------------------------ OpenACC
def _num(cell: str) -> float:
    try:
        return float(cell)
    except (TypeError, ValueError):
        return math.nan


def read_openacc(path: str | Path) -> PlatoonLog:
    """Read one OpenACC platoon CSV file."""
    path = Path(path)
    with open(path, encoding="utf-8-sig", newline="") as f:
        rows = list(csv.reader(f))
    try:
        h = next(k for k, r in enumerate(rows)
                 if r and r[0].strip().lower() == "time")
    except StopIteration:
        raise ValueError(f"{path}: no header row starting with 'Time'") from None
    header = [c.strip() for c in rows[h]]
    col = {name: j for j, name in enumerate(header)}
    n = 0
    while f"Speed{n + 1}" in col:
        n += 1
    if n < 2:
        raise ValueError(f"{path}: need Speed1, Speed2, ... columns")
    body = [r for r in rows[h + 1:] if r and r[0].strip()]
    t = np.array([_num(r[col["Time"]]) for r in body])

    def column(name: str) -> np.ndarray:
        j = col.get(name)
        return np.array([_num(r[j]) if j < len(r) else math.nan
                         for r in body]) if j is not None else np.full(t.size, np.nan)

    speed = np.column_stack([column(f"Speed{i}") for i in range(1, n + 1)])
    gap = np.column_stack([column(f"IVS{i}") for i in range(1, n)])
    engaged = None
    if all(f"Driver{i}" in col for i in range(1, n + 1)):
        engaged = np.column_stack([
            np.array([(r[col[f"Driver{i}"]].strip().upper() == "ACC")
                      if col[f"Driver{i}"] < len(r) else False for r in body])
            for i in range(1, n + 1)])
    meta = {r[0].strip(): [c.strip() for c in r[1:] if c.strip()]
            for r in rows[:h] if r and r[0].strip()}
    names_row = rows[1][1:] if h >= 2 else []
    names = [c.strip() for c in names_row if c.strip()][:n]
    names += [f"veh{i}" for i in range(len(names) + 1, n + 1)]
    return _uniform(PlatoonLog(t, speed, gap, names, engaged, str(path), meta))


# ------------------------------------------------------------------ generic
def read_log(path: str | Path) -> PlatoonLog:
    """Read a log in the generic format, or OpenACC if it looks like one."""
    path = Path(path)
    with open(path, encoding="utf-8-sig", newline="") as f:
        first = f.readline().strip().lower()
    if not first.startswith("time,speed_0"):
        return read_openacc(path)
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
    cols = data.dtype.names
    n = sum(c.startswith("speed_") for c in cols)
    speed = np.column_stack([data[f"speed_{i}"] for i in range(n)])
    gap = np.column_stack([data[f"gap_{i}"] for i in range(1, n)])
    engaged = None
    if all(f"engaged_{i}" in cols for i in range(1, n)):
        engaged = np.column_stack(
            [np.zeros(len(data), bool)]
            + [data[f"engaged_{i}"] > 0.5 for i in range(1, n)])
    names = ["leader"] + [f"follower_{i}" for i in range(1, n)]
    return _uniform(PlatoonLog(np.asarray(data["time"]), speed, gap, names,
                               engaged, str(path)))


def write_log(log: PlatoonLog, path: str | Path) -> None:
    """Write ``log`` in the generic CSV format."""
    n = log.n_vehicles
    head = (["time"] + [f"speed_{i}" for i in range(n)]
            + [f"gap_{i}" for i in range(1, n)])
    cols = [log.t, *log.speed.T, *log.gap.T]
    if log.engaged is not None:
        head += [f"engaged_{i}" for i in range(1, n)]
        cols += [log.engaged[:, i].astype(float) for i in range(1, n)]
    np.savetxt(path, np.column_stack(cols), delimiter=",",
               header=",".join(head), comments="", fmt="%.6g")


def write_openacc(log: PlatoonLog, path: str | Path) -> None:
    """Write ``log`` in the OpenACC layout (used for tests and demos)."""
    n = log.n_vehicles
    mode = (log.engaged if log.engaged is not None
            else np.ones_like(log.speed, dtype=bool))
    with open(path, "w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Vehicle order"] + [str(i) for i in range(1, n + 1)])
        w.writerow(["Vehicle make/model"] + list(log.names))
        w.writerow(["Source"] + [log.source or "synthetic"] * n)
        w.writerow(["Speed unit", "m/s"])
        w.writerow(["Spacing unit", "m"])
        w.writerow(["Time"] + [f"Speed{i}" for i in range(1, n + 1)]
                   + [f"IVS{i}" for i in range(1, n)]
                   + [f"Driver{i}" for i in range(1, n + 1)])
        for k in range(log.t.size):
            w.writerow([f"{log.t[k]:.2f}"]
                       + [f"{x:.4f}" for x in log.speed[k]]
                       + [f"{x:.4f}" for x in log.gap[k]]
                       + ["ACC" if m else "Human" for m in mode[k]])


def _uniform(log: PlatoonLog) -> PlatoonLog:
    """Drop rows without a time stamp and resample to a uniform grid."""
    keep = np.isfinite(log.t)
    t = log.t[keep]
    order = np.argsort(t)
    t = t[order]
    speed, gap = log.speed[keep][order], log.gap[keep][order]
    engaged = None if log.engaged is None else log.engaged[keep][order]
    d = np.diff(t)
    if d.size and np.allclose(d, np.median(d), rtol=1e-3, atol=1e-6):
        return PlatoonLog(t, speed, gap, log.names, engaged, log.source, log.meta)
    step = float(np.median(d[d > 0]))
    tu = np.arange(t[0], t[-1] + 0.5 * step, step)

    def interp(x):
        return np.column_stack([np.interp(tu, t, c, left=np.nan, right=np.nan)
                                for c in x.T])

    eng = None
    if engaged is not None:  # nearest-sample engagement flag
        idx = np.clip(np.searchsorted(t, tu), 0, t.size - 1)
        eng = engaged[idx]
    return PlatoonLog(tu, interp(speed), interp(gap), log.names, eng,
                      log.source, dict(log.meta, resampled_from=float(step)))


def log_from_sim(res, engaged: bool = True) -> PlatoonLog:
    """Convert a :class:`cacc.platoon.SimResult` into a :class:`PlatoonLog`."""
    gap = np.diff(-res.pos, axis=1) - res.config.vehicle.length
    n = res.vel.shape[1]
    eng = np.ones((res.t.size, n), bool) if engaged else None
    if eng is not None:
        eng[:, 0] = False  # the leader is driven, not automated
    return PlatoonLog(res.t.copy(), res.vel.copy(), gap, ["leader"] + [
        f"follower_{i}" for i in range(1, n)], eng, f"sim:{res.controller}")
