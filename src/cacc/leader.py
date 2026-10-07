"""Leader acceleration profiles ``a0(t)`` built from scenario mappings.

Moved out of :mod:`cacc.platoon` (which re-exports
:func:`make_leader_profile`) to keep the simulator module small.
"""

from __future__ import annotations

from typing import Callable

import numpy as np


def make_leader_profile(spec: dict) -> Callable[[float], float]:
    """Build a leader acceleration profile ``a0(t)`` from a scenario dict.

    Supported profiles:
      * ``brake``: constant ``decel`` for ``duration`` seconds from ``t_start``
      * ``sine``: ``amplitude * sin(2*pi*freq_hz*(t - t_start))`` from
        ``t_start``, optionally only for ``duration`` seconds
      * ``constant``: fixed ``value``
    """
    kind = spec["profile"].lower()
    if kind == "brake":
        t0 = float(spec.get("t_start", 5.0))
        dur = float(spec.get("duration", 2.0))
        dec = float(spec.get("decel", -4.0))
        base = lambda t: dec if t0 <= t < t0 + dur else 0.0  # noqa: E731
    elif kind == "sine":
        t0 = float(spec.get("t_start", 5.0))
        amp = float(spec.get("amplitude", 1.0))
        f = float(spec.get("freq_hz", 0.1))
        dur = spec.get("duration")  # seconds of sine; None = forever
        t1 = np.inf if dur is None else t0 + float(dur)
        base = (lambda t: amp * np.sin(2 * np.pi * f * (t - t0))  # noqa: E731
                if t0 <= t < t1 else 0.0)
    elif kind == "constant":
        val = float(spec.get("value", 0.0))
        base = lambda t: val  # noqa: E731
    elif kind == "bursts":
        # several sine bursts: [[t_start, duration, amplitude, freq_hz], ...]
        bursts = [tuple(float(v) for v in b) for b in spec["bursts"]]

        def base(t: float, _b=bursts) -> float:
            for t0, dur, amp, f in _b:
                if t0 <= t < t0 + dur:
                    return amp * np.sin(2 * np.pi * f * (t - t0))
            return 0.0
    else:
        raise ValueError(f"unknown leader profile: {kind!r}")
    return _with_probe(base, spec)


def _with_probe(base: Callable[[float], float], spec: dict) -> Callable[[float], float]:
    """Superpose a small multi-sine probing dither on a leader profile.

    Channel-noise estimation needs persistent excitation (w-samples exist
    only while the communicated acceleration is nonzero, D-016); a
    ``probe_amplitude`` of a few cm/s^2 keeps the estimators fed without
    disturbing the platoon meaningfully.
    """
    amp = float(spec.get("probe_amplitude", 0.0))
    if amp <= 0.0:
        return base
    w1, w2 = 2 * np.pi * 0.31, 2 * np.pi * 0.73  # incommensurate tones
    return lambda t: base(t) + 0.5 * amp * (np.sin(w1 * t) + np.sin(w2 * t))
