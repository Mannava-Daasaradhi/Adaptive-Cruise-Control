"""Bring-your-own-controller plugin interface (D-025).

A plugin is any class (or factory) that the simulator can drive exactly like
the built-in ACC / CACC / CTHP laws. It is referenced by a string

    'package.module:Name'        importable module (installed or on sys.path)
    'path/to/file.py:Name'       a plain Python file, no packaging needed

and constructed as ``Name(params, **options)`` where ``params`` is the
scenario's :class:`cacc.controllers.ControllerParams` (spacing policy h, r and
the standard gains) and ``options`` is the free-form ``options:`` mapping of
the test plan.

The contract is :class:`LongitudinalController`. The simulator integrates the
plugin's internal states (``n_states``) with the same RK4 step as the plant,
saturates its output to the vehicle's actuator limits, and passes it through
the actuator lag. Controllers that need more than the linear spacing-policy
signals (for example an IDM-style law that works on the raw gap and speed)
set ``raw_inputs = True`` and receive the measurements as keyword arguments.
"""

from __future__ import annotations

import hashlib
import importlib
import importlib.util
import sys
from pathlib import Path
from typing import Protocol, runtime_checkable

import numpy as np


@runtime_checkable
class LongitudinalController(Protocol):
    """What the simulator needs from a follower's longitudinal controller.

    Attributes
    ----------
    n_states:
        Number of internal controller states ``xc`` the simulator integrates
        (0 for a static law).
    uses_v2v:
        True if the law consumes the predecessor's broadcast over V2V.
    ff_signal:
        Which predecessor signal the V2V link carries: ``'a'`` (realized
        acceleration) or ``'u'`` (commanded acceleration).

    Optional attributes / hooks
    ---------------------------
    raw_inputs (bool, default False):
        When True, ``output`` and ``deriv`` also receive the keyword
        arguments ``gap`` (bumper-to-bumper [m]), ``v`` (ego speed [m/s]),
        ``a`` (ego acceleration [m/s^2]) and ``t`` (time [s]).
    equilibrium_gap(v) -> float:
        Bumper-to-bumper gap [m] the law holds at steady speed ``v``. The
        platoon is initialised in that equilibrium, so time-domain criteria
        are not polluted by a start-up transient. Without it the scenario's
        spacing policy ``r + h*v`` is used.

    Signals passed positionally to ``output`` / ``deriv``: ``xc`` (internal
    state), ``e`` (spacing error w.r.t. the scenario policy ``r + h*v``),
    ``e_dot``, ``u_ff`` (V2V value as delivered by the link; 0 when
    ``uses_v2v`` is False) and ``dv`` (predecessor minus ego speed).
    """

    n_states: int
    uses_v2v: bool
    ff_signal: str

    def output(self, xc: np.ndarray, e: float, e_dot: float, u_ff: float,
               dv: float = 0.0, **raw) -> float:
        """Commanded acceleration [m/s^2] (saturated by the simulator)."""
        ...

    def deriv(self, xc: np.ndarray, e: float, e_dot: float, u_ff: float,
              dv: float = 0.0, **raw) -> np.ndarray:
        """Time derivative of the internal state (length ``n_states``)."""
        ...


def _import_target(ref: str):
    """Resolve ``'module:Name'`` or ``'file.py:Name'`` to the named object."""
    mod_ref, _, attr = ref.rpartition(":")
    if not mod_ref or not attr:
        raise ValueError(f"plugin reference {ref!r} must look like "
                         "'package.module:Name' or 'path/to/file.py:Name'")
    if mod_ref.endswith(".py"):
        path = Path(mod_ref).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"plugin file not found: {path}")
        # one module object per file: repeated loads share class identity
        digest = hashlib.sha1(str(path).encode()).hexdigest()[:12]
        mod_name = f"cacc_plugin_{path.stem}_{digest}"
        module = sys.modules.get(mod_name)
        if module is None:
            spec = importlib.util.spec_from_file_location(mod_name, path)
            module = importlib.util.module_from_spec(spec)
            sys.modules[mod_name] = module
            try:
                spec.loader.exec_module(module)
            except BaseException:
                del sys.modules[mod_name]
                raise
    else:
        module = importlib.import_module(mod_ref)
    try:
        return getattr(module, attr)
    except AttributeError:
        raise ValueError(f"plugin module {mod_ref!r} has no attribute "
                         f"{attr!r}") from None


def validate_controller(ctrl, ref: str = "controller") -> None:
    """Raise ``TypeError`` with a precise message if ``ctrl`` breaks the
    :class:`LongitudinalController` contract."""
    problems = []
    n = getattr(ctrl, "n_states", None)
    if not isinstance(n, (int, np.integer)) or isinstance(n, bool) or n < 0:
        problems.append("n_states must be a non-negative int")
    if not isinstance(getattr(ctrl, "uses_v2v", None), bool):
        problems.append("uses_v2v must be a bool")
    if getattr(ctrl, "ff_signal", None) not in ("a", "u"):
        problems.append("ff_signal must be 'a' or 'u'")
    for name in ("output", "deriv"):
        if not callable(getattr(ctrl, name, None)):
            problems.append(f"missing method {name}(xc, e, e_dot, u_ff, dv)")
    gap = getattr(ctrl, "equilibrium_gap", None)
    if gap is not None and not callable(gap):
        problems.append("equilibrium_gap must be callable: v -> gap [m]")
    if problems:
        raise TypeError(f"{ref} does not satisfy the LongitudinalController "
                        "protocol: " + "; ".join(problems))


def load_plugin_controller(ref: str, params, options: dict):
    """Construct and validate a plugin controller from its reference."""
    target = _import_target(ref)
    ctrl = target(params, **options)
    validate_controller(ctrl, ref)
    return ctrl
