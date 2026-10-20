"""A controller FMU (FMI 2.0 co-simulation) as a cacc plugin.

Export the controller from its home tool, then point a plan or campaign at
it::

    under_test:
      controller: cacc.adapters.fmu:FMUController
      options: {fmu: controllers/acc.fmu, output: u_cmd,
                inputs: {gap: gap_m, v: ego_speed, dv: rel_speed},
                parameters: {time_gap: 1.6}}

    cacc stress -c cacc.adapters.fmu:FMUController --option fmu=acc.fmu

Simulink: *Simulink Coder / Compiler → Export → Standalone FMU*
(Co-Simulation, FMI 2.0) on the controller subsystem. Modelica tools,
CarMaker and most HIL suites export the same format.

The FMU is sampled every ``step`` seconds with zero-order hold: at each
sample the measured signals are written to its inputs, the FMU advances one
step and its output becomes the command until the next sample (so the
result carries the one-step latency a real ECU task would). Each follower
gets its own FMU instance. Needs ``pip install fmpy``.

Signals available to ``inputs`` (cacc name → FMU variable name): ``e``,
``e_dot``, ``dv``, ``u_ff`` (V2V value), ``gap``, ``v``, ``a``, ``t``. By
default every FMU input whose name matches a cacc signal is connected.
"""

from __future__ import annotations

import numpy as np

SIGNALS = ("e", "e_dot", "dv", "u_ff", "gap", "v", "a", "t")


def _fmpy():
    try:
        import fmpy
        from fmpy.fmi2 import FMU2Slave
    except ImportError:
        raise ImportError("FMU controllers need fmpy: pip install fmpy") from None
    return fmpy, FMU2Slave


class FMUController:
    """Plugin wrapper around one FMU instance (see module docstring)."""

    n_states = 0
    raw_inputs = True

    def __init__(self, params, fmu: str, output: str = "u",
                 inputs: dict | None = None, parameters: dict | None = None,
                 step: float = 0.01, ff_signal: str = "a",
                 uses_v2v: bool | None = None, time_gap: float | None = None,
                 standstill: float | None = None):
        fmpy, slave_cls = _fmpy()
        md = fmpy.read_model_description(fmu)
        if md.coSimulation is None:
            raise ValueError(f"{fmu} is not a co-simulation FMU")
        refs = {v.name: v.valueReference for v in md.modelVariables}
        if output not in refs:
            raise ValueError(f"FMU has no variable {output!r}; outputs: "
                             + ", ".join(v.name for v in md.modelVariables
                                         if v.causality == "output"))
        if inputs is None:
            fmu_in = {v.name for v in md.modelVariables if v.causality == "input"}
            inputs = {s: s for s in SIGNALS if s in fmu_in}
        bad = [s for s in inputs if s not in SIGNALS]
        missing = [n for n in inputs.values() if n not in refs]
        if bad or missing:
            raise ValueError(f"bad FMU input mapping: unknown cacc signals {bad}, "
                             f"unknown FMU variables {missing}")
        if step <= 0:
            raise ValueError("step must be > 0")
        self._names = list(inputs)
        self._in_refs = [refs[inputs[s]] for s in self._names]
        self._out_ref = refs[output]
        self.step = float(step)
        self.ff_signal = ff_signal
        self.uses_v2v = ("u_ff" in inputs) if uses_v2v is None else bool(uses_v2v)
        self.h = params.h if time_gap is None else float(time_gap)
        self.r = params.r if standstill is None else float(standstill)
        self._slave = slave_cls(
            guid=md.guid, unzipDirectory=fmpy.extract(fmu),
            modelIdentifier=md.coSimulation.modelIdentifier,
            instanceName="cacc_controller")
        self._slave.instantiate()
        self._slave.setupExperiment(startTime=0.0)
        self._slave.enterInitializationMode()
        for name, value in (parameters or {}).items():
            if name not in refs:
                raise ValueError(f"FMU has no parameter {name!r}")
            self._slave.setReal([refs[name]], [float(value)])
        self._slave.exitInitializationMode()
        self._tc = 0.0  # FMU communication time
        self._y = 0.0

    def equilibrium_gap(self, v):
        return self.r + self.h * v

    def output(self, xc, e, e_dot, u_ff, dv=0.0, *, gap, v, a, t):
        if t >= self._tc - 1e-9:  # next sample is due
            sig = {"e": e, "e_dot": e_dot, "dv": dv, "u_ff": u_ff, "gap": gap,
                   "v": v, "a": a, "t": t}
            if self._in_refs:
                self._slave.setReal(self._in_refs, [float(sig[s]) for s in self._names])
            self._slave.doStep(currentCommunicationPoint=self._tc,
                               communicationStepSize=self.step)
            self._tc += self.step
            self._y = float(self._slave.getReal([self._out_ref])[0])
        return self._y

    def deriv(self, xc, e, e_dot, u_ff, dv=0.0, **raw):
        return np.empty(0)

    def __del__(self):
        slave = getattr(self, "_slave", None)
        if slave is not None:
            try:
                slave.terminate()
                slave.freeInstance()
            except Exception:  # noqa: BLE001 — interpreter shutdown
                pass
