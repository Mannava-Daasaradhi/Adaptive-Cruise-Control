"""Adapters: FMU controller (fake fmpy), CARLA driver side, ROS 2 params."""

import sys
import types

import numpy as np
import pytest
import yaml

from cacc.controllers import ControllerParams


# ------------------------------------------------------------------- FMU
class _Var:
    def __init__(self, name, ref, causality):
        self.name, self.valueReference, self.causality = name, ref, causality


class _Slave:
    """A P-controller 'FMU': u_cmd = k * e, with k set as a parameter."""

    def __init__(self, **kw):
        self.vals, self.steps = {}, []

    def instantiate(self):
        pass

    def setupExperiment(self, startTime):
        pass

    def enterInitializationMode(self):
        pass

    def exitInitializationMode(self):
        pass

    def terminate(self):
        pass

    def freeInstance(self):
        pass

    def setReal(self, refs, vals):
        self.vals.update(zip(refs, vals))

    def doStep(self, currentCommunicationPoint, communicationStepSize):
        self.steps.append(currentCommunicationPoint)
        self.vals[9] = self.vals.get(3, 1.0) * self.vals.get(1, 0.0)

    def getReal(self, refs):
        return [self.vals.get(r, 0.0) for r in refs]


@pytest.fixture
def fake_fmpy(monkeypatch):
    md = types.SimpleNamespace(
        guid="g", coSimulation=types.SimpleNamespace(modelIdentifier="m"),
        modelVariables=[_Var("e", 1, "input"), _Var("v", 2, "input"),
                        _Var("k", 3, "parameter"), _Var("u_cmd", 9, "output")])
    fmpy = types.ModuleType("fmpy")
    fmpy.read_model_description = lambda path: md
    fmpy.extract = lambda path: "unzipped"
    fmi2 = types.ModuleType("fmpy.fmi2")
    fmi2.FMU2Slave = _Slave
    monkeypatch.setitem(sys.modules, "fmpy", fmpy)
    monkeypatch.setitem(sys.modules, "fmpy.fmi2", fmi2)


def test_fmu_controller_samples_with_zoh(fake_fmpy):
    from cacc.adapters.fmu import FMUController

    c = FMUController(ControllerParams(), fmu="x.fmu", output="u_cmd",
                      parameters={"k": 0.5}, step=0.1)
    assert c.uses_v2v is False and c._names == ["e", "v"]
    raw = dict(gap=20.0, v=25.0, a=0.0)
    assert c.output(None, 2.0, 0, 0, t=0.0, **raw) == pytest.approx(1.0)
    assert c.output(None, 8.0, 0, 0, t=0.05, **raw) == pytest.approx(1.0)  # held
    assert c.output(None, 8.0, 0, 0, t=0.1, **raw) == pytest.approx(4.0)
    assert c._slave.steps == pytest.approx([0.0, 0.1])
    with pytest.raises(ValueError, match="no variable"):
        FMUController(ControllerParams(), fmu="x.fmu", output="nope")


def test_fmu_controller_runs_in_a_platoon(fake_fmpy):
    from cacc.platoon import PlatoonSim
    from cacc.scenario import scenario_from_dict

    sc = scenario_from_dict({
        "platoon": {"n_followers": 2, "v0": 20.0},
        "vehicle": {"tau": 0.3, "length": 4.5, "u_min": -8.0, "u_max": 3.0},
        "controller": {"h": 1.5, "r": 3.0},
        "leader": {"profile": "brake", "t_start": 2.0, "duration": 2.0, "decel": -2.0},
        "sim": {"dt": 0.01, "t_final": 20.0}})
    res = PlatoonSim(sc.config, "cacc.adapters.fmu:FMUController", sc.leader,
                     {"fmu": "x.fmu", "output": "u_cmd", "parameters": {"k": 0.3},
                      "step": 0.05}).run()
    assert np.isfinite(res.pos).all() and res.u[:, 0].min() < -0.5


# ------------------------------------------------------------------ CARLA
def test_pedal_map():
    from cacc.adapters.carla_bridge import PedalMap, pedals

    m = PedalMap(k_a=0.0)
    assert pedals(m.coast, 0.0, m) == (0.0, 0.0)
    th, br = pedals(3.5, 3.5, m)
    assert th == pytest.approx(1.0) and br == 0.0
    th, br = pedals(-9.0, -9.0, m)
    assert th == 0.0 and br == pytest.approx(1.0)
    assert pedals(1.0, 0.0)[0] > pedals(1.0, 1.0)[0]  # loop closes on a


def test_platoon_driver_holds_equilibrium_and_brakes():
    from cacc.adapters.carla_bridge import PlatoonDriver
    from cacc.scenario import scenario_from_dict

    sc = scenario_from_dict({
        "platoon": {"n_followers": 2, "v0": 20.0},
        "vehicle": {"tau": 0.3, "length": 4.5, "u_min": -8.0, "u_max": 3.0},
        "controller": {"kp": 0.2, "kd": 0.7, "h": 0.7, "r": 2.5},
        "network": {"delay": 0.1},
        "leader": {"profile": "constant", "value": 0.0},
        "sim": {"dt": 0.01, "t_final": 10.0}})
    drv = PlatoonDriver(sc, "acc", dt=0.05)  # radar only: no V2V intent
    g = drv.spacing(1)
    u = drv.step(0.0, [g, g], [20.0] * 3, [0.0] * 3)
    assert np.allclose(u, 0.0, atol=1e-9)
    for k in range(10):  # the leader brakes hard and the gap closes
        u = drv.step(0.05 * (k + 1), [g - 3.0, g], [17.0, 20.0, 20.0], [-5.0, 0, 0])
    assert u[1] < -1.0


def test_run_carla_needs_the_client(monkeypatch):
    from cacc.adapters.carla_bridge import run_carla

    monkeypatch.setitem(sys.modules, "carla", None)
    with pytest.raises(ImportError, match="CARLA"):
        run_carla(None)


# ------------------------------------------------------------------ ROS 2
def test_ros2_params_from_a_lab_design(tmp_path):
    from cacc.cli import main
    from cacc.controllers import make_controller

    design = tmp_path / "d.yaml"
    assert main(["lab", "--preset", "lag-ff", "-o", str(design)]) == 0
    out = tmp_path / "case.yaml"
    assert main(["ros2-params", "scenarios/ev_highway.yaml", "-c", str(design),
                 "-o", str(out)]) == 0
    p = yaml.safe_load(out.read_text(encoding="utf-8"))["/**"]["ros__parameters"]
    assert p["controller"] == "cacc.tfctrl:TFController" and p["h"] == 0.9
    assert p["tau"] == 0.2 and p["delay_s"] == 0.05
    # what the ROS vehicle node does with it
    ctrl = make_controller(p["controller"], ControllerParams(h=p["h"]),
                           yaml.safe_load(p["controller_options"]))
    assert ctrl.uses_v2v and ctrl.ff_signal == "a" and ctrl.n_states == 1
