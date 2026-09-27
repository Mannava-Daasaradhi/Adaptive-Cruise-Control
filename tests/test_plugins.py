"""Bring-your-own-controller plugin interface (D-025)."""

import textwrap

import numpy as np
import pytest

from cacc import ControllerParams, PlatoonConfig, PlatoonSim, VehicleParams, make_leader_profile
from cacc.controllers import make_controller
from cacc.scenario import scenario_from_dict

CASE_A = PlatoonConfig(n_followers=3, t_final=30.0,
                       control=ControllerParams(kp=0.009, kv=0.63, ka=0.5,
                                                h=0.95, r=1.0),
                       vehicle=VehicleParams(tau=0.5), delay=0.1, noise_rho=5.0)
BRAKE = make_leader_profile({"profile": "brake", "t_start": 5.0,
                             "duration": 2.0, "decel": -3.0})

PLUGIN_SRC = textwrap.dedent('''
    import numpy as np

    class MyCTHP:
        """Re-implementation of the CTHP law as a plugin (for equivalence)."""
        n_states = 0
        uses_v2v = True
        ff_signal = "a"

        def __init__(self, params, scale=1.0):
            self.p, self.scale = params, scale

        def output(self, xc, e, e_dot, u_ff, dv=0.0):
            p = self.p
            return self.scale * (p.kp * e + p.kv * dv + p.ka * u_ff)

        def deriv(self, xc, e, e_dot, u_ff, dv=0.0):
            return np.empty(0)

    class Recorder:
        """Radar-only law that records the raw measurements it receives."""
        n_states = 0
        uses_v2v = False
        ff_signal = "a"
        raw_inputs = True
        seen = []

        def __init__(self, params):
            pass

        def equilibrium_gap(self, v):
            return 7.0 + 1.1 * v

        def output(self, xc, e, e_dot, u_ff, dv=0.0, *, gap, v, a, t):
            Recorder.seen.append((t, gap, v))
            return 0.0

        def deriv(self, xc, e, e_dot, u_ff, dv=0.0, **raw):
            return np.empty(0)

    class Broken:
        n_states = -1
        uses_v2v = "yes"
        ff_signal = "x"

        def __init__(self, params):
            pass
''')


@pytest.fixture()
def plugin_file(tmp_path):
    path = tmp_path / "my_ctrl.py"
    path.write_text(PLUGIN_SRC)
    return path


def test_file_plugin_reproduces_builtin_bit_for_bit(plugin_file):
    ref = PlatoonSim(CASE_A, "cthp", BRAKE).run()
    plug = PlatoonSim(CASE_A, f"{plugin_file}:MyCTHP", BRAKE).run()
    assert plug.controller == f"{plugin_file}:MyCTHP"  # case preserved
    assert np.array_equal(ref.pos, plug.pos)
    assert np.array_equal(ref.u, plug.u)


def test_module_plugin_reference_and_options(plugin_file):
    ref = PlatoonSim(CASE_A, "cthp", BRAKE).run()
    via_module = PlatoonSim(CASE_A, "cacc.controllers:CTHP", BRAKE).run()
    assert np.array_equal(ref.pos, via_module.pos)
    scaled = PlatoonSim(CASE_A, f"{plugin_file}:MyCTHP", BRAKE,
                        controller_options={"scale": 1.2}).run()
    assert not np.array_equal(ref.pos, scaled.pos)


def test_raw_inputs_and_equilibrium_gap(plugin_file):
    cfg = PlatoonConfig(n_followers=2, t_final=1.0, v0=20.0, delay=0.0)
    sim = PlatoonSim(cfg, f"{plugin_file}:Recorder")
    x0 = sim.initial_state()
    pitch = 7.0 + 1.1 * 20.0 + cfg.vehicle.length
    assert x0[2] == pytest.approx(-pitch)  # follower 1 at one pitch behind
    rec = type(sim.ctrls[0])
    rec.seen.clear()
    sim.run()
    t, gap, v = rec.seen[0]
    assert t == 0.0 and gap == pytest.approx(7.0 + 1.1 * 20.0) and v == 20.0


def test_contract_violations_are_reported_precisely(plugin_file):
    with pytest.raises(TypeError, match="n_states.*uses_v2v.*ff_signal.*output"):
        make_controller(f"{plugin_file}:Broken", ControllerParams())


@pytest.mark.parametrize("ref, exc", [
    ("nope.py:X", FileNotFoundError),
    ("cacc.controllers:Nope", ValueError),
    (":X", ValueError),
])
def test_bad_references(ref, exc):
    with pytest.raises(exc):
        make_controller(ref, ControllerParams())


def test_followers_must_run_the_same_kind_of_law(tmp_path):
    # a factory that alternates classes would silently corrupt the state
    # layout (n_states is read from the first follower only)
    f = tmp_path / "mixed.py"
    f.write_text(textwrap.dedent('''
        import numpy as np
        class A:
            n_states = 0
            uses_v2v = False
            ff_signal = "a"
            def __init__(self, params): pass
            def output(self, xc, e, e_dot, u_ff, dv=0.0): return 0.1 * e
            def deriv(self, xc, e, e_dot, u_ff, dv=0.0): return np.zeros(self.n_states)
        class B(A):
            n_states = 1
        _count = [0]
        def Mixed(params):
            _count[0] += 1
            return (A if _count[0] % 2 else B)(params)
    '''))
    with pytest.raises(ValueError, match="same kind of law"):
        PlatoonSim(CASE_A, f"{f}:Mixed")


def test_builtin_rejects_options():
    with pytest.raises(ValueError, match="takes no options"):
        make_controller("cthp", ControllerParams(), {"x": 1})


def test_scenario_rejects_unknown_block():
    with pytest.raises(ValueError, match="netwrk"):
        scenario_from_dict({"netwrk": {"delay": 0.1},
                            "leader": {"profile": "constant"}})
