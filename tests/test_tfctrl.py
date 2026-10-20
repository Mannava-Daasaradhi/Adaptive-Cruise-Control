"""Controller Lab: transfer-function blocks, analytic Γ, realization, CLI."""

import numpy as np
import pytest

from cacc import analysis
from cacc import tfctrl as tc
from cacc.platoon import PlatoonSim
from cacc.scenario import scenario_from_dict

W = np.logspace(-2, 1, 300)
BASE = {"platoon": {"n_followers": 3, "v0": 25.0},
        "vehicle": {"tau": 0.5, "length": 4.5, "u_min": -8.0, "u_max": 3.0},
        "network": {"delay": 0.1},
        "leader": {"profile": "brake", "t_start": 5.0, "duration": 3.0, "decel": -5.0},
        "sim": {"dt": 0.01, "t_final": 30.0}}


def test_parser_algebra_and_builtins():
    tf = tc.parse("0.2/(h*s + 1)", {"h": 0.5})
    assert np.allclose(tf.num, [0.4]) and np.allclose(tf.den, [1.0, 2.0])
    assert tc.parse("lag(0.2)")(0.0) == pytest.approx(1.0)
    assert tc.parse("pi(1, 2)").order == 1
    assert tc.parse("lowpass2(3, 0.7)")(1j * 3).real == pytest.approx(0.0, abs=1e-12)
    assert tc.parse("(s+1)^2/(s+2)**2")(0) == pytest.approx(0.25)
    assert tc.parse("0").is_zero


@pytest.mark.parametrize("bad, msg", [("__import__('os')", "only numbers"),
                                      ("x + 1", "unknown name"),
                                      ("s**0.5", "whole numbers"),
                                      ("1/(s-s)", "division by zero"),
                                      ("foo(1)", "unknown function")])
def test_parser_rejects_anything_else(bad, msg):
    with pytest.raises(ValueError, match=msg):
        tc.parse(bad)


def test_improper_block_names_the_fix():
    with pytest.raises(ValueError, match="e_dot"):
        tc.Design({"e": "0.7*s"}).tfs()


@pytest.mark.parametrize("key, kw", [("cacc", dict(kp=0.2, kd=0.7)),
                                     ("acc", dict(kp=0.2, kd=0.7)),
                                     ("cthp", dict(kp=0.009, kv=0.63, ka_eff=0.5))])
def test_presets_reproduce_the_analytic_gamma(key, kw):
    d = tc.Design.preset(key)
    theta = 0.0 if key == "acc" else 0.1
    got = tc.gamma(W, d, 0.5, theta, act_delay=0.05)
    ref = analysis.gamma(W, key, d.h, tau=0.5, theta=theta, act_delay=0.05, **kw)
    assert np.max(np.abs(got - ref)) < 1e-12


def test_report_flags_unstable_designs():
    _, _, acc = tc.analyse(tc.Design.preset("acc"), 0.2)
    assert acc.internally_stable and not acc.string_stable  # ACC at h=1.4, kd .7
    bad = tc.Design({"e": "-0.5"}, "u", 1.0, 2.0)
    assert not tc.analyse(bad, 0.2)[2].internally_stable
    for key in ("cacc", "cthp", "lag-ff"):
        rep = tc.analyse(tc.Design.preset(key), 0.2, 0.05)[2]
        assert rep.string_stable and rep.internally_stable, key


@pytest.mark.parametrize("key, gains", [
    ("cacc", {"kp": 0.2, "kd": 0.7, "h": 0.7, "r": 2.5}),
    ("cthp", {"kp": 0.009, "kv": 0.63, "ka": 0.5, "h": 0.95, "r": 2.5})])
def test_tf_controller_matches_builtin_and_contributions_add_up(key, gains):
    sc = scenario_from_dict({**BASE, "controller": gains})
    ref = PlatoonSim(sc.config, key, sc.leader).run()
    d = tc.Design.preset(key)
    res = PlatoonSim(sc.config, tc.REF, sc.leader, d.options).run()
    assert np.max(np.abs(res.pos - ref.pos)) < 1e-6
    parts = tc.contributions(res, d, follower=2)
    assert np.max(np.abs(parts["sum"] - parts["u"])) < 1e-4
    assert {"e", "u_ff", "sum", "u"} <= set(parts)


def test_design_roundtrip_matlab_and_cli(tmp_path, capsys):
    from cacc.cli import main

    path = tmp_path / "d.yaml"
    assert main(["lab", "--preset", "lag-ff", "-o", str(path)]) == 0
    d = tc.Design.load(path)
    assert d.blocks["u_ff"] == "0.8*lag(0.2)" and d.ff_signal == "a"
    assert tc.resolve(str(path))[2] == {"h": d.h, "r": d.r}
    assert main(["lab", str(path), "--matlab", str(tmp_path / "d.m")]) == 0
    m = (tmp_path / "d.m").read_text()
    assert "K_u_ff = tf(" in m and "Gamma" in m
    assert "string-stable" in capsys.readouterr().out
    code = main(["stress", "-c", str(path), "-m", "hard-brake", "-k", "nominal",
                 "--seeds", "1", "-o", str(tmp_path / "st")])
    assert code == 0
