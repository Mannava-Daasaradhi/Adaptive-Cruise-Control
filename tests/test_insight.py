"""Breaking points, root-cause explainer and the auto-tuner."""

import copy

import pytest

from cacc import autotune as at
from cacc import campaign as cp
from cacc import envelope as ev
from cacc import tfctrl as tc


# --------------------------------------------------------------- envelope
def test_axes_edit_the_scenario():
    base = {"network": {"delay": 0.05}, "ev": {"surface": "wet"},
            "platoon": {"n_followers": 5}, "vehicle": {}}
    t = ev.apply_axis(copy.deepcopy(base), "grip", 0.3)
    assert t["ev"] == {"mu": 0.3}
    t = ev.apply_axis(copy.deepcopy(base), "loss", 40.0)
    assert t["network"]["loss_prob"] == pytest.approx(0.4)
    assert "attack" not in ev.apply_axis(copy.deepcopy(base), "spoof", 0.0)
    assert ev.apply_axis(copy.deepcopy(base), "spoof", 3.0)["attack"]["value"] == 3.0
    t = ev.apply_axis(copy.deepcopy(base), "lead-brake", 5.0)
    assert t["leader"]["decel"] * t["leader"]["duration"] == pytest.approx(-15.0)
    with pytest.raises(ValueError, match="unknown axis"):
        ev.apply_axis({}, "rain", 1.0)


def test_breaking_point_bisects_to_the_edge():
    r = ev.breaking_point("acc", "hard-brake", "dead-time", coarse=3, bisect=3)
    assert r["status"] == "breaks"
    assert r["last_pass"] < r["value"] <= 0.8
    assert r["value"] - r["last_pass"] <= 0.4 / 2**3 + 1e-9
    assert "breaks at" in r["text"] and r["binding"]


def test_envelope_map_shape():
    env = ev.envelope("acc", "hard-brake", "dead-time", "grip", n=2)
    assert env["margin"].shape == (2, 2) and env["verdict"][0, 0] == "PASS"
    with pytest.raises(ValueError, match="two different"):
        ev.envelope("acc", "hard-brake", "grip", "grip", n=2)


# ---------------------------------------------------------------- explain
@pytest.fixture(scope="module")
def spoof_report():
    plan = cp.build_plan("cacc", ["hard-brake"], ["spoof"], seeds=1)
    return cp.run_campaign(plan)


def test_explain_names_the_crash_the_cause_and_a_verified_fix(spoof_report):
    from cacc.explain import explain, format_text

    ex = explain(spoof_report, "hard-brake | spoof")
    assert ex["verdict"] == "FAIL" and "hit the car in front" in ex["headline"]
    titles = " ".join(f["title"] for f in ex["findings"])
    assert "spoofed V2V" in titles
    gate = next(f for f in ex["fixes"] if "trust gate" in f["change"])
    assert gate["verdict"] == "PASS" and ex["fixes"][0] is gate
    assert "✓ turn on the V2V trust gate" in format_text(ex)
    with pytest.raises(ValueError, match="no case"):
        explain(spoof_report, "nope")


def test_design_of_builtins_matches_analysis():
    import numpy as np

    from cacc import analysis
    from cacc.controllers import ControllerParams
    from cacc.explain import design_of

    p = ControllerParams(kp=0.2, kd=0.7, h=0.7)
    d = design_of("cacc", p, None)
    w = np.logspace(-2, 1, 50)
    ref = np.abs(analysis.gamma(w, "cacc", 0.7, kp=0.2, kd=0.7, tau=0.1, theta=0.05))
    assert np.allclose(np.abs(tc.gamma(w, d, 0.1, 0.05)), ref, rtol=1e-9)
    assert design_of("pkg.mod:Thing", p, None) is None


# ---------------------------------------------------------------- autotune
def test_knobs_skip_structure_and_apply_keeps_it():
    d = tc.Design({"e": "0.2/(h*s + 1)", "e_dot": "0", "dv": "s**2/(s**2+3*s+1)",
                   "u_ff": "0.8*lag(0.2)"}, "a", 0.9, 2.5, "x")
    ks = at.knobs(d)
    assert [(k.signal, k.start) for k in ks] == [
        ("h", 0.9), ("e", 0.2), ("dv", 3.0), ("u_ff", 0.8), ("u_ff", 0.2)]
    d2 = at.apply(d, ks, [1.1, 0.5, 2.0, 0.6, 0.1])
    assert d2.h == 1.1 and d2.name == "x-tuned"
    assert d2.blocks["e"] == "0.5/(h*s + 1)"
    assert d2.blocks["dv"] == "s**2/(s**2+2*s+1)"
    assert d2.blocks["u_ff"] == "0.6*lag(0.1)"


def test_tune_finds_a_passing_shorter_headway():
    d = tc.Design.preset("lag-ff")
    res = at.tune(d, cells=(("hard-brake", "nominal"),), samples=32, finalists=2,
                  polish=1)
    assert res.passed and res.design.h <= 2.0
    ok, peak, _ = at.screen(res.design, 0.2, (0.05, 0.15), 0.0, 1.0)
    assert ok and peak <= 1.0 + 1e-9
    assert "all target cells pass" in res.summary()


# -------------------------------------------------------------------- CLI
def test_cli_envelope_explain_tune(tmp_path, spoof_report, capsys):
    import json

    from cacc.cli import main

    assert main(["envelope", "-c", "acc", "--map", "dead-time", "grip", "-n", "2",
                 "-j", "1"]) == 0
    assert "of the map passes" in capsys.readouterr().out
    rep = tmp_path / "report.json"
    rep.write_text(json.dumps(spoof_report, default=float), encoding="utf-8")
    assert main(["explain", str(rep), "--no-fixes", "-j", "1"]) == 0
    assert "hit the car in front" in capsys.readouterr().out
    assert main(["explain", str(rep), "nope"]) == 2
    out = tmp_path / "t.yaml"
    code = main(["tune", "--preset", "lag-ff", "--samples", "16", "-j", "1",
                 "-o", str(out)])
    assert code in (0, 1) and tc.Design.load(out).name == "lag-ff-tuned"
    assert main(["tune"]) == 2
