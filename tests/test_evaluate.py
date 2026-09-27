"""Test plans, criteria, reports and the CLI (D-025)."""

import json
import math
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import pytest
import yaml

from cacc import PlatoonConfig, PlatoonSim, VehicleParams, make_leader_profile
from cacc.cli import main
from cacc.criteria import METRICS, Criterion, min_gap, min_ttc
from cacc.evaluate import PlanError, evaluate, load_plan, plan_from_dict, set_path
from cacc.reporting import render_markdown, write_all

ROOT = Path(__file__).parents[1]

# short, cheap scenario: Ploeg family, 3 followers, 20 s brake
SCEN = {
    "platoon": {"n_followers": 3, "v0": 20.0},
    "vehicle": {"tau": 0.1},
    "controller": {"kp": 0.2, "kd": 0.7, "h": 0.7, "r": 2.5},
    "network": {"delay": 0.1},
    "leader": {"profile": "brake", "t_start": 2.0, "duration": 2.0,
               "decel": -4.0},
    "sim": {"dt": 0.01, "t_final": 20.0},
}
FAST_SWEEP = {"sweep": {"base_period": 40.0, "harmonics": [1, 3, 5, 9, 15]}}


def plan(**kw):
    raw = {"name": "t", "scenario": SCEN, "under_test": {"controller": "cacc"},
           "criteria": ["min_gap >= 2.0"]}
    raw.update(kw)
    return plan_from_dict(raw, root=ROOT)


# ---------------------------------------------------------------- criteria
def test_criterion_parsing_and_worst_case():
    c = Criterion.parse("min_gap >= 2.5")
    assert (c.metric, c.op, c.value) == ("min_gap", ">=", 2.5)
    assert c.worst([5.0, 1.0, 3.0]) == (1.0, 1)  # lowest is worst for >=
    d = Criterion.parse({"metric": "peak_decel", "op": "<=", "value": 4,
                         "name": "comfort"})
    assert d.worst([2.0, 4.5, 3.0]) == (4.5, 1)  # highest is worst for <=
    assert d.label == "comfort (peak_decel <= 4 m/s^2)"
    assert Criterion.parse("l2_amplification_max <= 1.05").metric in METRICS
    assert math.isnan(c.worst([1.0, math.nan])[0])
    assert not c.passes(math.nan) and c.passes(math.inf)


@pytest.mark.parametrize("bad", ["min_gap => 2", "gap >= 2",
                                 {"metric": "min_gap", "op": "=", "value": 1},
                                 {"metric": "min_gap", "op": ">=", "value": 1,
                                  "extra": 1}, 42])
def test_bad_criteria_rejected(bad):
    with pytest.raises(ValueError):
        Criterion.parse(bad)


def test_gap_and_ttc_metrics_on_known_motion():
    cfg = PlatoonConfig(n_followers=2, t_final=10.0, delay=0.0,
                        vehicle=VehicleParams(tau=0.1))
    res = PlatoonSim(cfg, "cacc", make_leader_profile(
        {"profile": "brake", "t_start": 1.0, "duration": 2.0,
         "decel": -4.0})).run()
    gaps = np.diff(-res.pos, axis=1) - cfg.vehicle.length
    assert min_gap(res) == pytest.approx(gaps.min())
    assert 0.0 < min_ttc(res) < math.inf  # followers close in while braking
    steady = PlatoonSim(cfg, "cacc").run()  # constant speed: never closing
    assert min_ttc(steady) == math.inf


# -------------------------------------------------------------------- plans
def test_set_path_creates_levels_and_rejects_typos():
    tree = {"network": {"delay": 0.1}}
    set_path(tree, "network.loss_prob", 0.2)
    set_path(tree, "leader", {"profile": "constant"})
    assert tree == {"network": {"delay": 0.1, "loss_prob": 0.2},
                    "leader": {"profile": "constant"}}
    with pytest.raises(PlanError, match="netwrk"):
        set_path(tree, "netwrk.delay", 0.1)


def test_cases_times_matrix_expansion_and_names():
    p = plan(seeds=3,
             cases=[{"name": "a"},
                    {"name": "b", "set": {"controller.h": 1.5}}],
             matrix={"network.delay": [0.0, 0.2], "vehicle.tau": [0.1, 0.2]})
    assert p.seeds == [1, 2, 3]
    assert len(p.cases) == 8
    assert p.cases[0].name == "a[delay=0.0,tau=0.1]"
    b = [c for c in p.cases if c.name.startswith("b")]
    assert all(c.raw["controller"]["h"] == 1.5 for c in b)
    assert p.cases[-1].raw["network"]["delay"] == 0.2


@pytest.mark.parametrize("kw, msg", [
    (dict(bogus=1), "unknown plan key"),
    (dict(under_test={}), "under_test.controller"),
    (dict(under_test={"controller": "pid"}), "unknown controller"),
    (dict(criteria=[]), "checks nothing"),
    (dict(criteria=["min_gap >= x"]), "criteria"),
    (dict(overrides={"controller.gain": 1}), "case 'base'"),
    (dict(string_stability={"maxgain": 1}), "string_stability"),
    (dict(under_test={"controller": "cacc", "options": {"k": 1}}),
     "takes no options"),
    (dict(under_test={"controller": "missing.py:X"}), "under_test"),
    (dict(under_test={"controller": "cacc.controllers:Nope"}), "no attribute"),
    (dict(overrides={"platoon.n_followers": 1},
          criteria=["l2_amplification_max <= 1.05"]), "n_followers >= 2"),
    (dict(matrix={"network.delay": []}), "no cases"),
])
def test_plan_errors_are_specific(kw, msg):
    with pytest.raises(PlanError, match=msg):
        plan(**kw)


def test_plan_needs_exactly_one_scenario_source():
    with pytest.raises(PlanError, match="exactly one"):
        plan_from_dict({"under_test": {"controller": "cacc"},
                        "criteria": ["min_gap >= 1"]})


def test_example_plans_load():
    for f in sorted((ROOT / "examples" / "plans").glob("*.yaml")):
        p = load_plan(f)
        assert p.cases and (p.criteria or p.sweep)
    idm = load_plan(ROOT / "examples/plans/idm_acc.yaml")
    assert Path(idm.controller.rpartition(":")[0]).is_file()  # plan-relative


# --------------------------------------------------------------- evaluation
def test_evaluate_pass_fail_and_worst_seed(tmp_path):
    p = plan(seeds=2, matrix={"controller.h": [0.3, 0.7]},
             criteria=["min_time_gap >= 0.6", "peak_decel <= 6.0"],
             string_stability=FAST_SWEEP)
    rep = evaluate(p, jobs=2)
    by = {c["name"]: c for c in rep["cases"]}
    # CACC at h = 0.7 s, 100 ms delay: string-stable, time gap >= 0.8 s
    assert by["base[h=0.7]"]["verdict"] == "PASS"
    # h = 0.3 s is below the ~0.55 s delay bound; time gap dips to ~0.41 s
    short = by["base[h=0.3]"]
    assert short["verdict"] == "FAIL"
    assert not short["string_stability"]["passed"]
    gap = short["checks"][0]
    assert not gap["passed"] and gap["worst_seed"] in (1, 2)
    assert set(gap["per_seed"]) == {"1", "2"}
    assert rep["verdict"] == "FAIL" and rep["counts"]["FAIL"] == 1

    paths = write_all(rep, tmp_path)
    data = json.loads(paths["json"].read_text())  # strict JSON (no NaN/inf)
    assert data["provenance"]["tool"] == "cacc"
    suites = ET.parse(paths["junit"]).getroot()
    assert suites.get("failures") == str(sum(
        (not c["passed"]) for case in rep["cases"] for c in case["checks"])
        + 1)  # + the failed sweep
    md = paths["markdown"].read_text()
    assert "FAIL" in md and "base[h=0.3]" in md
    entry = next(c for c in data["cases"] if c["name"] == "base[h=0.3]")
    saved = yaml.safe_load((tmp_path / entry["scenario_file"]).read_text())
    assert saved["controller"]["h"] == 0.3


def test_simulation_errors_become_error_verdicts(tmp_path):
    bad = tmp_path / "bad.py"
    bad.write_text("import numpy as np\n"
                   "class Boom:\n"
                   "    n_states = 0\n    uses_v2v = False\n"
                   "    ff_signal = 'a'\n"
                   "    def __init__(self, params): pass\n"
                   "    def output(self, *a, **k): raise RuntimeError('boom')\n"
                   "    def deriv(self, *a, **k): return np.empty(0)\n")
    p = plan(under_test={"controller": f"{bad}:Boom"})
    rep = evaluate(p)
    assert rep["verdict"] == "ERROR"
    assert "boom" in rep["cases"][0]["errors"][0]["error"]
    assert "ERROR" in render_markdown(dict(rep, provenance={}))


# ---------------------------------------------------------------------- cli
def test_cli_exit_codes(tmp_path):
    good = tmp_path / "good.yaml"
    good.write_text(yaml.safe_dump({"name": "g", "scenario": SCEN,
                                    "under_test": {"controller": "cacc"},
                                    "criteria": ["min_gap >= 2.0"]}))
    assert main(["evaluate", str(good), "-o", str(tmp_path / "out")]) == 0
    assert (tmp_path / "out" / "junit.xml").is_file()
    bad = tmp_path / "bad.yaml"
    bad.write_text("name: b\nunder_test: {controller: cacc}\n")
    assert main(["evaluate", str(bad)]) == 2
    scen = tmp_path / "s.yaml"
    scen.write_text(yaml.safe_dump(SCEN))
    assert main(["run", str(scen), "-c", "cacc", "--json"]) == 0
    assert main(["metrics"]) == 0


def test_cli_controller_crash_is_error_not_fail(tmp_path, capsys):
    # exit 1 means "the controller failed the check"; a crash must never be
    # read that way by a CI gate
    bad = tmp_path / "bad.py"
    bad.write_text("import numpy as np\n"
                   "class Boom:\n"
                   "    n_states = 0\n    uses_v2v = False\n"
                   "    ff_signal = 'a'\n"
                   "    def __init__(self, params): pass\n"
                   "    def output(self, *a, **k): raise RuntimeError('boom')\n"
                   "    def deriv(self, *a, **k): return np.empty(0)\n")
    scen = tmp_path / "s.yaml"
    scen.write_text(yaml.safe_dump(SCEN))
    for cmd in ("sweep", "run"):
        assert main([cmd, str(scen), "-c", f"{bad}:Boom"]) == 2
        assert "RuntimeError: boom" in capsys.readouterr().err


# ------------------------------------------------------------------ scaffold
def test_init_scaffold_produces_a_valid_project(tmp_path):
    proj = tmp_path / "acme"
    assert main(["init", str(proj), "--name", "acme-acc"]) == 0
    p = load_plan(proj / "plans" / "release_gate.yaml")  # imports the plugin
    assert p.name == "acme-acc-release-gate" and len(p.cases) == 6
    assert (proj / ".github" / "workflows" / "controller-gate.yml").is_file()
    assert main(["init", str(proj)]) == 2  # refuses to overwrite
    assert main(["init", str(proj), "--force"]) == 0
