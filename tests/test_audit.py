"""String-Stability Audit deliverable (D-027): numbers, files, report.

The twin numbers themselves are validated in test_twin.py; here the audit's
own quantities (latency budget, recommended gap, road test) are checked
against their definitions, and the written files against their consumers
(the release gate must load and give the intended verdicts).
"""

import json
import math
from pathlib import Path

import numpy as np
import pytest

from cacc.audit import (BUDGET_MAX, KA_GRID, LATENCY_GRID, audit_log,
                        latency_budget, recommended_time_gap, render_html,
                        road_test, v2v_grid, write_audit)
from cacc.cli import main
from cacc.evaluate import evaluate, load_plan
from cacc.fielddata import write_openacc
from cacc.twin import LinearACC, synthesize_log

UNSTABLE = LinearACC(k_s=0.10, k_v=0.30, T=1.4, s0=4.0, tau=0.8)  # needs T >= 2.39
STABLE = LinearACC(k_s=0.08, k_v=0.55, T=2.2, s0=5.0, tau=0.5)    # needs T >= 1.63


def perturbed_leader(t):
    v = np.full_like(t, 25.0)
    for t0, dur, dv in ((20, 10, -4.0), (80, 12, 3.0), (140, 8, -5.0)):
        up = np.clip((t - t0) / dur, 0, 1)
        dn = np.clip((t - t0 - dur - 15) / dur, 0, 1)
        v += dv * (0.5 - 0.5 * np.cos(np.pi * up)) - dv * (0.5 - 0.5 * np.cos(np.pi * dn))
    return v


T10 = np.round(np.arange(0.0, 200.0, 0.1), 2)


@pytest.fixture(scope="module")
def audited(tmp_path_factory):
    """One audit of a two-car log; a hostile vehicle name tests escaping."""
    d = tmp_path_factory.mktemp("audit")
    log = synthesize_log([UNSTABLE, STABLE], T10, perturbed_leader(T10),
                         ["Lead", "Car<U>", "CarS"], seed=3)
    path = d / "drive.csv"
    write_openacc(log, path)
    res = audit_log(path)
    return res, write_audit(res, d / "out")


# ------------------------------------------------------------------ numbers
def test_latency_budget_is_the_stability_boundary():
    m = UNSTABLE
    b = latency_budget(m, 0.5)
    assert 0.0 < b < BUDGET_MAX
    assert m.min_stable_time_gap(0.5, b) <= m.T            # inside
    assert m.min_stable_time_gap(0.5, b + 0.01) > m.T      # just outside
    # radar-only instability that no tested feedforward fixes at this gap
    assert latency_budget(m, 0.3) is None
    # a comfortably stable car tolerates the whole searched range at ka=0.3
    assert latency_budget(STABLE, 0.3) == BUDGET_MAX


def test_grid_covers_every_pair_and_matches_the_model():
    rows = v2v_grid(UNSTABLE)
    assert len(rows) == len(KA_GRID) * len(LATENCY_GRID)
    r = next(x for x in rows if x["ka"] == 0.8 and x["delay_s"] == 0.1)
    assert r["min_time_gap_s"] == pytest.approx(
        UNSTABLE.min_stable_time_gap(0.8, 0.1), abs=1e-3)


def test_recommended_gap_clears_the_boundary_by_two_se(audited):
    res, _ = audited
    for v in res.vehicles:
        r = v.twin
        rec = recommended_time_gap(r)
        assert rec >= r.min_time_gap + 2 * r.margin_se
        assert rec - (r.min_time_gap + 2 * r.margin_se) < 0.1 + 1e-9
        assert math.isclose(rec * 10, round(rec * 10))  # a 0.1 s step


def test_road_test_separates_unstable_from_recommended():
    bad = road_test(UNSTABLE, 25.0)
    assert bad["cars"] == 5 and bad["time_gap_s"] == pytest.approx(1.4)
    good = road_test(UNSTABLE, 25.0, 2.6)
    # amplification down the line eats the gap; the stable setting keeps
    # far more room at the tail of the platoon
    assert good["min_gap_m"] > bad["min_gap_m"] + 10.0
    assert not good["collision"]


# ------------------------------------------------------------------- files
def test_audit_writes_report_json_twins_and_gates(audited):
    res, paths = audited
    assert [v.twin.verdict for v in res.vehicles] == ["string-unstable",
                                                      "string-stable"]
    assert res.exit_code == 1
    data = json.loads(paths["calibration"].read_text())
    assert data["meta"]["log_sha256"] == res.meta["log_sha256"]
    assert len(data["meta"]["log_sha256"]) == 64
    f0 = data["followers"][0]
    assert f0["recommended_time_gap_s"] >= 2.4
    assert "road_test" in f0 and "as_calibrated" in f0["road_test"]
    for car in ("Car<U>", "CarS"):
        assert paths[f"gate:{car}"].is_file() and paths[f"twin:{car}"].is_file()


def test_report_is_self_contained_and_escapes_log_content(audited):
    res, paths = audited
    html = paths["report"].read_text()
    assert html.startswith("<!doctype html>")
    assert "Car&lt;U&gt;" in html and "Car<U>" not in html
    assert html.count("<svg") == 3 * len(res.vehicles)  # 3 charts per car
    for ref in ('src="http', "href=\"http", "url(http", "@import"):
        assert ref not in html  # nothing fetched when the file is opened
    assert "String-unstable" in html and "String-stable" in html
    assert render_html(res).count("<section") == len(res.vehicles) + 1


def test_release_gate_loads_and_gives_the_intended_verdicts(audited):
    _, paths = audited
    plan = load_plan(paths["gate:Car<U>"])
    assert [c.name for c in plan.cases] == ["as-calibrated",
                                            "recommended-time-gap"]
    rep = evaluate(plan, jobs=2)
    by_case = {c["name"]: c["verdict"] for c in rep["cases"]}
    assert by_case == {"as-calibrated": "FAIL", "recommended-time-gap": "PASS"}


def test_cli_audit_on_shipped_sample(tmp_path, capsys):
    sample = Path(__file__).parents[1] / "examples/data/synthetic_openacc_platoon.csv"
    assert main(["audit", str(sample), "-o", str(tmp_path / "a")]) == 1
    assert (tmp_path / "a" / "report.html").is_file()
    assert "report:" in capsys.readouterr().out
