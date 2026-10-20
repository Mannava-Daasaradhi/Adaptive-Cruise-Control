"""AEB layer, faults, standstill, crash library and the standards checklist."""

import numpy as np
import pytest
import yaml

from cacc import campaign as cp
from cacc import crashlib
from cacc.aeb import AEBConfig, activations, need
from cacc.criteria import METRICS
from cacc.faults import Fault, parse
from cacc.scenario import scenario_from_dict

BASE = yaml.safe_load(cp.BASE.read_text(encoding="utf-8"))


def cell(maneuver, condition, ctrl="cacc", aeb=False, **edits):
    tree = cp.compose(BASE, maneuver, condition, ctrl)
    if aeb:
        tree["aeb"] = {"enabled": True}
    for k, v in edits.items():
        tree[k] = v
    return cp.replay({"scenario": tree, "name": maneuver}, ctrl, 1)


def min_gap(res):
    return METRICS["min_gap"].fn(res)


# -------------------------------------------------------------------- AEB
def test_need_formula():
    # closing at 10 m/s with 27 m of room past the margin: 100 / 54
    assert need(29.0, 10.0, 25.0, 15.0, 0.0, 2.0) == pytest.approx(100 / 54)
    # a car ahead braking to a stop needs at least the stopping comparison
    d = need(20.0, 0.0, 25.0, 25.0, -6.0, 2.0)
    assert d == pytest.approx(625 / (2 * (18 + 625 / 12)))
    assert need(1.0, 1.0, 5.0, 4.0, 0.0, 2.0) == float("inf")
    with pytest.raises(ValueError):
        AEBConfig(ttc_full=3.0, ttc_warn=2.0)


def test_aeb_prevents_the_cut_in_crash_and_stays_quiet_otherwise():
    assert min_gap(cell("hard-brake", "cut-in")) < 0.0
    saved = cell("hard-brake", "cut-in", aeb=True)
    assert min_gap(saved) > 0.0 and activations(saved)
    calm = cell("traffic-wave", "nominal", aeb=True)
    plain = cell("traffic-wave", "nominal")
    assert not activations(calm)
    assert np.allclose(calm.pos, plain.pos)


def test_campaign_aeb_flag():
    plan = cp.build_plan("cacc", ["hard-brake"], ["cut-in"], seeds=1, aeb=True)
    rep = cp.run_campaign(plan)
    assert rep["campaign"]["aeb"] and rep["verdict"] == "PASS"


# ------------------------------------------------------------- standstill
def test_cars_never_reverse():
    res = cell("ncap-ccrs-50", "nominal", ctrl="acc")
    assert res.vel.min() > -0.02
    assert np.all(np.diff(res.pos, axis=0) > -1e-3)


# ------------------------------------------------------------------ faults
def test_fault_parsing():
    fs = parse([{"kind": "brake-fade", "cars": [2], "value": 0.5}])
    assert fs[0].cars == (2,) and fs[0].on(11.0) and not fs[0].on(14.0)
    with pytest.raises(ValueError, match="unknown fault kind"):
        Fault("melt")
    with pytest.raises(ValueError, match="0..1"):
        Fault("motor-derate", value=1.5)
    with pytest.raises(ValueError, match="not in 1..5"):
        cell("hard-brake", "nominal", faults=[{"kind": "radar-bias", "cars": [9]}])


def test_faults_change_the_run():
    nom = cell("hard-brake", "nominal")
    froze = cell("hard-brake", "radar-frozen")
    assert min_gap(froze) < min_gap(nom)
    fade = cell("emergency-stop", "brake-fade")
    assert METRICS["peak_decel"].fn(fade) > 0 and min_gap(fade) < min_gap(
        cell("emergency-stop", "nominal"))
    bias = cell("cruise", "radar-bias")  # thinks the gap is 3 m larger
    gaps = -np.diff(bias.pos, axis=1)[-1, 0] - 4.8
    assert gaps == pytest.approx(20.0 - 3.0, abs=0.2)
    blk = cell("hard-brake", "v2v-blackout")
    assert min_gap(blk) < min_gap(nom)


def test_stuck_brake_slows_the_car():
    res = cell("cruise", "stuck-brake")
    # the controller fights the drag: the car holds speed with extra throttle
    assert res.u[-1, 1] == pytest.approx(1.5, abs=0.1)


# ----------------------------------------------------------- crash library
def test_crash_library_scenes():
    assert len(crashlib.MANEUVERS) == 11
    t = cp.compose(BASE, "ncap-ccrs-50", "nominal", "cacc")
    sc = scenario_from_dict(t)
    assert sc.config.lead_v0 == 0.0 and sc.config.gap0 == pytest.approx(4 * 50 / 3.6)
    res = cell("nhtsa-lvs", "nominal")
    car = res.traffic["cut_ins"][0]
    k = int(crashlib.REVEAL / 0.01)
    assert np.isfinite(car["pos"][k - 5]) and np.isnan(car["pos"][k + 5])
    # merging traffic from the scene and from the condition keeps both
    t = cp.compose(BASE, "nhtsa-lvs", "cut-in", "cacc")
    assert len(t["traffic"]["cut_ins"]) == 2
    assert "crash-db" in cp.PRESETS


def test_aeb_rescues_the_revealed_stopped_car():
    assert min_gap(cell("nhtsa-lvs", "nominal")) < 0.0
    assert min_gap(cell("nhtsa-lvs", "nominal", aeb=True)) > 0.0


# --------------------------------------------------------------- standards
def test_standards_checklist(monkeypatch):
    from cacc import standards as st

    calls = []

    def fake(args):
        calls.append(args[:2])
        return {m: 5.0 if m != "peak_decel" else 9.0 for m in args[7]}

    monkeypatch.setattr(st, "_run_cell", fake)
    res = st.check("acc")
    by = {r["id"]: r for r in res["rows"]}
    assert by["ISO20035-string"]["status"] == "N/A"
    assert by["ISO15622-decel"]["status"] == "FAIL"  # 9 > 3.5
    assert by["NCAP-CCRS-50"]["status"] == "PASS"
    assert ("hard-brake", "spoof") not in calls
    res = st.check("cacc")
    assert {r["id"]: r for r in res["rows"]}["ISO15622-decel"]["status"] == "REVIEW"
    assert "pre-assessment" in st.markdown(res)


def test_standards_real_run_small(tmp_path, monkeypatch):
    from cacc import standards as st

    monkeypatch.setattr(st, "REQUIREMENTS", [r for r in st.REQUIREMENTS
                                             if r[0] in ("R157-cut-in", "ISO15622-jerk")])
    plain = st.check("cacc")
    safe = st.check("cacc", aeb=True)
    status = lambda r: {x["id"]: x["status"] for x in r["rows"]}  # noqa: E731
    assert status(plain)["R157-cut-in"] == "FAIL"
    assert status(safe)["R157-cut-in"] == "PASS"
    md = st.write(safe, tmp_path)
    assert md.is_file() and (tmp_path / "standards.json").is_file()
