"""Mixed traffic: human drivers (IDM + CAH) and cut-ins."""

import math

import numpy as np
import pytest

from cacc.criteria import METRICS
from cacc.platoon import PlatoonSim
from cacc.scenario import scenario_from_dict
from cacc.traffic import CutIn, IDMParams, TrafficConfig, front_tracks

BASE = {"platoon": {"n_followers": 4, "v0": 25.0},
        "vehicle": {"tau": 0.2, "length": 4.8, "u_min": -9.0, "u_max": 4.0},
        "ev": {"surface": "dry"},
        "controller": {"kp": 0.2, "kd": 0.7, "h": 0.7, "r": 2.5},
        "network": {"delay": 0.05},
        "leader": {"profile": "constant", "value": 0.0},
        "sim": {"dt": 0.01, "t_final": 30.0}}


def run(traffic, ctrl="cacc", leader=None):
    raw = dict(BASE, traffic=traffic)
    if leader:
        raw["leader"] = leader
    sc = scenario_from_dict(raw)
    return PlatoonSim(sc.config, ctrl, sc.leader).run()


def test_idm_equilibrium_and_cah_bound():
    idm = IDMParams()
    v_des = 1.3 * 25.0
    s = idm.eq_gap(25.0, v_des)
    assert idm.accel(s, 25.0, 25.0, v_des) == pytest.approx(0.0, abs=1e-9)
    # a car pulls in at the same speed, 8 m ahead: plain IDM would brake at
    # tens of m/s^2; IDM + CAH brakes about as hard as a comfortable b
    a = idm.accel(8.0, 25.0, 25.0, v_des, a_front=0.0)
    assert -2.5 * idm.b < a < 0.0
    # closing fast on a stopped car still brakes hard
    assert idm.accel(15.0, 20.0, 0.0, v_des, a_front=0.0) < -9.0


def test_human_starts_at_its_own_gap_and_cruises():
    res = run({"humans": [2]})
    gaps, _ = front_tracks(res)
    eq = IDMParams().eq_gap(25.0, 1.3 * 25.0)
    assert gaps[0, 1] == pytest.approx(eq, rel=1e-6)  # human's own gap
    assert np.ptp(gaps[:, 1]) < 0.05 and np.ptp(res.vel) < 0.05  # steady
    assert res.traffic["humans"] == [2]


def test_no_v2v_behind_a_human():
    brake = {"profile": "brake", "t_start": 5.0, "duration": 3.0, "decel": -4.0}
    linked = run(None, leader=brake)
    behind = run({"humans": [2], "idm": {"reaction": 0.0}}, leader=brake)
    # car 3 has no feedforward behind the human: it brakes later and
    # its spacing error is larger than in the all-connected platoon
    assert np.abs(behind.err[:, 2]).max() > 1.5 * np.abs(linked.err[:, 2]).max()


def test_cut_in_is_seen_by_radar_and_metrics():
    res = run({"cut_ins": [{"t": 5.0, "ahead_of": 3, "gap_frac": 0.5,
                            "dv": -2.0, "decel": -2.0, "duration": 1.0}]})
    car = res.traffic["cut_ins"][0]
    k = int(5.0 / 0.01)
    assert math.isnan(car["pos"][k - 2]) and np.isfinite(car["pos"][k + 1])
    assert car["lateral"][k + 1] > 3.0 and car["lateral"][-1] == 0.0
    gaps, closing = front_tracks(res)
    assert gaps.shape[1] == 5  # 4 followers + the cut-in car to its leader
    assert gaps[k + 5, 2] < 0.6 * gaps[k - 5, 2]  # car 3's gap was halved
    assert METRICS["min_gap"].fn(res) == pytest.approx(gaps.min())
    assert METRICS["min_ttc"].fn(res) < math.inf
    assert res.err[k + 5, 2] < -5.0  # car 3 now spaces to the new car


def test_cut_in_car_leaves():
    res = run({"cut_ins": [{"t": 5.0, "ahead_of": 2, "leave": 15.0}]})
    car = res.traffic["cut_ins"][0]
    assert np.isfinite(car["pos"][1000]) and np.isnan(car["pos"][2000])


def test_validation():
    with pytest.raises(ValueError, match="unknown traffic"):
        TrafficConfig.from_dict({"humanz": [1]})
    with pytest.raises(ValueError, match="gap_frac"):
        CutIn(t=1.0, ahead_of=1, gap_frac=1.5)
    with pytest.raises(ValueError, match="not in 1..4"):
        run({"humans": [7]})
    with pytest.raises(ValueError, match="ahead_of"):
        run({"cut_ins": [{"t": 1.0, "ahead_of": 9}]})
