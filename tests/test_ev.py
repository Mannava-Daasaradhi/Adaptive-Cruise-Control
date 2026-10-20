"""EV powertrain model: acceleration limits, energy bookkeeping, scenario block."""

import math

import numpy as np
import pytest

from cacc.ev import G, EVParams, energy_report
from cacc.scenario import scenario_from_dict
from cacc.vehicle import VehicleParams

BASE = {
    "platoon": {"n_followers": 2, "v0": 25.0},
    "vehicle": {"tau": 0.2, "length": 4.8, "u_min": -9.0, "u_max": 4.0},
    "controller": {"kp": 0.2, "kd": 0.7, "h": 0.7, "r": 2.5},
    "network": {"delay": 0.05},
    "leader": {"profile": "constant", "value": 0.0},
    "sim": {"dt": 0.01, "t_final": 20.0},
}


def test_without_ev_block_clamp_is_unchanged():
    veh = VehicleParams(u_min=-8.0, u_max=3.0)
    assert veh.clamp(5.0, v=40.0) == 3.0 and veh.clamp(-9.0, v=40.0) == -8.0


def test_power_limit_bites_at_high_speed():
    ev = EVParams(mass=2000.0, power_kw=100.0)
    lo, hi = ev.accel_bounds(40.0, -9.0, 4.0)
    expected = (100e3 / 40.0 - float(ev.resistance(40.0))) / 2000.0
    assert hi == pytest.approx(expected) and hi < 1.2
    assert ev.accel_bounds(5.0, -9.0, 4.0)[1] == 4.0  # low speed: u_max binds


def test_grip_limits_braking_and_traction():
    ice = EVParams(mu=0.1)
    lo, hi = ice.accel_bounds(10.0, -9.0, 4.0)
    assert lo == pytest.approx(-0.1 * G) and hi == pytest.approx(0.1 * G)
    veh = VehicleParams(u_min=-9.0, u_max=4.0, ev=ice)
    assert veh.clamp(-9.0, v=10.0) == pytest.approx(-0.981)


def test_steady_cruise_energy_matches_road_load():
    sc = scenario_from_dict({**BASE, "ev": {"draft_gain": 0.0}})
    from cacc import PlatoonSim

    res = PlatoonSim(sc.config, "cacc", sc.leader).run()
    ev = sc.config.vehicle.ev
    p = float(ev.resistance(25.0)) * 25.0 / ev.eta_drive + ev.aux_kw * 1e3
    expected = p / 25.0 * 1e5 / 3.6e6  # W -> kWh per 100 km at 25 m/s
    rep = energy_report(res)
    assert rep["kwh_per_100km"] == pytest.approx(expected, rel=1e-3)
    assert np.all(rep["regen_share"] == 1.0)  # no braking at all


def test_drafting_saves_energy_and_regen_limit_costs_it():
    from cacc import PlatoonSim

    brake = {"profile": "brake", "t_start": 5.0, "duration": 2.0, "decel": -5.0}
    def run(ev):
        sc = scenario_from_dict({**BASE, "ev": ev, "leader": brake})
        return energy_report(PlatoonSim(sc.config, "cacc", sc.leader).run())
    draft = run({})
    assert np.all(draft["kwh"] < draft["kwh_solo"])
    weak = run({"regen_kw": 5.0})
    assert np.all(weak["regen_share"] < draft["regen_share"])
    assert np.all(weak["kwh"] > draft["kwh"])


def test_surface_names_and_validation():
    sc = scenario_from_dict({**BASE, "ev": {"surface": "snow"}})
    assert sc.config.vehicle.ev.mu == 0.3
    with pytest.raises(ValueError, match="surface"):
        scenario_from_dict({**BASE, "ev": {"surface": "lava"}})
    with pytest.raises(ValueError, match="eta_drive"):
        EVParams(eta_drive=1.5)


def test_energy_metrics_registered():
    from cacc.criteria import METRICS, Criterion

    assert METRICS["energy_kwh_100km"].worse == "high"
    assert METRICS["regen_share"].worse == "low"
    assert Criterion.parse("energy_kwh_100km <= 25").passes(20.0)
    assert not math.isnan(METRICS["energy_kwh_100km"].fn(_run_cacc()))


def _run_cacc():
    from cacc import PlatoonSim

    sc = scenario_from_dict(BASE)  # no ev block: default sedan scoring
    return PlatoonSim(sc.config, "cacc", sc.leader).run()
