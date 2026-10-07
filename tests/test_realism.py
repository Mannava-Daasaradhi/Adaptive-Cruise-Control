"""Realism fixes of 2026-10-08: actuation dead time, bursty / jittery V2V,
a late and noisy radar, the lag-matched spoof gate, the sweep's band-edge
flag and the L-infinity amplification metric.

Each fix keeps its defaults backward compatible (the rest of the suite pins
the published numbers); these tests pin what the new options do.
"""

import dataclasses
import math

import numpy as np
import pytest

from cacc import ControllerParams, PlatoonConfig, PlatoonSim, VehicleParams, load_scenario
from cacc.analysis import gamma_magnitude, min_stable_headway
from cacc.criteria import METRICS
from cacc.network import V2VLink
from cacc.platoon import make_leader_profile
from cacc.radar import DelayLine, RadarConfig, RadarSensor
from cacc.scenario import scenario_from_dict
from cacc.stringstab import StringStabilityResult, measure_string_stability
from cacc.trust import TrustConfig, TrustGate

PLOEG = PlatoonConfig(control=ControllerParams(kp=0.2, kd=0.7, h=0.7),
                      vehicle=VehicleParams(tau=0.1), delay=0.0)
CTHP_KW = dict(kp=0.009, kv=0.63, ka_eff=0.5, tau=0.5)


# ------------------------------------------------------ actuation dead time
def test_act_delay_zero_is_the_classic_model():
    w = np.logspace(-2, 1, 200)
    for kind, kw in (("acc", {}), ("cacc", {"theta": 0.1}), ("cthp", CTHP_KW)):
        assert np.array_equal(gamma_magnitude(w, kind, 0.9, **kw),
                              gamma_magnitude(w, kind, 0.9, act_delay=0.0, **kw))


def test_act_delay_costs_headway():
    h0 = min_stable_headway("cthp", kp=0.009, kv=0.63, ka_eff=0.5, tau=0.5)
    h1 = min_stable_headway("cthp", kp=0.009, kv=0.63, ka_eff=0.5, tau=0.5,
                            act_delay=0.2)
    assert h1 > h0 + 0.1
    a0 = min_stable_headway("acc", kp=0.2, kd=0.7, tau=0.1)
    a1 = min_stable_headway("acc", kp=0.2, kd=0.7, tau=0.1, act_delay=0.1)
    assert a1 > a0


def test_simulated_dead_time_matches_the_analytic_transfer():
    cfg = dataclasses.replace(PLOEG, vehicle=VehicleParams(tau=0.1, act_delay=0.1))
    res = measure_string_stability(cfg, "acc")
    ref = gamma_magnitude(res.omega, "acc", h=0.7, act_delay=0.1)
    assert np.abs(res.gain - ref).max() < 2e-3


def test_dead_time_shorter_than_a_step_is_rejected():
    cfg = dataclasses.replace(PLOEG, vehicle=VehicleParams(tau=0.1, act_delay=0.004))
    with pytest.raises(ValueError, match="act_delay"):
        PlatoonSim(cfg, "acc")


# ------------------------------------------------- bursty / jittery channel
def _deliveries(link: V2VLink, seconds: float, dt: float = 0.01):
    got = []
    for k in range(int(seconds / dt)):
        t = k * dt
        link.send(t, t)  # the value is its own send time
        got.append(link.receive(t))
    return np.array(got)


def test_gilbert_elliott_keeps_the_loss_rate_and_makes_bursts():
    link = V2VLink(0.1, loss_prob=0.2, msg_rate=10.0, seed=4, loss_burst=5.0)
    lost = [link._lost() for _ in range(40000)]
    assert np.mean(lost) == pytest.approx(0.2, abs=0.02)
    runs, n = [], 0
    for x in lost:
        if x:
            n += 1
        elif n:
            runs.append(n)
            n = 0
    assert np.mean(runs) == pytest.approx(5.0, rel=0.1)


def test_jitter_never_delivers_an_older_message():
    link = V2VLink(0.05, msg_rate=10.0, seed=2, delay_jitter=0.2)
    vals = _deliveries(link, 60.0)
    assert np.all(np.diff(vals) >= 0.0)  # held message only moves forward
    st = link.stats()
    assert st["delivered"] < st["sent"]  # stale arrivals were discarded
    assert st["max_pir_s"] > 0.1  # jitter opens gaps beyond the 0.1 s period


def test_new_channel_options_validate():
    with pytest.raises(ValueError, match="sampled"):
        V2VLink(0.1, delay_jitter=0.05)
    with pytest.raises(ValueError, match="loss_burst"):
        V2VLink(0.1, loss_prob=0.1, msg_rate=10.0, loss_burst=0.5)
    with pytest.raises(ValueError, match="unreachable"):
        V2VLink(0.1, loss_prob=0.8, msg_rate=10.0, loss_burst=1.0)


def test_scenario_accepts_jitter_burst_and_radar_blocks():
    raw = {"platoon": {"n_followers": 2}, "network": {
               "delay": 0.05, "msg_rate": 10.0, "loss_prob": 0.1,
               "loss_burst": 3.0, "delay_jitter": 0.02},
           "radar": {"lag": 0.3, "noise": 0.05},
           "leader": {"profile": "constant", "value": 0.0}}
    sc = scenario_from_dict(raw)
    assert sc.config.loss_burst == 3.0 and sc.config.delay_jitter == 0.02
    assert sc.config.radar == RadarConfig(lag=0.3, noise=0.05)
    res = PlatoonSim(dataclasses.replace(sc.config, t_final=5.0), "cacc",
                     sc.leader).run()
    assert res.link_stats is not None and len(res.link_stats) == 2


# --------------------------------------------------------------- the radar
def test_delay_line_reads_the_past():
    line = DelayLine(0.3)
    for k in range(101):
        line.record(0.01 * k, float(k))
    assert line.read(0.5) == pytest.approx(20.0)  # value recorded at 0.2 s
    assert line.read(0.505) == pytest.approx(20.5)  # interpolated


def test_radar_sensor_is_late_and_noisy_but_deterministic():
    cfg = RadarConfig(lag=0.2, noise=0.1, seed=3)
    a, b = RadarSensor(cfg), RadarSensor(cfg)
    for s in (a, b):
        for k in range(100):
            s.record(0.01 * k, math.sin(0.01 * k))
    assert a.estimate(0.6, 99.0) == b.estimate(0.6, 99.0)  # same seed, same t
    assert a.estimate(0.6, 99.0) == pytest.approx(math.sin(0.4), abs=0.5)
    assert a.estimate(0.6, 99.0) != pytest.approx(99.0)  # live value ignored


def test_radar_config_conflicts_are_explicit():
    with pytest.raises(ValueError, match="radar block only"):
        PlatoonSim(PlatoonConfig(trust=TrustConfig(enabled=True, radar_noise=0.1),
                                 radar=RadarConfig(lag=0.3)), "cthp")
    with pytest.raises(ValueError, match="match_lag"):
        PlatoonSim(PlatoonConfig(trust=TrustConfig(enabled=True, match_lag=True)),
                   "cthp")


# ----------------------------------------------------- the lag-matched gate
def test_trust_braking_never_weakens_a_harder_brake():
    gate = TrustGate(TrustConfig(enabled=True, r0=3.0, trust_braking=True))
    for a_v2v, a_radar in ((-6.0, 0.0), (-2.0, -1.0), (1.0, 4.0)):
        u, _g, _r = gate.fuse(a_v2v, a_radar)
        assert u <= a_v2v + 1e-12
    u, _g, _r = gate.fuse(5.0, 0.0)  # a phantom throttle stays bounded
    assert u - 0.0 <= 1.5 + 1e-12


@pytest.fixture(scope="module")
def honest_brake():
    """Case-A platoon at h = 0.8 s, honest -7 m/s^2 brake, radar lag 0.5 s."""
    sc = load_scenario("scenarios/spoof_defense.yaml")
    ctl = dataclasses.replace(sc.config.control, h=0.8)
    base = dataclasses.replace(sc.config, control=ctl, attack=None, t_final=40.0,
                               trust=TrustConfig(enabled=False))
    brake = make_leader_profile({"profile": "brake", "t_start": 20.0,
                                 "duration": 3.0, "decel": -7.0})
    radar = RadarConfig(lag=0.5, noise=0.05)

    def min_gap(cfg):
        res = PlatoonSim(cfg, "cthp", brake).run()
        return float((res.pos[:, :-1] - res.pos[:, 1:] - 4.0).min()), res

    out = {"off": min_gap(base)}
    out["direct"] = min_gap(dataclasses.replace(
        base, radar=radar, trust=TrustConfig(enabled=True, r0=3.0)))
    out["matched"] = min_gap(dataclasses.replace(
        base, radar=radar, trust=TrustConfig(enabled=True, r0=3.0, match_lag=True)))
    return out


def test_late_radar_makes_the_original_gate_unsafe_in_honest_braking(honest_brake):
    off, direct = honest_brake["off"][0], honest_brake["direct"][0]
    assert off > 0.0  # ungated: a near miss, no collision
    assert direct < off - 0.5 and direct < 0.0  # original gate: a collision


def test_lag_matched_gate_restores_honest_braking(honest_brake):
    assert honest_brake["matched"][0] == pytest.approx(honest_brake["off"][0],
                                                       abs=0.15)


def test_direct_gate_bound_still_holds_with_a_late_radar(honest_brake):
    res = honest_brake["direct"][1]
    assert np.abs(res.ff_inj).max() <= 3.0 / 2 + 1e-9


# ------------------------------------------- band edge and L-inf amplification
def _result(peak_at: int, f: int = 5) -> StringStabilityResult:
    gain = np.full((1, f), 0.9)
    gain[0, peak_at] = 0.99
    return StringStabilityResult(np.linspace(0.1, 1.0, f), gain.astype(complex), (2,))


def test_sweep_flags_a_peak_on_the_band_edge():
    assert _result(0).peak_band_edge == "low"
    assert _result(4).peak_band_edge == "high"
    assert _result(2).peak_band_edge is None
    assert _result(0).to_dict()["peak_band_edge"] == "low"


def test_peak_amplification_metric_is_registered():
    m = METRICS["peak_amplification_max"]
    assert m.worse == "high"
    sc = load_scenario("scenarios/leader_brake.yaml")
    res = PlatoonSim(dataclasses.replace(sc.config, t_final=20.0), "acc",
                     sc.leader).run()
    peaks = np.abs(res.err).max(axis=0)
    assert m.fn(res) == pytest.approx(float((peaks[1:] / peaks[:-1]).max()))
