"""Field data + digital-twin calibration (D-026), against ground truth.

The real OpenACC files are not redistributed here; the parser is exercised
on files written in the same layout, and calibration is validated where the
truth is known: (1) logs synthesized from known twins, (2) logs from the
independent RK4 platoon simulator, (3) a nonlinear (IDM) platoon whose
linearization is the reference.
"""

import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from cacc import (ControllerParams, PlatoonConfig, PlatoonSim, VehicleParams,
                  make_leader_profile)
from cacc.cli import main
from cacc.fielddata import (PlatoonLog, log_from_sim, read_log, read_openacc,
                            write_log, write_openacc)
from cacc.twin import LinearACC, calibrate_hop, calibrate_log, synthesize_log

ROOT = Path(__file__).parents[1]
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
def mixed_log():
    return synthesize_log([UNSTABLE, STABLE], T10, perturbed_leader(T10),
                          ["Lead", "CarU", "CarS"], seed=3)


# ---------------------------------------------------------------- field data
def test_openacc_round_trip_and_header_detection(tmp_path, mixed_log):
    p = tmp_path / "platoon.csv"
    write_openacc(mixed_log, p)
    back = read_openacc(p)
    assert back.names == ["Lead", "CarU", "CarS"]
    assert back.speed.shape == (T10.size, 3) and back.gap.shape == (T10.size, 2)
    assert back.dt == pytest.approx(0.1)
    assert np.allclose(back.speed, mixed_log.speed, atol=1e-4)
    assert not back.engaged[:, 0].any() and back.engaged[:, 1:].all()
    # an extra metadata row must not break the parser (header found by 'Time')
    lines = p.read_text().splitlines()
    p2 = tmp_path / "extra.csv"
    p2.write_text("\n".join(lines[:4] + ["Campaign,X"] + lines[4:]) + "\n")
    assert read_openacc(p2).speed.shape == back.speed.shape
    (tmp_path / "bad.csv").write_text("a,b\n1,2\n")
    with pytest.raises(ValueError, match="Time"):
        read_openacc(tmp_path / "bad.csv")


def test_generic_log_round_trip_and_autodetect(tmp_path, mixed_log):
    p = tmp_path / "log.csv"
    write_log(mixed_log, p)
    back = read_log(p)
    assert np.allclose(back.gap, mixed_log.gap, atol=1e-4)
    assert back.engaged[:, 1:].all()
    q = tmp_path / "oacc.csv"
    write_openacc(mixed_log, q)
    assert read_log(q).names[1] == "CarU"  # autodetected OpenACC layout


def test_segments_follow_engagement_and_min_duration(mixed_log):
    eng = mixed_log.engaged.copy()
    eng[(T10 >= 100) & (T10 < 120), 1] = False  # driver takeover
    lg = replace(mixed_log, engaged=eng)
    segs = lg.hop_segments(1, min_duration=30)
    assert [(s.start, s.stop) for s in segs] == [(0, 1000), (1200, 2000)]
    assert lg.hop_segments(1, min_duration=150) == []
    with pytest.raises(ValueError):
        lg.hop_segments(3)


def test_uniform_resampling_of_jittered_time():
    t = np.cumsum(np.full(500, 0.1)) + np.random.default_rng(0).normal(0, 0.004, 500)
    lg = PlatoonLog(t, np.full((500, 2), 20.0), np.full((500, 1), 30.0), ["a", "b"])
    from cacc.fielddata import _uniform

    u = _uniform(lg)
    assert np.allclose(np.diff(u.t), u.dt)
    assert "resampled_from" in u.meta


# ---------------------------------------------------------------- calibration
def test_recovers_known_twins_and_verdicts(mixed_log):
    reps, skipped = calibrate_log(mixed_log)
    assert not skipped
    for rep, truth in zip(reps, (UNSTABLE, STABLE), strict=True):
        for p in ("k_s", "k_v", "T", "s0", "tau"):
            assert getattr(rep.model, p) == pytest.approx(getattr(truth, p), rel=0.03), p
        assert rep.hinf == pytest.approx(truth.hinf()[0], abs=0.01)
        assert rep.gap_rmse < 0.1 and rep.speed_rmse < 0.05  # = injected noise
    u, s = reps
    assert u.verdict == "string-unstable" and u.margin < -0.9
    assert s.verdict == "string-stable" and s.margin > 0.5
    # V2V what-if: 100 ms feedforward makes the unstable car stable at T=1.4
    assert u.what_if["hinf"] <= 1.0 + 1e-6
    assert u.empirical["peak_gain"] > 1.05  # model-free data agrees


def test_recovers_parameters_from_independent_rk4_simulator():
    """Data from PlatoonSim (fixed-step RK4, 100 Hz, then 10 Hz + noise),
    calibration by exact LTI solution: two implementations must agree."""
    leader = make_leader_profile({"profile": "bursts", "probe_amplitude": 0.1,
                                  "bursts": [[20, 8, -1.5, 0.0625],
                                             [90, 10, 1.2, 0.05]]})
    cfg = PlatoonConfig(n_followers=2, v0=25.0, t_final=180.0, delay=0.0,
                        control=ControllerParams(kp=0.10, kv=0.30, ka=0.0,
                                                 h=1.4, r=4.0),
                        vehicle=VehicleParams(tau=0.8))
    lg = log_from_sim(PlatoonSim(cfg, "cthp", leader).run())
    k = slice(None, None, 10)
    rng = np.random.default_rng(1)
    lg = PlatoonLog(lg.t[k], lg.speed[k] + rng.normal(0, 0.03, lg.speed[k].shape),
                    lg.gap[k] + rng.normal(0, 0.05, lg.gap[k].shape), lg.names,
                    lg.engaged[k])
    rep = calibrate_hop(lg, 2)
    assert rep.model.T == pytest.approx(1.4, rel=0.01)
    assert rep.model.tau == pytest.approx(0.8, rel=0.03)
    assert rep.hinf == pytest.approx(UNSTABLE.hinf()[0], abs=0.01)


def test_nonlinear_idm_platoon_gets_the_right_verdict():
    """IDM has no linear twin; the fitted twin must still land on its
    small-signal verdict (linearized ||Gamma|| = 1.043 -> unstable)."""
    ref = f"{ROOT / 'examples/controllers/idm_acc.py'}:IDMACC"
    leader = make_leader_profile({"profile": "bursts", "probe_amplitude": 0.1,
                                  "bursts": [[20, 8, -1.0, 0.0625],
                                             [90, 10, 0.8, 0.05]]})
    cfg = PlatoonConfig(n_followers=2, v0=20.0, t_final=200.0, delay=0.0,
                        vehicle=VehicleParams(tau=0.5))
    res = PlatoonSim(cfg, ref, leader, dict(T=1.2, a_max=0.6, b=2.0)).run()
    lg = log_from_sim(res)
    k = slice(None, None, 10)
    lg = PlatoonLog(lg.t[k], lg.speed[k], lg.gap[k], lg.names, lg.engaged[k])
    rep = calibrate_hop(lg, 2)
    assert rep.hinf == pytest.approx(1.043, abs=0.02)
    assert rep.verdict == "string-unstable"


def test_twin_scenario_runs_and_sweep_agrees(tmp_path):
    from cacc.scenario import scenario_from_dict
    from cacc.stringstab import measure_string_stability

    sc = scenario_from_dict(UNSTABLE.scenario(25.0))
    res = measure_string_stability(sc.config, "cthp")
    assert res.peak_gain == pytest.approx(UNSTABLE.hinf()[0], abs=0.01)


def test_no_usable_segment_is_reported():
    lg = synthesize_log([STABLE], T10[:200], perturbed_leader(T10[:200]))
    with pytest.raises(ValueError, match="no ACC-engaged segment"):
        calibrate_hop(lg, 1, min_duration=30)
    reps, skipped = calibrate_log(lg)
    assert reps == [] and len(skipped) == 1


# ------------------------------------------------------------------- tools
def test_calibrate_cli_on_shipped_sample(tmp_path):
    sample = ROOT / "examples/data/synthetic_openacc_platoon.csv"
    code = main(["calibrate", str(sample), "-o", str(tmp_path)])
    assert code == 1  # the sample contains string-unstable ACCs
    twins = sorted(tmp_path.glob("twin_*.yaml"))
    assert len(twins) == 3 and (tmp_path / "calibration.json").is_file()
    assert main(["sweep", str(twins[-1]), "-c", "cthp"]) == 0  # EV-C: stable


def test_openacc_study_script(tmp_path, mixed_log):
    sys.path.insert(0, str(ROOT / "scripts"))
    import openacc_study

    (tmp_path / "data" / "Camp").mkdir(parents=True)
    write_openacc(mixed_log, tmp_path / "data" / "Camp" / "p1.csv")
    (tmp_path / "data" / "Camp" / "junk.csv").write_text("x\n")
    rows, errors = openacc_study.study_file(
        str(tmp_path / "data" / "Camp" / "p1.csv"), str(tmp_path / "data"), 30.0)
    assert [r["follower"] for r in rows] == ["CarU", "CarS"]
    assert rows[0]["campaign"] == "Camp" and rows[0]["verdict"] == "string-unstable"
    agg = openacc_study.aggregate(rows)
    assert {a["vehicle"]: a["unstable_share"] for a in agg} == {"CarS": 0.0,
                                                                "CarU": 1.0}
    md = openacc_study.summary_md(agg, 2, 2, errors)
    assert "CC BY 4.0" in md
