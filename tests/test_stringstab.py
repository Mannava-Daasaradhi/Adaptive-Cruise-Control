"""Black-box string-stability sweep (D-025) verified against analytic Gamma.

The sweep must reproduce |Gamma(jw)| of every built-in controller family —
that is what licenses using it on controllers whose transfer function is
unknown (plugins, recorded vehicles).
"""

import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from cacc import ControllerParams, LinkAttack, PlatoonConfig, VehicleParams
from cacc.analysis import gamma_magnitude, hinf_norm
from cacc.stringstab import SweepConfig, measure_string_stability, multisine, tone_phasors

PLOEG = PlatoonConfig(control=ControllerParams(kp=0.2, kd=0.7, h=0.7),
                      vehicle=VehicleParams(tau=0.1), delay=0.0)
CASE_A = PlatoonConfig(control=ControllerParams(kp=0.009, kv=0.63, ka=0.5,
                                                h=0.95, r=1.0),
                       vehicle=VehicleParams(tau=0.5), delay=0.0)
CTHP_KW = dict(kp=0.009, kv=0.63, ka_eff=0.5, tau=0.5)
IDM_REF = f"{Path(__file__).parents[1] / 'examples/controllers/idm_acc.py'}:IDMACC"


def test_multisine_is_periodic_and_small():
    sw = SweepConfig()
    a0 = multisine(sw)
    ts = np.linspace(0.0, sw.base_period, 2001)
    vals = np.array([a0(t) for t in ts])
    assert np.allclose(vals, [a0(t + sw.base_period) for t in ts], atol=1e-12)
    # Schroeder phasing: crest factor (peak / RMS) well below the ~5.7 of
    # 16 in-phase tones, and a gentle absolute excitation
    rms = np.sqrt(np.sum((sw.amplitude * sw.omegas) ** 2) / 2)
    assert np.abs(vals).max() / rms < 3.0
    assert np.abs(vals).max() < 0.15


def test_tone_phasors_recover_known_signal_despite_trend():
    t = np.arange(0.0, 200.0, 0.01)
    w = np.array([2 * np.pi / 200.0, 2 * np.pi * 3 / 200.0])
    X = np.array([0.3 - 0.1j, -0.05 + 0.2j])
    x = 20.0 + 0.004 * t + np.real(X[0] * np.exp(1j * w[0] * t)
                                   + X[1] * np.exp(1j * w[1] * t))
    assert np.allclose(tone_phasors(t, x, w), X, atol=1e-9)


def test_acc_sweep_matches_analytic_and_flags_instability():
    res = measure_string_stability(PLOEG, "acc")
    ref = gamma_magnitude(res.omega, "acc", h=0.7)
    assert np.abs(res.gain - ref).max() < 1e-4
    # Ploeg ACC at h = 0.7 s is string-unstable, ||Gamma|| ~ 1.135
    assert not res.string_stable()
    assert res.peak_gain == pytest.approx(hinf_norm("acc", 0.7), abs=0.01)


def test_cacc_with_delay_matches_analytic():
    cfg = replace(PLOEG, control=replace(PLOEG.control, h=0.4), delay=0.1)
    res = measure_string_stability(cfg, "cacc")
    ref = gamma_magnitude(res.omega, "cacc", h=0.4, theta=0.1)
    assert np.abs(res.gain - ref).max() < 1e-4
    assert res.peak_gain > 1.0  # 100 ms delay breaks h = 0.4 s (needs ~0.55)


def test_cthp_case_a_is_stable_and_low_frequency_instability_is_caught():
    stable = measure_string_stability(CASE_A, "cthp")
    ref = gamma_magnitude(stable.omega, "cthp", h=0.95, **CTHP_KW)
    # 5e-4 not 1e-4: case A's ~70 s closed-loop pole leaves a start-up
    # residue that the linear trend only mostly absorbs at the lowest tone
    assert np.abs(stable.gain - ref).max() < 5e-4
    assert stable.string_stable()
    # below the noiseless bound 2 tau / (1 + ka) = 0.667 s the peak sits at
    # very low frequency — the lowest tone must resolve it
    short = replace(CASE_A, control=replace(CASE_A.control, h=0.6))
    res = measure_string_stability(short, "cthp")
    assert not res.string_stable()
    assert res.peak_gain == pytest.approx(
        hinf_norm("cthp", 0.6, **CTHP_KW), abs=5e-4)


def test_sweep_ignores_attack_and_reports_hops():
    atk = LinkAttack(kind="bias", t0=250.0, t1=300.0, value=5.0, target_link=1)
    cfg = replace(CASE_A, attack=atk)
    res = measure_string_stability(cfg, "cthp")
    assert res.hops == (2, 3)
    assert res.string_stable()
    d = res.to_dict()
    assert len(d["gain"]) == 2 and len(d["omega_rad_s"]) == len(res.omega)


def test_nonlinear_plugin_matches_its_linearization():
    """IDM has no closed-form Gamma; its small-signal linearization does."""
    ref = IDM_REF
    opts = dict(T=1.2, a_max=0.6, b=2.0)
    res = measure_string_stability(
        PlatoonConfig(vehicle=VehicleParams(tau=0.5), delay=0.0), ref,
        controller_options=opts)
    from cacc.plugins import _import_target

    law = _import_target(ref)(None, **opts)
    v, h = 20.0, 1e-5
    s = law.equilibrium_gap(v)

    def f(s_, v_, dv_):
        return law.output(None, 0, 0, 0, dv_, gap=s_, v=v_, a=0.0, t=0.0)

    fs = (f(s + h, v, 0) - f(s - h, v, 0)) / (2 * h)
    fv = (f(s, v + h, 0) - f(s, v - h, 0)) / (2 * h)
    fdv = (f(s, v, h) - f(s, v, -h)) / (2 * h)
    S = 1j * res.omega
    lin = np.abs((fs + fdv * S) / (0.5 * S**3 + S**2 + (fdv - fv) * S + fs))
    assert np.abs(res.gain - lin).max() < 0.02
    assert not res.string_stable()  # sluggish IDM tuning amplifies waves


@pytest.mark.parametrize("bad", [
    dict(base_period=0.0), dict(harmonics=()), dict(harmonics=(1, 1)),
    dict(measure_periods=0), dict(first_hop=4), dict(amplitude=-1.0)])
def test_sweep_config_validation(bad):
    with pytest.raises(ValueError):
        SweepConfig(**bad)


def test_periods_and_omegas():
    sw = SweepConfig(base_period=100.0, harmonics=(3, 1))
    assert np.allclose(sw.omegas, [2 * math.pi / 100, 6 * math.pi / 100])
    assert sw.periods_for(CASE_A) == 1  # deterministic channel: exact
    assert sw.periods_for(replace(CASE_A, noise_rho=5.0)) == 4
    assert SweepConfig(measure_periods=2).periods_for(CASE_A) == 2


def test_noisy_channel_reports_standard_error():
    """Channel noise makes one period's estimate uncertain; the sweep must
    say how uncertain instead of issuing a falsely precise verdict."""
    res = measure_string_stability(replace(CASE_A, noise_rho=5.0), "cthp")
    assert res.periods == 4 and res.gain_se is not None
    se = res.peak_gain_se
    assert 0.0 < se < 5e-3
    # average-channel (E[w]) nominal is 0.9975 at DC; the noisy estimate
    # stays within a few 1e-3 of it
    assert res.peak_gain == pytest.approx(0.9975, abs=3e-3)
    assert res.marginal(res.peak_gain + se)  # within 2 SE -> unresolved
    assert not res.marginal(1.1)
