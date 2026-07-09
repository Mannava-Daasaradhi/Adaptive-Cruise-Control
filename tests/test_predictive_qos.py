"""Predictive QoS-map spacing (D-021): map lookahead + anticipatory headway.

Pins the two claims that make the extension patent-grade:
  1. the headway is opened *before* the platoon reaches the interference patch
     (reactive adaptation only opens after entry);
  2. the in-force string-stability margin ||H~||_inf is held (<= 1) throughout
     the patch, where the reactive design is transiently unstable on entry.
"""

from __future__ import annotations

import dataclasses

import numpy as np
import pytest

from cacc import (AdaptConfig, ControllerParams, PlatoonConfig, PlatoonSim,
                  QoSMap, VehicleParams, hinf_norm, make_leader_profile)

KA, KP, KV, TAU0 = 0.5, 0.009, 0.63, 0.5


def _hinf_worst(h: float, rho: float) -> float:
    return max(hinf_norm("cthp", h, kp=KP, tau=TAU0, kv=KV, ka_eff=e * KA)
               for e in (1.0 - 1.0 / rho, 1.0 + 1.0 / rho))


# ------------------------------------------------------------------- map math
def test_qos_map_rho_at():
    m = QoSMap([(1000, 2000, 3.0)], rho_base=10.0)
    assert m.rho_at(500) == 10.0
    assert m.rho_at(1000) == 3.0  # left-closed
    assert m.rho_at(1999) == 3.0
    assert m.rho_at(2000) == 10.0  # right-open
    assert m.rho_at(5000) == 10.0


def test_qos_map_min_rho_ahead():
    m = QoSMap([(1000, 2000, 3.0)], rho_base=10.0)
    # patch 100 m ahead: seen within a 10 s @ 20 m/s (200 m) preview, not a 2 s
    assert m.min_rho_ahead(900, 20.0, 10.0) == 3.0
    assert m.min_rho_ahead(900, 20.0, 2.0) == 10.0
    assert m.min_rho_ahead(1500, 20.0, 5.0) == 3.0  # already inside
    assert m.min_rho_ahead(900, 0.0, 100.0) == 10.0  # stopped: only current x


def test_qos_map_worst_of_overlapping_zones():
    m = QoSMap([(1000, 3000, 5.0), (1500, 2000, 2.5)], rho_base=10.0)
    assert m.rho_at(1700) == 2.5  # worst wins
    assert m.min_rho_ahead(1100, 20.0, 60.0) == 2.5


def test_qos_map_rejects_bad_input():
    with pytest.raises(ValueError):
        QoSMap([(2000, 1000, 3.0)], rho_base=10.0)  # x1 <= x0
    with pytest.raises(ValueError):
        QoSMap([(1000, 2000, 0.9)], rho_base=10.0)  # rho <= 1


def test_qos_map_time_schedule():
    m = QoSMap([(1000, 2000, 3.0)], rho_base=10.0)
    t = np.linspace(0, 200, 2001)
    x = 20.0 * t  # enters at t=50 (x=1000), exits at t=100 (x=2000)
    sched = m.time_schedule(t, x)
    assert sched[0] == (0.0, 10.0)
    rhos = [r for _, r in sched]
    assert rhos == [10.0, 3.0, 10.0]
    t_enter = [tk for tk, r in sched if r == 3.0][0]
    assert abs(t_enter - 50.0) < 0.5


# --------------------------------------------------------- closed-loop claims
def _runs():
    leader = make_leader_profile({
        "profile": "bursts", "bursts": [[70.0, 40.0, 0.3, 0.06]],
        "probe_amplitude": 0.08})
    base = dict(
        n_followers=4, v0=20.0,
        vehicle=VehicleParams(tau=0.5, length=4.0, u_min=-8.0, u_max=3.0),
        control=ControllerParams(kp=KP, kv=KV, ka=KA, h=0.95, r=1.0),
        delay=0.0, noise_rho=10.0, noise_rate=100.0,
        qos_map=((1000.0, 2600.0, 3.0),), dt=0.02, t_final=150.0, seed=1)

    def run(preview):
        ad = AdaptConfig(enabled=True, est_rate=25.0, rho_safety=1.15,
                         margin=0.08, rate=0.05, h_max=2.5, preview_s=preview)
        return PlatoonSim(PlatoonConfig(adapt=ad, **base), "cthp", leader).run()

    return run(0.0), run(30.0)


@pytest.fixture(scope="module")
def runs():
    return _runs()


def test_predictive_opens_headway_before_entry(runs):
    react, pred = runs
    xr, xp = react.pos[:, -1], pred.pos[:, -1]
    # last follower's zone entry (position crosses x0 = 1000 m)
    k_pred_entry = int(np.argmax(xp >= 1000.0))
    t_entry = pred.t[k_pred_entry]
    # sample a few seconds BEFORE entry
    k_pre = int(np.searchsorted(pred.t, t_entry - 8.0))
    assert pred.h[k_pre, -1] > 1.05, "predictive should pre-open the gap"
    assert react.h[k_pre, -1] == pytest.approx(0.95, abs=1e-3), \
        "reactive should still be at nominal before entry"


def test_predictive_holds_string_stability_margin(runs):
    react, pred = runs
    x0, x1 = 1000.0, 2600.0

    def in_zone_margins(res):
        xl = res.pos[:, -1]
        zin = (xl >= x0) & (xl <= x1)
        return np.array([_hinf_worst(float(h), 3.0)
                         for h in res.h[zin, -1][::20]])

    mr, mp = in_zone_margins(react), in_zone_margins(pred)
    assert mp.max() <= 1.0009, "predictive must hold the margin in the patch"
    assert mr.max() > mp.max(), "reactive is worse (transiently unstable)"
    assert (mp > 1.001).mean() == 0.0
    assert (mr > 1.001).mean() > 0.0
