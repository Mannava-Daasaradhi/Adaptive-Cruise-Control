"""Physics-consistency V2V gate (D-022): bounded-injection spoof defence.

Pins the claims that make the security extension patent-grade:
  1. the fused feedforward can deviate from the radar-consistent value by at
     most ``r0 / 2`` for ANY spoof magnitude (the bounded-injection theorem);
  2. the gate is inert under honest operation (channel noise, no attack) — it
     does not perturb the validated string-stability behaviour;
  3. under a large V2V spoof the gate holds a safe gap where the ungated design
     collapses it, and the closed-loop injection stays within ``r0 / 2``.
"""

from __future__ import annotations

import numpy as np
import pytest

from cacc import (AdaptConfig, ControllerParams, LinkAttack, PlatoonConfig,
                  PlatoonSim, TrustConfig, TrustGate, VehicleParams,
                  make_leader_profile)
from cacc.network import V2VLink

KA, KP, KV = 0.5, 0.009, 0.63


# ------------------------------------------------------- the injection theorem
def test_gate_injection_bounded_for_any_spoof():
    """|u_ff_eff - a_radar| <= r0/2 for every a_v2v, including huge spoofs."""
    g = TrustGate(TrustConfig(enabled=True, r0=3.0))
    a_radar = -1.2
    for a_v2v in np.linspace(-1e6, 1e6, 100001):
        u_eff, _, _ = g.fuse(float(a_v2v), a_radar)
        assert abs(u_eff - a_radar) <= 3.0 / 2 + 1e-9


def test_gate_injection_peaks_at_r0():
    """The bound r0/2 is tight: g(r)*r is maximised exactly at r = r0."""
    r0 = 4.0
    g = TrustGate(TrustConfig(enabled=True, r0=r0))
    # inject a_v2v = a_radar + r0 -> residual exactly r0
    u_eff, gw, r = g.fuse(0.0 + r0, 0.0)
    assert r == pytest.approx(r0)
    assert gw == pytest.approx(0.5)
    assert abs(u_eff) == pytest.approx(r0 / 2)
    # no other residual beats it
    rs = np.linspace(0.0, 20 * r0, 5000)
    inj = rs / (1.0 + (rs / r0) ** 2)
    assert inj.max() <= r0 / 2 + 1e-9


def test_gate_transparent_when_consistent():
    """A physically consistent message passes through untouched (g = 1)."""
    g = TrustGate(TrustConfig(enabled=True, r0=3.0))
    u_eff, gw, r = g.fuse(2.5, 2.5)
    assert r == 0.0 and gw == 1.0 and u_eff == pytest.approx(2.5)


def test_gate_monotone_trust():
    """Trust decreases monotonically with inconsistency."""
    g = TrustGate(TrustConfig(enabled=True, r0=2.0))
    rs = [0.0, 0.5, 1.0, 2.0, 5.0, 50.0]
    gws = [g.trust(r) for r in rs]
    assert all(a > b for a, b in zip(gws, gws[1:]))
    assert gws[0] == 1.0 and 0.0 < gws[-1] < 1e-2


def test_gate_rejects_bad_config():
    with pytest.raises(ValueError):
        TrustGate(TrustConfig(enabled=True, r0=0.0))
    with pytest.raises(ValueError):
        TrustGate(TrustConfig(enabled=True, r0=3.0, radar_noise=-0.1))


def test_radar_noise_deterministic_and_zero_mean():
    """Radar noise is reproducible in t and unbiased; 0 std -> exact."""
    ideal = TrustGate(TrustConfig(enabled=True, r0=3.0, radar_noise=0.0))
    assert ideal.radar_at(12.3, -2.0) == -2.0
    noisy = TrustGate(TrustConfig(enabled=True, r0=3.0, radar_noise=0.1, seed=3))
    ts = np.linspace(0, 50, 5000)
    a = np.array([noisy.radar_at(float(t), 1.0) for t in ts])
    same = TrustGate(TrustConfig(enabled=True, r0=3.0, radar_noise=0.1, seed=3))
    b = np.array([same.radar_at(float(t), 1.0) for t in ts])
    assert np.array_equal(a, b)  # deterministic in t for a given seed
    assert abs(a.mean() - 1.0) < 0.02  # unbiased


# --------------------------------------------------------- the attack model
def test_link_attack_kinds():
    bias = LinkAttack(kind="bias", t0=1.0, t1=2.0, value=5.0)
    assert bias.apply(0.5, 3.0) == 3.0  # outside window
    assert bias.apply(1.5, 3.0) == 8.0  # additive inside
    assert bias.apply(2.0, 3.0) == 3.0  # right-open
    over = LinkAttack(kind="override", t0=0.0, value=-8.0)
    assert over.apply(1.0, 2.0) == -8.0
    scale = LinkAttack(kind="scale", t0=0.0, value=3.0)
    assert scale.apply(1.0, 2.0) == pytest.approx(6.0)


def test_link_attack_ramp():
    a = LinkAttack(kind="bias", t0=10.0, value=4.0, ramp=2.0)
    assert a.apply(10.0, 0.0) == pytest.approx(0.0)  # ramp start
    assert a.apply(11.0, 0.0) == pytest.approx(2.0)  # half ramp
    assert a.apply(12.0, 0.0) == pytest.approx(4.0)  # full
    assert a.apply(50.0, 0.0) == pytest.approx(4.0)  # held after ramp


def test_attacked_link_not_passthrough():
    """An attacked ideal link must route through receive(), not shortcut."""
    a = LinkAttack(kind="bias", value=5.0)
    link = V2VLink(delay=0.0, attack=a)
    assert not link.passthrough
    clean = V2VLink(delay=0.0)
    assert clean.passthrough


def test_link_delivers_spoof():
    a = LinkAttack(kind="bias", t0=1.0, t1=3.0, value=10.0, target_link=0)
    link = V2VLink(delay=0.0, attack=a)
    for k in range(400):
        link.send(k * 0.01, 1.0)
    assert link.receive(0.5) == pytest.approx(1.0)  # pre-attack
    assert link.receive(2.0) == pytest.approx(11.0)  # spoofed
    assert link.receive(3.5) == pytest.approx(1.0)  # post-attack


# --------------------------------------------------- closed-loop spoof defence
def _leader():
    return make_leader_profile({"profile": "bursts",
                                "bursts": [[20.0, 30.0, 0.4, 0.05]],
                                "probe_amplitude": 0.05})


def _base(attack):
    return dict(
        n_followers=6, v0=20.0,
        vehicle=VehicleParams(tau=0.5, length=4.0, u_min=-8.0, u_max=3.0),
        control=ControllerParams(kp=KP, kv=KV, ka=KA, h=0.95, r=1.0),
        delay=0.1, noise_rho=10.0, noise_rate=100.0,
        adapt=AdaptConfig(enabled=False), attack=attack,
        dt=0.01, t_final=140.0, seed=1)


def _gap(res, i):  # gap of follower i behind follower i-1
    return res.pos[:, i - 1] - res.pos[:, i] - res.config.vehicle.length


def test_gate_inert_without_attack():
    """No attack: gate on vs off give essentially identical trajectories."""
    base = _base(None)
    off = PlatoonSim(PlatoonConfig(trust=TrustConfig(enabled=False), **base),
                     "cthp", _leader()).run()
    on = PlatoonSim(PlatoonConfig(
        trust=TrustConfig(enabled=True, r0=3.0, radar_noise=0.0), **base),
        "cthp", _leader()).run()
    assert np.abs(on.err - off.err).max() < 1e-3  # gate barely touches honest FF
    assert on.trust.min() > 0.99  # trust stays ~1 all run


def test_gate_blocks_spoof_and_bounds_injection():
    """Large spoof: ungated gap collapses; gated stays safe, injection<=r0/2."""
    r0 = 3.0
    atk = LinkAttack(kind="bias", t0=60.0, t1=75.0, value=5.0, ramp=2.0,
                     target_link=2)
    base = _base(atk)
    ung = PlatoonSim(PlatoonConfig(trust=TrustConfig(enabled=False), **base),
                     "cthp", _leader()).run()
    gat = PlatoonSim(PlatoonConfig(
        trust=TrustConfig(enabled=True, r0=r0, radar_noise=0.05), **base),
        "cthp", _leader()).run()
    # attacked follower is #3 (gap behind #2)
    assert _gap(ung, 3).min() < 0.0  # ungated: collision (gap crosses 0)
    assert _gap(gat, 3).min() > 2.0  # gated: safe standoff kept
    # the provable cap holds in closed loop
    assert np.abs(gat.ff_inj).max() <= r0 / 2 + 1e-9
    # trust collapses during the attack, recovers after
    assert gat.trust[:, 2].min() < 0.5
    assert gat.trust[-1, 2] > 0.95
