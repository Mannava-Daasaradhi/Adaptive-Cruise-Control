"""Tests for the QoS-adaptive extension (D-016): estimation, prediction,
headway adaptation, and the time-varying channel."""

import numpy as np
import pytest

from cacc import (AdaptConfig, ChannelEstimator, ControllerParams,
                  HeadwayAdapter, PlatoonConfig, PlatoonSim, V2VLink,
                  VehicleParams, min_stable_headway)

CASE_A = dict(kp=0.009, kv=0.63, ka=0.5)
TAU0 = 0.5


# ------------------------------------------------------------ rho schedule
def test_rho_schedule_switches_noise_support():
    link = V2VLink(delay=0.0, seed=3, noise_rho=10.0,
                   rho_schedule=((0.0, 10.0), (5.0, 2.0)))
    w_good = [link.noise_at(t) for t in np.arange(0.0, 4.9, 0.01)]
    w_bad = [link.noise_at(t) for t in np.arange(5.1, 9.9, 0.01)]
    assert max(abs(w - 1.0) for w in w_good) <= 0.1 + 1e-12
    assert max(abs(w - 1.0) for w in w_bad) <= 0.5 + 1e-12
    assert max(abs(w - 1.0) for w in w_bad) > 0.1  # visibly noisier


def test_rho_schedule_same_bits_different_rho():
    """The U' cache is rho-independent: schedules only rescale the noise."""
    mk = lambda sched: V2VLink(delay=0.0, seed=7, noise_rho=5.0,  # noqa: E731
                               rho_schedule=sched)
    a = mk(((0.0, 5.0),))
    b = mk(((0.0, 10.0),))
    for t in (0.1, 0.5, 1.3):
        # w - 1 = (U' - 1)/rho: same U' means ratio of deviations = 2
        da, db = a.noise_at(t) - 1.0, b.noise_at(t) - 1.0
        assert da == pytest.approx(2.0 * db, rel=1e-9)


# -------------------------------------------------------------- estimator
def test_estimator_recovers_rho():
    cfg = AdaptConfig(enabled=True)
    link = V2VLink(delay=0.0, seed=11, noise_rho=5.0)
    est = ChannelEstimator(cfg)
    a_true = 0.5  # constant excitation
    for k in range(400):
        t = k * 0.04
        est.add_sample(t, link.noise_at(t) * a_true, a_true)
    rho = est.rho_hat(400 * 0.04)
    assert rho is not None
    assert 4.5 <= rho <= 7.5  # over-estimates by design, safety covers it


def test_estimator_expires_old_samples():
    cfg = AdaptConfig(enabled=True, max_age_s=5.0)
    est = ChannelEstimator(cfg)
    for k in range(100):  # rho = 2 era, t in [0, 4)
        est.add_sample(k * 0.04, 1.0 + 0.49 * (-1) ** k, 1.0)
    for k in range(100, 400):  # rho = 10 era, t in [4, 16)
        est.add_sample(k * 0.04, 1.0 + 0.099 * (-1) ** k, 1.0)
    rho = est.rho_hat(16.0)  # every era-1 sample is now > 5 s old
    assert rho == pytest.approx(1.0 / 0.099, rel=0.05)


def test_estimator_rejects_weak_excitation():
    cfg = AdaptConfig(enabled=True, a_min=0.03)
    est = ChannelEstimator(cfg)
    for k in range(200):
        est.add_sample(k * 0.04, 0.001, 0.001)  # below a_min
    assert est.rho_hat(8.0) is None


# ---------------------------------------------------------------- adapter
def test_adapter_fixed_gain_targets():
    """Fixed-gain requirement: matches case-A design at rho=5, grows as
    the channel degrades, and exceeds the re-tuned closed form."""
    adp = HeadwayAdapter(tau0=TAU0, cfg=AdaptConfig(enabled=True),
                         h0=0.95, **CASE_A)
    h5 = adp.h_required(5.0)
    h3 = adp.h_required(3.0)
    h10 = adp.h_required(10.0)
    assert h5 == pytest.approx(0.945, abs=0.01)  # the case-A design point
    assert h10 < h5 < h3


def test_adapter_rate_limit_and_direction():
    cfg = AdaptConfig(enabled=True, rate=0.05, margin=0.08)
    adp = HeadwayAdapter(tau0=TAU0, cfg=cfg, h0=0.95, **CASE_A)
    h1 = adp.update(1.0, rho_hat=3.0)  # target well above 0.95
    assert h1 == pytest.approx(1.0, abs=1e-9)  # slew-limited: +0.05 in 1 s
    for _ in range(60):
        adp.update(1.0, rho_hat=3.0)
    settled = adp.h
    assert settled > 1.15  # above the rho=3 fixed-gain requirement
    adp.update(1.0, rho_hat=None)  # unobservable channel: hold
    assert adp.h == settled


def test_predictor_restores_delay_budget():
    """Theory: at theta = 0.15 s the timestamp predictor brings the
    fixed-gain required headway back to (nearly) the theta = 0 value."""
    kw = dict(kp=0.009, tau=TAU0, kv=0.63)
    h_no = max(min_stable_headway("cthp", theta=0.15, ka_eff=e * 0.5, **kw)
               for e in (0.8, 1.2))
    h_pred = max(min_stable_headway("cthp", theta=0.15, ka_eff=e * 0.5,
                                    pred_theta_hat=0.15, **kw)
                 for e in (0.8, 1.2))
    h_0 = max(min_stable_headway("cthp", theta=0.0, ka_eff=e * 0.5, **kw)
              for e in (0.8, 1.2))
    assert h_no > 1.5  # uncompensated: budget blown
    assert h_pred < h_0 + 0.05  # predictor: near the zero-delay requirement


# ------------------------------------------------------------ closed loop
def _adaptive_cfg(**over):
    base = dict(
        n_followers=3, v0=20.0,
        control=ControllerParams(h=0.95, r=1.0, **CASE_A),
        vehicle=VehicleParams(tau=TAU0, length=4.0),
        delay=0.0, noise_rho=10.0,
        rho_schedule=((0.0, 10.0), (40.0, 3.0)),
        adapt=AdaptConfig(enabled=True), t_final=100.0, seed=1)
    base.update(over)
    return PlatoonConfig(**base)


def _probe_leader(t):
    return 0.04 * (np.sin(1.9 * t) + np.sin(4.6 * t))


def test_adaptive_platoon_tracks_zone():
    res = PlatoonSim(_adaptive_cfg(), "cthp", _probe_leader).run()
    k_pre, k_post = 3800, 9800
    assert res.rho_hat[k_pre, 0] > 6.0  # good channel identified
    assert res.rho_hat[k_post, 0] < 4.0  # zone identified
    assert res.h[k_post, 0] > res.h[k_pre, 0] + 0.15  # headway opened
    # headway respects the slew limit (rate is enforced per estimator
    # update, so check over 1 s windows)
    h1s = res.h[::100, 0]
    assert np.abs(np.diff(h1s)).max() <= res.config.adapt.rate + 1e-6


def test_adaptation_requires_cthp_and_continuous_link():
    with pytest.raises(ValueError, match="cthp"):
        PlatoonSim(_adaptive_cfg(), "cacc", _probe_leader)
    with pytest.raises(ValueError, match="continuous"):
        PlatoonSim(_adaptive_cfg(msg_rate=25.0), "cthp", _probe_leader)


def test_predicted_receive_tracks_delayed_ramp():
    """Steady ramp through a noiseless delayed link: the predictor should
    cancel most of the delay-induced offset."""
    link = V2VLink(delay=0.2, seed=1)
    for k in range(600):
        link.send(k * 0.01, 0.5 * k * 0.01)  # value = 0.5 t
    t = 5.0
    plain = link.receive(t)
    pred = link.receive_predicted(t, theta_hat=0.2)
    true = 0.5 * t
    assert abs(true - plain) == pytest.approx(0.1, abs=1e-3)  # 0.5 * 0.2
    assert abs(true - pred) < 0.02


# ------------------------------------------------------- gain re-tuning
def test_gain_scheduler_reproduces_case_a_at_rho10():
    """The 90%-of-ceiling rule recovers the paper's own case-A kv at the
    design channel quality — evidence it is the implicit design recipe."""
    from cacc import GainScheduler
    sch = GainScheduler(ka=0.5, kp0=0.009, kv0=0.63, tau0=TAU0,
                        cfg=AdaptConfig(enabled=True, adapt_gains=True))
    kv, kp, _ = sch.targets(10.0)
    assert kv == pytest.approx(0.628, abs=0.005)
    assert kp == pytest.approx(0.009, abs=0.0005)


def test_gain_scheduler_breaks_the_wall_at_rho2():
    """Re-tuned gains make rho = 2 feasible with a modest headway, where
    the fixed case-A gains require > 5 s (09_QoS_Adaptive_CACC.md)."""
    from cacc import GainScheduler
    sch = GainScheduler(ka=0.5, kp0=0.009, kv0=0.63, tau0=TAU0,
                        cfg=AdaptConfig(enabled=True, adapt_gains=True))
    _, _, h_req = sch.targets(2.0)
    assert h_req < 2.0


def test_joint_adaptation_stabilizes_deep_zone():
    """In-force configuration in a rho = 2 zone must be string stable
    (worst case over the noise interval) with joint gain + h adaptation."""
    from cacc import hinf_norm
    cfg = _adaptive_cfg(
        adapt=AdaptConfig(enabled=True, adapt_gains=True),
        rho_schedule=((0.0, 10.0), (40.0, 2.0)), t_final=120.0)
    res = PlatoonSim(cfg, "cthp", _probe_leader).run()
    k = int(115 / cfg.dt)
    kp_f, kv_f = res.gains[k, 0]
    h_f = float(res.h[k, 0])
    assert kv_f < 0.45  # gains actually re-tuned away from case-A
    hinf = max(hinf_norm("cthp", h_f, kp=kp_f, tau=TAU0, kv=kv_f,
                         ka_eff=e * 0.5) for e in (0.5, 1.5))
    assert hinf <= 1.0 + 1e-4


# ------------------------------------------- smoothing & stagger (D-019)
def test_estimator_smoothing_fast_down_slow_up():
    """Asymmetric rho-hat filter: channel worsening passes through in one
    read (safety), the post-expiry recovery jump is low-passed (comfort)."""
    mk = lambda tau: ChannelEstimator(  # noqa: E731
        AdaptConfig(enabled=True, max_age_s=10.0, rho_lpf_tau=tau))
    raw, smo = mk(0.0), mk(8.0)

    def feed(t, dev):
        for est in (raw, smo):
            est.add_sample(t, 1.0 + dev, 1.0)
            est.rho_hat(t)  # advance filters at the sampling cadence

    for k in range(200):  # good era, rho = 10:  t in [0, 8)
        feed(k * 0.04, 0.099 * (-1) ** k)
    assert raw.rho_hat(8.0) == pytest.approx(1 / 0.099, rel=0.05)
    for k in range(50):  # interference era, rho ~ 2:  t in [8, 10)
        feed(8.0 + k * 0.04, 0.49 * (-1) ** k)
    r_zone, s_zone = raw.rho_hat(10.0), smo.rho_hat(10.0)
    assert r_zone < 3.0
    assert s_zone == pytest.approx(r_zone, abs=1e-9)  # drop NOT filtered
    for k in range(275):  # recovery era:  t in [10, 21); era-2 expires ~20
        feed(10.0 + k * 0.04, 0.099 * (-1) ** k)
    r_up, s_up = raw.rho_hat(21.0), smo.rho_hat(21.0)
    assert r_up > 8.0  # raw estimate has already jumped back
    assert s_up < 0.6 * r_up  # filtered rise is gradual...
    for k in range(1000):  # ...but converges:  t in [21, 61)
        feed(21.0 + k * 0.04, 0.099 * (-1) ** k)
    assert smo.rho_hat(61.0) > 0.8 * raw.rho_hat(61.0)


def test_adapter_stagger_gates_recovery_only():
    """The serialization slot delays headway DECREASES by stagger_delay;
    increases (the safety direction) always slew immediately."""
    cfg = AdaptConfig(enabled=True, rate=0.05, margin=0.08, stagger_s=5.0)
    front = HeadwayAdapter(tau0=TAU0, cfg=cfg, h0=0.95, **CASE_A)
    rear = HeadwayAdapter(tau0=TAU0, cfg=cfg, h0=0.95, **CASE_A,
                          table=front._table)
    rear.stagger_delay = 5.0  # what the platoon assigns follower 2
    for adp in (front, rear):
        for _ in range(30):
            adp.update(1.0, rho_hat=3.0)  # settle high inside the zone
    assert rear.h == pytest.approx(front.h)
    h_zone = rear.h
    for _ in range(4):  # channel recovers
        front.update(1.0, rho_hat=10.0)
        rear.update(1.0, rho_hat=10.0)
    assert front.h < h_zone - 0.15  # ungated follower shrinks at once
    assert rear.h == pytest.approx(h_zone)  # gated follower holds its slot
    for _ in range(4):
        rear.update(1.0, rho_hat=10.0)
    assert rear.h < h_zone - 0.1  # slot served: recovery proceeds
    h_before = rear.h
    rear.update(1.0, rho_hat=2.5)  # zone returns mid-recovery
    assert rear.h == pytest.approx(h_before + 0.05, abs=1e-6)  # instant rise


def test_platoon_staggered_recovery_serializes():
    """Closed loop: with stagger_s set, follower i's h starts dropping
    ~i * stagger_s after follower 1's (front-first), and strictly in order.

    The zone is DEEP (rho = 2) and h_max sits well below the in-zone
    requirement, so the plateau is pinned at the ceiling for any estimator
    wander (at true rho = 2 even a +50 % over-estimate still demands more
    than h_max): every follower leaves the zone from exactly h_max and the
    level-crossing times compare start-time, not start-level. In shallower
    zones the estimator's slow over-estimation drift creates a standing
    shrink demand that blurs the recovery front — measured and documented
    in D-019."""
    cfg = _adaptive_cfg(
        rho_schedule=((0.0, 2.0), (50.0, 10.0)), t_final=110.0,
        adapt=AdaptConfig(enabled=True, stagger_s=8.0, rho_lpf_tau=4.0,
                          h_max=2.0))
    res = PlatoonSim(cfg, "cthp", _probe_leader).run()
    k50 = int(50 / cfg.dt)
    assert res.h[k50].min() > 1.9  # plateau pinned at the h_max ceiling
    # recovery start ~ first descent through 1.7 s (plateau at 2.0, the
    # post-zone requirement at ~0.96 — the crossing is unambiguous)
    drop_t = [50.0 + float(res.t[np.argmax(res.h[k50:, i] < 1.7)])
              for i in range(3)]
    assert drop_t[0] < drop_t[1] < drop_t[2]  # front-first order
    # separations are AT LEAST stagger_s apart (episode resets during the
    # wobbly estimator transition can only extend a gate, never shorten it
    # — the conservative direction; see D-019)
    assert drop_t[1] - drop_t[0] > 5.0
    assert drop_t[2] - drop_t[1] > 5.0
    assert drop_t[2] - drop_t[0] < 35.0  # recovery still completes promptly
