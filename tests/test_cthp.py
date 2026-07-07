"""Ma/Pagilla/Darbha 2025 CTHP: theory equations, noise channel, platoon sims.

The literal numbers (0.9375 s, 0.3183, 0.8727 s) are quoted from Section IV
of the paper and pin the implementation to the published results.
"""

import numpy as np
import pytest

from cacc.analysis import (
    cthp_gains_feasible,
    cthp_h_lb,
    cthp_optimal,
    expected_w,
    hinf_norm,
)
from cacc.controllers import CTHP, ControllerParams, make_controller
from cacc.metrics import amplification_ratios
from cacc.network import MA2025_GAMMAS, V2VLink
from cacc.platoon import PlatoonConfig, PlatoonSim
from cacc.vehicle import VehicleParams

TAU0 = 0.5
RHO = 5.0

# paper Section IV, case A (string-stable) and case B (string-unstable)
CASE_A = dict(kp=0.009, kv=0.63, ka=0.5, h=0.95)
CASE_B = dict(kp=0.009, kv=0.63, ka=0.5, h=0.65)


def paper_platoon(h: float, n: int = 6, t_final: float = 120.0,
                  noise_rho: float | None = RHO) -> PlatoonConfig:
    return PlatoonConfig(
        n_followers=n,
        v0=20.0,
        vehicle=VehicleParams(tau=TAU0, length=4.0),
        control=ControllerParams(kp=0.009, kv=0.63, ka=0.5, h=h, r=1.0),
        delay=0.0,
        noise_rho=noise_rho,
        dt=0.01,
        t_final=t_final,
        seed=7,
    )


def paper_leader(t: float) -> float:
    """One full 0.5 m/s^2 sine cycle at 0.1 rad/s from t = 10 s (eq. (46))."""
    return 0.5 * np.sin(0.1 * (t - 10.0)) if 10.0 < t < 10.0 + 20.0 * np.pi else 0.0


# ------------------------------------------------------------ theory (Thm 3.2)
def test_h_lb_matches_paper_case():
    """tau0=0.5, rho=5, ka=0.5 -> h_lb = 0.9375 s exactly (their Sec. IV)."""
    assert cthp_h_lb(0.5, RHO, TAU0) == pytest.approx(0.9375, abs=1e-10)


def test_h_lb_noiseless_limit_is_classic_bound():
    """rho -> inf reduces to h_lb = 2*tau0/(1+ka) (their Remark 3.3)."""
    assert cthp_h_lb(0.5, None, TAU0) == pytest.approx(2 * TAU0 / 1.5)
    assert cthp_h_lb(0.5, 1e12, TAU0) == pytest.approx(2 * TAU0 / 1.5, rel=1e-6)


def test_optimal_gain_and_headway_match_paper():
    """rho=5 -> ka* = 0.3183, h*_lb = 0.8727 s (their Sec. IV)."""
    ka_star, h_star = cthp_optimal(RHO, TAU0)
    assert ka_star == pytest.approx(0.3183, abs=2e-4)
    assert h_star == pytest.approx(0.8727, abs=2e-4)


def test_paper_gain_choices_are_feasible():
    assert cthp_gains_feasible(tau0=TAU0, rho=RHO, **CASE_A)
    # their optimal case C: ka = ka*, hw = 0.88, kp = 0.003, kv = 0.85
    ka_star, _ = cthp_optimal(RHO, TAU0)
    assert cthp_gains_feasible(kp=0.003, kv=0.85, ka=ka_star, h=0.88,
                               rho=RHO, tau0=TAU0)
    # below the headway bound no (kp, kv) may pass
    assert not cthp_gains_feasible(tau0=TAU0, rho=RHO, **CASE_B)


def test_expected_w_of_paper_channel():
    """E[w] for the paper's 16 gammas at rho=5 (hand-computed 1.0481)."""
    w = expected_w(RHO, MA2025_GAMMAS)
    assert w == pytest.approx(1.0481, abs=1e-3)
    assert 1 - 1 / RHO < w < 1 + 1 / RHO


def test_hinf_stable_above_bound_unstable_below():
    """||H~||_inf <= 1 for case A over the whole noise interval, > 1 for
    case B at the low end.

    The noise interval is I = [(1-1/rho) ka, (1+1/rho) ka]; the low end is
    the binding one for the low-frequency condition (their eq. (28b)), the
    high end for the high-frequency condition (eq. (28a)) — so robustness is
    checked at both endpoints.
    """
    lo = (1 - 1 / RHO) * CASE_A["ka"]
    hi = (1 + 1 / RHO) * CASE_A["ka"]
    for ka_eff in (lo, hi):
        assert hinf_norm("cthp", CASE_A["h"], kp=CASE_A["kp"], kv=CASE_A["kv"],
                         ka_eff=ka_eff, tau=TAU0) <= 1.0 + 1e-6
    unstable = hinf_norm("cthp", CASE_B["h"], kp=CASE_B["kp"], kv=CASE_B["kv"],
                         ka_eff=lo, tau=TAU0)
    assert unstable > 1.0


# ------------------------------------------------------------- noise channel
def test_noise_factor_range_and_determinism():
    link = V2VLink(delay=0.0, seed=3, noise_rho=RHO)
    w = np.array([link.noise_at(t) for t in np.arange(0.0, 5.0, 0.01)])
    assert np.all(w >= 1 - 1 / RHO - 1e-12)
    assert np.all(w < 1 + 1 / RHO)
    assert w.std() > 0  # actually random
    # deterministic: same seed -> same trajectory; same t -> same value
    link2 = V2VLink(delay=0.0, seed=3, noise_rho=RHO)
    w2 = np.array([link2.noise_at(t) for t in np.arange(0.0, 5.0, 0.01)])
    assert np.array_equal(w, w2)
    assert link.noise_at(1.234) == link.noise_at(1.234)


def test_noise_scales_received_value():
    link = V2VLink(delay=0.0, seed=3, noise_rho=RHO)
    link.send(0.0, 2.0)
    link.send(0.01, 2.0)
    assert link.receive(0.01) == pytest.approx(2.0 * link.noise_at(0.01))
    assert not link.passthrough  # noise disables the transparent shortcut


def test_noiseless_link_unchanged():
    link = V2VLink(delay=0.0, seed=3)
    assert link.passthrough
    assert link.noise_at(1.0) == 1.0


# ---------------------------------------------------------------- controller
def test_cthp_formula_and_factory():
    prm = ControllerParams(kp=0.009, kv=0.63, ka=0.5)
    ctrl = make_controller("cthp", prm)
    assert isinstance(ctrl, CTHP)
    assert ctrl.n_states == 0 and ctrl.uses_v2v and ctrl.ff_signal == "a"
    u = ctrl.output(np.empty(0), e=2.0, e_dot=99.0, u_ff=0.4, dv=-1.0)
    assert u == pytest.approx(0.009 * 2.0 + 0.63 * (-1.0) + 0.5 * 0.4)


# ------------------------------------------------------------- platoon sims
def test_cthp_platoon_quiescent_without_disturbance():
    """Cruise + noise but no maneuver: noise multiplies a ~ 0, platoon stays put."""
    sim = PlatoonSim(paper_platoon(h=0.95, n=3, t_final=20.0), controller="cthp")
    res = sim.run()
    assert float(np.max(np.abs(res.err))) < 1e-6


def test_cthp_platoon_stable_case_attenuates():
    """Case A (h = 0.95 s > h_lb): L2 spacing errors shrink down the string."""
    sim = PlatoonSim(paper_platoon(h=0.95), controller="cthp",
                     leader_accel=paper_leader)
    ratios = amplification_ratios(sim.run(), "l2")
    assert np.all(ratios <= 1.0 + 0.02), ratios


def test_cthp_platoon_below_bound_amplifies():
    """Case B (h = 0.65 s < h_lb = 0.9375 s): errors grow along the platoon."""
    sim = PlatoonSim(paper_platoon(h=0.65), controller="cthp",
                     leader_accel=paper_leader)
    ratios = amplification_ratios(sim.run(), "l2")
    assert np.max(ratios) > 1.0, ratios
