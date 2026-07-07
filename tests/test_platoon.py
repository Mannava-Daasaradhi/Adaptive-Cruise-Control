"""End-to-end platoon simulations: equilibrium, string stability, determinism.

The brake-scenario assertions encode the verified headline result: at
h = 0.7 s with 100 ms V2V delay, ACC amplifies the disturbance down the
platoon while CACC attenuates it with ~8x smaller peak errors.
"""

import numpy as np
import pytest

from cacc.controllers import ControllerParams
from cacc.metrics import (
    amplification_ratios,
    empirically_string_stable,
    l2_errors,
    peak_abs_errors,
)
from cacc.platoon import PlatoonConfig, PlatoonSim, make_leader_profile


BRAKE = make_leader_profile({"profile": "brake", "t_start": 5.0, "duration": 2.0,
                             "decel": -4.0})
CFG = PlatoonConfig(n_followers=5, delay=0.1, dt=0.01, t_final=40.0)


def test_quiescent_platoon_stays_at_equilibrium():
    """No leader disturbance: spacing errors must remain ~0 (clean init)."""
    for kind in ("acc", "cacc"):
        res = PlatoonSim(CFG, kind).run()
        assert np.max(np.abs(res.err)) < 1e-9


def test_brake_cacc_attenuates_down_the_platoon():
    res = PlatoonSim(CFG, "cacc", BRAKE).run()
    assert empirically_string_stable(res)
    peaks = peak_abs_errors(res)
    assert peaks[-1] < peaks[0]  # last vehicle sees less than first


def test_brake_acc_amplifies_down_the_platoon():
    res = PlatoonSim(CFG, "acc", BRAKE).run()
    ratios = amplification_ratios(res)
    assert np.all(ratios > 1.0)  # verified: 1.04 - 1.08 each hop
    assert l2_errors(res)[-1] > l2_errors(res)[0]


def test_cacc_peak_errors_much_smaller_than_acc():
    acc = PlatoonSim(CFG, "acc", BRAKE).run()
    cacc = PlatoonSim(CFG, "cacc", BRAKE).run()
    assert np.max(peak_abs_errors(cacc)) < 0.2 * np.max(peak_abs_errors(acc))


def test_lossy_link_simulation_is_seed_deterministic():
    cfg = PlatoonConfig(n_followers=3, delay=0.1, loss_prob=0.3, msg_rate=10.0,
                        dt=0.01, t_final=20.0, seed=42)
    a = PlatoonSim(cfg, "cacc", BRAKE).run()
    b = PlatoonSim(cfg, "cacc", BRAKE).run()
    assert np.array_equal(a.err, b.err)
    cfg2 = PlatoonConfig(n_followers=3, delay=0.1, loss_prob=0.3, msg_rate=10.0,
                         dt=0.01, t_final=20.0, seed=43)
    c = PlatoonSim(cfg2, "cacc", BRAKE).run()
    assert not np.array_equal(a.err, c.err)


def test_packet_loss_degrades_but_does_not_destroy_stability():
    """30% loss at 10 Hz beacons: errors grow vs lossless but stay bounded."""
    lossless = PlatoonConfig(n_followers=5, delay=0.1, dt=0.01, t_final=40.0)
    lossy = PlatoonConfig(n_followers=5, delay=0.1, loss_prob=0.3, msg_rate=10.0,
                          dt=0.01, t_final=40.0, seed=7)
    r0 = PlatoonSim(lossless, "cacc", BRAKE).run()
    r1 = PlatoonSim(lossy, "cacc", BRAKE).run()
    assert np.max(peak_abs_errors(r1)) >= np.max(peak_abs_errors(r0)) - 1e-9
    assert np.max(peak_abs_errors(r1)) < 3.0  # far below ACC's ~6.8 m


def test_small_positive_delay_below_dt_rejected():
    cfg = PlatoonConfig(delay=0.001, dt=0.01)
    with pytest.raises(ValueError):
        PlatoonSim(cfg, "cacc", BRAKE)


def test_headway_config_propagates():
    cfg = PlatoonConfig(control=ControllerParams(h=1.2), t_final=1.0)
    res = PlatoonSim(cfg, "cacc").run()
    assert res.config.control.h == 1.2
