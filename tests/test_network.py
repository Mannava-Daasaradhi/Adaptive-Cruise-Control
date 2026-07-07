"""V2V link: pure delay reconstruction, sampling, packet loss, guards."""

import numpy as np
import pytest

from cacc.network import V2VLink


def _feed_ramp(link, t_end=2.0, dt=0.01):
    """Transmit u(t) = t on a fixed grid."""
    for k in range(int(t_end / dt) + 1):
        link.send(k * dt, k * dt)


def test_continuous_delay_reconstructs_shifted_signal():
    link = V2VLink(delay=0.1)
    _feed_ramp(link)
    for t in (0.3, 0.77, 1.5):
        assert abs(link.receive(t) - (t - 0.1)) < 1e-9


def test_before_history_returns_initial():
    link = V2VLink(delay=0.5, initial=-3.0)
    _feed_ramp(link, t_end=0.2)
    assert link.receive(0.1) == -3.0  # t - delay < 0: nothing received yet


def test_passthrough_property():
    assert V2VLink(delay=0.0).passthrough
    assert not V2VLink(delay=0.1).passthrough
    assert not V2VLink(delay=0.0, msg_rate=10.0).passthrough


def test_sampled_total_loss_holds_initial():
    link = V2VLink(delay=0.05, loss_prob=1.0, msg_rate=10.0, initial=7.0)
    _feed_ramp(link)
    assert link.receive(1.9) == 7.0


def test_sampled_lossless_tracks_with_zoh():
    """10 Hz sampling + 50 ms delay: value lags between delay and delay+period."""
    link = V2VLink(delay=0.05, loss_prob=0.0, msg_rate=10.0)
    _feed_ramp(link)
    t = 1.5
    got = link.receive(t)
    assert t - 0.05 - 0.1 - 1e-9 <= got <= t - 0.05 + 1e-9


def test_loss_without_sampling_rejected():
    with pytest.raises(ValueError):
        V2VLink(delay=0.1, loss_prob=0.5)  # loss needs msg_rate


def test_seeded_loss_is_deterministic():
    def run(seed):
        link = V2VLink(delay=0.05, loss_prob=0.5, msg_rate=20.0, seed=seed)
        _feed_ramp(link)
        return [link.receive(t) for t in np.arange(0.2, 2.0, 0.05)]

    assert run(3) == run(3)
    assert run(3) != run(4)
