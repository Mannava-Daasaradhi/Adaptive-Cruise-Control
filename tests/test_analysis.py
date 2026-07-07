"""Frequency-domain string-stability results verified against theory.

Reference values (kp=0.2, kd=0.7, tau=0.1, defaults of the base paper):
  * CACC with zero delay is string-stable for ANY headway (Gamma = 1/H).
  * ACC at h=0.7 has ||Gamma||_inf ~ 1.13 (unstable); needs h ~ 3.16 s.
  * CACC minimum stable headway grows with delay:
      theta = 0.1 -> ~0.55 s, theta = 0.5 -> ~1.26 s.
"""

import numpy as np
import pytest

from cacc.analysis import gamma_magnitude, hinf_norm, is_string_stable, min_stable_headway


def test_cacc_zero_delay_is_one_over_H():
    """With theta=0 the exact algebra gives Gamma = 1/(h s + 1)."""
    w = np.logspace(-2, 2, 200)
    mag = gamma_magnitude(w, "cacc", h=0.7, theta=0.0)
    assert np.allclose(mag, 1.0 / np.abs(0.7 * 1j * w + 1.0), rtol=1e-9)
    assert hinf_norm("cacc", 0.7, theta=0.0) <= 1.0 + 1e-9


def test_acc_short_headway_amplifies():
    assert hinf_norm("acc", 0.7) > 1.05
    assert not is_string_stable("acc", 0.7)


def test_acc_minimum_headway_matches_theory():
    h = min_stable_headway("acc")
    assert 3.0 < h < 3.3  # verified: 3.156 s


def test_cacc_minimum_headway_grows_with_delay():
    h0 = min_stable_headway("cacc", theta=0.0)
    h1 = min_stable_headway("cacc", theta=0.1)
    h3 = min_stable_headway("cacc", theta=0.3)
    assert h0 <= 0.02  # any headway stable without delay
    assert 0.5 < h1 < 0.6  # verified: 0.547 s
    assert h1 < h3 < min_stable_headway("cacc", theta=0.5)


def test_cacc_beats_acc_under_realistic_delay():
    """The project's core claim: CACC at 100 ms delay still allows ~6x
    shorter gaps than ACC."""
    assert min_stable_headway("cacc", theta=0.1) < 0.2 * min_stable_headway("acc")


def test_unknown_kind_rejected():
    with pytest.raises(ValueError):
        hinf_norm("human", 1.0)
