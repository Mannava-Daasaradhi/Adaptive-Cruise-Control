"""Chance-constrained string-stability risk certificate (D-023).

Pins the claims of the flagship's certificate backbone:
  1. the exact 16-bit channel law (atoms, mean, variance, quantiles);
  2. the risk map exceedance(h) = P(||H~||_inf > 1) and its monotonicity;
  3. the risk-parameterised headway h_cc(eps) achieves the budget and is
     bracketed by [h_nom, h_wc]; Chebyshev is conservative (>= chance);
  4. the honest caveat that mean-square is too permissive (large exceedance);
  5. the deployed case-A design is string-stable across every channel state.
"""

from __future__ import annotations

import numpy as np
import pytest

from cacc import (ChannelLaw, certify_design, chance_headway, chebyshev_headway,
                  exceedance_prob, expected_w, meansquare_headway,
                  nominal_headway, worstcase_headway)
from cacc.analysis import cthp_h_lb
from cacc.network import MA2025_GAMMAS


# ------------------------------------------------------------- the channel law
def test_law_is_a_distribution():
    law = ChannelLaw.build(3.0)
    assert law.w.size == 2 ** 16
    assert law.p.sum() == pytest.approx(1.0, abs=1e-9)
    assert np.all(np.diff(law.w) >= 0)  # ascending
    wlo, whi = law.support
    assert wlo == pytest.approx(1 - 1 / 3)  # U' = 0 atom, exact
    # U'_max = 2 - 2^-15, so w_max = 1 + 1/rho - 2^-15/rho (just below 1+1/rho)
    assert whi == pytest.approx(1 + 1 / 3 - 2 ** -15 / 3, abs=1e-9)


def test_law_moments_match_closed_form():
    g = np.asarray(MA2025_GAMMAS)
    for rho in (2.0, 5.0):
        law = ChannelLaw.build(rho)
        assert law.mean == pytest.approx(expected_w(rho), abs=1e-9)
        var_cf = float((2.0 ** (-2 * np.arange(g.size)) @ (g * (1 - g)))) / rho ** 2
        assert law.var == pytest.approx(var_cf, rel=1e-9)


def test_quantile_monotone_and_valid():
    law = ChannelLaw.build(3.0)
    qs = [law.quantile(e) for e in (1e-4, 1e-3, 1e-2, 1e-1)]
    assert all(a <= b for a, b in zip(qs, qs[1:]))  # nondecreasing in eps
    for e in (1e-3, 1e-2, 1e-1):
        assert law.cdf(law.quantile(e)) >= e - 1e-12
    with pytest.raises(ValueError):
        law.quantile(0.0)
    with pytest.raises(ValueError):
        ChannelLaw.build(1.0)  # rho must be > 1


# ------------------------------------------------------------- the risk map
def test_exceedance_monotone_decreasing_in_h():
    law = ChannelLaw.build(3.0)
    hs = [0.75, 0.85, 0.95, 1.05]
    ex = [exceedance_prob(h, law) for h in hs]
    assert all(a >= b for a, b in zip(ex, ex[1:]))
    assert ex[0] > 0.1  # nominal region is risky
    assert exceedance_prob(worstcase_headway(law), law) <= 1e-6  # ~0 at h_wc


def test_worstcase_headway_near_paper_closed_form():
    # numeric two-tailed worst case vs the paper's eq.(17) — agree within the
    # known ||H~||_inf ~ 1 +/- 5e-3 numeric subtlety (closed form is slightly
    # conservative; both stabilise the whole channel support)
    for rho in (3.0, 5.0):
        law = ChannelLaw.build(rho)
        assert abs(worstcase_headway(law) - cthp_h_lb(0.5, rho, 0.5)) < 0.07


def test_deployed_case_a_is_almost_surely_stable():
    # h = 0.95 at rho = 5 (the reproduced base design)
    d = certify_design(0.95, ChannelLaw.build(5.0))
    assert d["exceedance"] == 0.0  # stable for every one of the 65536 states
    assert d["hinf_worst"] <= 1.0 + 1e-4
    assert d["mean_square_amp"] < 1.0


# ------------------------------------------------- chance-constrained headway
@pytest.fixture(scope="module")
def law3():
    return ChannelLaw.build(3.0)


def test_chance_headway_achieves_budget_and_is_bracketed(law3):
    h_nom, h_wc = nominal_headway(law3), worstcase_headway(law3)
    for e in (1e-2, 1e-3):
        h_cc = chance_headway(e, law3)
        assert exceedance_prob(h_cc, law3) <= e + 1e-6
        assert h_nom <= h_cc <= h_wc + 1e-6


def test_chance_headway_monotone_in_eps(law3):
    # smaller risk budget => larger (or equal) required headway
    h = [chance_headway(e, law3) for e in (1e-1, 1e-2, 1e-3, 1e-4)]
    assert all(a <= b + 1e-9 for a, b in zip(h, h[1:]))  # nondecreasing
    assert h[0] < h[-1]  # strictly cheaper to accept more risk


def test_chebyshev_is_conservative(law3):
    # distribution-free bound must ask for at least as much headway
    for e in (1e-2, 1e-3):
        assert chebyshev_headway(e, law3) >= chance_headway(e, law3) - 1e-6


def test_meansquare_is_too_permissive(law3):
    # the honest caveat: E[A^2] <= 1 licenses a headway riskier than a 1 %
    # chance budget, i.e. it is not robustly string-stable
    h_ms = meansquare_headway(law3)
    assert h_ms < worstcase_headway(law3)
    assert h_ms < chance_headway(1e-2, law3)  # more permissive than 1 % risk
    assert exceedance_prob(h_ms, law3) > 1e-2
