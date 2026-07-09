# D-023 — Chance-constrained string-stability risk certificate

**Date:** 2026-07-09 · **Status:** accepted, **validated** · **Type:** novelty
/ certification

## Context
The Ma-2025 guarantee ``||H~||_inf <= 1`` ([D-007]) is enforced at the
channel's *worst case*: the effective feedforward gain ``ka_eff = w * ka`` is
evaluated at the extremes of ``w``'s support. But ``w = (1 - 1/rho) + U'/rho``
with ``U' = sum_j z_j 2^-j`` is a **random variable** whose distribution is
known exactly from the 16 channel-bit expectations ``gamma_j`` ([D-016]). The
binary pass/fail throws that information away and pays for events of
probability ``~1e-6``. This is the certificate **backbone** of the flagship
(part C of A + B + certificate) — it makes the paper's guarantee *quantitative*
and turns the QoS layer's ad-hoc safety margin into a calibrated risk budget.

## Decision — the exact channel law + three risk certificates
`cacc.certificate.ChannelLaw.build(rho)` forms the **exact** distribution of
``w`` (``2^16 = 65536`` atoms, by iterated convolution of the 16 two-point bit
laws; mean matches :func:`expected_w`, variance the closed form
``sum_j 2^-2j gamma_j(1-gamma_j)/rho^2``). From it:

* **exceedance(h) = P(||H~||_inf > 1)** — the amplification ``A(w)`` swept over
  the atoms; the *risk map* of any design.
* **chance-constrained** ``h_cc(eps)`` (headline) — smallest headway with
  ``exceedance <= eps``. A risk-parameterised headway that **replaces the
  heuristic ``rho_safety`` divisor of [D-016]** with a calibrated budget.
* **mean-square** ``h_ms`` with ``E[A(w)^2] <= 1`` — companion, reported **with
  a caveat** (below).
* **distribution-free (Chebyshev)** ``h_cheb(eps)`` — two-sided
  ``t = sigma/sqrt(eps)`` interval, robust to uncertainty in the ``gamma_j``.

### A two-tailed instability (the key structural finding)
``A(w)`` is **not** monotone in ``w``. At a degraded channel the feedforward is
too *weak* at low ``w`` (deep fade) **and over-amplified** at high ``w`` (the
multiplicative noise inflates the ``ka_eff s^2`` term), so string stability
holds only for ``w`` in a **middle band** and the unstable set is *two-tailed*.
The robust worst-case headway must therefore stabilise **both** rare extremes —
which is exactly why the paper's closed form ``cthp_h_lb`` carries the
``(1+1/rho)`` high-end term. The numeric ``h_wc`` matches ``cthp_h_lb`` to
within the known ``||H~||_inf ~ 1 +- 5e-3`` subtlety (it is the exact frequency-
domain boundary; the closed form is slightly conservative).

## Evidence (`scripts/stochastic_cert_study.py`)
Case-A design, headline channel ``rho = 3``:

| headway | value [s] | exceedance ``P(||H~||>1)`` | note |
|---|---|---|---|
| ``h_nom`` (design at ``E[w]``) | 0.726 | **0.55** | naive/unsafe |
| ``h_ms`` (mean-square) | 0.994 | **0.076** | *too permissive* |
| ``h_cc(1e-2)`` | **1.033** | 0.0095 | 1-in-100 budget |
| ``h_cc(1e-3)`` | 1.052 | 0.0010 | 1-in-1000 |
| ``h_cheb(1e-2)`` | 1.154 | 0 | distribution-free (≈ worst-case) |
| ``h_wc`` (worst-case, both tails) | 1.154 | 0 | ≈ paper ``cthp_h_lb`` = 1.20 |

* **Real throughput dividend.** Accepting a 1-in-100 instantaneous-exceedance
  budget trims **10.5 %** off the worst-case headway (1.154 → 1.033 s) — a
  genuine capacity gain, because the worst case pays to stabilise two
  ``~1e-6``-probability tails.
* **Mean-square is too permissive.** ``h_ms = 0.994`` s carries a **7.6 %**
  exceedance — riskier than a 1 % budget and not robustly string-stable; its
  role here is to *justify* the worst-case / chance-constrained notion, not to
  be deployed.
* **Chebyshev is honest but blunt.** Distribution-free with only mean+variance,
  it collapses to the worst case at ``rho = 3`` (no dividend) — the price of
  not using the shape of the law.
* **The deployed base design is a.s. stable.** ``h = 0.95`` at ``rho = 5`` has
  exceedance **0** — string-stable across *every one* of the 65536 channel
  states (``||H~||_inf`` worst = 1.000, ``E[A^2] = 0.99997``).
* **Validated against the live channel.** The analytic exceedance matches the
  empirical fraction of time the actual V2V noise process lands in the unstable
  set across four orders of magnitude (e.g. 0.55/0.55, 7.6e-2/7.6e-2,
  9.5e-3/9.6e-3, 1.0e-3/1.1e-3, 0/0).

## Alternatives considered
- **Keep the binary worst-case certificate** — safe but silent on *how* safe;
  cannot trade a quantified risk for headway, and hides the two-tailed
  structure.
- **Mean-square as the design notion** — licenses non-robust headways (above);
  kept only as an instructive companion.
- **Monte-Carlo-only risk estimate** — needs enormous samples to resolve
  ``1e-4`` tails; the exact 16-bit law gives them in closed form.

## What was added
- `src/cacc/certificate.py` — `ChannelLaw`, `exceedance_prob`, `amp_at`,
  `chance_headway`, `meansquare_headway`, `chebyshev_headway`,
  `worstcase_headway`, `nominal_headway`, `certify_design`.
- `scripts/stochastic_cert_study.py`, `tests/test_certificate.py` (10 tests;
  suite 87).

## See also
[D-004] · [D-007] · [D-016] · [D-018] · flagship parts A ([D-021]) and B
([D-022])
