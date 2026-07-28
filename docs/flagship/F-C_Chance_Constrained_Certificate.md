# Flagship Part C — Chance-constrained Risk Certificate (in depth)

**Claim.** The exact 16-bit channel law turns the binary string-stability
guarantee `||H~||_inf <= 1` into a calibrated failure probability
`P(||H~||_inf > 1 | h, rho)`, and into a risk-parameterised headway `h_cc(eps)`
that replaces the QoS layer's ad-hoc safety margin with a budget — the
certificate backbone of the flagship.

Companion ADR: [D-023](../decisions/D-023-chance-constrained-certificate.md).
Overview: [00_Flagship_Overview](00_Flagship_Overview.md).

---

## 1. The problem: a binary guarantee hides how safe you are
The Ma-2025 guarantee `||H~||_inf <= 1` is enforced at the channel's **worst
case**: the effective feedforward gain `ka_eff = w·ka` is taken at the extreme
of `w`'s support. But `w = (1 - 1/rho) + U'/rho` with `U' = sum_j z_j 2^-j` and
`z_j ~ Bernoulli(gamma_j)` is a **random variable** whose distribution is known
exactly from the 16 channel-bit expectations `gamma_j`. Enforcing the worst case
pays for an event of probability `prod_j (1 - gamma_j) ≈ 1e-6` and says nothing
about *how much* margin a given design actually has.

## 2. The exact channel law
`cacc.certificate.ChannelLaw.build(rho)` forms the **exact** distribution of `w`
by iterated convolution of the 16 two-point bit laws:

    vals, p = {0}, {1}
    for j in 0..15:  vals ← vals ∪ (vals + 2^-j);  p ← p·(1-γ_j) ⊕ p·γ_j
    w = (1 - 1/rho) + vals/rho          # 2^16 = 65536 atoms

Checks: `sum p = 1`; `mean` matches `expected_w(rho)` (paper eq. 12); `var`
matches the closed form `sum_j 2^-2j γ_j(1-γ_j) / rho²`; support
`[1 - 1/rho, 1 + 1/rho)`. 65536 atoms is trivial to enumerate, so every quantile
and moment is exact — no Monte-Carlo sampling error in the tails.

## 3. A structural finding: the instability is two-tailed
The channel amplification `A(w) = ||H~(.; ka_eff = w·ka)||_inf` is **not**
monotone in `w`. At a degraded channel the feedforward is too *weak* at low `w`
(deep fade) **and over-amplified** at high `w` (the multiplicative noise inflates
the `ka_eff·s²` term), so string stability holds only for `w` in a **middle
band** and the unstable set is generally *two-tailed*
`{w < w_lo} ∪ {w > w_hi}`.

This is why the paper's closed form `cthp_h_lb` carries the `(1 + 1/rho)`
high-end term, and why the robust worst-case headway must stabilise **both**
rare extremes. The numeric worst-case headway (min `h` with `max_w A(w) <= 1`)
matches `cthp_h_lb` to within the known `||H~||_inf ≈ 1 ± 5e-3` numeric subtlety
(the numeric value is the exact frequency-domain boundary; the closed form is
slightly conservative). All amplification is computed on the project-standard
frequency grid `OMEGA_DEFAULT` so the numbers match the rest of the analysis.

## 4. The three certificates
Because `A(w)` can be two-tailed, exceedance is summed over the atoms directly —
no monotonicity shortcut.

- **exceedance(h) = P(||H~||_inf > 1)** — `A(w)` swept over the atoms; the *risk
  map* of any design (`exceedance_prob`, `certify_design`).
- **chance-constrained `h_cc(eps)`** (headline) — smallest `h` with
  `exceedance(h) <= eps`, by bisection on `h` against the exact two-tailed
  exceedance. The risk-parameterised headway.
- **mean-square `h_ms`** — smallest `h` with `E[A(w)²] <= 1`. The classic
  stochastic notion; reported **with a caveat** (§6).
- **distribution-free `h_cheb(eps)`** — a two-sided Chebyshev interval
  `t = sigma/sqrt(eps)`, designed stable for all `w in [E[w]-t, E[w]+t]`.
  Robust to uncertainty in the `gamma_j`, using only mean and variance.

`h_cc(eps)` is the principled replacement for the heuristic `rho_safety` divisor
of [D-016]: instead of an arbitrary "assume the channel is 1.15× worse", set the
headway from an explicit failure-probability budget.

## 5. Results (case-A design, headline channel `rho = 3`)

| headway | value [s] | exceedance `P(||H~||>1)` | note |
|---|---|---|---|
| `h_nom` (design at `E[w]`) | 0.726 | **0.55** | naive / unsafe |
| `h_ms` (mean-square) | 0.994 | **0.076** | *too permissive* |
| `h_cc(1e-2)` | **1.033** | 0.0095 | 1-in-100 budget |
| `h_cc(1e-3)` | 1.052 | 0.0010 | 1-in-1000 |
| `h_cheb(1e-2)` | 1.154 | 0 | distribution-free (≈ worst-case) |
| `h_wc` (worst-case, both tails) | 1.154 | 0 | ≈ paper `cthp_h_lb` = 1.20 |

* **Real throughput dividend.** Accepting a 1-in-100 instantaneous-exceedance
  budget trims **10.5 %** off the worst-case headway (1.154 → 1.033 s) — a
  genuine capacity gain, because the worst case pays to stabilise two
  `~1e-6`-probability tails.
* **The deployed base design is a.s. stable.** `h = 0.95` at `rho = 5` has
  exceedance **0** — string-stable across *every one* of the 65536 channel
  states (`||H~||_inf` worst = 1.000, `E[A²] = 0.99997`).

**Figure** (`fig1_stochastic_cert.png`, 2×2): (a) the CDF of `w` with the
two-tailed unstable band shaded — string stability needs `w` in a middle band;
(b) the risk curve `exceedance(h)` with the `eps` budgets and their `h_cc(eps)`;
(c) the certified headway ladder `h_nom < h_ms < h_cc(eps) <= h_cheb(eps) ≈
h_wc`; (d) validation — analytic exceedance vs the empirical fraction of time
the *live* V2V channel lands in the unstable set, on `y = x`.

**Validated against the live channel.** The analytic exceedance matches the
empirical fraction across four orders of magnitude: `0.55/0.55`, `0.076/0.076`,
`9.5e-3/9.6e-3`, `1.0e-3/1.1e-3`, `0/0`.

## 6. Honest limitations
- **Mean-square is too permissive.** `h_ms = 0.994 s` carries a **7.6 %**
  exceedance — riskier than a 1 % budget and not robustly string-stable. It is
  kept only as an instructive companion that *justifies* the worst-case /
  chance-constrained notion, not as a design to deploy.
- **Chebyshev is honest but blunt.** With only mean+variance it collapses to the
  worst case at `rho = 3` (no dividend) — the price of not using the shape of
  the law.
- **Numeric vs closed form.** The `||H~||_inf ≈ 1 ± 5e-3` subtlety means the
  numeric worst-case and the paper's `cthp_h_lb` differ by up to ~0.05 s; both
  are reported.

## 7. Relation to the rest
Part C is the backbone that quantifies the risk Parts A and B trade against:
`h_cc(eps)` is the calibrated headway law that could replace the heuristic
margin used inside Part A's predictive adapter, and the same exceedance map
grades any headway the platoon adopts. It extends the certification harness
([D-018]) with a probabilistic, per-`(h, rho)` verdict.

## 8. Reproduce
```bash
python scripts/stochastic_cert_study.py    # -> results/stochastic_cert/<stamp>/
pytest tests/test_certificate.py -q         # 10 tests
```

[D-016]: ../decisions/D-016-qos-adaptive-cacc.md
[D-018]: ../decisions/D-018-certification-harness.md
