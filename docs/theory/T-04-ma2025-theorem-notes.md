# T-04 — Ma 2025 Theorem III.2: implemented closed forms and pinned numbers

**Code:** `analysis.cthp_h_lb`, `analysis.cthp_optimal`,
`analysis.cthp_gains_feasible`, `analysis.expected_w` · **Paper:** Ma,
Pagilla & Darbha, T-ITS 26(1), 2025, DOI 10.1109/TITS.2024.3498701.

## The noise model feeding the theorem

w(t) = (1 − 1/ρ) + (1/ρ)·Σ_{j} z_j·2^{−j}, z_j ~ Bernoulli(γ_j), n = 16
bits ⇒ w ∈ [1 − 1/ρ, 1 + 1/ρ) with strictly bounded support (the property
the ρ̂ estimator exploits, T-05). Effective feedforward gain
k̃a = w·ka ∈ [(1−1/ρ)ka, (1+1/ρ)ka].

## Implemented closed forms

Robust minimum headway (their eq. 17), requires 0 < ka < 1/(1+1/ρ):

    h_lb(ka, ρ) = 2 τ0 (1 − (1−1/ρ) ka) / (1 − (1+1/ρ)² ka²)

Noiseless limit (Remark 3.3): h_lb → 2τ0/(1+ka).

Headway-minimizing gain and bound (eqs. 18–19), with rs = 1/√ρ:

    ka* = ((1 − rs)/(1 + rs)) / (1 + 1/ρ)
    h*  = τ0 (1 + rs)² / (1 + 1/ρ)

Gain feasibility (eqs. 16 + 28 + Routh–Hurwitz), γ := kv + h·kp:

    (16)   0 < ka < 1/(1 + 1/ρ)
    (28a)  γ ≤ (1 − (1+1/ρ)² ka²) / (2 τ0)      [h-independent ceiling!]
    (28b)  γ ≥ sqrt(2 kp (1 − (1−1/ρ) ka) + kv²)
    (RH)   γ > τ0 · kp                            [internal stability]

## Pinned numbers (unit tests, `tests/test_cthp.py`)

| quantity | paper | ours |
|---|---|---|
| h_lb(0.5, ρ=5), τ0=0.5 | 0.9375 s | 0.9375 (exact) |
| ka*(ρ=5) | 0.3183 | 0.31831 |
| h*(ρ=5) | 0.8727 s | 0.87268 |
| E[w], their 16 γ's | – | 1.0482 |
| case A (0.009, 0.63) feasible / case B (h=0.65) | yes / no | yes / no |

These are the project's credibility anchor: any regression that shifts
them fails CI loudly.

## Reading the conditions — the intuitions used throughout

- **(28a)** is the high-frequency/high-noise-end condition; its ceiling on
  γ is *independent of h* — the root of the fixed-gain wall (T-07,
  D-017). More feedback authority (kv) than the ceiling allows can never
  be "paid off" by headway.
- **(28b)** is the low-frequency/low-noise-end condition; it is what makes
  the LOW end of the noise interval binding in most of the design space
  (D-009) and what h buys you (γ grows with h).
- The **corner** of [28b ≤ γ ≤ 28a] is where the paper's designs sit —
  and the 90 %-of-ceiling scheduler rule (D-017) is precisely "stay a
  fixed relative distance inside 28a", which reproduces case-A at ρ = 10.

## Scope caution

The closed forms presume gains *re-tuned per ρ*. For an operating platoon
with fixed gains, use the bisection requirement (T-07) — treating h_lb as
"the" requirement was an early-development error worth remembering.
