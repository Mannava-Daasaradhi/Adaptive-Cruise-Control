# D-009 — Worst case is the LOW end of the noise interval

**Date:** 2026-07-05 · **Status:** verified observation · **Type:** theory

## Context
The multiplicative channel noise makes the effective feedforward gain a
random variable k̃a = w·ka with w ∈ [1 − 1/ρ, 1 + 1/ρ). Intuition (mine,
initially wrong) said the *high* end — "more feedforward than designed
for" — should be the dangerous case, and an early unit test encoded that
assumption and failed.

## What is actually true
For CTHP string stability the **binding realization is the low end**
k̃a = (1 − 1/ρ)·ka. The paper's robust condition (their eq. (28b)) is a
*low-frequency* condition that tightens as k̃a decreases: with less
feedforward the platoon relies more on feedback, and the low-frequency
gain of H̃ crosses 1 first. The high end mainly affects high-frequency
behavior, which the headway term already rolls off.

## Evidence
- Case B (h = 0.65 s): ‖H̃‖∞ = **1.0035 at k̃a = 0.4** (unstable) but
  0.99991 at k̃a = 0.6 (marginally stable) — the instability lives at the
  low end only. Case A: 0.99997 (low) vs 0.99848 (high) — both stable but
  the low end is the tighter one.
- Closed form: h_lb(ka) = 2τ0(1 − (1−1/ρ)ka) / (1 − (1+1/ρ)²ka²) — the
  numerator carries the *low* endpoint, the denominator the high one; for
  the paper's parameters the numerator effect dominates where designs live.
- Encoded as `test_hinf_stable_above_bound_unstable_below`
  (`tests/test_cthp.py`), which checks **both** interval endpoints.

## Consequences
- The delay-budget study ([D-008]) evaluates h_min(θ) at the low end as the
  robust case — its delay budget (θ ≈ 0.42 s) is the *largest*, another
  counter-intuitive consequence: less feedforward ⇒ less to delay.
- Any future gain-scheduling work must treat "channel got worse" as
  "effective ka dropped", not "grew".
- Process note: the failed test was the *assumption* being wrong, not the
  code — the fix was re-deriving from eq. (28b), then correcting the test.
