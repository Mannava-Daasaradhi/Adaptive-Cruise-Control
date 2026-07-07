# D-010 — Time-domain contrast at the paper's operating points is subtle

**Date:** 2026-07-05 · **Status:** verified observation · **Type:** expectations management

## Context
The natural course-demo expectation is: stable case A shows waves shrinking
dramatically, unstable case B shows them exploding. The first reproduction
runs looked "disappointingly similar" between A and B, which initially read
like a bug.

## What is actually true
The paper *designed* both cases at the stability boundary: case A sits just
inside (‖H̃‖∞ ≈ 0.9990–0.99997 across the noise interval) and case B just
outside (up to 1.0035 at the binding low end, [D-009]). Per-hop L2 growth
is therefore ≲ 0.4 % — over 12 followers that compounds to only a few
percent, visually mild in any time series. The paper's own Figs. 7 vs 10
have exactly the same character; the decisive verdicts live in the
**frequency domain** (‖H̃‖∞ vs 1), not in dramatic time-domain divergence.

Physical intuition: string instability at the margin is a *slow* geometric
growth per hop, not a blow-up — with N = 12 and 0.35 %/hop you get ~4 %
total amplification, invisible next to ±1.3 m error waves.

## Consequences
- The reproduction figures report per-vehicle L2 **bars** and per-hop
  ratios alongside time series, because that's where the A/B contrast is
  quantitative (`fig3_time_domain.png`).
- The course demo's visual drama comes from elsewhere: ACC-vs-CACC
  comparisons (large contrast), the delay sweep h_min(θ) exploding past the
  budget ([D-008]), and the animated platoon view (D-015).
- Downstream, this is also why single noisy runs cannot classify A vs B by
  per-hop ratios alone — noise-realization scatter (±5 %) exceeds the
  design separation (±0.4 %/hop); see the verdict logic discussion in
  [D-014] and `08_ROS2_Architecture.md` §4.
