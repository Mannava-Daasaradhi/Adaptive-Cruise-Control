# D-007 — Reproduction scope and validation style

**Date:** 2026-07-05 · **Status:** accepted · **Type:** methodology

## Context
"Reproduce the base paper" can mean anything from re-plotting one figure to
re-deriving every lemma. The course deliverable needs a defensible,
regression-proof reproduction that a grader can re-run.

## Decision — scope
`scripts/reproduce_base_paper.py` (one entry point, `--quick` smoke mode)
reproduces, in order:
1. **Design space** (their Fig. 5): h_lb(ka) curves from Theorem III.2 for
   ρ ∈ {2, 5, 10, ∞} with the optima (ka*, h*) marked
   → `fig1_design_space.png`.
2. **Frequency domain** (their Figs. 6, 9): |H̃(jω)| for cases A and B at
   the three effective feedforward gains k̃a ∈ {0.4 (low end), 0.524
   (nominal = ka·E[w]), 0.6 (high end)} → `fig2_frequency_response.png`.
3. **Time domain** (their Figs. 7, 8, 10): 12-follower runs of cases A/B
   under the 16-bit noisy channel, δ_i time series, the noisy communicated
   acceleration w(t)·a1(t) on link 1→2, and per-vehicle L2 bars
   → `fig3_time_domain.png`.
4. **Optimal case** (their Figs. 12, 13): case C run + platoon-length
   comparison x0 − xN vs case A → `fig4_optimal_case.png`.
5. **Extension** (ours, [D-008]): delay sweep + packet-loss study
   → `fig5_delay_extension.png`, `metrics.json`.

## Decision — validation style
Every number the paper *prints* is pinned as a unit test so a regression
cannot pass silently (`tests/test_cthp.py`):
- `cthp_h_lb(0.5, 5, 0.5) == 0.9375` (their eq. 17 example),
- `cthp_optimal(5, 0.5) == (0.31831, 0.87268)` (eqs. 18–19),
- case-A/C gains feasible, case-B infeasible (eq. 16 + 29 + Routh–Hurwitz),
- E[w] = 1.0482 for their γ's,
- noiseless limit h_lb → 2τ0/(1+ka),
- ‖H̃‖∞ ≤ 1 boundary behavior at both noise-interval endpoints ([D-009]),
- platoon-level: quiescent platoon stays quiescent; case A attenuates,
  case B amplifies (empirical L2 per-hop).
Additionally `theory_checks` inside `metrics.json` recomputes the closed
forms at run time so the figures and the constants can never drift apart.

## Reference run
`results/base_paper/20260705-213624/` (N = 12, 200 s, seed 1) — the
numbers quoted in `07_Base_Paper_Reproduction_Results.md` §§1–3.

## Consequences
- 41/41 tests green is the project's definition of "reproduction intact".
- Any gain/parameter change that breaks agreement with the paper fails CI
  loudly rather than producing subtly wrong figures.
