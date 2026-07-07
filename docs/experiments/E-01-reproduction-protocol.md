# E-01 — Base-paper reproduction protocol

**Script:** `reproduce_base_paper.py` · **Reference run:**
`results/base_paper/20260705-213624` · **Report:** `07_…_Results.md` §§1–3.

## Purpose
Establish the credibility anchor: our stack reproduces Ma 2025 to its
printed digits before we claim anything beyond it.

## Design
- Parameters exactly per D-006 extraction (τ0=0.5, d=5 m, N=12, ρ=5,
  their 16 γ's, cases A/B/C, eq.-46 sine leader, 200 s, seed 1).
- Five figure groups mapping to their Figs. 5–13 (D-007 scope).
- `theory_checks` recomputed at run time inside `metrics.json` so figures
  and closed forms cannot drift apart silently.

## Acceptance (all pinned as unit tests too)
| check | expected |
|---|---|
| h_lb(0.5, ρ=5) | 0.9375 s exact |
| ka*, h* (ρ=5) | 0.31831, 0.87268 |
| case-A/B/C feasibility | yes / no / yes |
| ‖H̃‖∞ case A (low/nom/high k̃a) | 0.99997 / 0.99905 / 0.99848 |
| ‖H̃‖∞ case B low end | 1.00350 (> 1: designed unstable) |
| E[w] | 1.0482 |

## Interpretation caveats
- A/B time-domain contrast is ≲0.4 %/hop by the paper's own design —
  do not expect visual drama (D-010); the L2 bars + frequency plots
  carry the verdict.
- v0 = 20 m/s is our choice (speed-independent error dynamics, D-006).

## Change policy
This experiment is frozen. Any code change that shifts a number here is
a regression by definition — fix the code, never the expectation.
