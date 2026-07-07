# V-01 — Verification & validation strategy

The layered defense that lets this simulation claim testbed-grade
credibility (the product argument, P-02). Each layer catches what the
previous cannot.

## Layer 1 — Pinned-theory unit tests (55, `tests/`)
Every closed form the base paper prints is a test; every algorithmic
property we rely on (noise determinism, estimator convergence/expiry,
rate limits, predictor budget restoration, scheduler design-point
reproduction) is a test. Gate: `pytest -q` after every change (RB-02).
Catches: regressions in equations and algorithms.

## Layer 2 — Runtime theory checks
`reproduce_base_paper.py` recomputes `theory_checks` inside its
`metrics.json`; studies tabulate ‖H̃‖∞ next to every experimental claim;
fig4 evaluates the *in-force* configuration live during adaptive runs.
Catches: figure/theory drift, invalid quasi-static excursions.

## Layer 3 — Cross-backend validation (V-03)
The distributed ROS 2 backend must agree with the monolithic core:
noiseless per-hop ratios within ~2 % (measured 0.976–0.996 vs 0.9973);
noisy runs inside the seed envelope at matching noise hold. Catches:
real-time integration artifacts (the D-013/D-014 class), transport bugs.

## Layer 4 — Monte-Carlo certification (D-018, E-06)
Seeded scenario matrix × acceptance criteria → PASS/FAIL report. Catches:
statistical fragility a single run hides; provides the stakeholder
artifact.

## Layer 5 — Nonlinear safety checks
Min-gap ≥ 5 m on every certified run; saturation is outside the linear
theory, so it is checked empirically, never assumed. Catches: everything
the L2 framework cannot see.

## Change-management rules
1. Tests may grow, never shrink; expectations change only with a
   D-record explaining why the old expectation was wrong.
2. Every quotable number traces to a `results/**/metrics.json` path.
3. New feature ⇒ new tests + a certification suite + an E-doc,
   in the same change (the D-017/D-018 pattern).
4. `certify.py --quick` is the pre-merge smoke; full 5-seed before any
   external claim (paper submission, demo).

## Known verification gaps (tracked, not hidden)
- The quasi-static adaptation argument is engineering-grade, not a
  theorem (T-08 "for the journal version").
- ROS QoS pipeline not yet cross-validated (Workstream A).
- No hardware-in-the-loop anchor point — see P-02 for why the claim
  structure still stands and what a minimal HIL anchor would look like.
