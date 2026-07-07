# R-04 — `analysis.py` reference: frequency-domain machinery

All verdicts in the project originate here (T-03/T-04).

## Functions

`gamma(omega, kind, h, kp, kd, tau, theta, kv, ka_eff, pred_theta_hat,
pred_base)` → complex Γ(jω) on the grid.
- kinds: `'acc'`, `'cacc'` (Ploeg family, gains kp/kd), `'cthp'`
  (Ma family, gains kp/kv/ka_eff).
- `theta` — transport delay on the (CACC/CTHP) feedforward path,
  evaluated exactly as e^{−jωθ}.
- `ka_eff` — the *effective* feedforward gain: pass w·ka. Robust analysis
  = evaluate at both interval ends (1±1/ρ)·ka and take the worst.
- `pred_theta_hat > 0` (cthp only) multiplies the delay term by the
  predictor lead P(s) (T-06) with averaging baseline `pred_base`.

`gamma_magnitude(...)` → |Γ|; `hinf_norm(...)` → grid max;
`is_string_stable(..., tol=1e-6)` → boolean verdict.

`min_stable_headway(kind, theta, kp, kd, tau, lo=0.01, hi=10.0, tol=1e-3,
kv, ka_eff, pred_theta_hat, pred_base)` — bisection on h; **raises
ValueError if h=10 is unstable** (callers use this as the infeasibility
signal; adapters cap tables at 10.0).

Closed forms (Ma Theorem III.2): `cthp_h_lb(ka, rho, tau0)` (rho=None ⇒
noiseless 2τ0/(1+ka)), `cthp_optimal(rho, tau0)` → (ka*, h*),
`cthp_gains_feasible(kp, kv, ka, h, rho, tau0)` (eqs. 16+28+RH),
`expected_w(rho, gammas=None)` → E[w].

## Grid and accuracy

`OMEGA_DEFAULT = logspace(−3, 2.5, 8000)` rad/s. The peaks that decide
verdicts in this project sit at 0.02–0.5 rad/s and are resolved to
< 0.1 %. When a *new* controller family is added, verify grid adequacy by
doubling the density once and comparing verdicts.

## Usage conventions (keep these exact in new code)

- Worst-case-over-noise helper used everywhere:

      def hinf_worst(h, rho, **kw):
          return max(hinf_norm("cthp", h, ka_eff=end * KA, **kw)
                     for end in (1 - 1/rho, 1 + 1/rho))

- The closed forms are for *re-tuned* designs and theory figures; the
  operational requirement is always the bisection (T-07) — do not "fix"
  adapter code back to `cthp_h_lb`.
- No Padé, ever: exact e^{−jωθ} costs nothing on a grid and the whole
  project lives at the stability boundary where approximants lie.

## Performance

One `hinf_norm` call ≈ 1 ms (8000-point complex evaluation). Bisection
≈ 15 calls. The adapter/scheduler tables (11 ρ × 7 θ × 2 ends ≈ 154
bisections) build in seconds and are shared across a platoon's followers
(`table=` parameter) — never rebuild per vehicle.

## Tests pinning this module

Pinned paper numbers (T-04 table), noiseless limit, both-ends boundary
behavior (D-009), predictor budget restoration
(`test_predictor_restores_delay_budget`), plus every study/certification
run recomputing `theory_checks` at runtime.
