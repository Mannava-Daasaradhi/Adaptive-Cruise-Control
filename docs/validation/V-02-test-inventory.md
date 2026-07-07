# V-02 — Test inventory (55 tests, mapped to what they protect)

`python -m pytest tests -q` · Files: `test_*.py` (Ploeg-era suites),
`test_cthp.py` (13), `test_qos_adaptive.py` (14).

## Ploeg-era core (28 tests, v0.1 heritage)
Vehicle ODE + saturation; ACC/CACC laws and their Γ(s); link
delay-line/sampled-mode/loss semantics; platoon integration basics;
metrics. Protect: the foundation everything sits on.

## `test_cthp.py` — the Ma-2025 anchor (13)
- Closed forms pinned to printed digits: h_lb 0.9375 / ka* 0.3183 /
  h* 0.8727; noiseless limit 2τ0/(1+ka).
- Feasibility: case A & C yes, case B no. E[w] = 1.0482.
- ‖H̃‖∞ boundary behavior at BOTH interval ends (encodes D-009 — the
  test that was originally written wrong and taught us the low end
  binds).
- Noise machinery: determinism per seed, range, scaling, first-sample
  boundary semantics.
- Platoon-level: quiescent stays quiescent; case A attenuates; case B
  amplifies (empirical, long-window).

## `test_qos_adaptive.py` — the extension (14)
- rho_schedule: support switching; **bit-cache invariance across
  schedules** (same seed ⇒ same U′(t) — the property zone studies
  depend on).
- Estimator: convergence to ρ (over-estimate direction asserted);
  age-based expiry (the recovery bug's regression test); weak-excitation
  rejection (returns None, never a stale lie).
- Adapter: fixed-gain targets (0.945 @ ρ=5 — the case-A pin);
  monotonicity in ρ; slew-rate compliance (measured over 1 s windows —
  per-update semantics); hold-on-unobservable.
- Predictor: theory budget restoration (h_req(0.15, pred) ≈ h_req(0));
  delayed-ramp tracking through a real link.
- Closed loop: zone tracking (ρ̂ before/after, h opens, slew respected);
  guard rails (cthp-only, continuous-link-only raise).
- **Gain re-tuning (D-017):** scheduler reproduces case-A at ρ=10
  (kv 0.628, kp 0.009); ρ=2 feasible with h_req < 2; joint adaptation's
  in-force config in a ρ=2 zone has ‖H̃‖∞ ≤ 1+1e-4 with kv actually
  re-tuned (< 0.45).

## Conventions for new tests
1. Pin *numbers with provenance* (paper digits, measured landmarks), not
   incidental values.
2. Closed-loop tests use small configs (n=3, ≤ 120 s) — the suite must
   stay under ~30 s total.
3. A test that encodes a wrong assumption is itself a bug — when theory
   and test disagree, re-derive before "fixing" either (the D-009
   lesson, twice paid).
4. Regression tests for fixed bugs reference the D-record in their
   docstring.
