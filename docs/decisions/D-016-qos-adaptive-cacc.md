# D-016 — Novel extension: QoS-aware CACC (estimate + predict + adapt)

**Date:** 2026-07-07 · **Status:** accepted (offline core validated) ·
**Type:** research scope / architecture

## Context
The user asked for a "more complex simulation, novel idea, Q1-worthy" and
(separately) framed the project ambition as **product-grade virtual
validation that can replace hardware testbeds**. Asked which direction
(AskUserQuestion), the user chose the full pipeline over the three lighter
options (channel-adaptive headway only / delay compensation only /
event-triggered beaconing): **joint estimation, prediction and headway
adaptation over imperfect V2V**.

## The idea in one paragraph
Ma 2025 gives h_lb(ka, ρ) for a *known, constant* ρ with gains re-tuned
per ρ and no latency. We close the loop the literature leaves open: each
follower (1) **estimates** its own link's noise level ρ̂ online from
beacon-vs-radar residuals — possible because the n-bit channel has
strictly bounded support, so a windowed quantile of |ŵ−1| inverts to a
conservative ρ̂; (2) **predicts** the delayed feedforward with a
timestamp-based lead (slope of two window-averaged blocks of its own
received samples), whose exact frequency-domain operator we evaluate in
the same analysis code; (3) **adapts** its time headway with a
rate-limited slew toward the *fixed-gain* requirement
h_req(ρ̂/κ, θ̂) — a bisection table of `min_stable_headway`, worst-case
over the noise interval — NOT the paper's closed form, which presumes
re-tuned gains (see the fixed-vs-retuned divergence below).

## Key design decisions inside the extension
1. **Estimator uses the support bound, not moments.** |w−1| ≤ 1/ρ strictly
   ⇒ quantile inversion *over-estimates* ρ (assumes the channel better);
   the κ = rho_safety divisor flips the bias to the conservative side.
   Age-based sample expiry (20 s) — count-based windows recover too slowly
   after a zone (measured: ρ̂ stuck at 3.1 after returning to ρ=10).
2. **Persistent excitation by probing dither** (±0.08 m/s² multi-sine on
   the leader): the channel is unobservable without communicated
   acceleration content — standard adaptive-control requirement, honest
   about it in the scenarios.
3. **Predictor slope from averaged received samples** (T = 0.4 s
   baseline): a raw two-sample derivative of the multiplicative-noise
   signal is useless (noise slope ≫ signal slope); block averaging makes
   the lead implementable and its exact operator
   P(s) = 1 + θ̂·A(s)(1−e^{−sT/2})/(T/2) analyzable.
4. **Adaptation targets the fixed-gain requirement.** Discovered en route:
   case-A gains (kv = 0.63) leave the eq.-29 feasible region below ρ ≈ 3,
   and the paper's eq.-28a is sufficient-only — direct ‖H̃‖∞ bisection is
   the ground truth. h-only adaptation has a feasibility wall at
   ρ* ≈ 1.8 (h ≤ 10 s) and an economic wall near ρ ≈ 2.5.
5. **Per-follower h is admissible** because each hop's error-propagation
   TF depends only on its own follower's headway (one-vehicle look-ahead);
   the 0.05 s/s rate limit keeps the configuration quasi-static between
   frozen string-stable designs.
6. **rho_schedule with a ρ-independent bit cache** (`V2VLink`): the same
   seed produces the same bit pattern U′(t) under any schedule — zone
   studies stay seed-comparable.

## Headline results (results/qos_adaptive/20260707-143007)
- Fixed-gain requirement hits the case-A design exactly: h_req(ρ=5) =
  0.945 s ≈ the paper's chosen h = 0.95 s.
- Fixed-vs-retuned divergence: 5.28 s vs 1.71 s at ρ = 2; ρ* ≈ 1.8.
- **Predictor holds h_req flat at 0.945 s for θ up to 0.3 s** (vs 7.18 s
  uncompensated at θ = 0.3) — latency effectively removed from the design.
- **Noise × delay interaction:** at θ = 0.15 s, noiseless ‖H̃‖∞ = 0.99999
  and noisy-compensated = 1.00000, but noisy-uncompensated = 1.00923 —
  the interaction is the killer, and the predictor breaks it.
- Zone experiment (ρ 10→3→10): ρ̂ medians 10.4/3.1/10.7; adaptive platoon
  restores ‖H̃‖∞ ≤ 0.99995 inside the zone where fixed-good runs at
  1.00224, and returns to h = 0.95 s in good phases where the worst-case
  design permanently pays 1.23 s (≈ +25 % lane capacity at 20 m/s).
- Cost of estimation uncertainty made measurable: zone h = 1.76 s vs the
  omniscient 1.23 s, because ρ̂/κ lands on the steep part of h_req.

## Verification
`tests/test_qos_adaptive.py` (11 tests; suite 52/52): rho-schedule support
scaling and bit-cache invariance, estimator convergence/expiry/rejection,
adapter targets (0.945 pin), rate limit + hold-on-unobservable, predictor
budget restoration (theory), delayed-ramp tracking, closed-loop zone
tracking, and guard rails (cthp + continuous link only).

## Consequences
- Doc `09_QoS_Adaptive_CACC.md` is the paper skeleton for this extension.
- ROS 2 integration of the pipeline is *specified* (not yet built) in
  `10_Handoff_Plan.md` — the offline core is the validated reference the
  distributed implementation must match.
- Future work ranked: online gain re-tuning below ρ ≈ 2.5; staggered
  transitions; ρ̂ smoothing.
