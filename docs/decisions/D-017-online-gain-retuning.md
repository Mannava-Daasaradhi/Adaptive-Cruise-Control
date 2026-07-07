# D-017 — Online gain re-tuning breaks the fixed-gain wall

**Date:** 2026-07-07 · **Status:** accepted, validated offline ·
**Type:** research / control design

## Context
D-016 discovered the **fixed-gain wall**: with case-A gains frozen, the
required headway explodes below ρ ≈ 3 (5.28 s at ρ = 2 vs the re-tuned
bound 1.71 s) and h-only adaptation is infeasible below ρ* ≈ 1.8. Root
cause: the eq.-28a ceiling γ = kv + h·kp ≤ (1 − (1+1/ρ)²ka²)/(2τ0) is
h-independent — γ only *grows* with h, so once the fixed kv exceeds the
ceiling, no headway restores the low-frequency string-stability condition
efficiently. This was ranked future-work #1 in `09_QoS_Adaptive_CACC.md`;
the user then asked for the project to be pushed further, so it was built.

## Decision — the 90 %-of-ceiling rule (`GainScheduler`)
Re-tune per estimated channel quality:

    kv(ρ) = kv_frac · (1 − (1 + 1/ρ)² ka²) / (2 τ0),   kv_frac = 0.9
    kp(ρ) = kp0 · kv(ρ) / kv0                          (kp/kv ratio kept)

then bisect the required headway at those gains (worst case over both
noise-interval ends), tabulated once over the ρ grid and interpolated at
runtime. Gains slew with a relative rate limit (`gain_rate` = 0.05 /s)
exactly like the headway; the `HeadwayAdapter` consumes the scheduler's
h_req instead of its fixed-gain table, so headway and gains move toward a
*consistent* target design together.

**Why this rule and not an optimizer:** (i) it is closed-form, monotone in
ρ, and trivially verifiable; (ii) it *reproduces the paper's own case-A
choice at the design point* — kv(10) = 0.628 vs their 0.63, kp(10) = 0.009
— strong evidence this is the recipe the authors applied implicitly, which
makes the scheduler a principled generalization rather than an ad-hoc
tuner; (iii) 90 % leaves a uniform margin to the ceiling at every ρ.

## Alternatives considered
1. Full (kp, kv) optimization per ρ̂ (minimize h_req subject to eqs. 16/29)
   — better headways in principle, but non-convex corner-chasing against
   an *estimated* ρ invites chattering; rejected for the first iteration.
2. Switching between a small set of certified gain sets — robust but
   coarse; the rate-limited continuous schedule subsumes it.
3. Adapting ka — rejected: ka enters the noise interval itself
   ((1±1/ρ)ka), coupling estimation and design in a loop that is hard to
   argue quasi-statically.

## Safety argument (quasi-static, same as D-016)
Every point of the schedule (kv(ρ), kp(ρ), h ≥ h_req(ρ) + margin) is a
string-stable frozen design (verified by construction via the bisection).
Both h and gains are rate-limited, so the configuration moves slowly
through a *continuum of certified designs*; transient excursions between
certified points are bounded by the slew rates and covered by the margin.
The in-force ‖H̃‖∞ is additionally monitored live in fig4 and in the
certification suite (D-018) — trust, but verify.

## Evidence (results/qos_adaptive/20260707-144337, tests 55/55)
Deep zone ρ: 10 → 2 → 10, 8 followers, case-A start:

| variant | zone kv | zone h | zone ‖H̃‖∞ in force |
|---|---|---|---|
| h-only (D-016) | 0.63 (stuck) | 2.50 (= h_max, saturated) | **1.019 — unstable** |
| joint gains + h | **0.353** | 2.33 | **0.99999 — stable** |

Pinned as `test_gain_scheduler_reproduces_case_a_at_rho10`,
`test_gain_scheduler_breaks_the_wall_at_rho2`,
`test_joint_adaptation_stabilizes_deep_zone`.

## Consequences
- The QoS-adaptive pipeline now covers the **entire** ρ range of the noise
  model — no feasibility hole left; the paper's contribution list gains
  its strongest item.
- `SimResult.gains` records the live (kp, kv); fig4 of the study shows the
  in-force ‖H̃‖∞ trace crossing back under 1 as the gains re-tune.
- ROS integration (Workstream A of `10_Handoff_Plan.md`) must swap the
  controller params at the estimator rate exactly as `platoon.py` does
  (stateless CTHP makes the swap safe).
