# E-03 — QoS interference-zone experiment

**Script:** `qos_adaptive_study.py::fig2_zone_experiment` · **Scenario:**
`scenarios/qos_adaptive.yaml` · **Decision:** D-016 · **Report:** `09_…` §4.

## Question
Does the estimate→adapt pipeline keep a platoon string-stable through a
channel-quality collapse, and what does it cost?

## Design
- 8 followers, case-A gains, v0 = 20 m/s, τ = 0.5 s.
- ρ schedule: 10 → 3 (t ∈ [80, 175]) → 10 (zone entry between maneuvers).
- Leader: burst 1 (0.1 rad/s, fully in good channel), burst 2 at
  0.40 rad/s (≈ the fixed design's worst in-zone frequency), probe dither
  ±0.08 m/s² throughout.
- Three arms: fixed-good (h=0.95), fixed-worst (h=h_req(3)+margin=1.23),
  adaptive (h0=0.95, estimator+adapter on).

## Metrics & acceptance (also a certification suite)
- ρ̂ tracking: phase medians within 30 % of true (measured: 10.4/3.1/10.7).
- ‖H̃‖∞ at the in-force h per phase: adaptive ≤ 1 everywhere (measured
  0.99995–0.99999); fixed-good must show > 1 in-zone (1.00224) — the
  point of the experiment.
- Min gap ≥ 5 m (measured: 20.0 m — never approached).
- Per-phase L2 ratios recorded but interpreted per T-08 (transition
  windows contain commanded waves; ±14 m at the last follower here).

## Cost accounting (the honest numbers)
- Good phases: adaptive h ≈ 0.95 vs fixed-worst 1.23 → ~23 % shorter
  headway ≈ +25 % lane capacity where the channel allows it.
- In zone: adaptive h ≈ 1.76 vs omniscient-worst 1.23 — the **price of
  estimation uncertainty** (ρ̂/κ lands on the steep part of h_req; see
  T-07). Reducing κ via a better-characterized estimator directly
  converts to capacity.
- Recovery latency ≈ max_age = 20 s (safe-direction asymmetry, T-05).

## Known artifacts
ρ̂ startup spike (~15 for the first second, few samples); h plateau
wiggle (±0.05 s) from ρ̂ jitter — both cosmetic, both in the roadmap.
