# E-05 — Deep-zone gain re-tuning experiment

**Script:** `qos_adaptive_study.py::fig4_gain_retuning` · **Decision:**
D-017 · **Theory:** T-07/T-08.

## Question
Below ρ ≈ 2.5, h-only adaptation hits the fixed-gain wall. Does joint
(kp, kv) + h adaptation actually stabilize a platoon there?

## Design
- Deep zone: ρ 10 → **2** (t ∈ [80, 175]) → 10; same leader/probe as E-03;
  8 followers.
- Two arms, both fully adaptive estimator-side: h-only (D-016) vs joint
  gains+h (`adapt_gains: true`).
- The live ‖H̃‖∞ of the configuration *actually in force* (mean h, mean
  live gains, true ρ, worst noise end) is evaluated at 4 s probes — the
  center panel of fig4 and the strongest visual in the project: the red
  curve (h-only) crossing above 1 in the zone, the green one staying under.

## Results (run 20260707-144337)
| arm | zone kv | zone h | zone ‖H̃‖∞ in force |
|---|---|---|---|
| h-only | 0.63 (stuck) | 2.50 (= h_max, saturated) | **1.019 — the wall, demonstrated** |
| joint | **0.353** | 2.33 | **0.99999 — stable** |

Scheduler sanity built into the same run: kv(ρ=10) = 0.628 ≈ case-A's
0.63 (the 90 %-rule reproduces the paper's design point, D-017).

## Acceptance (certification suite `deep-zone-gains`)
- In-force ‖H̃‖∞ ≤ 1 + 1e-4 inside the ρ=2 zone, all seeds (joint arm).
- Min gap ≥ 5 m.

## Interpretation cautions
- The h-only arm's *time series* does not visibly explode (‖H̃‖ = 1.019
  ⇒ ~2 %/hop growth at the worst frequency — slow; D-010 again). The
  in-force-hinf panel is the verdict; the arm exists to *demonstrate the
  wall*, not to crash cars.
- Gains and h slew toward a consistent target (adapter consumes the
  scheduler's h_req); intermediate configs interpolate between certified
  designs (T-08 layer 4).
- kv re-tuning trades feedback authority for noise robustness: transient
  tracking in the zone is softer (visible as slightly larger maneuver
  errors) — the correct physical price, worth a sentence in the paper.
