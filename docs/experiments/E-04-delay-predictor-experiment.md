# E-04 — Delay-predictor experiment (the noise × delay 2×2)

**Script:** `qos_adaptive_study.py::fig3_delay_experiment` · **Theory:**
T-06 · **Report:** `09_…` §§3, 5.

## Question
Does timestamp-based feedforward prediction restore string stability
under realistic V2V latency — and what exactly was breaking it?

## Design
- θ = 0.15 s, h = 0.95 s (the case-A design), 8 followers, 150 s.
- Leader: single burst at ω* = 0.4546 rad/s (0.07235 Hz) — the
  *uncompensated* configuration's worst frequency; no probe (nothing is
  estimated here; θ̂ = θ given by stamps).
- 2 × 2 cells: {noiseless, ρ=5} × {uncompensated, predictor}.

## Results (run 20260707-143007)
| cell | ‖H̃‖∞ worst | max per-hop L2 |
|---|---|---|
| noiseless, uncompensated | 0.99999 | 0.989 |
| noiseless, predictor | 0.99999 | 0.983 |
| noisy, uncompensated | **1.00923** | 1.017 |
| noisy, predictor | **1.00000** | 1.013 |

Theory companion (fig1 right): h_req flat at 0.945 s with the predictor
for θ ≤ 0.3 s, vs 1.13/1.89/7.18 s uncompensated at θ = 0.1/0.15/0.3.

## The finding (sharper than the original hypothesis)
Delay alone at 0.15 s is survivable (noiseless budget ≈ 0.23 s); noise
alone is survivable (design point). The **interaction** — the delayed,
noise-inflated high end k̃a = 0.6 — is what violates the condition. The
predictor breaks precisely the interaction term.

## Interpretation cautions
- The noisy L2 columns sit on the realization envelope (±2 % here);
  the ‖H̃‖∞ column is the verdict.
- The predictor adds ~+1 % broadband per-hop L2 via its slope term —
  priced in T-06; does not affect the verdict.
- θ̂-mismatch sensitivity: behaves like an uncompensated |θ−θ̂|; with
  stamp-based estimation the mismatch is ~ms — negligible. Test the
  claim under clock skew if the ROS backends ever run on separate hosts.
