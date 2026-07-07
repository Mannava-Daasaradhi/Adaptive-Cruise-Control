# T-06 — Feedforward delay and the timestamp predictor operator

**Code:** `network.V2VLink.receive_predicted`, the `pred_theta_hat` branch
of `analysis.gamma` · **Decisions:** D-008 (delay budget), D-016
(predictor).

## The delay problem, quantified

V2V latency θ enters only the feedforward path: numerator term
k̃a·s²·e^{−θs}. At the paper's case-A design (fixed gains, ρ = 5) the
required headway explodes with θ:

| θ | h_req uncompensated |
|---|---|
| 0 | 0.945 s |
| 0.10 s | 1.133 s |
| 0.15 s | 1.888 s |
| 0.30 s | 7.181 s |

Deployment context: DSRC/C-V2X end-to-end latencies of 20–100 ms are
routine; congestion pushes higher. The nominal budget of the design is
~0.1 s — thin.

## The noise × delay interaction (a project finding)

At θ = 0.15 s: the *noiseless* design is still stable (‖H̃‖∞ = 0.99999 —
the noiseless budget is ≈ 0.23 s) and the *undelayed noisy* design is
stable by construction. The instability at (ρ=5, θ=0.15) lives in the
**interaction**: the delayed, noise-inflated high end k̃a = 0.6 violates
the condition (‖H̃‖∞ = 1.00923 at ω* = 0.45 rad/s). Fig3 of the study
shows the 2×2.

## The predictor

The receiver reconstructs a(t) from a(t−θ) using only its own received
samples and the message timestamps:

    â(t) = y(t) + θ̂ · slope,   slope = (mean(y over [t−T/2, t])
                                       − mean(y over [t−T, t−T/2])) / (T/2)

with T = `pred_base` = 0.4 s. Block averaging is essential: a two-sample
derivative of a multiplicative-noise signal has noise slope
~(Δw)·|a|/Δt ≫ signal slope — the averaged version suppresses it by
~1/√(N_samples per block).

**Exact frequency-domain operator** (same expression evaluated by theory
and executed by simulation — no model/implementation gap):

    P(s) = 1 + θ̂ · A(s) · (1 − e^{−sT/2}) / (T/2),
    A(s) = (1 − e^{−sT/2}) / (s·T/2)                (block average)

Low frequency: P(s) → 1 + θ̂s — the ideal first-order lead, canceling
e^{−θs} to first order exactly where the string-stability conditions
bind. High frequency: A(s) rolls the lead off, bounding noise
amplification.

## Result

h_req with the predictor stays **flat at 0.945 s for θ up to at least
0.3 s** — matched compensation (θ̂ = θ) removes latency from the design
problem. Residual costs: (i) mild broadband noise injection via the slope
(visible as ~+1 % per-hop L2 in noisy runs, verdict unaffected); (ii)
sensitivity to θ̂ error is second-order near the flat part of the curve
(mismatch δθ behaves like an uncompensated delay of |δθ| — budget ≈
0.1 s of estimation error before it matters, vs ms-accurate timestamp
estimation).

## Relation to prior art (for the paper's related-work section)

Lead/predictor compensation of communication delay in CACC exists (e.g.
Xing/Ploeg/Nijmeijer 2020, TVT — model-based H∞ synthesis); the elements
novel here are (a) the *timestamp-driven, receiver-only* construction
with an exactly analyzable operator, (b) its evaluation *jointly with the
multiplicative noise model* of Ma 2025, exposing the noise × delay
interaction, and (c) its integration into the adaptive pipeline where θ̂
comes from the same stamps that drive the estimator pairing.
