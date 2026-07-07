# E-02 — Delay & packet-loss study (fixed case-A gains)

**Script:** part of `reproduce_base_paper.py` (fig5 + loss table) ·
**Decision:** D-008 · **Report:** `07_…_Results.md` §4.

## Question
With the paper's design frozen, which V2V imperfection actually binds:
latency or loss?

## Design
- **Delay branch:** h_min(θ) by bisection, θ ∈ [0, 0.5] s, evaluated at
  three effective gains: noiseless k̃a=0.5, noisy low end 0.4 (the robust
  case per D-009), nominal 0.524 (= ka·E[w]).
- **Loss branch:** time-domain, 10 Hz beacons, ZOH reconstruction, 20 ms
  link delay, Bernoulli loss ∈ {0, 10, 30, 50} %, 5 seeds; metric = worst
  per-hop L2 amplification.

## Headline numbers (reference run)
| effective gain | h_min(0) | design h=0.95 survives to | h_min(0.5 s) |
|---|---|---|---|
| noiseless | 0.789 s | θ ≈ 0.23 s | 5.69 s |
| low end | 0.945 s | θ ≈ 0.42 s | 1.19 s |
| nominal | 0.751 s | θ ≈ 0.19 s | 8.35 s |

Loss: max L2 amp 1.007 (0 %) → 1.013 (50 %) — graceful.

## Conclusions
1. **Delay, not loss, is the binding constraint** at realistic DSRC/C-V2X
   latencies (~0.1–0.2 s of budget only).
2. Loss under ZOH ≈ added effective delay of order the beacon period —
   which is why it barely moves the metric at 10 Hz.
3. Counter-intuitive: larger feedforward gain ⇒ *more* delay-sensitive
   (the s²e^{−θs} term), so the noisy low end has the flattest curve.

## Follow-ups that grew out of this experiment
The θ-budget finding motivated the timestamp predictor (T-06, E-04); the
fixed-gain framing motivated T-07. This is the pattern: every experiment
doc ends by naming what it seeded.
