# D-008 — Extension design: delay and packet loss at fixed case-A gains

**Date:** 2026-07-05 · **Status:** accepted · **Type:** methodology / novelty

## Context
The base paper covers *noise* → minimum headway. The project's claimed
novelty ([D-001]) is extending the robustness question to the other two
V2V imperfections: **latency** and **packet loss**. The extension must stay
comparable to the paper, so it re-uses the paper's own design instead of
re-tuning gains per condition.

## Decision
Keep the paper's case-A gains (kp = 0.009, kv = 0.63, ka = 0.5) **fixed**
and quantify:

**(a) Delay budget** — add a feedforward transport delay θ to the error
propagation TF (numerator term k̃a·s²·e^{−θs}; our extension of their
eq. (27)) and bisect the minimum string-stable headway h_min(θ) for
θ ∈ [0, 0.5] s at three effective gains: noiseless (k̃a = 0.5), noisy low
end (k̃a = 0.4, the binding case per [D-009]), nominal (k̃a = 0.524).

**(b) Loss study** — time-domain: 10 Hz beaconing, ZOH reconstruction,
20 ms link delay, Bernoulli loss ∈ {0, 10, 30, 50}%, 5 seeds; metric =
worst per-hop L2 amplification.

## Findings (results/base_paper/20260705-213624)
| effective gain | h_min(θ=0) | design h=0.95 s holds until | h_min(θ=0.5 s) |
|---|---|---|---|
| noiseless k̃a=0.5 | 0.789 s | θ ≈ 0.23 s | 5.69 s |
| low end k̃a=0.4 | 0.945 s | θ ≈ 0.42 s | 1.19 s |
| nominal k̃a=0.524 | 0.751 s | θ ≈ 0.19 s | 8.35 s |

| Bernoulli loss | max L2 amp (mean / worst seed) |
|---|---|
| 0 % | 1.007 / 1.008 |
| 10 % | 1.008 / 1.009 |
| 30 % | 1.010 / 1.010 |
| 50 % | 1.013 / 1.015 |

## Headline conclusion
**Communication delay, not packet loss, is the binding constraint** at the
paper's design point: the nominal delay budget is only ~0.2 s (DSRC/C-V2X
latencies are 20–100 ms, so real systems sit uncomfortably close), while
even 50 % loss (effective 5 Hz updates) only moves the max L2 amplification
from 1.007 to 1.013 — loss under ZOH acts mainly as *added effective delay*
of order the beacon period. Counter-intuitive footnote: a *larger*
feedforward gain is *more* delay-sensitive (the s²·e^{−θs} term), which is
why the low-end curve is the flattest in θ.

## Consequences
- These two tables are the project's original contribution for the report;
  regenerate with `python scripts/reproduce_base_paper.py`.
- The ROS channel node exposes the same knobs (`delay_s`, `loss_prob`)
  so any point of the study can be demonstrated live ([D-011]).
