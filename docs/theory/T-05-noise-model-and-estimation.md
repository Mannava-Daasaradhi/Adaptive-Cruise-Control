# T-05 — The multiplicative channel and why ρ is estimable online

**Code:** `network.V2VLink.noise_at`, `estimation.ChannelEstimator` ·
**Decisions:** D-004 (noise in the channel), D-016 (estimator).

## Channel model (implementation details that matter)

- w is **piecewise constant** over 1/`noise_rate` intervals (10 ms
  offline; per-beacon ≈ 40 ms in ROS). Hold time changes the noise's
  low-frequency power: seed-to-seed per-hop-L2 envelopes measured
  [0.981, 1.007] at 10 ms vs [0.948, 1.028] at 40 ms (D-014 cross-check)
  — always state the hold time when comparing backends.
- The RNG stream is separate from packet-loss draws (`[seed, 0xCACC]`),
  so toggling noise never shifts loss realizations.
- **ρ-independent bit cache:** the cache stores U′(t) = Σ z_j 2^{−j};
  w = (1−1/ρ(t)) + U′(t)/ρ(t) is formed per query. Consequence: a
  `rho_schedule` rescales the *same* realization — zone studies are
  seed-comparable across schedules (pinned by
  `test_rho_schedule_same_bits_different_rho`).

## Estimability — the key structural fact

|w − 1| ≤ 1/ρ **strictly** (bounded support, no tails). Therefore any
empirical maximum/quantile of |ŵ − 1| *under-estimates* 1/ρ, i.e.
**over-estimates ρ** — the estimator's bias direction is known by
construction, and a single safety divisor κ flips it conservative. This
is much stronger than moment-based estimation (which would need the
unknown γ_j's).

## Receiver-side sample construction

ŵ = y / a_ref, where y = received beacon (w·a(t−θ)) and a_ref = radar
acceleration of the predecessor **looked up θ̂ into the past** (timestamp
alignment). Guards: |a_ref| ≥ a_min (0.03 m/s²) — division noise; reject
ŵ ∉ (0.2, 1.8) — physically impossible for ρ > 1.25, catches pairing
errors.

## Estimator dynamics (measured, zone ρ 10→3→10)

- **Detection is fast** (seconds): one large |ŵ−1| sample immediately
  dominates the 99th percentile.
- **Recovery is slow by design** (~`max_age_s` = 20 s): improvement can
  only be *learned by absence* of large deviations, so old samples must
  age out. The asymmetry is in the safe direction. A count-based window
  was tried first and failed (ρ̂ stuck at 3.1 after zone exit with weak
  excitation) — hence age-based expiry.
- **Excitation requirement:** no communicated acceleration ⇒ no samples ⇒
  ρ̂ holds. Scenarios superpose a ±0.08 m/s² incommensurate-tone dither
  on the leader (persistent excitation; standard adaptive control).

Accuracy achieved: median ρ̂ ∈ {10.4, 3.1, 10.7} against true {10, 3, 10}
(8-follower zone run, all links).

## Delay estimation θ̂

Timestamps make θ̂ trivial: median of (arrival − stamp) over a small
window. Offline, the receiver is granted the true configured delay
(equivalent up to clock sync); the ROS backend measures it from real
stamps (Workstream A) — same host clock, so sync error ≈ scheduling
jitter (ms).
