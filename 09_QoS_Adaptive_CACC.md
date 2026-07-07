# QoS-Aware CACC — Joint Estimation, Prediction and Headway Adaptation

The project's novel extension (D-016). Title framing for the paper:
**"QoS-Aware CACC: Online Channel Estimation, Timestamp-Based Feedforward
Prediction, and String-Stability-Preserving Headway Adaptation over
Imperfect V2V".** Everything here is implemented in `src/cacc`
(`estimation.py`, extensions in `network.py` / `analysis.py` /
`platoon.py`), exercised by `scripts/qos_adaptive_study.py`, and pinned by
`tests/test_qos_adaptive.py` (52-test suite green).

## 1. The gap in the base paper

Ma 2025 answers: *given* a channel noise level ρ, what is the minimum
string-stable time headway h_lb(ka, ρ)? Three assumptions block deployment:

1. **ρ is assumed known and constant.** Real V2V quality varies with
   distance, congestion and interference — nobody hands the controller ρ.
2. **Gains are re-tuned per ρ.** The closed form h_lb presumes (kp, kv)
   chosen inside the eq.-29 feasible region *for that ρ*; an operating
   platoon has fixed gains.
3. **Zero feedforward latency.** Our D-008 study showed the design only
   tolerates ~0.1–0.2 s of V2V delay — uncomfortably close to real
   DSRC/C-V2X latencies.

We close all three: **estimate** ρ online per link, **adapt** h(t) against
the *fixed-gain* requirement, and **predict** the delayed feedforward from
its timestamps.

## 2. Architecture (all receiver-side quantities only)

```
 beacons (w·a, delayed θ) ──┬──────────────► predictor ──► u_ff  ──► CTHP
                            │                  ▲  θ̂ from stamps      law
 radar (pred. accel a) ──►  ratio = ŵ          │
                            │                  │
                    ChannelEstimator ──► ρ̂ ──► HeadwayAdapter ──► h(t)
                    (|ŵ−1| ≤ 1/ρ strictly)     (rate-limited slew to
                                                margin + h_req(ρ̂/κ, θ̂))
```

* **Channel estimator** (`ChannelEstimator`): the beacon delivers
  y = w·a(t−θ); the radar measures a and, looked up θ̂ into the past,
  provides the matching clean reference → each pair yields a sample of w.
  Because the n-bit channel has *strictly bounded* support
  (|w−1| ≤ 1/ρ), the windowed 99th percentile of |ŵ−1| **under-estimates**
  1/ρ, i.e. over-estimates ρ; dividing by κ = `rho_safety` makes the
  design conservative instead. Samples expire after `max_age_s` (20 s) —
  a count-based window recovers far too slowly after a zone when the
  excitation is weak. The channel is unobservable without excitation, so
  scenarios add a ±0.08 m/s² multi-sine **probing dither** on the leader
  (persistent excitation, standard adaptive-control practice).
* **Headway adapter** (`HeadwayAdapter`): slews h(t) (|ḣ| ≤ 0.05 s/s)
  toward `margin + h_req(ρ̂/κ, θ̂)`, where h_req is a precomputed
  `min_stable_headway` bisection table over (ρ, θ), worst-case over both
  ends of the noise interval, for the predictor setting in use. Each hop's
  transfer function depends only on its own follower's h, so per-vehicle
  heterogeneous headways are admissible; the rate limit keeps the design
  quasi-static between the frozen string-stable configurations.
* **Feedforward predictor** (`V2VLink.receive_predicted`): the receiver
  adds θ̂ × (slope from two 0.2 s window-averaged blocks of its own
  received samples) — exactly the lead operator
  P(s) = 1 + θ̂·A(s)(1−e^{−sT/2})/(T/2), A = moving average, which
  `analysis.gamma(kind='cthp', pred_theta_hat=…)` evaluates on the
  imaginary axis, so theory and simulation use the same operator. θ̂ comes
  from message timestamps (exact up to clock sync; measured directly in
  the ROS backend).

## 3. Theory results (fig1, `results/qos_adaptive/<stamp>/metrics.json`)

**Fixed-gain requirement vs the re-tuned bound** (ka=0.5, kp=0.009,
kv=0.63, τ0=0.5 s):

| ρ | h_req fixed gains | h_lb re-tuned (eq. 17) |
|---|---|---|
| 10 | 0.867 s | 0.789 s |
| 5 | **0.945 s** | 0.9375 s |
| 3 | 1.154 s | 1.200 s |
| 2.5 | 2.157 s | 1.372 s |
| 2 | 5.277 s | 1.714 s |
| ≤ ~1.8 | none with h ≤ 10 s | 1.87 s (ρ=1.8) |

Findings: (i) the fixed-gain requirement at ρ=5 lands exactly on the
paper's case-A design h = 0.95 s — strong validation; (ii) below ρ ≈ 3 the
fixed-gain requirement diverges violently from the re-tuned bound —
h-only adaptation has a practical **feasibility boundary ρ\* ≈ 1.8** (and
an *economic* one near ρ ≈ 2.5); re-tuning gains online is the natural
future extension. (iii) The paper's eq.-28a is sufficient-only: direct
‖H̃‖∞ evaluation shows large h can stabilize channels the closed-form
algebra rejects.

**Predictor erases the delay penalty** (ρ = 5, fixed gains, worst case
over the noise interval):

| θ | h_req uncompensated | h_req with predictor |
|---|---|---|
| 0 | 0.945 s | 0.945 s |
| 0.10 s | 1.133 s | 0.945 s |
| 0.15 s | 1.888 s | 0.945 s |
| 0.30 s | 7.181 s | 0.945 s |

The timestamp predictor holds the requirement **flat at the zero-delay
value for delays up to at least 0.3 s** — it effectively removes V2V
latency from the design problem (at the price of mildly amplified
channel noise through the slope term; the averaging baseline T = 0.4 s
bounds that).

## 4. Interference-zone experiment (fig2)

8 followers, case-A gains, ρ: 10 → 3 (t ∈ [80, 175]) → 10, maneuvers in
and out of the zone, probing dither on. Three platoons:

| variant | good-1 ‖H̃‖∞ | zone ‖H̃‖∞ | zone h | mean h | min gap |
|---|---|---|---|---|---|
| fixed-good (h=0.95) | 0.99999 | **1.00224** (unstable) | 0.95 | 0.95 | 20.0 m |
| fixed-worst (h=1.23) | 0.99997 | 0.99999 | 1.234 | 1.234 | 25.7 m |
| **adaptive** | 0.99999 | **0.99995** (margin restored) | 1.76 | 1.30 | 20.0 m |

* **Estimator tracking:** median ρ̂ = 10.4 / 3.1 / 10.7 against true
  10 / 3 / 10 across the three phases; zone entry is detected within
  seconds (new large deviations dominate the quantile immediately); exit
  recovery takes ~max_age = 20 s — asymmetric in the *safe* direction.
* **The capacity story is per-phase:** in good-channel phases the adaptive
  platoon runs h ≈ 0.95 s where the worst-case design permanently pays
  h = 1.23 s → ~23 % shorter headway ≈ +25 % lane capacity at 20 m/s,
  *while* the fixed-good design silently loses its stability margin inside
  the zone (‖H̃‖∞ > 1). In the zone the adapter pays h ≈ 1.76 s — more
  than the omniscient worst-case design because ρ̂/κ = 2.7 sits on the
  steep part of the fixed-gain curve: that gap is the measurable **price
  of estimation uncertainty**.
* **Interpretation caveat (important for the paper):** per-hop L2 ratios
  in windows containing policy transitions (h ramps) include the commanded
  gap-opening waves (±14 m at the last follower) and ring on the ~70 s
  slow mode — the decisive stability verdict is the frequency-domain
  ‖H̃‖∞ at the h in force (tabulated above; cf. D-010: the boundary is
  flat, so near-boundary time-domain growth is inherently slow).

## 5. Delay experiment (fig3) — the noise × delay interaction

θ = 0.15 s, h = 0.95 s, leader excites the uncompensated design's worst
frequency ω* = 0.45 rad/s; 2 × 2 cells (noise on/off × predictor on/off):

| cell | ‖H̃‖∞ (worst) | max per-hop L2 |
|---|---|---|
| noiseless, uncompensated | 0.99999 | 0.989 |
| noiseless, predictor | 0.99999 | 0.983 |
| **noisy (ρ=5), uncompensated** | **1.00923** | 1.017 |
| noisy (ρ=5), predictor | **1.00000** | 1.013 |

The refined finding: at θ = 0.15 s **delay alone is survivable** (noiseless
budget is ~0.23 s, D-008) and **noise alone is survivable** (case A is
designed for ρ = 5 at θ = 0) — it is their *interaction* (the delayed,
noise-inflated high end k̃a = 0.6 of the feedforward) that destroys string
stability. The timestamp predictor breaks exactly that interaction and
restores ‖H̃‖∞ = 1.0000 with the noise still present. (Noisy per-hop L2
values carry the realization envelope quantified in `07_…_Results.md` §5;
the frequency-domain column is the verdict.)

## 6. Reproduce

    conda activate cacc
    python scripts/qos_adaptive_study.py            # ~3 min, figs 1-3 + metrics
    python -m pytest tests -q                       # 52 tests

## 7. Gain re-tuning — the wall, broken (D-017, fig4)

Built and validated after the first version of this document: the
`GainScheduler` re-tunes kv to 90 % of the h-independent eq.-28a ceiling
(kp scaled with it) — a rule that *reproduces the paper's own case-A
choice at ρ = 10* (kv 0.628 vs 0.63) — and the adapter targets the
headway required at those gains. Deep-zone result (ρ 10 → 2 → 10):

| arm | zone kv | zone h | zone ‖H̃‖∞ in force |
|---|---|---|---|
| h-only | 0.63 (stuck) | 2.50 (saturated) | **1.019 — the wall** |
| joint gains + h | 0.353 | 2.33 | **0.99999 — stable** |

The pipeline now covers the noise model's entire ρ range. Details:
`docs/decisions/D-017`, protocol `docs/experiments/E-05`, theory
`docs/theory/T-07`.

## 8. Certification (D-018)

`python scripts/certify.py --seeds 5` runs the four-suite certification
matrix (base case A, QoS zone, deep-zone gains, delay predictor) and
emits a self-contained HTML PASS/FAIL report. First full run:
**CERTIFIED, 11/11 checks** (`results/certification/20260707-144611`).
Protocol: `docs/experiments/E-06`.

## 9. Remaining limitations / future work

1. **Staggered transitions** — simultaneous per-vehicle h ramps compound
   into a platoon-length wave; serializing them would smooth it.
2. **ρ̂ smoothing** — h(t) inherits estimator jitter on the steep part of
   h_req; a slew-limited or filtered ρ̂ would clean the zone plateau.
3. **Formal dwell-time proof** of the quasi-static argument
   (`docs/theory/T-08` sketches the route) — the journal-version upgrade.
4. **ROS 2 integration** — the offline pipeline is validated; the
   distributed integration is specified in `10_Handoff_Plan.md` §3.
Full register: `docs/validation/V-04`.
