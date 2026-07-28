# T-10 — QoS-adaptive & flagship formulas, fully explained

**Scope.** This is the direct sequel to Lessons 1–4 (`T-01`…`T-04`, and the
tutoring-session recap in `T-09`). Those four covered the *base* system —
the vehicle, the spacing error, string stability, and Ma-2025's closed-form
theorem, all at **fixed, known, noiseless-in-the-limit** settings. This doc
covers everything built **on top** of that base: how the project estimates
an unknown channel live (Lesson 5), compensates its delay (Lesson 6), copes
with the fact that real controllers can't re-tune their own gains on the fly
(Lesson 7), argues that adapting slowly is still safe (Lesson 8), and the
three flagship extensions (Lessons 9–11). Every formula below is read
directly from the source: `docs/theory/T-05…T-08` and
`docs/flagship/F-A…F-C`, cross-checked against the code paths each doc
names. Where a formula involves a design choice rather than a derivation
(e.g. the estimator's safety divisor), that's called out explicitly.

---

## Lesson 5 — Online channel estimation: how ρ̂ is actually measured

**The question.** Ma-2025's whole theorem assumes you already know `ρ`.
Nobody hands a real car that number. So: given only what a follower can
locally observe (radar and the messages it receives), how do you measure
"how good is my radio *right now*"?

**The key structural fact that makes this possible at all.** Recall the
noise model: `w ∈ [1−1/ρ, 1+1/ρ)`, and this support is **strictly bounded**
— no tails, no small chance of a huge outlier. That single fact is what
makes ρ *estimable* from a handful of samples with real confidence: if you
observe `|ŵ−1|` a few times and take, say, its 99th percentile, that
percentile can never exceed the true `1/ρ` — meaning an empirical
percentile of `|ŵ−1|` **always under-estimates `1/ρ`**, which means it
**always over-estimates `ρ`** (a smaller measured spread implies a
*better*-looking channel than reality). The direction of the bias is known
by construction — not guessed — which is strictly stronger than ordinary
moment-based estimation (which would need to know the unknown per-bit
probabilities `γ_j` first). Knowing the bias direction is exactly what lets
the project apply **one single safety divisor `κ`** and flip a
slightly-optimistic estimate into a conservative one on purpose.

**How a single sample is built.** A follower doesn't need anything exotic —
it already has two numbers it can compare:
$$\hat w = \frac{y}{a_{\text{ref}}}$$
where `y` is the received beacon value (`w·a(t−θ)`, the noisy, delayed
message) and `a_ref` is the **radar's own** measurement of the predecessor's
acceleration, looked up `θ̂` seconds into the past so the two measurements
are lined up in time (this is why accurate delay estimation, later in this
lesson, matters even before the noise estimator can trust its own samples).
Two guard conditions keep this division sane:
- `|a_ref| ≥ 0.03 m/s²` — dividing by a near-zero acceleration would blow
  the ratio up on pure measurement noise, not real channel noise;
- reject `ŵ ∉ (0.2, 1.8)` — physically impossible for any `ρ > 1.25`, so a
  value outside that band means the two samples were paired to the *wrong*
  timestamps, not that the channel is somehow that bad.

**The estimator's asymmetric dynamics — and why that asymmetry is deliberate.**
Measured directly from a zone run where the true `ρ` drops 10→3→10:
- **Detection is fast (seconds).** One sufficiently large `|ŵ−1|` sample
  immediately dominates a 99th-percentile statistic — the estimator can
  react to a channel getting *worse* almost the instant it happens.
- **Recovery is slow by design (~20 s).** A channel getting *better* can
  only be learned by the *absence* of large deviations over time, so old
  "bad" samples have to age out of the window before the estimate can climb
  back up. An earlier version used a fixed sample-count window instead of
  an age-based one, and it failed in testing — `ρ̂` got stuck at 3.1 even
  after the platoon had left the bad zone, because too few new samples
  arrived to flush the old ones out under weak excitation. Switching to
  age-based expiry fixed it. The asymmetry (fast-to-worry,
  slow-to-relax) is in the *safe* direction on purpose.
- **The excitation requirement.** If the leader isn't accelerating, there's
  no `u_ff` signal to compare against radar at all — no samples, so `ρ̂`
  simply holds its last value rather than updating. This is a completely
  standard fact from adaptive control (you cannot identify a system that
  isn't being excited), and the fix is equally standard: scenarios add a
  small, disclosed `±0.08 m/s²` dither to the leader's trajectory —
  imperceptible to a rider, but enough to keep the estimator fed
  continuously.

**Measured accuracy:** across an 8-follower zone run, median `ρ̂ ∈ {10.4,
3.1, 10.7}` against true `ρ ∈ {10, 3, 10}` — close, and on the conservative
side, exactly as the estimability argument predicts.

**Delay estimation `θ̂`, almost for free.** Because every message already
carries a timestamp, this part barely needs its own theory: `θ̂` is just the
median of (arrival time − stamp time) over a small recent window. Offline,
the receiver is simply given the true configured delay (equivalent to
perfect clock sync); the ROS backend measures it from real message stamps
on a shared host clock, so sync error there is just OS scheduling jitter
(milliseconds), not a real gap.

**Where it lives:** `network.V2VLink.noise_at` (the channel being
estimated), `estimation.ChannelEstimator` (the estimator itself).

---

## Lesson 6 — The delay problem, quantified, and the predictor that erases it

**How bad is uncompensated delay, really?** At the paper's own case-A design
(fixed gains, `ρ=5`), here's what happens to the *required* headway as pure
communication latency `θ` grows, with no compensation at all:

| θ | required headway |
|---|---|
| 0 | 0.945 s |
| 0.10 s | 1.133 s |
| 0.15 s | 1.888 s |
| 0.30 s | **7.181 s** |

For context, real DSRC/C-V2X links routinely see 20–100 ms of end-to-end
latency, more under congestion — and the design's nominal delay budget is
only about 0.1 s. That's thin, and the far-right column shows why it
matters: this isn't a small correction, it's the difference between a
usable platoon and a completely impractical one.

**A genuine research finding: noise and delay interact, not just add.** At
`θ=0.15 s`, the *noiseless* design is still comfortably stable
(`‖H̃‖∞=0.99999`; a noiseless system can tolerate delay up to ≈0.23 s), and
separately, the *undelayed* noisy design (`ρ=5`, `θ=0`) is stable by
construction. But run them **together** — delayed *and* noisy — and the
combination is unstable (`‖H̃‖∞=1.00923` at a specific frequency
`ω*=0.45 rad/s`). Neither impairment alone would have predicted this; it
only shows up when you evaluate them jointly, which is exactly what the
project's exact frequency-domain analyzer (Lesson 3) is built to do.

**The predictor: reconstructing "now" from "then," using only what you already have.**
A receiver only ever sees `a(t−θ)` — the stale value. It reconstructs an
estimate of the *current* value using nothing but its own past received
samples and the message timestamps:
$$\hat a(t) = y(t) + \hat\theta \cdot \text{slope}, \qquad
\text{slope} = \frac{\text{mean}(y \text{ over } [t-T/2,\,t]) - \text{mean}(y \text{ over } [t-T,\,t-T/2])}{T/2}$$
with `T = 0.4 s`. In plain terms: take the average of recent messages, take
the average of the *slightly-less*-recent messages before that, find the
slope between those two averages, and extrapolate forward by `θ̂` along that
slope. This is a **lead compensator**, in the same spirit as a Smith
predictor compensating a known dead time — except built entirely from
timestamps the receiver already has, not from a model of the plant.

**Why block-*averaging* and not just a two-sample slope.** If you tried to
estimate the slope from just two raw noisy samples, the noise itself would
dominate: a multiplicative-noise signal's sample-to-sample jump has a
"noise slope" of roughly `(Δw)·|a|/Δt`, which is typically *much larger*
than the real signal's own slope. Averaging a block of `N` samples before
differencing suppresses that noise contribution by roughly `1/√N` — this is
the entire reason the formula above averages over half-windows instead of
just subtracting two points.

**The exact frequency-domain operator** — the same expression is what the
theory evaluates *and* what the simulation actually executes, so there's no
gap between the two:
$$P(s) = 1 + \hat\theta \cdot A(s) \cdot \frac{1-e^{-sT/2}}{T/2}, \qquad
A(s) = \frac{1-e^{-sT/2}}{s\cdot T/2}$$
Two limits worth understanding by feel:
- **At low frequency,** `P(s) → 1 + θ̂s` — this is precisely the *ideal*
  first-order lead compensator, and it cancels the delay term `e^{−θs}` to
  first order **exactly in the frequency range where the string-stability
  conditions actually bind** (Lesson 3's low-frequency 28b condition). This
  is not a coincidence — it's why the predictor works as well as it does.
- **At high frequency,** the block-average term `A(s)` rolls the lead
  compensation off, which is precisely what keeps the predictor from
  amplifying noise without bound at frequencies where there's no real
  trend to extrapolate anyway.

**The payoff, stated plainly:** with the predictor active and matched
(`θ̂=θ`), the required headway stays **flat at 0.945 s for delays up to at
least 0.3 s** — compare that to 7.181 s uncompensated at the same delay.
Delay is, for practical purposes, removed from the design problem. It isn't
entirely free: there's a small (~1% per-hop) broadband noise cost from the
slope estimate itself, and if the delay *estimate* `θ̂` is wrong by some
amount `δθ`, that mismatch behaves like an ordinary uncompensated delay of
size `|δθ|` — but because the operating point sits on the *flat* part of the
h-vs-θ curve, the project's budget for `θ̂` estimation error is roughly
0.1 s before it starts to matter, which is generous compared to the
millisecond-accurate delay estimates timestamps actually provide.

**Where it lives:** `network.V2VLink.receive_predicted`, and the
`pred_theta_hat` branch inside `analysis.gamma`.

---

## Lesson 7 — The fixed-gain wall: why "just widen the gap" eventually stops working

**Two different questions that sound the same but aren't.** Ma-2025's
closed form `h_lb(ka, ρ)` (Lesson 4) answers: *"if I'm allowed to pick fresh
`(kp, kv)` for this exact `ρ`, what headway suffices?"* — a **design-time**
question. A real, already-built car asks something different: *"given the
gains I'm actually running right now, what headway do I need at this
`ρ`?"* — an **operational** question, and it's answered not by a formula but
by direct bisection against the exact `‖H̃‖∞` test from Lesson 3, checked at
both ends of the noise interval.

**How far apart these two answers really are** (case-A gains, `τ0=0.5`):

| ρ | h_req (fixed gains, operational) | h_lb (re-tuned, design-time) |
|---|---|---|
| 10 | 0.867 | 0.789 |
| 5 | **0.945** *(= the actual case-A design!)* | 0.9375 |
| 3 | 1.154 | 1.200 |
| 2.5 | 2.157 | 1.372 |
| 2 | **5.277** | 1.714 |
| 1.8 | ~8.3 (right at the edge of feasibility) | 1.87 |

At `ρ=5` the two numbers nearly coincide — and that's not a coincidence,
it's a confirmation: case-A's gains were designed *for* `ρ=5`, so at that
one point "fixed" and "re-tuned" describe the same design. Move away from
that design point, though, and they split hard — by `ρ=2` the fixed-gain
platoon needs a following gap over three times longer than the re-tuned
formula would suggest, and it keeps getting catastrophically worse from
there.

**Anatomy of *why* it blows up.** Recall the high-frequency feasibility
ceiling from Lesson 4: `γ = kv + h·kp` must satisfy
`γ ≤ (1−(1+1/ρ)²ka²)/(2τ0)` — and notice that ceiling **does not contain
`h` at all**. With `kv` fixed at 0.63, as `ρ` drops the right-hand side
shrinks (0.6975 at `ρ=10` down to 0.4375 at `ρ=2`) — and since `h` only ever
*adds* to `γ` (through the `h·kp` term), increasing `h` pushes you the
**wrong direction** relative to a ceiling that's already falling. Large `h`
can still eventually restabilize things (the sufficient condition isn't
necessary — a higher-order term rescues it, Lesson 3), but only at
absurdly large headways. In practice there are two walls, not one: an
*economic* wall around `ρ≈2.5` (headway costs become impractical) that
arrives before the *hard feasibility* wall `ρ*≈1.8` (beyond which no
headway up to a 10-second search cap works at all).

**The fix: let the gains move too, not just the headway.** The gain
scheduler re-tunes `kv` directly as a function of `ρ`:
$$k_v(\rho) = 0.9 \times \text{ceiling}(\rho), \qquad k_p \text{ scaled to hold } k_p/k_v \text{ fixed}$$
i.e. deliberately stay at 90% of the feasibility ceiling rather than pushing
right up against it. The payoff:

| ρ | kv(ρ) | h_req at re-tuned gains |
|---|---|---|
| 10 | 0.628 (≈ case-A's 0.63) | 0.870 |
| 5 | 0.576 | 1.033 |
| 3 | 0.500 | 1.319 |
| 2 | 0.394 | **1.877** *(vs 5.28 with fixed gains)* |

The rule naturally reproduces case-A at its own design point and keeps
every `ρ ≥ 1.5` comfortably inside a practical `h_max = 2.5 s` ceiling. A
deep-zone validation run makes the difference concrete: headway-only
adaptation saturates at `h_max` and is still measurably unstable
(`‖H̃‖∞=1.019`); adapting gains **and** headway together lands at
`‖H̃‖∞=0.99999`.

**One practical caveat worth remembering:** the estimator's safety divisor
`κ` (Lesson 5) lands `ρ̂` on exactly the *steep* part of this curve below
`ρ≈3`, where estimation uncertainty is expensive — a zone run measured
`h=1.76 s` against an "omniscient" (perfectly-known-`ρ`) `h=1.23 s` at the
same true `ρ=3`. Improving the estimator's confidence directly buys back
following-gap headway in exactly this regime.

**Where it lives:** `HeadwayAdapter._build_table` (the bisection),
`GainScheduler` (the fix).

---

## Lesson 8 — Why adapting slowly is still provably safe (the quasi-static argument)

**The claim, stated precisely.** An adaptive platoon — one whose `h_i(t)`,
and optionally `(kp_i, kv_i)(t)`, are changing live — remains L2 string
stable in practice, **because it only ever moves slowly through a continuum
of individually-certified frozen designs.** This is an engineering argument
backed by continuous numerical verification, not (yet) a closed-form
theorem — the project is explicit about that distinction, and about what a
future formal proof would need (a dwell-time argument for switched linear
systems, noted at the end).

**The argument, in five layers:**

1. **Frozen certification.** Every target the adapters ever aim for already
   satisfies `‖H̃(·;kp,kv,h)‖∞ ≤ 1` with margin — the `h` target is literally
   computed as `margin + h_req(ρ̂/κ)`, i.e. it's *built* from the same exact
   bisection test as Lesson 7. There is no admissible target that lives
   outside the certified set to begin with.
2. **Per-hop decoupling.** Because the platoon uses one-vehicle look-ahead,
   follower `i`'s transfer function depends *only* on follower `i`'s own
   parameters — not on any other follower's. This is what makes
   heterogeneous, per-vehicle adaptation admissible at all: you don't need
   a joint argument across the whole platoon, each hop's own
   `‖H̃_i‖∞ ≤ 1` is independently sufficient for the whole cascade.
3. **Slow variation, quantified against the loop's own speed.** The rate
   limits are `|ḣ| ≤ 0.05 s/s` and a relative gain slew `≤ 0.05/s`. Compare
   that to the loop's own fastest dynamics: `1/τ = 2 s⁻¹` (Lesson 1), with
   the dominant error dynamics living below `0.5 rad/s`. Parameter drift at
   roughly `0.03` rad-equivalent per second is one to two full orders of
   magnitude slower than the dynamics it's embedded in — textbook
   frozen-time / slowly-varying-systems territory. The extra `−ḣ·v` term
   this introduces into `ė` (Lesson 2) works out to at most about
   `1 m/s²`-equivalent at highway speed, which the margin in step 1 already
   absorbs.
4. **Consistency between what's moving.** Gains and headway both slew
   toward the *same* target design (the headway adapter consumes the gain
   scheduler's own `h_req`), so an in-between state is an interpolation
   between two certified points, never an unrelated wandering combination.
5. **Continuous spot-checking, not blind trust.** The in-force `‖H̃‖∞` is
   actually evaluated at probe times during every adaptive run and in every
   certification pass — measured values stay `≤0.99999` throughout, except
   in the *deliberate* headway-only "wall" demonstration built specifically
   to show what failing this argument looks like (Lesson 7).

**What this argument does *not* cover — stated honestly, not hidden.**
- **Transition transients.** A commanded step `Δh` at speed `v` produces a
  physical gap-adjustment wave of size `≈v·Δh` (measured up to ±14 m at the
  last follower during a `ρ=3` zone run). These are *commanded*, bounded
  motions, not instability — but they sit outside the L2-gain statement
  entirely. Minimum-gap safety during a transition is checked directly on
  the simulation (a `≥5 m` criterion in the certification suite), not
  proven analytically.
- **Simultaneity.** If every follower adapts at the same moment, their
  individual transition waves compound down the string (observed L2 ratios
  up to ≈1.11 in windows that contain a transition). *Staggering* when
  followers adapt is the known mitigation, currently on the roadmap rather
  than implemented.
- **Estimator-in-the-loop jitter.** Noise in `ρ̂` maps through the *steep*
  part of the `h_req` curve (Lesson 7) into visible jitter in `h(t)` itself
  (±0.05 s wiggle observed on a zone plateau). It's bounded by the rate
  limit, and smoothing `ρ̂` is flagged as a straightforward future
  improvement.

**Where it lives:** `HeadwayAdapter.update`, `GainScheduler.update` (the
rate limits themselves).

---

## Lesson 9 — Flagship A: predictive QoS-map spacing

**The gap this closes.** The Lesson 5–8 pipeline is entirely **reactive** —
`ρ̂` only starts dropping *after* the follower has already collected noisy
samples *inside* a bad patch, so `h(t)` only starts opening *after* entry.
At the exact moment of crossing into a degraded zone (say `ρ=3`), the
platoon is still sitting at the good-channel headway `h=0.95 s`, which
computes to `‖H̃‖∞=1.002` — measurably string-**un**stable, for a real
stretch of time, every single time a platoon crosses into a bad patch. Real
radio dead-zones, though (a tunnel, an underpass, a known interference
source), are — unlike genuine surprises — usually knowable *in advance* and
routinely pooled into shared maps in real deployments.

**The spatial channel map.** `QoSMap(zones, rho_base)` models channel
quality as a function of **position**, `ρ(x)`, built from a list of
interference patches `[(x0, x1, ρ), …]` in metres (overlapping patches: the
worse one wins). Two functions matter:
- `rho_at(x)` — the channel quality at a specific position.
- `min_rho_ahead(x, v, horizon)` — the **worst** channel the vehicle will
  encounter within the next `v·horizon` metres at its current speed — the
  preview itself.

**Two separate uses of the same map.** First, as **ground truth**: each
link's noise is now driven by *where that specific link physically is* —
follower `i` enters a patch strictly later than follower `i−1`, exactly as
it would on a real road, replacing the earlier (D-016) approach of one
shared time schedule for the whole platoon. Second, as a **preview** fed
into the headway adapter:
$$h_{\text{target}} = \max\big(h_{\text{required}}(\hat\rho_{\text{safe}}),\ h_{\text{required}}(\rho_{\text{preview}})\big)$$
— the adapter opens the gap for whichever is *worse*, the live (reactive)
estimate or the (anticipatory) preview. Because the map value is known
exactly (no extra safety divisor needed on it — it isn't an estimate)
and is available from `t=0`, before any online estimator has even warmed
up, the preview can drive `h(t)` open **even while `ρ̂` is still `None`**.
Fusing the two — instead of trusting the map alone — is what keeps the
design robust to a map that's stale or wrong somewhere: the reactive
estimator (Lessons 5–8) remains the safety backstop for anything the map
missed.

**Why the string-stability guarantee still carries over unchanged.** Part A
changes *when* `h` moves (earlier, while still in a good channel), not the
underlying guarantee: `h` is still slew-rate-limited exactly as in Lesson 8,
so the same frozen-design, quasi-static argument applies without
modification. And because each hop's transfer function still only depends
on that hop's own `h` (the per-hop decoupling fact from Lesson 8), the
per-vehicle heterogeneous headways this preview scheme produces are just as
admissible as before.

**Results (last follower, identical channel, reactive vs. predictive):**

| metric | reactive | predictive |
|---|---|---|
| headway at zone entry | 0.95 s | **1.23 s** (pre-opened) |
| `‖H̃‖∞` in force at entry | **1.002** (unstable) | **1.000** |
| fraction of in-zone time unstable | **6.9%** | **0%** |
| peak spacing error in zone | 17.2 m | **14.2 m** (−17%) |
| mean headway (capacity cost) | 1.395 s | 1.441 s (+3%) |

**Honest limitations.** The gap-opening transient doesn't disappear — it's
*relocated* into the good channel before entry, where it's benign (~5 m,
ample margin) instead of happening inside the degraded patch. That
relocation costs roughly 3% more average headway, a small throughput price
for holding the certified margin through the entire degraded region.
Prediction also can't beat causality if the map itself is simply wrong — an
*unmapped* patch is still only ever discovered reactively, which is exactly
why the fused "worse of the two" rule, not a map-only rule, is what's
actually implemented.

**Where it lives:** `src/cacc/qos_map.py` (`QoSMap`); `AdaptConfig.preview_s`
and the fused target inside `HeadwayAdapter.update` in `estimation.py`;
per-link position-driven scheduling in `platoon.py`.

---

## Lesson 10 — Flagship B: the physics-consistency trust gate

**The gap this closes.** The CTHP law applies `ka·u_ff` completely
unconditionally — `u_ff` arrives over a radio, which is an **external
input** a compromised or impersonating node can put *any* value on.
String stability (Lesson 3) says nothing about this: it bounds the
response to honest disturbances, not to a lying one. Cryptographic
authentication doesn't fully close this either — it stops impersonation,
but not a node that's authenticated *and* compromised, and it adds no
physics-level sanity check at all. Left undefended, a single phantom
+5 m/s² spoof on one link, in simulation, drives the attacked follower's
gap to **−27 m** — a rear-end collision.

**The independent second opinion.** Every follower already has a second,
completely separate way to estimate its predecessor's acceleration: its own
radar (range and range-rate, differentiated against the follower's own
IMU). An attacker sitting on the V2V network simply cannot touch this
sensor — it's physically local to the follower. That independence is the
entire mechanism.

**The trust weight and the fusion rule.** With `a_v2v` the radioed value and
`a_radar` the independent radar estimate:
$$r = |a_{\text{v2v}} - a_{\text{radar}}|, \qquad g(r) = \frac{1}{1+(r/r_0)^2} \in (0,1]$$
$$u_{\text{ff,eff}} = g\cdot a_{\text{v2v}} + (1-g)\cdot a_{\text{radar}}$$
Read it by feel: when the two sources roughly agree (`r` small), `g→1` and
the radio value passes through essentially unchanged. When they strongly
disagree (`r` large), `g→0` and the controller quietly falls back to the
radar-only estimate instead.

**The provable bound — worked in full, because it's a genuinely elegant piece of algebra.**
The deviation this fusion can ever introduce, relative to the honest radar
value, is:
$$|u_{\text{ff,eff}} - a_{\text{radar}}| = g(r)\cdot r = \frac{r}{1+(r/r_0)^2}$$
**Claim:** this quantity never exceeds `r0/2`, for *any* `r ≥ 0`, with
equality at exactly `r = r0`.

*Proof.* Write `f(r) = r/(1+(r/r0)²) = r·r0²/(r0²+r²)`. Differentiating,
$$f'(r) = \frac{r_0^2(r_0^2 - r^2)}{(r_0^2+r^2)^2}$$
which is zero only at `r=r0`, positive for `r<r0`, and negative for `r>r0`
— so `f` strictly increases up to `r=r0` and strictly decreases after it,
meaning `r=r0` is a genuine maximum, not just a critical point. Evaluating
there: `f(r0) = r0·r0²/(2r0²) = r0/2`. ∎

**Why this bound is remarkable:** it's completely **independent of the
spoof's size**. Whether the lie is 5 m/s² or 500 m/s², the spurious command
the CTHP law can ever actually inject is capped at `ka·r0/2` — for case-A
gains (`ka=0.5`) and `r0=3`, that's `0.75 m/s²`, against a raw, ungated
`ka·|spoof|` that grows without bound. The bound is stated relative to the
*radar* estimate and holds unconditionally; relative to physical *ground
truth* it's `r0/2 + |radar error|`, so a reasonably accurate radar keeps the
provably-injectable command genuinely small in absolute terms too.

**Staying invisible when nothing is wrong.** Under the validated noise
model, an *honest* disagreement is only `r = |w−1|·|a_prev| + \text{radar noise}`
— a few hundredths of a m/s² at `ρ≥10` — so `g≈1` essentially always, and
the gate doesn't perturb ordinary operation at all: gate-on vs. gate-off
spacing differs by less than `1e-3 m` across a full run, with the trust
weight staying above 0.99 throughout. The one design requirement this
imposes is that `r0` must be chosen larger than the worst plausible
*honest* disagreement — including a genuine hard emergency brake seen
through channel noise (`|w−1|·8 ≈ 0.8 m/s²` at `ρ=10`) — so a few m/s² keeps
the gate transparent to real traffic while still clamping any spoof far
below a dangerous command.

**Results (attacked follower, +5 m/s² bias spoof, `ρ=10`):**

| | ungated (raw CTHP) | gated |
|---|---|---|
| min inter-vehicle gap | **−27.4 m** (collision) | **+8.4 m** (safe) |
| peak spacing error | 50.5 m | **14.2 m** (−72%) |
| max injected command | `ka·spoof` ≈ 2.5 m/s² | **0.75 = ka·r0/2** (exactly the proven bound) |
| trust weight during spoof | — | collapses to 0.25, recovers to 1.0 |

**Honest limitation.** The gate bounds the *instantaneous* injection, not
its running total over time. Because the spacing loop's DC stiffness is low
(`kp=0.009` is deliberately gentle), a spoof that is both large **and**
sustained for tens of seconds can still let the bounded 0.75 m/s² slowly
accumulate — the gate converts an *unbounded, instant* catastrophe into a
*bounded, slow, detectable* drift, buying time for a supervisory response
rather than guaranteeing zero drift under an unbounded-duration attack. A
persistence detector (CUSUM-style) is explicit future work, not a silently
assumed gap.

**Where it lives:** `src/cacc/trust.py` (`TrustGate`); the spoof itself is
injected via `LinkAttack` in `network.py`.

---

## Lesson 11 — Flagship C: the chance-constrained risk certificate

**The gap this closes.** The base guarantee `‖H̃‖∞ ≤ 1` (Lesson 3) is
enforced at the channel's absolute **worst case** — the effective
feedforward gain `ka_eff = w·ka` taken at the extreme edge of `w`'s
possible range. But `w` is a random variable with a fully, exactly known
distribution (it's built from 16 independent bit probabilities `γ_j`), and
that worst case corresponds to an event of probability roughly
`∏_j(1-γ_j) ≈ 1×10⁻⁶` — enforcing against it says nothing about how much
margin a design actually carries on an ordinary day.

**Computing the exact distribution — not sampling it, computing it exactly.**
`ChannelLaw.build(rho)` constructs the complete probability law of `w` by
convolving the 16 independent two-outcome bit distributions together:
starting from the single value `{0}` with probability `{1}`, for each of
the 16 bits `j`, every existing possible value splits into two (itself, and
itself plus `2^{-j}`), weighted by `(1-γ_j)` and `γ_j` respectively. After
all 16 bits, you're left with exactly `2^{16} = 65536` possible outcomes,
each with an exact, known probability — and `w` itself is just
`(1-1/ρ) + \text{vals}/ρ`. Because 65,536 discrete outcomes is trivial to
enumerate completely on a computer, **every quantile and moment computed
from this law is exact** — there's no Monte-Carlo sampling error hiding in
the tails, which matters enormously here because the tails are exactly what
the worst-case analysis cares about. (Sanity checks confirm the
probabilities sum to 1, the mean matches the paper's own closed form for
`E[w]`, and the variance matches `Σ_j 2^{-2j}γ_j(1-γ_j)/ρ²`.)

**A structural finding along the way: the instability region is two-tailed.**
Define `A(w) = ‖H̃(·; ka_eff = w·ka)‖∞` — the channel amplification as a
function of the noise realization. It turns out `A(w)` is **not monotone**
in `w`: at a degraded channel, the feedforward is too *weak* at the low end
of `w` (a deep fade, not enough cooperative signal to help), **and** it's
*over-amplified* at the high end of `w` (the multiplicative noise inflates
the `ka_eff·s²` term past what's stable). So string stability only holds
for `w` in a **middle band** — the unstable set is generally two-tailed,
`{w < w_lo} ∪ {w > w_hi}`. This single fact is the reason the paper's own
closed form (`cthp_h_lb`, Lesson 4) carries a `(1+1/ρ)` high-end correction
term at all — the project's exact numerical sweep independently confirms
*why* that term has to be there, agreeing with the paper's own algebra
rather than contesting it.

**The three risk-aware certificates, computed by direct summation over all
65,536 atoms** (no monotonicity shortcut is available, precisely because of
the two-tailed finding above):

- **Exceedance probability** — the risk map of any design:
  $$\text{exceedance}(h) = P(\|\tilde H\|_\infty > 1 \mid h, \rho)$$
- **Chance-constrained headway (the headline result)** — the smallest `h`
  such that `exceedance(h) ≤ ε`, found by bisecting on `h` against the exact
  two-tailed exceedance function. This is the **principled replacement**
  for the old ad-hoc "assume the channel is 1.15× worse than measured"
  safety divisor used elsewhere in the QoS layer (Lesson 7's `κ`) — instead
  of an arbitrary pessimism factor, you state an explicit failure-probability
  budget and get the headway that achieves it.
- **Mean-square headway** — the smallest `h` with `E[A(w)²] ≤ 1`, the
  classical stochastic-robustness notion, reported *with an explicit
  caveat* below.
- **Distribution-free (Chebyshev) headway** — a two-sided interval
  `t = σ/√ε` around the mean, designed to stabilize every `w` in
  `[E[w]-t, E[w]+t]`, using only the mean and variance (robust to any
  uncertainty in the individual `γ_j` values, at the cost of using less
  information).

**Results (case-A design, headline channel `ρ=3`):**

| headway | value | exceedance `P(‖H̃‖∞>1)` | note |
|---|---|---|---|
| naive (design at `E[w]`) | 0.726 s | **0.55** | badly unsafe |
| mean-square | 0.994 s | **0.076** | *too permissive* — see caveat |
| chance-constrained, `ε=1e-2` | **1.033 s** | 0.0095 | 1-in-100 budget |
| chance-constrained, `ε=1e-3` | 1.052 s | 0.0010 | 1-in-1000 budget |
| Chebyshev, `ε=1e-2` | 1.154 s | 0 | distribution-free (≈ worst case here) |
| worst-case (both tails) | 1.154 s | 0 | ≈ paper's `cthp_h_lb` = 1.20 |

**The real dividend:** accepting a 1-in-100 instantaneous-exceedance risk
budget instead of the pure worst case trims the required headway from
1.154 s to **1.033 s — a genuine 10.5% capacity gain** — because the pure
worst case is paying to guard against two separate `~10⁻⁶`-probability
tail events simultaneously. And the number that matters most for what's
actually deployed: the base design (`h=0.95 s` at `ρ=5`) has **exactly
zero** exceedance probability — string-stable across *every single one* of
the 65,536 possible channel states, not just "probably fine."

**Validated against the live, simulated channel, not just against itself:**
the analytic exceedance probability matches the empirical fraction of time
a real running simulation actually spends unstable, across four full orders
of magnitude of probability (0.55 vs 0.55, 0.076 vs 0.076, 0.0095 vs 0.0096,
0.0010 vs 0.0011, 0 vs 0).

**Honest limitations, stated directly.** Mean-square is *too permissive* —
`h_ms=0.994 s` still carries a 7.6% exceedance rate, riskier than even a 1%
budget, and it's kept in the project only as an instructive contrast, never
as something to actually deploy. Chebyshev is *honest but blunt* — using
only mean and variance (not the full shape of the distribution), it
collapses back to the plain worst case at `ρ=3`, buying no dividend at all
here. And there's a small, separately-tracked numeric subtlety: because
`‖H̃‖∞` is evaluated to within about `±5×10⁻³` of the true boundary
numerically, the computed worst-case headway and the paper's own closed
form can differ by up to ~0.05 s — both numbers are reported rather than
picking one silently.

**Where it lives:** `src/cacc/certificate.py` (`ChannelLaw.build`,
`exceedance_prob`, `chance_headway`, `meansquare_headway`,
`chebyshev_headway`, `worstcase_headway`, `certify_design`).

---

## How Lessons 5–11 relate to each other, in one paragraph

Lessons 5–8 are one continuous story: **estimate** the channel (5),
**compensate** its delay (6), discover that fixed real-world gains hit a
**wall** the paper's own re-tuned formula doesn't warn you about (7), and
**argue** — with explicit honesty about what isn't yet proven — that moving
slowly through that machinery stays safe (8). Lessons 9–11 are three
independent extensions bolted onto that same foundation, none of which the
base paper poses as a question at all: **anticipate** a bad zone before
entering it (9), **defend** against an input that might simply be lying
(10), and **quantify** exactly how safe "safe" really is instead of a
binary pass/fail (11). None of the three flagship lessons modify the base
physics or the base control law in any way — they all sit strictly around
it, reading its outputs or feeding its inputs, which is precisely why they
can be understood (and graded) independently of one another.

---

*Written 2026-07-15 as the direct sequel to Lessons 1–4 in a tutoring
session. Sources: `docs/theory/T-05…T-08`, `docs/flagship/F-A…F-C`, cross-
checked against `src/cacc/network.py`, `estimation.py`, `qos_map.py`,
`trust.py`, `certificate.py`. If this doc and a script or test ever
disagree, the code wins — see `docs/INDEX.md`'s provenance rule.*
