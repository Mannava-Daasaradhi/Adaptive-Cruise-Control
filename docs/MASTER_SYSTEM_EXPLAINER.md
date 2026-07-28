# The Master Explainer — What We Built, Why, and How It All Talks to Itself

**What this doc is.** Your own plain-English walkthrough of the entire
project: the paper we started from, the **25 subsystems** we built on top
of it (grouped into 8 clusters), how those subsystems pass signals to each
other, exactly where we diverge from the base paper (with numbers, and
nothing that contradicts it), and the full literature backbone this
project stands on — the OATD-sourced classic foundation plus a fresh
sweep, both restricted to **published IEEE Transactions papers with a
DOI — no arXiv preprints presented as citations**. Nothing here is marketing — every claim about *our*
work links to the test, script, or `metrics.json` that proves it; every
claim about an *external* paper is stated only at the level of detail
confirmed from that paper's own abstract/publisher record, never invented.

**Reading map.** New to the project? Read Part 1 → Part 2 → Part 3 in
order — that's the whole story. Already know the base paper? Skip to
Part 2. Writing the paper's related-work section? Go straight to Part 6.
Want the "how much is actually here" gut-check? Part 2's summary table.

---

## Part 0 — The one-paragraph pitch

Take a published, peer-reviewed control law for self-driving convoys
(cars that follow each other using both a radar *and* a radio link).
Rebuild it from scratch until it reproduces the original paper's numbers
to four decimal places. Then show, with evidence, that the paper's own
assumptions don't survive contact with a real road: it assumes the
radio's noise level is known, its gains are re-tuned for that exact noise
level, and the radio has zero delay. None of that is true in practice. We
close all three gaps with a live estimation-and-adaptation loop, then go
three steps further than the base paper poses as questions at all: the
convoy *anticipates* bad radio zones before entering them, *defends*
itself against a radio that's lying to it, and *quantifies exactly how
risky* any given setting is instead of giving a pass/fail answer. The
result is **25 separate, individually-tested engineering subsystems**
(not "a simulation of the paper" — a stack), 87 automated tests, two
independent simulation engines (a monolithic Python one and a genuinely
distributed ROS 2 one) that agree with each other to within a few
percent, and every number traces to a reproducible script.

---

## Part 1 — The base paper, in full depth

### 1.1 What it is, exactly

> **G. Ma, P. R. Pagilla, and S. Darbha**, "Selection of Time Headway in
> Connected and Autonomous Vehicle Platoons Under Noisy V2V
> Communication," *IEEE Transactions on Intelligent Transportation
> Systems*, vol. 26, no. 1, pp. 1029–1038, Jan. 2025.
> DOI: [10.1109/TITS.2024.3498701](https://doi.org/10.1109/TITS.2024.3498701).
> Free preprint: arXiv:2404.08889, saved locally at
> `references/Ma2025_TITS_TimeHeadway_NoisyV2V_arXiv-2404.08889.pdf`.

This is a **Transactions** paper (the long, rigorous, fully-refereed
format — not a 4-page Letter), published in the flagship journal of its
subfield, by three established names in platoon control theory
(Oklahoma State / Texas A&M). It is also, deliberately, our anchor:
everything in this project either reproduces it exactly or extends it
explicitly, so any reader can check our extension against a fixed,
citable, external ground truth instead of trusting our own code.

**A note on scope, stated once, up front, so nothing below reads as
overreach:** we never modify, re-derive, or dispute a single equation of
this paper. Every one of the 25 systems below either (a) reproduces its
theory exactly, pinned by unit tests against its own printed numbers, or
(b) adds a receiver-side, outer-loop layer on top of that unmodified
theory. Nothing in this project contradicts a claim the paper makes about
the case it analyzes (known, constant `rho`, re-tuned gains, zero delay);
we only show, with our own separately-derived and separately-tested
numbers, what happens *outside* that case — which the paper itself never
claims to cover.

### 1.2 The everyday problem it's solving

Picture a line of cars on a highway, nose-to-tail, each one trying to
keep a safe, fixed gap from the car ahead. Old-school cruise control uses
only a radar: "the guy in front sped up, so I speed up." That reaction is
always a beat late, and — this is the counter-intuitive bit — **that
lag can amplify down the line**. Car 5 reacts a little too slowly, car 6
over-corrects a little to catch up, car 7 over-corrects even more, and by
car 20 a gentle tap of the brakes up front has turned into a full
stop-and-go wave. That amplification effect is called **string
instability**, and it's the mechanism behind a huge fraction of
"phantom" traffic jams — jams with no accident, no lane closure, nothing,
just amplified overreaction.

**Cooperative** ACC (CACC) fixes this by adding a second channel of
information: the car in front doesn't just get watched by radar, it also
*radios ahead* what it's currently doing (its acceleration). That gives
the follower a head start — it can start reacting to what the leader is
*doing right now* instead of waiting to *see the effect* through radar
lag. With that head start, gaps can be much shorter (which is the whole
economic point — shorter gaps per car means more cars per mile of
highway, i.e. more road capacity) **without** the amplification problem,
*provided the radio is trustworthy*.

Ma-2025's specific question: **that radio is never perfect** — it has
noise, like any wireless link. How much noise can you tolerate before
"provided the radio is trustworthy" breaks down, and what's the
*smallest* safe following gap you can design for, given a noise budget?

### 1.3 The vehicle and the spacing rule

Every vehicle is modeled the same simple way — this is standard in the
field and it's precise enough to be useful without needing a full tire/
suspension model:

```
 position   p' = v                     (position changes by velocity)
 velocity   v' = a                     (velocity changes by acceleration)
 accel lag  a' = (u - a) / tau         (the engine/brakes can't respond
                                         instantly — first-order lag,
                                         tau = 0.5 s)
```

`u` is the commanded acceleration (what the controller *asks for*); `a`
is what the car *actually* does a beat later, because engines and
brakes have inertia and response time.

The **spacing policy** is "constant time headway" (CTH): the desired gap
to the car ahead isn't a fixed number of meters — it *grows with your own
speed*, the same way a driving instructor tells you to leave "two
seconds" of following distance, not "two car lengths." Formally, for
follower *i* behind vehicle *i−1*:

```
 desired gap = r + h * v_i
```

`r` is a small fixed standstill buffer (≈1 m), `h` is the **time
headway** — the number of seconds of following distance — and `v_i` is
the follower's own speed. `h` is the single most important design number
in this entire project: it is *the knob that trades safety margin for
road capacity*. Smaller `h` = tighter platoons = more cars per mile, but
less margin against instability. The whole paper (and everything we add
on top) is, at bottom, about finding the smallest `h` you can safely use.

### 1.4 The control law (CTHP)

The paper's controller — "constant-time-headway policy" — for follower
*i*:

```
 u_i = kp * e_i + kv * dv_i + ka * u_ff
```

In plain terms, three ingredients get added together to decide how hard
to accelerate or brake:

* **`kp * e_i`** — "close the gap error." `e_i` is how far off you are
  from the desired gap (measured by radar). If you're too far back,
  push forward a little; too close, back off. `kp = 0.009` is
  deliberately *tiny* — a gentle nudge, not a slam.
* **`kv * dv_i`** — "match closing speed." `dv_i` is the relative
  velocity to the car ahead (also radar). This is what a human calls
  "smooth" driving — react to *how fast the gap is changing*, not just
  its current size.
* **`ka * u_ff`** — **the cooperative part.** `u_ff` is the *radioed*
  acceleration of the car directly ahead — not inferred from radar, told
  to you over the air, in real time. This is the term that doesn't exist
  in plain ACC, and it's what lets `h` get small without instability:
  the follower starts reacting to a disturbance *the instant the leader
  starts producing it*, not one radar-lag-cycle later.

Case-A numbers (the paper's headline design, which we reproduce exactly):
`kp = 0.009, kv = 0.63, ka = 0.5, h = 0.95 s, τ = 0.5 s`.

### 1.5 The radio is not perfect: the 16-bit noise model

Here's the paper's central modeling move. The acceleration value
`u_ff` doesn't arrive at the follower clean — it's quantized to a 16-bit
number for transmission (a real, standard V2V bandwidth constraint) and
that quantization is modeled as **multiplicative noise**:

```
 received value = w(t) * (true value)

 w(t) = (1 - 1/rho) + (1/rho) * sum_{j=0}^{15} z_j / 2^j ,   z_j ~ Bernoulli(gamma_j)
```

Don't worry about the summation — the important, simplifying fact is
this: `w` is a random number that always lands somewhere in the interval
`[1 - 1/rho, 1 + 1/rho)`. Think of `w = 1` as "radio told the truth,"
and `rho` as a single **channel-quality dial**: a *high* `rho`
(e.g. 25) squeezes that interval tight around 1 — a clean channel, the
radioed value is almost exactly right. A *low* `rho` (e.g. 2) opens the
interval wide — a noisy channel, the radioed value could be off by up to
50%. This one number, `rho`, is the paper's entire model of "how good is
the V2V link right now," and it's the number a large fraction of our own
extension is built around **estimating live** instead of assuming known
(see Cluster III).

### 1.6 The stability test: ‖H̃(jω)‖∞ ≤ 1

How do you know, mathematically, whether a disturbance grows or shrinks
as it travels down the platoon? You write the transfer function from one
vehicle's spacing error to the *next* vehicle's spacing error —
essentially "how does car 2's wobble turn into car 3's wobble, at every
possible frequency of wobble" — and check whether its magnitude ever
exceeds 1 at any frequency. If it's ≤ 1 everywhere, every disturbance
shrinks hop by hop: **string stable**. If it exceeds 1 anywhere, that
frequency of disturbance *grows* hop by hop: **string unstable** — the
phantom-jam mechanism.

For the CTHP law, accounting for the radio noise and its worst-case
swing, that transfer function is:

```
 H~(s) = (ka_eff * s^2 * e^(-theta*s) + kv*s + kp) / (tau*s^3 + s^2 + (kv + h*kp)*s + kp)
```

where `ka_eff = w * ka` — the noise directly scales how much the radioed
term counts. The stability requirement is `‖H̃(jω)‖∞ ≤ 1` for every
frequency ω, evaluated at the *worst* `w` in the noise interval (both
ends matter — see Cluster VIII, System 25). This is evaluated exactly on
the imaginary axis, no approximations, so every stability verdict in
this project is a real number, not a simulation guess — this is the
single machine every one of the 25 systems ultimately reports its safety
verdict through, which is *why* it is safe to add so many outer layers:
none of them get to define their own notion of "safe."

### 1.7 The headline theorem: minimum safe time headway

The paper's Theorem III.2 turns all of the above into a **closed-form
formula**: given the noise level `rho`, the feedforward gain `ka`, and
the actuator lag `τ`, what is the *smallest* `h` that is still guaranteed
string-stable? We implement this exactly and it reproduces the paper's
own printed numbers to four decimal places — this is the credibility
anchor for everything else in the project:

| quantity | paper's printed value | our reproduced value |
|---|---|---|
| `h_lb` (min. headway, case A, ρ = 5) | 0.9375 s | **0.9375 s** |
| `ka*` (optimal feedforward gain) | 0.3183 | **0.3183** |
| `h*` (optimal min. headway) | 0.8727 s | **0.8727 s** |

If our code got the paper's own arithmetic wrong, nothing downstream
would be trustworthy — this table is why it is.

### 1.8 The three assumptions the paper leaves on the table

This is the hinge on which the entire rest of this project turns, and it
bears repeating precisely, because it is the difference between
*extending* a paper and *contradicting* it. The theorem above answers:
*given* `rho`, what's the safe `h`? Three conditions are baked into that
"given" — the paper states them as its scope, not as claims about the
general case:

| # | the paper's stated scope | what an operating platoon actually has |
|---|---|---|
| **G1** | `rho` is **known and constant** — you plug in a number | Real V2V quality drifts with distance, buildings, interference, congestion. Nobody hands the controller a live `rho` reading — it has to be measured. |
| **G2** | The gains `(kp, kv)` are **re-tuned for that specific `rho`**, inside a feasible region the paper also derives | A deployed, certified platoon has **fixed** gains — you don't re-flash the ECU's control gains every time the radio quality changes. |
| **G3** | The radioed value arrives with **zero delay** (`θ = 0`) | Real DSRC/C-V2X links have tens to hundreds of milliseconds of latency. |

Everything from here on is our answer to "okay, but what does the
*deployed*, non-oracle version of this system actually look like?" — and
then, having built that, three further answers to failure modes the base
paper doesn't pose as questions at all (bad-zone anticipation, a lying
radio, and "how safe, exactly, is safe"). At no point does closing G1–G3
require disagreeing with the paper's own theorem *within its stated
scope* — every comparison table in this document either matches the
paper's printed numbers exactly (Part 1.7) or explicitly labels which
side of the G1–G3 boundary a number falls on.

---

## Part 2 — The 25 systems

**The headline number, upfront: this project is not "a simulation of the
paper." It is 25 separate, individually-tested engineering subsystems, in
8 clusters, each answering a question the paper leaves open.** 87
automated tests currently pass across the whole stack
(`python -m pytest tests -q`).

| # | system | cluster | one-line job |
|---|---|---|---|
| 1 | Vehicle longitudinal dynamics model | I. Core Engine | double-integrator + actuator-lag physics every other system stands on |
| 2 | Spacing policy & multi-controller law bank | I. Core Engine | CTH error signals + ACC / Ploeg-CACC / CTHP control laws |
| 3 | Platoon orchestrator & RK4 integrator | I. Core Engine | runs a leader + N followers forward in time, one V2V link per hop |
| 4 | Exact frequency-domain stability analyzer | I. Core Engine | the `‖H̃(jω)‖∞` machine every safety verdict in the project runs through |
| 5 | Multiplicative 16-bit V2V noise model | II. V2V Channel | the paper's own channel-quality (`rho`) model, reproduced exactly |
| 6 | V2V delay model | II. V2V Channel | continuous delay-line (theory) + discrete beacons (ROS/sampled) |
| 7 | V2V packet-loss model | II. V2V Channel | Bernoulli loss + zero-order hold, the receiver's real fallback behavior |
| 8 | Online channel estimator | III. QoS-Adaptive Pipeline | live `rho-hat` from beacon-vs-radar mismatch, safety-biased |
| 9 | Timestamp feedforward predictor | III. QoS-Adaptive Pipeline | un-delays the radioed value using its own message timestamp |
| 10 | Rate-limited headway adapter | III. QoS-Adaptive Pipeline | slews `h(t)` toward the live, fixed-gain-honest safety requirement |
| 11 | Online gain scheduler | IV. Gain Adaptation | also moves `(kp, kv)` live, breaking a hard wall System 10 hits alone |
| 12 | Metrics & run logging | V. Metrics & Certification | L2 amplification, throughput, RMS accel, deterministic run logs |
| 13 | Automated certification harness | V. Metrics & Certification | one-command PASS/FAIL verdict across a fixed scenario matrix |
| 14 | ROS 2 message/interface package | VI. ROS 2 Backend | the wire format every distributed node speaks |
| 15 | ROS 2 per-vehicle node | VI. ROS 2 Backend | one real OS process per car, wall-clock-robust, imports System 1–2's code |
| 16 | ROS 2 distributed V2V channel node | VI. ROS 2 Backend | Systems 5–7's impairments, over real message-passing instead of shared memory |
| 17 | ROS 2 leader & recorder nodes | VI. ROS 2 Backend | the prescribed leader trajectory + the CSV/telemetry logger |
| 18 | rviz2 live 3-D marker view | VII. Visualization | color-coded, camera-follows-the-platoon live 3-D |
| 19 | Gazebo kinematic 3-D world | VII. Visualization | a genuine physics-engine world, visually driven by the one true CACC physics |
| 20 | Offline MP4/GIF renderer | VII. Visualization | top-down animated video from any run, no ROS required |
| 21 | Interactive browser live simulator | VII. Visualization | the CTHP physics ported to JavaScript, live sliders, zero backend |
| 22 | Auto-built results dashboard | VII. Visualization | self-contained HTML report auto-discovering the latest run of every study |
| 23 | **Flagship A** — predictive QoS-map spacing | VIII. Flagship | opens the gap *before* a mapped bad-radio zone, not after |
| 24 | **Flagship B** — physics-consistency trust gate | VIII. Flagship | caps how much a lying radio can inject, with a closed-form proof |
| 25 | **Flagship C** — chance-constrained risk certificate | VIII. Flagship | turns the binary stability test into an exact failure probability |

The rest of this Part walks each cluster at the level of "what problem,
what mechanism, what number proves it," grouped exactly as the table
above groups them.

### Cluster I — The core physics & control engine (Systems 1–4)

**System 1 — Vehicle longitudinal dynamics model.** `src/cacc/vehicle.py`,
`tests/test_vehicle.py`. The double-integrator-plus-lag model of Part
1.3, nothing more, nothing less — every other system either feeds this
one new inputs or reads new outputs from it, and it never gets rewritten.

**System 2 — Spacing policy & multi-controller law bank.**
`src/cacc/controllers.py`, `tests/test_controllers.py`,
`tests/test_cthp.py`. The CTH error signals (`e_i`, `e_i'`, `dv_i`) and
**three** independently-selectable control laws, because the project
needs all three for honest comparison, not just the paper's own:

| law | control rule | V2V feedforward | who defines it |
|---|---|---|---|
| `acc` | `u = kp*e + kd*e'` | none — radar only | the "no cooperation" baseline |
| `cacc` | `h*ξ' + ξ = kp*e + kd*e' + u_ff; u = ξ` | predecessor's *commanded* signal | Ploeg et al. 2014 |
| `cthp` | `u = kp*e + kv*dv + ka*u_ff` | predecessor's *realized*, noise-scaled acceleration | **Ma, Pagilla & Darbha 2025 — our anchor** |

**System 3 — Platoon orchestrator & RK4 integrator.**
`src/cacc/platoon.py`, `tests/test_platoon.py`. A leader with a
prescribed trajectory plus N followers, one V2V link per hop, advanced
with a fixed-step (`dt = 0.01 s`) **RK4** integrator — the standard,
accurate way to step differential equations forward in time. This is the
one piece every other system either calls into or is called from.

**System 4 — Exact frequency-domain stability analyzer.**
`src/cacc/analysis.py`, `tests/test_analysis.py`. The `‖H̃(jω)‖∞`
evaluator of Part 1.6, plus the paper's own closed-form Theorem III.2
(reproduced to four decimal places, Part 1.7) — evaluated directly on the
imaginary axis, no Padé approximation, no simulation guesswork. Every
safety claim anywhere in this document, from the base reproduction to
Flagship C, is a number that came out of this one file.

### Cluster II — The V2V channel (Systems 5–7)

A real radio link fails in three genuinely independent ways, so it gets
three independent models, each separately testable — all three live in
`src/cacc/network.py` (`V2VLink`), `tests/test_network.py`.

**System 5 — Multiplicative 16-bit V2V noise model.** The paper's own
channel model from Part 1.5, reproduced exactly (`w(t)` in
`[1 - 1/rho, 1 + 1/rho)`), held piecewise-constant and seeded so every
run is exactly reproducible.

**System 6 — V2V delay model.** Either an exact continuous delay-line
lookup (for theory work — an exact treatment of the delay differential
equation under RK4) or discrete beacons at a fixed rate (for the
ROS/sampled-data world) — a message sent at time *t* is only visible at
time *t + θ*.

**System 7 — V2V packet-loss model.** A coin-flip per beacon; a lost
packet means the receiver just keeps using the last value it has
(zero-order hold) — exactly what a real receiver does when nothing new
arrives. An earlier delay/loss sweep found **delay, not loss, is the
binding constraint** — the design survives up to ≈50% packet loss almost
unchanged, but only ≈0.1–0.2 s of delay before it needs a much bigger
headway. That finding is *why* Cluster III and Flagship A both spend
their effort on delay/quality, not on loss.

### Cluster III — The QoS-adaptive pipeline (Systems 8–10)

**Files:** `src/cacc/estimation.py` (+ hooks in `network.py`,
`analysis.py`, `platoon.py`). **Tests:** `test_qos_adaptive.py`
(17 tests). **Deep dive:** `09_QoS_Adaptive_CACC.md`, decision record
`docs/decisions/D-016`. This is the direct answer to G1 and G3 from Part
1.8 — three cooperating systems in a loop, think of it as the car's own
"sixth sense" about its radio link:

```
 beacon (noisy, delayed) ──┬────────────► [Sys 9] predictor ──► u_ff ──► CTHP
                           │                 ▲ theta-hat            (Sys 2)
 radar (clean) ─────────►  │                 │ (from message stamps)
                           │                 │
                    [Sys 8] estimator ──► rho-hat ──► [Sys 10] adapter ──► h(t)
                    ("how good is my radio    (slew-limited step
                     link, right now?")        toward the safe h)
```

**System 8 — Online channel estimator.** "How good is my radio, right
now?" It compares what the radio *said* the leader's acceleration was
against what the follower's own radar independently measured a beat
later, and turns the mismatch into a live estimate of `rho`. Because the
noise has a strictly bounded range (Part 1.5), a statistical trick (a
99th-percentile of the mismatch, not an average) gives a robust, *always
slightly pessimistic* estimate — built to err on the side of assuming the
channel is a little worse than it really is, because being wrong in the
*safe* direction is free and being wrong the other way is not.

**System 9 — Timestamp feedforward predictor.** "Un-delay the radio
message." Every message carries a timestamp, so the follower knows
exactly how stale it is, and adds a small forward-looking correction (a
"lead," in control-theory language) that estimates where the signal
*would be right now* if it weren't delayed. This is the single most
dramatic number in the whole project: **without** the predictor, a
0.30 s delay forces the required headway up to 7.18 seconds (an
unusable, 7× blow-up); **with** the predictor, the required headway
stays flat at 0.945 s — the delay is effectively erased from the design
problem, for delays up to at least 0.3 s.

**System 10 — Rate-limited headway adapter.** "Act on what you now
know." Slowly and smoothly (never abruptly — a sudden gap change is
itself dangerous) slews the following gap `h(t)` toward whatever the
*current*, fixed-gain-honest stability requirement says, given the live
`rho` estimate.

**The buried finding that reshaped the whole project.** The first
version of System 10 targeted the paper's own closed-form formula (Part
1.7). Direct stability checks *contradicted* it — because that formula
is, by the paper's own stated scope, valid under G2 (gains re-tuned per
`rho`), which a fixed-gain deployed platoon does not have. This is not a
disagreement with the paper — it is a direct confirmation of the
boundary the paper itself draws. Building a table from the *exact*
stability test instead, at the paper's own fixed gains, produced a
striking validation: **at the paper's own design channel quality
(ρ = 5), our fixed-gain requirement is 0.945 s — the paper's own chosen
design is 0.95 s.** Our machinery independently recovers their design
decision to within half a percent. But push the channel worse (`rho`
below about 3), and the fixed-gain requirement **explodes** — 5.28 s at
`rho = 2`, versus the paper's own re-tuned-gain formula's 1.71 s at the
same point, exactly because that formula assumes the re-tuning G2 grants
it. We call the point where no finite fixed-gain headway works at all
the **ρ\* ≈ 1.8 feasibility wall**, and it's the reason Cluster IV exists.

### Cluster IV — Gain adaptation (System 11)

**Same files as Cluster III, plus `estimation.py`'s `GainScheduler`.**
**Decision record:** `docs/decisions/D-017`.

**System 11 — Online gain scheduler.** Cluster III, by design, keeps the
controller gains fixed and only moves the headway `h`. That's the
honest, conservative choice — but it hits the wall above. System 11 is
the deliberate next step: also let the gains `(kp, kv)` move, on the same
live `rho` estimate, ruled by the same safety machinery — this is where
the project *does* exercise the paper's own G2 flexibility (re-tuned
gains), just live and automatically instead of by hand. Tested at the
harshest zone in the project (`rho` dropping from 10 to 2 and back):
headway-only adaptation gets stuck saturated at `h = 2.50 s` and is still
*unstable* (‖H̃‖∞ = 1.019 — "the wall" in numbers); adapting gains **and**
headway together reaches `kv = 0.353, h = 2.33 s` and is stable
(‖H̃‖∞ = 0.99999). The entire `rho` range the paper's own noise model can
express is now covered by *some* combination of the two knobs.

### Cluster V — Metrics & certification (Systems 12–13)

**System 12 — Metrics & run logging.** `src/cacc/metrics.py`,
`logging_config.py`. Peak and L2 spacing errors, vehicle-to-vehicle L2
amplification ratios (the empirical, time-domain companion to System 4's
exact frequency-domain verdict — never the sole source of a stability
claim, see Part 7), lane throughput, RMS acceleration (an energy/comfort
proxy), and deterministic, seedable logging so every run is exactly
reproducible.

**System 13 — Automated certification harness.** `scripts/certify.py`,
decision `D-018`. Everything above produces *numbers*. This system turns
those numbers into a **pass/fail decision** you can hand to someone else
without asking them to read code: one command runs a fixed matrix of
scenarios (base case A, the QoS zone, the deep-zone gain re-tuning case,
the delay predictor case) across multiple random seeds, and emits a
self-contained HTML report. First full run: **CERTIFIED, 11/11 checks
green**. This is the piece that makes the "replaces a hardware testbed"
claim concrete rather than aspirational — a testbed's whole job is
producing a certify/reject verdict, and now the simulator does too, on
demand, deterministically.

### Cluster VI — The ROS 2 distributed backend (Systems 14–17)

**Directory:** `ros2_ws/src/cacc_platoon/`. **Deep dive:**
`08_ROS2_Architecture.md`. Everything in Clusters I–V can, and by default
does, run as one Python process doing one shared calculation for every
vehicle at once — fast to develop, but not how a real platoon works
(every car has its *own* computer, running its *own* clock, with no
shared memory). This cluster is the same control law, re-hosted as **one
real ROS 2 process per vehicle**, talking only over actual pub/sub
message topics.

```
                    (radar surrogate: direct state topics)
 [Sys17]leader ──/platoon/v0/state──► [Sys15]vehicle_1 ──state──► vehicle_2 ─...─► vehicle_N
    │                                     ▲   │                       ▲
    └─/platoon/v0/beacon─┐                │   └─/platoon/v1/beacon──┐│
                         ▼                │                         ▼│
                     ┌────────── [Sys16] v2v_channel ─────────────────┐
                     │  per link: delay (heap+flush) • loss • noise  │
                     └───────────────────────────────────────────────┘
 [Sys17]recorder ◄── all /platoon/v*/state ──► results CSV
 [Sys18/19]viz  ◄── all state ──► rviz2 markers / Gazebo poses
```

**System 14 — ROS 2 message/interface package.** `cacc_platoon_msgs`:
`VehicleState` (index, t, p, v, a, u, e), `V2VBeacon` (sender, t, accel,
u_cmd) — the wire format every other node in this cluster speaks.

**System 15 — ROS 2 per-vehicle node.** One process per car, RK4 at
100 Hz on its own state only, controller = `acc | cacc | cthp`; it
literally `import`s Systems 1–2's Python code, so there is exactly one
copy of the control law in existence — the distributed version cannot
silently drift from the one that's proven against the paper. This
surfaced two real distributed-systems problems a single-process
simulation never encounters: (1) **WSL2 periodically freezes every
process for up to ~1.4 s** — fixed by integrating over *measured*
wall-clock time instead of a fixed timestep, so a stall advances the
physics instead of silently pausing it; (2) **the very first messages
after startup can be stale by 10–100 ms** (normal ROS discovery delay) —
fixed by extrapolating each state message by its own age and holding
every follower pinned for its first 2 seconds.

**System 16 — ROS 2 distributed V2V channel node.** Systems 5–7's three
impairment models, re-implemented as a real message-passing node
(heap-scheduled republication for delay, at 2 ms resolution) instead of a
shared-memory object — the honest, harder version of the channel.

**System 17 — ROS 2 leader & recorder nodes.** `leader_node` (prescribed
sine-burst/brake/constant profiles) and `recorder_node` (subscribes to
every vehicle's state, writes the results CSV) — the two supporting
nodes that make a distributed run reproducible and analyzable after the
fact.

**Cross-validation, not just plausibility.** With the fixes above, the
distributed backend agrees with the single-process core to within a few
percent per hop: noiseless per-hop amplification ratios of
`[0.976, 0.994, 0.996, 0.994, 0.994]` in ROS 2 vs. a uniform `0.9973` in
the offline core — exactly the gap expected given ROS 2 uses per-vehicle
wall-clock timing instead of one shared simulated clock, not a modeling
disagreement.

### Cluster VII — The visualization layer (Systems 18–22)

The only cluster whose job is *communication*, not analysis, but every
entry is a real, separately-built system, which is why each is counted
on its own:

**System 18 — rviz2 live 3-D marker view.** `viz_node.py`: car-body
markers colored green→red by spacing error, text labels with live speed/
error, a `map → platoon` TF frame so the camera rides with the convoy
while world-fixed lane markings slide past.

**System 19 — Gazebo kinematic 3-D world.** A genuine physics-engine
world (`worlds/highway.sdf`) with colored car models — but the cars are
**kinematic** (gravity off, no collision), so Gazebo *renders* the
paper-validated CACC dynamics without ever *simulating* them itself
(`gz_bridge_node.py` teleports each car to its live CACC-computed pose at
50 Hz). This keeps a single source of physical truth even while adding a
second, independent 3-D view.

**System 20 — Offline MP4/GIF renderer.** `render_platoon_video.py`:
top-down highway animation with error-colored cars, a tracking camera,
and speed/error traces, built directly from any CSV or scenario file —
no ROS installation required at all.

**System 21 — Interactive browser live simulator.** `demo/cacc_live_sim.html`
— the actual CTHP physics (Part 1.4–1.6) ported line-for-line to
JavaScript, with live sliders for `rho`, delay, loss, follower count, and
an adaptive-QoS toggle, running in real time in a browser with zero
backend server.

**System 22 — Auto-built results dashboard.** `build_dashboard.py` +
`dashboard_assets.py` + the shared `plotstyle.py` house style —
auto-discovers the latest run of every study and builds one
self-contained, light/dark-theme-aware HTML page with a live animated
hero panel, stat tiles, and every verdict table in this document.

### Cluster VIII — The flagship (Systems 23–25)

Three claims that don't exist anywhere in the base paper's problem
statement at all — each is a patch on a different failure mode, each
shares Cluster I's core engine and System 4's exact stability math
without interfering with the others, and each carries its own rigorous
guarantee plus an honestly stated limit.

**System 23 — Flagship A: predictive QoS-map spacing.**
`src/cacc/qos_map.py`, 7 tests, decision `D-021`. **The everyday
analogy:** System 8's estimator is like driving through fog and only
realizing it's foggy once you're already inside the fog bank — by
definition, a beat too late. Real radio dead-zones (tunnels, underpasses,
known interference sources) are, like real fog banks, *knowable in
advance*. Flagship A gives the platoon a spatial **map** of where the bad
radio zones are and has each car check *ahead* of itself, opening its
headway for whichever is worse — the live estimate or the preview —
*before* crossing into the bad zone, while still in the good channel,
where a bigger gap costs nothing in stability margin. **The number:** at
the moment a reactive-only platoon crosses into a `rho = 3` dead zone at
the paper's design headway, it is already string-unstable for **6.9%**
of the time it spends in that zone; with the map and the preview, that
drops to **0%**, at an honest cost of about 3% more average headway.

**System 24 — Flagship B: physics-consistency trust gate.**
`src/cacc/trust.py`, 12 tests, decision `D-022`. **The everyday
analogy:** the radio link (`u_ff`) is, structurally, an *unverified tip
from a stranger* — the base paper's control law just believes it,
unconditionally. If that stranger is lying — a compromised V2V node
broadcasting a fake acceleration — the raw law has no defense at all: a
phantom +5 m/s² claim on one link, in simulation, nearly rear-ends the
attacked follower (gap collapses by 27 m). Every follower already has a
second, independent way to estimate the leader's acceleration — its own
radar, which an attacker on the V2V network cannot touch. Flagship B
fuses the two with a trust weight `g(r) = 1/(1+(r/r0)^2)` that fades from
"believe the radio" to "fall back to radar" as their disagreement `r`
grows. **The provable guarantee:** a three-line algebraic proof shows the
fused value can never deviate from the honest radar estimate by more
than `r0/2` — for *any* spoof magnitude, with equality at exactly one
point (`r = r0`), never exceeded. Re-running the same attack with the
gate on: the gap that collapsed to −27 m instead bottoms out at **+8.4 m
(safe)**, and the measured worst-case injected command lands *exactly*
on the theoretical bound. **Honest limit, stated up front:** the bound
covers the injection at any instant, not its running total — a small,
sustained lie can still slowly drag the gap down, converting an
*unbounded, instant* catastrophe into a *bounded, slow, detectable*
drift; a persistence watchdog is explicit future work, not a hidden gap.

**System 25 — Flagship C: chance-constrained risk certificate.**
`src/cacc/certificate.py`, 10 tests, decision `D-023`. **The everyday
analogy:** the base paper's stability test is a single binary switch —
stable/unstable — checked at the worst possible noise realization, an
event so rare it happens roughly once in a million. Useful, but it
throws away all information about how much margin a design has on a
normal day. Because the noise model (System 5) is exactly specified,
its entire probability distribution can be *computed exactly* — all
65,536 possible 16-bit outcomes, each with an exact probability — turning
the one binary answer into an exact failure probability and a
risk-budgeted headway ("give me the smallest `h` such that a stability
breach is under 1-in-100," instead of "smallest `h` such that the single
astronomically-unlikely worst case is still safe"). **A structural
finding along the way:** the instability region is **two-sided** — a
radio that under-reports acceleration is dangerous, but one that
*over*-amplifies it is *also* dangerous — stability only holds for a
*middle band*, which is exactly why the base paper's own formula carries
a high-end correction term the paper states but doesn't visually
motivate; our exact sweep numerically confirms why that term has to be
there, agreeing with the paper rather than contesting it. **The
dividend:** at `rho = 3`, accepting a 1-in-100 risk budget instead of the
pure worst case shrinks the required headway from 1.154 s to **1.033 s —
a 10.5% capacity gain**; the deployed base design (`rho = 5`, case A)
turns out to have **exactly zero** failure probability across all 65,536
possible outcomes.

**How the three relate.** A (anticipation) answers "the reaction is too
late." B (integrity) answers "the input might be a lie." C (calibration)
answers "how safe is 'safe,' exactly." C is the backbone: the
risk-budgeted headway it computes is the principled version of the
safety margin A's preview logic currently uses a plain heuristic number
for — a clearly marked next integration step, not yet wired together,
and documented as such rather than silently assumed.

---

## Part 3 — How the 25 systems actually talk to each other

Two separate answers, because there are genuinely two separate runtimes.

### 3.1 Inside one physics tick (the offline / monolithic engine)

This is the sequence that happens, in this order, **every single
0.01-second simulation step**, for every follower, in the Python engine
(Clusters I–V and VIII all participate; Cluster VI replaces this whole
section with real message-passing, Cluster VII only *reads* the output):

```
 1. System 23 (Flagship A) : "what's rho at my current position, and ahead?"
 2. System 5/6/7 (channel)  : delivers this tick's noisy, delayed beacon
 3. System 9 (predictor)    : un-delays the beacon using its timestamp
 4. System 24 (Flagship B)  : fuses the (possibly attacked) beacon value
                               against my own radar before it's trusted
 5. System 8 (estimator)    : turns (beacon vs radar) mismatch into rho-hat
 6. System 10/11 (adapter)  : turns rho-hat (+ Sys 23's preview, + Sys 25's
                               risk budget if wired) into h(t) and,
                               optionally, live gains kp(t)/kv(t)
 7. System 2 (CTHP law)     : u_i = kp*e_i + kv*dv_i + ka*u_ff_eff
 8. System 1 (vehicle)      : RK4-integrates u_i into the next p, v, a
 9. System 4 (analysis)     : the exact ||H~||_inf verdict for the
                               *current* frozen configuration, logged
                               alongside the time-domain trace every tick
```

Every arrow in that list is a real function call with a fixed signature
in `platoon.py` — nothing here is a metaphor. The result object that
comes out of a run literally carries a column for each of these: `h`,
`rho_hat`, `kp_live`/`kv_live`, `trust` (System 24's weight), `ff_inj`
(what got injected), and the running `hinf` verdict from System 4.

### 3.2 Across the distributed backend (ROS 2, Systems 14–17)

Same core ingredients, but instead of one function calling the next
inside one process, they're separated into **real operating-system
processes** that only know about each other through named message
topics — deliberately the harder, more realistic version:

```
 [17]leader_node ──/platoon/v0/beacon──► [16]v2v_channel_node ──/platoon/v1/v2v──► [15]vehicle_1_node
                                          (adds delay, loss,                        (RK4 @ 100Hz,
                                           noise per-link)                          imports Sys 1-2 code)
 [15]vehicle_1_node ──/platoon/v1/state──► vehicle_2_node   (also feeds [17]recorder_node,
                                                               [18/19]viz nodes)
```

The vehicle nodes run the *literal same Python functions* as the offline
engine — there is exactly one implementation of the control law in
existence, imported by both runtimes, which is *why* the two can be
meaningfully cross-checked against each other rather than each just
being internally consistent (agreement numbers in Cluster VI above).

### 3.3 One table: who reads what, who writes what

| system(s) | reads | writes |
|---|---|---|
| 1 Vehicle model | commanded `u_i` | `p, v, a`, spacing error `e_i` |
| 2 Control laws | `e_i, dv_i, u_ff_eff` | commanded `u_i` |
| 3 Platoon orchestrator | every vehicle's state | the run trace / `SimResult` |
| 4 Stability analyzer | the current frozen `(h, kp, kv, rho, theta)` | `‖H̃‖∞` verdict |
| 5–7 V2V channel | true `a_{i-1}`, position (for Sys 23's schedule) | noisy/delayed/lossy `u_ff` |
| 8 Estimator | beacon, radar, timestamps | `rho_hat` |
| 9 Predictor | beacon, timestamps | un-delayed `u_ff` |
| 10 Adapter | `rho_hat`, Sys 23's preview | `h(t)` |
| 11 Gain scheduler | `rho_hat` | `kp(t), kv(t)` |
| 12 Metrics | any `SimResult` | L2/throughput/RMS numbers |
| 13 Certification | every `metrics.json` from a run matrix | one PASS/FAIL HTML report |
| 14–17 ROS 2 backend | *replaces* the shared-memory calls above with topics | the same signals, over the wire |
| 18–22 Visualization | any `SimResult` / CSV / `metrics.json` | figures, HTML, MP4, 3-D poses |
| 23 Flagship A | vehicle position, the shared spatial map | a *preview* `rho` fed into System 10 |
| 24 Flagship B | raw `u_ff` from System 5, radar from System 1 | the *fused* `u_ff_eff` System 2 actually uses |
| 25 Flagship C | the exact channel law (System 5's noise model) | `exceedance_prob(h)`, risk-budgeted `h_cc(eps)` |

---

## Part 4 — Exactly where we differ from the base paper

Every row below is a claim the base paper's own stated scope does not
cover — either because Part 1.8's G1–G3 explicitly assume it away, or
because it simply never poses the question. **None of these rows dispute
a number the paper prints; every "our" figure is our own, separately
derived and separately tested, and every comparison against a paper
number is either an exact match (Part 1.7) or explicitly on the far side
of a scope boundary the paper states itself (G1–G3).**

| topic | Ma-2025's stated scope | what we do | quantified delta |
|---|---|---|---|
| radio quality `rho` | known, constant, handed to the controller (G1) | estimated **online**, per-link, from beacon-vs-radar residuals (System 8) | tracks true `rho` within ~5%, safety-biased |
| controller gains | re-tuned per `rho`, inside a feasible region (G2) | **fixed** by default (System 10), deployment-realistic; the divergence this causes is *measured*, not hidden; re-tuning is then exercised live (System 11) | ρ\* ≈ 1.8 feasibility wall found and reported |
| radio delay | zero, `θ = 0` (G3) | modeled explicitly (System 6), and **actively compensated** with a timestamp predictor (System 9) | required headway held flat at 0.945 s up to θ = 0.3 s (vs. 7.18 s uncompensated) |
| response to a bad zone | not modeled — the paper analyzes one frozen `rho` | anticipated **before entry** (System 23), using a spatial map + preview | in-zone string-instability drops from 6.9% of the time to 0% |
| trust in the radioed value | unconditional — the law "believes" whatever arrives | **fused against independent radar** (System 24) with a provable injection bound | unbounded spoof exposure → provably capped at `r0/2`, any spoof size |
| the stability guarantee | binary: `‖H̃‖∞ ≤ 1` at the single worst-case noise draw | an **exact failure probability** (System 25), and a headway chosen against a risk budget | worst-case headway 1.154 s → 1.033 s at a 1-in-100 budget (10.5% capacity gain) |
| execution model | not addressed — a theory/simulation paper | **cross-validated on a genuinely distributed backend** (Systems 14–17, real message passing, real OS scheduling jitter) | agreement within a few percent per hop |
| validation | closed-form derivation + the authors' own simulation | **87 automated regression tests**, a one-command certification harness, every number traceable to a script | `pytest -q` → 87 passed |

The paper is the physics and the stability theory — we never touch or
re-derive those (**Part 1.6's transfer function is used unmodified
everywhere**, including inside all three Flagship systems). Everything in
this table is an *outer layer*: what a controller has to do, on top of
that fixed physics, to survive being deployed on an actual road with an
actual imperfect radio.

---

## Part 5 — Where the implementation stands, and what's left

Per the master execution plan (`10_Handoff_Plan.md`):

| workstream | goal | status |
|---|---|---|
| **A** | Port Cluster III's QoS-adaptive pipeline (Systems 8–10) into the ROS 2 backend (Systems 14–17), so it runs distributed with live 3-D visualization, not just offline | **not yet built** — fully specified, step-by-step, in `10_Handoff_Plan.md` §3; the offline pipeline is done and validated, this is "translate it," not "invent it" |
| **B** | Write the IEEE-format paper | **not yet written** — every number it needs already exists in `results/**/metrics.json`; skeleton and section-by-section source map already drafted in `10_Handoff_Plan.md` §4 |
| **C** | Slides / live demo | **not yet built** — Cluster VII (Systems 18–22) already has every asset the demo script calls for |

Everything else — Systems 1–13 and 18–25, all offline-validated, 87 tests
green — is **complete**. The honest one-sentence status: *the science and
engineering are done and proven; what's left is packaging it for an
audience* (the paper, the slides, and the ROS port that makes the demo
distributed instead of a single script).

---

## Part 6 — Literature sweep: published IEEE Transactions papers only

Every entry below is a **peer-reviewed, published IEEE Transactions
article with a DOI** — no arXiv preprints, no conference papers, no
"early access, not yet in an issue" ambiguity where a firmer venue was
found instead. Five of the newer entries were located specifically to
refresh the mostly-2020–2022 supporting citations in
`02_IEEE_Base_Paper.md` against 2024–2025 Transactions literature, split
across the four areas the project touches. Two now have real PDFs saved
in `references/`; the rest are IEEE-Xplore-paywalled with no legally
open copy found, cited by DOI only — the same status the project's
existing Naus 2010 reference already carries, so this is not a new kind
of gap. **Every summary below is stated only at the level of detail
confirmed from the paper's own abstract or publisher record — we have
not read these papers' full derivations, and this document does not
claim to.**

### 6.0 The literature backbone: OATD and the classic foundation

Before the recency sweep in §6.1–6.3, the project already carries a
second literature channel, tracked in `03_OATD_Thesis.md`: **OATD**
(Open Access Theses and Dissertations, [oatd.org](https://oatd.org)) — a
search index over open-access graduate theses and dissertations, used
here not because a thesis outranks a Transactions paper, but because
theses from the major platooning research groups (TU Eindhoven's
Ploeg/Nijmeijer lab, UC Berkeley PATH, KTH, TU Delft) tend to contain the
**longest, most pedagogically complete derivations** of the exact string-
stability machinery this project depends on — useful as a worked-example
backbone underneath the terser Transactions papers, in the same way a
textbook chapter sits underneath a journal article. **Transparency note,
carried over from `03_OATD_Thesis.md` unchanged:** OATD itself blocks
automated fetching, so the specific works below were located and
verified through their own open repositories instead — the search terms
in `03_OATD_Thesis.md` are what you'd use to browse OATD by hand for
more.

Of the three works this channel currently anchors, **one is a genuine
published IEEE Transactions paper** (kept to the same citation standard
as §6.1–6.3) and **two are open-access preprints/reviews** used as
background reading only — consistent with this project's rule of never
presenting a preprint as if it were peer-reviewed Transactions literature.

> **S. Öncü, J. Ploeg, N. van de Wouw, and H. Nijmeijer**, "Cooperative
> Adaptive Cruise Control: Network-Aware Analysis of String Stability,"
> *IEEE Transactions on Intelligent Transportation Systems*, vol. 15,
> no. 4, pp. 1527–1537, 2014. DOI: [10.1109/TITS.2014.2302816](https://doi.org/10.1109/TITS.2014.2302816).
> `references/Oncu2014_CACC_Network-Aware_Analysis_of_String_Stability_IEEE-TITS.pdf`
> (open copy via the TU Eindhoven research portal).

**Why this one earns a place next to the anchor paper.** Confirmed from
the paper's own abstract: it approaches CACC design from a **Networked
Control Systems (NCS)** perspective and builds a modeling framework that
explicitly accounts for **sampling, hold, and network-induced delay** —
the same three real-world radio effects Systems 6–7 (the delay and
packet-loss models) and System 9 (the timestamp predictor) exist to
handle, but derived here from the general NCS literature rather than the
base paper's specific 16-bit noise model. It's the natural "why does
System 6's delay-line/sampled-beacon split make sense at all"
citation — the base paper (Part 1) supplies the noise model, Öncü et al.
supply the classical justification for treating delay and sampling as
their own first-class NCS phenomena rather than folding them into noise.
It is also, by publication venue and rigor, exactly the same class of
source as Ploeg et al. 2014 (already the project's "classic foundation"
reference in `02_IEEE_Base_Paper.md`) — both from the same TU Eindhoven
group, both IEEE T-ITS, both from 2014, and both cited here as
**foundational**, not **recent** — they predate, and do not compete
with, the 2024–2025 recency sweep in §6.1–6.3.

Background/context only (open-access, **not** Transactions-refereed, not
offered as citation-equivalent to the papers above — the same standard
§6.5 applies to the newer preprints applies here):

* "Stochastic Lp String Stability Analysis in Predecessor-Following
  Platoons Under Packet Losses," arXiv:2403.11043.
  `references/arXiv2403.11043_Stochastic_Lp_String_Stability_under_Packet_Losses.pdf`
  — background reading on packet-loss robustness, the same impairment
  System 7 models; useful for intuition, not for a citation list.
* "Adaptive Cruise Control in Autonomous Vehicles: Challenges, Gaps,
  Comprehensive Review, and Future Directions," arXiv:2510.03300.
  `references/arXiv2510.03300_ACC_in_Autonomous_Vehicles_Review.pdf`
  — a broad survey, useful for scoping a related-work section's breadth,
  not for any specific numeric claim.

### 6.1 General recency + delay compensation — a direct sequel by the base paper's own authors

> **G. Ma, P. R. Pagilla, and S. Darbha**, "Robust Cooperative Adaptive
> Cruise Control System Design: Trade-Off Between Parasitic Actuation Lag
> and Communication Delay," *IEEE Transactions on Intelligent
> Transportation Systems*, vol. 26, no. 6, pp. 7980–7989, Jun. 2025.
> DOI: [10.1109/TITS.2025.3553812](https://doi.org/10.1109/TITS.2025.3553812).
> *(IEEE Xplore / publisher access; no open PDF located.)*

This is the single best find of the sweep: the **same three authors** as
the anchor paper, publishing a follow-on Transactions article that
extends their own Theorem-III.2-style framework specifically to
**actuation lag and communication delay** — confirmed, from the paper's
own abstract, to derive a headway lower bound that depends explicitly on
the actuator-lag bound and the delay bound, and to provide a "systematic
procedure" for robust CACC design under both. This is the closest thing
in the literature to "what would Ma-2025's own authors say about the
delay gap (G3) our own Systems 6 and 9 close" — cite it in a related-work
section as direct evidence that the delay question is the field's (and
the original authors') own acknowledged next step, independently of our
own D-008 study reaching the same conclusion first. **We do not claim
our numeric results match this paper's** — we have not read its
full derivation, only confirmed its scope and headline claim from the
abstract; any detailed comparison should wait until the paper is read in
full.

Existing anchor, already in the project's bibliography and still the
best-established 2024 Transactions citation for delay + multi-vehicle
anticipation:
> M. Bouadi, R. Jiang, B. Jia, and S. Zheng, "String Stability Analysis
> of Cooperative Adaptive Cruise Control Vehicles Considering
> Multi-Anticipation and Communication Delay," *IEEE Trans. Intell.
> Transp. Syst.*, vol. 25, no. 9, pp. 11359–11369, 2024,
> doi: [10.1109/TITS.2024.3371426](https://doi.org/10.1109/TITS.2024.3371426).

### 6.2 V2V security / spoofing / trust (supports Flagship B — System 24)

> **[PDF]** **C. Zhao, R. Ma, M. Wang, J. Xu, and L. Cai**, "Safeguard
> Vehicle Platooning Based on Resilient Control Against False Data
> Injection Attacks," *IEEE Transactions on Intelligent Transportation
> Systems*, 2024. DOI: [10.1109/TITS.2024.3424687](https://doi.org/10.1109/TITS.2024.3424687).
> `references/Zhao2024_TITS_SafeguardPlatooning_ResilientControl_FDIA.pdf`
> (author-hosted open copy).

Confirmed mechanism, from the paper's own description: each vehicle
shares local state-deviation vectors with multiple neighbors and
discards the vectors farthest from the origin (a number of removals
matched to the assumed maximum number of simultaneous attackers) —
a **distributed, vector-consensus** defense, mechanistically distinct
from Flagship B's **single-link, physics-consistency** fusion. Good
related-work framing: *"[Zhao et al.] achieve resilience through
distributed consensus across multiple neighbors' shared state; our gate
achieves a provable, closed-form injection bound on a single link using
only the follower's own radar as a second opinion — the two are
complementary defense layers, not competing solutions to the same
attack surface."*

> **N. Ahmed, A. Ameli, and H. Naser**, "Detection, Identification, and
> Mitigation of False Data Injection Attacks in Vehicle Platooning,"
> *IEEE Transactions on Vehicular Technology*, vol. 74, no. 1,
> pp. 1296–1309, Jan. 2025. DOI: [10.1109/TVT.2024.3467813](https://doi.org/10.1109/TVT.2024.3467813).
> *(IEEE Xplore only — no open preprint found.)*

Confirmed mechanism: a state-space model general enough to cover any
information-flow topology, an **Unknown Input Observer (UIO)** estimating
each vehicle's internal states, and a dedicated Detection-UIO / 
Identification-UIO pair per vehicle to detect and isolate the specific
attacked parameter. This is the best direct-contrast citation for System
24: it is a **model-based, observer-residual detection** approach — flags
an attack after enough residual builds up — versus Flagship B's
**continuous, always-on trust-weighted fusion** that never needs a
separate detection step because the fusion itself is graded and
instantaneous. Framing: *"[Ahmed et al.] detect and then isolate FDIA via
observer residuals, with detection latency inherent to any residual-based
scheme; our gate instead fuses continuously and provides a bound on the
worst-case injected command from the first instant of an attack, at the
cost of not identifying or isolating the attacker — the two approaches
answer different questions (who attacked, vs. how much damage can any
attack possibly do) and would compose well together."*

### 6.3 Stochastic / probabilistic robustness (closest confirmed Transactions anchor for Flagship C — System 25)

> **H. Rezaee, K. Zhang, T. Parisini, and M. M. Polycarpou**,
> "Cooperative Adaptive Cruise Control in the Presence of Communication
> and Radar Stochastic Data Loss," *IEEE Transactions on Intelligent
> Transportation Systems*, vol. 25, no. 6, pp. 4964–4976, 2024.
> DOI: [10.1109/TITS.2023.3335310](https://doi.org/10.1109/TITS.2023.3335310).
> *(IEEE Xplore only — no open preprint found.)*

Confirmed result, from the paper's own abstract: under their proposed
control strategy, if the probability of successfully receiving the
required data is nonzero for every vehicle, the inter-vehicle distances
converge **almost surely** to their desired values — a probabilistic
guarantee on a platoon under stochastic communication/radar data loss,
by two established control theorists (Parisini, Polycarpou). This is the
closest **published, Transactions-grade** paper found to Flagship C's
probabilistic framing — though it should be stated honestly: its
"almost sure convergence under nonzero reception probability" result is
a different mathematical object from System 25's **exact failure
probability of a frequency-domain stability criterion** — the two are
not directly comparable numbers, and this document does not claim they
are. Cite it as evidence that **probabilistic, rather than purely
worst-case, robustness guarantees are an established and active line in
the same Transactions venue as the base paper itself** — the closest
available legitimation of Flagship C's general approach, not a source of
comparable numbers.

**An honest gap, stated rather than papered over:** a systematic search
for a published IEEE Transactions article using the specific term
"chance-constrained" applied to platoon or CACC string stability did not
turn up a confirmed match as of this sweep. If you find one through
campus IEEE Xplore access, it belongs in this section ahead of the
Rezaee et al. paper; until then, the paper above is the honest closest
anchor, not a perfect match, and is presented as such.

### 6.4 On "the letter you noticed was newer"

You're right that Letters venues (IEEE Control Systems Letters, Robotics
& Automation Letters, Communications Letters) tend to look newer in a
reference list — they're short-format and turn around in months, not the
12–18 months a full Transactions paper takes, so a literature sweep
naturally surfaces more recent Letters than recent Transactions on the
same topic. Every entry in §6.1–6.3 is a full Transactions paper by
design, matching your base paper's own venue class. If there's a
specific Letter you had in mind, point me at it and I'll find its
closest Transactions-grade counterpart directly.

### 6.5 A note on what this sweep excluded, on purpose

An earlier pass through this literature surfaced several very recent,
closely relevant **arXiv preprints** — including, notably, a *different*
2025 preprint by the same base-paper authors (arXiv:2509.20722, on
Pontryagin-interlacing-theorem delay bounds for ACC/CACC/CACC+) that sits
alongside the confirmed §6.1 Transactions paper as further evidence the
authors are actively extending their own delay analysis. Those preprints
are still saved in `references/` for your own reading, but per your
instruction they are **excluded from this citation list**: none of them
is confirmed as a published, DOI-bearing Transactions article, and a
citation list for a project anchored on a Transactions paper should not
quietly mix in unrefereed preprints as if they carried the same weight.

---

## Part 7 — Honest limitations, all in one place

Pulled up from each system's own doc, because a credible project states
its limits next to its claims, not in a separate place nobody reads:

* **Systems 8–11 (adaptation):** need *persistent excitation* — the
  channel is literally unobservable at zero acceleration, so scenarios
  inject a small, disclosed probing dither (≈1 cm/s velocity ripple,
  imperceptible) to keep the estimator fed.
* **System 23 (Flagship A):** relocates the gap-opening transient into
  the *good* channel rather than removing it, at a real (if small, ~3%)
  headway cost; cannot beat causality if the map itself is wrong — the
  live estimator (System 8) remains the safety backstop for unmapped
  zones.
* **System 24 (Flagship B):** bounds the *instantaneous* injected
  command, not its time-integral — a small, sustained lie can still
  slowly drag the gap down. A persistence detector (CUSUM-style) is
  explicit future work, not a hidden gap.
* **System 25 (Flagship C):** the classic mean-square risk notion is
  *too permissive* here (7.6% failure rate) and is kept only as an
  instructive contrast; the distribution-free (Chebyshev) bound is
  *honest but blunt* — it collapses back to the worst case at this
  project's headline channel quality, buying no dividend. No published
  Transactions paper using the literal "chance-constrained" framing for
  platoons was confirmed in the §6.3 sweep — the risk-certificate
  framing here is presented as our own construction on top of the exact
  channel law, not as replicating an existing named method from the
  literature.
* **Systems 14–17 (ROS 2 backend):** the QoS-adaptive pipeline (Systems
  8–10) is not yet ported into it — Workstream A (Part 5) — so today's
  distributed, real-time validation covers the *base* controller only,
  not yet the full adaptive/Flagship stack.
* **Cross-cutting:** every stability claim in every system is a
  frequency-domain `‖H̃‖∞` verdict from System 4, never a time-domain
  eyeball judgment — this project's own zone-experiment data shows that
  near the stability boundary, time-domain traces of a *stable* run can
  look rougher than an *unstable* one. Anyone reading the figures without
  the verdict table can be misled; the verdict table is always the
  ground truth.
* **§6's external-paper summaries:** every claim about a paper we did
  not write is bounded to what its own abstract/publisher record states —
  we have not obtained and read the full text of any of the newer §6
  Transactions papers, and this document does not claim otherwise. Where
  a full-text comparison would strengthen a claim, that is flagged as
  future work (campus IEEE Xplore access), not silently assumed.

---

*Written as the project's own master explainer, 2026-07-14; expanded to
25 systems and a Transactions-only literature sweep the same day. Cross-
refs: [[cacc-project-context]] for the running project history. Numbers
here are point-in-time — if this doc and a script disagree, the script
wins; regenerate with the commands in each system's linked doc.*
