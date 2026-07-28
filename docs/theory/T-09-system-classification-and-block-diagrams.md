# T-09 — System classification & control-block diagrams

**Scope.** This doc captures two things that don't live anywhere else in
the repo yet: (1) a formal classification of *what kind of system* this
project is, across the standard control-theory axes, and (2) the
canonical control-block-diagram breakdown (reference / summing junction /
controller / plant / feedback / feedforward / disturbance), for both the
base Ma-2025 loop and the flagship extensions. It also carries a durable,
git-tracked markdown copy of the signal-flow & communication architecture
that was built as an interactive artifact (published externally on
claude.ai, not itself part of this repo) — so the content survives even
if that link isn't revisited.

**Not duplicated here:** the vehicle model equations and parameters
(**T-01**), the spacing-policy error signals (**T-02**), the string-
stability transfer function (**T-03**), the Ma-2025 theorem (**T-04**).
This doc assumes those and builds the classification/diagram layer on
top of them.

---

## 1. What kind of system is this?

A single-sentence classification: **a distributed, cascade-interconnected,
dual-channel (feedback + feedforward) networked control system — nominally
linear time-invariant at its core, but stochastic, nonlinear at the
saturation boundary, and (in the extended layer) adaptive/gain-scheduled.**
Each clause is a separately-justifiable axis:

| Axis | Classification | Why |
|---|---|---|
| Feedback vs. feedforward | **Both — dual-channel** | Radar closes a feedback loop on the *effect* of the leader's maneuver (`e_i`, `Δv_i`, T-02); V2V feeds forward the *cause* (`u_ff`, the leader's own realized acceleration) before radar would see it. This second channel is the entire mechanism that lets small headways stay string-stable. |
| Linear vs. nonlinear | **Linear in the certified envelope; nonlinear at the boundary** | Plant + CTHP law are linear (all `‖H̃‖∞` theory assumes this, T-03). Actuator saturation (`u∈[u_min,u_max]`, `vehicle.py::clamp`) is a hard nonlinearity outside that certified region — the certification harness checks min-gap on the raw nonlinear sim separately for exactly this reason. |
| Time-invariant vs. time-varying | **LTI at the base design; quasi-LPV once adaptation is on** | Fixed `(h,kp,kv)` → classic LTI, admits a transfer function `H̃(s)`. With Systems 10/11 live, `h(t), kp(t), kv(t)` slew as functions of the estimated channel — a linear parameter-varying (LPV) system, stable only because the parameter drift is *quasi-static* relative to the loop dynamics (T-08). |
| Deterministic vs. stochastic | **Both, deliberately** | The channel gain `w(t)` is a random variable by construction (`network.py`, System 5) — the base question is "is `‖H̃‖∞≤1` at the *worst* `w`?" (deterministic worst-case robust control). Flagship C reframes the same plant as **chance-constrained / stochastic robust control**: the full probability law of `w` is computed exactly, and the design question becomes `P(‖H̃‖∞>1) ≤ ε`. |
| Centralized vs. distributed | **Distributed, string/cascade topology** | N identical, decoupled SISO loops, each follower sensing/communicating *only* with its immediate predecessor — a chain, not a mesh. The monolithic Python engine centrally *simulates* this distributed system; the ROS 2 backend actually *is* distributed (one OS process per vehicle, real message passing, no shared state). |
| Continuous vs. networked/sampled | **Continuous plant, networked control system (NCS) on the cooperative channel** | Vehicle dynamics are continuous-time ODEs; the V2V link is where sampling enters — 16-bit quantization, discrete beacon timing, delay, packet loss. Textbook NCS — the reason Öncü/Ploeg's NCS-framing paper anchors the delay/sampling treatment (`02_IEEE_Base_Paper.md`, literature refresh). |

---

## 2. The base control-block diagram (one predecessor–follower pair)

```
 desired-gap        +   ┌──────────┐  u_i   ┌────────────┐  a_i   ┌───────────┐
 policy: r+h·v_i  ──Σ──►│  CTHP    │───────►│  Actuator  │───────►│   Plant    │──► p_i, v_i
 (from own v_i)     │ − │controller│        │ saturation │        │ (double   │
                     ▲   │(compens.)│        │  + lag τ   │        │integrator)│
 radar (d_i, Δv_i) ──┘   └────▲─────┘        └────────────┘        └─────┬─────┘
        [feedback path]        │ ka·u_ff                                  │
                          ┌─────┴──────┐                                  │
        V2V link ────────►│ feedforward │◄── a_{i−1} (predecessor's       │
     [feedforward path,   │   channel   │     REALIZED accel — the       │
      the "disturbance"    └────────────┘     exogenous disturbance      │
      is measured, not                        entering the STRING here)  │
      rejected blind]                                                    │
                                                                          ▼
                                                            becomes p_{i−1}, v_{i−1}, a_{i−1}
                                                            for vehicle i+1's own loop
```

| Block | Classical name | This project's realization |
|---|---|---|
| Reference | Setpoint generator | Not fixed — `d_des,i = r + h·v_i`, a policy-generated, speed-dependent reference (`controllers.py`, T-02) |
| Summing junction | Comparator | `e_i = d_i − d_des,i` |
| Controller | Compensator (static PD + feedforward) | CTHP: `u_i = kp·e_i + kv·Δv_i + ka·u_ff` |
| Feedback path / sensor | Measurement | Radar: gives `d_i` (hence `e_i`) and `Δv_i` — closes the loop |
| Feedforward path | Anticipatory/preview compensation | V2V delivering `u_ff = w(t)·a_{i-1}(t)` — a disturbance-*measurement* problem, not disturbance-rejection |
| Disturbance input | Exogenous input | The leader's own commanded maneuver `v_0(t)` — what `‖H̃‖∞` measures the hop-to-hop amplification of |
| Actuator + saturation | Actuator dynamics | `τ`-lag + `[u_min,u_max]` clamp (T-01) |
| Plant | Process | Double integrator `ṗ=v, v̇=a` |
| Interconnection | Cascade coupling | Vehicle `i`'s own `(p_i,v_i,a_i)` becomes the "predecessor" signal for vehicle `i+1` — why it's a string, not N independent loops |

---

## 3. The extended control-block diagram (flagship additions)

Each addition maps onto a named, real control-theory concept — not an
ad-hoc patch:

| Block | Classical name | This project's realization |
|---|---|---|
| Timestamp predictor (Sys 9) | Dead-time / Smith-predictor-style compensation | Un-delays a channel with known transport delay `θ̂` by extrapolating along the signal's own trend |
| Channel estimator (Sys 8) | Online parameter estimator | Beacon-vs-radar residual → live `ρ̂` |
| Headway adapter + gain scheduler (Sys 10/11) | Indirect adaptive control / gain scheduling | Estimate a parameter (`ρ̂`), then move controller parameters (`h, kp, kv`) as a function of it — slew-rate-limited to stay quasi-static (T-08) |
| Trust gate (Sys 24, Flagship B) | Residual-based sensor fusion / robust estimator | Fuses `a_v2v` vs `a_radar` with `g(r)=1/(1+(r/r0)²)` — complementary-filter spirit, closed-form algebraic bound instead of a dynamic filter |
| Predictive QoS map (Sys 23, Flagship A) | Preview control | Feeds forward a known-in-advance future condition (spatial `ρ(x)` map) instead of reacting once inside it |
| Risk certificate (Sys 25, Flagship C) | Chance-constrained / probabilistic robust control (design-time) | Not in the real-time loop — a supervisory/offline verification block recomputing `‖H̃‖∞`'s exact failure probability, used to *choose* `h` before deployment |

**Platoon-level picture:** chain the per-vehicle blocks together — leader
→ follower 1's loop → follower 2's loop → … → follower N's loop, each
loop's plant output feeding the next loop's disturbance input. String
stability (T-03) is whether that chain of identical blocks contracts or
amplifies a disturbance, hence its own hop-to-hop transfer function
`H̃(s)`, distinct from any single loop's own stability.

---

## 4. Signal-flow & communication architecture

Full interactive version (schematic-style diagram, two runtimes, formula
ledger): published as a Claude Artifact,
`https://claude.ai/code/artifact/5cf63f9b-e639-4d97-96df-56dc2c79bb19`
(external hosting — private by default, not part of this repo; the table
below is the durable copy).

### 4.1 Two runtimes, two communication modes

- **Monolithic Python core** (`src/cacc/platoon.py::PlatoonSim.step()`):
  every arrow below is a **direct in-process function call** — same
  object, same tick, no serialization.
- **ROS 2 distributed backend** (`ros2_ws/src/cacc_platoon/`): every
  arrow is a **pub/sub message over a named DDS topic** — separate OS
  processes, separate clocks, no shared memory.

### 4.2 Per-tick call order (monolithic core)

```
 1. Flagship A   : rho_at(x), min_rho_ahead()              → preview rho
 2. V2V channel  : noise w(t), delay theta, loss            → noisy/delayed beacon
 3. Predictor    : y + theta_hat * slope(beacons)            → un-delayed u_ff
 4. Flagship B   : g(r)=1/(1+(r/r0)^2)                       → u_ff_eff (+ residual r)
 5. Estimator    : rho_hat = q99(beacon vs radar mismatch)   → rho_hat
 6. Flagship C   : exceedance_prob(h), chance_headway(eps)   → risk budget (NOT yet wired)
 7. Adapter/Sched: h(t) slew, kp(t)/kv(t) optional           → h, kp, kv
 8. Control law  : u_i = kp*e_i + kv*dv_i + ka*u_ff_eff      → u_i
 9. Vehicle      : RK4 step xdot=[v,a,(u-a)/tau]             → p,v,a (next tick)
10. Analyzer     : ||H~(jw)||_inf on frozen (h,kp,kv,rho,th) → stability verdict
```

### 4.3 ROS 2 topic chain (distributed backend)

```
 leader_node --/platoon/v0/beacon--> v2v_channel_node --/platoon/v1/v2v--> vehicle_1_node
                                    (delay heap, loss,                    (RK4 @ 100Hz,
                                     noise per-link)                      imports Sys 1-2 code)
 vehicle_1_node --/platoon/v1/state--> vehicle_2_node --> ... --> vehicle_N_node
 all vehicle /state topics --> recorder_node (CSV) and viz_node/gz_bridge_node (rviz2/Gazebo)
```

### 4.4 Full ledger

| Sys | System | File | Formula | Comm mode | Talks to |
|---|---|---|---|---|---|
| 1 | Vehicle dynamics | `vehicle.py` | `ẋ=[v,a,(u−a)/τ]` | call | ← Sys 2 (u_i) · → Sys 4, radar tap |
| 2 | Control law (CTHP) | `controllers.py` | `u=kp·e+kv·Δv+ka·u_ff` | call | ← Sys 24, Sys 10 · → Sys 1 |
| 3 | Platoon orchestrator | `platoon.py` | RK4, dt=0.01s, per-hop V2VLink | call | calls every system, every tick |
| 4 | Stability analyzer | `analysis.py` | `‖H̃(jω)‖∞ ≤ 1` | call | ← frozen (h,kp,kv,ρ,θ) · → SimResult |
| 5–7 | V2V channel (noise/delay/loss) | `network.py` | `w∈[1−1/ρ,1+1/ρ)`, θ, Bernoulli(p_loss) | call | → Sys 9 |
| 8 | Channel estimator | `estimation.py` | `ρ̂ = q99(beacon−radar)` | call | ← Sys 24 · → Sys 10/11 |
| 9 | Timestamp predictor | `estimation.py` | `ŷ=y+θ̂·slope(beacons)` | call | ← Sys 5–7 · → Sys 24 |
| 10/11 | Headway adapter + gain scheduler | `estimation.py` | `h(t)` slew; `kp(t),kv(t)` | call | ← Sys 8, 23, 25 · → Sys 2 |
| 14–17 | ROS 2 msgs, nodes, leader, recorder | `ros2_ws/src/cacc_platoon/` | `VehicleState`, `V2VBeacon` | topic | see §4.3 |
| 23 | Flagship A — predictive QoS map | `qos_map.py` | `rho_at(x)`, `min_rho_ahead()` | call | → Sys 10 (preview ρ) |
| 24 | Flagship B — trust gate | `trust.py` | `g=1/(1+(r/r₀)²)` | call | ← Sys 9, Sys 1(radar) · → Sys 2, Sys 8 |
| 25 | Flagship C — risk certificate | `certificate.py` | `exceedance_prob(h)`, `chance_headway(ε)` | call | → Sys 10 (dashed — not yet wired) |

---

*Written 2026-07-15 from a tutoring/Q&A session covering the control
theory and system architecture. Sources: `docs/theory/T-01…T-08`,
`docs/MASTER_SYSTEM_EXPLAINER.md` Parts 2–3, and `src/cacc/vehicle.py` /
`controllers.py` (read directly to verify Systems 1–2's formulas). If
this doc and the source ever disagree, the source wins.*
