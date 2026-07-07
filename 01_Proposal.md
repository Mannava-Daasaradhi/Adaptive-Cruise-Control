# Project Proposal

## Title
**String-Stable Cooperative Adaptive Cruise Control for Vehicle Platoons Under Imperfect V2V
Communication: Noise, Delay, and Packet Loss**

---

## 1. Problem & Motivation
- In a line of vehicles, a disturbance by the lead vehicle can **amplify** rearward → string
  instability → traffic waves, hard braking, reduced throughput.
- **Adaptive Cruise Control (ACC)** uses only onboard radar (large gaps needed). **Cooperative
  ACC (CACC)** adds **V2V wireless** (the predecessor shares its acceleration/intent), enabling
  **short gaps + string stability** — but wireless **noise, delay, and packet loss** threaten
  stability.
- **Idea:** design a CACC controller, analyze **string stability** (frequency-domain
  attenuation), and show robustness to imperfect V2V (noise, delay, packet loss) vs ACC.

## 2. Objectives
1. Model a platoon of N vehicles (longitudinal dynamics + spacing policy).
2. Implement **ACC baseline** and **CACC** (with V2V feedforward).
3. Analyze **string stability** (transfer-function magnitude ≤ 1) and verify in simulation.
4. Add **V2V noise (base paper), communication delay, and packet loss**; test robustness and
   reproduce the base paper's minimum-headway bound.
5. Quantify minimum stable time gap, throughput, and fuel/energy benefit.

## 3. Base paper + novelty
- **Anchor (IEEE Transactions):** Ma, Pagilla & Darbha, minimum time headway for CAV platoons
  under noisy V2V communication, *IEEE Trans. Intelligent Transportation Systems*, vol. 26,
  no. 1, Jan. 2025 (see `02_IEEE_Base_Paper.md`). Classic foundation: Ploeg et al. 2014.
- **Novelty:** the base paper handles **noise**; we extend the same CTHP + feedforward framework
  to **communication delay and packet loss** (and their combinations), comparing ACC and CACC
  and reporting the **smallest string-stable time gap** (→ road-capacity gain) and a fuel/energy
  estimate — clean, reproducible benchmark.

## 4. System architecture
```
   Lead vehicle accel a0(t)  ──V2V (noise, delay θ, loss)──┐
                                                    ▼  feedforward
   ┌──── vehicle i ───────────────────────────────────────────┐
   │ spacing error e_i = (x_{i-1}-x_i) - (standstill + h·v_i)  │
   │ CACC law: u_i = K·e_i (+ feedforward a_{i-1})             │
   └──────────────┬───────────────────────────────────────────┘
                  ▼  acceleration command
   ┌──────────────────────────────┐
   │ vehicle longitudinal dynamics│ → repeats for i = 1..N
   └──────────────────────────────┘
   String stability:  | Γ(jω) | = |E_i/E_{i-1}| ≤ 1  for all ω
```

## 5. Model (for the report)
- **Per-vehicle model:** double-integrator + actuator lag `τ` (acceleration first-order).
- **Constant-time-headway spacing policy:** desired gap `= r + h·v_i`.
- **String-stability criterion:** the spacing-error transfer function from vehicle *i−1* to *i*
  must satisfy `‖Γ(jω)‖_∞ ≤ 1`. CACC's V2V feedforward is what makes small `h` string-stable.
- **Imperfect V2V:** model acceleration noise (per base paper), latency `θ`, and packet loss;
  study their effect on `Γ`.

## 6. Implementation plan (free tools)
**Stack:** MATLAB/Simulink (transfer functions + platoon model; easy Bode/`Γ(jω)` plots), **or**
Python (`control` + numerical platoon simulation).

| Week | Deliverable |
|------|-------------|
| 1 | Read base paper + thesis; single-vehicle model |
| 2 | ACC baseline + spacing policy |
| 3–4 | CACC with V2V feedforward; string-stability analysis |
| 5 | Add noise/delay/packet loss; robustness study |
| 6 | Platoon sims (N=5–10), cut-in/braking scenarios |
| 7 | Metrics: min gap, throughput, fuel/energy |
| 8 | Paper + slides |

## 7. Experiments & expected results
- **Controllers:** ACC vs CACC (vs human car-following model optional).
- **Scenarios:** leader sinusoidal speed, hard braking, cut-in; delay 0→0.5 s, packet loss,
  accel noise per base paper.
- **Metrics:** string-stability margin `‖Γ‖_∞`, min stable time gap, road capacity, fuel proxy.
- **Headline plot:** spacing-error magnitude along the platoon — grows under ACC, attenuates under CACC.

## 8. Risks & mitigations
| Risk | Mitigation |
|------|-----------|
| Instability with delay | Add delay compensation / reduce feedforward gain; report stable region |
| Model too idealized | Include actuator lag, sensor noise, heterogeneous vehicles |
| Hard to show fuel benefit | Use a simple energy model tied to acceleration variance |
