# Part VII — Cross-paper synthesis, formula sheet, and drills

---

## 7.1 The six papers on one page

| | **Ma 2025** (base) | **Ploeg 2014** | **Köroğlu 2024** | **Zhao 2024** | **Ke 2022** |
|---|---|---|---|---|---|
| **question** | how much headway does channel *noise* cost? | what *is* string stability, and how do you synthesise for it? | how much headway does *delay* cost, at best? | how do you survive an *adversary*? | can a *learned* policy do better? |
| **impairment** | multiplicative `w ∈ [1−1/ρ, 1+1/ρ)` on `a_{i−1}` | delay `θ`, packet loss | delay `δ ∈ [0, δ̄τ]` | FDIA, `≤ F` liars, unbounded | — |
| **topology** | 1-vehicle look-ahead | 1- and 2-vehicle look-ahead | 1-vehicle look-ahead | general graph + pinned leader | 2 vehicles |
| **domain** | continuous, `s` | continuous, `s` | continuous, `s̄ = τs` | discrete, `z` | discrete, MDP |
| **spacing policy** | CTH | CTH | CTH | CTH (telescoped over the graph) | CTH |
| **feedforward** | `k_a a_{i−1}`, `k_a` free | `u_{i−1}`, unity, via `H⁻¹` | `u_{i−1}`, unity, via `ℋ` | `κ_a` acceleration consensus | learned |
| **key object** | `H̃ = Ñ/D` | `Γ = H⁻¹(KG+D)/(1+KG)` | `𝒢 = ℋ(1 − 𝒮₀(1−𝒟))` | `ρ(W(k)) < 1` | `Q(s,a)` |
| **certificate** | `‖H̃‖_∞ ≤ 1` ∀`τ∈(0,τ₀]`, ∀`k̃a∈I` | `‖Γ‖_{H∞} ≤ 1` (L₂) / `‖γ‖_{L₁} ≤ 1` (L∞) | `‖𝒢‖_∞ ≤ 1` ∀`δ∈[0,δ̄τ]` | asymptotic stability under attack | none |
| **headline** | `h_lb`, `k_a*`, `h*` | `Γ = 1/H` when comms are ideal | `h̄_o ≈ 2√δ̄` | `F+1`-robust graph + MSR filter | −44.7 % then −40.2 % gap |
| **string stability proved?** | **yes** (mean sense) | **yes** | **yes** | **no — open problem** | **no — not addressed** |

---

## 7.2 The one story that connects them

**Start from the ideal.** Ploeg (Part III §3.4): with a perfect radio and unity
feedforward through the `H⁻¹` precompensator,

```
    Γ(s) = 1/H(s) = 1/(hs + 1)   ⟹   ‖Γ‖_{H∞} = ‖γ‖_{L₁} = 1
```

**for any gains and any `h > 0`.** So a CACC platoon has *no intrinsic
minimum headway*. Every minimum-headway theorem in the literature is a
statement about how much of that ideal a specific imperfection destroys.

Then each paper removes one assumption:

```
                                 Γ = 1/H            h → 0⁺
                                     │
        ┌────────────────────────────┼────────────────────────────┐
        │                            │                            │
  remove: perfect               remove: perfect            remove: honest
   AMPLITUDE                        TIMING                     SENDERS
        │                            │                            │
   Ma 2025                     Köroğlu 2024                  Zhao 2024
   w ∈ [1∓1/ρ]                 𝒟 = e^{−δs}                   ≤ F liars
        │                            │                            │
  h > 2τ₀(1−(1−1/ρ)k_a)        h̄ > √(2δ̄(√(1+0.5δ̄)+1))     F+1-robust graph
        ────────────────            ≈ 2√δ̄                     + MSR filter
        1−(1+1/ρ)²k_a²
        │                            │                            │
   also removes the            keeps the H⁻¹              abandons the
   H⁻¹ precompensator          precompensator             chain topology
   ⟹ floor is τ₀ even          ⟹ floor is 0 as δ̄→0        ⟹ frequency
     with ρ → ∞                                             domain unusable
```

and Ploeg's own Chapter 4 removes **reliable delivery** (packet loss), landing
between CACC and ACC: `h ≥ 0.25 s → 1.23 s → 3.16 s`.

Ke sits outside the diagram: it removes the *model* rather than an assumption
about the channel, and consequently has no certificate at all.

---

## 7.3 Every minimum-headway result, in one table

Normalise everything by `τ` (`h̄ = h/τ`) so they are comparable:

| source | condition | `h̄` floor | at `τ = 0.5 s` |
|---|---|---|---|
| Ploeg, ideal comms | — | **0⁺** | 0⁺ |
| Köroğlu, `δ̄ = 0.1` | `h̄ > √(2δ̄(√(1+0.5δ̄)+1))` | 0.636 | 0.318 s |
| Köroğlu, `δ̄ = 0.2` | " | 0.905 | 0.453 s |
| Köroğlu, `δ̄ = 0.9` | " (`= h̄` of PD-only) | 1.992 | 0.996 s |
| Ma, `ρ → ∞`, `k_a → 1` | `h̄ > 2/(1+k_a)` | **1.0** | 0.500 s |
| Ma, `ρ = 5`, `k_a = k_a*` | `h̄ > (1+1/√ρ)²/(1+1/ρ)` | **1.745** | 0.873 s |
| Ma, `ρ = 5`, `k_a = 0.5` | `h̄ > h_lb/τ₀` | **1.875** | 0.938 s |
| Ma, `k_a → 0` (≈ACC) | `h̄ > 2` | 2.0 | 1.000 s |
| Ploeg experimental, CACC | measured, `θ ≈ 0.15 s`, `τ = 0.1 s` | 7.0 | (h = 0.7 s) |
| Ploeg, dCACC (est. feedforward) | `h ≥ 1.23 s`, `τ = 0.1 s` | 12.3 | — |
| Ploeg, ACC | `h ≥ 3.16 s`, `τ = 0.1 s` | 31.6 | — |

**Three things to read off this table:**

1. **`h̄ = 1` and `h̄ = 2` are the two landmarks.** `h̄ = 2` is what
   feedback alone achieves (ACC / `k_a → 0` / Köroğlu's PD-only reference);
   `h̄ = 1` is Ma's perfect-feedforward floor. **The whole value of V2V, in
   Ma's architecture, is the factor of 2 between them** — and channel noise
   eats into it (1.875 of the available 2.0 at `ρ = 5, k_a = 0.5`; i.e. noise
   at 14 dB SNR consumes **87 % of CACC's benefit**).
2. **Ploeg's experimental `h̄ = 7` looks terrible until you notice `τ = 0.1 s`.**
   Their bottleneck is `θ/τ = 1.5`, i.e. delay *fifteen times* the lag — deep in
   the regime where Köroğlu's bound says communication has stopped paying. In
   *absolute* terms, 0.7 s is better than Ma's 0.95 s. **Always normalise
   before comparing.**
3. **Ma's floor never goes below `h̄ = 1`; Köroğlu's goes to 0.** That is the
   `H⁻¹`-precompensator/unity-feedforward structural difference (Part III
   §3.5), not a difference in how good the channel is.

---

## 7.4 Master formula sheet

### Plant and policy (all papers)

```
    ẋ = v ,  v̇ = a ,  τ ȧ + a = u          G(s) = 1/(s²(τs+1))
    d_des = r + h·v                        H(s) = h s + 1  (Köroğlu: ℋ = 1/H)
    e_i = d_i − d_des    (Ploeg)           δ_i = e_i + h v_i  (Ma, sign-flipped)
    ė_i = (v_{i−1} − v_i) − h a_i          ← the CTH damping term
```

### Ma 2025

```
    w = (1 − 1/ρ) + (1/ρ)Σ_{j=0}^{15} z_j 2^{−j} ,  z_j ~ Bern(γ_j)
    E[w] = (1 − 1/ρ) + (1/ρ)Σ γ_j 2^{−j}                  = 1.04817 (ρ=5)
    k̃a = k_a E[w] ,   I = [(1−1/ρ)k_a, (1+1/ρ)k_a]        = [0.4, 0.6]
    u_i = k_a w a_{i−1} − k_v(v_i − v_{i−1}) − k_p δ_i

    H̃(s;τ) = (k̃a s² + k_v s + k_p)/(τs³ + s² + γs + k_p) ,  γ = k_v + h_w k_p

    |D|² − |Ñ|² = ω²[τ²ω⁴ + (1 − k̃a² − 2τγ)ω² + γ² − 2k_p − k_v² + 2k̃a k_p]

    (16)  0 < k_a < 1/(1 + 1/ρ)                              = 0.8333
    (28a) γ ≤ [1 − (1+1/ρ)²k_a²]/(2τ₀)          ← HIGH end, h-INDEPENDENT
    (28b) γ ≥ √(2k_p[1 − (1−1/ρ)k_a] + k_v²)    ← LOW end   (usually binds)
    (RH)  γ > τ₀ k_p
    (33)  2h_w k_v + h_w²k_p ≥ 2[1 − (1−1/ρ)k_a]     ← (28b) linearised

    (17)  h_w,lb = 2τ₀[1 − (1−1/ρ)k_a]/[1 − (1+1/ρ)²k_a²]      = 0.9375 s
    (18)  k_a*   = [(1−1/√ρ)/(1+1/√ρ)]·1/(1+1/ρ)               = 0.3183
    (19)  h*_w,lb = τ₀(1+1/√ρ)²/(1+1/ρ)                        = 0.8727 s
    (15)  Thm 1:  h_w > 2τ₀/(1 + k̃a)      ρ→∞:  h_lb → 2τ₀/(1+k_a)

    a₁/a₂ = h_w/h_w,lb        ← non-emptiness of S = S₁∩S₂ ⟺ h_w > h_lb
    a₁ = (1−n k_a²)/(2τ₀) , b₁ = a₁/h_w   |  a₂ = (1−m k_a)/h_w , b₂ = 2a₂/h_w
                                            m = 1−1/ρ , n = (1+1/ρ)²
```

### Ploeg 2014

```
    Γ(s) = (1/H(s)) · (K(s)G(s) + D(s))/(1 + K(s)G(s))      K = k_p+k_d s+k_dd s²
    Γ_ACC   = (1/H)·KG/(1+KG)                 D = 0
    Γ_dCACC = (1/H)·G(K + s²T_aa)/(1 + GK)    estimated feedforward
    D = 1  ⟹  Γ = 1/H ,  ‖Γ‖_{H∞} = 1 , γ(t) = h⁻¹e^{−t/h} , ‖γ‖_{L₁} = 1

    L₂:  ‖P₁‖_{H∞} < ∞  and  ‖Γ_i‖_{H∞} ≤ 1        (Thm 1)
    L∞:  ‖p₁‖_{L₁} < ∞  and  ‖γ_i‖_{L₁} ≤ 1        (Thm 2)   ‖γ‖_{L₁} ≥ ‖Γ‖_{H∞}
    Γ(0) = 1  always  ⟹  ‖Γ‖_{H∞} ≥ 1

    H∞ synthesis:  S̃ = (1+K_fb G)⁻¹
                   S = S̃G(1 − K_ff D) ,  Γ = S̃H⁻¹(K_fb G + K_ff D)
                   objective  ‖N‖_{H∞} = 1,  N = (W_e S ; Γ)
    Singer model:  ȧ = −αa + u ,  σ_a² = (a_max²/3)(1+4P_max−P₀) ,  C_uu = 2ασ_a²δ
```

### Köroğlu 2024

```
    s̄ = τs ,  h̄ = h/τ ,  ϖ = τω ,  δ ∈ [0, δ̄τ] ,  ψ := κφ
    𝒮₀ = s̄²(s̄+1)/𝒬₀ ,  𝒬₀ = s̄³ + s̄² + κφ s̄ + κ ,  𝒢 = ℋ[1 − 𝒮₀(1−𝒟)]
    |𝒬₀(jϖ)|² = ϖ⁶ + (1−2κφ)ϖ⁴ + κ(κφ²−2)ϖ² + κ²
    RH:  φ > 1 , κ > 0        string-stab necessary:  2/φ < ψ < 1/2 ⟹ φ > 4
    2(1−cos θ) ≤ θ² ,  θ sin θ ≤ θ²

    h̄ > h̄_o = √( 2δ̄(√(1 + 0.5δ̄) + 1) )  ≈ 2√δ̄
    ψ = h̄²/(2h̄² + δ̄²) ,  φ = (2h̄²+δ̄²)(2h̄²−2δ̄+δ̄²)/(h̄⁴ − 4δ̄h̄² − 2δ̄³) ,  κ = ψ/φ
```

### Zhao 2024

```
    A = [1 ξ ξ²/2; 0 1 ξ; 0 0 1−ξ/τ] ,  B = [0;0;ξ/τ]      (needs 0<ξ<2τ)
    ℒ = 𝒟 − 𝒜 ,  ℒ_P = ℒ + P     (P = pinning matrix to the leader)
    F-total:  |M| ≤ F            graph must be (F+1)-robust
    MSR:  discard the F largest ‖s̃_{i,j}‖  ⟹  ‖δ_{i,j}‖ ≤ max_j‖x_i − x_j‖
    x(k+1) = W(k)x(k) + C(k)δ(k) ,   stability iff  max_i|λ_i(W)| < 1
    ‖W'(k)‖ = (ξ/τ)√(κ_q²+κ_v²+κ_a²) ρ(ℒ'_P(k))
    convergence if  φ·exp((ϕξ)/(φτ)√(F̄(κ_q²+κ_v²+κ_a²))) < 1
    Jury (cubic λ³+a₂λ²+a₁λ+a₀):  P(1)>0, −P(−1)>0, |a₀|<1, |a₀²−1|>|a₀a₂−a₁|
```

### Ke 2022

```
    V^π(s) = R(s,π(s)) + γΣ_{s'}T(s,π(s),s')V^π(s')
    Q*(s,a) = R(s,a) + γΣ_{s'}P(s'|s,a) max_b Q*(s',b)
    ‖BV₁ − BV₂‖_∞ ≤ γ‖V₁ − V₂‖_∞          ⟹ Banach ⟹ V_k → V* at rate γ^k
    TD:      V ← V + α[r + γV(s') − V]          δ_t = r + γV(s_{t+1}) − V(s_t)
    SARSA:   Q ← Q + α[r + γQ(s',a') − Q]                    (on-policy)
    Q-learn: Q ← Q + α[r + γ max_{a'}Q(s',a') − Q]           (off-policy)
    Robbins–Monro:  Σα_t = ∞ ,  Σα_t² < ∞
    max bias:  E[max_a Q̂] ≥ max_a E[Q̂]     ⟹ double Q: select with Q₁, eval with Q₂
    backprop:  δ^{(n_l)} = −(y−a^{(n_l)})⊙f'(z) ,  δ^{(l)} = (W^{(l)ᵀ}δ^{(l+1)})⊙f'(z^{(l)})
    DQN:  experience replay (decorrelate) + fixed target w⁻ (stationarity)
    DDQN: y = r + γQ̂(s', argmax_{a'}Q̂(s',a';w); w⁻)
```

---

## 7.5 The twelve numbers to have memorised

| number | what it is |
|---|---|
| **0.9375 s** | `h_w,lb(k_a = 0.5, ρ = 5, τ₀ = 0.5)` — Ma eq. (45) |
| **0.3183** | `k_a*(ρ = 5)` — Ma eq. (18) |
| **0.8727 s** | `h*_w,lb(ρ = 5, τ₀ = 0.5)` — Ma eq. (19) |
| **0.8333** | `1/(1 + 1/ρ)`, the cap on `k_a` at `ρ = 5` — Ma eq. (44) |
| **1.0482** | `E[w]` with Table I at `ρ = 5` ⟹ `k̃a = 0.5241` |
| **[0.4, 0.6]** | the interval `I` for `k_a = 0.5, ρ = 5` |
| **13.98 dB** | `ρ = 5` expressed as SNR |
| **1.0035** | `‖H̃‖_∞` for Ma's Case 2 — the string-*unstable* one, at the **low** end |
| **345 / 324 m** | 12-vehicle platoon length at `h = 0.95 / 0.88 s`, `v = 25 m/s` |
| **2τ₀ → τ₀** | ACC floor → perfect-feedforward floor; CACC's factor of 2 |
| **≈2√δ̄** | Köroğlu's minimum normalised headway vs normalised delay |
| **0.25 / 1.23 / 3.16 s** | Ploeg's minimum `h` for CACC / dCACC / ACC |

---

## 7.6 Question bank

### Tier 1 — definitions and statements

1. Define string stability. Which definition, and why that one?
2. What is the constant time headway policy, and what does `h` mean physically?
3. Why does CTH enable string stability where constant spacing cannot?
4. Write the vehicle model and justify the first-order lag.
5. What is `ρ`? Convert to dB. Why is it a *minimum* over time, not an average?
6. State Ma's Theorem 2 in full.
7. What does `‖H̃‖_{H∞} ≤ 1` mean in terms of signal norms?
8. Why is `‖Γ(j0)| = 1` for every one of these systems?

### Tier 2 — derivations (all worked in Parts I–VI)

9. Derive `H̃(s) = Ñ(s)/D(s)` from Ma's eq. (13).
10. Show that the *position* and the *spacing-error* propagation share one
    transfer function, and say what breaks it.
11. Expand `|D(jω)|² − |Ñ(jω)|²` and obtain eq. (25).
12. Derive `h_w > 2τ₀/(1 + k̃a)`.
13. Derive `h_w,lb`, and say which end of `I` each of (28a), (28b) binds at.
14. Derive `k_a*` and `h*_w,lb`.
15. Show `a₁/a₂ = h_w/h_w,lb`.
16. Derive Routh–Hurwitz for a cubic and apply it to `D(s)`.
17. Derive Ploeg's `Γ = (1/H)(KG+D)/(1+KG)` from the block diagram.
18. Show `Γ = 1/H` with ideal comms; compute `‖γ‖_{L₁}`.
19. Prove `‖p‖_{L₁} ≥ ‖P‖_{H∞}`.
20. Derive Köroğlu's `h̄_o`, including the trigonometric bounding step.
21. Prove Zhao's eq. (18) (the MSR bound).
22. Prove the Bellman operator is a γ-contraction.
23. Prove `E[max_a Q̂] ≥ max_a E[Q̂]`.
24. Derive `σ_a² = (a_max²/3)(1 + 4P_max − P₀)` from Ploeg's `p(a)`.

### Tier 3 — the questions that separate understanding from memorisation

25. **Why is the LOW end of the noise interval the binding case?**
    *Because (28b) — the low-frequency condition — is hardest when `k̃a` is
    smallest: less feedforward pushes the tracking burden onto the feedback
    loop, which then needs a larger `γ`. Measured: Case 2 has `‖H̃‖_∞ = 1.0035`
    at `k̃a = 0.4` and exactly `1.0000` at `k̃a = 0.6`.*

26. **Why is (28a) independent of `h_w`, and what does that imply?**
    *Because it comes from the `ω⁴` coefficient `1 − k̃a² − 2τγ ≥ 0`, and `h_w`
    enters only through `γ`, which is being bounded — the ceiling on `γ` itself
    contains no `h_w`. Implication: a platoon whose channel degrades cannot
    always be rescued by lengthening the headway; once `γ` exceeds the ceiling
    for the new `ρ`, only re-tuning `k_v` restores feasibility. This is the
    "fixed-gain wall" (project D-017).*

27. **Ma's conditions are sufficient, not necessary. Show it.**
    *The split (26) demands each power of `ω²` in the quartic be non-negative
    separately, discarding the possibility that the positive `τ²ω⁶` term offsets
    a slightly negative `ω⁴` term. Counterexample found by search:
    `k_a=0.5, k_v=0.70, k_p=0.009, h_w=1.50, ρ=5` fails (28a) but has
    `‖H̃‖_∞ = 1.000000` over the whole interval.*

28. **Why does Ma need `τ → 0⁺` to prove `k̃a < 1`?**
    *Because `lim_{ω→∞}|H̃| = k̃a` only when `τ = 0`; with `τ > 0` the `τs³`
    term makes `|H̃| → 0` at high frequency. The constraint is imposed by the
    *fastest* admissible vehicle, which has no roll-off to hide behind.*

29. **Why does Ploeg get `h → 0` with ideal comms while Ma's floor is `τ₀`?**
    *Structural, not channel-related. Ploeg feeds forward the predecessor's
    **command** `u_{i−1}` through an `H⁻¹` precompensator, which makes the
    feedforward an exact model-inverse of the feedback path (`KG + 1` cancels
    `1 + KG`). Ma feeds forward the predecessor's **realised acceleration**,
    already low-passed by that vehicle's lag, with no precompensator. Check
    algebraically: `H̃ = 1/H` needs `h_w = τ` **and** `k_v = 0`.*

30. **Why is the reproduction of Fig. 7 done with `w = E[w]` rather than the
    realised channel?**
    *Remark 5: the analysis is on the equivalent deterministic system (13). The
    certified per-hop attenuation is ≈0.35 %, while a single stochastic
    realisation jitters `max|δ_i|` by ≈2 % — six times larger. The monotone
    profiles the paper prints are a property of the mean system. Measured both
    ways in `matlab/run_all.m`.*

31. **What licenses replacing `E[e^{Ât}]` by `e^{Āt}`?**
    *The block lower-bidiagonal structure. Every path in `Â^k` descends
    monotonically, so it uses each link at most once; the monomials are
    products of **distinct** independent `w`'s, and expectation factorises. It
    would fail for bidirectional or multi-predecessor topologies, where a path
    can revisit a link and `E[w²] ≠ (E[w])²`.*

32. **Köroğlu's bound is sub-linear in delay (`≈2√δ̄`); this project's D-008
    finds headway *exploding* with delay. Contradiction?**
    *No. Köroğlu **re-tunes** `κ, φ` for each `δ̄` (eqs. 33–34); D-008 holds
    Ma's Case-A gains **fixed** and asks only for headway. Same lesson as
    Ma's §2.6.2 — closed-form minima assume retuning, and a fielded platoon
    with frozen gains hits a wall much sooner. Always state whether the gains
    move.*

33. **Why can't you use `‖Γ‖_{H∞}` on Zhao's system?**
    *The MSR filter is nonlinear (it sorts and discards) and the effective
    topology is time-varying, so there is no LTI transfer function. Zhao say so
    explicitly and fall back on the spectral radius of `W(k)` plus
    Grönwall–Bellman.*

34. **Zhao's MSR filter needs `m > F` neighbours. What do you do in a
    predecessor-following chain, where `m = 1`?**
    *MSR is unavailable — discarding your one neighbour leaves nothing. You
    need a second, independent source of the same quantity. This project's
    D-022 uses the **radar**: compare the V2V-claimed acceleration against what
    the predecessor is physically observed to be doing, and de-rate the
    feedforward gain by a continuous trust weight `g ∈ [0,1]`. Redundancy
    across *sensors* replaces redundancy across *neighbours*.*

35. **Ma bounds the noise magnitude and allows it on every link; Zhao bounds
    the number of liars and allows unbounded magnitude. Which threat model is
    right?**
    *Different adversaries. Ma's is nature — thermal noise, quantisation,
    fading — which is ubiquitous but small. Zhao's is an intelligent attacker —
    rare but arbitrary. A deployed system needs both: a headway margin sized by
    `ρ` for the everyday case, and a filter/gate for the adversarial one.*

36. **When would you use RL for CACC?**
    *To choose among certified designs, not to replace certification. The
    learned policy has no `Γ(s)`, so `‖Γ‖_{H∞} ≤ 1` cannot even be stated, and
    Ke only ever tests two vehicles. The defensible architecture is a
    model-based backbone with proven `‖Γ‖_{H∞} ≤ 1` and learning tuning the
    free parameters — with every configuration traversed itself certified
    (this project's D-016/D-017/D-018 quasi-static argument).*

37. **Your platoon shows `‖H̃‖_∞ = 1.0035`. The 200 s simulation looks fine.
    Is it safe?**
    *No. Per-hop growth compounds geometrically: `1.0035^11 = 1.039` over
    11 hops (invisible), but `1.0035^50 = 1.19` and `1.0035^200 = 2.0`. The
    verdict lives in the frequency domain; time series only illustrate
    (project D-010). Also check which `k̃a` in `I` you evaluated at — 1.0035 is
    the low end, and the same design reads exactly 1.0000 at the high end.*

38. **What is the single strongest evidence that a reproduction is correct?**
    *Reproducing a result whose **sign** you could not have guessed. Ma's Case 2
    has `max|δ_i|` **increasing** 0.882 → 0.886 down the string — 0.5 % growth
    over 11 hops. The Simulink model gives 0.8821 → 0.8872. Matching both the
    magnitude and the direction of a sub-percent trend requires the transfer
    function, the manoeuvre, the initial conditions and the integrator all to be
    right simultaneously.*

---

## 7.7 Ten whiteboard drills, blank paper, no notes

Time yourself. Each should take 5–15 minutes.

1. Write the plant, the CTH policy, and `ė_i`; explain the damping term.
2. Derive `H̃ = Ñ/D` and show `Δ_i/Δ_{i−1} = X_i/X_{i−1}`.
3. Expand `|D|² − |Ñ|²`; get eq. (25); split into (26a)/(26b); state which end
   of `I` binds each.
4. Derive `h_w > 2τ₀/(1+k̃a)`, then `h_w,lb`, showing the `k_v²` cancellation.
5. Differentiate `h̄_w(k_a)`, find `r₁`, simplify to `k_a*`, evaluate to `h*`.
6. Draw Ploeg's block diagram; derive `Γ`; set `D = 1` and get `1/H`; compute
   `‖γ‖_{L₁}`.
7. State Ploeg's Definition 1 and prove Theorem 1.
8. Derive Köroğlu's `𝒮₀`, `𝒬₀`, `|𝒬₀(jϖ)|²`, and the `φ > 1` condition.
9. Balance Köroğlu's (30) and (31), take `φ → ∞`, rationalise to `h̄_o ≈ 2√δ̄`.
10. State Zhao's F-total model, Algorithm 1, and prove the MSR bound (18).

---

## 7.8 Where the code lives

| you want to | run |
|---|---|
| verify every closed form numerically | `matlab/scripts/theorem_checks.m` |
| regenerate Ma's Figs. 4–15 | `matlab/run_all.m` |
| see the block diagram | `addpath matlab/models; open_system('cacc_platoon')` |
| check the Simulink model against the Python core | `matlab/tests/crosscheck_python.py` |
| evaluate `‖H̃‖_∞` at arbitrary parameters | `ma2025.hinf_worst(ka, rho, kv, kp, hw, tau)` |
| audit a design against all of Theorem 2 | `ma2025.check_gains(ka, rho, kv, kp, hw, tau0)` |
| the delay extension (D-008) | `python scripts/reproduce_base_paper.py` |
| the QoS-adaptive pipeline (D-016/17) | `python scripts/qos_adaptive_study.py` |

---

*Back to [Part I — Foundations](01-foundations.md) · [index](00-INDEX.md)*
