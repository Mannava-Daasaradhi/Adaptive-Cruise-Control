# Part III — Ploeg: L_p string stability, H∞ synthesis, graceful degradation

Two documents, one body of work:

> J. Ploeg, N. van de Wouw, H. Nijmeijer, "L_p String Stability of Cascaded
> Systems: Application to Vehicle Platooning," *IEEE Trans. Control Syst.
> Technol.* **22**(2):786–793, 2014. doi 10.1109/TCST.2013.2258346
>
> J. Ploeg, *Controllers for Cooperative and Automated Driving*, PhD thesis,
> TU Eindhoven, 2014. — Ch. 2 = the paper above; **Ch. 3** = H∞ controller
> synthesis; **Ch. 4** = graceful degradation under packet loss.

**Why this matters for the base paper.** Ma et al. *use* string stability;
Ploeg *defines* it. Ma's Definition 2 is a specialisation of Ploeg's
Definition 1, and Ma's `‖H̃(jω)‖∞ ≤ 1` test is licensed by Ploeg's Theorem 1.
If you are asked "what does string stability actually mean, rigorously?", the
answer lives here, not in Ma.

---

## 3.1 The four schools of string stability — and why a fifth was needed

Ploeg §II reviews the field. Know the taxonomy; it is a standard opening
question.

| school | idea | fatal limitation |
|---|---|---|
| **Lyapunov** (Swaroop–Hedrick 1996; Sheikholeslam–Desoer) | asymptotic stability of the interconnected system w.r.t. **initial-condition** perturbations | ignores **external disturbances** — but lead-vehicle velocity variation is the disturbance that matters in practice |
| **single-vehicle IC perturbation** (Klinge–Middleton) | response to an IC perturbation of *one* vehicle | ignores ICs of the others *and* external disturbances |
| **infinite-length strings** (Melzer–Kuo; Chu; Bamieh; Curtain) | bilateral Z-transform over the vehicle index; assess eigenvalues in the "discrete spatial frequency" domain | properties of infinite strings need not converge to those of finite ones — a real platoon has a **first and a last vehicle** whose dynamics differ |
| **performance-oriented** (Naus; Rajamani–Zhu; Xiao–Gao) | `sup_ω |Γ_i(jω)| ≤ 1` | only linear systems, usually zero ICs; a *criterion*, not a stability *property* |

Ploeg's **Definition 1** (F-22) unifies them: nonlinear, admits both external
input `u_r` and IC perturbation `x(0)`, quantifies over **all string lengths
m ∈ ℕ**, and yields the performance criterion as a *theorem* rather than a
postulate.

**The "∀ m ∈ ℕ" quantifier is the load-bearing element.** Without it the
definition collapses to ordinary input–output stability of one particular
platoon. With it, string stability becomes a **scalability** statement.

---

## 3.2 The platoon model  (Lp paper eqs. 2–10)

### Plant and policy

```
    ḋ_i = v_{i−1} − v_i
    v̇_i = a_i                                                          (4)
    ȧ_i = −a_i/τ + u_i/τ
```

```
    d_r,i(t) = r_i + h v_i(t)                                           (2)
    e_i(t)   = d_i − d_r,i = (q_{i−1} − q_i − L_i) − (r_i + h v_i)      (3)
```

(F-4, F-6; `q_i` = rear-bumper position, `L_i` = length.)

### The controller — and the piece Ma does not have

```
    h u̇_i = −u_i + ξ_i                                                 (5)
    ξ_i   = K (e_i, ė_i, ë_i)ᵀ + u_{i−1} ,      K = (k_p  k_d  k_dd)    (6)
```

In the Laplace domain, (5) is `H(s) u_i = ξ_i` with `H(s) = hs + 1`, i.e.

```
    u_i(s) = H⁻¹(s) [ K(s) e_i(s) + D(s) u_{i−1}(s) ] ,  K(s) = k_p + k_d s + k_dd s²
```

Two structural features to notice, because they are exactly what differs from
Ma's law (6) and they explain everything downstream:

1. **A precompensator `H⁻¹(s)` sits in front of the controller output.** Ploeg
   (thesis, below eq. 3.7): it "cancels the spacing policy transfer function
   `H(s)`, which is located in the feedback loop, such that the driver can
   select any time gap `h` without compromising individual vehicle stability."
2. **The feedforward is the predecessor's *command* `u_{i−1}`, with unity
   gain** — not its measured acceleration, and not scaled by a tunable gain.

Ma's law has neither: no `H⁻¹`, and the feedforward is `k_a · a_{i−1}` with
`k_a` a free design parameter. §3.5 shows precisely what that costs.

### The virtual reference vehicle  (eqs. 9, 10)

The first vehicle has no predecessor, so it cannot use (6). Ploeg gives it a
*virtual* one (index 0) with dynamics

```
    ė₀ = 0 ,  v̇₀ = a₀ ,  ȧ₀ = −a₀/τ + u₀/τ ,  u̇₀ = −u₀/h + ξ₀/h        (9)
```

so that vehicle 1 runs the **same** controller as everyone else. `e₀(t) ≡ e₀(0)`
is a dummy state with no influence (the first column of `A_r` and `A₁` is
zero), and `e₀(0) = 0` is chosen.

Why bother: it makes the platoon **homogeneous including the head**, so `Γ(s)`
is genuinely index-independent, and `Γ(s)` then also equals the transfer
function from `â₀` to `â₁`. Without it the first hop is a special case and the
cascade argument loses its clean product form.

The equilibrium `x̄₀ = (0, v̄₀, 0, 0)ᵀ` is only **marginally** stable (the
virtual reference is uncontrolled — it has the free integrator that sets the
platoon speed). Hence the standing assumption (Assumption 3.1) that *unstable
and marginally stable modes are unobservable in the chosen output* — otherwise
`‖P₁‖_{H∞}` would not exist.

---

## 3.3 Definition 1 and its two theorems

### Definition (F-22, restated for reference)

```
    L_p string stable:
        ‖y_i(t) − h(x̄₀)‖_{L_p} ≤ α(‖u_r(t)‖_{L_p}) + β(‖x(0) − x̄‖)
                                              ∀ i ∈ S_m , ∀ m ∈ ℕ     (Def 1)
    strictly L_p string stable: additionally, with x(0) = x̄,
        ‖y_i(t) − h(x̄₀)‖_{L_p} ≤ ‖y_{i−1}(t) − h(x̄₀)‖_{L_p}
```

`i = 1` is excluded from the strict inequality because the virtual reference
vehicle has no associated output.

### Theorem 1 (L₂) — statement and proof

> Let (13),(14) be a linear unidirectionally interconnected system with
> input–output behaviour (15). Assume `(C_i, A)` is such that unstable and
> marginally stable modes are unobservable and `P_i(s)` is square and
> nonsingular ∀ i. Then the system is **L₂ string stable if**
> 1. `‖P₁(jω)‖_{H∞}` exists;
> 2. `‖Γ_i(jω)‖_{H∞} ≤ 1 ∀ i ∈ ℕ\{1}`,
>
> with `Γ_i(s) := P_i(s)P_{i−1}⁻¹(s)` (eq. 19). Moreover the system is
> **strictly** L₂ string stable **iff** 1 and 2 hold.

**Proof.** Factor the cascade (eq. 20):

```
    ŷ_i(s) = P_i(s)û_r(s) = ( ∏_{k=2}^{i} Γ_k(s) ) P₁(s) û_r(s)
```

Submultiplicativity of the H∞ norm (F-12) gives

```
    ‖P_i‖_{H∞} ≤ ( ∏_{k=2}^{i} ‖Γ_k‖_{H∞} ) ‖P₁‖_{H∞}                  (21)
```

Under conditions 1 and 2 the product is `≤ 1`, so `sup_{i∈ℕ}‖P_i‖_{H∞}` exists
— which by Definition 1 and Remark 2 (plus F-9, the induced-gain theorem) is
L₂ string stability, with the class-𝒦 function chosen as
`α(‖u_r‖_{L₂}) = (sup_i‖P_i‖_{H∞})‖u_r‖_{L₂}` (eq. 17). Because α is linear,
this is *finite-gain* L₂ string stability.

For strictness, condition 2 with `y_i = Γ_i y_{i−1}` and F-9 gives
`‖y_i‖_{L₂} ≤ ‖y_{i−1}‖_{L₂}` directly (eq. 22). Necessity of 1 and 2 for the
strict version is immediate: if `‖Γ_i‖_{H∞} > 1` there exists an input making
`‖y_i‖_{L₂} > ‖y_{i−1}‖_{L₂}` (F-9's non-conservatism). ∎

**Why the theorem is only *sufficient* for non-strict L₂ string stability:**
purely because of (21) — the peaks of the `Γ_k` need not align (F-12).

**When it becomes necessary and sufficient (Remark 4).** If `u_r, y_i` are
scalar and `Γ` is index-independent (homogeneous string), then

```
    ‖P_i(jω)‖_{H∞} = sup_ω { |Γ(jω)|^{i−1} |P₁(jω)| }                  (23)
```

and `sup_{i∈ℕ}` of that is finite **iff** `|P₁(jω)| < ∞` and `|Γ(jω)| ≤ 1`
∀ω. So for the homogeneous scalar case — which covers Ma's entire paper —
**L₂ string stability and strict L₂ string stability are equivalent**, and
`‖Γ‖_{H∞} ≤ 1` is exactly right.

The one caveat Ploeg flags: necessity holds "only in the absence of poles of
`P₁(s)` on the imaginary axis, being cancelled by zeros of `Γ(s)`", since then
`sup_ω|P₁|` is unbounded while `sup_ω|Γ^{i−1}P₁|` may not be.

### Theorem 2 (L∞)

Same structure with `‖·‖_{H∞}` replaced by `‖·‖_{L₁}` of the impulse response
(F-10):

> L∞ string stable if 1. `‖p₁(t)‖_{L₁}` exists; 2. `‖γ_i(t)‖_{L₁} ≤ 1`
> ∀ i ∈ ℕ\{1}. Strictly L∞ string stable iff both hold.

Proof by Young's inequality for convolutions, mirroring Theorem 1.

**Why L∞ matters and why nobody uses it.** L∞ is the *safety* norm — peak
spacing error is what causes collisions, whereas L₂ is an energy statement.
But (F-11) `‖γ‖_{L₁} ≥ ‖Γ‖_{H∞}`, so L∞ string stability is strictly harder
and demands "a significantly larger time headway." Ploeg also reports a
practical obstacle in his experiments: the impulse response computed by inverse
Fourier transform from measured data is too inaccurate to test `‖γ‖_{L₁} ≤ 1`,
so "L∞ string stability is not further investigated here." **Every quantitative
string-stability claim in these six papers is an L₂ claim.**

---

## 3.4 Γ(s) for CACC — the central transfer function  (eq. 30)

### Derivation (five lines; do it from the block diagram, Fig. 2)

Ingredients: `G(s) = 1/(s²(τs+1))` (F-2), `H(s) = hs + 1` (F-5),
`K(s) = k_p + k_d s + k_dd s²`, `D(s) = e^{−θs}`, and

```
    e_i = q_{i−1} − H q_i ,       q_i = G u_i ,
    H u_i = K e_i + D u_{i−1}
```

Substitute `e_i` and `q_i`:

```
    H u_i = K (G u_{i−1} − H G u_i) + D u_{i−1}
    H u_i + K H G u_i = (K G + D) u_{i−1}
    H (1 + K G) u_i = (K G + D) u_{i−1}
```

```
                 u_i          1     K(s)G(s) + D(s)
    Γ(s)  =  ───────  =  ────── · ─────────────────                   (30)
                u_{i−1}      H(s)     1 + K(s)G(s)
```

Because `q_i = G u_i` and `a_i = s² q_i` with the same `G` at every index
(homogeneity), the *same* Γ transfers `q_{i−1}→q_i`, `v_{i−1}→v_i`,
`a_{i−1}→a_i` and `e_{i−1}→e_i`. Choosing the output is therefore a matter of
which `P₁(s)` exists (Theorem 1, condition 1), not of which Γ you get.
Ploeg picks `y_i = a_i`, giving `P₁(s) = 1/(τs+1) · e^{−φs}` — bounded, so
`‖P₁‖_{H∞} = 1`. Picking `v_i` would give `P₁ = 1/(s(τs+1))`, unbounded at
DC, and condition 1 would fail.

### ACC as the special case D = 0

```
    Γ_ACC(s) = (1/H(s)) · K(s)G(s)/(1 + K(s)G(s))                    (4.17)
```

> **Cross-check with this project's code.** `src/cacc/analysis.py` implements
> CACC exactly as (30) but ACC as `Γ = GC/(1 + GCH)`. These are *different
> controller architectures*, not a discrepancy: Ploeg's ACC retains the `H⁻¹`
> precompensator (`u_i = H⁻¹Ke_i`), the project's ACC is the plain PD law
> (`u_i = C e_i`, no precompensator). Both appear in the literature; state
> which one you mean.

### The headline result: with perfect communication, h is free

Set `D(s) = 1` (no delay, no loss). Then the numerator becomes `KG + 1`, which
is exactly the denominator, and

```
    Γ(s) = (1/H(s)) · (1 + KG)/(1 + KG) = 1/H(s) = 1/(hs + 1)
    ⟹  ‖Γ‖_{H∞} = sup_ω 1/√(1 + h²ω²) = 1     (attained at ω = 0)
```

**So a CACC platoon with ideal V2V is strictly L₂ string stable for *any*
controller gains and *any* time headway h > 0.** No design constraint at all.

And in L∞: `γ(t) = ℒ⁻¹{1/(hs+1)} = h⁻¹e^{−t/h}`, so

```
    ‖γ‖_{L₁} = ∫₀^∞ h⁻¹ e^{−t/h} dt = 1
```

— L∞ string stability too, again for all gains and all h. This is Ploeg's
§V remark and it is the single most useful sentence in the paper:

> **Everything that forces a minimum headway is an imperfection of the
> communication or the actuator, not the spacing policy.**

### Where the minimum headway comes from, then

Reintroduce `D(s) = e^{−θs}`:

```
    Γ(s) = (1/H) [ KG + e^{−θs} ] / (1 + KG)
         = (1/H) [ 1 + (e^{−θs} − 1)/(1 + KG) ]
         = (1/H) [ 1 + (e^{−θs} − 1) S̃(s) ]  ,   S̃ := (1 + KG)⁻¹
```

The deviation from the ideal `1/H` is `(e^{−θs} − 1)S̃(s)`, which vanishes as
`θ → 0` and grows like `|e^{−jωθ} − 1| = 2|sin(ωθ/2)|` — zero at DC, rising to
2. So the delay injects error at **mid and high frequency**, where `1/H`
attenuates by `1/√(1+h²ω²)`. Larger `h` buys more attenuation to cover it.
**That is the entire mechanism of every minimum-headway theorem in this
literature**, and Köroğlu (Part IV) turns exactly this observation into a
closed-form bound.

Ploeg quantifies it (his Figs. 3, 4) with τ = 0.1, k_p = 0.2, k_d = 0.7,
k_dd = 0:

| criterion | θ = 0 | θ = 0.15 s | θ = 0.3 s |
|---|---|---|---|
| min h for `‖Γ‖_{H∞} ≤ 1` | 0⁺ | ≈0.2 s | ≈0.5 s |
| min h for `‖γ‖_{L₁} ≤ 1` | 0⁺ | larger | larger |

and `θ_max(h)` is **monotonically increasing and roughly linear** in h
(Fig. 3b): every extra 0.1 s of headway buys roughly 0.1 s of delay tolerance.

### Why `‖Γ‖_{H∞} = 1` is the best you can do  (eq. 32)

`G` has a double pole at the origin, so `KG → ∞` as `s → 0`, hence
`(KG + D)/(1 + KG) → 1` and `H(0) = 1`:

```
    Γ(0) = 1   ⟹   ‖Γ(jω)‖_{H∞} ≥ 1                                    (32)
```

Physically (Ploeg): with `u_r = 0` all vehicles converge to the same constant
velocity, so `lim_{ω→0}(v̂_i − v̂_{i−1}) = 0` and the DC gain is unity. This is
F-13, and its consequence is stated bluntly in the paper:

> "string stability robustness with respect to, e.g., model uncertainties, may
> be poor in case these cause `|Γ(jω)|` to increase in the lower frequency
> region."

**This sentence is the bridge to Ma's paper.** Multiplicative channel noise
perturbs the feedforward gain — a *low-frequency* perturbation of exactly the
kind Ploeg warns has no margin. Ma's whole contribution is to quantify how
much headway buys back that margin.

---

## 3.5 Ploeg's structure vs Ma's structure — the comparison to have ready

Both are "CACC with a constant time headway policy and one-vehicle
look-ahead." They give *qualitatively different* minimum-headway results, and
the reason is structural.

| | **Ploeg** | **Ma** |
|---|---|---|
| feedforward signal | predecessor's **command** `u_{i−1}` | predecessor's **measured acceleration** `a_{i−1}` |
| feedforward gain | unity (fixed) | `k_a` (free design parameter) |
| precompensator | `H⁻¹(s)` present | absent |
| `Γ` with ideal comms | `1/H(s)` exactly | `(k̃a s² + k_v s + k_p)/(τs³+s²+γs+k_p)` |
| min headway, ideal comms | **0⁺** (any h works) | `2τ₀/(1+k_a) > 0` |
| what forces `h > 0` | communication delay / loss | **actuator lag τ₀**, plus channel noise |

**Why the collapse `Γ = 1/H` happens for Ploeg and not for Ma.** Ploeg feeds
forward the predecessor's *desired* acceleration, i.e. the input the
predecessor's plant `G` is about to track, and the `H⁻¹` precompensator
cancels the policy filter that appears in the loop. The two together make the
feedforward path an *exact* model-inverse of the feedback path, so
`KG + 1 = 1 + KG` cancels. Ma feeds forward the predecessor's *realised*
acceleration, which has already been low-passed by that vehicle's lag
`1/(τs+1)`; there is no cancellation available and headway must cover the
residue.

**Verify it algebraically** (a good exam question): could Ma's `H̃` ever equal
`1/H`? Require `(s² + k_v s + k_p)(h_w s + 1) = τs³ + s² + γs + k_p` with
`k̃a = 1`. Expanding the left side gives `h_w s³ + (1 + h_w k_v)s² + (k_v +
h_w k_p)s + k_p`. Matching: `s³` needs `h_w = τ`; `s²` needs `k_v = 0`. So only
in the degenerate case `h_w = τ, k_v = 0`. **Ma's family structurally cannot
reach Ploeg's ideal**, which is why their headway floor is `τ₀` and not zero.

**Which model is "right"?** Neither — they answer different questions. Ploeg
assumes you can transmit the *command*, which requires all vehicles to run
compatible CACC software and to broadcast their intent. Ma assumes you receive
a *measurement*, which is what a generic V2V basic-safety message actually
carries and what a mixed fleet can rely on. Ma's is the more conservative and
more deployable assumption; Ploeg's the more capable one. Say this if asked
which to use.

---

## 3.6 H∞ controller synthesis  (thesis Ch. 3)

The Lp paper *analyses* a given controller. Chapter 3 *synthesises* one, with
string stability as an **a priori** design requirement rather than something
checked afterwards.

### The three conditions

Restating F-23 in the thesis' numbering, with
`P_i(s) = C_i(sI − A)⁻¹B`, `Θ_i := P_i P₁⁻¹`, `Γ_i := P_i P_{i−1}⁻¹`:

```
    Condition 3.1  (L₂ string stability):        sup_{i∈ℕ} ‖P_i‖_{H∞} < ∞
    Condition 3.2  (semi-strict):  ‖P₁‖_{H∞} < ∞ and ‖Θ_i‖_{H∞} ≤ 1 ∀i∈ℕ\{1}
    Condition 3.3  (strict):       ‖P₁‖_{H∞} < ∞ and ‖Γ_i‖_{H∞} ≤ 1 ∀i∈ℕ\{1}
```

`Θ` compares vehicle *i* to the **first**; `Γ` compares it to its
**predecessor**. Strict ⟹ semi-strict.

### The mixed-sensitivity problem, one-vehicle look-ahead

Generalised plant configuration (Fig. 3.3): `(z; v) = 𝒫 (w; r)`,
`r = K v`, `z = N w` with `N = 𝒫₁₁ + 𝒫₁₂K(I − 𝒫₂₂K)⁻¹𝒫₂₁` (lower LFT).

Choose exogenous input `w = u_{i−1}` and exogenous outputs
`z = (W_e S u_{i−1} ; u_i)`, so that

```
    ( e′_i )   ( W_e(s) S(s) )
    (      ) = (            ) u_{i−1}(s)  =:  N(s) u_{i−1}(s)         (3.32)
    (  u_i )   (    Γ(s)     )
```

with, derived exactly as in §3.4 but keeping the two controller blocks
`K = (K_fb  K_ff)` separate:

```
    S̃(s) = (1 + K_fb(s)G(s))⁻¹                                        (3.35)
    S(s) = S̃(s) G(s) (1 − K_ff(s)D(s))                                (3.33)
    Γ(s) = S̃(s) H⁻¹(s) (K_fb(s)G(s) + K_ff(s)D(s))                    (3.34)
```

*Derivation of (3.34).* `H u_i = K_fb e_i + K_ff D u_{i−1}`,
`e_i = G u_{i−1} − H G u_i`. Substitute:
`H u_i(1 + K_fb G) = (K_fb G + K_ff D)u_{i−1}`, hence (3.34). ∎
*Derivation of (3.33).* `e_i = G u_{i−1} − H G u_i = G(1 − HΓ)u_{i−1}`, and
`1 − HΓ = 1 − S̃(K_fb G + K_ff D) = S̃(1 − K_ff D)` using
`1 − S̃K_fb G = S̃`. ∎

### The synthesis objective is `= 1`, not `< 1`

From (3.32), `‖N‖_{H∞} = γ ⟹ ‖Γ‖_{H∞} ≤ γ` (3.36), so any `γ ≤ 1` delivers
string stability. But `lim_{ω→0}|Γ(jω)| = 1` (F-13/eq. 3.37) forces
`‖Γ‖_{H∞} ≥ 1`. Therefore

```
    ‖N(s)‖_{H∞} = 1                                                    (3.38)
```

is the achievable optimum, and **a synthesis that returns `γ = 1` is not a
failure — it is the certificate.**

`W_e(s)` trades vehicle-following performance against string stability;
Ploeg sets `W_e = 1` to weight all frequencies equally.

### The synthesised controller

With `τ = 0.1 s`, `φ = 0.2 s`, `θ = 0.02 s`, 3rd-order Padé for both delays,
`h = 1 s`, H∞ optimisation gives a 10th-order controller, reduced (Hankel
singular values + manual pole/zero pruning) to

```
    K_fb(s) = 2.6880(s+23.22)(s+10)(s+1)(s+0.3646)
              ───────────────────────────────────────
              (s+24.65)(s+5.926)(s+5.049)(s+0.9947)

    K_ff(s) = 1.0391(s+24.1)(s+7.233)(s+4.051)(s+1)
              ───────────────────────────────────────                  (3.39)
              (s+24.65)(s+5.926)(s+5.049)(s+0.9947)
```

Two readings worth stating:

* `|K_ff(jω)| ≈ 1` across the whole band — the synthesis *rediscovers* the
  unity feedforward that was postulated in the Lp paper. Independent
  confirmation that the structural choice was right.
* `|K_fb(jω)|` has slope ≈ +1 near ω = 1 rad/s — the feedback controller
  contains a differential action, i.e. the synthesis rediscovers PD.

### Two-vehicle look-ahead, and why it needs *semi*-strict stability

Add `u*_{i−2}` as a third controller input. The obstruction (§3.6.2): `u_{i−1}`
and `u_{i−2}` are **not independent** — both depend on `u_{i−3}`, which depends
on `u_{i−4}`, etc. A single-vehicle subsystem with an independent input cannot
be isolated, so `Γ_i = P_iP_{i−1}⁻¹` is not the right object. Take the lead
input `u₁` as exogenous instead, giving `u_i = Θ_i(s)u₁` (3.40) and Condition
3.2 (semi-strict).

The synthesised `N_i(s)` now **depends on the vehicle index** — undesirable for
ad hoc platooning. Ploeg's workaround: synthesise for vehicle 3 (the first
with two predecessors), apply it to all `i ≥ 4`, then *verify* (3.22b) for
every i by a clever recursion. Writing `X_i(jω) = (Θ_{i−1}; Θ_i)`,

```
    X_{i+1}(jω) = A(jω) X_i(jω) ,  i ≥ 2                               (3.47)
                  ⎛        0                    1              ⎞
    A(jω)      =  ⎜ S̃H⁻¹K_ff,2 D        S̃H⁻¹(K_fb G + K_ff,1 D) ⎟      (3.48)
                  ⎝                                            ⎠
```

with `X₂ᵀ = (1, Γ(jω))`. This is a **linear recursion in the vehicle index at
each fixed frequency**, so string stability becomes a question about the
eigenvalues `λ_j(jω)` of `A(jω)`: inside the unit circle ⟹ `Θ_i(jω) → 0` as
`i → ∞`. Ploeg's Fig. 3.9(a) shows `|λ_j| < 1` ∀ ω > 0, with `|λ₂(j0)| = 1`
(the DC constraint again).

**Result:** the two-vehicle topology is "particularly effective at a larger
communication delay" — extra information buys delay tolerance. **Cost:** larger
distance-error amplitudes (Fig. 3.10(d): ±0.25 m vs ±0.04 m for one-vehicle
look-ahead), and a synthesis that is no longer index-independent.

---

## 3.7 Graceful degradation: dCACC  (thesis Ch. 4)

**The problem.** The feedforward path is the whole advantage of CACC. If the
radio drops out (or the predecessor is not CACC-equipped), `D → 0` and the
system degrades **to ACC**, whose minimum string-stable headway is far larger
— a discontinuous, unsafe jump. Ploeg's numbers with Table 4.1 parameters
(τ = 0.1, φ = 0.2, k_p = 0.2, k_d = 0.7):

```
    CACC  : string stable for  h ≥ 0.25 s
    dCACC : string stable for  h ≥ 1.23 s
    ACC   : string stable for  h ≥ 3.16 s
```

At 80 km/h these are gaps of 5.6 m, 27.3 m and 70.2 m. Falling from CACC
straight to ACC means a vehicle must suddenly open a 70 m gap.

**The idea.** Replace the lost `u_{i−1}` by an **estimate of the
predecessor's actual acceleration** `â_{i−1}`, obtained from the ego vehicle's
own radar. No radio needed.

### The Singer target model  (eqs. 4.18–4.20)

Model the predecessor's acceleration as a first-order Gauss–Markov process:

```
    ȧ(t) = −α a(t) + u(t) ,      α = 1/τ_m  (inverse manoeuvre time constant)
```

`u` unknown ⟹ treat as zero-mean white noise (the "equivalent-noise
approach"). Its intensity is fixed by matching the *stationary variance* to a
physically-motivated acceleration distribution: impulses `P_max` at `±a_max`,
`P₀` at 0, and uniform density elsewhere on `[−a_max, a_max]`.

**Derive `σ_a²`** (Ploeg's eq. 4.19 — do this, it is a clean exercise):

```
    E[a] = 0                                          (symmetry)
    E[a²] = 2 P_max a_max²
            + [1 − (P₀ + 2P_max)]/(2a_max) · ∫_{−a_max}^{a_max} a² da
          = 2 P_max a_max² + [1 − P₀ − 2P_max]/(2a_max) · (2a_max³/3)
          = a_max² [ 2P_max + (1 − P₀ − 2P_max)/3 ]
          = (a_max²/3) (6P_max + 1 − P₀ − 2P_max)
    ⟹ σ_a² = (a_max²/3)(1 + 4P_max − P₀)                              (4.19) ✓
```

**Derive the noise intensity.** For `ȧ = −αa + u` driven by white noise of
intensity `q`, the stationary variance is `σ_a² = q/(2α)`. Hence
`q = 2ασ_a²`, i.e.

```
    C_uu(τ) = 2 α σ_a² δ(τ)                                            (4.20) ✓
```

State-space form (4.21)–(4.22): `x = (q, v, a)ᵀ`,

```
    A_a = [0 1  0 ]  B_a = [0]  C_a = [1 0 0]
          [0 0  1 ]        [0]        [0 1 0]
          [0 0 −α]         [1]
```

Note `A_a` is the vehicle model (4.9) with `α` in place of `1/τ` — the
estimator's target model has the *same structure* as the plant, which is why
the same `G(s)` intuition transfers.

### The Kalman estimator in relative coordinates  (eqs. 4.28–4.36)

The catch: the observer (4.28) `x̂̇ = A_a x̂ + L_a(y − C_a x̂)` wants **absolute**
position and velocity of the target, but radar measures **relative** distance
`d_i` and relative velocity `Δv_i`. Ploeg splits the estimator by linearity:

```
    q_{i−1} = d_i + q_i ,   v_{i−1} = Δv_i + v_i                       (4.32)
    ⟹  â_{i−1}(s) = T(s)(d_i; Δv_i)  +  T(s)(q_i; v_i)                (4.33)
                    ╰── relative part ──╯   ╰── absolute (own) part ──╯
```

The own-vehicle part is then re-expressed using `q_i = a_i/s²`, `v_i = a_i/s`
so that it needs only the ego vehicle's **measured acceleration**:

```
    â_i(s) = [ T_aq(s)/s² + T_av(s)/s ] a_i(s) =: T_aa(s) a_i(s)       (4.36)
```

"thereby avoiding the use of a potentially inaccurate absolute position
measurement by means of GPS." `T_aa` acts as a filter that **synchronises the
ego vehicle's own accelerometer with the estimator's dynamics** — you must pass
your own measurement through the same lag you imposed on the target's, or the
two halves of `â_{i−1}` would be phase-mismatched.

Final dCACC control law:

```
    u_i(s) = H⁻¹(s){ K(s)e_i(s) + T(s)(d_i; Δv_i) + T_aa(s)a_i(s) }    (4.37)
```

### String stability of dCACC  (eq. 4.38)

```
                    1     G(s)(K(s) + s² T_aa(s))
    Γ_dCACC(s) = ────── · ──────────────────────                       (4.38)
                   H(s)       1 + G(s)K(s)
```

Compare the three side by side — the *only* thing that changes is the
feedforward term in the numerator:

```
    Γ_ACC   = (1/H) · [ GK + 0        ] / (1 + GK)
    Γ_dCACC = (1/H) · [ GK + G s²T_aa ] / (1 + GK)
    Γ_CACC  = (1/H) · [ GK + e^{−θs}  ] / (1 + GK)
```

Since `G s² = e^{−φs}/(τs+1)`, the dCACC feedforward is
`T_aa(s)e^{−φs}/(τs+1)` — the ideal `e^{−θs}` **filtered by the estimator and
the vehicle lag**. dCACC therefore interpolates between ACC (feedforward 0)
and CACC (feedforward ≈1), and the gap is exactly the estimator's phase lag.
Ploeg's Fig. 4.7 makes this visible: `â₂` tracks `a₂` well in amplitude but
"shows a noticeable phase lag with respect to `u₂`, which is essentially the
reason for the degraded string stability performance of dCACC."

**The design knob and its trade-off.** Increasing `α` (a faster assumed
manoeuvre model) reduces the estimator lag and improves string stability, but
makes the estimate noisier and the ride harsher. Ploeg's rule of thumb:
`0.5 ≤ α ≤ 1.5` s⁻¹.

**Relation to this project.** The project's packet-loss study (D-008) uses
zero-order hold rather than a Singer/Kalman estimator, and finds graceful
degradation of a different kind: 50 % Bernoulli loss at 10 Hz leaves the
platoon marginally stable (max L₂ amplification 1.013), because ZOH loss
mainly adds *effective delay* of order the beacon period. Ploeg's dCACC is the
principled version of the same idea; the project's D-016 predictor
(timestamp-based lead compensation) is a third point on that spectrum.

---

## 3.8 Experimental validation — the numbers to quote

Six Toyota Prius III vehicles, IEEE 802.11p, 10 Hz beacons (Lp paper §VI;
thesis Ch. 5):

* identified model `G(s) = e^{−φs}/(s²(τs+1))` with **τ = 0.1 s, φ = 0.2 s**;
* gains `k_p = 0.2`, `k_d = 0.7`, `k_dd = 0` ("to avoid feedback of the jerk,
  which is in practice unfeasible");
* measured communication delay **θ ≈ 0.15 s**;
* chosen headway **h = 0.7 s**, "just achieving strict L₂ string stability";
* measured `|Γ(jω)|` via Welch's averaged periodogram on a random-phase
  multisine excitation covering `[0, 0.3] Hz = [0, 1.9] rad/s`;
* startup test: with ACC the sixth vehicle starts moving after **15.5 s**;
  with CACC after **8.5 s** — "showing that CACC may also be effective at
  traffic lights."

Two honest caveats Ploeg records, both worth repeating:

1. `|Γ(jω)|` is close to 1 at low frequency (eq. 32), "as a result of which
   estimation inaccuracy may compromise the second string stability criterion"
   — i.e. **you cannot experimentally distinguish `‖Γ‖ = 1.00` from `1.02`.**
   This is the empirical counterpart of the project's D-010.
2. Time gaps down to 0.3 s are theoretically feasible "when minimizing the
   latency of the wireless link," but require "insight in the string stability
   margins in the presence of uncertainties or unknown disturbances" — which
   is, verbatim, the research question Ma et al. answer.

---

## 3.9 Whiteboard drill

1. State Definition 1 and explain what each of `α`, `β`, and "∀m ∈ ℕ" does.
2. Prove Theorem 1 from the submultiplicative property.
3. Say when the conditions become **necessary** and why (Remark 4).
4. Derive `Γ(s) = (1/H)(KG + D)/(1 + KG)` from the block diagram.
5. Show `Γ = 1/H` when `D = 1`, hence `‖Γ‖_{H∞} = ‖γ‖_{L₁} = 1` for all gains
   and all `h > 0`. State the consequence in one sentence.
6. Show `‖γ‖_{L₁} ≥ ‖Γ‖_{H∞}`, so L∞ string stability is strictly harder.
7. Derive `Γ(0) = 1` two ways (algebraic and physical) and say why it matters.
8. Derive `S = S̃G(1 − K_ff D)` and `Γ = S̃H⁻¹(K_fb G + K_ff D)`.
9. Explain why `‖N‖_{H∞} = 1` — not `< 1` — is the synthesis objective.
10. Derive `σ_a² = (a_max²/3)(1 + 4P_max − P₀)` from Ploeg's `p(a)`.
11. Write `Γ_ACC`, `Γ_dCACC`, `Γ_CACC` and explain what changes between them.
12. Explain why the two-vehicle look-ahead topology forces *semi*-strict
    string stability.

---

*Next: [Part IV — Köroğlu: minimum headway under delayed
communication](04-koroglu-delay-headway.md).*
