# Part II — Ma, Pagilla & Darbha (IEEE T-ITS 2025): every equation derived

> G. Ma, P. R. Pagilla, S. Darbha, "Selection of Time Headway in Connected and
> Autonomous Vehicle Platoons Under Noisy V2V Communication," *IEEE Trans.
> Intell. Transp. Syst.* **26**(1):1029–1038, Jan. 2025.
> doi 10.1109/TITS.2024.3498701

**The one-sentence thesis.** *Given a signal-to-noise ratio ρ on the V2V link
that carries the predecessor's acceleration, what is the smallest time headway
at which a CTHP platoon can still be guaranteed string stable — and which
feedforward gain makes that smallest headway smallest?*

**The answer, in three formulas** (eqs. 17, 18, 19):

```
                             1 − (1 − 1/ρ) k_a
    h_w  >  h_w,lb(k_a) = 2τ₀ ─────────────────── ,   0 < k_a < 1/(1 + 1/ρ)
                             1 − (1 + 1/ρ)² k_a²

              1 − 1/√ρ       1                         (1 + 1/√ρ)²
    k_a*  =  ────────── · ───────── ,      h*_w,lb = τ₀ ───────────
              1 + 1/√ρ     1 + 1/ρ                        1 + 1/ρ
```

Everything below builds these from F-1 … F-24 of [Part I](01-foundations.md).

Section numbers here (§2.x) are this document's; equation numbers in
parentheses are **the paper's**.

---

## 2.1 Setup: the plant, the policy, the law  (eqs. 1, 2, 6)

**Plant, eq. (1)** — F-1:

```
    ẍ_i(t) = a_i(t) ,        τ ȧ_i(t) + a_i(t) = u_i(t)               (1)
```

with `τ` *uncertain*, `τ ∈ (0, τ₀]`, τ₀ known. `i ∈ 𝒩 = {1, …, N}` are the
followers; the leader is indexed 0.

**Spacing errors, Definition 1 and eq. (2)** — F-6:

```
    e_i(t) = x_i(t) − x_{i−1}(t) + d               (CSP error)
    δ_i(t) = e_i(t) + h_w v_i(t)                   (CTHP error)         (2)
```

**CTHP control law, eq. (6):**

```
    u_i(t) = k_a w_{i,i−1}(t) a_{i−1}(t) − k_v (v_i(t) − v_{i−1}(t)) − k_p δ_i(t)
                                                                        (6)
```

Read it as three physically distinct terms:

| term | information source | role |
|---|---|---|
| `+ k_a w a_{i−1}` | **V2V radio** — the only cooperative signal | feedforward: pre-empt the predecessor's manoeuvre |
| `− k_v (v_i − v_{i−1})` | **radar** (relative velocity) | damping |
| `− k_p δ_i` | **radar + own speedometer** | spacing regulation |

Only the feedforward term is corrupted, because only it travels over the
radio. Radar is assumed clean. That asymmetry is the entire premise of the
paper: **the noise multiplies the one signal that CACC adds over ACC.**

Note the signs: `−k_v(v_i − v_{i−1})` brakes when the ego vehicle is faster;
`−k_p δ_i` brakes when `δ_i > 0`, i.e. when the gap is too small (F-6). Both
are stabilising with `k_v, k_p > 0`.

**Substituting (6) into (1)** gives eq. (7), the stochastic governing equation:

```
    τ x⃛_i(t) + ẍ_i(t) = k_a w_{i,i−1}(t) a_{i−1}(t)
                          − k_v(v_i − v_{i−1}) − k_p δ_i(t)             (7)
```

*Check:* from (1), `τ ȧ_i + a_i = u_i` and `a_i = ẍ_i`, so
`τ x⃛_i + ẍ_i = u_i`. Substitute (6). ✓

---

## 2.2 The noise model  (eqs. 4, 5)

Fully derived at F-17 … F-20. Compressed restatement:

```
    w_{i,i−1}(t) = (1 − 1/ρ) + (1/ρ) Σ_{j=0}^{n−1} z_{i,j}(t) 2^{−j}   (5)
    z_{i,j} ~ Bernoulli(γ_{i,j}) independent,   n = 16
    ⟹  1 − 1/ρ ≤ w_{i,i−1}(t) < 1 + 1/ρ                       (strictly bounded)
```

with `ρ = 10^{SNR/20} = min_{t>0}|s(t)|/|n(t)|` (Definition 3, eq. 4).

**Three properties you will be asked about:**

1. **Bounded support, no tails.** Consequence of `z_j ∈ {0,1}` and a finite
   sum of positive weights. This is what makes a *deterministic guarantee*
   possible at all — with Gaussian noise no finite headway would suffice.
2. **`E[w] ≠ 1` in general.** With Table I and ρ = 5, `E[w] = 1.04817`
   (F-18). The channel is biased optimistic by 4.8 %.
3. **`γ_{i,j}` are unknown to the designer.** So the design must hold for
   every attainable mean, i.e. for `k̃a` anywhere in `I` (F-19a).

---

## 2.3 From a stochastic to a deterministic system  (eqs. 8–13, Remark 5)

This is the step most people skip and then cannot defend. Do not skip it.

### The problem

`w_{i,i−1}(t)` is a stochastic process, so (7) is a **stochastic differential
equation** and the state `X(t)` is a stochastic process. "String stability"
of a random trajectory is not a well-posed notion without further definition.

### The construction

Stack everything into an augmented state (their eq. 8):

```
    X̂(t) = [x₀, v₀, a₀, x₁, v₁, a₁, …, x_N, v_N, a_N]ᵀ
    Ẋ̂(t) = Â(ŵ(t)) X̂(t) + B U(t)                                     (8)
```

where `ŵ(t)` collects the `N` channel factors and `U(t)` is the leader's
acceleration input. `Â` depends on the random `ŵ` only through the
sub-diagonal blocks (the `k_a w_{i,i−1} a_{i−1}` coupling).

The averaging result they import from [28] (Vegamoor, Rathinam & Darbha,
T-ITS 2022) is eq. (9):

```
    E[Â] = Ā ,        E[e^{Ât}] = e^{Āt}                                (9)
```

with `Ā := Â(w̄)`, `w̄ = E[ŵ]`.

### Why the *second* equality is not automatic — and why it holds here

The first equality is linearity of expectation: trivial.

The second is **not** trivial, because `E[Â²] ≠ (E[Â])²` for general random
matrices — the square contains products of entries, and expectation does not
pass through products of dependent variables. The paper justifies it by "the
specific lower triangular and banded structure of the governing equations for
the CACC case." Unpacked, the argument is:

* `Â` is **block lower bidiagonal**: the block row for vehicle *i* has a
  diagonal block (deterministic) and exactly one sub-diagonal block carrying
  `w_{i,i−1}`.
* Expand `e^{Ât} = Σ_k Â^k t^k / k!`. The `(i,j)` block of `Â^k` is a sum over
  directed paths of length k from level *i* to level *j* in the block graph.
  Because the only off-diagonal coupling steps **down by exactly one level**,
  any such path descends monotonically: it uses link `(ℓ, ℓ−1)` **at most
  once**, for each ℓ.
* Therefore every monomial appearing in `Â^k` is a product of *distinct*
  `w_{ℓ,ℓ−1}` factors, and the channels are independent across links, so
  `E[∏_ℓ w_ℓ] = ∏_ℓ E[w_ℓ]`.
* Hence `E[Â^k] = Ā^k` for every k, and summing the series gives
  `E[e^{Ât}] = e^{Āt}`.

**Where it would fail:** a bidirectional topology, or multi-predecessor
look-ahead, would create paths that revisit a link, producing `E[w²] ≠ (E[w])²`
terms (and `Var(w) > 0` makes that strict). Ma's conclusion flags exactly this
as future work ("extension under multiple predecessor follower information
flow"). This is a good, specific answer to "what breaks if you change the
topology?"

### The equivalent deterministic system

With (9), the expected state satisfies the deterministic equation (10),
`Ẋ̄ = Ā X̄ + B U`, and per-vehicle (their eq. 11):

```
    τ x⃛̄_i + x̄̈_i = k_a E[w_{i,i−1}] ā_{i−1} − k_v(v̄_i − v̄_{i−1}) − k_p δ̄_i  (11)
```

Define the **effective feedforward gain** (their eq. 12) — F-19:

```
    k̃a := k_a E[w_{i,i−1}] = [1 − 1/ρ + (1/ρ)Σ_{j=0}^{n−1} γ_{i,j}/2^j] k_a  (12)
```

so (11) becomes the clean deterministic governing equation

```
    τ x⃛̄_i + x̄̈_i = k̃a x̄̈_{i−1} − k_v(x̄̇_i − x̄̇_{i−1}) − k_p δ̄_i         (13)
```

### Remark 5 and its honest reading

> *Remark 5: the deterministic system (13) is an equivalent system to the
> stochastic system (11) in the mean or expected sense, and robust string
> stability analysis is performed on the deterministic system (13).*

What this **does** give you: the certified object is `E[X(t)]`. Every ‖H̃‖∞
statement in the paper is about the mean trajectory.

What it **does not** give you: a probability-one statement about individual
sample paths. Realisation-to-realisation variance is not bounded by the
analysis.

This project measured that gap directly. Running the Simulink model with the
realised 16-bit channel versus its mean (`matlab/run_all.m` in both modes):

| Case 1, max\|δ_i\| | vehicle 1 | vehicle 12 | monotone in i? |
|---|---|---|---|
| deterministic equivalent (13) | 1.5959 | 1.5436 | **yes** |
| one stochastic realisation (5) | 1.5990 | 1.5566 | **no** — ±2 % jitter |

The per-hop attenuation being certified is ≈0.35 %; a single realisation
jitters `max|δ_i|` by ≈2 %, i.e. **six times larger than the effect**. The
monotone profiles the paper prints (their Figs. 7, 10, 12) are therefore
properties of the mean system, exactly as Remark 5 says — not of any one run.
Reproducing Fig. 7 requires running (13), not (5). That is a genuine finding
about how to read the paper, and it is the reason
`ma2025.params(..., 'noise_mode', 'mean')` is the default in the Simulink
backend.

---

## 2.4 The spacing-error propagation transfer function  (eq. 14)

**The central object.** Everything after this is algebra on `H̃`.

### Statement

```
                     Ñ(s)        k̃a s² + k_v s + k_p
    H̃(s; τ) = ───────── = ───────────────────────────── ,   γ := k_v + h_w k_p
                     D(s)      τ s³ + s² + γ s + k_p          (14)
```

and `δ̄_i(s) = H̃(s; τ) δ̄_{i−1}(s)`.

### Derivation (do this on the whiteboard; it takes four lines)

Laplace-transform (13) with zero initial conditions, writing `X_i(s)` for the
transform of `x̄_i(t)` (the constant `d` in δ shifts only the equilibrium and
drops out of the deviation dynamics):

```
    τ s³ X_i + s² X_i = k̃a s² X_{i−1} − k_v s (X_i − X_{i−1}) − k_p Δ_i
```

with, from (2),

```
    Δ_i = (X_i − X_{i−1}) + h_w s X_i = (1 + h_w s) X_i − X_{i−1}      (†)
```

**Step 1 — group by index.**

```
    (τ s³ + s² + k_v s) X_i − (k̃a s² + k_v s) X_{i−1} = − k_p Δ_i
```

**Step 2 — substitute (†) on the right only.**

```
    (τs³ + s² + k_v s) X_i − (k̃a s² + k_v s) X_{i−1}
        = − k_p[(1 + h_w s) X_i − X_{i−1}]
```

**Step 3 — collect `X_i` on the left, `X_{i−1}` on the right.**

```
    (τs³ + s² + k_v s + k_p + k_p h_w s) X_i = (k̃a s² + k_v s + k_p) X_{i−1}
```

Since `k_v s + k_p h_w s = (k_v + h_w k_p)s = γ s`:

```
    X_i        k̃a s² + k_v s + k_p        Ñ(s)
    ───── = ───────────────────────── =  ─────                        (‡)
    X_{i−1}   τs³ + s² + γ s + k_p         D(s)
```

**Step 4 — show the *spacing errors* propagate with the same function.**
Using (†) twice and `X_{i−2} = X_{i−1}/H̃`:

```
    Δ_i     = (1 + h_w s) X_i − X_{i−1}      = [(1 + h_w s) H̃ − 1] X_{i−1}
    Δ_{i−1} = (1 + h_w s) X_{i−1} − X_{i−2}  = [(1 + h_w s) − 1/H̃] X_{i−1}
                                             = [(1 + h_w s) H̃ − 1] X_{i−1} / H̃
    ⟹  Δ_i / Δ_{i−1} = H̃                                          ∎
```

### Three observations worth stating out loud

1. **Position propagation and spacing-error propagation share one transfer
   function.** That is special to homogeneity — the factor
   `[(1 + h_w s)H̃ − 1]` cancels because it is the *same* factor at both
   indices. Heterogeneous τ_i breaks it (each hop has its own H̃_i, and the
   argument becomes hop-wise: `∏_i ‖H̃_i‖∞ ≤ 1` suffices, cf. F-12).
2. **The headway `h_w` enters only through `γ = k_v + h_w k_p`, in the
   denominator.** It does *not* appear in the numerator. So increasing h_w
   adds damping to `D(s)` without adding anything to `Ñ(s)` — that is the
   mechanism by which headway buys string stability, made concrete.
3. **`k̃a` appears only in the `s²` coefficient of the numerator**, i.e. only
   at high frequency. This localises the noise's influence: the channel
   perturbs the *high-frequency* numerator while the feedback gains shape the
   denominator everywhere.

### Consistency checks

* `H̃(0) = k_p/k_p = 1` — the structural DC constraint F-13. ✓
* `k̃a = 1, k_v, k_p → 0`: `H̃ → s²/(τs³+s²) = 1/(τs+1)`, a stable low-pass —
  perfect feedforward makes the platoon a chain of first-order filters.
* Poles of `D`: Routh–Hurwitz `γ > τ k_p` (F-14a).

---

## 2.5 Theorem 1: the two necessary facts  (eqs. 15, 21, 22)

> **Theorem 1.** (a) `‖H̃(jω;τ)‖∞ ≤ 1 ∀τ ∈ (0,τ₀]` implies `k̃a ∈ (0,1)`.
> (b) Given `k̃a ∈ (0,1)`, for `k_v, k_p > 0` to exist with
> `‖H̃(jω;τ)‖∞ ≤ 1 ∀τ ∈ (0,τ₀]`, the headway must satisfy
> `h_w > 2τ₀/(1 + k̃a)`.  (15)

### 2.5.1 Proof of (a): why `k̃a < 1`

The uncertainty set contains **arbitrarily small τ**, and that is the whole
argument.

Let `τ → 0⁺`. Then `D(s) → s² + γs + k_p` and

```
    H̃(s) → (k̃a s² + k_v s + k_p)/(s² + γs + k_p)
    ⟹  lim_{ω→∞} |H̃(jω)| = k̃a
```

Since `‖H̃‖∞ ≥ lim_{ω→∞}|H̃(jω)|` and the bound must hold for *every* τ in the
interval, including τ arbitrarily close to 0, we need `k̃a ≤ 1`. Positivity
(`k̃a > 0`) is immediate from `k_a > 0` and `w > 0`. ∎

**Note what this says:** the constraint `k̃a < 1` is imposed by the *fastest
possible vehicle*, not the slowest. A vehicle with negligible lag has no
high-frequency roll-off to hide behind, and the feedforward gain becomes the
direct high-frequency gain of the hop. Feed forward more than 100 % of your
predecessor's acceleration and you amplify — with any τ.

Equivalently from eq. (25) with τ = 0: `(1 − k̃a²)ω² + (…) ≥ 0` for all ω
forces `1 − k̃a² ≥ 0`.

### 2.5.2 Proof of (b): the `2τ₀/(1 + k̃a)` bound

This is the noiseless-channel headway bound of Darbha–Konduri–Pagilla [9],
here re-derived from the coefficient conditions (which we prove independently
in §2.6). Take them as given for the moment:

```
    (26a)  1 − k̃a² − 2τγ ≥ 0        ⟹ (worst at τ=τ₀)  γ ≤ (1 − k̃a²)/(2τ₀)
    (26b)  γ² − 2k_p(1 − k̃a) − k_v² ≥ 0  ⟹  γ² ≥ 2k_p(1 − k̃a) + k_v²
```

**Step 1 — linearise (26b).** Substitute `γ = k_v + h_w k_p` and expand:

```
    (k_v + h_w k_p)² ≥ 2k_p(1 − k̃a) + k_v²
    k_v² + 2h_w k_v k_p + h_w²k_p² ≥ 2k_p(1 − k̃a) + k_v²
```

The `k_v²` cancels. Divide by `k_p > 0`:

```
    2 h_w k_v + h_w² k_p ≥ 2(1 − k̃a)                                   (★)
```

**This cancellation is the key structural fact of the paper** and it is worth
pausing on: (26b) is quadratic in the gains, but after substituting the
definition of γ the quadratic term in `k_v` cancels exactly, leaving a
*linear* inequality. That is why the feasible region in the `(k_v, k_p)` plane
is a polygon (Figs. 5 and 11) and not a conic section.

**Step 2 — bound the left side using (26a).** Since `k_p > 0`,

```
    2k_v + h_w k_p  ≤  2k_v + 2h_w k_p = 2(k_v + h_w k_p) = 2γ ≤ (1 − k̃a²)/τ₀
```

Multiply by `h_w > 0` and chain with (★):

```
    2(1 − k̃a) ≤ h_w(2k_v + h_w k_p) ≤ h_w (1 − k̃a²)/τ₀
```

**Step 3 — solve for h_w.**

```
    h_w ≥ 2τ₀ (1 − k̃a)/(1 − k̃a²) = 2τ₀ (1 − k̃a)/[(1 − k̃a)(1 + k̃a)]
        = 2τ₀/(1 + k̃a)                                                 ∎
```

Strictness comes from requiring `k_p > 0` strictly.

**Sanity check the limits.** `k̃a → 0` (no feedforward, i.e. ACC): `h_w > 2τ₀`
— you need twice the lag as headway. `k̃a → 1` (perfect feedforward):
`h_w > τ₀` — one lag time. So the *entire benefit of CACC over ACC in this
model is a factor of two in headway*, and it is bought by the feedforward
term. At τ₀ = 0.5 s that is 1.0 s versus 0.5 s; at 25 m/s, 25 m versus 12.5 m
per vehicle.

### 2.5.3 From (15) to a designable bound: eqs. (20)–(24)

Theorem 1(b) is stated for a *known* `k̃a`. The designer does not know it
(the γ_{i,j} are unknown), so bound it.

**eq. (20) — the upper end of k̃a.** From (12), `Σ_j γ_j 2^{−j} < Σ_j 2^{−j}
= 2 − 2^{−(n−1)} < 2`, so

```
    k̃a ≤ (1 − 1/ρ + 2/ρ) k_a = (1 + 1/ρ) k_a                          (20)
```

**eq. (21) — hence a designable cap on k_a.**

```
    k_a < 1/(1 + 1/ρ)   ⟹   k̃a < 1                                    (21)
```

This is why Theorem 2's admissible range is `k_a ∈ (0, 1/(1+1/ρ))` and not
`(0,1)`: you must leave room for the channel to *amplify* the feedforward
signal by up to `1 + 1/ρ`. Numerically at ρ = 5, `k_a < 0.8333` — the paper's
eq. (44).

**eqs. (23)–(24) — the worst-case headway bound.** Robustify (22) over the
unknown γ's:

```
    h_w > max_{γ₀,…,γ_{n−1}} 2τ₀/(1 + k̃a) = 2τ₀ / (1 + min_γ k̃a)
```

and `min_γ Σγ_j2^{−j} = 0` (all γ_j → 0), giving `min k̃a = (1 − 1/ρ)k_a`:

```
    h_w > 2τ₀ / (1 + (1 − 1/ρ) k_a)                                    (24)
```

**This bound is real but weak.** For `k_a = 0.5, ρ = 5, τ₀ = 0.5`:
`2(0.5)/(1 + 0.8×0.5) = 1/1.4 = 0.7143 s`, whereas Theorem 2 will give
0.9375 s. Verified numerically in `theorem_checks.m` block E. The gap is the
subject of the next section: (24) only asks that *some* `(k_v,k_p)` work for
*each* `k̃a` separately; Theorem 2 asks that **one** `(k_v,k_p)` work for
**all** `k̃a ∈ I` simultaneously — because you cannot retune the gains per
realisation of an unknown channel.

---

## 2.6 The string-stability inequality  (eqs. 25–29)

### 2.6.1 Deriving eq. (25) — the full expansion

Robust string stability requires `‖H̃(jω;τ)‖²∞ ≤ 1`, i.e. (F-16)
`|Ñ(jω)|² ≤ |D(jω)|²` for all ω.

**Numerator.** `Ñ(s) = k̃a s² + k_v s + k_p`. At `s = jω`, split by parity
(F-15): `s² = −ω²` (real), `s = jω` (imaginary):

```
    Ñ(jω) = (k_p − k̃a ω²) + j (k_v ω)
    |Ñ(jω)|² = (k_p − k̃a ω²)² + k_v²ω²
             = k_p² − 2k_p k̃a ω² + k̃a²ω⁴ + k_v²ω²
```

**Denominator.** `D(s) = τs³ + s² + γs + k_p`. At `s = jω`:
`s³ = −jω³`, `s² = −ω²`, `s = jω`:

```
    D(jω) = (k_p − ω²) + j (γω − τω³)
    |D(jω)|² = (k_p − ω²)² + (γω − τω³)²
             = k_p² − 2k_p ω² + ω⁴ + γ²ω² − 2γτω⁴ + τ²ω⁶
```

**Difference.** `k_p²` cancels:

```
  |D|² − |Ñ|² = τ²ω⁶ + (1 − 2γτ − k̃a²)ω⁴ + (γ² − 2k_p + 2k̃a k_p − k_v²)ω²
              = ω²[ τ²ω⁴ + (1 − k̃a² − 2τγ)ω² + γ² − 2k_p − k_v² + 2k̃a k_p ]
```

Requiring this `≥ 0` for all ω and dividing out `ω² > 0`:

```
    τ²ω⁴ + (1 − k̃a² − 2τγ)ω² + γ² − 2k_p − k_v² + 2k̃a k_p ≥ 0        (25)
```

which is exactly the paper's eq. (25). ✓

*Numerically verified*: `theorem_checks.m` block B computes both sides on a
4000-point grid and reports `max|LHS − RHS| = 1.8 × 10⁻¹²`, and confirms
pointwise that the sign of `|D|² − |Ñ|²` matches `|H̃| ≤ 1`.

### 2.6.2 The sufficient split — eqs. (26)–(28), and its price

(25) is a quadratic in `X := ω² ≥ 0`:

```
    q(X) = τ²X² + (1 − k̃a² − 2τγ) X + (γ² − 2k_p − k_v² + 2k̃a k_p) ≥ 0
```

The paper's route is to demand each coefficient be non-negative separately:

```
    1 − k̃a² − 2τγ ≥ 0                                                 (26a)
    γ² − 2k_p − k_v² + 2k̃a k_p ≥ 0                                    (26b)
```

Since `τ² X² ≥ 0` automatically, (26a) ∧ (26b) ⟹ `q(X) ≥ 0 ∀X ≥ 0`.

**Rearranged (eqs. 27a, 27b), worst case over τ ∈ (0, τ₀] is τ = τ₀:**

```
    (27a)   γ ≤ (1 − k̃a²)/(2τ₀)
    (27b)   γ ≥ √(2k_p(1 − k̃a) + k_v²)
```

**This split is SUFFICIENT, NOT NECESSARY** — be ready to say why and to give
a counterexample.

*Why not necessary.* `q(X) ≥ 0 ∀X ≥ 0` does not require both non-leading
coefficients to be non-negative. A quadratic with positive leading coefficient
`τ²`, a *negative* linear coefficient, and a positive constant term is
non-negative for all `X ≥ 0` provided its discriminant is non-positive:

```
    (1 − k̃a² − 2τγ)² ≤ 4τ²(γ² − 2k_p − k_v² + 2k̃a k_p)
```

Physically: a large enough `τ²ω⁶` term (the vehicle-lag roll-off) can rescue a
mildly negative `ω⁴` term. The paper's split throws that possibility away.

*A counterexample, produced by the code.* `theorem_checks.m` block F searches
the design box and finds:

```
    k_a = 0.5, ρ = 5, k_v = 0.70, k_p = 0.009, h_w = 1.50 s
    (28a) FAILS,  (28b) holds   ⟹  the sufficient conditions reject the design
    ‖H̃‖∞ over the whole interval I  =  1.000000  ≤ 1
    ⟹ the design IS robustly string stable
```

**Consequence for practice, and a project finding.** Because (28) is only
sufficient, minimum-headway requirements computed from the closed form are
*conservative* for a platoon whose gains are already fixed. This project
therefore computes fixed-gain headway requirements by bisection on the true
`‖H̃‖∞` rather than from `h_lb`, and found the critical SNR `ρ*` where a
fixed-gain design fails to be ≈1.8, not the ≈3 the closed form suggests
(`docs/theory/T-07`, decision D-017). Treating `h_lb` as *the* requirement for
a running platoon is a real error; `h_lb` is the requirement **when the gains
are retuned for ρ**.

### 2.6.3 Robustifying over the interval I — eqs. (28a), (28b)

Now impose (27) for **every** `k̃a ∈ I = [(1−1/ρ)k_a, (1+1/ρ)k_a]` (F-19a),
using one fixed `(k_v, k_p)`.

**(28a) binds at the HIGH end.** `(1 − k̃a²)/(2τ₀)` is *decreasing* in `k̃a`,
so its minimum over `I` is at `k̃a = (1 + 1/ρ)k_a`:

```
    γ ≤ min_{k̃a ∈ I} (1 − k̃a²)/(2τ₀) = [1 − (1 + 1/ρ)² k_a²] / (2τ₀)   (28a)
```

**(28b) binds at the LOW end.** `√(2k_p(1 − k̃a) + k_v²)` is *decreasing* in
`k̃a`, so its maximum over `I` is at `k̃a = (1 − 1/ρ)k_a`:

```
    γ ≥ max_{k̃a ∈ I} √(2k_p(1 − k̃a) + k_v²) = √(2k_p[1 − (1 − 1/ρ)k_a] + k_v²)
                                                                       (28b)
```

Together, eq. (29):

```
    √(2k_p[1 − (1 − 1/ρ)k_a] + k_v²)  ≤  γ  ≤  [1 − (1 + 1/ρ)²k_a²]/(2τ₀)
```

**The physical reading, and the intuition to carry away:**

| condition | binding channel state | what goes wrong there |
|---|---|---|
| (28a) upper | channel **amplifies** (`w = 1+1/ρ`) | too much feedforward + too much feedback authority ⟹ high-frequency amplification. Note the ceiling is **independent of h_w** — no amount of headway can rescue an over-aggressive `γ`. |
| (28b) lower | channel **attenuates** (`w = 1−1/ρ`) | too little feedforward ⟹ the low-frequency burden falls on the feedback loop, which needs *more* `γ` to carry it. Headway helps here, because `γ = k_v + h_w k_p` grows with h_w. |

**The h_w-independence of (28a) is the root of the "fixed-gain wall."** In the
QoS-adaptive extension of this project (D-016/D-017), a platoon whose channel
degrades cannot always be saved by lengthening the headway: once `γ` exceeds
the (28a) ceiling for the new ρ, *only re-tuning `k_v` restores feasibility*.
That finding is a direct corollary of reading which side of (29) contains
`h_w`.

**Which end binds the actual ‖H̃‖∞?** For the paper's designs, the **low**
end — measured, not asserted. From `matlab/results/run_all_log.txt`:

```
    Case 1: ‖H̃‖∞ = 1.000000 (low) | 1.000000 (mean) | 1.000000 (high)
    Case 2: ‖H̃‖∞ = 1.003500 (low) | 1.001164 (mean) | 1.000000 (high)
                     ↑ the violation lives at k̃a = 0.4 = (1−1/ρ)k_a
```

Case 2 is string-unstable **only at the attenuating end of the channel**. If
you evaluated at the nominal `E[w]` you would report 1.0012; at the amplifying
end you would report 1.0000 and conclude the design is fine. This is decision
D-009 of the project, and it is a good "gotcha" answer: *the worst case is a
channel that under-reports acceleration, not one that over-reports it.*

---

## 2.7 Theorem 2(b): the headway lower bound  (eqs. 30–35, 17)

> **Theorem 2(b).** Given `k_a ∈ (0, 1/(1+1/ρ))` and
> `h_w > h_w,lb(k_a) := 2τ₀ [1 − (1−1/ρ)k_a] / [1 − (1+1/ρ)²k_a²]`, (17)
> there exist `k_v, k_p > 0` with `‖H̃(jω;τ)‖∞ ≤ 1` for all `τ ∈ (0, τ₀]`.

Two proofs. Learn the short one; know the paper's one.

### 2.7.1 Short proof (the §2.5.2 argument with interval-robust constants)

Repeat §2.5.2 verbatim, replacing (26a)/(26b) by (28a)/(28b). Write
`m := 1 − 1/ρ` and `n := (1 + 1/ρ)²`.

Squaring (28b), substituting `γ = k_v + h_w k_p`, cancelling `k_v²`, dividing
by `k_p > 0` — exactly the (★) cancellation:

```
    2 h_w k_v + h_w² k_p ≥ 2(1 − m k_a)                                (33)
```

Bounding `2k_v + h_w k_p ≤ 2γ ≤ (1 − n k_a²)/τ₀` from (28a) and multiplying
by `h_w`:

```
    2(1 − m k_a) ≤ h_w(2k_v + h_w k_p) ≤ h_w (1 − n k_a²)/τ₀
    ⟹  h_w ≥ 2τ₀ (1 − m k_a)/(1 − n k_a²) = h_w,lb(k_a)                ∎
```

Note `1 − n k_a² > 0` is guaranteed by eq. (16): `k_a < 1/(1+1/ρ)` ⟹
`(1+1/ρ)k_a < 1` ⟹ `n k_a² < 1`. Without (16) the bound would be meaningless
(negative denominator).

### 2.7.2 The paper's proof — the two half-planes S₁, S₂ (eqs. 30–35)

The paper proves non-emptiness of the feasible gain set constructively, which
also produces the design charts (Figs. 5, 11). Both boundaries are **straight
lines** in the `(k_v, k_p)` plane.

**S₁ from (28a).** `k_v + h_w k_p ≤ (1 − n k_a²)/(2τ₀) =: A`. Put in intercept
form `k_v/a₁ + k_p/b₁ ≤ 1`:

```
    S₁ = {(k_v,k_p) : k_v>0, k_p>0, k_v/a₁ + k_p/b₁ ≤ 1}               (31)
    a₁ = (1 − (1+1/ρ)²k_a²)/(2τ₀)          [k_v-intercept]
    b₁ = (1 − (1+1/ρ)²k_a²)/(2τ₀ h_w)      [k_p-intercept]  = a₁/h_w
```

**S₂ from (33).** `2h_w k_v + h_w²k_p ≥ 2(1 − m k_a) =: 2B`. Divide by `2B`:

```
    S₂ = {(k_v,k_p) : k_v>0, k_p>0, k_v/a₂ + k_p/b₂ ≥ 1}               (34)
    a₂ = (1 − (1−1/ρ)k_a)/h_w              [k_v-intercept]
    b₂ = 2(1 − (1−1/ρ)k_a)/h_w²            [k_p-intercept]  = 2a₂/h_w
```

The feasible set is the wedge `S := S₁ ∩ S₂` (eq. 35).

**Non-emptiness ⟺ `h_w > h_w,lb`.** The paper's criterion is `a₁ ≥ a₂` or
`b₁ ≥ b₂`. Compute the first ratio:

```
    a₁     1 − n k_a²      h_w          h_w   1 − n k_a²
    ── = ───────────── · ─────────  =  ─── · ─────────────
    a₂       2τ₀         1 − m k_a      2τ₀    1 − m k_a
```

and since `h_w,lb = 2τ₀(1 − m k_a)/(1 − n k_a²)`,

```
    a₁/a₂ = h_w / h_w,lb                                               (♦)
```

**So `a₁ > a₂` ⟺ `h_w > h_w,lb` — the two proofs are the same statement.**
The paper writes "substituting h_w from (17), we have a₁/a₂ > 1. Thus S ≠ ∅",
and (♦) is why.

*Measured, Case 1:* `a₁ = 0.6400, a₂ = 0.6316, a₁/a₂ = 1.0133`, and
`h_w/h_w,lb = 0.95/0.9375 = 1.0133`. ✓ (`figs_design_space.m` prints both.)
*Case 3:* `a₁ = 0.8541, a₂ = 0.8470, ratio 1.0084 = 0.88/0.8727`. ✓

**Reading Fig. 5 (Case 1: k_a = 0.5, h_w = 0.95 s, ρ = 5, τ₀ = 0.5 s).**
The reproduced figure is `matlab/results/fig05_feasible_case1.png`:

```
    upper edge (28a): k_v-intercept 0.6400,  slope −h_w  = −0.95
    lower edge (28b): k_v-intercept 0.6316,  slope −h_w/2 = −0.475
    the two lines cross at ≈ (0.6235, 0.0175)
    the paper's chosen gains (0.63, 0.009) sit inside     ✓
```

The wedge is **tiny** — `k_v` is pinned to about a 1 % window, `[0.632, 0.640]`
at `k_p = 0`. That narrowness *is* the paper's design message: at h_w barely
above h_w,lb the feasible set collapses, and it opens up only as you buy more
headway (increase h_w and `a₁/a₂ = h_w/h_w,lb` grows). Design margin and road
capacity are directly traded.

---

## 2.8 Theorem 2(c): the optimal gain  (eqs. 36–42, 18, 19)

> **Theorem 2(c).** For any ρ > 1, the minimiser of `h_w,lb(k_a)` over
> `k_a ∈ (0, 1/(1+1/ρ))` and the resulting minimum are
>
> ```
>     k_a* = [(1 − 1/√ρ)/(1 + 1/√ρ)] · 1/(1 + 1/ρ)                     (18)
>     h*_w,lb = τ₀ (1 + 1/√ρ)²/(1 + 1/ρ)                               (19)
> ```

### Why there is an interior optimum at all

`h_w,lb(k_a) = 2τ₀(1 − m k_a)/(1 − n k_a²)` with `m = 1−1/ρ`, `n = (1+1/ρ)²`.

* **Increasing `k_a` helps** through the numerator: more feedforward means the
  feedback loop has less to do (the `1 − m k_a` term shrinks).
* **Increasing `k_a` hurts** through the denominator: more feedforward means
  the channel's *amplifying* excursion `(1+1/ρ)k_a` is larger, eating the
  (28a) budget (`1 − n k_a²` shrinks toward 0).

The optimum balances the two. Note it exists only because ρ is finite: as
`ρ → ∞`, `n → 1`, `m → 1` and the trade-off degenerates (Remark 4).

### The derivation

Write `h̄_w(k_a) := h_w,lb(k_a)/(2τ₀) = (1 − m k_a)/(1 − n k_a²)` (eq. 36).

**Quotient rule (eq. 37):**

```
    dh̄_w    −m(1 − n k_a²) − (1 − m k_a)(−2n k_a)
    ──── = ─────────────────────────────────────────
    dk_a                (1 − n k_a²)²

           −m + m n k_a² + 2n k_a − 2 m n k_a²      −m n k_a² + 2n k_a − m
         = ───────────────────────────────────── = ───────────────────────
                    (1 − n k_a²)²                      (1 − n k_a²)²
```

So the sign is that of `f(k_a) := −m n k_a² + 2n k_a − m` — the paper's
Fig. 4, reproduced as `matlab/results/fig04_f_of_ka.png`.

**Roots (the paper's `r_{1,2}`):** solving `m n k_a² − 2n k_a + m = 0`,

```
    k_a = [2n ± √(4n² − 4m²n)]/(2mn) = [n ∓ √(n(n − m²))]/(mn)   (choosing
                                                             r₁ = the − root)
```

**Simplify with `α := 1/ρ`,** so `m = 1 − α`, `n = (1 + α)²`, `√n = 1 + α`:

```
    n − m² = (1+α)² − (1−α)² = 4α
    √(n(n − m²)) = √((1+α)²·4α) = 2(1 + α)√α

           (1+α)² − 2(1+α)√α       (1+α)[(1+α) − 2√α]        (1 − √α)²
    r₁ = ───────────────────── = ───────────────────── = ─────────────────
             (1−α)(1+α)²             (1−α)(1+α)²          (1−α)(1+α)
```

and since `1 − α = (1 − √α)(1 + √α)`:

```
              (1 − √α)²                  1 − √α
    r₁ = ────────────────────────── = ─────────────────
         (1−√α)(1+√α)(1+α)            (1 + √α)(1 + α)
```

With `α = 1/ρ`, `√α = 1/√ρ`:

```
    k_a* = r₁ = [(1 − 1/√ρ)/(1 + 1/√ρ)] · 1/(1 + 1/ρ)                  (18) ✓
```

**Why `r₁` and not `r₂`.** `f` is a downward parabola (`−mn < 0`) with
`f(0) = −m < 0`, so `f < 0` on `(0, r₁)` and `f > 0` on `(r₁, r₂)`: `h̄_w`
*decreases* then *increases*, so `r₁` is the minimiser (eq. 38). One must also
check `r₁ < 1/(1+1/ρ) < r₂` so the minimiser is interior to the admissible
range. Numerically at ρ = 5 (`figs_design_space.m` prints these):
`r₁ = 0.31831`, `1/(1+1/ρ) = 0.83333`, `r₂ = 2.18170`. ✓

**Evaluate `h̄_w` at `r₁` (eq. 41).** Put `β := √α = 1/√ρ`, so
`k_a* = (1 − β)/[(1 + β)(1 + β²)]`, `m = 1 − β²`, `n = (1 + β²)²`:

```
                 (1 − β²)(1 − β)      (1−β)(1+β)(1−β)       (1 − β)²
    m k_a*  =  ─────────────────── = ─────────────────── = ──────────
                (1 + β)(1 + β²)       (1 + β)(1 + β²)        1 + β²

                       1 + β² − (1 − β)²      1 + β² − 1 + 2β − β²        2β
    1 − m k_a*  =  ───────────────────── = ─────────────────────── = ──────────
                          1 + β²                    1 + β²            1 + β²

                 (1 + β²)² (1 − β)²         (1 − β)²
    n k_a*² =  ───────────────────────── = ──────────
                (1 + β)²(1 + β²)²           (1 + β)²

                       (1 + β)² − (1 − β)²        4β
    1 − n k_a*² =  ───────────────────────── = ──────────
                          (1 + β)²              (1 + β)²

              2β/(1 + β²)        (1 + β)²
    h̄_w(r₁) = ───────────── = ───────────                              (41) ✓
              4β/(1 + β)²      2(1 + β²)
```

**Finally (eq. 42):**

```
    h*_w,lb = 2τ₀ h̄_w(r₁) = τ₀ (1 + β)²/(1 + β²) = τ₀ (1 + 1/√ρ)²/(1 + 1/ρ)
                                                                       (19) ✓
```

### Verifying internal stability at the optimum (eq. 43)

Theorem 2 must also deliver `γ > τ₀k_p` (F-14a). The paper's eq. (43):

```
    h_w k_p ≥ τ₀ k_p (1 + 1/√ρ)²/(1 + 1/ρ)  ⟹  γ = k_v + h_w k_p > τ₀ k_p
```

since `k_v > 0` and `h_w ≥ h*_w,lb = τ₀(1+1/√ρ)²/(1+1/ρ) ≥ τ₀` (the last step
because `(1+β)² ≥ 1 + β²` for `β ≥ 0`). So the headway needed for string
stability *already exceeds* what internal stability needs. **Internal
stability is never the binding constraint in this design family** — verified
numerically (`theorem_checks.m` block G) for all three cases: margins
`γ − τ₀k_p` of +0.634, +0.631, +0.851 with all poles in the open left
half-plane.

### Numerical values, ρ = 5, τ₀ = 0.5 s

| quantity | closed form | value | paper | our code |
|---|---|---|---|---|
| `k_a` cap, eq. (16)/(44) | `1/(1+1/ρ)` | 0.833333 | 0.8333 | 0.833333 ✓ |
| `h_w,lb(0.5)`, eq. (45) | `2τ₀(1−0.4)/(1−0.36)` = `0.6/0.64` | 0.937500 | 0.9375 | 0.937500 ✓ |
| `k_a*`, eq. (18) | `(1−0.4472)/[(1.4472)(1.2)]` | 0.318305 | 0.3183 | 0.318305 ✓ |
| `h*_w,lb`, eq. (19) | `0.5(1.4472)²/1.2` | 0.872678 | 0.8727 | 0.872678 ✓ |
| `E[w]`, eq. (12) | Table I | 1.048174 | — | 1.048174 |

`ka*` is also confirmed by a 400 001-point grid search on `h_w,lb`
(`theorem_checks.m` block C): grid minimiser 0.318304, closed form 0.318305.

---

## 2.9 The remarks — what each one is actually claiming

**Remark 2 (throughput).** Minimising `h_w,lb` is useful "because a lower time
headway can be employed while still guaranteeing robust string stability."
Quantified: platoon length `= N(d + h_w v_ss)`. At `N = 12`, `d = 5 m`,
`v_ss = 25 m/s`, going from `h_w = 0.95` to `h_w = 0.88` shortens the platoon
from **345 m to 324 m — 1.75 m per vehicle** (reproduced exactly; see
`fig13_platoon_length.png`). In flow terms, capacity `≈ v/(d + h_w v + L)`
vehicles per lane per second: shaving 0.07 s of headway is worth ≈6 % more
throughput at this speed.

**Remark 3 (ρ → ∞, no noise).** `k_a ∈ (0,1)` and
`h_w,lb(k_a) → 2τ₀/(1 + k_a)` — the classical noiseless bound of [9]. Verified:
`h_lb(k_a = 0.5, ρ = 10¹²) = 0.666667 = 2(0.5)/1.5` ✓.

**Remark 4 (monotonicity in ρ).** Three claims, all checkable:
* the lower bound on headway *increases as ρ decreases* — worse channel, more
  headway;
* the upper bound on `k_a` (`1/(1+1/ρ)`) is *smaller* than in the noiseless
  case (where it is 1) — noise costs you feedforward authority too;
* `ρ → ∞` gives `h*_w,lb → τ₀` and `k_a* → 1`. Verified:
  `k_a*(10¹²) = 0.999998`, `h*(10¹²) = 0.500001 = τ₀` ✓.

The `k_a* → 1` limit is worth a sentence: **with a clean channel you want to
feed forward as much as internal stability allows**, and the headway floor
becomes exactly one actuator lag. Every bit of headway above τ₀ is being paid
to the channel, not to the vehicle.

**Remark 5** — see §2.3.

---

## 2.10 The five numerical cases and what each demonstrates  (Sec. IV)

Common parameters (Sec. IV-A): `N = 12`, `τ₀ = 0.5 s`, `d = 5 m`, `ρ = 5`,
`v_ss = 25 m/s (90 km/h)`, `n = 16`, `τ = τ₀`, and the lead-vehicle manoeuvre

```
    a₀(t) = 0.5 sin(0.1(t − 10))   for 10 < t < 10 + 20π s,   else 0
```

**The manoeuvre is exactly one period** (`T = 2π/0.1 = 20π ≈ 62.83 s`), so it
starts and ends at `a₀ = 0` continuously — no impulsive content, which matters
because the certification is an L₂/frequency-domain statement. Its velocity
consequence is *not* small: `v₀(t) = v_ss + (0.5/0.1)(1 − cos(0.1(t−10)))`
rises to **35 m/s** at the half-period and returns to 25 m/s. So the desired
gap `d + h_w v` swings by `h_w × 10 = 9.5 m`, which is why spacing errors reach
O(1 m) in a string-stable platoon.

| case | gains | what it demonstrates | figures |
|---|---|---|---|
| 1 | `k_a=0.5, h_w=0.95, k_v=0.63, k_p=0.009` | `h_w > h_lb = 0.9375` ⟹ string stable; errors **decrease** down the string | 5, 6, 7, 8 |
| 2 | same gains, `h_w=0.65` | `h_w < h_lb` ⟹ string **unstable**; errors **increase** | 9, 10 |
| 3 | `k_a=k_a*=0.3183, h_w=0.88, k_v=0.85, k_p=0.003` | operating at the optimum: stable **and** shorter | 11, 12 |
| 4 | cases 1 vs 3 | platoon length `x₀ − x_N` — the throughput payoff | 13 |
| 5 | case-1 gains, heterogeneous `τ_i` (Table II) | the `τ ∈ (0,τ₀]` robustness claim is real | 14, 15 |

### Reproduction status (Simulink backend, this project)

Full table in [`../validation/V-05-simulink-reproduction.md`](../validation/V-05-simulink-reproduction.md).
Headline:

| quantity | paper | Simulink | rel. err |
|---|---|---|---|
| `h_w,lb`, `k_a*`, `h*`, `k_a` cap | 0.9375 / 0.3183 / 0.8727 / 0.8333 | identical | **0.00 %** |
| Case 1 `max|δ₁|` → `max|δ₁₂|` | 1.593 → 1.535 | 1.5959 → 1.5436 | 0.18 % / 0.56 % |
| Case 2 `max|δ₁|` → `max|δ₁₂|` | 0.882 → 0.886 | 0.8821 → 0.8872 | **0.02 % / 0.13 %** |
| Case 4 platoon length, `h=0.95 / 0.88` | 345 / 324 m | 345.00 / 324.00 m | 0.00 % |
| Case 4 peak length | ≈437 / ≈418 m | 436.72 / 416.67 m | ≈0.1 % |
| Case 5 `max|a_i|`, i = 1 → 12 | ≈0.49 → ≈0.455 | 0.4962 → 0.4523 | ≈1 % |
| Case 3 `max|δ₁|` → `max|δ₁₂|` | 0.802 → 0.798 | 0.9256 → 0.9208 | **15.4 %** ⚠ |

Case 2 matching to 0.02 % **including the sign of the trend** (errors growing
down the string) is the strongest single validation: it is the paper's
string-*instability* demonstration, and reproducing a 0.5 % growth over 11 hops
to two significant figures is only possible if the transfer function, the
manoeuvre, the initial conditions and the integrator are all right.

The Case 3 discrepancy is open. It is **not** a model error: the Simulink model
and this project's independent Python core (87 passing tests) agree to four
decimals on all three cases under an ideal channel
(`matlab/tests/crosscheck_python.py`) —

```
    Case 1: 1.2826 → 1.2442   (both backends, identical)
    Case 2: 1.1953 → 1.2057   (both backends, identical)
    Case 3: 0.7604 → 0.7566   (both backends, identical)
```

A parameter sweep (`V-05` §4) shows `max|δ|` in Case 3 is strongly sensitive to
`h_w` (0.8727 s → 0.858; 0.88 s → 0.926) and weakly sensitive to `k_p`
(0.003 → 0.926; 0.005 → 0.895), so no single stated parameter reproduces
0.802. Reported as a reproduction note rather than resolved.

---

## 2.11 What the paper does *not* claim — the honest boundary

Be ready for "what are the limitations?" Each of these is a real gap, not a
quibble:

1. **Mean-sense only.** The certified object is `E[X(t)]` (§2.3). No
   sample-path or probabilistic (e.g. "with probability ≥ 0.99") guarantee is
   made. This project's chance-constrained certificate (D-023) exists to fill
   exactly this gap.
2. **No communication delay.** The channel corrupts amplitude, not timing.
   `H̃` has no `e^{−θs}`. Köroğlu (Part IV) shows delay alone drives the
   minimum headway; this project's extension (D-008) finds delay, not noise,
   is the binding constraint at realistic DSRC latencies — beyond ≈0.2 s of
   delay the required headway explodes.
3. **No packet loss.** Perfect delivery is assumed. Ploeg's graceful
   degradation (Part III §3.7) is the standard treatment.
4. **Sufficient, not necessary, conditions** (§2.6.2), so `h_lb` is
   conservative for fixed gains.
5. **One-vehicle look-ahead, homogeneous, linear.** Saturation is outside the
   certified envelope; the paper's manoeuvre (`|a₀| ≤ 0.5 m/s²`) stays far from
   actuator limits. Emergency braking would exercise them and is not covered.
6. **`γ_{i,j}` time-invariant.** Stated as an assumption after eq. (5): "the
   characteristics of the noise processes are time-invariant." A channel whose
   quality varies with position or traffic (an interference zone) violates it
   — the premise of this project's QoS-adaptive extension (D-016).
7. **Radar assumed clean.** Only the V2V path is noisy. Sensor noise on
   `δ_i`, `v_i − v_{i−1}` is not modelled, and Köroğlu explicitly flags noise
   sensitivity of the feedback loop as the price of approaching minimum
   headway.

---

## 2.12 Whiteboard drill — reproduce these from blank paper

1. Derive `H̃(s) = Ñ/D` from (13). *(§2.4, four lines.)*
2. Expand `|D(jω)|² − |Ñ(jω)|²` and obtain (25). *(§2.6.1.)*
3. Show `(26b)` collapses to the **linear** inequality (33). *(§2.5.2 step 1 —
   the `k_v²` cancellation.)*
4. Derive `h_w > 2τ₀/(1 + k̃a)`. *(§2.5.2.)*
5. Derive `h_w,lb` (17) and explain which end of `I` each condition binds at.
   *(§2.7.1, §2.6.3.)*
6. Differentiate `h̄_w`, get `f(k_a)`, solve for `r₁`, simplify to `k_a*`.
   *(§2.8.)*
7. Evaluate `h̄_w(r₁)` and get `h* = τ₀(1+1/√ρ)²/(1+1/ρ)`. *(§2.8.)*
8. Show `a₁/a₂ = h_w/h_w,lb`, hence non-emptiness ⟺ (17). *(§2.7.2.)*
9. State why `k̃a < 1` and why the argument needs `τ → 0⁺`. *(§2.5.1.)*
10. Give a design that fails (28) but is string stable, and explain why.
    *(§2.6.2.)*

---

*Next: [Part III — Ploeg: L_p string stability, H∞ synthesis, graceful
degradation](03-ploeg-string-stability.md).*
