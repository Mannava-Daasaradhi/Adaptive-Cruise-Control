# Part V — Zhao et al. 2024: resilient control against false data injection

> C. Zhao, R. Ma, M. Wang, J. Xu, L. Cai, "Safeguard Vehicle Platooning Based
> on Resilient Control Against False Data Injection Attacks," *IEEE Trans.
> Intell. Transp. Syst.* **25**(11):17023–17036, Nov. 2024.
> doi 10.1109/TITS.2024.3424687

**The question.** Ma asks what *random* corruption of the V2V payload costs.
Zhao asks what *adversarial* corruption costs — and how to survive it without
cryptography.

**The shift in setting.** Everything changes relative to Parts II–IV, and you
should be able to list the changes before touching the maths:

| | Ma / Ploeg / Köroğlu | Zhao |
|---|---|---|
| topology | one-vehicle look-ahead (a chain) | **general graph** 𝒢 = (𝒱, ℰ), multiple neighbours, plus a pinned leader |
| spacing | CTH, predecessor-following | CTH via **consensus** over the graph |
| time | continuous | **discrete**, sample ξ |
| impairment | stochastic / delay | **adversarial**, worst case, bounded in *number* not magnitude |
| analysis tool | frequency domain, `‖·‖_{H∞}` | **spectral radius of a time-varying state matrix** + graph robustness |
| main result | minimum headway | conditions for **asymptotic stability under attack** |
| string stability | proved | **explicitly left open** (§V) |

The frequency-domain machinery of Parts II–IV **does not apply**: the resilient
filter is nonlinear (it sorts and discards), so the closed loop is time-varying
and `s`-domain methods "are inapplicable" (their words).

---

## 5.1 Plant and its discretisation  (eqs. 1, 2)

Continuous, identical to F-1 with `s_i = (q_i, v_i, a_i)ᵀ`:

```
    ṡ_i(t) = [0 1  0 ] s_i(t) + [ 0 ] u_i(t)                            (1)
             [0 0  1 ]          [ 0 ]
             [0 0 −1/τ]         [1/τ]
```

Discretised at sample time ξ (eq. 2), `s_i(k+1) = A s_i(k) + B u_i(k)`:

```
    A = [1  ξ  ξ²/2  ]        B = [ 0  ]
        [0  1  ξ     ]            [ 0  ]
        [0  0  1−ξ/τ ]            [ξ/τ ]
```

**What kind of discretisation is this?** Not the exact matrix exponential
(that would put `e^{−ξ/τ}` in the (3,3) entry and messier terms above it). It
is a **constant-acceleration hold on the kinematics** (`q += ξv + ξ²a/2`,
`v += ξa`, both exact if `a` is held) combined with **forward Euler on the lag**
(`1 − ξ/τ ≈ e^{−ξ/τ}`). Standard in the platooning literature.

Consequence worth stating: the `a`-update is stable only for `|1 − ξ/τ| < 1`,
i.e. `0 < ξ < 2τ`. With the paper's `ξ = 0.01 s`, `τ = 0.5 s`, the factor is
0.98 — comfortably inside. Remark 4 of the paper leans on this: "it is better
to use a smaller time sampling time ξ", because (as §5.6 shows) the attack
bound scales like `ξ/τ`.

---

## 5.2 Network model and the consensus law  (eqs. 3–5)

### Graph objects

`𝒢 = (𝒱, ℰ)` over the **followers** `𝒱 = {1,…,N}`; `(j,i) ∈ ℰ` means *i can
obtain j's information*. Adjacency `𝒜 = [a_{ij}]`, `a_{ij} = 1 ⟺ (j,i) ∈ ℰ`,
no self-loops. In-degree `d_i = Σ_j a_{ij}`, `𝒟 = diag(d₁,…,d_N)`, and the
**Laplacian**

```
    ℒ = 𝒟 − 𝒜
```

The leader (index 0) is handled by the **pinning matrix** `P = diag(p₁,…,p_N)`
with `p_i = 1` iff `(0,i) ∈ ℰ̃` — i.e. iff vehicle *i* hears the leader
directly. `ℙ_i = {0}` if `p_i = 1`, else `∅`.

Standard facts you should be able to state:
* `ℒ 1 = 0` — the all-ones vector is always in the kernel, so `ℒ` alone can
  never pin down absolute position; only *differences* are observable.
* `ℒ_P := ℒ + P` is nonsingular (and positive-stable) iff every follower has a
  directed path from the leader. **The pinning matrix is what breaks the
  kernel** and turns relative consensus into absolute tracking.

### Control objective  (eq. 3)

```
    lim_{k→∞} ‖q_i − q_{i−1} − d_{i,i−1}‖ = 0
    lim_{k→∞} ‖v_i − v₀‖ = 0
    lim_{k→∞} ‖a_i − a₀‖ = 0
```

with `d_{i,i−1}(k) = −(d + h v_i(k))` — the CTH policy again (F-4), sign as in
Zhao's `q_i − q_{i−1}` convention (i.e. Ma's, F-6).

### Desired spacing between *arbitrary* pairs  (eq. 4)

Because the topology is a general graph, vehicle *i* may hear vehicle *j* that
is several places away. The desired displacement telescopes:

```
                 ⎧ − Σ_{n=j+1}^{i} (d + h v_n(k)) ,   j < i
    d_{i,j}(k) = ⎨
                 ⎩ + Σ_{n=i+1}^{j} (d + h v_n(k)) ,   j > i             (4)
```

i.e. the sum of the desired gaps of every vehicle between them. This is the
step that generalises CTH from a chain to a graph, and it is why the closed
loop later contains a **lower-triangular matrix `H` with every entry on and
below the diagonal equal to `h`** (their eq. 8) — `H` is exactly the
"telescoping" operator.

### The consensus controller  (eq. 5)

```
    u_i(k) = − Σ_{j ∈ 𝕀_i} [ κ_q(q_i − q_{i,j} − d_{i,j})
                            + κ_v(v_i − v_{i,j})
                            + κ_a(a_i − a_{i,j}) ]                      (5)
    𝕀_i = ℕ_i ∪ ℙ_i
```

`q_{i,j}, v_{i,j}, a_{i,j}` = what *i* **received** from *j*. In the clean case
these equal `q_j, v_j, a_j`.

**Note the contrast with Parts II–IV**: there is no explicit feedforward of
the predecessor's acceleration with a separate gain; instead `κ_a(a_i − a_{i,j})`
is an *acceleration-consensus* term. This is a fundamentally different control
architecture, which is why none of Ma's closed forms transfer.

---

## 5.3 The attack model  (eq. 6, Definition 1)

```
    q_{i,j}(k) = q_j(k) + δ^q_{i,j}(k)
    v_{i,j}(k) = v_j(k) + δ^v_{i,j}(k)                                  (6)
    a_{i,j}(k) = a_j(k) + δ^a_{i,j}(k)
```

`δ_{i,j} = [δ^q, δ^v, δ^a]ᵀ` **can be any value** — unbounded in magnitude.

> **Definition 1 (F-total model).** The set of malicious nodes `M ⊂ 𝒱`
> satisfies `|M| ≤ F`, `F ∈ ℤ_{≥0}`.

**This is the crucial modelling choice.** The adversary is bounded in
*cardinality*, not in *magnitude*. Compare with Ma, where the corruption is
bounded in magnitude (`|w−1| ≤ 1/ρ`) but present on *every* link. The two
threat models are complementary:

| | Ma 2025 | Zhao 2024 |
|---|---|---|
| how many links corrupted | **all** | at most **F** |
| how badly | `|w − 1| ≤ 1/ρ`, bounded | **unbounded** |
| adversary | nature (thermal noise, quantisation) | intelligent attacker |
| defence | headway + gain design | **filtering** (discard outliers) |

Zhao's Remark 1 justifies the cardinality bound: mounting an FDIA requires
penetration testing, topology discovery, and protocol reverse-engineering per
node — "adversaries must undertake a series of penetration tests" — so
compromising *many* nodes is much harder than compromising one. They also note
the model covers compromised *radars* (mmWave spoofing is demonstrated in the
literature), so "our model also works in the case that the adversary
compromises both communication links and local radars."

**Assumption 2 (leader is trusted).** "The leading vehicle's information is
always trustable and safe." Justified by concentrating security/authentication
effort on the single most valuable node. This is a real assumption, not a
technicality — if the leader can be spoofed, no amount of graph robustness
helps, because every honest vehicle is tracking a lie.

---

## 5.4 Algorithm 1: the resilient filter (MSR) and its key bound

### The algorithm

```
    1.  receive s_{i,j}(k) from all j ∈ ℕ_i
    2.  form tracking-error vectors  s̃_{i,j}(k) = s_{i,j}(k) − (reference)
        and compute the norms ‖s̃_{i,j}(k)‖ ; sort them
    3.  DISCARD the F neighbours with the LARGEST norms → ℕ_{i,rem}
    4.  compute ū_i(k) with (5) restricted to 𝕀_{i,rem} = ℕ_{i,rem} ∪ ℙ_i
    5.  s_i(k+1) = A s_i(k) + B ū_i(k)
```

This is a **Mean-Subsequence-Reduced (MSR)** filter, the standard tool from
resilient distributed consensus, applied here to 3-vectors via their norms
rather than to scalars via ordering.

Cost per vehicle per step: three vector subtractions, `m` norm evaluations, a
sort — `𝒪(m log m)` for `m` neighbours. "Thus, the running time of Algo. 1 is
short, making it easily deployable." No cryptography, no keys, no
authentication handshake. **That lightness is the paper's selling point**: a
safety-critical, hard-real-time control loop cannot afford per-message
signature verification.

### The bound that makes it work  (eq. 18)

> **Claim.** After filtering, every surviving injection satisfies
>
> ```
>     ‖δ_{i,j}(k)‖ ≤ max_j { ‖x_i(k) − x_j(k)‖ } ,     ∀ j ∈ ℕ_i        (18)
> ```

**Proof (the MSR argument — know this).** At most `F` of `i`'s neighbours are
malicious (Definition 1). Vehicle *i* discards the `F` largest norms. Take any
surviving neighbour `j`:

* **Case A — `j` is honest.** Then `δ_{i,j} = 0` and the bound holds trivially.
* **Case B — `j` is malicious and survived.** Then all `F` discarded entries
  have norms `≥ ‖s̃_{i,j}‖`. Since at most `F` neighbours are malicious and `j`
  is one of them, at least one discarded entry belonged to an **honest**
  neighbour `j'`. Hence `‖s̃_{i,j}‖ ≤ ‖s̃_{i,j'}‖`, and `s̃_{i,j'}` is a genuine
  state difference. So the surviving malicious value is bounded by the spread
  of *legitimate* states. ∎

**The idea in one sentence:** *you cannot detect the attack, but you can force
the attacker to look ordinary — and ordinary is bounded by how far apart the
honest vehicles already are.* An attacker who injects a large lie gets
discarded; one who injects a small lie is, by construction, no worse than the
existing disagreement, which the consensus loop is already driving to zero.

This is why the closed-loop bound (§5.6) has `‖C(k)δ(k)‖ ≤ (…)‖x(k)‖`: the
attack becomes a **state-proportional perturbation**, not an exogenous input.
That is what makes asymptotic stability (rather than mere boundedness)
achievable.

### Why the graph must be robust  (Definition 2)

> **Definition 2 ((r,s)-robust).** A directed graph 𝒢 is (r,s)-robust
> (`r, s < N`) if for every pair of nonempty disjoint subsets `𝒮₁, 𝒮₂ ⊂ 𝒱`, at
> least one of: 1) `𝒳^r_{𝒮₁} = 𝒮₁`; 2) `𝒳^r_{𝒮₂} = 𝒮₂`; 3)
> `|𝒳^r_{𝒮₁}| + |𝒳^r_{𝒮₂}| ≥ s`, where `𝒳^r_{𝒮_i}` is the set of nodes in
> `𝒮_i` with at least `r` incoming edges from outside `𝒮_i`.

**Why this is needed and not just "connected".** Discarding `F` neighbours
throws away information. If the graph is merely connected, a vehicle might have
exactly `F+0` neighbours and discard *all* of them, becoming isolated. Robustness
guarantees that **enough redundancy survives the discarding** for information
to still flow across every possible cut.

Theorem 1 requires an `F+1`-robust graph. The paper's operational reading:
"to ensure that the information of the leader can be transmitted to the
following vehicles, it is necessary to have no less than `F + 1` following
vehicles communicate directly with the leader vehicle."

Their two example topologies (§VI-B):
* `ℒ₁` — bidirectional, each vehicle talks to its two nearest neighbours front
  and back: **(2,2)-robust**;
* `ℒ₂` — unidirectional, two nearest predecessors: **2-robust**.

and two pinning choices `P₁ = diag(1,1,1,1,…)`, `P₂ = diag(1,1,0,0,…)`.

---

## 5.5 The closed loop  (eqs. 7–13)

Tracking errors relative to the leader (eq. 7):

```
    q̃_i = q_i − q₀ − d_{i,0} ,   ṽ_i = v_i − v₀ ,   ã_i = a_i − a₀
    d_{i,0}(k) = −Σ_{n=1}^{i}(d + h v_n(k))
```

Stacking `x(k) = [q̃ᵀ, ṽᵀ, ãᵀ]ᵀ ∈ ℝ^{3N}` and substituting Algorithm 1's `ū`
into (2) gives (eq. 11):

```
    x(k+1) = W(k) x(k) + C(k) δ(k)                                      (11)
```

with

```
              ⎡    I_N          ξI_N         (ξ²/2)I_N + ξH        ⎤
    W(k)  =   ⎢    0_N          I_N              ξI_N              ⎥
              ⎣ −(ξ/τ)κ_q ℒ_P  −(ξ/τ)κ_v ℒ_P  (1−ξ/τ)I_N − (ξ/τ)κ_a ℒ_P ⎦

              ⎡      0                0               0        ⎤
    C(k)  =   ⎢      0                0               0        ⎥
              ⎣ −(ξ/τ)κ_q ℒ̄  −(ξ/τ)κ_v ℒ̄  −(ξ/τ)κ_a ℒ̄ ⎦
```

`H` is the lower-triangular matrix of `h`'s from the telescoped spacing (4);
`ℒ_P(k) = ℒ(k) + P` is the **pinned Laplacian of the graph that survives
filtering at step k** (hence time-varying); `ℒ̄(k) ∈ ℝ^{N×|ℰ|}` maps per-edge
injections into per-node effects.

**Two structural facts to notice:**

1. `W(k)` is time-varying **because the filter changes which edges are used at
   each step**. The nominal (unfiltered) matrix is `W`, with the fixed
   Laplacian `ℒ`; the filtered one differs by `W'(k) := W(k) − W`, which has
   nonzeros only in the bottom block row (their displayed `W'(k)`).
2. `C(k)` also has nonzeros only in the bottom block row — the attack enters
   **only through the acceleration channel**, scaled by `ξ/τ`. Smaller sample
   time ⟹ smaller per-step attack authority. This is Remark 4.

---

## 5.6 Stability: Lemmas 3–5 and Theorem 1

### Lemma 3 — the nominal Schur condition  (eqs. 14, 15)

> `x(k+1) = W x(k)` is asymptotically stable **iff**
> `max_{i=1..3N} |λ_i(W)| < 1`.                                        (14)

This is elementary discrete-time LTI stability: the spectral radius must be
inside the unit circle. The paper then gives per-vehicle Jury-type conditions
on the characteristic cubic `λ³ + a₂λ² + a₁λ + a₀` with (eq. 15)

```
    a₂ = ξ(κ_a r_i + 1)/τ − 3
    a₁ = ξ³κ_q r_i/(2τ) + ξ²κ_a r_i/τ + ξ²κ_v r_i/τ − 2ξκ_a r_i/τ − 2ξ/τ + 3
    a₀ = 1 + ξ/τ + ξκ_a r_i/τ − ξκ_v r_i/τ + ξ³κ_q r_i/(2τ) − ξ²κ_q h r_i/τ
```

where `r_i` is the number of vehicle *i*'s neighbours.

**Cross-check the algebra you can trust.** For a monic cubic
`P(λ) = λ³ + a₂λ² + a₁λ + a₀`, the standard **Jury** conditions for all roots
inside the unit disk are

```
    P(1) > 0            ⟺  1 + a₂ + a₁ + a₀ > 0
    −P(−1) > 0          ⟺  1 − a₂ + a₁ − a₀ > 0
    |a₀| < 1
    |a₀² − 1| > |a₀a₂ − a₁|
```

The first two combine to `|a₂ + a₀| < 1 + a₁` — the paper's first condition ✓.
The fourth, using `|a₀| < 1`, gives `1 − a₀² > |a₀a₂ − a₁|`, one half of which
is `a₀² + a₁ − a₀a₂ − 1 < 0` — the paper's third condition ✓.
The paper's middle condition is printed as `|a₂ − 3a₀| < 2 − a₁`. Deriving the
same test by the bilinear map `λ = (1+w)/(1−w)` and Routh (F-14) on the
resulting `w`-cubic gives coefficients
`b₃ = 1−a₂+a₁−a₀`, `b₂ = 3−a₂−a₁+3a₀`, `b₁ = 3+a₂−a₁−3a₀`, `b₀ = 1+a₂+a₁+a₀`,
whose positivity yields `|a₂ − 3a₀| < 3 − a₁`. **The two differ (3 vs 2)** —
check the printed inequality against the source before relying on it; the
Jury form above is the one to use.

### Lemma 4 — perturbation of a stable difference equation

> If `x(k+1) = Wx(k)` is asymptotically stable, then
> `x(k+1) = (W + W'(k))x(k)` is asymptotically stable provided there exist
> `α > 0` and sufficiently small `ε > 0` with
> `Σ_{j=k₀}^{k−1} ‖W'(k)‖ ≤ ε(k − k₀) + α`, ∀ k ≥ k₀, ∀ k₀ ≥ 0.

I.e. the perturbation must have **bounded average magnitude**. This is a
standard slowly-varying / small-perturbation result; the `ε(k−k₀) + α` form
allows a bounded transient (`α`) plus a small sustained rate (`ε`).

### Lemma 5 — applied to the filtered platoon  (eq. 16)

Since `W'(k)` has nonzeros only in the bottom block row,

```
    W'(k)W'(k)ᵀ = diag( 0, 0, (ξ²/τ²)(κ_q² + κ_v² + κ_a²) ℒ'_P(k)ℒ'_P(k)ᵀ )
    ⟹ ‖W'(k)‖ = (ξ/τ)√(κ_q² + κ_v² + κ_a²) · ρ(ℒ'_P(k))
```

so Lemma 4's hypothesis becomes (eq. 16)

```
    (ξ/τ)√(κ_q² + κ_v² + κ_a²) Σ_{j=k₀}^{k−1} ρ(ℒ'_P(k)) ≤ ε(k−k₀) + α
```

with `ℒ'_P = −ℒ_P(k) + ℒ_P` (the edges removed by the filter).

**Remark 3's reading**: for an undirected surviving graph with at most `F̄`
compromised links, `max|λ_i(ℒ'_P)| ≤ F̄` by a degree bound
(`max{d'_i + d'_j − |N'_i ∩ N'_j|}`), so (16) reduces to

```
    (ξ/τ)√(κ_q² + κ_v² + κ_a²) · F̄  ≤  ε + α/(k − k₀)
```

**The design lever is now explicit**: the product
`(ξ/τ)·‖(κ_q, κ_v, κ_a)‖·F̄` must be small. You can shrink it by taking a
**smaller sample time**, **smaller gains**, or **fewer tolerated attackers**.
Remark 4: "the control gains, the time sampling interval and the inertial delay
are adjustable parameters... it is better to use a smaller time sampling
time ξ."

### Theorem 1 and the Grönwall–Bellman finish  (eqs. 17–20)

> **Theorem 1.** Under Algorithm 1, Assumptions 1 and 2, and an `F+1`-robust
> graph 𝒢, all vehicles achieve stability under attacks if (14) holds and
> there exist `α > 0` and small `ε > 0` such that (16) holds and
> `√( F̄ (ξ²/τ²)(κ_q² + κ_v² + κ_a²) )` is small enough.

**Proof sketch — the chain to know.**

1. **Bound the attack by the state.** From (18),
   ```
       ‖C(k)δ(k)‖² ≤ F̄(ξ²/τ²)(κ_q²+κ_v²+κ_a²) max_{i,j}{‖x_i−x_j‖²}
                   ≤ F̄(ξ²/τ²)(κ_q²+κ_v²+κ_a²) max_{i,j}{‖x_i‖²+‖x_j‖²}
                   ≤ F̄(ξ²/τ²)(κ_q²+κ_v²+κ_a²) β(k)‖x(k)‖²
   ```
   with `β(k) = max{‖x_i‖²+‖x_j‖²}/‖x(k)‖² ∈ (0,1]`. **The attack has become
   a state-proportional perturbation** — this is the payoff of §5.4.

2. **Variation of constants.** With `Φ(k,k₀) = ∏_{i=0}^{k−k₀−1}W(k₀+i)`,
   ```
       ‖x(k)‖ ≤ ‖Φ(k,k₀)‖‖x(k₀)‖ + Σ_{i=k₀}^{k−1}‖Φ(k,i+1)‖‖C(i)δ(i)‖
   ```

3. **Exponential bound on the nominal transition.** From Lemma 5 there exist
   `ϕ > 0`, `φ ∈ (0,1)` with `‖Φ(k,k₀)‖ ≤ ϕφ^{k−k₀}`, giving
   ```
       ‖x(k)‖ ≤ ϕφ^{k−k₀}‖x(k₀)‖ + Σ ϕφ^{k−i−1}(ξ/τ)√(F̄(κ_q²+κ_v²+κ_a²))‖x(i)‖
   ```

4. **Grönwall–Bellman** on that inequality (eq. 19):
   ```
       ‖x(k)‖ ≤ ϕφ^{k−k₀}‖x(k₀)‖ exp( Σ_{i=k₀}^{k−1} (ϕξ)/(φτ)√(F̄(κ_q²+κ_v²+κ_a²)) )
   ```

5. **Collect the geometric factor** (eq. 20):
   ```
       ‖x(k)‖ ≤ ϕ [ φ · exp( (ϕξ)/(φτ)√(F̄(κ_q²+κ_v²+κ_a²)) ) ]^{k−k₀} ‖x(k₀)‖
   ```
   Convergence requires the bracket `< 1`, i.e.
   ```
       φ · exp( (ϕξ)/(φτ)√(F̄(κ_q²+κ_v²+κ_a²)) ) < 1
   ```
   which holds **iff `√(F̄(ξ²/τ²)(κ_q²+κ_v²+κ_a²))` is small enough**, since
   `φ < 1` leaves an exponential budget of exactly `ln(1/φ)`. ∎

**The result in words.** *A nominally-stable consensus platoon on an
`F+1`-robust graph survives up to `F` arbitrary liars, provided the product of
sample time, gain magnitude and attacker count stays below a margin set by the
nominal contraction rate `φ`.*

Note **Lemma 2**'s converse: "under attacks, if `lim_{k→∞}x(k) = 0`, then
there must hold `lim_{k→∞}δ(k) = 0`." Successful resilient control forces the
attack to become asymptotically ineffective — an attacker who wants to stay
undetected must eventually stop lying.

---

## 5.7 String stability — what the paper does *not* prove

Definition 3 defines ℓ_p string stability for the attacked platoon: there exist
a class-𝒦 function `α` and constants `c, c_ω, κ_ω > 0` such that for any
initial disturbance `e_{q1}(0)` and new disturbance `a₀(k)` with
`|e_{q1}(0)| < c`, `‖a₀(k)‖_{ℓ_∞} < c_ω`,

```
    ‖e_{qi}(k)‖_{ℓ_p} ≤ α(|e_{q1}(0)|) + κ_ω c_ω ,     ∀ i ∈ 𝒱, ∀ k ≥ 0
```

The paper explains why ℓ_p is the right choice here (and this is a good short
answer to "which string-stability definition should I use?"):

* **Lyapunov-like** string stability "only considers the response to initial
  condition disturbances, neglecting external disturbance" — inadequate,
  because the leader's acceleration is the disturbance that matters.
* **Input-to-output-like** (`‖e_{qi}‖_∞ ≤ ‖e_{q(i−1)}‖_∞`) "only applies to
  linear systems with zero initial conditions" — inadequate, because the
  resilient filter makes the system nonlinear.
* **ℓ_p** is input-to-state-like: it captures "1) boundedness of state
  fluctuations; 2) convergence of state fluctuations caused by initial
  condition disturbances; and 3) boundedness and convergence hold for any
  platoon length." Also, "a key feature of resilient control is that the
  topology is time-varying, which also makes the s-domain methods
  inapplicable."

Then the honest admission (their §V, verbatim):

> "The challenges for further analysis lie in: 1) We have two kinds of
> disturbances in the platoon system, i.e., the disturbance from the designed
> resilient control algorithm and that caused by the acceleration of the
> leading vehicle, which makes the string stability hard to analyze; 2) Since
> the disturbance from the designed resilient control algorithm does not have
> an analytical expression, it is difficult to further analyze the string
> stability of the system. We admit that we may require extra input design to
> guarantee string stability. String stability guaranteed controller design and
> theoretical analysis remains a challenging open issue."

**Be precise about this if asked.** Zhao et al. prove **asymptotic stability
under attack** (`x(k) → 0`), not string stability. The two are different: a
platoon can converge while amplifying disturbances along the string. The
frequency-domain tools that would settle it are unavailable because the filter
is nonlinear and the effective topology is time-varying.

---

## 5.8 Numerical and experimental results  (§§VI, VII)

Parameters: `N = 6` followers, `d = 20 m`, `h = 0.4 s`, `τ = 0.5 s`,
`ξ = 0.01 s`, `κ_q = 2`, `κ_v = 4`, `κ_a = 2`; each vehicle communicates with
its two nearest neighbours and the leader.

**Without attack** (their Fig. 1): platoon formation from random ICs, emergency
brake (`a₀ = −10 m/s²` at t = 5 s, all stop within 20 s without collision), and
disturbance resistance (`a₀ = ±2 m/s²` pulses).

**Without resilient control, under node attack** (their Fig. 2): with a fixed
injected vector `δ = [15, 10, 5]ᵀ`, "the platoon without resilient control
fails to maintain the desired inter-vehicle distance and even crashes under
attacks" — the spacing errors cross the collision boundary at −30 m. With a
*random* small injection, no crash but "spacing errors persistently fluctuate,
failing to converge to zero."

**With Algorithm 1** (their Fig. 3): all four scenarios S1–S4 achieve stable
formation. Findings worth quoting:
* "the convergence speed of the undirected topology is slower than that of the
  one-way topology" — bidirectional links propagate the *filtered-out*
  information both ways, slowing agreement;
* "the more vehicles that directly communicate with the leader, i.e., larger
  `|𝒫|`, the faster the convergence speed" — pinning more nodes helps;
* vehicles 1, 3, 4 show acceleration fluctuation because they neighbour the
  compromised vehicle 2;
* scaling to `N = 20` preserves stability.

**Hardware** (§VII): four TurtleBot3s. Real robots, real wireless, real
filtering — the resilient controller is not just a simulation artefact.

---

## 5.9 Relation to this project's spoof-defence gate (D-022)

This project implements a different defence against the same threat, and the
contrast is worth being able to draw.

| | Zhao's MSR filter | This project's physics-consistency gate (D-022) |
|---|---|---|
| principle | **redundancy** — discard the `F` most extreme of `m` neighbours | **plausibility** — compare the V2V-claimed acceleration against what the radar says the predecessor is actually doing |
| requires | `F+1`-robust graph, `m > F` neighbours | only the ego vehicle's own radar |
| topology | general graph | works in a plain **one-vehicle look-ahead chain**, where MSR is impossible (`m = 1`, so discarding one neighbour leaves nothing) |
| output | hard include/exclude | continuous **trust weight** `g ∈ [0,1]` that de-rates the feedforward gain |
| failure mode | an attacker who stays within the honest spread is admitted (bounded by (18)) | an attacker who spoofs *consistently with physics* is admitted |
| relation to Ma | orthogonal — different controller family | direct: `g` multiplies `k_a`, so a gated link degrades gracefully toward ACC |

**The key structural point:** MSR needs redundancy, and the predecessor-following
chain that Ma, Ploeg and Köroğlu all analyse has **none** — each vehicle has
exactly one V2V source. So Zhao's defence is unavailable in the base paper's
topology, and a physics-consistency check on the single link is the natural
substitute. Conversely, Zhao's defence needs no second sensor. The two are
complementary, and a deployed system would want both.

---

## 5.10 Whiteboard drill

1. Write the discretised model and say what kind of discretisation it is and
   what it requires of `ξ`.
2. Define `ℒ`, `P`, `ℒ_P`; explain why `ℒ` alone cannot achieve absolute
   tracking.
3. State the F-total model and contrast it with Ma's magnitude-bounded noise.
4. State Algorithm 1 and **prove eq. (18)** (the two-case MSR argument).
5. Explain why `F+1`-robustness (not mere connectivity) is required.
6. Write the closed loop `x(k+1) = W(k)x(k) + C(k)δ(k)` and say why both `W'`
   and `C` have nonzeros only in the bottom block row.
7. State the Jury conditions for a cubic and check the paper's eq. (15) form.
8. Run the proof chain: (18) → state-proportional bound → variation of
   constants → Grönwall–Bellman → geometric factor `< 1`.
9. Say precisely what Theorem 1 proves and what it does **not** (string
   stability).
10. Explain why ℓ_p string stability is the right definition here and why
    `s`-domain methods do not apply.

---

*Next: [Part VI — Ke 2022: CACC by deep reinforcement learning](06-ke-deep-learning-cacc.md).*
