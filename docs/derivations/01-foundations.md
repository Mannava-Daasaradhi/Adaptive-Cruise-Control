# Part I — Foundations

*Everything the six papers assume you already know, derived rather than
asserted. Nothing here is quoted from a paper without being rebuilt from
something more primitive.*

Read this once. After that, Parts II–VI only ever call back to a numbered
result here (F-1 … F-24), so you never have to re-derive a prerequisite in
the middle of a proof.

---

## 1. The longitudinal vehicle model

### F-1. Why the model is a double integrator with a first-order lag

Every one of the six papers writes the same plant. Ma et al. eq. (1), Ploeg
eq. (4.9), Köroğlu eq. (1), Zhao eq. (1):

```
    ẋ_i = v_i
    v̇_i = a_i                       (double integrator: Newton)
    τ ȧ_i + a_i = u_i               (first-order actuator/driveline lag)
```

**Where the double integrator comes from.** Newton's second law for a point
mass on a straight road, with all forces lumped into a commanded specific
force: `m ẍ = F` ⟹ `ẍ = F/m =: a`. Position is the twice-integrated
acceleration. This is exact, not an approximation.

**Where the lag comes from — and why it is *first order*.** The commanded
acceleration `u` does not appear instantly at the wheels. Between command and
realised acceleration sit: torque build-up in the engine/motor, driveline
compliance, brake hydraulics, and the low-level torque controller. A full
model of that chain is high-order and nonlinear. Two facts collapse it to one
pole:

1. Production vehicles run an *inner-loop acceleration controller* — the CACC
   layer commands acceleration, and a lower-level controller (or a
   feedback-linearising compensator) tracks it. Ploeg's Remark on eq. (3.4)
   is explicit: `G(s)` "adequately describes the dynamics of the native
   force-controlled hybrid drive line of the test vehicles, including a
   precompensator to convert the desired acceleration to the desired force,
   taking into account the actual vehicle mass and the estimated drag
   forces." Ma's Remark 1 says the same thing: heterogeneity "can be
   considered in the vehicle dynamics models which can be feedback linearized
   with a lower-level controller."
2. A well-tuned inner loop is *dominated by one slow pole*. Everything faster
   than that pole is outside the bandwidth the outer loop can see.

So `τ` is not a physical time constant of any single component — it is the
**closed inner loop's dominant time constant**, the aggregate of everything
the outer loop cannot command away.

Ploeg validates this experimentally (his Fig. 3.2): a step-response test on a
real Toyota Prius III gives τ = 0.1 s plus a pure delay φ = 0.2 s, and the
first-order fit tracks the measurement closely.

**Numbers across the papers.** τ = 0.5 s (Ma, and Zhao), τ = 0.1 s + φ = 0.2 s
(Ploeg), τ = 0.5 s (Köroğlu's example). Ma's τ = 0.5 s is deliberately
pessimistic — see F-3.

### F-2. State space and the plant transfer function

Stack `x = (x_i, v_i, a_i)ᵀ`:

```
    ẋ = A x + B u ,     A = [0 1  0 ]   B = [ 0 ]
                            [0 0  1 ]       [ 0 ]
                            [0 0 -1/τ]      [1/τ]
```

Take Laplace transforms with zero initial conditions. From `τ s A(s) + A(s) =
U(s)`:

```
    A(s)/U(s) = 1/(τs + 1)
```

and `X(s) = A(s)/s²`, so the command-to-position transfer function is

```
    G(s) = X(s)/U(s) = 1 / (s²(τ s + 1))                              (F-2)
```

This single expression appears as Ploeg eq. (26)/(3.4), Köroğlu's `P₀(s)`
eq. (2), and implicitly inside Ma's eq. (7). **Memorise it.** Three poles:
a double pole at the origin (the two integrators) and a real pole at −1/τ.

When a communication or actuation delay φ is modelled, `G(s) = e^{−φs} /
(s²(τs+1))` (Ploeg eq. 3.4).

### F-3. Why "τ uncertain in (0, τ₀]" is the right uncertainty model

Ma assume `τ ∈ (0, τ₀]` with τ₀ known, and design for the whole interval.
Two justifications, both worth being able to state:

* **Fleet heterogeneity without heterogeneous analysis.** Their Remark 1:
  `τ₀ = max_{i ∈ {1..N}} τ_i`. If the design is certified for every τ in
  `(0, τ₀]`, it is certified for any mix of vehicles whose lags are all below
  τ₀ — you never need to know the individual τ_i. Case 5 of their Sec. IV
  tests exactly this (Table II: twelve random τ_i, all ≤ 0.5 s).
* **Monotone worst case.** As shown in Part II §2.6, the string-stability
  condition (26a) reads `1 − k̃a² − 2τγ ≥ 0`, which is *hardest* at the
  largest τ. So certifying at τ = τ₀ certifies the whole interval; you do not
  have to sweep. That monotonicity is the reason the interval assumption
  costs nothing.

Contrast with a *lower* bound on τ: there is none, and none is needed —
τ → 0⁺ makes the plant faster and the condition easier. A fast vehicle is
never the problem.

---

## 2. Spacing policies

### F-4. Constant spacing vs constant time headway

Two candidate desired gaps between vehicle i and its predecessor:

```
    CSP:  d_des = d                       (constant spacing policy)
    CTH:  d_des = r + h·v_i               (constant time headway)
```

`h` [s] is the *time headway* (Ma writes h_w, Ploeg writes h, Köroğlu writes
h and normalises h̄ = h/τ); `r` (Ma's `d`) is the standstill distance.

`h` has a clean physical meaning: **the time it takes the ego vehicle to reach
the point its predecessor's rear bumper occupies now.** Travelling at v, the
extra gap h·v is covered in exactly h seconds.

### F-5. Why CTH buys string stability and CSP does not

This is the single most important intuition in the whole literature, and it
is a two-line argument.

Differentiate the CTH spacing error `e_i = d_i − r − h v_i` (Ploeg eq. 3.2):

```
    ė_i = ḋ_i − h v̇_i = (v_{i−1} − v_i) − h a_i
```

The `− h a_i` term is a **damping term proportional to the ego vehicle's own
acceleration, with no counterpart from the predecessor**. It is a
*self-generated* velocity-error feedback that grows with h. With CSP (h = 0)
that term is absent, and error attenuation must come entirely from the
feedback loop acting through the plant — which has a double integrator and
therefore no low-frequency margin to spare.

The frequency-domain statement of the same fact: the spacing policy enters
the error propagation as the factor `1/H(s)` with

```
    H(s) = h s + 1                                                     (F-5)
```

`|H(jω)| = √(1 + h²ω²) ≥ 1` for all ω, with equality only at ω = 0. So the
policy contributes a **low-pass factor whose attenuation grows with h and
with frequency** — free attenuation, purchased with road space. Every
"minimum time headway" result in these papers (Ma's h_lb, Köroğlu's h̄_o,
Ploeg's h ≥ 0.25 s / 1.23 s / 3.16 s) is a statement about how much of that
free attenuation you need to buy to cancel some other lag: the actuator lag
τ, the communication delay θ, or the noise-induced feedforward error.

Formal impossibility for CSP: Seiler–Pant–Hedrick (Ploeg's ref. [19]) show a
predecessor-following platoon with constant spacing and only relative-position
feedback *cannot* be string stable — the double integrator forces
`sup_ω |Γ(jω)| > 1`. CSP works only with additional information (leader
broadcast, bidirectional coupling), which is exactly what Zhao's paper uses
(consensus over a graph with a pinned leader).

### F-6. The two sign conventions — and why they differ

This trips people up constantly, so pin it down once.

| | Ploeg / Köroğlu / this project's Python core | Ma et al. 2025 |
|---|---|---|
| error | `e_i = d_i − d_des,i = (q_{i−1} − q_i − L_i) − (r + h v_i)` | `e_i = x_i − x_{i−1} + d` |
| sign | `e_i > 0` ⇒ gap too **large** | `δ_i > 0` ⇒ gap too **small** |
| relation | — | `e_i^Ploeg = − δ_i^Ma`, with `d = r + L` |

Ma work with front-bumper point masses, so their `d` absorbs the vehicle
length; Ploeg tracks rear-bumper positions and carries `L_i` explicitly. The
project maps them with `r = 1 m`, `L = 4 m`, `d = 5 m` (decision D-005), so
the two conventions agree numerically after one sign flip.

Ma's CTHP error, their eq. (2):

```
    δ_i(t) = e_i(t) + h_w v_i(t) = x_i − x_{i−1} + d + h_w v_i        (F-6)
```

Sanity check the equilibrium: `δ_i = 0` ⟺ `x_{i−1} − x_i = d + h_w v_i`, i.e.
the front-to-front spacing equals standstill distance plus headway times
speed. At v = 25 m/s and h_w = 0.95 s that is 5 + 23.75 = 28.75 m, so a
12-vehicle platoon is 345 m long — the value at t = 0 in their Fig. 13. ✓

---

## 3. Transfer functions, norms, and what "gain" means

### F-7. Why the analysis is done in the frequency domain at all

String stability is a statement about a **cascade**. If each hop multiplies a
signal by a transfer function Γ(s), then after N hops the signal has been
multiplied by Γ(s)^N. In the frequency domain that is `|Γ(jω)|^N` — a scalar
raised to a power. Nothing else in control theory composes this cleanly.

The practical consequence: per-hop gains that look harmless compound
geometrically. `g = 1.0035` (Ma's Case 2) over 11 hops is `1.0035^11 = 1.039`,
a 3.9 % growth — invisible in a time plot. Over a 50-vehicle platoon it is
1.19; over 200 it is 2.0. **String instability is a slow explosion, which is
exactly why you must certify it in the frequency domain and not by looking at
a 200-second simulation** (this project's decision D-010 says the same thing
after being bitten by it).

### F-8. Signal norms

For a scalar signal y(t) on `t ∈ [0, ∞)`:

```
    ‖y‖_{L₂}  = ( ∫₀^∞ |y(t)|² dt )^{1/2}          "energy"
    ‖y‖_{L∞} = ess sup_t |y(t)|                    "peak"
    ‖y‖_{L₁}  = ∫₀^∞ |y(t)| dt                     "total absolute"
```

Choice matters physically. Ploeg §IV-C: L₂ corresponds to *energy dissipation
along the string*; L∞ corresponds to *traffic safety*, because peak overshoot
is what causes a collision. They are not equivalent, and L∞ is strictly harder
(F-13).

### F-9. System norms and the induced-gain theorem

For a stable LTI system with transfer function `P(s)`:

```
    ‖P‖_{H∞} = sup_{Re s > 0} σ̄(P(s)) = sup_ω σ̄(P(jω))       (max modulus)
```

For SISO, `σ̄(P(jω)) = |P(jω)|`, so `‖P‖_{H∞} = sup_ω |P(jω)|`.

**Theorem (induced L₂ gain).** For a stable LTI system,

```
    ‖P‖_{H∞} = sup_{u ≠ 0}  ‖y‖_{L₂} / ‖u‖_{L₂}                        (F-9)
```

*Why.* Parseval: `‖y‖²_{L₂} = (1/2π)∫|P(jω)|²|U(jω)|²dω ≤
(sup_ω|P|²)·(1/2π)∫|U|²dω = ‖P‖²_{H∞}‖u‖²_{L₂}`, giving `≤`. For `≥`,
concentrate the input's energy in a narrow band around the peak frequency ω*;
as the band narrows the ratio → `|P(jω*)|`. Hence sup is attained in the
limit and the bound is *not conservative* — Ploeg makes this point explicitly
below his eq. (16).

**This is why ‖·‖_{H∞} ≤ 1 is the right string-stability test for L₂**: it is
exactly the statement "no input can gain energy crossing this hop."

### F-10. The L∞ counterpart is the L₁ norm of the impulse response

If `y = p * u` (convolution with impulse response p), then

```
    |y(t)| = |∫ p(σ) u(t−σ) dσ| ≤ (∫|p(σ)|dσ) · sup|u| = ‖p‖_{L₁} ‖u‖_{L∞}
```

and the bound is achieved by `u(t−σ) = sign p(σ)`. Therefore

```
    ‖p‖_{L₁} = sup_{u≠0} ‖y‖_{L∞} / ‖u‖_{L∞}                          (F-10)
```

This is Ploeg eq. (24). The L∞ string stability test is `‖γ_i‖_{L₁} ≤ 1`,
where γ is the impulse response of Γ.

### F-11. ‖p‖_{L₁} ≥ ‖P‖_{H∞}, so L∞ string stability is strictly harder

```
    |P(jω)| = |∫ p(t) e^{−jωt} dt| ≤ ∫|p(t)|dt = ‖p‖_{L₁}   for every ω
```

Take the sup over ω: `‖P‖_{H∞} ≤ ‖p‖_{L₁}`. So `‖p‖_{L₁} ≤ 1` implies
`‖P‖_{H∞} ≤ 1` but not conversely.

Ploeg quantifies the gap in his Fig. 3(b) vs 4(b): with τ = 0.1, kp = 0.2,
kd = 0.7, the maximum tolerable communication delay for `‖Γ‖_{H∞} ≤ 1` at
h = 1.0 s is ≈ 0.35 s, while for `‖γ‖_{L₁} ≤ 1` it is ≈ 0.30 s at the same
headway and requires "a significantly larger time headway" in general.
**Every string-stability claim in these papers except Ploeg's Theorem 2 is an
L₂ claim.** Say so when asked.

### F-12. The submultiplicative property, and why Theorem 1 is only sufficient

For the cascade `P_i(s) = (∏_{k=2}^{i} Γ_k(s)) P₁(s)` (Ploeg eq. 20),

```
    ‖P_i‖_{H∞} ≤ (∏_{k=2}^{i} ‖Γ_k‖_{H∞}) · ‖P₁‖_{H∞}                 (F-12)
```

because `sup_ω |AB| ≤ (sup_ω|A|)(sup_ω|B|)` — the peaks of A and B need not
occur at the same ω. So `‖Γ_k‖_{H∞} ≤ 1 ∀k` gives `sup_i ‖P_i‖_{H∞} < ∞`,
which is L₂ string stability: **sufficient**. It is not necessary in general
precisely because of that peak-misalignment slack.

For a *homogeneous* string, Γ is index-independent and Ploeg's Remark 4 shows
the conditions become **necessary and sufficient**: with
`‖P_i‖_{H∞} = sup_ω{|Γ(jω)|^{i−1}|P₁(jω)|}`, the sup over i is finite for all
i iff `|Γ(jω)| ≤ 1 ∀ω`. This is the case in every paper here except Zhao's.

### F-13. Why |Γ(j0)| = 1 always, and what it costs

Physically: with the platoon in equilibrium, all vehicles drive at the same
constant speed, so `lim_{ω→0}(v_i(jω) − v_{i−1}(jω)) = 0`, i.e. the DC gain
from predecessor to follower is exactly 1 (Ploeg eq. 32; Köroğlu's remark
`‖G‖_∞ ≥ |G(0)| = 1` below his eq. 20).

Algebraically for Ma's family: `H̃(0) = k_p/k_p = 1`.

Two consequences you should be ready to state:

1. **The string-stability inequality is always tight at DC.** Strict
   inequality `‖Γ‖_{H∞} < 1` is impossible; the best achievable is
   `‖Γ‖_{H∞} = 1`, attained at ω = 0. This is why Ma's Case 1 and Case 3
   both report `‖H̃‖_∞ = 1.000000` and why Köroğlu writes `‖G‖_∞ = 1` as the
   *design equation* rather than an inequality.
2. **Robustness margin at low frequency is structurally zero.** Ploeg:
   "string stability robustness with respect to model uncertainties may be
   poor, since these cause |Γ(jω)| to increase in the lower frequency region."
   Any modelling error near DC pushes you over 1. This is the deep reason Ma's
   whole paper exists: multiplicative channel noise is exactly such a
   low-frequency perturbation of the feedforward gain.

---

## 4. Stability tests used in the papers

### F-14. Routh–Hurwitz for a cubic

For `D(s) = a₃s³ + a₂s² + a₁s + a₀` with `a₃ > 0`, all roots have negative
real part **iff**

```
    a₂ > 0 , a₁ > 0 , a₀ > 0  ,  and  a₂ a₁ > a₃ a₀                   (F-14)
```

*Derivation of the product condition.* Build the Routh array:

```
    s³ |  a₃    a₁
    s² |  a₂    a₀
    s¹ |  b     0        b = (a₂a₁ − a₃a₀)/a₂
    s⁰ |  a₀
```

No sign changes in the first column requires `a₂ > 0`, `b > 0` ⟺
`a₂a₁ > a₃a₀`, and `a₀ > 0`.

**Applied to Ma's denominator** `D(s) = τs³ + s² + γs + k_p`, with
`γ := k_v + h_w k_p`:

```
    a₃ = τ, a₂ = 1, a₁ = γ, a₀ = k_p
    ⟹  γ > 0, k_p > 0, and  1·γ > τ·k_p   i.e.   γ > τ k_p           (F-14a)
```

Worst case over `τ ∈ (0, τ₀]` is τ = τ₀, giving Ma's internal-stability
condition `γ − τ₀ k_p > 0`. This is the "`γ − τ₀k_p > 0`" line in their proof
of Theorem 2. Numerically for their Case 1: `γ = 0.63855`, `τ₀k_p = 0.0045`,
margin `+0.634` — internal stability is never the binding constraint in these
designs; string stability is.

**Applied to Köroğlu's** `Q₀(s̄) = s̄³ + s̄² + κφs̄ + κ` (his eq. 21):
`a₃=a₂=1, a₁=κφ, a₀=κ`, so `κφ·1 > 1·κ` ⟺ `φ > 1`, plus `κ > 0` — exactly
his eq. (22).

**Applied to Ploeg's** controlled vehicle: his §III states the equilibrium is
asymptotically stable "for any time headway h > 0, and with any choice for
k_p, k_d > 0, k_dd > −1, such that `(1 + k_dd)k_d > k_p τ`" — the same
`a₂a₁ > a₃a₀` inequality after clearing the h's.

### F-15. Evaluating |H(jω)|² without complex algebra errors

The recurring manoeuvre in Part II. For a polynomial `P(s) = Σ c_k s^k`
evaluated at `s = jω`, split by parity of k:

```
    P(jω) = [c₀ − c₂ω² + c₄ω⁴ − …]  +  j[c₁ω − c₃ω³ + c₅ω⁵ − …]
              ↑ even part, real          ↑ odd part, imaginary
```

so

```
    |P(jω)|² = (c₀ − c₂ω² + …)² + (c₁ω − c₃ω³ + …)²                   (F-15)
```

*Always* group by parity first; sign errors in these papers' expansions come
from not doing so.

### F-16. `|Γ| ≤ 1  ⟺  |D|² − |N|² ≥ 0`

Trivial but used constantly: with `Γ = N/D` and `D(jω) ≠ 0`,

```
    |Γ(jω)| ≤ 1  ⟺  |N(jω)|² ≤ |D(jω)|²  ⟺  |D(jω)|² − |N(jω)|² ≥ 0
```

The point of the rewrite is that `|D|² − |N|²` is a **polynomial in ω²** with
real coefficients, so string stability becomes a *polynomial positivity*
problem — checkable by conditions on coefficients (Part II §2.6), not by
gridding frequency.

---

## 5. Probability facts the noise model needs

### F-17. Bernoulli variables and the binary expansion

`z ~ Bernoulli(γ)`: `P(z = 1) = γ`, `P(z = 0) = 1 − γ`, `E[z] = γ`.

Ma build their channel factor from n such bits (their eq. 5):

```
    w(t) = (1 − 1/ρ) + (1/ρ) Σ_{j=0}^{n−1} z_j(t) 2^{−j}              (F-17)
```

**Range.** `Σ_j z_j 2^{−j}` is minimised at 0 (all bits 0) and maximised at
`Σ_{j=0}^{n−1} 2^{−j} = 2 − 2^{−(n−1)} < 2` (all bits 1). Hence

```
    1 − 1/ρ  ≤  w  <  1 + 1/ρ                                        (F-17a)
```

— a **strictly bounded support**, half-open at the top. The upper end is
never attained for finite n. This is exactly the "admissible cone" drawn in
their Figs. 2 and 3, and reproduced in the Simulink Fig. 8
(`matlab/results/fig08_noise_link12.png`, measured range [0.8000, 1.2000]
with ρ = 5).

**Why a binary expansion and not, say, a Gaussian?** Because the paper's story
is an *n-bit communication channel*: the transmitted acceleration is quantised
into n bits, and each bit may or may not arrive corrupted. `z_j` is the
indicator that bit j is set; the weight `2^{−j}` is that bit's place value.
The model is therefore a *physical* description of bit-level corruption, and
its bounded support is a structural fact, not a modelling convenience.
Compare: a Gaussian noise model has unbounded tails, so no finite headway
could ever guarantee string stability with probability 1.

### F-18. E[w] and why it is not 1

Expectation is linear, so from (F-17):

```
    E[w] = (1 − 1/ρ) + (1/ρ) Σ_{j=0}^{n−1} γ_j 2^{−j}                 (F-18)
```

With Ma's Table I and ρ = 5:

```
    Σ γ_j 2^{−j} = 0.8055 + 0.5767/2 + 0.1829/4 + 0.2399/8 + 0.8865/16 + … 
                 = 1.24087
    E[w] = 0.8 + 0.2 × 1.24087 = 1.04817
```

(verified numerically: `ma2025.expected_w(5)` → 1.048174).

**The channel is biased.** `E[w] ≈ 1.048 ≠ 1`: on average the receiver
*over-estimates* the predecessor's acceleration by 4.8 %. There is nothing in
the model that forces the bits to be balanced. This is why the effective
feedforward gain (F-19) is `k_a E[w]`, not `k_a`, and why the paper never
assumes the noise "averages out."

### F-19. The effective gain k̃a, and the interval I

Ma eq. (12):

```
    k̃a := k_a E[w] = k_a [1 − 1/ρ + (1/ρ) Σ_j γ_j 2^{−j}]            (F-19)
```

But the `γ_j` are **not known a priori** (their sentence after eq. 5: "in
order to model a wide variety of noise processes, we do not assume that
γ_{i,j}'s are known a priori"). So a robust design must hold for every
possible mean, i.e. for every

```
    k̃a ∈ I := [(1 − 1/ρ) k_a , (1 + 1/ρ) k_a]                        (F-19a)
```

For `k_a = 0.5`, `ρ = 5`: `I = [0.4, 0.6]`. This interval is the object over
which every max/min in Theorem 2 is taken, and the shaded band in their
Figs. 6 and 9.

### F-20. The SNR definition and ρ

Ma's Definition 3:

```
    SNR = min_{t>0} 20 log₁₀ ( |s(t)| / |n(t)| )    [dB]
    ρ   = min_{t>0} |s(t)|/|n(t)| = 10^{SNR/20}
```

Note it is a **minimum over time**, not an average — a worst-case (L∞-flavoured)
SNR. `ρ = 5` corresponds to `20 log₁₀ 5 = 13.98 dB`. With `s = a_{i−1}` and
`n = (w−1)a_{i−1}`, `|n|/|s| = |w − 1| ≤ 1/ρ`, consistent with (F-17a).

### F-21. Estimability of ρ (used by this project's extension, not by the paper)

Because the support is *strictly* bounded (F-17a), any empirical maximum or
high quantile of `|ŵ − 1|` **under**-estimates `1/ρ`, hence **over**-estimates
ρ. The bias direction is known by construction, so a single safety divisor κ
makes an online estimator conservative. This is far stronger than
moment-matching, which would require the unknown γ_j. See `docs/theory/T-05`.

---

## 6. String stability: the definitions, in order of strength

### F-22. Ploeg's Definition 1 (L_p string stability)

For the cascade `ẋ₀ = f_r(x₀, u_r)`, `ẋ_i = f_i(x_i, x_{i−1})`, `y_i = h(x_i)`:

The system is **L_p string stable** if there exist class-𝒦 functions α, β with

```
    ‖y_i(t) − h(x̄₀)‖_{L_p} ≤ α(‖u_r(t)‖_{L_p}) + β(‖x(0) − x̄‖)
                                     ∀ i ∈ S_m  and  ∀ m ∈ ℕ         (F-22a)
```

If additionally, with `x(0) = x̄`,

```
    ‖y_i(t) − h(x̄₀)‖_{L_p} ≤ ‖y_{i−1}(t) − h(x̄₀)‖_{L_p}   ∀ i, ∀ m  (F-22b)
```

it is **strictly L_p string stable**.

Three things make this definition better than its predecessors, and you should
be able to name all three:

* **"∀ m ∈ ℕ" is the whole point.** The bound must hold *for every string
  length*. This is what makes string stability a scalability property rather
  than a property of one particular platoon. Drop it and the definition
  degenerates to ordinary input–output stability.
* **It admits both external disturbances (u_r) and initial-condition
  perturbations (x(0)).** The Lyapunov-flavoured definitions (Swaroop–Hedrick)
  cover only the latter; the performance-oriented `sup_ω|Γ| ≤ 1` criterion
  covers only the former with zero ICs. Definition 1 contains both as special
  cases.
* **It is stated for nonlinear systems.** The linear `‖Γ‖_{H∞} ≤ 1` test is
  then *derived* (Ploeg Theorem 1), not postulated.

`α` and `β` being class 𝒦 (continuous, strictly increasing, α(0) = 0) is what
makes (F-22a) a genuine stability statement: zero disturbance and zero IC
perturbation give zero output deviation.

### F-23. Semi-strict vs strict

Ploeg's thesis adds an intermediate notion (his eq. 3.11 vs 3.12):

```
    semi-strict:  ‖y_i‖ ≤ ‖y₁‖        compare every vehicle to the FIRST
    strict:       ‖y_i‖ ≤ ‖y_{i−1}‖   compare every vehicle to its PREDECESSOR
```

Strict ⟹ semi-strict. Semi-strict is what you must settle for when a
single-vehicle subsystem with an independent input cannot be isolated — which
happens as soon as the topology is *two*-vehicle look-ahead, because `u_{i−1}`
and `u_{i−2}` both depend on `u_{i−3}` and are therefore not independent
inputs (Ploeg §3.6.2). In that case the exogenous input must be taken as the
lead vehicle's `u₁`, giving `Θ_i(s) := P_i(s)P₁⁻¹(s)` and the condition
`‖Θ_i‖_{H∞} ≤ 1` (his Condition 3.2) instead of `‖Γ_i‖_{H∞} ≤ 1`
(Condition 3.3).

### F-24. Ma's Definition 2 (robust string stability)

Ma's is narrower and specialised to their problem, and the *robustness*
adjective does specific work:

> The platoon with vehicle dynamics model (1) is **robustly string stable** if
> the following two conditions hold for all τ ∈ (0, τ₀]: (i) H(s) achieves
> internal stability, and (ii) `‖δ_i(t)‖_∞ ≤ ‖δ_{i−1}(t)‖_∞`, or in the
> frequency domain `‖H(jω)‖_∞ ≤ 1`.

Read carefully, "robust" quantifies over **two** uncertainties:

1. `τ ∈ (0, τ₀]` — the parasitic lag, explicit in the definition;
2. `k̃a ∈ I` — the channel mean, implicit but enforced throughout Theorem 2
   by the `min`/`max` over `k̃a ∈ I` in eq. (28).

And it requires internal stability *in addition to* the H∞ bound: an unstable
`D(s)` would make `‖H̃‖_{H∞}` meaningless (the H∞ norm is defined for stable
transfer functions). That is why Routh–Hurwitz (F-14a) appears as a separate
clause in the theorem rather than being folded into the norm condition.

---

## 7. Quick reference — the objects that recur

| symbol | meaning | first defined |
|---|---|---|
| `τ`, `τ₀` | parasitic actuation lag; its upper bound | F-1, F-3 |
| `h`, `h_w`, `h̄ = h/τ` | time headway; Köroğlu's normalised headway | F-4 |
| `d`, `r`, `L` | standstill spacing; standstill gap; vehicle length | F-6 |
| `e_i`, `δ_i` | CSP spacing error; CTHP spacing error `= e_i + h_w v_i` | F-6 |
| `G(s) = 1/(s²(τs+1))` | command-to-position plant | F-2 |
| `H(s) = hs + 1` | spacing-policy transfer function | F-5 |
| `D(s) = e^{−θs}` | communication channel delay | Part III |
| `Γ(s)`, `H̃(s)` | string-stability complementary sensitivity | F-7, Part II |
| `‖·‖_{H∞}`, `‖·‖_{L₁}` | induced L₂ gain; induced L∞ gain | F-9, F-10 |
| `w_{i,i−1}(t)` | multiplicative channel factor | F-17 |
| `ρ` | SNR factor `= 10^{SNR/20}` | F-20 |
| `k̃a`, `I` | effective feedforward gain; its admissible interval | F-19 |
| `γ := k_v + h_w k_p` | Ma's lumped feedback parameter | F-14a |

---

*Next: [Part II — Ma, Pagilla & Darbha 2025](02-ma2025-base-paper.md), where
every equation of the base paper is derived using only F-1 … F-24.*
