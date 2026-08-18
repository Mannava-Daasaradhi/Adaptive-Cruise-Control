# Part IV — Köroğlu 2024: minimum time headway under delayed communication

> H. Köroğlu, "String-Stable Cooperative Adaptive Cruise Control With Minimized
> Time Headway in the Face of Delayed Communication," *IEEE Control Systems
> Letters* **8**:400–405, 2024. doi 10.1109/LCSYS.2024.3392716

**The question.** Ma asks: how much headway does *channel noise* cost?
Köroğlu asks the complementary question: how much headway does *communication
delay* cost — and what is the theoretical floor?

**The answer, one formula** (his eq. 32):

```
    h̄ := h/τ  >  h̄_o = √( 2δ̄ ( √(1 + 0.5δ̄) + 1 ) ) ,     δ ∈ [0, δ̄τ]
```

Everything is **normalised by the vehicle time constant τ**: `h̄ = h/τ`,
`δ̄ = δ_max/τ`, `s̄ = τs`, `ϖ = τω`. This is the paper's cleverest move — the
answer then depends on **one** number, the delay-to-lag ratio, and τ scales out
entirely.

---

## 4.1 Setup  (eqs. 1–6)

### Plant

Identical to F-1/F-2:

```
    ẋ_i = v_i ,  v̇_i = a_i ,  τ ȧ_i = u_i − a_i
    ⟹  x_i(s) = P₀(s) u_i(s) + w_i(s) ,    P₀(s) = 1/(s²(τs + 1))       (2)
```

with `w_i(s)` collecting the initial-condition terms
`τ a_i(0)/(s²(τs+1)) + v_i(0)/s² + x_i(0)/s`. Carrying `w_i` explicitly (most
papers drop it) is what lets Köroğlu treat initial-condition mismatch as a
*disturbance* `d_i` rather than assume it away — see (6).

### Spacing error and the "clean output"

```
    e_i(t) = [x_{i−1} − x_i − l_i] − h v_i − (g_i − l_i)                (3)
```

`g_i` = desired standstill *rear-bumper-to-front-bumper* distance, `l_i` =
length. Then the key definitional step (eq. 4):

```
    y_i(s) = (hs + 1)[x_i(s) − w_i(s)] = P_h(s) u_i(s) ,   P_h := ℋ⁻¹P₀
             ╰── ℋ⁻¹(s) ──╯                                             (4)
```

so `ℋ(s) = 1/(hs+1)` — **note the convention**: Köroğlu's `ℋ` is the
*reciprocal* of Ploeg's `H(s) = hs+1`. Getting this backwards inverts every
subsequent inequality, so pin it down: `|ℋ(jω)|⁻² = 1 + h²ω²`.

`y_i` is called the *clean* output because the initial-condition term `w_i` has
been subtracted, making `y_i` an exact linear function of `u_i` alone.

### Error and control propagation

```
    e_i(s) = x_{i−1}(s) − ℋ⁻¹(s)x_i(s) + h x_i(0) − g_i/s
           = r_i(s) − y_i(s) ,     r_i := x_{i−1} − w_{i−1} + d_i        (5)
```

with the disturbance (eq. 6)

```
    d_i(s) = w_{i−1}(s) − ℋ⁻¹(s)w_i(s) + h x_i(0) − g_i/s
           = τ[a_{i−1}(0) − (hs+1)a_i(0)]/(s²(τs+1)) + [v_{i−1}(0)−v_i(0)]/s²
             + [x_{i−1}(0) − x_i(0) − h v_i(0) − g_i]/s
```

**`d_i ≡ 0` exactly when the platoon starts in equilibrium**: equal initial
accelerations, equal initial velocities, and initial spacing equal to its
desired value `x_{i−1}(0) − x_i(0) = h v_i(0) + g_i`. This is a genuinely
useful thing to notice: it tells you precisely which initial conditions make
the "clean" analysis exact, and it is exactly the initial condition used in
the Simulink reproduction of Ma (`δ_i(0) = 0` for every follower).

### Controllers, sensitivities

```
    𝒮_h ≜ 1/(1 + 𝒫_h𝒞_h) ,   𝒯_h ≜ 𝒫_h𝒞_h/(1 + 𝒫_h𝒞_h) = 1 − 𝒮_h        (7)
    u_i = ℱ_h 𝒟 u_{i−1} + 𝒞_h(r_i − 𝒫_h u_i)
        ⟹ u_i = 𝒮_h ℱ_h 𝒟 u_{i−1} + 𝒞_h 𝒮_h r_i                        (8)
```

*Derivation of (8):* collect `u_i` terms —
`u_i(1 + 𝒞_h𝒫_h) = ℱ_h𝒟u_{i−1} + 𝒞_h r_i`, then divide by `(1+𝒫_h𝒞_h)`. ∎

Using `r_i = x_{i−1} − w_{i−1} + d_i = ℋ𝒫_h u_{i−1} + d_i` (since
`x_{i−1} − w_{i−1} = P₀u_{i−1} = ℋ𝒫_hu_{i−1}`), substitute into (8):

```
    u_i = 𝒢 u_{i−1} + 𝒞_h𝒮_h d_i                                       (10)
    𝒢 ≜ 𝒮_hℱ_h𝒟 + 𝒯_hℋ = ℋ − 𝒮_h(ℋ − ℱ_h𝒟)                            (11)
```

*The second form of (11):* `𝒯_h = 1 − 𝒮_h`, so
`𝒮_hℱ_h𝒟 + (1−𝒮_h)ℋ = ℋ − 𝒮_h(ℋ − ℱ_h𝒟)`. ∎

**`𝒢` is the string-stability transfer function** — the analogue of Ploeg's
`Γ` and Ma's `H̃`, here from `u_{i−1}` to `u_i`.

And the spacing error propagates as (eqs. 12–13):

```
    e_i = ℳ u_{i−1} + 𝒮_h d_i ,    ℳ ≜ 𝒫_h(ℋ − 𝒢) = 𝒮_h𝒫_h(ℋ − ℱ_h𝒟)   (13)
```

### The perfect-feedforward observation  (below eq. 14)

If `𝒟 = 1` (no delay) and `ℱ_h = ℋ`, then `ℳ = 0` and `𝒢 = ℋ`, so
`‖𝒢‖_∞ = 1` **and the spacing error is identically zero** for any preceding
manoeuvre. `h` could then be chosen arbitrarily close to 0.

This is Ploeg's `Γ = 1/H` result (Part III §3.4) in Köroğlu's notation.
**The entire paper is about how much of that ideal you lose to `𝒟 = e^{−δs}`.**

---

## 4.2 The specific controller structure  (eqs. 14–19)

Köroğlu restricts to

```
    ℱ_h(s) = 1/(h̄s̄ + 1) = ℋ(s)                     (unity feedforward)   (14)

    𝒞_h(s) = (κφ/τ · s + κ/τ²) · 1/(hs+1)
           = (κ/τ²)(φ s̄ + 1)/(h̄ s̄ + 1)                                  (15)
              ╰── 𝒞₀ ──╯      ╰── ℋ ──╯
```

with the scaled Laplace variable `s̄ := τs` and normalised headway
`h̄ := h/τ > 0` (eq. 16).

Structure: a **PD feedback** `𝒞₀ = (κ/τ²)(φs̄+1)` in series with the spacing
filter `ℋ`, plus **unity feedforward through the same filter**. Two design
parameters only: `κ` (gain) and `φ` (lead zero location). This minimalism is
the paper's selling point — "our controller does not require acceleration
feedforward, which is crucial for [1] and [5] especially for handling
heterogeneous platoons."

### Deriving `𝒮₀`  (eq. 17)

The `ℋ` factors in `𝒞_h` and `𝒫_h = ℋ⁻¹𝒫₀` cancel:
`𝒫_h𝒞_h = ℋ⁻¹𝒫₀ · 𝒞₀ℋ = 𝒫₀𝒞₀`. Hence `𝒮_h = 𝒮₀`, `𝒯_h = 𝒯₀`. In scaled
variables:

```
    𝒫₀ = 1/(s²(τs+1)) = τ²/(s̄²(s̄+1)) ,     𝒞₀ = (κφ s̄ + κ)/τ²
    𝒫₀𝒞₀ = (κφ s̄ + κ)/(s̄²(s̄+1))
    1 + 𝒫₀𝒞₀ = (s̄³ + s̄² + κφ s̄ + κ)/(s̄²(s̄+1))
```

```
                     1            𝒩₀(s̄)        s̄²(s̄ + 1)
    𝒮₀(s̄) = ───────────── = ───────── = ─────────────────────────      (17)
              1 + 𝒫₀𝒞₀        𝒬₀(s̄)      s̄³ + s̄² + κφ s̄ + κ
```

Note `𝒮₀` has a **double zero at the origin** — the double integrator in the
plant makes the loop reject constant and ramp disturbances perfectly.

### `𝒢` and `ℳ` in final form  (eqs. 18, 19)

With `ℱ_h = ℋ` (so `ℱ₀ = 1`):

```
    𝒢 = ℋ(1 − 𝒮₀(1 − 𝒟)) = ℋ[1 − (𝒩₀/𝒬₀)(1 − 𝒟)]                      (18)
    ℳ = 𝒫₀𝒮₀(1 − 𝒟) = [τ²/(s̄²(s̄+1))]·[s̄²(s̄+1)/𝒬₀]·(1−𝒟) = τ²(1−𝒟)/𝒬₀ (19)
```

The cancellation in (19) is total — `ℳ` is just `τ²(1−𝒟)/𝒬₀`. **Both `𝒢−ℋ`
and `ℳ` are proportional to `(1 − 𝒟)`,** so *everything* that degrades the
system is driven by that single factor. If `𝒟 = 1`, both vanish. That is the
structural insight the whole analysis rests on.

### Internal stability  (eqs. 21, 22)

`𝒬₀(s̄) = s̄³ + s̄² + κφs̄ + κ` (eq. 21). Routh–Hurwitz for a cubic (F-14) with
`a₃ = a₂ = 1`, `a₁ = κφ`, `a₀ = κ`:

```
    a₂a₁ > a₃a₀  ⟺  κφ > κ  ⟺  φ > 1 ,   and  κ > 0                    (22)
```

So the lead zero must be **slower than the plant pole** (`φ > 1` means the zero
`s̄ = −1/φ` lies to the right of `s̄ = −1`). This is a very tidy stability
condition and worth remembering as a sanity check on any gain choice.

---

## 4.3 The string-stability inequality  (eqs. 23–25)

### The delay factor

```
    1 − 𝒟(jω) = 1 − e^{−jδω} = (1 − cos δω) + j sin(δω) =: ς + jσ       (23)
```

Note `|1 − 𝒟(jω)|² = ς² + σ² = (1−cos)² + sin² = 2(1 − cos δω) = 2ς`. Useful
identity — it means `|1−𝒟|² = 2ς`, so the quadratic term is *linear* in ς.

### The requirement

String stability is `‖𝒢‖_∞ ≤ 1`, which by F-13 (`|𝒢(0)| = 1`) is really
`‖𝒢‖_∞ = 1`. From (18), `|𝒢| ≤ 1` ⟺ `|1 − 𝒮₀(1−𝒟)|² ≤ |ℋ|⁻² = 1 + h̄²ϖ²`:

```
    |1 − 𝒮₀(jϖ)[1 − 𝒟(jϖ)]|²
        = 1 + |𝒮₀|²|1−𝒟|² − 2ℜ{𝒮₀[1−𝒟]}  ≤  1 + h̄²ϖ²                  (24)
```

Cancel the 1's, multiply by `|𝒬₀(jϖ)|²/ϖ²` (recall `𝒮₀ = 𝒩₀/𝒬₀`):

```
    h̄² |𝒬₀(jϖ)|²/ϖ²  ≥  ϖ⁻²( |𝒩₀|²|1−𝒟|² − 2ℜ{𝒩₀𝒬₀*[1−𝒟]} )
```

which is the paper's eq. (25), with

```
    |𝒬₀(jϖ)|² = ϖ⁶ + (1 − 2κφ)ϖ⁴ + κ(κφ² − 2)ϖ² + κ²
```

**Verify that expansion** (F-15). `𝒬₀(jϖ) = −jϖ³ − ϖ² + jκφϖ + κ`, so real
part `κ − ϖ²`, imaginary part `κφϖ − ϖ³`:

```
    |𝒬₀|² = (κ − ϖ²)² + (κφϖ − ϖ³)²
          = κ² − 2κϖ² + ϖ⁴ + κ²φ²ϖ² − 2κφϖ⁴ + ϖ⁶
          = ϖ⁶ + (1 − 2κφ)ϖ⁴ + (κ²φ² − 2κ)ϖ² + κ²                       ✓
```

### The trigonometric obstruction, and how it is removed  (eqs. 26, 27)

The right-hand side of (25) contains `ς` and `σ` — transcendental functions of
`δϖ`. You cannot turn that into a polynomial condition directly. Köroğlu
bounds them with two elementary inequalities (eq. 26):

```
    2(1 − cos θ) ≤ θ²        and        θ sin θ ≤ θ²
```

*Proofs.* First: `1 − cos θ = 2sin²(θ/2) ≤ 2(θ/2)² = θ²/2`. Second:
`sin θ ≤ θ` for `θ ≥ 0`, multiply by `θ ≥ 0`. (For `θ < 0` both hold by
evenness/oddness.) ∎

With `θ = δϖ` and `δ ≤ δ̄τ` (so in scaled units `δϖ ≤ δ̄ϖ`):

```
    2ς ≤ δ̄²ϖ²  ,        σϖ ≤ δ̄ϖ²
```

Applying these to the RHS of (25) (eq. 27):

```
    κ(φϖ² + 1)·2ς + 2κ(φ − 1)·σϖ  ≤  δ̄²κφϖ⁴ + δ̄κ[δ̄ + 2(φ − 1)]ϖ²
```

**This is where conservatism enters**, and Köroğlu says so: "Since the upper
bound in (27) converges to zero as δ̄ → 0, we might be optimistic about
relaxing (25) based thereon." The bounds are tight as `θ → 0` (both are exact
to second order), so the result is *asymptotically* exact for small delay and
progressively conservative for large delay.

---

## 4.4 The cubic positivity condition  (eqs. 28, 29)

Substituting (27) into (25) and writing `p := ϖ² ≥ 0`, the sufficient
condition becomes a **cubic nonnegativity on the nonnegative real axis**:

```
    p³ + (1 − 2κφ − h̄⁻²δ̄²κφ) p²
       + κ( κφ·φ − 2 − h̄⁻²δ̄²[1 + 2δ̄⁻¹(φ−1)] ) p
       + κ² ≥ 0 ,      ∀ p ≥ 0                                          (28)
```

*(The paper writes `φ` for `ϖ²` here, colliding with the controller parameter
`φ`. Use a different letter on the whiteboard.)*

### The necessary condition on the gains  (eq. 29)

Let `ψ := κφ` (a natural single parameter: the "loop gain"). Nonnegativity of
the coefficients in the limit `h̄ → ∞` (i.e. `h̄⁻² → 0`) requires

```
    p² coefficient:   1 − 2ψ ≥ 0             ⟹  ψ ≤ 1/2
    p¹ coefficient:   κ²φ² − 2κ ≥ 0  ⟺  κφ² ≥ 2  ⟺  ψφ ≥ 2  ⟹  ψ ≥ 2/φ
```

Together:

```
    2/φ  <  ψ = κφ  <  1/2         ⟹     φ > 4                          (29)
```

**Read this.** Even with *unlimited* headway, the controller parameters are
boxed in: the loop gain `ψ` must lie in a window, and the lead zero must
satisfy `φ > 4` — four times more conservative than internal stability alone
demands (`φ > 1`, eq. 22). Köroğlu flags it as "an indication of potential
conservatism", since with `δ̄ = 0` there is genuinely no string-stability
requirement at all (§4.1). The gap between `φ > 1` and `φ > 4` is the price of
the trigonometric relaxation.

### The two sufficient conditions  (eqs. 30, 31)

Requiring the `p²` and `p¹` coefficients nonnegative at finite `h̄`:

```
    δ̄²/h̄²  ≤  1/ψ − 2                                                   (30)
    δ̄²/h̄²  ≤  (ψφ − 2)/(1 + 2δ̄⁻¹(φ − 1))                                (31)
```

*Derivations.* (30): `1 − 2ψ − h̄⁻²δ̄²ψ ≥ 0 ⟹ h̄⁻²δ̄² ≤ (1−2ψ)/ψ = 1/ψ − 2`.
(31): divide the `p` coefficient by `κ > 0`: `κφ² − 2 − h̄⁻²δ̄²[1+2δ̄⁻¹(φ−1)] ≥ 0`,
and `κφ² = ψφ`. ∎

---

## 4.5 Theorem 1 and its proof  (eqs. 32–39)

> **Theorem 1.** The CACC with feedforward (14) and feedback (15) and an
> uncertain constant delay `δ ∈ [0, δ̄τ]` is string stable provided
>
> ```
>     h̄ = h/τ  >  h̄_o = √( 2δ̄ ( √(1 + 0.5δ̄) + 1 ) )                    (32)
> ```
>
> A suitable controller is
>
> ```
>     φ = (2h̄² + δ̄²)(2h̄² − 2δ̄ + δ̄²) / (h̄⁴ − 4δ̄h̄² − 2δ̄³)                (33)
>     κ = (1/φ) · h̄²/(2h̄² + δ̄²)                                        (34)
> ```

### Proof

**Step 1 — balance the two constraints.** For a fixed `φ`, increasing `ψ`
*decreases* the RHS of (30) (it is `1/ψ − 2`) and *increases* the RHS of (31)
(it is `(ψφ−2)/…`). The `ψ` that maximises the smaller of the two — hence
minimises the required headway — is the one that makes them **equal** (eq. 35):

```
    (ψφ − 2)/(1 + 2δ̄⁻¹(φ−1)) = 1/ψ − 2
```

Write `A := 1 + 2δ̄⁻¹(φ−1)`. Cross-multiplying, `ψ²φ − 2ψ = A − 2Aψ`, hence
`ψ²φ − 2ψ + 2Aψ − A = 0`; divide by `ψ²` and use `2A − 2 = 4δ̄⁻¹(φ−1)`:

```
    φ + 4δ̄⁻¹(φ−1)ψ⁻¹ − [1 + 2δ̄⁻¹(φ−1)]ψ⁻² = 0
```

Multiply by `δ̄`:

```
    δ̄φ + 4(φ − 1)ψ⁻¹ − [δ̄ + 2(φ − 1)]ψ⁻² = 0                            (35) ✓
```

**Step 2 — solve for `ψ⁻¹`.** A quadratic in `x := ψ⁻¹`:
`[δ̄ + 2(φ−1)]x² − 4(φ−1)x − δ̄φ = 0`, whose positive root is

```
              2(φ − 1)         ⎡ ⎛   2(φ − 1)   ⎞²        δ̄φ        ⎤^{1/2}
    1/ψ_o = ───────────── + ⎢ ⎜ ───────────── ⎟  + ───────────── ⎥      (36)
             δ̄ + 2(φ−1)       ⎣ ⎝  δ̄ + 2(φ−1)  ⎠     δ̄ + 2(φ−1)   ⎦
```

**Step 3 — `1/ψ_o` is increasing in `φ`, so push `φ → ∞`.** Larger `ψ⁻¹` means
a larger RHS in (30), hence a smaller required `h̄`. Taking the limit:

```
    2(φ−1)/(δ̄ + 2(φ−1)) → 1 ,      δ̄φ/(δ̄ + 2(φ−1)) → δ̄/2
    ⟹  1/ψ_o → 1 + √(1 + δ̄/2)
```

Substituting into (30):

```
    δ̄²/h̄²  <  lim_{φ→∞}(1/ψ_o − 2) = √(1 + 0.5δ̄) − 1                    (37)
```

**Step 4 — invert and rationalise.**

```
    h̄²  >  δ̄² / (√(1 + 0.5δ̄) − 1)
        =  δ̄² (√(1+0.5δ̄) + 1) / ((1 + 0.5δ̄) − 1)          [× conjugate]
        =  δ̄² (√(1+0.5δ̄) + 1) / (0.5δ̄)
        =  2δ̄ (√(1 + 0.5δ̄) + 1)
    ⟹  h̄  >  √( 2δ̄(√(1 + 0.5δ̄) + 1) ) = h̄_o                           (32) ✓
```

**Step 5 — construct the controller.** For a *given* `h̄ > h̄_o`, take equality
in (30) and (31) (eqs. 38, 39):

```
    from (30):  1/ψ = 2 + δ̄²/h̄² = (2h̄² + δ̄²)/h̄²   ⟹  ψ = h̄²/(2h̄² + δ̄²)  (38)
    from (31):  δ̄² + 2δ̄(φ−1) = h̄²(ψφ − 2)
                ⟹ φ(h̄²ψ − 2δ̄) = 2h̄² − 2δ̄ + δ̄²
                ⟹ φ = (2h̄² − 2δ̄ + δ̄²)/(h̄²ψ − 2δ̄)                       (39) ✓
```

Substituting (38) into (39) gives (33), and `κ = ψ/φ` gives (34). ∎

### Sanity-check the construction

Take the paper's example: `δ̄ = 0.2`, `h̄ = h̄_o + 0.02`.

```
    h̄_o = √(2(0.2)(√1.1 + 1)) = √(0.4 × 2.04881) = √0.81952 = 0.90527
    h̄   = 0.92527          (paper: "h = 0.9253τ")                        ✓
    ψ   = h̄²/(2h̄² + δ̄²) = 0.85613/(1.71226 + 0.04) = 0.48857
```

`ψ = 0.48857`, just under the `ψ < 1/2` ceiling of (29) — the design sits
against the constraint, as expected for a near-optimal headway. The paper
notes exactly this: "`ψ = κφ` stays in the neighborhood of 0.5 for both
controllers, which is the limit value as `δ̄ → 0`."

With `τ = 0.5 s`: `δ = 0.1 s`, `h = 0.4626 s` — a **sub-half-second headway**,
far below the ≈0.7 s Ploeg achieved experimentally and the 0.95 s of Ma's
Case 1.

---

## 4.6 The most quotable consequence: √δ̄ scaling

Expand `h̄_o` for small `δ̄`. Since `√(1 + 0.5δ̄) → 1`:

```
    h̄_o  ≈  √(2δ̄ · 2)  =  2√δ̄            (δ̄ → 0)
```

**The minimum time headway scales as the *square root* of the communication
delay, not linearly.** Numerically:

| `δ̄ = δ_max/τ` | `h̄_o` exact | `2√δ̄` | at τ = 0.5 s: δ_max, h_min |
|---|---|---|---|
| 0.05 | 0.4523 | 0.4472 | 25 ms, 0.226 s |
| 0.10 | 0.6364 | 0.6325 | 50 ms, 0.318 s |
| 0.20 | 0.9053 | 0.8944 | 100 ms, 0.453 s |
| 0.40 | 1.2947 | 1.2649 | 200 ms, 0.647 s |
| 0.60 | 1.6026 | 1.5492 | 300 ms, 0.801 s |
| 0.80 | 1.8690 | 1.7889 | 400 ms, 0.935 s |
| 0.90 | 1.9919 | 1.8974 | 450 ms, 0.996 s |

Two readings:

* **Halving latency does not halve the headway** — it buys a factor `√2 ≈ 1.41`.
  Squeezing the last milliseconds out of the radio has diminishing returns.
* **`h̄ = 2` is the natural reference.** Köroğlu notes `h̄ = 2` "is known to be
  achievable with a PD-type feedback-only controller [20]" — i.e. with
  `h = 2τ`, ACC-style feedback suffices and communication buys you nothing.
  The plots are therefore drawn for `δ̄ ≲ 0.9`, exactly the range where
  `h̄_o < 2` and CACC is worth having. **If your delay exceeds `0.9τ`, the
  radio has stopped paying for itself.** At τ = 0.5 s that threshold is
  450 ms — comfortably above real DSRC/C-V2X latencies (10–50 ms), so
  cooperative control is worth it in practice, but the bound tells you where
  the cliff is.

Compare with **Ma's noiseless floor** `2τ₀/(1+k_a) ∈ (τ₀, 2τ₀]`, i.e.
`h̄ ∈ (1, 2]`. Ma's structure never gets below `h̄ = 1` even with a perfect
channel (Part III §3.5), whereas Köroğlu reaches `h̄ = 0.9` at `δ̄ = 0.2`. The
difference is the `ℋ⁻¹` precompensator plus unity feedforward.

---

## 4.7 The price of optimality  (§III-B)

Designing at `h̄ ≈ h̄_o` is *possible* but comes at a cost that Köroğlu is
unusually honest about. Three metrics, all in his Fig. 2:

### 1. Error response to preceding-vehicle manoeuvres

Governed by `ℳ` (eq. 13). Normalising out `τ` and `δ` (eq. 40):

```
    ℳ(s) = τ² · [ δ̄ s̄ / 𝒬₀(s̄) ] · [ (1 − e^{−δ̄s̄ /?}) / (δ̄ τ s) ]
         =: τ² ℳ̄(s̄) Δ(s̄)
```

with `‖Δ‖_∞ ≤ 1` (from `|1 − e^{−jθ}| ≤ |θ|`, i.e. eq. 26 again). Hence

```
    ‖ℳ‖_∞ ≤ τ² ‖ℳ̄‖_∞                                                   (41)
    ‖ℳ‖₂² ≤ τ⁴ · (1/2π)∫|ℳ̄(jϖ)|²dϖ · τ⁻¹ = τ³‖ℳ̄‖₂²                   (42)
```

so the worst-case error response can be assessed **independently of τ** on
`ℳ̄`. The finding: "the worst-case error response degrades with increasing
communication delay, which is quite intuitive. It is perhaps more interesting
to observe that approaching optimality also improves the worst-case error
response."

*So minimum headway is not purely a sacrifice* — the same controller that
achieves it also tracks better. The `‖ℳ‖²₂` bound is the design-relevant one
because it gives the *squared-peak* spacing error from the *energy* of
`u_{i−1}` — i.e. it converts a harsh-braking scenario into a required
standstill gap `g_i`.

### 2. Delay margin — surprisingly good

Delay margin (DM) = maximum extra loop delay before instability = phase margin
÷ gain crossover frequency. Köroğlu's Fig. 2(d):

```
    DM  >  2τ    for large δ̄ ;    DM → 1.3τ   as δ̄ → 0 (perfect comms)
```

"a good level of stability robustness, and more so for the controller that is
closer to the optimum." So the aggressive design is *not* fragile to
unmodelled loop delay. This is a genuinely counter-intuitive and quotable
result.

### 3. Noise sensitivity — the real drawback

Noise sensitivity is `‖𝒞_h𝒮_h‖_∞` (the transfer from measurement noise to
control effort). Fig. 2(d):

> "We identify from this plot a major drawback of getting closer to the optimum
> time headway as the increase in the noise sensitivity (especially for small
> maximum communication delays). In view of this, it would be sensible to keep
> noise sensitivity low by using a desirably large δ̄ value in synthesis."

**The practical design rule that falls out:** *do not synthesise for the delay
you have; synthesise for a delay you can tolerate.* Deliberately over-stating
`δ̄` gives up a little headway and buys a lot of noise immunity.

**This closes the loop with Ma.** Ma assume clean radar and noisy V2V;
Köroğlu shows the *feedback* loop's noise sensitivity is what blows up when you
push headway to its floor. Put together: the minimum deployable headway is set
by whichever of the two noise sources dominates, and neither paper alone tells
you which.

---

## 4.8 Example simulation  (§IV)

`τ = 0.5 s`, `δ̄ = 0.2`, `h = 0.9253τ = 0.4626 s`, four followers, a scenario
where "the leading vehicle brakes harshly after an aggressive acceleration."

| | ideal case | with measurement noise + vehicle delays |
|---|---|---|
| `‖u₁‖₂/‖u₀‖₂` | 0.9705 | 0.9792 |
| `‖u₂‖₂/‖u₁‖₂` | 0.9918 | **1.0016** |
| `‖u₃‖₂/‖u₂‖₂` | 0.9966 | 0.9889 |
| `‖u₄‖₂/‖u₃‖₂` | 0.9987 | 0.9907 |

Read the table carefully — it is a model of honest reporting:

* Ideal case: all ratios `< 1`, L₂ string stability **verified computationally**.
* Non-ideal: one ratio (`1.0016`) is "barely violated" once vehicle actuation
  delays `v_i ∈ {0.2τ, 0.3τ, 0.4τ}` are activated. Köroğlu does not hide it.
* Spacing error stays **below one metre** in the ideal case, but "the negative
  influence of large vehicle delay is evident especially from the spacing
  errors of the first and second vehicles, which reach two metres."

The conclusion drawn is exactly the right one: "one needs to adopt a framework
that is suitable for time-varying delays" and to extend the result to
"heterogeneous platoons with vehicle models that include the vehicle actuation
delay as well."

---

## 4.9 How Part IV connects to the rest

| | Ma 2025 | Köroğlu 2024 |
|---|---|---|
| impairment | multiplicative **amplitude** noise on `a_{i−1}` | pure **transport delay** on `u_{i−1}` |
| uncertain quantity | `k̃a ∈ I`, `τ ∈ (0,τ₀]` | `δ ∈ [0, δ̄τ]` |
| feedforward | `k_a a_{i−1}`, `k_a` free | unity, through `ℋ` |
| precompensator | none | `ℋ⁻¹` present |
| noiseless / delayless floor | `h > 2τ₀/(1+k_a)`, i.e. `h̄ > 1` | `h → 0⁺` |
| result | `h_w,lb(k_a, ρ)`, `k_a*`, `h*` | `h̄_o(δ̄) ≈ 2√δ̄` |
| what limits you | channel SNR **and** actuator lag | delay only |

**This project's own extension (D-008) sits exactly between them**: it takes
Ma's CTHP family and adds an `e^{−θs}` on the feedforward path, giving

```
    H̃(s) = (k̃a s² e^{−θs} + k_v s + k_p)/(τs³ + s² + γs + k_p)
```

and bisects for the minimum string-stable headway. The measured result:

| effective gain | `h_min(θ=0)` | still ≤ 0.95 s until | `h_min(θ=0.5 s)` |
|---|---|---|---|
| noiseless (`k̃a = 0.5`) | 0.789 s | θ ≈ 0.23 s | 5.69 s |
| noisy low end (`k̃a = 0.4`) | 0.945 s | θ ≈ 0.42 s | 1.19 s |
| noisy nominal (`k̃a = 0.524`) | 0.751 s | θ ≈ 0.19 s | 8.35 s |

**Delay, not noise, is the binding constraint at realistic latencies** —
Ma's design keeps only ≈0.2 s of nominal delay budget, and beyond that the
required headway explodes. The counter-intuitive footnote: a *larger*
feedforward gain is *more* delay-sensitive (the low-end curve is flattest),
because the feedforward term is the one carrying the stale information.

Note this project's `h_min(θ)` **explodes** super-linearly, whereas Köroğlu's
bound is `≈2√δ̄`, which is *sub*-linear. No contradiction: Köroğlu **re-tunes
`κ, φ` for each `δ̄`** (eqs. 33, 34), whereas D-008 holds Ma's Case-A gains
**fixed** and asks only for headway. Same lesson as Ma's §2.6.2: closed-form
minima assume retuning; a fielded platoon with frozen gains hits a wall much
sooner. Whenever you quote a "minimum headway", state whether the gains move.

---

## 4.10 Whiteboard drill

1. Derive `u_i = 𝒮_hℱ_h𝒟u_{i−1} + 𝒞_h𝒮_h r_i` from the block diagram.
2. Show `𝒢 = ℋ − 𝒮_h(ℋ − ℱ_h𝒟)` and explain why `𝒟 = 1, ℱ_h = ℋ` gives
   `ℳ = 0`, hence `h → 0⁺`.
3. Derive `𝒮₀ = s̄²(s̄+1)/(s̄³+s̄²+κφs̄+κ)` in scaled variables.
4. Get `φ > 1, κ > 0` from Routh–Hurwitz.
5. Expand `|𝒬₀(jϖ)|²`.
6. Prove `2(1−cos θ) ≤ θ²` and `θ sin θ ≤ θ²`.
7. Derive `2/φ < ψ < 1/2 ⟹ φ > 4`, and explain what the gap to `φ > 1` means.
8. Balance (30) and (31), get (35), solve for `1/ψ_o`, take `φ → ∞`, and
   rationalise to `h̄_o`.
9. Show `h̄_o ≈ 2√δ̄` and state the practical consequence.
10. Name the three costs of operating at `h̄ ≈ h̄_o` and say which one bites.

---

*Next: [Part V — Zhao et al.: resilient control against false data injection](05-zhao-fdia-resilient.md).*
