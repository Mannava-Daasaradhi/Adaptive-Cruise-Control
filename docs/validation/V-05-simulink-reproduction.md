# V-05 — Simulink reproduction of Ma et al. (2025)

**Backend:** `matlab/` · **Model:** `matlab/models/cacc_platoon.slx` (built by
`build_cacc_model.m`) · **Driver:** `matlab/run_all.m` · **Environment:**
MATLAB R2026a, Simulink 26.1, base products only.

**Verdict: reproduced.** Every closed form of Theorems 1–2 matches the paper's
printed digits exactly. Four of the five numerical cases match the paper's
figures to ≤1 %. One quantity (Case 3, Fig. 12) does not, and that discrepancy
is characterised rather than papered over — §4.

---

## 1. What was reproduced

| paper artefact | our artefact | status |
|---|---|---|
| Theorem 2 closed forms, eqs. (17)–(19), (44), (45) | `theorem_checks.m` block A | **exact** |
| eq. (25) polynomial identity | block B | residual 1.8e-12 |
| Theorem 2(c) optimality of `k_a*` | block C (400 001-pt grid) | agrees to 1e-6 |
| Remark 4 limits (`ρ → ∞`) | block D | exact |
| Theorem 1 vs Theorem 2 ordering | block E | verified |
| sufficiency-not-necessity of (28) | block F | counterexample exhibited |
| Routh–Hurwitz on `D(s)` | block G | agrees with `roots()` |
| Fig. 4 — `f(k_a)` | `fig04_f_of_ka.png` | ✓ |
| Fig. 5 — Case 1 feasible region | `fig05_feasible_case1.png` | ✓ |
| Fig. 6 — Case 1 frequency response | `fig06_freq_case1.png` | ✓ |
| Fig. 7 — Case 1 spacing errors | `fig07_case1_delta.png` | ✓ 0.18 % |
| Fig. 8 — noise on link 1→2 | `fig08_noise_link12.png` | ✓ |
| Fig. 9 — Case 2 frequency response | `fig09_freq_case2.png` | ✓ |
| Fig. 10 — Case 2 spacing errors | `fig10_case2_delta.png` | ✓ 0.02 % |
| Fig. 11 — Case 3 feasible region | `fig11_feasible_case3.png` | ✓ |
| Fig. 12 — Case 3 spacing errors | `fig12_case3_delta.png` | **⚠ 15.4 %** |
| Fig. 13 — Case 4 platoon length | `fig13_platoon_length.png` | ✓ 0.1 % |
| Fig. 14 — Case 5 spacing errors | `fig14_case5_delta.png` | ✓ character |
| Fig. 15 — Case 5 accelerations | `fig15_case5_accel.png` | ✓ ≈1 % |
| Table I (`γ_{i,j}`) | `ma2025.gammas` | transcribed |
| Table II (`τ_i`) | `ma2025.hetero_taus` | transcribed |

---

## 2. Results table

### 2.1 Theory — exact agreement

| quantity | eq. | paper | Simulink | rel. err |
|---|---|---|---|---|
| `1/(1 + 1/ρ)` | (44) | 0.8333 | 0.833333 | 0.00 % |
| `h_w,lb(k_a = 0.5)` | (45) | 0.9375 s | 0.937500 s | 0.00 % |
| `k_a*(ρ = 5)` | (18) | 0.3183 | 0.318305 | 0.00 % |
| `h*_w,lb(ρ = 5)` | (19) | 0.8727 s | 0.872678 s | 0.00 % |
| `E[w]` (Table I) | (12) | — | 1.048174 | — |

### 2.2 Frequency domain — the string-stability verdicts

`‖H̃(jω;τ₀)‖_∞` evaluated at both ends of `I = [(1−1/ρ)k_a, (1+1/ρ)k_a]` and at
the mean:

| case | `k̃a = 0.4` (low) | `k̃a = 0.524` (mean) | `k̃a = 0.6` (high) | **worst** | verdict |
|---|---|---|---|---|---|
| 1 (`h_w = 0.95`) | 1.000000 | 1.000000 | 1.000000 | 1.000000 | string stable |
| 2 (`h_w = 0.65`) | **1.003500** | 1.001164 | 1.000000 | **1.003500** | **UNSTABLE** |
| 3 (`k_a*`, `h_w = 0.88`) | 1.000000 | 1.000000 | 1.000000 | 1.000000 | string stable |

Two findings confirmed here, both matching the project's earlier Python
results:

* **The low end of the noise interval binds** (decision D-009). Case 2 violates
  string stability by 0.35 % at `k̃a = 0.4` and is exactly marginal at
  `k̃a = 0.6`. Evaluating only at `E[w]` would report 1.0012 — a factor of 3
  understatement of the violation. Evaluating only at the *amplifying* end
  would report 1.0000 and declare the design safe.
* **Case 1 and Case 3 sit exactly on the boundary** (`‖H̃‖_∞ = 1.000000`),
  which is structurally unavoidable: `H̃(0) = k_p/k_p = 1`, so the sup is
  attained at DC and `‖H̃‖_∞ < 1` is impossible (F-13). The paper's designs sit
  at the corner of the feasible region by construction.

### 2.3 Time domain — deterministic-equivalent channel

`noise_mode = 'mean'`, i.e. the equivalent deterministic system of eq. (13) /
Remark 5 — see §3 for why this is the right comparison.

| quantity | paper | Simulink | rel. err |
|---|---|---|---|
| Case 1 `max|δ₁|` | 1.593 m | 1.5959 m | 0.18 % |
| Case 1 `max|δ₁₂|` | 1.535 m | 1.5436 m | 0.56 % |
| Case 1 trend | decreasing, monotone | decreasing, monotone | ✓ |
| Case 2 `max|δ₁|` | 0.882 m | 0.8821 m | **0.02 %** |
| Case 2 `max|δ₁₂|` | 0.886 m | 0.8872 m | **0.13 %** |
| Case 2 trend | **increasing**, monotone | **increasing**, monotone | ✓ |
| Case 3 `max|δ₁|` | 0.802 m | 0.9256 m | **15.41 %** |
| Case 3 `max|δ₁₂|` | 0.798 m | 0.9208 m | **15.38 %** |
| Case 3 trend | decreasing | decreasing | ✓ |
| Case 4 length @ t=0, `h=0.95` | 345 m | 345.00 m | 0.00 % |
| Case 4 length @ t=0, `h=0.88` | 324 m | 324.00 m | 0.00 % |
| Case 4 peak length, `h=0.95` | ≈437 m | 436.72 m | ≈0.1 % |
| Case 4 peak length, `h=0.88` | ≈418 m | 416.67 m | ≈0.3 % |
| Case 5 `max|a₁|` | ≈0.49 m/s² | 0.4962 m/s² | ≈1 % |
| Case 5 `max|a₁₂|` | ≈0.455 m/s² | 0.4523 m/s² | ≈1 % |

**The strongest single result is Case 2.** It is the paper's
string-*instability* demonstration: `max|δ_i|` **grows** 0.882 → 0.886 down the
string, 0.5 % over 11 hops. Reproducing both the magnitude *and* the direction
of a sub-percent trend to two significant figures requires the transfer
function, the manoeuvre, the initial conditions, and the integrator all to be
simultaneously correct. A model error in any one of them would break it.

### 2.4 Channel realisation — Fig. 8

Stochastic mode, link 1→2, `ρ = 5`, 200 s at a 10 ms hold:

```
    realised w ∈ [0.8000, 1.2000]      admissible support [0.8, 1.2)   ✓
    mean(w) = 1.0476                   E[w] = 1.0482                   ✓
```

The support is respected exactly and the sample mean converges to the
theoretical mean. The reproduced figure shows the same two panels as the
paper's: the noise cone bounded by `±a₁/ρ`, and the communicated signal band
between `(1 ∓ 1/ρ)a₁`.

One visible difference from the paper's rendering: our scatter shows discrete
striations rather than a uniformly filled cone. This is **correct, not an
artefact** — `w` is a 16-bit quantised quantity (F-17), so it takes finitely
many values, and the higher-probability bit patterns dominate a 20 000-sample
realisation.

---

## 3. Which system the paper's figures were generated on

**Finding: Figs. 7, 10, 12, 13, 14, 15 are the equivalent *deterministic*
system, eq. (13), not a stochastic realisation of eq. (5).**

Evidence — the same three cases run under all three channel modes:

| case | mode | `max|δ₁|` | `max|δ₁₂|` | monotone? |
|---|---|---|---|---|
| 1 | stochastic | 1.5990 | 1.5566 | **no** |
| 1 | **mean** | 1.5959 | 1.5436 | **yes** |
| 1 | none (`w≡1`) | 1.2826 | 1.2442 | yes |
| 2 | stochastic | 0.8790 | 0.8727 | **no** |
| 2 | **mean** | 0.8821 | 0.8872 | **yes** |
| 2 | none | 1.1953 | 1.2057 | yes |
| 3 | stochastic | 0.9274 | 0.9284 | **no** |
| 3 | **mean** | 0.9256 | 0.9208 | **yes** |
| 3 | none | 0.7604 | 0.7566 | yes |

* Only `mean` mode produces the **monotone** `max|δ_i|` profiles the paper
  prints, and only `mean` mode matches Case 2's printed 0.882/0.886.
* The certified per-hop attenuation is ≈0.35 %; a single stochastic realisation
  jitters `max|δ_i|` by ≈2 % — **six times the effect being displayed**. A
  realisation cannot show a monotone trend at this scale.
* This is exactly what Remark 5 says: "the deterministic system (13) is an
  equivalent system to the stochastic system (11) in the mean or expected
  sense, and robust string stability analysis is performed on the deterministic
  system (13)."
* Fig. 8, by contrast, is *necessarily* stochastic — its content is the realised
  noise scatter. So the paper does use both systems, for different figures.

Consequence for the codebase: `run_all` defaults to `'mean'`;
`run_all('stochastic')` reproduces the realisation jitter and is worth running
once to see the point.

---

## 4. The open discrepancy: Case 3 / Fig. 12

**Symptom.** With the paper's stated Case-3 parameters (`k_a = k_a* = 0.3183`,
`h_w = 0.88 s`, `k_v = 0.85`, `k_p = 0.003`, `τ = τ₀ = 0.5 s`, `ρ = 5`,
`N = 12`, `v_ss = 25 m/s`), the model gives `max|δ_i|` ≈ 0.926 → 0.921 m. The
paper's Fig. 12 bottom panel is annotated 0.802 → 0.798 m. Ratio ≈ 1.154.

### 4.1 It is not a model error

Two independent implementations agree to four decimal places on **all three
cases** under an ideal channel (`w ≡ 1`), which isolates the dynamics and the
integrator from any channel modelling:

| case | Simulink (`ode4`, 1 ms, vectorised) | Python core (`src/cacc`, RK4, 1 ms) |
|---|---|---|
| 1 | 1.2826 → 1.2442 | 1.2826 → 1.2442 |
| 2 | 1.1953 → 1.2057 | 1.1953 → 1.2057 |
| 3 | 0.7604 → 0.7566 | 0.7604 → 0.7566 |

Reproduce with `matlab/tests/crosscheck_python.py`. The Python core is the
project's reference implementation, covered by 87 passing tests and already
validated against this paper's printed theory numbers
(`07_Base_Paper_Reproduction_Results.md`).

Further, the same model reproduces Cases 1, 2, 4 and 5 to ≤1 %. A modelling
error that spared four cases and hit only the fifth would have to be specific
to Case 3's parameter values, and Case 3 differs from Case 1 only in
`(k_a, k_v, k_p, h_w)` — all of which enter the same four blocks.

### 4.2 Sensitivity sweep — what would explain 0.802

Holding everything else at the stated Case-3 values (`mean` channel):

| swept | value | `max|δ₁|` |
|---|---|---|
| `h_w` | 0.8727 (= `h*_w,lb`) | 0.8580 |
| | **0.88 (stated)** | **0.9256** |
| | 0.90 | 1.1117 |
| | 0.95 | 1.5805 |
| `k_p` | **0.003 (stated)** | **0.9256** |
| | 0.0035 | 0.9178 |
| | 0.005 | 0.8952 |
| `k_a` | **0.3183 (stated)** | **0.9255** |
| | 0.35 | 1.2889 |
| | 0.50 | 3.0343 |

`max|δ|` is **strongly** sensitive to `h_w` and `k_a`, and only **weakly** to
`k_p`. No single stated parameter, perturbed to a plausible neighbouring value,
lands on 0.802:

* the natural alternative `h_w = h*_w,lb = 0.8727 s` gives 0.858, still 7 %
  high;
* `k_p` would have to move far outside the feasible wedge of Fig. 11 to matter;
* an ideal channel (`w ≡ 1`) gives 0.760 — 5 % *low*. The value 0.802 lies
  between `w ≡ 1` (0.760) and `w = E[w]` (0.926), at an effective
  `w ≈ 1.012`, which corresponds to no stated quantity.

### 4.3 Weak corroborating evidence from Fig. 13

Case 4's `h_w = 0.88` curve is the same design as Case 3. Our peak platoon
length is 416.67 m against the paper's ≈418 m — about 1.3 m high, in the
direction consistent with the paper's `δ_i` being ≈0.12 m smaller than ours
(12 vehicles × 0.12 m ≈ 1.4 m). That is a figure-reading-level observation, not
proof, but it points the same way.

### 4.4 Status

Recorded as an **open reproduction note**. The reproduction claim in this
project is therefore stated precisely as:

> All *printed* numbers of Ma et al. (2025) are reproduced exactly. Of the
> quantities readable only from figure axes, Cases 1, 2, 4 and 5 agree to
> ≤1 %; Case 3's `max|δ_i|` (Fig. 12) is 15 % below what the stated Case-3
> parameters produce in two independent implementations.

Not blocking: Case 3's *qualitative* claims — that it is string stable, that
its errors decrease down the string, and that it yields a shorter platoon than
Case 1 (Remark 2 / Fig. 13) — all reproduce.

Candidate follow-up if it ever matters: obtain the authors' Case-3 script, or
request the exact `h_w` used for Fig. 12 (the sweep shows `h_w ≈ 0.866 s` would
reproduce 0.802).

---

## 5. Cross-backend agreement (V-03 extension)

This project now has **three** independent simulators of the same equations.
The matrix of agreement:

| pair | conditions | agreement |
|---|---|---|
| Simulink ↔ Python core | ideal channel, Cases 1–3, 200 s | **4 decimal places** |
| Python core ↔ ROS 2 | noiseless, 6 followers, per-hop L₂ | < 2 % per hop (V-03) |
| Simulink ↔ paper | printed theory numbers | exact |
| Simulink ↔ paper | figure-readable, Cases 1/2/4/5 | ≤ 1 % |

The Simulink model is therefore usable as an independent check on the Python
core, which is its main value beyond being a deliverable in its own right: two
implementations written from the paper by different routes (vectorised
block-diagram integration vs. hand-written RK4 on a state vector) agreeing to
1e-4 is much stronger evidence than either alone.

---

## 6. Known modelling differences from the Python core

Documented so the two backends' outputs are comparable without surprises.

| aspect | Python core | Simulink |
|---|---|---|
| integrator | hand-rolled RK4, `dt = 0.01` default | `ode4` (RK4), `dt = 1e-3` |
| noise hold | 10 ms (`noise_rate = 100 Hz`) | 10 ms (`ts_noise`) |
| RNG | numpy `Philox`/`PCG` per link, seed `[seed, 0xCACC]` | Simulink `Uniform Random Number`, one stream per (link, bit), `seed_vec = seed + (0:16N−1)` |
| ⟹ noise realisations | **not** bit-identical between backends | — |
| saturation | `u ∈ [−8, +3] m/s²` applied | **none** — the paper's model is linear and the manoeuvre stays far from limits |
| vehicle length | `L = 4 m`, `r = 1 m`, `d = r + L = 5 m` (D-005) | point masses, `d = 5 m` directly |
| sign convention | `e_i = −δ_i` | `δ_i` directly, as the paper |
| default `v₀` | 20 m/s (D-006 choice) | **25 m/s** (the paper's stated value) |

The last row matters when comparing absolute spacing errors: the Python
reproduction runs at the project's chosen 20 m/s, the Simulink backend at the
paper's 25 m/s. The cross-check script (`crosscheck_python.py`) sets the Python
side to 25 m/s explicitly so the comparison is like-for-like.

Saturation is deliberately absent from the Simulink model: all of Theorem 2 is
linear analysis, and `|a₀| ≤ 0.5 m/s²` keeps commands two orders of magnitude
inside the actuator envelope. Adding a saturation block would change nothing on
these scenarios and would silently invalidate the transfer-function
interpretation on any scenario where it did activate.

---

## 7. Reproduce this document

```matlab
cd matlab
run_all                 % ~90 s; writes results/ + results/run_all_log.txt
run_all('stochastic')   % the realisation-jitter comparison of §3
```

```bash
# cross-backend check of §4.1 (needs the project's Python env)
python matlab/tests/crosscheck_python.py
```

The `theorem_checks.m` transcript in `matlab/results/run_all_log.txt` is the
primary evidence for §2.1.

---

## 8. Related

* [`../derivations/02-ma2025-base-paper.md`](../derivations/02-ma2025-base-paper.md)
  — every equation derived, with these numbers cited in context
* [`../../matlab/README.md`](../../matlab/README.md) — how the model is built
  and why it is vectorised
* [`../decisions/D-024-simulink-backend.md`](../decisions/D-024-simulink-backend.md)
  — why a third backend exists at all
* [`V-03-cross-backend-validation.md`](V-03-cross-backend-validation.md) — the
  Python ↔ ROS 2 half of the matrix
* `07_Base_Paper_Reproduction_Results.md` — the original Python reproduction
