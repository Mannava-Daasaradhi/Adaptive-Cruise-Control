# T-03 — L2 string stability: definitions, transfer functions, verdicts

**Code:** `src/cacc/analysis.py` · **The single most important concept in
the project.**

## Definition

A homogeneous one-vehicle look-ahead platoon is L2 string stable when the
spacing-error propagation transfer function Γ(s) (from e_{i−1} to e_i)
satisfies

    ‖Γ(jω)‖∞ = sup_ω |Γ(jω)| ≤ 1 ,

i.e. no frequency component of a disturbance grows as it travels down the
string. Growth compounds geometrically: per-hop gain g over N hops gives
g^N — at the margin (g ≈ 1.002) growth is slow and *invisible in short
time series* (D-010), which is why every verdict in this project is
frequency-domain, with time series as illustration.

## Transfer functions implemented (`analysis.gamma`)

With G(s) = 1/(s²(τs+1)), C(s) = kp + kd·s, H(s) = h·s + 1:

    ACC:   Γ = G·C / (1 + G·C·H)
    CACC:  Γ = (G·C + e^{−θs}) / (H·(1 + G·C))          (Ploeg 2014)
    CTHP:  H̃ = (k̃a s² e^{−θs} P(s) + kv s + kp)
               / (τ s³ + s² + (kv + h·kp) s + kp)        (Ma 2025 + ours)

where k̃a = w·ka is the noise-scaled feedforward gain and P(s) is the
optional predictor lead (T-06). Delays are evaluated **exactly** on the
imaginary axis (e^{−jωθ}), no Padé approximation — a deliberate choice
(approximation error near the stability boundary would corrupt exactly
the verdicts we care about).

## Worst case over the noise interval (D-009)

w(t) ∈ [1−1/ρ, 1+1/ρ) makes k̃a interval-valued. **The binding end is
usually the LOW one** k̃a = (1−1/ρ)ka: less feedforward pushes work onto
feedback and violates the low-frequency condition (their eq. 28b) first.
The high end can bind too (e.g. ρ=3 with delay). All tooling therefore
evaluates ‖H̃‖∞ at *both ends* and takes the max — `hinf_worst` in the
study/certification scripts.

## Numerics

- Frequency grid `OMEGA_DEFAULT = logspace(−3, 2.5, 8000)` rad/s: the
  violating peaks in this project live between 0.02 and 0.5 rad/s; the
  grid resolves them to <0.1 %. Sanity: doubling the grid changes no
  reported verdict.
- `min_stable_headway` bisects on h (monotone for these families) to tol
  1e-3 s; raises if even h=10 s is unstable (that ValueError is *used* as
  the infeasibility signal by the adapters — capped to 10.0 in tables).

## Sufficient vs necessary — a project finding

The paper's design conditions (eqs. 28a/b) are *sufficient*, derived by
splitting the polynomial positivity condition by powers of ω². Direct
‖H̃‖∞ evaluation shows configurations violating 28a can still be string
stable at large h (the ω⁶ term rescues the dipped quadratic) — this is
why the fixed-gain requirement is computed by bisection, not the closed
form, and why ρ* (≈1.8) is smaller than the algebra suggests (≈3). See
T-07.

## Frozen vs time-varying configurations

‖H̃‖∞ certifies *frozen* designs. The adaptive pipeline moves slowly
(rate-limited) through a continuum of certified frozen designs — the
quasi-static argument (T-08). The certification suite additionally
evaluates the *in-force* configuration at probe times during runs
(fig4, D-018) so the argument is continuously spot-checked.
