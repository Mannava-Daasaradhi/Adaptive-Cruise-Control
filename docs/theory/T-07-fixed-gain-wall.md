# T-07 — The fixed-gain requirement, the wall, and ρ*

**Code:** `HeadwayAdapter._build_table` (bisection over the interval),
`GainScheduler` (the fix) · **Decisions:** D-016 (discovery), D-017 (fix).

## Fixed-gain vs re-tuned: two different questions

- **Re-tuned bound h_lb(ka, ρ)** (paper, eq. 17): "what headway suffices
  if I may pick (kp, kv) fresh for this ρ?" — a *design-time* quantity.
- **Fixed-gain requirement h_req(ρ; kp, kv, ka)**: "what headway does my
  *running controller* need at this ρ?" — the *operational* quantity an
  adapter must target. Computed by direct ‖H̃‖∞ bisection, worst case
  over both noise-interval ends.

Measured divergence (case-A gains, τ0 = 0.5):

| ρ | h_req fixed | h_lb re-tuned |
|---|---|---|
| 10 | 0.867 | 0.789 |
| 5 | **0.945** (= the case-A design!) | 0.9375 |
| 3 | 1.154 | 1.200 |
| 2.5 | 2.157 | 1.372 |
| 2 | 5.277 | 1.714 |
| 1.8 | ~8.3 (last feasible ≤ 10 s) | 1.87 |

The coincidence at ρ = 5 is not luck — case A was designed *for* ρ = 5,
so its gains are optimal there and the two notions meet. Away from the
design point they split, catastrophically below ρ ≈ 3.

## Anatomy of the wall

γ = kv + h·kp must satisfy the h-independent ceiling (28a):
γ ≤ (1−(1+1/ρ)²ka²)/(2τ0). With kv fixed at 0.63 and the ceiling shrinking
as ρ drops (0.6975 at ρ=10 → 0.4375 at ρ=2), h can only *add* to γ — more
headway pushes γ the wrong way. Large h still eventually stabilizes
(the sufficient condition is not necessary; the ω⁶ term of the positivity
polynomial rescues it — see T-03), but at absurd headways: the *economic*
wall (~ρ = 2.5) precedes the *feasibility* wall (ρ* ≈ 1.8, h ≤ 10 s).

Note ρ* is a grid statement: feasible at ρ = 1.8 on the bisection cap
h ≤ 10 s, infeasible at ρ = 1.5. Tighter grids move the number slightly;
the phenomenon (explosive growth below ρ ≈ 2.5) is robust.

## The fix: re-tune kv with ρ (D-017)

kv(ρ) = 0.9 × ceiling(ρ), kp scaled to keep kp/kv. Then:

| ρ | kv(ρ) | h_req at re-tuned gains |
|---|---|---|
| 10 | 0.628 (≈ case-A 0.63) | 0.870 |
| 5 | 0.576 | 1.033 |
| 3 | 0.500 | 1.319 |
| 2 | 0.394 | **1.877** (vs 5.28 fixed) |

The rule reproduces case-A at its design point and keeps every ρ ≥ 1.5
comfortably inside h_max = 2.5 s. Deep-zone validation: h-only saturates
at h_max with in-force ‖H̃‖∞ = 1.019; joint gains+h lands at 0.99999.

## Practical caveats

- The safety divisor κ on ρ̂ lands on the *steep part* of h_req below
  ρ ≈ 3 — estimation uncertainty is expensive there (measured: zone
  h = 1.76 vs omniscient 1.23 at ρ = 3). Improving ρ̂ confidence directly
  buys back headway.
- Between table grid points the requirement is interpolated; the grid is
  dense where the curve is steep (1.5, 2.0, 2.5, 3.0 …) — extend it
  before trusting the adapter outside [1.5, 50].
