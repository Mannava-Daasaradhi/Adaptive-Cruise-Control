# The CACC Flagship — A + B + Certificate

**One line.** On top of a faithfully reproduced Ma-2025 string-stable CACC
platoon, three composable extensions make the design *anticipatory*,
*attack-resilient*, and *risk-calibrated* — each with a rigorous, closed-form
guarantee and an honestly stated limit.

This document is the map. The three claims each have an **in-depth writeup**
and a one-page **architecture decision record (ADR)**:

| part | claim | in-depth | ADR | commit |
|---|---|---|---|---|
| **A** | Predictive QoS-map spacing | [F-A](F-A_Predictive_QoS_Spacing.md) | [D-021](../decisions/D-021-predictive-qos-map.md) | `c2d5288` |
| **B** | Physics-consistency V2V gate | [F-B](F-B_Physics_Consistency_Gate.md) | [D-022](../decisions/D-022-physics-consistency-v2v-gate.md) | `902d66b` |
| **C** | Chance-constrained risk certificate | [F-C](F-C_Chance_Constrained_Certificate.md) | [D-023](../decisions/D-023-chance-constrained-certificate.md) | `2fc4103` |

## The base we build on
The core is the constant-time-headway policy (CTHP) of Ma, Pagilla & Darbha,
*"Selection of Time Headway in CAV Platoons Under Noisy V2V Communication,"*
IEEE T-ITS 26(1) 2025. The control law for follower *i* behind predecessor
*i-1* is

    u_i = kp * e_i + kv * dv_i + ka * u_ff ,

with the spacing error `e_i`, relative speed `dv_i`, and the predecessor's
acceleration `u_ff` delivered over a V2V radio corrupted by the paper's 16-bit
multiplicative channel noise `w(t) in [1 - 1/rho, 1 + 1/rho)`. Our
implementation reproduces the paper's printed design numbers exactly (case A:
`kp=0.009, kv=0.63, ka=0.5, h=0.95 s, tau=0.5 s`; `h_lb=0.9375`, `ka*=0.3183`,
`h*=0.8727`), pinned as unit tests. String stability is the L2 condition
`||H~(jw)||_inf <= 1` on the spacing-error propagation transfer function.

Everything below keeps that physics **fixed** as the single source of truth and
adds an outer layer.

## The three claims, and why they compose
The claims are deliberately **orthogonal** — each addresses a different failure
mode of the base design, and they share the platoon simulator and the
frequency-domain `||H~||_inf` machinery without interfering:

* **A — anticipation** answers *"the headway opens too late."* The reactive QoS
  loop ([D-016]) only widens the gap after a follower is already inside an
  interference patch — where the good-channel headway is transiently
  string-unstable. A shared spatial channel map plus a preview lookahead opens
  the gap *before* entry, in the good channel.
* **B — integrity** answers *"the feedforward is an untrusted input."* A
  compromised V2V node can inject an arbitrary acceleration; the raw law
  applies `ka * (lie)` and collapses the gap. A physics-consistency gate fuses
  the radio message toward the follower's independent radar estimate, capping
  the injectable command at a provable bound.
* **C — calibration** answers *"how safe is 'string-stable', exactly?"* The
  binary `||H~||_inf <= 1` is enforced at a `~1e-6`-probability worst case. The
  exact 16-bit channel law turns it into a failure probability and a
  risk-parameterised headway — the principled replacement for A's heuristic
  safety margin.

Read as patent structure: **A and B are independent dependent claims**
(anticipatory spacing; bounded-injection security), and **C is the backbone**
that quantifies the risk both trade against.

## Headline results (all validated in-repo)

| part | metric | baseline | with the extension |
|---|---|---|---|
| **A** | in-zone time string-**unstable** (last follower) | 6.9 % (reactive) | **0 %** (predictive) |
| | peak spacing error in the ρ=3 patch | 17.2 m | **14.2 m (−17 %)** |
| | mean-headway cost of anticipation | — | +3 % |
| **B** | min inter-vehicle gap under a +5 m/s² spoof | **−27.4 m (collision)** | **+8.4 m (safe)** |
| | peak spacing error | 50.5 m | **14.2 m (−72 %)** |
| | max injected command (any spoof size) | unbounded | **0.75 = ka·r0/2** |
| **C** | worst-case vs 1-in-100-risk headway (ρ=3) | 1.154 s | **1.033 s (−10.5 %)** |
| | deployed case-A exceedance (h=0.95, ρ=5) | — | **0** (a.s. stable) |

## Honest limits (carried in each ADR)
- **A** relocates the gap-opening transient into the good channel but does not
  remove it; it costs ~3 % mean headway and cannot beat causality if the map is
  wrong (the reactive estimate is the backstop).
- **B** bounds the *instantaneous* injection, not its time integral: a large
  **and persistent** spoof still drifts slowly because CTHP's `kp` is tiny — it
  converts an unbounded catastrophe into a bounded, detectable drift (a CUSUM
  persistence trip is future work).
- **C**'s chance-constrained dividend is real (~10.5 % at ρ=3) but the
  mean-square notion is *too permissive* (7.6 % exceedance) and the
  distribution-free Chebyshev bound is *blunt* (collapses to worst-case). The
  certificate's honesty is part of the result.

## A structural finding worth stating
Part C revealed that the channel amplification `A(w) = ||H~(.; ka_eff=w·ka)||_inf`
is **two-tailed**: at a degraded channel, string stability needs `w` in a
*middle band* — too little feedforward (deep fade) and too much (noise
over-amplification) both break it. This is why the paper's `h_lb` carries the
`(1+1/rho)` high-end term, and why the robust worst-case headway is larger than
a naive low-end analysis suggests.

## Reproduction
Conda env `cacc` (Python 3.13, numpy 2.2). Full suite: **87 tests pass**.

```bash
python scripts/predictive_qos_study.py     # Part A  -> results/predictive_qos/
python scripts/spoof_defense_study.py      # Part B  -> results/spoof_defense/
python scripts/stochastic_cert_study.py    # Part C  -> results/stochastic_cert/
pytest -q                                   # 87 passed
```

## See also
Base paper reproduction and string-stability theory: `02_IEEE_Base_Paper.md`,
`docs/theory/T-02…T-07`. QoS-adaptive foundation the flagship extends:
`09_QoS_Adaptive_CACC.md`, [D-016]/[D-017]/[D-019]. Certification harness:
[D-018].

[D-016]: ../decisions/D-016-qos-adaptive-cacc.md
[D-018]: ../decisions/D-018-certification-harness.md
