# Flagship Part B — Physics-consistency V2V Gate (in depth)

**Claim.** The follower's independent radar is an un-spoofable second opinion on
its predecessor's acceleration. Fusing the V2V feedforward toward the radar with
a trust weight `g(r) = 1/(1 + (r/r0)²)` caps the injectable feedforward at
`r0/2` **regardless of spoof magnitude** — a provable bounded-injection
certificate against V2V attack.

Companion ADR: [D-022](../decisions/D-022-physics-consistency-v2v-gate.md).
Overview: [00_Flagship_Overview](00_Flagship_Overview.md).

---

## 1. The problem: the feedforward is an untrusted input
The CTHP law applies the predecessor's acceleration as `ka * u_ff`, where
`u_ff` arrives over a V2V radio. That radio is an **external input**: a
compromised or impersonating node can put an arbitrary value on it. String
stability ([D-007]) says nothing about this — it bounds the response to *honest*
disturbances, not to a lying input. A raw follower therefore has **unbounded**
exposure: a phantom-throttle spoof drives `ka * (lie)` straight into the command
and collapses the gap. In simulation, a +5 m/s² phantom acceleration on one link
rear-ends the attacked follower (gap −27 m, peak spacing error 50 m).

Cryptographic authentication is necessary but not sufficient: it stops
impersonation, not a *compromised but authenticated* node, and it adds no
physics check.

## 2. The independent sensor
Every follower already carries a second estimate of its predecessor's
acceleration — its **radar** (range + range-rate, differentiated with the
follower's own IMU). Radar is a separate physical sensor on the ego vehicle; an
attacker sitting on the V2V bus cannot touch it. Part B uses that independence
as a consistency check.

In the simulator the radar observes the true predecessor acceleration `a_prev`
corrupted by a deterministic-in-`t` zero-mean noise (`TrustConfig.radar_noise`,
held piecewise-constant like the channel noise so RK4 stages stay consistent).
Modelling radar as *noisy but un-spoofable* is the honest idealisation — the
differentiation lag/noise is folded into `radar_noise`.

## 3. Mechanism — the trust gate
With `a_v2v` the received feedforward and `a_radar` the radar estimate, define
the inconsistency and a trust weight

    r = |a_v2v - a_radar|,      g(r) = 1 / (1 + (r/r0)²)  in (0, 1],

and fuse

    u_ff_eff = g·a_v2v + (1 - g)·a_radar .

Consistent messages (`r` small) pass through unchanged (`g -> 1`); inconsistent
messages (`r` large) fall back to the locally-measured radar value (`g -> 0`).
One gate guards each follower's incoming link (`cacc.trust.TrustGate`).

## 4. The bounded-injection certificate
The deviation of the fused feedforward from the radar-consistent value is

    |u_ff_eff - a_radar| = g(r)·r = r / (1 + (r/r0)²).

**Theorem.** `g(r)·r <= r0/2` for every `r >= 0`, with equality at `r = r0`.

*Proof.* Let `f(r) = r / (1 + (r/r0)²) = r·r0² / (r0² + r²)`. Then
`f'(r) = r0²·(r0² - r²) / (r0² + r²)²`, which is zero only at `r = r0`, positive
for `r < r0` and negative for `r > r0`; so `f` attains its maximum at `r = r0`,
where `f(r0) = r0·r0² / (2 r0²) = r0/2`. ∎

The cap is **independent of the spoof magnitude**: however large the lie, the
spurious command the CTHP law can inject is at most `ka·r0/2` (0.75 m/s² for
case-A `ka = 0.5`, `r0 = 3`), versus `ka·|spoof|` (unbounded) for the raw
design. The bound is stated relative to the radar estimate and holds
unconditionally; the safety bound relative to ground truth is
`r0/2 + |radar error|`, so an honest, low-noise radar makes the injected command
provably small.

## 5. Inert under honest operation
Under the validated channel-noise model the honest disagreement is
`r = |w - 1|·|a_prev| + radar noise` — a few hundredths of a m/s² for
`rho >= 10` — so `g ≈ 1` and the gate does not perturb the feedforward. The
string-stability results of [D-016]/[D-021] therefore carry over unchanged:
with no attack, gate-on vs gate-off spacing errors differ by `< 1e-3 m` and the
trust weight stays `> 0.99` all run. **`r0` must exceed the worst *honest*
disagreement**, including a genuine emergency brake seen through the channel
(`|w-1|·8 ≈ 0.8 m/s²` at `rho = 10`), so an `r0` of a few m/s² keeps the gate
transparent to real traffic while still clamping spoofs far below a dangerous
command.

## 6. Implementation
- `src/cacc/trust.py` — `TrustConfig`, `TrustGate` (radar model + `fuse`/`apply`).
- `src/cacc/network.py` — `LinkAttack` (`bias` / `override` / `scale` spoof,
  ramped, per-link); attacked links are routed through `receive` (never the
  passthrough shortcut) so the spoof actually reaches the controller.
- `src/cacc/platoon.py` — `PlatoonConfig.attack` / `.trust`; the gate is applied
  per-follower in the state derivative; `SimResult.trust` / `.ff_inj` record the
  trust weight and the injected feedforward for auditing.

## 7. Experimental setup (`scenarios/spoof_defense.yaml`)
6 followers, case-A gains, good channel `rho = 10` (honest disagreement well
below `r0`). A `bias` attack injects **+5 m/s²** onto the link feeding follower
#3, `t = 60…75 s`, faded in over 2 s. The study
(`scripts/spoof_defense_study.py`) runs **ungated** (`trust.enabled = false`)
and **gated** on the identical attack.

## 8. Results

| attacked follower #3 | ungated (raw CTHP) | gated (D-022) |
|---|---|---|
| min inter-vehicle gap | **−27.4 m (collision)** | **+8.4 m (safe)** |
| peak spacing error | 50.5 m | **14.2 m (−72 %)** |
| max injected command | `ka·spoof` ≈ 2.5 m/s² | **0.75 = ka·r0/2** |
| trust weight during spoof | — | collapses to 0.25, recovers to 1.0 |

**Figure** (`fig1_spoof_defense.png`, three stacked panels for follower #3):
(1) inter-vehicle gap — the ungated gap crosses 0 (collision), the gated gap
holds a safe standoff and recovers; (2) trust weight `g(t)` — collapses while
the message contradicts the radar, recovers after; (3) injected feedforward —
the raw design would inject the full `ka·spoof`, the gate caps it right at the
certificate line `ka·r0/2 = 0.75 m/s²`. The measured peak injection equals the
bound exactly (it is hit as the ramp sweeps `r` through `r0`).

## 9. Honest limitations
- The gate bounds the **instantaneous** injection, not the time-integral.
  Because the CTHP spacing loop has low DC stiffness (`kp = 0.009`), a spoof
  that is both large **and** persisted for tens of seconds still lets the
  bounded 0.75 m/s² accumulate — it converts an *unbounded, immediate*
  catastrophe into a *bounded, slow, detectable* drift, buying time for a
  supervisory trip or the reactive fallback rather than guaranteeing zero drift
  under an unbounded-duration attack. A trust-integrator / CUSUM persistence
  detector is left to future work.
- The guarantee is relative to the radar estimate; a compromised or blinded
  radar breaks the assumption (defence-in-depth: radar and V2V are independent
  attack surfaces, which is the point).

## 10. Relation to the rest
Orthogonal to Parts A and C: it touches only the feedforward path and is inert
when there is no attack, so it does not change the anticipatory spacing (A) or
the risk certificate (C). It composes with cryptographic authentication (which
it does not replace) as an independent physics layer.

## 11. Reproduce
```bash
python scripts/spoof_defense_study.py      # -> results/spoof_defense/<stamp>/
pytest tests/test_trust_gate.py -q          # 12 tests
```

[D-007]: ../decisions/D-007-reproduction-scope-and-validation.md
[D-016]: ../decisions/D-016-qos-adaptive-cacc.md
[D-021]: ../decisions/D-021-predictive-qos-map.md
