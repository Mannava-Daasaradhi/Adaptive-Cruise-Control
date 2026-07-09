# D-022 — Physics-consistency V2V gate (bounded-injection defence)

**Date:** 2026-07-09 · **Status:** accepted, **validated** · **Type:** novelty
/ security

## Context
The CACC/CTHP feedforward ``u_ff`` is the predecessor's acceleration delivered
over the V2V radio, and the CTHP law applies it as ``ka * u_ff`` ([D-004]).
That radio is an *external input*: a compromised or impersonating node can put
an arbitrary value on it. String stability ([D-007]) says nothing about this —
it bounds the platoon's response to *honest* disturbances, not to a lying
input. A raw follower therefore has **unbounded** exposure: a phantom-throttle
(or phantom-brake) spoof drives ``ka * (spoof)`` straight into the command and
collapses the gap. In simulation a +5 m/s² phantom acceleration on one link
rear-ends the attacked follower (gap −27 m, peak spacing error 50 m).

Crucially, every follower already carries a **second, independent** estimate of
the same quantity — its radar (range + range-rate, differentiated with the
follower's own IMU). Radar is a separate physical sensor on the ego vehicle; an
attacker on the V2V bus cannot touch it. That independence is a free
consistency check the reactive/predictive QoS layers ([D-016]/[D-021]) never
used. This is part **B** of the flagship (A predictive spacing + B this gate +
C the chance-constrained certificate).

## Decision — a trust gate that fuses V2V toward radar
`cacc.trust.TrustGate` compares the received feedforward ``a_v2v`` against the
radar estimate ``a_radar`` of the predecessor's acceleration. With the
inconsistency ``r = |a_v2v - a_radar|`` and a trust weight

    g(r) = 1 / (1 + (r / r0)^2)   in (0, 1],

it fuses

    u_ff_eff = g * a_v2v + (1 - g) * a_radar.

Consistent messages (``r`` small) pass through unchanged (``g -> 1``);
inconsistent messages (``r`` large) fall back to the locally-measured radar
value (``g -> 0``). One gate guards each follower's incoming link.

### Bounded-injection guarantee (the certificate)
The deviation of the fused feedforward from the radar-consistent value is

    |u_ff_eff - a_radar| = g(r) * r = r / (1 + (r/r0)^2)  <=  r0 / 2

for **every** ``r >= 0``, with equality at ``r = r0``. The cap is *independent
of the spoof magnitude*: however large the lie, the spurious command the CTHP
law can inject is at most ``ka * r0 / 2`` (0.75 m/s² for the case-A ``ka = 0.5``,
``r0 = 3``), versus ``ka * |spoof|`` (unbounded) for the raw design. The bound
is stated relative to the radar estimate and holds unconditionally; the safety
bound relative to ground truth is ``r0/2 + |radar error|``, so an honest,
low-noise radar makes the injected command provably small.

### Inert under honest operation
Under the validated channel-noise model the honest disagreement is
``r = |w - 1| * |a_prev| + radar noise`` — a few hundredths of a m/s² for
``rho >= 10`` — so ``g ≈ 1`` and the gate does not perturb the feedforward. The
string-stability results of [D-016]/[D-021] therefore carry over unchanged
(verified: gate-on vs gate-off spacing errors differ by < 1e-3 m with no
attack, trust stays > 0.99). ``r0`` must exceed the worst *honest*
V2V-vs-radar disagreement — including a genuine emergency brake seen through
the channel (``|w-1| * 8 ≈ 0.8`` m/s² at ``rho = 10``) — so ``r0`` of a few
m/s² keeps the gate transparent to real traffic while still clamping spoofs
far below a dangerous command.

## Evidence (`scripts/spoof_defense_study.py`, `scenarios/spoof_defense.yaml`)
6 followers, case-A gains, good channel (``rho = 10``); a +5 m/s² phantom
acceleration is injected onto the link feeding follower #3 for t = 60–75 s,
ungated vs gated on the identical attack:

| metric (attacked follower #3) | ungated (raw CTHP) | gated (D-022) |
|---|---|---|
| min inter-vehicle gap | **−27.4 m (collision)** | **+8.4 m (safe)** |
| peak spacing error | 50.5 m | **14.2 m (−72 %)** |
| max injected command | ``ka * spoof`` ≈ 2.5 m/s² | **0.75 m/s² = ka·r0/2** |
| trust weight during spoof | — | collapses to 0.25, recovers to 1.0 |

The measured peak injection equals the certificate ``ka·r0/2`` exactly (the
bound is tight — hit as the ramp sweeps ``r`` through ``r0``), and the platoon
recovers fully once the message becomes consistent again.

### Honest limitation
The gate bounds the *instantaneous* injection, not the time-integral. Because
the CTHP spacing loop has low DC stiffness (``kp = 0.009``), a spoof that is
both large **and** persisted for tens of seconds still lets the bounded 0.75
m/s² accumulate — it converts an *unbounded, immediate* catastrophe into a
*bounded, slow, and detectable* drift, buying time for a supervisory trip or
the reactive fallback rather than guaranteeing zero drift under an unbounded-
duration attack. Certifying rejection of a *persistent* spoof (a trust
integrator / CUSUM persistence detector) is left to future work.

## Alternatives considered
- **Hard reject on threshold** (drop V2V when ``r`` > τ) — discontinuous,
  chatters at the boundary, and a smart attacker parks just under τ; the smooth
  ``g(r)`` degrades gracefully and gives the closed-form ``r0/2`` bound.
- **Cryptographic authentication only** — necessary but not sufficient: it
  stops impersonation, not a *compromised but authenticated* node, and adds no
  physics check. The gate is orthogonal and composes with it.
- **Kalman/observer fusion of V2V + radar** — stronger in principle but ties
  the guarantee to a noise model and a linearization; the static gate yields an
  auditable, model-free cap.

## What was added
- `src/cacc/trust.py` — `TrustConfig`, `TrustGate` (radar model + fusion).
- `src/cacc/network.py` — `LinkAttack` (bias / override / scale spoof, ramped,
  per-link); attacked links routed through `receive` (never passthrough).
- `src/cacc/platoon.py` — `PlatoonConfig.attack` / `.trust`; per-follower gate
  in the derivative; `SimResult.trust` / `.ff_inj` telemetry; loader support.
- `scripts/spoof_defense_study.py`, `scenarios/spoof_defense.yaml`,
  `tests/test_trust_gate.py` (12 tests; suite 77).

## See also
[D-004] · [D-007] · [D-016] · [D-021] · flagship part C (chance-constrained
string-stability certificate)
