# P-02 — The hardware-replacement argument (and its honest boundary)

The claim "this simulation can stand in for a hardware testbed" must be
argued, not asserted. The argument has four legs plus a stated boundary.

## Leg 1 — Fidelity where it matters
The phenomena that decide CACC deployment are *communication
imperfections coupled to control dynamics*. Those are exactly what a
physical testbed struggles to produce **controllably** (you cannot dial
an interference zone to ρ = 3.0 on a track) and exactly what the
platform models natively, with a published, peer-reviewed channel model
as the contract (Ma 2025). For this phenomenon class, simulation is not
the cheap substitute — it is the *better instrument*.

## Leg 2 — Verdicts, not pictures
Testbeds output measurements that humans interpret. This platform
outputs **criteria-checked verdicts**: frequency-domain string-stability
margins (worst case over the channel), min-gap safety on the nonlinear
sim, statistical acceptance over seeds — assembled in a certification
report (D-018). The interpretation is in the artifact, auditable.

## Leg 3 — The reproduction anchor
Credibility is transferred, not claimed: the stack reproduces the
published design's printed numbers exactly (E-01) and its own novel
findings are continuous with that anchor (same operators, same code
paths). Anyone can re-run the anchor in minutes. A testbed cannot even
do that for itself.

## Leg 4 — Real-time transferability
The distributed backend runs the *same control code* per-vehicle under
genuine asynchrony, OS scheduling faults included, and agrees with the
mathematical reference to < 2 %/hop (V-03). The timing-robustness
patterns that survive WSL's 1.4 s freezes (wall-dt substepping,
stamp-age extrapolation, matched horizons) are the same patterns an
embedded deployment needs — the sim-to-real gap is narrowed by
construction, and the residual gap is *characterized*, not ignored.

## The honest boundary (what still needs hardware)
1. **Plant fidelity** — the τ-lag model needs one instrumented-vehicle
   system-identification campaign to anchor τ, saturation, and jerk
   limits per vehicle class. One campaign, reused forever.
2. **Radio ground truth** — the channel model's parameters (ρ ranges,
   latency distributions) need field characterization per radio stack.
   Again: measure once, then sweep in simulation.
3. **Perception artifacts** — radar multipath, association errors; the
   platform's radar surrogate is kinematically honest but sensor-clean.
4. **Final integration sign-off** — regulatory/track validation of the
   *integrated vehicle* stays physical; the platform's role is to make
   that final pass a confirmation, not an exploration.

**The replacement claim, precisely stated:** the platform replaces the
*exploration and robustness-certification* portion of testbed work — the
expensive, combinatorial part — and converts the physical program into
(a) one-time parameter anchoring and (b) final confirmation. That is
where the cost and time live, which is why the claim has product value
even with the boundary drawn honestly.
