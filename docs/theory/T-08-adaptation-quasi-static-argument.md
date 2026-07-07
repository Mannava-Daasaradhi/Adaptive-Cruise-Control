# T-08 — Why slow adaptation preserves string stability (the argument)

**Code:** `HeadwayAdapter.update`, `GainScheduler.update` (rate limits) ·
**Status:** engineering argument + continuous verification; a formal
switching-systems proof is future work for the journal version.

## Claim

The adaptive platoon (h_i(t), and with D-017 also (kp_i, kv_i)(t))
remains L2 string stable in practice because it moves *slowly through a
continuum of certified frozen designs*.

## The argument, in layers

1. **Frozen certification.** Every target the adapters emit satisfies
   ‖H̃(·; kp, kv, h)‖∞ ≤ 1 with margin: h targets are `margin +
   h_req(ρ̂/κ)` where h_req comes from the bisection at the (possibly
   re-tuned) gains, worst case over the noise interval. By construction
   there is no admissible target outside the certified set.
2. **Per-hop decoupling.** With one-vehicle look-ahead, hop i's transfer
   function depends only on follower i's parameters. Heterogeneous
   (h_i, kv_i) across the platoon therefore need no joint argument —
   each hop's ‖H̃_i‖∞ ≤ 1 suffices for the cascade (this is what makes
   per-vehicle adaptation admissible at all).
3. **Slow variation.** Rate limits: |ḣ| ≤ 0.05 s/s, relative gain slew
   ≤ 0.05 /s. The loop's fastest dynamics are 1/τ = 2 s⁻¹ and the
   dominant error dynamics live below 0.5 rad/s; parameter variation at
   ~0.03 rad-equivalent/s is one to two orders slower — the classic
   frozen-time/slowly-varying-systems regime. The extra −ḣ·v term in ė
   is ≤ 1 m/s² equivalent at v = 20 (T-02), absorbed by the margin.
4. **Between-designs consistency (D-017).** Gains and headway slew toward
   the *same* target design (the adapter consumes the scheduler's h_req),
   so intermediate states interpolate between two certified points rather
   than wandering.
5. **Continuous spot-checking.** The in-force ‖H̃‖∞ is evaluated at probe
   times in fig4 and in every certification run (D-018) — the argument is
   never taken on faith. Measured: in-force values ≤ 0.99999 throughout
   adaptive runs (except the *deliberate* h-only-wall demonstration).

## What the argument does NOT cover (honesty section)

- **Transition transients:** a commanded Δh at speed v produces a
  v·Δh gap-adjustment wave (±14 m at the last follower in the ρ=3 zone
  run). These are bounded, commanded motions — not instability — but
  they are *outside* the L2-gain statement; min-gap safety during
  transitions is checked in simulation (certification min-gap ≥ 5 m
  criterion) rather than proven.
- **Simultaneity:** all followers adapting at once compounds the
  transition waves down the string (observed L2 ratios up to ~1.11 in
  windows containing transitions). Staggering is the known mitigation
  (roadmap).
- **Estimator-in-the-loop:** ρ̂ jitter maps through the steep h_req curve
  to h(t) jitter (visible ±0.05 s wiggle on the zone plateau). Bounded by
  the rate limit; a smoothed ρ̂ is a one-line improvement (roadmap).

## For the journal version

The clean formalization route: dwell-time/average-dwell-time results for
switched linear systems with L2 gains (Hespanha-style), using the margin
to buy the dwell-time constant, plus an ISS-style bound for the commanded
transition inputs. The simulation infrastructure to validate any such
bound already exists (probe-time ‖H̃‖∞ + seeded Monte-Carlo).
