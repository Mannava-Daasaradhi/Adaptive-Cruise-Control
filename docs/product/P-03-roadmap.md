# P-03 — Product & research roadmap (prioritized)

Near-term items carry acceptance criteria; far-term items carry intent.
Workstreams A–C of `10_Handoff_Plan.md` remain the active plan; this doc
is the horizon beyond them.

## Now (in the handoff plan)
- **A: ROS 2 QoS pipeline** — the full estimate/predict/adapt loop
  distributed; acceptance bands in 10 §3 A6 + V-03 extension.
- **B: IEEE paper** — skeleton in 10 §4; the contribution list now
  includes D-017 (gain re-tuning) and the certification methodology.
- **C: slides/demo** — sequence in 10 §5; fig4's live-margin panel and
  the certification report are the two closers.

## Next (research, ranked by value/effort)
1. **Staggered transitions** — serialize per-vehicle h ramps (vehicle i
   delays i·t_s); acceptance: transition-window L2 ratios ≤ 1.02 without
   verdict changes elsewhere. Small change, removes the ugliest artifact.
2. **Smoothed ρ̂** — rate-limit or low-pass the estimate before the
   steep h_req map; acceptance: zone-plateau h jitter < 0.02 s.
3. **Formal dwell-time result** for the adaptation (T-08 route) — the
   journal-version upgrade.
4. **Emergency-brake saturation suite** — |a0| = 6–8 m/s² scenarios
   exercising u_min; certification criterion on min-gap under worst
   seeds; connects the platform to the safety conversation.
5. **Heterogeneous platoons** — per-vehicle (τ, L, gains); per-hop
   verdicts already generalize (T-08 layer 2).
6. **Event-triggered beaconing** — the unbuilt option from the original
   novelty menu; bandwidth-vs-h_req tradeoff curve.

## Later (product hardening)
7. **CI pipeline** — GitHub Actions: pytest + reproduce --quick +
   certify --quick on every push; artifact-upload the HTML report.
8. **Config-sync checker** — scenario YAML ↔ ROS params YAML drift
   detector (R-11 known gap).
9. **Scenario DSL upgrades** — spatial (position-based) interference
   zones; per-link heterogeneous channels; scripted cut-in disturbances
   (needs lateral hooks — see V-04 #1).
10. **Multi-host deployment** — real network between vehicle processes,
    PTP clock sync characterization (V-04 #14); the step that makes the
    "distributed" claim literal.
11. **Plant-anchoring kit** — the P-02 boundary item: a system-ID
    recipe + import path for measured (τ, saturation, jerk) per vehicle
    class.

## Deliberately out of scope (with reasons)
- Full traffic simulation (SUMO-class) — different product; our value is
  controller-level certification depth, not network-level flow.
- Learning-based controllers — until the verdict discipline (worst-case
  frequency-domain checks) has an equivalent for them, they would
  dilute the platform's core promise.
