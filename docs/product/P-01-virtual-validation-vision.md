# P-01 — Product vision: virtual validation for cooperative driving

## The problem being sold
Validating a cooperative (V2V-coupled) driving controller today means
hardware: instrumented vehicles or scaled testbeds, a test track, radio
equipment, and weeks per configuration. Each *channel condition* — a
noise level, a latency, a loss pattern — is a physical setup. The result:
controllers are certified at a handful of operating points and deployed
into a continuum.

## The product
A **virtual validation platform** for CACC-class controllers that
replaces the testbed for the imperfection-robustness portion of
validation:

1. **Impairment-faithful channel** — published noise model (Ma 2025
   16-bit multiplicative), latency, loss, *time-varying quality*
   (interference zones) — every condition is a YAML line, not a lab day.
2. **Theory in the loop** — every configuration carries a frequency-
   domain verdict (‖H̃‖∞, worst case over the noise interval) computed
   from the same operators the simulation executes.
3. **Deterministic reproducibility** — seeded everything; a bug report
   is a (config, seed) pair; a certification is re-runnable forever.
4. **Distributed real-time execution** — one process per vehicle on
   ROS 2 with timing robustness designed in (wall-clock substepped
   integration, stamp-age extrapolation) and *measured* agreement with
   the monolithic reference (< 2 %/hop).
5. **Certification artifact** — one command → seeded scenario matrix →
   self-contained HTML report with PASS/FAIL against explicit criteria
   (D-018).

## Proof of capability (already delivered)
The platform reproduced a published T-ITS design to its printed digits,
then found and fixed three deployment gaps the paper could not see:
- the **fixed-gain wall** (operational h_req diverges from the re-tuned
  bound below ρ ≈ 3; infeasible ρ* ≈ 1.8) — T-07;
- the **noise × delay interaction** (each survivable alone at θ = 0.15 s,
  fatal together) and its fix, the timestamp predictor that holds the
  requirement flat to θ = 0.3 s — T-06;
- **closed-loop QoS adaptation** (estimate ρ̂ → re-tune (kp, kv) → adapt
  h) validated through interference zones down to ρ = 2 — D-016/D-017.

That arc — *reproduce, expose, fix, certify, all in simulation* — is the
product demo.

## Who buys it (positioning)
- ADAS/AD teams validating platooning/CACC features against comms KPIs.
- V2X stack vendors needing controller-level impact evidence for
  latency/loss budgets.
- Researchers: a reproducible benchmark harness with theory-anchored
  ground truth (every baseline number is pinned to a published paper).

## Moat / differentiation
Not the simulator (many exist) — the **verdict discipline**: pinned
reproduction of published theory, worst-case-over-channel frequency-
domain checks beside every time series, cross-backend agreement bands,
and per-claim provenance to metrics.json. That discipline is what lets
"simulation" stand where "testbed" stood.
