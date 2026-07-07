# D-018 — Monte-Carlo certification harness with HTML reporting

**Date:** 2026-07-07 · **Status:** accepted, first full run green ·
**Type:** product / V&V infrastructure

## Context
The project's product ambition (`10_Handoff_Plan.md` §0) is simulation
rigorous enough to **replace hardware testbeds** for controller
validation. A testbed's deliverable is not a plot — it is a *signed-off
verdict against explicit acceptance criteria*. The repo had all the
ingredients (deterministic seeds, theory-in-the-loop ‖H̃‖∞, scenario
matrix) but no single artifact a stakeholder could read as PASS/FAIL.

## Decision
`scripts/certify.py` (D-018) runs a **certification suite** — a matrix of
scenarios × seeds — and emits:
1. `report.html` — fully self-contained (inline CSS, base64-embedded
   thumbnails), dark-theme, with an overall CERTIFIED/FAILED banner and a
   per-suite table of *check / criterion / measured value / verdict*;
2. `metrics.json` — the same checks machine-readable, for CI gating.

Suites and their acceptance criteria (v1):

| suite | checks |
|---|---|
| base-case-a | ‖H̃‖∞(0.95, ρ=5) ≤ 1 · per-hop L2 ≤ 1.05 all seeds · min gap ≥ 5 m |
| qos-zone | in-force ‖H̃‖∞ ≤ 1+1e-4 all phases/seeds · ρ̂ tracking ≤ 30 % · min gap ≥ 5 m |
| deep-zone-gains | in-force ‖H̃‖∞ ≤ 1+1e-4 inside ρ=2 · min gap ≥ 5 m |
| delay-predictor | compensated ‖H̃‖∞ ≤ 1+1e-3 · per-hop L2 ≤ 1.05 · min gap ≥ 5 m |

Design choices: criteria are stated *in the report next to the measured
value* (auditability); thresholds live in one place (`certify.py`) and are
deliberately conservative; the last seed's error trace is embedded per
suite as a visual sanity check; seeds default to 1..5 (deterministic —
rerunning certifies the same thing).

## Why HTML and not a PDF/notebook
Self-contained single-file HTML opens anywhere with zero toolchain,
diffs in git, and embeds images without external assets — the right
artifact for "send the certification to a reviewer".

## Evidence
`results/certification/<stamp>/report.html` — first full 5-seed run:
**CERTIFIED, all checks green** (quick 2-seed validation also green).

## Consequences / future extensions (in the roadmap)
- CI hook: run `certify.py --quick` after every change; gate on
  `metrics.json` (documented in `docs/validation/V-01`).
- New features must land with a suite entry (the deep-zone-gains suite
  was added in the same commit as D-017 — the pattern to follow).
- Candidate additions: ROS-backend suites once Workstream A lands;
  actuator-saturation stress suite; heterogeneous-platoon suite.
