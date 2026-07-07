# E-06 — The certification suite (running and extending)

**Script:** `certify.py` · **Decision:** D-018 · **First full run:**
`results/certification/20260707-144611` — CERTIFIED, 5 seeds, 11/11 checks.

## What it is
The product artifact: a seeded Monte-Carlo scenario matrix with explicit
acceptance criteria, emitting a self-contained `report.html`
(PASS/FAIL banner, per-suite check tables, embedded trace thumbnails)
plus `metrics.json` for CI gating.

## Current suites and criteria
| suite | criteria |
|---|---|
| base-case-a | ‖H̃‖∞(0.95, ρ=5) ≤ 1 · L2 ≤ 1.05 ∀ seeds · min gap ≥ 5 m |
| qos-zone | in-force ‖H̃‖∞ ≤ 1+1e-4 ∀ phases/seeds · ρ̂ err ≤ 30 % · gap ≥ 5 m |
| deep-zone-gains | in-force ‖H̃‖∞ ≤ 1+1e-4 in ρ=2 zone · gap ≥ 5 m |
| delay-predictor | compensated ‖H̃‖∞ ≤ 1+1e-3 · L2 ≤ 1.05 · gap ≥ 5 m |

## Running
    python scripts/certify.py --seeds 5        # full, ~4 min
    python scripts/certify.py --quick --seeds 2  # smoke, ~1 min

Determinism: seeds 1..N — the same command certifies the same thing
forever; a verdict change means the *code* changed.

## Extending (the D-018 pattern — follow it exactly)
1. Write `suite_<name>(sc, seeds, quick) -> (checks, last_res)` using the
   `check(name, criterion, value, ok)` helper — criterion stated as a
   string next to the measured value (auditability is the point).
2. Register in the `suites` dict with a one-line description (it renders
   into the report).
3. Criteria must be *conservative and justified* — cite the doc that
   derives the threshold in the suite's docstring.
4. New features land WITH a suite (deep-zone-gains landed in the same
   change as D-017 — that is the standard).

## Threshold provenance
- ‖H̃‖∞ tolerances: numerical grid tolerance (T-03) — 1e-4/1e-3.
- L2 ≤ 1.05: outside the measured seed envelope at 10 ms noise hold
  ([0.981, 1.007]) with margin for the predictor's injection (T-06).
- min gap ≥ 5 m: the standstill design distance d — going below it in a
  *linear-regime* run means something is deeply wrong.
- ρ̂ ≤ 30 %: 3× the measured tracking error (~4 %) — loose by intent;
  tighten after the ROS backend's noisier pairing is measured.

## Planned suites (roadmap)
ROS-backend cross-validation (post Workstream A), emergency-brake
saturation stress, heterogeneous-τ platoon, packet-loss + adaptation
combined.
