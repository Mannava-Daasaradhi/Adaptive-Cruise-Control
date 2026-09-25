# D-026 — Digital twin of a production ACC from drive logs

**Date:** 2026-09-25 · **Status:** accepted, **validated on synthetic ground
truth; real-data run pending** · **Type:** product / research ·
**Depends on:** D-025 (product core), D-023 (verdict with uncertainty) ·
**Strategy:** P-04 roadmap step 2 · **GTM:** docs/company/GTM-01…03

## Context

P-04 names credibility as the product's biggest risk: buyers will not trust
simulated verdicts that are never connected to real cars. The JRC OpenACC
database (CC BY 4.0) holds 10 Hz platoon logs of 20+ production cars with
ACC engaged, and the literature found them all string-unstable. Connecting
the engine to such logs is the step that turns "the simulation says" into
"your car does".

The build environment's network policy blocks the JRC hosts
(`data.jrc.ec.europa.eu`, `jeodpp.jrc.ec.europa.eu`) and the usual mirrors
(figshare, Zenodo, Kaggle). The file layout was therefore taken from the
open ULTra-AV processing code that consumes OpenACC (CATS-Lab,
`Code/data_transformation.py::OpenACC_convert_format`: five metadata rows,
row 2 = vehicle names, then `Time, Speed{i}, IVS{i}, Driver{i}`, 10 Hz), and
the pipeline was validated where the truth is known.

## Decision

1. **`cacc.fielddata`** — `PlatoonLog` (uniform time base, speeds, bumper
   gaps, engagement mask), a tolerant OpenACC reader (finds the header by its
   `Time` cell, not a fixed row count), a documented generic CSV format for
   customer logs, writers for both, resampling of jittered time stamps, and
   ACC-engaged segment extraction.
2. **`cacc.twin`** — the twin is the linear lag ACC
   `u = k_s (s − s0 − T v) + k_v (v_lead − v)`, `a' = (u − a)/tau`, which is
   exactly the repository's CTHP law with `ka = 0`. Consequences: an exact
   analytic Γ, the simulator, test plans and a V2V what-if all apply to a
   calibrated twin with no new physics.
3. **Calibration by simulation**, not regression: the follower is simulated
   against the *measured* leader speed (exact LTI solution, first-order-hold
   input) and gap + speed trajectories are fitted by bounded least squares
   over all engaged segments, from three starts. The equilibrium gap at the
   mean speed is fitted instead of `s0` (far better conditioned when speed
   varies little).
4. **Verdict on the time-gap margin** `T − T_min` at 2 SE, not on
   ‖Γ‖∞: a stable lag ACC has ‖Γ‖∞ = 1 exactly (at ω → 0) whatever its gains,
   so ‖Γ‖∞'s standard error cannot say how close it is to the boundary; the
   margin can, and is also the number a customer can act on ("needs 0.9 s
   more time gap"). Standard errors from the Jacobian, inflated by the AR(1)
   effective-sample-size factor of the residuals.
5. **Model-free cross-check**: Welch H1 estimate |V_i/V_{i−1}| over
   coherent frequencies, reported next to the twin's verdict.
6. **Tools**: `cacc calibrate LOG [-o DIR]` (report, twin scenarios,
   calibration.json; exit 1 if any follower is string-unstable),
   `scripts/openacc_study.py` (whole database → per-vehicle table),
   `scripts/make_synthetic_openacc.py` → `examples/data/` sample, and
   `cacc init` (project scaffold with a release gate and CI workflow).

## Options considered

| option | verdict | reason |
|---|---|---|
| Regress acceleration on (gap, speed, Δv) | rejected as the estimator (kept as a start value) | acceleration must be differentiated from 10 Hz GNSS speeds; noise biases the gains |
| **Simulate-and-fit gap + speed (chosen)** | chosen | uses the signals that were measured; exact LTI solution is fast (~3 s per 300 s follower, including uncertainty) |
| Non-parametric only (Welch transfer estimate) | kept as a cross-check | needs rich excitation, window leakage flattens peaks (1.16 vs true 1.21 on synthetic data); no margin, no what-if |
| Nonlinear twin (IDM/FVD) | deferred | no closed-form Γ; would need the sweep per twin; the linear twin already gets IDM's verdict right |
| Pure response delay instead of lag | deferred | first-order lag is the standard abstraction and maps onto CTHP; a delay (Padé) variant is the first model extension if real residuals demand it |

## Evidence (`tests/test_twin.py`, 11 tests)

- **Known twins, 10 Hz + GNSS-like noise** (0.03 m/s, 0.05 m): all five
  parameters within 3 % (typically < 0.5 %), ‖Γ‖∞ within 0.01, residual RMSE
  equal to the injected noise, correct verdicts (unstable margin −0.99 s;
  stable margin +0.58 s).
- **Independent simulator**: logs from the RK4 platoon simulator (100 Hz →
  10 Hz + noise) calibrated by the LTI solver: T and tau within 1–3 %, ‖Γ‖∞
  within 0.01 of truth (1.214).
- **Model mismatch**: a nonlinear IDM platoon (large maneuvers) — twin
  ‖Γ‖∞ 1.04–1.05 ± 0.013 vs the IDM linearization 1.043 → unstable (correct);
  a stable IDM tuning → stable (correct).
- **Engagement**: a 40 s driver takeover splits the follower's data into two
  segments; segments shorter than the minimum are dropped.
- **Round trip**: twin → exported scenario → black-box sweep agrees with the
  twin's analytic ‖Γ‖∞ (1.212 vs 1.214).

## Limits

- **Not yet run on the real OpenACC files** (network policy). The reader
  follows a published consumer of the format; the first real run may need
  parser fixes (units, extra columns, missing values). The study script
  isolates per-file failures so one bad file cannot stop it.
- Linear small-signal twin around the log's mean speed; `s0` is the
  equilibrium intercept, not necessarily the physical standstill distance.
- The leader's *measured* speed drives the fit; logs without the
  predecessor's speed cannot be calibrated.
- Parameter uncertainty assumes AR(1)-like residuals; strongly nonlinear
  segments (stop-and-go) should be excluded by the analyst.
