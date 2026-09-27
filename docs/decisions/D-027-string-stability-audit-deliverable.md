# D-027 — The String-Stability Audit as a one-command deliverable

**Date:** 2026-09-27 · **Status:** accepted · **Type:** product ·
**Depends on:** D-026 (digital twin), D-025 (test plans) ·
**GTM:** GTM-01 §3 (the wedge offer), GTM-03 (demo)

## Context

GTM-01 sells a paid **String-Stability Audit** before any software license.
It promises the customer, per vehicle, a twin with parameter uncertainty,
the time-gap margin, ‖Γ‖∞ with a model-free cross-check, a V2V what-if
"(latency sweep)", a release-gate test plan they keep, and a report. It sets
the metric "time from log received to report delivered, target < 1 day".
After D-026 the numbers existed, but only as terminal text and JSON. The
latency sweep and the gate did not exist, and a report still had to be
hand-built for every audit.

Two findings shaped the V2V part. On the sample cars:

- the smallest string-stable time gap barely depends on latency at ka = 0.5
  (the limit is at low frequency);
- stronger feedforward trades gap for latency sensitivity, and can even be
  *worse*: the EV needs 0.86 s at ka = 0.5 but 1.18 s at ka = 0.8;
- ka = 1 is never strictly stable, because |Γ| → ka at high frequency.

A single what-if point hides all three effects.

## Decision

`cacc audit LOG [-o DIR]` (`src/cacc/audit.py`) turns one drive log into the
whole deliverable in seconds (12 s for the three-car sample).

1. **`report.html`.** One self-contained file with inline CSS and SVG and no
   network fetches, so a customer's confidential audit opens anywhere. It
   prints one car per page.
   - **Summary:** verdict per car, time gap in use vs. needed, margin,
     ‖Γ‖∞, model-free peak, a five-car brake result, and a one-line
     recommendation.
   - **Per car:** parameters ± standard error, the fit against the log,
     |Γ(jω)| with and without V2V plus the model-free point, and the V2V grid
     table and chart.
   - **Method, limits and reproduction:** the exact command, tool version
     and the log's SHA-256.
2. **V2V what-if grid.** For ka ∈ {0.3, 0.5, 0.8} × latency ∈ {0, 50, 100,
   200, 300} ms, the smallest string-stable time gap. Also a **latency
   budget** per ka: the largest latency at which the car's *current* gap is
   string-stable, found by bisection on the twin's exact boundary. This is
   the number a V2X vendor (ICP rank 1) asks for.
3. **Recommended time gap.** The boundary plus two standard errors of the
   margin, rounded up to 0.1 s.
4. **Road test.** Five cars tuned like the audited one, with the lead
   braking at 3 m/s² for 3 s from the log's mean speed: smallest gap, which
   car, and whether it is a collision. It runs as-calibrated and at the
   recommended gap. It is the twin's own scenario, the one the gate runs.
   A manager reads "car 5 collides" faster than "|Γ| = 1.31".
5. **`gate_<i>_<car>.yaml`.** A release-gate test plan per car:
   - cases *as-calibrated* and *recommended-time-gap*;
   - `min_gap ≥ 2 m` under the hard brake;
   - the multisine sweep with max gain 1.0.

   It loads with `cacc evaluate` next to its twin. Twins and
   `calibration.json` are written as by `cacc calibrate -o`, through the
   same writer.
6. **Exit codes** are the same as `cacc calibrate`: 1 if any car is
   string-unstable, 2 if nothing could be calibrated.

## Evidence (`tests/test_audit.py`, 8 tests; sample log)

- **Latency budget:** the budget lies on the boundary, inside at b and
  outside at b + 10 ms. A car no tested gain fixes gets `None`; a
  comfortably stable car gets the full 1 s range.
- **Recommended gap:** it clears the boundary by two standard errors, by
  less than one 0.1 s step.
- **Release gate:** the generated gate loads and evaluates to
  *as-calibrated FAIL, recommended PASS* for the unstable car. On the
  sample: SedanA at 1.2 s gives a **collision at car 5 (−7.0 m)**, with a
  sweep of 1.306 vs. the twin's analytic 1.309. At the recommended 3.0 s it
  keeps 48 m. The stable EV passes as calibrated.
- **Report:** vehicle names from the log are HTML-escaped (a `Car<U>` name
  is tested). There are three charts per car and no external references.
  It fits a 390 px phone screen with no horizontal page scroll; charts and
  the grid scroll inside their own box.

## Options considered

| option | verdict | reason |
|---|---|---|
| Keep text + JSON, hand-build reports per audit | rejected | defeats the "< 1 day" metric; every audit re-does layout work |
| PDF output | deferred | adds a heavy dependency; the HTML prints to PDF from any browser with page breaks per car |
| Run the release gates inside the audit | rejected (for now) | adds ~10 s per car for numbers the twin already gives exactly; the gate stays a file the customer runs after re-tuning |
| One what-if point (D-026) | superseded | hides the latency/feedforward trade-off above |

## Limits

- The road test and gate simulate the **twin**, a linear lag model around
  the log's mean speed with saturation at −8/+3 m/s². They are not a crash
  prediction for the real vehicle. The report says what the verdict means
  for a line of identical cars.
- The V2V what-if assumes a lossless link with the stated latency. Loss and
  noise are available in the simulator (D-016, D-023), but not yet in the
  audit.
- Everything inherits D-026's limits: the model is small-signal, `s0` is an
  intercept, and the predecessor's measured speed is required.
