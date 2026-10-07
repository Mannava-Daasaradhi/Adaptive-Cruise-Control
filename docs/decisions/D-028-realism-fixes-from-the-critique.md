# D-028 — Realism fixes from the 2026-10-08 critique

**Date:** 2026-10-08 · **Status:** accepted · **Type:** research + product ·
**Depends on:** D-016 (QoS adaptation), D-022 (spoof gate), D-025 (test
plans), D-026 (twin), D-027 (audit) · **Evidence:** `tests/test_realism.py`,
`tests/test_twin_structure.py`, `scripts/radar_lag_study.py`

## Context

An outside review of the project found that several headline results rest on
idealisations nobody had tested:

1. the spoof gate (D-022) and the channel estimator (D-016) used an **ideal
   radar**, the predecessor's true acceleration with no lag;
2. the V2V channel had no **latency jitter** and only **independent** loss,
   while V2X engineers budget for bursts and packet inter-reception time;
3. the plant had **no actuation dead time**;
4. the drive-log twin was validated only on data from **its own model
   family**, so its tiny error bars proved the optimiser, not the model;
5. the sweep never warned when its peak sat on the **edge of the band**, and
   only **L2** string stability could be gated.

Every default below keeps the old behaviour, so all published numbers still
reproduce. The new options are what a realistic study or audit should use.

## Decision

| # | change | where |
|---|---|---|
| 1 | Radar as a modelled sensor: `radar: {lag, noise}`; the gate and the estimator read it | `radar.py`, `platoon.py`, `scenario.py` |
| 2 | Gate options: `trust.match_lag` (compare the radar with the V2V value received `radar.lag` ago) and `trust.trust_braking` (never weaken a harder brake) | `trust.py` |
| 3 | `adaptation.rho_source: reported`: the sender reports its value-noise level (ETSI CAMs carry a confidence with the acceleration) | `estimation.py`, `platoon.py` |
| 4 | `network.delay_jitter` (exponential extra latency, newest-message-wins receiver) and `network.loss_burst` (Gilbert–Elliott, same long-run rate); `V2VLink.stats()` reports the worst packet inter-reception time | `network.py` |
| 5 | `vehicle.act_delay`: actuation dead time in the simulator and in the exact analytic Γ of ACC, CACC and CTHP | `vehicle.py`, `platoon.py`, `analysis.py` |
| 6 | Twin fits lag and lag + dead time (Padé in the fit, exact delay in the analysis), picks by an autocorrelation-aware BIC (dead time only on ΔBIC ≥ 10), uses a 5 s burn-in per segment, holds bound-pinned parameters fixed in the covariance, and reports `confidence` high/medium/low with warnings | `twin*.py` |
| 7 | Audit recommends the gap for the worse plausible structure and shows structure, confidence and warnings | `audit.py`, `audit_report.py` |
| 8 | Sweep result `peak_band_edge`; criterion metric `peak_amplification_max` (L∞) | `stringstab.py`, `criteria.py`, `reporting.py` |

## Evidence

**Radar lag vs the gate** (`scripts/radar_lag_study.py`, case-A gains, honest
−7 m/s² brake for 3 s, minimum gap in the platoon):

| radar lag [s] | h = 0.95 s original | h = 0.95 s lag-matched | h = 0.8 s original | h = 0.8 s lag-matched |
|---|---|---|---|---|
| 0.0 | 2.71 | 2.71 | 0.16 | 0.16 |
| 0.3 | 2.70 | 2.74 | **−0.26** | 0.10 |
| 0.5 | 2.04 | 2.74 | **−0.93** | 0.09 |
| 0.8 | 0.83 | 2.76 | **−2.14** | 0.08 |
| 1.2 | **−2.91** | 2.77 | **−6.02** | 0.05 |

Ungated: 2.74 m (h = 0.95 s) and 0.11 m (h = 0.8 s). Negative = collision.
Under a sudden +5 m/s² spoof at 0.3 s lag the minimum gap is 8.18 m with the
original gate, 7.49 m lag-matched and −30.64 m ungated. `trust_braking` passes
a −5 m/s² phantom-brake spoof unbounded (peak spacing error 53 m, same as
ungated), so it is not recommended.

**Radar noise vs the D-016 estimator** (`scenarios/qos_adaptive.yaml`,
true ρ = 10 → 3 → 10): median ρ̂ / mean headway in the first good window is
10.4 / 0.96 s with an ideal radar, 3.5 / 1.86 s with 0.02 m/s² radar noise and
1.5 / 2.46 s with 0.05 m/s². The probe excitation is ±0.08 m/s², so a few cm/s²
of radar error swamps the ratio the estimator relies on. With
`rho_source: reported` and the same 0.05 m/s² radar: ρ = 10.0, headway 0.96 s.

**Twin structure.** On data from the lag model the lag twin is kept with
high confidence (ΔBIC 5–9 in favour of the simpler model). On data with a
real 0.4 s dead time the dead-time structure wins by ΔBIC 239–335 and T_min
is recovered to 0.01 s. Only the *sum* of lag and dead time is identified
with this excitation; the split varies with the noise seed, and the twin
says so when one part sits on its bound. Noisy data (R² of gap < 0.8) drops
confidence to low.

## Options considered

| option | verdict | reason |
|---|---|---|
| Change the defaults to the realistic models | rejected for now | would silently change every published number; a study opts in explicitly |
| Radar lag as a first-order filter state in the ODE | rejected | a delay line reuses the tested V2V history code and keeps RK4 stages consistent |
| Noise-aware gating of estimator samples | rejected | with probe-level excitation it starves the estimator, and a starved estimator holds a stale ρ̂ into a zone (unsafe) |
| Fit initial states as free parameters per segment | deferred | the burn-in removes the bias at no extra cost |

## Consequences and limits

- The "~25 % capacity recovered" result of D-016 holds for a sender-reported
  ρ, not for the receiver-side estimator with a realistic radar.
- D-022's "inert under honest operation" holds only when the radar lag is
  small next to the V2V delay; use `match_lag` otherwise.
- No published study has been re-run with jitter, bursts or dead time yet,
  and nothing here is fitted to field radio or radar data.
- The twin's confidence level is a heuristic over R², residual
  autocorrelation and structure agreement; it is not a statistical test.
