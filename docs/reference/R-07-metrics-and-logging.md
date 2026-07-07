# R-07 — `metrics.py` and `logging_config.py` reference

## `cacc.metrics` (operates on `SimResult`)

| function | returns | notes |
|---|---|---|
| `peak_abs_errors(res)` | (n,) max |e_i| per follower | transition waves inflate this in adaptive runs — interpret with T-08 |
| `l2_errors(res)` | (n,) ‖e_i‖₂ (trapezoid in t) | the building block of the empirical stability readout |
| `amplification_ratios(res, kind='l2')` | (n−1,) ‖e_i‖/‖e_{i−1}‖ | *empirical*; the decisive verdict is ‖H̃‖∞ (D-010) |
| `empirically_string_stable(res, tol=0.02)` | bool | all ratios ≤ 1+tol; noise-injected runs scatter around 1 — see the three-way verdict discussion in `08_…` §4 |
| `throughput_veh_per_hour(v, control, length)` | veh/h | 3600·v/(L + r + h·v); the capacity claims' source |
| `accel_rms(res)` | (n+1,) RMS acceleration | comfort/energy proxy |
| `summary(res)` | dict | one-stop aggregation for reports |

Windowed variants (per-phase L2, trimmed tails) live in the analysis
scripts (`qos_adaptive_study.l2_per_phase`, `plot_ros2_run`) because the
windows are experiment-specific; promote them here only if a third
consumer appears.

## Interpretation rules (bind these to any new metric)

1. Never present a per-hop L2 ratio from a *noise-injected* run as a
   stability verdict — realization scatter is ±(2–5) % depending on the
   noise hold time (T-05). Lead with the frequency-domain number.
2. Windows containing h-transitions measure *commanded* waves (T-08);
   either exclude them or label them.
3. Min-gap is a *nonlinear safety* check, complementary to all linear
   verdicts — it's a first-class criterion in certification (D-018).

## `cacc.logging_config`

`setup_logging(level=INFO)` — root config for the `cacc.*` logger tree;
scripts call it in `main()`. Library modules only `getLogger(__name__)` —
never configure handlers at import (standard library-logging etiquette,
and it keeps pytest output clean).

Conventions: sim start/done lines include the parameters that matter for
reproduction (controller, n, h, delay, loss, mode, dt, t_final); table
builds log once; per-step logging is forbidden (24 000 steps).
