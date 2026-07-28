# Flagship Part A — Predictive QoS-map Spacing (in depth)

**Claim.** A shared spatial channel-quality map plus a preview lookahead opens
the platoon's headway *before* it enters an interference patch, so the certified
string-stability margin holds through the whole degraded region — where the
reactive design is transiently unstable on entry.

Companion ADR: [D-021](../decisions/D-021-predictive-qos-map.md). Overview:
[00_Flagship_Overview](00_Flagship_Overview.md).

---

## 1. The problem: the reactive loop opens the gap too late
The QoS-adaptive layer ([D-016]) closes a loop between channel sensing and the
spacing policy: each follower estimates the multiplicative-noise level `rho` of
its own link from receiver-side samples and slews its time headway `h(t)` toward
the value the string-stability bound requires at that `rho`. It works, but it is
**reactive** — `rho_hat` only drops after the follower has collected enough
noisy samples *inside* the interference patch, so `h(t)` opens *behind* the
disturbance.

The most dangerous instant is exactly the crossing into a degraded zone: the
platoon is still at the good-channel headway `h = 0.95 s`, which for a deep
patch (`rho = 3`) gives `||H~||_inf = 1.002 > 1` — **string-unstable**. The
reactive design pays an entry transient, *in the bad channel*, every time.

Real connected vehicles do not discover interference for the first time on
arrival: channel quality is strongly geo-correlated (foliage, cuttings, urban
canyons, known dead zones) and is routinely pooled into shared maps. Part A
uses that.

## 2. Mechanism

### 2.1 A spatial channel map
`cacc.qos_map.QoSMap(zones, rho_base)` models channel quality as a function of
**position**, `rho(x)`, from a list of interference patches
`[(x0, x1, rho), …]` over metres (worst overlap wins):

- `rho_at(x)` — the channel quality at position `x` (left-closed, right-open
  patches; `rho_base` outside all patches).
- `min_rho_ahead(x, v, horizon)` — the **worst** channel the vehicle will meet
  within `v * horizon` metres ahead at its current speed `v` (the preview).
- `time_schedule(t_grid, x_of_t)` — converts `rho(x)` along a trajectory into a
  `V2VLink` time schedule `[(t_k, rho_k), …]` (how the channel is *driven*).

### 2.2 Two uses of the map
**(i) Ground truth for the channel.** Each link's noise level is driven by
*where that link physically is*. `PlatoonSim._link_rho_schedules` integrates the
nominal (maneuver-driven) leader trajectory and offsets follower *i* by its
standoff `(i+1)·gap0`, so follower *i* enters a patch **later** down the string,
exactly as on a real road. This replaces D-016's single shared time schedule
with a spatially faithful, per-link one.

**(ii) A preview.** In the adaptation step each follower calls
`min_rho_ahead(x_i, v_i, preview_s)` and passes it to the headway adapter as
`rho_preview`. The adapter opens for the **worse of the previewed and the
measured** channel:

    h_target = max( h_required(rho_safe_est) , h_required(rho_preview) ) .

Because the map is known exactly (no `rho_safety` divide on the map value) and
from `t = 0` (before any estimator has warmed up), the preview drives the
headway **even when `rho_hat` is still `None`**. Fusing preview *with* the live
estimate keeps the design robust to a stale or incomplete map — the reactive
estimate remains the safety backstop.

## 3. Why the guarantee carries over
The string-stability argument is unchanged from D-016: `h` is still
slew-rate-limited, so the frozen-design guarantee (string-stable for every
`h >= h_lb + margin`, [D-007]) carries over **quasi-statically**. Prediction
only changes *when* `h` moves — earlier, in the good channel — not the guarantee
itself. Each hop's transfer function depends only on its own follower's `h`, so
the per-vehicle heterogeneous headways the preview produces are admissible.

## 4. Implementation
- `src/cacc/qos_map.py` — `QoSMap` (`rho_at`, `min_rho_ahead`, `time_schedule`).
- `src/cacc/estimation.py` — `AdaptConfig.preview_s`; `HeadwayAdapter.update`
  gains a fused `rho_preview` target and acts on the preview even when
  `rho_hat is None`.
- `src/cacc/platoon.py` — `PlatoonConfig.qos_map`; per-link position-driven
  channel via `_link_rho_schedules`; per-follower preview in the run loop
  (`min_rho_ahead(x[b], x[b+1], preview_s)`); scenario-loader support.

## 5. Experimental setup (`scenarios/predictive_qos.yaml`)
6 followers, case-A gains, `v0 = 20 m/s`. A geo-referenced interference patch
`rho: 10 -> 3` between `x = 1600 m` and `4200 m`; the leader reaches 1600 m near
`t = 80 s`, followers progressively later. One gentle in-zone maneuver
(`[95 s, 60 s, 0.30, 0.06 Hz]`) probes in-zone string stability, plus a
persistent `0.08 m/s²` dither to keep the estimators fed. Preview horizon
`preview_s = 45 s` (~900 m). The study (`scripts/predictive_qos_study.py`) runs
**reactive** (`preview_s = 0`) and **predictive** on the *identical* channel.

## 6. Results

| metric (last follower) | reactive (D-016) | predictive (D-021) |
|---|---|---|
| headway at zone entry | 0.95 s | **1.23 s** (pre-opened) |
| `||H~||_inf` in force at entry | **1.002 (> 1)** | **1.000** |
| fraction of in-zone time string-**unstable** | **6.9 %** | **0 %** |
| peak spacing error in zone | 17.2 m | **14.2 m (−17 %)** |
| mean headway (capacity price) | 1.395 s | 1.441 s (+3 %) |

**Figure** (`fig1_predictive_vs_reactive.png`, three stacked panels for the last
follower): (1) the live `||H~||_inf` in force — reactive breaches 1 on entry,
predictive holds `<= 1` throughout; (2) the time headway `h(t)` — predictive
opens *before* the position-marked zone entry (dotted line); (3) the spacing
error — smaller in-zone peak for predictive.

## 7. Honest limitations
- The gap-opening transient still exists — a large headway change is a large
  physical gap change — but it is **relocated into the good channel before
  entry** (a benign ~5 m bump where the margin is ample) instead of being taken
  inside the patch.
- The honest cost is **~3 % more mean headway** (slightly earlier/longer
  opening): a small throughput price for holding the certified margin through
  the whole degraded region.
- Prediction cannot beat causality if the map is *wrong*: an unmapped patch is
  still discovered reactively. Fusing preview with the live estimate (open for
  the worse) is what keeps the safety backstop.

## 8. Relation to the rest
Extends the reactive QoS loop of [D-016]/[D-017]/[D-019]. The headway target it
computes is still the heuristic `margin + h_required` of D-016 — **Part C**
([F-C](F-C_Chance_Constrained_Certificate.md)) supplies the principled,
risk-calibrated replacement for that margin. Part A leaves the base string-
stability certificate ([D-007]/[D-018]) intact.

## 9. Reproduce
```bash
python scripts/predictive_qos_study.py     # -> results/predictive_qos/<stamp>/
pytest tests/test_predictive_qos.py -q      # 7 tests
```

[D-007]: ../decisions/D-007-reproduction-scope-and-validation.md
[D-016]: ../decisions/D-016-qos-adaptive-cacc.md
