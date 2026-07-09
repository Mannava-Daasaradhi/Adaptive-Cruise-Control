# D-021 — Predictive QoS-map spacing (anticipatory headway)

**Date:** 2026-07-09 · **Status:** accepted, **validated** · **Type:** novelty
/ control design

## Context
The QoS-adaptive loop of [D-016] is *reactive*: a follower lowers its
``rho_hat`` only after it has collected enough noisy samples inside an
interference patch, and the rate-limited headway then opens *behind* the
disturbance. The most dangerous instant — crossing into a degraded zone — is
exactly when the platoon is still at the good-channel headway, which for a deep
patch is **string-unstable** (‖H̃‖∞ > 1). The reactive design pays an entry
transient, in the bad channel, every time.

Real connected vehicles do not discover interference for the first time on
arrival: channel quality is strongly geo-correlated (foliage, cuttings, urban
canyons, known dead zones) and is routinely pooled into **shared maps**. This
is the first of the flagship claims (A of the A+B+certificate program).

## Decision — a shared spatial map + a preview lookahead
A :class:`cacc.qos_map.QoSMap` models channel quality as a function of
*position*, ``rho(x)`` (interference patches over metres, worst-overlap wins).
Every vehicle carries the same map. It is used two ways:

1. **Ground truth for the channel** — each link's noise level is driven by
   *where that link physically is* (``_link_rho_schedules`` builds a per-link
   ``V2VLink`` rho step-schedule from the follower's maneuver-integrated
   nominal trajectory), so follower *i* enters a patch later than the leader by
   its standoff, exactly as on a real road. This *replaces* the single shared
   time schedule of [D-016] with a spatially faithful one.

2. **A preview** — ``min_rho_ahead(x, v, preview_s)`` returns the worst channel
   the follower will meet within the horizon at its current speed. The headway
   target (``HeadwayAdapter.update(..., rho_preview=)``) opens for the **worse
   of the previewed and the measured** channel, so the gap is already open on
   entry. Because the map is known exactly (no ``rho_safety`` divide) and from
   ``t = 0`` (before any estimator warms up), the preview drives the headway
   even when ``rho_hat`` is still ``None``. Fusing preview *with* the live
   estimate keeps the design robust to a stale/incomplete map — the reactive
   estimate remains the safety backstop.

The string-stability argument is unchanged: h is still slew-rate limited, so
the frozen-design guarantee (string-stable for every h ≥ h_lb + margin,
[D-016]) carries over quasi-statically. Prediction only changes *when* h moves
— earlier, in the good channel — not the guarantee. [D-007] therefore holds and
the certificate ([D-018], extended in the flagship's part C) still applies.

## Evidence (`scripts/predictive_qos_study.py`, `scenarios/predictive_qos.yaml`)
6 followers through a geo-referenced ρ 10 → 3 patch, reactive vs predictive on
the identical channel (45 s / ~900 m preview):

| metric (last follower) | reactive (D-016) | predictive (D-021) |
|---|---|---|
| headway at zone entry | 0.95 s | **1.23 s** (pre-opened) |
| ‖H̃‖∞ in force at entry | **1.002 (> 1)** | **1.000** |
| fraction of in-zone time string-**unstable** | **6.9 %** | **0 %** |
| peak spacing error in zone | 17.2 m | **14.2 m** (−17 %) |
| mean headway (capacity price) | 1.395 s | 1.441 s (+3 %) |

The gap-opening transient still exists — a large headway change is a large
physical gap change — but the predictive design **relocates it into the good
channel before entry** (a benign ~5 m bump where the margin is ample) instead
of taking it inside the ρ = 3 patch. The honest cost is ~3 % more mean headway
(slightly earlier/longer opening), i.e. a small throughput price for holding
the certified margin through the whole degraded region.

## Alternatives considered
- **Longer estimator window / faster estimator** — cannot beat causality: the
  channel is unobservable until the platoon is *in* the patch.
- **Pure map, no live estimate** — brittle to map staleness; fusing with the
  reactive estimate (open for the worse) keeps the safety backstop.
- **Feed-forward the whole rho(x) into a time-varying H∞ synthesis** — a
  stronger result but abandons the frozen-design/quasi-static argument that
  makes the guarantee auditable; deferred.

## What was added
- `src/cacc/qos_map.py` — `QoSMap` (``rho_at``, ``min_rho_ahead``,
  ``time_schedule``).
- `src/cacc/estimation.py` — `AdaptConfig.preview_s`; `HeadwayAdapter.update`
  gains a fused ``rho_preview`` target.
- `src/cacc/platoon.py` — `PlatoonConfig.qos_map`; per-link position-driven
  channel; per-follower preview in the adaptation step; loader support.
- `scripts/predictive_qos_study.py`, `scenarios/predictive_qos.yaml`,
  `tests/test_predictive_qos.py` (7 tests; suite 65).

## See also
[D-004] · [D-007] · [D-016] · [D-017] · [D-018] · flagship parts B (V2V
physics-consistency gate) and C (chance-constrained certificate)
