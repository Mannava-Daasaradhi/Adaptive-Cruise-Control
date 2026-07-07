# R-05 — `estimation.py` reference: the QoS-adaptive components

The heart of D-016/D-017. Three classes + one config dataclass.

## `AdaptConfig` (frozen; scenario `adaptation:` block)

| field | default | meaning |
|---|---|---|
| enabled | False | master switch for the outer loop |
| est_rate | 25.0 Hz | estimator/adapter update rate |
| window | 256 | max ŵ-samples kept |
| max_age_s | 20.0 s | samples older than this expire (T-05 recovery) |
| a_min | 0.03 m/s² | min radar excitation to accept a sample |
| rho_safety | 1.15–1.2 | κ: ρ̂ is divided by this (conservative side) |
| rho_min / rho_max | 1.5 / 50 | ρ̂ clip range |
| margin | 0.08 s | headway margin above the requirement |
| rate | 0.05 s/s | headway slew limit |
| h_max | 2.5 s | adaptation ceiling |
| predictor | False | timestamp feedforward lead (T-06) |
| pred_base | 0.4 s | predictor averaging baseline |
| adapt_gains | False | D-017 gain re-tuning |
| gain_rate | 0.05 /s | relative gain slew limit |
| kv_frac | 0.9 | kv target as fraction of the eq.-28a ceiling |

## `ChannelEstimator(cfg)`

- `add_sample(t, y_received, a_radar_delayed)` — guards: |a_ref| ≥ a_min,
  ŵ ∈ (0.2, 1.8). Stores (t, |ŵ−1|).
- `rho_hat(t=None) -> float | None` — expires by age, needs ≥ 25 fresh
  samples, inverts the 0.99-quantile of |ŵ−1|, clips to
  [rho_min, rho_max]. Returns the last estimate (or None) when starved.
  **Bias: over-estimates ρ by construction** (bounded support, T-05) —
  never remove κ thinking the estimator is unbiased.

## `HeadwayAdapter(ka, kp, kv, tau0, cfg, h0, theta=0, table=None)`

- Builds (or shares via `table=`) the fixed-gain requirement table over
  `RHO_GRID × THETA_GRID` (bisection, both noise ends, predictor-aware;
  infeasible cells capped at 10.0).
- `h_required(rho)` — bilinear interpolation at (ρ, θ).
- `update(dt, rho_hat, h_required=None)` — slew toward
  min(h_max, margin + h_req(ρ̂/κ)); holds when ρ̂ is None; the
  `h_required` override is how the gain scheduler injects its
  re-tuned requirement.
- `rho_safe_of` — exposes the κ mapping so callers stay consistent.

## `GainScheduler(ka, kp0, kv0, tau0, cfg, theta=0, table=None)` (D-017)

- Table rows per ρ: (kv_t, kp_t, h_req) with kv_t = kv_frac × ceiling(ρ),
  kp_t keeping kp/kv ratio, h_req bisected at those gains.
- `targets(rho)` → interpolated (kv_t, kp_t, h_req).
- `update(dt, rho_safe)` → rate-limited live (kp, kv) + h_req; the
  platoon then swaps the (stateless) CTHP instance with the new params.

## Wiring diagram (who calls what, per estimator tick)

    PlatoonSim.run  ─► est.add_sample(t, link.receive(t), acc[k−lag, pred])
                    ─► rho = est.rho_hat(t)
       adapt_gains? ─► kp,kv,h_req = scheduler.update(dt, rho_safe(rho))
                       ctrls[i] = make_controller('cthp', replace(...))
                    ─► h_i[i] = adapter.update(dt, rho, h_required=h_req)

Recorded per step into `SimResult.h`, `.rho_hat`, `.gains`.

## Table-sharing invariant

Adapters/schedulers of one platoon share a single table (built once,
passed via `table=`) — 154 bisections per table; building per-follower
was the first performance bug of the feature. Preserve this in the ROS
port (build at node start, log the build).
