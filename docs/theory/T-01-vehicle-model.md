# T-01 — Longitudinal vehicle model and actuator lag

**Code:** `src/cacc/vehicle.py` · **Used by:** every simulation and every
transfer function in the project.

## Model

Each vehicle i is a double integrator with a first-order actuator/driveline
lag (the standard longitudinal model of both Ploeg 2014 and Ma 2025):

    ṗ_i = v_i
    v̇_i = a_i
    ȧ_i = (u_i − a_i) / τ           u_i ∈ [u_min, u_max]

State vector per vehicle: x = [p, v, a] (`N_STATES = 3`); `vehicle_deriv`
returns ẋ for a commanded u after saturation by `VehicleParams.clamp`.

## Parameters and their provenance

| param | value | source |
|---|---|---|
| τ (lag) | **0.5 s** for all Ma-2025 work (their τ0; homogeneous τ = τ0) | D-006 |
| τ default in code | 0.1 s (Ploeg-era demo value) — **always set τ=0.5 in Ma scenarios**; forgetting this was a real bug during development (adapter targeted a τ=0.1 plant) | D-016 dev notes |
| L (length) | 4.0 m | D-005 (d = r + L = 5 m map) |
| u_min / u_max | −8 / +3 m/s² | comfortable braking/accel envelope |

## Why the lag matters so much

The pole at 1/τ = 2 s⁻¹ is the *fastest* dynamics in the loop and sets:
- the classic noiseless minimum headway 2τ0/(1+ka) (Ma Remark 3.3) — the
  headway exists to "hide" the actuator lag;
- the **RK4 single-step stability limit** h_step < 2.78·τ ≈ 1.4 s, which
  is why the ROS wall-clock integration substeps long stalls (D-014);
- the transfer function denominator τs³ + s² + γs + kp (T-03).

## Transfer function (command → position)

    G(s) = 1 / (s² (τ s + 1))

used verbatim in `analysis.gamma` for the Ploeg family; the CTHP error
propagation (T-03) absorbs G into the closed form.

## Saturation

Saturation is applied to u before integration (`veh.clamp`) in both
backends. All string-stability theory is linear — saturation is *outside*
the certified envelope, which is why the certification suites (D-018)
check min-gap on the nonlinear simulation as a separate criterion rather
than trusting ‖H̃‖∞ alone. The paper's maneuvers (|a0| ≤ 0.5 m/s²) stay
far from the limits; emergency-braking studies would exercise them and
are on the roadmap (P-03).

## Homogeneity assumption

All vehicles share (τ, L). Heterogeneous τ_i is supported mechanically
(per-vehicle `VehicleParams` would need plumbing in `PlatoonSim`) but the
per-hop transfer function then differs per vehicle; the one-vehicle
look-ahead L2 argument still applies hop-wise (each hop's ‖H̃_i‖∞ ≤ 1
suffices) — the same argument that already justifies per-follower
heterogeneous h(t) in the adaptive pipeline (T-08).
