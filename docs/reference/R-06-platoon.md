# R-06 — `platoon.py` reference: the monolithic simulator

## `PlatoonConfig` (frozen)

`n_followers, v0, vehicle, control, delay, loss_prob, msg_rate,
noise_rho, noise_gammas, noise_rate, rho_schedule, adapt, dt=0.01,
t_final, seed`. Everything an experiment needs in one hashable object;
vary with `dataclasses.replace`.

## `PlatoonSim(config, controller='cacc'|'acc'|'cthp', leader_accel)`

State layout: [p0, v0_leader | per follower: p, v, a, xc…] — leader is
kinematic (realizes its commanded profile exactly). One `V2VLink` per hop
(seeded `seed + i`); link 0 carries the leader signal.

**`_deriv(t, x)`** (called 4× per RK4 step):
- per follower: e/ė/dv with the *per-follower* headway `h_i[i]` (T-02);
- feedforward: passthrough → live value; predictor on → 
  `link.receive_predicted(t, θ̂)`; else `link.receive(t)`;
- fills `_io_*` scratch buffers at stage-1 for recording.

**`run() -> SimResult`**: fixed-step RK4; per accepted step: record,
`send` on every link (CTHP transmits realized acceleration, Ploeg-CACC
the commanded input — `ff_signal`), then the **adaptive outer loop** at
`est_rate` (see R-05 wiring), then the RK4 step.

Guard rails: adaptation requires kind='cthp' and continuous links (raises
otherwise); delay must be 0 or ≥ dt.

## `SimResult`

Arrays (S = steps+1): `t, pos, vel, acc (S, n+1)` — column 0 = leader
(acc col 0 = commanded a0); `err, u (S, n)`; adaptive extras: `h (S, n)`,
`rho_hat (S, n; NaN = no estimate)`, `gains (S, n, 2; kp, kv)`.

## Scenario I/O

`load_scenario(path) -> Scenario(name, config, leader, description)`.
YAML sections → config fields: `platoon / vehicle / controller / network /
adaptation / sim` (see R-11 for the schema). `rho_schedule` lists are
tupled; `adaptation:` maps to `AdaptConfig`.

`make_leader_profile(spec)` — profiles: `brake` (t_start, duration,
decel), `sine` (t_start, amplitude, freq_hz, duration), `constant`
(value), `bursts` (list of [t_start, duration, amplitude, freq_hz]).
All support `probe_amplitude` — a ±amp/2 two-tone dither (0.31/0.73 Hz)
superposed for estimator excitation (T-05).

## Numerical notes

- dt = 0.01 s everywhere; the fastest pole (1/τ = 2) gives λ·dt = 0.02 —
  deep inside RK4's region; halving dt changes reported L2 by < 1e-4.
- The delay line interpolates the *transmitted history*, making the DDE
  explicit — standard and exact to interpolation order.
- Noise is stage-consistent by the interval cache (D-004); never replace
  it with per-call draws.

## Performance envelope

8 followers × 24 000 steps ≈ 4 s wall; the adaptive outer loop adds ~10 %;
table builds dominate the first adaptive run (seconds, shared). Seed
sweeps parallelize trivially at the process level if ever needed.

## Extension recipe (per-follower vehicle heterogeneity)

Mirror the `h_i` pattern: hold `self.veh_i: list[VehicleParams]`, index in
`_deriv`, thread a `vehicles:` list through the scenario loader. The
metrics and viz need no change (they read arrays).
