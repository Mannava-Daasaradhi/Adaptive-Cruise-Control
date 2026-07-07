# R-03 — `network.py` reference: the V2V link

One directed link per hop (predecessor → follower). Constructor:

    V2VLink(delay=0.1, loss_prob=0.0, msg_rate=None, seed=0, initial=0.0,
            noise_rho=None, noise_gammas=None, noise_rate=100.0,
            rho_schedule=None)

## Transport modes

- **Continuous** (`msg_rate=None`): receiver sees x(t−θ) by linear
  interpolation of the transmitted history (exact DDE treatment for the
  fixed-step RK4; `send` appends, `receive` interpolates at t−θ).
  `receive` is a *pure function of t* in this mode — safe to call from
  RK4 stages, estimator sampling, and the predictor's history queries.
- **Sampled** (`msg_rate` in Hz): beacons at fixed rate; each lost i.i.d.
  with `loss_prob`; receiver zero-order-holds the latest arrival.
  `receive` **mutates** delivery state in this mode — call once per
  timestep, never from stages. (Adaptation therefore requires continuous
  mode; enforced in `PlatoonSim`.)

## Noise (D-004, T-05)

- `noise_at(t)`: piecewise-constant per 1/`noise_rate` interval;
  **ρ-independent bit cache** U′(t) with w = (1−1/ρ(t)) + U′(t)/ρ(t);
  separate RNG stream `[seed, 0xCACC]` so loss draws never shift.
- `rho_schedule=[(t_k, ρ_k), …]` step schedule; `rho_at(t)` resolves it;
  passing a schedule without `noise_rho` enables noise from the first
  entry. Values must be > 1.
- `MA2025_GAMMAS` — the paper's 16 bit-expectations; the module-level
  constant is also imported by the ROS channel node.

## Predictor (D-016, T-06)

`receive_predicted(t, theta_hat, base=0.4, n_avg=8)` — continuous mode
only; returns receive(t) + θ̂ × slope of two `base/2` block averages of
the receiver's own past values. Falls back to plain receive when
θ̂ ≤ 0, in sampled mode, or before t = base. Deterministic (reuses
`noise_at`), hence RK4-stage safe.

## `passthrough` property

True iff delay = 0, continuous, lossless, noiseless — the simulator then
wires the live predecessor value directly (skipping interpolation at the
current instant, which would be self-referential at stage times).

## Sharp edges (learned the hard way)

1. `receive` in continuous mode requires **non-decreasing history**;
   `send` must be called once per accepted step (the platoon does this
   after recording, before stepping).
2. `delay` must be 0 or ≥ dt (delay-line lookup needs one sample of
   history); enforced by `PlatoonSim`.
3. Querying `noise_at` far beyond the simulated horizon grows the bit
   cache — harmless but unbounded; don't probe t = 1e6.
4. The noise hold time (`noise_rate`) changes low-frequency noise power
   and therefore per-hop L2 envelopes — record it with any experiment
   (T-05, D-014 cross-check).

## Tests pinning this module

`test_cthp.py`: determinism per seed, range of w, scaling of received
values, boundary semantics of the first sample. `test_qos_adaptive.py`:
schedule support switching, bit-cache invariance across schedules,
predicted-receive ramp tracking.
