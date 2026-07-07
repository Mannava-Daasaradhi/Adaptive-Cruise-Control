# R-11 — Scenario YAML schema (`scenarios/*.yaml`)

Loaded by `cacc.platoon.load_scenario`; sections map 1:1 onto
`PlatoonConfig` fields. Everything optional falls back to dataclass
defaults — but see the τ warning below.

```yaml
name: qos_adaptive              # results/<name>/<stamp>/
description: >                  # free text, lands in figures/logs

platoon:
  n_followers: 8
  v0: 20.0                      # [m/s]; paper doesn't state one (D-006)

vehicle:
  tau: 0.5                      # REQUIRED for Ma-2025 work (default 0.1!)
  length: 4.0
  u_min: -8.0
  u_max: 3.0

controller:
  kp: 0.009                     # Ma-family scale (Ploeg family ~0.2)
  kd: 0.7                       # Ploeg family only
  kv: 0.63                      # CTHP
  ka: 0.5                       # CTHP feedforward gain
  h: 0.95                       # initial/fixed headway [s]
  r: 1.0                        # r + length = paper's d = 5 m (D-005)

network:
  delay: 0.0                    # theta [s]; 0 or >= dt
  loss_prob: 0.0                # needs msg_rate
  msg_rate: null                # null = continuous link
  noise_rho: 10.0               # > 1; null/0-in-ROS = noiseless
  noise_rate: 100.0             # w hold rate [Hz] — affects L2 envelopes!
  rho_schedule: [[0.0, 10.0], [80.0, 3.0], [175.0, 10.0]]  # step (t, rho)

adaptation:                     # -> AdaptConfig; omit = disabled
  enabled: true
  est_rate: 25.0
  window: 256
  max_age_s: 20.0
  a_min: 0.03
  rho_safety: 1.15
  margin: 0.08
  rate: 0.05
  h_max: 2.5
  predictor: false
  adapt_gains: false            # D-017
  gain_rate: 0.05
  kv_frac: 0.9

leader:
  profile: bursts               # sine | brake | constant | bursts
  bursts:                       # [t_start, duration, amplitude, freq_hz]
    - [15.0, 62.83185, 0.4, 0.015915494]
    - [100.0, 62.5, 0.4, 0.064]
  probe_amplitude: 0.08         # estimator excitation dither (T-05)

sim:
  dt: 0.01
  t_final: 240.0
  seed: 1
```

## Canonical scenarios

| file | purpose |
|---|---|
| `ma2025_sine.yaml` | base-paper case A (12 followers, ρ=5, eq.-46 sine) |
| `qos_adaptive.yaml` | the D-016 interference-zone experiment |

## Validation behavior

Unknown keys raise (dataclass constructors reject them) — typos fail
loudly. Semantic guards live in `V2VLink`/`PlatoonSim` (ρ > 1, delay ≥ dt,
adaptation ⇒ cthp + continuous). The ROS `config/*.yaml` files mirror
these values in `/**: ros__parameters:` form — keep the two in sync by
hand and say so in commit messages (a sync checker script is on the
roadmap).
