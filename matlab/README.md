# MATLAB / Simulink backend — Ma 2025 reproduction

A third simulation backend for this project, alongside the Python core
(`src/cacc`) and the distributed ROS 2 stack (`ros2_ws`). It exists to
reproduce **Ma, Pagilla & Darbha, "Selection of Time Headway in Connected and
Autonomous Vehicle Platoons Under Noisy V2V Communication", IEEE T-ITS
26(1):1029–1038, 2025** (doi 10.1109/TITS.2024.3498701) end to end — every
theorem, every table, every figure — inside Simulink.

Built and validated on **MATLAB R2026a** (Simulink 26.1). Only base MATLAB +
Simulink are required; no add-on toolboxes.

## Run it

```matlab
cd matlab
run_all                 % deterministic-equivalent channel (the paper's Remark 5)
run_all('stochastic')   % realised 16-bit Bernoulli channel, eq. (5)
```

~90 s. Writes twelve PNGs and a transcript to `matlab/results/`. The model is
built automatically on first run; delete `models/cacc_platoon.slx` to force a
rebuild.

To open the block diagram:

```matlab
addpath models; open_system('cacc_platoon')
```

Note the model resolves its parameters from the caller's workspace, so it will
not *simulate* from a bare `open_system` — run a case through
`ma2025.run_case` (or `run_all`) instead.

## Layout

| path | what |
|---|---|
| `models/build_cacc_model.m` | **source of truth** for the model — builds `cacc_platoon.slx` programmatically so the diagram is reviewable and diffable |
| `models/cacc_platoon.slx` | build artefact |
| `+ma2025/` | analysis package: Theorem 1–2 closed forms, transfer function, ‖·‖∞, feasible sets, parameter sets |
| `scripts/figs_design_space.m` | Figs. 4, 5, 11 |
| `scripts/figs_frequency.m` | Figs. 6, 9 |
| `scripts/figs_time_domain.m` | Figs. 7, 8, 10, 12, 13, 14, 15 |
| `scripts/theorem_checks.m` | numerical verification of every closed form |
| `run_all.m` | driver |
| `tests/crosscheck_python.py` | agreement check against the Python core |

## Model structure

```
cacc_platoon
├── Lead Vehicle      a0(t) = 0.5 sin(0.1(t-10)) gated to one period,
│                     integrated twice  ->  [x0; v0; a0]
├── V2V Channel       w = bias + K·z,  z_j = 1{U_j < gamma_j}
│                     K = (1/rho)·kron(I_N, [2^0 … 2^-15])       eq. (5)
└── Platoon           vector states x, v, a  (width N)
    ├── pred_* = PredSel·[lead; own],  PredSel = [I_N  0]   <- the one-vehicle
    │                                                          look-ahead topology
    ├── delta = x - x_pred + hw·v + d                        Def. 1 + eq. (2)
    ├── u     = ka·(w .* a_pred) - kv·(v - v_pred) - kp·delta  eq. (6)
    └── adot  = (u - a) ./ tau                                eq. (1)
```

Vector signals of width `N` rather than `N` copies of a subsystem: platoon size
becomes a parameter, heterogeneous parasitic lags (Case 5) become a vector, and
the entire inter-vehicle coupling collapses to one constant matrix. There is no
algebraic loop — `u_i` depends on `a_{i-1}`, which is a *state* of vehicle
*i−1*, never a direct feedthrough of `u_{i-1}`.

Solver: fixed-step `ode4` at 1 ms. Fixed-step because the channel is a
piecewise-constant discrete signal (10 ms hold); a variable-step solver would
take a zero-crossing hit at every hold boundary.

## Three channel modes

The paper uses two systems and it matters which figure comes from which:

| mode | `w` | used for |
|---|---|---|
| `stochastic` | 16 Bernoulli bits redrawn every 10 ms, eq. (5) | Fig. 8 — the realised noise scatter |
| `mean` | `E[w] = 1.0482`, the **equivalent deterministic system** eq. (13) / Remark 5 | Figs. 7, 10, 12, 13, 14, 15 |
| `none` | `w ≡ 1`, ideal channel | Remark 3/4 limits, cross-validation |

All three are the *same diagram*: the mode only sets the values of `noise_K`
and `noise_bias`. In `mean` and `none` mode `noise_K = 0`, so the random path
is present but contributes nothing.

Why `mean` for the time-domain figures: the per-hop attenuation being displayed
is ≈0.35 %, while a single stochastic realisation jitters `max|δ_i|` by ≈2 %.
The monotone profiles the paper prints are a property of the mean system, which
is precisely what Remark 5 says the robust analysis is performed on. Running
`run_all('stochastic')` shows the jitter — worth doing once to see the point.

## Reproduction results

See [`../docs/validation/V-05-simulink-reproduction.md`](../docs/validation/V-05-simulink-reproduction.md)
for the full table, the cross-validation against the Python core, and the one
open discrepancy (Case 3 / Fig. 12).
