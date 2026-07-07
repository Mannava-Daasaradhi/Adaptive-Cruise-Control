# Simulation Design (model → code map)

How the mathematics of the base paper (Ma, Pagilla & Darbha 2025) and the
classic foundation (Ploeg et al. 2014) map onto `src/cacc` and `ros2_ws/`.
Decisions behind this design: see `05_Decision_Log.md` (D-003 … D-014).

## 1. Vehicle model — `src/cacc/vehicle.py`
Per vehicle (double integrator + first-order actuator lag; papers agree):

    p' = v,   v' = a,   a' = (u - a) / tau        u clamped to [u_min, u_max]

Base-paper values: `tau = 0.5 s` (their τ0), `length = 4 m`.

## 2. Spacing policy and error signals — `src/cacc/controllers.py`
Constant time headway (CTH), front-bumper positions:

    e_i    = (p_{i-1} - p_i - L) - (r + h v_i)     spacing error
    e_i'   = v_{i-1} - v_i - h a_i                 (radar + IMU)
    dv_i   = v_{i-1} - v_i                         (radar)

Sign map to the paper: **e_i = -delta_i**, with their d = r + L = 5 m (D-005).

## 3. Controllers — `src/cacc/controllers.py`
| kind | law | V2V feedforward | states |
|------|-----|-----------------|--------|
| `acc`  | u = kp e + kd e' | — | 0 |
| `cacc` (Ploeg 2014 eq. 10) | h ξ' + ξ = kp e + kd e' + u_ff;  u = ξ | predecessor **commanded** u (`ff_signal="u"`) | 1 |
| `cthp` (Ma 2025 eq. 6) | u = kp e + kv dv + ka u_ff | predecessor **realized** a, noise-scaled (`ff_signal="a"`) | 0 |

Paper case-A gains: `kp=0.009, kv=0.63, ka=0.5, h=0.95, r=1.0`.

## 4. V2V channel — `src/cacc/network.py`
Per directed link i−1 → i (`V2VLink`):
* **delay** θ: continuous mode = interpolated transmitted-history lookup at
  t−θ (exact DDE treatment for RK4); sampled mode = beacons at `msg_rate` Hz.
* **packet loss**: Bernoulli per beacon (sampled mode), receiver zero-order-holds.
* **noise** (base paper eq. 5): received value × w(t),
  `w = (1-1/rho) + (1/rho) * sum_j z_j / 2^j`, z_j ~ Bernoulli(gamma_j),
  n = 16 bits, default gammas = `MA2025_GAMMAS` (the paper's). Piecewise
  constant at `noise_rate` (100 Hz), deterministic per seed (D-004).

## 5. String-stability analysis — `src/cacc/analysis.py`
Exact imaginary-axis evaluation (no Padé). Error-propagation transfer
functions, G(s) = 1/(s²(τs+1)), C(s) = kp + kd s, H(s) = h s + 1:

    ACC :  Gamma = G C / (1 + G C H)
    CACC:  Gamma = (G C + e^{-theta s}) / (H (1 + G C))
    CTHP:  H~(s) = (ka_eff s² e^{-theta s} + kv s + kp)
                   / (tau s³ + s² + (kv + h kp) s + kp)

`ka_eff` = ka·(channel factor): worst-case robust = (1±1/ρ)ka endpoints
(low end binds at low frequency — D-009), nominal = ka·E[w]
(`expected_w`). String stable ⇔ ‖Γ(jω)‖∞ ≤ 1 (`hinf_norm`,
`is_string_stable`, bisection `min_stable_headway`).

**Theorem III.2 in closed form:** `cthp_h_lb(ka, rho, tau0)` (eq. 17),
`cthp_optimal(rho, tau0)` (eqs. 18–19), `cthp_gains_feasible(...)`
(eqs. 16 + 29 + Routh–Hurwitz internal stability). Unit-tested against the
paper's printed numbers (`tests/test_cthp.py`).

## 6. Platoon simulator — `src/cacc/platoon.py`
Leader (prescribed a0(t)) + N identical followers, one-vehicle look-ahead,
fixed-step **RK4** (`dt = 0.01 s`), V2V via one `V2VLink` per link. The
follower loop reads its predecessor's state at the current RK4 stage (radar
is instantaneous); the feedforward comes from the link (delay/loss/noise).
Scenario YAML schema (see `scenarios/*.yaml`): `platoon / vehicle /
controller / network / leader / sim` sections; `scenarios/ma2025_sine.yaml`
is the base-paper case A.

## 7. Metrics — `src/cacc/metrics.py`
Peak and L2 spacing errors, vehicle-to-vehicle **L2 amplification ratios**
(empirical string stability: all ≤ 1), lane throughput of the spacing
policy, RMS acceleration (energy/comfort proxy).

## 8. ROS 2 backend (distributed twin) — `ros2_ws/`
One node per vehicle; nodes import `cacc.vehicle` / `cacc.controllers`
(single source of truth). Each follower runs the standard **sampled-data
loop** at 100 Hz: (1) advance own state by RK4 with the input computed at
the previous sample (ZOH actuation), (2) measure fresh — radar surrogate =
last predecessor state message **extrapolated by its stamp age** (real
radar sees the *current* gap), (3) compute the next input. Real-time
robustness (D-013/D-014): integrate over the *measured* wall dt (substepped
RK4, 5 s clamp = radar-age clip) so OS stalls advance the state instead of
freezing it, and hold each follower pinned at exact spacing for its first
2 s so spawn/discovery skew is never frozen into the slow error dynamics.
Full node graph, ops and validated agreement: `08_ROS2_Architecture.md`.

## 9. How to run (conda env `cacc`)
    python -m pytest tests -q                       # 41 tests
    python scripts/run_scenario.py scenarios/ma2025_sine.yaml
    python scripts/reproduce_base_paper.py          # full reproduction + extension
    python scripts/sweep_delay.py                   # Ploeg-family delay sweep

Outputs land in `results/<name>/<timestamp>/` (figures + `metrics.json`).
ROS 2 build/run commands: `08_ROS2_Architecture.md` §3; ROS run analysis:
`scripts/plot_ros2_run.py results/ros2/<stamp>_<ctrl>_n<N>.csv`.
Car visualization (rviz2 live / MP4): `08_ROS2_Architecture.md` §7.
