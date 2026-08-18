# Decision Log (index)

Every project decision is a numbered record in **`docs/decisions/`**
(ADR-style: context → options → decision → evidence → consequences).
This file is the index; new decisions get the next `D-###` and a line here.

| # | Decision | Type | Date |
|---|---|---|---|
| [D-001](docs/decisions/D-001-base-paper-ma2025.md) | Base paper switched to Ma, Pagilla & Darbha 2025 (T-ITS, noisy-V2V min headway) | scope | 2026-07-05 |
| [D-002](docs/decisions/D-002-stack-python-plus-ros2.md) | Stack: Python scientific core + ROS 2 Jazzy distributed sim (WSL2), single source of truth | architecture | 2026-07-05 |
| [D-003](docs/decisions/D-003-cthp-controller-integration.md) | CTHP as third controller behind the shared interface (`dv` input, `ff_signal` attribute) | code design | 2026-07-05 |
| [D-004](docs/decisions/D-004-noise-in-channel-not-controller.md) | Multiplicative 16-bit channel noise lives in the V2V link; RK4-safe deterministic draws | code design | 2026-07-05 |
| [D-005](docs/decisions/D-005-standstill-distance-mapping.md) | Their d = 5 m ⇒ our r = 1 m + L = 4 m; sign map e = −δ | convention | 2026-07-05 |
| [D-006](docs/decisions/D-006-paper-parameters-from-pdf.md) | Section-IV parameters extracted from the PDF via pypdf; v0 = 20 m/s is our choice | provenance | 2026-07-05 |
| [D-007](docs/decisions/D-007-reproduction-scope-and-validation.md) | Reproduction scope (Figs. 5–13) + every printed number pinned as a unit test | methodology | 2026-07-05 |
| [D-008](docs/decisions/D-008-extension-design-delay-and-loss.md) | Extension: delay budget + packet loss at fixed case-A gains — **delay is the binding constraint** | novelty | 2026-07-05 |
| [D-009](docs/decisions/D-009-low-noise-end-is-worst-case.md) | Worst case is the LOW end of the noise interval, k̃a = (1−1/ρ)ka | theory | 2026-07-05 |
| [D-010](docs/decisions/D-010-time-domain-contrast-is-subtle.md) | A/B time-domain contrast is ≲0.4 %/hop by design — verdicts live in the frequency domain | observation | 2026-07-05 |
| [D-011](docs/decisions/D-011-ros2-one-node-per-vehicle.md) | ROS 2: genuinely distributed, one process per vehicle + channel + recorder | architecture | 2026-07-05 |
| [D-012](docs/decisions/D-012-ros2-jazzy-install-wsl.md) | ROS 2 Jazzy via apt in WSL2 Ubuntu 24.04; the full WSL gotcha list | environment | 2026-07-05 |
| [D-013](docs/decisions/D-013-startup-anchor-and-radar-extrapolation.md) | Startup: radar extrapolation by stamp age + 2 s anchor hold (±2 m → ±2 mm) | real-time | 2026-07-06 |
| [D-014](docs/decisions/D-014-wall-clock-integration-substep.md) | Wall-clock-measured dt, substepped RK4, matched 5 s horizons — WSL's periodic 1.4 s freezes neutralized | real-time | 2026-07-06 |
| [D-015](docs/decisions/D-015-visualization-rviz2-and-mp4.md) | Car visualization: rviz2 live 3D (platoon-frame camera) + MP4/GIF renderer | demo | 2026-07-07 |
| [D-016](docs/decisions/D-016-qos-adaptive-cacc.md) | Novel extension: QoS-aware CACC — online ρ̂ estimation, timestamp feedforward predictor, fixed-gain headway adaptation; ρ* wall, predictor flatline, noise×delay interaction | research | 2026-07-07 |
| [D-017](docs/decisions/D-017-online-gain-retuning.md) | Online gain re-tuning (90 %-of-ceiling rule) breaks the fixed-gain wall — ρ=2 zone stabilized (in-force ‖H̃‖∞ 1.019 → 0.99999) | research | 2026-07-07 |
| [D-018](docs/decisions/D-018-certification-harness.md) | Monte-Carlo certification harness: scenario×seed matrix → self-contained HTML PASS/FAIL report (first full run CERTIFIED) | product | 2026-07-07 |
| [D-020](docs/decisions/D-020-gazebo-kinematic-renderer.md) | Gazebo 3-D view as a kinematic renderer driven by the certified core (no physics duplication) | demo | 2026-07-09 |
| [D-021](docs/decisions/D-021-predictive-qos-map.md) | Flagship A: predictive QoS map — spatial ρ(x) preview drives headway *before* the platoon enters the zone | research | 2026-07-14 |
| [D-022](docs/decisions/D-022-physics-consistency-v2v-gate.md) | Flagship B: physics-consistency V2V gate — radar-vs-beacon trust weight g ∈ [0,1] de-rates spoofed feedforward | research | 2026-07-14 |
| [D-023](docs/decisions/D-023-chance-constrained-certificate.md) | Flagship C: chance-constrained risk certificate — exact channel law → h_cc(ε); two-tailed instability finding | research | 2026-07-14 |
| [D-024](docs/decisions/D-024-simulink-backend.md) | MATLAB/Simulink third backend: programmatically-built vectorised model reproduces Figs. 4–15; agrees with the Python core to 1e-4 | architecture | 2026-08-18 |

Docs are numbered serially (`01_Proposal` … `10_Handoff_Plan`); the
cross-validation results the decisions refer to are in
`07_Base_Paper_Reproduction_Results.md` §5 and `08_ROS2_Architecture.md`;
the novel extension is `09_QoS_Adaptive_CACC.md`; the execution plan for
everything remaining is `10_Handoff_Plan.md`.
