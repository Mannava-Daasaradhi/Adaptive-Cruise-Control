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

Docs are numbered serially (`01_Proposal` … `08_ROS2_Architecture`); the
cross-validation results the decisions refer to are in
`07_Base_Paper_Reproduction_Results.md` §5 and `08_ROS2_Architecture.md`.
