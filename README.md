# Project 6 — Cooperative Adaptive Cruise Control (Vehicle Platooning)

> **Software only** (MATLAB/Simulink or Python). Intelligent-transport control — high real-world value.

## The one-line story
When cars follow each other, small speed changes by the leader get **amplified** down the line
("phantom traffic jams" and crashes) — this is **string instability**. Cooperative Adaptive
Cruise Control (CACC) lets vehicles **share their intent over V2V wireless** so the platoon stays
tight and stable. We design a CACC controller, prove/show **string stability**, and demonstrate
shorter, safer, more fuel-efficient platoons than human/ACC driving — even with communication delay.

## Why it's impressive & useful
- ✅ Self-driving + V2V is highly modern and fundable
- ✅ Real **IEEE Transactions on Intelligent Transportation Systems** base paper
- ✅ "String stability" is a rigorous, paper-worthy control concept
- ✅ Pure simulation; clean to demonstrate and quantify

## Deliverables
`01_Proposal.md` · `02_IEEE_Base_Paper.md` · `03_OATD_Thesis.md` · `04_Industry_Resume_Solutions.md` ·
`05_Decision_Log.md` · `06_Simulation_Design.md` · `07_Base_Paper_Reproduction_Results.md` ·
`08_ROS2_Architecture.md` · `09_QoS_Adaptive_CACC.md` · `10_Handoff_Plan.md`

**Code:** Python core (`src/cacc`, 52 unit tests) + ROS 2 Jazzy distributed sim (`ros2_ws/`) —
base-paper (Ma 2025, T-ITS) reproduction in `scripts/reproduce_base_paper.py`, outputs in `results/`.
**Novel extension (D-016):** QoS-aware CACC — online channel estimation, timestamp feedforward
prediction, string-stability-preserving headway adaptation (`scripts/qos_adaptive_study.py`).
**Car simulation:** live 3D in rviz2 (`scripts/ros2_view_demo.sh`) and MP4 highway animation
(`scripts/render_platoon_video.py`). **Decisions:** `05_Decision_Log.md` → `docs/decisions/D-###`.
**Continuing the project:** start from `10_Handoff_Plan.md`.

## Headline result
> *"A leader speed perturbation is attenuated (not amplified) down a 5–10 vehicle platoon under
> CACC, enabling smaller gaps (higher road capacity) and lower fuel use vs ACC — robust to V2V delay."*
