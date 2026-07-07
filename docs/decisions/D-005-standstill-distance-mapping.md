# D-005 — Standstill distance mapping: d = 5 m ⇒ r = 1 m + L = 4 m

**Date:** 2026-07-05 · **Status:** accepted · **Type:** modeling convention

## Context
The paper treats vehicles as points and defines the spacing error (their
eq. (4)) with a fixed standstill distance d = r + L = 5 m:

    δ_i = x_{i−1} − x_i − d − h·v_i        (their sign: δ > 0 ⇒ gap too big)

Our codebase keeps **front-bumper positions** and explicit vehicle length
because the throughput metric (vehicles/km at speed) needs physical length:

    e_i = (p_{i−1} − p_i − L) − (r + h·v_i)

## Decision
Map the paper's d = 5 m to `r = 1.0 m` (bumper-to-bumper standstill gap) +
`L = 4.0 m` (vehicle length). With that, our e_i and their δ_i measure the
same physical quantity with **opposite sign**: `e_i = −δ_i`.

## Why r = 1 and not r = 5, L = 0
- L = 0 would silently break the lane-throughput metric
  (`src/cacc/metrics.py`) and make the rviz/animation cars zero-length.
- Any (r, L) split with r + L = 5 reproduces the paper's dynamics exactly —
  the error dynamics only see the sum — so we chose physically sensible
  values (typical compact-car length 4 m, tight standstill gap 1 m).

## Consequences
- Everything that reproduces a paper figure plots **δ_i = −e_i** so the
  curves match the paper's orientation (`scripts/reproduce_base_paper.py`,
  `scripts/plot_ros2_run.py` middle panel).
- Scenario files set `r: 1.0` with the comment "r + length = d = 5 m of the
  paper" (`scenarios/ma2025_sine.yaml`,
  `ros2_ws/src/cacc_platoon/config/*.yaml`).
- The ROS follower's desired-gap computation uses
  `pre.position − L − (r + h·v)` for its anchor ([D-013]) — same map.
