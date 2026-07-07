# D-002 — Stack: Python scientific core + ROS 2 Jazzy distributed sim

**Date:** 2026-07-05 · **Status:** accepted · **Type:** architecture

## Context
The theory work (transfer functions, ‖·‖∞ evaluation, headway bisection,
batch parameter sweeps, unit tests) was already implemented in a pure-Python
package `src/cacc` (conda env `cacc`, Python 3.13, NumPy/matplotlib/pytest).
The user was asked how the simulation should be built and answered:
*"i want the simulation to be in ros2 too i want this to be as polished and
great and technical."* The machine is Windows 11; ROS 2 has no first-class
Windows support for Jazzy, so ROS means WSL2.

## Options considered
1. **Python core + ROS 2 on WSL2 (both)** — CHOSEN.
2. ROS 2 only — would make every batch study (16-γ sweeps, delay bisection,
   seed sweeps) painfully slow and untestable with pytest; frequency-domain
   analysis doesn't belong in nodes.
3. Python only — fails the user's explicit ROS 2 requirement.
4. MATLAB/Simulink — the course allows it, but nothing existed in MATLAB
   and the user asked for ROS 2.

## Decision
Two backends, one model:
- `src/cacc` stays the **scientific core**: vehicle model, controllers,
  V2V link (delay/loss/noise), monolithic RK4 platoon simulator, string
  stability analysis, metrics, 41 unit tests.
- `ros2_ws/src/` adds the **distributed systems demo**: one process per
  vehicle on ROS 2 Jazzy under WSL2 Ubuntu 24.04 ([D-011], [D-012]).
- **Single source of truth:** the ROS nodes `import cacc.vehicle` and
  `cacc.controllers` — the equations exist in exactly one place. The sync
  script copies `src/cacc` into the WSL workspace (`~/cacc_ws/pysrc`) and
  the nodes get it via PYTHONPATH.

## Build/run mechanics (consequences)
- colcon builds happen on WSL ext4 (`~/cacc_ws`) because building on
  `/mnt/c` (9p filesystem) is an order of magnitude slower;
  `scripts/ros2_sync_build.sh` rsyncs repo → `~/cacc_ws` and builds (~4 s
  incremental).
- Timed demos: `scripts/ros2_run_demo.sh` runs the launch under
  `timeout --signal=SIGINT`, then copies the recorder CSV back to
  `results/ros2/<stamp>_<controller>_n<N>.csv` for analysis on Windows.
- Windows→WSL gotchas discovered and worked around: CRLF must be stripped
  (`tr -d "\r"`) before executing repo scripts in WSL; `wsl.exe` +
  PowerShell mangle quoted `bash -c` strings when the path contains spaces
  (`sem 5`), so drivers are written to a space-free scratchpad path and
  invoked as `wsl -d Ubuntu -- bash /mnt/c/<no-spaces>/x.sh`; WSL
  idle-termination wipes `/tmp`, so nothing persistent lives there;
  ROS `setup.bash` breaks under `set -u` ([D-012]).

## Validation
Cross-validation between the two backends is a first-class deliverable:
`scripts/plot_ros2_run.py` computes the same per-hop L2 amplification
metric on ROS CSVs as the offline metrics module; agreement numbers live in
`07_Base_Paper_Reproduction_Results.md` §5 (closed by [D-013]/[D-014]).
