# RB-03 — ROS 2 workflows (build, run, view, analyze)

## The invocation pattern (memorize this; everything else follows)

wsl.exe mangles quoted `bash -c` strings when the repo path contains a
space. **Never** inline commands. Instead:

1. Write an LF-ended driver script to a **space-free** path (a temp dir).
2. `wsl -d Ubuntu -- bash /mnt/c/<space-free-path>/driver.sh`

Driver template:

    #!/usr/bin/env bash
    set -eo pipefail            # NEVER -u (ROS setup.bash breaks)
    R="/mnt/c/Users/<user>/.../06_Cooperative-Adaptive-Cruise-Control"
    tr -d "\r" < "$R/scripts/ros2_sync_build.sh" > /tmp/sb.sh
    bash /tmp/sb.sh 2>&1 | tail -2          # BUILD_OK expected
    tr -d "\r" < "$R/scripts/ros2_run_demo.sh" > /tmp/rd.sh
    bash /tmp/rd.sh 6 200 cthp ma2025_case_a.yaml 2>&1 | tail -3

## Build

`scripts/ros2_sync_build.sh`: rsync repo → `~/cacc_ws` (ROS sources +
`src/cacc` → `pysrc/cacc`), strips CR from .py, colcon builds both
packages on ext4 (~4 s incremental). Message changes (`.msg`) require
this full path — never edit inside `~/cacc_ws` (it's a build artifact).

## Timed headless run

`ros2_run_demo.sh [N] [DURATION_S] [CONTROLLER] [CASE_YAML]` — launches
under `timeout --signal=SIGINT` (recorder closes the CSV cleanly), copies
to `results/ros2/<stamp>_<controller>_n<N>.csv`. The `[ERROR] process has
died, exit code -2` lines at the end are the SIGINT teardown — expected.

## Live 3D view

`ros2_view_demo.sh [N] [CONTROLLER] [CASE_YAML]` — platoon with
`viz:=true` + rviz2 (`config/platoon.rviz`, fixed frame `platoon` so the
camera rides along). Close the rviz window to stop everything. Needs
WSLg (present on this machine).

## Analysis (back on Windows)

    conda run -n cacc python scripts/plot_ros2_run.py results/ros2/<f>.csv
    conda run -n cacc python scripts/render_platoon_video.py results/ros2/<f>.csv

Expected cross-validation quality (noiseless case A): per-hop ratios
within ~2 % of the offline 0.9973; see `07_…_Results.md` §5 for the
reference numbers and `docs/validation/V-03` for the protocol.

## Reading launch logs

Log lives at the path printed by the driver (or `/tmp/*launch*.log` if
you redirect). Grep survival guide: `grep -i "error\|traceback" | grep -v
"exit code -2"` — anything left is real. Historical teardown tracebacks
in `destroy_node()` were fixed by guarding; if they reappear, the guard
was lost in a merge.

## Timing sanity for any new node

Wall-clock dt (measured, clamped 5 s, substepped) and stamp-age
extrapolation with the SAME clip are not optional patterns — they are
what makes runs on WSL trustworthy (D-013/D-014). A new node that keeps
its own tick-counted time WILL produce metre-scale artifacts at the
~32.5 s WSL freezes.
