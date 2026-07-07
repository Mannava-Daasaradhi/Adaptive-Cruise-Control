# D-012 — ROS 2 Jazzy desktop installed via apt in WSL2 (2026-07-05)

**Date:** 2026-07-05 · **Status:** accepted · **Type:** environment

## Context
The ROS backend ([D-002]) needs a ROS 2 distribution on a Windows 11
machine. Native Windows ROS 2 is second-class (binary archives, no desktop
metapackage, poor community support). WSL2 Ubuntu 24.04 was already
installed.

## Options considered
1. **ROS 2 Jazzy Jalisco on WSL2 Ubuntu 24.04 via apt** — CHOSEN.
   Jazzy is the LTS matching 24.04 (Tier 1, supported until May 2029).
2. Humble on a 22.04 container — older LTS, would add a container layer.
3. Native Windows ROS 2 — brittle toolchain, no WSLg-style GUI story.
4. Rolling — not for a course deliverable.

## What was installed (as root, inside WSL Ubuntu 24.04)
- Classic keyring method: key to
  `/usr/share/keyrings/ros-archive-keyring.gpg`, repo
  `packages.ros.org/ros2/ubuntu noble main`.
- `ros-jazzy-desktop` (~2.6 GB into /opt/ros/jazzy — includes rviz2),
  `python3-colcon-common-extensions`, `python3-numpy`, `python3-yaml`
  via apt (no rosdep init needed for this project's dependency set).

## Gotchas encountered & fixed (institutional knowledge)
- **`set -u` kills ROS**: `/opt/ros/jazzy/setup.bash` references
  `AMENT_TRACE_SETUP_FILES` unbound → all build/run scripts use
  `set -eo pipefail` (never `-u`) with a comment explaining why.
- **CRLF**: scripts authored on Windows must be `tr -d "\r"`-ed before
  execution in WSL; the sync script also strips `.py` files it copies.
- **/tmp is volatile**: WSL idle-termination wipes `/tmp`; anything needed
  across sessions lives in `~/cacc_ws` or the repo.
- **Build location**: colcon on `/mnt/c` (9p) is ~10× slower; the
  workspace lives on ext4 at `~/cacc_ws`, sources rsync'd in by
  `scripts/ros2_sync_build.sh` (~4 s incremental build for 2 packages).
- **wsl.exe argument mangling**: quoted `bash -c '…'` strings passed from
  PowerShell lose their internal quoting when the command line is re-joined
  on the Linux side; with the repo path containing a space (`sem 5`) this
  broke variable assignments silently. Robust pattern: write an LF-ended
  driver script to a **space-free** path (the session scratchpad) and run
  `wsl -d Ubuntu -- bash /mnt/c/<no-spaces>/driver.sh`.
- **WSLg works** on this machine — rviz2 GUI is usable directly (relied on
  by D-015).

## Consequences
- Reproducible two-command workflow from Windows:
  sync+build script, then timed demo script
  (`scripts/ros2_run_demo.sh [N] [DURATION] [CONTROLLER] [CASE_YAML]`).
- WSL2's scheduling quirks (periodic ~1.4 s freezes) later became the
  subject of [D-014] — an inherent cost of this hosting choice, now
  handled in the node design itself.
