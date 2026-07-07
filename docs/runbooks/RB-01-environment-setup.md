# RB-01 — Environment setup (from a clean machine)

Target: Windows 11 host for the scientific core, WSL2 Ubuntu 24.04 for
the ROS 2 backend. Everything below is already done on the development
machine — this runbook exists to rebuild it elsewhere.

## Windows side (scientific core)

1. Miniconda; create the env:

       conda create -n cacc python=3.13 numpy matplotlib pyyaml pytest pypdf
       conda activate cacc

2. ffmpeg for MP4 rendering (winget is fine): `winget install ffmpeg` —
   `render_platoon_video.py` falls back to GIF without it.
3. Sanity gate:

       cd <repo>
       conda run -n cacc python -m pytest tests -q          # 55 passed
       conda run -n cacc python scripts/reproduce_base_paper.py --quick

## WSL side (ROS 2 backend)

1. WSL2 Ubuntu 24.04 (Noble). WSLg gives rviz2 a display for free.
2. ROS 2 **Jazzy** via apt (classic keyring; D-012):

       sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
         -o /usr/share/keyrings/ros-archive-keyring.gpg
       echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
         http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list
       sudo apt update && sudo apt install -y ros-jazzy-desktop \
         python3-colcon-common-extensions python3-numpy python3-yaml

3. First build (from Windows PowerShell — see the invocation pattern in
   RB-03; in short: run `scripts/ros2_sync_build.sh` through an LF driver
   script in a space-free path):

       # expected tail: "BUILD_OK — workspace at /home/<user>/cacc_ws"

## Environment gotchas (all discovered the hard way — D-012)

| symptom | cause / fix |
|---|---|
| `AMENT_TRACE_SETUP_FILES: unbound variable` | ROS setup.bash is incompatible with `set -u`; scripts use `set -eo pipefail` only |
| `$'\r': command not found` | CRLF from Windows editing; every driver does `tr -d "\r"` before executing |
| variables mysteriously empty inside `wsl bash -c '…'` | wsl.exe re-joins the command line, losing inner quoting when the path contains a space (`sem 5`); ALWAYS use a driver *file* in a space-free path |
| `/tmp/…: No such file` after a pause | WSL idle-termination wipes /tmp; regenerate drivers from the repo every invocation |
| colcon glacially slow | building on /mnt/c (9p); the workspace lives on ext4 `~/cacc_ws`, sources rsync'd in |
| periodic ~1.4 s freezes of all nodes | WSL2/Windows housekeeping, every ~32.5 s on this machine; the node design absorbs it (D-014) — do not chase it as a bug |

## Verify the full stack

    # Windows:
    conda run -n cacc python scripts/certify.py --quick   # CERTIFIED expected
    # WSL (via driver): 60 s demo, then on Windows:
    conda run -n cacc python scripts/plot_ros2_run.py results/ros2/<stamp>_cthp_n6.csv
