#!/usr/bin/env bash
# Sync the repo's ROS 2 sources + shared cacc modules into ~/cacc_ws (native
# ext4 — colcon on /mnt/c is painfully slow) and build with colcon.
# Run inside WSL Ubuntu:  bash scripts/ros2_sync_build.sh
# (no `set -u`: ROS setup.bash references unset variables)
set -eo pipefail

REPO="${REPO:-/mnt/c/Users/daasa/OneDrive/Desktop/sem 5/06-Control-System/PROJECT/06_Cooperative-Adaptive-Cruise-Control}"
WS="$HOME/cacc_ws"

source /opt/ros/jazzy/setup.bash

mkdir -p "$WS/src" "$WS/pysrc"
rsync -a --delete "$REPO/ros2_ws/src/" "$WS/src/"
rsync -a --delete "$REPO/src/cacc/" "$WS/pysrc/cacc/"
# bash scripts came from Windows: strip CRs defensively
find "$WS/src" -name "*.py" -exec sed -i 's/\r$//' {} +

cd "$WS"
colcon build --symlink-install
echo "BUILD_OK — workspace at $WS"
