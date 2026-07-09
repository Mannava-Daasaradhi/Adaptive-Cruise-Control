#!/usr/bin/env bash
# Live 3D platoon demo in Gazebo (gz-sim Harmonic) + optional rviz2. The cars
# are driven kinematically from the SAME CACC nodes as every other backend, so
# Gazebo only renders the validated dynamics. Close the Gazebo window to stop.
#
# Usage (inside WSL Ubuntu 24.04, after ros2_sync_build.sh):
#   bash scripts/ros2_gazebo_demo.sh [N_FOLLOWERS] [CONTROLLER] [CASE_YAML] [rviz]
# e.g.  bash scripts/ros2_gazebo_demo.sh 6 cthp ma2025_case_a.yaml rviz
# (no `set -u`: ROS setup.bash references unset variables)
set -eo pipefail

N="${1:-6}"
CONTROLLER="${2:-cthp}"
CASE="${3:-ma2025_case_a.yaml}"
RVIZ="${4:-}"

WS="$HOME/cacc_ws"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export PYTHONPATH="$WS/pysrc:${PYTHONPATH:-}"          # shared cacc modules
# let gz find the standalone cacc_car model if inspected directly
export GZ_SIM_RESOURCE_PATH="$WS/install/cacc_platoon/share/cacc_platoon/models:${GZ_SIM_RESOURCE_PATH:-}"

# --- one-time dependency check (install line printed, not run: needs sudo) ---
missing=""
command -v gz >/dev/null 2>&1 || missing="$missing gz-harmonic"
ros2 pkg prefix ros_gz_sim >/dev/null 2>&1 || missing="$missing ros-jazzy-ros-gz"
python3 -c "import gz.transport13, gz.msgs10" >/dev/null 2>&1 \
  || missing="$missing python3-gz-transport13 python3-gz-msgs10"
if [ -n "$missing" ]; then
  echo "!! missing Gazebo stack:$missing"
  echo "   install once, then re-run this script:"
  echo "     sudo apt update && sudo apt install -y$missing"
  exit 1
fi

RVIZ_ARG="false"; [ "$RVIZ" = "rviz" ] && RVIZ_ARG="true"
PARAMS="$WS/src/cacc_platoon/config/$CASE"
echo "gazebo view: n=$N controller=$CONTROLLER params=$PARAMS rviz=$RVIZ_ARG"

ros2 launch cacc_platoon gazebo.launch.py \
  n:="$N" controller:="$CONTROLLER" params:="$PARAMS" \
  outfile:=/tmp/platoon_gz.csv rviz:="$RVIZ_ARG"

echo "GAZEBO_OK"
