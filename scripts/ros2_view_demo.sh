#!/usr/bin/env bash
# Live 3D platoon demo: launch the platoon with viz markers and open rviz2
# (WSLg). Close the rviz window to stop everything. Usage (inside WSL):
#   bash scripts/ros2_view_demo.sh [N_FOLLOWERS] [CONTROLLER] [CASE_YAML]
# (no `set -u`: ROS setup.bash references unset variables)
set -eo pipefail

N="${1:-6}"
CONTROLLER="${2:-cthp}"
CASE="${3:-ma2025_case_a.yaml}"

WS="$HOME/cacc_ws"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export PYTHONPATH="$WS/pysrc:${PYTHONPATH:-}"   # shared cacc modules

PARAMS="$WS/src/cacc_platoon/config/$CASE"
echo "live view: n=$N controller=$CONTROLLER params=$PARAMS"
ros2 launch cacc_platoon platoon.launch.py \
  n:="$N" controller:="$CONTROLLER" params:="$PARAMS" \
  outfile:=/tmp/viz_run.csv viz:=true &
LAUNCH_PID=$!
trap 'kill -INT $LAUNCH_PID 2>/dev/null || true' EXIT

rviz2 -d "$WS/src/cacc_platoon/config/platoon.rviz" || true

kill -INT "$LAUNCH_PID" 2>/dev/null || true
wait "$LAUNCH_PID" 2>/dev/null || true
echo "VIEW_OK"
