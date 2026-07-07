#!/usr/bin/env bash
# Run a timed platoon demo in WSL and copy the CSV back into the repo's
# results/ros2/. Usage (inside WSL Ubuntu, after ros2_sync_build.sh):
#   bash scripts/ros2_run_demo.sh [N_FOLLOWERS] [DURATION_S] [CONTROLLER] [CASE_YAML]
# (no `set -u`: ROS setup.bash references unset variables)
set -eo pipefail

N="${1:-6}"
DURATION="${2:-120}"
CONTROLLER="${3:-cthp}"
CASE="${4:-ma2025_case_a.yaml}"

REPO="${REPO:-/mnt/c/Users/daasa/OneDrive/Desktop/sem 5/06-Control-System/PROJECT/06_Cooperative-Adaptive-Cruise-Control}"
WS="$HOME/cacc_ws"
STAMP="$(date +%Y%m%d-%H%M%S)"
OUT_WSL="/tmp/platoon_${STAMP}.csv"
OUT_REPO="$REPO/results/ros2/${STAMP}_${CONTROLLER}_n${N}.csv"

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export PYTHONPATH="$WS/pysrc:${PYTHONPATH:-}"   # shared cacc modules

PARAMS="$WS/src/cacc_platoon/config/$CASE"
echo "launching: n=$N controller=$CONTROLLER duration=${DURATION}s params=$PARAMS"
# SIGINT after DURATION lets the recorder close the CSV cleanly
timeout --signal=SIGINT --preserve-status "${DURATION}s" \
  ros2 launch cacc_platoon platoon.launch.py \
    n:="$N" controller:="$CONTROLLER" params:="$PARAMS" outfile:="$OUT_WSL" \
  || true

mkdir -p "$(dirname "$OUT_REPO")"
cp "$OUT_WSL" "$OUT_REPO"
echo "RUN_OK — csv copied to: $OUT_REPO"
