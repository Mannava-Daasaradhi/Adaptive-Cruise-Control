#!/usr/bin/env bash
# One-time install of the Gazebo (gz-sim Harmonic) stack needed by
# gazebo.launch.py / ros2_gazebo_demo.sh. Ubuntu 24.04 (noble) + ROS 2 Jazzy.
# Adds the OSRF apt repo (gz-harmonic lives there, not in the Ubuntu/ROS repos)
# then installs ros_gz + Gazebo + the Python transport/msgs bindings.
#
# Run inside a WSL Ubuntu terminal (asks for your sudo password once):
#     bash scripts/setup_gazebo_wsl.sh
set -eo pipefail

CODENAME="$(. /etc/os-release 2>/dev/null; echo "${VERSION_CODENAME:-noble}")"
echo ">> Ubuntu codename: $CODENAME"

# --- 1) add the OSRF/Gazebo apt repo (idempotent) ---------------------------
if [ ! -f /etc/apt/sources.list.d/gazebo-stable.list ]; then
  echo ">> adding the Gazebo (OSRF) apt repository"
  sudo apt-get update
  sudo apt-get install -y curl lsb-release gnupg
  sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $CODENAME main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
else
  echo ">> Gazebo apt repo already present — skipping"
fi

# --- 2) install the stack ---------------------------------------------------
echo ">> apt update + install (this is the ~500 MB step)"
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-ros-gz \
  gz-harmonic \
  python3-gz-transport13 \
  python3-gz-msgs10

# --- 3) verify --------------------------------------------------------------
echo ""; echo ">> verifying:"
ok=1
command -v gz >/dev/null 2>&1 && echo "   [ok] gz cli: $(gz sim --version 2>/dev/null | head -1)" || { echo "   [MISSING] gz cli"; ok=0; }
( source /opt/ros/jazzy/setup.bash && ros2 pkg prefix ros_gz_sim >/dev/null 2>&1 ) \
  && echo "   [ok] ros_gz_sim package" || { echo "   [MISSING] ros_gz_sim"; ok=0; }
python3 -c "import gz.transport13, gz.msgs10" 2>/dev/null \
  && echo "   [ok] python bindings (gz.transport13 + gz.msgs10)" || { echo "   [MISSING] python bindings"; ok=0; }

echo ""
if [ "$ok" = 1 ]; then
  echo ">> SETUP_OK — now run:"
  echo "     bash scripts/ros2_sync_build.sh"
  echo "     bash scripts/ros2_gazebo_demo.sh 6 cthp ma2025_case_a.yaml rviz"
else
  echo ">> some pieces are still missing; run 'apt-cache search python3-gz' to"
  echo "   check the exact binding package names for your mirror, then re-run."
  exit 1
fi
