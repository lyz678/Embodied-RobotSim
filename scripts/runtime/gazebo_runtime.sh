#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
INSTALL_DIR="${ROBOT_INSTALL_PREFIX:-$ROOT_DIR/install}"
WORLD=simple_room
X=0.0 Y=0.0 YAW=0.0 HEADLESS=false
while (($#)); do
  case "$1" in
    --world) shift; WORLD="${1:?}" ;;
    --x) shift; X="${1:?}" ;;
    --y) shift; Y="${1:?}" ;;
    --yaw) shift; YAW="${1:?}" ;;
    --headless) HEADLESS=true ;;
    *) echo "Unknown Gazebo option: $1" >&2; exit 2 ;;
  esac
  shift
done
set +u
source /opt/ros/jazzy/setup.bash
source "$INSTALL_DIR/setup.bash"
set -u
PLUGIN_DIR="$(ros2 pkg prefix x_bot_gazebo)/lib"
[[ -f "$PLUGIN_DIR/libembodied_gazebo_backend.so" ]] || { echo 'Gazebo backend not built; install libembree-dev and rebuild x_bot_gazebo.' >&2; exit 1; }
ros2 pkg prefix gz_ros2_control >/dev/null
WORLD_FILE="$(python3 "$ROOT_DIR/scripts/assets/prepare_gazebo_scene.py" --world "$WORLD")"
SCENE_MODELS="$(dirname "$(dirname "$WORLD_FILE")")/source/models"
export GZ_SIM_RESOURCE_PATH="$SCENE_MODELS${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PLUGIN_DIR${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
# Embree may be unpacked locally for development without sudo.
export LD_LIBRARY_PATH="$ROOT_DIR/.cache/embree/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
if [[ "${ISAAC_SIM_CLEANUP_DONE:-0}" != 1 ]]; then bash "$ROOT_DIR/stop_robot_sim.sh"; fi
exec ros2 launch x_bot_gazebo backend.launch.py world_file:="$WORLD_FILE" x:="$X" y:="$Y" yaw:="$YAW" headless:="$HEADLESS"
