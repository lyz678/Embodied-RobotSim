#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# 直接运行时的默认场景；演示入口会用自身配置覆盖这些值。
WORLD=simple_room
INITIAL_X=0.0
INITIAL_Y=0.0
INITIAL_YAW=0.0
HEADLESS=false
SIM_ARGS=(--world "$WORLD" --x "$INITIAL_X" --y "$INITIAL_Y" --yaw "$INITIAL_YAW")
[[ "$HEADLESS" == true ]] && SIM_ARGS+=(--headless)

ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

if [[ ! -f "$ROS_SETUP" ]]; then
    echo "错误：未找到 ROS 2 Jazzy 环境：$ROS_SETUP" >&2
    exit 1
fi

# ROS/colcon setup scripts reference optional unset variables.
set +u
source "$ROS_SETUP"
if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
    source "$ROOT_DIR/install/setup.bash"
else
    echo "错误：未找到 install/setup.bash，请先构建工作空间。" >&2
    exit 1
fi

set -u

ISAAC_PYTHON="${ISAAC_SIM_PYTHON:-}"
if [[ -z "$ISAAC_PYTHON" && -n "${ISAAC_SIM_PATH:-}" ]]; then
    ISAAC_PYTHON="$ISAAC_SIM_PATH/python.sh"
fi
if [[ -z "$ISAAC_PYTHON" && -x "$HOME/isaacsim/python.sh" ]]; then
    ISAAC_PYTHON="$HOME/isaacsim/python.sh"
fi
if [[ -z "$ISAAC_PYTHON" || ! -x "$ISAAC_PYTHON" ]]; then
    echo "错误：请设置 ISAAC_SIM_PATH（包含 python.sh）或 ISAAC_SIM_PYTHON。" >&2
    exit 1
fi

export ROS_DISTRO=jazzy
ISAAC_ROS_LIB="$(dirname "$(readlink -f "$ISAAC_PYTHON")")/exts/isaacsim.ros2.core/jazzy/lib"
if [[ -d "$ISAAC_ROS_LIB" ]]; then
    export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$ISAAC_ROS_LIB"
fi
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# The demo launcher already cleans before starting any service. Direct launches
# must clean too, without stopping controllers from the same demo session.
if [[ "${ISAAC_SIM_CLEANUP_DONE:-0}" != 1 ]]; then
    echo "🛑 正在清理残留进程..."
    bash "$ROOT_DIR/stop_robot_sim.sh"
    sleep 2
fi
unset ISAAC_SIM_CLEANUP_DONE

exec "$ISAAC_PYTHON" "$ROOT_DIR/src/x_bot/isaac_sim/run_sim.py" \
    --package-share "$ROOT_DIR/src/x_bot" \
    --franka-share "$ROOT_DIR/src/franka_description" \
    --controller-config "$ROOT_DIR/src/x_bot/config/isaac_controllers.yaml" \
    "${SIM_ARGS[@]}" "$@"
