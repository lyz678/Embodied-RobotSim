#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

if [[ ! -f "$ROS_SETUP" ]]; then
    echo "错误：未找到 ROS 2 Jazzy 环境：$ROS_SETUP" >&2
    exit 1
fi

source "$ROS_SETUP"
if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
    source "$ROOT_DIR/install/setup.bash"
else
    echo "错误：未找到 install/setup.bash，请先构建工作空间。" >&2
    exit 1
fi

ISAAC_PYTHON="${ISAAC_SIM_PYTHON:-}"
if [[ -z "$ISAAC_PYTHON" && -n "${ISAAC_SIM_PATH:-}" ]]; then
    ISAAC_PYTHON="$ISAAC_SIM_PATH/python.sh"
fi
if [[ -z "$ISAAC_PYTHON" || ! -x "$ISAAC_PYTHON" ]]; then
    echo "错误：请设置 ISAAC_SIM_PATH（包含 python.sh）或 ISAAC_SIM_PYTHON。" >&2
    exit 1
fi

export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

exec "$ISAAC_PYTHON" "$ROOT_DIR/src/x_bot/isaac_sim/run_sim.py" \
    --package-share "$ROOT_DIR/src/x_bot" \
    --franka-share "$ROOT_DIR/src/franka_description" \
    --controller-config "$ROOT_DIR/src/x_bot/config/isaac_controllers.yaml" \
    "$@"
