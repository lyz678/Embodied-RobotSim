#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

TASK_MODE=default
USER_ARGS=()
while (($#)); do
    case "$1" in
        --mode) shift; TASK_MODE="${1:?--mode 缺少模式}" ;;
        --help|-h) echo "用法：$0 [--llm local|cloud] [--sim isaac|gazebo] [--odometry lio|vio] [--world 场景] [--bundle 地图目录] [--semantic-cloud-source fastlio|depth] [--depth-source sim|lsm] [--headless] [--build]"; exit 0 ;;
        *) USER_ARGS+=("$1") ;;
    esac
    shift
done
MODE=llm
[[ "$TASK_MODE" == default ]] || { echo "具身入口没有额外任务模式" >&2; exit 2; }

# 默认配置：公共参数以 Isaac Sim 为基准，--sim gazebo 可选择后端。
SIM_BACKEND=isaac
export LLM_BACKEND="${LLM_BACKEND:-local}" # local：本地 Qwen；cloud：DashScope。
ODOMETRY_SOURCE=lio # vio：Isaac ROS 双目惯性里程计；lio：FAST-LIO。
WORLD=simple_room
MAP_BUNDLE="$ROOT_DIR/maps/gazebo_simple_room"
INITIAL_X=0.0
INITIAL_Y=0.0
INITIAL_YAW=1.5708
HEADLESS=false
BUILD=false

DEFAULT_ARGS=(--sim "$SIM_BACKEND" --odometry "$ODOMETRY_SOURCE" --world "$WORLD" --bundle "$MAP_BUNDLE"
    --initial-x "$INITIAL_X" --initial-y "$INITIAL_Y" --initial-yaw "$INITIAL_YAW"
    --semantic-cloud-source depth --depth-source sim
    --yoloe-config "$ROOT_DIR/src/perception/yoloe_infer/configs/manipulation.yaml")
[[ "$HEADLESS" == true ]] && DEFAULT_ARGS+=(--headless)
[[ "$BUILD" == true ]] && DEFAULT_ARGS+=(--build)
exec bash "$ROOT_DIR/scripts/runtime/robot_services.sh" "$MODE" "${DEFAULT_ARGS[@]}" "${USER_ARGS[@]}"
