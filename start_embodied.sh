#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

TASK_MODE=default
USER_ARGS=()
while (($#)); do
    case "$1" in
        --mode) shift; TASK_MODE="${1:?--mode 缺少模式}" ;;
        --help|-h) echo "用法：$0 [--sim isaac|gazebo] [--world 场景] [--bundle 地图目录] [--depth-source sim|lsm] [--headless] [--build]"; exit 0 ;;
        *) USER_ARGS+=("$1") ;;
    esac
    shift
done
MODE=llm
[[ "$TASK_MODE" == default ]] || { echo "具身入口没有额外任务模式" >&2; exit 2; }

# 默认配置：公共参数以 Isaac Sim 为基准，--sim gazebo 可选择后端。
SIM_BACKEND=isaac
WORLD=simple_room
MAP_BUNDLE="$ROOT_DIR/maps/gazebo_simple_room"
INITIAL_X=0.0
INITIAL_Y=0.0
INITIAL_YAW=1.5708
HEADLESS=false
BUILD=false

DEFAULT_ARGS=(--sim "$SIM_BACKEND" --world "$WORLD" --bundle "$MAP_BUNDLE"
    --initial-x "$INITIAL_X" --initial-y "$INITIAL_Y" --initial-yaw "$INITIAL_YAW")
[[ "$HEADLESS" == true ]] && DEFAULT_ARGS+=(--headless)
[[ "$BUILD" == true ]] && DEFAULT_ARGS+=(--build)
exec bash "$ROOT_DIR/scripts/runtime/robot_services.sh" "$MODE" "${DEFAULT_ARGS[@]}" "${USER_ARGS[@]}"
