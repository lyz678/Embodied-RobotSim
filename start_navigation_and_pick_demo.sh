#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 默认配置：公共参数以 Isaac Sim 为基准，--sim gazebo 可选择后端。
SIM_BACKEND=isaac
WORLD=simple_room
MAP_BUNDLE="$ROOT_DIR/maps/gazebo_simple_room"
INITIAL_X=0.0
INITIAL_Y=0.0
INITIAL_YAW=0.0
HEADLESS=false
BUILD=true

DEFAULT_ARGS=(--sim "$SIM_BACKEND" --world "$WORLD" --bundle "$MAP_BUNDLE"
    --initial-x "$INITIAL_X" --initial-y "$INITIAL_Y" --initial-yaw "$INITIAL_YAW")
[[ "$HEADLESS" == true ]] && DEFAULT_ARGS+=(--headless)
[[ "$BUILD" == true ]] && DEFAULT_ARGS+=(--build)
exec bash "$ROOT_DIR/scripts/robot_services.sh" navigation_pick "${DEFAULT_ARGS[@]}" "$@"
