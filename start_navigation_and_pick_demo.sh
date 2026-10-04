#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 默认配置：直接运行本脚本即可；需要调整时修改这里。
MAP_BUNDLE="$ROOT_DIR/maps/room_a"
INITIAL_X=0.0
INITIAL_Y=0.0
INITIAL_YAW=0.0
HEADLESS=false
BUILD=false

DEFAULT_ARGS=(--bundle "$MAP_BUNDLE"
    --initial-x "$INITIAL_X" --initial-y "$INITIAL_Y" --initial-yaw "$INITIAL_YAW")
[[ "$HEADLESS" == true ]] && DEFAULT_ARGS+=(--headless)
[[ "$BUILD" == true ]] && DEFAULT_ARGS+=(--build)
exec bash "$ROOT_DIR/start_isaac_demo.sh" navigation_pick "${DEFAULT_ARGS[@]}" "$@"
