#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 默认配置：直接运行本脚本即可；需要调整时修改这里。
WORLD=simple_room
MAP_BUNDLE="$ROOT_DIR/maps/gazebo_simple_room"
INITIAL_X=0.0
INITIAL_Y=0.0
INITIAL_YAW=1.5708
HEADLESS=false
BUILD=true
AUTO_EXPLORE=true
DEPTH_SOURCE=isaac #lsm
SEMANTIC_CLOUD_SOURCE=fastlio #depth：depth 时由 DEPTH_SOURCE 选择 Isaac/LSM。
LSM_CONFIG_FILE="$ROOT_DIR/src/LSM_depth_infer/config/config.yaml"
LSM_PARAMS_FILE="$ROOT_DIR/src/LSM_depth_infer/config/isaac_params.yaml"
# 深度输入空值按 DEPTH_SOURCE 选择，lsm 默认 nn_depth。
# OCTOMAP_CLOUD_TOPIC 仅用于 legacy；新后端输入在 SEMANTIC_MAP_CONFIG 配置。
DEPTH_IMAGE_TOPIC=""
OCTOMAP_CLOUD_TOPIC=""
OCTOMAP_BACKEND=semantic_cuda
SEMANTIC_MAP_CONFIG="$ROOT_DIR/src/semantic_voxel_mapping/config/map.yaml"

DEFAULT_ARGS=(--world "$WORLD" --bundle "$MAP_BUNDLE"
    --initial-x "$INITIAL_X" --initial-y "$INITIAL_Y" --initial-yaw "$INITIAL_YAW"
    --depth-source "$DEPTH_SOURCE" --lsm-config "$LSM_CONFIG_FILE" --lsm-params "$LSM_PARAMS_FILE"
    --semantic-cloud-source "$SEMANTIC_CLOUD_SOURCE"
    --octomap-backend "$OCTOMAP_BACKEND" --semantic-map-config "$SEMANTIC_MAP_CONFIG")
[[ -n "$DEPTH_IMAGE_TOPIC" ]] && DEFAULT_ARGS+=(--depth-image-topic "$DEPTH_IMAGE_TOPIC")
[[ -n "$OCTOMAP_CLOUD_TOPIC" ]] && DEFAULT_ARGS+=(--octomap-cloud-topic "$OCTOMAP_CLOUD_TOPIC")
[[ "$HEADLESS" == true ]] && DEFAULT_ARGS+=(--headless)
[[ "$BUILD" == true ]] && DEFAULT_ARGS+=(--build)
[[ "$AUTO_EXPLORE" == false ]] && DEFAULT_ARGS+=(--no-explore)
exec bash "$ROOT_DIR/scripts/robot_services.sh" explore "${DEFAULT_ARGS[@]}" "$@"
