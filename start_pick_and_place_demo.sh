#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 默认配置：直接运行本脚本即可；需要调整时修改这里。
WORLD=manipulation_test
MAP_BUNDLE="$ROOT_DIR/maps/manipulation_test"
INITIAL_X=0.0
INITIAL_Y=0.0
INITIAL_YAW=0.0
HEADLESS=false
BUILD=true
YOLOE_CONFIG="$ROOT_DIR/src/yoloe_infer/configs/manipulation.yaml"
DEPTH_SOURCE=isaac # lsm 可切换为双目推理深度。
SEMANTIC_CLOUD_SOURCE=depth
SEMANTIC_MAP_CONFIG="$ROOT_DIR/src/semantic_voxel_mapping/config/manipulation.yaml"
OCTOMAP_RESOLUTION=0.02 # 与 main/Gazebo 抓取的 MoveIt OctoMap 一致：2 cm。
LSM_CONFIG_FILE="$ROOT_DIR/src/LSM_depth_infer/config/config.yaml"
LSM_PARAMS_FILE="$ROOT_DIR/src/LSM_depth_infer/config/isaac_params.yaml"
# 留空时选择 Isaac/LSM 深度；建图统一接 YOLOE 的 pointcloud_semantic。
DEPTH_IMAGE_TOPIC=""
OCTOMAP_CLOUD_TOPIC=""

DEFAULT_ARGS=(--world "$WORLD" --bundle "$MAP_BUNDLE"
    --initial-x "$INITIAL_X" --initial-y "$INITIAL_Y" --initial-yaw "$INITIAL_YAW"
    --yoloe-config "$YOLOE_CONFIG" --depth-source "$DEPTH_SOURCE" --semantic-cloud-source "$SEMANTIC_CLOUD_SOURCE"
    --octomap-resolution "$OCTOMAP_RESOLUTION" --semantic-map-config "$SEMANTIC_MAP_CONFIG"
    --lsm-config "$LSM_CONFIG_FILE" --lsm-params "$LSM_PARAMS_FILE")
[[ -n "$DEPTH_IMAGE_TOPIC" ]] && DEFAULT_ARGS+=(--depth-image-topic "$DEPTH_IMAGE_TOPIC")
[[ -n "$OCTOMAP_CLOUD_TOPIC" ]] && DEFAULT_ARGS+=(--octomap-cloud-topic "$OCTOMAP_CLOUD_TOPIC")
[[ "$HEADLESS" == true ]] && DEFAULT_ARGS+=(--headless)
[[ "$BUILD" == true ]] && DEFAULT_ARGS+=(--build)
exec bash "$ROOT_DIR/scripts/robot_services.sh" pick "${DEFAULT_ARGS[@]}" "$@"
