#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

TASK_MODE=default
USER_ARGS=()
while (($#)); do
    case "$1" in
        --mode) shift; TASK_MODE="${1:?--mode 缺少模式}" ;;
        --help|-h) echo "用法：$0 [--mode navigation] [--sim isaac|gazebo] [--world 场景] [--bundle 地图目录] [--depth-source sim|lsm] [--headless] [--build]"; exit 0 ;;
        *) USER_ARGS+=("$1") ;;
    esac
    shift
done
MODE=pick
case "$TASK_MODE" in default) ;; navigation) MODE=navigation_pick ;; *) echo "模式必须为 default 或 navigation" >&2; exit 2 ;; esac

# 默认配置：公共参数以 Isaac Sim 为基准，--sim gazebo 可选择后端。
SIM_BACKEND=isaac
WORLD=manipulation_test
MAP_BUNDLE="$ROOT_DIR/maps/manipulation_test"
INITIAL_X=0.0
INITIAL_Y=0.0
INITIAL_YAW=0.0
HEADLESS=false
BUILD=true
YOLOE_CONFIG="$ROOT_DIR/src/perception/yoloe_infer/configs/manipulation.yaml"
DEPTH_SOURCE=sim # lsm 可切换为双目推理深度。
SEMANTIC_CLOUD_SOURCE=depth
SEMANTIC_MAP_CONFIG="$ROOT_DIR/src/mapping/semantic_voxel_mapping/config/manipulation.yaml"
OCTOMAP_RESOLUTION=0.02 # 与 main/Gazebo 抓取的 MoveIt OctoMap 一致：2 cm。
LSM_CONFIG_FILE="$ROOT_DIR/src/perception/LSM_depth_infer/config/config.yaml"
LSM_PARAMS_FILE="$ROOT_DIR/src/perception/LSM_depth_infer/config/isaac_params.yaml"
# 留空时选择 Isaac/LSM 深度；建图统一接 YOLOE 的 pointcloud_semantic。
DEPTH_IMAGE_TOPIC=""
OCTOMAP_CLOUD_TOPIC=""

if [[ "$MODE" == navigation_pick ]]; then
    WORLD=simple_room
    MAP_BUNDLE="$ROOT_DIR/maps/gazebo_simple_room"
    YOLOE_CONFIG="$ROOT_DIR/src/perception/yoloe_infer/configs/config.yaml"
fi
DEFAULT_ARGS=(--sim "$SIM_BACKEND" --world "$WORLD" --bundle "$MAP_BUNDLE"
    --initial-x "$INITIAL_X" --initial-y "$INITIAL_Y" --initial-yaw "$INITIAL_YAW"
    --yoloe-config "$YOLOE_CONFIG" --depth-source "$DEPTH_SOURCE" --semantic-cloud-source "$SEMANTIC_CLOUD_SOURCE"
    --octomap-resolution "$OCTOMAP_RESOLUTION" --semantic-map-config "$SEMANTIC_MAP_CONFIG"
    --lsm-config "$LSM_CONFIG_FILE" --lsm-params "$LSM_PARAMS_FILE")
[[ -n "$DEPTH_IMAGE_TOPIC" ]] && DEFAULT_ARGS+=(--depth-image-topic "$DEPTH_IMAGE_TOPIC")
[[ -n "$OCTOMAP_CLOUD_TOPIC" ]] && DEFAULT_ARGS+=(--octomap-cloud-topic "$OCTOMAP_CLOUD_TOPIC")
[[ "$HEADLESS" == true ]] && DEFAULT_ARGS+=(--headless)
[[ "$BUILD" == true ]] && DEFAULT_ARGS+=(--build)
exec bash "$ROOT_DIR/scripts/runtime/robot_services.sh" "$MODE" "${DEFAULT_ARGS[@]}" "${USER_ARGS[@]}"
