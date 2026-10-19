#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

TASK_MODE=default
PICK_BACKEND=graspnet
ACT_MODEL=rgb
USER_ARGS=()
while (($#)); do
    case "$1" in
        --mode) shift; TASK_MODE="${1:?--mode 缺少模式}" ;;
        --pipeline) shift; case "${1:?--pipeline 缺少 traditional/act}" in traditional) PICK_BACKEND=graspnet ;; act) PICK_BACKEND=act ;; *) echo "链路必须为 traditional 或 act" >&2; exit 2 ;; esac ;;
        --act-model) shift; ACT_MODEL="${1:?--act-model 缺少 rgb/rgbd/rgbxyz}" ;;
        --pick-backend) shift; PICK_BACKEND="${1:?--pick-backend 缺少 act/graspnet}" ;;
        --help|-h) echo "用法：$0 [--pipeline traditional|act] [--act-model rgb|rgbd|rgbxyz] [--pick-backend act|graspnet] [--llm local|cloud] [--sim isaac|gazebo] [--odometry lio|vio] [--world 场景] [--bundle 地图目录] [--semantic-cloud-source fastlio|depth] [--depth-source sim|lsm] [--headless] [--build]"; exit 0 ;;
        *) USER_ARGS+=("$1") ;;
    esac
    shift
done
MODE=llm
case "$PICK_BACKEND" in act|graspnet) ;; *) echo "抓取后端必须为 act 或 graspnet" >&2; exit 2 ;; esac
case "$ACT_MODEL" in rgb|rgbd|rgbxyz) ;; *) echo "ACT 模型必须为 rgb、rgbd 或 rgbxyz" >&2; exit 2 ;; esac
export EMBODIED_PICK_BACKEND="$PICK_BACKEND"
export EMBODIED_ACT_MODEL="$ACT_MODEL"
echo "具身链路：$PICK_BACKEND；ACT模型：$ACT_MODEL" >&2
if [[ "$PICK_BACKEND" == act ]]; then
    python3 - "$ROOT_DIR/src/embodied/act_policy/config.json" "$ACT_MODEL" <<'PY'
import json,sys,os
from pathlib import Path
c=json.loads(Path(sys.argv[1]).read_text())
c['checkpoint']=os.environ.get('EMBODIED_ACT_CHECKPOINT') or c['checkpoints'][sys.argv[2]]
for key in ('ros_python','checkpoint'):
    if not Path(c[key]).is_file():raise SystemExit('ACT 配置文件不存在：'+c[key])
print('具身抓取后端：ACT '+sys.argv[2]+' ResNet18 + category | '+c['checkpoint'],file=sys.stderr)
PY
fi
[[ "$TASK_MODE" == default ]] || { echo "具身入口没有额外任务模式" >&2; exit 2; }

# 默认使用 Gazebo；--sim isaac 可选择 Isaac Sim。
SIM_BACKEND=gazebo
export LLM_BACKEND="${LLM_BACKEND:-local}" # local：本地 Qwen；cloud：DashScope。
ODOMETRY_SOURCE=lio # vio：Isaac ROS 双目惯性里程计；lio：FAST-LIO。
WORLD=simple_room
MAP_BUNDLE="$ROOT_DIR/maps/semantic_simple_room"
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
