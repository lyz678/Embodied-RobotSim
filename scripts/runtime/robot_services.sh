#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
INSTALL_DIR="${ROBOT_INSTALL_PREFIX:-$ROOT_DIR/install}"
MODE="${1:-explore}"
if (($#)); then shift; fi

case "$MODE" in
    explore|navigation|pick|navigation_pick|llm) ;;
    *)
        echo "用法：$0 {explore|navigation|pick|navigation_pick|llm} [--sim isaac|gazebo] [--odometry lio|vio] [--world 场景] [--headless] [--build] [--no-explore] [--bundle 地图目录] [--initial-x X --initial-y Y --initial-yaw RAD]" >&2
        exit 2
        ;;
esac

SIM_BACKEND=isaac
ODOMETRY_SOURCE=lio
HEADLESS=false
BUILD=false
AUTO_EXPLORE=true
BUNDLE="$ROOT_DIR/maps/gazebo_simple_room"
WORLD=simple_room
if [[ "$MODE" == pick ]]; then
    WORLD=manipulation_test
    BUNDLE="$ROOT_DIR/maps/manipulation_test"
fi
INITIAL_X=0.0
INITIAL_Y=0.0
INITIAL_YAW=""
YOLOE_CONFIG="$ROOT_DIR/src/perception/yoloe_infer/configs/config.yaml"
[[ "$MODE" == explore ]] && YOLOE_CONFIG="$ROOT_DIR/src/perception/yoloe_infer/configs/tracking.yaml"
[[ "$MODE" == pick ]] && YOLOE_CONFIG="$ROOT_DIR/src/perception/yoloe_infer/configs/manipulation.yaml"
DEPTH_SOURCE=sim
[[ "$MODE" == explore ]] && DEPTH_SOURCE=lsm
LSM_CONFIG_FILE="$ROOT_DIR/src/perception/LSM_depth_infer/config/config.yaml"
LSM_PARAMS_FILE="$ROOT_DIR/src/perception/LSM_depth_infer/config/isaac_params.yaml"
DEPTH_IMAGE_TOPIC=""
OCTOMAP_CLOUD_TOPIC=""
OCTOMAP_BACKEND=semantic_cuda
OCTOMAP_RESOLUTION=0.10
[[ "$MODE" != explore ]] && OCTOMAP_RESOLUTION=0.02
[[ "$MODE" != llm ]] || OCTOMAP_RESOLUTION=0.02
SEMANTIC_MAP_CONFIG="$ROOT_DIR/src/mapping/semantic_voxel_mapping/config/map.yaml"
[[ "$MODE" != explore ]] && SEMANTIC_MAP_CONFIG="$ROOT_DIR/src/mapping/semantic_voxel_mapping/config/manipulation.yaml"
[[ "$MODE" != llm ]] || SEMANTIC_MAP_CONFIG="$ROOT_DIR/src/mapping/semantic_voxel_mapping/config/manipulation.yaml"
SEMANTIC_CLOUD_SOURCE=depth
[[ "$MODE" == explore ]] && SEMANTIC_CLOUD_SOURCE=fastlio
# GraspNet consumes depth-based colored clouds and Detection3DArray. The
# FAST-LIO semantic branch currently publishes map geometry only.
[[ "$MODE" != llm ]] || SEMANTIC_CLOUD_SOURCE=depth
while (($#)); do
    case "$1" in
        --llm) shift; export LLM_BACKEND="${1:?缺少 local/cloud}" ;;
        --sim) shift; SIM_BACKEND="${1:?缺少 isaac/gazebo}" ;;
        --odometry) shift; ODOMETRY_SOURCE="${1:?缺少 lio/vio}" ;;
        --headless) HEADLESS=true ;;
        --build) BUILD=true ;;
        --no-build) BUILD=false ;;
        --no-explore) AUTO_EXPLORE=false ;;
        --bundle) shift; BUNDLE="${1:?--bundle 缺少目录}" ;;
        --world) shift; WORLD="${1:?--world 缺少场景}" ;;
        --initial-x) shift; INITIAL_X="${1:?缺少 x}" ;;
        --initial-y) shift; INITIAL_Y="${1:?缺少 y}" ;;
        --initial-yaw) shift; INITIAL_YAW="${1:?缺少 yaw}" ;;
        --yoloe-config) shift; YOLOE_CONFIG="${1:?缺少 YOLOE 配置}" ;;
        --depth-source) shift; DEPTH_SOURCE="${1:?缺少深度来源 sim/lsm}" ;;
        --lsm-config) shift; LSM_CONFIG_FILE="${1:?缺少 LSM 配置文件}" ;;
        --lsm-params) shift; LSM_PARAMS_FILE="${1:?缺少 LSM ROS 参数文件}" ;;
        --depth-image-topic) shift; DEPTH_IMAGE_TOPIC="${1:?缺少深度图话题}" ;;
        --octomap-cloud-topic) shift; OCTOMAP_CLOUD_TOPIC="${1:?缺少 OctoMap 点云话题}" ;;
        --octomap-backend) shift; OCTOMAP_BACKEND="${1:?缺少 semantic_cuda}" ;;
        --octomap-resolution) shift; OCTOMAP_RESOLUTION="${1:?缺少 OctoMap 分辨率（米）}" ;;
        --semantic-map-config) shift; SEMANTIC_MAP_CONFIG="${1:?缺少语义地图配置}" ;;
        --semantic-cloud-source) shift; SEMANTIC_CLOUD_SOURCE="${1:?缺少语义点云来源 fastlio/depth}" ;;
        *.yaml) echo "定位需要配套 PCD 地图目录，请用 --bundle；不能只用旧二维 YAML 地图。" >&2; exit 2 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
    shift
done

if [[ "$BUNDLE" == "$ROOT_DIR/maps/semantic_simple_room" ]]; then
    BUNDLE="$ROOT_DIR/maps/$SIM_BACKEND/semantic_$WORLD"
fi
[[ "$BUNDLE" == /* ]] || BUNDLE="$ROOT_DIR/$BUNDLE"

case "$SIM_BACKEND" in
    isaac|gazebo) ;;
    *) echo "仿真器必须为 isaac 或 gazebo" >&2; exit 2 ;;
esac
case "$ODOMETRY_SOURCE" in
    lio|vio) ;;
    *) echo "里程计必须为 lio 或 vio" >&2; exit 2 ;;
esac
if [[ "$SEMANTIC_CLOUD_SOURCE" == auto ]]; then
    if [[ "$ODOMETRY_SOURCE" == lio ]]; then
        SEMANTIC_CLOUD_SOURCE=fastlio
    else
        SEMANTIC_CLOUD_SOURCE=depth
    fi
fi
if [[ "$ODOMETRY_SOURCE" == vio && "$SEMANTIC_CLOUD_SOURCE" == fastlio ]]; then
    SEMANTIC_CLOUD_SOURCE=depth
    echo "VIO 模式：语义建图使用 depth 点云"
fi
if [[ "$SIM_BACKEND" == gazebo ]]; then
    case "$WORLD" in simple_room|manipulation_test) ;; *) echo "Gazebo 仅支持 simple_room/manipulation_test" >&2; exit 2 ;; esac
    [[ "$DEPTH_SOURCE" != isaac ]] || { echo "Gazebo 深度来源请使用 sim（或 lsm）" >&2; exit 2; }
    [[ "$BUNDLE" != "$ROOT_DIR/maps/gazebo_simple_room" ]] || BUNDLE="$ROOT_DIR/maps/gazebo/simple_room"
    [[ "$BUNDLE" != "$ROOT_DIR/maps/manipulation_test" ]] || BUNDLE="$ROOT_DIR/maps/gazebo/manipulation_test"
fi

if ! python3 -c 'import sys; value=float(sys.argv[1]); sys.exit(not (0.005 <= value <= 1.0))' "$OCTOMAP_RESOLUTION"; then
    echo "OctoMap 分辨率必须为 0.005 到 1.0 米" >&2
    exit 2
fi

case "$OCTOMAP_BACKEND" in
    semantic_cuda)
        [[ -f "$SEMANTIC_MAP_CONFIG" && "$SEMANTIC_MAP_CONFIG" != *"'"* && "$SEMANTIC_MAP_CONFIG" != *$'\n'* ]] || { echo "语义地图配置无效：$SEMANTIC_MAP_CONFIG" >&2; exit 2; }
        ;;
    *) echo "建图后端统一为 semantic_cuda" >&2; exit 2 ;;
esac

case "$SEMANTIC_CLOUD_SOURCE" in
    fastlio|depth) ;;
    *) echo "语义点云来源必须为 fastlio 或 depth" >&2; exit 2 ;;
esac
if [[ ( "$MODE" == llm || "$MODE" == pick || "$MODE" == navigation_pick ) && "$SEMANTIC_CLOUD_SOURCE" != depth ]]; then
    echo "抓取局部图需要 --semantic-cloud-source depth；FAST-LIO 语义分支不发布 GraspNet 的三维检测和彩色点云。" >&2
    exit 2
fi
USE_LSM=false
[[ "$DEPTH_SOURCE" == lsm && "$SEMANTIC_CLOUD_SOURCE" == depth ]] && USE_LSM=true

case "$DEPTH_SOURCE" in
    lsm)
        DEPTH_IMAGE_TOPIC="${DEPTH_IMAGE_TOPIC:-/x_bot/camera_left/nn_depth}"
        OCTOMAP_CLOUD_TOPIC="${OCTOMAP_CLOUD_TOPIC:-/yoloe_multi_text_prompt/pointcloud_semantic}"
        if [[ "$USE_LSM" == true ]]; then
            for config_path in "$LSM_CONFIG_FILE" "$LSM_PARAMS_FILE"; do
                [[ -f "$config_path" && "$config_path" != *"'"* && "$config_path" != *$'\n'* ]] || { echo "LSM 配置文件无效：$config_path" >&2; exit 2; }
            done
        fi
        ;;
    sim|isaac)
        DEPTH_IMAGE_TOPIC="${DEPTH_IMAGE_TOPIC:-/x_bot/camera_left/depth/image_raw}"
        OCTOMAP_CLOUD_TOPIC="${OCTOMAP_CLOUD_TOPIC:-/yoloe_multi_text_prompt/pointcloud_semantic}"
        ;;
    *) echo "深度来源必须为 sim、lsm（Isaac 可用旧值 isaac）" >&2; exit 2 ;;
esac
for topic in "$DEPTH_IMAGE_TOPIC" "$OCTOMAP_CLOUD_TOPIC"; do
    [[ "$topic" =~ ^/[a-zA-Z0-9_/]+$ ]] || { echo "无效 ROS 话题：$topic" >&2; exit 2; }
done

[[ -f "$YOLOE_CONFIG" && "$YOLOE_CONFIG" != *"'"* && "$YOLOE_CONFIG" != *$'\n'* ]] || { echo "YOLOE 配置无效：$YOLOE_CONFIG" >&2; exit 2; }
if [[ "$MODE" == explore && -e "$BUNDLE" ]]; then
    echo "错误：建图目标目录已存在：$BUNDLE；保存器不会覆盖旧地图。请用 --bundle 指定新目录，随后让 start_embodied.sh 使用同一 --bundle；尚未停止当前仿真。" >&2
    exit 1
fi

cd "$ROOT_DIR"
# Keep large camera/cloud writes off lifecycle/service callback threads.
export RMW_FASTRTPS_PUBLICATION_MODE="${RMW_FASTRTPS_PUBLICATION_MODE:-ASYNCHRONOUS}"
# Load ROS for the cleanup script even when launched from a fresh shell.
set +u
source /opt/ros/jazzy/setup.bash
[[ ! -f "$INSTALL_DIR/setup.bash" ]] || source "$INSTALL_DIR/setup.bash"
set -u
has_runtime_package() (
    # Keep the relocated native ROS libraries out of Isaac Sim's environment.
    if [[ "$ODOMETRY_SOURCE" == vio && -f "$ROOT_DIR/.cache/vslam/env.bash" ]]; then
        source "$ROOT_DIR/.cache/vslam/env.bash"
    fi
    ros2 pkg prefix "$1" >/dev/null 2>&1
)
if [[ "$ODOMETRY_SOURCE" == vio ]] && ! has_runtime_package isaac_ros_visual_slam; then
    echo "错误：VIO 需要 Isaac ROS 4.x 的 isaac_ros_visual_slam。安装说明：$ROOT_DIR/src/localization/README.md；尚未停止当前仿真。" >&2
    exit 1
fi
case "${LLM_BACKEND:-cloud}" in
    local|cloud) ;;
    *) echo "LLM 后端必须为 local 或 cloud" >&2; exit 2 ;;
esac
if [[ ( "$MODE" == llm || "$MODE" == explore ) && "${LLM_BACKEND:-cloud}" == local ]]; then
    bash "$ROOT_DIR/src/embodied/llm_agent/start_local_model.sh" --check
fi
if [[ "$MODE" == explore || "$MODE" == llm ]]; then
    semantic_python="${AGENT_PYTHON:-$ROOT_DIR/.venv/agent/bin/python}"
    [[ -x "$semantic_python" ]] || semantic_python=python3
    "$semantic_python" -c 'import openai, PIL, numpy, yaml' || {
        echo "错误：缺少语义建图 Python 依赖；请执行 scripts/setup/setup_embodied_dependencies.sh --local；尚未停止当前仿真。" >&2
        exit 1
    }
    if [[ "${LLM_BACKEND:-cloud}" == cloud && -z "${AGENT_API_KEY:-${DASHSCOPE_API_KEY:-}}" ]]; then
        echo "错误：云端语义建图需要 AGENT_API_KEY 或 DASHSCOPE_API_KEY，或使用 --llm local；尚未停止当前仿真。" >&2
        exit 1
    fi
fi
# Check optional Web dependencies before stopping an existing simulation.
if [[ "$MODE" == llm ]]; then
    (
        [[ ! -f "$ROOT_DIR/.cache/embodied_sysroot/env.bash" ]] || source "$ROOT_DIR/.cache/embodied_sysroot/env.bash"
        for package in rosbridge_server web_video_server; do
            ros2 pkg prefix "$package" >/dev/null 2>&1 || {
                echo "错误：缺少 $package；请执行 bash scripts/setup/setup_embodied_dependencies.sh --local；尚未停止当前仿真。" >&2
                exit 1
            }
        done
        agent_python="${AGENT_PYTHON:-python3}"
        [[ ! -x "$ROOT_DIR/.venv/agent/bin/python" || "$agent_python" != python3 ]] || agent_python="$ROOT_DIR/.venv/agent/bin/python"
        "$agent_python" -c 'import openai, fastapi, uvicorn, websockets' || {
            echo "错误：缺少 Agent Python 依赖；请执行 bash scripts/setup/setup_embodied_dependencies.sh --local；尚未停止当前仿真。" >&2
            exit 1
        }
    ) || exit 1
fi
if [[ "$SIM_BACKEND" == gazebo && "$BUILD" != true ]]; then
    if ! bash "$ROOT_DIR/scripts/runtime/check_gazebo_plugins.sh"; then
        echo "Gazebo 启动检查失败；尚未停止当前仿真。" >&2
        exit 1
    fi
fi
echo "🛑 正在清理残留进程..."
bash "$ROOT_DIR/stop_robot_sim.sh"
sleep 2

if [[ "$BUNDLE" != /* ]]; then
    BUNDLE="$ROOT_DIR/$BUNDLE"
fi
# Preserve existing paired maps; every repeated mapping run gets a new output.
if [[ "$MODE" == explore && -e "$BUNDLE" ]]; then
    PREVIOUS_BUNDLE="$BUNDLE"
    BUNDLE="${BUNDLE}_$(date +%Y%m%d_%H%M%S)_$$"
    echo "已有地图保留在 $PREVIOUS_BUNDLE；本次新地图保存到 $BUNDLE"
fi
if [[ "$BUILD" == true ]]; then
    # ROS setup scripts are not compatible with nounset.
    set +u
    source /opt/ros/jazzy/setup.bash
    set -u
    colcon build --build-base "${ROBOT_BUILD_BASE:-$ROOT_DIR/build}" --install-base "$INSTALL_DIR" --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
fi
if [[ ! -f "$INSTALL_DIR/setup.bash" ]]; then
    echo "错误：未找到 install/setup.bash；请先执行 colcon build --symlink-install。" >&2
    exit 1
fi
set +u
source "$INSTALL_DIR/setup.bash"
set -u
if [[ "$MODE" == llm && -f "$ROOT_DIR/.cache/embodied_sysroot/env.bash" ]]; then
    source "$ROOT_DIR/.cache/embodied_sysroot/env.bash"
fi
AGENT_PYTHON="${AGENT_PYTHON:-python3}"
[[ ! -x "$ROOT_DIR/.venv/agent/bin/python" || "$AGENT_PYTHON" != python3 ]] || AGENT_PYTHON="$ROOT_DIR/.venv/agent/bin/python"
REQUIRED_PACKAGES=(x_bot x_bot_localization x_bot_bringup x_bot_control x_bot_sensors x_bot_mapping x_bot_navigation)
if [[ "$ODOMETRY_SOURCE" == lio ]]; then
    REQUIRED_PACKAGES+=(livox_ros_driver2 fast_lio)
else
    REQUIRED_PACKAGES+=(isaac_ros_visual_slam rclcpp_components cv_bridge)
fi
[[ "$SIM_BACKEND" != isaac ]] || REQUIRED_PACKAGES+=(x_bot_isaac)
[[ "$MODE" != llm ]] || REQUIRED_PACKAGES+=(explore_lite)
case "$MODE" in pick|navigation_pick|llm) REQUIRED_PACKAGES+=(x_bot_manipulation x_bot_moveit_config graspnet_ros yoloe_infer) ;; explore|navigation) REQUIRED_PACKAGES+=(yoloe_infer) ;; esac
[[ "$MODE" != llm ]] || REQUIRED_PACKAGES+=(rosbridge_server web_video_server)
[[ "$SIM_BACKEND" != gazebo ]] || REQUIRED_PACKAGES+=(x_bot_gazebo ros_gz_sim ros_gz_bridge gz_ros2_control)
for package in "${REQUIRED_PACKAGES[@]}"; do
    if ! has_runtime_package "$package"; then
        echo "错误：未找到 $SIM_BACKEND 运行依赖 $package；具身 Web 依赖见 scripts/setup/setup_embodied_dependencies.sh，其余依赖见 README；VIO 依赖见 src/localization/README.md。" >&2
        exit 1
    fi
done
if [[ "$MODE" == llm ]]; then
    "$AGENT_PYTHON" -c 'import openai, fastapi, uvicorn, websockets' || {
        echo "缺少 Agent Python 依赖，请执行 scripts/setup/setup_embodied_dependencies.sh。" >&2
        exit 1
    }
fi
if [[ "$SIM_BACKEND" == gazebo ]]; then
    bash "$ROOT_DIR/scripts/runtime/check_gazebo_plugins.sh"
fi
if [[ "$USE_LSM" == true ]]; then
    ros2 pkg prefix stereo_matching >/dev/null 2>&1 || { echo "错误：缺少 stereo_matching，请先构建 LSM_depth_infer 或使用 --build。" >&2; exit 1; }
fi
if [[ "$OCTOMAP_BACKEND" == semantic_cuda ]]; then
    ros2 pkg prefix semantic_voxel_mapping >/dev/null 2>&1 || { echo "错误：缺少 semantic_voxel_mapping，请使用 --build。" >&2; exit 1; }
fi
if [[ "$MODE" == explore && "$AUTO_EXPLORE" == true ]]; then
    for package in explore_lite nav2_bringup nav2_graceful_controller nav2_rotation_shim_controller; do
        ros2 pkg prefix "$package" >/dev/null 2>&1 || { echo "错误：缺少自动探索依赖 $package" >&2; exit 1; }
    done
fi
# 与 main 的启动流程一致：探索/抓取建图，导航类要求已有地图。
LOCALIZATION_MODE=mapping
AUTO_SEMANTIC_EXPLORE=false
if [[ "$MODE" == explore || "$MODE" == llm ]]; then
    export EMBODIED_MAP_BUNDLE="$BUNDLE"
    export EMBODIED_SEMANTIC_WORLD="$BUNDLE.semantic.json"
    export EMBODIED_YOLOE_CONFIG="$YOLOE_CONFIG"
    export EMBODIED_GEOMETRY_READY=0
    export EMBODIED_SAVE_VOXELS=""
    if [[ "$MODE" == explore ]]; then
        [[ "$ODOMETRY_SOURCE" != lio ]] || export EMBODIED_SAVE_VOXELS=global
    fi
fi
if [[ "$MODE" == llm ]]; then
    AUTO_SEMANTIC_EXPLORE=true
    if [[ -f "$BUNDLE/bundle.json" && -f "$BUNDLE/map.pcd" && -f "$BUNDLE/map.yaml" && -f "$BUNDLE/map.pgm" ]]; then
        AUTO_SEMANTIC_EXPLORE=false
        export EMBODIED_GEOMETRY_READY=1
    fi
    if [[ "$AUTO_SEMANTIC_EXPLORE" == true ]]; then
        [[ "$ODOMETRY_SOURCE" != lio ]] || export EMBODIED_SAVE_VOXELS=global
    fi
fi
if [[ "$MODE" == navigation || "$MODE" == navigation_pick || ( "$MODE" == llm && "$AUTO_SEMANTIC_EXPLORE" == false ) ]]; then
    if [[ -f "$BUNDLE/bundle.json" && -f "$BUNDLE/map.pcd" && -f "$BUNDLE/map.yaml" && -f "$BUNDLE/map.pgm" ]]; then
        LOCALIZATION_MODE=localization
        echo "使用已探索地图定位：$BUNDLE"
    else
        echo "缺少完整几何地图包，请先自动探索建图：$BUNDLE" >&2
        exit 1
    fi
fi

ROS_ENV="source /opt/ros/jazzy/setup.bash && source '$INSTALL_DIR/setup.bash'"
if [[ "$ODOMETRY_SOURCE" == vio && -f "$ROOT_DIR/.cache/vslam/env.bash" ]]; then
    ROS_ENV="$ROS_ENV && source '$ROOT_DIR/.cache/vslam/env.bash'"
fi
LOG_DIR="$ROOT_DIR/log/$SIM_BACKEND"
mkdir -p "$LOG_DIR"

launch_window() {
    local title="$1"
    local command="$2"
    if [[ "$HEADLESS" != true ]] && command -v gnome-terminal >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
        # IDE shells can inherit references to a GNOME window that has closed.
        if ! env -u GNOME_TERMINAL_SCREEN -u GNOME_TERMINAL_SERVICE \
            gnome-terminal --title="$title" -- env ROBOT_SIM_TERMINAL=Embodied-RobotSim bash -lc "$command; exec bash"; then
            echo "错误：无法创建终端 [$title]，仿真启动已中止。" >&2
            exit 1
        fi
    else
        local log_name
        log_name="$(tr ' /' '__' <<<"$title")"
        bash -lc "$command" >"$LOG_DIR/$log_name.log" 2>&1 &
        echo "$!" >"$LOG_DIR/$log_name.pid"
        echo "[$title] 后台运行，日志：$LOG_DIR/$log_name.log"
    fi
}

YAW="0.0"
if [[ "$MODE" == "pick" ]]; then
    YAW="0.0"
elif [[ "$MODE" == "explore" ]]; then
    YAW="1.5708"
elif [[ "$MODE" == "llm" ]]; then
    YAW="1.5708"
fi
INITIAL_YAW="${INITIAL_YAW:-$YAW}"
case "$WORLD" in
    office|simple_room|isaac_simple_room|legacy_room|gazebo_simple_room|manipulation_test|small_house|ware_house|obstacle_avoidance_test|empty) ;;
    *) echo "不支持的 Isaac 场景：$WORLD" >&2; exit 2 ;;
esac
ASSETS_PATH="${ISAAC_ASSETS_PATH:-$HOME/isaacsim_assets/6.1}"
if [[ "$SIM_BACKEND" == isaac && ( "$WORLD" == office || "$WORLD" == isaac_simple_room ) ]]; then
    ASSET_FILE="$ASSETS_PATH/Office/office.usd"
    [[ "$WORLD" == isaac_simple_room ]] && ASSET_FILE="$ASSETS_PATH/Simple_Room/simple_room.usd"
    if [[ ! -f "$ASSET_FILE" ]]; then
        echo "缺少本地场景资产：$ASSET_FILE；请用 Isaac python.sh 执行 scripts/assets/download_isaac_environments.py。" >&2
        exit 1
    fi
fi
if [[ "$SIM_BACKEND" == isaac && "$WORLD" != office && "$WORLD" != isaac_simple_room ]]; then
    SOURCE_WORLD="$WORLD"
    [[ "$SOURCE_WORLD" == legacy_room || "$SOURCE_WORLD" == gazebo_simple_room ]] && SOURCE_WORLD=simple_room
    if [[ ! -f "$ASSETS_PATH/GazeboMain/$SOURCE_WORLD.usd" || ! -f "$ASSETS_PATH/GazeboMain/$SOURCE_WORLD.json" ]]; then
        MIGRATION_PYTHON="${ISAAC_SIM_PYTHON:-${ISAAC_SIM_PATH:-$HOME/isaacsim}/python.sh}"
        echo "首次启动，迁移 main 原始场景和贴图：$SOURCE_WORLD"
        "$MIGRATION_PYTHON" "$ROOT_DIR/scripts/assets/migrate_gazebo_scenes.py" --worlds "$SOURCE_WORLD"
    fi
fi
for coordinate in "$INITIAL_X" "$INITIAL_Y" "$INITIAL_YAW"; do
    [[ "$coordinate" =~ ^-?[0-9]+([.][0-9]+)?$ ]] || { echo "初始位姿必须为有限十进制数" >&2; exit 2; }
done
# Values below enter a shell command; reject quoting/control characters in paths.
[[ "$BUNDLE" != *"'"* && "$BUNDLE" != *$'\n'* ]] || { echo '地图路径含非法字符' >&2; exit 2; }

SIM_ARGS="--world $WORLD --x $INITIAL_X --y $INITIAL_Y --yaw $INITIAL_YAW"
[[ "$HEADLESS" == true ]] && SIM_ARGS="$SIM_ARGS --headless"

SIM_RUNTIME=isaac_runtime.sh
SIM_TITLE="Isaac Sim"
if [[ "$SIM_BACKEND" == gazebo ]]; then SIM_RUNTIME=gazebo_runtime.sh; SIM_TITLE="Gazebo"; fi
echo "启动 $SIM_BACKEND：mode=$MODE world=$WORLD localization=$LOCALIZATION_MODE odometry=$ODOMETRY_SOURCE map=$BUNDLE"
if [[ "$SEMANTIC_CLOUD_SOURCE" == fastlio ]]; then
    echo "OctoMap 几何：FAST-LIO 激光点云 /fastlio/cloud_odom；相机仅提供语义标签。"
else
    echo "OctoMap 几何：相机深度点云（depth_source=$DEPTH_SOURCE）。"
fi
launch_window "$SIM_TITLE" "cd '$ROOT_DIR' && ISAAC_SIM_CLEANUP_DONE=1 bash '$ROOT_DIR/scripts/runtime/$SIM_RUNTIME' $SIM_ARGS"
sleep 8
[[ "$SIM_BACKEND" != isaac ]] || launch_window "Isaac Controllers" "$ROS_ENV && ros2 launch x_bot_isaac controllers.launch.py"
launch_window "$ODOMETRY_SOURCE Localization" "$ROS_ENV && ros2 launch x_bot_bringup localization.launch.py odometry_source:=$ODOMETRY_SOURCE mode:=$LOCALIZATION_MODE bundle:='$BUNDLE' initial_x:=$INITIAL_X initial_y:=$INITIAL_Y initial_yaw:=$INITIAL_YAW"
[[ "$HEADLESS" == true ]] || launch_window "RViz" "$ROS_ENV && rviz2 -d '$(ros2 pkg prefix --share x_bot_navigation)/rviz/octomap.rviz' --ros-args -r __node:=rviz2_unified -p use_sim_time:=true"
READY="ros2 run x_bot_control wait_ready"
launch_navigation() {
    local precise_parking=false
    [[ "$MODE" != llm ]] || precise_parking=true
    launch_window "Nav2 $ODOMETRY_SOURCE" "$ROS_ENV && $READY && ros2 launch x_bot_navigation navigation.launch.py odometry_source:=$ODOMETRY_SOURCE precise_parking:=$precise_parking"
}

launch_yoloe() {
    local lidar_semantics=false
    [[ "$ODOMETRY_SOURCE" != lio ]] || lidar_semantics=true
    launch_window "YOLOE Vision" "$ROS_ENV && ros2 run yoloe_infer ros2_trt_infer_text_prompt_multi_node --ros-args -p use_sim_time:=true -p config_path:='$YOLOE_CONFIG' -p depth_topic:='$DEPTH_IMAGE_TOPIC' -p semantic_cloud_source:='$SEMANTIC_CLOUD_SOURCE' -p publish_lidar_semantics:=$lidar_semantics -p enable_pointcloud:=false"
}

if [[ "$USE_LSM" == true ]]; then
    echo "深度来源：LSM 双目推理；OctoMap 点云：$OCTOMAP_CLOUD_TOPIC"
    launch_window "LSM Stereo Depth" "$ROS_ENV && ros2 launch stereo_matching stereo_matching.launch.py config_file:='$LSM_CONFIG_FILE' params_file:='$LSM_PARAMS_FILE' use_sim_time:=true use_rviz:=false"
fi


launch_global_octomap() {
    if [[ "$ODOMETRY_SOURCE" != lio ]]; then
        echo "VIO 模式没有 FAST-LIO 点云，全局 FAST-LIO OctoMap 不启动。"
        return
    fi
    echo "全局 OctoMap：FAST-LIO 几何 + 相机掩码投影的 LiDAR 语义射线；可视化 /global_map/occupied_cells_vis_array"
    launch_window "Global OctoMap" "$ROS_ENV && ros2 launch x_bot_mapping octomap_server.launch.py map_scope:=global semantic_config:='$ROOT_DIR/src/mapping/semantic_voxel_mapping/config/global_fastlio.yaml' palette_file:='$YOLOE_CONFIG' map_file:='$BUNDLE/global_octomap.svm' use_sim_time:=true use_rviz:=false"
}

SEMANTIC_SAVE_ARGS="map_file:='$BUNDLE/local_octomap.svm'"

launch_semantic_world() {
    MODEL_WAIT=""
    if [[ "${LLM_BACKEND:-cloud}" == local ]]; then
        launch_window "Local Qwen Model" "bash '$ROOT_DIR/src/embodied/llm_agent/start_local_model.sh'"
        MODEL_WAIT="python3 '$ROOT_DIR/src/embodied/llm_agent/wait_local_model.py' && "
    fi
    echo "结构化语义地图：$EMBODIED_SEMANTIC_WORLD（同目录另存 .gz；只复用匹配几何地图的语义信息）"
    launch_window "Semantic World" "$ROS_ENV && $MODEL_WAIT'$AGENT_PYTHON' '$ROOT_DIR/src/embodied/semantic_navigation/semantic_world.py'"
}

launch_manipulation_stack() {
    launch_window "MoveIt" "$ROS_ENV && ros2 launch x_bot_moveit_config move_group.launch.py sim_backend:=$SIM_BACKEND use_sim_time:=true"
    launch_yoloe
    sleep 2
    launch_window "Local OctoMap" "$ROS_ENV && ros2 launch x_bot_mapping octomap_server.launch.py map_scope:=local backend:='$OCTOMAP_BACKEND' $SEMANTIC_SAVE_ARGS semantic_config:='$SEMANTIC_MAP_CONFIG' palette_file:='$YOLOE_CONFIG' cloud_topic:='$OCTOMAP_CLOUD_TOPIC' resolution:='$OCTOMAP_RESOLUTION' enabled_on_start:=false use_sim_time:=true use_rviz:=false"
    launch_global_octomap
    echo "局部 OctoMap：默认暂停；到达目标区域开始搜物时清空并开启深度建图，供 MoveIt 避障"
    sleep 2
    if [[ "$MODE" != llm || "${EMBODIED_PICK_BACKEND:-graspnet}" != act ]]; then
        launch_window "GraspNet" "$ROS_ENV && ros2 run graspnet_ros graspnet_node --ros-args -p use_sim_time:=true --params-file '$(ros2 pkg prefix --share graspnet_ros)/config/config.yaml' -p graspnet_node:engine_path:='$(ros2 pkg prefix --share graspnet_ros)/models/graspnet.trt' -p graspnet_node:plugin_path:='$INSTALL_DIR/fps_plugin/lib/libfps_plugin.so' -p graspnet_node:yoloe_config_path:='$YOLOE_CONFIG' -p target_frame:=base_footprint"
    fi
    launch_window "Arm Ctrl" "$ROS_ENV && ros2 run x_bot_manipulation robot_actions --ros-args -p use_sim_time:=true"
}

case "$MODE" in
    explore)
        launch_navigation
        launch_yoloe
        launch_semantic_world
        if [[ "$AUTO_EXPLORE" == true ]]; then
            echo "🧭 自动探索已启用：等待定位和 Nav2 就绪后发送探索目标。"
            EXPLORE_SAVE_ARGS="map_save_service:=/localization/save_map"
            launch_window "Auto Explore" "$ROS_ENV && $READY --nav2 && ros2 launch explore_lite explore.launch.py $EXPLORE_SAVE_ARGS"
        else
            echo "手动建图导航模式：在 RViz 中设置 Nav2 Goal。"
        fi
        launch_global_octomap
        ;;
    navigation)
        launch_navigation
        [[ "$ODOMETRY_SOURCE" != lio ]] || launch_yoloe
        launch_global_octomap
        ;;
    pick)
        launch_manipulation_stack
        sleep 3
        launch_window "Pick and Place Demo" "$ROS_ENV && $READY && ros2 run x_bot_manipulation pick_and_place_demo.py --ros-args --params-file '$(ros2 pkg prefix --share x_bot_manipulation)/config/pick_and_place_demo.yaml' -p use_sim_time:=true -p class_config:='$YOLOE_CONFIG'"
        ;;
    navigation_pick)
        launch_navigation
        sleep 5
        launch_manipulation_stack
        sleep 3
        launch_window "Navigation Pick Demo" "$ROS_ENV && $READY --nav2 && ros2 run x_bot_manipulation navigation_and_pick_demo.py --ros-args -p use_sim_time:=true"
        ;;
    llm)
        launch_semantic_world
        launch_navigation
        sleep 5
        launch_manipulation_stack
        sleep 3
        # Keep chat/status available while localization or Nav2 is starting.
        # Robot actions check readiness inside the Agent; the base safety gate
        # continues to enforce localization health on every velocity command.
        if [[ "$AUTO_SEMANTIC_EXPLORE" == true ]]; then
            echo "缺少几何地图：先自动探索并保存地图包；取物前还需确认目标房间语义区域。"
            launch_window "Auto Explore" "$ROS_ENV && $READY --nav2 && ros2 launch explore_lite explore.launch.py map_save_service:=/localization/save_map return_to_init:=false"
        fi
        launch_window "Qwen3 LLM Agent" "$ROS_ENV && unset ALL_PROXY all_proxy HTTP_PROXY HTTPS_PROXY http_proxy https_proxy && $MODEL_WAIT'$AGENT_PYTHON' '$ROOT_DIR/src/embodied/llm_agent/agent_server.py' --ros-args -p use_sim_time:=true"
        launch_window "Web UI" "cd '$ROOT_DIR' && $ROS_ENV && bash '$ROOT_DIR/scripts/runtime/web_ui.sh'"
        if [[ "$HEADLESS" != true ]] && command -v xdg-open >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
            sleep 3
            xdg-open http://localhost:8888 >/dev/null 2>&1 &
        fi
        ;;
esac

echo "$SIM_TITLE 启动命令已发出；请检查 /localization/ready 和各窗口/日志。"
echo "停止所有服务：bash '$ROOT_DIR/stop_robot_sim.sh'"
