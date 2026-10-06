#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-explore}"
if (($#)); then shift; fi

case "$MODE" in
    explore|navigation|pick|navigation_pick|llm) ;;
    *)
        echo "用法：$0 {explore|navigation|pick|navigation_pick|llm} [--world 场景] [--headless] [--build] [--no-explore] [--bundle 地图目录] [--initial-x X --initial-y Y --initial-yaw RAD]" >&2
        exit 2
        ;;
esac

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
YOLOE_CONFIG="$ROOT_DIR/src/yoloe_infer/configs/config.yaml"
[[ "$MODE" == pick ]] && YOLOE_CONFIG="$ROOT_DIR/src/yoloe_infer/configs/manipulation.yaml"
DEPTH_SOURCE=isaac
[[ "$MODE" == explore ]] && DEPTH_SOURCE=lsm
LSM_CONFIG_FILE="$ROOT_DIR/src/LSM_depth_infer/config/config.yaml"
LSM_PARAMS_FILE="$ROOT_DIR/src/LSM_depth_infer/config/isaac_params.yaml"
DEPTH_IMAGE_TOPIC=""
OCTOMAP_CLOUD_TOPIC=""
OCTOMAP_BACKEND=semantic_cuda
OCTOMAP_RESOLUTION=0.10
[[ "$MODE" != explore ]] && OCTOMAP_RESOLUTION=0.02
SEMANTIC_MAP_CONFIG="$ROOT_DIR/src/semantic_voxel_mapping/config/map.yaml"
[[ "$MODE" != explore ]] && SEMANTIC_MAP_CONFIG="$ROOT_DIR/src/semantic_voxel_mapping/config/manipulation.yaml"
SEMANTIC_CLOUD_SOURCE=depth
[[ "$MODE" == explore ]] && SEMANTIC_CLOUD_SOURCE=fastlio
while (($#)); do
    case "$1" in
        --headless) HEADLESS=true ;;
        --build) BUILD=true ;;
        --no-explore) AUTO_EXPLORE=false ;;
        --bundle) shift; BUNDLE="${1:?--bundle 缺少目录}" ;;
        --world) shift; WORLD="${1:?--world 缺少场景}" ;;
        --initial-x) shift; INITIAL_X="${1:?缺少 x}" ;;
        --initial-y) shift; INITIAL_Y="${1:?缺少 y}" ;;
        --initial-yaw) shift; INITIAL_YAW="${1:?缺少 yaw}" ;;
        --yoloe-config) shift; YOLOE_CONFIG="${1:?缺少 YOLOE 配置}" ;;
        --depth-source) shift; DEPTH_SOURCE="${1:?缺少深度来源 lsm/isaac}" ;;
        --lsm-config) shift; LSM_CONFIG_FILE="${1:?缺少 LSM 配置文件}" ;;
        --lsm-params) shift; LSM_PARAMS_FILE="${1:?缺少 LSM ROS 参数文件}" ;;
        --depth-image-topic) shift; DEPTH_IMAGE_TOPIC="${1:?缺少深度图话题}" ;;
        --octomap-cloud-topic) shift; OCTOMAP_CLOUD_TOPIC="${1:?缺少 OctoMap 点云话题}" ;;
        --octomap-backend) shift; OCTOMAP_BACKEND="${1:?缺少 semantic_cuda}" ;;
        --octomap-resolution) shift; OCTOMAP_RESOLUTION="${1:?缺少 OctoMap 分辨率（米）}" ;;
        --semantic-map-config) shift; SEMANTIC_MAP_CONFIG="${1:?缺少语义地图配置}" ;;
        --semantic-cloud-source) shift; SEMANTIC_CLOUD_SOURCE="${1:?缺少语义点云来源 fastlio/depth}" ;;
        *.yaml) echo "Isaac 定位需要配套 PCD 地图目录，请用 --bundle；不能只用旧二维 YAML 地图。" >&2; exit 2 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
    shift
done

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
    isaac)
        DEPTH_IMAGE_TOPIC="${DEPTH_IMAGE_TOPIC:-/x_bot/camera_left/depth/image_raw}"
        OCTOMAP_CLOUD_TOPIC="${OCTOMAP_CLOUD_TOPIC:-/yoloe_multi_text_prompt/pointcloud_semantic}"
        ;;
    *) echo "深度来源必须为 lsm 或 isaac" >&2; exit 2 ;;
esac
for topic in "$DEPTH_IMAGE_TOPIC" "$OCTOMAP_CLOUD_TOPIC"; do
    [[ "$topic" =~ ^/[a-zA-Z0-9_/]+$ ]] || { echo "无效 ROS 话题：$topic" >&2; exit 2; }
done

[[ -f "$YOLOE_CONFIG" && "$YOLOE_CONFIG" != *"'"* && "$YOLOE_CONFIG" != *$'\n'* ]] || { echo "YOLOE 配置无效：$YOLOE_CONFIG" >&2; exit 2; }

cd "$ROOT_DIR"
# Keep large camera/cloud writes off lifecycle/service callback threads.
export RMW_FASTRTPS_PUBLICATION_MODE="${RMW_FASTRTPS_PUBLICATION_MODE:-ASYNCHRONOUS}"
echo "🛑 正在清理残留进程..."
# Load ROS for the cleanup script even when launched from a fresh shell.
set +u
source /opt/ros/jazzy/setup.bash
set -u
bash "$ROOT_DIR/stop_robot_sim.sh"
sleep 2

if [[ "$BUNDLE" != /* ]]; then
    BUNDLE="$ROOT_DIR/$BUNDLE"
fi
if [[ "$BUILD" == true ]]; then
    # ROS setup scripts are not compatible with nounset.
    set +u
    source /opt/ros/jazzy/setup.bash
    set -u
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
fi
if [[ ! -f "$ROOT_DIR/install/setup.bash" ]]; then
    echo "错误：未找到 install/setup.bash；请先执行 colcon build --symlink-install。" >&2
    exit 1
fi
# These CMake PROGRAMS have explicit permissions and are installed as copies,
# even with --symlink-install. Refresh stale entrypoints before launching them.
if [[ "$BUILD" == false ]]; then
    for entrypoint in bridge adapter map_builder safety_gate wait_ready; do
        source_entry="$ROOT_DIR/src/x_bot_localization/scripts/$entrypoint"
        installed_entry="$ROOT_DIR/install/x_bot_localization/lib/x_bot_localization/$entrypoint"
        if [[ -f "$source_entry" ]] && ! cmp -s "$source_entry" "$installed_entry"; then
            echo "更新定位启动脚本：x_bot_localization"
            colcon build --packages-select x_bot_localization --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
            break
        fi
    done
fi
set +u
source "$ROOT_DIR/install/setup.bash"
set -u
for package in x_bot x_bot_localization livox_ros_driver2 fast_lio; do
    if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
        echo "错误：未找到 Isaac 运行依赖 $package；请安装依赖并执行 colcon build --symlink-install，或使用 --build。" >&2
        exit 1
    fi
done
if [[ "$USE_LSM" == true ]]; then
    ros2 pkg prefix stereo_matching >/dev/null 2>&1 || { echo "错误：缺少 stereo_matching，请先构建 LSM_depth_infer 或使用 --build。" >&2; exit 1; }
fi
if [[ "$OCTOMAP_BACKEND" == semantic_cuda ]]; then
    ros2 pkg prefix semantic_voxel_mapping >/dev/null 2>&1 || { echo "错误：缺少 semantic_voxel_mapping，请使用 --build。" >&2; exit 1; }
fi
if [[ "$MODE" == explore && "$AUTO_EXPLORE" == true ]]; then
    for package in explore_lite nav2_bringup nav2_regulated_pure_pursuit_controller nav2_rotation_shim_controller; do
        ros2 pkg prefix "$package" >/dev/null 2>&1 || { echo "错误：缺少自动探索依赖 $package" >&2; exit 1; }
    done
fi
# 与 main 的启动流程一致：探索/抓取建图，导航类要求已有地图。
LOCALIZATION_MODE=mapping
if [[ "$MODE" == navigation || "$MODE" == navigation_pick || "$MODE" == llm ]]; then
    if [[ -f "$BUNDLE/bundle.json" && -f "$BUNDLE/map.pcd" && -f "$BUNDLE/map.yaml" && -f "$BUNDLE/map.pgm" ]]; then
        LOCALIZATION_MODE=localization
        echo "使用默认地图定位：$BUNDLE"
    else
        echo "错误：缺少完整地图：$BUNDLE（需要 bundle.json、map.pcd、map.yaml、map.pgm）。请先在同一场景建图并保存地图。" >&2
        exit 1
    fi
fi

ROS_ENV="source /opt/ros/jazzy/setup.bash && source '$ROOT_DIR/install/setup.bash'"
LOG_DIR="$ROOT_DIR/log/isaac_sim"
mkdir -p "$LOG_DIR"

launch_window() {
    local title="$1"
    local command="$2"
    if command -v gnome-terminal >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
        # IDE shells can inherit references to a GNOME window that has closed.
        if ! env -u GNOME_TERMINAL_SCREEN -u GNOME_TERMINAL_SERVICE \
            gnome-terminal --title="$title" -- env ROBOT_SIM_TERMINAL=Embodied-RobotSim bash -lc "$command; exec bash"; then
            echo "错误：无法创建终端 [$title]，Isaac 启动已中止。" >&2
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
if [[ "$WORLD" == office || "$WORLD" == isaac_simple_room ]]; then
    ASSET_FILE="$ASSETS_PATH/Office/office.usd"
    [[ "$WORLD" == isaac_simple_room ]] && ASSET_FILE="$ASSETS_PATH/Simple_Room/simple_room.usd"
    if [[ ! -f "$ASSET_FILE" ]]; then
        echo "缺少本地场景资产：$ASSET_FILE；请用 Isaac python.sh 执行 scripts/assets/download_isaac_environments.py。" >&2
        exit 1
    fi
fi
if [[ "$WORLD" != office && "$WORLD" != isaac_simple_room ]]; then
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

echo "启动 Isaac Sim 6.1：mode=$MODE world=$WORLD localization=$LOCALIZATION_MODE map=$BUNDLE"
launch_window "Isaac Sim" "cd '$ROOT_DIR' && ISAAC_SIM_CLEANUP_DONE=1 bash '$ROOT_DIR/scripts/isaac_runtime.sh' $SIM_ARGS"
sleep 8
launch_window "Isaac Controllers" "$ROS_ENV && ros2 launch x_bot isaac_controllers.launch.py"
launch_window "FAST-LIO Localization" "$ROS_ENV && ros2 launch x_bot_localization localization.launch.py mode:=$LOCALIZATION_MODE bundle:='$BUNDLE' initial_x:=$INITIAL_X initial_y:=$INITIAL_Y initial_yaw:=$INITIAL_YAW"
launch_window "RViz" "$ROS_ENV && rviz2 -d '$ROOT_DIR/src/x_bot/rviz/octomap.rviz' --ros-args -r __node:=rviz2_unified -p use_sim_time:=true"
READY="ros2 run x_bot_localization wait_ready"
launch_navigation() {
    launch_window "Nav2 FAST-LIO" "$ROS_ENV && $READY && ros2 launch x_bot_localization navigation.launch.py"
}

launch_yoloe() {
    launch_window "YOLOE Vision" "$ROS_ENV && ros2 run yoloe_infer ros2_trt_infer_text_prompt_multi_node --ros-args -p use_sim_time:=true -p config_path:='$YOLOE_CONFIG' -p depth_topic:='$DEPTH_IMAGE_TOPIC' -p semantic_cloud_source:='$SEMANTIC_CLOUD_SOURCE'"
}

if [[ "$USE_LSM" == true ]]; then
    echo "深度来源：LSM 双目推理；OctoMap 点云：$OCTOMAP_CLOUD_TOPIC"
    launch_window "LSM Stereo Depth" "$ROS_ENV && ros2 launch stereo_matching stereo_matching.launch.py config_file:='$LSM_CONFIG_FILE' params_file:='$LSM_PARAMS_FILE' use_sim_time:=true use_rviz:=false"
fi

launch_manipulation_stack() {
    launch_window "MoveIt" "$ROS_ENV && ros2 launch x_bot move_group.launch.py use_sim_time:=true use_rviz:=false"
    launch_yoloe
    sleep 2
    launch_window "OctoMap" "$ROS_ENV && ros2 launch x_bot octomap_server.launch.py backend:='$OCTOMAP_BACKEND' semantic_config:='$SEMANTIC_MAP_CONFIG' palette_file:='$YOLOE_CONFIG' cloud_topic:='$OCTOMAP_CLOUD_TOPIC' resolution:='$OCTOMAP_RESOLUTION' use_sim_time:=true use_rviz:=false"
    sleep 2
    launch_window "GraspNet" "$ROS_ENV && ros2 run graspnet_ros graspnet_node --ros-args -p use_sim_time:=true --params-file '$ROOT_DIR/src/graspnet_infer/graspnet_ros/config/config.yaml' -p engine_path:='$ROOT_DIR/src/graspnet_infer/graspnet.trt' -p plugin_path:='$ROOT_DIR/src/graspnet_infer/tensorrt_plugins/build/libfps_plugin.so' -p yoloe_config_path:='$YOLOE_CONFIG' -p target_frame:=base_footprint"
    launch_window "Arm Ctrl" "$ROS_ENV && ros2 run x_bot robot_actions --ros-args -p use_sim_time:=true"
}

case "$MODE" in
    explore)
        launch_navigation
        launch_yoloe
        if [[ "$AUTO_EXPLORE" == true ]]; then
            echo "🧭 自动探索已启用：等待定位和 Nav2 就绪后发送探索目标。"
            launch_window "Auto Explore" "$ROS_ENV && $READY --nav2 && ros2 launch explore_lite explore.launch.py"
        else
            echo "手动建图导航模式：在 RViz 中设置 Nav2 Goal。"
        fi
        launch_window "OctoMap" "$ROS_ENV && ros2 launch x_bot octomap_server.launch.py backend:='$OCTOMAP_BACKEND' semantic_config:='$SEMANTIC_MAP_CONFIG' palette_file:='$YOLOE_CONFIG' cloud_topic:='$OCTOMAP_CLOUD_TOPIC' use_rviz:=false"
        ;;
    navigation)
        launch_navigation
        ;;
    pick)
        launch_manipulation_stack
        sleep 3
        launch_window "Pick and Place Demo" "$ROS_ENV && $READY && python3 '$ROOT_DIR/src/x_bot/scripts/pick_and_place_demo.py' --ros-args --params-file '$ROOT_DIR/src/x_bot/config/pick_and_place_demo.yaml' -p use_sim_time:=true -p class_config:='$YOLOE_CONFIG'"
        ;;
    navigation_pick)
        launch_navigation
        sleep 5
        launch_manipulation_stack
        sleep 3
        launch_window "Navigation Pick Demo" "$ROS_ENV && $READY --nav2 && python3 '$ROOT_DIR/src/x_bot/scripts/navigation_and_pick_demo.py' --ros-args -p use_sim_time:=true"
        ;;
    llm)
        launch_navigation
        sleep 5
        launch_manipulation_stack
        sleep 3
        launch_window "Qwen3 LLM Agent" "$ROS_ENV && $READY --nav2 && unset ALL_PROXY all_proxy HTTP_PROXY HTTPS_PROXY http_proxy https_proxy && python3 '$ROOT_DIR/llm_agent/agent_server.py'"
        launch_window "Web UI" "cd '$ROOT_DIR' && $ROS_ENV && bash '$ROOT_DIR/start_web_ui.sh'"
        if command -v xdg-open >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
            sleep 3
            xdg-open http://localhost:8888 >/dev/null 2>&1 &
        fi
        ;;
esac

echo "Isaac Sim 启动命令已发出；请检查 /localization/ready 和各窗口/日志。"
echo "停止所有服务：bash '$ROOT_DIR/stop_robot_sim.sh'"
