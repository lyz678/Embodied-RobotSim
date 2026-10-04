#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-explore}"
if (($#)); then shift; fi

case "$MODE" in
    explore|navigation|pick|navigation_pick|llm) ;;
    *)
        echo "用法：$0 {explore|navigation|pick|navigation_pick|llm} [--headless] [--build] [--no-explore] [--bundle 地图目录] [--initial-x X --initial-y Y --initial-yaw RAD]" >&2
        exit 2
        ;;
esac

HEADLESS=false
BUILD=false
AUTO_EXPLORE=true
BUNDLE="$ROOT_DIR/maps/room_a"
[[ "$MODE" == pick ]] && BUNDLE="$ROOT_DIR/maps/manipulation_test"
INITIAL_X=0.0
INITIAL_Y=0.0
INITIAL_YAW=""
while (($#)); do
    case "$1" in
        --headless) HEADLESS=true ;;
        --build) BUILD=true ;;
        --no-explore) AUTO_EXPLORE=false ;;
        --bundle) shift; BUNDLE="${1:?--bundle 缺少目录}" ;;
        --initial-x) shift; INITIAL_X="${1:?缺少 x}" ;;
        --initial-y) shift; INITIAL_Y="${1:?缺少 y}" ;;
        --initial-yaw) shift; INITIAL_YAW="${1:?缺少 yaw}" ;;
        *.yaml) echo "Isaac 定位需要配套 PCD 地图目录，请用 --bundle；不能只用旧二维 YAML 地图。" >&2; exit 2 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
    shift
done

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
if [[ "$MODE" == explore && "$AUTO_EXPLORE" == true ]]; then
    for package in explore_lite nav2_bringup nav2_regulated_pure_pursuit_controller; do
        ros2 pkg prefix "$package" >/dev/null 2>&1 || { echo "错误：缺少自动探索依赖 $package" >&2; exit 1; }
    done
fi
# 固定默认策略：探索/抓取实时建图；导航类优先使用地图，没有地图也能启动。
LOCALIZATION_MODE=mapping
if [[ "$MODE" == navigation || "$MODE" == navigation_pick || "$MODE" == llm ]]; then
    if [[ -f "$BUNDLE/bundle.json" && -f "$BUNDLE/map.pcd" && -f "$BUNDLE/map.yaml" && -f "$BUNDLE/map.pgm" ]]; then
        LOCALIZATION_MODE=localization
        echo "使用默认地图定位：$BUNDLE"
    else
        echo "默认地图尚未保存，自动使用 FAST-LIO 实时建图：$BUNDLE"
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

WORLD="simple_room"
YAW="0.0"
if [[ "$MODE" == "pick" ]]; then
    WORLD="manipulation_test"
elif [[ "$MODE" == "explore" ]]; then
    YAW="1.5708"
elif [[ "$MODE" == "llm" ]]; then
    YAW="1.5708"
fi
INITIAL_YAW="${INITIAL_YAW:-$YAW}"
for coordinate in "$INITIAL_X" "$INITIAL_Y" "$INITIAL_YAW"; do
    [[ "$coordinate" =~ ^-?[0-9]+([.][0-9]+)?$ ]] || { echo "初始位姿必须为有限十进制数" >&2; exit 2; }
done
# Values below enter a shell command; reject quoting/control characters in paths.
[[ "$BUNDLE" != *"'"* && "$BUNDLE" != *$'\n'* ]] || { echo '地图路径含非法字符' >&2; exit 2; }

SIM_ARGS="--world $WORLD --x $INITIAL_X --y $INITIAL_Y --yaw $INITIAL_YAW"
[[ "$HEADLESS" == true ]] && SIM_ARGS="$SIM_ARGS --headless"

echo "启动 Isaac Sim 6.1：mode=$MODE world=$WORLD localization=$LOCALIZATION_MODE map=$BUNDLE"
launch_window "Isaac Sim" "cd '$ROOT_DIR' && ISAAC_SIM_CLEANUP_DONE=1 bash '$ROOT_DIR/start_isaac_sim.sh' $SIM_ARGS"
sleep 8
launch_window "Isaac Controllers" "$ROS_ENV && ros2 launch x_bot isaac_controllers.launch.py"
launch_window "FAST-LIO Localization" "$ROS_ENV && ros2 launch x_bot_localization localization.launch.py mode:=$LOCALIZATION_MODE bundle:='$BUNDLE' initial_x:=$INITIAL_X initial_y:=$INITIAL_Y initial_yaw:=$INITIAL_YAW"
READY="ros2 run x_bot_localization wait_ready"
launch_navigation() {
    launch_window "Nav2 FAST-LIO" "$ROS_ENV && $READY && ros2 launch x_bot_localization navigation.launch.py"
}

launch_yoloe() {
    launch_window "YOLOE Vision" "$ROS_ENV && ros2 run yoloe_infer ros2_trt_infer_text_prompt_multi_node --ros-args -p use_sim_time:=true -p config_path:='$ROOT_DIR/src/yoloe_infer/configs/config.yaml'"
}

launch_manipulation_stack() {
    launch_window "MoveIt" "$ROS_ENV && ros2 launch x_bot move_group.launch.py use_sim_time:=true use_rviz:=true"
    launch_yoloe
    sleep 2
    launch_window "OctoMap" "$ROS_ENV && ros2 launch x_bot octomap_server.launch.py use_sim_time:=true use_rviz:=true"
    launch_window "GraspNet" "$ROS_ENV && ros2 run graspnet_ros graspnet_node --ros-args -p use_sim_time:=true --params-file '$ROOT_DIR/src/graspnet_infer/graspnet_ros/config/config.yaml' -p engine_path:='$ROOT_DIR/src/graspnet_infer/graspnet.trt' -p plugin_path:='$ROOT_DIR/src/graspnet_infer/tensorrt_plugins/build/libfps_plugin.so'"
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
        launch_window "OctoMap" "$ROS_ENV && ros2 launch x_bot octomap_server.launch.py"
        ;;
    navigation)
        launch_navigation
        ;;
    pick)
        launch_manipulation_stack
        sleep 3
        launch_window "Pick and Place Demo" "$ROS_ENV && $READY && python3 '$ROOT_DIR/src/x_bot/scripts/pick_and_place_demo.py' --ros-args -p use_sim_time:=true -p prompts:=\"['coke', 'book', 'cup']\""
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
        ;;
esac

echo "Isaac Sim 启动命令已发出；请检查 /localization/ready 和各窗口/日志。"
echo "停止所有服务：bash '$ROOT_DIR/stop_robot_sim.sh'"
