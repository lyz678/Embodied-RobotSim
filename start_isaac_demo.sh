#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-}"
shift || true

case "$MODE" in
    explore|navigation|pick|navigation_pick|llm) ;;
    *)
        echo "用法：$0 {explore|navigation|pick|navigation_pick|llm} [--headless] [--build] [--bundle 地图目录] [--initial-x X --initial-y Y --initial-yaw RAD]" >&2
        exit 2
        ;;
esac

HEADLESS=false
BUILD=false
BUNDLE="$ROOT_DIR/maps/isaac_session"
INITIAL_X=0.0
INITIAL_Y=0.0
INITIAL_YAW=""
while (($#)); do
    case "$1" in
        --sim)
            shift
            [[ "${1:-}" == "isaac" ]] || { echo "只支持 --sim isaac" >&2; exit 2; }
            ;;
        --sim=isaac) ;;
        --headless) HEADLESS=true ;;
        --build) BUILD=true ;;
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
if [[ "$BUNDLE" != /* ]]; then
    BUNDLE="$ROOT_DIR/$BUNDLE"
fi
if [[ "$BUILD" == true ]]; then
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
fi
if [[ ! -f "$ROOT_DIR/install/setup.bash" ]]; then
    echo "错误：未找到 install/setup.bash；请先执行 colcon build --symlink-install。" >&2
    exit 1
fi
if [[ "$MODE" == "navigation" || "$MODE" == "navigation_pick" || "$MODE" == "llm" ]]; then
    if [[ ! -f "$BUNDLE/bundle.json" || ! -f "$BUNDLE/map.pcd" || ! -f "$BUNDLE/map.yaml" ]]; then
        echo "错误：缺少配套地图目录：$BUNDLE。先 explore 建图并调用 /localization/save_map。" >&2
        exit 1
    fi
fi

# Do not kill unrelated user processes or erase previous logs automatically.

ROS_ENV="source /opt/ros/jazzy/setup.bash && source '$ROOT_DIR/install/setup.bash'"
LOG_DIR="$ROOT_DIR/log/isaac_sim"
mkdir -p "$LOG_DIR"

launch_window() {
    local title="$1"
    local command="$2"
    if command -v gnome-terminal >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
        gnome-terminal --title="$title" -- bash -lc "$command; exec bash"
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

SIM_ARGS="--world $WORLD --x 0.0 --y 0.0 --yaw $YAW"
[[ "$HEADLESS" == true ]] && SIM_ARGS="$SIM_ARGS --headless"

echo "启动 Isaac Sim 6.1：mode=$MODE world=$WORLD"
launch_window "Isaac Sim" "cd '$ROOT_DIR' && bash '$ROOT_DIR/start_isaac_sim.sh' $SIM_ARGS"
sleep 8
launch_window "Isaac Controllers" "$ROS_ENV && ros2 launch x_bot isaac_controllers.launch.py"
LOCALIZATION_MODE=localization
[[ "$MODE" == explore || "$MODE" == pick ]] && LOCALIZATION_MODE=mapping
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
        launch_window "Auto Explore" "$ROS_ENV && $READY && ros2 launch explore_lite explore.launch.py"
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
        launch_window "Navigation Pick Demo" "$ROS_ENV && $READY && python3 '$ROOT_DIR/src/x_bot/scripts/navigation_and_pick_demo.py' --ros-args -p use_sim_time:=true"
        ;;
    llm)
        launch_navigation
        sleep 5
        launch_manipulation_stack
        sleep 3
        launch_window "Qwen3 LLM Agent" "$ROS_ENV && $READY && unset ALL_PROXY all_proxy HTTP_PROXY HTTPS_PROXY http_proxy https_proxy && python3 '$ROOT_DIR/llm_agent/agent_server.py'"
        launch_window "Web UI" "cd '$ROOT_DIR' && $ROS_ENV && bash '$ROOT_DIR/start_web_ui.sh'"
        ;;
esac

echo "Isaac Sim 启动命令已发出；请检查 /localization/ready 和各窗口/日志。"
echo "停止时优先结束本次会话；stop_robot_sim.sh 会清理所有 ROS 会话并删除日志，谨慎使用。"
