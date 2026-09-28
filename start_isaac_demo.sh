#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-}"
shift || true

case "$MODE" in
    explore|navigation|pick|navigation_pick|llm) ;;
    *)
        echo "用法：$0 {explore|navigation|pick|navigation_pick|llm} [--headless] [--build] [地图.yaml]" >&2
        exit 2
        ;;
esac

HEADLESS=false
BUILD=false
MAP_FILE="$ROOT_DIR/src/x_bot/maps/isaac_simple_room.yaml"
while (($#)); do
    case "$1" in
        --sim)
            shift
            [[ "${1:-}" == "isaac" ]] || { echo "只支持 --sim isaac" >&2; exit 2; }
            ;;
        --sim=isaac) ;;
        --headless) HEADLESS=true ;;
        --build) BUILD=true ;;
        *.yaml) MAP_FILE="$1" ;;
        *) echo "警告：忽略未知参数：$1" >&2 ;;
    esac
    shift
done

cd "$ROOT_DIR"
if [[ "$MAP_FILE" != /* ]]; then
    MAP_FILE="$ROOT_DIR/$MAP_FILE"
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
    if [[ ! -f "$MAP_FILE" ]]; then
        echo "错误：地图不存在：$MAP_FILE" >&2
        exit 1
    fi
fi

bash "$ROOT_DIR/stop_robot_sim.sh" || true
sleep 2

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
PUBLISH_ODOM_TF=false
if [[ "$MODE" == "pick" ]]; then
    WORLD="manipulation_test"
elif [[ "$MODE" == "explore" ]]; then
    YAW="1.5708"
elif [[ "$MODE" == "llm" ]]; then
    YAW="1.5708"
    PUBLISH_ODOM_TF=true
else
    PUBLISH_ODOM_TF=true
fi

SIM_ARGS="--world $WORLD --x 0.0 --y 0.0 --yaw $YAW"
[[ "$HEADLESS" == true ]] && SIM_ARGS="$SIM_ARGS --headless"
[[ "$PUBLISH_ODOM_TF" == true ]] && SIM_ARGS="$SIM_ARGS --publish-odom-tf"

echo "启动 Isaac Sim 6.1：mode=$MODE world=$WORLD"
launch_window "Isaac Sim" "cd '$ROOT_DIR' && bash '$ROOT_DIR/start_isaac_sim.sh' $SIM_ARGS"
sleep 8
launch_window "Isaac Controllers" "$ROS_ENV && ros2 launch x_bot isaac_controllers.launch.py"

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
        launch_window "Cartographer SLAM" "$ROS_ENV && ros2 launch x_bot cartographer.launch.py"
        sleep 3
        launch_window "Nav2 Cartographer" "$ROS_ENV && ros2 launch x_bot nav2_cartographer.launch.py"
        launch_yoloe
        launch_window "Auto Explore" "$ROS_ENV && ros2 launch explore_lite explore.launch.py"
        launch_window "OctoMap" "$ROS_ENV && ros2 launch x_bot octomap_server.launch.py"
        ;;
    navigation)
        launch_window "Nav2 Static Map" "$ROS_ENV && ros2 launch x_bot nav2.launch.py use_slam:=false map_file:='$MAP_FILE'"
        ;;
    pick)
        launch_window "Cartographer SLAM" "$ROS_ENV && ros2 launch x_bot cartographer.launch.py"
        launch_manipulation_stack
        sleep 3
        launch_window "Pick and Place Demo" "$ROS_ENV && python3 '$ROOT_DIR/src/x_bot/scripts/pick_and_place_demo.py' --ros-args -p use_sim_time:=true -p prompts:=\"['coke', 'book', 'cup']\""
        ;;
    navigation_pick)
        launch_window "Nav2 Static Map" "$ROS_ENV && ros2 launch x_bot nav2.launch.py use_slam:=false map_file:='$MAP_FILE'"
        sleep 5
        launch_manipulation_stack
        sleep 3
        launch_window "Navigation Pick Demo" "$ROS_ENV && python3 '$ROOT_DIR/src/x_bot/scripts/navigation_and_pick_demo.py' --ros-args -p use_sim_time:=true"
        ;;
    llm)
        launch_window "Nav2 Static Map" "$ROS_ENV && ros2 launch x_bot nav2.launch.py use_slam:=false map_file:='$MAP_FILE'"
        sleep 5
        launch_manipulation_stack
        sleep 3
        launch_window "Qwen3 LLM Agent" "$ROS_ENV && unset ALL_PROXY all_proxy HTTP_PROXY HTTPS_PROXY http_proxy https_proxy && python3 '$ROOT_DIR/llm_agent/agent_server.py'"
        launch_window "Web UI" "cd '$ROOT_DIR' && $ROS_ENV && bash '$ROOT_DIR/start_web_ui.sh'"
        ;;
esac

echo "Isaac Sim 模式已启动。停止：bash $ROOT_DIR/stop_robot_sim.sh"
