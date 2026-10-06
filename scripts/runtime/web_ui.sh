#!/usr/bin/env bash
# Web transport only; dependency installation belongs in scripts/setup/.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
INSTALL_DIR="${ROBOT_INSTALL_PREFIX:-$ROOT_DIR/install}"
set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source "$INSTALL_DIR/setup.bash"
set -u
for package in rosbridge_server web_video_server; do
    ros2 pkg prefix "$package" >/dev/null 2>&1 || {
        echo "缺少 $package，请执行 scripts/setup/setup_embodied_dependencies.sh。" >&2
        exit 1
    }
done
LOG_DIR="$ROOT_DIR/log/web"
mkdir -p "$LOG_DIR"
SERVICE_PIDS=()
cleanup() {
    trap - EXIT INT TERM
    for pid in "${SERVICE_PIDS[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
    for pid in "${SERVICE_PIDS[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 address:=0.0.0.0 \
    unregister_timeout:=10.0 send_action_goals_in_new_thread:=true >"$LOG_DIR/rosbridge.log" 2>&1 &
SERVICE_PIDS+=("$!")
ros2 run web_video_server web_video_server --ros-args -p port:=8080 -p address:=0.0.0.0 \
    -p default_stream_type:=mjpeg >"$LOG_DIR/video.log" 2>&1 &
SERVICE_PIDS+=("$!")
python3 -m http.server 8888 --directory "$ROOT_DIR/src/embodied/web_ui" >"$LOG_DIR/http.log" 2>&1 &
SERVICE_PIDS+=("$!")
echo "Web UI: http://localhost:8888；ROS: ws://localhost:9090；相机: http://localhost:8080；Agent: ws://localhost:8889/ws/chat"
wait -n "${SERVICE_PIDS[@]}"
