#!/bin/bash

# =====================================================
#  RobotSim Web UI 启动脚本
# =====================================================
#  启动以下服务：
#   1. rosbridge_server  (ws://localhost:9090)  - ROS <-> WebSocket 桥
#   2. web_video_server  (http://localhost:8080) - 相机 MJPEG 流
#   3. Python HTTP 服务器 (http://localhost:8888) - 前端页面托管
#
#  浏览器访问：http://localhost:8888
# =====================================================

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"

# 颜色
RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[1;33m'; CYN='\033[0;36m'; NC='\033[0m'
info()  { echo -e "${CYN}[INFO]${NC}  $*"; }
ok()    { echo -e "${GRN}[OK]${NC}    $*"; }
warn()  { echo -e "${YLW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; }

echo -e "${CYN}=========================================="
echo -e "   🤖  RobotSim Web UI"
echo -e "===========================================${NC}"

# ─── 检查 ROS 环境 ─────────────────────────────────────
if [ -z "$ROS_DISTRO" ]; then
  error "ROS 环境未初始化，请先 source /opt/ros/<distro>/setup.bash"
  exit 1
fi
info "ROS_DISTRO = $ROS_DISTRO"

# source ROS
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

# source 工作空间（如果存在）
WS_SETUP="${SCRIPT_DIR}/../install/setup.bash"
if [ -f "$WS_SETUP" ]; then
  source "$WS_SETUP"
  info "已加载工作空间: $WS_SETUP"
fi

# ─── 安装依赖 ──────────────────────────────────────────
install_if_missing() {
  local pkg="$1"
  if ! ros2 pkg list 2>/dev/null | grep -q "^${pkg}$"; then
    warn "${pkg} 未找到，尝试安装..."
    sudo apt-get install -y "ros-${ROS_DISTRO}-${pkg//_/-}" 2>/dev/null \
      || warn "安装 ${pkg} 失败，请手动安装"
  fi
}

info "检查依赖包..."
install_if_missing "rosbridge_server"
install_if_missing "web_video_server"

# ─── 清理旧进程 ────────────────────────────────────────
cleanup() {
  info "清理后台进程..."
  kill $PID_BRIDGE $PID_VIDEO $PID_HTTP 2>/dev/null || true
  wait $PID_BRIDGE $PID_VIDEO $PID_HTTP 2>/dev/null || true
  info "已退出"
  exit 0
}
trap cleanup INT TERM EXIT

# 杀死残留进程
pkill -f "rosbridge_websocket"    2>/dev/null || true
pkill -f "web_video_server"       2>/dev/null || true
pkill -f "http.server.*8888"      2>/dev/null || true
sleep 1

# ─── 启动 rosbridge ────────────────────────────────────
info "启动 rosbridge_server (ws://localhost:9090)..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  port:=9090 \
  address:=0.0.0.0 \
  unregister_timeout:=10.0 \
  send_action_goals_in_new_thread:=true \
  > /tmp/rosbridge.log 2>&1 &
PID_BRIDGE=$!
sleep 2

if kill -0 $PID_BRIDGE 2>/dev/null; then
  ok "rosbridge_server 已启动 (PID=$PID_BRIDGE)"
else
  error "rosbridge_server 启动失败，查看 /tmp/rosbridge.log"
fi

# ─── 启动 web_video_server ─────────────────────────────
info "启动 web_video_server (http://localhost:8080)..."
ros2 run web_video_server web_video_server \
  --ros-args \
  -p port:=8080 \
  -p address:=0.0.0.0 \
  -p default_stream_type:=mjpeg \
  > /tmp/web_video.log 2>&1 &
PID_VIDEO=$!
sleep 2

if kill -0 $PID_VIDEO 2>/dev/null; then
  ok "web_video_server 已启动 (PID=$PID_VIDEO)"
else
  warn "web_video_server 启动失败（将回退到 rosbridge 模式），查看 /tmp/web_video.log"
  PID_VIDEO=0
fi

# ─── 启动前端 HTTP 服务器 ─────────────────────────────
WEB_PORT=8888
info "启动前端 HTTP 服务器 (http://localhost:${WEB_PORT})..."
cd "$SCRIPT_DIR"
python3 -m http.server $WEB_PORT \
  > /tmp/webui_http.log 2>&1 &
PID_HTTP=$!
sleep 1

if kill -0 $PID_HTTP 2>/dev/null; then
  ok "HTTP 服务器已启动 (PID=$PID_HTTP)"
else
  error "HTTP 服务器启动失败"
  cleanup
fi

# ─── 打印访问信息 ──────────────────────────────────────
echo ""
echo -e "${GRN}=========================================="
echo -e "  ✅ Web UI 启动成功！"
echo -e "==========================================${NC}"
echo ""
echo -e "  🌐 打开浏览器访问："
echo -e "     ${CYN}http://localhost:${WEB_PORT}${NC}"
echo ""
echo -e "  📡 服务状态："
echo -e "     rosbridge    : ws://localhost:9090"
echo -e "     video_server : http://localhost:8080"
echo -e "     前端页面     : http://localhost:${WEB_PORT}"
echo ""
echo -e "  🗺️  相机流示例："
echo -e "     ${CYN}http://localhost:8080/stream?topic=/x_bot/camera_left/image_raw${NC}"
echo ""
echo -e "  ⌨️  快捷键（在浏览器中）："
echo -e "     G = 设置导航目标  |  Esc = 取消目标"
echo -e "     F = 适应地图视图  |  R   = 重置3D视角"
echo -e "     Space = 急停"
echo ""
echo -e "  🛑 停止: ${YLW}Ctrl+C${NC}"
echo ""

# 等待子进程
wait $PID_BRIDGE
