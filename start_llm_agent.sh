#!/bin/bash

# ========================================
# 机器人导航 + LLM 智能体（Qwen3）启动脚本
# ========================================
# 功能：启动 Gazebo + Nav2 + 感知栈 + Qwen3 Agent
# 流程：
#   1. 启动 Gazebo 仿真环境
#   2. 启动 Nav2 导航（静态地图 + AMCL 定位）
#   3. 启动 MoveIt 运动规划
#   4. 启动 YOLOE 视觉检测 / OctoMap / GraspNet / robot_actions
#   5. 启动 Qwen3 LLM Agent（agent_server.py），替代 navigation_and_pick_demo.py
#
# 修复说明：
#   1. 使用固定初始位置 (0,0,0)，与建图时一致，保证 AMCL 正确定位
#   2. 移除 SLAM 节点，避免与 AMCL 产生 map->odom TF 冲突
#   3. 里程计必须与建图一致：odometry_source:=world（与 start_explore_and_mapping.sh 相同）
#      若用 encoders，轮式里程计与 Cartographer 建图时不一致，RViz 中雷达点会严重错位
#   4. colcon build 改为可选（--build 参数触发）
#   5. Qwen3 Agent 后端在此处启动，无需在 web_ui 中单独启动

echo "========================================"
echo "  启动机器人导航 + Qwen3 LLM Agent 模式"
echo "========================================"

# 0. 停止之前的进程
echo "🛑 正在清理残留进程..."
bash stop_robot_sim.sh
# 同时清理已有的 agent_server 进程
pkill -f "llm_agent/agent_server" 2>/dev/null || true
sleep 2

# 配置参数
MAP_FILE="${1:-src/x_bot/maps/map.yaml}"

echo "📋 使用地图文件: $MAP_FILE"
echo ""

# 检查地图文件是否存在
if [ ! -f "$MAP_FILE" ]; then
    echo "❌ 错误：地图文件不存在: $MAP_FILE"
    echo ""
    echo "💡 请先运行探索建图模式生成地图："
    echo "   bash start_explore_and_mapping.sh"
    echo ""
    echo "   保存地图："
    echo "   ros2 run nav2_map_server map_saver_cli -f src/x_bot/maps/map"
    echo ""
    echo "   或指定其他地图文件："
    echo "   bash start_llm_agent.sh /path/to/your/map.yaml"
    exit 1
fi

# 可选构建（传入 --build 参数时才执行）
if [[ "$*" == *"--build"* ]]; then
    echo "Step 2: Building packages..."
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    if [ $? -ne 0 ]; then
        echo "❌ 构建失败，请检查错误信息"
        exit 1
    fi
    echo "✅ 构建成功！"
    echo ""
fi

# ========================================
# 初始位置：使用固定值 (0,0,0)，与建图时一致
# ========================================
# 注意：如果建图时机器人从其他位置出发，需修改此处
INIT_X=0.0
INIT_Y=0.0
INIT_YAW=1.5708

echo "📍 机器人初始位置 (与建图时一致):"
echo "   X: $INIT_X  Y: $INIT_Y  Yaw: $INIT_YAW"
echo ""

# source 工作空间
source /opt/ros/jazzy/setup.bash
source install/setup.bash 2>/dev/null || true

WORKSPACE_ROOT="$(pwd)"

# ========================================
# Step 1: 启动 Gazebo（固定初始位置）
# ========================================
echo "📡 启动 Gazebo 仿真环境..."
gnome-terminal --title="Gazebo" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source $(pwd)/install/setup.bash
  ros2 launch x_bot gz.launch.py \
    world_name:=simple_room \
    odometry_source:=world \
    publish_odom_tf:=true \
    position_x:=$INIT_X \
    position_y:=$INIT_Y \
    orientation_yaw:=$INIT_YAW
  exec bash"

sleep 6

# ========================================
# Step 2: 启动 Nav2（静态地图 + AMCL 定位）
# ========================================
# 注意：不再启动 SLAM，避免与 AMCL 产生 map->odom TF 冲突
echo "🧭 启动 Nav2 导航（静态地图 + AMCL 定位）..."
gnome-terminal --title="Nav2 (Static Map)" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source $(pwd)/install/setup.bash
  ros2 launch x_bot nav2.launch.py \
    use_slam:=false \
    map_file:=$(pwd)/$MAP_FILE
  exec bash"

sleep 5

# ========================================
# Step 3: 启动抓取相关节点
# ========================================
echo "🤖 启动 MoveIt 运动规划..."
gnome-terminal --title="MoveIt" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source $WORKSPACE_ROOT/install/setup.bash
  ros2 launch x_bot move_group.launch.py use_sim_time:=true use_rviz:=true
  exec bash"

echo "👁️  启动 YOLOE 视觉检测..."
gnome-terminal --title="YOLOE Vision" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source $WORKSPACE_ROOT/install/setup.bash
  ros2 run yoloe_infer ros2_trt_infer_text_prompt_multi_node --ros-args \
    -p use_sim_time:=true \
    -p config_path:=$WORKSPACE_ROOT/src/yoloe_infer/configs/config.yaml
  exec bash"
sleep 2

echo "🗺️  启动 OctoMap 3D 建图..."
gnome-terminal --title="OctoMap" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source $WORKSPACE_ROOT/install/setup.bash
  ros2 launch x_bot octomap_server.launch.py use_sim_time:=true use_rviz:=true
  exec bash"
sleep 2

echo "👁️  启动 GraspNet..."
gnome-terminal --title="GraspNet" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source $WORKSPACE_ROOT/install/setup.bash
  ros2 run graspnet_ros graspnet_node --ros-args \
    -p use_sim_time:=true \
    --params-file $WORKSPACE_ROOT/src/graspnet_infer/graspnet_ros/config/config.yaml &
  rviz2 -d $WORKSPACE_ROOT/src/graspnet_infer/graspnet.rviz
  exec bash"

echo "🦾 启动机械臂控制器 (robot_actions)..."
gnome-terminal --title="Arm Ctrl" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source $WORKSPACE_ROOT/install/setup.bash
  ros2 run x_bot robot_actions --ros-args -p use_sim_time:=true
  exec bash"
sleep 3

# ========================================
# Step 4: 启动 Qwen3 LLM Agent（替代 navigation_and_pick_demo.py）
# ========================================
AGENT_SCRIPT="$WORKSPACE_ROOT/llm_agent/agent_server.py"
AGENT_PORT=8889

echo "🧠 启动 Qwen3 LLM Agent (ws://localhost:${AGENT_PORT}/ws/chat)..."
if [ -f "$AGENT_SCRIPT" ]; then
    if [ -z "$DASHSCOPE_API_KEY" ]; then
        echo "⚠️  未设置 DASHSCOPE_API_KEY，Qwen3 对话将无法调用 API"
        echo "   请运行: export DASHSCOPE_API_KEY=your_key"
    fi
    gnome-terminal --title="Qwen3 LLM Agent" -- bash -c "
      source /opt/ros/jazzy/setup.bash
      source $WORKSPACE_ROOT/install/setup.bash
      # 绕过 SOCKS 代理（防止 httpx Unknown scheme 错误）
      unset ALL_PROXY all_proxy HTTP_PROXY HTTPS_PROXY http_proxy https_proxy
      python3 $AGENT_SCRIPT
      exec bash"
else
    echo "❌ 未找到 agent_server.py: $AGENT_SCRIPT"
fi


# ========================================
# Step 5: 启动 Web UI
# ========================================
WEB_UI_SCRIPT="$WORKSPACE_ROOT/web_ui/start_web_ui.sh"

echo "🌐 启动 Web UI (http://localhost:8888)..."
if [ -f "$WEB_UI_SCRIPT" ]; then
    gnome-terminal --title="Web UI" -- bash -c "
      cd $WORKSPACE_ROOT
      source /opt/ros/jazzy/setup.bash
      source $WORKSPACE_ROOT/install/setup.bash 2>/dev/null || true
      bash $WEB_UI_SCRIPT
      exec bash"
    sleep 3
    # 自动打开浏览器
    if command -v xdg-open &>/dev/null; then
        xdg-open "http://localhost:8888" 2>/dev/null &
    fi
else
    echo "⚠️  未找到 Web UI 脚本: $WEB_UI_SCRIPT"
fi

echo ""
echo "========================================"
echo "  ✅ 所有服务已启动完成！"
echo "========================================"
echo ""
echo "🗺️  使用地图: $MAP_FILE"
echo ""
echo "📍 初始位置 (0,0,0)，里程计 world（与探索建图一致）"
echo "   若雷达点与地图仍有偏差，在 RViz 用 '2D Pose Estimate' 校正一次"
echo ""
echo "🤖 Qwen3 LLM Agent 已接管机械臂控制："
echo "   ws://localhost:${AGENT_PORT}/ws/chat"
echo "   示例指令: 帮我去厨房拿瓶可乐 | 到书房拿本书 | 导航到客厅"
echo ""
echo "📦 已启动节点：Gazebo / Nav2 / MoveIt / YOLOE / OctoMap / GraspNet / robot_actions / Qwen3 Agent / Web UI"
echo "🌐 Web UI 地址：http://localhost:8888"
echo "📟 查看各服务日志：对应 gnome-terminal 窗口"
echo ""
echo "🛑 停止所有服务："
echo "   bash stop_robot_sim.sh"
echo ""
