#!/bin/bash

# ========================================
# 机器人导航模式启动脚本
# ========================================
# 功能：启动 Gazebo + Nav2（使用预先保存的地图）
# 用途：在已知地图中进行导航，不进行建图
#
# 修复说明：
#   1. 使用固定初始位置 (0,0,0)，与建图时一致，保证 AMCL 正确定位
#   2. 移除 SLAM 节点，避免与 AMCL 产生 map->odom TF 冲突
#   3. 里程计必须与建图一致：odometry_source:=world（与 start_explore_and_mapping.sh 相同）
#      若用 encoders，轮式里程计与 Cartographer 建图时不一致，RViz 中雷达点会严重错位
#   4. colcon build 改为可选（--build 参数触发）

echo "========================================"
echo "  启动机器人导航模式"
echo "========================================"

# 0. 停止之前的进程
echo "🛑 正在清理残留进程..."
bash stop_robot_sim.sh
sleep 2

echo "Building remaining packages..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

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
    echo "   bash start_navigation.sh /path/to/your/map.yaml"
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
INIT_YAW=0.0

echo "📍 机器人初始位置 (与建图时一致):"
echo "   X: $INIT_X  Y: $INIT_Y  Yaw: $INIT_YAW"
echo ""

# source 工作空间
source /opt/ros/jazzy/setup.bash
source install/setup.bash 2>/dev/null || true

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

echo ""
echo "========================================"
echo "  ✅ 所有服务已启动完成！"
echo "========================================"
echo ""
echo "🗺️  使用地图: $MAP_FILE"
echo ""
echo "📍 初始位置 (0,0,0)，里程计 world（与探索建图一致）"
echo "   若雷达点与地图仍有偏差，在 RViz 用 '2D Pose Estimate' 校正一次"
echo "   地图须为探索结束后 map_saver_cli 保存的 src/x_bot/maps/map"
echo ""
echo "🎯 在 Web 前端设置导航目标："
echo "   1. 打开 http://localhost:8888"
echo "   2. 点击 '设置目标' 按钮"
echo "   3. 在地图上点击拖动确定位置和朝向"
echo ""
echo "🎯 在 RViz 中设置导航目标："
echo "   1. 点击 '2D Nav Goal' 按钮"
echo "   2. 在地图上点击并拖动设置目标位置和朝向"
echo ""
echo "🛑 停止所有服务："
echo "   bash stop_robot_sim.sh"
echo ""
