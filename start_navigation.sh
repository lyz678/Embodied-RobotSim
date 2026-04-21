#!/bin/bash

# ========================================
# 机器人导航模式启动脚本
# ========================================
# 功能：启动 Gazebo + Nav2（使用预先保存的地图）
# 用途：在已知地图中进行导航，不进行建图

echo "========================================"
echo "  启动机器人导航模式"
echo "========================================"

# 0. 停止之前的进程
echo "🛑 正在清理残留进程..."
bash stop_robot_sim.sh
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
    echo "   bash start_explore_mapping.sh"
    echo ""
    echo "   或指定其他地图文件："
    echo "   bash start_navigation.sh /path/to/your/map.yaml"
    exit 1
fi

# Step 2: Build the rest with symlink-install
echo "Step 2: Building remaining packages..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 检查构建是否成功
if [ $? -ne 0 ]; then
    echo "❌ 构建失败，请检查错误信息"
    exit 1
fi

echo "✅ 构建成功！"
echo ""

# ========================================
# 生成机器人随机初始位置
# ========================================
# 配置随机位置范围（根据您的地图调整这些值）
X_MIN=-3.0
X_MAX=3.0
Y_MIN=-3.0
Y_MAX=3.0

# 使用 awk 生成随机浮点数
RANDOM_X=$(awk -v min=$X_MIN -v max=$X_MAX 'BEGIN{srand(); print min+rand()*(max-min)}')
RANDOM_Y=$(awk -v min=$Y_MIN -v max=$Y_MAX 'BEGIN{srand(); print min+rand()*(max-min)}')
RANDOM_YAW=$(awk 'BEGIN{srand(); print -3.14159+rand()*6.28318}')  # -π 到 π

echo "📍 机器人随机初始位置:"
echo "   X: $RANDOM_X"
echo "   Y: $RANDOM_Y"
echo "   Yaw: $RANDOM_YAW (弧度)"
echo ""

# 启动所有服务在多个终端标签页中
echo "📡 启动 Gazebo 仿真环境..."
gnome-terminal --title="Gazebo" -- bash -c "ros2 launch x_bot gz.launch.py world_name:=simple_room position_x:=$RANDOM_X position_y:=$RANDOM_Y orientation_yaw:=$RANDOM_YAW; exec bash"

sleep 5

echo "🗺️  启动 SLAM 建图..."
gnome-terminal --title="SLAM Mapping" -- bash -c "ros2 launch x_bot mapping.launch.py; exec bash"
sleep 3
echo "✅ SLAM 建图已启动（将实时更新地图）"

echo "🧭 启动 Nav2 导航（静态地图模式）..."
gnome-terminal --title="Nav2 (Static Map)" -- bash -c "ros2 launch x_bot nav2.launch.py use_slam:=false map_file:=$MAP_FILE; exec bash"

echo ""
echo "========================================"
echo "  ✅ 所有服务已启动完成！"
echo "========================================"
echo ""
echo "🗺️  使用地图: $MAP_FILE"
echo ""
echo "🎯 在 RViz 中设置导航目标："
echo "   1. 点击 '2D Nav Goal' 按钮"
echo "   2. 在地图上点击并拖动设置目标位置和朝向"
echo "   3. 机器人将自动规划路径并导航"
echo ""
echo "📍 设置初始位姿（如果机器人位置不准确）："
echo "   1. 点击 '2D Pose Estimate' 按钮"
echo "   2. 在地图上点击并拖动设置机器人当前位置"
echo ""
echo "🛑 停止所有服务："
echo "   bash stop_robot_sim.sh"
echo ""
