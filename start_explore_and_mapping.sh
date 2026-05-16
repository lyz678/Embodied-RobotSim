#!/bin/bash

# ========================================
# 机器人自动探索建图模式启动脚本 (Cartographer 版本)
# ========================================
# 功能：启动 Gazebo + Cartographer SLAM + Nav2 + 自动探索
# 用途：在未知环境中使用 Cartographer (2D LiDAR + IMU) 自动探索并生成地图
# 
# 与 SLAM Toolbox 版本的区别：
#   - 使用 Cartographer 进行 SLAM (支持 IMU 数据融合)
#   - TF 由 Cartographer 提供 (map -> odom -> base_footprint)
#   - 地图由 cartographer_occupancy_grid_node 提供

echo "========================================"
echo "  启动机器人自动探索建图模式 (Cartographer)"
echo "========================================"

# 0. 停止之前的进程
echo "🛑 正在清理残留进程..."
bash stop_robot_sim.sh
sleep 2

echo "Step 2: Building remaining packages..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 检查构建是否成功
if [ $? -ne 0 ]; then
    echo "❌ 构建失败，请检查错误信息"
    exit 1
fi

echo "✅ 构建成功！"
echo ""

# 启动所有服务在多个终端标签页中
echo "📡 启动 Gazebo 仿真环境..."
gnome-terminal --title="Gazebo" -- bash -c "ros2 launch x_bot gz.launch.py world_name:=simple_room; exec bash"
sleep 5

echo "🗺️  启动 Cartographer SLAM 建图..."
gnome-terminal --title="Cartographer SLAM" -- bash -c "ros2 launch x_bot cartographer.launch.py; exec bash"

# echo "👁️  启动立体匹配..."
# gnome-terminal --title="Stereo Matching" -- bash -c "ros2 launch stereo_matching stereo_matching.launch.py; exec bash"

sleep 3
echo "🧭 启动 Nav2 导航 (Cartographer 模式)..."
gnome-terminal --title="Nav2 (Cartographer)" -- bash -c "ros2 launch x_bot nav2_cartographer.launch.py; exec bash"

echo "👁️  启动 YOLOE 视觉检测..."
gnome-terminal --title="YOLOE Vision" -- bash -c "ros2 run yoloe_infer ros2_trt_infer_text_prompt_multi_node --ros-args \
    -p config_path:=$(pwd)/src/yoloe_infer/configs/config.yaml; exec bash"

sleep 3
echo "🔍 启动自动探索..."
gnome-terminal --title="Auto Explore" -- bash -c "ros2 launch explore_lite explore.launch.py; exec bash"

echo "🔍 启动OctoMap建图..."
gnome-terminal --title="OctoMap" -- bash -c "ros2 launch x_bot octomap_server.launch.py; exec bash"


echo ""
echo "========================================="
echo "  ✅ 所有服务已启动完成！"
echo "========================================="
echo ""
echo "🤖 机器人将自动探索未知环境并实时建图"
echo "📊 使用 Cartographer (2D LiDAR + IMU) 进行 SLAM"
echo ""
echo "📊 监控探索进度："
echo "   ros2 topic echo /explore/frontiers"
echo ""
echo "💾 保存地图 (Cartographer 格式)："
echo "   ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \"{filename: '$(pwd)/src/x_bot/maps/cartographer_map.pbstream'}\""
echo ""
echo "💾 保存地图 (标准格式)："
echo "   ros2 run nav2_map_server map_saver_cli -f $(pwd)/src/x_bot/maps/cartographer_map"
echo ""
echo "🛑 停止所有服务："
echo "   bash stop_robot_sim.sh"
echo ""
