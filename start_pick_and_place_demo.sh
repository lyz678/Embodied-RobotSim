#!/bin/bash

# ========================================
# 机器人抓取演示启动脚本
# ========================================

# 0. 停止之前的进程
echo "🛑 正在清理残留进程..."
bash stop_robot_sim.sh
sleep 2

source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "========================================"
echo "  启动机器人抓取演示"
echo "========================================"

echo "Step 2: Building remaining packages..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
if [ $? -ne 0 ]; then
    echo "❌ 构建失败，请检查错误"
    exit 1
fi
echo "✅ 构建成功！"
source install/setup.bash

echo "📡 启动 Gazebo 仿真..."
gnome-terminal --title="Gazebo" -- bash -c "ros2 launch x_bot gz.launch.py world_name:=manipulation_test; exec bash"
sleep 5

echo "🗺️  启动 Cartographer SLAM 建图..."
gnome-terminal --title="Cartographer SLAM" -- bash -c "ros2 launch x_bot cartographer.launch.py; exec bash"

# echo "👁️  启动立体匹配..."
# gnome-terminal --title="Stereo Matching" -- bash -c "ros2 launch stereo_matching stereo_matching.launch.py; exec bash"
sleep 2

echo "🤖 启动 MoveIt 运动规划..."
gnome-terminal --title="MoveIt" -- bash -c "ros2 launch x_bot move_group.launch.py use_sim_time:=true use_rviz:=true; exec bash"

echo "👁️  启动 YOLOE 视觉检测..."
gnome-terminal --title="YOLOE Vision" -- bash -c "ros2 run yoloe_infer ros2_trt_infer_text_prompt_multi_node --ros-args \
    -p config_path:=$(pwd)/src/yoloe_infer/configs/config.yaml; exec bash"
sleep 2

echo "👁️  启动 GraspNet..."
gnome-terminal --title="GraspNet" -- bash -c "ros2 run graspnet_ros graspnet_node --ros-args \
    --params-file $(pwd)/src/graspnet_infer/graspnet_ros/config/config.yaml & \
    rviz2 -d $(pwd)/src/graspnet_infer/graspnet.rviz; exec bash"

echo "🦾 启动机械臂控制器..."
gnome-terminal --title="Arm Ctrl" -- bash -c "ros2 run x_bot robot_actions --ros-args -p use_sim_time:=true; exec bash"
sleep 1

echo "🧠 启动业务逻辑节点（循环抓取 coke, book, cup）..."
gnome-terminal --title="Logic Node" -- bash -c "python3 src/x_bot/scripts/pick_and_place_demo.py --ros-args -p use_sim_time:=true -p prompts:=\"['coke', 'book', 'cup']\"; exec bash"

echo ""
echo "=========================================="
echo "  ✅ 所有服务已启动！"
echo "=========================================="

