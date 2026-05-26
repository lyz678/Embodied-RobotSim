#!/bin/bash

# ========================================
# 机械臂避障规划演示启动脚本
# ========================================

# 0. 停止之前的进程
echo "正在清理残留进程..."
bash stop_robot_sim.sh
sleep 2

source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "========================================"
echo "  启动机械臂避障规划演示"
echo "========================================"

echo "Step 1: 编译项目..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
if [ $? -ne 0 ]; then
    echo "构建失败，请检查错误"
    exit 1
fi
echo "构建成功！"
source install/setup.bash

echo "Step 2: 启动 Gazebo 仿真（避障测试环境）..."
gnome-terminal --title="Gazebo" -- bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch x_bot gz.launch.py world_name:=obstacle_avoidance_test; exec bash"
sleep 5

echo "Step 3: 启动 MoveIt 运动规划（带 RViz 可视化 + Octomap 避障）..."
gnome-terminal --title="MoveIt" -- bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch x_bot move_group.launch.py use_sim_time:=true use_rviz:=true; exec bash"
sleep 3

echo "Step 4: 启动机械臂控制器..."
gnome-terminal --title="Arm Ctrl" -- bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run x_bot robot_actions --ros-args -p use_sim_time:=true; exec bash"
sleep 2

echo "Step 5: 启动避障演示脚本（发布障碍物点云 + 运动控制）..."
gnome-terminal --title="Obstacle Avoidance Demo" -- bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 src/x_bot/scripts/obstacle_avoidance_demo.py --ros-args -p use_sim_time:=true; exec bash"

echo ""
echo "=========================================="
echo "  所有服务已启动！"
echo "  在 RViz 中观察机械臂避障规划路径"
echo "=========================================="
