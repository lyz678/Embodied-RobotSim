#!/bin/bash

echo "🛑 正在停止所有机器人仿真服务..."

echo "关闭机器人服务终端窗口..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 "$SCRIPT_DIR/scripts/close_robot_terminals.py"

# 1. 停止 ROS2 守护进程 (最先停止，防止节点被发现或自动重连)
echo "清理 ROS2 守护进程..."
ros2 daemon stop
pkill --ignore-ancestors -9 -f "ros2-daemon"

# 2. 核心组件清理
echo "停止核心组件 (Launch, Isaac Sim, Gazebo, RViz)..."
# 先尝试正常停止
pkill --ignore-ancestors -f "ros2 launch"
sleep 1
# 强制停止
pkill --ignore-ancestors -9 -f "ros2 launch"
pkill --ignore-ancestors -9 -f "rviz2"
pkill --ignore-ancestors -9 -f "rqt"

# 3. 强力清理模式：匹配关键字
echo "正在强力清理所有 ROS 相关进程..."
patterns=(
    "move_group"
    "moveit"
    "nav2"
    "amcl"
    "bt_navigator"
    "planner_server"
    "controller_server"
    "recoveries_server"
    "map_server"
    "waypoint_follower"
    "explore_lite"
    "slam_toolbox"
    "robot_state_publisher"
    "joint_state_publisher"
    "static_transform_publisher"
    "component_container"
    # 用户项目特定节点
    "yoloe"
    "graspnet"
    "pick_and_place"
    "franka"
    "x_bot"
    "fps_plugin"
    "stereo_matching"
    "semantic_voxel_node"
    "color_octomap_server_node"
    # Gazebo backend
    "gz sim"
    "gz-sim-server"
    "gz-sim-gui"
    "gazebo_backend.launch.py"
    # Isaac Sim backend
    "src/x_bot/isaac_sim/run_sim.py"
    "isaac-sim"
    "isaacsim"
)

for pattern in "${patterns[@]}"; do
    pkill --ignore-ancestors -9 -f "$pattern"
done

# 4. 终极清理：清理所有 Python ROS 节点
# 匹配运行 ros2 库或相关的 python 进程，以及所有 _node.py 脚本
echo "清理残留 Python 节点..."
pkill --ignore-ancestors -9 -f "python3.*ros"
pkill --ignore-ancestors -9 -f "python3.*launch"
pkill --ignore-ancestors -9 -f "_node.py"
pkill --ignore-ancestors -9 -f "tf2_monitor"
pkill --ignore-ancestors -9 -f "tf2_echo"
pkill --ignore-ancestors -9 -f "tf2_tools"

# 5. 清理可能卡住的 colcon 进程
pkill --ignore-ancestors -9 -f "colcon build"

# 6. 清理日志文件 (可选，保持环境整洁)
echo "清理 ROS2 日志..."
rm -rf ~/.ros/log/*

echo "✅ 所有服务已停止，环境清理完成！"
