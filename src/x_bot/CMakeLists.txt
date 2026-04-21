cmake_minimum_required(VERSION 3.5)
project(x_bot)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# Build move_to_point executable




# Install resource directories
install(DIRECTORY launch meshes models urdf worlds rviz config photos scripts
    DESTINATION share/${PROJECT_NAME}/
)

# Build robot_actions executable
add_executable(robot_actions src/robot_actions.cpp)
ament_target_dependencies(robot_actions
  rclcpp
  moveit_ros_planning_interface
  moveit_ros_planning
  std_srvs
  ament_index_cpp
  tf2
  geometry_msgs
  control_msgs
  rclcpp_action
)


install(TARGETS robot_actions
    DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
    scripts/teleop_keyboard.py
    scripts/tf_to_odom.py
    DESTINATION lib/${PROJECT_NAME}
)
         
ament_package()
