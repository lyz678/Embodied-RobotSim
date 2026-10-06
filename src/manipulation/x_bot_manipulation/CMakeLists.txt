cmake_minimum_required(VERSION 3.16)
project(x_bot_manipulation)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/physical_pick_verification.py scripts/pick_and_place_demo.py scripts/navigation_and_pick_demo.py scripts/gripper_action.py DESTINATION lib/${PROJECT_NAME})
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(robot_actions src/robot_actions.cpp)
target_include_directories(robot_actions PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(robot_actions PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(robot_actions
  rclcpp
  moveit_ros_planning_interface
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  std_srvs
  ament_index_cpp
  yaml-cpp
  control_msgs
  rclcpp_action
  sensor_msgs
  pcl_conversions
  cv_bridge
  moveit_msgs
  std_msgs
)

target_link_libraries(robot_actions yaml-cpp::yaml-cpp)

install(TARGETS robot_actions
  DESTINATION lib/${PROJECT_NAME})


ament_package()
