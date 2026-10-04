cmake_minimum_required(VERSION 3.14)
project(stereo_matching LANGUAGES CXX CUDA)

# Set C++ and CUDA standards
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# Set CUDA architecture (RTX 5060 Ti = Blackwell, compute_120)
set(CMAKE_CUDA_ARCHITECTURES 120)

# Default to Release build if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Find ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

# Find non-ROS dependencies
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)

# Include directories
include_directories(
  include
  cuda/inc
  ${CUDAToolkit_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# TensorRT is installed via apt, libraries are in system paths


# Config manager library
add_library(${PROJECT_NAME}_lib
  src/config_manager.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_lib
  yaml-cpp
  Eigen3::Eigen
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  ament_index_cpp
  sensor_msgs
)

# Stereo matching node
add_executable(stereo_matching_node
  src/stereo_matching_node.cpp
  cuda/src/inference.cpp
  cuda/src/cuda_process.cu
  cuda/src/pointcloud_generator.cu
)
target_link_libraries(stereo_matching_node
  ${PROJECT_NAME}_lib
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
  CUDA::cudart
  CUDA::cublas
  nvinfer
  nvonnxparser
  nvinfer_plugin
)
ament_target_dependencies(stereo_matching_node
  rclcpp
  ament_index_cpp
  std_msgs
  std_srvs
  sensor_msgs
  stereo_msgs
  cv_bridge
  image_transport
  message_filters
  tf2_ros
  tf2_eigen
  pcl_conversions
)
set_target_properties(stereo_matching_node PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
)


# Install targets
install(TARGETS 
  stereo_matching_node 
  ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING 
  PATTERN "*.py"
  PATTERN "*.rviz"
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

# Install TensorRT engine files
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/LSM_conf_640_fp16.engine
  DESTINATION share/${PROJECT_NAME}/engines
)

# Install CUDA headers for potential downstream packages
install(DIRECTORY cuda/inc/
  DESTINATION include/${PROJECT_NAME}/cuda
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(
  rclcpp
  ament_index_cpp
  sensor_msgs
  cv_bridge
  image_transport
)

ament_package()
