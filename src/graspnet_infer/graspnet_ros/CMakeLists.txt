cmake_minimum_required(VERSION 3.8)
project(graspnet_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CUDA REQUIRED)

# TensorRT
set(TRT_ROOT /usr/local/TensorRT-10.14.1.48.Linux.x86_64-gnu.cuda-13.0/TensorRT-10.14.1.48)
find_library(NVINFER_LIB nvinfer HINTS ${TRT_ROOT}/lib)
find_library(NVINFER_PLUGIN_LIB nvinfer_plugin HINTS ${TRT_ROOT}/lib)
find_library(NVINFER_PARSERS_LIB nvonnxparser HINTS ${TRT_ROOT}/lib)
find_path(TRT_INCLUDE_DIR NvInfer.h HINTS ${TRT_ROOT}/include)

if(NOT NVINFER_LIB)
    message(FATAL_ERROR "Could not find nvinfer library")
endif()

# CUDA
set(CUDA_ROOT /usr/local/cuda-13.1)
find_library(CUDA_LIBRARIES cudart HINTS ${CUDA_ROOT}/lib64)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${TRT_INCLUDE_DIR}
  ${CUDA_ROOT}/include
  /usr/include/eigen3
)

add_executable(graspnet_node
  src/graspnet_node.cpp
  src/graspnet_cpp.cpp
  src/trt_wrapper.cpp
  src/transform_utils.cpp
  src/visualization_utils.cpp
  src/grasp_processing.cpp
)

ament_target_dependencies(graspnet_node
  rclcpp
  sensor_msgs
  visualization_msgs
  vision_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
  tf2_geometry_msgs
  shape_msgs
  moveit_msgs
  std_msgs
  std_srvs
)

target_link_libraries(graspnet_node
  ${OpenCV_LIBS}
  ${NVINFER_LIB}
  ${NVINFER_PLUGIN_LIB}
  ${NVINFER_PARSERS_LIB}
  ${CUDA_LIBRARIES}
  yaml-cpp
  dl
)

install(TARGETS graspnet_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
