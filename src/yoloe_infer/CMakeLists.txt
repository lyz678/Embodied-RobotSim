cmake_minimum_required(VERSION 3.8)
project(yoloe_infer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)


# CUDA and TensorRT
find_package(CUDAToolkit REQUIRED)

# Try to find TensorRT in common locations
set(TENSORRT_FOUND FALSE)
set(TENSORRT_ROOT "/usr/local/TensorRT-10.14.1.48.Linux.x86_64-gnu.cuda-13.0/TensorRT-10.14.1.48" CACHE PATH "TensorRT installation root directory")

# Common TensorRT installation paths
set(TENSORRT_SEARCH_PATHS
  ${TENSORRT_ROOT}
  /usr/local/TensorRT
  /opt/TensorRT
  /usr
  /usr/local
)

foreach(path ${TENSORRT_SEARCH_PATHS})
  if(EXISTS "${path}/include/NvInfer.h")
    set(TENSORRT_INCLUDE_DIR "${path}/include")
    find_library(TENSORRT_LIBRARY_INFER nvinfer HINTS "${path}/lib" "${path}/lib64")
    if(TENSORRT_LIBRARY_INFER)
      set(TENSORRT_FOUND TRUE)
      break()
    endif()
  endif()
endforeach()

if(NOT TENSORRT_FOUND)
  message(WARNING "TensorRT not found. The TensorRT-based inference node will not be built. Please install TensorRT and set TENSORRT_ROOT if needed.")
  set(BUILD_TRT_NODE OFF)
else()
  set(BUILD_TRT_NODE ON)
  message(STATUS "Found TensorRT: ${TENSORRT_INCLUDE_DIR}, ${TENSORRT_LIBRARY_INFER}")
endif()

find_package(nlohmann_json REQUIRED)


# Simple Tokenizer Library
add_library(simple_tokenizer_lib
  src/simple_tokenizer.cpp
)
target_include_directories(simple_tokenizer_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(simple_tokenizer_lib
  nlohmann_json::nlohmann_json
  z
)
ament_target_dependencies(simple_tokenizer_lib
  rclcpp
  ament_index_cpp
)

if(BUILD_TRT_NODE)
  # TensorRT Engine Library
  add_library(trt_engine_lib
    src/trt_engine.cpp
  )
  target_include_directories(trt_engine_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${TENSORRT_INCLUDE_DIR}
  )
  target_link_libraries(trt_engine_lib
    simple_tokenizer_lib
    ${TENSORRT_LIBRARY_INFER}
    CUDA::cudart
    ${OpenCV_LIBRARIES}
  )
  ament_target_dependencies(trt_engine_lib
    rclcpp
  )

  # PointCloud Colorizer Library
  add_library(pointcloud_colorizer_lib
    src/pointcloud_colorizer.cpp
  )
  target_include_directories(pointcloud_colorizer_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${TENSORRT_INCLUDE_DIR}
  )
  target_link_libraries(pointcloud_colorizer_lib
    ${pcl_conversions_LIBRARIES}
    ${OpenCV_LIBRARIES}
    CUDA::cudart
  )
  ament_target_dependencies(pointcloud_colorizer_lib
    rclcpp
    sensor_msgs
    pcl_msgs
    pcl_msgs
    cv_bridge
    image_geometry
    tf2_ros
    tf2_geometry_msgs
  )

  # Multi Text Prompt Node
  add_executable(ros2_trt_infer_text_prompt_multi_node
    src/ros2_trt_infer_text_prompt_multi_node.cpp
  )
  target_include_directories(ros2_trt_infer_text_prompt_multi_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${TENSORRT_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(ros2_trt_infer_text_prompt_multi_node
    trt_engine_lib
    simple_tokenizer_lib
    ${TENSORRT_LIBRARY_INFER}
    CUDA::cudart
    ${OpenCV_LIBRARIES}
    yaml-cpp
    ament_index_cpp::ament_index_cpp
    pointcloud_colorizer_lib
    ${PCL_LIBRARIES}
  )
  ament_target_dependencies(ros2_trt_infer_text_prompt_multi_node
    rclcpp
    sensor_msgs
    std_srvs
    cv_bridge
    vision_msgs
    ament_index_cpp
    pcl_conversions
    pcl_msgs
  )

  install(TARGETS
    trt_engine_lib
    pointcloud_colorizer_lib
    ros2_trt_infer_text_prompt_multi_node
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(STATUS "Skipping TensorRT-based inference node due to missing TensorRT")
endif()

install(TARGETS
  simple_tokenizer_lib
  DESTINATION lib/${PROJECT_NAME}
)

# Install models directory
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY configs
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
