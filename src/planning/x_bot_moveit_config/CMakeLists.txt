cmake_minimum_required(VERSION 3.16)
project(x_bot_moveit_config)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
