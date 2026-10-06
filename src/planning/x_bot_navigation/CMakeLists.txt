cmake_minimum_required(VERSION 3.16)
project(x_bot_navigation)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config behavior_trees rviz launch DESTINATION share/${PROJECT_NAME})

ament_package()
