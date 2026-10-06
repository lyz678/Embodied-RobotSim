cmake_minimum_required(VERSION 3.16)
project(x_bot_bringup)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
