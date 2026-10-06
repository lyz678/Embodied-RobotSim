cmake_minimum_required(VERSION 3.16)
project(x_bot_isaac)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
install(DIRECTORY runtime launch DESTINATION share/${PROJECT_NAME})

ament_package()
