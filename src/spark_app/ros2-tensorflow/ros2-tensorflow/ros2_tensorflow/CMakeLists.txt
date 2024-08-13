cmake_minimum_required(VERSION 3.5)
project(ros2_tensorflow)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
install(
    DIRECTORY
    launch rviz  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
