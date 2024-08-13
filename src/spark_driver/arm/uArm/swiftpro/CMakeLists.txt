cmake_minimum_required(VERSION 3.5)
project(swiftpro)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()




# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(serial REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(msg_files
  "msg/Angle4th.msg"
  "msg/Position.msg"
  "msg/Status.msg"
  "msg/SwiftproState.msg"
  "srv/Scene.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

include_directories(/opt/ros/humble/include/tf2_geometry_msgs)

include_directories(
  include
)


set(LIB_DEPENDENCIES
  "rclcpp"
  "std_msgs"
  "serial"
  "tf2"
  "tf2_ros"
  "sensor_msgs"
)

add_executable(swiftpro_write_node src/swiftpro_write_node.cpp)
ament_target_dependencies(swiftpro_write_node ${LIB_DEPENDENCIES})

add_executable(swiftpro_moveit_node src/swiftpro_moveit_node.cpp)
ament_target_dependencies(swiftpro_moveit_node ${LIB_DEPENDENCIES})

add_executable(swiftpro_rviz_node src/swiftpro_rviz_node.cpp)
ament_target_dependencies(swiftpro_rviz_node ${LIB_DEPENDENCIES})

add_executable(swiftpro_read_node src/swiftpro_read_node.cpp)
ament_target_dependencies(swiftpro_read_node ${LIB_DEPENDENCIES})


rosidl_target_interfaces(swiftpro_moveit_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(swiftpro_read_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(swiftpro_rviz_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(swiftpro_write_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


install(

)
install(
  DIRECTORY include/
  DESTINATION include
)
install(TARGETS  
  swiftpro_write_node  
  swiftpro_moveit_node
  swiftpro_rviz_node
  swiftpro_read_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)
  
ament_export_include_directories(
  include
)

ament_package()
