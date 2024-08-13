cmake_minimum_required(VERSION 3.5)
project(lib_consumer)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# find our package!
find_package(the_library REQUIRED)


# Add our library consumer executable
add_executable(consumer src/consumer.cpp)

# link the consumer to the library in our package, it is referenced as follows: package_name::library_exported_name_in_the_library_cmakelists 
target_link_libraries(consumer the_library::my_lib)

# this installs the executable in the lib folder to be seen by setup.bash to be seen by ros2 :)
install(TARGETS consumer
DESTINATION lib/${PROJECT_NAME}/)

ament_package()
