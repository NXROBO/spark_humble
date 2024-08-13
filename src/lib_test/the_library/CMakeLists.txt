cmake_minimum_required(VERSION 3.5)
project(the_library)

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


include_directories(include/the_library) # to tell CMake to add the directory as -I to the compilation command line

set(HEADER_FILES include/the_library/lib_header.hpp) # you can use just the file you're using but this is mostly used for multiple header files

add_library(my_lib src/lib_source.cpp ${HEADER_FILES}) 

add_executable(main src/main.cpp)

target_link_libraries(main PUBLIC my_lib) # this is important to tell CMake that the executable "main" depends on the library "my_lib"

# this installs the executable in the lib folder to be seen by setup.bash to be seen by ros2 :)
install(TARGETS 
  main
  DESTINATION lib/${PROJECT_NAME}/
)

# this line to export the targets, in our case it's a library!
ament_export_targets(my_lib HAS_LIBRARY_TARGET)

# install the include/the_library directory to the install/include/the_library
install(
  DIRECTORY include/the_library
  DESTINATION include
)


install(
  TARGETS my_lib
  EXPORT my_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
