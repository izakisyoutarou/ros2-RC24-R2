cmake_minimum_required(VERSION 3.5)
project(spline_pid)

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
find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(path_msg REQUIRED)
find_package(socketcan_interface_msg REQUIRED)
find_package(utilities REQUIRED)

add_library(spline_pid_node src/spline_pid_node.cpp)
target_include_directories(spline_pid_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(spline_pid_node
  rclcpp
  std_msgs
  geometry_msgs
  path_msg
  socketcan_interface_msg
  utilities
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(spline_pid_node PRIVATE "SPLINE_PID_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config
 DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS spline_pid_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  spline_pid_node
)
ament_export_targets(
  export_${PROJECT_NAME}
)

#-- for Python
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/spline_trajectories.py
  #scripts/py_node.py
  DESTINATION lib/${PROJECT_NAME}
)
#--- end Python

ament_package()
