cmake_minimum_required(VERSION 3.8)
project(spline_pid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
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
find_package(controller_interface_msg REQUIRED)
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
  controller_interface_msg
  utilities
)

target_compile_definitions(spline_pid_node PRIVATE "SPLINE_PID_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS spline_pid_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  spline_pid_node
)

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  path_msg
  controller_interface_msg
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#-- for Python
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/spline_trajectories.py
  scripts/path_plot.py
  DESTINATION lib/${PROJECT_NAME}
)
#--- end Python

ament_package()
