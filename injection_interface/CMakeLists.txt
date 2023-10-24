cmake_minimum_required(VERSION 3.8)
project(injection_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(injection_interface_msg REQUIRED)
find_package(utilities REQUIRED)

add_library(injection_interface_node SHARED
  src/injection_interface_node.cpp
)

target_compile_definitions(injection_interface_node
  PRIVATE "INJECTION_INTERFACE_BUILDING_LIBRARY"
)

target_compile_options(injection_interface_node PUBLIC -Wall)

ament_target_dependencies(injection_interface_node
  rclcpp
  std_msgs
  geometry_msgs
  injection_interface_msg
  utilities
)

target_include_directories(injection_interface_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
  )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  injection_interface_msg
  utilities
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
injection_interface_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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

ament_package()
