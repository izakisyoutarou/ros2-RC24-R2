cmake_minimum_required(VERSION 3.8)
project(detection_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(bboxes_ex_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(detection_interface_msg REQUIRED)
find_package(controller_interface_msg REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

add_library(detection_interface_node
    src/detection_interface_node.cpp
    src/coordinate_transformation.cpp
)

target_compile_definitions(detection_interface_node
  PRIVATE "DETECTION_INTERFACE_LIBRARY"
)

target_compile_options(detection_interface_node PUBLIC -Wall)

ament_target_dependencies(detection_interface_node
  rclcpp
  bboxes_ex_msgs
  std_msgs
  geometry_msgs
  detection_interface_msg
  controller_interface_msg
  image_transport
  cv_bridge
)

target_include_directories(detection_interface_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(NOT WIN32)
  ament_environment_hooks(
     "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS detection_interface_node
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
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

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
