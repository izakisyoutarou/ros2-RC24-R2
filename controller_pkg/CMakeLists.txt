cmake_minimum_required(VERSION 3.5)
project(controller_pkg)
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
find_package(rclcpp REQUIRED)
find_package(utilities REQUIRED)
find_package(socketcan_interface_msg REQUIRED)
find_package(rosidl_default_generators REQUIRED)
#独自メッセージ
set(msg_files
  "msg/SubButton.msg"
  "msg/SubJoy.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)
#共有ライブラリ
add_library(controller_pkg_node
        src/controller_pkg_node.cpp
        )
target_compile_definitions(controller_pkg_node
        PRIVATE "CONTROLLER_PKG_LIBRARY"
        )
target_compile_options(controller_pkg_node PUBLIC -Wall)
ament_target_dependencies(controller_pkg_node
        rclcpp
        utilities
        socketcan_interface_msg
        )
target_include_directories(controller_pkg_node
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
        utilities
        socketcan_interface_msg
)
install(
        DIRECTORY include/
        DESTINATION include
)
install(
        TARGETS controller_pkg_node
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        DESTINATION include/${PROJECT_NAME})
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
ament_package()