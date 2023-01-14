
cmake_minimum_required(VERSION 3.5)
project(icp_base_slam)

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

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(dependencies
  ament_cmake
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  PCL
  pcl_conversions
  utilities
  socketcan_interface_msg
)
#add_library(${PROJECT_NAME} ${SRC_FILES})
include_directories(icp_base_slam
  ${PCL_INCLUDE_DIRS}
)
add_library(icp_base_slam
  src/icp_base_slam.cpp
  src/pose_fuser.cpp
  src/ransac_lines.cpp
  src/dynamic_voxel_grid_filter.cpp
)
target_compile_definitions(icp_base_slam
  PRIVATE "ICP_BASE_SLAM_LIBRARY"
)
target_compile_options(icp_base_slam PUBLIC -Wall)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

ament_target_dependencies(icp_base_slam
  ${dependencies}
)

target_link_libraries(icp_base_slam
  ${PCL_LIBRARIES}
)

target_include_directories(icp_base_slam PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ${dependencies}
)
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS icp_base_slam
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION include/${PROJECT_NAME}
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

ament_package()
