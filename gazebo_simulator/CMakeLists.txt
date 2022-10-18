cmake_minimum_required(VERSION 3.8)
project(gazebo_simulator)

# Default to C99
if(NOT CMAKE_C_STANDARD)
 set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
 set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(socketcan_interface_msg REQUIRED)
find_package(utilities REQUIRED)

add_executable(planar_bot_converter
    src/planar_bot_converter.cpp
)
ament_target_dependencies(planar_bot_converter
    rclcpp
    geometry_msgs
    nav_msgs
    socketcan_interface_msg
    utilities
)
target_include_directories(planar_bot_converter
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Generate .urdf files from .urdf.xacro files
find_package(xacro REQUIRED)
# Xacro files
file(GLOB xacro_files urdf/*.urdf.xacro)

foreach(it ${xacro_files})
# remove .xacro extension
string(REGEX MATCH "(.*)[.]xacro$" unused ${it})
set(output_filename ${CMAKE_MATCH_1})

# create a rule to generate ${output_filename} from {it}
xacro_add_xacro_file(${it} ${output_filename})

list(APPEND urdf_files ${output_filename})
endforeach(it)

# add an abstract target to actually trigger the builds
add_custom_target(media_files ALL DEPENDS ${urdf_files})

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

install(DIRECTORY worlds
 DESTINATION share/${PROJECT_NAME})

install(DIRECTORY urdf
 DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
 DESTINATION share/${PROJECT_NAME})

install(TARGETS
    planar_bot_converter
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
