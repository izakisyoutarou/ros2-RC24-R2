cmake_minimum_required(VERSION 3.5)
project(odom_delay)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(my_messages REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs  REQUIRED)
find_package(tf2_eigen REQUIRED)

add_executable(odom_delay src/odom_delay.cpp)
ament_target_dependencies(odom_delay
  rclcpp
  my_messages
  sensor_msgs
  geometry_msgs
  ament_cmake
  nav_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen 
 )

# add_executable(listener src/subscriber_member_function.cpp)
# ament_target_dependencies(listener rclcpp my_messages)

install(TARGETS
  odom_delay
  # listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
