#include <rclcpp/rclcpp.hpp>
#include <rcl/rcl.h>
#include "icp_base_slam/icp_base_slam.hpp"
#include <iostream>

int main(int argc, char * argv[]){
  rclcpp::init(argc,argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  auto icp_base_slam = std::make_shared<self_localization::IcpBaseSlam>();
  exec.add_node(icp_base_slam);
  exec.spin();
  rclcpp::shutdown();

  return 0;
}
