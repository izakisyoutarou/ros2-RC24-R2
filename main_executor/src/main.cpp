#include <rclcpp/rclcpp.hpp>
#include <rcl/rcl.h>
#include "socketcan_interface/socketcan_interface_node.hpp"
#include "icp_base_slam/icp_base_slam.hpp"
#include "mcl_2d/mcl_2d_node.hpp"
#include <iostream>

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    // auto socketcan_node = std::make_shared<socketcan_interface::SocketcanInterface>();
    // auto mcl_2d_node = std::make_shared<mcl_2d::Mcl2D>();
    auto icp_base_slam = std::make_shared<IcpBaseSlam>();
    exec.add_node(icp_base_slam);

    // exec.add_node(socketcan_node);
    // exec.add_node(mcl_2d_node);

    exec.spin();
    rclcpp::shutdown();
    return 0;
}
