#include <rclcpp/rclcpp.hpp>
#include <rcl/rcl.h>
#include "icp_base_slam/icp_base_slam.hpp"
#include <iostream>

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    rclcpp::NodeOptions nodes_option;
    nodes_option.allow_undeclared_parameters(true);
    nodes_option.automatically_declare_parameters_from_overrides(true);

    // auto socketcan_node = std::make_shared<socketcan_interface::SocketcanInterface>(nodes_option);
    auto icp_base_slam = std::make_shared<self_localization::IcpBaseSlam>(nodes_option);
    // auto mcl_2d_node = std::make_shared<mcl_2d::Mcl2D>(nodes_option);

    // exec.add_node(socketcan_node);
    exec.add_node(icp_base_slam);
    // exec.add_node(mcl_2d_node);


    exec.spin();
    rclcpp::shutdown();
    return 0;
}
