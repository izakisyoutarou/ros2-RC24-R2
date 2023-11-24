#include <rclcpp/rclcpp.hpp>
#include <rcl/rcl.h>
#include "socketcan_interface/socketcan_interface_node.hpp"
#include "controller_interface/controller_interface_node.hpp"
#include "ransac_localization/ransac_localization.hpp"
#include <iostream>

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    rclcpp::NodeOptions nodes_option;
    nodes_option.allow_undeclared_parameters(true);
    nodes_option.automatically_declare_parameters_from_overrides(true);

    auto controller_node = std::make_shared<controller_interface::SmartphoneGamepad>(nodes_option);
    //auto socketcan_node = std::make_shared<socketcan_interface::SocketcanInterface>(nodes_option);
    auto ransac_localization = std::make_shared<self_localization::ransaclocalization>(nodes_option);

    exec.add_node(controller_node);
    //exec.add_node(socketcan_node);
    exec.add_node(ransac_localization);
    //auto socketcan_node = std::make_shared<socketcan_interface::SocketcanInterface>(nodes_option);


    exec.spin();
    rclcpp::shutdown();
    return 0;
}
