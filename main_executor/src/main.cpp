#include <rclcpp/rclcpp.hpp>
#include <rcl/rcl.h>
#include "socketcan_interface/socketcan_interface_node.hpp"
#include <iostream>

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    auto socketcan_node = std::make_shared<socketcan_interface::SocketcanInterface>();
    exec.add_node(socketcan_node);

    exec.spin();
    rclcpp::shutdown();
    return 0;
}
