#include "spline_pid/spline_pid_node.hpp"

namespace spline_pid{

SplinePid::SplinePid(const rclcpp::NodeOptions &options) : SplinePid("", options) {}

SplinePid::SplinePid(const std::string &name_space, const rclcpp::NodeOptions &options)
: rclcpp::Node("spline_pid_node", name_space, options){

    _subscription_path = this->create_subscription<path_msg::msg::Path>(
        "spline_path",
        _qos,
        std::bind(&SplinePid::_subscriber_callback_path, this, std::placeholders::_1)
    );
}

void SplinePid::_subscriber_callback_path(const path_msg::msg::Path::SharedPtr msg){
}

}  // namespace spline_pid
