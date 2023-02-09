#pragma once

#include <rclcpp/rclcpp.hpp>

// #include <std_msgs/msg/empty.hpp>
#include "path_msg/msg/path.hpp"
#include "spline_pid/visibility_control.h"

namespace spline_pid{

class SplinePid : public rclcpp::Node {
public:
    SPLINE_PID_PUBLIC
    explicit SplinePid(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    SPLINE_PID_PUBLIC
    explicit SplinePid(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    rclcpp::Subscription<path_msg::msg::Path>::SharedPtr _subscription_path;

    rclcpp::QoS _qos = rclcpp::QoS(40).keep_all();

    void _subscriber_callback_path(const path_msg::msg::Path::SharedPtr msg);

};

}  // namespace spline_pid
