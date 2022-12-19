#pragma once
#include <rclcpp/rclcpp.hpp>
#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "controller_interface_msg/msg/sub_joy.hpp"
#include "utilities/can_utils.hpp"
#include "my_visibility.h"

namespace controller_interface
{
    class ControllerInterface : public rclcpp::Node
    {
        public:
            explicit ControllerInterface(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
            explicit ControllerInterface(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        private:
            rclcpp::Subscription<controller_interface_msg::msg::SubJoy>::SharedPtr _sub_joy;
            rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_linear;
            rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_angular;
            rclcpp::QoS _qos = rclcpp::QoS(40).keep_all();

            void callback_joy(const controller_interface_msg::msg::SubJoy::SharedPtr msg);
            float roundoff(const float &value, const float &epsilon);
            float max_linear_x = 1.0f;
            float max_linear_y = 1.0f;
            float max_angular_z = 1.0f;
    };
}