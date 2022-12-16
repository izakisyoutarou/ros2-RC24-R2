#pragma once
#include <rclcpp/rclcpp.hpp>
#include "controller_pkg/msg/sub_button.hpp"
#include "controller_pkg/msg/sub_joy.hpp"
#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "utilities/can_utils.hpp"
#include "visibility_control.h"
namespace controller_pkg
{
    class ControllerPkg : public rclcpp::Node
    {
        public:
            explicit ControllerPkg(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
            explicit ControllerPkg(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        private:
            rclcpp::Subscription<controller_pkg::msg::SubButton>::SharedPtr _sub_button;
            rclcpp::Subscription<controller_pkg::msg::SubJoy>::SharedPtr _sub_joy;
            rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_button;
            rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_linear;
            rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_angular;
            rclcpp::QoS _qos = rclcpp::QoS(40).keep_all();
            void callback_button(const controller_pkg::msg::SubButton::SharedPtr msg);
            void callback_joy(const controller_pkg::msg::SubJoy::SharedPtr msg);
            float roundoff(const float &value, const float &epsilon);
            float max_linear_x = 1.0f;
            float max_linear_y = 1.0f;
            float max_angular_z = 1.0f;
    };
}