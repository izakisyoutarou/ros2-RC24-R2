#pragma once
#include <rclcpp/rclcpp.hpp>
#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "visibility_control.h"

namespace controller_interface
{
    class Heartbeat : public rclcpp::Node
    {
        public:
            CONTROLLER_INTERFACE_PUBLIC
            explicit Heartbeat(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
            
            CONTROLLER_INTERFACE_PUBLIC
            explicit Heartbeat(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        private:
            rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb;

            rclcpp::TimerBase::SharedPtr _pub_heartbeat;

            rclcpp::QoS _qos = rclcpp::QoS(10);
            const int16_t can_heartbeat_id;
    };
}
