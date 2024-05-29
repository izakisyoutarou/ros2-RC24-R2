#include "controller_interface/heartbeat_interface_node.hpp"
#include <sys/time.h>
#include <sys/types.h>
#include <chrono>
#include <iostream>
#include <future>

namespace controller_interface
{
    using std::string;

    Heartbeat::Heartbeat(const rclcpp::NodeOptions &options) : Heartbeat("",options) {}
    Heartbeat::Heartbeat(const std::string &name_space, const rclcpp::NodeOptions &options): rclcpp::Node("heartbeat_interface_node", name_space, options),

    can_heartbeat_id(get_parameter("canid.heartbeat").as_int())
    {
        const auto heartbeat_ms = this->get_parameter("heartbeat_ms").as_int();
        _pub_canusb = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);


        //ハートビート
        _pub_heartbeat = this->create_wall_timer(
            std::chrono::milliseconds(heartbeat_ms),
            [this] {
                auto msg_heartbeat = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                msg_heartbeat->canid = can_heartbeat_id;
                msg_heartbeat->candlc = 0;
                _pub_canusb->publish(*msg_heartbeat);
            }
        );
    }
}