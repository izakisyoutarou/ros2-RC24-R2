#include "controller_interface/controller_interface_node.hpp"

namespace controller_interface
{
    ControllerInterface::ControllerInterface(const rclcpp::NodeOptions &options) : ControllerInterface("", options) {}
    ControllerInterface::ControllerInterface(const std::string &name_space, const rclcpp::NodeOptions &options)
        : rclcpp::Node("controller_interface_node", name_space, options)
        {
            _sub_joy = this->create_subscription<controller_interface_msg::msg::SubJoy>(
                "sub_joy",
                _qos,
                std::bind(&ControllerInterface::callback_joy, this, std::placeholders::_1)
            );

            _sub_reset = this->create_subscription<std_msgs::msg::Empty>(
                "reset",
                _qos,
                std::bind(&ControllerInterface::callback_reset, this, std::placeholders::_1)
            );

            _sub_emergency = this->create_subscription<std_msgs::msg::Empty>(
                "emergency",
                _qos,
                std::bind(&ControllerInterface::callback_emergency, this, std::placeholders::_1)
            );

            _pub_linear = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
            _pub_angular = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
            _pub_reset = this->create_publisher<std_msgs::msg::Empty>("can_tx", _qos);
            _pub_emergency = this->create_publisher<std_msgs::msg::Empty>("can_tx", _qos);
        }
        
        void ControllerInterface::callback_joy(const controller_interface_msg::msg::SubJoy::SharedPtr msg)
        {
        auto msg_linear = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_linear->canid = 0x110;
        msg_linear->candlc = 8;

        auto msg_angular = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_angular->canid = 0x111;
        msg_angular->candlc = 4;

        uint8_t _candata[8];

        float_to_bytes(_candata, roundoff(msg->linear_x,1e-4)*max_linear_x);
        float_to_bytes(_candata+4, roundoff(msg->linear_y,1e-4)*max_linear_y);
        for(int i=0; i<msg_linear->candlc; i++) msg_linear->candata[i] = _candata[i];
        
        float_to_bytes(_candata, roundoff(msg->angular_z,1e-4)*max_angular_z);
        for(int i=0; i<msg_angular->candlc; i++) msg_angular->candata[i] = _candata[i];
        RCLCPP_INFO(this->get_logger(), "%u", msg_angular->candata);

        _pub_linear->publish(*msg_linear);
        _pub_angular->publish(*msg_angular);
        }

        void ControllerInterface::callback_reset(const std_msgs::msg::Empty::SharedPtr msg)
        {
            auto msg_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_reset->canid = 0x002;
            msg_reset->candlc = 1;

            uint8_t _candata = msg->structure_needs_at_least_one_member;
            msg_reset->candata[0] = _candata;
        }

        void ControllerInterface::callback_emergency(const std_msgs::msg::Empty::SharedPtr msg)
        {
            auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_emergency->canid = 0x000;
            msg_emergency->candlc = 1;

            uint8_t _candata = msg->structure_needs_at_least_one_member;
            msg_emergency->candata[0] = _candata;
        }

        float ControllerInterface::roundoff(const float &value, const float &epsilon)
        {
            float ans = value;
            if(abs(ans) < epsilon) ans = 0.0;
            return ans;
        }
}