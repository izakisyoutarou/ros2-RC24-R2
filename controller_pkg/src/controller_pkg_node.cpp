#include "controller_pkg/controller_pkg_node.hpp"

namespace controller_pkg
{
    ControllerPkg::ControllerPkg(const rclcpp::NodeOptions &options) : ControllerPkg("", options) {}

    ControllerPkg::ControllerPkg(const std::string &name_space, const rclcpp::NodeOptions &options)
        : rclcpp::Node("controller_node", name_space, options)
        {
            _sub_button = this->create_subscription<controller_pkg::msg::SubButton>(
                "sub_button",
                _qos,
                std::bind(&ControllerPkg::callback_button, this, std::placeholders::_1)
            );
            _sub_joy = this->create_subscription<controller_pkg::msg::SubJoy>(
                "sub_joy",
                _qos,
                std::bind(&ControllerPkg::callback_joy, this, std::placeholders::_1)
            );

            _pub_button = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
            _pub_linear = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
            _pub_angular = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
        }

        void ControllerPkg::callback_button(const controller_pkg::msg::SubButton::SharedPtr msg)
        {
        auto msg_button = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_button->canid = 0x100;
        msg_button->candlc = 6;

        uint8_t _candata[6];
        short_to_bytes(_candata, msg->button_num0);
        short_to_bytes(_candata+2, msg->button_num1);
        short_to_bytes(_candata+4, msg->button_num2);
        for(int i=0; i<msg_button->candlc; i++) msg_button->candata[i] = _candata[i];

        _pub_button->publish(*msg_button);
        }

        void ControllerPkg::callback_joy(const controller_pkg::msg::SubJoy::SharedPtr msg)
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

        _pub_linear->publish(*msg_linear);
        _pub_angular->publish(*msg_angular);
        }

        float ControllerPkg::roundoff(const float &value, const float &epsilon)
        {
            float ans = value;
            if(abs(ans) < epsilon) ans = 0.0;
            return ans;
        }

}