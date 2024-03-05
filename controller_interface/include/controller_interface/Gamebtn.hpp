#pragma once
#include <rclcpp/rclcpp.hpp>
#include <float.h>
#include <string>
//使うmsg
#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "controller_interface_msg/msg/base_control.hpp"
#include "controller_interface_msg/msg/convergence.hpp"
#include "controller_interface_msg/msg/colorball.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std;

struct physics_btn_msg{

};

class Gamebtn{
    public:
        Gamebtn();
        void steer_reset(int16_t can_steer_reset_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void calibrate(int16_t can_calibrate_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void main_reset(int16_t can_reset_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void io_reset(int16_t can_reset_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void paddy_install(int16_t can_paddy_install_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void paddy_collect_1(int16_t can_paddy_collect_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void paddy_collect_2(int16_t can_paddy_collect_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
};