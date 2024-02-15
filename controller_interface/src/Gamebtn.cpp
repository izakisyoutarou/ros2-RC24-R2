#include "controller_interface/Gamebtn.hpp"
#include <rclcpp/rclcpp.hpp>

Gamebtn::Gamebtn(){}

void Gamebtn::steer_reset(int16_t can_steer_reset_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    auto msg_steer_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_steer_reset->canid = can_steer_reset_id;
    msg_steer_reset->candlc = 0;
    _pub_canusb->publish(*msg_steer_reset);
}

void Gamebtn::calibrate(int16_t can_calibrate_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    auto msg_calibrate = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_calibrate->canid = can_calibrate_id;
    msg_calibrate->candlc = 0;
    _pub_canusb->publish(*msg_calibrate);
}

void Gamebtn::main_reset(int16_t can_reset_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    auto msg_main_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_main_reset->canid = can_reset_id;
    msg_main_reset->candlc = 1;
    msg_main_reset->candata[0] = 0;
    _pub_canusb->publish(*msg_main_reset);
}

void Gamebtn::io_reset(int16_t can_reset_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    auto msg_io_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_io_reset->canid = can_reset_id;
    msg_io_reset->candlc = 1;
    msg_io_reset->candata[0] = 1;
    _pub_canusb->publish(*msg_io_reset);
}

void Gamebtn::paddy_install(int16_t can_paddy_install_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    auto msg_paddy_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_paddy_install->canid = can_paddy_install_id;
    msg_paddy_install->candlc = 0;
    _pub_canusb->publish(*msg_paddy_install); 
}

void Gamebtn::paddy_collect_1(int16_t can_paddy_collect_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    auto msg_paddy_collect = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_paddy_collect->canid = can_paddy_collect_id;
    msg_paddy_collect->candlc = 1;
    msg_paddy_collect->candata[0] = 0;
    _pub_canusb->publish(*msg_paddy_collect);
}

void Gamebtn::paddy_collect_2(int16_t can_paddy_collect_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    auto msg_paddy_collect = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_paddy_collect->canid = can_paddy_collect_id;
    msg_paddy_collect->candlc = 1;
    msg_paddy_collect->candata[0] = 1;
    _pub_canusb->publish(*msg_paddy_collect);
}