#include "controller_interface/Gamebtn.hpp"
#include <rclcpp/rclcpp.hpp>

Gamebtn::Gamebtn(){}

void Gamebtn::calibrate(rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    cout<<"calibrate"<<endl;
    auto msg_calibrate = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_calibrate->canid = canid.calibrate;
    msg_calibrate->candlc = 0;
    _pub_canusb->publish(*msg_calibrate);
}

void Gamebtn::main_reset(rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    cout<<"main_reset"<<endl;
    auto msg_main_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_main_reset->canid = canid.reset;
    msg_main_reset->candlc = 1;
    msg_main_reset->candata[0] = 0;
    _pub_canusb->publish(*msg_main_reset);
}

void Gamebtn::io_reset(rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    cout<<"io_reset"<<endl;
    auto msg_io_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_io_reset->canid = canid.reset;
    msg_io_reset->candlc = 1;
    msg_io_reset->candata[0] = 1;
    _pub_canusb->publish(*msg_io_reset);
}

void Gamebtn::steer_reset(rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    cout<<"steer_reset"<<endl;
    auto msg_steer_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_steer_reset->canid = canid.steer_reset;
    msg_steer_reset->candlc = 0;
    _pub_canusb->publish(*msg_steer_reset);
}

void Gamebtn::paddy_collect_0(bool is_arm_convergence,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(!is_arm_convergence){
        cout<<"paddy_collect_0"<<endl;
        auto msg_paddy_collect = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_paddy_collect->canid = canid.paddy_collect;
        msg_paddy_collect->candlc = 1;
        msg_paddy_collect->candata[0] = 0;
        _pub_canusb->publish(*msg_paddy_collect);
    }
}

void Gamebtn::paddy_collect_1(bool is_arm_convergence,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(!is_arm_convergence){
        cout<<"paddy_collect_1"<<endl;
        auto msg_paddy_collect = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_paddy_collect->canid = canid.paddy_collect;
        msg_paddy_collect->candlc = 1;
        msg_paddy_collect->candata[0] = 1;
        _pub_canusb->publish(*msg_paddy_collect);
    }
}

void Gamebtn::paddy_install(bool is_arm_convergence,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(!is_arm_convergence){
        cout<<"paddy_install"<<endl;
        auto msg_paddy_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_paddy_install->canid = canid.paddy_install;
        msg_paddy_install->candlc = 0;
        _pub_canusb->publish(*msg_paddy_install);
    }
}