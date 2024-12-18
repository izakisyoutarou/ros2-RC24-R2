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

void Gamebtn::board_reset(rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    cout<<"board_reset"<<endl;
    auto msg_board_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_board_reset->canid = canid.reset;
    msg_board_reset->candlc = 1;
    _pub_canusb->publish(*msg_board_reset);
}

void Gamebtn::steer_reset(rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    cout<<"steer_reset"<<endl;
    auto msg_steer_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_steer_reset->canid = canid.steer_reset;
    msg_steer_reset->candlc = 0;
    _pub_canusb->publish(*msg_steer_reset);
}

void Gamebtn::paddy_collect_0(bool is_arm_convergence,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(is_arm_convergence){
        cout<<"paddy_collect_0"<<endl;
        auto msg_paddy_collect = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_paddy_collect->canid = canid.paddy_collect;
        msg_paddy_collect->candlc = 1;
        msg_paddy_collect->candata[0] = 0;
        _pub_canusb->publish(*msg_paddy_collect);
    }
}

void Gamebtn::paddy_collect_1(bool is_arm_convergence,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(is_arm_convergence){
        cout<<"paddy_collect_1"<<endl;
        auto msg_paddy_collect = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_paddy_collect->canid = canid.paddy_collect;
        msg_paddy_collect->candlc = 1;
        msg_paddy_collect->candata[0] = 1;
        _pub_canusb->publish(*msg_paddy_collect);
    }
}

void Gamebtn::paddy_collect_2(bool is_arm_convergence,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(is_arm_convergence){
        cout<<"paddy_collect_2"<<endl;
        auto msg_paddy_collect = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_paddy_collect->canid = canid.paddy_collect;
        msg_paddy_collect->candlc = 1;
        msg_paddy_collect->candata[0] = 2;
        _pub_canusb->publish(*msg_paddy_collect);
    }
}

void Gamebtn::paddy_install(bool is_arm_convergence,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(is_arm_convergence){
        cout<<"paddy_install"<<endl;
        auto msg_paddy_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_paddy_install->canid = canid.paddy_install;
        msg_paddy_install->candlc = 0;
        _pub_canusb->publish(*msg_paddy_install);
    }
}

void Gamebtn::net_open(bool is_net_convergence,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(is_net_convergence){
        cout<<"net_open"<<endl;
        auto msg_net_open = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_net_open->canid = canid.net;
        msg_net_open->candlc = 1;
        msg_net_open->candata[0] = 0;
        _pub_canusb->publish(*msg_net_open);
    }
}

void Gamebtn::net_close(bool is_net_convergence,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(is_net_convergence){
        cout<<"net_close"<<endl;
        auto msg_net_close = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_net_close->canid = canid.net;
        msg_net_close->candlc = 1;
        msg_net_close->candata[0] = 1;
        _pub_canusb->publish(*msg_net_close);
    }
}

void Gamebtn::canusb_test(const int16_t id, const uint8_t data, rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
        cout<<"id:"<<id<<", "<<"data:"<<data<<endl;
        auto msg_test = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_test->canid = id;
        msg_test->candlc = 1;
        msg_test->candata[0] = data;
        _pub_canusb->publish(*msg_test);   
}