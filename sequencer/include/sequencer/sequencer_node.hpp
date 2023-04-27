#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "controller_interface_msg/msg/base_control.hpp"
#include "controller_interface_msg/msg/convergence.hpp"

#include "sequencer/visibility_control.h"

namespace sequencer{

class Sequencer : public rclcpp::Node {
public:
    SEQUENCER_PUBLIC
    explicit Sequencer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    SEQUENCER_PUBLIC
    explicit Sequencer(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    rclcpp::Subscription<controller_interface_msg::msg::BaseControl>::SharedPtr _subscription_base_control;
    rclcpp::Subscription<controller_interface_msg::msg::Convergence>::SharedPtr _subscription_convergence;
    rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _subscription_movable;

    void _subscriber_callback_base_control(const controller_interface_msg::msg::BaseControl::SharedPtr msg);
    void _subscriber_callback_convergence(const controller_interface_msg::msg::Convergence::SharedPtr msg);
    void _subscriber_callback_movable(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
    void _recv_robot_state(const unsigned char data[2]);
    void _recv_pole_state(const unsigned char data[11]);

    rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr publisher_can;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_pole_m0;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_pole_m1;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_move_node;

    rclcpp::QoS _qos = rclcpp::QoS(10);

    const int16_t can_movable_id;
    const int16_t can_digital_button_id;
    const int16_t can_inject_id;

    controller_interface_msg::msg::Convergence judge_convergence;

    std::string current_pickup_state;
    std::string current_inject_state;

    const std::string pole_priorityA_file_path;   //状態Aでのポール優先のファイルパス
    const std::string pole_priorityB_file_path;   //状態Bでのポール優先のファイルパス
    const std::string pole_priorityC_file_path;   //状態Cでのポール優先のファイルパス

    std::vector<int> pole_priority_m0;
    std::vector<int> pole_priority_m1;

    int last_pole_num_m0 = 11;   //最後に狙ったポール(最初は入らないように11で初期化)
    int last_pole_num_m1 = 11;
    unsigned char last_pole_state[11] = {0,0,0,0,0,0,0,0,0,0,0};
};

}  // namespace sequencer
