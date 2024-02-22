#pragma once
#include "sequencer/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <geometry_msgs/msg/vector3.hpp>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "controller_interface_msg/msg/convergence.hpp"
#include "controller_interface_msg/msg/base_control.hpp"
#include "socketcan_interface_msg/msg/socketcan_if.hpp"

namespace sequencer{

class Sequencer : public rclcpp::Node {
public:

    SEQUENCER_PUBLIC
    explicit Sequencer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    SEQUENCER_PUBLIC
    explicit Sequencer(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:

    enum class SEQUENCE_MODE{
        stop,
        storage,
        transfer,
        collect,
        silo
    } sequence_mode = SEQUENCE_MODE::stop;

    rclcpp::Subscription<controller_interface_msg::msg::Convergence>::SharedPtr _subscription_convergence;
    rclcpp::Subscription<controller_interface_msg::msg::BaseControl>::SharedPtr _subscription_base_control;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr _subscription_is_start;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscription_collection_point;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr _subscription_self_pose;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _subscription_front_ball;

    void callback_convergence(const controller_interface_msg::msg::Convergence::SharedPtr msg);
    void callback_base_control(const controller_interface_msg::msg::BaseControl::SharedPtr msg);
    void callback_is_start(const std_msgs::msg::UInt8::SharedPtr msg);
    void callback_collection_point(const std_msgs::msg::String::SharedPtr msg);
    void callback_self_pose(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void callback_front_ball(const std_msgs::msg::Bool::SharedPtr msg);
    void callback(const std_msgs::msg::String::SharedPtr msg);

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher_move_node;
    rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _publisher_canusb;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher_way_point;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher_now_sequence;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher_move_interrupt_node;

    void command_move_node(const std::string node);
    void command_move_interrupt_node(const std::string node);
    void command_sequence(SEQUENCE_MODE sequence);
    void command_canusb_uint8(const int16_t id, const uint8_t data);
    void command_canusb_empty(const int16_t id);
    void command_paddy_collect_front();
    void command_paddy_collect_back();
    void command_paddy_install();
    void command_net_open();
    void command_net_close();

    const int16_t can_paddy_collect_id;
    const int16_t can_paddy_install_id;
    const int16_t can_net_id;

    //QoS
    rclcpp::QoS _qos = rclcpp::QoS(10);

    struct Node{
        std::string name;
        double x;
        double y;
    };

    std::vector<Node> node_list;
    std::string sequence_list[5] = {"stop","storage","transfer","collect","silo"};

    int progress = 0;
    int select_silo = 0;
    int ball_num = 0;

    bool is_start = false;
    bool get_front_ball = false;
    bool front_ball = false;

    std::string way_point = "O";

    geometry_msgs::msg::Vector3 self_pose;
    
};

}  // namespace sequencer