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
#include "path_msg/msg/turning.hpp"
#include "utilities/utils.hpp"

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
    rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _subscription_tof;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr _subscription_ball_coordinate;

    void callback_convergence(const controller_interface_msg::msg::Convergence::SharedPtr msg);
    void callback_base_control(const controller_interface_msg::msg::BaseControl::SharedPtr msg);
    void callback_is_start(const std_msgs::msg::UInt8::SharedPtr msg);
    void callback_collection_point(const std_msgs::msg::String::SharedPtr msg);
    void callback_self_pose(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void callback_front_ball(const std_msgs::msg::Bool::SharedPtr msg);
    void callback_tof(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
    void callback_ball_coordinate(const geometry_msgs::msg::Vector3::SharedPtr msg);

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher_move_node;
    rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _publisher_canusb;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher_now_sequence;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher_move_interrupt_node;
    rclcpp::Publisher<path_msg::msg::Turning>::SharedPtr _pub_spin_position;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr _publisher_ball_tracking;

    void command_move_node(const std::string node);
    void command_move_interrupt_node(const std::string node);
    void command_spin_position(const float angle);
    void command_sequence(SEQUENCE_MODE sequence);

    void command_canusb_uint8(const int16_t id, const uint8_t data);
    void command_canusb_empty(const int16_t id);
    void command_paddy_collect_front();
    void command_paddy_collect_back();
    void command_paddy_install();
    void command_net_open();
    void command_net_close();
    void command_hand_lift_suction_before();
    void command_hand_lift_suction();
    void command_hand_lift_pickup();
    void command_hand_lift_silo();
    void command_hand_fb_front();
    void command_hand_fb_back();
    void command_hand_fb_silo();
    void command_hand_wrist_up();
    void command_hand_wrist_down();
    void command_hand_suction_on();
    void command_hand_suction_off();
    int silo_evaluate(std::string camera[15]);

    const int16_t can_paddy_collect_id;
    const int16_t can_paddy_install_id;
    const int16_t can_net_id;
    const int16_t can_hand_lift_id;
    const int16_t can_hand_fb_id;
    const int16_t can_hand_wrist_id;
    const int16_t can_hand_suction_id;
    const int16_t can_tof_id;

    //QoS
    rclcpp::QoS _qos = rclcpp::QoS(10);

    std::string sequence_list[5] = {"stop","storage","transfer","collect","silo"};

    int progress = 0;
    int select_silo = 0;
    int ball_num = 0;

    bool is_start = false;
    bool get_front_ball = false;
    bool get_ball_pose = false;
    bool front_ball = false;

    geometry_msgs::msg::Vector3 self_pose;
    geometry_msgs::msg::Vector3 ball_pose;

    std::string silo_data[5][3]; 
    std::string silo_norm[11][4];//3,2,1,num
    int silo_priority[5]; 
    const std::string court_color;

    const std::string R2_state;
    
    bool tof[3] = {false, false, false};

    SEQUENCE_MODE pre_sequence = SEQUENCE_MODE::stop;
    
};

}  // namespace sequencer