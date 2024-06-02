#pragma once
#include "sequencer/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <chrono>
#include <geometry_msgs/msg/vector3.hpp>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"
#include "controller_interface_msg/msg/convergence.hpp"
#include "controller_interface_msg/msg/base_control.hpp"
#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "detection_interface_msg/msg/silo_param.hpp"
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
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscription_way_point;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr _subscription_self_pose;
    rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _subscription_tof;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr _subscription_ball_coordinate;
    rclcpp::Subscription<detection_interface_msg::msg::SiloParam>::SharedPtr _subscription_silo_param;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscription_suction_check;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _subscription_move_progress;

    void callback_convergence(const controller_interface_msg::msg::Convergence::SharedPtr msg);
    void callback_base_control(const controller_interface_msg::msg::BaseControl::SharedPtr msg);
    void callback_is_start(const std_msgs::msg::UInt8::SharedPtr msg);
    void callback_collection_point(const std_msgs::msg::String::SharedPtr msg);
    void callback_way_point(const std_msgs::msg::String::SharedPtr msg);
    void callback_self_pose(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void callback_tof(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
    void callback_ball_coordinate(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void callback_silo_param(const detection_interface_msg::msg::SiloParam::SharedPtr msg);
    void callback_suction_check(const std_msgs::msg::String::SharedPtr msg);
    void callback_move_progress(const std_msgs::msg::Float64::SharedPtr msg);

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher_move_node;
    rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _publisher_canusb;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher_now_sequence;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher_move_interrupt_node;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr _publisher_coord_tracking;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _publisher_move_autonomous;

    void command_move_node(const std::string node);//node位置に通常移動
    void command_move_interrupt_node(const std::string node);//node位置に割り込み直線移動
    void command_ball_tracking();
    void command_sequence(SEQUENCE_MODE sequence);
    void command_canusb_uint8(const int16_t id, const uint8_t data);//テンプレート
    void command_canusb_empty(const int16_t id);//テンプレート
    void command_net_open();//ネット開く(固定)
    void command_net_close();//ネット閉じる(自由)
    void command_hand_suction_on();//吸引on
    void command_hand_suction_off();//吸引off

    void command_base_state();
    void command_strage_state(bool front_ball);
    void command_strage_state2();
    void command_transfer_state();
    void command_silo_state();
    void command_silo_state2();

    void command_hand_lift_pickup();//アーム高さ_中央

    void silo_evaluate(std::string camera[15]);
    void command_move_autonomous(bool flag);
    void timer(int ms);
    bool timer();

    bool check_way_ST();
    bool check_way_SI();
    bool check_way_si();

    const int16_t can_net_id;
    const int16_t can_tof_id;
    const int16_t can_hand_suction_id;
    const int16_t can_base_state_id;
    const int16_t can_strage_state_id;
    const int16_t can_strage_state2_id;
    const int16_t can_transfer_state_id;
    const int16_t can_silo_state_id;
    const int16_t can_silo_state2_id;
    const int16_t can_hand_lift_id;

    //QoS
    rclcpp::QoS _qos = rclcpp::QoS(10);

    std::string sequence_list[5] = {"stop","storage","transfer","collect","silo"};

    int progress = 0;
    int select_silo = 0;
    int camera_check[5][3] = {};
    int silo_priority[5] = {}; 
    int check_time = 0;
    int target_silo = 0;
    int priority_num = 0;
    int ball_num = 0;

    const std::vector<double> strage_dist;
    const double suction_wait; 
    const double silo_wait; 

    bool is_start = false;
    bool get_suction_check = false;
    bool get_ball_pose = false;
    bool tof[4] = {false, false, false, false};
    bool silo_flag = false;
    bool c3orc6_flag = false; 
    bool c1_flag = false; 
    bool c3a_flag = false;
    bool c3b_flag = false;
    bool c6a_flag = false;
    bool c6b_flag = false;
    bool retry_flag = false;
    bool move_progress = false;
    bool net_flag = false;
    bool kado_flag = false;
    bool move_automonous = false;

    geometry_msgs::msg::Vector3 self_pose;
    geometry_msgs::msg::Vector3 ball_pose;

    const std::string court_color;
    std::string silo_data[5][3]; 
    std::string silo_norm[11][4];//3,2,1,num
    std::string way_point = "";
    std::string interrupt_node = "";
    std::string pre_interrupt_node = "";
    std::string move_node = "";
    std::string pre_move_node = "";
    std::string suction_check = "";

    SEQUENCE_MODE pre_sequence = SEQUENCE_MODE::stop;

    std::chrono::system_clock::time_point time;
};

}  // namespace sequencer