#pragma once
#include "sequencer/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <string>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "controller_interface_msg/msg/convergence.hpp"
#include "controller_interface_msg/msg/colorball.hpp"
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

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _subscription_seedling_collection;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _subscription_seedling_installation;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _subscription_ball_collection;
    rclcpp::Subscription<controller_interface_msg::msg::Convergence>::SharedPtr _subscription_convergence;
    rclcpp::Subscription<controller_interface_msg::msg::Colorball>::SharedPtr _subscription_color_information;
    rclcpp::Subscription<controller_interface_msg::msg::BaseControl>::SharedPtr _subscription_base_control;

    void callback_seedling_collection(const std_msgs::msg::Bool::SharedPtr msg);
    void callback_seedling_installation(const std_msgs::msg::Bool::SharedPtr msg);
    void callback_ball_collection(const std_msgs::msg::Bool::SharedPtr msg);
    void callback_convergence(const controller_interface_msg::msg::Convergence::SharedPtr msg);
    void callback_color_information(const controller_interface_msg::msg::Colorball::SharedPtr msg);
    void callback_base_control(const controller_interface_msg::msg::BaseControl::SharedPtr msg);

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _publisher_in_process;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher_move_node;
    rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _publisher_canusb;

    void set_in_process(const bool flag);
    void command_canusb(const int16_t id, const uint8_t dlc, const uint8_t data[8]);
    void command_move_node(const std::string node);
    void command_seedling_collect(const uint8_t num);
    void command_seedling_install(const uint8_t num);
    void command_paddy_collect(const uint8_t num);
    void command_paddy_install(const uint8_t num);

    //QoS
    rclcpp::QoS _qos = rclcpp::QoS(10);

    const int16_t canid_inject_spinning;
    const int16_t canid_inject;
    const int16_t canid_seedling_collect;
    const int16_t canid_seedling_install;
    const int16_t canid_paddy_collect;
    const int16_t canid_paddy_install;

    bool in_process = false;
    int seedling_step = 0;
    int planting_step = 0;
    int harvesting_step = 0;
    int sequence_process = 0;

    const int select_algorithm;
    std::vector<std::string> seedling_order;
    std::vector<std::string> planting_order;
    std::vector<std::string> harvesting_order;

    enum class SEQUENCE_MODE{
        stop,
        seedling,
        planting,
        harvesting,
        injection
    } sequence_mode = SEQUENCE_MODE::stop;

};

}  // namespace sequencer