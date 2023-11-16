#pragma once
#include "sequencer/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <string>
#include "std_msgs/msg/bool.hpp"
#include "controller_interface_msg/msg/convergence.hpp"
#include "controller_interface_msg/msg/colorball.hpp"

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

    void callback_seedling_collection(const std_msgs::msg::Bool::SharedPtr msg);
    void callback_seedling_installation(const std_msgs::msg::Bool::SharedPtr msg);
    void callback_ball_collection(const std_msgs::msg::Bool::SharedPtr msg);
    void callback_convergence(const controller_interface_msg::msg::Convergence::SharedPtr msg);
    void callback_color_information(const controller_interface_msg::msg::Colorball::SharedPtr msg);

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _publisher_in_process;

    void set_in_process(bool flag);

    //QoS
    rclcpp::QoS _qos = rclcpp::QoS(10);

    bool in_process = false;
    int seedling_step = 0;
    int planting_step = 0;
    int harvesting_step = 0;
    int sequence_process = 0;

    int selsect_algorithm = 0;
    std::string seedling_order[4] = {"S0", "S1", "S2", "S3"};
    std::string planting_order[8] = {"P0", "P1", "P2", "P3", "P4", "P5", "P6", "P7"};
    std::string harvesting_order[12] = {"H0", "H1", "H2", "H3", "H4", "H5", "H6", "H7", "H8", "H9", "H10", "H11"};

    enum class SEQUENCE_MODE{
        stop,
        seedling,
        planting,
        harvesting
    } sequence_mode = SEQUENCE_MODE::stop;

};

}  // namespace sequencer