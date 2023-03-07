#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "injection_interface_msg/msg/injection_command.hpp"
#include "utilities/two_vector.hpp"

#include "injection_interface/visibility_control.h"

namespace injection_interface{

class InjectionInterface : public rclcpp::Node {
    using TwoVector = utils::TwoVectord;
public:
    INJECTION_INTERFACE_PUBLIC
    explicit InjectionInterface(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(), const int mech_num=0);

    INJECTION_INTERFACE_PUBLIC
    explicit InjectionInterface(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions(), const int mech_num=0);

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscription_pole;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr _subscription_self_pose;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr _subscription_move_target_pose;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _subscription_is_move_tracking;

    void _subscriber_callback_pole(const std_msgs::msg::String::SharedPtr msg);
    void _subscriber_callback_self_pose(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void _subscriber_callback_move_target_pose(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void _subscriber_callback_is_move_tracking(const std_msgs::msg::Bool::SharedPtr msg);

    rclcpp::QoS _qos = rclcpp::QoS(10);

    rclcpp::Publisher<injection_interface_msg::msg::InjectionCommand>::SharedPtr publisher_injection;

    //定数
    const int mech_num;
    const std::vector<double> tf_injection2robot;

    const std::string pole_file_path;   //ポールコンフィグのパス

    //フィールド
    geometry_msgs::msg::Vector3 self_pose;
    geometry_msgs::msg::Vector3 move_target_pose;
    bool is_move_tracking = false;
    bool is_correction_required = false;
    std::shared_ptr<std_msgs::msg::String> last_target;



};

}  // namespace injection_interface
