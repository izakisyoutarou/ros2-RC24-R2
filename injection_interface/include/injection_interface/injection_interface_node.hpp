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
    explicit InjectionInterface(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    INJECTION_INTERFACE_PUBLIC
    explicit InjectionInterface(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_aim_in_storage;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_is_move_tracking;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr _sub_self_pose;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr _sub_move_target_pose;

    void _callback_aim_in_storage(const std_msgs::msg::Bool::SharedPtr msg);
    void _callback_is_move_tracking(const std_msgs::msg::Bool::SharedPtr msg);
    void _callback_self_pose(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void _callback_move_target_pose(const geometry_msgs::msg::Vector3::SharedPtr msg);
    
    rclcpp::QoS _qos = rclcpp::QoS(10);

    rclcpp::Publisher<injection_interface_msg::msg::InjectionCommand>::SharedPtr _pub_injection;

    //定数
    const std::vector<double> tf_injection2robot;
    const std::vector<double> strage_backside;
    const std::vector<double> strage_front;

    const std::string pole_file_path;   //ポールコンフィグのパス

    //フィールド
    geometry_msgs::msg::Vector3 self_pose;
    geometry_msgs::msg::Vector3 move_target_pose;
    bool is_move_tracking = false;
    bool is_correction_required = false;
    std::shared_ptr<std_msgs::msg::Bool> last_target;
};

}