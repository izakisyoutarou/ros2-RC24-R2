#include "controller_interface/unity_interface_node.hpp"
#include <sys/time.h>
#include <sys/types.h>
#include <chrono>
#include <iostream>
#include <future>

namespace controller_interface
{
    using std::string;

    Unity::Unity(const rclcpp::NodeOptions &options) : Unity("",options) {}
    Unity::Unity(const std::string &name_space, const rclcpp::NodeOptions &options): rclcpp::Node("unity_interface_node", name_space, options),
    //リスタート
    defalt_restart_flag(get_parameter("defalt_restart_flag").as_bool()),
    //緊急停止
    defalt_emergency_flag(get_parameter("defalt_emergency_flag").as_bool()),
    //自動化
    defalt_move_autonomous_flag(get_parameter("defalt_move_autonomous_flag").as_bool()),
    //経路収束
    defalt_spline_convergence(get_parameter("defalt_spline_convergence").as_bool()),
    //アーム収束
    defalt_arm_convergence(get_parameter("defalt_arm_convergence").as_bool())
    {

        const auto convergence_ms = this->get_parameter("convergence_ms").as_int();

        _sub_convergence_unity = this->create_subscription<controller_interface_msg::msg::Convergence>(
            "convergence",
            _qos,
            std::bind(&Unity::convergence_unity_callback, this, std::placeholders::_1)
        );
        _sub_unity = this->create_subscription<controller_interface_msg::msg::BaseControl>(
            "base_control",
            _qos,
            std::bind(&Unity::unity_callback, this, std::placeholders::_1)
        );
        _sub_initial_state = this->create_subscription<std_msgs::msg::String>(
            "initial_state",
            _qos,
            std::bind(&Unity::callback_initial_state, this, std::placeholders::_1)
        );
        _pub_initial_state = this->create_publisher<std_msgs::msg::String>("initial_state_unity", _qos);
        _pub_base_restart = this->create_publisher<std_msgs::msg::Bool>("restart_unity", _qos);
        _pub_base_emergency = this->create_publisher<std_msgs::msg::Bool>("emergency_unity", _qos);
        _pub_move_auto = this->create_publisher<std_msgs::msg::Bool>("move_autonomous_unity", _qos);
        _pub_con_spline_convergence = this->create_publisher<std_msgs::msg::Bool>("spline_convergence_unity", _qos);
        _pub_con_arm_convergence = this->create_publisher<std_msgs::msg::Bool>("arm_convergence_unity", _qos);
        _pub_con_net_convergence = this->create_publisher<std_msgs::msg::Bool>("net_convergence_unity", _qos);


        this->is_reset = defalt_restart_flag;
        this->is_emergency = defalt_emergency_flag;
        this->is_move_autonomous = defalt_move_autonomous_flag;
        this->initial_state = "O";
        this->spline_convergence = defalt_spline_convergence;
        this->arm_convergence = defalt_arm_convergence;
        this->net_convergence = defalt_net_convergence;


        auto msg_unity_initial_state = std::make_shared<std_msgs::msg::String>();
        msg_unity_initial_state->data = initial_state;
        _pub_initial_state->publish(*msg_unity_initial_state);

        auto msg_unity_control = std::make_shared<std_msgs::msg::Bool>();
        msg_unity_control->data = is_reset;
        _pub_base_restart->publish(*msg_unity_control);

        msg_unity_control->data = is_emergency;
        _pub_base_emergency->publish(*msg_unity_control);

        msg_unity_control->data = is_move_autonomous;
        _pub_move_auto->publish(*msg_unity_control);

        msg_unity_control->data = spline_convergence;
        _pub_con_spline_convergence->publish(*msg_unity_control);

        msg_unity_control->data = arm_convergence;
        _pub_con_arm_convergence->publish(*msg_unity_control);

        msg_unity_control->data = net_convergence;
        _pub_con_net_convergence->publish(*msg_unity_control);

    }

    void Unity::unity_callback(const controller_interface_msg::msg::BaseControl::SharedPtr msg){

        msg_unity_control.data = msg->is_restart;
        _pub_base_restart->publish(msg_unity_control);

        msg_unity_control.data = msg->is_emergency;
        _pub_base_emergency->publish(msg_unity_control);

        msg_unity_control.data = msg->is_move_autonomous;
        _pub_move_auto->publish(msg_unity_control);
    }

    void Unity::convergence_unity_callback(const controller_interface_msg::msg::Convergence::SharedPtr msg){
        auto msg_unity_control = std::make_shared<std_msgs::msg::Bool>();
        msg_unity_control->data = msg->spline_convergence;
        _pub_con_spline_convergence->publish(*msg_unity_control);
        msg_unity_control->data = msg->arm_convergence;
        _pub_con_arm_convergence->publish(*msg_unity_control);
        msg_unity_control->data = msg->net_convergence;
        _pub_con_net_convergence->publish(*msg_unity_control);
    }

    void Unity::callback_initial_state(const std_msgs::msg::String::SharedPtr msg){
        initial_state = msg->data[0];
        auto msg_unity_initial_state = std::make_shared<std_msgs::msg::String>();
        msg_unity_initial_state->data = initial_state;
        _pub_initial_state->publish(*msg_unity_initial_state);
    }
}