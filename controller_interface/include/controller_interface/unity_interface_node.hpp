#pragma onece
#include <rclcpp/rclcpp.hpp>
#include <float.h>
#include <string>
//使うmsg
#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "controller_interface_msg/msg/base_control.hpp"
#include "controller_interface_msg/msg/convergence.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
//他のpkg
#include "utilities/can_utils.hpp"
#include "utilities/utils.hpp"
#include "socket_udp.hpp"
#include "trapezoidal_velocity_planner.hpp"

#include "visibility_control.h"

namespace controller_interface
{
    using VelPlanner = velocity_planner::trapezoidal_velocity_planner::TrapezoidalVelocityPlanner;
    using VelPlannerLimit = velocity_planner::trapezoidal_velocity_planner::Limit_t;

    class Unity : public rclcpp::Node
    {
        public:
            CONTROLLER_INTERFACE_PUBLIC
            explicit Unity(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
            
            CONTROLLER_INTERFACE_PUBLIC
            explicit Unity(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        private:

            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub_initial_state;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_base_restart;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_base_emergency;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_move_auto;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_base_arm;
            
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_con_spline_convergence;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_con_arm_convergence;
            rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr _pub_base_state_communication;
            
            rclcpp::Subscription<controller_interface_msg::msg::Convergence>::SharedPtr _sub_convergence_unity;
            rclcpp::Subscription<controller_interface_msg::msg::BaseControl>::SharedPtr _sub_unity;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_initial_state;


            rclcpp::TimerBase::SharedPtr _pub_state_communication_timer;
            rclcpp::TimerBase::SharedPtr _pub_timer_convergence;

            void unity_callback(const controller_interface_msg::msg::BaseControl::SharedPtr msg);
            void convergence_unity_callback(const controller_interface_msg::msg::Convergence::SharedPtr msg);
            void callback_initial_state(const std_msgs::msg::String::SharedPtr msg);

            //base_control用
            bool is_reset = false;
            bool is_emergency = false;
            bool is_move_autonomous = false;
            bool is_slow_speed = false;
            std::string initial_state = "";

            bool spline_convergence = false;
            bool arm_convergence = false;
            bool arm_flag = false;

            //unityにpublish
            bool is_reset_unity = false;
            bool is_emergency_unity = false;
            bool is_move_autonomous_unity = false;
            bool is_slow_speed_unity = false;
            std::string initial_state_unity = "";

            //初期設定用
            bool defalt_restart_flag;
            bool defalt_move_autonomous_flag;
            bool defalt_emergency_flag;
            bool defalt_slow_speed_flag;
            bool defalt_spline_convergence;
            bool defalt_arm_convergence;

            std_msgs::msg::Bool msg_unity_control;
            std_msgs::msg::String msg_unity_initial_state;



            //QoS
            rclcpp::QoS _qos = rclcpp::QoS(10);

    };
}