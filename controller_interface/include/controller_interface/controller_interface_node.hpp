#pragma once
#include <rclcpp/rclcpp.hpp>

#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "controller_interface_msg/msg/sub_pad.hpp"
#include "controller_interface_msg/msg/sub_scrn.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

#include "utilities/can_utils.hpp"
#include "trapezoidal_velocity_planner.hpp"
#include "my_visibility.h"
#include "config.hpp"

using TraVelPlanner = velocity_planner::trapezoidal_velocity_planner::TrapezoidalVelocityPlanner;
using TraVelPlannerLimit = velocity_planner::trapezoidal_velocity_planner::Limit_t;
using VelPlanner = velocity_planner::trapezoidal_velocity_planner::VelPlanner;

namespace controller_interface
{
    class ControllerInterface : public rclcpp::Node
    {
        public:
            explicit ControllerInterface(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
            explicit ControllerInterface(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        private:
            rclcpp::Subscription<controller_interface_msg::msg::SubPad>::SharedPtr _sub_pad;
            rclcpp::Subscription<controller_interface_msg::msg::SubScrn>::SharedPtr _sub_scrn;
            //rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_scrn;

            rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_linear;
            rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_angular;
            rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr _pub_reset;
            rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr _pub_emergency;

            rclcpp::QoS _qos = rclcpp::QoS(40).keep_all();

            void callback_pad(const controller_interface_msg::msg::SubPad::SharedPtr msg);
            void callback_scrn(const controller_interface_msg::msg::SubScrn::SharedPtr msg);
            //void callback_scrn(const std_msgs::msg::String::SharedPtr msg);
            void callback_reset(const std_msgs::msg::Empty::SharedPtr msg);
            void callback_emergency(const std_msgs::msg::Empty::SharedPtr msg);
            float roundoff(const float &value, const float &epsilon);

            float anl_lft_x = 0.0f;
            float anl_lft_y = 0.0f;
            float anl_rgt_x = 0.0f;
            float anl_rgt_y = 0.0f;

            float max_linear_x = 1.0f;
            float max_linear_y = 1.0f;
            float max_angular_z = 1.0f;
            
            bool flag = 0;
    };
}