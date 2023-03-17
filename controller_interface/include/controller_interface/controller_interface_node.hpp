#pragma once
#include <rclcpp/rclcpp.hpp>
#include <float.h>
#include <string>
//使うmsg
#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "controller_interface_msg/msg/base_control.hpp"
#include "controller_interface_msg/msg/sub_pad.hpp"
#include "controller_interface_msg/msg/convergence.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
//他のpkg
#include "utilities/can_utils.hpp"
#include "utilities/utils.hpp"
#include "trapezoidal_velocity_planner.hpp"
#include "my_visibility.h"
//UDP
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <cstring>//memcpyのため

namespace controller_interface
{
    using VelPlanner = velocity_planner::trapezoidal_velocity_planner::TrapezoidalVelocityPlanner;
    using VelPlannerLimit = velocity_planner::trapezoidal_velocity_planner::Limit_t;

    class SmartphoneGamepad : public rclcpp::Node
    {
        public:
            CONTROLLER_INTERFACE_PUBLIC
            explicit SmartphoneGamepad(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(), const int tcp_endpoint_num = 0, const int udp_port = 0);
            
            CONTROLLER_INTERFACE_PUBLIC
            explicit SmartphoneGamepad(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions(), const int tcp_endpoint_num = 0, const int udp_port = 0);

        private:
            //controllerから
            rclcpp::Subscription<controller_interface_msg::msg::SubPad>::SharedPtr _sub_pad;
            //mainボードから
            rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _sub_main;
            //spline_pidから
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_spline;
            //injection_param_calculatorから
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_injection_calculator_er_left;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_injection_calculator_er_right;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_injection_calculator_rr;
            //CanUsbへ
            rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb;
            //controllerへ
            rclcpp::Publisher<controller_interface_msg::msg::Convergence>::SharedPtr _pub_convergence;
            //各nodeへリスタートと手自動の切り替えをpub
            rclcpp::Publisher<controller_interface_msg::msg::BaseControl>::SharedPtr _pub_base_control;
            //test用のpub
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_test;
            //timer
            rclcpp::TimerBase::SharedPtr _pub_timer;
            //QoS
            rclcpp::QoS _qos = rclcpp::QoS(10);

            //controllerからのcallback
            void callback_pad(const controller_interface_msg::msg::SubPad::SharedPtr msg);
            void callback_udp();
            //mainからのcallback
            void callback_main(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
            //splineからのcallback
            void callback_spline(const std_msgs::msg::Bool::SharedPtr msg);
            //injection_param_calculatorから
            void callback_injection_calculator_er_left(const std_msgs::msg::Bool::SharedPtr msg);
            void callback_injection_calculator_er_right(const std_msgs::msg::Bool::SharedPtr msg);
            void callback_injection_calculator_rr(const std_msgs::msg::Bool::SharedPtr msg);
            
            //base_control用
            bool is_wheel_autonomous;
            bool is_injection_autonomous;
            bool is_injection_m0;
            bool is_reset;
            bool is_emergency;

            //convergence用
            bool is_spline_convergence;
            bool is_injection_calculator0_convergence;
            bool is_injection_calculator1_convergence;
            bool is_injection0_convergence;
            bool is_injection1_convergence;

            //初期化指定用
            const float manual_linear_max_vel;
            const float manual_angular_max_vel;
            const float manual_injection_max_vel;
            const float defalt_pitch;
            const int udp_port;
            const int tcp_endpoint_num;
            const bool defalt_restart_flag;
            const bool defalt_wheel_autonomous_flag;
            const bool defalt_injection_autonomous_flag;
            const bool defalt_emergency_flag;

            //UDP用
            int sockfd, n;
            socklen_t clilen;
            char* buffer = new char[16];
            struct sockaddr_in servaddr, cliaddr;
            std::thread udp_thread_;

            //計画機
            VelPlanner velPlanner_linear_x;
            VelPlanner velPlanner_linear_y;
            const VelPlannerLimit limit_linear;

            VelPlanner velPlanner_angular_z;
            const VelPlannerLimit limit_angular;

            VelPlanner velPlanner_injection_v;
            const VelPlannerLimit limit_injection;
    };

    class DualSense : public rclcpp::Node
    {
        public:
            CONTROLLER_INTERFACE_PUBLIC
            explicit DualSense(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

            CONTROLLER_INTERFACE_PUBLIC
            explicit DualSense(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        private:
            //controllerから
            rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _sub_SPpad;
            rclcpp::QoS _qos = rclcpp::QoS(10);

            void callback_SPpad(const sensor_msgs::msg::Joy::SharedPtr msg);

            float x = 0.f;
            
            //計画機
            VelPlanner velPlanner_linear_x;
            VelPlanner velPlanner_linear_y;
            const VelPlannerLimit limit_linear;
            VelPlanner velPlanner_angular_z;
            const VelPlannerLimit limit_angular;
    };
}
