#pragma once
#include <rclcpp/rclcpp.hpp>
#include <float.h>
#include <string>
//使うmsg
#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "controller_interface_msg/msg/base_control.hpp"
#include "controller_interface_msg/msg/sub_pad.hpp"
#include "controller_interface_msg/msg/sub_scrn.hpp"
#include "controller_interface_msg/msg/convergence.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
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
            explicit SmartphoneGamepad(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

            CONTROLLER_INTERFACE_PUBLIC
            explicit SmartphoneGamepad(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        private:
            //ER_mainのcontrollerから
            rclcpp::Subscription<controller_interface_msg::msg::SubPad>::SharedPtr _sub_pad_er_main;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_move_node;
            //ER_subのcontrollerから
            rclcpp::Subscription<controller_interface_msg::msg::SubPad>::SharedPtr _sub_pad_er_sub;
            rclcpp::Subscription<controller_interface_msg::msg::SubScrn>::SharedPtr _sub_scrn_er_sub;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_injection_pole_m0;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_injection_pole_m1;
            //RRのcontrollerから
            rclcpp::Subscription<controller_interface_msg::msg::SubPad>::SharedPtr _sub_pad_rr;

            //mainボードから
            rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _sub_main_injection_possible;
            rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _sub_main_injection_complete;

            //spline_pidから
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_spline;

            //injection_param_calculatorから
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_injection_calculator_er_left;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_injection_calculator_er_right;

            //common_processから
            rclcpp::Subscription<controller_interface_msg::msg::BaseControl>::SharedPtr _sub_common_base_control;

            //CanUsbへ
            rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb;

            //controllerへ
            rclcpp::Publisher<controller_interface_msg::msg::Convergence>::SharedPtr _pub_convergence;
            rclcpp::Publisher<controller_interface_msg::msg::SubScrn>::SharedPtr _pub_scrn;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub_injection_pole_m0;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub_injection_pole_m1;
            //各nodeへリスタートと手自動の切り替えをpub
            rclcpp::Publisher<controller_interface_msg::msg::BaseControl>::SharedPtr _pub_common_base_control;

            //

            //test用のpub
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_test;

            //timer
            rclcpp::TimerBase::SharedPtr _pub_timer;
            rclcpp::TimerBase::SharedPtr _pub_timer_convergence;

            //QoS
            rclcpp::QoS _qos = rclcpp::QoS(10);

            //共有のやつ
            std_msgs::msg::String::SharedPtr pole_m0_data;
            std_msgs::msg::String::SharedPtr pole_m1_data;

            //controllerからのcallback
            void callback_pad_er_main(const controller_interface_msg::msg::SubPad::SharedPtr msg);
            void callback_pad_er_sub(const controller_interface_msg::msg::SubPad::SharedPtr msg);
            void callback_pad_rr(const controller_interface_msg::msg::SubPad::SharedPtr msg);

            void callback_move_node(const std_msgs::msg::String::SharedPtr msg);
            void callback_injection_pole_m0(const std_msgs::msg::String::SharedPtr msg);
            void callback_injection_pole_m1(const std_msgs::msg::String::SharedPtr msg);
            void callback_scrn_er_sub(const controller_interface_msg::msg::SubScrn::SharedPtr msg);

            void callback_udp_er_main(int sockfd);
            void callback_udp_er_sub(int sockfd);
            void callback_udp_rr(int sockfd);

            //common_processからのcallback
            void callback_common_base_control(const controller_interface_msg::msg::BaseControl::SharedPtr msg);

            //mainからのcallback
            void callback_main(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
            void callback_injection_complete(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);

            //splineからのcallback
            void callback_spline(const std_msgs::msg::Bool::SharedPtr msg);

            //injection_param_calculatorからのcallback
            void callback_injection_calculator_er_left(const std_msgs::msg::Bool::SharedPtr msg);
            void callback_injection_calculator_er_right(const std_msgs::msg::Bool::SharedPtr msg);

            //base_control用
            bool is_reset = false;
            bool is_emergency = false;
            bool is_wheel_autonomous = false;
            bool is_injection_autonomous = false;
            bool is_injection_m0 = false;

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
            const bool defalt_restart_flag;
            const bool defalt_wheel_autonomous_flag;
            const bool defalt_injection_autonomous_flag;
            const bool defalt_emergency_flag;
            const bool defalt_injection_m0_flag;
            const int udp_port_ER_main;
            const int udp_port_ER_sub;
            const int udp_port_RR;
            const int udp_timeout_ms = 20;

            int convergence_ms = 100;
            int s_num = 1;
            bool emergency_flag;
            std::string move_node;
            std::string pole_data_m0;
            std::string pole_data_m1;

            //UDP用
            int sockfd, n;
            socklen_t clilen;
            char* buffer = new char[16];
            struct sockaddr_in servaddr, cliaddr;
            std::thread udp_thread_;

            int sockfd2, n2;
            socklen_t clilen2;
            char* buffer2 = new char[16];
            struct sockaddr_in servaddr2, cliaddr2;
            std::thread udp_thread_2;

            int sockfd3, n3;
            socklen_t clilen3;
            char* buffer3 = new char[16];
            struct sockaddr_in servaddr3, cliaddr3;
            std::thread udp_thread_3;

            //計画機
            VelPlanner velPlanner_linear_x;
            VelPlanner velPlanner_linear_y;
            const VelPlannerLimit limit_linear;

            VelPlanner velPlanner_angular_z;
            const VelPlannerLimit limit_angular;

            VelPlanner velPlanner_injection_v;
            const VelPlannerLimit limit_injection;
    };

    class CommonProces : public rclcpp::Node
    {
        public:
            CONTROLLER_INTERFACE_PUBLIC
            explicit CommonProces(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

            CONTROLLER_INTERFACE_PUBLIC
            explicit CommonProces(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        private:
            //controllerから
            rclcpp::Subscription<controller_interface_msg::msg::SubScrn>::SharedPtr _sub_tcp1_scrn;

            //controller_interfaceから
            rclcpp::Subscription<controller_interface_msg::msg::BaseControl>::SharedPtr _sub_tcp1_base_control;

            //controllへ
            rclcpp::Publisher<controller_interface_msg::msg::SubScrn>::SharedPtr _pub_tcp1_scrn;

            rclcpp::Publisher<controller_interface_msg::msg::BaseControl>::SharedPtr _pub_tcp1_base_control;

            //timer
            rclcpp::TimerBase::SharedPtr _pub_timer;

            rclcpp::QoS _qos = rclcpp::QoS(10);

            void callback_scrn(const controller_interface_msg::msg::SubScrn::SharedPtr msg);

            void callback_base_contol_ER_main(const controller_interface_msg::msg::BaseControl::SharedPtr msg);

            void callback_base_contol_ER_sub(const controller_interface_msg::msg::BaseControl::SharedPtr msg);

            void callback_base_contol_RR(const controller_interface_msg::msg::BaseControl::SharedPtr msg);

            void assignment_base_control_ER();

            bool sub_scrn[11] = {false};
            bool sub_base_control[5] = {false};
            bool reset_flag = false;
    };
}
