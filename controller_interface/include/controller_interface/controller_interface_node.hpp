#pragma once
#include <rclcpp/rclcpp.hpp>
#include <float.h>
#include <string>
//使うmsg
#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "controller_interface_msg/msg/base_control.hpp"
#include "controller_interface_msg/msg/convergence.hpp"
#include "controller_interface_msg/msg/colorball.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
//他のpkg
#include "utilities/can_utils.hpp"
#include "utilities/utils.hpp"
#include "socket_udp.hpp"
#include "trapezoidal_velocity_planner.hpp"

#include "send_udp.hpp"
#include "super_command.hpp"

#include "visibility_control.h"


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
            //R2_mainのcontrollerから
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_main_pad;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_screen_pad;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_state_num_R2;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_initial_state;

            //R2_subのcontrollerから
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_pad_sub;

            //mainボードから
            rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _sub_main_arm_possible;

            //spline_pidから
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_spline;

            //arm_param_calculatorから
            // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_arm_calculator;
            //sequencerから
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_arm_strange;

            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_collection_ball;

            //CanUsbへ
            rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb;

            //各nodeと共有
            rclcpp::Publisher<controller_interface_msg::msg::BaseControl>::SharedPtr _pub_base_control;
            rclcpp::Publisher<controller_interface_msg::msg::Convergence>::SharedPtr _pub_convergence;
            rclcpp::Publisher<controller_interface_msg::msg::Colorball>::SharedPtr _pub_color_ball_R2;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_arm;
            //ボールと苗の回収&設置
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub_seedling_collection;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub_seedling_installation;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub_ball_collection;
            
            //gazebo_simulator用のpub
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _pub_gazebo;

            //main_controlerからのsubscriber
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub_initial_state;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_base_restart;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_base_emergency;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_move_auto;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_base_arm;
            
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_con_spline;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_con_colcurator;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_con_arm;
            rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr _pub_base_state_communication;

            //sprine_pid
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub_move_node;

            //timer
            rclcpp::TimerBase::SharedPtr _pub_heartbeat;
            rclcpp::TimerBase::SharedPtr _pub_timer_convergence;
            rclcpp::TimerBase::SharedPtr _socket_timer;
            rclcpp::TimerBase::SharedPtr _start_timer;
            rclcpp::TimerBase::SharedPtr _pub_state_communication_timer;


            //QoS
            rclcpp::QoS _qos = rclcpp::QoS(10);
            
            //controller_mainからのcallback
            void callback_main_pad(const std_msgs::msg::String::SharedPtr msg);
            void callback_screen_pad(const std_msgs::msg::String::SharedPtr msg);
            void callback_coatstate_pad(const std_msgs::msg::Bool::SharedPtr msg);
            void callback_state_num_R2(const std_msgs::msg::String::SharedPtr msg);

            //controller_subからのcallback
            void callback_sub_pad(const std_msgs::msg::String::SharedPtr msg);

            //mainからのcallback
            void callback_main(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
            // void callback_arm_complete(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);

            //splineからのcallback
            void callback_spline(const std_msgs::msg::Bool::SharedPtr msg);

            //arm_param_calculatorからのcallback
            // void callback_arm_calculator(const std_msgs::msg::Bool::SharedPtr msg);
            // void callback_calculator_convergenced_arm(const std_msgs::Bool::SharedPtr msg);
            void callback_initial_state(const std_msgs::msg::String::SharedPtr msg);

            //sequencerからのcallback
            void callback_arm_strage(const std_msgs::msg::String::SharedPtr msg);
            void callback_collecting_ball(const std_msgs::msg::String::SharedPtr msg);
            void _recv_callback();


            void _recv_joy_main(const unsigned char data[16]);

            //メッセージ型の宣言
            controller_interface_msg::msg::BaseControl msg_base_control;
            controller_interface_msg::msg::Colorball msg_colorball_info;
            std_msgs::msg::Bool msg_unity_control;
            std_msgs::msg::Bool msg_unity_sub_control;
            std_msgs::msg::String msg_unity_initial_state;


            //base_control用
            bool is_reset = false;
            bool is_emergency = false;
            bool is_move_autonomous = false;
            bool is_arm_autonomous = false;
            bool is_slow_speed = false;
            bool is_arm_mech_stop_m = false;
            std::string initial_state = "";

            //unityにsubscrib
            bool is_reset_unity = false;
            bool is_emergency_unity = false;
            bool is_move_autonomous_unity = false;
            bool is_arm_autonomous_unity = false;
            bool is_slow_speed_unity = false;
            bool is_arm_mech_stop_m_unity = false;
            std::string initial_state_unity = "";
            
            bool spline_convergence = false;
            bool arm_calculator = false;
            bool arm = false;
            bool arm_flag = false;


            //canusb
            bool a;
            bool b;
            bool y;
            bool x;
            bool r1;
            bool r2;
            bool r3;
            bool l1;
            bool l2;
            bool l3;
            bool s;
            bool g;
            bool up;
            bool left;
            bool down;
            bool right;

            //convergence用
            bool is_spline_convergence;
            // bool is_arm_calculator_convergence;
            bool is_arm_convergence;

            //初期化指定用
            const float high_manual_linear_max_vel;
            const float slow_manual_linear_max_vel;
            const float manual_angular_max_vel;
            
            const bool defalt_restart_flag;
            const bool defalt_move_autonomous_flag;
            const bool defalt_arm_autonomous_flag;
            const bool defalt_emergency_flag;
            const bool defalt_slow_speed_flag;
            //subcontrollerのカラー情報
            const bool defalt_color_information_flag;

            const bool defalt_spline_convergence;
            const bool defalt_arm_calculator_convergence;
            const bool defalt_arm_convergence;
            
            const int16_t can_emergency_id;
            const int16_t can_heartbeat_id;
            const int16_t can_restart_id;
            const int16_t can_calibrate_id;
            const int16_t can_reset_id;
            const int16_t can_linear_id;
            const int16_t can_angular_id;
            const int16_t can_steer_reset_id;
            const int16_t can_paddy_collect_id;
            const int16_t can_paddy_install_id;
            const int16_t can_main_button_id;

            const std::string r1_pc;
            const std::string r2_pc;

            const std::string initial_pickup_state;
            const std::string initial_inject_state;

            //udp初期化用
            const int udp_port_state;
            const int udp_port_pole;
            const int udp_port_spline_state;

            bool start_r2_main;

            bool start_flag;

            //計画機
            VelPlanner high_velPlanner_linear_x;
            VelPlanner high_velPlanner_linear_y;
            const VelPlannerLimit high_limit_linear;

            VelPlanner slow_velPlanner_linear_x;
            VelPlanner slow_velPlanner_linear_y;
            const VelPlannerLimit slow_limit_linear;

            VelPlanner velPlanner_angular_z;
            const VelPlannerLimit limit_angular;

            send_udp send;
            super_command command;

            RecvUDP joy_main;
    };
}