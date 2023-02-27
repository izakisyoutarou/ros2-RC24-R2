#pragma once
#include <rclcpp/rclcpp.hpp>
#include <float.h>
#include <string>
//使うmsg
#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "controller_interface_msg/msg/robot_controll.hpp"
#include "controller_interface_msg/msg/sub_pad.hpp"
#include "controller_interface_msg/msg/sub_scrn.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
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

#define BUFSIZE 1024

using VelPlanner = velocity_planner::trapezoidal_velocity_planner::TrapezoidalVelocityPlanner;
using VelPlannerLimit = velocity_planner::trapezoidal_velocity_planner::Limit_t;

using std::string;

namespace controller_interface
{
    class SmartphoneGamepad : public rclcpp::Node
    {
        public:
            CONTROLLER_INTERFACE_PUBLIC
            explicit SmartphoneGamepad(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
            
            CONTROLLER_INTERFACE_PUBLIC
            explicit SmartphoneGamepad(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        private:
            //controllerから
            rclcpp::Subscription<controller_interface_msg::msg::SubPad>::SharedPtr _sub_pad;
            rclcpp::Subscription<controller_interface_msg::msg::SubScrn>::SharedPtr _sub_scrn;
            //CanUsbへ
            rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_restart;
            rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_emergency;
            rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_linear;
            rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_angular;
            //経路生成・計画へ
            rclcpp::Publisher<controller_interface_msg::msg::RobotControll>::SharedPtr _pub_route;
            //上モノインターフェイスへ
            rclcpp::Publisher<controller_interface_msg::msg::RobotControll>::SharedPtr _pub_tool;
            //gazebo_simulatorへ
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _pub_gazebo;
            //QoS
            rclcpp::QoS _qos = rclcpp::QoS(10);

            void callback_pad(const controller_interface_msg::msg::SubPad::SharedPtr msg);
            void callback_scrn(const controller_interface_msg::msg::SubScrn::SharedPtr msg);
            void callback_udp();

            double upcast(float value2);
            float roundoff(const float &value, const float &epsilon);

            float analog_l_x = 0.0f;
            float analog_l_y = 0.0f;
            float analog_r_x = 0.0f;
            float analog_r_y = 0.0f;

            float max_linear_x = 2.0f;
            float max_linear_y = 2.0f;
            float max_angular_z = 2.0f;

            //utility用
            uint8_t _candata_joy[8];
            uint8_t _candata_btn;

            //empty
            std_msgs::msg::Empty empty;

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

            enum class Mode {
		        manual,
		        automatic
	        } mode = Mode::manual;
            

    };

    class DualSense : public rclcpp::Node
    {
        public:
            CONTROLLER_INTERFACE_PUBLIC
            explicit DualSense(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
            CONTROLLER_INTERFACE_PUBLIC
            explicit DualSense(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        private:
            float x = 0.f;
    };
}
