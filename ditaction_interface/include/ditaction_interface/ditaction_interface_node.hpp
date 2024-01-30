#pragma once
#include <rclcpp/rclcpp.hpp>
#include <float.h>
#include <string>
#include <math.h>
//使うmsg
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "bboxes_ex_msgs/msg/bounding_box.hpp"
#include "bboxes_ex_msgs/msg/bounding_boxes.hpp"
#include "ditaction_interface_msg/msg/arm_param.hpp"
#include "ditaction_interface_msg/msg/siro_param.hpp"
#include "controller_interface_msg/msg/base_control.hpp"

#include "visibility_control.h"

namespace ditaction_interface
{
    class DitactionInterface : public rclcpp::Node{
        public:
            DITACTION_INTERFACE_PUBLIC
            explicit DitactionInterface(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
            
            DITACTION_INTERFACE_PUBLIC
            explicit DitactionInterface(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        private:
            //yolox_ros_cppのrealsenseから
            rclcpp::Subscription<bboxes_ex_msgs::msg::BoundingBox>::SharedPtr _sub_realsense;

            //yolox_ros_cppのc1から
            rclcpp::Subscription<bboxes_ex_msgs::msg::BoundingBox>::SharedPtr _sub_c1;

            //ransacから
            rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr _sub_self_pose;

            //cntroller_interfaceから
            rclcpp::Subscription<controller_interface_msg::msg::BaseControl>::SharedPtr _sub_base_control;

            //sequncerから
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_now_sequence;

            //splineから
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_way_point;

            //sequncerへ
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub_collection_point;

            //arm_param_caluculatorへ
            rclcpp::Publisher<ditaction_interface_msg::msg::ArmParam>::SharedPtr _pub_arm_param;

            //sequnserへ
            rclcpp::Publisher<ditaction_interface_msg::msg::SiroParam>::SharedPtr _pub_siro_param;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_front_ball;

            //Qos
            rclcpp::QoS _qos = rclcpp::QoS(10);

            //yolox_ros_cppのrealsenseからのcallback
            void callback_realsense(const bboxes_ex_msgs::msg::BoundingBox::SharedPtr msg);

            //yolox_ros_cppのc1からのcallback
            void callback_c1(const bboxes_ex_msgs::msg::BoundingBox::SharedPtr msg);

            //controller_interfaceからのcallback
            void callback_base_control(const controller_interface_msg::msg::BaseControl::SharedPtr msg);

            //sequncerからのcallback
            void callback_now_sequence(const std_msgs::msg::String::SharedPtr msg);

            //splineからのcallback
            void callback_way_point(const std_msgs::msg::String::SharedPtr msg);

            //ransacからのcallback
            void callback_self_pose(const geometry_msgs::msg::Vector3::SharedPtr msg);

            //坂上から見たときにC3かC5かをみる境界線の計算
            bool bounday_line(int x, int y);

            double center_x;
            double center_y;
            std::string now_sequence;
            std::string way_point;

            bool is_self_pose_range_x_str;
            bool is_self_pose_range_y_str;
            bool is_self_pose_range_z_str;
            bool is_self_pose_range_x_siro;
            bool is_self_pose_range_y_siro;
            bool is_self_pose_range_z_siro;

            //定数
            const std::vector<double> str_self_pose_range;
            const std::vector<double> siro_self_pose_range;

            const std::vector<double> str_range_point1;
            const std::vector<double> str_range_point2;
            const std::vector<double> str_range_x_C3orC5;
            const double str_ball_range_y;
            const std::vector<double> siro_ball_range_y;
            const std::vector<double> siro_ball_range_x;

            //フィールド
            geometry_msgs::msg::Vector3 self_pose;

    };
}