#pragma once
#include <rclcpp/rclcpp.hpp>
#include <float.h>
#include <string>
#include <math.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
//使うmsg
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "bboxes_ex_msgs/msg/bounding_box.hpp"
#include "bboxes_ex_msgs/msg/bounding_boxes.hpp"
#include "detection_interface_msg/msg/silo_param.hpp"
#include "detection_interface_msg/msg/threshold.hpp"
#include "controller_interface_msg/msg/base_control.hpp"
#include "realsense2_camera_msgs/msg/rgbd.hpp"

#include "detection_interface/coordinate_transformation.hpp"

#include <image_transport/image_transport.hpp>

#include "visibility_control.h"

namespace detection_interface
{
    class DetectionInterface : public rclcpp::Node{
        public:
            DETECTION_INTERFACE_PUBLIC
            explicit DetectionInterface(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
            
            DETECTION_INTERFACE_PUBLIC
            explicit DetectionInterface(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        private:
            //yolox_ros_cppのc1から
            rclcpp::Subscription<bboxes_ex_msgs::msg::BoundingBoxes>::SharedPtr _sub_c1;

            //yolox_ros_cppのrealsense_d455から
            rclcpp::Subscription<bboxes_ex_msgs::msg::BoundingBoxes>::SharedPtr _sub_realsense_d455;

            //yolox_ros_cppのrealsense_d435iから
            // rclcpp::Subscription<bboxes_ex_msgs::msg::BoundingBoxes>::SharedPtr _sub_realsense_d435i;

            //ransacから
            rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr _sub_self_pose;

            //controller_interfaceから
            rclcpp::Subscription<controller_interface_msg::msg::BaseControl>::SharedPtr _sub_base_control;

            //sequncerから、現在のシーケンスを入れて
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_now_sequence;

            //splineから
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_way_point;

            // realsenseのrgbdeをsub
            rclcpp::Subscription<realsense2_camera_msgs::msg::RGBD>::SharedPtr _sub_realsense_d435i;
            void d435iImageCallback(const realsense2_camera_msgs::msg::RGBD::ConstSharedPtr&);

            //sequncerへ
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub_collection_point;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub_suction_check;//ボールの吸引判定
            rclcpp::Publisher<detection_interface_msg::msg::SiloParam>::SharedPtr _pub_silo_param;
            rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr _pub_ball_coordinate;

            //Qos
            rclcpp::QoS _qos = rclcpp::QoS(10);

            //yolox_ros_cppのc1cameraからのcallback
            void callback_c1camera(const bboxes_ex_msgs::msg::BoundingBoxes::SharedPtr msg);

            //yolox_ros_cppのrealsense_d455からのcallback
            void callback_realsense_d455(const bboxes_ex_msgs::msg::BoundingBoxes::SharedPtr msg);

            //yolox_ros_cppのrealsense_d435iからのcallback
            // void callback_realsense_d435i(const bboxes_ex_msgs::msg::BoundingBoxes::SharedPtr msg);

            //controller_interfaceからのcallback
            void callback_base_control(const controller_interface_msg::msg::BaseControl::SharedPtr msg);

            //sequncerから、現在のシーケンスを入れてのcallback
            void callback_now_sequence(const std_msgs::msg::String::SharedPtr msg);

            //splineからのcallback
            void callback_way_point(const std_msgs::msg::String::SharedPtr msg);

            //ransacからのcallback
            void callback_self_pose(const geometry_msgs::msg::Vector3::SharedPtr msg);

            //c1cameraのc1nodeから見たとき、サイロのボール情報取得。
            void c1camera_c1node(   const std::vector<int> ymax, std::array<std::array<int, 4>, 5>& min_max_xy, 
                                    const std::vector<int> center_x, const std::vector<int> center_y, std::vector<int> bbounbox_size,
                                    const std::vector<std::string> class_id);

            //c1cameraのc2nodeから見たとき、サイロのボール情報取得。
            void c1camera_c2node(const std::vector<int> ymax, const std::vector<int> center_x);

            //同じ領域にボールが複数存在している場合、バウンディングボックスのサイズが大きい方を採用する。それのところ
            void c1camera_pick_best_BoundingBox(std::vector<int> before_ball_place, std::vector<int> bbounbox_size, std::vector<std::string> ball_color);

            //例えば、サイロの1段目が空いている状態で2段目が埋まったとき、2段目のボールは空中に浮いていることになり、正しくない。
            //そういうときに1段目が埋まって、2段目が埋まったように修正するためのところ
            void c1camera_correct_silo_levels(std::vector<int> before_ball_place, const std::vector<std::string> ball_color);

            //realseneのc3、c4から見たとき、どこのSTに行くか。中でfront_ball関数を呼び出す。
            void realsense_c3_c6node(   const std::vector<int> center_x, const std::vector<int> center_y, const std::vector<int> center_depth, 
                                        const std::vector<int> rb_ymax, const int rbp_ymax);

            void realsense_c7_c8node(const std::vector<int> center_x, const std::vector<int> center_y, const std::vector<int> center_depth);

            /////////////////////////トピックのグローバル変数
            Vector3d pose;
            std::string now_sequence;
            std::string way_point;
            ////////////////////////

            //実行時間の計測用
            chrono::system_clock::time_point time_start, time_end;
            
            /////////////////////////flag系
            bool storage_flag = true;
            bool c3_c4_flag = true;//ST系のcollection_pointを出すトリガー
            bool silo_flag = true;
            bool c1caera_c2ode_flag = true;
            /////////////////////////

            /////////////////////////座標変換
            coordinate_transformation ct;
            ////////////////////////

            ////////////////////////定数
            //ST系に行ったときの吸引判定
            const std::vector<long int> front_suction_check_point;
            const std::vector<long int> back_suction_check_point;
            const int front_depth_suction_check_value;
            const int back_depth_suction_check_value;

            //c3orc6から見たときの外れ値を防ぐ
            const int realsense_max_x;
            const int realsense_min_x;
            const int realsense_max_y;

            //c1cameraがc2node(坂上)から見たxの閾値
            const int c2node_threshold_x;

            //realsenseがc3 or c6nodeから見たボール1個分のy
            const int str_range_y_C3orC5;

            //コートの色
            const std::string court_color;
            ////////////////////////

            ///////////////////////mutex
            std::mutex c1camera_mutex;
            //////////////////////

            ////////////////////////変数
    };
}
