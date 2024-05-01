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
#include "detection_interface_msg/msg/arm_param.hpp"
#include "detection_interface_msg/msg/siro_param.hpp"
#include "detection_interface_msg/msg/threshold.hpp"
#include "controller_interface_msg/msg/base_control.hpp"

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
            //yolox_ros_cppのrealsenseから
            rclcpp::Subscription<bboxes_ex_msgs::msg::BoundingBoxes>::SharedPtr _sub_realsense;

            //yolox_ros_cppのc1から
            rclcpp::Subscription<bboxes_ex_msgs::msg::BoundingBoxes>::SharedPtr _sub_c1;

            //ransacから
            rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr _sub_self_pose;

            //controller_interfaceから
            rclcpp::Subscription<controller_interface_msg::msg::BaseControl>::SharedPtr _sub_base_control;

            //sequncerから、現在のシーケンスを入れて
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_now_sequence;

            //splineから
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_way_point;

            //realsenseのimageをsub
            image_transport::Subscriber _sub_realsense_d435i;
            void d435iImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr&);

            //realsenseのdepthカメラ
            image_transport::Subscriber _sub_depth_;
            void depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr&);

            //sequncerへ
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub_collection_point;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub_suction_check;//ボールの吸引判定
            rclcpp::Publisher<detection_interface_msg::msg::SiroParam>::SharedPtr _pub_siro_param;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_front_ball;
            rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr _pub_ball_coordinate;

            //arm_param_caluculatorへ
            rclcpp::Publisher<detection_interface_msg::msg::ArmParam>::SharedPtr _pub_arm_param;

            //yolox_rosへ
            rclcpp::Publisher<detection_interface_msg::msg::Threshold>::SharedPtr _pub_threshold;

            //Qos
            rclcpp::QoS _qos = rclcpp::QoS(10);

            //yolox_ros_cppのrealsenseからのcallback
            void callback_realsense(const bboxes_ex_msgs::msg::BoundingBoxes::SharedPtr msg);

            //yolox_ros_cppのc1からのcallback
            void callback_c1(const bboxes_ex_msgs::msg::BoundingBoxes::SharedPtr msg);

            //controller_interfaceからのcallback
            void callback_base_control(const controller_interface_msg::msg::BaseControl::SharedPtr msg);

            //sequncerから、現在のシーケンスを入れてのcallback
            void callback_now_sequence(const std_msgs::msg::String::SharedPtr msg);

            //splineからのcallback
            void callback_way_point(const std_msgs::msg::String::SharedPtr msg);

            //ransacからのcallback
            void callback_self_pose(const geometry_msgs::msg::Vector3::SharedPtr msg);

            //C1cameraのc2から見たとき、c3とc4のどっちに行くか
            void c1camera_c2(const std::vector<int> center_x, const std::vector<int> center_y);

            //C1cameraのc1から見たとき、サイロのボール情報取得。
            void c1camera_c1(   const std::vector<int> ymax, std::array<std::array<int, 4>, 5>& min_max_xy, 
                                const std::vector<int> center_x, const std::vector<int> center_y,
                                std::vector<int> before_ball_place, std::vector<std::string> ball_color, const std::vector<std::string> class_id);

            //同じ領域にボールが複数存在している場合、バウンディングボックスのサイズが大きい方を採用する。それのところ
            void c1camera_pick_best_BoundingBox(std::vector<int> before_ball_place, std::vector<int> bbounbox_size, std::vector<std::string> ball_color);

            //例えば、サイロの1段目が空いている状態で2段目が埋まったとき、2段目のボールは空中に浮いていることになり、正しくない。
            //そういうときに1段目が埋まって、2段目が埋まったように修正するためのところ
            void c1camera_correct_silo_levels(std::vector<int> before_ball_place, const std::vector<std::string> ball_color, std::vector<int> after_ball_place);

            //realseneのc3、c4から見たとき、どこのSTに行くか。中でfront_ball関数を呼び出す。
            void realsense_c3_c4(int xmax, int xmin, int ymin, const std::vector<std::string> class_id, const std::vector<int> center_x, std::vector<int> center_y);

            //ボールが手前かどうか
            bool realsense_front_ball(int center_y, int threshold_y);

            //坂上から見たときにC3かC5かをみる境界線の計算
            bool bounday_line(int x, int y, const std::vector<double> point1, const std::vector<double> point2);

            //坂上から見たときに(C2付近)ひし形の内側かどうかの計算をまとめる関数
            bool rhombus_inside(int x, int y);

            /////////////////////////トピックのグローバル変数
            Vector3d pose;
            std::string now_sequence;
            std::string way_point;
            std_msgs::msg::Bool msg_front_ball;
            ////////////////////////

            //実行時間の計測用
            chrono::system_clock::time_point time_start, time_end;
            
            /////////////////////////flag系
            bool storage_flag = true;
            bool c3_c4_flag = true;
            /////////////////////////

            /////////////////////////座標変換
            coordinate_transformation ct;
            Vector3d test111;
            cv::Mat cv_image_;
            ////////////////////////

            ////////////////////////定数

            //坂上の画像認識の範囲
            const std::vector<double> str_range_point1_blue;
            const std::vector<double> str_range_point2_blue;
            const std::vector<double> str_range_point1_red;
            const std::vector<double> str_range_point2_red;

            //C2から見たひし形の枠の範囲
            const std::vector<double> str_range_point1_1_C2;
            const std::vector<double> str_range_point1_2_C2;
            const std::vector<double> str_range_point2_1_C2;
            const std::vector<double> str_range_point2_2_C2;
            const std::vector<double> str_range_point3_1_C2;
            const std::vector<double> str_range_point3_2_C2;
            const std::vector<double> str_range_point4_1_C2;
            const std::vector<double> str_range_point4_2_C2;

            //C3かC5についたときのST1~8につながる画像認識の範囲
            const int str_range_y_C3orC5_2;

            //ST系に行ったときの吸引判定
            const std::vector<long int> front_suction_check;
            const std::vector<long int> back_suction_check;

            //コートの色
            const std::string court_color;
            ////////////////////////

            ////////////////////////変数
            
    };
}