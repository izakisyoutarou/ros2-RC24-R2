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

            //C1cameraのc1から見たとき、サイロのボール情報取得。
            void c1camera_c1(   const std::vector<int> ymax, std::array<std::array<int, 4>, 5>& min_max_xy, 
                                const std::vector<int> center_x, const std::vector<int> center_y, std::vector<int> bbounbox_size,
                                const std::vector<std::string> class_id);

            //同じ領域にボールが複数存在している場合、バウンディングボックスのサイズが大きい方を採用する。それのところ
            void c1camera_pick_best_BoundingBox(std::vector<int> before_ball_place, std::vector<int> bbounbox_size, std::vector<std::string> ball_color);

            //例えば、サイロの1段目が空いている状態で2段目が埋まったとき、2段目のボールは空中に浮いていることになり、正しくない。
            //そういうときに1段目が埋まって、2段目が埋まったように修正するためのところ
            void c1camera_correct_silo_levels(std::vector<int> before_ball_place, const std::vector<std::string> ball_color);

            //realseneのc3、c4から見たとき、どこのSTに行くか。中でfront_ball関数を呼び出す。
            void realsense_c3_c4(const std::vector<int> center_x, std::vector<int> center_y, const std::vector<int> center_depth, const std::vector<int> ymax);

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
            bool is_c3_c6 = false;//吸引判定のトリガー用
            bool silo_flag = true;
            /////////////////////////

            /////////////////////////座標変換
            coordinate_transformation ct;
            ////////////////////////

            ////////////////////////定数
            //ST系に行ったときの吸引判定
            const std::vector<long int> suction_check_point;
            const int depth_suction_check_value;

            //コートの色
            const std::string court_color;
            ////////////////////////

            ///////////////////////mutex
            std::mutex c1camera_mutex;
            //////////////////////

            ////////////////////////変数
            //c1カメラのリスト
            uint8_t c1camera_count = 0;
            std::array<std::vector<std::string>, 5> class_id_c1camera_list;
            std::array<std::vector<int>, 5> center_x_c1camera_list;
            std::array<std::vector<int>, 5> center_y_c1camera_list;
            std::array<std::vector<int>, 5> bbounbox_size_c1camera_list;
            std::array<std::vector<int>, 5> ymax_c1camera_list;

            // //c1カメラから受け取った情報を入れる配列
            // std::vector<std::string> class_id_c1camera;//redballとかblueballとか
            // std::vector<int> center_x_c1camera;//バウンディングボックスの真ん中(x)
            // std::vector<int> center_y_c1camera;//バウンディングボックスの真ん中(y)
            // std::vector<int> bbounbox_size_c1camera;//バウンディングの面積
            // std::vector<int> ymax_c1camera;//ボールのバウンディングの右下(y)

            //realsenseカメラのリスト
            uint8_t realsense_count = 0;
            std::array<std::vector<int>, 5> center_x_realsense_list;
            std::array<std::vector<int>, 5> center_y_realsense_list;
            std::array<std::vector<int>, 5> center_depth_realsense_list;
            std::array<std::vector<int>, 5> ymax_realsense_list;

            // //realsenseから受け取った情報を入れる配列
            // std::vector<int> center_x_realsense;//バウンディングボックスの真ん中(x)
            // std::vector<int> center_y_realsense;//バウンディングボックスの真ん中(y)
            // std::vector<int> center_depth_realsense;//バウンディングボックスの真ん中のdepth(y)
            // std::vector<int> ymax_realsense;//ボールのバウンディングの右下(y)
    };
}
