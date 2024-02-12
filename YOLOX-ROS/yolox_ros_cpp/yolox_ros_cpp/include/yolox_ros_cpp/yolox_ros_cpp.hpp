#pragma once

#include <cmath>
#include <chrono>

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>


#include "bboxes_ex_msgs/msg/bounding_box.hpp"
#include "bboxes_ex_msgs/msg/bounding_boxes.hpp"
#include "ditaction_interface_msg/msg/visualizer_c1.hpp"
#include "ditaction_interface_msg/msg/visualizer_realsense.hpp"

#include "yolox_cpp/yolox.hpp"
#include "yolox_cpp/utils.hpp"
#include "yolox_param/yolox_param.hpp"

namespace yolox_ros_cpp{

    class YoloXNode : public rclcpp::Node
    {
    public:
        YoloXNode(const rclcpp::NodeOptions&);

    protected:
        std::shared_ptr<yolox_parameters::ParamListener> param_listener_;
        yolox_parameters::Params params_;
    private:
        void onInit();
        rclcpp::TimerBase::SharedPtr init_timer_;

        std::unique_ptr<yolox_cpp::AbcYoloX> yolox_;
        std::vector<std::string> class_names_;

        image_transport::Subscriber sub_image_;
        void colorImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr&);

        rclcpp::Publisher<bboxes_ex_msgs::msg::BoundingBoxes>::SharedPtr pub_bboxes_;
        image_transport::Publisher pub_image_;

        bboxes_ex_msgs::msg::BoundingBoxes objects_to_bboxes(cv::Mat, std::vector<yolox_cpp::Object>, std_msgs::msg::Header);

        //ditaction_interfaceから
        rclcpp::Subscription<ditaction_interface_msg::msg::VisualizerC1>::SharedPtr _sub_viz_c1;
        rclcpp::Subscription<ditaction_interface_msg::msg::VisualizerRealsense>::SharedPtr _sub_viz_realsense;
        void callback_viz_c1(const ditaction_interface_msg::msg::VisualizerC1::SharedPtr msg);
        void callback_viz_realsense(const ditaction_interface_msg::msg::VisualizerRealsense::SharedPtr msg);

        //坂上の画像認識の範囲
        const std::vector<double> str_range_point1;
        const std::vector<double> str_range_point2;

        //C3かC5についたときの画像認識の範囲
        const std::vector<double> str_range_x_C3orC5;

        //サイロのボールが何段目か
        const std::vector<double> siro_ball_range_y;

        //サイロのボールの画像認識の範囲
        const std::vector<double> siro_ball_range_x;

        //ひし形のボールが手前か奥か
        const double str_ball_range_y;

        bool str_from_upslope;
        bool siro_form_upslope;
        bool str_from_c3_c5;
        bool str_front_ball;
    };
}
