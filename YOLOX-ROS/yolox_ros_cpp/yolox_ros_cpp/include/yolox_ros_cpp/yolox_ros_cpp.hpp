#pragma once

#include <cmath>
#include <chrono>

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>


#include "bboxes_ex_msgs/msg/bounding_box.hpp"
#include "bboxes_ex_msgs/msg/bounding_boxes.hpp"

#include "detection_interface_msg/msg/threshold.hpp"

#include "realsense2_camera_msgs/msg/rgbd.hpp"

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

        bboxes_ex_msgs::msg::BoundingBoxes objects_to_bboxes(cv::Mat, cv::Mat, std::vector<yolox_cpp::Object>, std_msgs::msg::Header);

        rclcpp::Subscription<detection_interface_msg::msg::Threshold>::SharedPtr _sub_threshold;
        void ThresholdCallback(const detection_interface_msg::msg::Threshold::ConstSharedPtr&);

        rclcpp::Subscription<realsense2_camera_msgs::msg::RGBD>::SharedPtr _sub_rgbd;
        void RgbdCallback(const realsense2_camera_msgs::msg::RGBD::ConstSharedPtr&);

        int num;
        int xmax = 0;
        int xmin = 0;
        int ymin = 0;
    };
}
