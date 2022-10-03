#pragma once

#include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "mcl_2d/visibility_control.h"
#include "../mcl/mcl.h"

namespace mcl_2d{

class Mcl2D : public rclcpp::Node {
    public:
        MCL_2D_PUBLIC
        explicit Mcl2D(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        MCL_2D_PUBLIC
        explicit Mcl2D(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _subscription_laser;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _subscription_odom;

        rclcpp::QoS _qos = rclcpp::QoS(40).keep_all();

        void _subscriber_callback_laser(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        void _subscriber_callback_odom(const nav_msgs::msg::Odometry::SharedPtr msg);

        mcl mclocalizer;
        void check_data();

        std::vector<Eigen::Matrix4f> vec_poses;
        std::vector<double> vec_poses_time;
        std::vector<Eigen::Matrix4Xf> vec_lasers;
        std::vector<double>vec_lasers_time;
};

}  // namespace mcl_2d
