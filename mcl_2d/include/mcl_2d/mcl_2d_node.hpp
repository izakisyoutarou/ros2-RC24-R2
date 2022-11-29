#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
// #include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "mcl_2d/visibility_control.h"
#include "mcl.h"

namespace mcl_2d{

class Mcl2D : public rclcpp::Node {
    public:
        MCL_2D_PUBLIC
        explicit Mcl2D(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        MCL_2D_PUBLIC
        explicit Mcl2D(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _subscription_laser;
        rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _subscription_odom_linear;
        rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _subscription_odom_angular;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _subscription_initialize;

        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_selfpose;

        rclcpp::QoS _qos = rclcpp::QoS(40).keep_all();

        void _subscriber_callback_laser(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        void _subscriber_callback_odom_linear(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
        void _subscriber_callback_odom_angular(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
        void _subscriber_callback_initialize(const std_msgs::msg::Empty::SharedPtr msg);

        mcl mclocalizer;
        void check_data();

        std::vector<Eigen::Matrix4f> vec_poses;
        std::vector<double> vec_poses_time;
        std::vector<Eigen::Matrix4Xf> vec_lasers;
        std::vector<double>vec_lasers_time;

        geometry_msgs::msg::Point latest_pose;
        geometry_msgs::msg::Point initial_pose;
        rclcpp::Time observed_time;
};

}  // namespace mcl_2d
