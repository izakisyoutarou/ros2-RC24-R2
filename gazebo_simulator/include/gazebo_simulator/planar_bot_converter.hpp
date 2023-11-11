#pragma once

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "socketcan_interface_msg/msg/socketcan_if.hpp"

namespace gazebo_simulator{

class PlanarBotConverter : public rclcpp::Node{
public:
    explicit PlanarBotConverter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    explicit PlanarBotConverter(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _subscription_velocity;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _subscription_odom;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_velocity;
    rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr publisher_odom_linear;
    rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr publisher_odom_angular;

    rclcpp::QoS _qos = rclcpp::QoS(40).keep_all();

    double yaw;
    std::vector<double> pose_array;
    std::string court;


    void callback_velocity(const geometry_msgs::msg::Twist::SharedPtr msg);
    void callback_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
};
}