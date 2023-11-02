#pragma once

#include <iostream>
#include <memory>
#include <eigen3/Eigen/Dense>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "tf2/LinearMath/Quaternion.h"

#include "ransac_localization.hpp"
#include "config.hpp"
#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "utilities/can_utils.hpp"
#include "utilities/utils.hpp"
#include <boost/format.hpp>

using namespace std;
using namespace Eigen;

class Visualizer : public rclcpp::Node {
public:
  explicit Visualizer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  explicit Visualizer(const string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void callback_selfpose(const geometry_msgs::msg::Vector3::SharedPtr self_pose);
  void callback_odom_linear(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
  void callback_odom_angular(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
  void callback_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void callback_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void load_map_config(vector<LineData> &lines);
  void create_map(vector<LineData> &lines);

  Converter converter;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
  rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr odom_linear_subscriber;
  rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr odom_angular_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr self_pose_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher;

  sensor_msgs::msg::PointCloud2 map_cloud;

  Vector3d odom = Vector3d::Zero();
  Vector3d last_odom = Vector3d::Zero();
  Vector3d diff_odom = Vector3d::Zero();
  Vector3d self_pose = Vector3d::Zero();
  Vector6d tf_laser2robot = Vector6d::Zero();

  vector<LaserPoint> map_points;

  vector<double> tf_array;
  double last_odom_received_time;
  double last_jy_received_time;
  const int odom_linear_can_id;
  const int odom_angular_can_id;
};
