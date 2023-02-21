
#pragma once

#include <iostream>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "utilities/can_utils.hpp"

#include "RANSAC_localization/config.hpp"
#include "RANSAC_localization/pose_fuser.hpp"
#include "RANSAC_localization/detect_lines.hpp"
#include "RANSAC_localization/converter.hpp"

#include "visibility.h"

using namespace std;

namespace self_localization{
class RANSACLocalization : public rclcpp::Node{
public:
  RANSAC_LOCALIZATION_PUBLIC
  explicit RANSACLocalization(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  RANSAC_LOCALIZATION_PUBLIC
  explicit RANSACLocalization(const string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  PoseFuser pose_fuser;  // センサ融合器
  DtectLines detect_lines;
  Converter converter;

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void callback_odom_linear(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
  void callback_odom_angular(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
  void create_elephant_map();
  void publishers(vector<config::LaserPoint> &points);
  config::LaserPoint rotate(config::LaserPoint point, double theta);
  vector<config::LaserPoint> transform(const vector<config::LaserPoint> &points, const Pose &pose);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
  rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr odom_linear_subscriber;
  rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr odom_angular_subscriber;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ransaced_publisher;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr self_pose_publisher;
  geometry_msgs::msg::Vector3 vector_msg;
  sensor_msgs::msg::PointCloud2 map_cloud;
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped corrent_pose_stamped;
  rclcpp::QoS _qos = rclcpp::QoS(40).keep_all();

  vector<config::LaserPoint> map_points;
  Pose init;
  Pose odom;
  Pose current_scan_odom;
  Pose last_odom;
  Pose diff_odom;
  Pose last_estimated;
  Pose ransac_estimated;
  Pose diff_estimated_sum;
  double odom_to_lidar_length = 0.4655;
  double reso = 0.125;
  double view_ranges = 270.0;
  bool odom_flag=false;
  double init_pose_x = -5.45;
  double init_pose_y = 0.0;
  double last_scan_received_time=0.0;
  double last_odom_received_time=0.0;
  int scan_execution_time=0;

  ///////////////////////////////////////////////チューニング///////////////////////////////////////////////
  double laser_weight_;
  double odom_weight_;
  int trial_num_;
  double inlier_dist_threshold_;
};
}
